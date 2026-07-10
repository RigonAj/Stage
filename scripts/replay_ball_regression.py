#!/usr/bin/env python3
"""Replay recorded raw ball detections through BallisticRegression offline.

Tune the regression gates on REAL captures without a robot session (plan 2.2,
docs/Robot_Control/plan_amelioration_perception_transmission.md). Record during
real throws with:

    ros2 bag record /ball_state_raw /tf_static

then replay, overriding any RegressionConfig field:

    python3 scripts/replay_ball_regression.py rosbag2_2026_07_10-14_03_22 \
        --set depth_sigma_scale=8.0 --set max_rms_m=0.05

The tool transforms camera-frame samples to base_link using the static TF
chain found in the bag's /tf_static (base -> camera_optical from
publish_camera_tf.py plus the URDF base_link -> base fixed joint recorded by
robot_state_publisher). Samples already stamped base_link need no TF. It then
feeds BallRegression sample by sample, stepping the 60 Hz output clock in
between, and prints per-flight summaries (pop latency, rms, v0, end reason).

Requires a sourced workspace (rosbag2_py + ur3e_catch_msgs).
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import sys
from pathlib import Path

from ur3e_live_catch.ball_regression import BallRegression, RegressionConfig

Vec3 = tuple[float, float, float]


# --- CLI / config overrides -----------------------------------------------------


def parse_overrides(pairs: list[str]) -> dict:
    """'key=value' strings -> typed RegressionConfig kwargs (plan 2.2)."""
    fields = {f.name: f.type for f in dataclasses.fields(RegressionConfig)}
    out: dict = {}
    for pair in pairs:
        key, sep, raw = pair.partition("=")
        if not sep:
            raise ValueError(f"--set expects key=value, got {pair!r}")
        if key not in fields:
            raise ValueError(
                f"unknown RegressionConfig field {key!r}; valid: {', '.join(sorted(fields))}"
            )
        annotation = str(fields[key])
        if "bool" in annotation:
            out[key] = raw.strip().lower() in ("1", "true", "yes", "on")
        elif "int" in annotation:
            out[key] = int(raw)
        else:
            out[key] = float(raw)
    return out


# --- static TF resolution ---------------------------------------------------------


class StaticTfGraph:
    """Minimal parent->child static transform graph (translation + quaternion)."""

    def __init__(self) -> None:
        # child -> (parent, translation, quaternion xyzw)
        self._edges: dict[str, tuple[str, Vec3, tuple[float, float, float, float]]] = {}

    def add(self, parent: str, child: str, translation: Vec3,
            quaternion: tuple[float, float, float, float]) -> None:
        self._edges[child] = (parent, translation, quaternion)

    def chain_to_ancestor(self, frame: str, ancestor: str):
        """Compose ancestor<-frame from stored edges; None when unreachable."""
        translation = (0.0, 0.0, 0.0)
        quaternion = (0.0, 0.0, 0.0, 1.0)
        current = frame
        for _ in range(32):  # cycle guard
            if current == ancestor:
                return translation, quaternion
            edge = self._edges.get(current)
            if edge is None:
                return None
            parent, t_pc, q_pc = edge
            # X_parent = R_pc * X_child + t_pc, compose onto the accumulated map.
            translation = _add(_rotate(q_pc, translation), t_pc)
            quaternion = _q_mul(q_pc, quaternion)
            current = parent
        return None


def _rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    # Rodrigues via quaternion: v + 2*q_vec x (q_vec x v + w*v)
    cx = y * vz - z * vy + w * vx
    cy = z * vx - x * vz + w * vy
    cz = x * vy - y * vx + w * vz
    return (
        vx + 2.0 * (y * cz - z * cy),
        vy + 2.0 * (z * cx - x * cz),
        vz + 2.0 * (x * cy - y * cx),
    )


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


# --- bag reading -------------------------------------------------------------------


def _open_bag(path: str):
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=""),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    return reader


def _load_messages(path: str, ball_topic: str):
    """Return (ball_msgs sorted by stamp, StaticTfGraph) from the bag."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = _open_bag(path)
    type_by_topic = {info.name: info.type for info in reader.get_all_topics_and_types()}
    if ball_topic not in type_by_topic:
        raise SystemExit(
            f"topic {ball_topic!r} not in bag (found: {', '.join(sorted(type_by_topic))})"
        )
    tf_graph = StaticTfGraph()
    balls = []
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic == ball_topic:
            msg = deserialize_message(raw, get_message(type_by_topic[topic]))
            balls.append(msg)
        elif topic == "/tf_static":
            msg = deserialize_message(raw, get_message(type_by_topic[topic]))
            for tr in msg.transforms:
                t = tr.transform.translation
                q = tr.transform.rotation
                tf_graph.add(
                    tr.header.frame_id, tr.child_frame_id,
                    (t.x, t.y, t.z), (q.x, q.y, q.z, q.w),
                )
    balls.sort(key=lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    return balls, tf_graph


# --- replay -----------------------------------------------------------------------


def replay(balls, tf_graph: StaticTfGraph, base_frame: str, cfg: RegressionConfig,
           rate_hz: float = 60.0) -> list[dict]:
    reg = BallRegression(cfg)
    summaries: list[dict] = []
    seen_flights = 0
    tf_cache: dict[str, object] = {}
    next_tick = None
    dropped_tf = 0

    for msg in balls:
        if not msg.valid:
            continue
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        frame = msg.header.frame_id
        pos = (msg.position.x, msg.position.y, msg.position.z)
        camera_pos = None
        if frame and frame != base_frame:
            if frame not in tf_cache:
                tf_cache[frame] = tf_graph.chain_to_ancestor(frame, base_frame)
            chain = tf_cache[frame]
            if chain is None:
                dropped_tf += 1
                continue
            translation, quaternion = chain
            pos = _add(_rotate(quaternion, pos), translation)
            camera_pos = translation  # camera origin expressed in base_frame

        # Step the output clock up to this sample (mirrors the 60 Hz node timer).
        if next_tick is None:
            next_tick = stamp
        while next_tick <= stamp:
            reg.step(next_tick)
            next_tick += 1.0 / rate_hz
        reg.add_sample(stamp, pos, confidence=float(msg.confidence),
                       camera_pos_base=camera_pos)
        if reg.flights_ended != seen_flights:
            seen_flights = reg.flights_ended
            summaries.append(dict(reg.last_flight_summary or {}))

    # Drain: run the clock past the last sample so coasting flights terminate.
    if next_tick is not None:
        horizon = next_tick + cfg.max_coast_s + cfg.max_flight_s + cfg.collect_timeout_s
        while next_tick <= horizon:
            reg.step(next_tick)
            next_tick += 1.0 / rate_hz
            if reg.flights_ended != seen_flights:
                seen_flights = reg.flights_ended
                summaries.append(dict(reg.last_flight_summary or {}))
    if dropped_tf:
        print(f"WARNING: {dropped_tf} samples dropped (frame not reachable from "
              f"{base_frame!r} via /tf_static)", file=sys.stderr)
    return summaries


def _print_summaries(summaries: list[dict]) -> None:
    if not summaries:
        print("no flight ended during the replay (check gates / topic / TF)")
        return
    for i, s in enumerate(summaries):
        lat = s.get("pop_latency_s")
        pop = s.get("pop_position")
        rms = s.get("fit_rms_m")
        span = s.get("fit_span_s")
        lat_txt = "-" if lat is None else f"{lat * 1000:.0f}ms"
        pop_txt = "-" if pop is None else f"({pop[0]:.2f},{pop[1]:.2f},{pop[2]:.2f})"
        rms_txt = "-" if rms is None else f"{rms:.4f}m"
        span_txt = "-" if span is None else f"{span:.3f}s"
        print(
            f"flight {i}: end={s.get('reason')} pop_latency={lat_txt} "
            f"pop_pos={pop_txt} accepted={s.get('n_accepted')} "
            f"rejected={s.get('n_rejected')} rms={rms_txt} span={span_txt}"
        )
    popped = sum(1 for s in summaries if s.get("pop_t") is not None)
    print(f"total: {len(summaries)} flight(s), {popped} popped")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag", help="rosbag2 directory recorded during real throws")
    parser.add_argument("--topic", default="/ball_state_raw")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--rate-hz", type=float, default=60.0)
    parser.add_argument("--set", dest="overrides", action="append", default=[],
                        metavar="KEY=VALUE",
                        help="override a RegressionConfig field (repeatable)")
    args = parser.parse_args(argv)

    if not Path(args.bag).exists():
        raise SystemExit(f"bag not found: {args.bag}")
    cfg = RegressionConfig(**parse_overrides(args.overrides))
    print(f"config overrides: {parse_overrides(args.overrides) or 'none'}")
    balls, tf_graph = _load_messages(args.bag, args.topic)
    print(f"{len(balls)} raw messages on {args.topic}")
    summaries = replay(balls, tf_graph, args.base_frame, cfg, rate_hz=args.rate_hz)
    _print_summaries(summaries)
    return 0


if __name__ == "__main__":
    sys.exit(main())
