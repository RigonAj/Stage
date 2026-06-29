"""Online sweep stage (doc §6.1): excite one joint, record /joint_states -> CSV.

Mirrors ``ur3e_rollout_replay.send`` (ActionClient + ``build_joint_trajectory``) for the
step/ramp profiles and ``ur3e_live_catch.live_catch_node`` for the controller switch and
the 60 Hz ``forward_position_controller`` streaming used by the chirp (the deployment
path, doc §1). Safety gates (``excitation.check_excitation``) run before any motion.

Run under the ROS python (``ros2 run ur3e_sysid run_sweep ...``), NOT the torch .venv.

  ros2 run ur3e_sysid run_sweep --joint elbow --signal chirp \
      --f0 0.1 --f1 3.0 --amplitude 0.02 --duration 20 --out-dir recordings/sysid
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

from .config import ID_POSE, JOINTS, VELOCITY_LIMIT, joint_index, resolve_joint
from .excitation import ExcitationUnsafe, SweepParams, check_excitation
from . import signals
from .recorder import Recorder

TRAJECTORY_CONTROLLER = "scaled_joint_trajectory_controller"
COMMAND_CONTROLLER = "forward_position_controller"


@dataclass(frozen=True)
class _Plan:
    """Minimal plan accepted by ``ur3e_rollout_replay.send.build_joint_trajectory``."""

    joint_names: tuple[str, ...]
    positions: tuple[tuple[float, ...], ...]
    time_from_start_s: tuple[float, ...]


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="UR3e system-id: excite one joint and record /joint_states.")
    p.add_argument("--joint", required=True,
                   help=f"Joint to excite, one at a time (full or short name, e.g. elbow or elbow_joint); one of {list(JOINTS)}")
    p.add_argument("--signal", required=True, choices=["step", "chirp", "ramp"])
    p.add_argument("--amplitude", type=float, default=0.03, help="Swing amplitude (rad), step/chirp")
    p.add_argument("--f0", type=float, default=0.1, help="Chirp start frequency (Hz)")
    p.add_argument("--f1", type=float, default=3.0, help="Chirp end frequency (Hz)")
    p.add_argument("--duration", type=float, default=20.0, help="Chirp duration (s)")
    p.add_argument("--velocity", type=float, default=0.1, help="Ramp speed (rad/s)")
    p.add_argument("--distance", type=float, default=0.2, help="Ramp move length (rad, sign allowed)")
    p.add_argument("--rate", type=float, default=60.0, help="Chirp command rate (Hz), matches streaming.py")
    p.add_argument("--out-dir", default="recordings/sysid", help="Directory for the CSV + meta JSON")
    p.add_argument("--id-pose", help="Comma-separated 6-joint identification pose (rad); default config.ID_POSE")
    p.add_argument("--margin", type=float, default=0.1, help="Joint-limit safety margin (rad)")
    p.add_argument("--velocity-fraction", type=float, default=0.5, help="Speed guard as fraction of limit")
    p.add_argument("--joint-state-topic", default="/joint_states")
    p.add_argument("--command-topic", default="/forward_position_controller/commands")
    p.add_argument("--action-name", default=f"/{TRAJECTORY_CONTROLLER}/follow_joint_trajectory")
    p.add_argument("--trajectory-controller", default=TRAJECTORY_CONTROLLER)
    p.add_argument("--command-controller", default=COMMAND_CONTROLLER)
    p.add_argument("--goto-velocity", type=float, default=0.3, help="Max speed for the move to the id pose")
    p.add_argument("--joint-state-timeout", type=float, default=10.0)
    p.add_argument("--payload-kg", type=float, default=0.0, help="Tool/payload at the wrist (documented in meta)")
    p.add_argument("--robot-serial", default="", help="Robot serial, recorded in meta")
    p.add_argument("--dry-run", action="store_true", help="Validate gates + write meta, do not move the robot")
    return p


def _parse_pose(value: str | None) -> tuple[float, ...]:
    if not value:
        return tuple(ID_POSE)
    parts = tuple(float(x.strip()) for x in value.split(","))
    if len(parts) != 6:
        raise ValueError("--id-pose must contain 6 comma-separated values")
    return parts


def main(argv: list[str] | None = None) -> int:
    try:
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from control_msgs.action import FollowJointTrajectory
        from controller_manager_msgs.srv import SwitchController
        from builtin_interfaces.msg import Duration
    except ImportError as exc:  # pragma: no cover - depends on ROS env
        print("ERROR: ROS 2 dependencies missing; source the workspace and run via ros2 run.", file=sys.stderr)
        print(f"missing import: {exc}", file=sys.stderr)
        return 2

    from ur3e_live_catch.joint_order import JointOrderError, reorder_by_name
    from ur3e_live_catch.limits import load_ur3e_joint_limits
    from ur3e_rollout_replay.replay_core import SafetyLimits, retime_segments
    from ur3e_rollout_replay.send import build_joint_trajectory, seconds_to_duration

    args = build_parser().parse_args(argv)
    try:
        args.joint = resolve_joint(args.joint)
        id_pose = _parse_pose(args.id_pose)
    except (ValueError, KeyError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    k = joint_index(args.joint)
    center = id_pose[k]
    params = SweepParams(
        signal=args.signal,
        amplitude=args.amplitude,
        f0=args.f0,
        f1=args.f1,
        duration=args.duration,
        velocity=args.velocity,
        distance=args.distance,
    )

    limits = load_ur3e_joint_limits()
    lim = limits[args.joint]
    v_limit = lim.max_velocity if math.isfinite(lim.max_velocity) else VELOCITY_LIMIT[k]
    try:
        metrics = check_excitation(
            center,
            params,
            q_min=lim.min_position,
            q_max=lim.max_position,
            velocity_limit=v_limit,
            margin=args.margin,
            velocity_fraction=args.velocity_fraction,
        )
    except ExcitationUnsafe as exc:
        print(f"ERROR: excitation rejected by safety gate: {exc}", file=sys.stderr)
        return 2
    print(f"safety: OK joint={args.joint} signal={args.signal} center={center:.3f} {metrics}")

    meta = {
        "joint": args.joint,
        "signal": args.signal,
        "params": {
            "amplitude": args.amplitude, "f0": args.f0, "f1": args.f1,
            "duration": args.duration, "velocity": args.velocity,
            "distance": args.distance, "rate": args.rate,
        },
        "id_pose_rad": list(id_pose),
        "payload_kg": args.payload_kg,
        "robot_serial": args.robot_serial,
        "interface": f"{args.command_controller}@{args.rate:g}Hz" if args.signal == "chirp"
        else f"{args.trajectory_controller}",
        "velocity_limit": v_limit,
        "safety_metrics": metrics,
    }
    recorder = Recorder(args.joint)
    out_dir = args.out_dir

    if args.dry_run:
        print("dry_run: validating profile generation, no robot motion")
        _build_profile(args, center, signals)  # raises on bad params
        recorder.save_meta(f"{out_dir}/{args.joint}_{args.signal}.meta.json", {**meta, "dry_run": True})
        print(f"wrote meta to {out_dir}/{args.joint}_{args.signal}.meta.json")
        return 0

    rclpy.init(args=None)

    class SysIdSweep(Node):
        def __init__(self) -> None:
            super().__init__("ur3e_sysid_run_sweep")
            self._k = k
            self._latest_q: list[float] | None = None
            self._t0: float | None = None
            self._recording = False
            self.create_subscription(JointState, args.joint_state_topic, self._on_joint_state, 50)
            self._action = ActionClient(self, FollowJointTrajectory, args.action_name)
            self._switch = self.create_client(SwitchController, "/controller_manager/switch_controller")
            self._cmd_pub = self.create_publisher(Float64MultiArray, args.command_topic, 10)

        def _on_joint_state(self, msg) -> None:
            try:
                q = reorder_by_name(msg.name, msg.position)
            except JointOrderError:
                return
            self._latest_q = q
            if not self._recording or self._t0 is None:
                return
            qd = reorder_by_name(msg.name, msg.velocity) if len(msg.velocity) == len(msg.name) else [0.0] * 6
            eff = reorder_by_name(msg.name, msg.effort) if len(msg.effort) == len(msg.name) else [0.0] * 6
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            recorder.add_state(stamp - self._t0, q[self._k], qd[self._k], eff[self._k])

        def now_s(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def wait_for_state(self, timeout: float) -> bool:
            deadline = self.now_s() + timeout
            while rclpy.ok() and self._latest_q is None and self.now_s() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            return self._latest_q is not None

        def switch_controllers(self, activate, deactivate, timeout: float = 5.0) -> bool:
            if not self._switch.wait_for_service(timeout_sec=timeout):
                self.get_logger().error("switch_controller service unavailable")
                return False
            req = SwitchController.Request()
            req.activate_controllers = list(activate)
            req.deactivate_controllers = list(deactivate)
            req.strictness = SwitchController.Request.BEST_EFFORT
            fut = self._switch.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
            res = fut.result()
            return bool(res and res.ok)

        def send_trajectory(self, plan, timeout_extra: float = 10.0) -> bool:
            if not self._action.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(f"action server unavailable: {args.action_name}")
                return False
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = build_joint_trajectory(plan, JointTrajectory, JointTrajectoryPoint, Duration)
            send_fut = self._action.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_fut)
            handle = send_fut.result()
            if handle is None or not handle.accepted:
                self.get_logger().error("trajectory goal rejected")
                return False
            result_fut = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_fut, timeout_sec=plan.time_from_start_s[-1] + timeout_extra)
            return result_fut.result() is not None

        def publish_command(self, pose) -> None:
            msg = Float64MultiArray()
            msg.data = [float(x) for x in pose]
            self._cmd_pub.publish(msg)

    node = SysIdSweep()
    rc = 0
    try:
        if not node.wait_for_state(args.joint_state_timeout):
            print(f"ERROR: no /joint_states within {args.joint_state_timeout:.1f}s", file=sys.stderr)
            return 2

        # 1. go to the identification pose via the trajectory controller.
        if not node.switch_controllers([args.trajectory_controller], [args.command_controller]):
            node.get_logger().warn("could not (re)activate the trajectory controller; continuing")
        current = tuple(node._latest_q)
        goto_limits = SafetyLimits(max_joint_velocity=args.goto_velocity, max_joint_acceleration=0.5,
                                   min_segment_duration=0.5)
        goto_dur = retime_segments((current, id_pose), goto_limits, min_duration=1.0)[0]
        goto_plan = _Plan(JOINTS, (current, tuple(id_pose)), (0.0, goto_dur))
        print(f"goto id pose ({goto_dur:.2f}s) ...")
        if not node.send_trajectory(goto_plan):
            print("ERROR: failed to reach the identification pose", file=sys.stderr)
            return 1

        # 2. excite + record.
        node._t0 = node.now_s()
        node._recording = True
        if args.signal in ("step", "ramp"):
            times, values = _build_profile(args, center, signals)
            plan = _Plan(JOINTS, tuple(signals.embed(id_pose, k, values)), tuple(times))
            for t_cmd, v_cmd in zip(times, values):
                recorder.add_command(t_cmd, v_cmd)
            print(f"excite {args.signal} ({times[-1]:.2f}s) ...")
            if not node.send_trajectory(plan):
                print("ERROR: excitation trajectory failed", file=sys.stderr)
                rc = 1
        else:  # chirp on the deployment path
            if not node.switch_controllers([args.command_controller], [args.trajectory_controller]):
                print("ERROR: could not activate forward_position_controller for the chirp", file=sys.stderr)
                return 1
            print(f"excite chirp {args.f0}->{args.f1} Hz over {args.duration:.1f}s @ {args.rate:g} Hz ...")
            node._t0 = node.now_s()
            dt = 1.0 / args.rate
            next_pub = 0.0
            last_pose = signals.embed(id_pose, k, [center])[0]
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.001)
                t = node.now_s() - node._t0
                if t >= args.duration:
                    break
                if t >= next_pub:
                    val = signals.chirp_value(t, center, args.amplitude, args.f0, args.f1, args.duration)
                    last_pose = signals.embed(id_pose, k, [val])[0]
                    node.publish_command(last_pose)
                    recorder.add_command(t, val)
                    next_pub += dt
            for _ in range(5):  # controlled stop: hold the last set-point
                node.publish_command(last_pose)
                rclpy.spin_once(node, timeout_sec=0.02)

        node._recording = False
        node.switch_controllers([args.trajectory_controller], [args.command_controller])

        # 3. persist.
        csv_path = f"{out_dir}/{args.joint}_{args.signal}.csv"
        recorder.save_csv(csv_path, f0=args.f0 if args.signal == "chirp" else 0.0,
                          f1=args.f1 if args.signal == "chirp" else 0.0)
        recorder.save_meta(f"{out_dir}/{args.joint}_{args.signal}.meta.json",
                           {**meta, "num_states": recorder.num_states, "num_commands": recorder.num_commands})
        print(f"recorded {recorder.num_states} states / {recorder.num_commands} commands -> {csv_path}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


def _build_profile(args, center: float, signals_mod) -> tuple[list[float], list[float]]:
    if args.signal == "step":
        return signals_mod.step_profile(center, args.amplitude)
    if args.signal == "ramp":
        return signals_mod.ramp_profile(center, args.distance, args.velocity)
    return signals_mod.chirp_profile(center, args.amplitude, args.f0, args.f1, args.duration, rate=args.rate)


if __name__ == "__main__":
    raise SystemExit(main())
