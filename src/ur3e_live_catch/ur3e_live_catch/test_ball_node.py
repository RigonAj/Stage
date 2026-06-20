"""Artificial ball source for testing the whole chain WITHOUT a camera (archi §4.2).

Publishes ``BallState`` in the SAME contract as the real tracker (stamped,
``frame_id`` always set). Two source modes:

  parabola : analytic p(t) = p0 + v0 t + 1/2 g t^2, defined in the z-up ``base``
             frame (no axis conversion needed, archi §4.2).
  csv      : replay ground-truth, e.g. sequences/<seq>/labels/ground_truth.csv
             (columns ball_*_world_m and ball_*_cam_m).

``publish_frame`` selects the DECLARED frame (header.frame_id):
  - "base": publish base coordinates directly (short-circuits the hand-eye, lets
    you isolate policy/obs errors from extrinsic-calibration errors);
  - "<camera_frame>": publish camera-frame coordinates (exercises the full TF /
    hand-eye path). For the parabola this applies the inverse of the configured
    camera pose in base (``camera_translation`` / ``camera_quaternion``); for csv
    it uses the recorded camera columns directly.

Injectable Gaussian ``noise_std`` and ``dropout_prob`` stress the velocity filter
and staleness handling.
"""

from __future__ import annotations

import csv
import math
import random
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node

from ur3e_catch_msgs.msg import BallState
from ur3e_live_catch.ball_frame import RigidTransform


class TestBallNode(Node):
    def __init__(self) -> None:
        super().__init__("test_ball_node")
        self.declare_parameter("output_topic", "ball_state")
        self.declare_parameter("publish_frame", "base")
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("source", "parabola")  # parabola | csv
        self.declare_parameter("noise_std", 0.0)
        self.declare_parameter("dropout_prob", 0.0)
        # parabola params (base frame, z-up, metres / m·s⁻¹ / m·s⁻²)
        self.declare_parameter("p0", [-1.0, 1.5, 0.4])
        self.declare_parameter("v0", [1.5, -1.0, 2.5])
        self.declare_parameter("gravity", [0.0, 0.0, -9.81])
        self.declare_parameter("restart_after_s", 2.0)
        # camera pose in base (base<-camera), used only when publish_frame != base
        self.declare_parameter("camera_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("camera_quaternion", [0.0, 0.0, 0.0, 1.0])
        # csv params
        self.declare_parameter("csv_path", "")

        self._frame = str(self.get_parameter("publish_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._noise = float(self.get_parameter("noise_std").value)
        self._dropout = float(self.get_parameter("dropout_prob").value)
        self._source = str(self.get_parameter("source").value)
        self._restart = float(self.get_parameter("restart_after_s").value)
        self._base_to_cam = self._build_camera_transform()

        self._csv_rows: list[tuple[float, tuple, tuple]] = []
        self._csv_idx = 0
        if self._source == "csv":
            self._load_csv(str(self.get_parameter("csv_path").value))

        self._pub = self.create_publisher(BallState, str(self.get_parameter("output_topic").value), 10)
        rate = float(self.get_parameter("rate_hz").value)
        self._t0 = self.get_clock().now()
        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"test_ball_node: source={self._source}, publish_frame={self._frame!r}, rate={rate} Hz"
        )

    # --- setup helpers --------------------------------------------------------

    def _build_camera_transform(self) -> Optional[RigidTransform]:
        if self._frame == self._base_frame:
            return None
        t = [float(x) for x in self.get_parameter("camera_translation").value]
        q = [float(x) for x in self.get_parameter("camera_quaternion").value]
        if t == [0.0, 0.0, 0.0] and q == [0.0, 0.0, 0.0, 1.0]:
            self.get_logger().warn(
                "publish_frame != base but camera transform is identity; "
                "camera-frame output will equal base coordinates"
            )
        return RigidTransform(translation=(t[0], t[1], t[2]), quaternion=(q[0], q[1], q[2], q[3]))

    def _load_csv(self, path: str) -> None:
        if not path or not Path(path).is_file():
            self.get_logger().error(f"csv source selected but csv_path invalid: {path!r}")
            return
        with open(path, newline="") as handle:
            for row in csv.DictReader(handle):
                if int(float(row.get("visible", "1"))) == 0:
                    continue
                t = float(row["timestamp_s"])
                world = (float(row["ball_x_world_m"]), float(row["ball_y_world_m"]), float(row["ball_z_world_m"]))
                cam = (float(row["ball_x_cam_m"]), float(row["ball_y_cam_m"]), float(row["ball_z_cam_m"]))
                self._csv_rows.append((t, world, cam))
        self.get_logger().info(f"loaded {len(self._csv_rows)} csv samples from {path}")

    # --- per-tick -------------------------------------------------------------

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._t0).nanoseconds * 1e-9

    def _parabola_point(self, t: float):
        p0 = [float(x) for x in self.get_parameter("p0").value]
        v0 = [float(x) for x in self.get_parameter("v0").value]
        g = [float(x) for x in self.get_parameter("gravity").value]
        base_pt = tuple(p0[i] + v0[i] * t + 0.5 * g[i] * t * t for i in range(3))
        if self._base_to_cam is None:
            return base_pt
        # Express a base point in the camera frame: P^cam = R^T (P^base - t).
        tx, ty, tz = self._base_to_cam.translation
        shifted = (base_pt[0] - tx, base_pt[1] - ty, base_pt[2] - tz)
        qx, qy, qz, qw = self._base_to_cam.quaternion
        inv = RigidTransform(translation=(0.0, 0.0, 0.0), quaternion=(-qx, -qy, -qz, qw))
        return inv.apply(shifted)

    def _next_point(self):
        if self._source == "csv":
            if not self._csv_rows:
                return None
            t, world, cam = self._csv_rows[self._csv_idx % len(self._csv_rows)]
            self._csv_idx += 1
            return cam if self._base_to_cam is not None else world
        # parabola
        t = self._elapsed()
        if self._restart > 0:
            t = math.fmod(t, self._restart)
        return self._parabola_point(t)

    def _tick(self) -> None:
        if self._dropout > 0.0 and random.random() < self._dropout:
            return  # simulated detection gap

        point = self._next_point()
        msg = BallState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        if point is None:
            msg.valid = False
            msg.confidence = 0.0
        else:
            n = self._noise
            msg.position.x = point[0] + (random.gauss(0.0, n) if n > 0 else 0.0)
            msg.position.y = point[1] + (random.gauss(0.0, n) if n > 0 else 0.0)
            msg.position.z = point[2] + (random.gauss(0.0, n) if n > 0 else 0.0)
            msg.valid = True
            msg.confidence = 1.0
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TestBallNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
