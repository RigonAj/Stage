"""Live catch loop — DRY-RUN build (archi §2, §4.3; roadmap steps 1–5).

One rclpy node, one 60 Hz timer. The hot path ``observation -> inference`` runs as
DIRECT CALLS into the pure modules (no intra-process topic, no DDS hop, archi §2):

    BallState ─▶ ball_frame ─▶ ObservationBuilder ─▶ PolicyRunner ─▶ raw action
    /joint_states ─▶ cache (reordered)              (TF: frame_id→base, base→hoop)

In this pass the node is DRY-RUN: it logs the raw policy action and publishes
``CatchTelemetry`` only. ActionMapper / SafetyLimiter / streaming to the
forward_position_controller are step 6 and are intentionally NOT wired here — no
robot command is ever emitted. The raw action is fed back as observation
component 9 on the next tick (raw, matching training; archi §6).
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from ur3e_catch_msgs.msg import BallState, CatchTelemetry
from ur3e_live_catch.ball_frame import BallFrameTransformer, FrameError, RigidTransform
from ur3e_live_catch.joint_order import JointOrderError, reorder_by_name
from ur3e_live_catch.observation import ObservationBuilder
from ur3e_live_catch.policy_runtime import PolicyRunner

# Default model: canonical data/models/, falling back to the dated training export.
CANONICAL_MODEL = "data/models/policy_deterministic.ts"
FALLBACK_MODEL = (
    "data/ur3e_rollouts/2026-05-26_17-13-29_ppo_torch/exports/policy_deterministic.ts"
)


def _stamp_to_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _transform_from_msg(tf_msg) -> RigidTransform:
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    return RigidTransform(translation=(t.x, t.y, t.z), quaternion=(q.x, q.y, q.z, q.w))


class LiveCatchNode(Node):
    def __init__(self) -> None:
        super().__init__("live_catch_node")
        self.declare_parameter("loop_hz", 60.0)
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("hoop_frame", "hoop_center")
        self.declare_parameter("ball_topic", "ball_state")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("telemetry_topic", "catch_telemetry")
        self.declare_parameter("units", "m")
        self.declare_parameter("model_path", "")  # "" => canonical then fallback
        self.declare_parameter("stale_after_s", 0.1)
        # Disk geometry fallback when no TF base->hoop is available (placeholder,
        # to be replaced by the Isaac-sourced mount geometry). normal in base.
        self.declare_parameter("disk_pos_fallback", [0.0, 0.0, 0.5])
        self.declare_parameter("disk_normal_fallback", [0.0, 0.0, 1.0])
        # action_mode is reserved for step 6 (ActionMapper); unused in dry-run.
        self.declare_parameter("action_mode", "faithful")

        self._base_frame = str(self.get_parameter("base_frame").value)
        self._hoop_frame = str(self.get_parameter("hoop_frame").value)

        self._joint_pos: Optional[list[float]] = None
        self._joint_vel: Optional[list[float]] = None
        self._ball_msg: Optional[BallState] = None
        self._prev_action: list[float] = [0.0] * 6

        self._ball_frame = BallFrameTransformer(
            base_frame=self._base_frame,
            units=str(self.get_parameter("units").value),
            stale_after_s=float(self.get_parameter("stale_after_s").value),
        )
        self._obs_builder = ObservationBuilder()
        self._policy = self._make_policy()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(JointState, str(self.get_parameter("joint_states_topic").value),
                                 self._on_joint_states, 10)
        self.create_subscription(BallState, str(self.get_parameter("ball_topic").value),
                                 self._on_ball, 10)
        self._telemetry_pub = self.create_publisher(
            CatchTelemetry, str(self.get_parameter("telemetry_topic").value), 10
        )

        hz = float(self.get_parameter("loop_hz").value)
        self._timer = self.create_timer(1.0 / hz, self._on_tick)
        self.get_logger().info(f"live_catch_node DRY-RUN @ {hz} Hz (no robot command emitted)")

    # --- setup ---------------------------------------------------------------

    def _make_policy(self) -> Optional[PolicyRunner]:
        configured = str(self.get_parameter("model_path").value)
        candidates = [configured] if configured else [CANONICAL_MODEL, FALLBACK_MODEL]
        for cand in candidates:
            if cand and Path(cand).exists():
                self.get_logger().info(f"policy model: {cand}")
                return PolicyRunner(cand)
        self.get_logger().warn(
            "no policy model found (looked for "
            f"{candidates}); running observation-only, action will be zeros"
        )
        return None

    # --- subscriptions -------------------------------------------------------

    def _on_joint_states(self, msg: JointState) -> None:
        try:
            self._joint_pos = reorder_by_name(msg.name, msg.position)
            self._joint_vel = reorder_by_name(msg.name, msg.velocity) if msg.velocity else [0.0] * 6
        except (JointOrderError, ValueError) as exc:
            self.get_logger().warn(f"joint_states ignored: {exc}", throttle_duration_sec=2.0)

    def _on_ball(self, msg: BallState) -> None:
        self._ball_msg = msg

    # --- TF lookups ----------------------------------------------------------

    def _ball_transform(self, frame_id: str) -> Optional[RigidTransform]:
        if frame_id == self._base_frame:
            return None  # identity handled inside ball_frame
        try:
            tf_msg = self._tf_buffer.lookup_transform(self._base_frame, frame_id, rclpy.time.Time())
            return _transform_from_msg(tf_msg)
        except Exception as exc:  # tf2 raises several exception types
            self.get_logger().warn(
                f"TF {frame_id}->{self._base_frame} unavailable: {exc}", throttle_duration_sec=2.0
            )
            return None

    def _disk_pose(self):
        """Return (disk_pos_base, disk_normal_base) from TF, else the fallback."""
        try:
            tf_msg = self._tf_buffer.lookup_transform(self._base_frame, self._hoop_frame, rclpy.time.Time())
            tf = _transform_from_msg(tf_msg)
            normal = RigidTransform(quaternion=tf.quaternion).apply((0.0, 0.0, 1.0))
            return list(tf.translation), list(normal)
        except Exception:
            return (
                [float(x) for x in self.get_parameter("disk_pos_fallback").value],
                [float(x) for x in self.get_parameter("disk_normal_fallback").value],
            )

    # --- hot loop ------------------------------------------------------------

    def _on_tick(self) -> None:
        if self._joint_pos is None:
            self.get_logger().warn("waiting for /joint_states", throttle_duration_sec=2.0)
            return
        if self._ball_msg is None or not self._ball_msg.valid:
            self.get_logger().warn("waiting for a valid BallState", throttle_duration_sec=2.0)
            return

        ball = self._ball_msg
        frame_id = ball.header.frame_id
        stamp_s = _stamp_to_s(ball.header.stamp)
        transform = self._ball_transform(frame_id)
        try:
            ball_pos, ball_vel = self._ball_frame.process(
                (ball.position.x, ball.position.y, ball.position.z),
                frame_id, stamp_s, transform=transform,
            )
        except FrameError as exc:
            self.get_logger().warn(f"ball frame rejected: {exc}", throttle_duration_sec=2.0)
            return

        disk_pos, disk_normal = self._disk_pose()

        obs = self._obs_builder.build(
            joint_pos=self._joint_pos,
            joint_vel=self._joint_vel,
            disk_pos=disk_pos,
            disk_normal=disk_normal,
            ball_pos=ball_pos,
            ball_vel=ball_vel,
            prev_action=self._prev_action,
        )

        raw_action = self._policy.infer(obs) if self._policy is not None else [0.0] * 6

        telem = CatchTelemetry()
        telem.observation = [float(x) for x in obs]
        telem.raw_action = [float(x) for x in raw_action]
        telem.joint_target = []  # empty in dry-run (no command, archi step 6 wires this)
        telem.ball_base.x, telem.ball_base.y, telem.ball_base.z = ball_pos
        self._telemetry_pub.publish(telem)

        self.get_logger().info(
            "raw_action=[" + ", ".join(f"{a:+.3f}" for a in raw_action) + "]"
            f"  ball_base=({ball_pos[0]:+.3f},{ball_pos[1]:+.3f},{ball_pos[2]:+.3f})"
            f"  pass_through={self._obs_builder.pass_through_count}",
            throttle_duration_sec=0.5,
        )

        self._prev_action = raw_action  # feedback as observation component 9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LiveCatchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
