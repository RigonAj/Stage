"""Live catch loop — perception -> policy -> (optional) robot command (archi §2, §4.3).

One rclpy node, one 60 Hz timer. The hot path ``observation -> inference -> action``
runs as DIRECT CALLS into the pure modules (no intra-process topic, no DDS hop,
archi §2):

    BallState ─▶ ball_frame ─▶ ObservationBuilder ─▶ PolicyRunner ─▶ raw action
    /joint_states ─▶ cache (reordered)              (TF: frame_id→base, base→hoop)
                                                            │
                  ActionMapper ─▶ SafetyLimiter ─▶ CommandStreamer ─▶ command  ◀┘
                  (metadata)       (clip+rate+accel)  (Float64MultiArray)

Two modes, behind ``enable_command`` (default **False = DRY-RUN**):

  DRY-RUN (default): the full pipeline runs (so telemetry and the comp-9 feedback
  are exactly what they would be live), but **no robot command is published**. With
  ``dry_run_simulate`` (default true) the node integrates a VIRTUAL joint state that
  follows the safe target, so ``CatchTelemetry.joint_target`` (the green policy ghost)
  moves as the real arm would — a closed-loop preview without touching hardware. With
  it false the loop is open against the frozen measured pose, so the ghost can only
  ever reach one rate-limited step (~v_safe·dt) from the start and barely moves.

  COMMAND (``enable_command:=true``): the safe joint target is streamed to the
  ``forward_position_controller``. The node auto-switches the controller
  (scaled_joint_trajectory_controller -> forward_position_controller, archi §8) and
  enforces a Watchdog controlled-stop (stale perception / loop overrun / tracking
  error -> hold in place, archi §9). It refuses to command without a policy model.

The comp-9 observation feedback is the ActionMapper's recorded action. Legacy
absolute exports feed back the raw action; current incremental Isaac exports
feed back the clipped action, matching ``firsttraining_env``.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

from ur3e_catch_msgs.msg import BallState, CatchTelemetry
from ur3e_live_catch.action import ActionMapper
from ur3e_live_catch.ball_frame import BallFrameTransformer, FrameError, RigidTransform
from ur3e_live_catch.joint_order import JointOrderError, reorder_by_name
from ur3e_live_catch.limits import (
    build_joint_bounds,
    load_ur3e_joint_limits,
    v_safe_vector,
    UR3E_JOINT_LIMITS_PATH,
)
from ur3e_live_catch.observation import ObservationBuilder
from ur3e_live_catch.policy_runtime import PolicyRunner, load_metadata
from ur3e_live_catch.safety import JointBound, SafetyLimiter, Watchdog
from ur3e_live_catch.streaming import CommandStreamer

# Default model: prefer ONNX on the ROS PC, fall back to TorchScript exports.
CANONICAL_ONNX_MODEL = "data/models/policy_deterministic.onnx"
CANONICAL_TORCH_MODEL = "data/models/policy_deterministic.ts"
FALLBACK_ONNX_MODEL = (
    "data/ur3e_rollouts/2026-05-26_17-13-29_ppo_torch/exports/policy_deterministic.onnx"
)
FALLBACK_TORCH_MODEL = (
    "data/ur3e_rollouts/2026-05-26_17-13-29_ppo_torch/exports/policy_deterministic.ts"
)
CONTROLLER_POLL_PERIOD_S = 0.5


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
        self.declare_parameter("disk_radius", 0.1)

        # --- step 6: action mapping, safety, streaming ---------------------------
        self.declare_parameter("enable_command", False)  # False => DRY-RUN (no robot command)
        # DRY-RUN preview: integrate a virtual arm that follows the safe target so the
        # ghost moves (closed loop). False => open loop vs the frozen pose (ghost ~still).
        self.declare_parameter("dry_run_simulate", True)
        self.declare_parameter("action_mode", "faithful")  # faithful resolves from metadata; safe is manual
        self.declare_parameter("command_topic", "/forward_position_controller/commands")
        self.declare_parameter("command_controller", "forward_position_controller")
        self.declare_parameter("trajectory_controller", "scaled_joint_trajectory_controller")
        self.declare_parameter("auto_switch_controller", True)
        self.declare_parameter("command_substeps", 1)
        self.declare_parameter("v_safe_factor", 0.5)   # v_safe = URDF max_velocity * factor
        self.declare_parameter("a_safe", 10.0)         # rad/s^2 accel cap (tune on hw)
        self.declare_parameter("loop_budget_s", 0.02)  # watchdog: compute budget per tick
        self.declare_parameter("max_tracking_error", 0.5)  # rad; |q - last_command|
        self.declare_parameter("joint_limits_path", UR3E_JOINT_LIMITS_PATH)

        self._base_frame = str(self.get_parameter("base_frame").value)
        self._hoop_frame = str(self.get_parameter("hoop_frame").value)
        self._loop_hz = float(self.get_parameter("loop_hz").value)
        self._dt = 1.0 / self._loop_hz
        self._enable_command = bool(self.get_parameter("enable_command").value)
        self._dry_run_simulate = bool(self.get_parameter("dry_run_simulate").value)
        self._auto_switch = bool(self.get_parameter("auto_switch_controller").value)
        self._command_controller = str(self.get_parameter("command_controller").value)
        self._trajectory_controller = str(self.get_parameter("trajectory_controller").value)

        self._joint_pos: Optional[list[float]] = None
        self._joint_vel: Optional[list[float]] = None
        self._ball_msg: Optional[BallState] = None
        self._prev_action: list[float] = [0.0] * 6
        self._policy_metadata: dict[str, Any] = {}
        self._policy_model_path: Optional[Path] = None
        # Virtual arm for the dry-run closed-loop preview (None => re-init from the
        # measured pose on the next valid-ball tick, i.e. each throw starts fresh).
        self._sim_q: Optional[list[float]] = None
        self._sim_vel: Optional[list[float]] = None

        self._ball_frame = BallFrameTransformer(
            base_frame=self._base_frame,
            units=str(self.get_parameter("units").value),
            stale_after_s=float(self.get_parameter("stale_after_s").value),
        )
        self._policy = self._make_policy()
        self._obs_builder = ObservationBuilder(disk_radius=self._disk_radius_from_metadata())

        # Command pipeline (built always; used only when commanding is allowed).
        self._bounds = self._make_bounds()
        action_mode = self._resolve_action_mode(str(self.get_parameter("action_mode").value))
        self._action_mapper = ActionMapper(
            mode=action_mode,
            action_scale=float(self._policy_metadata.get("action_scale", 0.5)),
            v_safe=v_safe_vector(self._bounds),
            a_safe=[bound.a_safe for bound in self._bounds],
            position_lower=[bound.min_position for bound in self._bounds],
            position_upper=[bound.max_position for bound in self._bounds],
            dt=self._dt,
        )
        self._safety = SafetyLimiter(self._bounds, self._dt)
        self._streamer = CommandStreamer(substeps=int(self.get_parameter("command_substeps").value))
        self._watchdog = Watchdog(
            stale_after_s=float(self.get_parameter("stale_after_s").value),
            loop_budget_s=float(self.get_parameter("loop_budget_s").value),
            max_tracking_error=float(self.get_parameter("max_tracking_error").value),
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(JointState, str(self.get_parameter("joint_states_topic").value),
                                 self._on_joint_states, 10)
        self.create_subscription(BallState, str(self.get_parameter("ball_topic").value),
                                 self._on_ball, 10)
        self._telemetry_pub = self.create_publisher(
            CatchTelemetry, str(self.get_parameter("telemetry_topic").value), 10
        )

        # Controller switching / command publisher only exist in command mode.
        self._command_pub = None
        self._switch_client = None
        self._list_client = None
        self._command_controller_active = False
        self._switch_in_flight = False
        self._halted = False
        self._command_setup_done = False
        # Runtime command toggle (web UI "run on real robot"): flip enable_command
        # without restarting the node. Always offered, even when starting in dry-run.
        self._enable_command_srv = self.create_service(
            SetBool, "~/enable_command", self._on_enable_command
        )
        if self._enable_command:
            self._setup_command_mode()

        self._timer = self.create_timer(self._dt, self._on_tick)
        mode = "COMMAND" if self._enable_command else "DRY-RUN"
        self.get_logger().info(
            f"live_catch_node {mode} @ {self._loop_hz} Hz, action_mode="
            f"{self._action_mapper.mode}"
            + ("" if self._enable_command else " (no robot command emitted)")
        )

    # --- setup ---------------------------------------------------------------

    def _make_policy(self) -> Optional[PolicyRunner]:
        configured = str(self.get_parameter("model_path").value)
        default_candidates = [
            CANONICAL_ONNX_MODEL,
            CANONICAL_TORCH_MODEL,
            FALLBACK_ONNX_MODEL,
            FALLBACK_TORCH_MODEL,
        ]
        candidates = [c for c in ([configured] if configured else default_candidates) if c]
        self._policy_metadata = {}
        self._policy_model_path = None
        for cand in candidates:
            model_path = Path(cand)
            if not model_path.exists():
                continue
            try:
                # Eager load so a missing backend (torch/onnx) or a bad export
                # fails HERE, not inside the 60 Hz timer (which would kill the node).
                runner = PolicyRunner(model_path).load()
            except Exception as exc:  # ImportError, torch load errors, ...
                self.get_logger().warn(f"policy model {cand} failed to load ({exc}); trying next")
                continue
            metadata_path = model_path.with_name("policy_metadata.json")
            if metadata_path.exists():
                try:
                    self._policy_metadata = load_metadata(metadata_path)
                    self.get_logger().info(f"policy metadata loaded: {metadata_path}")
                except Exception as exc:
                    self._policy_metadata = {}
                    self.get_logger().warn(f"policy metadata {metadata_path} failed to load: {exc}")
            else:
                self.get_logger().warn(f"policy metadata not found next to model: {metadata_path}")
            self._policy_model_path = model_path
            self.get_logger().info(f"policy model loaded: {model_path}")
            return runner
        self.get_logger().warn(
            f"no usable policy model (looked for {candidates}); "
            "running observation-only, action will be zeros"
        )
        return None

    def _resolve_action_mode(self, configured: str) -> str:
        mode = configured.strip() or "faithful"
        metadata_incremental = self._metadata_uses_incremental_actions()
        if mode in ("auto", "metadata"):
            return "incremental" if metadata_incremental else "faithful"
        if mode == "faithful" and metadata_incremental:
            self.get_logger().info("action_mode=faithful resolved to incremental from policy metadata")
            return "incremental"
        return mode

    def _metadata_uses_incremental_actions(self) -> bool:
        semantics = str(self._policy_metadata.get("action_semantics", "")).lower()
        return (
            "incremental joint target integrator" in semantics
            or "previous joint_position_target" in semantics
            or "previous joint position target" in semantics
        )

    def _make_bounds(self):
        metadata_bounds = self._bounds_from_metadata()
        if metadata_bounds is not None:
            self.get_logger().info("using policy_metadata.json joint limits for action/safety bounds")
            return metadata_bounds
        path = str(self.get_parameter("joint_limits_path").value)
        limits = load_ur3e_joint_limits(path)
        return build_joint_bounds(
            limits,
            v_safe_factor=float(self.get_parameter("v_safe_factor").value),
            a_safe=float(self.get_parameter("a_safe").value),
        )

    def _metadata_vector(self, key: str) -> Optional[list[float]]:
        value = self._policy_metadata.get(key)
        if not isinstance(value, list) or len(value) != 6:
            return None
        try:
            return [float(item) for item in value]
        except (TypeError, ValueError):
            return None

    def _disk_radius_from_metadata(self) -> float:
        for key in ("disk_radius_m", "disk_radius"):
            if key not in self._policy_metadata:
                continue
            try:
                value = float(self._policy_metadata[key])
            except (TypeError, ValueError):
                continue
            if value > 0.0:
                return value
        return float(self.get_parameter("disk_radius").value)

    def _bounds_from_metadata(self) -> Optional[list[JointBound]]:
        lower = self._metadata_vector("joint_position_lower_rad")
        upper = self._metadata_vector("joint_position_upper_rad")
        v_safe = self._metadata_vector("joint_velocity_safe_rad_s")
        a_safe = self._metadata_vector("joint_acceleration_safe_rad_s2")
        if lower is None or upper is None or v_safe is None or a_safe is None:
            return None
        return [
            JointBound(
                min_position=lower[i],
                max_position=upper[i],
                v_safe=v_safe[i],
                a_safe=a_safe[i],
            )
            for i in range(6)
        ]

    def _setup_command_mode(self) -> bool:
        """Create the command publisher + controller-switch clients (idempotent).

        Callable from ``__init__`` (when starting in command mode) or later from the
        ``~/enable_command`` service. Returns False and stays in dry-run when there is
        no policy.
        """
        if self._policy is None:
            self.get_logger().error(
                "enable_command requested but no policy model loaded: refusing to "
                "command. Staying in dry-run."
            )
            self._enable_command = False
            return False
        if self._command_setup_done:
            return True
        from controller_manager_msgs.srv import ListControllers, SwitchController

        self._command_pub = self.create_publisher(
            Float64MultiArray, str(self.get_parameter("command_topic").value), 10
        )
        self._switch_client = self.create_client(SwitchController, "/controller_manager/switch_controller")
        self._list_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.create_timer(CONTROLLER_POLL_PERIOD_S, self._poll_controllers)
        self._command_setup_done = True
        self.get_logger().warn(
            "COMMAND mode armed: robot WILL move when commanding is allowed. v_safe="
            + ", ".join(f"{b.v_safe:.3f}" for b in self._bounds)
            + " rad/s. Keep the E-stop in hand (archi §9)."
        )
        return True

    def _on_enable_command(self, request, response):
        """Toggle command mode at runtime (``~/enable_command`` SetBool service)."""
        if request.data:
            if not self._setup_command_mode():
                response.success = False
                response.message = "no policy model loaded; staying in dry-run"
                return response
            self._enable_command = True
            response.success = True
            response.message = "command mode ENABLED (robot will move)"
            self.get_logger().warn("enable_command service: COMMAND mode ON")
        else:
            was_on = self._enable_command
            self._enable_command = False
            self._halted = False
            # Restore the trajectory controller so normal motion (jog/rollout) works.
            if was_on and self._command_setup_done:
                self._request_switch_back()
            response.success = True
            response.message = "command mode disabled (dry-run)"
            self.get_logger().info("enable_command service: COMMAND mode OFF")
        return response

    # --- subscriptions -------------------------------------------------------

    def _on_joint_states(self, msg: JointState) -> None:
        try:
            self._joint_pos = reorder_by_name(msg.name, msg.position)
            self._joint_vel = reorder_by_name(msg.name, msg.velocity) if msg.velocity else [0.0] * 6
        except (JointOrderError, ValueError) as exc:
            self.get_logger().warn(f"joint_states ignored: {exc}", throttle_duration_sec=2.0)

    def _on_ball(self, msg: BallState) -> None:
        self._ball_msg = msg

    # --- controller switching (command mode) ---------------------------------

    def _poll_controllers(self) -> None:
        from controller_manager_msgs.srv import ListControllers

        if self._list_client is None or not self._list_client.service_is_ready():
            return
        future = self._list_client.call_async(ListControllers.Request())
        future.add_done_callback(self._on_controller_list)

    def _on_controller_list(self, future) -> None:
        try:
            response = future.result()
        except Exception:
            return
        active = False
        for controller in response.controller:
            if controller.name == self._command_controller:
                active = controller.state == "active"
                break
        if active and not self._command_controller_active:
            self.get_logger().info(f"{self._command_controller} is active — streaming enabled")
        self._command_controller_active = active

    def _request_switch_once(self) -> None:
        from controller_manager_msgs.srv import SwitchController

        if self._switch_in_flight or self._switch_client is None:
            return
        if not self._switch_client.service_is_ready():
            self.get_logger().warn("switch_controller service not ready yet", throttle_duration_sec=2.0)
            return
        req = SwitchController.Request()
        req.activate_controllers = [self._command_controller]
        req.deactivate_controllers = [self._trajectory_controller]
        req.strictness = SwitchController.Request.BEST_EFFORT
        self._switch_in_flight = True
        self.get_logger().info(
            f"switching controllers: -{self._trajectory_controller} +{self._command_controller}"
        )
        self._switch_client.call_async(req).add_done_callback(self._on_switch_done)

    def _on_switch_done(self, future) -> None:
        self._switch_in_flight = False
        try:
            ok = bool(future.result().ok)
        except Exception as exc:
            self.get_logger().error(f"switch_controller failed: {exc}")
            return
        if not ok:
            self.get_logger().error("switch_controller returned ok=false; will retry")

    def _request_switch_back(self) -> None:
        """Restore the trajectory controller when leaving command mode (best effort)."""
        from controller_manager_msgs.srv import SwitchController

        if self._switch_client is None or not self._switch_client.service_is_ready():
            self.get_logger().warn(
                "switch_controller not ready; cannot restore the trajectory controller",
                throttle_duration_sec=2.0,
            )
            return
        req = SwitchController.Request()
        req.activate_controllers = [self._trajectory_controller]
        req.deactivate_controllers = [self._command_controller]
        req.strictness = SwitchController.Request.BEST_EFFORT
        self.get_logger().info(
            f"switching controllers back: -{self._command_controller} +{self._trajectory_controller}"
        )
        self._switch_client.call_async(req).add_done_callback(self._on_switch_done)

    def _commanding_allowed(self) -> bool:
        if not self._enable_command or self._policy is None:
            return False
        if not self._command_controller_active:
            if self._auto_switch:
                self._request_switch_once()
            return False
        return True

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

    def _disk_pose(self, *, commanding: bool = False):
        """Return (disk_pos_base, disk_normal_base) from TF, else dry-run fallback."""
        try:
            tf_msg = self._tf_buffer.lookup_transform(self._base_frame, self._hoop_frame, rclpy.time.Time())
            tf = _transform_from_msg(tf_msg)
            normal = RigidTransform(quaternion=tf.quaternion).apply((0.0, 0.0, 1.0))
            return list(tf.translation), list(normal)
        except Exception as exc:
            if commanding:
                self.get_logger().error(
                    f"TF {self._base_frame}->{self._hoop_frame} unavailable in command mode: {exc}",
                    throttle_duration_sec=2.0,
                )
                return None
            self.get_logger().warn(
                f"TF {self._base_frame}->{self._hoop_frame} unavailable, using disk fallback: {exc}",
                throttle_duration_sec=2.0,
            )
            return (
                [float(x) for x in self.get_parameter("disk_pos_fallback").value],
                [float(x) for x in self.get_parameter("disk_normal_fallback").value],
            )

    # --- command helpers -----------------------------------------------------

    def _publish_commands(self, commands) -> None:
        if self._command_pub is None:
            return
        for cmd in commands:
            msg = Float64MultiArray()
            msg.data = [float(x) for x in cmd]
            self._command_pub.publish(msg)

    def _controlled_stop(self, q: list[float], reasons) -> None:
        """Hold in place and reset the rate/accel memory so resume ramps from rest."""
        if not self._commanding_allowed():
            return
        self._publish_commands([self._streamer.hold(fallback=q)])
        self._safety.reset()
        if not self._halted:
            self.get_logger().error("WATCHDOG stop -> holding: " + "; ".join(reasons))
        self._halted = True

    # --- hot loop ------------------------------------------------------------

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _reset_sim(self) -> None:
        """Drop the virtual arm so the next throw's preview restarts from the real pose."""
        self._sim_q = None
        self._sim_vel = None
        self._reset_policy_state()

    def _reset_policy_state(self) -> None:
        self._safety.reset()
        self._action_mapper.reset()
        self._obs_builder.reset()
        self._prev_action = [0.0] * 6

    def _on_tick(self) -> None:
        t_start = time.perf_counter()
        if self._joint_pos is None:
            self.get_logger().warn("waiting for /joint_states", throttle_duration_sec=2.0)
            return
        commanding = self._commanding_allowed()
        simulate = self._dry_run_simulate and not commanding
        real_q = self._joint_pos
        if self._ball_msg is None or not self._ball_msg.valid:
            self.get_logger().warn("waiting for a valid BallState", throttle_duration_sec=2.0)
            self._controlled_stop(real_q, ["no_valid_ball"])
            self._reset_sim()  # re-arm policy state for the next throw
            return

        # Effective joint state fed to policy/safety: the measured arm when commanding,
        # else a VIRTUAL arm that follows the policy so the dry-run ghost actually moves
        # (closed-loop preview). It advances because the safety rate-limit is taken
        # against the virtual pose, not the frozen measurement (which never moves in
        # dry-run, pinning the open-loop target to ~one v_safe·dt step from start).
        if simulate:
            if self._sim_q is None:
                self._sim_q = list(real_q)
                self._sim_vel = [0.0] * 6
                self._safety.reset()
            q = self._sim_q
            joint_vel = self._sim_vel
        else:
            self._sim_q = None
            q = real_q
            joint_vel = self._joint_vel

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
            self._controlled_stop(real_q, ["ball_frame_rejected"])
            return

        disk_pose = self._disk_pose(commanding=commanding)
        if disk_pose is None:
            self._controlled_stop(real_q, ["hoop_tf_unavailable"])
            return
        disk_pos, disk_normal = disk_pose

        obs = self._obs_builder.build(
            joint_pos=q,
            joint_vel=joint_vel,
            disk_pos=disk_pos,
            disk_normal=disk_normal,
            ball_pos=ball_pos,
            ball_vel=ball_vel,
            prev_action=self._prev_action,
        )

        raw_action = self._policy.infer(obs) if self._policy is not None else [0.0] * 6

        # Map -> safety (always, so dry-run telemetry == what would be commanded).
        target = self._action_mapper.map(raw_action, q)
        report = self._safety.limit(target, q)
        safe_target = report.target
        # comp-9 feedback: ActionMapper records RAW (faithful) or CLIPPED (safe).
        self._prev_action = self._action_mapper.prev_action

        # Advance the virtual arm: it tracks the safe target so the preview accumulates.
        if simulate:
            self._sim_vel = [(safe_target[i] - self._sim_q[i]) / self._dt for i in range(6)]
            self._sim_q = list(safe_target)

        # Watchdog (archi §9): perception age, compute budget, tracking error.
        perception_age = self._now_s() - stamp_s
        tracking_error = None
        last_cmd = self._streamer.last_command
        if last_cmd is not None:
            tracking_error = max(abs(real_q[i] - last_cmd[i]) for i in range(6))
        loop_compute = time.perf_counter() - t_start
        ok, reasons = self._watchdog.check(
            perception_age_s=perception_age,
            loop_time_s=loop_compute,
            tracking_error=tracking_error,
        )

        if commanding:
            if ok:
                self._publish_commands(self._streamer.stream(safe_target))
                self._halted = False
            else:
                self._controlled_stop(real_q, reasons)

        self._publish_telemetry(obs, raw_action, safe_target, ball_pos, ball_vel,
                                perception_age, loop_compute)
        self.get_logger().info(
            "raw_action=[" + ", ".join(f"{a:+.3f}" for a in raw_action) + "]"
            f"  ball_base=({ball_pos[0]:+.3f},{ball_pos[1]:+.3f},{ball_pos[2]:+.3f})"
            f"  pass_through={self._obs_builder.pass_through_count}"
            f"  mode={'cmd' if commanding else ('sim' if simulate else 'dry')}",
            throttle_duration_sec=0.5,
        )

    def _publish_telemetry(self, obs, raw_action, safe_target, ball_pos, ball_vel,
                           perception_age, loop_compute) -> None:
        telem = CatchTelemetry()
        telem.observation = [float(x) for x in obs]
        telem.raw_action = [float(x) for x in raw_action]
        telem.joint_target = [float(x) for x in safe_target]
        telem.ball_base.x, telem.ball_base.y, telem.ball_base.z = ball_pos
        # Fields below exist once ur3e_catch_msgs is rebuilt (steps 7-8); guarded so
        # the node still runs against an older message build.
        if hasattr(telem, "ball_vel_base"):
            telem.ball_vel_base.x, telem.ball_vel_base.y, telem.ball_vel_base.z = ball_vel
        if hasattr(telem, "perception_age_s"):
            telem.perception_age_s = float(perception_age)
        if hasattr(telem, "loop_compute_s"):
            telem.loop_compute_s = float(loop_compute)
        if hasattr(telem, "command_enabled"):
            telem.command_enabled = bool(self._enable_command)
        self._telemetry_pub.publish(telem)


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
