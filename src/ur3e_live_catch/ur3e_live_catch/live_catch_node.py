"""Live catch loop — perception -> policy -> (optional) robot command (archi §2, §4.3).

One rclpy node, one 60 Hz timer. The hot path ``observation -> inference -> action``
runs as DIRECT CALLS into the pure modules (no intra-process topic, no DDS hop,
archi §2):

    BallState ─▶ ball_frame ─▶ ObservationBuilder ─▶ PolicyRunner ─▶ raw action
    /joint_states ─▶ cache (reordered)     (TF: frame_id→base_link, base_link→hoop)
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
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

from ur3e_catch_msgs.msg import BallState, CatchTelemetry
from ur3e_live_catch.action import ActionMapper
from ur3e_live_catch.ball_frame import BallFrameTransformer, FrameError, RigidTransform
from ur3e_live_catch.joint_order import JointOrderError, reorder_by_name
from ur3e_live_catch.joint_order import JOINT_ORDER
from ur3e_live_catch.limits import (
    build_joint_bounds,
    load_ur3e_joint_limits,
    scale_bounds,
    v_safe_vector,
    UR3E_JOINT_LIMITS_PATH,
)
from ur3e_live_catch.observation import ObservationBuilder
from ur3e_live_catch.policy_runtime import PolicyRunner, load_metadata, policy_model_candidates
from ur3e_live_catch.safety import JointBound, SafetyLimiter, Watchdog, start_pose_violations
from ur3e_live_catch.streaming import CommandStreamer, step_toward

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
EXPECTED_OBSERVATION_SPACE = 33
EXPECTED_ACTION_SPACE = 6
EXPECTED_OBSERVATION_FRAME = "base_link"


def _stamp_to_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _transform_from_msg(tf_msg) -> RigidTransform:
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    return RigidTransform(translation=(t.x, t.y, t.z), quaternion=(q.x, q.y, q.z, q.w))


def _metadata_uses_incremental_actions(metadata: dict[str, Any]) -> bool:
    semantics = str(metadata.get("action_semantics", "")).lower()
    return (
        "incremental joint target integrator" in semantics
        or "previous joint_position_target" in semantics
        or "previous joint position target" in semantics
    )


class LiveCatchNode(Node):
    def __init__(self) -> None:
        super().__init__("live_catch_node")
        self.declare_parameter("loop_hz", 60.0)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("hoop_frame", "hoop_center")
        self.declare_parameter("ball_topic", "ball_state")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("telemetry_topic", "catch_telemetry")
        self.declare_parameter("units", "m")
        self.declare_parameter("model_path", "")  # "" => canonical then fallback
        self.declare_parameter("stale_after_s", 0.1)
        # Dry-run fallback when no TF base_link->hoop is available. Command mode
        # refuses fallback; serious inference tests should publish hoop_center TF.
        self.declare_parameter("disk_pos_fallback", [0.0, 0.0, 0.5])
        self.declare_parameter("disk_normal_fallback", [0.0, 0.0, 1.0])
        self.declare_parameter("disk_radius", 0.05)

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
        # High-rate command interpolation: the UR driver checks consecutive servoj
        # set-points every 2 ms, so raw 60 Hz steps (v_safe*dt) get rejected as
        # velocity violations. > loop_hz => a fast timer publishes interpolated
        # set-points; <= loop_hz => legacy one-command-per-tick behavior.
        self.declare_parameter("command_rate_hz", 500.0)
        # Bring-up slowdown of metadata bounds (v_safe AND a_safe). 1.0 = trained
        # contract; < 1.0 diverges from training but keeps early hardware runs slow.
        self.declare_parameter("v_safe_scale", 1.0)
        # Refuse the FIRST command of a session when a measured joint exceeds this:
        # catches ±2π-wrapped /joint_states branches (wrist at "360°") that the UR
        # controller would reject as a full-turn jump and then ignore all commands.
        self.declare_parameter("start_pose_limit_rad", 3.0)
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
        self._action_mapper = self._make_action_mapper()
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
        # High-rate command path: the 60 Hz tick writes the safe target here; the
        # fast timer walks the published command toward it in v_safe*dt_cmd steps.
        self._cmd_goal: Optional[list[float]] = None
        self._cmd_timer = None
        self._command_rate_hz = float(self.get_parameter("command_rate_hz").value)
        self._start_pose_limit = float(self.get_parameter("start_pose_limit_rad").value)
        self._last_switch_request_s = float("-inf")
        # Runtime command toggle (web UI "run on real robot"): flip enable_command
        # without restarting the node. Always offered, even when starting in dry-run.
        self._enable_command_srv = self.create_service(
            SetBool, "~/enable_command", self._on_enable_command
        )
        self.add_on_set_parameters_callback(self._on_set_parameters)
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

    def _default_model_candidates(self, configured: str) -> list[str]:
        default_candidates = [
            CANONICAL_ONNX_MODEL,
            CANONICAL_TORCH_MODEL,
            FALLBACK_ONNX_MODEL,
            FALLBACK_TORCH_MODEL,
        ]
        return policy_model_candidates(configured, default_candidates)

    def _validate_policy_metadata(self, metadata: dict[str, Any], metadata_path: Path) -> None:
        """Validate the deployment-critical parts of policy_metadata.json."""
        if not metadata:
            raise ValueError("metadata is empty")
        obs = int(metadata.get("observation_space", -1))
        action = int(metadata.get("action_space", -1))
        if obs != EXPECTED_OBSERVATION_SPACE:
            raise ValueError(f"observation_space {obs} != {EXPECTED_OBSERVATION_SPACE}")
        if action != EXPECTED_ACTION_SPACE:
            raise ValueError(f"action_space {action} != {EXPECTED_ACTION_SPACE}")
        joint_names = metadata.get("joint_names")
        if joint_names != [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]:
            raise ValueError("joint_names do not match the UR3e live-catch joint order")
        if "dt_s" in metadata:
            dt_s = float(metadata["dt_s"])
            if abs(dt_s - self._dt) > 1.0e-6:
                raise ValueError(f"dt_s {dt_s:g} does not match loop dt {self._dt:g}")
        semantics = str(metadata.get("action_semantics", ""))
        if not semantics:
            raise ValueError("action_semantics is missing")
        observation_frame = str(metadata.get("observation_frame", "") or "")
        if observation_frame and observation_frame != EXPECTED_OBSERVATION_FRAME:
            raise ValueError(
                f"observation_frame {observation_frame!r} != {EXPECTED_OBSERVATION_FRAME!r}"
            )
        if observation_frame and observation_frame != self._base_frame:
            raise ValueError(
                f"observation_frame {observation_frame!r} does not match live base_frame {self._base_frame!r}"
            )
        if _metadata_uses_incremental_actions(metadata):
            required = (
                "joint_velocity_safe_rad_s",
                "joint_acceleration_safe_rad_s2",
                "joint_position_lower_rad",
                "joint_position_upper_rad",
            )
            missing = [key for key in required if self._metadata_vector_from(metadata, key) is None]
            if missing:
                raise ValueError(f"incremental metadata missing {', '.join(missing)}")
        self.get_logger().info(f"policy metadata validated: {metadata_path}")

    def _load_policy_bundle(
        self,
        configured: str,
        *,
        allow_missing: bool,
    ) -> tuple[Optional[PolicyRunner], dict[str, Any], Optional[Path]]:
        candidates = self._default_model_candidates(configured)
        last_error: Optional[str] = None
        for cand in candidates:
            model_path = Path(cand)
            if not model_path.exists():
                continue
            try:
                # Eager load so a missing backend (torch/onnx) or a bad export
                # fails HERE, not inside the 60 Hz timer (which would kill the node).
                runner = PolicyRunner(model_path).load()
            except Exception as exc:  # ImportError, torch load errors, ...
                last_error = f"{cand}: {exc}"
                self.get_logger().warn(f"policy model {cand} failed to load ({exc}); trying next")
                continue
            metadata_path = model_path.with_name("policy_metadata.json")
            metadata: dict[str, Any] = {}
            if metadata_path.exists():
                try:
                    metadata = load_metadata(metadata_path)
                    self._validate_policy_metadata(metadata, metadata_path)
                    self.get_logger().info(f"policy metadata loaded: {metadata_path}")
                except Exception as exc:
                    last_error = f"{metadata_path}: {exc}"
                    self.get_logger().warn(f"policy metadata {metadata_path} rejected: {exc}; trying next")
                    continue
            else:
                last_error = f"metadata not found next to model: {metadata_path}"
                self.get_logger().warn(last_error)
                continue
            return runner, metadata, model_path
        message = f"no usable policy model (looked for {candidates})"
        if last_error:
            message += f"; last error: {last_error}"
        if allow_missing:
            self.get_logger().warn(message + "; running observation-only, action will be zeros")
            return None, {}, None
        raise RuntimeError(message)

    def _make_policy(self) -> Optional[PolicyRunner]:
        configured = str(self.get_parameter("model_path").value)
        runner, metadata, model_path = self._load_policy_bundle(configured, allow_missing=True)
        self._policy_metadata = metadata
        self._policy_model_path = model_path
        if model_path is not None:
            self.get_logger().info(f"policy model loaded: {model_path}")
        return runner

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
        return _metadata_uses_incremental_actions(self._policy_metadata)

    def _make_action_mapper(self) -> ActionMapper:
        action_mode = self._resolve_action_mode(str(self.get_parameter("action_mode").value))
        return ActionMapper(
            mode=action_mode,
            action_scale=float(self._policy_metadata.get("action_scale", 0.5)),
            v_safe=v_safe_vector(self._bounds),
            a_safe=[bound.a_safe for bound in self._bounds],
            position_lower=[bound.min_position for bound in self._bounds],
            position_upper=[bound.max_position for bound in self._bounds],
            dt=self._dt,
        )

    def _rebuild_policy_dependent_state(self) -> None:
        self._obs_builder = ObservationBuilder(disk_radius=self._disk_radius_from_metadata())
        self._bounds = self._make_bounds()
        self._action_mapper = self._make_action_mapper()
        self._safety = SafetyLimiter(self._bounds, self._dt)
        self._streamer = CommandStreamer(substeps=int(self.get_parameter("command_substeps").value))
        self._cmd_goal = None
        self._reset_sim()

    def _set_model_path_runtime(self, configured: str) -> None:
        runner, metadata, model_path = self._load_policy_bundle(configured, allow_missing=False)
        assert runner is not None
        assert model_path is not None
        self._policy = runner
        self._policy_metadata = metadata
        self._policy_model_path = model_path
        self._rebuild_policy_dependent_state()
        self.get_logger().info(
            f"policy model switched: {model_path}, action_mode={self._action_mapper.mode}"
        )

    def _on_set_parameters(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name != "model_path":
                continue
            configured = str(parameter.value or "")
            if configured == str(self.get_parameter("model_path").value or ""):
                continue
            if self._enable_command:
                return SetParametersResult(
                    successful=False,
                    reason="cannot change model_path while command mode is enabled",
                )
            try:
                self._set_model_path_runtime(configured)
            except Exception as exc:
                return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

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
        return self._metadata_vector_from(self._policy_metadata, key)

    @staticmethod
    def _metadata_vector_from(metadata: dict[str, Any], key: str) -> Optional[list[float]]:
        value = metadata.get(key)
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
        bounds = [
            JointBound(
                min_position=lower[i],
                max_position=upper[i],
                v_safe=v_safe[i],
                a_safe=a_safe[i],
            )
            for i in range(6)
        ]
        scale = float(self.get_parameter("v_safe_scale").value)
        if scale != 1.0:
            bounds = scale_bounds(bounds, scale)
            self.get_logger().warn(
                f"v_safe_scale={scale:g}: metadata velocity/acceleration bounds scaled "
                "down for bring-up (closed-loop dynamics diverge from training)"
            )
        return bounds

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
        if self._command_rate_hz > self._loop_hz:
            self._cmd_timer = self.create_timer(1.0 / self._command_rate_hz, self._on_command_tick)
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
            # Fresh command session: never hold/ramp from a command recorded before
            # the robot was moved by other means (trajectory controller, freedrive).
            self._streamer.reset()
            self._cmd_goal = None
            self._enable_command = True
            response.success = True
            response.message = "command mode ENABLED (robot will move)"
            self.get_logger().warn("enable_command service: COMMAND mode ON")
        else:
            was_on = self._enable_command
            self._enable_command = False
            self._halted = False
            self._cmd_goal = None
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
        # The active flag refreshes on the 0.5 s poll; without a cooldown every
        # 60 Hz tick re-requests the same switch and spams controller_manager.
        now_s = self._now_s()
        if now_s - self._last_switch_request_s < CONTROLLER_POLL_PERIOD_S:
            return
        if not self._switch_client.service_is_ready():
            self.get_logger().warn("switch_controller service not ready yet", throttle_duration_sec=2.0)
            return
        self._last_switch_request_s = now_s
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

    def _commanding_allowed(self, *, request_switch: bool = True) -> bool:
        if not self._enable_command or self._policy is None:
            return False
        if not self._command_controller_active:
            if self._auto_switch and request_switch:
                self._request_switch_once()
            return False
        # Start-pose gate: before the FIRST command of a session, refuse to echo a
        # measured pose sitting on a ±2π-wrapped branch — the UR controller would
        # see a full-turn jump, reject it and latch "Ignoring commands". Re-checked
        # every tick so streaming arms itself once the operator fixes the pose.
        if self._streamer.last_command is None:
            if self._joint_pos is None:
                return False
            violations = start_pose_violations(
                self._joint_pos, JOINT_ORDER, self._start_pose_limit
            )
            if violations:
                self.get_logger().error(
                    "refusing to start command streaming: " + " | ".join(violations),
                    throttle_duration_sec=2.0,
                )
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

    def _fast_command_path(self) -> bool:
        return self._cmd_timer is not None

    def _submit_command_goal(self, target: list[float]) -> None:
        """Route a safe target to the robot: fast timer if enabled, else direct."""
        if self._fast_command_path():
            self._cmd_goal = list(target)
        else:
            self._publish_commands(self._streamer.stream(target))

    def _on_command_tick(self) -> None:
        """High-rate interpolation toward the current goal (see command_rate_hz)."""
        if self._cmd_goal is None or not self._commanding_allowed(request_switch=False):
            return
        current = self._streamer.last_command
        if current is None:
            if self._joint_pos is None:
                return
            current = list(self._joint_pos)
        dt_cmd = 1.0 / self._command_rate_hz
        max_deltas = [b.v_safe * dt_cmd for b in self._bounds]
        next_cmd = step_toward(current, self._cmd_goal, max_deltas)
        self._publish_commands([self._streamer.format(next_cmd)])

    def _controlled_stop(self, q: list[float], reasons) -> None:
        """Hold in place and reset policy/safety memory so resume ramps from ``q``."""
        self._safety.reset()
        self._action_mapper.reset(reference_q=q)
        self._prev_action = [0.0] * 6
        if not self._commanding_allowed():
            return
        hold_cmd = self._streamer.hold(fallback=q)
        if self._fast_command_path():
            self._cmd_goal = hold_cmd
        else:
            self._publish_commands([hold_cmd])
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
                self._submit_command_goal(safe_target)
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
