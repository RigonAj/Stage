# Message Contracts And Topics

> Sources: live-catch architecture, 2026-06-29; implementation status, 2026-06-29; package README, 2026-06-29; inconsistency review, 2026-06-29; heartbeat telemetry change, 2026-07-02; ball regression publisher, 2026-07-03
> Raw: [Live-catch architecture](../../docs/Robot_Control/ur3e_live_catch_architecture.md); [Implementation status](../../docs/Robot_Control/ur3e_live_catch_implementation_status.md); [ur3e_live_catch README](../../src/ur3e_live_catch/README.md); [Incoherences](../../docs/incoherences_code_logique.md); [BallState.msg](../../src/ur3e_catch_msgs/msg/BallState.msg); [Ball regression node](../../src/ur3e_live_catch/ur3e_live_catch/ball_regression_node.py)

## Overview

This page is the compact contract map for the perception-to-live-catch ROS
interfaces. Read it before changing message fields, topics, timestamps or
consumer/producer assumptions.

## Core Messages

`ur3e_catch_msgs/BallState` is the contract between perception and live catch:

- `header.stamp`: event/perception time when available. Exception: the
  regression node stamps at EVALUATION time (now + lead), which is deliberate
  latency compensation — see
  [Observation Latency And Models](../sim-to-real/observation-latency-and-models.md).
- `header.frame_id`: declared frame of `position`; must be nonempty.
- `position`: meters.
- `velocity`: optional. Convention: exactly `(0,0,0)` means "not provided"
  (both the C++ tracker and `test_ball_node` leave it zero) and the live node
  falls back to its EMA finite-difference filter. The regression node fills it
  from the fit derivative; `live_catch_node` trusts it when
  `use_ball_state_velocity=true` (default), rotating it into `base_link`.
- `valid`: whether the ball sample is usable.
- `confidence`: producer confidence when available. The regression node makes
  it meaningful: support/residual quality, decaying while coasting.

`ur3e_catch_msgs/CatchTelemetry` is debug/visualization, not hot-path control:

- `observation`: 33-D policy observation (empty in idle heartbeats).
- `raw_action`: 6-D policy output before mapping/safety (empty in heartbeats).
- `joint_target`: safe target after mapping/limits, also filled in dry-run; in
  idle heartbeats it carries the measured pose so the UI ghost stays live.
- `ball_base`: historical field name; ball position transformed into the current
  policy frame `base_link`.
- `ball_valid`: false marks an idle heartbeat (no valid ball / no hoop TF) where
  ball fields are placeholders; the viewer hides the ball marker then.
- `command_enabled`: live node command mode; published every tick since
  2026-07-02 (heartbeats included) so the Web UI sees command state even while
  the trigger-mode ball is idle.
- latency and command state fields are used by UI and `latency_report`.

`test_ball_node` ends a parabola flight when the ball drops below `ground_z_m`
(default 0.05 m in `base_link`), matching Isaac's `ball_on_ground` episode
termination — an underground ball is out-of-distribution for the policy.

## Topics And Producers

| Topic | Producer | Consumer | Notes |
|---|---|---|---|
| `ball_state` | `ball_tracking_cpp`, `test_ball_node`, legacy adapter, or `ball_regression_node` | `live_catch_node`, UI/debug tools | Preferred perception contract. The tracker's `pose_source` selects trace (bring-up config) or legacy circle poses. |
| `ball_state_raw` | raw sources when `use_ball_regression:=true` | `ball_regression_node` | Launch re-points tracker/adapter/test-ball here; the regression republishes the fitted `BallState` on `ball_state` at 60 Hz (valid=False heartbeats before the start gate and after flight end). |
| `ball_position_3d_mm` | old tracker path | `float32_adapter.py` only | Legacy fallback; timestamps at reception. |
| `/joint_states` | UR driver or fake hardware | live catch, web UI | Must match canonical joint order through reorder helpers. |
| `/catch_telemetry` | `live_catch_node` | web UI, `latency_report` | Debug/visualization only. |
| `/forward_position_controller/commands` | `CommandStreamer` | ros2_control controller | Emitted only when command mode is enabled. |

## Rules

- Prefer native `ball_tracking_cpp -> BallState` for real perception.
- Use `use_adapter:=true` only for old tracker builds.
- Never infer a missing frame. Empty or unknown `frame_id` is a reject condition.
- Keep message changes synchronized across `ur3e_catch_msgs`, C++ tracker,
  `ur3e_live_catch`, `ur3e_web_ui` and tests.

## See Also

- [Live Catch Loop](live-catch-loop.md)
- [Frames And Transforms](../calibration/frames-and-transforms.md)
- [Safety And Commanding](safety-and-commanding.md)
