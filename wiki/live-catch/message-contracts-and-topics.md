# Message Contracts And Topics

> Sources: live-catch architecture, 2026-06-29; implementation status, 2026-06-29; package README, 2026-06-29; inconsistency review, 2026-06-29
> Raw: [Live-catch architecture](../../docs/Robot_Control/ur3e_live_catch_architecture.md); [Implementation status](../../docs/Robot_Control/ur3e_live_catch_implementation_status.md); [ur3e_live_catch README](../../src/ur3e_live_catch/README.md); [Incoherences](../../docs/incoherences_code_logique.md)

## Overview

This page is the compact contract map for the perception-to-live-catch ROS
interfaces. Read it before changing message fields, topics, timestamps or
consumer/producer assumptions.

## Core Messages

`ur3e_catch_msgs/BallState` is the contract between perception and live catch:

- `header.stamp`: event/perception time when available.
- `header.frame_id`: declared frame of `position`; must be nonempty.
- `position`: meters.
- `velocity`: optional; live catch can recompute/filter velocity.
- `valid`: whether the ball sample is usable.
- `confidence`: producer confidence when available.

`ur3e_catch_msgs/CatchTelemetry` is debug/visualization, not hot-path control:

- `observation`: 33-D policy observation.
- `raw_action`: 6-D policy output before mapping/safety.
- `joint_target`: safe target after mapping/limits, also filled in dry-run.
- `ball_base`: ball position transformed into robot base.
- latency and command state fields are used by UI and `latency_report`.

## Topics And Producers

| Topic | Producer | Consumer | Notes |
|---|---|---|---|
| `ball_state` | `ball_tracking_cpp`, `test_ball_node`, or legacy adapter | `live_catch_node`, UI/debug tools | Preferred perception contract. |
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
