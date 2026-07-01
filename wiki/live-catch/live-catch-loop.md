# Live Catch Loop

> Sources: live-catch architecture, 2026-06-30; implementation status, 2026-06-30; package README, 2026-06-29
> Raw: [Live-catch architecture](../../docs/Robot_Control/ur3e_live_catch_architecture.md); [Implementation status](../../docs/Robot_Control/ur3e_live_catch_implementation_status.md); [Package README](../../src/ur3e_live_catch/README.md)

## Overview

`ur3e_live_catch` is the single-process 60 Hz live loop that transforms ball
state into policy observations, runs the exported policy, maps actions to safe
joint targets and optionally streams commands to the robot.

## Hot Path

```text
BallState + /joint_states
  -> ball_frame
  -> ObservationBuilder
  -> PolicyRunner
  -> ActionMapper
  -> SafetyLimiter
  -> CommandStreamer
```

The intermediate policy path is direct Python calls inside one process, not ROS
topics, to reduce latency.

## Main Modules

- `ball_frame.py`: frame transform and ball velocity estimate.
- `observation.py`: 33-D policy observation.
- `policy_runtime.py`: TorchScript/ONNX runtime.
- `action.py`: policy action mapping.
- `safety.py`: limits and watchdog.
- `streaming.py`: controller command path.
- `live_catch_node.py`: 60 Hz rclpy node.
- `test_ball_node.py`: artificial ball source.

## Contracts

- `enable_command=false` is the safe default.
- Unknown or empty `BallState.header.frame_id` must be rejected.
- `BallState` should be native from `ball_tracking_cpp`; the legacy adapter is a
  fallback for old builds.
- Command mode must refuse to run when no policy model is loaded.
- Current Isaac exports use metadata-driven incremental action mapping; legacy
  absolute exports remain supported for fallback/debug only.
- Safety remains independent from policy output.

## See Also

- [Current Status And Blockers](current-status-and-blockers.md)
- [Message Contracts And Topics](message-contracts-and-topics.md)
- [Safety And Commanding](safety-and-commanding.md)
- [Frames And Transforms](../calibration/frames-and-transforms.md)
- [Policy Transfer And Action Semantics](../sim-to-real/policy-transfer-and-action-semantics.md)
- [UR3e Control Stack](../robot-control/ur3e-control-stack.md)
