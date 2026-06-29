# Frames And Transforms

> Sources: camera-base calibration reference, 2026-06-29; live-catch architecture, 2026-06-29; robot control architecture, 2026-06-29; remaining work checklist, 2026-06-29
> Raw: [Camera-base calibration](../../docs/Robot_Control/ur3e_camera_base_calibration.md); [Live-catch architecture](../../docs/Robot_Control/ur3e_live_catch_architecture.md); [Robot control architecture](../../docs/Robot_Control/ur3e_robot_control_architecture.md); [Reste a faire](../../docs/reste_a_faire.md)

## Overview

Frame and unit consistency is a project-wide contract. This page is separate
from the calibration procedure because it is needed by perception, live catch,
web UI and debugging.

## Important Frames

- `camera_optical`: declared camera frame for native `BallState` from
  `ball_tracking_cpp`.
- `base`: robot frame expected by the policy/live-catch observation.
- `base_link`: not interchangeable with `base`; docs warn about this trap.
- `tool0`: robot tool frame used during hand-eye capture.
- `mire`: phone calibration target frame.
- `hoop_center`: target/catcher frame used by observation and UI visualization.

## Transform Contracts

- `T_base_camera` must come from validated eye-to-hand calibration.
- `base -> camera_optical` must be present in TF before using camera-frame ball
  positions.
- `wrist_3_link -> hoop_center` must be present or the live node falls back to
  configured placeholders.
- `BallState.header.frame_id` decides the transform path. Do not assume camera
  when the field is empty or unknown.

## Units

- ROS-side robot and ball positions are meters.
- Some legacy perception/debug paths use millimeters and must convert
  explicitly.
- Camera calibration files follow OpenCV conventions.

## Diagnostic Checks

```bash
ros2 run tf2_ros tf2_echo base camera_optical
ros2 run tf2_ros tf2_echo base hoop_center
ros2 topic echo /ball_state
ros2 topic echo /catch_telemetry
```

## See Also

- [Camera And Hand-Eye Calibration](camera-and-handeye-calibration.md)
- [Message Contracts And Topics](../live-catch/message-contracts-and-topics.md)
- [Live Catch Loop](../live-catch/live-catch-loop.md)
