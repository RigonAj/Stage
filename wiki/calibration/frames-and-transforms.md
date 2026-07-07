# Frames And Transforms

> Sources: camera-base calibration reference, 2026-06-29; live-catch architecture, 2026-06-29; robot control architecture, 2026-06-29; remaining work checklist, 2026-07-01; live-catch README, 2026-07-01; model README, 2026-07-01; web UI docs, 2026-07-01; hold-side (left/right racket) change, 2026-07-06
> Raw: [Camera-base calibration](../../docs/Robot_Control/ur3e_camera_base_calibration.md); [Live-catch architecture](../../docs/Robot_Control/ur3e_live_catch_architecture.md); [Robot control architecture](../../docs/Robot_Control/ur3e_robot_control_architecture.md); [Reste a faire](../../docs/reste_a_faire.md); [Live-catch README](../../src/ur3e_live_catch/README.md); [Model README](../../data/models/README.md); [Web UI docs](../../docs/Robot_Control/ur3e_web_ui.md)

## Overview

Frame and unit consistency is a project-wide contract. This page is separate
from the calibration procedure because it is needed by perception, live catch,
web UI and debugging.

## Important Frames

- `camera_optical`: declared camera frame for native `BallState` from
  `ball_tracking_cpp`.
- `base_link`: current Isaac FirstTraining policy/live-catch observation frame.
- `base`: teach-pendant/MoveIt frame in some robot-control paths; not
  interchangeable with `base_link` because UR rotates it 180 degrees about Z.
- `tool0`: robot tool frame used during hand-eye capture.
- `mire`: phone calibration target frame.
- `hoop_center`: target/catcher frame used by observation and UI visualization.

## Transform Contracts

- `T_base_camera` still comes from validated eye-to-hand calibration, but the
  live catch policy frame is `base_link`; TF must provide the full path from
  `camera_optical` to `base_link` before using camera-frame ball positions.
- `wrist_3_link -> hoop_center` must match the Isaac hoop geometry for the
  racket hold side in use. The `virtual_ball_robot.launch.py` argument
  `hold_side` (default `right`) selects it; explicit `hoop_xyz`/`hoop_quat`
  still override:
  - `right` (historical): translation `(-0.5, 0, 0)` m, quaternion
    `(1, 0, 0, 0)` xyzw (180 deg about X);
  - `left`: translation `(0.5, 0, 0)` m, quaternion `(0, 1, 0, 0)` xyzw
    (180 deg about Y, i.e. the right mount rotated 180 deg about wrist_3 Z).
  Both map hoop +Z to the Isaac disk normal `(0, 0, -1)` in `wrist_3_link`.
  The side must agree with the physical mount and with the loaded model's
  `hold_side` metadata (see the model README).
- The physical hoop visual is `0.15 m` radius; `disk_radius_m=0.05` is the
  policy validation radius around `hoop_center`, not the real hoop radius.
- `base_link -> hoop_center` must be present for command mode. The live node
  only uses disk fallback in dry-run/debug.
- The Web UI 3D robot root is also `base_link`; applying the UR `base` 180 deg Z
  rotation to the whole robot would make the displayed robot disagree with Isaac.
- `BallState.header.frame_id` decides the transform path. Do not assume camera
  when the field is empty or unknown.

## Units

- ROS-side robot and ball positions are meters.
- Some legacy perception/debug paths use millimeters and must convert
  explicitly.
- Camera calibration files follow OpenCV conventions.

## Diagnostic Checks

```bash
ros2 run tf2_ros tf2_echo base_link camera_optical
ros2 run tf2_ros tf2_echo base_link hoop_center
ros2 topic echo /ball_state
ros2 topic echo /catch_telemetry
```

## See Also

- [Camera And Hand-Eye Calibration](camera-and-handeye-calibration.md)
- [Message Contracts And Topics](../live-catch/message-contracts-and-topics.md)
- [Live Catch Loop](../live-catch/live-catch-loop.md)
