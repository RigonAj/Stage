# Camera And Hand-Eye Calibration

> Sources: calibration Python architecture, 2026-06-29; camera-base calibration reference, 2026-06-29; current status docs, 2026-06-29; physical hand-eye solve, 2026-07-10
> Raw: [Calibration scripts architecture](../../docs/Context/calibration_python_architecture.md); [Camera-base calibration](../../docs/Robot_Control/ur3e_camera_base_calibration.md); [Reste a faire](../../docs/reste_a_faire.md); [Hand-eye result](../../calibration/handeye_result.yaml); [Cleaned physical samples](../../recordings/mire_calibration/handeye/handeye_samples_20260710_140949.json)

## Overview

Calibration has two separate jobs: DVXplorer intrinsics and eye-to-hand
camera-to-UR3e extrinsics. The live-catch path needs both, but the current
critical blocker is physical validation of `T_base_camera` and publication of
stable TFs.

## Intrinsics

The intrinsics workflow uses an event mire, OpenCV calibration and exported
camera parameters. Main scripts:

- `scripts/event_mire_calibration.py`
- `scripts/calibrate_intrinsics_from_mire.py`

The docs emphasize blob detection, robust association, held-out validation and
physical spacing checks.

## Eye-To-Hand

The eye-to-hand workflow estimates `T_base_camera` with a phone mire mounted on
the UR3e tool. Main tools:

- `scripts/serve_phone_mire.py`
- `scripts/run_handeye_session.sh`
- `scripts/solve_handeye.py`
- `scripts/publish_camera_tf.py`

The key convention risk is OpenCV hand-eye argument naming. The docs specify a
verified derivation and require synthetic tests before accepting real data.

Session samples land in `recordings/mire_calibration/handeye/`; the solve
result must be written to `calibration/handeye_result.yaml`, the path expected
by the web UI (`GET /api/calibration/camera`) and `publish_camera_tf.py`
(aligned 2026-07-06). The operator procedure is the
[Extrinsic Calibration Runbook](extrinsic-calibration-runbook.md).

## Dated Physical Example — 2026-07-10

The physical solve completed on **2026-07-10 at 14:30:52 Europe/Paris** is the
current reference example. It uses 18 cleaned poses from
`handeye_samples_20260710_140949.json`; known invalid screen-orientation poses
were removed before the final solve. Values are in meters and quaternions use
the ROS `xyzw` order.

```yaml
T_base_camera_optical:
  xyz_m: [0.4294113403, -0.5945960870, 0.4344958630]
  quat_xyzw: [0.4319456062, 0.5814835261, -0.5497853237, -0.4159759609]
  rpy_deg: [-92.831685, -0.504876, 106.307055]

T_tool0_screen_center:
  xyz_m: [-0.0053138655, -0.0005829202, 0.1587624887]
  quat_xyzw: [-0.0001764947, 0.7118134160, -0.0007382283, 0.7023681974]
  rpy_deg: [-174.445677, 89.231082, -174.491194]
```

Reference static publisher command:

```bash
ros2 run tf2_ros static_transform_publisher \
  0.429411 -0.594596 0.434496 \
  0.431946 0.581484 -0.549785 -0.415976 \
  base camera_optical
```

Validation for this example: solver agreement `0.275 mm / 0.0054 deg`, pose
translation residual `0.94 mm` mean and `2.24 mm` maximum, rotation residual
`0.247 deg` mean and `0.436 deg` maximum, leave-one-out worst case
`0.36 mm / 0.050 deg`, and end-to-end pixel RMS `1.22 px` (`3.46 px` maximum,
342 points). Rotation-axis diversity is `89.99 deg` and the estimated mire
normal points into the screen.

This is an installation-specific example, not a universal default. Moving the
camera or changing the phone mount invalidates it. Prefer the quaternion over
the `T_tool0_screen_center` Euler angles, whose pitch near 90 degrees makes the
RPY representation numerically close to gimbal lock.

## Current Blockers

- The 2026-07-10 physical solve passes the numerical acceptance gates; tape/CAD
  consistency and live TF parity still need physical validation.
- `base -> camera_optical` TF must be published and verified.
- `wrist_3_link -> hoop_center` TF must be published and verified.
- The parity test `publish_frame=base_link` vs `publish_frame=camera_optical` should
  pass before relying on real camera perception.

## See Also

- [Extrinsic Calibration Runbook](extrinsic-calibration-runbook.md)
- [Frames And Transforms](frames-and-transforms.md)
- [Current Status And Blockers](../live-catch/current-status-and-blockers.md)
- [Live Catch Loop](../live-catch/live-catch-loop.md)
- [UR3e Control Stack](../robot-control/ur3e-control-stack.md)
