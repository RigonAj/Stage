# Camera And Hand-Eye Calibration

> Sources: calibration Python architecture, 2026-06-29; camera-base calibration reference, 2026-06-29; current status docs, 2026-06-29; strict-landscape physical hand-eye solve, 2026-07-23
> Raw: [Calibration scripts architecture](../../docs/Context/calibration_python_architecture.md); [Camera-base calibration](../../docs/Robot_Control/ur3e_camera_base_calibration.md); [Reste a faire](../../docs/reste_a_faire.md); [Hand-eye result](../../calibration/handeye_result.yaml); [Strict-landscape physical samples](../../recordings/mire_calibration/handeye/handeye_samples_20260723_093733.json)

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

## Current Physical Solve — 2026-07-23

The physical solve completed on **2026-07-23 at 09:46:19 Europe/Paris** is the
current reference. It uses only the 13 poses from
`handeye_samples_20260723_093733.json`, captured after the strict landscape
guard was enabled. The recorded phone geometry is 2712×1220 px,
154.50×69.55 mm, with `fullscreen_ok=true` and `landscape_ok=true`. Values are
in meters and quaternions use the ROS `xyzw` order.

```yaml
T_base_camera_optical:
  xyz_m: [0.4098707814, -0.5857585782, 0.4396545837]
  quat_xyzw: [0.4445146305, 0.5764225530, -0.5385073471, -0.4244450740]
  rpy_deg: [-93.423498, -0.605664, 104.153463]

T_tool0_screen_center:
  xyz_m: [-0.0060016995, -0.0024537592, 0.1597571153]
  quat_xyzw: [0.0044498713, 0.7145657427, -0.0087563531, 0.6994993383]
  rpy_deg: [-163.513213, 88.730275, -164.583375]
```

Reference static publisher command:

```bash
ros2 run tf2_ros static_transform_publisher \
  0.409871 -0.585759 0.439655 \
  0.444515 0.576423 -0.538507 -0.424445 \
  base camera_optical
```

Validation: solver agreement `0.619 mm / 0.0072 deg`, pose translation
residual `1.11 mm` mean and `1.87 mm` maximum, rotation residual `0.270 deg`
mean and `0.576 deg` maximum, leave-one-out worst case
`1.79 mm / 0.160 deg`, and end-to-end pixel RMS `0.98 px` (`2.59 px` maximum,
247 points). Rotation-axis diversity is `89.98 deg` and the estimated mire
normal points into the screen. Relative to the previous 2026-07-10 reference,
the co-solved phone mount differs by `2.23 mm / 1.15 deg`; this remains within
the runbook consistency gate.

This is an installation-specific example, not a universal default. Moving the
camera or changing the phone mount invalidates it. Prefer the quaternion over
the `T_tool0_screen_center` Euler angles, whose pitch near 90 degrees makes the
RPY representation numerically close to gimbal lock.

## Current Blockers

- The strict-landscape 2026-07-23 solve passes the numerical acceptance gates;
  camera tape-measure consistency and live TF parity still need physical
  validation.
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
