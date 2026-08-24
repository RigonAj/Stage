# Camera And Hand-Eye Calibration

> Sources: calibration Python architecture, 2026-06-29; camera-base calibration reference, 2026-06-29; current status docs, 2026-06-29; strict-landscape physical hand-eye solve, 2026-07-23; cleaned 18-pose physical hand-eye solve, 2026-08-24
> Raw: [Calibration scripts architecture](../../docs/Context/calibration_python_architecture.md); [Camera-base calibration](../../docs/Robot_Control/ur3e_camera_base_calibration.md); [Reste a faire](../../docs/reste_a_faire.md); [Hand-eye result](../../calibration/handeye_result.yaml); [Current cleaned physical samples](../../recordings/mire_calibration/handeye/handeye_samples_20260824_150722_clean.json); [Previous strict-landscape physical samples](../../recordings/mire_calibration/handeye/handeye_samples_20260723_093733.json)

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

## Current Physical Solve Candidate — 2026-08-24

The active result file was regenerated on **2026-08-24 at 15:35:52
Europe/Paris** from `handeye_samples_20260824_150722_clean.json`. The cleaned
file retains 18 of the 19 strict-landscape poses. Original sample index 1
(`2026-08-24T15:08:48.153`) is explicitly excluded because its asymmetric
anchor was associated to the wrong end of the phone pattern, producing a
180-degree mire-pose flip. The immutable session JSON remains beside the
cleaned copy. Values are in meters and quaternions use ROS `xyzw` order.

```yaml
T_base_camera_optical:
  xyz_m: [0.4139677472, -0.6015156224, 0.4384910442]
  quat_xyzw: [0.4272089664, 0.5872940742, -0.5549682109, -0.4056950264]
  rpy_deg: [-93.143896, -0.134633, 107.806838]

T_tool0_screen_center:
  xyz_m: [-0.0063567498, -0.0012466041, 0.1589236132]
  quat_xyzw: [-0.0034098044, 0.7141471653, 0.0003975192, 0.6999871724]
  rpy_deg: [-168.144636, 88.826942, -167.836117]
```

Reference static publisher command:

```bash
ros2 run tf2_ros static_transform_publisher \
  0.413968 -0.601516 0.438491 \
  0.427209 0.587294 -0.554968 -0.405695 \
  base camera_optical
```

Validation: solver agreement `1.196 mm / 0.0087 deg`, pose translation
residual `1.60 mm` mean and `2.56 mm` maximum, rotation residual `0.454 deg`
mean and `1.043 deg` maximum, leave-one-out worst case
`1.66 mm / 0.149 deg`, and end-to-end pixel RMS `1.65 px` (`3.53 px` maximum,
342 points). Rotation-axis diversity is `89.98 deg`, the estimated mire normal
points into the screen, and all numerical runbook gates pass. Relative to the
2026-07-23 reference, the co-solved phone mount changes by approximately
`1.51 mm / 1.38 deg`, inside the CAD-consistency gate. `T_base_camera` changes
by `16.32 mm / 3.70 deg`; since the operator reports that the camera did not
move, this difference must be checked with the physical overlay and frame
parity test before treating the candidate as physically accepted.

This is an installation-specific example, not a universal default. Moving the
camera or changing the phone mount invalidates it. Prefer the quaternion over
the `T_tool0_screen_center` Euler angles, whose pitch near 90 degrees makes the
RPY representation numerically close to gimbal lock.

## Current Blockers

- The cleaned 18-pose 2026-08-24 solve passes the numerical acceptance gates;
  camera tape-measure consistency, overlay and live TF parity still need
  physical validation, especially because it differs from the 2026-07-23
  camera transform despite no reported camera movement.
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
