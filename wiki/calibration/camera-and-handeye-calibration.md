# Camera And Hand-Eye Calibration

> Sources: calibration Python architecture, 2026-06-29; camera-base calibration reference, 2026-06-29; current status docs, 2026-06-29
> Raw: [Calibration scripts architecture](../../docs/Context/calibration_python_architecture.md); [Camera-base calibration](../../docs/Robot_Control/ur3e_camera_base_calibration.md); [Reste a faire](../../docs/reste_a_faire.md)

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

## Current Blockers

- Physical eye-to-hand session still needs acceptance.
- `base -> camera_optical` TF must be published and verified.
- `wrist_3_link -> hoop_center` TF must be published and verified.
- The parity test `publish_frame=base_link` vs `publish_frame=camera_optical` should
  pass before relying on real camera perception.

## See Also

- [Frames And Transforms](frames-and-transforms.md)
- [Current Status And Blockers](../live-catch/current-status-and-blockers.md)
- [Live Catch Loop](../live-catch/live-catch-loop.md)
- [UR3e Control Stack](../robot-control/ur3e-control-stack.md)
