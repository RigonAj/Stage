# Calibration

## Scope

Calibration covers:

- DVXplorer camera intrinsics.
- Eye-to-hand camera-to-UR3e transform.
- ROS frame publication for perception-to-robot conversion.

## Read First

- [[Current_Status]]
- `docs/Context/calibration_python_architecture.md`
- `docs/Robot_Control/ur3e_camera_base_calibration.md`
- `docs/Robot_Control/ur3e_motion_issue_resolution.md`

## Main Scripts

- `scripts/event_mire_calibration.py`: event-camera target detection.
- `scripts/calibrate_intrinsics_from_mire.py`: OpenCV intrinsics calibration.
- `scripts/run_handeye_session.sh`: hand-eye capture workflow.
- `scripts/solve_handeye.py`: solve camera-base transform.
- `scripts/publish_camera_tf.py`: publish camera TF.
- `scripts/serve_phone_mire.py`: phone target server.

## Contracts

- Camera intrinsics use OpenCV conventions.
- Robot-side positions are in meters.
- `BallState.header.frame_id` must match a TF frame.
- Be explicit about `base`, `base_link`, `tool0`, `mire` and `camera_optical`.

## Risks

- Unit mixups between camera millimeters and ROS meters.
- Inverted hand-eye transform.
- Using `base` where the current FirstTraining policy expects `base_link`.
- Captures with insufficient pose diversity.

## Current Blockers

- Physical eye-to-hand session still needs acceptance.
- Static TFs for camera and hoop must be published and checked.
- `publish_frame=base_link` vs `publish_frame=camera_optical` parity test is the
  main way to isolate extrinsic errors before using real perception.

## Related Notes

- [[Current_Status]]
- [[UR3e_Live_Catch]]
- [[Robot_Control]]
