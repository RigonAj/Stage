# Real Perception Trace Test Runbook

> Sources: Trace pipeline and launch verification, 2026-07-09; local calibration files, 2026-07-09; left policy model selection, 2026-07-09; explicit tracker intrinsics parameter, 2026-07-09; first real command test analysis, 2026-07-09
> Raw: [operator procedure](../../docs/Robot_Control/procedure_test_perception_trace.md); [Analyse pipeline commande](../../docs/Robot_Control/analyse_pipeline_commande_trace_2026-07-09.md); [Procédure session réelle commandée](../../docs/Robot_Control/procedure_lancement_reel_trace_commande.md); [Publisher node](../../src/Ball_Tracking_Cpp/src/publisher_member_function.cpp); [Camera front-end](../../src/Ball_Tracking_Cpp/src/Camera.cpp); [live_catch launch](../../src/ur3e_live_catch/launch/live_catch.launch.py); [live_catch config](../../src/ur3e_live_catch/config/live_catch.yaml); [TF publisher](../../scripts/publish_camera_tf.py); [model README](../../data/models/README.md)

## Overview

This runbook is the operational path for first real-ball perception tests after
intrinsic and hand-eye calibration. It keeps robot command emission off, starts
the Trace tracker as the real ball source, and optionally feeds `live_catch_node`
in dry-run so policy inference and `catch_telemetry` can be inspected without
moving the UR3e.

The full French operator procedure is
[procedure_test_perception_trace.md](../../docs/Robot_Control/procedure_test_perception_trace.md).

## Current Local Calibration State

- Intrinsics: `recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml`
  from 2026-07-09, 17 `grid_7x5` observations, RMS about 0.149 px.
- Tracker intrinsics are selected by the `ball_tracking_cpp`
  `camera_calibration_file` parameter. The live-catch launch default is the
  recent real-test file:
  `recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml`.
- Hand-eye: `calibration/handeye_result.yaml` from 2026-07-09 publishes
  `base -> camera_optical`; the result uses 6 poses, good residuals for first
  testing, but fewer than the 15-20 poses targeted by the extrinsic runbook.
- Policy: inference for this procedure uses
  `data/models/latest-left/policy_deterministic.onnx` with
  `hold_side=left`. The hoop TF and physical mount must therefore also be left:
  `wrist_3_link -> hoop_center` at about `(+0.5, 0, 0)` with quaternion
  `(0, 1, 0, 0)`.

## Minimal Commands

Common setup:

```bash
cd ~/Dv-Rosws/Dv-Rosws
source env.sh
source install/setup.bash
```

Verify the latest intrinsics exist and rebuild:

```bash
LATEST_INTRINSICS=recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml
test -f "$LATEST_INTRINSICS"
test -f calibration/handeye_result.yaml
colcon build --symlink-install --packages-select ur3e_catch_msgs ball_tracking_cpp ur3e_live_catch
source install/setup.bash
```

Publish camera TF:

```bash
python3 scripts/publish_camera_tf.py calibration/handeye_result.yaml
ros2 run tf2_ros tf2_echo base_link camera_optical
```

Run Trace-only perception:

```bash
ros2 run ball_tracking_cpp talker --ros-args \
  --params-file src/ur3e_live_catch/config/live_catch.yaml \
  -p pose_source:=trace \
  -p ball_state_topic:=ball_state \
  -p camera_frame_id:=camera_optical \
  -p camera_calibration_file:=recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml \
  -p ball_radius_mm:=20.0 \
  -p publish_legacy_pose:=false
```

Run integrated live-catch dry-run instead, with the tracker started by launch:

```bash
ros2 launch ur3e_live_catch live_catch.launch.py \
  use_tracker:=true \
  use_ball_regression:=false \
  enable_command:=false \
  model_path:=data/models/latest-left/policy_deterministic.onnx \
  camera_calibration_file:=recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml \
  ball_radius_mm:=20.0
```

Optional ballistic-regression mode:

```bash
ros2 launch ur3e_live_catch live_catch.launch.py \
  use_tracker:=true \
  use_ball_regression:=true \
  enable_command:=false \
  model_path:=data/models/latest-left/policy_deterministic.onnx \
  camera_calibration_file:=recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml \
  ball_radius_mm:=20.0
```

In regression mode the raw Trace detections move to `ball_state_raw`, and the
fitted 60 Hz `base_link` estimate lands on `ball_state`.

## Validation

- Raw tracker output: `/ball_state`, `header.frame_id=camera_optical`,
  `valid=true`, position in metres, positive camera depth for a ball in front of
  the camera.
- Live-catch dry-run: `/catch_telemetry` publishes during valid throws, and the
  Web UI Test tab shows the ball and dry-run target ghost. The
  `live_catch_node` logs must confirm that a policy backend (`onnxruntime` or
  `torch`) loaded the model; otherwise telemetry can still exist with zero
  policy action.
- TF checks: `base_link <- camera_optical` must resolve before inference relies
  on camera-frame samples. `base_link <- hoop_center` is required for realistic
  inference and mandatory before command mode. For the left model, this must be
  the left hoop transform.

## Command-Mode Session (Robot Moving)

The dry-run commands above start NEITHER the UR driver NOR the web UI. Do not
"add" them by keeping the virtual-ball stack running next to a manual
`live_catch.launch.py`: that duplicates `live_catch_node` and `ball_state`
producers, which stalled the robot and made the UI command state flicker on
the 2026-07-09 first real command test
([Single Producer Contract](../live-catch/single-producer-contract.md)). For a
command session use the single stack with the tracker swapped in:

```bash
ur3e_catch_stop
ur3e_catch_stack --real --tracker --hold-side left --ball-radius 45.0 \
  --model-path data/models/latest-left/policy_deterministic.onnx
```

`--hold-side left` drives the hoop TF side since 2026-07-09 (the script no
longer hardcodes the right-side `hoop_xyz`); it must match the physical mount
and the model's `hold_side` metadata. Then verify one publisher each on
`/ball_state` and `/catch_telemetry` (`ros2 topic info ... --verbose`), no
`PRODUCER CONFLICT` in the live node log, and arm command mode from the web UI
Test tab only. The complete ordered operator checklist (terminals, pendant,
camera TF, ROI, blank-run checks, staged v_safe_scale ramp, incident table) is
[procedure_lancement_reel_trace_commande.md](../../docs/Robot_Control/procedure_lancement_reel_trace_commande.md).

## Operator Risks

- Command mode stays off for this procedure (`enable_command=false`).
- Only one ball producer may publish a given topic. Stop virtual-ball launches
  before starting the real tracker on `ball_state`; since 2026-07-09 the live
  node blocks command emission while `ball_state` has multiple publishers.
- Trace assumes the ball is the dominant mover in the work ROI. Crop out the
  robot, hand, support and reflective clutter.
- The 2D display uses the sampled event stream; reduce `Max Events` if the UI
  lags. The Trace estimator still uses the full filtered/undistorted stream.
- Depth scales directly with the configured ball radius and the loaded
  intrinsics. Set `ball_radius_mm` to the measured physical radius in
  millimetres, then adjust live with the Option-panel slider if needed. Verify
  the tracker terminal prints
  `Calibration loaded from recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml`
  before throwing the ball.
- The left policy metadata, the physical racket side, the Web UI hold-side
  toggle and the published `hoop_center` TF must all agree.
- With `use_ball_regression:=true`, inspect `ball_state_raw` for tracker bugs and
  `ball_state` for the filtered policy input. `ball_state` may stay
  `valid=false` until the regression start gate has enough samples.

## See Also

- [Trace Ball Tracking](trace-ball-tracking.md)
- [Frames And Transforms](../calibration/frames-and-transforms.md)
- [Camera And Hand-Eye Calibration](../calibration/camera-and-handeye-calibration.md)
- [Live Catch Loop](../live-catch/live-catch-loop.md)
- [Message Contracts And Topics](../live-catch/message-contracts-and-topics.md)
- [Single Producer Contract](../live-catch/single-producer-contract.md)
