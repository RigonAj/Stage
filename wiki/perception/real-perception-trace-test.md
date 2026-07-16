# Real Perception Trace Test Runbook

> Sources: Trace pipeline and launch verification, 2026-07-09; local calibration files, 2026-07-09; left policy model selection, 2026-07-09; explicit tracker intrinsics parameter, 2026-07-09; first real command test analysis, 2026-07-09; independent lead/timestamp review, 2026-07-10; real-ball ROS graph and log diagnosis, 2026-07-16; root-cause code diagnosis (reader-mode default) + offline replay validation, 2026-07-16
> Raw: [operator procedure](../../docs/Robot_Control/procedure_test_perception_trace.md); [Analyse pipeline commande](../../docs/Robot_Control/analyse_pipeline_commande_trace_2026-07-09.md); [Procédure session réelle commandée](../../docs/Robot_Control/procedure_lancement_reel_trace_commande.md); [Perception/control review](../../docs/Robot_Control/revue_perception_robuste_controle_fluide_2026-07-10.md); [Publisher node](../../src/Ball_Tracking_Cpp/src/publisher_member_function.cpp); [Camera front-end](../../src/Ball_Tracking_Cpp/src/Camera.cpp); [Ball regression node](../../src/ur3e_live_catch/ur3e_live_catch/ball_regression_node.py); [live_catch launch](../../src/ur3e_live_catch/launch/live_catch.launch.py); [live_catch config](../../src/ur3e_live_catch/config/live_catch.yaml); [TF publisher](../../scripts/publish_camera_tf.py); [model README](../../data/models/README.md)

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

- Raw tracker output: `/ball_state` without regression, or `/ball_state_raw`
  with regression; `header.frame_id=camera_optical`,
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

## 2026-07-16 Real-Ball Session Diagnosis

The live ROS graph observed during the real-UR3e test was correctly wired and
had exactly one producer at every boundary:

```text
DVXplorer events
  -> ball_tracking_cpp
  -> /ball_state_raw (camera_optical, fresh valid Trace measurements only)
  -> ball_regression_node
  -> /ball_state at 60 Hz (base_link, position + fitted velocity)
  -> live_catch_node
```

Active parameters matched the intended real-camera configuration:
`pose_source=trace`, `ball_state_topic=ball_state_raw`, the dated constrained
intrinsics file, `ball_radius_mm=45.0`, tracker lead/hold at zero, regression
rate 60 Hz and regression lead zero. The fitted output nevertheless stayed at
`position=(0,0,0)`, `valid=false`, and `catch_telemetry.ball_valid=false`.

The decisive evidence is upstream of the regression. Its log contained only
the startup line: there was no `regression state: idle -> collecting`, no
skewed-stamp drop and no TF rejection. The tracker log also contained only its
publisher/radius startup messages. Therefore the regression was alive but had
not received even a first valid `/ball_state_raw` measurement. Its 60 Hz output
was the expected invalid heartbeat, not an estimated ball trajectory. The live
node consequently logged `WATCHDOG stop -> holding: no_valid_ball`; holding the
robot was the correct safety response.

Assessment: the current blocker is the C++ Trace validity path, not a missing
node, a duplicate producer, the robot controller or the Python regression. The
next disarmed test must inspect the perception GUI while the ball crosses the
ROI: `events: ... accumulated`, `trace ribbon: not enough coherent events` and
the `Trace 3D` status. First checks are a live DVXplorer source, successful
intrinsics load, a flight-covering ROI, `Polarity: All` instead of the default
`Negative`, and enough support (about 500 accumulated events and a trail longer
than about 35 px). Do not arm until both `/ball_state_raw` and `/ball_state`
have been observed with `valid=true` during repeatable dry-run throws.

### Root Cause Found and Fixed (2026-07-16, same day)

Code diagnosis of `ball_tracking_cpp` explained the zero-valid-sample session:
the GUI constructor hardcoded `reader_mode = true`, so **the tracker always
started in File mode and processed no live camera events** until the operator
clicked "Reader → Camera". Second aggravating default: the trace polarity
filter started at `Negative` (`trace_polarity_mode = 2`), discarding half the
ball's events. Neither state was a ROS parameter, so the launch could not
force them and the failure was silent (log with startup lines only — exactly
the observed signature, since the "Using camera calibration" line only prints
once a first event window is processed).

Fixes shipped the same day (all in `live_catch.yaml`, overridable per launch):
`use_reader` (default `false` = live camera), `trace_polarity_mode` (default
`all`), throttled warnings when File mode idles or no camera is connected, a
2 s `trace status` heartbeat exposing per-stage peaks in the terminal, manual
H5 event recording (`record` default `false`, GUI REC toggle;
`record_file` default `recordings/realtest.h5`; an existing non-empty target
is archived with a timestamp suffix when the writer opens, never truncated)
and scripted replay (`reader_file`). See
[Trace Ball Tracking](trace-ball-tracking.md) for the parameter table.

Offline validation on the 2026-07-09 real-throw recording
(`recordings/realtest_2026-07-09_backup.h5`, 212 354 events, 9.4 s): the
tracker produced 12–13 `valid=true` raw samples with a coherent approach
trajectory, and the full tracker → `ball_regression_node` → hand-eye-TF chain
produced a complete `idle → collecting → tracking → ended` flight and 27
`valid=true` fitted samples on `/ball_state` in `base_link`. Replay command
(robot fully disarmed, no driver needed):

```bash
python3 scripts/publish_camera_tf.py calibration/handeye_result.yaml &
ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id base &  # test-only identity
ros2 run ur3e_live_catch ball_regression_node --ros-args \
  --params-file src/ur3e_live_catch/config/live_catch.yaml &
ros2 run ball_tracking_cpp talker --ros-args \
  --params-file src/ur3e_live_catch/config/live_catch.yaml \
  -p ball_state_topic:=ball_state_raw \
  -p use_reader:=true -p reader_file:=realtest_2026-07-09_backup.h5 -p record:=false
ros2 topic echo /ball_state ur3e_catch_msgs/msg/BallState   # expect valid=true bursts
```

For live-session ROS-level diagnosis, record a bag alongside the default H5
event recording:

```bash
ros2 bag record -o rosbags/real_$(date +%Y%m%d_%H%M%S) \
  /ball_state_raw /ball_state /catch_telemetry /joint_states /tf /tf_static
```

Remaining before arming: repeat the same validity with **live physical
throws** (camera mode is now the default), with the measured 45 mm radius and
a flight-covering ROI.

Commands used to establish the graph and the failure boundary:

```bash
ros2 node list | grep -E 'live_catch|ball_regression|ball_tracking'
ros2 topic info /ball_state_raw --verbose
ros2 topic info /ball_state --verbose
ros2 topic info /catch_telemetry --verbose
ros2 topic echo /catch_telemetry --once
ros2 param dump /ball_tracking_cpp
ros2 param dump /ball_regression_node
```

For the next test, first use **Stop / back to safe** and verify the heartbeat,
then observe each estimator boundary in separate terminals:

```bash
ros2 topic echo /catch_telemetry --once | grep command_enabled
# Expected before a physical throw: command_enabled: false

ros2 topic echo /ball_state_raw
ros2 topic echo /ball_state
```

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

The unmeasured 0.2 s regression lead was removed from the bring-up default on
2026-07-10. Keep command off and verify zero before blank throws or arming:

```bash
ros2 param get /ball_regression_node lead_time_s  # expected: 0.0
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
- Do not infer real latency from the current `perception_age_s`: the tracker
  re-anchors the first event after a gap to ROS `now`, and the regression
  evaluates at `now+lead` but stamps at `now`. Keep lead zero until separate
  measurement/state/publish timestamps are instrumented.

## See Also

- [Trace Ball Tracking](trace-ball-tracking.md)
- [Frames And Transforms](../calibration/frames-and-transforms.md)
- [Camera And Hand-Eye Calibration](../calibration/camera-and-handeye-calibration.md)
- [Live Catch Loop](../live-catch/live-catch-loop.md)
- [Message Contracts And Topics](../live-catch/message-contracts-and-topics.md)
- [Single Producer Contract](../live-catch/single-producer-contract.md)
- [Perception Robustness And Flight Lifecycle](perception-robustness-flight-lifecycle.md)
