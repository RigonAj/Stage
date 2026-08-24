# Current Status And Blockers

> Sources: live-catch implementation status, 2026-06-30; remaining work checklist, 2026-06-29; inconsistency review, 2026-06-30; 2026-07-02 pendant incident analysis; user hardware report, 2026-07-02; v_safe_scale UI implementation, 2026-07-03; ball regression publisher, 2026-07-03; 2026-07-09 first real Trace command test analysis; independent perception/control review, 2026-07-10; real-ball ROS graph and log diagnosis, 2026-07-16; tracker reader-mode root cause + offline replay validation, 2026-07-16; first real ball caught, 2026-08-05
> Raw: [Implementation status](../../docs/Robot_Control/ur3e_live_catch_implementation_status.md); [Reste a faire](../../docs/reste_a_faire.md); [Incoherences](../../docs/incoherences_code_logique.md); [Analyse pipeline commande](../../docs/Robot_Control/analyse_pipeline_commande_trace_2026-07-09.md); [Perception/control review](../../docs/Robot_Control/revue_perception_robuste_controle_fluide_2026-07-10.md); [Tracker publisher](../../src/Ball_Tracking_Cpp/src/publisher_member_function.cpp); [Ball regression node](../../src/ur3e_live_catch/ur3e_live_catch/ball_regression_node.py); [Live-catch config](../../src/ur3e_live_catch/config/live_catch.yaml); [Web UI app](../../src/ur3e_web_ui/ur3e_web_ui/app.py); [Catch panel](../../src/ur3e_web_ui/ur3e_web_ui/static/js/catch_panel.js); [Live catch node](../../src/ur3e_live_catch/ur3e_live_catch/live_catch_node.py)

## Overview

The live-catch code path is implemented and has now caught a real ball
end-to-end (2026-08-05), at roughly a 1-in-5 success rate. The virtual-ball path
was validated earlier through real UR3e command streaming but is still slow
under the current bring-up limits. Making the real catch reliable still needs
the calibration, TF, timestamp and latency validation work listed below.

## Working

- `ur3e_catch_msgs` exists and is buildable.
- `ball_tracking_cpp` publishes native `BallState`.
- `ur3e_live_catch` computes observations, actions, safe targets and telemetry.
- Command streaming is wired behind `enable_command`.
- The web UI Test tab can launch virtual balls and toggle command mode.
- The current TorchScript policy was verified without an external scaler.
- `data/models/` contains current `latest` and `best` Isaac exports; the root
  canonical model is `latest`.
- `ActionMapper` resolves the action contract from `policy_metadata.json` for
  current incremental exports while preserving legacy absolute compatibility.
- `ObservationBuilder` mirrors Isaac pass-through logic using loaded metadata
  (`disk_radius_m=0.05` for the historical right export, 0.10 for the current
  `latest-left` export), and command mode fails closed without the hoop TF.
- `CatchTelemetry` publishes idle heartbeats with `ball_valid=false`, so the UI
  keeps command state live between trigger-mode throws.
- `test_ball_node` terminates virtual flights at `ground_z_m=0.05` by default,
  matching Isaac `ball_on_ground` and avoiding underground policy inputs.
- The Web UI Test tab can change `v_safe_scale` through the live node parameter
  service while command mode is off, with staged buttons from `0.5` through
  `4.0`; values above `1.0` are explicit overdrive tests.
- User hardware report, 2026-07-02: virtual ball -> policy -> 500 Hz streaming
  -> real UR3e follows and holds after the ball grounds. It works, but the robot
  response is still slow and needs tuning/optimization.
- 2026-07-03: `ball_tracking_cpp` can now publish the Trace-pipeline pose
  (`pose_source: "trace"` in the bring-up config): the outlier-filtered
  mid-window sample, stamped at its own event time — the primary algorithm
  finally feeds ROS instead of the legacy circle fit (`"circle"` remains the
  code default and fallback). Not yet validated on live camera data.
- 2026-07-03: the ballistic-regression ball publisher (`ball_regression_node`,
  `use_ball_regression:=true`) is implemented and sim-validated: single pop per
  throw, fit-derived velocity within 0.3 m/s of the analytic throw at pop and
  converging to exact by flight end under 2 cm noise + 20 % dropout, clean
  ground termination and 60 Hz output. `live_catch_node` now trusts
  `BallState.velocity` (`use_ball_state_velocity`) and resets its EMA velocity
  filter between throws (bugfix). Known tuning point: at a 30 Hz raw rate with
  dropout, the start gate can take ~150 ms, making the ball first appear
  slightly closer than the Isaac spawn envelope (y >= 1.2 m); the real event
  tracker's higher raw rate shortens this, and `min_samples`/`min_span_s` are
  tunable.

## 2026-07-09 First Real Trace Command Test (Diagnosed)

First camera+Trace+robot command attempt: the robot only twitched and the Web
UI command state flickered ON/OFF. Root cause was NOT perception quality: the
operator started `live_catch.launch.py use_tracker:=true enable_command:=true`
while the virtual-ball stack was still running, so two `live_catch_node`
instances published `/catch_telemetry` with opposite `command_enabled` (the UI
flicker) and the stack's idle `test_ball_node` interleaved `valid=false`
heartbeats with the regression output on `ball_state` — each one triggering a
controlled stop plus policy-state reset (the twitching). Secondary finding:
the tracker's publish cadence is capped by the raylib render loop
(`SetTargetFPS(60)`), so `ball_state_raw` is not a guaranteed 60 Hz; the
regression node's 60 Hz resampling covers the policy input. Fixes shipped the
same day: producer-conflict watchdog with fail-closed commanding, Web UI flap
detection, `--tracker` stack option and wider `--stop` cleanup — see
[Single Producer Contract](single-producer-contract.md). Decoupling tracker
publication from the GUI render remains open. Note: the test used
`ball_radius_mm:=45.0`; verify it is the measured radius (Ø 90 mm ball), not
the diameter, since Trace depth scales directly with it.

## 2026-07-10 Independent Review (Not Yet Hardware-Validated)

The post-incident changes build and their unit tests pass (127 live-catch tests,
1 skipped; 51 Web UI tests), but no real rosbag or 3D ground truth yet validates
the new pipeline. The review found four P0 items before performance tuning:

- keep the now-applied `lead_time_s=0.0` default for baseline tests; the fit
  already evaluates old measurements at `now`, while extra lead would present
  a future ball with current robot joints and ground-terminate early;
- fix/extend timestamp semantics: the tracker re-anchors first events to ROS
  `now`, and the regression evaluates `now+lead` but stamps `now`, so current
  `perception_age_s` cannot measure source latency;
- add real Trace quality/covariance and an explicit flight lifecycle; fresh
  Trace windows all carry confidence 1.0 and live control ignores regression
  confidence;
- record H5 + rosbag perception-only sessions and validate false positives,
  time-to-pop, depth, velocity and hoop-plane intersection before raising robot
  speed.

See
[Perception Robustness And Flight Lifecycle](../perception/perception-robustness-flight-lifecycle.md)
for the phase model, left-policy envelope and ordered validation gates.

## 2026-07-16 Real-Ball Test: Trace Produced No Valid Raw Sample

The second inspected real-camera/real-UR3e session no longer had the duplicate
stack problem from 2026-07-09. The active topology was correct and singular:
`ball_tracking_cpp -> /ball_state_raw -> ball_regression_node -> /ball_state ->
live_catch_node`. The regression node was present and publishing its fixed-rate
60 Hz heartbeat as designed.

The robot did not move because `ball_state.valid` and
`catch_telemetry.ball_valid` remained false. Logs showed no regression state
transition, raw-stamp rejection or camera-to-base TF rejection, which means the
C++ tracker never delivered a first valid Trace sample on `/ball_state_raw`.
`live_catch_node` then reported `WATCHDOG stop -> holding: no_valid_ball` and
kept the arm at its safe target. This is evidence that command/controller
bring-up and watchdog holding worked; it is not evidence of successful real
perception.

Current opinion: prioritize a robot-disarmed Trace-only test and make
`/ball_state_raw` validity repeatable before changing regression gates or robot
speed. Inspect event support, polarity, ROI, ribbon validity, 3D validity and
intrinsics loading in that order. The detailed evidence and exact commands are
recorded in
[Real Perception Trace Test Runbook](../perception/real-perception-trace-test.md).

**Resolved the same day (code diagnosis + offline validation).** The tracker
GUI hardcoded `reader_mode = true`: every launch started in File mode and
processed no live camera events until a manual GUI click, which is why the
whole session logged startup lines only. The trace polarity filter also
defaulted to `Negative`. Both are now ROS parameters (`use_reader` default
live camera, `trace_polarity_mode` default `all`) with throttled idle
warnings, a 2 s `trace status` heartbeat, manual H5 event recording (REC
toggle, timestamp-archived `recordings/realtest.h5` target) and scripted
replay (`reader_file`). Replaying the
2026-07-09 real-throw recording through the fixed tracker + regression + TF
chain produced 12–13 valid raw samples and 27 `valid=true` fitted samples on
`/ball_state` in `base_link` (flight `idle → collecting → tracking → ended`,
RMS 0.013 m). Blocker 1 below is therefore validated offline; it still needs
confirmation with live physical throws before arming.

## 2026-08-05 First Real Ball Caught

User hardware report, 2026-08-05: the full chain — real event camera, Trace
perception, policy, command mode armed — **intercepted a hand-thrown ball and
held it in the net**. This is the first end-to-end success and supersedes the
"real perception deployment is blocked" framing above for the happy path.

It is not yet reliable: roughly **1 catch in 5 throws**. Two causes reported,
neither quantified yet:

- **Chain latency**: the net reaches the interception point late enough to miss
  a fast ball. Blocker 6 below (timestamp instrumentation before using latency
  percentiles) is now the direct path to fixing this.
- **Apparent spatial offset** between the aimed position and the ball's real
  position. Origin not yet separated between residual perception bias, residual
  `T_base_camera` error, and sim-to-real dynamics mismatch. Blockers 2–5 below
  are the way to isolate the calibration/TF share.

Treat both as open; the user has stated they will address them later.

## Blockers Before Real Perception

1. Obtain repeatable `valid=true` samples from C++ Trace on `/ball_state_raw`
   during robot-disarmed physical throws. (Validated offline on the 2026-07-09
   recording after the 2026-07-16 reader-mode/polarity fix; live physical
   confirmation pending.)
2. Validate `T_base_camera` physically.
3. Publish and verify `base -> camera_optical`.
4. Publish and verify `wrist_3_link -> hoop_center`; without it, command mode
   holds instead of using a fallback disk pose.
5. Compare `publish_frame=base_link` against `publish_frame=camera_optical`.
6. Correct or instrument measurement/state/publish timestamps before using
   latency percentiles to tune prediction lead.
7. Capture real Trace H5 + rosbag data and validate the flight estimator at
   30/60 Hz with dropouts; synthetic 120 Hz tests are not deployment evidence.

## 2026-07-02 Pendant Incident (Diagnosed)

First real command-mode attempt failed: pendant reported `Velocity 3139
required in joint 5 to go from 0.004778 to 6.28321 within 0.002 seconds ...
Ignoring commands until a valid command is received`. Log forensics
(`~/.ros/log/python_8044_*.log`) showed the node only ever published hold
commands (= the measured pose); `6.28321 - 2π = 0.00002`, so `/joint_states`
reported wrist_3 on a +2π-wrapped branch while the UR controller internally
sat at ~0.0048 rad. Not a policy-speed problem. Fixes shipped: start-pose gate,
per-session streamer reset, 500 Hz interpolated streaming, `v_safe_scale`
bring-up slow-down (see [Safety And Commanding](safety-and-commanding.md)).
Operator action when the gate triggers: jog/unwind the wrist (or reboot the
arm) until `/joint_states` matches the pendant.

Also observed: the current policy outputs saturated raw actions (±7..±24), so
after clipping every joint runs at full metadata `v_safe` (UR3e hard limits).
The halving of `joint_velocity_safe_rad_s` / `joint_acceleration_safe_rad_s2`
and the ±π position bounds exist in the Isaac `FirstTraining` cfg only as
**uncommitted working-tree changes** in the local checkout (verified via
`git diff` on 2026-07-03; the last commit still has the full limits). They must
be committed to survive, and are effective only after retraining and re-export
(review Volet 3, action B4). See
[Isaac Training Environment](../sim-to-real/isaac-training-environment.md).

Follow-up the same day: the retry attempt looked like "the controller goes
inactive", but driver logs show the controller stayed active and no throw ever
reached `test_ball_node` after arming. Real cause: `CatchTelemetry` was only
published during valid-ball ticks, so the Web UI never saw
`command_enabled=true` in trigger-mode idle and kept re-posting the command
toggle. Fixed with 60 Hz heartbeat telemetry (`ball_valid=false`), plus
ground-termination of the virtual ball flight (`ground_z_m`, Isaac parity).
The full chain (throw -> policy -> 500 Hz streaming -> robot follows -> hold
after grounding, controller stays active) was verified end-to-end on fake
hardware on 2026-07-02. A same-day user hardware report then confirmed the same
virtual-ball chain on the real UR3e; the remaining issue is speed/latency, not a
basic command-path failure.

## Remaining Robot Work

- Optimize the real-robot virtual-ball response. Current bring-up uses
  `v_safe_scale=0.5`, and the current policy still saturates actions against
  metadata limits, so use the Test tab to step through virtual balls before
  full-speed or overdrive real throws.
- Test watchdog behavior on hardware.
- Tune safety parameters on the real robot (`v_safe_scale`, `a_safe`,
  `loop_budget_s`, `max_tracking_error`, `start_pose_limit_rad`).
- Measure end-to-end latency with real perception.
- Redo the real Trace command test through the single stack
  (`ur3e_catch_stack --real --tracker`), measure `ros2 topic hz` on
  `ball_state_raw`/`ball_state` during throws, and decouple tracker
  publication from the GUI render if the raw rate is too low.

## Documentation/Reproducibility Gaps

- Fallback dated model behavior still matters for old rollouts, but the current
  canonical model is now present under `data/models/`.
- The dated Stage fallback export uses legacy absolute action semantics; current
  Isaac exports use incremental velocity/acceleration-limited semantics selected
  through metadata.
- `src/ur3e_catch_msgs/README.md` is documented as obsolete.
- `handeye_result.yaml` path conventions are not fully unified.

## See Also

- [Real Robot Bring-Up Runbook](../operations/real-robot-bringup-runbook.md)
- [Live Catch Loop](live-catch-loop.md)
- [Safety And Commanding](safety-and-commanding.md)
- [Message Contracts And Topics](message-contracts-and-topics.md)
- [Camera And Hand-Eye Calibration](../calibration/camera-and-handeye-calibration.md)
- [Frames And Transforms](../calibration/frames-and-transforms.md)
- [Testing And Commands](../operations/testing-and-commands.md)
- [Perception Robustness And Flight Lifecycle](../perception/perception-robustness-flight-lifecycle.md)
