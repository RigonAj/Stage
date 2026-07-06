# Current Status And Blockers

> Sources: live-catch implementation status, 2026-06-30; remaining work checklist, 2026-06-29; inconsistency review, 2026-06-30; 2026-07-02 pendant incident analysis; user hardware report, 2026-07-02; v_safe_scale UI implementation, 2026-07-03; v_safe_scale overdrive range to 4.0, 2026-07-03; agent review corrections, 2026-07-03; ball regression publisher, 2026-07-03
> Raw: [Implementation status](../../docs/Robot_Control/ur3e_live_catch_implementation_status.md); [Reste a faire](../../docs/reste_a_faire.md); [Incoherences](../../docs/incoherences_code_logique.md); [Web UI app](../../src/ur3e_web_ui/ur3e_web_ui/app.py); [Catch panel](../../src/ur3e_web_ui/ur3e_web_ui/static/js/catch_panel.js); [Live catch node](../../src/ur3e_live_catch/ur3e_live_catch/live_catch_node.py)

## Overview

The live-catch code path is implemented. The virtual-ball path has now been
validated through real UR3e command streaming, but it is still slow under the
current bring-up limits. Real perception deployment still has blocking
calibration, TF, watchdog and latency validation work.

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
- `ObservationBuilder` mirrors Isaac pass-through logic for the current export
  (`disk_radius_m=0.05`), and command mode fails closed without the hoop TF.
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

## Blockers Before Real Perception

1. Validate `T_base_camera` physically.
2. Publish and verify `base -> camera_optical`.
3. Publish and verify `wrist_3_link -> hoop_center`; without it, command mode
   holds instead of using a fallback disk pose.
4. Compare `publish_frame=base_link` against `publish_frame=camera_optical`.

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
