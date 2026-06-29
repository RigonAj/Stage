# Current Status

This note summarizes the current state from `docs/reste_a_faire.md`,
`docs/incoherences_code_logique.md` and
`docs/Robot_Control/ur3e_live_catch_implementation_status.md`.

## Working State

- Live catch is implemented as a 60 Hz Python ROS 2 node with direct module
  calls on the hot path.
- `ur3e_catch_msgs` defines typed `BallState` and `CatchTelemetry` messages.
- `ball_tracking_cpp` now publishes native `BallState` in meters with a nonempty
  `header.frame_id`; the legacy float array path remains only as fallback.
- `ActionMapper -> SafetyLimiter -> CommandStreamer` is wired behind
  `enable_command`.
- `enable_command=false` is the default and means dry-run: telemetry and targets
  are computed, but no robot command is emitted.
- The web UI has a Test tab for virtual-ball launch, policy ghost and hot
  command enable/disable.
- The current TorchScript policy has been checked against rollout actions and
  does not require a separate scaler file.

## Open Blockers Before Real Perception

1. Validate the eye-to-hand calibration physically and produce
   `T_base_camera`.
2. Publish stable TFs for `base -> camera_optical` and
   `wrist_3_link -> hoop_center`.
3. Compare `test_ball_node publish_frame=base` against
   `publish_frame=camera_optical` to isolate TF/extrinsic errors.

## Open Robot Bring-Up Work

1. Validate command streaming on the real UR3e with a virtual ball.
2. Test watchdog behavior on the real robot.
3. Tune `a_safe`, `loop_budget_s` and `max_tracking_error` on hardware.

## Open Perception Work

1. Measure end-to-end latency with real perception.
2. Use the native `BallState` path for perception tests; reserve
   `use_adapter:=true` for old tracker builds because the legacy path timestamps
   at reception.

## Reproducibility And Documentation Gaps

- `data/models/` is the intended canonical model location, but the live node may
  still fall back to the dated rollout export.
- `src/ur3e_catch_msgs/README.md` is documented as obsolete.
- `handeye_result.yaml` path conventions are not fully unified across script and
  UI docs.
- Calibration assets need a clone-clean decision.
- `src/ur3e_sysid/` exists locally and is documented, but is currently untracked.

## Recommended Bring-Up Order

1. Source the workspace and build only the needed packages.
2. Run live catch in dry-run with `use_test_ball:=true`.
3. Verify telemetry and policy ghost from the web UI.
4. Validate controller switching and watchdog with the virtual ball on the real
   robot at reduced speed.
5. Validate camera extrinsics and static TFs.
6. Repeat the dry-run with real tracker `BallState`.
7. Measure latency before attempting real ball interception.
