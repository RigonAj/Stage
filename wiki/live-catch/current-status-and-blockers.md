# Current Status And Blockers

> Sources: live-catch implementation status, 2026-06-30; remaining work checklist, 2026-06-29; inconsistency review, 2026-06-30
> Raw: [Implementation status](../../docs/Robot_Control/ur3e_live_catch_implementation_status.md); [Reste a faire](../../docs/reste_a_faire.md); [Incoherences](../../docs/incoherences_code_logique.md)

## Overview

The live-catch code path is implemented and dry-run tested, but the physical
robot/perception deployment still has blocking validation work.

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
  (`disk_radius_m=0.1`), and command mode fails closed without the hoop TF.

## Blockers Before Real Perception

1. Validate `T_base_camera` physically.
2. Publish and verify `base -> camera_optical`.
3. Publish and verify `wrist_3_link -> hoop_center`; without it, command mode
   holds instead of using a fallback disk pose.
4. Compare `publish_frame=base` against `publish_frame=camera_optical`.

## Robot Bring-Up Still Open

- Validate real command streaming with a virtual ball.
- Test watchdog behavior on hardware.
- Tune safety parameters on the real robot.
- Measure end-to-end latency with real perception.

## Documentation/Reproducibility Gaps

- Fallback dated model behavior still matters for old rollouts, but the current
  canonical model is now present under `data/models/`.
- The dated Stage fallback export uses legacy absolute action semantics; current
  Isaac exports use incremental velocity/acceleration-limited semantics selected
  through metadata.
- `src/ur3e_catch_msgs/README.md` is documented as obsolete.
- `handeye_result.yaml` path conventions are not fully unified.
- `src/ur3e_sysid/` is present locally but untracked in the current worktree.

## See Also

- [Live Catch Loop](live-catch-loop.md)
- [Safety And Commanding](safety-and-commanding.md)
- [Message Contracts And Topics](message-contracts-and-topics.md)
- [Camera And Hand-Eye Calibration](../calibration/camera-and-handeye-calibration.md)
- [Frames And Transforms](../calibration/frames-and-transforms.md)
- [Testing And Commands](../operations/testing-and-commands.md)
