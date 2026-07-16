# Wiki Log

## [2026-07-09] ingest | Trace: Option-panel ball-radius slider drives width→depth live

- Updated: [Trace Ball Tracking](perception/trace-ball-tracking.md)
- Ball radius is no longer the fixed `BALL_RADIUS_MM = 20` node constant; it is a
  live Option-panel slider (`Ui::BallRadiusMm()`, clamped 1–100 mm) pushed into
  pose calibration and per-frame tracker settings. Updated depth-from-width step
  and tuning table.

## [2026-07-08] ingest | Trace: full detailed perception pipeline + ROI accumulation, lead/coast prediction

- Updated: [Trace Ball Tracking](perception/trace-ball-tracking.md)
- Full stage-by-stage pipeline (front-end, accumulation, ribbon fit, width→3D,
  prediction), design-choice rationale, tuning table; ROI-gated accumulation
  (no circle) and node-side lead/coast trajectory prediction.

## [2026-06-29] ingest | Initial repository documentation compile

- Updated: [Project Overview](overview/project-overview.md)
- Updated: [Repository Map](overview/repository-map.md)
- Updated: [Trace Ball Tracking](perception/trace-ball-tracking.md)
- Updated: [Camera And Hand-Eye Calibration](calibration/camera-and-handeye-calibration.md)
- Updated: [UR3e Control Stack](robot-control/ur3e-control-stack.md)
- Updated: [Live Catch Loop](live-catch/live-catch-loop.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)
- Updated: [Policy Transfer And Action Semantics](sim-to-real/policy-transfer-and-action-semantics.md)
- Updated: [UR3e Actuator Identification](system-id/ur3e-actuator-identification.md)
- Updated: [Rollout Replay And Driver Setup](replay/rollout-replay-and-driver-setup.md)
- Updated: [UR3e Web UI](web-ui/ur3e-web-ui.md)
- Updated: [Testing And Commands](operations/testing-and-commands.md)
- Updated: [Wiki Maintenance](operations/wiki-maintenance.md)

## [2026-06-29] lint | Initial structure check

- Result: index/log created; topic directories use one level; articles include Sources/Raw metadata.

## [2026-06-29] ingest | Split broad pages into stable concept pages

- Updated: [Frames And Transforms](calibration/frames-and-transforms.md)
- Updated: [Message Contracts And Topics](live-catch/message-contracts-and-topics.md)
- Updated: [Safety And Commanding](live-catch/safety-and-commanding.md)
- Updated: [Observation Latency And Models](sim-to-real/observation-latency-and-models.md)
- Updated: [Source Document Map](operations/source-document-map.md)
- Updated: [Live Catch Loop](live-catch/live-catch-loop.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)
- Updated: [Camera And Hand-Eye Calibration](calibration/camera-and-handeye-calibration.md)
- Updated: [Policy Transfer And Action Semantics](sim-to-real/policy-transfer-and-action-semantics.md)
- Updated: [Testing And Commands](operations/testing-and-commands.md)
- Updated: [Wiki Maintenance](operations/wiki-maintenance.md)

## [2026-06-29] ingest | Harden agent wiki-building instructions

- Updated: [Wiki Maintenance](operations/wiki-maintenance.md)

## [2026-06-29] ingest | Add Agent Skills entry point

- Updated: [Wiki Maintenance](operations/wiki-maintenance.md)

## [2026-06-29] ingest | Clarify Isaac sim2real action contract

- Updated: [Policy Transfer And Action Semantics](sim-to-real/policy-transfer-and-action-semantics.md)
- Updated: [Observation Latency And Models](sim-to-real/observation-latency-and-models.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)
- Updated: [Testing And Commands](operations/testing-and-commands.md)

## [2026-06-29] lint | Remove missing perception source link

- Updated: [Trace Ball Tracking](perception/trace-ball-tracking.md)

## [2026-06-29] ingest | Align Isaac action integrator and actuator limits

- Updated: [Policy Transfer And Action Semantics](sim-to-real/policy-transfer-and-action-semantics.md)
- Updated: [Observation Latency And Models](sim-to-real/observation-latency-and-models.md)

## [2026-06-30] ingest | Transfer latest Isaac policies and UI random ball

- Updated: [Policy Transfer And Action Semantics](sim-to-real/policy-transfer-and-action-semantics.md)
- Updated: [Observation Latency And Models](sim-to-real/observation-latency-and-models.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)
- Updated: [Live Catch Loop](live-catch/live-catch-loop.md)
- Updated: [Safety And Commanding](live-catch/safety-and-commanding.md)
- Updated: [UR3e Web UI](web-ui/ur3e-web-ui.md)
- Updated: [Trace Ball Tracking](perception/trace-ball-tracking.md)
- Updated: [Testing And Commands](operations/testing-and-commands.md)

## [2026-07-01] ingest | Integrate Isaac policies on main

- Updated: [Policy Transfer And Action Semantics](sim-to-real/policy-transfer-and-action-semantics.md)
- Updated: [Observation Latency And Models](sim-to-real/observation-latency-and-models.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)
- Updated: [Live Catch Loop](live-catch/live-catch-loop.md)
- Updated: [Safety And Commanding](live-catch/safety-and-commanding.md)
- Updated: [UR3e Web UI](web-ui/ur3e-web-ui.md)
- Updated: [Trace Ball Tracking](perception/trace-ball-tracking.md)
- Updated: [Testing And Commands](operations/testing-and-commands.md)

## [2026-07-01] ingest | Pre-push inference contract review

- Updated: [Policy Transfer And Action Semantics](sim-to-real/policy-transfer-and-action-semantics.md)
- Updated: [Observation Latency And Models](sim-to-real/observation-latency-and-models.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)
- Updated: [Live Catch Loop](live-catch/live-catch-loop.md)
- Updated: [Safety And Commanding](live-catch/safety-and-commanding.md)

## [2026-07-01] ingest | Add Web UI model selection for live catch

- Updated: [UR3e Web UI](web-ui/ur3e-web-ui.md)
- Updated: [Policy Transfer And Action Semantics](sim-to-real/policy-transfer-and-action-semantics.md)
- Updated: [Observation Latency And Models](sim-to-real/observation-latency-and-models.md)
- Updated: [Safety And Commanding](live-catch/safety-and-commanding.md)

## [2026-07-01] ingest | Align virtual ball defaults with Isaac config

- Updated: [UR3e Web UI](web-ui/ur3e-web-ui.md)
- Updated: [Safety And Commanding](live-catch/safety-and-commanding.md)

## [2026-07-01] ingest | Add policy backend fallback

- Updated: [UR3e Web UI](web-ui/ur3e-web-ui.md)
- Updated: [Observation Latency And Models](sim-to-real/observation-latency-and-models.md)

## [2026-07-01] ingest | Align live catch frame with Isaac FirstTraining

- Updated: [Frames And Transforms](calibration/frames-and-transforms.md)
- Updated: [Live Catch Loop](live-catch/live-catch-loop.md)
- Updated: [Message Contracts And Topics](live-catch/message-contracts-and-topics.md)
- Updated: [Safety And Commanding](live-catch/safety-and-commanding.md)
- Updated: [Policy Transfer And Action Semantics](sim-to-real/policy-transfer-and-action-semantics.md)
- Updated: [Observation Latency And Models](sim-to-real/observation-latency-and-models.md)
- Updated: [UR3e Web UI](web-ui/ur3e-web-ui.md)

## [2026-07-01] ingest | Verify Web UI robot orientation

- Updated: [Frames And Transforms](calibration/frames-and-transforms.md)
- Updated: [UR3e Web UI](web-ui/ur3e-web-ui.md)

## [2026-07-01] ingest | Audit action and observation parity

- Updated: [Observation Latency And Models](sim-to-real/observation-latency-and-models.md)
- Updated: [Policy Transfer And Action Semantics](sim-to-real/policy-transfer-and-action-semantics.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)
- Updated: [Testing And Commands](operations/testing-and-commands.md)

## [2026-07-02] ingest | Add Isaac hoop visual radius to Web UI

- Updated: [Frames And Transforms](calibration/frames-and-transforms.md)
- Updated: [UR3e Web UI](web-ui/ur3e-web-ui.md)

## [2026-07-02] ingest | Document live-catch fake and real bring-up

- Updated: [Testing And Commands](operations/testing-and-commands.md)

## [2026-07-02] ingest | Diagnose pendant 2π-jump incident and harden command streaming

- Updated: [Safety And Commanding](live-catch/safety-and-commanding.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)
- Updated: [Policy Transfer And Action Semantics](sim-to-real/policy-transfer-and-action-semantics.md)

## [2026-07-02] ingest | Heartbeat telemetry, ball ground termination and fake-hardware validation

- Updated: [Message Contracts And Topics](live-catch/message-contracts-and-topics.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)

## [2026-07-02] ingest | Real-UR3e virtual-ball status and slow bring-up tuning

- Updated: [Project Overview](overview/project-overview.md)
- Updated: [Camera And Hand-Eye Calibration](calibration/camera-and-handeye-calibration.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)
- Updated: [Safety And Commanding](live-catch/safety-and-commanding.md)
- Updated: [Testing And Commands](operations/testing-and-commands.md)

## [2026-07-02] ingest | Document v_safe_scale UI gap

- Updated: [Safety And Commanding](live-catch/safety-and-commanding.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)
- Updated: [UR3e Web UI](web-ui/ur3e-web-ui.md)

## [2026-07-03] ingest | Add v_safe_scale Test-tab control

- Updated: [Safety And Commanding](live-catch/safety-and-commanding.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)
- Updated: [UR3e Web UI](web-ui/ur3e-web-ui.md)

## [2026-07-03] ingest | Extend v_safe_scale test range

- Updated: [Safety And Commanding](live-catch/safety-and-commanding.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)
- Updated: [UR3e Web UI](web-ui/ur3e-web-ui.md)

## [2026-07-03] ingest | Extend v_safe_scale overdrive to 4.0

- Updated: [Safety And Commanding](live-catch/safety-and-commanding.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)
- Updated: [UR3e Web UI](web-ui/ur3e-web-ui.md)

## [2026-07-03] ingest | Apply agent-review Volet 1: corrections, Isaac env page, runbook, latency plan

- Source: raw/reviews/2026-07-02-stage-wiki-and-training-review.md (agent
  review excerpt); raw/isaac/ snapshots of the Isaac FirstTraining README and
  environment-and-frames doc.
- Corrections: removed the false "ur3e_sysid untracked" claim (package is
  tracked per `git ls-files`); the Isaac cfg limit halving/±π bounds verified
  as uncommitted working-tree changes in the local Isaac checkout (not lost,
  not committed) and both pages now say so.
- Created: [Isaac Training Environment](sim-to-real/isaac-training-environment.md)
- Created: [Real Robot Bring-Up Runbook](operations/real-robot-bringup-runbook.md)
- Updated: [Observation Latency And Models](sim-to-real/observation-latency-and-models.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)
- Updated: [Policy Transfer And Action Semantics](sim-to-real/policy-transfer-and-action-semantics.md)
- Updated: [Testing And Commands](operations/testing-and-commands.md)
- Updated: [Safety And Commanding](live-catch/safety-and-commanding.md)
- Contract: AGENTS.md "How to Start" now defers to wiki/index.md instead of a
  duplicated page list; sibling-repo (Isaac) sources are snapshotted into
  raw/isaac/; docs/Agent_Wiki/Current_Status.md reduced to a navigation
  pointer; SKILL.md ingest note updated.

## [2026-07-03] ingest | Ballistic-regression ball publisher (Isaac pop parity)

- Source: new code `src/ur3e_live_catch/ur3e_live_catch/ball_regression.py` /
  `ball_regression_node.py`, `use_ball_regression` launch wiring, live-node
  `use_ball_state_velocity` + velocity-filter reset bugfix; sim validation with
  noisy/dropout virtual ball (single pop, fit velocity ≈ v0, ground end).
- Updated: [Live Catch Loop](live-catch/live-catch-loop.md)
- Updated: [Message Contracts And Topics](live-catch/message-contracts-and-topics.md)
- Updated: [Observation Latency And Models](sim-to-real/observation-latency-and-models.md)
- Updated: [Testing And Commands](operations/testing-and-commands.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)

## [2026-07-03] ingest | Tracker publishes Trace-pipeline pose (pose_source)

- Source: code review of Ball_Tracking_Cpp confirming all three regression
  paths (Update3DTrack, Trace ribbon, Draw3DScene stabilized curve) were
  GUI-only; new `pose_source` parameter ("trace" in bring-up config, "circle"
  code default) publishes the outlier-filtered mid-window Trace pose stamped at
  its own event time, remap inverted back to camera_optical.
- Updated: [Trace Ball Tracking](perception/trace-ball-tracking.md)
- Updated: [Message Contracts And Topics](live-catch/message-contracts-and-topics.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)

## [2026-07-06] ingest | Extrinsic calibration runbook and result-path alignment

- Source: code verification pass before the physical eye-to-hand session
  (solver/collector self-tests, web UI calibration tests all pass); result
  path aligned on `calibration/handeye_result.yaml` (run_handeye_session.sh
  doc, solve_handeye.py now creates output parent dirs); new French operator
  doc `docs/Robot_Control/procedure_calibration_extrinseque.md`; comparison
  with the standalone repo `event-camera-ca` (confirmed to be a subset of
  this workspace's scripts, nothing borrowed).
- Created: [Extrinsic Calibration Runbook](calibration/extrinsic-calibration-runbook.md)
- Updated: [Camera And Hand-Eye Calibration](calibration/camera-and-handeye-calibration.md)

## [2026-07-06] ingest | Support3D ghost clocking fixed; mire chain verified

- Source: operator report (viewer ghost 90 deg off about the tool roll axis);
  verification of the full mire chain (serve_phone_mire.py layout self-test,
  physical-pixel drawing in phone_mire.html, screen-center object-point
  origin, fullscreen gate in fetch_external_layout, TF base->tool0 reads);
  `support_mount.json` yaw set to +pi/2 about tool0 Z with the plate offset
  rotated accordingly (display-only, T_tool0_mire stays hand-eye co-solved).
- Updated: [Extrinsic Calibration Runbook](calibration/extrinsic-calibration-runbook.md)

## [2026-07-06] ingest | Left-hand racket (hold_side) variant across Isaac and Stage

- Source: implementation session 2026-07-06 — Isaac FirstTraining left-hand
  task (`Template-Firsttraining-Direct-Left-v0`, mirrored USD
  `UR-with-gripper-left.usd` via `scripts/make_left_hand_usd.py`, mirrored
  ball x ranges, `hold_side` + disk/ball fields in the metadata export,
  `FT_TASK` env.zsh switch); Stage side: `hold_side` launch argument for the
  hoop_center TF, `hold_side=right` retro-annotation of deployed models,
  web UI allowlist `latest-left`/`best-left`, racket hold-side toggle
  (hoop visual + yz-mirrored ball defaults/Isaac ranges + mismatch warning).
  Also corrected the stale Isaac repo path claim: on this PC the training
  repo is `~/Documents/IsaacTrain/Cartpole/Cartpole/FirstTraining`.
- Updated: [Isaac Training Environment](sim-to-real/isaac-training-environment.md)
- Updated: [Frames And Transforms](calibration/frames-and-transforms.md)
- Updated: [UR3e Web UI](web-ui/ur3e-web-ui.md)
- Updated: [Testing And Commands](operations/testing-and-commands.md)

## [2026-07-06] ingest | Hold-side train/play shortcuts in Isaac env.zsh

- Source: user request; env.zsh now defines train-left / train-right /
  play-left / play-right wrappers pinning FT_TASK (and thus the checkpoint
  root) per racket hold side.
- Updated: [Testing And Commands](operations/testing-and-commands.md)
- Updated: [Isaac Training Environment](sim-to-real/isaac-training-environment.md)

## [2026-07-09] ingest | Intrinsic calibration runbook

- Added: [Intrinsic Calibration Runbook](calibration/intrinsic-calibration-runbook.md)
- Updated: wiki/index.md

## [2026-07-09] ingest | Robust rotation/perspective blob association

- event_mire_calibration.py: associate_blobs_to_layout rewritten as a
  corner-seeded homography + ICP match (replaces the image-axis row/column
  split that failed on tilted/rolled views). --self-test now covers strongly
  tilted synthetic poses for mire/grid_5x4/grid_7x5.
- Updated: [Intrinsic Calibration Runbook](calibration/intrinsic-calibration-runbook.md)

## [2026-07-09] ingest | Intrinsic association corner-candidate fix

- event_mire_calibration.py: homography association now tries convex-hull
  quadrilateral corner candidates when the PCA corner cycle collapses; default
  `--min-matched 0` means the active pattern's full point count.
- Updated: [Intrinsic Calibration Runbook](calibration/intrinsic-calibration-runbook.md)

## [2026-07-09] ingest | Real Trace perception test procedure

- Added: [Real Perception Trace Test Runbook](perception/real-perception-trace-test.md)
- Source: new operator procedure
  `docs/Robot_Control/procedure_test_perception_trace.md` covering the current
  calibrated files, tracker `pose_source:=trace`, TF publication, dry-run
  live-catch integration and attention points before robot command mode.
- Updated: [Trace Ball Tracking](perception/trace-ball-tracking.md)

## [2026-07-09] ingest | Real perception runbook uses left model

- Updated: [Real Perception Trace Test Runbook](perception/real-perception-trace-test.md)
- Source: operator correction that inference for the real perception test uses
  `data/models/latest-left/policy_deterministic.onnx`; procedure now requires
  the left `hoop_center` TF and matching hold-side state.

## [2026-07-09] ingest | Tracker ball radius launch option

- Updated: [Trace Ball Tracking](perception/trace-ball-tracking.md)
- Updated: [Real Perception Trace Test Runbook](perception/real-perception-trace-test.md)
- Source: code change adding `ball_radius_mm` as a `ball_tracking_cpp` ROS
  parameter and `live_catch.launch.py` launch argument, while keeping the
  Option-panel slider for live adjustment.

## [2026-07-09] ingest | Tracker displays sampled events

- Updated: [Trace Ball Tracking](perception/trace-ball-tracking.md)
- Updated: [Real Perception Trace Test Runbook](perception/real-perception-trace-test.md)
- Source: code change making the GUI 2D texture draw `camera.Samples` after
  `Echantillon(maxevent)`, while Trace accumulation continues to use the full
  filtered/undistorted event streams.

## [2026-07-09] ingest | Explicit tracker intrinsics parameter

- Updated: [Trace Ball Tracking](perception/trace-ball-tracking.md)
- Updated: [Real Perception Trace Test Runbook](perception/real-perception-trace-test.md)
- Source: code change adding `camera_calibration_file` to `ball_tracking_cpp`
  and `live_catch.launch.py`, defaulting the live/replay path to
  `recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml`.

## [2026-07-09] ingest | First real Trace command test diagnosis + single-producer enforcement

- Created: [Single Producer Contract](live-catch/single-producer-contract.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)
- Updated: [Message Contracts And Topics](live-catch/message-contracts-and-topics.md)
- Updated: [Safety And Commanding](live-catch/safety-and-commanding.md)
- Updated: [Real Perception Trace Test Runbook](perception/real-perception-trace-test.md)
- Updated: [Trace Ball Tracking](perception/trace-ball-tracking.md)
- Updated: [UR3e Web UI](web-ui/ur3e-web-ui.md)
- Updated: [Testing And Commands](operations/testing-and-commands.md)
- Source: docs/Robot_Control/analyse_pipeline_commande_trace_2026-07-09.md and
  the same-day code changes: live_catch diagnostics producer-conflict watchdog
  (fail-closed ball-topic gate), Web UI command_enabled flap detection, stack
  --tracker/--ball-radius/--camera-calib options with use_test_ball now
  configurable, and wider --stop cleanup (tracker, regression, manual
  live_catch launches).

## [2026-07-09] ingest | Ordered real command-session procedure + stack --hold-side fix

- Updated: [Real Perception Trace Test Runbook](perception/real-perception-trace-test.md)
- Updated: [Single Producer Contract](live-catch/single-producer-contract.md)
- Updated: [Testing And Commands](operations/testing-and-commands.md)
- Source: docs/Robot_Control/procedure_lancement_reel_trace_commande.md (full
  ordered operator checklist for a real Trace command session, left model,
  45 mm radius) and a stack-script fix: launch_ur3e_virtual_ball_stack.sh no
  longer hardcodes the right-side hoop_xyz/hoop_quat (which silently overrode
  hold_side:=left); a new --hold-side option / UR3E_HOLD_SIDE env drives the
  hoop TF side and hoop overrides are only passed when explicitly set.

## [2026-07-09] ingest | Perception-transmission improvement plan, items 1.1/1.2/2.1/2.2

- Updated: [Live Catch Loop](live-catch/live-catch-loop.md)
- Updated: [Message Contracts And Topics](live-catch/message-contracts-and-topics.md)
- Updated: [Trace Ball Tracking](perception/trace-ball-tracking.md)
- Updated: [Testing And Commands](operations/testing-and-commands.md)
- Source: docs/Robot_Control/plan_amelioration_perception_transmission.md and
  same-day code changes: regression min_input_confidence measurement gate,
  anisotropic depth_sigma_scale weighting (camera-ray residuals, bring-up 8.0),
  tracker trace_lead_ms/trace_hold_ms ROS params pinned to 0 under
  use_ball_regression, and scripts/replay_ball_regression.py offline tuner.

## [2026-07-09] ingest | Provisional 0.2 s regression lead, runtime-tunable

- Updated: [Message Contracts And Topics](live-catch/message-contracts-and-topics.md)
- Source: operator request (2026-07-09) + code change: live_catch.yaml
  ball_regression_node.lead_time_s set to 0.2 s pending the plan-2.4 latency
  measurement; BallRegression.set_lead_time + a node set_parameters callback
  make it live-tunable via `ros2 param set /ball_regression_node lead_time_s`
  (bounded [0, 1] s). Side effects documented: perception_age ≈ -lead and
  ground-termination lead seconds before real impact.

## [2026-07-10] ingest | Independent perception robustness and flight-lifecycle review

- Added: [Perception Robustness And Flight Lifecycle](perception/perception-robustness-flight-lifecycle.md)
- Updated: [Trace Ball Tracking](perception/trace-ball-tracking.md)
- Updated: [Real Perception Trace Test Runbook](perception/real-perception-trace-test.md)
- Updated: [Message Contracts And Topics](live-catch/message-contracts-and-topics.md)
- Updated: [Safety And Commanding](live-catch/safety-and-commanding.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)
- Updated: [Live Catch Loop](live-catch/live-catch-loop.md)
- Updated: [Observation Latency And Models](sim-to-real/observation-latency-and-models.md)
- Updated: [Isaac Training Environment](sim-to-real/isaac-training-environment.md)
- Updated: [Real Robot Bring-Up Runbook](operations/real-robot-bringup-runbook.md)
- Secondary navigation: `docs/Agent_Wiki/Perception_Trace.md` now routes to the
  review and compiled lifecycle page.
- Source: docs/Robot_Control/revue_perception_robuste_controle_fluide_2026-07-10.md,
  code/config audit and current `latest-left` metadata. Records the duplicate
  stack as the diagnosed first-test cause, but keeps real perception unvalidated;
  identifies provisional 0.2 s lead, tracker/regression timestamp semantics,
  binary Trace confidence, periodic-only command-authority checks and missing
  explicit flight start/end phases as the next blockers.

## [2026-07-10] ingest | Ball-regression extra lead defaults to zero

- Updated: [Perception Robustness And Flight Lifecycle](perception/perception-robustness-flight-lifecycle.md)
- Updated: [Real Perception Trace Test Runbook](perception/real-perception-trace-test.md)
- Updated: [Message Contracts And Topics](live-catch/message-contracts-and-topics.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)
- Updated: [Observation Latency And Models](sim-to-real/observation-latency-and-models.md)
- Updated: [Real Robot Bring-Up Runbook](operations/real-robot-bringup-runbook.md)
- Source: operator decision and `src/ur3e_live_catch/config/live_catch.yaml`.
  The former provisional `lead_time_s: 0.2` bring-up override is now 0.0,
  matching `RegressionConfig` and the synchronous current ball/current robot
  policy observation. Nonzero future prediction remains runtime-tunable but is
  not a deployment default without timing, replay and training validation.

## [2026-07-10] ingest | Hand-eye phone layout survives server restarts

- Updated: [Extrinsic Calibration Runbook](calibration/extrinsic-calibration-runbook.md)
- Source: operator request and changes to `scripts/serve_phone_mire.py` and
  `scripts/event_mire_calibration.py`. The server now persists the last valid
  fullscreen phone layout and provides a documented Poco X7 Pro fallback, so
  hand-eye capture no longer requires a phone reload after every PC-side
  restart. Live non-fullscreen layouts remain rejected.

## [2026-07-10] ingest | Dated physical hand-eye transform example

- Updated: [Camera And Hand-Eye Calibration](calibration/camera-and-handeye-calibration.md)
- Source: `calibration/handeye_result.yaml` generated 2026-07-10 at 14:30:52
  Europe/Paris from 18 cleaned physical samples. Records the example
  `base -> camera_optical` and `tool0 -> screen_center` transforms, static TF
  command, solver/residual validation and remaining physical parity checks.

## [2026-07-16] ingest | Centralize useful project commands in the root README

- Updated: [Testing And Commands](operations/testing-and-commands.md)
- Source: expanded `README.md` command reference covering dependencies, scoped
  builds, perception, intrinsic/extrinsic calibration, UR3e/UI and live-catch
  bring-up, diagnostics, replay, system identification, tests and maintenance.

## [2026-07-16] ingest | Split quick start from detailed command reference

- Updated: [Testing And Commands](operations/testing-and-commands.md)
- Source: `README.md` now keeps only perception, real robot/UI, hand-eye TF and
  fake-hardware live-catch launch commands; the complete inventory moved to
  `docs/COMMANDS.md`.

## [2026-07-16] ingest | Clarify virtual-ball testing on the real UR3e

- Updated: [Testing And Commands](operations/testing-and-commands.md)
- Source: operator validation and `README.md` quick-start update. The real-UR3e
  virtual-ball command omits `--tracker`, uses the left hold/model pair, and
  starts in dry-run; `--tracker` intentionally disables virtual-ball controls.

## [2026-07-16] ingest | Real-ball session blocked before ballistic regression

- Updated: [Real Perception Trace Test Runbook](perception/real-perception-trace-test.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)
- Updated: [Testing And Commands](operations/testing-and-commands.md)
- Source: operator real-UR3e/real-ball report, live ROS graph/parameter
  inspection and current-session ROS logs. The tracker, regression and live
  node were singular and correctly wired, but the regression never left idle:
  C++ Trace produced no first valid `/ball_state_raw` sample. The 60 Hz
  `/ball_state` stream was only an invalid heartbeat and the live watchdog
  correctly held the robot with `no_valid_ball`. Records the launch and
  boundary-diagnostic commands and makes robot-disarmed raw Trace validation
  the first blocker.

## [2026-07-16] ingest | Tracker reader-mode root cause fixed; perception chain validated offline

- Updated: [Trace Ball Tracking](perception/trace-ball-tracking.md)
- Updated: [Real Perception Trace Test Runbook](perception/real-perception-trace-test.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)
- Updated: [Testing And Commands](operations/testing-and-commands.md)
- Source: same-day code diagnosis and changes in `Ball_Tracking_Cpp`
  (publisher node, Gui) + `live_catch.yaml`/launch. Root cause of the
  2026-07-16 no-valid-sample session: the GUI hardcoded `reader_mode = true`,
  so the tracker always started in File mode and processed no camera events;
  polarity also defaulted to Negative. New ROS parameters: `use_reader`
  (default live camera), `trace_polarity_mode` (default all), `record` +
  `record_file` (default H5 capture to `recordings/realtest.h5`, overwritten
  per session), `reader_file` (scripted replay, autoplay); throttled
  idle/no-camera warnings and a 2 s trace-status heartbeat with stage peaks.
  Offline replay of the 2026-07-09 real throw
  (`recordings/realtest_2026-07-09_backup.h5`) through tracker + regression +
  hand-eye TF produced 12–13 valid raw samples and 27 valid fitted samples on
  `/ball_state` (flight idle→collecting→tracking→ended, RMS 0.013 m).

## [2026-07-16] ingest | README gains the ordered disarmed real-ball test procedure

- Updated: [Testing And Commands](operations/testing-and-commands.md)
- Source: user request; `README.md` section 5 now holds the four-terminal
  robot-disarmed real-ball procedure (hand-eye TF, `--tracker` stack with
  `--ball-radius 45.0` and left model, raw/fitted echo boundaries, optional
  rosbag), the startup log checks, the heartbeat/validity acceptance criteria
  and the post-session copy of the overwritten `recordings/realtest.h5`.
  Radius semantics documented: 45.0 mm radius = Ø 90 mm ball.

## [2026-07-16] ingest | Reader UI preselects realtest.h5 at startup

- Updated: [Trace Ball Tracking](perception/trace-ball-tracking.md)
- Source: user request; new `ball_tracking_cpp` parameter
  `default_reader_file` (default `realtest.h5`, set in `live_catch.yaml`).
  It fills the GUI Read-file box and switches the reader source to
  Recordings at startup without changing modes, so File → Play replays the
  session buffer in one click. `reader_file` (forced autoplay replay) takes
  priority. Verified: camera-mode startup unchanged, forced replay unchanged.

## [2026-07-16] ingest | Recording made manual again + timestamp archiving (data-loss incident)

- Updated: [Trace Ball Tracking](perception/trace-ball-tracking.md)
- Updated: [Real Perception Trace Test Runbook](perception/real-perception-trace-test.md)
- Updated: [Current Status And Blockers](live-catch/current-status-and-blockers.md)
- Updated: [Testing And Commands](operations/testing-and-commands.md)
- Source: operator incident during the first live real-ball session after the
  reader-mode fix. The brief `record: true` default truncated the previous
  `realtest.h5` on session start and a File-mode review click closed an empty
  writer over it. Note: the same session log showed `published=210` — live
  Trace published 210 valid samples on real throws, first live evidence the
  perception path works. Changes: `record` default back to `false` (manual
  GUI REC toggle; `record:=true` arms at launch), and `Gui::OpenWriterFromUi`
  now archives any existing non-empty recording target as
  `<name>_YYYYMMDD_HHMMSS.h5` (collision-safe) instead of truncating.
  The 2026-07-09 real throw remains safe in
  `recordings/realtest_2026-07-09_backup.h5` and was restored to
  `recordings/realtest.h5` for the one-click GUI replay.
