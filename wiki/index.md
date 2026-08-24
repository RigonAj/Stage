# Knowledge Base Index

## overview

Global repository map and project-level synthesis.

| Article | Summary | Updated |
|---------|---------|---------|
| [Project Overview](overview/project-overview.md) | Overall goal, architecture and active state, including real-UR3e virtual-ball validation and remaining real-perception work. | 2026-07-02 |
| [Repository Map](overview/repository-map.md) | Package responsibilities, high-value files and context-loading boundaries. | 2026-06-29 |

## perception

Event-camera ball tracking and 3D pose estimation.

| Article | Summary | Updated |
|---------|---------|---------|
| [Trace Ball Tracking](perception/trace-ball-tracking.md) | Trace algorithm, ROI/ribbon/width-depth pipeline, output contract, event-clock re-anchoring, binary live confidence, the 2026-07-16 I/O parameters (camera-mode startup, polarity all, H5 recording, scripted replay, trace-status heartbeat), and the 2026-07-17 latency optimization (analysis/publish decoupled from the 60 FPS render via `trace_analysis_period_ms`, incremental live undistortion, paused-reader cache, DBSCAN gated off in the live-catch config). | 2026-07-17 |
| [Trace Vs Circle Fitting Benchmark](perception/trace-vs-circle-benchmark.md) | Headless offline benchmark replaying Isaac/v2e sequences through both production pipelines: intrinsics/frame/timestamp contracts, the scale-dependent apparent-size bias of the ribbon edge estimator and its density-adaptive fix (RMSE 3D 1.72 -> 0.157 m at range, 0.058 -> 0.045 m nominal), why circle fitting collapses to a 1.3% detection rate at range, the parameter-search script and its ablation, and why the pre-2026-08-04 benchmark numbers are void. | 2026-08-04 |
| [Real Perception Trace Test Runbook](perception/real-perception-trace-test.md) | Operator procedure for calibrated real Trace and the single `--tracker` stack; the 2026-07-16 no-valid-sample diagnosis, its root cause (tracker started in File/reader mode) and the offline replay validation of the fixed chain. | 2026-07-16 |
| [Perception Robustness And Flight Lifecycle](perception/perception-robustness-flight-lifecycle.md) | Independent review of the 2026-07-09 incident: lead=0 default decision, timestamp/quality gaps, explicit throw lifecycle, left-policy envelope and validation gates. | 2026-07-10 |

## calibration

Camera intrinsics, hand-eye calibration and TF contracts.

| Article | Summary | Updated |
|---------|---------|---------|
| [Camera And Hand-Eye Calibration](calibration/camera-and-handeye-calibration.md) | Intrinsics, phone-mire workflow, current strict-landscape 2026-07-23 physical transform with validation metrics, base_link parity gates and remaining physical checks. | 2026-07-23 |
| [Extrinsic Calibration Runbook](calibration/extrinsic-calibration-runbook.md) | Operator checklist for the physical eye-to-hand session: strict landscape-only phone mire with browser/server/collector guards, persistent safe layout, capture, solve acceptance gates, TF publication and validation. | 2026-07-23 |
| [Intrinsic Calibration Runbook](calibration/intrinsic-calibration-runbook.md) | Operator checklist to redo DVXplorer intrinsics: working/output dirs, env.sh `calib` alias, event-mire capture, robust solve and acceptance gates. | 2026-07-09 |
| [Frames And Transforms](calibration/frames-and-transforms.md) | `base_link` policy frame, TF, UI robot orientation, hold_side-driven hoop_center TF (right/left racket mount), hoop radius distinction and unit contracts used by perception, live catch and UI. | 2026-07-06 |

## robot-control

UR3e driver, controllers, MoveIt, web backend and operator safety gates.

| Article | Summary | Updated |
|---------|---------|---------|
| [UR3e Control Stack](robot-control/ur3e-control-stack.md) | Real robot control architecture, process launch, controllers, frames and safety gates. | 2026-06-29 |

## live-catch

Closed-loop perception-to-policy-to-robot path.

| Article | Summary | Updated |
|---------|---------|---------|
| [Live Catch Loop](live-catch/live-catch-loop.md) | Single-process 60 Hz live-catch pipeline, optional ballistic regression, measurement-purity/anisotropic-depth handling, action mapping, safety and link to the explicit flight-lifecycle review. | 2026-07-10 |
| [Message Contracts And Topics](live-catch/message-contracts-and-topics.md) | `BallState`, raw/fitted topics, velocity/confidence/heartbeat contracts, single-producer rule, lead=0 default and the measurement-vs-state timestamp mismatch. | 2026-07-10 |
| [Safety And Commanding](live-catch/safety-and-commanding.md) | Command modes, model/v_safe_scale runtime gates, safety/watchdog, 500 Hz streaming, ±2π gate, producer-conflict protection and the open synchronous command-authority exclusivity gap. | 2026-07-10 |
| [Single Producer Contract](live-catch/single-producer-contract.md) | 2026-07-09 duplicate live_catch_node / dual ball_state producer incident, failure signatures (twitching robot, flapping UI command state), diagnostics watchdog, UI flap detection, `--tracker` stack rule and pre-command checks. | 2026-07-09 |
| [Current Status And Blockers](live-catch/current-status-and-blockers.md) | Working state, diagnosed duplicate-stack incident, lead=0 baseline and the 2026-07-16 real-ball result: reader-mode root cause fixed, perception chain validated offline on the recorded throw, live-throw confirmation pending. | 2026-07-16 |

## sim-to-real

PPO transfer, policy semantics, action space and latency.

| Article | Summary | Updated |
|---------|---------|---------|
| [Isaac Training Environment](sim-to-real/isaac-training-environment.md) | FirstTraining observation/action/reward contracts, right and current `latest-left` throw envelopes/metadata, hold-side geometry, limits, commands and export sync. | 2026-07-10 |
| [Policy Transfer And Action Semantics](sim-to-real/policy-transfer-and-action-semantics.md) | Legacy absolute vs current incremental target-integrator semantics, metadata-driven mapper selection and full-speed metadata limits vs bring-up slow-down (Isaac limit halving still uncommitted). | 2026-07-03 |
| [Observation Latency And Models](sim-to-real/observation-latency-and-models.md) | Observation construction, velocity source, lead=0 default, tracker/regression timestamp mismatch, required timing split, conditional latency plan and model management. | 2026-07-10 |

## system-id

UR3e actuator identification for simulation transfer.

| Article | Summary | Updated |
|---------|---------|---------|
| [UR3e Actuator Identification](system-id/ur3e-actuator-identification.md) | System-id program, measured gains, validation caveats and package entry points. | 2026-06-29 |

## replay

Open-loop rollout replay and UR driver setup.

| Article | Summary | Updated |
|---------|---------|---------|
| [Rollout Replay And Driver Setup](replay/rollout-replay-and-driver-setup.md) | Real-robot replay semantics, realized-vs-target distinction and current/legacy driver setup. | 2026-06-29 |

## web-ui

Browser UI and visualization/control panels.

| Article | Summary | Updated |
|---------|---------|---------|
| [UR3e Web UI](web-ui/ur3e-web-ui.md) | FastAPI/Three.js UI structure, base_link robot/viewer contract, Isaac hoop visual, Test tab, model selector with hold_side labels, racket hold-side toggle (mirrored ball + hoop visual), v_safe_scale control, command-flap conflict warning, calibration tab, rollout tab and API scope. | 2026-07-09 |

## operations

Agent workflow, commands and wiki maintenance.

| Article | Summary | Updated |
|---------|---------|---------|
| [Testing And Commands](operations/testing-and-commands.md) | README quick start, detailed command reference, report compilation, the exact 2026-07-16 real-ball launch/diagnostic commands and the offline replay / rosbag diagnostic commands. | 2026-07-16 |
| [Real Robot Bring-Up Runbook](operations/real-robot-bringup-runbook.md) | Operator checklist before enable_command, ±2π gate, staged v_safe_scale ramp-up, monitoring and the current lead=0/timestamp-latency warning. | 2026-07-10 |
| [Wiki Maintenance](operations/wiki-maintenance.md) | Ingest/query/lint rules adapted from the Karpathy LLM wiki pattern. | 2026-06-29 |
| [Source Document Map](operations/source-document-map.md) | How the raw Markdown docs are combined or split into compiled wiki concepts. | 2026-06-29 |
