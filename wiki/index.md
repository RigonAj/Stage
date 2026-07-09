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
| [Trace Ball Tracking](perception/trace-ball-tracking.md) | Trace algorithm, C++ entry points, pose_source trace/circle contract, ROI-gated accumulation (no circle), node-side lead/coast trajectory prediction, Option-panel ball-radius slider and depth-estimation risks. | 2026-07-09 |

## calibration

Camera intrinsics, hand-eye calibration and TF contracts.

| Article | Summary | Updated |
|---------|---------|---------|
| [Camera And Hand-Eye Calibration](calibration/camera-and-handeye-calibration.md) | Intrinsics, phone-mire hand-eye workflow, aligned result path, base_link parity validation gates and current blockers. | 2026-07-06 |
| [Extrinsic Calibration Runbook](calibration/extrinsic-calibration-runbook.md) | Operator checklist for the physical eye-to-hand session: prerequisites, self-tests, capture, solve acceptance gates, TF publication and validation. | 2026-07-06 |
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
| [Live Catch Loop](live-catch/live-catch-loop.md) | Single-process 60 Hz live-catch pipeline, modules, optional ballistic-regression ball publisher, metadata-driven action mapping, safety and tests. | 2026-07-03 |
| [Message Contracts And Topics](live-catch/message-contracts-and-topics.md) | `BallState` (velocity convention, regression stamps), `ball_state_raw`, `CatchTelemetry`, topics, producers, idle heartbeat/`ball_valid` contract and timestamp rules. | 2026-07-03 |
| [Safety And Commanding](live-catch/safety-and-commanding.md) | Command modes, model/v_safe_scale runtime gates, metadata/model limits, safety limiter, watchdog, controller switching, 500 Hz interpolated streaming, start-pose ±2π gate, hardware gates and slow bring-up tuning. | 2026-07-03 |
| [Current Status And Blockers](live-catch/current-status-and-blockers.md) | Working state, real-UR3e virtual-ball validation, ballistic-regression ball publisher (sim-validated), Test-tab v_safe_scale tuning, remaining speed/perception blockers, 2026-07-02 pendant incident diagnosis, uncommitted Isaac cfg limit change and current model/action status. | 2026-07-03 |

## sim-to-real

PPO transfer, policy semantics, action space and latency.

| Article | Summary | Updated |
|---------|---------|---------|
| [Isaac Training Environment](sim-to-real/isaac-training-environment.md) | FirstTraining 33-D observation layout, action integrator, reward, terminations, ball distribution, left-hand (hold_side) mirrored task variant, cfg-limit state, train/play/evaluate/export commands and cross-repo export sync. | 2026-07-06 |
| [Policy Transfer And Action Semantics](sim-to-real/policy-transfer-and-action-semantics.md) | Legacy absolute vs current incremental target-integrator semantics, metadata-driven mapper selection and full-speed metadata limits vs bring-up slow-down (Isaac limit halving still uncommitted). | 2026-07-03 |
| [Observation Latency And Models](sim-to-real/observation-latency-and-models.md) | Observation construction, ball velocity source (fit vs EMA), latency instrumentation, regression stamp semantics, p50/p95/p99 measurement plan with acceptance anchors, current model exports and metadata validation. | 2026-07-03 |

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
| [UR3e Web UI](web-ui/ur3e-web-ui.md) | FastAPI/Three.js UI structure, base_link robot/viewer contract, Isaac hoop visual, Test tab, model selector with hold_side labels, racket hold-side toggle (mirrored ball + hoop visual), v_safe_scale control, calibration tab, rollout tab and API scope. | 2026-07-06 |

## operations

Agent workflow, commands and wiki maintenance.

| Article | Summary | Updated |
|---------|---------|---------|
| [Testing And Commands](operations/testing-and-commands.md) | Build, launch, live-catch fake/real bring-up status, hold_side:=left bring-up, package tests, Isaac sim2real export/check commands (current Isaac repo path, FT_TASK left variant) and wiki maintenance commands. | 2026-07-06 |
| [Real Robot Bring-Up Runbook](operations/real-robot-bringup-runbook.md) | Operator checklist before enable_command, ±2π start-pose gate procedure, staged v_safe_scale ramp-up and monitoring points. | 2026-07-03 |
| [Wiki Maintenance](operations/wiki-maintenance.md) | Ingest/query/lint rules adapted from the Karpathy LLM wiki pattern. | 2026-06-29 |
| [Source Document Map](operations/source-document-map.md) | How the raw Markdown docs are combined or split into compiled wiki concepts. | 2026-06-29 |
