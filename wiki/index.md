# Knowledge Base Index

## overview

Global repository map and project-level synthesis.

| Article | Summary | Updated |
|---------|---------|---------|
| [Project Overview](overview/project-overview.md) | Overall goal, architecture and active state of the event-camera-to-UR3e project. | 2026-06-29 |
| [Repository Map](overview/repository-map.md) | Package responsibilities, high-value files and context-loading boundaries. | 2026-06-29 |

## perception

Event-camera ball tracking and 3D pose estimation.

| Article | Summary | Updated |
|---------|---------|---------|
| [Trace Ball Tracking](perception/trace-ball-tracking.md) | Trace algorithm, C++ entry points, output contracts and depth-estimation risks. | 2026-07-01 |

## calibration

Camera intrinsics, hand-eye calibration and TF contracts.

| Article | Summary | Updated |
|---------|---------|---------|
| [Camera And Hand-Eye Calibration](calibration/camera-and-handeye-calibration.md) | Intrinsics, phone-mire hand-eye workflow, validation gates and current blockers. | 2026-06-29 |
| [Frames And Transforms](calibration/frames-and-transforms.md) | `base_link` policy frame, TF, UI robot orientation, hoop radius distinction and unit contracts used by perception, live catch and UI. | 2026-07-02 |

## robot-control

UR3e driver, controllers, MoveIt, web backend and operator safety gates.

| Article | Summary | Updated |
|---------|---------|---------|
| [UR3e Control Stack](robot-control/ur3e-control-stack.md) | Real robot control architecture, process launch, controllers, frames and safety gates. | 2026-06-29 |

## live-catch

Closed-loop perception-to-policy-to-robot path.

| Article | Summary | Updated |
|---------|---------|---------|
| [Live Catch Loop](live-catch/live-catch-loop.md) | Single-process 60 Hz live-catch pipeline, modules, metadata-driven action mapping, safety and tests. | 2026-07-01 |
| [Message Contracts And Topics](live-catch/message-contracts-and-topics.md) | `BallState`, `CatchTelemetry`, topics, producers, base_link policy-frame telemetry, idle heartbeat/`ball_valid` contract and timestamp rules. | 2026-07-02 |
| [Safety And Commanding](live-catch/safety-and-commanding.md) | Command modes, model-switch gate, metadata/model limits, safety limiter, watchdog, controller switching, 500 Hz interpolated streaming, start-pose ±2π gate and hardware gates. | 2026-07-02 |
| [Current Status And Blockers](live-catch/current-status-and-blockers.md) | Working state, open robot/perception blockers, 2026-07-02 pendant incident diagnosis and current model/action status. | 2026-07-02 |

## sim-to-real

PPO transfer, policy semantics, action space and latency.

| Article | Summary | Updated |
|---------|---------|---------|
| [Policy Transfer And Action Semantics](sim-to-real/policy-transfer-and-action-semantics.md) | Legacy absolute vs current incremental target-integrator semantics, metadata-driven mapper selection and full-speed metadata limits vs bring-up slow-down. | 2026-07-02 |
| [Observation Latency And Models](sim-to-real/observation-latency-and-models.md) | Observation construction, latency instrumentation, current model exports, Web UI model selection and metadata validation. | 2026-07-01 |

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
| [UR3e Web UI](web-ui/ur3e-web-ui.md) | FastAPI/Three.js UI structure, base_link robot/viewer contract, Isaac hoop visual, Test tab, model selector, calibration tab, rollout tab and API scope. | 2026-07-02 |

## operations

Agent workflow, commands and wiki maintenance.

| Article | Summary | Updated |
|---------|---------|---------|
| [Testing And Commands](operations/testing-and-commands.md) | Build, launch, live-catch fake/real bring-up, package tests, Isaac sim2real export/check commands and wiki maintenance commands. | 2026-07-02 |
| [Wiki Maintenance](operations/wiki-maintenance.md) | Ingest/query/lint rules adapted from the Karpathy LLM wiki pattern. | 2026-06-29 |
| [Source Document Map](operations/source-document-map.md) | How the raw Markdown docs are combined or split into compiled wiki concepts. | 2026-06-29 |
