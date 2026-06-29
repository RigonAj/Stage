# Source Document Map

> Sources: repository Markdown docs, 2026-06-29; docs inventory, 2026-06-29; local wiki organization, 2026-06-29
> Raw: [Inventory](../../docs/Agent_Wiki/Inventory.md); [Synthese projet](../../docs/Context/synthese_projet.md); [Incoherences](../../docs/incoherences_code_logique.md); [Reste a faire](../../docs/reste_a_faire.md)

## Overview

This page explains how the original Markdown files should be mentally combined
or separated. The raw docs remain untouched; the compiled wiki is the organized
view.

## Canonical Source Groups

| Knowledge area | Primary raw docs | Compiled wiki pages |
|---|---|---|
| Global project | `README.md`, `docs/Context/synthese_projet.md` | [Project Overview](../overview/project-overview.md), [Repository Map](../overview/repository-map.md) |
| Perception | `README.md`, `docs/Context/AGENT.md` | [Trace Ball Tracking](../perception/trace-ball-tracking.md) |
| Calibration procedure | `docs/Context/calibration_python_architecture.md`, `docs/Robot_Control/ur3e_camera_base_calibration.md` | [Camera And Hand-Eye Calibration](../calibration/camera-and-handeye-calibration.md) |
| Frames/TF/units | calibration, live-catch and robot-control docs | [Frames And Transforms](../calibration/frames-and-transforms.md) |
| Robot control | `ur3e_robot_control_architecture.md`, `ur3e_web_ui.md`, driver setup docs | [UR3e Control Stack](../robot-control/ur3e-control-stack.md), [UR3e Web UI](../web-ui/ur3e-web-ui.md) |
| Live catch | architecture, status, package README | [Live Catch Loop](../live-catch/live-catch-loop.md), [Message Contracts And Topics](../live-catch/message-contracts-and-topics.md), [Safety And Commanding](../live-catch/safety-and-commanding.md) |
| Current execution state | `docs/reste_a_faire.md`, `docs/incoherences_code_logique.md`, implementation status | [Current Status And Blockers](../live-catch/current-status-and-blockers.md) |
| Sim-to-real | sim-to-real plan, action-space decision, proposals, model README | [Policy Transfer And Action Semantics](../sim-to-real/policy-transfer-and-action-semantics.md), [Observation Latency And Models](../sim-to-real/observation-latency-and-models.md) |
| System-id | program, results, actuator reference | [UR3e Actuator Identification](../system-id/ur3e-actuator-identification.md) |
| Replay/driver | replay guide, current/legacy setup, motion issue history | [Rollout Replay And Driver Setup](../replay/rollout-replay-and-driver-setup.md) |

## Split/Merge Rule

- Split a page when the knowledge changes on a different cadence. Example:
  architecture is stable, current blockers change often.
- Merge source docs when they describe the same operational decision. Example:
  sim-to-real action-space docs and policy metadata belong in one action
  semantics page.
- Keep source docs as provenance; do not delete or rewrite them during wiki
  maintenance.

## See Also

- [Wiki Maintenance](wiki-maintenance.md)
- [Testing And Commands](testing-and-commands.md)
