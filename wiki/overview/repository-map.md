# Repository Map

> Sources: Repository README, 2026-06-29; Project synthesis, 2026-06-29; package manifests, 2026-06-29
> Raw: [README](../../README.md); [Synthese projet](../../docs/Context/synthese_projet.md); [Inventory](../../docs/Agent_Wiki/Inventory.md)

## Overview

The workspace is organized as a ROS 2 project with C++ perception, Python robot
control packages, a browser UI, messages and support scripts. Load context by
package and task, not by scanning the full tree.

## Package Responsibilities

- `src/Ball_Tracking_Cpp/`: event-camera ball tracking node and GUI.
- `src/ball_tracking/`: Python utility package for the tracking workspace.
- `src/ur3e_catch_msgs/`: `BallState` and `CatchTelemetry` message contract.
- `src/ur3e_live_catch/`: live closed-loop catch node, fake ball source, policy
  runtime, safety, streaming and latency.
- `src/ur3e_rollout_replay/`: validate and replay Isaac Lab rollouts.
- `src/ur3e_sysid/`: actuator/system-identification tools.
- `src/ur3e_web_ui/`: FastAPI backend and static browser UI.

## High-Value Files

- `README.md`: perception overview and build/run basics.
- `docs/Context/synthese_projet.md`: broad project synthesis.
- `docs/reste_a_faire.md`: current execution checklist.
- `docs/incoherences_code_logique.md`: open inconsistencies and stale docs.
- `src/ur3e_live_catch/README.md`: live-catch package map.
- `docs/Robot_Control/ur3e_robot_control_architecture.md`: robot/UI architecture.

## Context Boundaries

Avoid generated folders (`build/`, `install/`, `log/`, `.deps/`), binary data,
models, PDFs and large external headers unless the task explicitly requires
them. For C++ GUI work, avoid reading `raygui.h` unless the issue is directly in
raygui integration.

## See Also

- [Project Overview](project-overview.md)
- [Testing And Commands](../operations/testing-and-commands.md)
