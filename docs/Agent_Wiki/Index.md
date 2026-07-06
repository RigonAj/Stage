# Agent Wiki Index

This folder is a Markdown/Obsidian-compatible wiki for humans and coding agents.
Its job is to make the repository explorable without loading the entire codebase
into context.

Primary Karpathy-style wiki:

- `wiki/index.md`
- `wiki/log.md`
- `raw/`

`docs/Agent_Wiki/` is a secondary navigation layer kept for compatibility with
the earlier organization. Agents should use `wiki/index.md` first.

## Start Here

- `wiki/index.md`: primary compiled knowledge index.
- `wiki/live-catch/current-status-and-blockers.md`: current blockers and bring-up
  order.
- [[Project_Map]]: package map and data flow.
- [[Source_Docs]]: curated map of the Markdown documentation.
- [[Current_Status]]: pointer to the primary wiki status page (content moved
  to `wiki/live-catch/current-status-and-blockers.md`).
- [[Inventory]]: generated list of packages and Markdown docs.
- [[Testing_and_Commands]]: build, launch and test commands.
- [[Agent_Workflow]]: how an agent should use and maintain this wiki.

## Domain Notes

- [[Perception_Trace]]: DVXplorer event tracking, Trace algorithm and C++ entry
  points.
- [[UR3e_Live_Catch]]: live closed-loop robot catch pipeline.
- [[Calibration]]: camera intrinsics and camera-to-robot hand-eye calibration.
- [[Robot_Control]]: UR driver, controllers, MoveIt, web backend and operator
  gates.
- [[Sim_to_Real]]: PPO transfer, action semantics, latency and policy export.
- [[System_ID]]: UR3e actuator gain identification and measured values.
- [[Replay_and_Driver]]: open-loop rollout replay and UR driver setup.
- [[Web_UI]]: FastAPI/Three.js robot interface and test controls.

## Task Routing

- Need a broad overview: read [[Project_Map]], then `docs/Context/synthese_projet.md`.
- Need to edit perception: read [[Perception_Trace]], then `README.md`.
- Need to bring up live catch: read
  `wiki/live-catch/current-status-and-blockers.md` and
  `wiki/operations/real-robot-bringup-runbook.md`, then [[UR3e_Live_Catch]].
- Need calibration: read [[Calibration]], then
  `docs/Robot_Control/ur3e_camera_base_calibration.md`.
- Need robot motion/UI: read [[Robot_Control]], [[Web_UI]], then
  `docs/Robot_Control/ur3e_robot_control_architecture.md`.
- Need sim-to-real or PPO policy behavior: read [[Sim_to_Real]], then
  `docs/Robot_Control/ur3e_sim2real_propositions.md`.
- Need system-id: read [[System_ID]], then
  `docs/Robot_Control/ur3e_programme_identification_gains.md`.

## Existing Source Documents

The wiki intentionally links to existing documentation instead of copying it:

- `README.md`
- `docs/Context/synthese_projet.md`
- `docs/Context/AGENT.md`
- `docs/Robot_Control/ur3e_live_catch_architecture.md`
- `docs/Robot_Control/ur3e_live_catch_implementation_status.md`
- `docs/Robot_Control/ur3e_camera_base_calibration.md`
- `src/ur3e_live_catch/README.md`
