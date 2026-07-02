# Source Docs

This note organizes the Markdown documentation already present in the repository.
The files listed here remain the source of truth; the vault is the navigation
layer.

Primary compiled wiki pages now live in `wiki/`. Use this note only to locate
repository source documents behind those compiled pages.

## Global Context

- `README.md`: current high-level project README focused on event-based 3D ball
  tracking and the Trace algorithm.
- `docs/Context/synthese_projet.md`: best broad synthesis of the full project:
  perception, calibration, UR3e control, rollout replay, PPO and current state.
- `docs/Context/AGENT.md`: older tracker-specific agent note. Useful for C++
  perception context, but superseded globally by `AGENTS.md`.
- `docs/latex_compilation.md`: report build commands and LaTeX dependencies.

## Current State And Risks

- `docs/reste_a_faire.md`: execution checklist for moving from dry-run live
  catch to real robot and real perception.
- `docs/incoherences_code_logique.md`: resolved and still-open code/docs/logic
  inconsistencies. Read before changing live-catch, calibration, model paths or
  system-id packaging.

## Perception

- `README.md`: Trace pipeline, circle fitting fallback, main C++ files and
  build/run commands.
- `docs/Context/synthese_projet.md`: perception overview and relation to robot
  interception.
- `docs/Context/AGENT.md`: C++ tracker file map and context-loading exclusions.

Non-Markdown but important:

- `docs/trace_algorithm_explanation.html`
- `docs/Context/algo_trace_graph.html`
- `docs/Context/algo_circle_fitting_graph.html`

## Calibration

- `docs/Context/calibration_python_architecture.md`: Python intrinsics
  calibration scripts, event mire pipeline, exports and commands.
- `docs/Robot_Control/ur3e_camera_base_calibration.md`: eye-to-hand reference
  document for `T_base_camera`, phone mire, OpenCV conventions and validation.
- `docs/Robot_Control/ur3e_motion_issue_resolution.md`: useful frame/control
  history and verification commands around real robot motion.

## Robot Control

- `docs/Robot_Control/ur3e_robot_control_architecture.md`: source-of-truth
  architecture for UR driver, FastAPI backend, ROS bridge, UI tabs, frames,
  data flows, safety gates and debug commands.
- `docs/Robot_Control/ur3e_web_ui.md`: operator-facing web UI setup and usage.
- `docs/Robot_Control/ur3e_current_driver_setup.md`: current recommended UR ROS
  2 Humble driver setup.
- `docs/Robot_Control/ur3e_legacy_driver_setup.md`: deprecated legacy driver
  setup; read only if an old PolyScope path is explicitly needed.
- `docs/Robot_Control/ur3e_motion_issue_resolution.md`: history of motion
  problems, root causes and current working procedure.

## Live Catch

- `docs/Robot_Control/ur3e_live_catch_architecture.md`: design reference for
  the mono-process 60 Hz live-catch loop.
- `docs/Robot_Control/ur3e_live_catch_implementation_status.md`: implementation
  state, verified tests, real-robot virtual-ball status, tuning gaps and launch
  commands.
- `src/ur3e_live_catch/README.md`: package-level module map and run commands.
- `src/ur3e_catch_msgs/README.md`: package README for messages. Note:
  `docs/incoherences_code_logique.md` says this README is obsolete and should
  be updated to match the implemented `BallState` / `CatchTelemetry` contract.

## Sim-To-Real And Policy

- `docs/Robot_Control/ur3e_ball_catch_sim_to_real.md`: plan and constraints for
  PPO transfer, simulation changes, observation, action semantics and latency.
- `docs/Robot_Control/ur3e_choix_espace_action_isaac.md`: decision document on
  position, velocity and acceleration action spaces.
- `docs/Robot_Control/ur3e_sim2real_propositions.md`: review findings and
  prioritized proposals for better transfer.
- `docs/Robot_Control/ur3e_parametres_actionneur_reference.md`: UR3e limits and
  actuator parameter references.
- `data/models/README.md`: intended canonical location and naming convention for
  the live-catch model files.

## System Identification

- `docs/Robot_Control/ur3e_programme_identification_gains.md`: design and
  safety procedure for identifying actuator gains.
- `docs/Robot_Control/ur3e_resultats_identification_gains.md`: measured
  stiffness/damping results, validation and Isaac Lab connection.
- `src/ur3e_sysid/`: local package listed by the docs. `git status` currently
  shows it as untracked, so treat packaging/versioning as an explicit decision.

## Replay

- `docs/Robot_Control/ur3e_real_robot_replay.md`: safe open-loop rollout replay,
  realized-vs-target semantics and physical robot gates.
- `docs/Robot_Control/ur3e_robot_control_architecture.md`: rollout tab,
  validation, preview and execution data flow.
