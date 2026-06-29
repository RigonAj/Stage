# Project Map

## System Shape

```text
DVXplorer events
  -> Ball_Tracking_Cpp
  -> ur3e_catch_msgs/BallState
  -> ur3e_live_catch
  -> UR3e command stream
  -> ur3e_web_ui telemetry and controls
```

The project has two major paths:

- Perception path: event camera to 3D ball position.
- Robot path: ball state plus joint state to policy action and safe joint target.

## Packages

- `src/Ball_Tracking_Cpp/`: C++ ROS 2 event-camera tracker. Main algorithm is
  Trace; circle fitting remains as an optional comparison mode.
- `src/ur3e_catch_msgs/`: typed ROS 2 messages, including `BallState` and
  catch telemetry.
- `src/ur3e_live_catch/`: Python ROS 2 node for the 60 Hz closed-loop catch
  pipeline.
- `src/ur3e_web_ui/`: browser UI for robot visualization, replay and catch test
  controls.
- `src/ur3e_rollout_replay/`: safe replay and validation of exported Isaac
  rollouts.
- `src/ur3e_sysid/`: UR3e actuator/system-identification tools.
- `src/ball_tracking/`: Python utilities around the ball tracking workspace.

## Documentation Map

- Global context: [[Source_Docs]], `docs/Context/synthese_projet.md`.
- Current execution state: [[Current_Status]], `docs/reste_a_faire.md`,
  `docs/incoherences_code_logique.md`.
- Perception: [[Perception_Trace]], `README.md`, `docs/Context/AGENT.md`.
- Calibration: [[Calibration]], `docs/Context/calibration_python_architecture.md`,
  `docs/Robot_Control/ur3e_camera_base_calibration.md`.
- Robot control and UI: [[Robot_Control]], [[Web_UI]],
  `docs/Robot_Control/ur3e_robot_control_architecture.md`.
- Live catch: [[UR3e_Live_Catch]],
  `docs/Robot_Control/ur3e_live_catch_implementation_status.md`.
- Sim-to-real: [[Sim_to_Real]],
  `docs/Robot_Control/ur3e_ball_catch_sim_to_real.md`.
- System identification: [[System_ID]],
  `docs/Robot_Control/ur3e_programme_identification_gains.md`.
- Replay and driver setup: [[Replay_and_Driver]],
  `docs/Robot_Control/ur3e_real_robot_replay.md`.

## High-Value Files

- `README.md`: current global project overview.
- `src/Ball_Tracking_Cpp/src/Gui.cpp`: Trace algorithm and visual diagnostics.
- `src/Ball_Tracking_Cpp/src/publisher_member_function.cpp`: tracker node loop
  and `BallState` publication.
- `src/Ball_Tracking_Cpp/src/Camera.cpp`: camera acquisition, filtering and
  undistortion.
- `src/ur3e_live_catch/ur3e_live_catch/live_catch_node.py`: live catch node.
- `src/ur3e_live_catch/ur3e_live_catch/observation.py`: policy observation.
- `src/ur3e_live_catch/ur3e_live_catch/action.py`: policy action mapping.
- `src/ur3e_live_catch/ur3e_live_catch/safety.py`: safety limiter and watchdog.
- `src/ur3e_web_ui/ur3e_web_ui/app.py`: FastAPI application.
- `src/ur3e_web_ui/ur3e_web_ui/static/js/`: frontend panels.

## Current Reading Strategy

This repository has many long design documents. For agent work, do not read all
of them by default. Use [[Source_Docs]] to pick the source-of-truth document for
the current task, then inspect the corresponding package files.

## Do Not Load By Default

- `build/`, `install/`, `log/`, `.deps/`, `.venv/`.
- `*.bin`, `*.h5`, `*.onnx`, `*.pdf`, `*.glb`, `*.step`, images.
- `src/Ball_Tracking_Cpp/include/Ball_Tracking_Cpp/raygui.h`.
