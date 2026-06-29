# Testing and Commands

## Environment

```bash
source env.sh
```

## Build

```bash
build
```

Targeted ROS build examples:

```bash
colcon build --packages-select ur3e_catch_msgs ur3e_live_catch
colcon build --packages-select ball_tracking_cpp
```

## Run

```bash
run
ur3e_stack
ur3e_catch_stack
```

## Tests

```bash
cd src/ur3e_live_catch && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
cd src/ur3e_rollout_replay && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
cd src/ur3e_web_ui && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
cd src/ur3e_sysid && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

## Agent Wiki Maintenance

```bash
python3 scripts/lint_llm_wiki.py
python3 scripts/update_agent_wiki.py
```

## Reading Before Running

- Real robot bring-up: [[Current_Status]], then `docs/reste_a_faire.md`.
- Driver/UI: [[Robot_Control]], then `docs/Robot_Control/ur3e_web_ui.md`.
- Live catch: [[UR3e_Live_Catch]], then `src/ur3e_live_catch/README.md`.
- Calibration: [[Calibration]], then
  `docs/Robot_Control/ur3e_camera_base_calibration.md`.
