# Web UI

## Purpose

`ur3e_web_ui` provides the browser interface for UR3e visualization, jogging,
rollout replay and the live catch test controls.

## Read First

- [[Robot_Control]]
- `docs/Robot_Control/ur3e_web_ui.md`
- `src/ur3e_live_catch/README.md`
- `docs/Robot_Control/ur3e_robot_control_architecture.md`

## Main Code

- `src/ur3e_web_ui/ur3e_web_ui/app.py`: FastAPI app and routes.
- `src/ur3e_web_ui/ur3e_web_ui/ros_interface.py`: ROS bridge.
- `src/ur3e_web_ui/ur3e_web_ui/static/index.html`: single-page UI.
- `src/ur3e_web_ui/ur3e_web_ui/static/js/main.js`: app bootstrap.
- `src/ur3e_web_ui/ur3e_web_ui/static/js/catch_panel.js`: catch panel.
- `src/ur3e_web_ui/ur3e_web_ui/static/js/rollout_panel.js`: rollout panel.
- `src/ur3e_web_ui/ur3e_web_ui/static/js/calibration_panel.js`: calibration
  panel.
- `src/ur3e_web_ui/ur3e_web_ui/static/css/app.css`: UI styling.

## Tests

```bash
cd src/ur3e_web_ui
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

## Related Notes

- [[Robot_Control]]
- [[UR3e_Live_Catch]]
- [[Replay_and_Driver]]
