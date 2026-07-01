# UR3e Web UI

> Sources: web UI docs, 2026-06-30; robot control architecture, 2026-06-29; live-catch package README, 2026-06-29
> Raw: [Web UI docs](../../docs/Robot_Control/ur3e_web_ui.md); [Robot control architecture](../../docs/Robot_Control/ur3e_robot_control_architecture.md); [Live-catch README](../../src/ur3e_live_catch/README.md)

## Overview

`ur3e_web_ui` is the FastAPI/Three.js browser UI for robot status, control,
target validation, rollout replay, calibration and live-catch test interaction.

## Main Areas

- Backend: `src/ur3e_web_ui/ur3e_web_ui/app.py`.
- ROS bridge: `src/ur3e_web_ui/ur3e_web_ui/ros_interface.py`.
- Static UI: `src/ur3e_web_ui/ur3e_web_ui/static/`.
- Catch test panel: `static/js/catch_panel.js`.
- Rollout panel: `static/js/rollout_panel.js`.
- Calibration panel: `static/js/calibration_panel.js`.

## Live-Catch UI Role

The Test tab can launch a virtual ball, display the ball trajectory and policy
ghost, and toggle `live_catch_node` command mode through services. It is not on
the hot path; it is telemetry and operator control.

The Test tab also has an `Isaac random` throw path. It samples the current Isaac
ball distribution before calling the same virtual-ball services: spawn ranges
`x=(-0.6,-0.2)`, `y=(1.2,2.1)`, `z=(0.5,1.2)` with `0.01 m` Gaussian position
noise, and velocity ranges `vx=(-0.7,0.6)`, `vy=(-5.0,-3.5)`,
`vz=(-0.1,1.5)` m/s.

## See Also

- [UR3e Control Stack](../robot-control/ur3e-control-stack.md)
- [Live Catch Loop](../live-catch/live-catch-loop.md)
- [Rollout Replay And Driver Setup](../replay/rollout-replay-and-driver-setup.md)
