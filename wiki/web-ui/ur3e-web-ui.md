# UR3e Web UI

> Sources: web UI docs, 2026-07-01; robot control architecture, 2026-06-29; live-catch package README, 2026-07-01; v_safe_scale UI implementation, 2026-07-03; v_safe_scale overdrive range to 4.0, 2026-07-03
> Raw: [Web UI docs](../../docs/Robot_Control/ur3e_web_ui.md); [Robot control architecture](../../docs/Robot_Control/ur3e_robot_control_architecture.md); [Live-catch README](../../src/ur3e_live_catch/README.md); [Web UI app](../../src/ur3e_web_ui/ur3e_web_ui/app.py); [Catch panel](../../src/ur3e_web_ui/ur3e_web_ui/static/js/catch_panel.js); [Live catch node](../../src/ur3e_live_catch/ur3e_live_catch/live_catch_node.py)

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
ghost, select the active `latest` or `best` model from `data/models/`, tune
`live_catch_node.v_safe_scale`, and toggle `live_catch_node` command mode
through services. It is not on the hot path; it is telemetry and operator
control.

The `v_safe_scale` control reads/writes `/live_catch_node` parameters through
`/api/catch/v_safe_scale`. It exposes the staged values `0.5`, `0.7`, `0.85`,
`1.0`, `1.25`, `1.5`, `2.0`, `2.5`, `3.0` and `4.0`, plus a numeric input
constrained to `(0, 4]`.
Values above `1.0` are labeled as overdrive tests because they exceed the trained
metadata contract. The control is disabled while command mode is active, the
FastAPI endpoint rejects changes when telemetry reports `command_enabled=true`,
and `live_catch_node` also rejects runtime `v_safe_scale` parameter changes while
command mode is enabled. Accepted changes rebuild the live node's mapper, safety
bounds, streamer and dry-run simulation state explicitly, so operators no longer
need to edit `src/ur3e_live_catch/config/live_catch.yaml` between dry-run trials.

Model selection is deliberately narrow: the backend exposes only
`data/models/latest` and `data/models/best`, prefers ONNX over TorchScript, and
sends the selected path to `/live_catch_node` as the `model_path` parameter. The
live node loads and validates the new policy before replacing the active policy;
if ONNX cannot load because `onnxruntime` is missing, it tries the sibling
TorchScript export. Model changes are rejected while command mode is active.

The Test tab's launch frame, ball telemetry and predicted arcs now use the
policy frame `base_link`, not the UR `base` frame used by the TCP target panel.
Its `Isaac random` throw path samples the current FirstTraining ball
distribution before calling the same virtual-ball services: spawn ranges
`x=(-0.6,-0.2)`, `y=(1.2,2.1)`, `z=(0.5,1.2)` with `0.01 m` Gaussian
position noise, and velocity ranges `vx=(-0.7,0.6)`, `vy=(-5.0,-3.5)`,
`vz=(-0.1,1.5)` m/s. The visible defaults are the midpoints of those ranges,
with `gravity=(0,0,-9.81) m/s^2` and `flight_s=4.0`; the API carries `p0`,
`v0`, `gravity` and `flight_s` to `test_ball_node`.

The 3D robot and all robot ghosts are anchored in the URDF root `base_link`,
matching Isaac FirstTraining's identity robot frame. `ur_description` exposes
the teach-pendant `base` frame as a fixed child of `base_link` rotated pi about
Z, so the viewer applies only the ROS Z-up to Three.js Y-up display transform to
the robot root. TCP target and camera widgets still accept/display `base` poses,
then convert them through the explicit `base -> base_link` rotation; live-catch
ball overlays use the separate `base_link -> Three.js` conversion.

The viewer adds an Isaac-style hoop visual to the live robot and robot ghosts.
It is a primitive torus and support rod attached to `wrist_3_link`, centered at
`(-0.5, 0, 0)` m with disk normal `(0, 0, -1)`. The displayed hoop radius is
`0.15 m`; the smaller `0.05 m` ring is only the policy/pass-through validation
radius, not the physical hoop size.

## See Also

- [UR3e Control Stack](../robot-control/ur3e-control-stack.md)
- [Live Catch Loop](../live-catch/live-catch-loop.md)
- [Rollout Replay And Driver Setup](../replay/rollout-replay-and-driver-setup.md)
