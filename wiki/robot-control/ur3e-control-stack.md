# UR3e Control Stack

> Sources: robot control architecture, 2026-06-29; web UI docs, 2026-06-29; motion issue history, 2026-06-29
> Raw: [Robot control architecture](../../docs/Robot_Control/ur3e_robot_control_architecture.md); [Web UI docs](../../docs/Robot_Control/ur3e_web_ui.md); [Motion issue resolution](../../docs/Robot_Control/ur3e_motion_issue_resolution.md)

## Overview

The UR3e control side is a ROS 2 stack wrapped by a FastAPI/Three.js web UI. The
normal robot-control path sends joint trajectories through the scaled trajectory
controller. The live-catch path is separate and streams through
`forward_position_controller`.

## Runtime Shape

```text
Browser UI
  -> FastAPI backend
  -> rclpy ROS bridge
  -> UR ROS 2 driver + MoveIt + controllers
  -> physical UR3e
```

## Main Files

- `scripts/launch_ur3e_stack.sh`
- `scripts/launch_ur3e_virtual_ball_stack.sh`
- `src/ur3e_web_ui/ur3e_web_ui/app.py`
- `src/ur3e_web_ui/ur3e_web_ui/ros_interface.py`
- `src/ur3e_web_ui/ur3e_web_ui/motion.py`
- `src/ur3e_web_ui/ur3e_web_ui/joint_limits.py`

## Safety Gates

The docs repeatedly treat real robot motion as gated by reduced speed, External
Control state, controller state, validation/preview, explicit UI confirmation and
operator E-stop readiness. Do not remove these gates as cleanup.

## See Also

- [UR3e Web UI](../web-ui/ur3e-web-ui.md)
- [Live Catch Loop](../live-catch/live-catch-loop.md)
- [Rollout Replay And Driver Setup](../replay/rollout-replay-and-driver-setup.md)
