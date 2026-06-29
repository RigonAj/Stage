# Robot Control

## Scope

This domain covers the UR ROS 2 driver, controller selection, MoveIt IK, the
FastAPI web backend, operator gates and real robot motion safety.

## Source Of Truth

- `docs/Robot_Control/ur3e_robot_control_architecture.md`
- `docs/Robot_Control/ur3e_web_ui.md`
- `docs/Robot_Control/ur3e_current_driver_setup.md`
- `docs/Robot_Control/ur3e_motion_issue_resolution.md`

Use `docs/Robot_Control/ur3e_legacy_driver_setup.md` only for an old PolyScope
or legacy driver path. The current workflow points to the current Humble driver.

## Runtime Shape

```text
Browser UI
  -> FastAPI backend (`ur3e_web_ui`)
  -> rclpy ROS bridge
  -> UR ROS 2 driver + MoveIt + controllers
  -> physical UR3e
```

The backend does not send Cartesian commands directly to the UR controller.
Physical moves are converted into joint-space trajectories and sent through the
scaled trajectory controller, except the live-catch streaming path which uses
`forward_position_controller`.

## Main Files

- `src/ur3e_web_ui/ur3e_web_ui/app.py`
- `src/ur3e_web_ui/ur3e_web_ui/ros_interface.py`
- `src/ur3e_web_ui/ur3e_web_ui/motion.py`
- `src/ur3e_web_ui/ur3e_web_ui/joint_limits.py`
- `src/ur3e_web_ui/ur3e_web_ui/urdf_provider.py`
- `scripts/launch_ur3e_stack.sh`
- `scripts/launch_ur3e_virtual_ball_stack.sh`

## Safety Rules

- Keep reduced speed and E-stop readiness for real robot tests.
- Do not bypass UI confirmation gates for replay or TCP target execution.
- Treat External Control / speed scaling state as an operator pre-check.
- For live catch, command emission must remain behind `enable_command`.

## Related Notes

- [[Web_UI]]
- [[UR3e_Live_Catch]]
- [[Replay_and_Driver]]
- [[Current_Status]]
