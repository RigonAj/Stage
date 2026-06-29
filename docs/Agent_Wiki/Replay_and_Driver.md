# Replay And Driver

## Scope

This domain covers open-loop rollout replay, UR driver setup and the difference
between replay validation and live closed-loop catch.

## Source Of Truth

- `docs/Robot_Control/ur3e_real_robot_replay.md`
- `docs/Robot_Control/ur3e_current_driver_setup.md`
- `docs/Robot_Control/ur3e_legacy_driver_setup.md`
- `docs/Robot_Control/ur3e_motion_issue_resolution.md`
- `docs/Robot_Control/ur3e_robot_control_architecture.md`

## Replay Semantics

- Prefer replaying realized simulation motion, not raw policy command targets.
- Raw targets can be aggressive and useful for diagnostics, but are not the
  faithful physical motion path.
- Replay is open-loop validation and does not solve live ball interception.
- Physical execution requires validation, preview and explicit operator
  confirmation.

## Driver Setup

- Current workflow: `docs/Robot_Control/ur3e_current_driver_setup.md`.
- Legacy workflow: `docs/Robot_Control/ur3e_legacy_driver_setup.md`, deprecated
  unless old PolyScope compatibility forces it.
- Motion troubleshooting history is in
  `docs/Robot_Control/ur3e_motion_issue_resolution.md`.

## Related Notes

- [[Robot_Control]]
- [[Web_UI]]
- [[UR3e_Live_Catch]]
