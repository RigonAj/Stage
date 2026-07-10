# Safety And Commanding

> Sources: live-catch architecture, 2026-06-30; implementation status, 2026-06-30; live-catch README, 2026-07-01; remaining work checklist, 2026-06-29; robot control architecture, 2026-06-29; 2026-07-02 pendant incident analysis; user hardware report, 2026-07-02; v_safe_scale UI implementation, 2026-07-03; v_safe_scale overdrive range to 4.0, 2026-07-03; producer-conflict fail-closed gate, 2026-07-09
> Raw: [Live-catch architecture](../../docs/Robot_Control/ur3e_live_catch_architecture.md); [Implementation status](../../docs/Robot_Control/ur3e_live_catch_implementation_status.md); [Reste a faire](../../docs/reste_a_faire.md); [Robot control architecture](../../docs/Robot_Control/ur3e_robot_control_architecture.md); [Live catch node](../../src/ur3e_live_catch/ur3e_live_catch/live_catch_node.py); [Web UI app](../../src/ur3e_web_ui/ur3e_web_ui/app.py)

## Overview

This page separates safety and command emission from the rest of the live-catch
architecture. Read it before editing `action.py`, `safety.py`, `streaming.py`,
controller switching or UI command gates.

## Command Modes

- `enable_command=false`: dry-run. The node computes observations, policy
  actions, safety-limited targets and telemetry, but emits no robot command.
- `enable_command=true`: command mode. The node may switch to
  `forward_position_controller` and stream safe joint targets.
- Command mode must fail closed when no policy is loaded.
- Runtime `model_path` and `v_safe_scale` changes are accepted only while command
  mode is off. The live node loads and validates a new policy before replacing
  the active policy; accepted `model_path` or `v_safe_scale` changes rebuild
  mapper/safety state before the parameter request succeeds.

## Mapping And Limits

- `ActionMapper` translates policy output into a joint target according to the
  model metadata/action mode. Current Isaac exports use the incremental target
  integrator; legacy exports use the absolute `action * 0.5` contract.
- `SafetyLimiter` applies position, velocity and acceleration constraints
  independently from the policy. With current exports, bounds come from
  `policy_metadata.json`; otherwise they fall back to URDF/config limits.
  The fallback acceleration default is `12.5664 rad/s^2`; current Isaac exports
  still override this with per-joint metadata.
- `Watchdog` handles stale perception, budget overruns and tracking errors.
- `CommandStreamer` publishes to `/forward_position_controller/commands`.
- `v_safe_scale` (default 1.0; bring-up config 0.5) scales metadata
  `v_safe`/`a_safe` for both mapper and safety. `1.0` is the trained contract;
  below `1.0` is a deliberate slow-down, and above `1.0` is lab-test overdrive
  that diverges from training and can exceed metadata/UR nominal limits.
  The 2026-07-02 real-UR3e virtual-ball validation worked under this conservative
  setting but was visibly slow, so speed tuning is now an optimization task. The
  Web UI Test tab can change the scale while command mode is off, with staged
  buttons `0.5 -> 0.7 -> 0.85 -> 1.0 -> 1.25 -> 1.5 -> 2.0 -> 2.5 -> 3.0 -> 4.0`.
- Standard bring-up (`ur3e_catch_stack --real` /
  `virtual_ball_robot.launch.py` -> `live_catch.launch.py`) loads
  `config/live_catch.yaml`; because `v_safe_scale=0.5` lives in that config and
  is not overridden at launch, it applies to real-robot command tests launched
  through the standard stack until the operator changes it in the Test tab. If
  the node is launched manually without that config, the code default is 1.0.
- In command mode, missing `base_link -> hoop_center` TF is fail-closed: the
  node does not use the disk fallback for robot motion.

## Streaming Rate And The UR Driver Velocity Check

The UR driver validates consecutive servoj set-points every 2 ms against the
joint velocity limits. A raw 60 Hz safe target step (`v_safe * dt`, ~0.105 rad
on wrists at full metadata speed) lands in one 2 ms frame and is rejected with
`Velocity ... exceeding the joint velocity limits`, after which the driver
latches `Ignoring commands until a valid command is received`. The node
therefore runs a high-rate command timer (`command_rate_hz`, default 500 Hz)
that walks the published command toward the current 60 Hz safe target in
per-frame steps capped at `v_safe / command_rate_hz` (`streaming.step_toward`).
`command_substeps` burst upsampling is legacy and only used when
`command_rate_hz <= loop_hz`.

## Producer-Conflict Gate (Exclusive Ball Source)

Since 2026-07-09 the node checks every 2 s that it is the only
`live_catch_node` and that its ball topic has a single publisher
(`diagnostics.producer_conflict_warnings`). Multiple `ball_state` publishers
fail command emission **closed**: interleaved `valid=false` heartbeats from a
second producer (typically the stack's idle `test_ball_node` next to the real
tracker) trigger a controlled stop plus a policy-state reset on every message,
which reads as a twitching robot while looking "armed" to the operator
(2026-07-09 incident). Telemetry/duplicate-name conflicts log `PRODUCER
CONFLICT` errors without blocking, since the other node may be the commanding
one. See [Single Producer Contract](single-producer-contract.md).

## Start-Pose Gate (±2π Branch Protection)

Before the first command of a command session, the node refuses to stream when
any measured joint exceeds `start_pose_limit_rad` (default 3.0 rad,
`safety.start_pose_violations`). This catches `/joint_states` reporting a joint
on a ±2π-wrapped branch (e.g. wrist_3 at 6.2832 ~ "360°" while the UR
controller internally sits at ~0°): echoing that pose back looks like a
full-turn jump to the driver, which rejects it and then ignores all commands
(the 2026-07-02 incident). The gate re-checks every tick and arms streaming
once the operator jogs/unwinds the joint (or reboots the arm) so
`/joint_states` matches the pendant. Each `enable_command` toggle also resets
`CommandStreamer` so a hold can never replay a pose recorded before the robot
was moved by other means.

## Operator Gates

The real robot path is intentionally layered:

1. Reduced robot speed and E-stop readiness.
2. External Control / UR driver state checked.
3. Controller availability checked.
4. Policy model and `v_safe_scale` changes are completed in dry-run before
   command mode.
5. UI or launch explicitly enables command mode.
6. Watchdog can return to a safe hold/stop path.

At `v_safe_scale=1.0`, the current policy may drive every joint at the metadata
limits (UR3e hard velocity limits for the current export). Above `1.0`, the
command path intentionally asks for more than those metadata limits; expect UR
driver velocity rejections, protective stops or reduced-mode limiting during
limit tests. Use a clear workspace, keep the E-stop in hand, verify pendant
safety limits/reduced mode, and step through virtual-ball trials before
commanding real throws.

## Current Hardware Work

The virtual-ball command path has been reported working on the real UR3e after
the 2026-07-02 heartbeat/grounding fixes. Still open:

- test watchdog behavior on hardware;
- tune `v_safe_scale`, `a_safe`, `loop_budget_s`, `max_tracking_error` and
  `start_pose_limit_rad` for faster but still bounded behavior;
- verify controller restoration after stop.

## See Also

- [Real Robot Bring-Up Runbook](../operations/real-robot-bringup-runbook.md)
- [Live Catch Loop](live-catch-loop.md)
- [Current Status And Blockers](current-status-and-blockers.md)
- [Single Producer Contract](single-producer-contract.md)
- [UR3e Control Stack](../robot-control/ur3e-control-stack.md)
