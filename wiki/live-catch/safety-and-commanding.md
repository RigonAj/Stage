# Safety And Commanding

> Sources: live-catch architecture, 2026-06-30; implementation status, 2026-06-30; remaining work checklist, 2026-06-29; robot control architecture, 2026-06-29
> Raw: [Live-catch architecture](../../docs/Robot_Control/ur3e_live_catch_architecture.md); [Implementation status](../../docs/Robot_Control/ur3e_live_catch_implementation_status.md); [Reste a faire](../../docs/reste_a_faire.md); [Robot control architecture](../../docs/Robot_Control/ur3e_robot_control_architecture.md)

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

## Mapping And Limits

- `ActionMapper` translates policy output into a joint target according to the
  model metadata/action mode. Current Isaac exports use the incremental target
  integrator; legacy exports use the absolute `action * 0.5` contract.
- `SafetyLimiter` applies position, velocity and acceleration constraints
  independently from the policy. With current exports, bounds come from
  `policy_metadata.json`; otherwise they fall back to URDF/config limits.
- `Watchdog` handles stale perception, budget overruns and tracking errors.
- `CommandStreamer` publishes to `/forward_position_controller/commands`.

## Operator Gates

The real robot path is intentionally layered:

1. Reduced robot speed and E-stop readiness.
2. External Control / UR driver state checked.
3. Controller availability checked.
4. UI or launch explicitly enables command mode.
5. Watchdog can return to a safe hold/stop path.

## Current Hardware Work

Still open from the docs:

- validate command streaming with virtual ball on real UR3e;
- test watchdog behavior on hardware;
- tune `a_safe`, `loop_budget_s` and `max_tracking_error`;
- verify controller restoration after stop.

## See Also

- [Live Catch Loop](live-catch-loop.md)
- [Current Status And Blockers](current-status-and-blockers.md)
- [UR3e Control Stack](../robot-control/ur3e-control-stack.md)
