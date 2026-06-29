# Rollout Replay And Driver Setup

> Sources: real robot replay guide, 2026-06-29; current driver setup, 2026-06-29; legacy driver setup, 2026-06-29; motion issue history, 2026-06-29
> Raw: [Replay guide](../../docs/Robot_Control/ur3e_real_robot_replay.md); [Current driver setup](../../docs/Robot_Control/ur3e_current_driver_setup.md); [Legacy driver setup](../../docs/Robot_Control/ur3e_legacy_driver_setup.md); [Motion issue resolution](../../docs/Robot_Control/ur3e_motion_issue_resolution.md)

## Overview

Rollout replay is the safe open-loop validation path for Isaac trajectories. It
is related to, but not a replacement for, live closed-loop catch.

## Replay Rule

Prefer replaying realized simulation motion rather than raw policy command
targets. The raw target series can be aggressive and useful for diagnostics, but
it is not the physically reached motion.

## Driver Setup

- Current recommended path: `ur3e_current_driver_setup.md`.
- Legacy path: `ur3e_legacy_driver_setup.md`, only for old PolyScope
  compatibility.
- Motion issue history: `ur3e_motion_issue_resolution.md`.

## See Also

- [UR3e Control Stack](../robot-control/ur3e-control-stack.md)
- [UR3e Web UI](../web-ui/ur3e-web-ui.md)
- [Live Catch Loop](../live-catch/live-catch-loop.md)
