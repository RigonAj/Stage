# Project Overview

> Sources: Repository README, 2026-06-29; Project synthesis, 2026-06-29
> Raw: [README](../../README.md); [Synthese projet](../../docs/Context/synthese_projet.md)

## Overview

This repository is a ROS 2 workspace for detecting a fast ball with a DVXplorer
event camera, estimating its 3D position, and making that signal usable by a UR3e
robot for interception. The project joins four concerns: event-camera
perception, camera/robot calibration, UR3e control and PPO sim-to-real transfer.

## System Flow

```text
DVXplorer events
  -> event filtering and undistortion
  -> Trace or circle-fitting 3D ball estimate
  -> ur3e_catch_msgs/BallState
  -> ball frame transform into UR3e base
  -> 33-D policy observation
  -> policy action
  -> safety/rate limiting
  -> UR3e command stream
```

## Main Project Areas

- Perception: `src/Ball_Tracking_Cpp/`.
- Message contract: `src/ur3e_catch_msgs/`.
- Live closed-loop catch: `src/ur3e_live_catch/`.
- Browser UI and robot bridge: `src/ur3e_web_ui/`.
- Rollout replay: `src/ur3e_rollout_replay/`.
- System identification: `src/ur3e_sysid/`.
- Calibration and launch helpers: `scripts/`.
- Technical docs: `docs/`.

## Current Interpretation

The perception stack is more mature than the physical catch deployment. The
live-catch software path is implemented and dry-run tested, but the real robot
bring-up still depends on calibration, static TFs, hardware watchdog testing and
latency measurement.

## See Also

- [Repository Map](repository-map.md)
- [Trace Ball Tracking](../perception/trace-ball-tracking.md)
- [Live Catch Loop](../live-catch/live-catch-loop.md)
- [Current Status And Blockers](../live-catch/current-status-and-blockers.md)
