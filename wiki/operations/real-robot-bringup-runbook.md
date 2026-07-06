# Real Robot Bring-Up Runbook

> Sources: 2026-07-02 pendant incident analysis; user hardware report, 2026-07-02; v_safe_scale Test-tab implementation, 2026-07-03; agent review, 2026-07-02
> Raw: [Live catch config](../../src/ur3e_live_catch/config/live_catch.yaml); [Live catch node](../../src/ur3e_live_catch/ur3e_live_catch/live_catch_node.py); [Web UI app](../../src/ur3e_web_ui/ur3e_web_ui/app.py); [Agent review](../../raw/reviews/2026-07-02-stage-wiki-and-training-review.md)

## Overview

Durable operator procedure for enabling real-UR3e command mode with the
live-catch stack. The churning state of the bring-up lives in
[Current Status And Blockers](../live-catch/current-status-and-blockers.md);
the safety design rationale lives in
[Safety And Commanding](../live-catch/safety-and-commanding.md). This page is
the checklist to follow at the robot.

## Pre-Command Checklist

Run through all of these **before** toggling `enable_command`:

1. **Stack up in dry-run.** `UR3E_ROBOT_IP=... UR3E_REVERSE_IP=...
   ur3e_catch_stack --real` starts with `enable_command=false` and
   `v_safe_scale=0.5` from `config/live_catch.yaml`.
2. **UR driver / External Control running.** Pendant program playing, no
   protective stop, robot in normal (not reduced) mode unless intended.
3. **Controller active.** `scaled_joint_trajectory_controller` active at rest;
   the node auto-switches to `forward_position_controller` when command mode
   starts (`auto_switch_controller=true`).
4. **`/joint_states` coherent with the pendant.** Compare each joint on the
   pendant Move screen against `ros2 topic echo /joint_states --once`. A joint
   reading ±2π away from the pendant value (e.g. 6.28 vs 0.00) will trip the
   start-pose gate — fix it first (next section).
5. **Hoop TF present.** `base_link -> hoop_center` must resolve; command mode
   fails closed without it (no disk-fallback motion).
6. **Policy loaded and telemetry alive.** Web UI Test tab shows the model, and
   `catch_telemetry` heartbeats arrive (`ball_valid=false` while idle).
7. **Model and `v_safe_scale` final.** Both can only change while command mode
   is off; pick them now.
8. **Physical safety.** Workspace clear of people and cables in the hoop swing
   volume, E-stop in hand, first sessions at reduced speed.

Then enable command mode from the Web UI Test tab (or launch option) and step
through **virtual** balls before any real throw.

## Start-Pose Gate Procedure (±2π branch)

Symptom: command mode enables but the node refuses to stream, logging
`start_pose_violations`; or (before the gate existed) the pendant reports
`Velocity ... required in joint N ... Ignoring commands until a valid command
is received` (2026-07-02 incident: wrist_3 at 6.28321 rad on a +2π branch).

Procedure:

1. Disable command mode.
2. Identify the offending joint: any `|q| > start_pose_limit_rad` (default
   3.0 rad) in `/joint_states`.
3. Jog/unwind that joint on the pendant toward its 0-turn branch until
   `/joint_states` matches the pendant reading (typically the wrist). A full
   arm reboot also resets the reported branch.
4. Re-enable command mode. The gate re-checks every tick and arms streaming
   once all joints are within the limit; each `enable_command` toggle resets
   the streamer so no stale hold pose can replay.

## v_safe_scale Ramp-Up

`v_safe_scale` scales the metadata `v_safe`/`a_safe` for both mapper and
safety limiter. `1.0` is the trained contract; the current export carries UR3e
hard limits and the policy saturates actions, so `1.0` means full-speed joints.

- Change it only while command mode is off (Test tab staged buttons
  `0.5 -> 0.7 -> 0.85 -> 1.0 -> 1.25 -> 1.5 -> 2.0 -> 2.5 -> 3.0 -> 4.0`).
- Start every session at `0.5` (the config default). At each step: enable
  command, run 2–3 virtual balls, watch the monitoring points below, disable
  command, then move one step up only if the previous step was clean.
- Do not exceed `1.0` for catch attempts: `>1.0` is lab overdrive that diverges
  from training and can exceed metadata/UR nominal limits — expect UR driver
  velocity rejections or protective stops; use it only for deliberate limit
  tests with a clear workspace.
- Real-ball throws only after the target scale is clean on virtual balls.

## What To Monitor

- **Tracking error**: watchdog `max_tracking_error` (0.5 rad) on
  `|q_measured - last_command|`; rising error before the watchdog fires means
  the arm cannot follow the commanded speed.
- **Protective stops / pendant faults**: any `Velocity ... exceeding the joint
  velocity limits` or protective stop ends the ramp-up step; investigate
  before retrying.
- **Latency**: run the `latency_report` node and watch
  `perception_age_s` / `loop_compute_s` percentiles; see the measurement plan
  in [Observation Latency And Models](../sim-to-real/observation-latency-and-models.md).
  `perception_age_s` beyond `stale_after_s=0.1` triggers the stale-perception
  watchdog.
- **Controller state**: the controller must stay active across throws; a
  controller that appears inactive with no throws reaching `test_ball_node`
  was the 2026-07-02 heartbeat regression signature, fixed since.
- **Loop budget**: `loop_compute_s` against `loop_budget_s=0.02`.

## After The Session

- Disable command mode; the node restores the trajectory controller.
- Record protective stops, gate trips and the highest clean `v_safe_scale` in
  [Current Status And Blockers](../live-catch/current-status-and-blockers.md).

## See Also

- [Safety And Commanding](../live-catch/safety-and-commanding.md)
- [Current Status And Blockers](../live-catch/current-status-and-blockers.md)
- [Testing And Commands](testing-and-commands.md)
- [UR3e Control Stack](../robot-control/ur3e-control-stack.md)
- [Observation Latency And Models](../sim-to-real/observation-latency-and-models.md)
