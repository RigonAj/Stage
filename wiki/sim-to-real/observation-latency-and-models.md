# Observation Latency And Models

> Sources: sim-to-real plan, 2026-07-01; live-catch implementation status, 2026-06-30; sim-to-real proposals, 2026-06-30; model README, 2026-07-01; latency measurement plan from agent review, 2026-07-02
> Raw: [Sim-to-real plan](../../docs/Robot_Control/ur3e_ball_catch_sim_to_real.md); [Implementation status](../../docs/Robot_Control/ur3e_live_catch_implementation_status.md); [Proposals](../../docs/Robot_Control/ur3e_sim2real_propositions.md); [Model README](../../data/models/README.md); [Latency report node](../../src/ur3e_live_catch/ur3e_live_catch/latency_report.py)

## Overview

This page separates observation construction, latency and model management from
the action-space discussion. These concerns change at deployment time and are
easy to mix up with training decisions.

## Observation Contract

The live node reconstructs the 33-D PPO observation from:

- robot joint positions and velocities;
- ball position in `base_link`, matching Isaac FirstTraining's local frame;
- filtered ball velocity, which is equivalent to world velocity because the
  Isaac local frame differs from world by a constant environment origin;
- hoop/disk position in `base_link`;
- pass-through state;
- previous policy action according to the model contract: raw action for legacy
  absolute exports, clipped action for current incremental Isaac exports.
- disk trigger radius. The current metadata carries `disk_radius_m=0.05`, hoop
  offset `(-0.5, 0, 0)` in `wrist_3_link`, and hoop normal `(0, 0, -1)`.
- pass-through ordering. Isaac updates `prev_disk_signed_dist` and
  `pass_through_count` in `_get_dones()` before the next observation, so the live
  builder emits the current signed flag and updated pass count for the current
  ball/disk state.
- ball velocity source. With `use_ball_state_velocity=true` (default) the live
  node trusts a producer-supplied `BallState.velocity` (the ballistic
  regression node fills it from the fit derivative — clean velocity from the
  first valid tick, like an Isaac spawn); an exactly-zero vector means "not
  provided" and falls back to the internal EMA finite-difference filter. That
  filter is now reset between throws (`_reset_policy_state`), fixing a stale
  velocity leaking into the first tick of a new throw.

The ordering and units must mirror the Isaac environment. Any change here needs
tests against recorded rollout or exported policy expectations.

## Latency

The docs treat perception-to-command latency as a top transfer risk. Important
rules:

- Native `BallState` event timestamps are preferred.
- Legacy float-array adapter timestamps at reception and is less reliable for
  latency analysis.
- `latency_report` and `CatchTelemetry.perception_age_s` are the main runtime
  instruments.
- Real-perception p50/p95/p99 must be measured before real ball interception.
- With `use_ball_regression:=true`, published `BallState.stamp` is the fit
  EVALUATION time, so `perception_age_s` ≈ 0 by design (the regression predicts
  the ball at "now", compensating measurement latency inside the fit). The
  staleness watchdog then monitors regression liveness, not the camera; a dead
  camera mid-flight ends the flight through the regression coast timeout
  (`max_coast_s`) instead. Measure true camera latency on `ball_state_raw`.

## Latency Measurement Plan

Instrumentation already in the tree:

- `CatchTelemetry.perception_age_s` = live-node time minus `BallState.stamp`
  at each 60 Hz tick (end-to-end perception staleness as seen by the policy);
- `CatchTelemetry.loop_compute_s` = hot-path compute time
  (observation -> policy -> safety) for that tick;
- the `latency_report` node (`ros2 run ur3e_live_catch latency_report`)
  subscribes to `catch_telemetry` and prints rolling and final summaries with
  count / mean / p50 / p95 / p99 / max in ms (`report_period_s`, default 5 s).

Collection procedure (real perception):

1. Bring up the real-perception stack (`ball_tracking_cpp` publishing native
   `BallState`, not the legacy float-array adapter — the adapter timestamps at
   reception and undercounts latency).
2. Run `latency_report` alongside `live_catch_node` in dry-run.
3. Throw or move the real ball through the tracked volume for enough ticks to
   make p99 meaningful (several hundred valid-ball ticks; heartbeat ticks with
   `ball_valid=false` carry `perception_age_s=0` and must be excluded by
   collecting during active tracking).
4. Record the final summary (printed on shutdown) for `perception_age` and
   `loop_compute`.

Acceptance anchors (proposed until training-latency modeling is done):

- Hard bound: `perception_age_s` p99 must stay below `stale_after_s=0.1`,
  otherwise the watchdog rejects ticks as stale in normal operation.
- Working target: p95 ≤ ~0.05 s. A ball flight from the Isaac spawn ranges
  lasts roughly 0.3–0.5 s, so 50 ms of staleness already consumes a large
  fraction of the interception horizon.
- `loop_compute_s` p99 must stay below `loop_budget_s=0.02` (watchdog budget)
  and ideally below one loop period (16.7 ms).
- Per the sim-to-real plan (§5.6): if measured perception age exceeds the
  latency modeled at training, widen the latency randomisation and retrain
  rather than deploying with an unmodeled gap.

## Model Management

- `data/models/` is the canonical live model location.
- `data/models/latest` contains the 2026-06-30 export from `agent_118000.pt`;
  `data/models/best` contains the matching latest `best_agent.pt` export.
- The root `data/models/policy_deterministic.ts` is a copy of `latest` and is
  loaded by default.
- The Web UI Test tab can switch between `latest` and `best`; the backend only
  exposes these named folders and prefers ONNX before TorchScript. If an
  explicit ONNX path fails to load because `onnxruntime` is unavailable,
  `live_catch_node` tries the sibling TorchScript export.
- On a model switch, `live_catch_node` validates metadata, reloads the policy,
  rebuilds the observation/action/safety state, and resets dry-run policy state.
  It rejects model switches while command mode is active.
- Model metadata encodes action semantics so the live node selects the correct
  mapper.
- Current Isaac exports should include `rollout_schema_version`, `dt_s`,
  `joint_names`, `observation_frame=base_link`, action semantics,
  disk/ball-distribution metadata and per-joint safety limits.
  Legacy metadata without those fields is insufficient for V1 sim-to-real validation.
- The SKRL policy for the current export used `clip_actions=false`; clipping
  happens in Isaac's env and in Stage's incremental mapper.
- Current Isaac actuator limits should align with `ur_description`: velocity
  `[pi, pi, pi, 2*pi, 2*pi, 2*pi]` rad/s and effort
  `[56, 56, 28, 12, 12, 12]` Nm.
- The current TorchScript export was verified as self-contained for scaling.

## See Also

- [Isaac Training Environment](isaac-training-environment.md)
- [Policy Transfer And Action Semantics](policy-transfer-and-action-semantics.md)
- [Real Robot Bring-Up Runbook](../operations/real-robot-bringup-runbook.md)
- [Message Contracts And Topics](../live-catch/message-contracts-and-topics.md)
- [Current Status And Blockers](../live-catch/current-status-and-blockers.md)
