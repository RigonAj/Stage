# Observation Latency And Models

> Sources: sim-to-real plan, 2026-06-30; live-catch implementation status, 2026-06-30; sim-to-real proposals, 2026-06-30; model README, 2026-06-30
> Raw: [Sim-to-real plan](../../docs/Robot_Control/ur3e_ball_catch_sim_to_real.md); [Implementation status](../../docs/Robot_Control/ur3e_live_catch_implementation_status.md); [Proposals](../../docs/Robot_Control/ur3e_sim2real_propositions.md); [Model README](../../data/models/README.md)

## Overview

This page separates observation construction, latency and model management from
the action-space discussion. These concerns change at deployment time and are
easy to mix up with training decisions.

## Observation Contract

The live node reconstructs the 33-D PPO observation from:

- robot joint positions and velocities;
- ball position in `base`;
- filtered ball velocity;
- hoop/disk position;
- pass-through state;
- previous policy action according to the model contract: raw action for legacy
  absolute exports, clipped action for current incremental Isaac exports.

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

## Model Management

- `data/models/` is the canonical live model location.
- `data/models/latest` contains the 2026-06-30 export from `agent_118000.pt`;
  `data/models/best` contains the matching latest `best_agent.pt` export.
- The root `data/models/policy_deterministic.ts` is a copy of `latest` and is
  loaded by default.
- Model metadata encodes action semantics so the live node selects the correct
  mapper.
- Current Isaac exports should include `rollout_schema_version`, `dt_s`,
  `joint_names`, action semantics and per-joint safety limits. Legacy metadata
  without those fields is insufficient for V1 sim-to-real validation.
- Current Isaac actuator limits should align with `ur_description`: velocity
  `[pi, pi, pi, 2*pi, 2*pi, 2*pi]` rad/s and effort
  `[56, 56, 28, 12, 12, 12]` Nm.
- The current TorchScript export was verified as self-contained for scaling.

## See Also

- [Policy Transfer And Action Semantics](policy-transfer-and-action-semantics.md)
- [Message Contracts And Topics](../live-catch/message-contracts-and-topics.md)
- [Current Status And Blockers](../live-catch/current-status-and-blockers.md)
