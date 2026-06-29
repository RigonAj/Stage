# Policy Transfer And Action Semantics

> Sources: sim-to-real plan, 2026-06-29; action-space decision, 2026-06-29; sim-to-real proposals, 2026-06-29; model README, 2026-06-29
> Raw: [Sim-to-real plan](../../docs/Robot_Control/ur3e_ball_catch_sim_to_real.md); [Action-space decision](../../docs/Robot_Control/ur3e_choix_espace_action_isaac.md); [Proposals](../../docs/Robot_Control/ur3e_sim2real_propositions.md); [Model README](../../data/models/README.md)

## Overview

The PPO transfer problem is not just model loading. It depends on action
semantics, observation equivalence, latency, actuator limits, safety behavior and
model metadata.

## Current Policy Semantics

The documented rollout check showed:

```text
joint_position_target_rad = action_normalized * 0.5
```

For the current export, the previous raw policy action is preserved in the
observation in `faithful` mode. Safety still clips/rate-limits command targets
independently.

## Main Transfer Risks

- Safety in the live loop changes closed-loop dynamics unless represented during
  training.
- Real perception latency must be measured and modeled or compensated.
- Ball velocity is noisy because it is inferred from position history.
- Action semantics must be encoded in model metadata to avoid deploying a model
  with the wrong mapper.
- `data/models/` should contain the canonical model and metadata.

## See Also

- [Observation Latency And Models](observation-latency-and-models.md)
- [Live Catch Loop](../live-catch/live-catch-loop.md)
- [UR3e Actuator Identification](../system-id/ur3e-actuator-identification.md)
- [Current Status And Blockers](../live-catch/current-status-and-blockers.md)
