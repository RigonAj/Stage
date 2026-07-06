# Policy Transfer And Action Semantics

> Sources: sim-to-real plan, 2026-07-01; action-space decision, 2026-06-29; sim-to-real proposals, 2026-06-30; model README, 2026-07-01; agent review corrections, 2026-07-03
> Raw: [Sim-to-real plan](../../docs/Robot_Control/ur3e_ball_catch_sim_to_real.md); [Action-space decision](../../docs/Robot_Control/ur3e_choix_espace_action_isaac.md); [Proposals](../../docs/Robot_Control/ur3e_sim2real_propositions.md); [Model README](../../data/models/README.md); [Agent review](../../raw/reviews/2026-07-02-stage-wiki-and-training-review.md)

## Overview

The PPO transfer problem is not just model loading. It depends on action
semantics, observation equivalence, latency, actuator limits, safety behavior and
model metadata.

## Current Policy Semantics

There are now two distinct action contracts to keep separate.

The dated reference export used by the current Stage fallback rollouts is a
legacy absolute-action contract:

```text
joint_position_target_rad = action_normalized * 0.5
```

The current Isaac FirstTraining environment uses an incremental target-integrator
contract: the normalized action is clipped to `[-1, 1]`, scaled by
`joint_velocity_safe_rad_s * dt_s`, acceleration- and joint-limit-clamped, then
integrated from the previous `joint_position_target_rad`. This keeps the command
trajectory velocity-bounded while giving Isaac's position actuator a target it
can actually chase, instead of keeping the target only one tiny step ahead of the
measured joint position.

For deployment, `policy_metadata.json` is the source of truth for the action
contract. The Stage `ActionMapper` now keeps legacy absolute `faithful`
compatibility, but resolves `action_mode=faithful` to the incremental mapper when
the loaded metadata declares the current Isaac target-integrator semantics.
The Web UI model selector does not choose an action mapper directly; it only
sets `model_path`, then the live node validates metadata and rebuilds the mapper
from that metadata.

The 2026-06-30 `latest` and `best` exports in `data/models/` both declare
`observation_space=33`, `action_space=6`, `dt_s=1/60`, per-joint velocity and
acceleration limits, `observation_frame=base_link`, `disk_radius_m=0.05`, and
the incremental action semantics.
Those exports carry the UR3e HARD velocity limits as `v_safe` (π rad/s base
joints, 2π rad/s wrists) and ±2π position bounds, and the policy saturates its
raw actions, so deployment runs every joint at full speed. The halved
velocities/accelerations and ±π position bounds exist in the Isaac
`FirstTraining` cfg only as uncommitted working-tree changes in the local
checkout (verified 2026-07-03, not in the last commit) — they only land in
metadata after commit, retraining and re-export; until then the live node's
`v_safe_scale` parameter provides the robot-side slow-down.
The SKRL policy has `clip_actions=false`; the environment/action mapper clip to
`[-1, 1]` before integration and feed back that clipped action in observation
component 9.

## Main Transfer Risks

- Safety in the live loop changes closed-loop dynamics unless represented during
  training.
- Real perception latency must be measured and modeled or compensated.
- Ball velocity is noisy because it is inferred from position history.
- Action semantics must stay encoded in model metadata to avoid deploying a
  model with the wrong mapper.
- Runtime model changes must stay disabled while real-robot command mode is
  active, otherwise the policy, mapper and safety state could change mid-flight.
- Legacy absolute-action exports and current incremental-action exports share
  the live node only through metadata-driven mapper selection.
- `data/models/` should contain the canonical model and metadata.

## See Also

- [Isaac Training Environment](isaac-training-environment.md)
- [Observation Latency And Models](observation-latency-and-models.md)
- [Live Catch Loop](../live-catch/live-catch-loop.md)
- [UR3e Actuator Identification](../system-id/ur3e-actuator-identification.md)
- [Current Status And Blockers](../live-catch/current-status-and-blockers.md)
