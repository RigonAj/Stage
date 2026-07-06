# Isaac Training Environment

> Sources: Isaac FirstTraining env/cfg working tree, 2026-07-03; Isaac README snapshot, 2026-07-03; Isaac environment-and-frames snapshot, 2026-07-03; policy metadata `latest`, 2026-06-30; agent review, 2026-07-02
> Raw: [Isaac README snapshot](../../raw/isaac/2026-07-03-firsttraining-readme.md); [Environment and frames snapshot](../../raw/isaac/2026-07-03-environment-and-frames.md); [Policy metadata](../../data/models/latest/policy_metadata.json); [Agent review](../../raw/reviews/2026-07-02-stage-wiki-and-training-review.md)

## Overview

The deployed policy is trained in the sibling Isaac Lab repository
`/home/rigon/Documents/6-Dof-Ur3e-Catch-a-ball` (task
`Template-Firsttraining-Direct-v0`, direct workflow, SKRL PPO). This page
compiles the environment definition the Stage live node must mirror:
observation layout, action integrator, reward, terminations, commands and the
cross-repo export/sync procedure. Authoritative code:
`source/FirstTraining/FirstTraining/tasks/direct/firsttraining/firsttraining_env.py`
and `firsttraining_env_cfg.py` in that repo.

## Observation (33-D, order matters)

`_get_observations()` concatenates, in this exact order:

| Index | Component | Dim |
|-------|-----------|-----|
| 0–5 | joint positions (6 arm joints, `UR3E_ARM_JOINTS` order) | 6 |
| 6–11 | joint velocities | 6 |
| 12–14 | disk (hoop) position, local/env frame | 3 |
| 15–17 | ball position, local/env frame | 3 |
| 18–20 | unit direction disk -> ball | 3 |
| 21 | disk–ball distance (m) | 1 |
| 22–24 | ball velocity, world frame | 3 |
| 25 | previous-tick "ball in front of disk" signed flag (0/1) | 1 |
| 26–31 | previous clipped action | 6 |
| 32 | cumulative pass-through count | 1 |

The Isaac "local" frame is world minus the environment origin and matches the
UR3e `base_link` (metadata `observation_frame=base_link`). The pass-through
flag and count are updated in `_get_dones()` before the next observation, which
is why the Stage `ObservationBuilder` emits current-state values (see
[Observation Latency And Models](observation-latency-and-models.md)).

## Action Contract

The policy outputs 6 normalized actions. `_pre_physics_step()`:

1. clamps the action to `[-1, 1]` (the clipped value is what enters obs 26–31);
2. desired step `Δq = action * joint_velocity_safe_rad_s * dt` (`dt = 1/60`,
   decimation 2 over a 120 Hz physics step);
3. clamps the implied command velocity so the joint can still stop before its
   position bounds given `joint_acceleration_safe_rad_s2` (stopping-distance
   term);
4. rate-limits the command-velocity change per step to `a_safe * dt` and the
   velocity to `±v_safe`;
5. integrates into `joint_position_target_rad` and clamps to position bounds.

This is the incremental target-integrator semantics recorded in
`policy_metadata.json` and mirrored by the Stage `ActionMapper` (see
[Policy Transfer And Action Semantics](policy-transfer-and-action-semantics.md)).

## Reward

`compute_rewards()` returns the sum of:

- `rew_dist = exp(-2 * d) - d` with `d` the disk–ball distance;
- per-joint action penalty `-Σ coeff_j * a_j²`, with `coeff_j` ramped by a
  smoothstep warmup over `action_penalty_warmup_steps = 150_000` steps from the
  low to the high end of `joint_action_penalty_coeff_ranges` (e.g. shoulder
  0.85→2.55, wrist_3 0.25→0.75);
- `+400` on a pass-through event;
- `-100` on termination (`reset_terminated`).

## Terminations And Ball Respawn

From `_get_dones()`:

- `hit_arm`: any arm-link contact force > 0.1 N (contact sensor on base,
  shoulder, upper-arm, forearm, wrist_1, wrist_2 links) terminates.
- `ball_on_ground`: ball world z < 0.05 m terminates (mirrored by the Stage
  `test_ball_node` `ground_z_m=0.05`).
- `reset_on_success=False`: a successful pass-through does **not** end the
  episode; with `reset_ball_on_success=True` the ball respawns while the
  episode continues, and `ball_respawn_hold_at_disk_center=True` holds the
  respawned ball at the disk center during the respawn delay.
- Timeout at `episode_length_s = 4.0`.
- `extras["success"]` marks episodes with at least one pass-through; the
  headless evaluation success rate is cumulative over completed episodes.

Robot reset: episode resets do not re-pose the robot
(`reset_robot_on_episode_reset=False`); the pose is randomized around a home
configuration on first init and with probability 0.05 on ball reset.

## Ball Distribution And Noise

Ball spawns uniformly in x `(-0.6, -0.2)`, y `(1.2, 2.1)`, z `(0.5, 1.2)` m
with velocity x `(-0.7, 0.6)`, y `(-5.0, -3.5)`, z `(-0.1, 1.5)` m/s, toward
the robot. Gaussian spawn-position noise is enabled with
`ball_position_noise_std = 0.01` in the current cfg. Discrepancy: the Isaac
README's reference result (~98 % success in headless evaluation) is stated at
`ball_position_noise_std = 0.05`; the current cfg and the deployed metadata
carry `0.01`. Treat the 98 % figure as measured under the 0.05 setting until
re-evaluated (review Volet 3, action B6).

## Safety Limits In The Cfg (current state)

As of 2026-07-03 the Isaac checkout carries **uncommitted working-tree
changes** that halve the training limits and shrink position bounds:
`joint_velocity_safe_rad_s = (π/2, π/2, π/2, π, π, π)`,
`joint_acceleration_safe_rad_s2 = (2π, 2π, 2π, 4π, 4π, 4π)`, all joints
`±π`. The last commit still has the full limits (π base / 2π wrists, ±2π
bounds, elbow ±π), and the deployed `data/models/` metadata was exported from
the full-limit training. The halved limits only take effect after the change is
committed, retrained and re-exported; until then the Stage-side `v_safe_scale`
parameter is the operative slow-down (see
[Safety And Commanding](../live-catch/safety-and-commanding.md)).

## Commands

From the Isaac repo root (`source script.zsh` provides the aliases):

```bash
# Train (headless, 12000 envs by default in the README invocation)
train

# Interactive play with the newest checkpoint (dashboard UI, 1 env)
play            # variants: play latest | play best | checkpoint [best]

# Headless success-rate evaluation (512 envs, cumulative success rate)
evaluate

# Export deterministic policy + ONNX + policy_metadata.json
sim2real_export

# Validate an export directory (metadata + rollout safety)
sim2real_validate
```

The raw `python scripts/skrl/play.py` forms with `--export_policy
--export_onnx`, `--eval_episodes`, `--record_actions` are in the README
snapshot. Checkpoints and exports land under `logs/skrl/cartpole_direct/`
(directory name inherited from the template).

## Cross-Repo Sync Procedure

1. Export from the chosen checkpoint with `--export_policy --export_onnx
   --export_dir=<Stage>/data/models/<latest|best>` (or copy the
   `exports/` directory).
2. `data/models/latest` is the canonical deployment model; the root
   `policy_deterministic.ts` is a copy of it.
3. Verify metadata parity before deployment:

```bash
python3 /home/rigon/Documents/6-Dof-Ur3e-Catch-a-ball/scripts/sim2real_validate_export.py \
  --exports data/models/latest --metadata-only
```

4. Check `policy_metadata.json` fields the live node consumes:
   `observation_space=33`, `action_space=6`, `dt_s=1/60`, `joint_names`,
   `action_semantics` (incremental integrator), `observation_frame=base_link`,
   per-joint `joint_velocity_safe_rad_s` / `joint_acceleration_safe_rad_s2` /
   position bounds, `disk_radius_m`, hoop offset/normal in `wrist_3_link`.
   Any cfg limit change is invisible to deployment until it appears in this
   file.
5. The live node validates metadata on load and rejects model switches while
   command mode is active.

## See Also

- [Policy Transfer And Action Semantics](policy-transfer-and-action-semantics.md)
- [Observation Latency And Models](observation-latency-and-models.md)
- [Safety And Commanding](../live-catch/safety-and-commanding.md)
- [Testing And Commands](../operations/testing-and-commands.md)
- [Real Robot Bring-Up Runbook](../operations/real-robot-bringup-runbook.md)
