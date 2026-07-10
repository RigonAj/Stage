# Isaac Training Environment

> Sources: Isaac FirstTraining env/cfg working tree, 2026-07-03; Isaac README snapshot, 2026-07-03; Isaac environment-and-frames snapshot, 2026-07-03; policy metadata `latest`, 2026-06-30; agent review, 2026-07-02; left-hand variant and `latest-left` metadata, 2026-07-06; deployment review, 2026-07-10
> Raw: [Isaac README snapshot](../../raw/isaac/2026-07-03-firsttraining-readme.md); [Environment and frames snapshot](../../raw/isaac/2026-07-03-environment-and-frames.md); [Right policy metadata](../../data/models/latest/policy_metadata.json); [Left policy metadata](../../data/models/latest-left/policy_metadata.json); [Agent review](../../raw/reviews/2026-07-02-stage-wiki-and-training-review.md); [Perception/control review](../../docs/Robot_Control/revue_perception_robuste_controle_fluide_2026-07-10.md)

## Overview

The deployed policy is trained in the sibling Isaac Lab repository — on this
PC `/home/rigon/Documents/IsaacTrain/Cartpole/Cartpole/FirstTraining` (task
`Template-Firsttraining-Direct-v0`, direct workflow, SKRL PPO; the older
`/home/rigon/Documents/6-Dof-Ur3e-Catch-a-ball` path is another checkout of
the same project and is absent here). This page
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

The historical right task spawns uniformly in x `(-0.6, -0.2)`, y `(1.2, 2.1)`, z `(0.5, 1.2)` m
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
bounds, elbow ±π), and the right `latest`/`best` metadata was exported from the
full-limit training. The newer `latest-left` metadata also retains the full
velocity/acceleration values but records ±π position bounds. The halved limits
only take effect after a matching change is committed, retrained and
re-exported; until then the Stage-side `v_safe_scale` parameter is the
operative slow-down (see
[Safety And Commanding](../live-catch/safety-and-commanding.md)).

## Left-Hand Variant (hold_side)

Since 2026-07-06 the repo carries a mirrored task
`Template-Firsttraining-Direct-Left-v0` (cfg `FirsttrainingEnvCfgLeft`,
skrl cfg `skrl_ppo_cfg_left.yaml`, checkpoints under
`logs/skrl/cartpole_direct_left/`). It is the yz-plane mirror (`x -> -x`) of
the historical right-hand setup, seen from in front of the robot:

- The racket USD is rotated 180 deg about wrist_3 Z:
  `USD_File/UR-with-gripper-left.usd`, generated from `UR-with-gripper.usd`
  by `scripts/make_left_hand_usd.py` (run with the `~/env_isaaclab` python;
  the disk offset read at startup becomes `(+0.5, 0, 0)` and the disk normal
  stays `(0, 0, -1)` in `wrist_3_link`).
- Ball distribution x components are mirrored: spawn x `(0.2, 0.6)`,
  velocity x `(-0.6, 0.7)`. y/z ranges and everything else are inherited.
- The cfg exposes `hold_side` (`right` in the base cfg, `left` in the
  variant); `play.py` writes it to `policy_metadata.json` together with the
  disk offset/normal/radius and ball ranges, and
  `sim2real_validate_export.py` cross-checks `hold_side` against the sign of
  `disk_offset_wrist_3_link_m[0]` when present.

The deployed `data/models/latest-left/policy_metadata.json` was re-checked on
2026-07-10 and is the source of truth for the current real-perception runbook:

- spawn `x=[0.2,0.6]`, `y=[1.2,2.1]`, `z=[0.5,1.2]` m;
- velocity `vx=[-0.6,0.7]`, `vy=[-5.0,-4.0]`, `vz=[0.2,1.5]` m/s;
- `ball_position_noise_std_m=0.05` and `disk_radius_m=0.1`;
- disk offset about `(+0.5,0,0)` and normal `(0,0,-1)` in `wrist_3_link`;
- full velocity limits (π base joints, 2π wrists), corresponding full
  accelerations, and ±π position bounds.

These left-export values supersede generic/right-model values whenever
`latest-left` is loaded. They define the initial bring-up throw envelope; note
that a spawn-position-noise field does not by itself prove robustness to
temporally correlated perception jitter, latency or dropouts.

Observation, reward and termination math is side-agnostic (it derives from
the disk pose read from the USD), so no env-code change was needed. A policy
trained on one side is not expected to work on the other; deployment must
match the physical mount, the `hold_side` launch argument of the Stage
bring-up and the model metadata (see
[Policy Transfer And Action Semantics](policy-transfer-and-action-semantics.md)).

## Commands

From the Isaac repo root (`source env.zsh` provides the aliases; the older
`script.zsh` was renamed). All commands honor
`FT_TASK=Template-Firsttraining-Direct-Left-v0` to target the left-hand
variant (which also switches the default checkpoint root to
`logs/skrl/cartpole_direct_left`), and `train-left` / `train-right` /
`play-left` / `play-right` are shortcuts that pin the task per hold side:

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
   `exports/` directory). Left-hand trainings go to
   `data/models/latest-left` / `best-left` — the web UI allowlist only knows
   these four names.
2. `data/models/latest` is the canonical deployment model; the root
   `policy_deterministic.ts` is a copy of it.
3. Verify the export before deployment (the validator also needs the
   `rollouts_*_episodes.json` recorded by `sim2real_export`):

```bash
python3 /home/rigon/Documents/IsaacTrain/Cartpole/Cartpole/FirstTraining/scripts/sim2real_validate_export.py \
  --exports data/models/latest
```

4. Check `policy_metadata.json` fields the live node consumes:
   `observation_space=33`, `action_space=6`, `dt_s=1/60`, `joint_names`,
   `action_semantics` (incremental integrator), `observation_frame=base_link`,
   per-joint `joint_velocity_safe_rad_s` / `joint_acceleration_safe_rad_s2` /
   position bounds, `disk_radius_m`, hoop offset/normal in `wrist_3_link`,
   and `hold_side` (`right`/`left`; exports predating 2026-07-06 lack the
   field and are all right-hand). Any cfg limit change is invisible to
   deployment until it appears in this file.
5. The live node validates metadata on load and rejects model switches while
   command mode is active.

## See Also

- [Policy Transfer And Action Semantics](policy-transfer-and-action-semantics.md)
- [Observation Latency And Models](observation-latency-and-models.md)
- [Safety And Commanding](../live-catch/safety-and-commanding.md)
- [Testing And Commands](../operations/testing-and-commands.md)
- [Real Robot Bring-Up Runbook](../operations/real-robot-bringup-runbook.md)
- [Perception Robustness And Flight Lifecycle](../perception/perception-robustness-flight-lifecycle.md)
