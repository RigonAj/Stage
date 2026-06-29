# Sim To Real

## Scope

This domain covers PPO policy transfer from Isaac Lab to the UR3e live-catch
stack: action semantics, observation equivalence, latency, domain
randomization, actuator limits and policy export metadata.

## Source Of Truth

- `docs/Robot_Control/ur3e_ball_catch_sim_to_real.md`
- `docs/Robot_Control/ur3e_choix_espace_action_isaac.md`
- `docs/Robot_Control/ur3e_sim2real_propositions.md`
- `docs/Robot_Control/ur3e_parametres_actionneur_reference.md`
- `docs/Robot_Control/ur3e_resultats_identification_gains.md`
- `data/models/README.md`

## Current Decisions

- The live policy path uses a 33-D observation and a 6-D action.
- The current exported TorchScript policy is treated as self-contained for
  observation scaling.
- The rollout data showed the current policy action semantics:
  `joint_position_target_rad = action_normalized * 0.5`.
- `faithful` mode preserves the raw previous policy action in the observation.
- Safety remains independent from the policy.
- Position/incremental semantics and safety-in-loop training are the main
  transfer issues to keep explicit.

## Main Risks

- Real latency must be measured and either modeled or compensated.
- Ball velocity is inferred from noisy positions and needs filtering.
- Action semantics must be encoded by model metadata to prevent loading a model
  with the wrong live mapper.
- The safety layer can change closed-loop behavior; training should eventually
  include equivalent limits.
- The canonical model in `data/models/` is not yet fully established.

## Related Notes

- [[UR3e_Live_Catch]]
- [[System_ID]]
- [[Current_Status]]
