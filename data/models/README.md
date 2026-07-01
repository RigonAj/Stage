# data/models — modèles IA pour la boucle live

Emplacement **canonique** des modèles chargés par `ur3e_live_catch`
(`policy_runtime.py` / `PolicyRunner`, voir
`docs/Robot_Control/ur3e_live_catch_architecture.md` §4.3.3).

On y place (ou on y lie par symlink) le modèle réellement utilisé en live, pour
ne pas dépendre d'un chemin d'export d'entraînement daté.

## Contenu attendu

- `policy_deterministic.ts` — politique TorchScript déterministe (runtime principal).
- `policy_deterministic.onnx` — même politique en ONNX (runtime alternatif).
- `policy_metadata.json` — métadonnées d'export (dims obs/action, etc.).
- scaler SKRL (`RunningStandardScaler`) **si** non embarqué dans l'export :
  `mean` / `var` à appliquer **avant** le réseau (point critique §4.3.3).

Plusieurs modèles → un sous-dossier par modèle (ex. `data/models/<nom>/`).

## Source actuelle

Export Isaac du 2026-06-30 depuis :

```
~/Documents/IsaacTrain/Cartpole/Cartpole/FirstTraining/logs/skrl/cartpole_direct/
  2026-06-30_19-02-25_ppo_torch/checkpoints/
```

Deux exports sont conservés dans Git :

```
data/models/latest/
  checkpoint_agent_latest.pt
  policy_deterministic.ts
  policy_deterministic.onnx
  policy_metadata.json

data/models/best/
  checkpoint_best_agent.pt
  policy_deterministic.ts
  policy_deterministic.onnx
  policy_metadata.json
```

Le modèle canonique chargé par défaut par `live_catch_node` est `latest` :

```
data/models/policy_deterministic.ts
data/models/policy_deterministic.onnx
data/models/policy_metadata.json
```

Le `best` reste disponible en passant explicitement :

```bash
ros2 launch ur3e_live_catch live_catch.launch.py \
  model_path:=data/models/best/policy_deterministic.ts
```

## Contrat actuel

Les deux exports 2026-06-30 utilisent :

- observation 33-D ;
- action 6-D ;
- `dt_s = 1/60` ;
- `joint_names = [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]` ;
- sémantique d'action incrémentale :
  `previous joint_position_target_rad + clamp(action_normalized, -1, 1) * joint_velocity_safe_rad_s * dt_s`,
  puis limites d'accélération et de position.

`live_catch_node` lit `policy_metadata.json` et fait résoudre
`action_mode=faithful` vers le mapper incrémental pour ces exports. Ne pas
charger ces modèles avec l'ancien mapper absolu `action * 0.5`.

## Rollouts de validation

Les fichiers `rollouts_10_episodes.json` ne sont pas versionnés dans `main` pour
garder le dépôt léger. Ils peuvent être régénérés depuis IsaacTrain si une
validation/relecture exacte est nécessaire :

```bash
cd ~/Documents/IsaacTrain/Cartpole/Cartpole/FirstTraining
source script.zsh
play latest --headless --livestream 0 --rendering_mode performance \
  --export_policy --export_onnx --record_actions --record_episodes=10 \
  --export_dir=/home/rigon/Documents/Stage/Stage/data/models/latest
```
