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

## Côté de tenue de la raquette (`hold_side`)

Chaque `policy_metadata.json` porte un champ `hold_side` (`right` ou `left`),
vu de face du robot :

- `right` — montage historique : centre du cerceau `(-0.5, 0, 0)` dans
  `wrist_3_link`, balle lancée depuis `x < 0`. Tous les exports antérieurs au
  2026-07-06 sont `right` (le champ a été ajouté rétroactivement).
- `left` — raquette tournée de 180° autour du Z de `wrist_3` : centre
  `(+0.5, 0, 0)`, distribution balle en miroir du plan yz (`x → -x`). Tâche
  Isaac `Template-Firsttraining-Direct-Left-v0`, exports attendus dans
  `data/models/latest-left/` et `data/models/best-left/`.

Un modèle ne doit être utilisé que si le montage physique de la raquette, la
TF `hoop_center` (argument `hold_side` du launch `ur3e_live_catch`) et son
`hold_side` concordent. Le sélecteur du Web UI affiche le côté de chaque
modèle et avertit en cas d'incohérence avec le toggle de tenue.

## Source actuelle

`data/models/latest-left/` : export Isaac du 2026-07-06 (`hold_side=left`,
task `Template-Firsttraining-Direct-Left-v0`) depuis
`logs/skrl/cartpole_direct_left/2026-07-06_14-29-14_ppo_torch/checkpoints/agent_194000.pt`.

Modèles droite (`latest`, `best`) : export Isaac du 2026-06-30 depuis :

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

Depuis le 2026-07-01, l'onglet **Test** du Web UI expose aussi un sélecteur
autorisé `latest` / `best`. Le backend préfère
`data/models/<nom>/policy_deterministic.onnx`, puis
`policy_deterministic.ts` si l'ONNX n'existe pas, et envoie ce chemin au
paramètre ROS `/live_catch_node model_path`. Si l'ONNX existe mais que
`onnxruntime` n'est pas disponible dans l'interpréteur du nœud, `live_catch_node`
essaie automatiquement le `policy_deterministic.ts` voisin. Le nœud charge et
valide le nouveau modèle avant de remplacer la policy active, et refuse tout
changement pendant `enable_command=true`.

## Contrat actuel

Les deux exports 2026-06-30 utilisent :

- observation 33-D ;
- observation construite en frame `base_link` (`observation_frame=base_link`) ;
- action 6-D ;
- `dt_s = 1/60` ;
- `joint_names = [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]` ;
- `disk_radius_m = 0.05` pour le trigger de passage dans le cerceau ;
- géométrie cerceau Isaac : centre `(-0.5, 0, 0)` et normale `(0, 0, -1)` dans
  `wrist_3_link` ;
- distribution balle FirstTraining : `p0.x=(-0.6,-0.2)`, `p0.y=(1.2,2.1)`,
  `p0.z=(0.5,1.2)`, bruit position `0.01 m`, `v0.x=(-0.7,0.6)`,
  `v0.y=(-5.0,-3.5)`, `v0.z=(-0.1,1.5)` ;
- policy SKRL exportée avec `clip_actions: False`; le clipping `[-1, 1]`
  appartient au contrat de l'environnement/action mapper, pas au modèle ;
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
