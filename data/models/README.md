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

L'export d'entraînement de référence vit pour l'instant dans :

```
data/ur3e_rollouts/2026-05-26_17-13-29_ppo_torch/exports/
  policy_deterministic.ts
  policy_deterministic.onnx
  policy_metadata.json
```

Lier le modèle choisi ici, par exemple :

```bash
ln -s ../ur3e_rollouts/2026-05-26_17-13-29_ppo_torch/exports/policy_deterministic.ts \
      data/models/policy_deterministic.ts
```
