# ur3e_live_catch

Nœud **live mono-processus** (boucle 60 Hz) + nœud de test, sécurité, runtime
politique et streaming. Voir
`docs/Robot_Control/ur3e_live_catch_architecture.md` §4.3.

Type de paquet : **`ament_python`** (miroir de `ur3e_rollout_replay` /
`ur3e_web_ui`).

## Arborescence

```
ur3e_live_catch/
  ur3e_live_catch/      # modules Python (boucle + nœud de test)
  launch/               # fichiers launch (live, test, fake_hardware/URSim)
  config/               # paramètres YAML (cadence, limites sûres, repères)
  resource/             # marqueur ament (resource/ur3e_live_catch)
  test/                 # tests (équivalence obs, dry-run, parité repère)
```

## Modules prévus (`ur3e_live_catch/`)

| Fichier | Rôle (réf. archi) |
|---|---|
| `live_catch_node.py` | nœud rclpy, boucle 60 Hz qui appelle les modules ci-dessous |
| `test_ball_node.py` | source de balle artificielle, `publish_frame` = `base` \| `<camera_frame>` (§4.2) |
| `ball_frame.py` | `frame_id → base` via TF (identité si `base`) + filtre vitesse (§4.3.1) |
| `observation.py` | `ObservationBuilder` 33-D (§4.3.2, §6) |
| `policy_runtime.py` | `PolicyRunner`, charge le modèle depuis `data/models/` (§4.3.3) |
| `action.py` | `ActionMapper` (×0.5, mémorise l'action brute) (§4.3.4) |
| `safety.py` | `SafetyLimiter` + watchdog (§4.3.5, §9) |
| `streaming.py` | sortie vers `forward_position_controller` (§4.3.6) |

## Modèle IA

Chargé depuis `data/models/` (voir `data/models/README.md`).

## À ajouter à l'implémentation

`package.xml`, `setup.py`, `setup.cfg`, `resource/ur3e_live_catch`,
`__init__.py`, puis les modules.
