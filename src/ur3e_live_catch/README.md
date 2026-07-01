# ur3e_live_catch

Nœud **live mono-processus** (boucle 60 Hz) perception → policy → robot, plus
source de balle de test, sécurité, runtime politique, streaming et mesure de
latence. Voir `docs/Robot_Control/ur3e_live_catch_architecture.md` §4.3 et
`docs/Robot_Control/ur3e_live_catch_implementation_status.md` pour l'état détaillé.

Type de paquet : **`ament_python`** (miroir de `ur3e_rollout_replay` /
`ur3e_web_ui`).

## Arborescence

```
ur3e_live_catch/
  ur3e_live_catch/      # modules Python (logique pure + nœuds rclpy)
  launch/               # test_dry_run.launch.py, live_catch.launch.py
  config/               # live_catch.yaml (cadence, repères, safety, action_mode)
  resource/             # marqueur ament (resource/ur3e_live_catch)
  test/                 # tests stdlib (obs, ball_frame, action, safety,
                        #   streaming, limits, command_pipeline, latency, policy)
```

## Modules (`ur3e_live_catch/`)

| Fichier | Rôle (réf. archi) |
|---|---|
| `joint_order.py` | ordre articulaire canonique + réordonnancement `/joint_states` |
| `ball_frame.py` | `frame_id → base` via TF (identité si `base`) + filtre vitesse (§4.3.1) |
| `observation.py` | `ObservationBuilder` 33-D (§4.3.2, §6) |
| `policy_runtime.py` | `PolicyRunner` (torch/onnx) + `ObsScaler` (§4.3.3) |
| `action.py` | `ActionMapper` `faithful` \| `safe` (§4.3.4) |
| `safety.py` | `SafetyLimiter` (clip + rate + accel) + `Watchdog` (§4.3.5, §9) |
| `limits.py` | bornes `SafetyLimiter` depuis les limites URDF (`v_safe = max_vel·facteur`) |
| `streaming.py` | `CommandStreamer` → `forward_position_controller` (§4.3.6) |
| `latency.py` | `LatencyStats` (budget de latence, §10) |
| `live_catch_node.py` | nœud rclpy 60 Hz : dry-run **ou** commande (flag `enable_command`) |
| `test_ball_node.py` | source de balle artificielle, `publish_frame` = `base` \| `<camera_frame>` (§4.2) |
| `float32_adapter.py` | fallback legacy `Float32MultiArray → BallState` (§4.1) |
| `latency_report.py` | nœud d'agrégation latence (`catch_telemetry` → percentiles) |

## Lancer

```bash
colcon build --packages-select ur3e_catch_msgs ur3e_live_catch
source install/setup.bash

# 1) Dry-run (rien ne bouge), balle simulée — fournir /joint_states à part :
ros2 launch ur3e_live_catch live_catch.launch.py use_test_ball:=true enable_command:=false
ros2 topic echo /catch_telemetry

# 2) Commande sur fake hardware / URSim, balle simulée :
ros2 launch ur3e_live_catch live_catch.launch.py use_test_ball:=true enable_command:=true

# 3) Perception réelle (tracker C++ natif → BallState) + commande — GARDER L'E-STOP :
ros2 launch ur3e_live_catch live_catch.launch.py use_tracker:=true enable_command:=true

# Fallback legacy si un ancien tracker ne publie que ball_position_3d_mm :
ros2 launch ur3e_live_catch live_catch.launch.py use_adapter:=true enable_command:=true

# 4) Stack robot réel + UI + balle virtuelle à la demande (défaut : dry-run) :
ros2 launch ur3e_live_catch virtual_ball_robot.launch.py

# Latence :
ros2 run ur3e_live_catch latency_report

# Tests de logique pure (stdlib) :
cd src/ur3e_live_catch && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

Ne pas lancer `use_tracker:=true` et `use_adapter:=true` ensemble : les deux
publieraient sur `ball_state`.

`enable_command:=false` est le **défaut sûr** : tout le pipeline tourne et publie
`CatchTelemetry`, mais aucune commande n'est émise. En mode commande, le nœud
bascule automatiquement le contrôleur (`scaled_joint_trajectory_controller` →
`forward_position_controller`, §8) et applique le watchdog (arrêt contrôlé, §9).

## Launch robot réel + balle virtuelle

`virtual_ball_robot.launch.py` démarre le driver UR3e réel, MoveIt, la chaîne
`test_ball_node` → `live_catch_node` et l'UI web :

```bash
source env.sh
ur3e_catch_stack
```

Ouvrir ensuite `http://127.0.0.1:8080`, onglet **Test**. Le repère cyan dans
la vue 3D est le point de lancement `p0` de la balle virtuelle ; il peut être
déplacé à la souris ou réglé par les champs `X/Y/Z`. Les champs `Vx/Vy/Vz`
règlent la vitesse initiale `v0`, puis **Launch virtual ball** applique ces
valeurs au `test_ball_node` et lance un vol. Les défauts suivent la configuration
locale documentée : `robot_ip:=192.168.0.5`, `reverse_ip:=192.168.0.3`, balle en
`publish_frame:=base` et `trigger_mode:=true`. Le lancement reste en
`enable_command:=false` par défaut :
le robot ne reçoit aucune commande tant que l'UI n'active pas explicitement
**Run on real robot** avec confirmation E-stop/workspace.

Arguments utiles :

```bash
ur3e_catch_stack --help
ur3e_catch_stack --fake
ur3e_catch_stack --no-moveit --port 8081
ur3e_catch_stop

ros2 launch ur3e_live_catch virtual_ball_robot.launch.py --show-args
ros2 launch ur3e_live_catch virtual_ball_robot.launch.py use_fake_hardware:=true
ros2 launch ur3e_live_catch virtual_ball_robot.launch.py launch_moveit:=false ui_port:=8081
```

Le launch utilise `$HOME/ur3e_calibration.yaml` s'il existe ; sinon il laisse le
driver charger la cinématique par défaut de `ur_description`. `publish_hoop_tf`
reste `false` par défaut : ne le passer à `true` avec `hoop_xyz` / `hoop_quat`
qu'après mesure réelle de la géométrie `wrist_3_link -> hoop_center`.

## UI « Test » (balle virtuelle → robot)

L'onglet **Test** de `ur3e_web_ui` pilote la chaîne sans ligne de commande, via deux
services ajoutés sur les nœuds :

- Repère cyan **Launch frame** → paramètres `p0` / `v0` du `test_ball_node`
  via ses services ROS 2 de paramètres. Le repère se déplace dans la vue 3D ;
  la flèche jaune prévisualise la vitesse initiale.
- **Launch virtual ball** → applique `p0` / `v0`, puis appelle `~/throw`
  (`std_srvs/Trigger`) sur `test_ball_node`. Avec `trigger_mode:=true`, le nœud
  reste **inactif** (`valid=False`) entre deux lancers et ne renvoie qu'**un**
  vol de parabole par appel. Le **fantôme vert** de la vue 3D suit `joint_target`
  (la pose commandée par le réseau) ; le marqueur rouge + l'arc, la balle.
- **Run on real robot** (case de confirmation) → `~/enable_command` (`std_srvs/SetBool`)
  sur `live_catch_node` : bascule la commande robot **à chaud** (sans relancer le nœud) ;
  **Stop / back to safe** la coupe et restaure `scaled_joint_trajectory_controller`.
  Refusé (`success=false`) si aucun modèle de politique n'est chargé.

Pour que le fantôme bouge réellement, `live_catch_node` doit avoir `onnxruntime`
ou `torch`. Par défaut il préfère `data/models/policy_deterministic.onnx`, donc
`onnxruntime` suffit sur le PC ROS. Bring-up type :
`... live_catch.launch.py use_test_ball:=true trigger_mode:=true` puis
`ros2 run ur3e_web_ui ur3e_web_ui` et ouvrir l'onglet **Test**.

## Modèle IA

Chargé depuis `data/models/` puis l'export daté en fallback (voir
`data/models/README.md`). Sans torch/onnx installé, le nœud tourne en
observation-seule (action = zéros) et **refuse de commander**.
