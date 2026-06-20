# UR3e Ball-Catch — État d'implémentation de la boucle live (passage 1)

> Statut (2026-06-20) : **étapes 1–5 de la feuille de route §11 implémentées**
> (contrat de messages, source de balle de test, transform de repère,
> observation 33-D, runtime policy en **dry-run**). Aucune commande robot n'est
> émise. ActionMapper / safety / streaming (étape 6+) sont **conçus mais non
> câblés**.

Ce document décrit **précisément ce qui existe dans le code à ce jour**, ce qui a
été **vérifié**, ce qui reste **ouvert**, et comment construire/exécuter. Il
complète (et n'remplace pas) le plan d'architecture
`ur3e_live_catch_architecture.md`, qui reste la référence de conception.

Documents liés :
- `ur3e_live_catch_architecture.md` — plan d'architecture (référence de conception).
- `../Context/synthese_projet.md` — synthèse générale du projet.
- `ur3e_ball_catch_sim_to_real.md` — contraintes sim/inférence.

---

## 1. Résumé exécutif

La boucle live (perception → policy → robot) est désormais **amorcée** dans le
workspace ROS (`Dv-Rosws`, c'est-à-dire ce dépôt). Deux paquets, jusqu'ici de
simples squelettes (READMEs), contiennent maintenant du code :

- **`src/ur3e_catch_msgs`** — contrat de messages typés/horodatés (`BallState`,
  `CatchTelemetry`). **Complet.**
- **`src/ur3e_live_catch`** — paquet Python : modules de logique pure
  (repère/observation/policy/action/safety), trois nœuds rclpy (adaptateur,
  source de test, boucle live dry-run), config et launch, plus une batterie de
  tests. **Chemin perception→policy complet, en dry-run.**

Le **chemin chaud** `observation → inférence` tourne par **appels directs** dans
un seul processus (pas de topic ROS interne), conformément à l'architecture
mono-processus retenue (§2 du plan).

État global de la feuille de route (§11 du plan) :

| Étape | Sujet | État |
|---|---|---|
| 1 | `ur3e_catch_msgs` (`BallState`, `CatchTelemetry`) | ✅ implémenté |
| 2 | `test_ball_node` + adaptateur `Float32 → BallState` | ✅ implémenté |
| 3 | `ball_frame` conscient du repère + filtre vitesse | ✅ implémenté (TF statiques à fournir au lancement) |
| 4 | `ObservationBuilder` 33-D + test d'équivalence | ✅ implémenté ; comps 3/8/10 paramétrées, à figer (source Isaac) |
| 5 | `PolicyRunner` (question scaler) → action en dry-run | ✅ implémenté ; scaler à trancher (test torch) |
| 6 | `ActionMapper` + safety + streaming | 🟡 conçu (modules+tests), **non câblé** |
| 7 | Bascule contrôleur + viz web UI | ⬜ à faire |
| 8 | Mesure de latence bout-en-bout | ⬜ à faire |
| 9 | Bring-up robot par étapes | ⬜ à faire |

---

## 2. Décisions verrouillées (Q&A du 2026-06-20)

1. **Périmètre de ce passage = étapes 1–5** (pas de mouvement robot).
2. **Mapping action / observation = fidélité sim + safety indépendante, via un
   flag `action_mode = faithful | safe` (défaut `faithful`)**.
   - Conséquence *en vigueur dès maintenant* : **l'observation composante 9 = action
     policy BRUTE précédente** (non clippée), comme à l'entraînement.
   - Le mapping commande lui-même (faithful = cible absolue `action×0.5` ;
     safe = incrémental `q + clamp(action,-1,1)·v_safe·dt`) appartient à
     l'**ActionMapper (étape 6)**.
3. **Source Isaac fournie par l'utilisateur** (`firsttraining_env.py`,
   `firsttraining_env_cfg.py`) → composantes obs **3 / 8 / 10** finalisées exactement
   d'après ces fichiers. En attendant, elles sont **isolées et paramétrées**.

### Découverte clé (confirmée sur les rollouts enregistrés)
Les données `rollouts_10_episodes.json` ont permis de **vérifier la sémantique
réelle** de la policy, ce qui a tranché deux ambiguïtés du plan :
- `joint_position_target_rad == action_normalized × 0.5` → la cible sim est
  **absolue et non clippée** (le plan §4.3.4 décrivait une variante incrémentale
  *de sécurité*, différente — d'où le flag).
- `observation[26:32]` (composante 9) = action **brute** précédente (valeurs hors
  `[-1,1]`, p.ex. −4.2) → comp 9 n'est **pas** clippée (le plan §6 disait
  « clippée »). Le code suit la donnée : **brute**.

---

## 3. Paquet `ur3e_catch_msgs` (rosidl / `ament_cmake`)

Fichiers : `package.xml`, `CMakeLists.txt`, `msg/BallState.msg`,
`msg/CatchTelemetry.msg`.

**`BallState.msg`** — pose balle horodatée, partagée perception ↔ commande :
```
std_msgs/Header        header       # stamp = temps d'événement ; frame_id = repère DÉCLARÉ
geometry_msgs/Point    position     # mètres
geometry_msgs/Vector3  velocity     # m/s (optionnel ; sinon recalculée côté ball_frame)
bool                   valid
float32                confidence
```

**`CatchTelemetry.msg`** — debug/visualisation, hors chemin critique :
```
float32[]            observation    # 33
float32[]            raw_action     # 6 (sortie policy, avant mapping/clip)
float32[]            joint_target   # 6 (après clip+rate-limit ; VIDE en dry-run)
geometry_msgs/Point  ball_base      # balle dans le repère base
```

Le paquet est buildable et visible par le producteur C++ (`Ball_Tracking_Cpp`,
futur) et le consommateur Python (`ur3e_live_catch`) — workspace unique (§3 du plan).

---

## 4. Paquet `ur3e_live_catch` (`ament_python`)

Arborescence effective :
```
ur3e_live_catch/
  package.xml, setup.py, setup.cfg, pytest.ini, resource/ur3e_live_catch
  config/live_catch.yaml
  launch/test_dry_run.launch.py
  ur3e_live_catch/
    joint_order.py       ball_frame.py     observation.py
    policy_runtime.py    action.py         safety.py
    float32_adapter.py   test_ball_node.py live_catch_node.py
  test/
    conftest.py
    test_observation_equivalence.py  test_ball_frame.py
    test_action_mapper.py            test_safety.py
    test_policy_equivalence.py
```

Entry points (`setup.py`) : `test_ball_node`, `float32_adapter`, `live_catch_node`.

### 4.1 Modules de logique pure (sans rclpy / sans numpy → testables hors ROS)

**`joint_order.py`** — constante canonique `JOINT_ORDER`
(`shoulder_pan_joint … wrist_3_joint`, identique à `policy_metadata.json` et
`replay_core.py:14`) + `reorder_by_name(names, values)` qui réordonne
`/joint_states` (ordre non garanti) vers l'ordre canonique. Lève
`JointOrderError` si un joint manque — **jamais** de zéro silencieux.

**`ball_frame.py`** (§4.3.1) — conscience du repère + vitesse :
- `RigidTransform(translation, quaternion)` : `P^target = R·P^source + t`
  (exactement ce que retourne un lookup tf2). Rotation par quaternion en pur Python.
- `BallVelocityFilter(ema_alpha, max_dt=0.5)` : vitesse par **différence finie**
  des positions `base` + lissage EMA ; garde `max_dt` (gap trop grand → ré-ancre,
  ne calcule pas). La vitesse `base` **est** la vitesse monde (translation
  d'origine constante, §6 du plan).
- `BallFrameTransformer(base_frame, units, stale_after_s)` :
  - `to_base(position, frame_id, transform)` → **identité si `frame_id == base`**,
    sinon applique la transformée fournie ; convertit mm→m ; **rejette** un
    `frame_id` vide (`FrameError`) ou un repère non-`base` sans transformée.
  - `process(...)` renvoie `(pos_base, vel_base)` et met à jour le filtre.
  - `is_stale(now_s)` → drapeau watchdog (péremption).

**`observation.py`** (§4.3.2, §6) — `ObservationBuilder`, vecteur **33-D** dans
l'ordre exact, avec les indices de tranche exposés en constantes (`IDX_*`) :

| Tranche | # | Composante | Source |
|---|---|---|---|
| `[0:6]` | 1 | `joint_pos` | `/joint_states` réordonné |
| `[6:12]` | 2 | `joint_vel` | `/joint_states` |
| `[12:15]` | 3 | `disk_pos_local` | pose disque (TF) — *à figer Isaac* |
| `[15:18]` | 4 | `ball_pos_local` | `ball_frame` |
| `[18:21]` | 5 | `direction` = ball−disk | dérivé |
| `[21]` | 6 | `distance` = ‖direction‖ | dérivé |
| `[22:25]` | 7 | `ball_vel_w` | vitesse filtrée |
| `[25]` | 8 | `prev_signed_dist > 0` | signe — *à figer Isaac* |
| `[26:32]` | 9 | `actions` (brute précédente) | mémorisé |
| `[32]` | 10 | `pass_through_count` | recompté — *à figer Isaac* |

- Méthodes **isolées** : `disk_pos_local`, `signed_distance`, `_update_pass_through`
  (marquées `TODO(isaac)`). État interne : `_prev_signed_dist`, `_pass_through_count`.
- Composante 8 lit le signe **du tick précédent** ; l'état avance **après** émission.
- Unités strictes (rad, rad/s, m, m/s). Lève si longueur ≠ 33.

**`policy_runtime.py`** (§4.3.3) — `PolicyRunner` :
- Charge `policy_deterministic.ts` (torch) **ou** `.onnx` (onnxruntime), **imports
  backend paresseux** (le module s'importe sans torch ni onnx).
- `ObsScaler(mean, var)` : `(obs − mean)/√(var+eps)`, chargeable depuis
  `policy_scaler.json` ; appliqué **avant** le réseau **si fourni**.
- `infer(obs33) -> action6`. `load_metadata()` lit `policy_metadata.json`.
- ⚠️ **Question scaler ouverte** : `policy_metadata.json` ne porte pas mean/var →
  soit le `.ts` embarque le scaler, soit il faut un sidecar. **Tranché par le test
  d'équivalence policy** (§6 ci-dessous) sur la machine ROS.

**`action.py`** (§4.3.4, **conçu, non câblé**) — `ActionMapper(mode)` :
- `faithful` : `target = action × 0.5` (absolu, non clippé) ; mémorise l'action
  **brute** pour comp 9.
- `safe` : `target = q + clamp(action,−1,1)·v_safe·dt` ; mémorise l'action
  **clippée**. Nécessite `v_safe`.
- Ne **fait pas** respecter les bornes — c'est le rôle de `safety.py`.

**`safety.py`** (§4.3.5, §9, **conçu, non câblé**) :
- `SafetyLimiter(bounds, dt)` : **clip** position (URDF) → **rate-limit**
  `|Δ| ≤ v_safe·dt` → **accel-limit** `|Δv| ≤ a_safe·dt` ; renvoie un `SafetyReport`
  (quelles contraintes ont mordu).
- `Watchdog(stale_after_s, loop_budget_s, max_tracking_error)` : `check(...)`
  renvoie `(ok, reasons)` ; `ok=False` ⇒ arrêt contrôlé. `v_safe = limite URDF × 0.5`.

### 4.2 Nœuds rclpy (exécutables sur la machine ROS)

**`float32_adapter.py`** (§4.1, intérim) — abonne `ball_position_3d_mm`
(`Float32MultiArray`, caméra, mm) → republie `BallState` (stamp = réception,
`frame_id` = caméra constante, mm→m). Permet de brancher le tracker C++ existant
**sans toucher à son build**.

**`test_ball_node.py`** (§4.2) — source de balle artificielle :
- param **`publish_frame`** = `base` **ou** `<camera_frame>` (`header.frame_id`
  toujours renseigné) ;
- sources `parabola` (analytique, repère `base` z-up) ou `csv`
  (`ground_truth.csv`, colonnes monde/caméra sélectionnées selon `publish_frame`) ;
- en mode caméra, la parabole passe par l'**inverse** de la pose caméra
  (`camera_translation`/`camera_quaternion`) → permet le **test de parité** §12 ;
- `noise_std` (bruit gaussien) et `dropout_prob` (trous) injectables.

**`live_catch_node.py`** (§2, §4.3) — **boucle live DRY-RUN, 60 Hz** :
- abonne `/joint_states` (cache réordonné) et `ball_state` ; tf2 `Buffer`+`Listener` ;
- par tick : `ball_frame.process` (TF `frame_id→base`) → pose disque (TF
  `base→hoop_center`, sinon **fallback config**) → `ObservationBuilder.build` →
  `PolicyRunner.infer` → **log de l'action brute** + publication `CatchTelemetry` ;
- **aucune commande robot** ; l'action brute est réinjectée comme **composante 9**
  au tick suivant ;
- modèle chargé depuis `data/models/` puis **fallback** sur l'export daté ; si
  aucun modèle, tourne en observation-seule (action = zéros) avec un warning.

### 4.3 Config & launch
- `config/live_catch.yaml` : sections `live_catch_node`, `test_ball_node`,
  `ball_float32_adapter`. Contient `loop_hz: 60`, repères, géométrie disque
  **placeholder**, `model_path`, et les clés safety/`action_mode` **réservées**
  (étape 6).
- `launch/test_dry_run.launch.py` : `test_ball_node` (`publish_frame=base`) +
  `live_catch_node` ; argument CLI `publish_frame:=camera_optical` pour la parité.

---

## 5. Cohérence des repères (rappel des pièges, §7 du plan)
- **Un seul `<camera_frame>`** : le tracker publie dedans, le hand-eye `{}^B T_C`
  est résolu dedans, publié en **TF statique**. À fournir au lancement.
- **`base` vs `base_link`** : rotation π autour de Z. Le « local » de la sim =
  `base`. Le code construit l'obs dans **`base`** (param `base_frame`).
- **`hoop_center`** : TF statique `wrist_3_link → hoop_center` (offset+normale du
  montage). Tant qu'elle n'est pas publiée, `live_catch_node` utilise le **fallback**
  config (placeholder) — à remplacer par la vraie géométrie.

---

## 6. Vérification

### Exécuté **ici** (Python stdlib, sans ROS/torch/numpy) — **19 tests OK**
- `test_observation_equivalence.py` — rejoue les rollouts ; vérifie longueur 33,
  ordre, et **bit-proximité** sur comp 1/2 (vs `joint_position_before_rad` /
  `velocity`), comp 5/6 (dérivations), **comp 9 (action brute précédente)**.
  → **Confirme** que la donnée enregistrée correspond à notre reconstruction.
- `test_ball_frame.py` — identité si `base`, mm→m, rotation+translation, rejet
  frame vide/inconnu, vitesse par différence finie + invariance par translation,
  staleness.
- `test_action_mapper.py` — faithful (`target=action×0.5`, comp 9 brute) ; safe
  (incrémental + clip, comp 9 clippée) ; **faithful reproduit
  `joint_position_target_rad`** des rollouts.
- `test_safety.py` — clip position, rate-limit, accel-limit, watchdog (stale /
  overrun / tracking error).
- `py_compile` **OK** sur **tous** les fichiers, y compris les nœuds rclpy
  (validation de syntaxe ; les imports ROS se résolvent sur la machine ROS).

### À exécuter **sur la machine ROS Humble** (différé ici)
- `test_policy_equivalence.py` (**`skip` si torch absent**) — charge le `.ts`,
  alimente l'`observation` enregistrée, compare à `action_normalized`. Un écart
  serré ⇒ scaler OK ; un écart large ⇒ **scaler manquant** → fournir
  `policy_scaler.json` et passer un `ObsScaler` au `PolicyRunner`.
- `colcon build` des deux paquets ; lancement du dry-run ; `ros2 topic echo` ;
  **parité de repère** (base vs caméra, §12).

---

## 7. Points ouverts / à finaliser

1. **Scaler SKRL** (§4.3.3) — trancher via `test_policy_equivalence.py` (torch).
   *Risque de divergence sim-to-real majeur si non résolu.*
2. **Composantes obs 3 / 8 / 10** — figer d'après la **source Isaac**
   (`_read_disk_pose_in_body_frame`, signe `(ball−disk)·normale`,
   `detect_pass_through`) ; renseigner la géométrie disque réelle (montage). Voir
   les `TODO(isaac)` dans `observation.py`.
3. **TF statiques** — publier `base → <camera_frame>` (hand-eye) et
   `wrist_3_link → hoop_center` (montage) avant tout test réaliste.
4. **Étape 6** — câbler `ActionMapper` (flag `action_mode`) + `SafetyLimiter` +
   streaming vers `forward_position_controller` (bascule de contrôleur via la
   plomberie existante de `ros_interface.py`).
5. **Horodatage événement** — `BallState.header.stamp` est pour l'instant le temps
   de réception (adaptateur/test node) ; migrer vers le temps d'événement pour le
   budget de latence (étape 8).

---

## 8. Construire & exécuter

```bash
# Sur la machine ROS Humble, à la racine du workspace (Dv-Rosws) :
colcon build --packages-select ur3e_catch_msgs ur3e_live_catch
source install/setup.bash

# (optionnel) lier le modèle canonique :
ln -s ../ur3e_rollouts/2026-05-26_17-13-29_ppo_torch/exports/policy_deterministic.ts \
      data/models/policy_deterministic.ts

# Dry-run (fournir /joint_states via use_fake_hardware/URSim séparément) :
ros2 launch ur3e_live_catch test_dry_run.launch.py
ros2 topic echo /catch_telemetry
ros2 topic echo /ball_state

# Tests de logique pure (stdlib) :
cd src/ur3e_live_catch && python3 -m pytest test/ -q
```

---

## 9. Récapitulatif réutilisation
- Pattern paquet `ament_python`, joint order : `src/ur3e_rollout_replay/`.
- Bridge rclpy, `switch_controller`, gates sécurité (étapes 6+) :
  `src/ur3e_web_ui/ur3e_web_ui/ros_interface.py`, `app.py`, `joint_limits.py`.
- Vérité terrain (équivalence obs/policy) :
  `data/ur3e_rollouts/2026-05-26_17-13-29_ppo_torch/exports/rollouts_10_episodes.json`.
- Modèle + métadonnées : `data/models/` → export
  `.../exports/policy_deterministic.{ts,onnx}`, `policy_metadata.json`.
