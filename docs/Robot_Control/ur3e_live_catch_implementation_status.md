# UR3e Ball-Catch — État d'implémentation de la boucle live (passage 2)

> Statut (2026-06-22) : **étapes 1–8 de la feuille de route §11 implémentées et
> buildées sur la machine ROS Humble** ; **le robot UR3e est désormais disponible**,
> donc l'étape 9 (bring-up matériel) — outillée (launch + procédure + onglet **Test**
> du web UI) — est **prête à valider sur le robot réel** (E-stop en main, procédure §8).
> Le chemin de commande (ActionMapper → SafetyLimiter → CommandStreamer →
> `forward_position_controller`) est **câblé** derrière un flag `enable_command`
> (défaut **`false` = dry-run, rien ne bouge**), avec bascule de contrôleur
> automatique (§8) et watchdog d'arrêt contrôlé (§9), activable **à chaud** depuis
> l'onglet **Test** du web UI (service `~/enable_command`) ou au lancement. La
> visualisation balle + **fantôme policy** + télémétrie est ajoutée au web UI
> (§4.4). La mesure de latence bout-en-bout (§10) est en place.
>
> Vérifié **sur cette machine ROS** : `colcon build` des paquets, introspection
> du message, résolution complète des imports des nœuds, **smoke-test dry-run en
> process**, **smoke-test ROS des services de l'onglet Test** (`~/throw` arme/désarme
> un vol ; `~/enable_command` refusé proprement sans modèle), et — avec
> `torch 2.12.1+cpu` installé dans `.venv` (uv, py3.10) — **toute la chaîne réelle**
> `test_ball_node` (balle en `base`) → obs → **vraie policy** → safety, en dry-run
> (238 msgs en 4 s, actions non nulles, pass-through qui s'incrémente, cibles sûres
> bornées). **Question scaler tranchée** : la policy reproduit `action_normalized` à
> **max |Δ| = 4.6e-6** sans scaler externe → le `.ts` embarque le scaler, **aucun
> `policy_scaler.json` requis**. **À valider sur le robot (désormais disponible)** :
> commande réelle sur `forward_position_controller` (onglet Test ou `enable_command:=true`).
>
> Passage 1 (2026-06-20) : étapes 1–5 (dry-run), ActionMapper/safety/streaming
> conçus mais non câblés.

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
| 1 | `ur3e_catch_msgs` (`BallState`, `CatchTelemetry`) | ✅ implémenté (+ champs latence/vitesse balle) |
| 2 | `test_ball_node` + fallback legacy `Float32 → BallState` | ✅ implémenté |
| 3 | `ball_frame` conscient du repère + filtre vitesse | ✅ implémenté (TF statiques à fournir au lancement) |
| 4 | `ObservationBuilder` 33-D + test d'équivalence | ✅ implémenté ; comps 3/8/10 paramétrées, à figer (source Isaac) |
| 5 | `PolicyRunner` (question scaler) → action en dry-run | ✅ implémenté ; **scaler tranché** : pas de scaler externe requis (`.ts` l'embarque, Δ=4.6e-6) |
| 6 | `ActionMapper` + safety + streaming | ✅ **câblé** (flag `enable_command`, défaut dry-run) + `limits.py`/`streaming.py` + tests |
| 7 | Bascule contrôleur + viz web UI | ✅ bascule auto dans le nœud (§8) ; marqueur balle + arc prédit + télémétrie dans le web UI |
| 8 | Mesure de latence bout-en-bout | ✅ champs télémétrie + `latency.py` + nœud `latency_report` |
| 9 | Bring-up robot par étapes | 🟡 outillé (`live_catch.launch.py` + onglet **Test** du web UI + procédure §8) ; **robot disponible — à valider sur le robot réel** |

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
float32[]            joint_target   # 6 (cible sûre après clip+rate+accel ;
                                     #    renseignée même en dry-run, mais non émise)
geometry_msgs/Point  ball_base      # balle dans le repère base
```

Le paquet est buildable et visible par le producteur C++ (`Ball_Tracking_Cpp`,
qui publie maintenant `BallState` nativement) et le consommateur Python
(`ur3e_live_catch`) — workspace unique (§3 du plan).

---

## 4. Paquet `ur3e_live_catch` (`ament_python`)

Arborescence effective :
```
ur3e_live_catch/
  package.xml, setup.py, setup.cfg, pytest.ini, resource/ur3e_live_catch
  config/live_catch.yaml
  launch/test_dry_run.launch.py   launch/live_catch.launch.py
  ur3e_live_catch/
    joint_order.py       ball_frame.py     observation.py
    policy_runtime.py    action.py         safety.py
    limits.py            streaming.py      latency.py
    float32_adapter.py   test_ball_node.py live_catch_node.py
    latency_report.py
  test/
    conftest.py
    test_observation_equivalence.py  test_ball_frame.py
    test_action_mapper.py            test_safety.py
    test_streaming.py                test_limits.py
    test_command_pipeline.py         test_latency.py
    test_policy_equivalence.py
```

Entry points (`setup.py`) : `test_ball_node`, `float32_adapter`, `live_catch_node`,
`latency_report`.

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
- **Scaler résolu pour l'export courant** : `policy_metadata.json` ne porte pas
  mean/var, mais le test d'équivalence policy (§6) confirme que le `.ts`
  embarque la normalisation nécessaire ; aucun sidecar n'est requis pour ce
  modèle.

**`action.py`** (§4.3.4, **câblé**) — `ActionMapper(mode)` :
- `faithful` : `target = action × 0.5` (absolu, non clippé) ; mémorise l'action
  **brute** pour comp 9.
- `safe` : `target = q + clamp(action,−1,1)·v_safe·dt` ; mémorise l'action
  **clippée**. Nécessite `v_safe`.
- Ne **fait pas** respecter les bornes — c'est le rôle de `safety.py`.

**`safety.py`** (§4.3.5, §9, **câblé**) :
- `SafetyLimiter(bounds, dt)` : **clip** position (URDF) → **rate-limit**
  `|Δ| ≤ v_safe·dt` → **accel-limit** `|Δv| ≤ a_safe·dt` ; renvoie un `SafetyReport`
  (quelles contraintes ont mordu). `reset()` remet la mémoire de vitesse à zéro
  (appelé à chaque arrêt contrôlé pour redémarrer la rampe à l'arrêt).
- `Watchdog(stale_after_s, loop_budget_s, max_tracking_error)` : `check(...)`
  renvoie `(ok, reasons)` ; `ok=False` ⇒ arrêt contrôlé. `v_safe = limite URDF × 0.5`.

**`limits.py`** (§4.3.5, §9 ; nouveau) — bornes `SafetyLimiter` depuis les limites
articulaires :
- `build_joint_bounds(limits_by_name, v_safe_factor=0.5, a_safe)` → liste ordonnée
  de `JointBound` (`v_safe = max_velocity × facteur`, sim-to-real §2.1). Lève si un
  joint manque — **jamais** de borne silencieuse.
- `load_ur3e_joint_limits(path)` : lit `ur_description/.../joint_limits.yaml`
  (PyYAML paresseux, tag `!degrees`) ; **fallback** sur les limites nominales UR3e
  documentées (3.142 / 6.283 rad/s) si le fichier est absent → paquet autonome.

**`streaming.py`** (§4.3.6 ; nouveau) — `CommandStreamer` :
- `format(target)` → commande 6-D dans l'**ordre canonique** (payload
  `Float64MultiArray`). `stream(target)` peut sur-échantillonner (substeps,
  interpolation linéaire) ; le matériel interpole déjà 60 Hz → 500 Hz.
- `hold(fallback=q)` → répète la dernière commande (arrêt contrôlé : un
  contrôleur de position tient sa dernière consigne).

**`latency.py`** (§10, étape 8 ; nouveau) — `LatencyStats` : agrégats exacts
(count/mean/min/max) + percentiles p50/p95/p99 sur fenêtre glissante bornée, sans
numpy.

### 4.2 Nœuds rclpy (exécutables sur la machine ROS)

**`float32_adapter.py`** (§4.1, fallback legacy) — abonne `ball_position_3d_mm`
(`Float32MultiArray`, caméra, mm) → republie `BallState` (stamp = réception,
`frame_id` = caméra constante, mm→m). À réserver aux anciens builds du tracker ;
le chemin courant est la publication native `BallState` depuis `ball_tracking_cpp`.

**`test_ball_node.py`** (§4.2) — source de balle artificielle :
- param **`publish_frame`** = `base` **ou** `<camera_frame>` (`header.frame_id`
  toujours renseigné) ;
- sources `parabola` (analytique, repère `base` z-up) ou `csv`
  (`ground_truth.csv`, colonnes monde/caméra sélectionnées selon `publish_frame`) ;
- en mode caméra, la parabole passe par l'**inverse** de la pose caméra
  (`camera_translation`/`camera_quaternion`) → permet le **test de parité** §12 ;
- `noise_std` (bruit gaussien) et `dropout_prob` (trous) injectables.

**`live_catch_node.py`** (§2, §4.3) — **boucle live 60 Hz, dry-run OU commande** :
- abonne `/joint_states` (cache réordonné) et `ball_state` ; tf2 `Buffer`+`Listener` ;
- par tick : `ball_frame.process` (TF `frame_id→base`) → pose disque (TF
  `base→hoop_center`, sinon **fallback config**) → `ObservationBuilder.build` →
  `PolicyRunner.infer` → `ActionMapper.map` → `SafetyLimiter.limit` → publication
  `CatchTelemetry` (obs, action brute, **cible sûre**, balle base+vitesse, latence) ;
- **`enable_command` (défaut `false`)** : tout le pipeline tourne (la cible sûre
  est calculée et publiée en télémétrie) mais **aucune commande n'est émise**.
  `true` : `CommandStreamer` publie sur `forward_position_controller/commands`
  (`Float64MultiArray`, ordre canonique) ;
- **bascule de contrôleur** (§8) : si `auto_switch_controller`, le nœud appelle
  `controller_manager/switch_controller` (−`scaled_joint_trajectory_controller`
  +`forward_position_controller`) et **ne stream qu'une fois le contrôleur actif**
  (vérifié par `list_controllers`) ;
- **watchdog** (§9) : perception périmée / budget de boucle dépassé / erreur de
  suivi `|q − dernière commande|` ⇒ **arrêt contrôlé** (hold + `safety.reset()`) ;
- **garde-fou** : sans modèle policy chargé, le nœud **refuse de commander** (une
  action nulle serait une cible absolue dangereuse) et reste en dry-run ;
- l'action mémorisée par l'`ActionMapper` (brute en `faithful`, clippée en `safe`)
  est réinjectée comme **composante 9** au tick suivant ;
- modèle chargé **eagerly** depuis `data/models/` puis **fallback** sur l'export
  daté ; échec de chargement (torch/onnx absent, export invalide) → observation-
  seule (action = zéros) avec un warning, **au démarrage** (pas dans le timer).

**`latency_report.py`** (§10, étape 8) — abonne `catch_telemetry`, agrège
`perception_age_s` et `loop_compute_s` via `LatencyStats`, imprime un résumé
périodique (et final à l'arrêt) en ms. À comparer au budget de latence modélisé à
l'entraînement (sim-to-real §5.6).

### 4.3 Config & launch
- `config/live_catch.yaml` : sections `live_catch_node`, `test_ball_node`,
  `ball_float32_adapter`, `ball_tracking_cpp`. Contient `loop_hz: 60`, repères, géométrie disque
  **placeholder**, `model_path`, et les clés étape 6 **actives** : `enable_command`
  (défaut `false`), `action_mode`, `command_topic`, noms de contrôleurs,
  `auto_switch_controller`, `v_safe_factor`, `a_safe`, `loop_budget_s`,
  `max_tracking_error`, `joint_limits_path`.
- `launch/test_dry_run.launch.py` : `test_ball_node` (`publish_frame=base`) +
  `live_catch_node` ; argument CLI `publish_frame:=camera_optical` pour la parité.
- `launch/live_catch.launch.py` (nouveau) : point d'entrée unique de bring-up
  (§9). Arguments : `enable_command` (défaut `false`), `action_mode`, `model_path`,
  `use_tracker` (tracker C++ natif), `use_adapter` (fallback legacy),
  `use_test_ball` (balle simulée), `publish_frame`. La **même** config sert du
  dry-run sim au robot réel.

### 4.4 Visualisation web UI (étape 7, paquet `ur3e_web_ui`)
**Hors chemin critique** : pure télémétrie. Le bridge `ros_interface.py` abonne
`catch_telemetry` (`CatchTelemetry`) et range balle/vitesse/action/cible/latence
dans le `StateSnapshot` (drapeau `catch_alive`, péremption 0,5 s). `app.py` ajoute
une section `catch` au payload WebSocket. `viewer3d.js` rend un **marqueur balle**
(sphère) dans le repère `base` (même conversion base→three.js `(-x, z, y)` que les
repères TCP/caméra) et un **arc balistique prédit** (intégration `p + v t +
½ g t²`). `main.js` appelle `viewer.setCatch(state.catch)`.

**Onglet « Test » (étape 9 / UI).** Deux boutons pilotent la chaîne depuis l'UI :
- **Launch virtual ball** → service `~/throw` (`std_srvs/Trigger`) de `test_ball_node`
  (param `trigger_mode` : nœud inactif entre deux lancers, **un** vol de parabole par
  appel). Un **fantôme vert** (`policyGhost` de `viewer3d.js`) suit `joint_target` — la
  pose commandée par le réseau — superposé au robot live.
- **Run on real robot** (case de confirmation) → service `~/enable_command`
  (`std_srvs/SetBool`) de `live_catch_node` : bascule la commande **à chaud** (refus si
  aucun modèle chargé) ; **Stop / back to safe** restaure
  `scaled_joint_trajectory_controller`. `CatchTelemetry.command_enabled` remonte l'état
  réel ; le badge `catch:` de l'en-tête le reflète.

Backend : `POST /api/catch/throw` et `/api/catch/command` (`app.py`) ; clients de
service + `throw_ball()` / `set_catch_command()` dans `ros_interface.py`. La plomberie
`switch_controller` historique reste disponible pour un usage manuel.

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

### Tests de logique pure (Python stdlib, sans ROS/torch/numpy) — **42 OK, 1 skip**
- `test_observation_equivalence.py` — rejoue les rollouts ; vérifie longueur 33,
  ordre, **bit-proximité** comp 1/2, comp 5/6, **comp 9 (action brute précédente)**.
- `test_ball_frame.py` — identité si `base`, mm→m, rotation+translation, rejet
  frame vide/inconnu, vitesse par différence finie + invariance, staleness.
- `test_action_mapper.py` — faithful (`target=action×0.5`, comp 9 brute) ; safe
  (incrémental + clip, comp 9 clippée) ; **faithful reproduit
  `joint_position_target_rad`** des rollouts.
- `test_safety.py` — clip position, rate-limit, accel-limit, watchdog.
- `test_streaming.py` — `format`/`stream` (interpolation)/`hold` du `CommandStreamer`.
- `test_limits.py` — `v_safe = max_vel × 0.5` (1.571/…/3.142), ordre canonique,
  joint manquant ⇒ lève.
- `test_command_pipeline.py` — **pipeline composé** ActionMapper→Safety→Streamer :
  une cible **absolue agressive** (faithful) est **rate-limitée sans saut** et
  converge vers `action×0.5` ; clip position ; mode safe.
- `test_latency.py` — agrégats + percentiles nearest-rank + fenêtre bornée.
- `test_policy_equivalence.py` — **`skip` ici** (torch non installé sur cette
  machine) : voir point ouvert #1.

### Exécuté **sur cette machine ROS Humble** (nouveau dans ce passage)
- `colcon build --packages-select ur3e_catch_msgs ur3e_live_catch` ✅ (puis
  `ur3e_rollout_replay`, `ur3e_web_ui` pour la viz).
- `ros2 interface show ur3e_catch_msgs/msg/CatchTelemetry` ✅ — champs
  `ball_vel_base`, `perception_age_s`, `loop_compute_s`, `command_enabled` présents.
- **Résolution complète des imports** de `live_catch_node`, `latency_report`,
  `ur3e_web_ui.ros_interface`, `ur3e_web_ui.app` ✅ (rclpy, `controller_manager_msgs`,
  `std_msgs/Float64MultiArray`, `ur3e_catch_msgs`, modules `limits`/`streaming`).
- **Smoke-test dry-run en process** (nœud réel + stub `/joint_states`/`ball_state`)
  ✅ : `CatchTelemetry` publiée, obs 33-D, action 6-D, **cible sûre 6-D**, balle
  transformée (identité `base`), `ball_vel_base` par différence finie cohérente
  (≈ v0), `perception_age_s`/`loop_compute_s` renseignés. Fallback policy gracieux
  (torch absent → action zéros, pas de crash).
- `node --check` sur `viewer3d.js` / `main.js` / `catch_panel.js` ✅.
- **Smoke-test ROS de l'onglet Test** ✅ : `test_ball_node trigger_mode:=true` +
  `live_catch_node`, puis appels de service — `~/throw` arme un vol (balle
  `valid=False` → `True` avec parabole → `False` après `restart_after_s`),
  `~/enable_command(true)` **refusé** (`success=false`) faute de modèle en python
  système, `~/enable_command(false)` OK. Tests web UI : **24 OK** ; logique pure
  live-catch : **42 OK / 1 skip**.

### Exécuté **avec torch** (`.venv` uv, torch 2.12.1+cpu)
- **Équivalence policy** ✅ : la policy reproduit `action_normalized` enregistré à
  **max |Δ| = 4.6e-6** (50 échantillons) **sans scaler externe** → le `.ts`
  embarque le scaler, `test_policy_equivalence.py` passerait (`< 1e-2`).
- **Chaîne complète en process** ✅ : `test_ball_node` (parabole en `base`) →
  `live_catch_node` dry-run avec la **vraie policy** → 238 `CatchTelemetry` en 4 s,
  **toutes** avec action non nulle, `pass_through` 0→1→2, `joint_target` borné par
  la safety (action brute jusqu'à −16 → pas articulaire minuscule), `loop_compute`
  ≈ 1 ms, `perception_age` ≈ 31 ms.
- Note venv : `torch` est dans `.venv` (py3.10, compat ROS Humble) ; `rclpy` vient
  du `PYTHONPATH` ROS. PyYAML absent du venv → `load_ur3e_joint_limits` retombe sur
  les **limites nominales** (vitesses identiques à l'URDF ; positions ±2π). Ajouter
  `pyyaml` au venv pour des bornes de position exactes si on lance depuis le venv.

### À valider **sur le robot** (désormais disponible)
- Bring-up par étapes (séquence §8) avec **E-stop en main** ; `ros2 topic echo` ;
  **parité de repère** (base vs caméra, §12) ; **test watchdog** (couper la
  perception → hold) ; **commande réelle** sur `forward_position_controller`,
  déclenchée depuis l'onglet **Test** du web UI (*Run on real robot*) ou
  `enable_command:=true`. Vérifier la bascule (`ros2 control list_controllers` →
  `forward_position_controller` actif) et le retour au contrôleur de trajectoire au *Stop*.

---

## 7. Points ouverts / à finaliser

1. ~~**Scaler SKRL**~~ — **RÉSOLU** (2026-06-22, torch dans `.venv`) : la policy
   reproduit `action_normalized` à Δ=4.6e-6 **sans scaler externe** ; le `.ts`
   l'embarque. Aucun `policy_scaler.json` requis. *(Risque sim-to-real majeur levé.)*
2. **Composantes obs 3 / 8 / 10** — figer d'après la **source Isaac**
   (`_read_disk_pose_in_body_frame`, signe `(ball−disk)·normale`,
   `detect_pass_through`) ; renseigner la géométrie disque réelle (montage). Voir
   les `TODO(isaac)` dans `observation.py`.
3. **TF statiques** — publier `base → <camera_frame>` (hand-eye) et
   `wrist_3_link → hoop_center` (montage) avant tout test réaliste.
4. **`a_safe` / `loop_budget_s`** — valeurs par défaut conservatrices à **caler sur
   le matériel** (l'accélération sûre n'est pas dans l'URDF ; le budget de boucle
   dépend du temps d'inférence torch réel).
5. **Étape 9 — bring-up matériel** — **robot disponible, à valider** : enchaîner
   perception seule → dry-run → balle lente E-stop en main → montée en vitesse
   (séquence §8 ci-dessous, pilotable depuis l'onglet **Test**). Vérifier
   `forward_position_controller` **spawné** (`ros2 control list_controllers`).
6. **Horodatage événement** — le tracker C++ natif remplit `BallState.header.stamp`
   depuis `BallPose3D.timestampUs` ancré sur l'horloge ROS. Le fallback
   `float32_adapter.py` timestamp encore à la réception.

---

## 8. Construire & exécuter

```bash
# Sur la machine ROS Humble, à la racine du workspace (Dv-Rosws) :
colcon build --packages-select ur3e_catch_msgs ur3e_live_catch
source install/setup.bash

# (optionnel) lier le modèle canonique :
ln -s ../ur3e_rollouts/2026-05-26_17-13-29_ppo_torch/exports/policy_deterministic.ts \
      data/models/policy_deterministic.ts

# Démarrer le driver UR (robot réel) pour fournir /joint_states + contrôleurs ;
# en l'absence de robot, use_fake_hardware / URSim font l'affaire.
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=<IP> launch_rviz:=false

# Dry-run (rien ne bouge) :
ros2 launch ur3e_live_catch live_catch.launch.py use_test_ball:=true enable_command:=false
ros2 topic echo /catch_telemetry
ros2 run ur3e_live_catch latency_report          # budget de latence (§10)

# Bring-up par étapes (§8, ci-dessous) — GARDER L'E-STOP en main :
#  a) commande sur le robot (ou fake hardware / URSim), balle simulée :
ros2 launch ur3e_live_catch live_catch.launch.py use_test_ball:=true enable_command:=true
#  b) perception réelle (tracker C++ -> BallState natif) + commande :
ros2 launch ur3e_live_catch live_catch.launch.py use_tracker:=true enable_command:=true
#  c) piloté depuis le web UI (onglet Test) — balle à la demande + activation à chaud :
#     IMPORTANT : lancer live_catch sous le .venv (torch) pour que la policy infère.
ros2 launch ur3e_live_catch live_catch.launch.py use_test_ball:=true trigger_mode:=true
ros2 run ur3e_web_ui ur3e_web_ui   # onglet "Test" : Launch virtual ball / Run on real robot

# Tests de logique pure (stdlib) — note: le pytest système entre en conflit avec
# des plugins ROS, d'où PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 :
cd src/ur3e_live_catch && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

### Étape 9 — séquence de bring-up (procédure du plan §9, §11.9)
1. **Perception seule** : vérifier `ball_state` (echo + marqueur balle web UI).
2. **Dry-run** (`enable_command:=false`) : inspecter `catch_telemetry`
   (obs/action/cible sûre), vérifier la latence ≤ budget modélisé.
3. **Approche** vers la pose de départ avec le `scaled_joint_trajectory_controller`
   (web UI / `ur3e_rollout_replay`), déjà sûr.
4. **Commande, fake hardware / URSim** (`enable_command:=true`, ou bouton *Run on
   real robot* de l'onglet Test) : le nœud bascule sur `forward_position_controller`
   et stream ; **tester le watchdog** (couper la perception → hold) ; *Stop* restaure
   `scaled_joint_trajectory_controller`.
5. **Robot réel** (désormais disponible) **, vitesse réduite, E-stop en main, sans
   vraie balle** (balle virtuelle via l'onglet Test), puis balle lente, puis montée
   en vitesse de balle.

---

## 9. Récapitulatif réutilisation
- Pattern paquet `ament_python`, joint order : `src/ur3e_rollout_replay/`.
- Bridge rclpy, `switch_controller`, gates sécurité (étapes 6+) :
  `src/ur3e_web_ui/ur3e_web_ui/ros_interface.py`, `app.py`, `joint_limits.py`.
- Vérité terrain (équivalence obs/policy) :
  `data/ur3e_rollouts/2026-05-26_17-13-29_ppo_torch/exports/rollouts_10_episodes.json`.
- Modèle + métadonnées : `data/models/` → export
  `.../exports/policy_deterministic.{ts,onnx}`, `policy_metadata.json`.
