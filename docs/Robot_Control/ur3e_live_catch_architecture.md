# UR3e Ball-Catch — Architecture des nœuds ROS 2 (boucle live closed-loop)

Ce document est le **plan d'architecture** des nœuds ROS 2 à construire pour connecter la
perception (caméra événementielle DVXplorer) à la commande du UR3e, afin d'**attraper une balle
lancée** avec la politique PPO entraînée en simulation. Il détaille chaque nœud/module, les
interfaces (topics, messages, repères, taux), les contraintes techniques, une feuille de route
d'implémentation, la vérification, et des architectures alternatives.

> Statut (2026-06-17) : **design, pas encore implémenté.** Les paquets `ur3e_catch_msgs` et
> `ur3e_live_catch` n'existent pas encore. Ce document est la référence d'exécution.

Documents liés :
- `ur3e_ball_catch_sim_to_real.md` — contraintes sim/entraînement/inférence (ce document
  implémente concrètement son §5 « inférence closed-loop »).
- `ur3e_robot_control_architecture.md` — stack de contrôle existante (web UI, bridge, replay).
- `ur3e_real_robot_replay.md` — replay open-loop (validation, complémentaire).
- `ur3e_camera_base_calibration.md` — hand-eye `{}^{B}T_C` (balle repère caméra → repère `base`).
- Perception : `Dv-Rosws/Stage_summary.tex` (méthode Trace, régression, calibration extrinsèque).

---

## 1. Vue d'ensemble et contraintes directrices

L'objectif est une **boucle fermée temps réel** : la caméra estime la balle en 3D, le système la
transforme dans le repère robot, reconstruit l'observation attendue par la politique, infère une
action, la convertit en cible articulaire sûre, et la stream au robot — le tout assez vite pour
intercepter la balle.

Quatre contraintes dominent toute la conception :

1. **Latence perception→commande = risque n°1.** Soulignée à la fois par le sim-to-real (§1,
   §5.6) et le Stage_summary (table « Défis pour l'intégration perception-robot »). Une bonne
   estimation **trop tardive** déplace le point d'interception. Conséquence directe sur
   l'architecture (cf. §2) : **minimiser les sauts inter-process sur le chemin chaud**.
2. **Fréquence de contrôle = 60 Hz** (`dt_step = 1/60`, aligné sur la simulation). L'interface
   de commande bas-niveau tourne plus vite (125/500 Hz) → **interpolation** nécessaire (§8).
3. **Cohérence des repères.** La position balle doit arriver dans le **repère `base`** du robot,
   axes z-up, mètres, **exactement** comme le « local » de la simulation
   (`monde − env_origins`). Piège `base` vs `base_link` (rotation π autour Z) déjà documenté.
4. **Sécurité d'un robot réactif.** Cibles bornées (clip + rate-limit), watchdog/dead-man,
   vitesse réduite, E-stop. La politique sort des cibles agressives (cf. sim-to-real §1) : la
   sécurité est une **défense en profondeur indépendante de la sim**.

---

## 2. Architecture retenue : composition mono-processus

La boucle chaude `observation → inférence → action` tourne dans **un seul processus**. Les blocs
fonctionnels du schéma initial (Observation convertisseur, Inférence, Convertisseur Action)
deviennent des **modules Python wirés par appels directs** — aucun topic ROS, donc aucune
sérialisation ni saut DDS, sur le chemin critique 60 Hz. Les topics ROS ne servent qu'aux
**frontières** (balle en entrée, `/joint_states` en entrée, commande en sortie) et à la
**télémétrie/debug** (hors chemin chaud).

```
 ball_tracking_cpp (Dv-Rosws)  ──┐                         ┌── forward_position_controller
   ou test_ball_node (sim)      ▼  BallState (repère cam)  ▼   (ur_robot_driver, 125/500 Hz)
                          ┌─ ur3e_live_catch : 1 PROCESSUS, boucle 60 Hz ─────────────┐
   /joint_states ────────▶│  [ball→base via TF {}^B T_C] ▸ [filtre vitesse balle]     │
                          │  ▸ [ObservationBuilder 33-D] ▸ [PolicyRunner .ts+scaler]  │
                          │  ▸ [ActionMapper ×0.5 + clip + rate-limit] ▸ [streaming]  │
                          │   ── appels directs, pas de topic sur le chemin chaud ──   │
                          └──┬─────────────────────────────────────────────────┬─────┘
            télémétrie/debug ▼ (BallState base, obs, action, cible) hors-chemin │ commande
                          ur3e_web_ui (visualisation balle + robot + infos)      ▼
```

**Le retour « Observation » du schéma initial** se réalise sans topic :
- (a) l'état articulaire via l'abonnement `/joint_states` (callback qui met à jour un cache) ;
- (b) l'**action brute précédente**, stockée comme attribut du nœud (c'est la composante 9 de
  l'observation, cf. §6) et réinjectée à l'itération suivante.

### Nuance rclpy (à connaître)

La vraie composition **intra-process zéro-copie** est une fonctionnalité **rclcpp** (C++). Comme
l'inférence est PyTorch/Python, on ne peut pas l'obtenir telle quelle. On réalise donc la
« composition » par **un nœud rclpy unique** dont les sous-blocs sont des classes appelées
directement dans le même processus (mémoire partagée, pas de DDS). On garde le **bénéfice
latence** et une **structure modulaire** (un module = un fichier = une responsabilité). Les
signaux intermédiaires (obs, action brute, cible) restent **inspectables** via des topics de
**debug en best-effort, hors chemin critique**. Une vraie composition rclcpp est listée comme
option future en §13.

---

## 3. Paquets ROS 2 et placement

Deux nouveaux paquets à créer plus tard dans `$DV_ROSWS_ROOT/src/` :

| Paquet | Type | Rôle |
|---|---|---|
| `ur3e_catch_msgs` | rosidl (`ament_cmake`) | Messages typés et **horodatés** du contrat de catch (§5) |
| `ur3e_live_catch` | `ament_python` | Nœud live mono-processus, nœud de test, launch, sécurité, runtime politique |

**Workspace ROS unique.** Le tracker `ball_tracking_cpp`, les paquets robot existants
(`ur3e_web_ui`, `ur3e_rollout_replay`) et les futurs paquets live doivent être buildés dans
`Dv-Rosws`. Cela évite de devoir sourcer deux overlays et rend `ur3e_catch_msgs` visible à la fois
par le tracker C++ qui publiera `BallState` et par le nœud live qui le consommera.

Le code Isaac/training peut rester hors du workspace ROS, mais les paquets ROS et les exports
nécessaires au rejeu doivent rester dans `Dv-Rosws`. L'ancien `ros2_ws/src/` du dépôt Isaac n'est
plus un emplacement cible.

---

## 4. Spécification des nœuds et processus

### 4.1 `ball_tracking_cpp` — perception (existant, `Dv-Rosws`, à adapter)

- **Rôle.** Estimer la position 3D de la balle dans le flux événementiel (méthode **Trace** :
  largeur de traînée → profondeur monoculaire `Z = f_eff·D / w_px`), puis ajuster une trajectoire
  (régression pondérée `x,y` linéaires, `z` quadratique).
- **État actuel.** `Dv-Rosws/src/Ball_Tracking_Cpp/src/publisher_member_function.cpp` crée un
  publisher `ball_position_3d_mm` (`std_msgs/Float32MultiArray`, repère caméra, **mm**) mais **ne
  le peuple pas** (`publishBallPose()` laisse `msg.data` vide). Première chose à corriger.
- **Cible.** Publier `ur3e_catch_msgs/BallState` :
  - **horodaté au temps d'événement** (pas au temps de réception) — crucial pour la
    synchronisation et le calcul de latence ;
  - dans un **repère caméra nommé** fixe (`frame_id`), en **mètres** ;
  - convention d'axes **figée** = celle que résout le hand-eye. Aujourd'hui le code convertit
    caméra(mm)→monde(m) via `util.hpp::ToMeters` : `world = (x, z, −y)·10⁻³`. Il faut que la
    transformée `{}^{B}T_C` (§7) soit résolue **pour ce même repère** : un seul repère, partout.
- **Entrée :** caméra DVXplorer (`dv-processing`) ou séquences enregistrées. **Sortie :**
  `BallState` (~30 Hz, cadence perception).
- **Intérim (feuille de route).** Pour démarrer sans toucher au build C++, un **adaptateur
  Python** (dans `ur3e_live_catch`) abonne `ball_position_3d_mm` et republie `BallState`
  (stamp = réception, repère = constante). On migre ensuite vers la publication native horodatée.

### 4.2 `test_ball_node` — source de balle artificielle (nouveau)

C'est le « Nœud Test position Ball » du schéma : il **remplace la perception** pour tester toute
la chaîne **sans caméra ni lancer réel**.

- **Rôle.** Publier des trajectoires de balle **scriptées ou rejouées**, dans le **même contrat
  `BallState`** que le tracker réel (horodatage + champ `frame_id` toujours renseigné).
- **Repère de publication sélectionnable (paramètre `publish_frame`).** Le nœud peut publier la
  position balle **soit dans le repère caméra `<camera_frame>`** (chemin réaliste, identique au
  tracker : oblige à traverser le hand-eye `{}^{B}T_C`), **soit directement dans le repère `base`**
  du robot (court-circuite l'extrinsèque). Dans **les deux cas**, le `header.frame_id` du
  `BallState` **déclare** le repère effectif — c'est ce champ, et lui seul, qui dit au consommateur
  (`ball_frame`, §4.3.1) comment transformer. Intérêt :
  - publier en `base` **isole les erreurs politique/obs des erreurs de calibration extrinsèque** :
    si la chaîne marche en `base` mais pas en `<camera_frame>`, le hand-eye `{}^{B}T_C` est en cause ;
  - publier en `<camera_frame>` **valide la transformée complète** (TF statique + conversion
    d'axes/unités) de bout en bout, comme en conditions réelles.
  - En mode `base`, les sources analytiques (parabole, vérité terrain) sont naturellement déjà en
    coordonnées monde z-up : aucune conversion d'axes `util.hpp::ToMeters` n'est nécessaire.
- **Sources possibles.** (a) parabole analytique paramétrable (position/vitesse de lancer,
  gravité) ; (b) vérité terrain simulée `Dv-Rosws/sequences/<seq>/labels/ground_truth.csv` ;
  (c) rejeu d'un enregistrement.
- **Paramètres.** cadence (~30 Hz), bruit gaussien injectable (pour stresser le filtrage et la
  robustesse, cohérent avec le bruit d'obs randomisé à l'entraînement), trous/latence simulés.
- **Intérêt.** Permet de valider obs→inférence→action→robot avec `use_fake_hardware`/URSim avant
  toute caméra. Évalue aussi la robustesse au bruit/latence en amont.

### 4.3 `ur3e_live_catch` — la boucle live (un nœud, modules internes)

Un **seul nœud rclpy**, boucle pilotée à **60 Hz** (timer, ou déclenchée par `/joint_states`).
Modules (classes appelées directement) :

#### 4.3.1 `ball_frame.py` — repère + filtrage vitesse
- **Conscient du repère : ne suppose jamais la caméra.** Le module lit le `header.frame_id` du
  `BallState` reçu et transforme **vers le repère `base`** par un lookup TF générique
  `header.frame_id → base` (`P^B = {}^{B}T_{frame} · P^{frame}`) :
  - `frame_id == <camera_frame>` → applique le hand-eye `{}^{B}T_C` (§7) ;
  - `frame_id == base` → transformée **identité**, balle déjà en `base` (cas `test_ball_node`
    `publish_frame=base`) : **aucune** transformation appliquée ;
  - tout autre `frame_id` présent dans l'arbre TF est accepté (lookup tf2 standard) ;
  - `frame_id` **vide ou inconnu de TF** ⇒ **rejet** + drapeau watchdog (§9) : jamais d'hypothèse
    silencieuse de repère, qui enverrait la balle au mauvais endroit.
  Gère mm→m si nécessaire.
- **Filtre de vitesse balle** (Kalman, ou EMA sur différence finie) sur les positions `base` →
  `ball_vel`. Aucune mesure directe de vitesse n'existe : c'est le maillon bruité (cf.
  sim-to-real §5.2).
- **Gestion de la péremption** (staleness) : si plus de détection depuis `T_stale`, geler ou
  extrapoler brièvement, et lever un **drapeau watchdog** (§9).

#### 4.3.2 `observation.py` (ObservationBuilder) — observation 33-D
Reconstruit l'observation **dans l'ordre et les unités exacts** de
`firsttraining_env.py:197-208` (détail complet en §6). Miroir strict de `_get_observations`,
`_update_local_pose_tensors` et `detect_pass_through`. **Aucun degré**, tout en rad/rad/s/m/m·s⁻¹.

#### 4.3.3 `policy_runtime.py` (PolicyRunner) — inférence
- Charge l'export déterministe
  `$DV_ROSWS_ROOT/data/ur3e_rollouts/2026-05-26_17-13-29_ppo_torch/exports/policy_deterministic.ts`
  (+ `policy_metadata.json` ; `.onnx` disponible en alternative runtime).
- **Risque de correction critique — le scaler.** Le PPO SKRL utilise un `RunningStandardScaler`
  sur les observations. Il faut **vérifier si l'export embarque le scaler** ou s'il faut appliquer
  `mean`/`var` sauvegardés **avant** le réseau. Inspecter le script d'export du dépôt Isaac
  (`<ISAAC_REPO>/scripts/skrl/play.py`) et `policy_metadata.json`. Alimenter le réseau brut avec
  une observation non normalisée serait
  une source majeure de divergence sim-to-real.
- **Sortie = action brute (6)** (avant `action_scale`). C'est aussi ce qui est mémorisé pour la
  composante 9 de l'obs suivante.

#### 4.3.4 `action.py` (ActionMapper) — cible articulaire
- `joint_target = action_brute × action_scale`, avec `action_scale = 0.5`
  (`_pre_physics_step`, env.py:182). **Cible de position absolue** (pas vitesse/couple).
- Mémorise l'action **brute** pour l'obs suivante.

#### 4.3.5 `safety.py` (SafetyLimiter + watchdog) — défense en profondeur
- **Clip** aux bornes articulaires URDF + **bornes de workspace** (rejet hors zone sûre).
- **Rate-limit** : `|target − q| ≤ v_safe·dt_step` et accélération `≤ a_safe`, avec
  `v_safe = URDF × 0.5` → base/épaule/coude **1.571 rad/s** (90 °/s), poignets **3.142 rad/s**
  (180 °/s). Identique au clamp côté sim (sim-to-real §2.2) : même borne des deux côtés.
- **Watchdog/dead-man** : arrêt contrôlé si perception périmée, dépassement du budget temps de
  boucle, ou écart commande/réalisé trop grand. Réutilise les **gates** existants de `app.py`
  (External Control actif, speed scaling > 0, contrôleur actif).

#### 4.3.6 `streaming.py` — sortie vers le contrôleur
- **Interpole** la cible 60 Hz vers le taux du `forward_position_controller` et publie sur
  `/forward_position_controller/commands` (`std_msgs/Float64MultiArray`, 6 valeurs dans le
  **joint order** canonique). Démarrer avec des réglages conservateurs.

**Joint order canonique** (réutilisé partout, `replay_core.py:14`) :
`shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3`.

### 4.4 `ur_robot_driver` + `forward_position_controller` (existant, à configurer)

- La commande live passe par le **streaming controller** `forward_position_controller`
  (ros2_control), **pas** `scaled_joint_trajectory_controller` (qui bufferise des trajectoires —
  inadapté au réactif, cf. sim-to-real §5.4).
- Les deux contrôleurs revendiquent les **mêmes interfaces** → mutuellement exclusifs. La gestion
  de la **bascule** est en §8.

### 4.5 `ur3e_web_ui` — visualisation (existant, à étendre)

C'est la boîte « Visualisation Robot et Ball et autre information utile ».
- Abonner le bridge (`ros_interface.py`) à `BallState` (repère `base`) + topics de télémétrie
  (`CatchTelemetry`), ajouter un payload WebSocket, et un **marqueur balle + trajectoire prédite**
  dans `viewer3d.js`. Réutilise le viewer three.js et le bridge existants.
- **Hors chemin critique** : pure télémétrie, jamais sur la boucle 60 Hz.

---

## 5. Contrat de messages (`ur3e_catch_msgs`)

```
# BallState.msg — pose balle horodatée, partagée perception ↔ commande
std_msgs/Header   header        # stamp = temps d'événement ; frame_id = repère DÉCLARÉ de la
                                #   position : <camera_frame> (tracker), ou <camera_frame> OU base
                                #   (test_ball_node). Le consommateur transforme selon ce champ.
geometry_msgs/Point    position # mètres
geometry_msgs/Vector3  velocity # m/s (optionnel ; sinon recalculée côté ball_frame)
bool   valid                    # estimation exploitable ?
float32 confidence              # qualité (résidu/visibilité)
```

```
# CatchTelemetry.msg — debug/visualisation (hors chemin critique)
float32[]  observation   # 33
float32[]  raw_action    # 6
float32[]  joint_target  # 6 (après clip + rate-limit)
geometry_msgs/Point ball_base   # balle dans le repère base
```

> Alternative rapide pour le debug : `std_msgs/Float32MultiArray`. Mais pour `BallState` on
> garde un type **stampé** : la synchronisation temporelle est explicitement citée comme un défi
> (Stage_summary), et un timestamp d'événement fiable est indispensable au budget de latence.

---

## 6. Reconstruction de l'observation 33-D

Ordre **exact** (somme = 6+6+3+3+3+1+3+1+6+1 = **33**), miroir de `firsttraining_env.py:197-208` :

| # | Composante | Dim | Source live | Difficulté |
|---|---|---:|---|---|
| 1 | `joint_pos` | 6 | `/joint_states` réordonné (joint order §4.3) | facile |
| 2 | `joint_vel` | 6 | `/joint_states` | facile |
| 3 | `disk_pos_local` | 3 | FK `base → hoop_center` (TF statique `wrist_3_link → hoop`) | calibrage montage |
| 4 | `ball_pos_local` | 3 | balle en `base` (module `ball_frame`) | **critique** (perception+extrinsèque) |
| 5 | `direction` | 3 | `ball_pos_local − disk_pos_local` | dérivé |
| 6 | `distance` | 1 | `‖direction‖` | dérivé |
| 7 | `ball_vel_w` | 3 | vitesse balle **filtrée** | **critique** (bruit) |
| 8 | flag `prev_disk_signed_dist > 0` | 1 | recalculé (projection balle sur normale disque) | moyen |
| 9 | `actions` (action **brute** précédente) | 6 | attribut mémorisé (pré-`action_scale`) | facile |
| 10 | `pass_through_count` | 1 | recalculé (`detect_pass_through`, env.py:327) | moyen |

Points d'attention :
- **`ball_vel_w` est une vitesse « monde ».** Comme le passage en `base` n'est qu'une translation
  constante (origine d'environnement), la **vitesse est invariante** : la différence finie filtrée
  des positions `base` donne directement `ball_vel_w`.
- **Composante 9 = action brute**, pas la cible mise à l'échelle. C'est le retour interne décrit
  en §2.
- **Composantes 8 et 10** dépendent de la **géométrie disque/balle** (centre + normale du hoop).
  Elles doivent reproduire `detect_pass_through` et le signe de `(balle − disque)·normale`.
- **Unités strictes** : rad, rad/s, m, m/s. Aucune conversion en degrés ne doit traîner.

Source de vérité : `<ISAAC_REPO>/source/FirstTraining/FirstTraining/tasks/direct/firsttraining/firsttraining_env.py`
(`_get_observations`, `_update_local_pose_tensors`, `_read_disk_pose_in_body_frame`,
`detect_pass_through`) et `firsttraining_env_cfg.py` (`action_scale`, géométrie disque).

---

## 7. Cohérence des repères (piège majeur)

Graphe TF cible :

```
base ──(driver/URDF: chaîne cinématique)── base_link, ..., wrist_3_link ──(TF statique montage)── hoop_center
  │
  └──(TF statique hand-eye {}^B T_C)── <camera_frame>
        ▲
        └── BallState publiée dans <camera_frame>, transformée vers base par ball_frame
```

- **Un seul `<camera_frame>`.** Le tracker publie dedans ; le hand-eye résout `base →
  <camera_frame>` ; on publie cette transformée en **TF statique** (depuis `handeye_result.yaml`,
  via `static_transform_publisher` ou un petit nœud). Vérifier **signe et ordre des axes** avant
  tout test (cohérence avec `util.hpp::ToMeters`).
- **Transformation pilotée par le `frame_id`.** Le consommateur (`ball_frame`, §4.3.1) ne code
  jamais en dur « caméra » : il transforme `header.frame_id → base` via TF. Une `BallState`
  déclarée en `base` (test_ball_node) **court-circuite proprement le hand-eye** (transformée
  identité), ce qui permet de tester la boucle indépendamment de l'extrinsèque.
- **Calibration extrinsèque.** Rappel (Stage_summary) : `{}^{B}T_{S,i} = {}^{B}T_{E,i}·{}^{E}T_S`,
  `{}^{B}T_{C,i} = {}^{B}T_{S,i}·({}^{C}T_{S,i})^{-1}`, puis estimation multi-poses d'un unique
  `{}^{B}T_C`. La qualité de cette extrinsèque conditionne directement la précision : quelques cm
  d'erreur suffisent à rater la balle.
- **`hoop_center` (disque).** TF statique `wrist_3_link → hoop_center` (offset + normale) mesurée
  sur le **montage imprimé** : miroir réel de `_read_disk_pose_in_body_frame` (qui lit la
  géométrie dans le USD en sim).
- **`base` vs `base_link`.** Rotation π autour de Z (déjà documenté dans
  `ur3e_robot_control_architecture.md`). Le « local » de la sim = `monde − env_origins` = repère
  **`base`** du robot. Vérifier qu'on construit l'obs dans `base`, pas `base_link`.

---

## 8. Interface de commande et bascule de contrôleur

`forward_position_controller` et `scaled_joint_trajectory_controller` revendiquent les **mêmes
interfaces de commande** → ils ne peuvent pas être actifs ensemble. Séquence de bring-up :

1. **Approche** vers la pose de départ avec le `scaled_joint_trajectory_controller` (outils
   existants : web UI / `ur3e_rollout_replay`), qui est déjà sûr et retimé.
2. **Bascule** : désactiver le trajectory controller, activer `forward_position_controller` via
   `controller_manager/switch_controller` (plomberie **déjà présente** dans `ros_interface.py`).
3. **Catch** : la boucle live stream les cibles (interpolées) sur
   `/forward_position_controller/commands`.
4. **Re-bascule** vers le trajectory controller après la séquence (hold/retour).

Vérifications : `forward_position_controller` doit être **spawné** (au moins inactif) —
`ros2 control list_controllers`. S'il est absent, l'ajouter au `controllers.yaml` du driver.
Les modes « live catch » (streaming) et jog/home/replay (trajectoire) sont **mutuellement
exclusifs** ; l'exposer clairement dans l'UI/launch.

---

## 9. Couche de sécurité (obligatoire)

- **Clip + rate-limit** (`v_safe`, `a_safe`) appliqués **côté robot** aussi (indépendamment de
  la sim) — la politique sort des cibles agressives (sim-to-real §1).
- **Bornes** : positions URDF + workspace (rejet hors zone sûre).
- **Watchdog/dead-man** : perception périmée, boucle qui dépasse son budget, ou écart
  commande/réalisé trop grand → **arrêt contrôlé**.
- **Gates réutilisés** (`app.py`) : External Control en lecture, speed scaling > 0, contrôleur
  actif.
- **Procédure** : vitesse réduite, opérateur à l'E-stop, **pas de vraie balle** aux premiers
  essais, montée en vitesse progressive.

---

## 10. Budget de latence et synchronisation

- **Horodater** chaque message (`BallState` au temps d'événement). Mesurer la latence
  bout-en-bout : capture → détection → obs → policy → commande → début de mouvement (timestamps
  ROS). Fournir un petit outil de rapport de latence.
- **Contrainte** (sim-to-real §5.6) : latence réelle mesurée **≤** latence modélisée à
  l'entraînement (§2.4 du sim-to-real, `L_a`, `L_o`). Sinon, élargir la randomisation de latence
  et **ré-entraîner**.
- L'architecture mono-processus (§2) sert directement cet objectif en supprimant les sauts DDS
  internes.

---

## 11. Feuille de route d'implémentation (par étapes)

1. **`ur3e_catch_msgs`** (`BallState`, `CatchTelemetry`) ; buildable et visible par les deux
   workspaces.
2. **`test_ball_node`** (param `publish_frame` : `base` **ou** `<camera_frame>`) + **adaptateur**
   `Float32MultiArray → BallState` → chaîne testable sans caméra. Commencer en `publish_frame=base`
   (sans dépendre du hand-eye), puis passer en `<camera_frame>`.
3. **`ball_frame`** **conscient du repère** (transforme selon `header.frame_id`) + TF statiques
   (`base → <camera_frame>`, `wrist_3_link → hoop_center`) → `ball_pos_local` validé (echo +
   visualisation), vérifié en parité base/caméra (§12).
4. **`ObservationBuilder`** 33-D + **test d'équivalence** : rejouer un épisode
   `rollouts_10_episodes.json`, reconstruire l'obs, comparer bit-à-bit à l'obs sim.
5. **`PolicyRunner`** (trancher la question du scaler) → action brute en **dry-run** (logs, aucune
   commande robot).
6. **`ActionMapper` + `safety` + `streaming`** vers `forward_position_controller` sur
   **fake hardware/URSim**.
7. **Bascule de contrôleur** + mode « live catch » dans le web UI + **visualisation balle**.
8. **Mesure de latence** bout-en-bout ; vérifier ≤ latence modélisée.
9. **Bring-up robot par étapes** : perception seule → dry-run → balle lente E-stop en main →
   montée en vitesse de balle.

---

## 12. Vérification

- **Équivalence d'observation** : obs reconstruite par le nœud vs obs sim (ordre, dims, unités,
  **scaler**, repères) sur un épisode rejoué — tolérance serrée.
- **Bornes physiques** : `vitesse réalisée ≤ v_safe` par joint (réutiliser l'analyse de vitesse du
  sim-to-real §1).
- **Dry-run** : enregistrer obs/actions sans envoi robot ; inspecter `BallState` (base) via la
  visualisation et `ros2 topic echo`.
- **Parité de repère** : rejouer la **même** trajectoire avec `test_ball_node` en `publish_frame=base`
  puis en `publish_frame=<camera_frame>` (la version caméra étant la version `base` passée par
  `({}^{B}T_C)^{-1}`). Le `ball_pos_local` reconstruit doit **coïncider** (à l'erreur hand-eye
  près) : confirme que `ball_frame` est bien conscient du repère, et isole une erreur extrinsèque
  d'une erreur de chaîne.
- **Fake hardware / URSim d'abord** ; `forward_position_controller` actif vérifié ; **watchdog
  testé** (couper la perception → arrêt contrôlé).
- **Latence** mesurée et tracée ; vitesse réduite + E-stop pour les premiers essais réels.

---

## 13. Alternatives d'architecture

| Variante | Avantages | Inconvénients |
|---|---|---|
| **Composition mono-processus** (retenue) | Latence minimale (aucun saut DDS chaud) ; code modulaire ; signaux exposables en debug | « Composition » Python = un nœud rclpy (pas de vrai intra-process zéro-copie rclcpp) |
| **Multi-nœuds séparés** (schéma initial) | Modularité maximale ; chaque étape inspectable par topic | N sauts DDS sur le chemin 60 Hz (sérialisation + jitter d'ordonnancement) → latence |
| **Nœud politique unique pur** | Le plus simple à raisonner ; le moins de code | Moins de signaux intermédiaires observables (compensable par topics de debug) |
| **RTDE `servoj`** (au lieu de `forward_position_controller`) | Latence potentiellement plus basse | Hors ros2_control ; interface séparée ; plus délicat à stabiliser/sécuriser |
| **Composition rclcpp réelle** | Vrai intra-process zéro-copie | Exige une inférence **C++** (TorchScript C++) — option future si Python devient limitant |

Recommandation : **mono-processus + `forward_position_controller`** maintenant ; garder `servoj`
et la composition rclcpp en réserve si la latence mesurée (§10) l'exige.

---

## 14. Récapitulatif — réutilisation de l'existant

| Besoin | Réutiliser |
|---|---|
| Action ROS, joint order, build trajectoire | `Dv-Rosws/src/ur3e_rollout_replay/` (`replay_core.py`, `send.py:build_joint_trajectory`) |
| Bridge rclpy, gates sécurité, `switch_controller`, TF, WebSocket, viewer | `Dv-Rosws/src/ur3e_web_ui/ur3e_web_ui/` (`ros_interface.py`, `app.py`, `motion.py`, `static/js/viewer3d.js`) |
| Sémantique obs/action (source de vérité) | `<ISAAC_REPO>/source/FirstTraining/.../firsttraining/firsttraining_env.py`, `firsttraining_env_cfg.py`, `ur_gripper.py`, `agents/skrl_ppo_cfg.yaml`, `<ISAAC_REPO>/scripts/skrl/play.py` |
| Export politique | `$DV_ROSWS_ROOT/data/ur3e_rollouts/2026-05-26_17-13-29_ppo_torch/exports/` (`policy_deterministic.ts`/`.onnx`, `policy_metadata.json`) |
| Perception (Trace, régression, conversion 3D) | `Dv-Rosws/src/Ball_Tracking_Cpp/` (`Gui.cpp`, `BallTracker.cpp`, `RegressionAccumulator.hpp`, `util.hpp::ToMeters`) |
| Calibration caméra + extrinsèque | `Dv-Rosws/calibration_camera_DVXplorer_*.xml`, `Dv-Rosws/scripts/solve_handeye.py`, `ur3e_camera_base_calibration.md`, `handeye_result.yaml` |

> Source de vérité des limites de vitesse : `ur_description/config/ur3e/joint_limits.yaml` du
> driver réellement lancé ; `v_safe = limite × 0.5` (cf. sim-to-real §1, §2.1).
