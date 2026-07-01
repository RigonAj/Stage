# UR3e Ball-Catch — Plan & contraintes Sim-to-Real (PPO)

Ce document est le plan de référence pour rendre la politique PPO « attrape-balle »
**transférable au vrai UR3e**. Il documente les contraintes de **simulation**,
**d'entraînement** et **d'inférence (closed-loop live)**, avec les valeurs numériques ancrées sur
les vraies limites du robot.

> Statut code (mise à jour 2026-06-24) : les corrections de base côté Isaac Lab
> sont documentées dans `<ISAAC_REPO>/source/FirstTraining/...` : actionneur
> borné par joint, action incrémentale clippée/rate-limitée, `clip_actions: True`
> côté policy, et métadonnées d'export mises à jour. Côté ROS, les paquets
> `ur3e_catch_msgs` et `ur3e_live_catch` existent dans ce workspace : le chemin
> `BallState` + `/joint_states` -> observation 33-D -> policy -> safety ->
> streaming est câblé derrière `enable_command=false` par défaut. Le diagnostic
> chiffré ci-dessous reste celui de l'**ancien** rollout copié dans
> `$DV_ROSWS_ROOT/data/ur3e_rollouts/...`. Restent à valider/terminer :
> perception réelle, timestamp d'événement, TF statiques, latence réelle, domain
> randomization, reward shaping, dynamique de balle ralentie et bring-up robot.

> Décisions cadrant ce plan :
> 1. **Cible de déploiement = closed-loop live** — la politique tourne sur le robot à la
>    fréquence de contrôle, alimentée par la perception en direct. C'est le seul mode capable
>    d'attraper une balle réellement lancée (le replay open-loop ne peut pas ralentir une vraie
>    balle).
> 2. **Enveloppe = limites articulaires UR3e nominales** — contraindre la sim aux vitesses UR3e
>    constructeur (`180 deg/s` autres joints, `360 deg/s` poignets),
>    modéliser la latence, et **reformuler la dynamique de la balle** pour que la tâche reste
>    faisable dans ce budget.

Documents liés : `ur3e_real_robot_replay.md` (réalisé vs commande, replay open-loop),
`ur3e_robot_control_architecture.md` (stack de contrôle), `ur3e_camera_base_calibration.md`
(hand-eye → balle en frame `base`), `ur3e_web_ui.md` (URDF/limites chargées par l'UI).

---

## 0. Changements appliqués (2026-06-20)

Dans `<ISAAC_REPO>/source/FirstTraining/FirstTraining/tasks/direct/firsttraining/` :

- `ur_gripper.py` : historique appliqué au 2026-06-20 :
  `effort_limit_sim = [54, 54, 28, 9, 9, 9] Nm` pour
  `[shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]`. La doc de
  référence actionneur du 2026-06-23 demande maintenant d'aligner la cible
  Isaac sur `ur_description` : `[56, 56, 28, 12, 12, 12] Nm`.
- `ur_gripper.py` : `velocity_limit_sim` nominal UR3e par joint =
  `[3.1416, 3.1416, 3.1416, 6.2832, 6.2832, 6.2832] rad/s`
  (`180 deg/s` autres joints, `360 deg/s` poignets). Les gains restent provisoires :
  `stiffness = 800`, `damping = 40`, à identifier par mesure step-response réelle.
- `firsttraining_env_cfg.py` : `a_safe = 4 * v_safe` =
  `[12.5664, 12.5664, 12.5664, 25.1328, 25.1328, 25.1328] rad/s^2`.
- `firsttraining_env_cfg.py` : bornes opérationnelles de position =
  `[-2π, -2π, -π, -2π, -2π, -2π]` à `[2π, 2π, π, 2π, 2π, 2π]`.
- `firsttraining_env.py` : l'action PPO est maintenant clippée dans `[-1, 1]`, transformée en
  cible incrémentale `q + action * v_safe * dt_s`, puis limitée en accélération et en position.
- `agents/skrl_ppo_cfg.yaml` : `models.policy.clip_actions: True`.
- `scripts/skrl/play.py` : les nouveaux rollouts exportent la vraie cible
  `base_env.joint_pos_target` et les métadonnées `dt_s`, `action_delta_scale_rad`, `v_safe`,
  `a_safe`, bornes articulaires et nouvelle note `action_semantics`.
- `firsttraining_env_cfg.py` : distribution de balle actuelle utilisée par
  l'export 2026-06-30 et le bouton web UI `Isaac random` :
  spawn `x=(-0.6,-0.2)`, `y=(1.2,2.1)`, `z=(0.5,1.2)`,
  bruit position `0.01 m`, vitesse `vx=(-0.7,0.6)`,
  `vy=(-5.0,-3.5)`, `vz=(-0.1,1.5)`.
- `data/models/` : exports `latest` et `best` du run
  `2026-06-30_19-02-25_ppo_torch`, avec TorchScript, ONNX et métadonnées ; le
  modèle canonique par défaut est `latest`. Les rollouts de validation ne sont
  pas versionnés dans `main` et doivent être régénérés au besoin.

Conséquence importante : les anciennes policies et l'ancien
`rollouts_10_episodes.json` sont **incompatibles** avec la nouvelle sémantique d'action
absolu → incrémental. Il faut réentraîner puis régénérer les rollouts avant toute comparaison
avant/après ou replay de la nouvelle policy.

## 1. Diagnostic chiffré de l'écart sim↔réel

Tout est contrôlé à **60 Hz** : `sim.dt = 1/120`, `decimation = 2` → `dt_step = 1/60 ≈
16.67 ms` (`firsttraining_env_cfg.py`, confirmé par `metadata.dt_s`).

Dans l'ancien code/export, l'action était une **cible de position articulaire absolue** : `joint_pos_target =
action × action_scale` avec `action_scale = 0.5` (`_pre_physics_step`), appliquée via
`set_joint_position_target` (`_apply_action`). La politique a **`clip_actions: False`**
(`skrl_ppo_cfg.yaml`) → aucune borne sur la sortie. Dans le rollout exporté,
`action_normalized` sort largement de `[-1, 1]` (plage globale observée : **[-6.165,
2.871]**), ce qui explique les cibles articulaires très éloignées.

Mesures sur `rollouts_10_episodes.json` (10 épisodes) :

| Grandeur | Réalisé (`joint_position_before_rad`) | Commande (`joint_position_target_rad`) |
|---|---|---|
| Pas max par step | 0.105 rad | 2.815 rad |
| **Vitesse max impliquée** | **6.3 rad/s (361 °/s)** | **168.9 rad/s (9677 °/s)** |
| **Accélération max impliquée** | **754.0 rad/s²** | **13224.7 rad/s²** |
| Plage absolue | [-2.45, 1.07] rad | [-3.08, 1.44] rad |
| Durée d'un catch | **~11 steps ≈ 0.18 s** | idem |

Détail des vitesses max impliquées par joint :

| Joint | Réalisé max | Commande max | Limite URDF actuelle |
|---|---:|---:|---:|
| shoulder_pan | 3.19 rad/s | 81.85 rad/s | 3.14 rad/s |
| shoulder_lift | 3.08 rad/s | 162.47 rad/s | 3.14 rad/s |
| elbow | 3.43 rad/s | 71.78 rad/s | 3.14 rad/s |
| wrist_1 | 6.29 rad/s | 168.89 rad/s | 6.28 rad/s |
| wrist_2 | 6.29 rad/s | 74.94 rad/s | 6.28 rad/s |
| wrist_3 | 6.28 rad/s | 112.85 rad/s | 6.28 rad/s |

Limites réelles UR3e (source constructeur Universal Robots :
<https://www.universal-robots.com/manuals/EN/HTML/SW5_24/Content/prod-usr-man/complianceUR3e/H_g5_sections/appendix_g5/tech_spec_data.htm>,
à comparer au `ur_description/config/ur3e/joint_limits.yaml` du driver réellement lancé) :

| Joint | effort max | vitesse max | position |
|---|---|---|---|
| shoulder_pan / shoulder_lift | 54 Nm | 180 °/s = **3.14 rad/s** | ±360° |
| elbow | 28 Nm | 180 °/s = **3.14 rad/s** | ±180° (limité) |
| wrist_1 / wrist_2 | 9 Nm | 360 °/s = **6.28 rad/s** | ±360° |
| wrist_3 | 9 Nm | 360 °/s = **6.28 rad/s** | sans limite de position dans l'URDF Humble actuel |

Actionneur sim avant correctif (`ur_gripper.py`, `ImplicitActuatorCfg`) : `effort_limit_sim = 23.0`
**uniforme**, `stiffness = 800`, `damping = 40`, **aucune `velocity_limit`**.

**Conséquences (les 4 causes de non-transfert) :**

1. **Pas de limite de vitesse en sim** → le drive PD atteint ou dépasse les limites URDF
   actuelles : base/coude autour de 3.2–3.4 rad/s, poignets autour de 6.28 rad/s. C'est déjà
   au niveau des vitesses nominales UR3e, et les **commandes brutes** impliquent 70–169 rad/s
   selon le joint.
2. **Stiffness trop raide (800)** → suivi quasi instantané de la cible en 1/60 s. Le vrai
   `scaled_joint_trajectory_controller` a une bande passante finie : il y a un retard de suivi
   et une accélération bornée que la sim ignore.
3. **Actions non clippées** → cibles jusqu'à ±3 rad : dangereuses et non suivables ; sur le
   vrai robot elles saturent le contrôleur ou déclenchent un arrêt de sécurité.
4. **Tâche trop rapide** → catch en 0.18 s. Avec la latence réelle (perception ~30 Hz +
   détection + comms + actuation ≈ 50–150 ms), 0.18 s est inférieur au temps de réaction : la
   tâche est physiquement infaisable telle quelle.

Le pipeline replay (`replay_core.py`, défauts `max_joint_velocity = 0.25 rad/s`,
`max_joint_acceleration = 0.5 rad/s²`) contourne 2 et 3 en **ralentissant** la trajectoire
enregistrée — acceptable pour rejouer un mouvement sim, **impossible** pour attraper une balle
en temps réel. D'où le besoin du closed-loop.

---

## 2. Contraintes de SIMULATION

Objectif : faire en sorte que **tout mouvement réalisable en sim soit réalisable sur le robot
réel**, en injectant la physique et les retards réels dans l'entraînement.

### 2.1 Modèle d'actionneur réaliste (`ur_gripper.py`)
- **Appliqué : `velocity_limit_sim` par joint** = limites UR3e nominales constructeur :
  - base/épaule/coude : `v_safe = 3.142 rad/s` (180 °/s)
  - poignets : `v_safe = 6.283 rad/s` (360 °/s)
- **À aligner : `effort_limit_sim` par joint** (ne pas garder 23 Nm uniforme).
  La valeur historique du projet était `[54, 54, 28, 9, 9, 9]` Nm ; la cible
  cohérente avec `ur_description` est `[56, 56, 28, 12, 12, 12]` Nm. Voir
  `ur3e_parametres_actionneur_reference.md`.
- **Reste à faire : ré-identifier `stiffness`/`damping`** pour reproduire la **bande passante de suivi réelle**.
  Méthode (system-id) : sur le vrai robot, envoyer un échelon de position et enregistrer la
  réponse via `/joint_states` ; ajuster `k`/`d` en sim pour matcher le temps de montée et le
  dépassement. La stiffness 800 actuelle donne un suivi trop rapide → la réduire jusqu'à
  retrouver le retard réel. Documenter les valeurs trouvées par joint.
- Garder un actionneur PD implicite, mais possibilité de passer à `DCMotorCfg`/modèle explicite
  si le system-id montre une saturation couple importante.

### 2.2 Redéfinition de l'espace d'action (le levier principal)
L'option A est maintenant appliquée. L'option B reste une alternative historique non retenue :

- **Option A appliquée — Cibles incrémentales (delta) bornées.** Dans `_pre_physics_step` :
  `target[t] = clip(q[t] + action × Δ, q_min, q_max)` avec `Δ = v_safe × dt_step`. Cela
  **borne intrinsèquement la vitesse commandée** à `v_safe`, indépendamment de la sortie de la
  politique. Plus robuste, et identique à ce qu'on appliquera côté robot.
- **Option B non retenue — Garder l'absolu + rate-limiter.** Conserver `target = action × scale` puis
  clamper : `|target − q| ≤ v_safe × dt_step` et limite d'accélération
  `|target − 2·q + q_prev| ≤ a_safe × dt_step²`. Plus proche du code actuel mais laisse la
  politique apprendre des cibles hors-limite que le clamp masque (moins propre).

Dans les deux cas :
- **Clipping des actions appliqué** : `clip_actions: True` dans `skrl_ppo_cfg.yaml` comme
  garde-fou, mais **ne pas compter dessus comme seule sécurité** : selon l'action space exposé
  à SKRL, le clipping peut ne pas suffire. La source de vérité doit être un clamp/rate-limit
  explicite dans `_pre_physics_step`, puis le même clamp côté robot. Objectif : supprimer les
  cibles ±3 rad vues dans le rollout actuel.
- **Limite d'accélération `a_safe` appliquée avec une valeur initiale conservatrice** :
  `a_safe = 4 * v_safe`. Cette valeur reste à confirmer par system-id ; la borne empêche déjà
  la politique d'exploiter des sauts d'accélération infinis irréalistes.

### 2.3 Limites articulaires
- Position : la couche sim utilise une enveloppe opérationnelle finie
  `[±360°, ±360°, ±180°, ±360°, ±360°, ±360°]`. Si un autre `ur_description` est utilisé,
  vérifier le fichier de limites du driver réellement lancé : l'ancien workspace legacy local
  contient des efforts/limites légèrement différents.
- Vitesse : `v_safe` nominal UR3e — voir 2.1. Pour le vrai robot, garder une marge opérateur
  via le speed slider / safety layer, même si la policy est entraînée à la limite nominale.
- Vérifier que la **distribution de reset** (`_reset_idx`) reste dans ces bornes (c'est déjà le
  cas) et que la pose de départ est atteignable par le vrai robot.

### 2.4 Latence (contrainte la plus importante pour une tâche réactive)
- **Retard d'action** : appliquer l'action avec `L_a` steps de retard (ring buffer), `L_a`
  randomisé sur ~**2–8 steps (33–133 ms)**. Couvre comms ROS + temps de réponse driver +
  actuation.
- **Retard + sous-échantillonnage de la perception** : la balle (`ball_pos`, `ball_vel`) n'est
  pas connue à 60 Hz. Modéliser une caméra à **~30 Hz** (mise à jour de la balle tous les
  ~2 steps) **+ latence de détection** (`L_o` ~1–4 steps) : geler/retarder les composantes
  `ball_*` de l'observation en conséquence. Le reste de l'obs (joints) reste à 60 Hz.
- Ces retards doivent être **présents pendant l'entraînement** (et randomisés) pour que la
  politique apprenne à anticiper, pas seulement à réagir.

### 2.5 Fréquence de contrôle
- Conserver **60 Hz** (= `dt_step`), aligné sur la boucle live ROS 2.
- Documenter le taux exact de l'interface streaming choisie (`servoj`/RTDE/driver, typiquement
  plus rapide que 60 Hz selon robot et driver) : à l'inférence on **interpole** entre deux
  commandes de politique à 60 Hz pour alimenter ce contrôleur (voir §5).

---

## 3. Contraintes d'ENTRAÎNEMENT

### 3.1 Reward shaping pour la transférabilité (`compute_rewards`)
Récompense actuelle : `rew_dist` (façonnage distance, conditionné « approche »), `rew_action
= -0.5·Σ action²`, `rew_pass = +400` (catch), `rew_termination = -100`.

Problèmes & ajouts :
- Le terme `rew_action = -0.5·Σ action²` pénalise la **magnitude de la cible absolue** → biais
  vers la pose zéro, pas vers la douceur. **À remplacer/compléter** par :
  - **Pénalité de vitesse articulaire** : `-w_v · Σ joint_vel²` (lisse et limite la vitesse).
  - **Pénalité de taux d'action** : `-w_a · ‖aₜ − aₜ₋₁‖²` (l'action précédente est déjà dans
    l'obs → calculable ; réduit le jerk et le chattering, crucial pour le réel).
  - **Pénalité d'accélération/jerk** : optionnelle si 2.2 borne déjà l'accel.
  - **Pénalité de proximité des limites** articulaires (barrière douce).
- Recalibrer les poids pour que `rew_pass` reste dominant mais sans encourager les mouvements
  violents. Garder le bonus de catch et la pénalité de collision bras (`hit_arm`).

### 3.2 Domain randomization (obligatoire pour le transfert)
Randomiser à chaque reset / par épisode :
- **Gains actionneur** `stiffness`/`damping` (±20–30 %) autour des valeurs system-id.
- **Friction / damping articulaire**, **masse de la charge utile** (hoop/gripper, ±%) et léger
  décalage géométrique du hoop/disque si le montage réel n'est pas exactement celui du USD.
- **Latences** `L_a`, `L_o` (cf. 2.4) tirées par épisode.
- **Dynamique de la balle** : masse, restitution, + bruit de spawn (déjà `ball_position_noise`).
- **Bruit d'observation** sur `ball_pos`/`ball_vel` (la perception réelle est bruitée et
  biaisée) et bruit léger sur `joint_pos`/`joint_vel`.

### 3.3 Distribution de reset & timing
- Pose de départ réaliste et atteignable (déjà échantillonnée autour de
  `[0, -1.57, 0, -1.57, 0, 0]`) ; vérifier qu'elle correspond à la pose réelle de départ
  utilisée à l'inférence.
- Après ralentissement de la balle (§4), **revoir `episode_length_s`** pour que la fenêtre de
  catch soit atteignable dans le budget de vitesse réel.

### 3.4 Hyperparamètres PPO (`skrl_ppo_cfg.yaml`)
- Globalement conservés (PPO, 512 envs, lr 5e-4 KL-adaptatif, 200k timesteps).
- `discount_factor = 0.995` + épisodes très courts : réévaluer après §4 (horizon plus long
  une fois la balle ralentie).
- Détail cosmétique mais utile : `experiment.directory = "cartpole_direct"` est trompeur
  (ce n'est pas du cartpole) → renommer en `ur3e_ball_catch`. Idem nom de tâche/log.

---

## 4. Reformulation de la DYNAMIQUE DE LA BALLE (faisabilité)

L'ancienne balle (`ball_velocity_y_range = (-10, -2.5)` m/s, spawn ~1.2 m) arrivait trop vite
pour le budget de vitesse sûr. Elle est aussi balistique : rayon `0.03 m`, masse `0.05 kg`,
gravité activée, et `max_linear_velocity = 10.0` dans `ball_cfg`. **Inégalité de faisabilité**
à respecter :

```
t_arrivée_balle  ≥  L_perception + t_déplacement_robot
t_déplacement_robot ≈ Δθ_max / v_safe       (Δθ_max = plus grand mouvement articulaire requis)
```

Actions :
- **Réduire `ball_velocity_y_range`** et/ou **augmenter la distance de spawn** (`ball_spawn_*`)
  pour que `t_arrivée` laisse le temps au robot de se positionner à `v_safe ≈ 3.14 rad/s`
  (joints base/épaule/coude). Le réglage actuel de debug/training resserre déjà la balle à
  `vy=(-5.0,-3.5)` m/s, avec `x=(-0.6,-0.2)`, `y=(1.2,2.1)`, `z=(0.5,1.2)`.
- **Méthode de calibrage** : mesurer en sim le `t_déplacement_robot` typique sur des catches
  réussis (= `Δθ_max / v_safe`), fixer `t_arrivée` à ≥ ce temps + marge de latence, en déduire
  vitesse/distance de balle.
- Revoir `reset_on_success` et `episode_length_s` en cohérence.
- Garder à l'esprit la trajectoire balistique (gravité activée) : la distance verticale de
  chute pendant `t_arrivée` augmente avec un vol plus long → ajuster `ball_spawn_z` et surtout
  `ball_velocity_z_range`. Ralentir seulement `ball_velocity_y_range` peut rendre la balle
  trop basse avant d'atteindre le hoop.

---

## 5. Contraintes d'INFÉRENCE / DÉPLOIEMENT (closed-loop live)

### 5.1 Architecture de la boucle (implémentée côté ROS, à valider sur robot)
Séparée de `ur3e_rollout_replay` (qui reste pour la validation open-loop). Le
paquet local `src/ur3e_live_catch` implémente déjà cette boucle à 60 Hz en
dry-run ou commande protégée par `enable_command` :

```
caméra ─▶ détection balle 3D ─▶ TF base ─▶ construit obs 33-D ─▶ politique (TorchScript/ONNX)
                                                                        │
   /joint_states ──────────────────────────────────────────────▶ obs   ▼
                                                                   action (6)
                                                                        │
                                            clip + rate-limit (v_safe, a_safe)
                                                                        │
                                       interface streaming bas-niveau (servoj / forward ctrl)
```

La politique est exportable (`play.py --export_policy [--export_onnx]` →
`policy_deterministic.ts` / `.onnx` + `policy_metadata.json`). Pour les policies réentraînées
après le 2026-06-20, la sémantique attendue est incrémentale :
`joint_position_target_rad = q + clamp(action, -1, 1) * v_safe * dt_s`, avec limite
d'accélération puis clamp articulaire. Comme le PPO utilise `RunningStandardScaler` pour les
observations/valeurs, le noeud live doit charger l'export déterministe complet ou reproduire
exactement le prétraitement SKRL ; alimenter directement le réseau brut avec une observation non
normalisée serait une autre source de divergence.

### 5.2 Reconstruction de l'observation 33-D (champ par champ)
Ordre exact (`_get_observations`) — **doit être reproduit à l'identique** :

| Composantes | Dim | Source réelle | Difficulté |
|---|---|---|---|
| `joint_pos` | 6 | `/joint_states` (réordonné UR3e) | ✓ facile |
| `joint_vel` | 6 | `/joint_states` | ✓ facile |
| `disk_pos_local` | 3 | FK : TF `base → wrist_3_link` + offset disque (cf. `_read_disk_pose_in_body_frame`) | ✓ / calibrage montage |
| `ball_pos_local` | 3 | **perception caméra → frame `base`** (hand-eye calibré) | ⚠ critique |
| `direction` = balle − disque | 3 | dérivé | ✓ |
| `distance` | 1 | dérivé | ✓ |
| `ball_vel_w` | 3 | **différence finie filtrée** des positions balle | ⚠ critique (bruit) |
| flag `prev_disk_signed_dist > 0` | 1 | recomputé (géométrie disque/balle) | ✓ |
| `actions` (action précédente) | 6 | mémorisée dans le nœud | ✓ |
| `pass_through_count` | 1 | recomputé (`detect_pass_through`) | ✓ |

Points critiques :
- **Ordre et unités** : reproduire exactement `cfg.joint_names` et les unités du code
  (`rad`, `rad/s`, `m`, `m/s`). Aucun degré côté politique.
- **`disk_pos_local`** : dans Isaac, l'offset disque est lu dans le USD. Sur le robot réel, il
  faut mesurer/calibrer cet offset du hoop par rapport à `wrist_3_link`/TCP ; le TF URDF seul
  ne connaît pas forcément la géométrie imprimée ou montée.
- **`ball_pos_local`** : nécessite une détection 3D de la balle robuste (ex. couleur/profondeur)
  exprimée dans le **frame `base`** via le résultat hand-eye (`handeye_result.yaml`). C'est le
  maillon le plus sensible.
- **`ball_vel_w`** : aucune mesure directe → différence finie des détections, **filtrée**
  (Kalman/EMA) pour gérer le bruit et la latence. Le bruit de perception doit avoir été
  randomisé à l'entraînement (§3.2) sous peine de divergence sim-to-real.

### 5.3 Cohérence des frames
- Le « local » de la sim = `pos_monde − origine_env` = **frame `base` du robot**.
- Le frame `base` de la perception doit être **exactement** le même : axes z-up, unités mètres,
  même origine. Attention au piège `base` vs `base_link` déjà documenté
  dans `ur3e_robot_control_architecture.md` et `ur3e_motion_issue_resolution.md`
  (conversion dans le viewer/UI). Vérifier signe et ordre des axes avant tout test live.

### 5.4 Interface de contrôle
- **Ne pas utiliser `scaled_joint_trajectory_controller` pour le live** : il bufferise des
  trajectoires, inadapté au contrôle réactif step-par-step. Il reste pour l'**approche** et la
  **validation replay** (outils existants).
- Utiliser une **interface streaming** : `forward_position_controller` / passthrough du
  `ur_robot_driver`, ou **RTDE `servoj`** (UR doc). Envoyer la cible à chaque step, interpoler
  vers le taux du contrôleur (125/500 Hz). Démarrer avec `lookahead_time`/gain conservateurs.
- La cible envoyée = sortie politique mappée **et re-clippée/rate-limitée côté robot**
  (défense en profondeur, indépendante de la sim).

### 5.5 Couche de sécurité (obligatoire)
- Clip + rate-limit (`v_safe`, `a_safe`) appliqués **aussi** côté robot.
- Bornes articulaires URDF + bornes de **workspace** (rejeter toute cible hors zone sûre).
- **Watchdog / dead-man** : si la perception décroche ou la boucle dépasse son budget temps,
  arrêt contrôlé.
- **Abort sur erreur de suivi** (écart commande/réalisé trop grand).
- Speed slider pendant réduit, opérateur à l'E-stop, pas de balle réelle aux premiers essais.

### 5.6 Budget de latence (à mesurer)
- Mesurer la latence bout-en-bout : capture caméra → détection → obs → policy → commande →
  début de mouvement (timestamps ROS).
- **Contrainte** : latence réelle mesurée ≤ latence modélisée en sim (§2.4). Sinon élargir la
  randomisation de latence et ré-entraîner.

---

## 6. Vérification

Baseline ancien export (`rollouts_10_episodes.json`, policy absolue incompatible) :

| Mesure avant correctif | Valeur |
|---|---:|
| Plage globale `action_normalized` | `[-6.165, 2.871]` |
| Vitesse max réalisée | `6.292 rad/s` |
| Vitesse max cible brute | `168.891 rad/s` |
| Accélération max réalisée | `753.991 rad/s²` |
| Accélération max cible brute | `13224.679 rad/s²` |

Chiffres après correctif : **à produire après réentraînement et nouvel export**. Les anciens
rollouts ne peuvent pas servir de validation car leur policy commande des cibles absolues.

1. **Sim — succès** : `play.py --eval_episodes N` avant/après contraintes (taux de succès).
   Une baisse est attendue (tâche plus dure) ; viser un compromis succès/réalisme.
2. **Sim — bornes physiques** : ré-enregistrer des rollouts (`play.py --record_actions`) puis
   vérifier programmatiquement que **`max vitesse réalisée ≤ v_safe`** par joint, que les
   cibles sont bornées par `v_safe * dt_s` et `a_safe`, et qu'aucune cible ne sort des bornes
   articulaires. Les nouveaux rollouts doivent contenir `metadata.dt_s`, `metadata.action_scale`,
   `metadata.action_delta_scale_rad`, `metadata.action_semantics`, `joint_velocity_safe_rad_s`
   et `joint_acceleration_safe_rad_s2`.
3. **Robustesse latence** : balayer `L_a`/`L_o` en éval, tracer la dégradation du succès →
   confirme que la politique tolère la latence réelle.
4. **Observation live** : enregistrer des observations construites par le noeud ROS2 en dry-run
   et vérifier ordre, dimensions, unités, scaler et frames avant toute commande robot.
5. **Replay (étape intermédiaire)** : valider le mouvement appris via les outils existants
   (`ur3e_replay_validate`, web UI) — passe désormais les limites sans ralentissement extrême.
6. **Robot — bring-up par étapes** : (a) boucle perception seule (vérifier `ball_pos_local`),
   (b) politique en « dry-run » sans envoi de commande (logs), (c) live balle lente E-stop en
   main, (d) montée en vitesse de balle progressive.

---

## 7. Récapitulatif des fichiers impactés

| Fichier | Changement |
|---|---|
| `source/.../firsttraining/ur_gripper.py` | appliqué : `velocity_limit_sim` + `effort_limit_sim` par joint ; reste : k/d system-id |
| `<ISAAC_REPO>/source/.../firsttraining/firsttraining_env.py` | appliqué : action delta/rate-limit (`_pre_physics_step`) ; reste : buffers de latence, reward shaping (`compute_rewards`), bruit d'obs |
| `<ISAAC_REPO>/source/.../firsttraining/firsttraining_env_cfg.py` | appliqué : `v_safe`/`a_safe`/bornes position/ranges balle ; reste : `episode_length_s`, flags DR |
| `<ISAAC_REPO>/source/.../firsttraining/agents/skrl_ppo_cfg.yaml` | appliqué : `clip_actions: True` côté policy ; reste : renommage expérience, hyperparams |
| `src/ur3e_live_catch` | nœud closed-loop : perception → obs 33-D → policy → sécurité → contrôleur streaming ; câblé derrière `enable_command`, validation robot/perception réelle encore ouverte |
| `docs/Robot_Control/ur3e_real_robot_replay.md`, `docs/Robot_Control/ur3e_robot_control_architecture.md` | lien croisé vers ce document |

> Source de vérité côté robot : la documentation constructeur UR3e et le
> `ur_description/config/ur3e/joint_limits.yaml` du driver ROS réellement lancé. Toute valeur
> d'entraînement doit indiquer explicitement si elle utilise la limite nominale ou une marge
> volontaire sous cette limite.
