# UR3e Ball-Catch — Propositions d'amélioration sim-to-real & revue d'inférence

> Statut (2026-06-23) : **document de propositions**, issu d'une revue ciblée de la
> boucle **live catch** et de l'**inférence**. Aucun code n'est modifié par ce
> document. Le **nouveau modèle à sémantique incrémentale est encore en cours
> d'entraînement** côté utilisateur ; les propositions « entraînement » ci-dessous
> visent donc précisément ce modèle en cours, et les propositions « inférence »
> préparent le nœud live pour qu'il le déploie correctement dès qu'il sera prêt.

Ce document complète, sans les remplacer :
- `ur3e_ball_catch_sim_to_real.md` — plan de référence sim/entraînement/inférence.
- `ur3e_live_catch_architecture.md` — architecture des nœuds ROS 2.
- `ur3e_live_catch_implementation_status.md` — état d'implémentation (étapes 1–9).
- `incoherences_code_logique.md` — registre des incohérences (laissé inchangé).
- `reste_a_faire.md` — checklist d'exécution bring-up.

Les constats **déjà** suivis ailleurs (fallback perception legacy, TF statiques
hand-eye/hoop, modèle canonique `data/models/` absent,
chemins hand-eye divergents) ne sont **pas redétaillés ici** : voir les deux derniers
documents. Ce document se concentre sur (a) **des bugs/risques d'inférence
nouvellement identifiés**, et (b) **des propositions sim-to-real concrètes**.

---

## 1. Statut de la revue

**Docs : globalement à jour et cohérents** (dernière revue d'ensemble 2026-06-22).
L'architecture, l'état d'implémentation et le plan sim-to-real concordent avec le
code lu. Les modules de logique pure (`observation.py`, `action.py`, `safety.py`,
`ball_frame.py`, `limits.py`, `streaming.py`, `policy_runtime.py`) sont testés hors
ROS et le chemin chaud `perception → policy → action → safety` tourne en dry-run.

**Code : sain dans l'ensemble**, avec une bonne discipline « jamais de substitution
silencieuse » (ex. `joint_order.reorder_by_name` et `limits.build_joint_bounds`
lèvent au lieu d'inventer une valeur). Les exceptions à cette discipline et les
écarts sim/réel sont l'objet des §3 à §5.

---

## 2. Le point dur sim-to-real : pas encore de modèle transférable

Le seul export présent sur disque est l'**ancienne policy à action absolue** :

- `data/ur3e_rollouts/2026-05-26_17-13-29_ppo_torch/exports/policy_metadata.json:20`
  déclare `action_semantics = "joint_position_target_rad = action_normalized *
  action_scale"` avec `action_scale = 0.5`.
- Dans `rollouts_10_episodes.json`, `action_normalized` sort largement de `[-1, 1]`
  (plage globale `[-6.165, 2.871]`, cf. `ur3e_ball_catch_sim_to_real.md §1`) → cibles
  articulaires jusqu'à ±3 rad, vitesses impliquées jusqu'à ~169 rad/s.
- `data/models/` ne contient qu'un `README.md` → `live_catch_node.py:62-65` retombe
  sur cet export daté (`FALLBACK_MODEL`).

Les correctifs Isaac (action **incrémentale** bornée, `clip_actions: True`, limites
de vitesse/effort par joint) listés dans `ur3e_ball_catch_sim_to_real.md §0`
**n'ont pas encore été ré-entraînés ni ré-exportés** (entraînement en cours).

**Conséquence à retenir :** tant que le nouveau modèle n'est pas exporté, **aucune
commande robot ne devrait être tentée avec le modèle présent**. Le mode `faithful`
(défaut, `live_catch_node.py:101`) reproduit fidèlement une policy que le plan
sim-to-real qualifie lui-même de non transférable (§3.4 ci-dessous explique
pourquoi la couche safety ne « sauve » pas ce modèle, elle change sa dynamique).

---

## 3. Bugs / risques d'inférence nouvellement trouvés

> Tous ces points sont **nouveaux** (absents de `incoherences_code_logique.md` et
> `reste_a_faire.md`). Aucun n'est corrigé ici — uniquement constaté + proposition.

### 3.1 Reset d'épisode absent en mode commande (bug fonctionnel)

**Constat.** `_reset_sim()` (`live_catch_node.py:421-429`) remet à zéro
l'`ObservationBuilder` (`pass_through_count`, `_prev_signed_dist`) **et**
`self._prev_action`. Or il n'est appelé **que** en dry-run :
`live_catch_node.py:443` → `if not commanding: self._reset_sim()`. En mode commande,
quand la balle redevient invalide (fin d'un lancer), rien ne réinitialise l'obs ;
`_controlled_stop` (`live_catch_node.py:406-414`) ne reset que `self._safety`.

**Impact.** Sur une session live à plusieurs lancers, la composante 10
(`pass_through_count`, `observation.py:149`) **s'accumule** d'un catch à l'autre et
la composante 9 (`prev_action`) **persiste**, alors qu'en simulation chaque épisode
repart de zéro (`_reset_idx`). Dès le 2ᵉ lancer, l'observation envoyée à la policy
est **hors-distribution** → comportement imprévisible exactement au moment où le
robot bouge.

**Correction suggérée.** Détecter le front « balle valide → invalide » (fin
d'épisode) **dans les deux modes** et réinitialiser alors `ObservationBuilder`,
`prev_action`, et la mémoire `safety`/`streamer`. Alternative : exposer un service
`~/reset_episode` que l'onglet Test appelle avant chaque `~/throw`.

### 3.2 Fallback de géométrie disque silencieux (sûreté + correction)

**Constat.** `_disk_pose()` (`live_catch_node.py:383-394`) tente le lookup TF
`base→hoop_center` et, **sur toute exception**, renvoie sans aucun log le
placeholder de config : `disk_pos_fallback = [0,0,0.5]`, `disk_normal_fallback =
[0,0,1]` (`config/live_catch.yaml:18-19`, `live_catch_node.py:93-94`).

**Impact.** Ce placeholder est un point **fixe dans `base`**, alors que le hoop réel
est monté sur `wrist_3_link` et **se déplace avec le bras**. Les composantes
d'observation **3** (`disk_pos_local`), **5** (`direction`), **6** (`distance`),
**8** (signe) et **10** (`pass_through`) deviennent alors physiquement fausses — et
ce **silencieusement**, ce qui contredit la discipline du reste du code (cf.
`joint_order.py`, `limits.py` qui lèvent). Un opérateur peut commander le robot en
croyant l'observation correcte.

**Correction suggérée.** (a) Logguer un warning *throttlé* quand le fallback est
utilisé ; (b) en **mode commande**, refuser de commander tant que la TF
`base→hoop_center` n'est pas disponible (même logique que le refus « pas de modèle »
de `live_catch_node.py:229-235`) ; (c) distinguer dans `_disk_pose` l'absence de TF
(attendue au démarrage) d'une vraie erreur de lookup.

### 3.3 Composantes obs 3 / 8 / 10 non validées, source Isaac absente

**Constat.** Le repo Isaac n'est pas présent sur la machine (`<ISAAC_REPO>` est un
placeholder ; le checkpoint pointe vers `/home/rigon/Documents/IsaacTrain/...` non
monté). Les fonctions `signed_distance` (`observation.py:89-97`) et
`_update_pass_through` (`observation.py:99-113`) portent un `TODO(isaac)` : la
**convention de signe de la normale** et la logique exacte de `detect_pass_through`
(porte « dans le cerceau », test de direction de vitesse) sont **devinées**.

Surtout, `test_observation_equivalence.py:64-76` **injecte** les valeurs
enregistrées de `disk`/`ball`/`ball_vel` dans le builder et **ne compare pas** les
comps 8/10 reconstruites aux comps enregistrées (seules 1/2/5/6/9 sont vérifiées,
cf. docstring du test). → le test **donne une fausse confiance** : il ne couvre pas
la partie géométrie, qui est précisément la plus incertaine.

**Correction suggérée.** Quand la source Isaac est disponible : figer
`_read_disk_pose_in_body_frame` (repère exact : `base`/local vs *body* du root) et
`detect_pass_through`, puis **étendre le test d'équivalence** pour comparer comp 8 et
comp 10 reconstruites aux valeurs enregistrées sur les rollouts (le nouveau rollout
post-retrain les contiendra). Tant que ce n'est pas fait, marquer comps 3/8/10
comme **non garanties** dans la télémétrie.

### 3.4 Mismatch de boucle fermée `faithful` + safety (cœur du non-transfert)

**Constat.** En sim, la policy a été entraînée avec un actionneur très raide
(`stiffness = 800`, `ur_gripper.py`) → la cible **absolue** est atteinte quasi
instantanément en 1 pas. En réel, `ActionMapper` faithful pose `target = action·0.5`
(`action.py:60`), puis `SafetyLimiter` rate-limite `|Δ| ≤ v_safe·dt`
(`safety.py:67-75`) et borne l'accélération (`safety.py:77-86`). Avec
`v_safe_factor = 0.5` (`config/live_catch.yaml:33`), `v_safe·dt ≈ 1.571/60 ≈
0.026 rad/tick`.

**Impact.** Comme l'ancienne policy sort des actions extrêmes, la cible est
quasi-toujours saturée loin de `q` → le bras **rampe à vitesse constante `v_safe`**
vers les bornes articulaires, au lieu de suivre une trajectoire de catch. La
dynamique boucle-fermée *vue par la policy* (donc l'observation `joint_pos`/`vel`
réinjectée) est **radicalement différente** de l'entraînement. C'est l'explication
mécanique du non-transfert : ce n'est pas un réglage de safety, c'est une
**incohérence structurelle entre la dynamique d'entraînement et celle de
déploiement**. La proposition §4.2 (safety-in-the-loop) la résout à la racine.

### 3.5 Pass-through avancé à 60 Hz sur une balle perçue à 30 Hz (à vérifier)

**Constat.** `_on_tick` reconstruit l'observation à 60 Hz mais la balle n'est mise à
jour qu'à ~30 Hz (`test_ball_node` `rate_hz: 30`, et la vraie perception est ~30 Hz).
La balle est donc **gelée** un tick sur deux, tandis que `ObservationBuilder.build`
fait **avancer** `_prev_signed_dist` et le compteur à chaque tick
(`observation.py:151-154`).

**Impact (mineur, non confirmé).** Si la simulation calcule le pass-through sur la
balle **vraie** à 60 Hz, le live le calcule sur la balle **perçue** gelée → comp 10
peut diverger près du plan du disque. À trancher avec la source Isaac (lié à §3.3).

**Correction suggérée.** Décider si pass-through doit n'avancer que sur une
**nouvelle** détection (stamp changé) plutôt qu'à chaque tick, et l'aligner sur le
choix fait en sim.

---

## 4. Propositions sim-to-real — côté ENTRAÎNEMENT (Isaac)

> Ces points alimentent directement le **modèle en cours d'entraînement**. Plusieurs
> sont déjà listés « à faire » dans `ur3e_ball_catch_sim_to_real.md` ; ils sont
> repris ici avec une **proposition concrète** et un lien vers ce que le nœud live
> sait déjà mesurer.

### 4.1 Ré-entraîner + ré-exporter avec la sémantique incrémentale, et graver les métadonnées

La sémantique cible est `joint_position_target_rad = q + clamp(action,-1,1)·v_safe·dt`
(`ur3e_ball_catch_sim_to_real.md §2.2`, option A appliquée). À l'export :

- écrire dans `policy_metadata.json` : `action_semantics = "incremental"`,
  `normalization = "embedded"` (confirmé Δ=4.6e-6 sans scaler externe, cf. status
  §7.1), `dt_s`, `v_safe`, `a_safe`, bornes articulaires, `joint_order` ;
- régénérer `rollouts_*.json` avec `joint_position_target_rad`, comps 8/10 et les
  champs `joint_velocity_safe_rad_s`/`joint_acceleration_safe_rad_s2` (pour les tests
  de bornes physiques, `ur3e_ball_catch_sim_to_real.md §6.2`).

Côté inférence, cela permettra le **gate de sémantique** §5.4 et l'**équivalence
comp 8/10** §3.3.

### 4.2 Proposition clé — entraîner avec la couche safety **dans la boucle**

**Idée.** Faire en sorte que sim et réel partagent **exactement** le même pipeline
`action → cible → clip position → rate-limit v_safe → accel-limit a_safe`. En
pratique : appliquer en simulation la **même** transformation que `ActionMapper` +
`SafetyLimiter` (mêmes `v_safe`, `a_safe`, mêmes bornes) **avant** d'envoyer la cible
à l'actionneur, et baisser `stiffness` pour refléter la bande passante réelle (§4.7).

**Pourquoi.** C'est la correction *racine* du bug §3.4 : la policy apprend alors la
dynamique réellement réalisable (rampe à `v_safe`, accélération bornée), si bien que
le mode `faithful` au déploiement devient cohérent — plus de « rampe vers une cible
extrême ». Cela rend aussi le choix `faithful`/`safe` **moins critique**.

**Lien code.** La logique existe déjà côté robot (`safety.py:54-91`,
`action.py:55-67`) et pourrait servir de spécification de référence pour la version
sim (mêmes formules, mêmes constantes via `limits.py`).

### 4.3 Modélisation de latence (la contrainte n°1 d'une tâche réactive)

`ur3e_ball_catch_sim_to_real.md §2.4` la décrit mais elle **n'est pas implémentée**.
Proposition :

- **Retard d'action** `L_a` (ring buffer) randomisé ~2–8 steps ; **retard +
  sous-échantillonnage perception** `L_o` (~30 Hz + 1–4 steps) sur les composantes
  `ball_*` uniquement.
- **Calibrer la distribution sur le réel mesuré** : le nœud expose déjà
  `perception_age_s` et `loop_compute_s` (`live_catch_node.py:504-509`,
  `CatchTelemetry`), agrégés par `latency_report.py` (p50/p95/p99). Mesurer en
  dry-run avec la vraie perception, puis fixer `L_o`/`L_a` ≥ p95 mesuré.
- Contrainte de validation : latence réelle ≤ latence modélisée
  (`ur3e_ball_catch_sim_to_real.md §5.6`), sinon élargir et ré-entraîner.

### 4.4 Domain randomization (obligatoire pour le transfert)

Non implémentée (`ur3e_ball_catch_sim_to_real.md §3.2`). Randomiser par épisode :
gains `stiffness`/`damping` (±20–30 % autour du system-id §4.7), friction/damping
articulaire, masse charge utile (hoop/gripper), léger décalage géométrique du hoop,
dynamique de balle (masse/restitution), **bruit d'observation** sur `ball_pos`/
`ball_vel` (cohérent avec le `noise_std` que `test_ball_node` sait déjà injecter) et
bruit léger sur `joint_pos`/`joint_vel`.

### 4.5 Reward shaping pour la douceur (transférabilité)

`rew_action = -0.5·Σ action²` pénalise la **magnitude de la cible absolue** (biais
vers la pose zéro), pas la douceur (`ur3e_ball_catch_sim_to_real.md §3.1`). Avec la
sémantique incrémentale, ajouter/compléter par : pénalité de **vitesse articulaire**
`-w_v·Σ joint_vel²`, pénalité de **taux d'action** `-w_a·‖aₜ−aₜ₋₁‖²` (l'action
précédente est déjà dans l'obs, comp 9 → calculable), barrière douce de proximité
des bornes. Recalibrer pour garder `rew_pass` dominant. Réduit jerk/chattering,
critique pour le réel.

### 4.6 Dynamique de la balle (faisabilité)

Catch actuel ~11 steps ≈ **0.18 s** < temps de réaction réel (perception ~30 Hz +
détection + comms + actuation ≈ 50–150 ms) → tâche infaisable telle quelle
(`ur3e_ball_catch_sim_to_real.md §1, §4`). Proposition : respecter l'inégalité
`t_arrivée ≥ L_perception + Δθ_max/v_safe`, donc ralentir `ball_velocity_y_range`
et/ou augmenter la distance de spawn, en réajustant `ball_velocity_z_range`/
`ball_spawn_z` pour la chute balistique, puis revoir `episode_length_s`.

### 4.7 System-identification stiffness/damping

`stiffness = 800` donne un suivi trop rapide. Méthode : envoyer un échelon de
position sur le vrai robot, enregistrer `/joint_states`, ajuster `k`/`d` en sim pour
matcher temps de montée + dépassement par joint
(`ur3e_ball_catch_sim_to_real.md §2.1`). Indispensable pour que §4.2 reflète la vraie
bande passante.

---

## 5. Propositions sim-to-real — côté INFÉRENCE / DÉPLOIEMENT (nœud live)

### 5.1 Corriger les bugs §3 (reset, disque, équivalence)

Repris ici comme actions : reset d'épisode en mode commande (§3.1) ; warning +
refus de commande sans TF hoop (§3.2) ; extension du test d'équivalence aux comps
8/10 quand la source Isaac est là (§3.3). Ce sont des **pré-requis** à tout essai
robot multi-lancer.

### 5.2 Compensation de latence à l'inférence (extrapolation de la balle)

**Idée.** L'observation utilise `ball_pos` au temps de la détection, qui a déjà
`perception_age_s` de retard (mesuré, `live_catch_node.py:504`). Avant de construire
l'obs, **extrapoler** la balle à `t_now` : `ball_pos += ball_vel·age + ½·g·age²`
(g connu, balistique). Cela aligne l'observation sur l'instant courant sans
ré-entraîner, et réduit l'erreur d'interception due au retard. À borner par un
`age_max` (au-delà, geler/lever le watchdog).

**Lien.** `ball_vel` filtrée est déjà disponible (`ball_frame.py`), et `g` est déjà
utilisé pour l'arc prédit côté web UI (`viewer3d.js`). Cohérent avec la modélisation
de latence §4.3 : on **modélise** le retard en sim et on le **compense** au réel.

### 5.3 Filtre de vitesse balle de type Kalman

`BallVelocityFilter` (`ball_frame.py:60-98`) est une EMA sur différences finies :
simple mais bruitée et en retard de phase. Proposition : Kalman à modèle
**vitesse/accélération constante + gravité** (état `[p, v]`, prédiction balistique),
qui (a) lisse mieux, (b) fournit `ball_vel` plus propre pour comp 7, (c) se prête
naturellement à l'extrapolation §5.2. Garder l'API `process()` actuelle pour ne pas
toucher le nœud.

### 5.4 Gate de sémantique d'action piloté par les métadonnées

Aujourd'hui `action_mode` est un **flag manuel** (`config/live_catch.yaml:25`) sans
lien avec le modèle chargé. Risque : déployer un modèle incrémental en mode
`faithful` (ou l'inverse). Proposition : lire `policy_metadata.action_semantics`
(`policy_runtime.load_metadata`, déjà présent `policy_runtime.py:108-110`) et
**refuser le mode commande** si la sémantique du modèle ≠ `action_mode` demandé
(même esprit que le refus « pas de modèle » `live_catch_node.py:229-235`). Graver la
sémantique attendue dans `data/models/` (cf. §4.1).

### 5.5 Protocole de montée `v_safe_factor`

Le live utilise `v_safe_factor = 0.5` (`config/live_catch.yaml:33`) alors que la sim
vise les limites nominales (×1.0). Documenter une **montée explicite** 0.5 → 1.0 :
chaque comparaison sim/réel doit indiquer le facteur utilisé
(`incoherences_code_logique.md #6` le note déjà côté registre). Idéalement,
entraîner et déployer **au même facteur** (lié à §4.2).

### 5.6 Timestamp d'événement (rappel)

`perception_age_s` n'est fiable que si `BallState.header.stamp` porte le **temps
d'événement**. Le tracker C++ natif le fait maintenant via `BallPose3D.timestampUs`
ancré sur l'horloge ROS ; le fallback `float32_adapter.py` reste timestampé à la
réception. Pré-requis à la calibration de latence §4.3.

---

## 6. Priorisation / feuille de route

| Prio | Action | Réf. |
|---|---|---|
| **P0 — transfert** | Finir le ré-entraînement incrémental + export métadonnées | §4.1 |
| **P0 — transfert** | Entraîner **avec la safety dans la boucle** (même pipeline que le robot) | §4.2, §3.4 |
| **P0 — transfert** | Modéliser la latence en sim, calibrée sur `latency_report` réel | §4.3 |
| **P1 — inférence** | Reset d'épisode en mode commande | §3.1, §5.1 |
| **P1 — inférence** | Géométrie disque : warning + refus de commande sans TF hoop | §3.2, §5.1 |
| **P1 — inférence** | Gate de sémantique d'action piloté par métadonnées | §5.4 |
| **P1 — inférence** | Compensation de latence (extrapolation balle) | §5.2 |
| **P2 — robustesse** | Domain randomization + reward shaping + dynamique balle | §4.4-§4.6 |
| **P2 — robustesse** | Filtre Kalman + équivalence comps 8/10 + pass-through 30/60 Hz | §5.3, §3.3, §3.5 |
| **P2 — robustesse** | System-id k/d ; montée `v_safe_factor` documentée | §4.7, §5.5 |

---

## 7. Vérification (par proposition)

- **Équivalence obs/policy étendue** : après retrain, rejouer les nouveaux rollouts,
  comparer comps **1/2/5/6/9 ET 8/10** bit-à-bit, puis sortie policy vs
  `action_normalized` (tolérance serrée) — étend `test_observation_equivalence.py` et
  `test_policy_equivalence.py`.
- **Bornes physiques** : sur les nouveaux rollouts, vérifier `vitesse réalisée ≤
  v_safe` par joint, cibles bornées par `v_safe·dt`/`a_safe`, aucune hors bornes
  (`ur3e_ball_catch_sim_to_real.md §6.2`).
- **Robustesse latence** : balayer `L_a`/`L_o` en éval, tracer la dégradation du
  succès ; confirmer latence réelle (`latency_report`) ≤ modélisée.
- **Safety-in-the-loop** : vérifier que la trajectoire sim (cible après safety) et la
  trajectoire réelle coïncident pour la même séquence d'actions (mêmes constantes).
- **Reset d'épisode** : en mode commande, deux lancers successifs → comp 10 repart à
  0 et comp 9 à zéro au 1ᵉʳ tick du 2ᵉ lancer.
- **Géométrie disque** : couper la TF `base→hoop_center` → warning émis et commande
  refusée (pas de fallback silencieux).
- **Extrapolation balle** : en dry-run, comparer `ball_pos` extrapolée vs vérité
  terrain décalée de `perception_age_s` (via `test_ball_node` csv).
- **Parité de repère** : `publish_frame=base` vs `<camera_frame>` (déjà outillé,
  `reste_a_faire.md`), inchangé par ces propositions.
- **Bring-up** : fake hardware / URSim → robot réel, E-stop en main, balle virtuelle,
  vitesse réduite, montée progressive (`ur3e_live_catch_implementation_status.md §8`).

---

## 8. Récapitulatif

- Les **docs sont à jour** ; le **code live est sain** mais comporte **5 risques
  d'inférence nouvellement identifiés** (§3), dont deux à corriger avant tout essai
  robot multi-lancer (reset d'épisode §3.1, fallback disque silencieux §3.2).
- Le **non-transfert** du modèle actuel est **structurel** (§3.4), pas un réglage :
  il se résout en **alignant la dynamique d'entraînement sur celle de déploiement**
  (safety-in-the-loop §4.2) en plus du ré-entraînement incrémental (§4.1) et de la
  modélisation de latence (§4.3).
- Côté inférence, les gains les plus rentables sans ré-entraîner sont la
  **compensation de latence par extrapolation** (§5.2) et le **gate de sémantique**
  (§5.4), qui sécurisent le déploiement du futur modèle.
