# Reste a faire - UR3e live catch

Date : 2026-06-22

Ce document liste les actions restantes pour passer de la chaine live-catch
outillee et testee en dry-run a une validation robot reel. Il complete
`docs/Robot_Control/ur3e_live_catch_implementation_status.md` et
`docs/incoherences_code_logique.md`.

## Priorite 0 - Bloquants avant perception reelle

### [ ] Calibrer extrinsequement la camera vers la base robot

**Etat actuel :** les scripts, la page mire telephone, l'onglet Calibration UI
et le solveur hand-eye existent, mais la session physique finale n'est pas
encore acceptee.

**Pourquoi c'est bloquant :** toute balle publiee dans le repere camera doit
etre transformee en `base`. Quelques centimetres d'erreur sur `T_base_camera`
suffisent a rater le panier.

**Action :**

```bash
source env.sh
ur3e_stack
scripts/run_handeye_session.sh

python3 scripts/solve_handeye.py recordings/mire_calibration/handeye/handeye_samples_*.json \
  --output-yaml calibration/handeye_result.yaml

scripts/publish_camera_tf.py calibration/handeye_result.yaml --print-only
```

**Validation attendue :** 15 a 20 poses variees, residu pixel proche du RMS
`solvePnP`, leave-one-out stable, position camera plausible au metre ruban, et
repere camera visible dans l'UI via `/api/calibration/camera`.

References : `docs/Robot_Control/ur3e_camera_base_calibration.md`,
`scripts/solve_handeye.py`, `scripts/publish_camera_tf.py`.

### [ ] Publier les TF statiques `base -> camera` et `wrist_3_link -> hoop_center`

**Etat actuel :** `live_catch_node` sait utiliser TF, mais sans TF
`base -> <camera_frame>` la perception camera est inutilisable, et sans
`hoop_center` le node retombe sur un placeholder dans `live_catch.yaml`.

**Pourquoi c'est bloquant :** l'observation PPO depend de `ball_pos_local`,
`disk_pos_local`, du signe de passage et du `pass_through_count`.

**Action :**

```bash
scripts/publish_camera_tf.py calibration/handeye_result.yaml
ros2 run tf2_ros static_transform_publisher <x> <y> <z> <qx> <qy> <qz> <qw> wrist_3_link hoop_center
```

**Validation attendue :** `ros2 run tf2_ros tf2_echo base camera_optical` et
`ros2 run tf2_ros tf2_echo base hoop_center` repondent de facon stable; le
viewer affiche une camera et un hoop coherents avec le montage.

### [ ] Tester la parite `publish_frame=base` vs `publish_frame=<camera_frame>`

**Etat actuel :** `test_ball_node` peut publier la meme parabole directement en
`base` ou dans un repere camera via la pose camera configuree.

**Pourquoi c'est bloquant :** ce test isole une erreur de TF/extrinseque d'une
erreur policy/observation.

**Action :**

```bash
ros2 launch ur3e_live_catch live_catch.launch.py use_test_ball:=true publish_frame:=base enable_command:=false
ros2 topic echo /catch_telemetry

ros2 launch ur3e_live_catch live_catch.launch.py use_test_ball:=true publish_frame:=camera_optical enable_command:=false
ros2 topic echo /catch_telemetry
```

**Validation attendue :** a trajectoire equivalente, `ball_base`,
`ball_vel_base`, l'observation 33-D et la cible policy restent coherentes a la
precision TF/calibration pres.

## Priorite 1 - Bring-up robot reel sans vraie balle

### [ ] Valider la commande sur robot reel avec balle virtuelle

**Etat actuel :** la chaine a ete testee en dry-run et en process avec torch.
Le robot reel est disponible, mais la commande physique via
`forward_position_controller` n'est pas encore validee.

**Pourquoi c'est bloquant :** il faut verifier la bascule de controleur, le
streaming, le watchdog et le retour au controller de trajectoire avant toute
vraie balle.

**Action :**

```bash
source env.sh
ur3e_stack

# Terminal live-catch, avec torch disponible dans l'environnement utilise.
ros2 launch ur3e_live_catch live_catch.launch.py \
  use_test_ball:=true trigger_mode:=true enable_command:=false

# Terminal UI.
ros2 run ur3e_web_ui ur3e_web_ui
```

Dans l'onglet `Test` :

- lancer une balle virtuelle avec `Launch virtual ball`;
- verifier le fantome policy vert;
- cocher la confirmation E-stop / workspace;
- activer `Run on real robot` a vitesse reduite;
- arreter avec `Stop / back to safe`.

**Validation attendue :** `forward_position_controller` devient actif pendant
la commande, la cible est bornee, le robot bouge sans saut, `Stop / back to safe`
restaure `scaled_joint_trajectory_controller`, et l'UI reflete
`command_enabled=false` apres l'arret.

### [ ] Tester le watchdog sur robot reel

**Etat actuel :** le watchdog est implemente : perception perimee, depassement
budget boucle ou erreur de suivi provoquent un hold controle.

**Pourquoi c'est bloquant :** en live, une perte de perception ne doit pas
laisser la policy continuer a commander des cibles obsoletes.

**Action :** en mode commande avec balle virtuelle, couper volontairement la
source balle ou attendre la fin du vol `trigger_mode`.

**Validation attendue :** le node publie un hold, `SafetyLimiter.reset()` evite
une reprise brutale, et les logs signalent la raison du stop.

### [ ] Caler `a_safe`, `loop_budget_s` et `max_tracking_error` sur le materiel

**Etat actuel :** les valeurs par defaut sont conservatrices mais pas
identifiees sur le robot.

**Pourquoi c'est bloquant :** des seuils trop hauts masquent des problemes de
suivi; des seuils trop bas arretent la boucle sans raison.

**Action :** enregistrer `/joint_states`, `catch_telemetry`,
`ros2 control list_controllers` et les logs pendant les essais balle virtuelle.

**Validation attendue :** seuils documentes dans `live_catch.yaml` ou dans une
note de test, avec vitesse slider, mode `action_mode`, modele et observations de
tracking.

## Priorite 2 - Brancher la vraie perception

### [ ] Publier une vraie position depuis le tracker C++

**Etat actuel :** `publisher_member_function.cpp` appelle `publishBallPose()`,
mais publie un `Float32MultiArray` vide sur `ball_position_3d_mm`.

**Pourquoi c'est bloquant :** `float32_adapter.py` ne peut produire un
`BallState` valide que si le tableau contient au moins `[x, y, z]`.

**Action minimale :** remplir `msg.data` avec la position 3D, en documentant
l'ordre, l'unite et le repere.

**Action cible :** publier directement `ur3e_catch_msgs/BallState` depuis le
tracker C++, avec `header.frame_id` et `header.stamp` au temps d'evenement.

**Validation attendue :** `ros2 topic echo /ball_state` montre `valid=true`,
une position en metres, un `frame_id` non vide, et une latence coherente dans
`CatchTelemetry.perception_age_s`.

### [ ] Propager un timestamp evenement reel

**Etat actuel :** l'adaptateur legacy timestamp a la reception; `test_ball_node`
timestamp a l'heure de simulation/node.

**Pourquoi c'est bloquant :** la compensation de latence et les comparaisons
sim-to-real dependent de l'age de la mesure.

**Action :** choisir le timestamp de l'estimation Trace (par exemple milieu ou
fin de la fenetre d'evenements utilisee), le convertir en temps ROS, puis le
mettre dans `BallState.header.stamp`.

**Validation attendue :** `perception_age_s` reste positif, stable, compatible
avec la cadence camera/tracker, et reagit correctement si la perception ralentit.

### [ ] Mesurer la latence bout-en-bout avec perception reelle

**Etat actuel :** `latency_report` existe et la latence dry-run a ete mesuree
avec balle virtuelle.

**Pourquoi c'est bloquant :** le robot ne peut attraper une balle reelle que si
la latence mesuree reste dans l'enveloppe modelisee ou acceptee par la policy.

**Action :**

```bash
ros2 run ur3e_live_catch latency_report
ros2 topic echo /catch_telemetry
```

**Validation attendue :** p50/p95/p99 documentes pour `perception_age_s` et
`loop_compute_s`, avec le backend torch reel et la vraie source camera.

## Priorite 3 - Nettoyage et reproductibilite

### [ ] Canonicaliser le modele policy utilise en live

**Etat actuel :** `data/models/` ne contient pas de modele suivi; le node peut
retomber sur l'export date.

**Pourquoi c'est important :** un test robot doit pouvoir dire exactement quel
modele a commande, avec quelle semantique et quelle normalisation.

**Action :**

```bash
ln -s ../ur3e_rollouts/2026-05-26_17-13-29_ppo_torch/exports/policy_deterministic.ts \
  data/models/policy_deterministic.ts
```

Ajouter ou copier aussi `policy_metadata.json`, puis documenter :
`action_semantics`, `normalization=embedded`, `dt_s`, `joint_order` et
`action_mode` attendu.

**Validation attendue :** lancement sans `model_path` explicite charge le modele
canonique voulu, et le log n'utilise pas le fallback par surprise.

### [ ] Mettre a jour `src/ur3e_catch_msgs/README.md`

**Etat actuel :** le README dit encore que les messages et fichiers de build
sont a ajouter.

**Validation attendue :** le README decrit les messages existants, les champs
`CatchTelemetry` actuels, le build et les producteurs/consommateurs.

### [ ] Unifier le chemin `handeye_result.yaml`

**Etat actuel :** le script de session documente
`recordings/mire_calibration/handeye/handeye_result.yaml`; l'UI lit
`calibration/handeye_result.yaml` par defaut.

**Validation attendue :** une calibration resolue apparait dans l'UI sans copie
manuelle non documentee, ou la commande UI utilise explicitement
`--camera-calibration`.

### [ ] Stabiliser la documentation des assets de calibration

**Etat actuel :** `Support3D.obj` est present localement mais ignore par Git.
Les assets versionnes sont le STEP, les GLB et le MTL.

**Validation attendue :** un clone propre suffit pour ouvrir la doc, afficher le
support dans l'UI et comprendre comment regenerer l'OBJ si necessaire.

## Sequence recommandee courte

1. Faire la calibration hand-eye et publier `base -> camera_optical`.
2. Mesurer/publier `wrist_3_link -> hoop_center`.
3. Valider la parite balle virtuelle `base` puis `camera_optical`.
4. Tester l'onglet `Test` en dry-run avec torch.
5. Tester `enable_command` sur fake hardware / URSim.
6. Tester robot reel avec balle virtuelle, vitesse reduite, E-stop en main.
7. Brancher le tracker C++ en `BallState` horodate.
8. Mesurer la latence reelle et seulement ensuite passer a une vraie balle lente.
