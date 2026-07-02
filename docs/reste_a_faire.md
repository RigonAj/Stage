# Reste a faire - UR3e live catch

Date : 2026-06-24

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

**Validation attendue :** `ros2 run tf2_ros tf2_echo base_link camera_optical`
et `ros2 run tf2_ros tf2_echo base_link hoop_center` repondent de facon stable; le
viewer affiche une camera et un hoop coherents avec le montage.

### [ ] Tester la parite `publish_frame=base_link` vs `publish_frame=<camera_frame>`

**Etat actuel :** `test_ball_node` peut publier la meme parabole directement en
`base_link` ou dans un repere camera via la pose camera configuree.

**Pourquoi c'est bloquant :** ce test isole une erreur de TF/extrinseque d'une
erreur policy/observation.

**Action :**

```bash
ros2 launch ur3e_live_catch live_catch.launch.py use_test_ball:=true publish_frame:=base_link enable_command:=false
ros2 topic echo /catch_telemetry

ros2 launch ur3e_live_catch live_catch.launch.py use_test_ball:=true publish_frame:=camera_optical enable_command:=false
ros2 topic echo /catch_telemetry
```

**Validation attendue :** a trajectoire equivalente, `ball_base`,
`ball_vel_base`, l'observation 33-D et la cible policy restent coherentes a la
precision TF/calibration pres.

## Priorite 1 - Bring-up robot reel sans vraie balle

### [x] Valider la commande sur robot reel avec balle virtuelle

**Etat actuel :** selon rapport utilisateur du 2026-07-02, la chaine balle
virtuelle -> policy -> streaming 500 Hz -> robot reel fonctionne. Le robot suit
et tient apres fin de vol, mais le comportement reste lent sous les limites de
bring-up (`v_safe_scale=0.5`).

**Pourquoi ca reste important :** ce n'est plus un blocage de commande de base,
mais il faut encore valider le watchdog, le retour controleur et le tuning avant
toute vraie balle.

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

**Validation observee :** le robot reel suit la policy avec la balle virtuelle
et tient apres l'arret du vol. A re-verifier pendant les prochains essais :
`Stop / back to safe` restaure `scaled_joint_trajectory_controller`, et l'UI
reflete `command_enabled=false` apres l'arret.

### [ ] Tester le watchdog sur robot reel

**Etat actuel :** le watchdog est implemente : perception perimee, depassement
budget boucle ou erreur de suivi provoquent un hold controle.

**Pourquoi c'est bloquant :** en live, une perte de perception ne doit pas
laisser la policy continuer a commander des cibles obsoletes.

**Action :** en mode commande avec balle virtuelle, couper volontairement la
source balle ou attendre la fin du vol `trigger_mode`.

**Validation attendue :** le node publie un hold, `SafetyLimiter.reset()` evite
une reprise brutale, et les logs signalent la raison du stop.

### [ ] Optimiser la vitesse et caler les seuils materiel

**Etat actuel :** le chemin robot reel fonctionne avec balle virtuelle, mais il
est encore lent. Les valeurs par defaut restent conservatrices et ne sont pas
identifiees sur le robot.

**Pourquoi c'est bloquant :** des seuils trop hauts masquent des problemes de
suivi; des seuils trop bas arretent la boucle sans raison.

**Action :** enregistrer `/joint_states`, `catch_telemetry`,
`ros2 control list_controllers` et les logs pendant les essais balle virtuelle ;
tester des valeurs documentees de `v_safe_scale`, `a_safe`, `loop_budget_s`,
`max_tracking_error` et `start_pose_limit_rad`.

**Validation attendue :** seuils documentes dans `live_catch.yaml` ou dans une
note de test, avec vitesse slider, mode `action_mode`, modele et observations de
tracking.

## Priorite 2 - Brancher la vraie perception

### [x] Publier une vraie position depuis le tracker C++ (`BallState` natif)

**Etat actuel :** `publisher_member_function.cpp` appelle `publishBallPose()`,
qui publie maintenant `ur3e_catch_msgs/BallState` sur `ball_state`, en metres,
avec `header.frame_id` non vide. Le topic legacy `ball_position_3d_mm` reste
publie en option pour compatibilite.

**Pourquoi ce n'est plus bloquant :** `live_catch_node` peut consommer directement
le `BallState` natif, sans passer par `float32_adapter.py`.

**Action minimale :** terminee cote C++. `BallState.position` est en metres dans
le repere camera declare par `camera_frame_id`.

**Action cible restante :** valider le repere camera et la TF hand-eye reelle
pendant un essai perception seule.

**Validation attendue :** `ros2 topic echo /ball_state` montre `valid=true`,
une position en metres, un `frame_id` non vide, et une latence coherente dans
`CatchTelemetry.perception_age_s`.

### [x] Propager un timestamp evenement reel depuis le tracker C++

**Etat actuel :** le tracker C++ copie `poseTimestampUs` dans `BallPose3D` et le
publisher natif ancre ce temps evenement sur l'horloge ROS du nœud. L'adaptateur
legacy timestamp encore a la reception; `test_ball_node` timestamp a l'heure de
simulation/node, ce qui est attendu pour une source analytique.

**Pourquoi ce n'est plus bloquant :** `CatchTelemetry.perception_age_s` peut se
baser sur le stamp evenement du chemin natif.

**Action restante :** mesurer la latence reelle avec `latency_report` et verifier
que l'age reste positif et stable en acquisition camera.

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

### [ ] Decider et versionner le paquet `ur3e_sysid`

**Etat actuel :** `src/ur3e_sysid/` existe localement avec `run_sweep`,
`fit_gains` et des tests, mais il est non suivi par Git.

**Validation attendue :** soit le paquet est ajoute au depot avec ses tests, soit
la doc system-id le marque explicitement comme prototype local non requis pour un
clone propre.

## Sequence recommandee courte

1. Faire la calibration hand-eye et publier `base -> camera_optical`.
2. Mesurer/publier `wrist_3_link -> hoop_center`.
3. Valider la parite balle virtuelle `base` puis `camera_optical`.
4. Tester l'onglet `Test` en dry-run avec torch.
5. Tester `enable_command` sur fake hardware / URSim.
6. Rejouer robot reel avec balle virtuelle, vitesse reduite, E-stop en main, puis
   optimiser la lenteur.
7. Lancer le tracker C++ natif (`use_tracker:=true`) et verifier `/ball_state`.
8. Mesurer la latence reelle et seulement ensuite passer a une vraie balle lente.
