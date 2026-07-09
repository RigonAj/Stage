# Procedure de test perception Trace avec balle reelle

Objectif: lancer la perception reelle DVXplorer, publier la position 3D de la
balle via l'algorithme Trace, puis verifier que la chaine live-catch peut
consommer cette position en dry-run. Cette procedure suppose que le driver UR3e
et le Web UI sont deja lances.

Ne pas activer la commande robot pendant ce premier test. Garder
`enable_command:=false` tant que la position balle, les TF et la telemetrie ne
sont pas plausibles.

## 0. Etat attendu avant de commencer

- Le driver UR3e publie `/joint_states` et la TF robot.
- Le Web UI est ouvert, mais il ne remplace pas les commandes ci-dessous.
- Le DVXplorer est disponible en USB. Le tracker C++ ouvre la camera directement
  via `dv_processing`; si un driver camera separe garde deja le peripherique
  ouvert, le tracker affichera que la camera n'est pas disponible.
- La calibration extrinseque existe dans `calibration/handeye_result.yaml`.
- La calibration intrinseque robuste du 2026-07-09 existe dans
  `recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml`.
- L'inference doit utiliser le modele gauche:
  `data/models/latest-left/policy_deterministic.onnx` (`hold_side=left`).

Dans un terminal commun:

```bash
cd ~/Dv-Rosws/Dv-Rosws
source env.sh
source install/setup.bash

ros2 node list
ros2 topic echo /joint_states --once
```

Si `/live_catch_node` existe deja parce que `ur3e_catch_stack` tourne, ne lance
pas un deuxieme `live_catch_node`. Arrete l'ancien stack avec `Ctrl-C` dans son
terminal, ou:

```bash
ur3e_catch_stop
```

## 1. Compiler et pointer le tracker sur la bonne intrinseque

Le tracker accepte le parametre ROS `camera_calibration_file`. Pour les tests
reels, toujours pointer explicitement vers la calibration intrinseque recente:

```text
recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml
```

Avant un test de profondeur, verifier que ce fichier et l'extrinseque existent:

```bash
cd ~/Dv-Rosws/Dv-Rosws
source env.sh

LATEST_INTRINSICS=recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml

test -f "$LATEST_INTRINSICS"
test -f calibration/handeye_result.yaml

colcon build --symlink-install --packages-select ur3e_catch_msgs ball_tracking_cpp ur3e_live_catch
source install/setup.bash
```

Au lancement du tracker, verifier dans le terminal qu'il affiche bien
`Calibration loaded from recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml`
avec `fx/fy` autour de 526 px pour la calibration du 2026-07-09. Si le tracker
affiche seulement l'ancien chemin racine
`calibration_camera_DVXplorer_DXA00265-2026_04_23_13_33_50.xml`, le lancement
n'utilise pas encore le bon parametre.

## 2. Publier les TF necessaires

Terminal TF camera, a laisser ouvert:

```bash
cd ~/Dv-Rosws/Dv-Rosws
source env.sh
source install/setup.bash

python3 scripts/publish_camera_tf.py calibration/handeye_result.yaml
```

Dans un autre terminal, verifier que `camera_optical` se resout vers le repere
policy `base_link`:

```bash
cd ~/Dv-Rosws/Dv-Rosws
source env.sh
source install/setup.bash

ros2 run tf2_ros tf2_echo base_link camera_optical
```

Si tu testes aussi l'inference live-catch, il faut `hoop_center`. Pour ce test,
le modele est **left**, donc la TF du cerceau doit aussi etre le montage gauche
(`+0.5 m` sur X de `wrist_3_link`). Si le stack qui a lance ton driver publie
deja ce TF gauche, ne le duplique pas. Sinon:

```bash
ros2 run tf2_ros static_transform_publisher 0.5 0 0 0 1 0 0 wrist_3_link hoop_center
```

Verification:

```bash
ros2 run tf2_ros tf2_echo base_link hoop_center
```

## 3A. Test perception seule: Trace publie `/ball_state`

Utilise ce mode pour verifier que la balle reelle est mesuree avant de lancer
l'inference policy.

```bash
cd ~/Dv-Rosws/Dv-Rosws
source env.sh
source install/setup.bash

ros2 run ball_tracking_cpp talker --ros-args \
  --params-file src/ur3e_live_catch/config/live_catch.yaml \
  -p pose_source:=trace \
  -p ball_state_topic:=ball_state \
  -p camera_frame_id:=camera_optical \
  -p camera_calibration_file:=recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml \
  -p ball_radius_mm:=20.0 \
  -p publish_legacy_pose:=false
```

Dans la fenetre du tracker:

- laisser `Trace use raw input` sur `Undist`;
- laisser `Circle fit` sur `OFF`;
- regler `Ball radius (mm)` sur le vrai rayon de la balle;
- reduire `Max Events` dans le panneau Trace si l'affichage 2D lagge: ce
  slider limite les evenements dessines et le clustering, sans retirer les
  evenements complets utilises par l'accumulation Trace;
- commencer avec `Trace ms` entre 80 et 150 ms si le defaut 40 ms ne donne pas
  assez de support;
- regler la ROI orange sur le couloir de lancer, en excluant le bras, le support
  et les mains autant que possible;
- activer `Edge refine` puis `Width fit` si la profondeur saute trop.

Monitorer dans un autre terminal:

```bash
cd ~/Dv-Rosws/Dv-Rosws
source env.sh
source install/setup.bash

ros2 topic echo /ball_state --once
ros2 topic hz /ball_state
```

Pendant un lancer valide, `/ball_state` doit avoir:

- `header.frame_id: camera_optical`;
- `valid: true`;
- `position` en metres;
- `position.z > 0` pour une balle devant la camera;
- `confidence: 1.0` sur une fenetre fraiche.

## 3B. Test inference live-catch en dry-run

Arrete le tracker du mode 3A avant d'utiliser ce mode, sinon deux producteurs
publieront la balle. Ce lancement ne relance ni le driver UR ni le Web UI; il
ajoute seulement le tracker et `live_catch_node`.

Sans regression balistique, `/ball_state` reste la sortie Trace brute en
`camera_optical` et `live_catch_node` calcule la vitesse par difference finie:

```bash
cd ~/Dv-Rosws/Dv-Rosws
source env.sh
source install/setup.bash

ros2 launch ur3e_live_catch live_catch.launch.py \
  use_tracker:=true \
  use_ball_regression:=false \
  enable_command:=false \
  model_path:=data/models/latest-left/policy_deterministic.onnx \
  camera_calibration_file:=recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml \
  ball_radius_mm:=20.0
```

Avec regression balistique, le tracker publie les detections Trace sur
`/ball_state_raw`, puis `ball_regression_node` publie `/ball_state` en
`base_link` a 60 Hz avec vitesse estimee. Utiliser ce mode apres avoir vu que
le mode brut detecte bien la balle:

```bash
cd ~/Dv-Rosws/Dv-Rosws
source env.sh
source install/setup.bash

ros2 launch ur3e_live_catch live_catch.launch.py \
  use_tracker:=true \
  use_ball_regression:=true \
  enable_command:=false \
  model_path:=data/models/latest-left/policy_deterministic.onnx \
  camera_calibration_file:=recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml \
  ball_radius_mm:=20.0
```

Verification:

```bash
ros2 topic echo /ball_state --once
ros2 topic echo /catch_telemetry --once
ros2 topic hz /catch_telemetry
ros2 run ur3e_live_catch latency_report
```

Dans le Web UI, l'onglet Test doit afficher la balle et le fantome de cible
pendant le vol. Verifier aussi les logs de `live_catch_node`: le modele policy
doit etre charge via `onnxruntime` ou `torch`. Sans backend policy, la chaine
peut encore publier de la telemetrie, mais l'action restera nulle et ce n'est
pas un vrai test d'inference. En dry-run, le robot reel ne recoit aucune
commande.

## 4. Lancer la balle

Pour le premier essai:

- robot immobile, loin du couloir de lancer;
- personne dans la zone robot;
- commande robot desactivee;
- lancer dans la ROI et dans le champ camera complet;
- eviter que la main reste dans la ROI apres le depart;
- commencer avec une trajectoire laterale propre, pas directement vers la
  camera.

La premiere validation est visuelle et ROS:

```bash
ros2 topic echo /ball_state --once
ros2 topic echo /catch_telemetry --once
```

Si la position devient plausible dans `camera_optical` puis dans
`catch_telemetry.ball_base`, l'etape perception -> TF -> inference dry-run est
validee pour ce lancer.

## Points d'attention

- **Intrinseque consommee par le tracker.** Pour le reel, passer explicitement
  `camera_calibration_file:=recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml`.
  Le lancement `ur3e_live_catch live_catch.launch.py` utilise ce chemin par
  defaut, mais il reste volontairement visible dans les commandes de test.
  Verifier le log `Calibration loaded from ...` avant de lancer la balle.
- **Extrinseque actuelle.** Le fichier du 2026-07-09 contient 6 poses. Les
  residus sont bons pour un premier essai (`pixel rms` environ 1.44 px), mais
  le runbook cible plutot 15 a 20 poses pour verrouiller physiquement la
  calibration. Refaire plus de poses si la position en `base_link` semble
  tournee ou translatee.
- **Un seul producteur de balle.** Ne pas faire tourner `test_ball_node`,
  `ball_tracking_cpp` direct et `live_catch.launch.py use_tracker:=true` en
  meme temps sur le meme topic.
- **Repere obligatoire.** `BallState.header.frame_id` doit rester
  `camera_optical` en sortie tracker brute. Le live-catch rejette les frames
  vides ou inconnues.
- **TF camera.** Si `base_link <- camera_optical` manque, la perception brute
  peut publier, mais l'inference ne peut pas construire correctement
  l'observation.
- **TF hoop.** En dry-run, `live_catch_node` peut utiliser un fallback disque;
  en commande robot, l'absence de `base_link -> hoop_center` est un echec ferme.
  Avec le modele left, le TF doit etre le montage gauche
  `wrist_3_link -> hoop_center = (0.5, 0, 0)` et quaternion `(0, 1, 0, 0)`.
- **Modele left.** Le `model_path` de cette procedure est
  `data/models/latest-left/policy_deterministic.onnx`. Le metadata annonce
  `hold_side=left`; il faut que le montage physique, le TF `hoop_center`, le
  toggle Web UI et le modele charge soient coherents.
- **Profondeur Trace.** La profondeur est proportionnelle au rayon balle et
  inversement proportionnelle a la largeur apparente. Rayon faux, intrinseque
  obsolete, ROI polluee ou trace trop courte donnent une distance fausse.
- **Rayon de balle.** Remplacer `20.0` dans `ball_radius_mm:=20.0` par le vrai
  rayon mesure en millimetres. Le parametre initialise le tracker au lancement;
  le slider `Ball radius (mm)` du panneau Option permet encore d'ajuster en
  live.
- **ROI.** Trace suppose que la balle est le mouvement dominant dans la ROI.
  Le bras, le support, une main ou un reflet rapide dans la ROI peuvent corrompre
  le ruban.
- **Regression.** Avec `use_ball_regression:=true`, `/ball_state` n'est plus la
  mesure Trace brute mais une estimation balistique en `base_link`; regarder
  `/ball_state_raw` pour diagnostiquer le tracker. Avant que la regression ait
  assez de support, `/ball_state` peut publier `valid: false`; ce n'est pas
  forcement un echec du tracker.
- **Backend policy.** Pour tester l'inference live-catch, `live_catch_node` doit
  charger un modele avec `onnxruntime` ou `torch`. Si le backend manque, les
  observations et la telemetrie peuvent exister, mais la policy ne produit pas
  une action utile.
- **Commande robot.** Ne passer a `enable_command=true` qu'apres plusieurs
  lancers plausibles en dry-run, E-stop en main, vitesses reduites et
  `v_safe_scale` choisi dans l'UI pendant que la commande est desactivee.

## Arret

Couper d'abord toute commande robot dans l'UI si elle a ete activee plus tard,
puis arreter les terminaux de test avec `Ctrl-C`. Si tu as lance le stack
combine:

```bash
ur3e_catch_stop
```
