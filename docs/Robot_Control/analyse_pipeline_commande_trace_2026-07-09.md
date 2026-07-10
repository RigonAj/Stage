# Analyse du premier test réel Trace → commande robot (2026-07-09)

Diagnostic complet de la pipeline d'envoi et d'exécution des commandes après le
premier test caméra réelle + algorithme Trace + robot, et correctifs appliqués.

## 1. Contexte et symptômes observés

Commande utilisée par l'opérateur :

```bash
ros2 launch ur3e_live_catch live_catch.launch.py \
  use_tracker:=true \
  use_ball_regression:=true \
  enable_command:=true \
  model_path:=data/models/latest-left/policy_deterministic.onnx \
  camera_calibration_file:=recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml \
  ball_radius_mm:=45.0
```

Symptômes rapportés :

1. le robot ne bouge qu'un tout petit peu pendant les lancers ;
2. dans le web UI, l'état « command » s'active et se désactive à chaque
   instant ;
3. soupçon que la position de la balle n'est pas fournie à 60 Hz.

## 2. Cause racine : deux `live_catch_node` et deux producteurs `ball_state`

`live_catch.launch.py` ne démarre **ni le driver UR ni le web UI**. Pour que le
robot bouge et que le web UI affiche l'état, le stack virtual-ball
(`ur3e_catch_stack` → `virtual_ball_robot.launch.py`) tournait donc en
parallèle. Or ce stack démarrait **toujours** son propre `live_catch_node` et
`test_ball_node` (l'argument `use_test_ball` était codé en dur à `true` avant
le correctif de ce jour). Conséquences :

- **Deux `live_catch_node`.** Les deux publient `/catch_telemetry` à 60 Hz avec
  des `command_enabled` opposés (le launch manuel : `true` ; le stack :
  `false`). Le web UI recopie le dernier message reçu → l'état « command »
  clignote ON/OFF en permanence. C'est le symptôme n° 2, et la signature
  caractéristique d'un nœud dupliqué : avec un seul nœud ce booléen ne peut pas
  osciller. Les deux nœuds se disputent aussi le switch de contrôleur
  (`scaled_joint_trajectory_controller` ↔ `forward_position_controller`).
- **Deux producteurs sur `ball_state`.** En mode trigger au repos,
  `test_ball_node` publie quand même des heartbeats `valid=false` à 30 Hz sur
  `ball_state` (contrat d'idle). Le nœud de régression y publie le fit à
  60 Hz. Dans `live_catch_node._on_tick`, chaque message `valid=false` reçu en
  dernier déclenche `_controlled_stop` (commande de hold) **plus `_reset_sim`**
  qui réinitialise l'action mapper, la mémoire policy et l'EMA de vitesse. Le
  tick suivant repart d'un état vierge, fait un pas, puis est de nouveau
  réinitialisé. Le robot ne peut donc que « frétiller » : c'est le symptôme
  n° 1.
- Le service `~/enable_command` et les services de paramètres existaient en
  double sous le même nom : les appels de l'UI pouvaient atteindre n'importe
  lequel des deux nœuds.

Le runbook interdisait déjà le double producteur (« Only one ball producer may
publish a given topic ») mais rien ne le détectait ni ne l'empêchait.

## 3. Constat secondaire : la cadence Trace n'est pas un 60 Hz garanti

Le soupçon n° 3 est fondé, indépendamment du doublon :

- Le timer du tracker C++ est déclaré à 1 ms, mais `gui.Update()` fait le rendu
  raylib avec `SetTargetFPS(60)` : `EndDrawing` bloque, donc **toute la boucle
  de traitement (acquisition → filtre → trace → publication) est plafonnée au
  framerate de la GUI**. Si le rendu ralentit (vue 3D, `Max Events` élevé), la
  cadence de `ball_state_raw` descend sous 60 Hz.
- En mode trace, `publishTracePose()` publie une pose par **nouvelle fenêtre
  trace** (dédupliquée par `tMaxUs`), soit au mieux une par frame GUI, de
  manière irrégulière ; pendant le coast (`holdSeconds`), il publie au
  contraire à chaque callback (rafales), puis plus rien entre les lancers.
- Le tracker laisse `BallState.velocity = (0,0,0)` (« non fournie »).

**Atténuation déjà en place** : le `ball_regression_node` (activé par
`use_ball_regression:=true`, ce que l'opérateur a fait) décime les rafales
(`min_sample_interval_s`), refait un fit balistique robuste dans `base_link` et
republie un `BallState` **à 60 Hz réguliers** avec la vitesse dérivée du fit.
La policy voit donc bien du 60 Hz propre… tant que le tracker fournit assez
d'échantillons pendant le vol (gate de départ : 4 échantillons sur ≥ 60 ms).
Le découplage publication/rendu dans le tracker reste un travail futur (§6).

## 4. Réponse à la question d'architecture (nœud intermédiaire à coefficients)

L'idée « envoyer seulement les coefficients de la trajectoire à un nœud
intermédiaire qui publie la position à chaque instant » est déjà implémentée,
en mieux, par `ball_regression_node` :

- il refait le fit dans le repère robot (`base_link`) où la gravité est connue,
  au lieu de faire confiance au fit caméra du tracker ;
- il applique des gates de sécurité impossibles à faire côté tracker :
  distance de pop minimale, cohérence balistique (courbure −g/2), vitesse
  horizontale plancher, gel près du robot, fin de vol au sol ;
- il fournit vitesse et confiance, ce que le contrat `BallState` attend.

Transmettre les coefficients du fit interne du tracker ferait doublon avec ce
nœud et déplacerait le fit dans le mauvais repère. Recommandation : conserver
l'architecture actuelle `tracker → ball_state_raw → régression → ball_state`.

## 5. Correctifs appliqués (2026-07-09)

1. **Watchdog producteur exclusif dans `live_catch_node`**
   (`ur3e_live_catch/diagnostics.py`, timer 2 s) : erreur loggée si plusieurs
   publishers sur le topic balle ou sur le topic télémétrie, ou plusieurs
   nœuds nommés `live_catch_node`. Un conflit sur le topic **balle** bloque
   l'émission de commandes (fail closed) tant qu'il persiste ; un conflit
   télémétrie/nom avertit sans bloquer (l'autre nœud peut être celui qui
   commande). Tests : `test/test_diagnostics.py`.
2. **Détection de flapping dans le web UI**
   (`ur3e_web_ui/flapping.py`) : ≥ 3 transitions de `command_enabled` en 2 s
   ⇒ `catch_status.command_flapping=true`, le panneau Test affiche
   « command: CONFLICT — two live_catch_node running? » en rouge et le badge
   passe à « catch: CONFLICT » au lieu de clignoter. Tests :
   `test/test_flapping.py`.
3. **Source balle réelle intégrée au stack** : `virtual_ball_robot.launch.py`
   expose `use_test_ball` (défaut `true`), `use_tracker`,
   `camera_calibration_file`, `ball_radius_mm`, transmis à
   `live_catch.launch.py`. Le script `launch_ur3e_virtual_ball_stack.sh` gagne
   `--tracker` (désactive le test ball, active la régression par défaut),
   `--no-regression`, `--ball-radius MM`, `--camera-calib FILE`.
4. **`--stop` et le nettoyage pré-lancement tuent aussi** un tracker, un
   `ball_regression_node` ou un `live_catch.launch.py` manuel égarés — le
   scénario exact de l'incident ne peut plus survivre à un relancement du
   stack.

## 6. Ce qu'il reste à faire

- **Découpler la publication du rendu dans le tracker C++** : sortir le
  traitement événements + fit trace de la cadence `SetTargetFPS(60)` de
  raylib (thread de publication ou rendu 1 frame sur N), pour que
  `ball_state_raw` suive la cadence des batchs caméra même si la GUI lague.
  Refactor non trivial de `Gui::Update()` ; à faire hors session robot.
- **Vitesse** : une fois la perception validée, remonter `v_safe_scale`
  (0.5 dans `config/live_catch.yaml` → paliers du Test tab) — au premier test
  le robot était de toute façon bridé à mi-vitesse.
- **Rayon de balle** : `ball_radius_mm:=45.0` a été utilisé ; la profondeur
  Trace est proportionnelle à ce rayon. Vérifier au pied à coulisse qu'il
  s'agit bien du **rayon** (balle Ø 90 mm) et non du diamètre, sinon toute la
  profondeur est fausse d'un facteur 2.
- Mesurer les cadences réelles en conditions : `ros2 topic hz /ball_state_raw`
  et `/ball_state` pendant un lancer.

## 7. Procédure recommandée pour le prochain test réel

```bash
# 1. Repartir propre (tue driver, UI, live-catch, tracker, régression) :
ur3e_catch_stop

# 2. Un seul stack, source balle = tracker réel + régression :
ur3e_catch_stack --real --tracker --ball-radius 45.0 \
  --model-path data/models/latest-left/policy_deterministic.onnx

# (hold_side left : ajouter hold_side:=left en argument supplémentaire)

# 3. Vérifications avant d'armer la commande :
ros2 topic info /ball_state --verbose          # exactement 1 publisher
ros2 topic info /catch_telemetry --verbose     # exactement 1 publisher
ros2 topic hz /ball_state                      # ~60 Hz pendant un lancer
# et l'absence de "PRODUCER CONFLICT" dans les logs live_catch_node

# 4. Armer la commande depuis le Test tab du web UI uniquement.
```

Si le web UI affiche « catch: CONFLICT » ou que les logs montrent
`PRODUCER CONFLICT`, arrêter le doublon avant tout test : la commande est de
toute façon bloquée fail-closed sur le conflit de topic balle.
