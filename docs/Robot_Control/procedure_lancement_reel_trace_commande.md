# Procédure — Session réelle Trace + commande robot (dans l'ordre)

Procédure complète et ordonnée pour un test réel : caméra DVXplorer +
algorithme Trace + régression balistique + **mouvements du robot activés**,
modèle gauche `latest-left`, balle de rayon 45 mm. Chaque étape suppose que la
précédente a réussi. Établie le 2026-07-09 après le diagnostic
[analyse_pipeline_commande_trace_2026-07-09.md](analyse_pipeline_commande_trace_2026-07-09.md).

**Règle d'or : un seul stack.** Ne jamais lancer un
`ros2 launch ur3e_live_catch live_catch.launch.py` séparé pendant que le stack
tourne (doublon de `live_catch_node` et de producteur `ball_state` = robot qui
frétille + état commande qui clignote). Tout passe par `ur3e_catch_stack`.

---

## 0. Prérequis physiques (avant tout terminal)

- [ ] UR3e sous tension, freins relâchés, programme **External Control** chargé
      sur le pendant (IP du PC ROS : `192.168.0.3`).
- [ ] **E-stop à portée de main** pendant toute la session.
- [ ] Zone de travail dégagée (personne dans l'enveloppe du bras).
- [ ] Raquette/hoop montée **côté gauche** (vue de face) — doit correspondre au
      modèle `latest-left` et à `--hold-side left`.
- [ ] Caméra DVXplorer branchée, orientée vers la zone de lancer, fixée (la
      calibration hand-eye du 2026-07-09 suppose la caméra immobile).
- [ ] Balle **mesurée au pied à coulisse : rayon 45 mm = diamètre 90 mm**.
      Si votre balle fait 45 mm de *diamètre*, utilisez `--ball-radius 22.5` —
      toute la profondeur Trace est proportionnelle à ce rayon.
- [ ] PC ROS relié au robot : `ping 192.168.0.5` répond.

## 1. Terminal A — environnement et build

```bash
cd ~/Dv-Rosws/Dv-Rosws
source env.sh
source install/setup.bash

# Rebuild (obligatoire après les correctifs du 2026-07-09 : nouveaux modules
# diagnostics/flapping + options de launch) :
colcon build --symlink-install \
  --packages-select ur3e_catch_msgs ball_tracking_cpp ur3e_live_catch ur3e_web_ui
source install/setup.bash
```

## 2. Terminal A — repartir propre

```bash
ur3e_catch_stop
```

Tue driver, MoveIt, web UI, live-catch, **et aussi** tout tracker,
`ball_regression_node` ou `live_catch.launch.py` manuel égarés (élargi le
2026-07-09). En cas de doute sur des nœuds fantômes : `ros2 daemon stop` puis
`ros2 node list` doit être vide.

## 3. Terminal A — lancer le stack complet (dry-run au départ)

```bash
ur3e_catch_stack --real --tracker \
  --hold-side left \
  --ball-radius 45.0 \
  --model-path data/models/latest-left/policy_deterministic.onnx
```

Ce que ça démarre : driver UR + MoveIt + `live_catch_node` (**dry-run**,
`v_safe_scale=0.5` du config) + tracker Trace (intrinsèques
`intrinsics_from_mire_robust_constrained.xml` par défaut) + régression
balistique (60 Hz sur `ball_state`) + hoop TF **gauche** + web UI sur
`http://127.0.0.1:8080`.

Ne PAS utiliser `--enable-command` : la commande s'arme plus tard depuis l'UI,
après les vérifications.

## 4. Pendant — démarrer External Control

Sur le pendant : lancer le programme External Control (bouton play). Dans les
logs du driver, attendre `Robot connected to reverse interface`. Sans ça, le
robot ne bougera jamais (les contrôleurs rejettent tout).

## 5. Terminal B — publier le TF caméra (reste ouvert)

Le stack ne publie PAS le TF caméra ; sans lui, les positions de balle en
`camera_optical` sont rejetées.

```bash
cd ~/Dv-Rosws/Dv-Rosws
source env.sh && source install/setup.bash
python3 scripts/publish_camera_tf.py calibration/handeye_result.yaml
```

Laisser ce terminal ouvert toute la session.

## 6. Terminal C — vérifications avant tout lancer

```bash
cd ~/Dv-Rosws/Dv-Rosws && source env.sh && source install/setup.bash

# a) TF : les deux chaînes doivent résoudre (valeurs stables, pas d'erreur)
ros2 run tf2_ros tf2_echo base_link camera_optical
ros2 run tf2_ros tf2_echo base_link hoop_center           # translation x ≈ +0.5 côté gauche

# b) Producteur unique (le point qui a tué le test du 2026-07-09) :
ros2 topic info /ball_state --verbose        # Publisher count: 1 (ball_regression_node)
ros2 topic info /catch_telemetry --verbose   # Publisher count: 1 (live_catch_node)

# c) Joint states cohérents avec le pendant (piège ±2π du 2026-07-02) :
ros2 topic echo /joint_states --once
# comparer chaque joint au pendant ; un joint à ±6.28 rad alors que le pendant
# affiche ~0° => dévisser/jogguer ce joint avant d'armer (le start-pose gate
# bloquera sinon, limite 3.0 rad).
```

Dans les logs du **Terminal A**, vérifier :

- `Calibration loaded from recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml`
  (tracker) ;
- `policy model loaded: data/models/latest-left/policy_deterministic.onnx` et
  un backend (`onnxruntime` ou `torch`) chargé (live node) ;
- `Ball radius set to 45.0 mm` (tracker) ;
- **aucun** message `PRODUCER CONFLICT` (sinon un doublon tourne encore :
  refaire l'étape 2 — la commande resterait de toute façon bloquée fail-closed).

## 7. Fenêtre du tracker — régler la ROI

Dans la fenêtre « Event Viewer » : dessiner/ajuster la **work ROI** (rectangle
orange) pour couvrir la trajectoire de la balle et **exclure le robot, la main
du lanceur et les reflets** (Trace suppose que la balle est le seul objet
mobile dans la ROI). Si l'affichage lague, baisser `Max Events`.

## 8. Répétition à blanc (commande toujours OFF)

Lancer quelques balles réelles **sans commande armée** et vérifier :

```bash
ros2 topic hz /ball_state_raw   # pendant le vol : détections trace (irrégulier, OK)
ros2 topic hz /ball_state       # pendant le vol : ~60 Hz régulier (régression)
```

Dans le web UI (onglet Test, `http://127.0.0.1:8080`) :

- la balle apparaît au lancer, trajectoire plausible (profondeur cohérente
  avec la réalité — sinon revoir rayon/intrinsèques) ;
- le ghost vert (cible policy) réagit pendant le vol ;
- badge `catch: ready`, état `command: off`, **pas** de `catch: CONFLICT` ;
- `perception age` ≈ **−200 ms** pendant le vol : c'est NORMAL avec le
  `lead_time_s: 0.2` provisoire (la régression publie la position prédite
  200 ms dans le futur, stamp à l'instant d'évaluation). La balle affichée
  court en avance sur la vraie balle et le vol se termine ~200 ms avant
  l'impact réel. Réglage à chaud entre deux lancers :
  `ros2 param set /ball_regression_node lead_time_s 0.05` (borné [0, 1] s).

Ne passer à l'étape 9 que si tout est vrai.

## 9. Armer la commande (web UI uniquement)

1. Vérifier une dernière fois : E-stop en main, zone dégagée.
2. Onglet Test → cocher la case de confirmation → **command ON**.
3. Le live node bascule sur `forward_position_controller` ; log :
   `COMMAND mode armed ... forward_position_controller is active — streaming enabled`.
4. Premiers lancers à `v_safe_scale=0.5` (défaut config) : le robot est bridé à
   mi-vitesse, c'est voulu pour valider la boucle.
5. Pour accélérer : **command OFF** → monter d'un palier
   (`0.5 → 0.7 → 0.85 → 1.0`) dans l'onglet Test → **command ON** → relancer.
   Ne pas dépasser `1.0` tant que l'interception n'est pas fiable (au-delà =
   overdrive hors contrat d'entraînement).

## 10. Lancer la balle

Lancer depuis une zone visible de la caméra, trajectoire vers la raquette,
distance de départ > 0.6 m du robot (gate `min_pop_distance_m` de la
régression). Entre deux lancers, l'état `valid=false` de la régression remet le
robot en hold : normal.

## 11. En cas de problème

| Symptôme | Cause probable | Action |
|---|---|---|
| Badge `catch: CONFLICT` / `command: CONFLICT` | Deux `live_catch_node` (doublon de launch) | `ur3e_catch_stop` puis reprendre à l'étape 3 |
| Log `PRODUCER CONFLICT ... ball topic` | Deuxième producteur sur `ball_state` | Idem ; la commande est déjà bloquée fail-closed |
| Log `refusing to start command streaming` | Joint sur branche ±2π (`/joint_states` ≠ pendant) | Jogguer/dévisser le joint (ou reboot bras), re-vérifier étape 6c |
| Pendant : `Velocity ... exceeding joint limits` puis `Ignoring commands` | Set-point rejeté par le driver | Command OFF, vérifier étape 6c, réarmer ; si récurrent baisser `v_safe_scale` |
| `WATCHDOG stop -> holding: perception_stale` en vol | Trou de détection > 100 ms (ROI trop petite, balle sortie du champ, GUI qui lague) | Élargir la ROI, baisser `Max Events`, re-tester à blanc |
| Balle affichée à une mauvaise profondeur | Rayon faux ou mauvaises intrinsèques | Re-mesurer la balle (rayon ≠ diamètre), vérifier le log calibration |
| Robot ne bouge pas, `command: ON` | External Control pas lancé, ou hoop TF absent | Étape 4 ; `tf2_echo base_link hoop_center` |
| Le ghost bouge mais pas le robot | Contrôleur pas basculé | Log `switch_controller` ; vérifier `ros2 control list_controllers` |

## 12. Arrêt propre

1. Web UI : **command OFF** (le nœud rebascule sur le contrôleur trajectoire).
2. `Ctrl+C` dans le Terminal B (TF caméra).
3. Terminal A : `Ctrl+C` puis `ur3e_catch_stop`.
4. Pendant : arrêter External Control.

---

## Récapitulatif express (une fois la procédure maîtrisée)

```bash
# Terminal A
cd ~/Dv-Rosws/Dv-Rosws && source env.sh && source install/setup.bash
ur3e_catch_stop
ur3e_catch_stack --real --tracker --hold-side left --ball-radius 45.0 \
  --model-path data/models/latest-left/policy_deterministic.onnx
# Pendant : play External Control

# Terminal B
cd ~/Dv-Rosws/Dv-Rosws && source env.sh && source install/setup.bash
python3 scripts/publish_camera_tf.py calibration/handeye_result.yaml

# Terminal C — checks, puis lancers à blanc, puis command ON dans le web UI
ros2 topic info /ball_state --verbose && ros2 topic info /catch_telemetry --verbose
ros2 run tf2_ros tf2_echo base_link camera_optical
ros2 run tf2_ros tf2_echo base_link hoop_center
ros2 topic hz /ball_state
```
