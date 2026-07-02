# Synthese generale du projet

Ce document resume les grandes lignes du projet a partir de `Stage_summary.tex`,
du `README.md` et des documents presents dans `docs/`. Il sert de point d'entree
rapide pour comprendre l'objectif global, les briques techniques et l'etat
d'avancement.

## Objectif global

Le projet vise a detecter et estimer en 3D la position d'une balle rapide avec
une camera evenementielle DVXplorer, puis a rendre cette information exploitable
par un robot UR3e pour une tache d'interception.

Le travail s'inscrit dans le cadre d'un stage a l'Institut Pascal
(UMR 6602 UCA/CNRS), dans l'axe Image, Systemes de Perception, Robotique. Il
prolonge une ligne de travail deja presente dans `docs/Antonio_Stage.pdf` :
utiliser une camera evenementielle pour fournir a des taches robotiques
dynamiques une perception plus rapide qu'une camera classique, avec un lien vers
l'apprentissage par renforcement et le transfert simulation-reel.

La difficulte principale vient du caractere tres dynamique du probleme. Une
balle rapide se deplace de plusieurs centimetres en quelques millisecondes. Le
systeme doit donc produire une position 3D avec une faible latence, une precision
suffisante et des timestamps coherents pour que le robot puisse anticiper le
point d'interception.

Le projet combine quatre grands axes :

- perception evenementielle et estimation 3D de la balle ;
- calibration intrinseque de la camera et calibration extrinseque camera-base
  robot ;
- commande et supervision du robot UR3e sous ROS 2 ;
- transfert simulation-reel d'une politique PPO entrainee dans Isaac Lab.

## Vue d'ensemble de l'architecture

La chaine complete peut etre lue ainsi :

```text
Camera evenementielle DVXplorer
  -> acquisition d'evenements
  -> filtrage, undistortion, selection de la balle
  -> estimation 2D par circle fitting ou Trace
  -> conversion en position 3D dans le repere camera
  -> regression / filtrage de trajectoire
  -> publication ROS 2
  -> transformation vers le repere base du robot
  -> observation PPO / commande robot
  -> UR3e
```

Le workspace contient plusieurs briques :

- `src/Ball_Tracking_Cpp/` : suivi de balle evenementiel en C++/ROS 2, interface
  Raylib/raygui, algorithmes circle fitting et Trace.
- `scripts/` : outils de calibration, conversion de donnees, lancement du stack
  robot, publication TF et utilitaires UR3e.
- `src/ur3e_web_ui/` : interface web FastAPI/Three.js pour visualiser et
  controler le UR3e.
- `src/ur3e_rollout_replay/` : validation et rejeu securise de rollouts Isaac
  Lab sur le robot.
- `data/ur3e_rollouts/` : exports PPO, rollouts et politique deterministe.
- `docs/` : documentation technique sur les algorithmes, la calibration, le
  controle robot, le sim-to-real et la compilation du rapport.

## Perception evenementielle

La camera evenementielle ne fournit pas des images completes a frequence fixe.
Elle emet des evenements lorsqu'un pixel observe une variation de luminosite.
Chaque evenement contient une position image, un timestamp et une polarite.

Ce fonctionnement est adapte aux objets rapides, car il limite la quantite de
donnees et donne une information temporelle fine. En contrepartie, les
algorithmes doivent travailler sur un nuage d'evenements irregulier, bruite et
fortement dependant du mouvement.

Deux methodes principales sont documentees.

### Circle fitting

La premiere approche ajuste un cercle sur le cluster d'evenements correspondant
a la balle. Le centre et le rayon apparent permettent ensuite de calculer la
profondeur :

```text
Z = fx * R / r
```

Cette methode est simple et efficace lorsque la projection de la balle reste
proche d'un cercle. Elle utilise notamment :

- une fenetre temporelle recente ;
- un filtrage et une correction de distorsion ;
- un clustering DBSCAN ;
- une validation par polarites positives/negatives ;
- un second ajustement apres rejet des points aberrants ;
- une regression 3D de trajectoire.

Sa limite est structurelle : toute la profondeur depend du rayon apparent. Pour
une balle rapide, les evenements forment une trainee. Le cercle ajuste peut alors
surestimer le rayon, ce qui donne une profondeur trop faible et rapproche
artificiellement la balle de la camera. A 1,47 m, une erreur de 1 pixel sur le
diametre represente deja environ 7 cm d'erreur de profondeur.

### Methode Trace

La methode Trace est devenue l'approche principale du projet. Elle part du
constat qu'une balle rapide produit un ruban evenementiel plus qu'un cercle
instantane.

Le principe est de mesurer la largeur de cette trace, puis d'utiliser cette
largeur comme diametre apparent de la balle :

```text
Z = f_eff * diametre_reel / largeur_px
```

Le pipeline Trace suit les grandes etapes suivantes :

1. accumuler les evenements recents dans une fenetre qui suit la balle ;
2. estimer la direction principale de la trace par PCA globale et PCA locales ;
3. projeter les evenements dans un repere local `s/h`, avec `s` le long de la
   trace et `h` dans son epaisseur ;
4. decouper la trace en bins spatiaux ;
5. detecter dans chaque bin les deux bords soutenus, en ignorant les extremes
   isoles ;
6. filtrer les bins incoherents ;
7. ajuster les courbes haut, milieu et bas du ruban ;
8. mesurer localement la largeur perpendiculaire a la ligne mediane ;
9. convertir les mesures en points 3D ;
10. filtrer les outliers 3D et ajuster une trajectoire.

La regression de trajectoire utilise un modele simple :

```text
X(t), Y(t) : lineaires
Z(t)      : quadratique
```

Une version ponderee favorise les points recents et reduit l'influence des
mesures aberrantes. Cela rend la trajectoire plus utilisable en temps reel.

## Calibration

Le projet distingue deux calibrations.

### Calibration intrinseque DVXplorer

La calibration intrinseque estime les parametres de la camera :

- focales `fx`, `fy` ;
- centre optique `cx`, `cy` ;
- coefficients de distorsion ;
- resolution et erreur de reprojection.

Une premiere calibration a ete faite avec les outils iniVation et une mire
d'echiquier. Elle a donne une erreur d'environ 0,486 px. Une seconde approche a
ete developpee avec une mire evenementielle clignotante affichee sur ecran. Les
scripts principaux sont :

- `scripts/event_mire_calibration.py` : capture et association des blobs de mire ;
- `scripts/calibrate_intrinsics_from_mire.py` : calibration OpenCV depuis les
  observations JSON ;
- `docs/Context/calibration_python_architecture.md` : description du pipeline.

La mire utilise des disques lumineux sur fond noir, parfois avec un point
manquant et une ancre pour lever les ambiguites. Les evenements ON/OFF sont
accumules positivement, les blobs sont detectes, puis associes a des points 3D
connus dans le plan de l'ecran.

### Calibration extrinseque camera-base robot

Pour que le robot utilise la position de balle, il faut transformer les points du
repere camera vers le repere `base` du UR3e. La transformation recherchee est :

```text
T_base_camera
```

La procedure documentee est de type eye-to-hand :

- la camera DVXplorer est fixe dans l'environnement ;
- un smartphone affiche une mire clignotante ;
- le smartphone est fixe sur un support imprime en 3D monte sur `tool0` ;
- le robot deplace la mire a plusieurs poses connues ;
- la camera estime `T_camera_mire` par `solvePnP` ;
- le robot fournit `T_base_tool0` par TF ;
- un solveur hand-eye estime `T_base_camera` et `T_tool0_mire`.

Les documents insistent sur plusieurs points critiques :

- utiliser les bonnes conventions OpenCV et inverser les entrees attendues par
  `calibrateRobotWorldHandEye` ;
- convertir explicitement les millimetres camera vers les metres ROS ;
- definir clairement les frames `base`, `tool0`, `mire`, `camera_optical` ;
- eviter la confusion `base` / `base_link` ;
- capturer 15 a 20 poses variees, statiques, avec des orientations non
  paralleles ;
- verifier la stabilite par leave-one-out, residus pixels, coherence CAD et
  plausibilite dans le viewer 3D.

Les outils `serve_phone_mire.py`, `event_mire_calibration.py --external-mire`,
`solve_handeye.py`, `run_handeye_session.sh` et `publish_camera_tf.py` preparent
cette chaine. La session physique finale de hand-eye reste un point de
validation important.

## Simulation et validation

Les sequences simulees servent a tester l'algorithme dans un cadre controle. Le
workflow est :

```text
Isaac Sim video
  -> conversion en evenements avec v2e
  -> estimation 3D par l'algorithme
  -> comparaison avec la verite terrain Isaac Sim
```

Les sequences documentees utilisent notamment :

- resolution 640 x 480 ;
- entree video 500 fps ;
- rayon de balle 0,02 m ;
- calibration synthetique `fx = fy = 520 px` ;
- resolution temporelle evenementielle de 200 us.

La simulation separe deux problemes :

- si l'algorithme echoue en simulation, l'erreur vient du modele ou du traitement ;
- s'il fonctionne en simulation mais devient instable en reel, il faut chercher
  du cote de la calibration, du bruit, de l'eclairage, de la densite
  d'evenements ou des conditions experimentales.

## Controle du robot UR3e

Le cote robot est documente autour d'un stack ROS 2 Humble consolide dans ce
workspace.

### Driver et lancement

La voie recommandee utilise les paquets binaires `ros-humble-ur` actuels, avec
un UR3e configure en External Control. Les informations locales documentees sont :

- robot : UR3e ;
- IP robot : `192.168.0.5` ;
- IP PC filaire : `192.168.0.3` ;
- PolyScope : 5.12.4 ;
- driver : `ros-humble-ur` 2.13.0.

Le script `scripts/launch_ur3e_stack.sh`, expose par `source env.sh` via
`ur3e_stack`, lance le driver, attend `/joint_states` et le controleur de
trajectoire, puis demarre l'interface web.

Les docs indiquent que l'ancien chemin legacy est obsolete et garde seulement
pour historique.

### Interface web

Le paquet `ur3e_web_ui` fournit une interface navigateur pour :

- visualiser le robot en 3D depuis l'URDF et `/joint_states` ;
- lire les positions articulaires, vitesses et pose TCP `base -> tool0` ;
- faire du jog articulaire et un retour home ;
- envoyer une cible TCP apres validation IK MoveIt ;
- valider, previsualiser et executer des rollouts Isaac Lab retimes ;
- enregistrer et rejouer des poses de calibration hand-eye ;
- afficher le support telephone et la camera calibree dans le viewer.

Le backend applique des garde-fous avant tout mouvement :

- `/joint_states` vivant ;
- action server pret ;
- External Control en cours d'execution ;
- speed scaling superieur a 0 % ;
- controleur `scaled_joint_trajectory_controller` actif ;
- confirmation explicite pour les mouvements physiques ;
- rejet si le robot bouge deja pour les actions sensibles.

Une partie importante du travail robot a ete le diagnostic de problemes reels :
External Control arrete, speed slider a 0 %, controleur inactif, branches IK
MoveIt instables et confusion de conventions RPY. Ces problemes ont conduit a
des checks plus explicites dans l'UI et a une selection d'IK plus proche de
l'etat courant.

### Replay de rollouts

Le paquet `ur3e_rollout_replay` permet de relire les rollouts Isaac Lab stockes
dans `data/ur3e_rollouts/.../rollouts_10_episodes.json`.

Le point important est que le robot reel ne doit pas rejouer directement la
commande brute de la politique. Chaque sample contient :

- `joint_position_target_rad` : cible brute commandee en simulation ;
- `joint_position_before_rad` : position effectivement atteinte en simulation.

Les cibles brutes peuvent impliquer des vitesses tres elevees, non realistes
pour le robot. Le replay utilise donc par defaut `joint_position_before_rad`,
puis retime le mouvement avec des limites de vitesse et acceleration
conservatrices. Ce replay est utile pour verifier un mouvement appris, mais il
reste open-loop : il ne suffit pas a attraper une balle reelle.

## PPO, sim-to-real et boucle fermee

La politique d'interception a ete entrainee dans Isaac Lab avec PPO. Les exports
disponibles incluent une politique TorchScript et ONNX. Les resultats de stage
rapportent une convergence autour de 15 000 iterations, avec un entrainement
pousse jusqu'a environ 35 000 iterations.

Le passage au reel demande cependant de corriger l'ecart simulation-reel :

- les actions sim actuelles sont trop agressives ;
- la simulation ne borne pas assez les vitesses et efforts ;
- la latence perception/commande n'est pas encore correctement modelisee ;
- la balle simulee est trop rapide pour une enveloppe sure du UR3e ;
- l'observation de la politique doit etre reconstruite exactement en live.

La boucle fermee live est maintenant amorcee dans le workspace ROS. Au
2026-06-22, les paquets `ur3e_catch_msgs` et `ur3e_live_catch` existent, le
chemin perception -> observation 33-D -> policy -> safety -> streaming est
cable, et la commande robot est protegee par `enable_command=false` par defaut.
L'architecture retenue reste un noeud Python mono-processus pour limiter la
latence :

```text
BallState + /joint_states
  -> transformation balle vers base
  -> filtrage vitesse balle
  -> reconstruction observation 33-D
  -> inference policy
  -> action brute
  -> clip + rate-limit + watchdog
  -> streaming vers forward_position_controller
```

Deux paquets portent cette boucle :

- `ur3e_catch_msgs` : messages `BallState` et `CatchTelemetry` ;
- `ur3e_live_catch` : boucle live, source de balle test, observation, inference,
  action, securite et streaming.

Le message `CatchTelemetry` expose maintenant l'observation, l'action brute, la
cible articulaire sure, la balle en `base_link`, la vitesse balle, l'age perception,
le temps de calcul de boucle et l'etat `command_enabled`. La question du scaler
de l'export courant est tranchee : le TorchScript reproduit les actions
enregistrees sans scaler externe. Le point encore ouvert cote perception reelle
est le timestamp d'evenement de `BallState`, indispensable pour un budget de
latence fiable.

## Etat actuel synthetique

Les elements deja bien couverts par le depot sont :

- le rapport de stage `Stage_summary.tex` et sa compilation ;
- la chaine C++/ROS 2 de tracking evenementiel ;
- la methode Trace, avec documentation visuelle et interface de diagnostic ;
- les scripts de calibration intrinseque par mire clignotante ;
- les outils de preparation hand-eye camera-base robot ;
- le stack UR3e avec driver, UI web, jog, TCP target, replay, calibration tab et
  onglet `Test` pour la balle virtuelle/live catch ;
- la validation/replay de rollouts Isaac Lab ;
- les documents d'architecture et d'etat d'implementation pour le sim-to-real et
  la boucle live.

Les points qui restent critiques ou a finaliser sont :

- valider physiquement la calibration hand-eye `T_base_camera` ;
- publier les TF statiques `base -> <camera_frame>` et
  `wrist_3_link -> hoop_center` avec la vraie geometrie ;
- figer les composantes observation 3 / 8 / 10 avec la source Isaac et la
  geometrie reelle du montage ;
- tester sur robot reel avec balle virtuelle avant toute vraie balle ;
- valider la vraie perception C++ native `BallState` en perception seule ;
- reentrainer ou adapter la simulation avec vitesses, efforts, latences et bruit
  realistes ;
- mesurer la latence bout-en-bout avec perception reelle avant tout essai
  dynamique avec vraie balle.

## Documents utiles par sujet

- Rapport principal : `Stage_summary.tex`
- Vue rapide du tracker : `README.md`
- Trace : `docs/trace_algorithm_explanation.html`,
  `docs/Context/algo_trace_graph.html`
- Circle fitting : `docs/Context/algo_circle_fitting_graph.html`
- Chaine de traitement : `docs/Context/chaine_traitement_graph.html`
- Calibration intrinseque : `docs/Context/calibration_python_architecture.md`
- Calibration camera-base : `docs/Robot_Control/ur3e_camera_base_calibration.md`
- Stack robot : `docs/Robot_Control/ur3e_robot_control_architecture.md`
- Interface web : `docs/Robot_Control/ur3e_web_ui.md`
- Replay reel : `docs/Robot_Control/ur3e_real_robot_replay.md`
- Driver UR3e : `docs/Robot_Control/ur3e_current_driver_setup.md`
- Probleme de mouvement UR3e : `docs/Robot_Control/ur3e_motion_issue_resolution.md`
- Sim-to-real PPO : `docs/Robot_Control/ur3e_ball_catch_sim_to_real.md`
- Architecture boucle live : `docs/Robot_Control/ur3e_live_catch_architecture.md`
- État d'implémentation boucle live : `docs/Robot_Control/ur3e_live_catch_implementation_status.md`
- Reste a faire : `docs/reste_a_faire.md`
- Compilation du rapport : `docs/latex_compilation.md`
- Contexte historique : `docs/Antonio_Stage.pdf`
- Consignes de rapport : `docs/Consignes pour le rapport M1_M2MTN-1.pdf`

## Lecture rapide

En une phrase, le projet construit une chaine perception-robotique pour attraper
une balle rapide : la camera evenementielle mesure une trace, l'algorithme Trace
en deduit une trajectoire 3D, la calibration place cette trajectoire dans le
repere UR3e, puis une politique PPO ou un controleur doit commander le robot en
boucle fermee avec des contraintes strictes de latence et de securite.
