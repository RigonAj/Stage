# Incoherences code / docs / logique

Date de revue : 2026-06-22

Portee : documents sous `docs/`, README de packages cites par ces documents,
scripts de calibration, et code ROS/C++ directement implique dans la boucle
UR3e live-catch. Les changements non commites presents dans le workspace sont
consideres comme l'etat courant du projet.

## Resume court

Depuis la revue du 2026-06-20, l'etat de la boucle live UR3e a beaucoup change :
`ur3e_live_catch` contient maintenant le chemin ActionMapper -> SafetyLimiter ->
CommandStreamer, le streaming vers `forward_position_controller` est cable
derriere `enable_command` (defaut `false`), la latence est instrumentee,
l'onglet web UI `Test` existe, et la question du scaler est tranchee par le
test policy (`.ts` auto-contenu, pas de `policy_scaler.json` requis pour
l'export courant).

Les incoherences restantes se concentrent donc sur les derniers points qui
empechent un essai realiste : publication C++ de balle encore vide, horodatage
evenement non fourni par la vraie perception, calibration extrinseque non
validee physiquement, TF statiques camera/hoop a fournir, modele canonique
`data/models` absent, chemins hand-eye divergents, et quelques documents de
reprise encore obsoletes.

Voir aussi `docs/reste_a_faire.md` pour la checklist d'execution.

## Constats resolus ou devenus obsoletes depuis le 2026-06-20

### R1. `streaming.py` maintenant present

**Ancien constat :** `streaming.py` et la couche de commande etaient mentionnes
mais absents/non cables.

**Etat 2026-06-22 :** resolu. `src/ur3e_live_catch/ur3e_live_catch/streaming.py`,
`limits.py`, `latency.py` et les tests associes existent. `live_catch_node.py`
calcule une cible sure meme en dry-run et peut publier sur
`/forward_position_controller/commands` quand `enable_command=true`.

### R2. Safety / commande robot non cablees

**Ancien constat :** la boucle live ne commandait jamais le robot.

**Etat 2026-06-22 :** partiellement resolu cote code. Le chemin commande est
cable derriere `enable_command`, avec bascule de controleur, watchdog et refus
de commander si aucun modele n'est charge. Il reste a valider sur robot reel.

### R3. Scaler SKRL non tranche

**Ancien constat :** impossible de savoir si la normalisation etait embarquee
dans l'export TorchScript.

**Etat 2026-06-22 :** resolu pour l'export courant. Le test d'equivalence policy
avec torch dans `.venv` reproduit `action_normalized` a max `|delta| = 4.6e-6`
sans scaler externe. Le runtime peut donc utiliser le `.ts` tel quel pour ce
modele.

### R4. README `ur3e_live_catch` obsolete

**Ancien constat :** le README interne listait encore des fichiers a ajouter.

**Etat 2026-06-22 :** resolu. `src/ur3e_live_catch/README.md` decrit maintenant
les modules existants, les launchs, `enable_command`, la latence et l'onglet
`Test`.

## Incoherences ou risques encore ouverts

## 1. Topic `ball_position_3d_mm` : le code C++ publie encore un message vide

**Type :** bug fonctionnel / contrat ROS ambigu

**Constat :** la documentation generale annonce une position de balle sur
`ball_position_3d_mm`, mais le tracker C++ actuel publie un
`Float32MultiArray` vide.

**Preuves :**

- `README.md` decrit encore la sortie `ball_position_3d_mm`.
- `src/Ball_Tracking_Cpp/src/publisher_member_function.cpp` cree le publisher
  `ball_position_3d_mm`.
- `publishBallPose()` construit `std_msgs::msg::Float32MultiArray msg` puis le
  publie sans remplir `msg.data`.
- `src/ur3e_live_catch/ur3e_live_catch/float32_adapter.py` rejette le flux si
  `len(msg.data) < 3`.

**Impact :** la vraie perception ne peut pas alimenter la boucle live. Les tests
actuels passent avec `test_ball_node`, mais pas avec le tracker C++ tel quel.

**Correction suggeree :** publier au minimum `[x, y, z]` avec ordre, unite et
repere documentes, puis migrer vers une publication native
`ur3e_catch_msgs/BallState` horodatee au temps d'evenement.

## 2. `BallState.header.stamp` : le temps d'evenement n'est pas encore garanti

**Type :** contrat message / latence

**Constat :** `BallState.msg` exige un timestamp d'evenement, mais les producteurs
intermediaires utilisent l'heure courante ou l'heure de reception.

**Preuves :**

- `src/ur3e_catch_msgs/msg/BallState.msg` documente `stamp` comme temps
  d'evenement.
- `float32_adapter.py` utilise le temps de reception car le flux legacy n'a pas
  de timestamp source.
- `test_ball_node.py` utilise l'heure du noeud, ce qui est correct pour une
  balle analytique mais pas equivalent a une acquisition camera.
- `CatchTelemetry.perception_age_s` depend directement de ce champ.

**Impact :** le budget de latence mesure est utile en test, mais fragile en
perception reelle tant que le timestamp de capture camera/tracker n'est pas
propager.

**Correction suggeree :** faire publier le tracker natif en `BallState` avec un
timestamp issu des evenements utilises pour l'estimation, ou ajouter un champ de
diagnostic clair distinguant `event_time`, `receive_time` et `sim_time`.

## 3. README `ur3e_catch_msgs` encore obsolete

**Type :** documentation de package obsolete

**Constat :** `src/ur3e_catch_msgs/README.md` dit encore que `package.xml`,
`CMakeLists.txt` et les fichiers `.msg` sont a ajouter, alors qu'ils existent.

**Preuves :**

- `src/ur3e_catch_msgs/CMakeLists.txt` genere deja `BallState.msg` et
  `CatchTelemetry.msg`.
- `src/ur3e_catch_msgs/package.xml` existe.
- Les messages sont consommes par `ur3e_live_catch` et `ur3e_web_ui`.

**Impact :** un lecteur qui reprend seulement le README du package croit que le
contrat de messages n'est pas implemente.

**Correction suggeree :** transformer le README en description d'utilisation
actuelle : champs de messages, build, consumers/producers, statut du timestamp.

## 4. Modele canonique `data/models` absent et fallback date encore actif

**Type :** configuration / reproductibilite

**Constat :** `data/models` est documente comme emplacement canonique, mais ne
contient actuellement que son README. `live_catch_node.py` retombe donc sur
l'export date `data/ur3e_rollouts/2026-05-26.../policy_deterministic.ts`.

**Preuves :**

- `data/models/README.md` demande un `policy_deterministic.ts` et des
  metadonnees associees.
- `find data/models -type f` ne liste que `data/models/README.md`.
- `live_catch_node.py` definit `CANONICAL_MODEL` puis `FALLBACK_MODEL`.
- L'export fallback est l'ancien modele dont la semantique verified est
  `joint_position_target_rad = action_normalized * 0.5`.

**Impact :** un lancement sans `model_path` explicite peut charger un modele
utile pour les tests actuels, mais pas forcement le modele final a deployer.

**Correction suggeree :** mettre le modele choisi dans `data/models/` ou le
lier explicitement, avec `policy_metadata.json` et une note claire sur
`action_semantics`, `normalization=embedded`, `dt_s` et limites attendues.

## 5. Semantique d'action a figer par modele

**Type :** logique de controle / risque robot

**Constat :** le code live gere deux modes (`faithful` et `safe`), tandis que
les documents sim-to-real decrivent la cible incrementale comme objectif de
reentrainement. La semantique actuelle doit rester explicite par export de
policy.

**Preuves :**

- `action.py` implemente `faithful` : `target = action * 0.5`, action precedente
  brute dans l'observation.
- `action.py` implemente aussi `safe` : cible incrementale avec action clippee.
- `test_policy_equivalence.py` confirme que l'export courant reproduit les
  actions de l'ancien rollout.
- `ur3e_ball_catch_sim_to_real.md` indique que les nouvelles policies issues de
  la correction sim doivent etre regenerees et comparees separement.

**Impact :** une policy entrainee avec une semantique peut etre rejouee avec une
autre si le choix n'est pas encode dans les metadonnees et le launch.

**Correction suggeree :** imposer `action_semantics` dans les metadonnees de
chaque modele, refuser le mode commande si la semantique attendue n'est pas
compatible, et documenter le choix `action_mode` utilise pour chaque test.

## 6. `v_safe` reel a 50 % vs enveloppe nominale simulation

**Type :** logique sim-to-real / securite

**Constat :** l'implementation live utilise `v_safe_factor=0.5`, alors que le
plan sim-to-real de base parle des limites nominales UR3e.

**Preuves :**

- `src/ur3e_live_catch/config/live_catch.yaml` fixe `v_safe_factor: 0.5`.
- `limits.py` construit `v_safe = max_velocity * v_safe_factor`.
- `ur3e_ball_catch_sim_to_real.md` prend les limites nominales UR3e comme
  enveloppe de simulation.

**Impact :** la dynamique live testee est plus conservatrice que l'enveloppe
simulee nominale. C'est plus sur pour le bring-up, mais les comparaisons
sim/reel doivent le mentionner.

**Correction suggeree :** garder `0.5` pour les premiers essais reels, puis
documenter dans les resultats si la policy est testee a 50 %, a 100 %, ou
reentrainee avec la meme enveloppe.

## 7. Calibration extrinseque et TF statiques pas encore valides physiquement

**Type :** integration robot / perception

**Constat :** les scripts et l'UI de calibration existent, mais la session
physique finale hand-eye et les TF statiques associes ne sont pas encore valides.

**Preuves :**

- `docs/Robot_Control/ur3e_camera_base_calibration.md` indique que la session
  physique reste a faire.
- `live_catch_node.py` a besoin de TF `base -> <camera_frame>` et
  `base -> hoop_center`; sans `hoop_center`, il utilise le fallback placeholder
  de `live_catch.yaml`.
- Aucun `calibration/handeye_result.yaml` versionne n'est present.

**Impact :** la boucle peut etre testee avec une balle virtuelle en `base`, mais
pas encore avec une perception camera fiable dans le repere robot.

**Correction suggeree :** executer la session hand-eye, publier
`base -> camera_optical`, mesurer/publier `wrist_3_link -> hoop_center`, puis
faire le test de parite `publish_frame=base` vs `publish_frame=<camera_frame>`.

## 8. Chemin du resultat hand-eye divergent entre script et UI

**Type :** configuration / integration

**Constat :** le wrapper de session indique une sortie sous
`recordings/mire_calibration/handeye/handeye_result.yaml`, tandis que l'UI lit
par defaut `calibration/handeye_result.yaml`.

**Preuves :**

- `scripts/run_handeye_session.sh` documente la commande
  `--output-yaml recordings/mire_calibration/handeye/handeye_result.yaml`.
- `src/ur3e_web_ui/ur3e_web_ui/calibration.py` definit
  `DEFAULT_CAMERA_RESULT_PATH = Path("calibration/handeye_result.yaml")`.
- `docs/Robot_Control/ur3e_web_ui.md` documente ce chemin par defaut.

**Impact :** une calibration reussie peut ne pas apparaitre dans l'UI si le YAML
n'est pas copie ou si `--camera-calibration` n'est pas renseigne.

**Correction suggeree :** choisir un chemin canonique unique ou ajouter au
script une option qui ecrit directement vers `calibration/handeye_result.yaml`.

## 9. Assets calibration : OBJ local ignore et ancien chemin photo

**Type :** documentation / reproductibilite

**Constat :** la doc calibration listait `Support3D.obj` comme fichier disponible
et pointait la photo vers un ancien chemin a la racine de `docs/`. L'OBJ existe
localement mais est ignore par Git via `*.obj`; la photo versionnee est sous
`docs/Robot_Control/`.

**Preuves :**

- `.gitignore` ignore `*.obj`.
- `git ls-files docs/Robot_Control/3D_model` ne liste pas `Support3D.obj`.
- Les fichiers versionnes incluent `Support3D.step`, `Support3D.glb`,
  `Support3D_meters.glb`, `Support3D.mtl`.
- Les photos versionnees sont
  `docs/Robot_Control/Smartphone_Sur_Support.jpeg` et
  `docs/Robot_Control/Smartphone_Sur_Support_vue_coté.jpeg`.

**Impact :** un clone propre ne retrouvera pas forcement l'OBJ intermediaire.

**Correction suggeree :** traiter le STEP et les GLB comme sources versionnees,
ou forcer l'ajout de l'OBJ si ce format doit rester une entree officielle.

## 10. Test robot reel avec balle virtuelle encore a faire

**Type :** validation materielle

**Constat :** le code permet maintenant de tester la policy avec une balle
virtuelle et `enable_command`, mais la validation sur UR3e reel n'est pas encore
faite.

**Preuves :**

- `ur3e_live_catch_implementation_status.md` marque l'etape 9 comme outillee
  mais a valider sur robot reel.
- L'onglet `Test` appelle `/test_ball_node/throw` et
  `/live_catch_node/enable_command`.
- Les smoke-tests documentes sont dry-run, services ROS et chaine en process
  avec torch, pas commande physique validee.

**Impact :** la securite logicielle est prete a tester, mais le comportement
reel du `forward_position_controller`, du watchdog et du retour au trajectory
controller reste inconnu.

**Correction suggeree :** suivre la checklist `docs/reste_a_faire.md` :
fake hardware/URSim, puis robot reel sans vraie balle, E-stop en main, vitesse
reduite, validation watchdog et retour controleur.

## 11. Compilation du rapport non reproductible sur clone propre

**Type :** reproductibilite rapport / donnees ignorees

**Constat :** le rapport peut referencer des figures issues de `recordings/`,
dossier ignore par Git.

**Preuves :**

- `.gitignore` ignore `recordings/`.
- `docs/latex_compilation.md` rappelle que les images incluses doivent rester
  accessibles depuis la racine.
- Les captures de calibration presentes localement sous `recordings/` ne sont
  pas garanties dans un clone propre.

**Impact :** la compilation de `Stage_summary.tex` peut dependre d'artefacts
locaux non versionnes.

**Correction suggeree :** copier les figures finales du rapport dans un dossier
versionne, par exemple `docs/report_assets/`, puis mettre a jour les chemins
LaTeX.

## Verification documentaire effectuee

- Lecture des documents Markdown/HTML sous `docs/`.
- Extraction texte des PDF disponibles dans `docs/`.
- Croisement avec `src/ur3e_live_catch`, `src/ur3e_catch_msgs`,
  `src/ur3e_web_ui`, `src/Ball_Tracking_Cpp`, `scripts/` et `data/models`.
- Verification que `Support3D.obj` est ignore par `.gitignore` et non suivi par
  Git malgre sa presence locale.
