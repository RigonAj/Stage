# Incoherences code / docs / logique

Date de revue : 2026-06-24

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
empechent un essai realiste : calibration extrinseque non validee physiquement,
TF statiques camera/hoop a fournir, vitesse robot encore lente, chemins
hand-eye divergents, et quelques documents de reprise encore obsoletes.

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

**Etat 2026-07-02 :** resolu pour la commande de base avec balle virtuelle. Le
chemin commande est cable derriere `enable_command`, avec bascule de controleur,
watchdog et refus de commander si aucun modele n'est charge. Selon rapport
utilisateur, la balle virtuelle commande maintenant le robot reel, mais la
reponse reste lente et le watchdog/tuning restent a finaliser.

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

### R5. Le tracker C++ publie maintenant une sortie legacy non vide

**Ancien constat :** `publishBallPose()` publiait un `Float32MultiArray` sans
remplir `msg.data`.

**Etat 2026-06-24 :** resolu pour le chemin legacy. Le tracker C++ publie
`[x, y, z]` en millimetres depuis `pose.positionMm` sur `ball_position_3d_mm`.

### R6. Le tracker C++ publie maintenant `BallState` nativement

**Ancien constat :** la vraie perception devait passer par
`Float32MultiArray -> BallState`, avec `stamp` a la reception.

**Etat 2026-06-24 :** resolu pour le chemin natif. `ball_tracking_cpp` depend de
`ur3e_catch_msgs` et publie `ur3e_catch_msgs/BallState` sur `ball_state`, en
metres, avec `header.frame_id` parametre (`camera_frame_id`) et un timestamp ROS
ancre sur `BallPose3D.timestampUs`. L'adaptateur `float32_adapter.py` reste
disponible seulement comme fallback legacy.

## Incoherences ou risques encore ouverts

## 1. Chemin fallback `use_adapter:=true` : latence encore approximative

**Type :** configuration legacy / latence

**Constat :** le chemin recommande est maintenant `ball_tracking_cpp -> BallState`
natif. Si on force encore `use_adapter:=true`, l'adaptateur lit le topic legacy
`ball_position_3d_mm` et timestamp a la reception, car le tableau legacy n'a ni
header ni timestamp evenement.

**Preuves :**

- `src/Ball_Tracking_Cpp/src/publisher_member_function.cpp` publie nativement
  `BallState`.
- `float32_adapter.py` utilise le temps de reception car le flux legacy n'a pas
  de timestamp source.
- `CatchTelemetry.perception_age_s` depend directement de ce champ.

**Impact :** le budget de latence est fiable avec le chemin natif, mais reste
approximatif avec le fallback legacy.

**Correction suggeree :** reserver `use_adapter:=true` aux anciens builds, et
utiliser `use_tracker:=true` pour les essais de perception reelle.

## 2. README `ur3e_catch_msgs` encore obsolete

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

## 3. Modele canonique `data/models` present, fallback date conserve

**Type :** configuration / reproductibilite

**Constat :** ce point est resolu pour le lancement courant. `data/models`
contient maintenant les exports `latest` et `best`, ainsi qu'une copie racine
`policy_deterministic.{onnx,ts}` et `policy_metadata.json`. Le fallback date
reste seulement pour compatibilite/debug legacy.

**Preuves :**

- `data/models/latest/` et `data/models/best/` contiennent TorchScript, ONNX et
  `policy_metadata.json`.
- La racine `data/models/` contient aussi `policy_deterministic.onnx`,
  `policy_deterministic.ts` et `policy_metadata.json`.
- `live_catch_node.py` definit toujours `CANONICAL_MODEL` puis `FALLBACK_MODEL`
  pour ne pas casser les anciens essais.
- L'export fallback date est l'ancien modele dont la semantique verifiee est
  `joint_position_target_rad = action_normalized * 0.5`; les exports courants
  utilisent la semantique incrementale via metadata.

**Impact :** un lancement sans `model_path` explicite charge maintenant le modele
canonique courant. Le risque restant est de confondre un fallback legacy avec un
export Isaac recent pendant les audits ou les replays anciens.

**Correction suggeree :** conserver `data/models/README.md` a jour, verifier que
chaque export porte `policy_metadata.json`, et garder les docs explicites sur
`action_semantics`, `normalization=embedded`, `dt_s` et limites attendues.

## 4. Semantique d'action a figer par modele

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

## 5. `v_safe` reel a 50 % vs enveloppe nominale simulation

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

## 6. Calibration extrinseque et TF statiques pas encore valides physiquement

**Type :** integration robot / perception

**Constat :** les scripts et l'UI de calibration existent, mais la session
physique finale hand-eye et les TF statiques associes ne sont pas encore valides.

**Preuves :**

- `docs/Robot_Control/ur3e_camera_base_calibration.md` indique que la session
  physique reste a faire.
- `live_catch_node.py` a besoin de TF `base_link -> <camera_frame>` et
  `base_link -> hoop_center`; sans `hoop_center`, il peut utiliser le fallback
  placeholder de `live_catch.yaml` seulement en dry-run/debug. En mode commande,
  l'absence de TF hoop est un echec ferme.
- Aucun `calibration/handeye_result.yaml` versionne n'est present.

**Impact :** la boucle peut etre testee avec une balle virtuelle en `base_link`, mais
pas encore avec une perception camera fiable dans le repere robot.

**Correction suggeree :** executer la session hand-eye, publier
`base_link -> camera_optical`, mesurer/publier `wrist_3_link -> hoop_center`,
puis faire le test de parite `publish_frame=base_link` vs
`publish_frame=<camera_frame>`.

## 7. Chemin du resultat hand-eye divergent entre script et UI

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

## 8. Assets calibration : OBJ local ignore et ancien chemin photo

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

## 9. Test robot reel avec balle virtuelle valide mais lent

**Type :** validation materielle

**Constat :** ce point n'est plus une incoherence ouverte de commande de base.
Selon rapport utilisateur du 2026-07-02, la policy avec balle virtuelle et
`enable_command` fonctionne sur UR3e reel, mais le mouvement reste lent sous les
limites de bring-up.

**Preuves :**

- `ur3e_live_catch_implementation_status.md` marque maintenant l'etape 9 comme
  validee avec balle virtuelle sur robot reel selon rapport utilisateur.
- L'onglet `Test` appelle `/test_ball_node/throw` et
  `/live_catch_node/enable_command`.
- Les correctifs 2026-07-02 ajoutent le heartbeat `CatchTelemetry.ball_valid`
  et l'arret de vol `ground_z_m`, ce qui evite le faux symptome "controller
  inactive" en idle trigger.

**Impact :** la commande robot n'est plus le blocage principal. Les risques
restants sont la lenteur, le watchdog materiel, le retour controleur et la
perception reelle.

**Correction suggeree :** poursuivre avec la checklist `docs/reste_a_faire.md` :
rejouer balle virtuelle sur robot reel, mesurer `/joint_states` et
`catch_telemetry`, regler `v_safe_scale`/`a_safe`/budgets, valider watchdog et
retour controleur avant vraie balle.

## 10. Compilation du rapport non reproductible sur clone propre

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

## 11. Paquet `ur3e_sysid` present localement mais non suivi par Git

**Type :** reproductibilite code / documentation

**Constat :** la documentation system-id decrit maintenant un paquet
`src/ur3e_sysid/` avec les executables `run_sweep` et `fit_gains`, mais ce dossier
est actuellement non suivi dans Git.

**Preuves :**

- `git status --short` liste `?? src/ur3e_sysid/`.
- `src/ur3e_sysid/setup.py` declare les entry points `run_sweep` et `fit_gains`.
- `src/ur3e_sysid/package.xml` decrit le paquet `ur3e_sysid`.

**Impact :** un clone propre ne contient pas le programme system-id alors que les
docs peuvent laisser penser qu'il est disponible.

**Correction suggeree :** decider si `ur3e_sysid` doit etre versionne; si oui,
l'ajouter au depot avec ses tests, puis etendre les commandes de build/test dans
`env.sh` ou la documentation robot.

## Verification documentaire effectuee

- Lecture des 20 fichiers Markdown sous `docs/`.
- Croisement avec `src/ur3e_live_catch`, `src/ur3e_catch_msgs`,
  `src/ur3e_web_ui`, `src/Ball_Tracking_Cpp`, `src/ur3e_sysid`, `scripts/` et
  `data/models`.
- Verification que `Support3D.obj` est ignore par `.gitignore` et non suivi par
  Git malgre sa presence locale.
