# Incoherences code / docs / logique

Date de revue : 2026-06-20

Portee : documents sous `docs/`, fichiers de synthese/rapport a la racine, code et scripts cites par ces documents. Les PDF ont ete relus comme contexte projet et rapport, mais les constats ci-dessous se concentrent sur les contradictions ou zones ambigues directement actionnables dans le depot.

## Resume court

Les incoherences les plus importantes concernent la chaine de rattrapage live UR3e : plusieurs documents de conception sont restes au stade "non implemente", alors que des packages existent maintenant; le topic historique `ball_position_3d_mm` est encore publie vide cote C++; les contrats de temps et de normalisation/scaler ne sont pas completement resolus; et les semantiques d'action ont change entre l'ancienne conception, la simulation corrigee et l'implementation actuelle.

Un second groupe de problemes est documentaire : chemins HTML obsoletes dans le README racine, README de packages non mis a jour, references a des fichiers de calibration/support absents ou deplaces, et compilation du rapport non reproductible sur une installation propre si les fichiers ignores de `recordings/` ne sont pas fournis.

## 1. README racine : chemins HTML obsoletes

**Type :** documentation / navigation

**Constat :** le README racine pointe vers des fichiers HTML a la racine du depot, alors qu'ils sont dans `docs/` ou `docs/Context/`.

**Preuves :**

- `README.md` cite `trace_algorithm_explanation.html`, `algo_trace_graph.html` et `algo_circle_fitting_graph.html` comme fichiers directement accessibles depuis la racine.
- Les fichiers presents sont `docs/trace_algorithm_explanation.html`, `docs/Context/algo_trace_graph.html` et `docs/Context/algo_circle_fitting_graph.html`.
- `docs/Context/synthese_projet.md` utilise deja les chemins corrects avec le prefixe `docs/`.

**Impact :** un lecteur qui suit uniquement le README racine obtient des liens ou commandes faux.

**Correction suggeree :** mettre a jour les chemins du README racine pour pointer vers `docs/trace_algorithm_explanation.html` et `docs/Context/*.html`.

## 2. Topic `ball_position_3d_mm` : la documentation annonce une position, le code publie un message vide

**Type :** bug fonctionnel / contrat ROS ambigu

**Constat :** la documentation generale decrit une chaine qui publie la position de balle sur `ball_position_3d_mm`, mais l'implementation C++ actuelle publie un `Float32MultiArray` vide.

**Preuves :**

- `README.md` presente la chaine Trace comme publiant la position de la balle sur le topic ROS `ball_position_3d_mm`.
- `docs/Context/synthese_projet.md` signale deja comme point critique restant la publication d'une position native, typee et timestamped plutot qu'un tableau vide ou non structure.
- `docs/Robot_Control/ur3e_live_catch_architecture.md` precise que `publisher_member_function.cpp` cree `ball_position_3d_mm`, mais que `publishBallPose()` laisse `msg.data` vide.
- `src/Ball_Tracking_Cpp/src/publisher_member_function.cpp` construit un `Float32MultiArray msg` puis le publie sans remplir `msg.data`.
- `src/ur3e_live_catch/ur3e_live_catch/float32_adapter.py` marque un `BallState` invalide si `len(msg.data) < 3`.

**Impact :** la chaine live ne peut pas consommer une position valide depuis le tracker C++ actuel. Tout node qui attend `[x, y, z]` sur `ball_position_3d_mm` ne recevra que des etats invalides.

**Correction suggeree :** soit remplir explicitement `msg.data = [x_mm, y_mm, z_mm]` avec un ordre et une unite documentes, soit remplacer ce topic par un message type `BallState` publie directement par le tracker.

## 3. Etat d'implementation live catch contradictoire entre documents

**Type :** documentation obsolete / risque d'exploitation

**Constat :** certains documents indiquent que le rattrapage live UR3e est seulement une conception non implementee, tandis que d'autres et le code montrent que deux packages et plusieurs modules existent.

**Preuves :**

- `docs/Robot_Control/ur3e_live_catch_architecture.md` est date du 2026-06-17 et annonce un statut "design, pas encore implemente", avec packages attendus mais non existants.
- `docs/Context/synthese_projet.md` presente encore les deux packages `ur3e_catch_msgs` et `ur3e_live_catch` comme une boucle future a creer.
- `docs/Robot_Control/ur3e_live_catch_implementation_status.md` est date du 2026-06-20 et indique que les etapes 1 a 5 sont implementees.
- Le code contient bien `src/ur3e_live_catch/setup.py` avec des entry points et `src/ur3e_catch_msgs/CMakeLists.txt` avec la generation de messages.

**Impact :** un lecteur ne peut pas savoir sans inspection du code si la bonne reference est l'architecture initiale ou l'etat d'implementation. Cela peut conduire a ignorer du code existant ou a croire que le systeme commande deja le robot.

**Correction suggeree :** ajouter en tete des documents obsoletes un avertissement renvoyant vers `docs/Robot_Control/ur3e_live_catch_implementation_status.md`, ou fusionner la conception et l'etat d'implementation dans un seul document maintenu.

## 4. README des packages `ur3e_live_catch` et `ur3e_catch_msgs` non synchronises avec le code

**Type :** documentation de package obsolete

**Constat :** les README internes de packages decrivent encore des fichiers "a ajouter" qui existent deja, et presentent certaines capacites comme presentes alors qu'elles sont seulement esquissees.

**Preuves :**

- `src/ur3e_live_catch/README.md` liste `package.xml`, `setup.py`, `setup.cfg`, `resource`, `__init__.py` et les modules comme elements "a ajouter".
- Ces fichiers existent dans `src/ur3e_live_catch/`, et `setup.py` declare deja des entry points.
- `src/ur3e_catch_msgs/README.md` demande encore d'ajouter `package.xml`, `CMakeLists.txt` et les fichiers `.msg`.
- `src/ur3e_catch_msgs/CMakeLists.txt` reference deja `BallState.msg` et `CatchTelemetry.msg`.
- Le README `ur3e_live_catch` mentionne une politique runtime, une couche de securite et du streaming; le node actuel reste volontairement en dry-run et ne commande pas le robot.

**Impact :** les README ne peuvent pas etre utilises comme etat fiable d'avancement ou guide de reprise.

**Correction suggeree :** transformer ces README en documentation d'utilisation actuelle : lancement, topics, mode dry-run, limites restantes, tests disponibles.

## 5. Semantique d'action : ancienne conception, simulation corrigee et implementation actuelle ne disent pas la meme chose

**Type :** logique de controle / risque robot

**Constat :** la semantique de l'action policy a evolue, mais tous les documents ne sont pas alignes. L'architecture initiale decrit une action incrementale clippee et memorisee; l'etat d'implementation indique que les rollouts corriges utilisent une cible absolue non clippee; le code live a deux modes, dont un mode par defaut `faithful` qui conserve l'action brute.

**Preuves :**

- `docs/Robot_Control/ur3e_live_catch_architecture.md` decrit l'action precedente comme une action clippee injectee dans l'observation et un mapping robot incremental.
- `docs/Robot_Control/ur3e_live_catch_implementation_status.md` indique que les nouveaux rollouts ont confirme une cible absolue et que `obs[26:32]` contient l'action precedente brute.
- `src/ur3e_live_catch/ur3e_live_catch/action.py` implemente un mode `faithful` ou la cible vaut `action * 0.5` et ou `prev_action` reste brut, ainsi qu'un mode `safe` incremental clippe.
- `src/ur3e_live_catch/ur3e_live_catch/live_catch_node.py` utilise le mode live en dry-run, log l'action brute et renvoie cette action comme composante 9 de l'observation.
- `src/ur3e_live_catch/README.md` decrit encore un `ActionMapper` en delta clippe/rate-limited qui memorise l'action clippee.

**Impact :** une politique entrainee avec une semantique peut etre rejouee avec une autre. Sur robot reel, cette confusion est critique car elle change la signification de chaque sortie policy.

**Correction suggeree :** documenter une seule semantique officielle par modele : `absolute_target`, `incremental_target`, action brute ou clippee dans l'observation, echelle appliquee, et mode `ActionMapper` autorise pour le reel.

## 6. Modele fallback ancien et repertoire canonique `data/models` vide

**Type :** configuration / reproductibilite / risque d'incompatibilite

**Constat :** la documentation sim-to-real indique que les anciennes politiques et les anciens rollouts sont incompatibles avec les nouvelles corrections d'action; pourtant le node live pointe encore par defaut vers un export date ancien, et le repertoire canonique annonce pour les modeles ne contient pas de modele.

**Preuves :**

- `docs/Robot_Control/ur3e_ball_catch_sim_to_real.md` explique que les anciennes policies et `rollouts_10_episodes.json` ne sont pas comparables apres correction de la semantique d'action.
- `src/ur3e_live_catch/ur3e_live_catch/live_catch_node.py` definit un `FALLBACK_MODEL` vers `data/ur3e_rollouts/2026-05-26.../policy_deterministic.ts`.
- Le metadata de ce modele date indique une semantique `joint_position_target_rad = action_normalized * action_scale`.
- `data/models/README.md` presente `data/models` comme emplacement canonique, avec sidecars attendus.
- `data/models/` ne contient pas de modele actif dans l'etat actuel du depot.

**Impact :** le lancement sans parametre explicite peut charger un modele historiquement utile pour tests, mais potentiellement incoherent avec la semantique corrigee de la simulation ou du robot.

**Correction suggeree :** supprimer le fallback implicite ou le remplacer par un modele canonique compatible, versionne avec `policy_metadata.json` et un champ explicite `action_semantics`.

## 7. Scaler / normalisation policy non resolu

**Type :** contrat ML / ambiguite runtime

**Constat :** plusieurs documents indiquent que la normalisation des observations est critique, mais le depot ne fixe pas encore clairement si elle est integree au `.ts` ou fournie par sidecar.

**Preuves :**

- `docs/Robot_Control/ur3e_live_catch_architecture.md` avertit que lancer une policy sans scaler d'observation peut faire diverger les sorties.
- `src/ur3e_live_catch/ur3e_live_catch/policy_runtime.py` documente que le metadata actuel ne contient pas de moyenne/variance et qu'il faut soit un `.ts` auto-contenu, soit un `policy_scaler.json`.
- `docs/Robot_Control/ur3e_live_catch_implementation_status.md` laisse le sujet scaler ouvert.
- `data/models/README.md` prevoit un `policy_scaler.json` si la normalisation n'est pas embarquee.
- Aucun `policy_scaler.json` canonique n'est present dans `data/models/`.

**Impact :** meme avec un modele valide, le runtime peut produire des actions hors distribution si le preprocessing exact d'entrainement n'est pas reproduit.

**Correction suggeree :** imposer dans le metadata une cle `normalization = embedded | sidecar | none`, refuser le lancement live si cette information est absente, et ajouter un test qui compare une observation de rollout a la sortie attendue.

## 8. Limites de vitesse `v_safe` : facteur 2 entre simulation documentee et code live

**Type :** logique sim-to-real / securite

**Constat :** les documents sim-to-real et l'architecture live initiale prennent les limites de vitesse UR3e nominales comme enveloppe, alors que l'implementation live actuelle utilise la moitie des limites URDF.

**Preuves :**

- `docs/Robot_Control/ur3e_ball_catch_sim_to_real.md` donne `velocity_limit_sim` autour de `[3.1416, ..., 6.2832]` et definit l'increment `Delta = v_safe * dt_step`.
- `docs/Robot_Control/ur3e_live_catch_architecture.md` reprend les memes valeurs nominales pour `v_safe`.
- `src/ur3e_live_catch/ur3e_live_catch/safety.py` indique `v_safe = URDF_limit * 0.5`.
- `src/ur3e_live_catch/config/live_catch.yaml` documente aussi `v_safe = URDF joint limit x 0.5`.
- `docs/Robot_Control/ur3e_live_catch_implementation_status.md` confirme cette reduction a 50%.

**Impact :** la dynamique robot live n'est pas identique a celle annoncee dans les documents sim-to-real. Cela peut rendre les comparaisons de trajectoires trompeuses et limiter la capacite de rattrapage par rapport a l'entrainement.

**Correction suggeree :** choisir explicitement entre "meme enveloppe que simulation" et "enveloppe reel reduite a 50%", puis reporter ce choix dans les metadata policy, les docs sim-to-real et les tests.

## 9. `streaming.py` mentionne mais absent

**Type :** documentation / architecture incomplete

**Constat :** la documentation de package mentionne un module `streaming.py`, mais ce fichier n'existe pas dans le package actuel.

**Preuves :**

- `src/ur3e_live_catch/README.md` liste `safety.py` et `streaming.py` comme modules de la couche robot.
- `docs/Robot_Control/ur3e_live_catch_implementation_status.md` indique que l'action mapper, la securite et le streaming sont concus mais pas encore cables au robot.
- Dans `src/ur3e_live_catch/ur3e_live_catch/`, les modules presents incluent notamment `action.py`, `safety.py`, `live_catch_node.py`, `policy_runtime.py`, mais pas `streaming.py`.
- `src/ur3e_live_catch/ur3e_live_catch/live_catch_node.py` precise que le node ne commande pas encore le robot.

**Impact :** le lecteur peut croire que la couche de streaming URScript/RTDE existe deja alors qu'elle n'est pas implementee dans le depot.

**Correction suggeree :** soit creer un squelette `streaming.py` avec statut non cable et tests minimaux, soit retirer la reference tant que l'implementation n'existe pas.

## 10. Calibration main-oeil : references a un OBJ absent et a une photo au mauvais chemin

**Type :** documentation / fichiers manquants

**Constat :** le document de calibration cite des fichiers de support 3D et une photo avec des chemins qui ne correspondent pas a l'arborescence actuelle.

**Preuves :**

- `docs/Robot_Control/ur3e_camera_base_calibration.md` liste `Support3D.obj` avec `Support3D.step` et `Support3D.mtl`.
- Le dossier `docs/Robot_Control/3D_model/` contient `Support3D.glb`, `Support3D_meters.glb`, `Support3D.mtl` et `Support3D.step`, mais pas `Support3D.obj`.
- Le meme document cite `docs/Smartphone_Sur_Support.jpeg`.
- Les photos presentes sont sous `docs/Robot_Control/Smartphone_Sur_Support.jpeg` et `docs/Robot_Control/Smartphone_Sur_Support_vue_coté.jpeg`.

**Impact :** les instructions de consultation du support mecanique ne sont pas reproductibles telles quelles.

**Correction suggeree :** remplacer `Support3D.obj` par les fichiers reellement versionnes, et corriger le chemin des images vers `docs/Robot_Control/...`.

## 11. Chemin du resultat hand-eye : script et UI n'utilisent pas le meme emplacement par defaut

**Type :** configuration / integration

**Constat :** le script de session main-oeil produit le resultat sous `recordings/mire_calibration/handeye/`, tandis que l'UI web lit par defaut `calibration/handeye_result.yaml`.

**Preuves :**

- `scripts/run_handeye_session.sh` indique que la sortie de resolution se trouve sous `recordings/mire_calibration/handeye/handeye_result.yaml`.
- `src/ur3e_web_ui/ur3e_web_ui/calibration.py` definit par defaut `calibration/handeye_result.yaml`.
- `docs/Robot_Control/ur3e_web_ui.md` documente ce meme chemin par defaut cote UI.
- `docs/Robot_Control/ur3e_camera_base_calibration.md` explique d'utiliser le script puis l'API `/api/calibration/camera`, mais ne precise pas clairement le transfert ou l'argument `--camera-calibration` a fournir.

**Impact :** apres une calibration reussie par script, l'onglet calibration de l'UI peut continuer a afficher un ancien resultat ou aucun resultat si le YAML n'est pas copie au chemin attendu.

**Correction suggeree :** unifier le chemin par defaut, ou faire ecrire le script directement dans `calibration/handeye_result.yaml` avec une option `--output` documentee.

## 12. Contrat temporel `BallState.stamp` non satisfait par les producteurs intermediaires

**Type :** contrat message / latence

**Constat :** le message `BallState` exige un timestamp d'evenement, mais les producteurs intermediaires utilisent l'heure de reception ou l'heure courante.

**Preuves :**

- `src/ur3e_catch_msgs/msg/BallState.msg` documente `stamp` comme temps d'evenement, pas temps de reception, et precise que c'est critique pour la compensation de latence.
- `src/ur3e_live_catch/ur3e_live_catch/float32_adapter.py` indique que le flux legacy n'a pas de timestamp source et utilise l'heure de reception.
- `src/ur3e_live_catch/ur3e_live_catch/test_ball_node.py` publie `stamp = self.get_clock().now()`.
- `docs/Robot_Control/ur3e_live_catch_implementation_status.md` garde le timestamp event-time comme point ouvert.

**Impact :** toute extrapolation de balle ou budget de latence base sur `BallState.stamp` est fragile tant que la source camera/tracker ne fournit pas un temps d'acquisition reel.

**Correction suggeree :** ajouter un champ ou un metadata explicite pour distinguer `event_time`, `receive_time` et `sim_time`, ou refuser le mode live reel si le timestamp n'est pas de type evenement.

## 13. Compilation du rapport non reproductible sur clone propre

**Type :** reproductibilite rapport / donnees ignorees

**Constat :** la documentation LaTeX annonce une compilation directe du rapport, mais le rapport reference au moins une image issue de `recordings/`, dossier ignore par Git, et l'environnement courant n'a pas `latexmk`.

**Preuves :**

- `docs/latex_compilation.md` indique que `compile-report` compile le rapport et que les images incluses doivent rester accessibles depuis la racine.
- `Stage_summary.tex` inclut des images sous `recordings/mire_calibration/...`.
- `.gitignore` ignore `recordings/`.
- `Stage_summary.tex` reference notamment `recordings/mire_calibration/square_test_20260611_110401_090251.png`, absent de l'arborescence courante inspectee.
- L'execution de `./scripts/compile_stage_summary.sh` echoue dans l'environnement courant car `latexmk` est absent.

**Impact :** un clone propre ou une machine sans les artefacts locaux de `recordings/` ne peut pas forcement reconstruire `Stage_summary.pdf`.

**Correction suggeree :** copier les figures finales necessaires dans un dossier versionne, par exemple `docs/report_assets/`, mettre a jour les `\includegraphics`, et documenter les paquets LaTeX requis ou fournir un conteneur de compilation.

## Verification effectuee

- Lecture des documents Markdown/HTML sous `docs/`.
- Extraction texte des PDF disponibles dans `docs/`.
- Croisement avec les fichiers source, scripts et README cites.
- Verification des tests du package live catch : `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/ur3e_live_catch/test -q` retourne `19 passed, 1 skipped`.
- Tentative de compilation du rapport : `./scripts/compile_stage_summary.sh` echoue dans l'environnement courant par absence de `latexmk`.
