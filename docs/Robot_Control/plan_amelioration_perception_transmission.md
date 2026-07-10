# Plan — Amélioration de la transmission perception → position balle

Plan d'exécution des améliorations identifiées dans la revue du 2026-07-09
(voir [analyse_pipeline_commande_trace_2026-07-09.md](analyse_pipeline_commande_trace_2026-07-09.md)
§ discussion « comment j'aurais géré la transmission »). Cocher chaque étape
une fois faite, testée et documentée dans le wiki.

Légende : `[x]` fait · `[ ]` à faire · `(op)` nécessite une session robot.

## Priorité 1 — Pureté des mesures (une seule couche de prédiction)

- [x] **1.1 Régression : n'accepter que des mesures pures.** *(fait 2026-07-09 :
  `min_input_confidence` dans `RegressionConfig`/nœud/yaml, gate dans
  `add_sample`, tests `test_ball_regression_anisotropy.py`)*
  `ball_regression_node` rejette les échantillons `confidence < min_input_confidence`
  (paramètre, défaut 1.0) : les points de coast du tracker (confidence
  décroissante) ne nourrissent plus le fit comme de vraies mesures.
  Tests unitaires sur le seuil.
- [x] **1.2 Tracker : lead/hold pilotables en ROS et forcés à 0 sous régression.**
  *(fait 2026-07-09 : params `trace_lead_ms`/`trace_hold_ms` +
  `Ui::SetTraceLeadMs/SetTraceHoldMs`, épinglés à 0 par `live_catch.launch.py`
  quand `use_ball_regression:=true`, build OK)*
  Exposer `trace_lead_ms` / `trace_hold_ms` comme paramètres ROS de
  `ball_tracking_cpp` (initialisent les sliders, défaut 0). Le launch
  `live_catch.launch.py` les force à 0 quand `use_ball_regression:=true` :
  la couche mesure n'invente jamais de données, la prédiction appartient à
  l'estimateur (un `lead > 0` publie des points extrapolés à confidence 1.0
  que le gate 1.1 ne peut pas filtrer).

## Priorité 2 — Qualité du fit et outillage

- [x] **2.1 Modèle de bruit anisotrope (profondeur ≫ latéral).** *(fait
  2026-07-09 : `depth_sigma_scale` (yaml : 8.0), rayon caméra→balle par
  échantillon, poids par axe + résidus/rms en métrique « équivalent latéral » ;
  tests : l'isotrope est fragile sous 6 cm de bruit profondeur (pas de pop ou
  rejets selon le tirage), l'anisotrope pope toujours, zéro rejet, vy < 0.15
  d'erreur)*
  La profondeur Trace (1/largeur de traînée) est ~10-100× plus bruitée que le
  latéral. Ajouter à la régression un `depth_sigma_scale` (défaut 1.0 =
  comportement actuel, isotrope) : les résidus sont décomposés le long du
  rayon caméra→balle vs perpendiculaire, la composante profondeur est
  dé-pondérée d'un facteur `depth_sigma_scale` dans l'IRLS et le gate
  d'acceptation. Le nœud fournit la position caméra dans `base_link` via le
  TF déjà consulté. Tests : lancer synthétique avec bruit le long du rayon —
  la vitesse ajustée doit être meilleure avec le scale qu'en isotrope.
- [x] **2.2 Outil de rejeu offline des captures réelles.** *(fait 2026-07-09 :
  `scripts/replay_ball_regression.py` — rosbag2 → BallRegression avec
  overrides `--set clef=valeur`, résolution TF statique du bag, horloge 60 Hz
  simulée, résumés par vol)*
  `scripts/replay_ball_regression.py` : lit un rosbag2 (`/ball_state_raw`,
  positions déjà en base_link ou TF statique fourni), rejoue les échantillons
  dans `BallisticRegression` avec des overrides de paramètres en CLI, imprime
  les résumés de vol (pop latency, rms, v0, raisons de fin). Permet de tuner
  les gates sur données réelles sans session robot.
- [ ] **2.3 (op) Enregistrer des rosbags au prochain test réel.**
  `ros2 bag record /ball_state_raw /ball_state /tf /tf_static /catch_telemetry`
  pendant les lancers ; matière première de 2.2.
- [ ] **2.4 (op) Mesurer la latence bout-en-bout puis régler `lead_time_s`.**
  `latency_report` en conditions réelles ; fixer `lead_time_s` ≈ p95
  (événement → commande) mesuré, pas deviné.
  *Provisoire 2026-07-09 (demande opérateur) : `lead_time_s: 0.2` dans
  `live_catch.yaml`, désormais modifiable À CHAUD sans relancer :
  `ros2 param set /ball_regression_node lead_time_s 0.05` (borné [0, 1] s).
  Attention : 0.2 s ≈ la moitié d'un vol — la balle publiée court 200 ms en
  avance, le vol se termine (sol prédit) 200 ms plus tôt, et l'UI affiche un
  `perception age` ≈ −200 ms. À redescendre vers la latence mesurée.*

## Priorité 3 — Chantiers structurels (passes dédiées)

- [ ] **3.1 Découpler mesure et rendu dans le tracker C++.**
  Sortir acquisition → trace → publication de la cadence raylib
  (`SetTargetFPS(60)`, `EndDrawing` bloquant) : mode headless ou rendu
  1 frame sur N, la publication suivant les batchs d'événements. Conditionne
  la latence du gate de départ (pop trop proche si cadence brute basse).
  Refactor de `Gui::Update()` — à faire hors session robot, avec rejeu 2.2
  comme non-régression.
- [ ] **3.2 Message `BallisticFit` (coefficients) régression → live node.**
  Publier l'état du fit (c0/c1 par axe, t0, g, fenêtre de validité, rms,
  support, confidence) au lieu d'échantillonner à 60 Hz ; le live node
  évalue analytiquement au timestamp exact de chaque observation (+ latence
  mesurée en 2.4). Supprime la double quantification 60 Hz. Changement de
  contrat : msg + live node + UI + tests.

## Hors périmètre (décision assumée)

- Remplacer le refit batch IRLS par un Kalman : non — pour des vols de
  0.3-0.5 s le batch est plus robuste aux outliers de début de vol, trivial à
  rejouer offline, et le coût CPU est négligeable. À reconsidérer seulement si
  3.2 exige une covariance propre.
