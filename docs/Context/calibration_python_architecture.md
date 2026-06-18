# Architecture des scripts Python de calibration

Ce document resume le role des scripts Python utilises pour calibrer la camera
evenementielle DVXplorer avec une mire clignotante.

Le flux principal est en deux temps :

1. `scripts/event_mire_calibration.py` capture des observations de mire.
2. `scripts/calibrate_intrinsics_from_mire.py` transforme ces observations en
   calibration intrinseque OpenCV.

Des scripts annexes existent pour les cas telephone/hand-eye :

- `scripts/serve_phone_mire.py` sert une mire sur telephone.
- `scripts/solve_handeye.py` exploite les echantillons hand-eye.

## Vue d'ensemble

```text
DVXplorer events
    |
    v
event_mire_calibration.py
    - affiche la mire
    - permet de choisir le type de mire
    - accumule les evenements ON/OFF en positif
    - filtre eventuellement le bruit de fond
    - detecte les blobs
    - associe les blobs aux points de mire
    - exporte mire_observation_*.json + mire_overlay_*.png
    |
    v
calibrate_intrinsics_from_mire.py
    - charge les observations JSON
    - peut filtrer les observations par type de mire
    - extrait object_mm <-> camera_px
    - valide la diversite des vues
    - lance cv.calibrateCameraExtended
    - exporte intrinsics_from_mire.xml + rapport JSON
```

## Dossiers et fichiers produits

Par defaut, tous les fichiers de capture/calibration intrinseque sont ecrits
dans :

```text
recordings/mire_calibration/
```

Ce dossier est configurable avec `--output-dir` pour
`event_mire_calibration.py`, et avec `--input-dir`, `--output-xml` et
`--output-json` pour `calibrate_intrinsics_from_mire.py`.

Fichiers principaux :

- `recordings/mire_calibration/mire_observation_*.json` : observation exportee
  par une capture `Calib`.
- `recordings/mire_calibration/mire_overlay_*.png` : image de controle de la
  detection/association de la meme capture.
- `recordings/mire_calibration/intrinsics_from_mire.xml` : intrinseques OpenCV
  produits par `calibrate_intrinsics_from_mire.py`.
- `recordings/mire_calibration/intrinsics_from_mire_report.json` : rapport de
  calibration lisible avec erreurs, vues utilisees et avertissements.
- `recordings/mire_calibration/calibration_test_*.json` et
  `recordings/mire_calibration/calibration_test_*.png` : sorties du bouton
  `Test calib`.
- `recordings/mire_calibration/square_test_*.json` et
  `recordings/mire_calibration/square_test_*.png` : sorties du bouton
  `Test carre`.
- `recordings/mire_calibration/handeye/handeye_samples_*.json` : echantillons
  du mode telephone/hand-eye.

## `event_mire_calibration.py`

Ce script est l'outil interactif de capture. Il ouvre une interface Qt, affiche
une mire clignotante sur l'ecran selectionne et lit les evenements de la
DVXplorer.

### Structures importantes

- `MonitorInfo` : geometrie ecran en pixels et taille physique en mm.
- `DotGridPattern` : definition d'un type de mire a points, avec dimensions,
  point d'ancrage, trou eventuel et nombre attendu de points.
- `ScreenDot` : point de mire cote ecran, avec coordonnees image/ecran et
  coordonnees objet en mm.
- `Blob` : composant detecte dans l'image d'accumulation.
- `Match` : association entre un `ScreenDot` et un `Blob`.
- `ControlWindow` : controle toute l'interface, la camera, la capture, les
  tests et les exports.

### Cycle d'une capture

1. `_start_capture()` remet `self.activity` a zero et lance la sequence
   `noir -> mire clignotante`.
2. `begin_accumulation()` active la fenetre d'accumulation apres la phase noire.
3. `poll_camera()` lit les batches d'evenements camera.
4. Si le filtre bruit est actif, `filter_background_noise()` retire les
   evenements isoles.
5. `add_events_to_activity()` ajoute les evenements dans `self.activity`.
6. `finish_calibration()` arrete l'accumulation et appelle `detect_blobs()` sur
   `self.activity`.
7. `associate_blobs_to_layout()` associe les centres detectes a la mire connue.
8. `export_observation()` ecrit le JSON d'observation et l'overlay PNG.

Point important : les polarites ON et OFF sont additionnees positivement. Le
code ne soustrait pas les evenements negatifs aux evenements positifs. Chaque
evenement valide ajoute `+1` dans l'image d'accumulation.

### Types de mire

La mire n'est plus limitee a la grille asymetrique historique de 19 points. Le
type de mire se choisit dans le menu deroulant `Type de mire` de l'interface,
ou au demarrage avec `--pattern`.

Types disponibles :

- `mire` : grille asymetrique 5 x 4, 19 points, avec un point manquant. C'est
  le mode historique et le mode par defaut.
- `grid_5x4` : grille complete 5 x 4, 20 points.
- `grid_7x5` : grille complete 7 x 5, 35 points.
- `grid_9x6` : grille complete 9 x 6, 54 points.

Le dessin, la detection attendue, l'association des blobs et l'export JSON
utilisent le nombre de points de la mire active. L'association n'est donc plus
codee en dur pour 19 points.

Le JSON d'observation exporte contient les metadonnees de mire :

- `mire.pattern` : identifiant (`mire`, `grid_5x4`, `grid_7x5`,
  `grid_9x6`) ;
- `mire.pattern_type` : type logique (`dot_grid`) ;
- `mire.pattern_label` : libelle affiche dans l'interface ;
- `mire.rows`, `mire.cols`, `mire.expected_dots` ;
- `mire.missing_dot` et `mire.anchor_dot` ;
- `mire.layout` et `mire.dots`, avec les coordonnees ecran et objet en mm.

### Detection des blobs

La detection de blobs se fait dans `detect_blobs(activity, expected)`.

Les grandes etapes sont :

1. normalisation de l'image d'accumulation ;
2. flou gaussien ;
3. seuillage par combinaison percentile/Otsu ;
4. ouverture/fermeture morphologique ;
5. composantes connexes ;
6. filtrage par aire ;
7. estimation robuste du centre ;
8. tri par activite puis selection des `expected` blobs les plus actifs.

Le centre d'un blob ne depend pas uniquement du barycentre pondere par
l'activite. Le script estime d'abord le centre par la forme du composant
(`fitEllipse`, puis moments en fallback), puis utilise le centre pondere comme
petite correction seulement s'il est coherent. Cela evite qu'un cote plus actif
du blob deplace trop le centre.

Les exports JSON contiennent aussi :

- `center_method` : methode utilisee pour le centre (`ellipse`,
  `ellipse+weighted`, `contour_moments`, etc.) ;
- `center_agreement_px` : distance entre centre de forme et centre pondere.

### Filtre bruit de fond

Le filtre optionnel utilise
`dv_processing.noise.BackgroundActivityNoiseFilter`.

Il supprime des evenements isoles qui n'ont pas de voisin recent. Il ne change
pas le principe d'accumulation ON/OFF : les evenements conserves sont toujours
additionnes positivement.

Les parametres sont :

- `--noise-filter` : active le filtre au demarrage ;
- `--noise-cutoff-hz` : regle la duree de support, avec
  `duration = 1 / cutoff`.

Exemple : `500 Hz` correspond a environ `2 ms`.

### Fichiers exportes

Pour une capture intrinseque classique :

- `recordings/mire_calibration/mire_observation_*.json`
- `recordings/mire_calibration/mire_overlay_*.png`

Le JSON contient :

- resolution camera ;
- nombre d'evenements accumules ;
- taille ecran et conversion mm/px ;
- layout de la mire ;
- blobs detectes ;
- matches `object_mm <-> camera_px` ;
- type de mire et nombre de points attendus ;
- configuration d'accumulation ;
- configuration/statistiques du filtre bruit si utilise.

### Modes de test

Le script peut aussi tester une calibration existante :

- `Test calib` / `F9` : solvePnP et reprojection sur une capture mire.
- `Test carre` / `F10` : phase de pose avec la mire selectionnee puis
  plusieurs carres 4 points pour valider la calibration avec une geometrie
  differente.

## `calibrate_intrinsics_from_mire.py`

Ce script ne redetecte pas les blobs. Il lit les `mire_observation_*.json`
produits par `event_mire_calibration.py`.

### Pipeline

1. `collect_observations()` parcourt les fichiers JSON d'observation.
2. `load_observation()` extrait :
   - `object_points` depuis `object_mm` ;
   - `image_points` depuis `camera_px` ;
   - la resolution camera ;
   - le nombre d'evenements ;
   - le type de mire (`pattern`, dimensions, nombre attendu de points).
3. `validate_observations()` verifie :
   - nombre minimum de vues ;
   - resolution identique pour toutes les observations ;
   - nombre de points coherent.
4. `diversity_warnings()` signale les vues trop similaires, trop centrees ou
   avec trop peu de variation d'echelle.
5. `calibration_flags()` convertit les options CLI en flags OpenCV.
6. `run_calibration()` appelle `cv.calibrateCameraExtended`.
7. `reprojection_errors()` calcule les erreurs par vue.
8. `write_opencv_xml()` ecrit le XML OpenCV utilisable par le code C++.
9. `write_report_json()` ecrit un rapport lisible avec erreurs, flags et vues.

Les anciens JSON sans champ `mire.pattern` restent compatibles : ils sont
interpretes comme le pattern historique `mire`.

L'option `--pattern` permet de ne calibrer qu'avec un type de mire donne. Par
defaut, `--pattern all` accepte toutes les observations trouvees. Si plusieurs
types de mire sont melanges, le script le signale dans la sortie console et dans
le rapport JSON.

### Mode robuste

L'option `--robust` active une selection de vues type RANSAC :

1. tire des sous-ensembles d'observations ;
2. calibre sur chaque sous-ensemble ;
3. teste toutes les vues avec les intrinseques obtenus ;
4. garde les vues dont l'erreur RMS est sous `--ransac-threshold-px` ;
5. relance la calibration finale uniquement sur les inliers.

Ce mode est utile quand certaines captures ont des blobs mal associes ou une
pose peu fiable.

### Sorties

Par defaut :

- `recordings/mire_calibration/intrinsics_from_mire.xml`
- `recordings/mire_calibration/intrinsics_from_mire_report.json`

Le XML contient :

- `camera_matrix`
- `distortion_coefficients`
- `image_width`, `image_height`
- `calibration_error`
- metadonnees du pattern (`pattern_width`, `pattern_height`, `pattern_type`,
  `pattern_id`, `pattern_label`, `pattern_expected_dots`)
- `input_pattern_filter`, c'est-a-dire la valeur de `--pattern` utilisee

Le JSON de rapport contient :

- filtre de pattern utilise ;
- resume des patterns presents dans les observations ;
- RMS global ;
- erreur moyenne et max ;
- matrice camera ;
- distorsion ;
- ecarts-types OpenCV des intrinseques ;
- erreurs par vue ;
- avertissements de diversite ;
- informations de selection robuste si active.

## Points importants pour une bonne calibration

- Mesurer la taille physique active de l'ecran en mm. L'EDID peut etre faux.
- Capturer des poses variees : centre, coins, bords, proche, loin, inclinees.
- Ne pas utiliser uniquement des vues tres similaires.
- Verifier les overlays PNG apres capture : les labels doivent tomber sur les
  vrais points.
- Surveiller `center_method` et `center_agreement_px` dans les JSON si une vue
  donne une erreur anormale.
- Garder assez d'evenements, mais eviter une accumulation trop longue si la
  camera/ecran bougent.
- Le filtre bruit peut aider avec un fond actif, mais il peut aussi retirer des
  evenements utiles si le cutoff est trop strict.
- `calibrate_intrinsics_from_mire.py` ne peut pas corriger une mauvaise
  detection exportee ; il ne fait que calibrer a partir des matches JSON.
- Une calibration stable demande plus que 3 vues. En pratique, viser au moins
  10 a 20 captures variees.

## Commandes utiles

Lister les ecrans detectes :

```bash
python3 scripts/event_mire_calibration.py --list-monitors
```

Lancer l'outil de capture avec taille ecran mesuree :

```bash
python3 scripts/event_mire_calibration.py \
  --monitor 1 \
  --screen-width-mm 344 \
  --screen-height-mm 194 \
  --accum-ms 240
```

Lancer directement avec une mire 7 x 5 :

```bash
python3 scripts/event_mire_calibration.py \
  --monitor 1 \
  --screen-width-mm 344 \
  --screen-height-mm 194 \
  --pattern grid_7x5
```

Lancer avec filtre bruit de fond :

```bash
python3 scripts/event_mire_calibration.py \
  --monitor 1 \
  --screen-width-mm 344 \
  --screen-height-mm 194 \
  --noise-filter \
  --noise-cutoff-hz 500
```

Augmenter la duree d'accumulation si les blobs sont faibles :

```bash
python3 scripts/event_mire_calibration.py --accum-ms 500
```

Lancer les tests synthetiques du script de capture :

```bash
python3 scripts/event_mire_calibration.py --self-test
```

Calculer les intrinseques avec les observations par defaut :

```bash
python3 scripts/calibrate_intrinsics_from_mire.py
```

Calculer les intrinseques seulement avec une mire donnee :

```bash
python3 scripts/calibrate_intrinsics_from_mire.py --pattern grid_7x5
```

Calculer les intrinseques avec selection robuste :

```bash
python3 scripts/calibrate_intrinsics_from_mire.py \
  --robust \
  --ransac-threshold-px 0.5
```

Forcer une distorsion tangentielle nulle :

```bash
python3 scripts/calibrate_intrinsics_from_mire.py --zero-tangent-dist
```

Changer les chemins de sortie :

```bash
python3 scripts/calibrate_intrinsics_from_mire.py \
  --input-dir recordings/mire_calibration \
  --output-xml recordings/mire_calibration/intrinsics_from_mire.xml \
  --output-json recordings/mire_calibration/intrinsics_from_mire_report.json
```

Tester rapidement la syntaxe Python :

```bash
python3 -m py_compile \
  scripts/event_mire_calibration.py \
  scripts/calibrate_intrinsics_from_mire.py
```

## Mode telephone / hand-eye

Servir la mire telephone :

```bash
python3 scripts/serve_phone_mire.py
```

Lancer l'outil de capture en mode mire externe :

```bash
python3 scripts/event_mire_calibration.py \
  --external-mire http://127.0.0.1:8081 \
  --robot-base-frame base \
  --robot-tool-frame tool0
```

Dans ce mode, les captures hand-eye combinent :

- pose camera -> mire via solvePnP ;
- transformee TF base -> tool0 ;
- verification de stationnarite du robot pendant l'accumulation.

## Checklist avant d'accepter une calibration

- Les overlays montrent tous les centres au bon endroit.
- Les observations couvrent plusieurs zones du capteur.
- Les distances/tailles apparentes varient.
- Le RMS global est coherent avec la precision attendue.
- Les per-view RMS ne montrent pas une vue tres mauvaise.
- Les fichiers XML/JSON de sortie ont ete regeneres apres les dernieres
  captures.
- Le fichier XML utilise par le C++ est bien celui que l'on vient de produire.
