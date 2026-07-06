# Procédure — Calibration extrinsèque caméra → base UR3e (session physique)

Date : 2026-07-06. Version condensée opérateur ; le document de référence
complet est `docs/Robot_Control/ur3e_camera_base_calibration.md` et le runbook
wiki `wiki/calibration/extrinsic-calibration-runbook.md`.

But : estimer `T_base_camera` (DVXplorer fixe dans le repère `base` du robot)
avec la mire téléphone montée sur `tool0` (eye-to-hand). Le code a été vérifié
le 2026-07-06 : self-tests solveur et collecteur OK, tests web UI OK.

## 1. Pré-requis

- **Stack robot lancé** (`source env.sh` puis `run`) : TF `base → tool0` et
  `/joint_states` publiés. Même `ROS_DOMAIN_ID` que le terminal de calibration.
- **DVXplorer branchée en USB** : le collecteur l'ouvre directement via
  `dv_processing`, aucun driver ROS caméra nécessaire.
- **Intrinsèques validées AVANT** : les XML existants
  (`recordings/mire_calibration/intrinsics_from_mire*.xml`) sont approximatifs
  (~0,49 px). Faire « Test calib » (F9) et « Test carré » (F10) : erreurs de
  distance et d'espacement < 1 %. Le hand-eye ne rattrape jamais de mauvaises
  intrinsèques.
- **Téléphone (Poco X7 Pro) monté sur tool0** : luminosité 100 % ou DC dimming
  (sinon le PWM AMOLED noie la caméra événementielle), rafraîchissement fixe
  60 Hz, luminosité auto / always-on / notifications désactivées, veille écran
  « jamais ». Première fois : vérifier l'espacement des points au pied à
  coulisse via le « Mode mesure » de la page.

## 2. Vérifications avant session

```bash
cd ~/Dv-Rosws/Dv-Rosws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/solve_handeye.py --self-test
python3 scripts/event_mire_calibration.py --self-test
```

Les deux doivent afficher `self-test ok`.

## 3. Session de capture

```bash
scripts/run_handeye_session.sh
```

Lance le serveur mire (port 8081) + le collecteur en mode `--external-mire`.
Ouvrir l'URL affichée sur le téléphone, passer en vrai plein écran, appuyer
sur « Démarrer ».

Par pose (viser 15–20 échantillons acceptés) :

1. Onglet Calibration de la web UI : « Go to next pose » (poses articulaires
   enregistrées dans `calibration/calibration_poses.json`, jamais de cibles
   cartésiennes).
2. Attendre l'arrêt complet du robot.
3. Fenêtre collecteur : « Capture hand-eye » (F11). Shift+F11 supprime le
   dernier échantillon.

Diversité des poses = ce qui contraint la rotation : incliner l'écran vers la
caméra à ±25–40° en variant les trois axes, couvrir le champ caméra et 2–3
distances, mire entière et nette à chaque pose.

Rejets automatiques : points manquants, robot bougé pendant l'accumulation de
240 ms (> 0,1 mm / 0,02°), ambiguïté IPPE trop faible à faible inclinaison
(< 1,5 sous 15° de tilt). Échantillons :
`recordings/mire_calibration/handeye/handeye_samples_<horodatage>.json`.

## 4. Résolution

```bash
python3 scripts/solve_handeye.py \
    recordings/mire_calibration/handeye/handeye_samples_*.json \
    --output-yaml calibration/handeye_result.yaml
```

`calibration/handeye_result.yaml` est le chemin attendu par la web UI
(`GET /api/calibration/camera`) et par `publish_camera_tf.py`. N'accepter le
résultat que si TOUT est vert dans le rapport :

- accord des deux solveurs OpenCV (quelques mm / dixièmes de degré) ;
- résidus par pose et leave-one-out stables (< ~2–3 mm) ;
- diversité des axes de rotation suffisante ;
- RMS pixel bout-en-bout < ~1–2 px ;
- normale de la mire rentrant dans l'écran ; `T_tool0_mire` cohérent avec le
  CAD (< ~5 mm, < ~2–3°) ; `t_base_camera` cohérent au mètre ruban (2–3 cm).

## 5. Publication et validation

```bash
python3 scripts/publish_camera_tf.py calibration/handeye_result.yaml
```

Publie le TF statique `base → camera_optical` (défaut cohérent avec la
perception). Options : `--with-mire` (aussi `tool0 → screen_center`),
`--write-xacro`, `--print-only`.

À vérifier ensuite :

- **`base` vs `base_link`** : la calibration publie sous `base` (convention
  UR) ; le live catch travaille en `base_link` (tourné de π autour de Z). Le
  stack robot doit tourner pour que `camera_optical → base_link` se résolve.
- **Test de parité** : `publish_frame=base_link` vs
  `publish_frame=camera_optical` doivent coïncider avant de faire confiance à
  la perception réelle.
- Overlay du repère caméra dans le viewer web (onglet Calibration).
- Le **ghost du support 3D** sur `tool0` dans le viewer est purement visuel :
  sa pose est dans `src/ur3e_web_ui/ur3e_web_ui/static/models/support_mount.json`
  (clocking +90° autour de tool0 Z appliqué le 2026-07-06 pour coller au
  montage réel ; si c'est le mauvais sens, inverser les signes du yaw et de
  l'offset x dans ce fichier). Aucun effet sur le solve — `T_tool0_mire` est
  co-résolu par le hand-eye.
- **Ne plus toucher la caméra** : tout déplacement invalide la calibration.
