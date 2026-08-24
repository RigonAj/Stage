# [Archived] Analyse de la session de lancers réels du 24 août 2026

> Sources: [Robustesse perception et cycle de vol](perception-robustness-flight-lifecycle.md); [Benchmark Trace contre circle fitting](trace-vs-circle-benchmark.md); [Latence des observations et modèles](../sim-to-real/observation-latency-and-models.md); [Environnement Isaac](../sim-to-real/isaac-training-environment.md); enregistrement H5 et rosbag réels, 2026-08-24; étiquettes de résultat fournies par l'opérateur, 2026-08-24
> Archived: 2026-08-24

## Objet et verdict

Cette page archive l'analyse quantitative de la session réelle enregistrée le
24 août 2026. Le dernier rosbag est valide et contient quatre lancers. Les
trois premiers sont étiquetés comme des échecs par l'opérateur. Le quatrième a
été lancé volontairement dans le filet : il confirme que la trajectoire
estimée passe près du filet, mais il ne démontre pas que le robot a réalisé
l'interception.

Le défaut n'est pas attribuable à la seule régression finale. Les problèmes
les mieux établis sont, par ordre d'importance :

1. la correction de profondeur Trace validée hors ligne n'est pas activée dans
   le chemin live ;
2. les lancers réels sortent souvent de l'enveloppe de la politique ;
3. la chaîne prend environ 156 à 185 ms entre la première détection brute et
   le premier mouvement articulaire significatif ;
4. la régression continue à publier après le passage de la balle près du
   robot, faute d'un état explicite `PASSED_OR_IMPACT` ;
5. l'extrinsèque et le TF du filet n'ont pas encore de vérité terrain physique
   indépendante au niveau centimétrique.

## Données utilisées

- [Enregistrement événementiel](../../recordings/realtest.h5) : 700 595
  événements, durée 118,542 s, sept rafales physiques détectables.
- [Métadonnées du rosbag](../../rosbags/real_20260824_160858/metadata.yaml) :
  durée 55,514 s, 117 512 messages, de 16:08:58.650 à 16:09:54.164.
- Le rosbag couvre les quatre dernières rafales du H5, à environ 16:09:07.7,
  16:09:23.8, 16:09:35.6 et 16:09:45.3.

Les messages personnalisés sont bien présents : 81 `/ball_state_raw`, 3 331
`/ball_state`, 3 331 `/catch_telemetry`, ainsi que les flux complets
`/joint_states`, `/forward_position_controller/commands`, TF et speed scaling.
Les avertissements antérieurs « package `ur3e_catch_msgs` not found »
correspondent à un shell qui n'avait pas sourcé le workspace ; ils ne
caractérisent pas ce dernier rosbag.

## Interprétation des profondeurs lointaines

Le fait que les deux premiers lancers soient partis de loin explique leurs
profondeurs initiales caméra proches de 1,8 m. Ce n'est pas en soi une mesure
aberrante. En revanche, le deuxième lancer finit par sauter jusqu'à 3,943 m
alors que la balle approche : cette inversion tardive n'est pas expliquée par
la distance de lancement et reste une mauvaise estimation de largeur ou de
profondeur.

| Lancer | Échantillons bruts | Durée brute | Profondeur initiale | Étendue de profondeur |
|---|---:|---:|---:|---:|
| 1, échec | 24 | 0,571 s | 1,812 m | 1,218–1,995 m |
| 2, échec | 22 | 0,530 s | 1,767 m | 1,077–3,943 m |
| 3, échec | 18 | 0,434 s | 1,792 m | 1,044–2,066 m |
| 4, lancé dans le filet | 17 | 0,459 s | 1,463 m | 0,888–1,463 m |

Cette sensibilité est attendue pour une profondeur monoculaire obtenue par
`profondeur = focale × diamètre réel / largeur apparente`. À 2,5 m, la balle
n'occupe qu'environ huit pixels : une erreur de largeur d'un pixel produit
plusieurs dizaines de centimètres d'erreur de profondeur.

## Écart avec l'enveloppe de la politique

La politique `latest-left` a été entraînée avec les plages suivantes dans
`base_link` : position initiale `x=[0,2; 0,6]`, `y=[1,2; 2,1]`,
`z=[0,5; 1,2]` m et vitesse `vx=[-0,6; 0,7]`, `vy=[-5; -4]`,
`vz=[0,2; 1,5]` m/s.

| Lancer | Première position ajustée `(x,y,z)` m | Première vitesse `(vx,vy,vz)` m/s | Hors distribution notable |
|---|---|---|---|
| 1 | `(1,062; 1,866; 1,084)` | `(0,61; -4,58; 0,37)` | position `x` ; vitesse compatible |
| 2 | `(1,343; 1,921; 0,870)` | `(-1,07; -6,80; -1,35)` | position `x` et trois composantes de vitesse |
| 3 | `(0,822; 1,291; 0,654)` | `(-3,01; -7,73; -1,54)` | position `x` et trois composantes de vitesse |
| 4 | `(0,545; 1,367; 0,835)` | `(-2,32; -6,43; -0,96)` | vitesse ; position compatible |

La distance de départ caméra et l'enveloppe Isaac ne sont pas la même chose :
la première est mesurée dans `camera_optical`, la seconde dans `base_link`.
Ici, le problème le plus net est surtout la composante latérale `x` des trois
premiers lancers et la vitesse des trois derniers. La politique reçoit donc
des observations qu'elle n'a pas appris à traiter.

## Proximité entre balle estimée et filet réel

La distance ci-dessous est le minimum 3D entre la balle ajustée et le centre
du filet déduit de la télémétrie au même instant. Ce n'est pas une vérité
terrain externe : une erreur commune de perception ou de TF peut influencer
le résultat.

| Lancer | Résultat opérateur | Distance minimale | État logiciel `pass_through` |
|---|---|---:|---:|
| 1 | échec | 0,142 m | 0 |
| 2 | échec | 0,464 m | 0 |
| 3 | échec | 0,332 m | 0 |
| 4 | lancé volontairement dans le filet | 0,113 m | 0 |

L'étiquette réelle est cohérente avec la géométrie estimée : le dernier lancer
est le plus proche. Le premier est néanmoins estimé à seulement 14,2 cm tout
en étant un échec, ce qui confirme qu'une distance calculée depuis la même
perception ne suffit pas à valider l'exactitude spatiale.

`pass_through` reste nul sur le dernier lancer parce que la politique utilise
un disque de rayon 0,10 m et exige un croisement strict du plan à l'intérieur
de ce rayon. Le minimum 3D observé vaut 0,113 m. Le filet physique peut attraper
une balle hors de ce disque idéal, ou la perception/TF peut encore être décalée.
Il ne faut pas élargir ce rayon uniquement pour rendre ce lancer « réussi » :
cela changerait le contrat appris dans Isaac.

## Extrinsèque actuelle contre extrinsèque précédente

Les mêmes points caméra ont été transformés avec la solution nettoyée du
24 août et avec la référence du 23 juillet. Au point de proximité minimale :

| Lancer | Distance avec la nouvelle extrinsèque | Distance avec l'ancienne |
|---|---:|---:|
| 1 | 0,141 m | 0,137 m |
| 2 | 0,463 m | 0,421 m |
| 3 | 0,331 m | 0,324 m |
| 4 | 0,113 m | 0,157 m |

Sur le lancer volontairement dirigé dans le filet, la nouvelle extrinsèque est
4,4 cm plus cohérente que l'ancienne. C'est un indice favorable, pas une
validation : les deux transformations déplacent ces points principalement de
6,5 à 9,0 cm suivant l'axe `y` de `base_link`, et aucune mesure externe ne
donne encore la vraie position du centre de la balle.

## Latence et suivi du robot

Temps mesurés depuis la réception du premier `/ball_state_raw` de chaque
lancer :

| Étape | Étendue observée |
|---|---:|
| Première régression valide | 67–99 ms |
| Première télémétrie valide | 80–113 ms |
| Première cible articulaire significative | 120–148 ms |
| Premier mouvement articulaire significatif | 156–185 ms |
| Retard cible → mouvement réel | 34–39 ms |

La majeure partie du retard est donc antérieure au contrôleur UR. La porte de
démarrage de la régression exige à elle seule une étendue temporelle minimale
de 60 ms. Le délai caméra physique → première détection brute n'est pas
mesurable proprement avec les timestamps actuels : le tracker ré-ancre
certains timestamps sur l'horloge ROS et ne sépare pas mesure, évaluation et
publication.

L'écart cible–articulations atteint 0,044 à 0,051 rad, soit environ
2,5 à 2,9 degrés. Il peut ajouter quelques centimètres d'erreur au filet, mais
les 34–39 ms du contrôleur ne suffisent pas à expliquer seuls les échecs.

## Diagnostic de la régression finale

Le rosbag ne montre pas de deuxième vol parasite ni de redémarrage erroné pour
ces quatre lancers. La régression robuste rejette aussi une partie des points
incohérents. L'hypothèse « la dernière régression envoie à elle seule une pose
complètement fausse » n'est donc pas confirmée pour cette session.

Un défaut structurel reste confirmé : le vol ne se termine que sur le sol,
un timeout ou un coast timeout. Il n'existe pas de terminaison lorsque la balle
a croisé le plan du filet, s'éloigne du robot ou vient d'avoir un impact. Les
états valides continuent donc derrière le robot, avec `y` négatif sur plusieurs
lancers, et la politique peut continuer à poursuivre une cible déjà perdue.

## Problèmes confirmés et incertitudes

### Confirmé par les données ou le code

- Le live Trace utilise encore `borderRatio=3,5 %`. Les champs corrigés
  `borderPixels` et `borderSpacingFactor` existent et ont été validés dans le
  benchmark, mais `MakeTraceSupportEdgeSettings()` ne les renseigne pas.
- Le lancer 2 contient une profondeur brute tardive de 3,943 m incompatible
  avec l'approche générale.
- Plusieurs états initiaux sont hors de l'enveloppe `latest-left`.
- La réaction réelle depuis la première mesure brute prend 156–185 ms.
- La régression n'arrête pas le vol au passage ou à l'impact près du filet.
- `pass_through` ne représente pas exactement la capture physique par le filet.
- Le suivi articulaire présente jusqu'à environ 0,05 rad d'écart.

### Probable mais pas encore isolé

- Une part du décalage spatial vient de la profondeur Trace à longue portée.
- Une part peut venir de `T_base_camera` ou de `wrist_3_link -> hoop_center`.
- Le retard et l'erreur de suivi non modélisés pendant l'entraînement réduisent
  la précision de la politique.

### Non démontré

- Que l'extrinsèque nettoyée soit fausse : ce lancer la favorise plutôt.
- Que la régression soit l'unique cause des mauvaises poses.
- Que le robot aurait attrapé le quatrième lancer sans que l'opérateur le
  dirige volontairement dans le filet.

## Améliorations classées par priorité

### P0 — Activer la correction de profondeur adaptative dans le live

Brancher `borderPixels` et `borderSpacingFactor` dans le chemin GUI/live et les
exposer comme paramètres persistants. Le point de départ justifié par le
benchmark est `borderPixels=0,75` et `borderSpacingFactor=1,75`, à valider en
rejouant ce H5 avant de commander le robot. Cette correction a réduit le RMSE
3D hors ligne de 1,716 m à 0,157 m sur les balles lointaines, sans réduire le
taux de détection. C'est le changement ayant le lien causal le plus direct
avec les profondeurs des lancers lointains.

Ajouter en même temps une télémétrie minimale de qualité : largeur apparente,
espacement des bords, support, profondeur avant/après correction et incertitude
le long du rayon caméra. Une discontinuité telle que 1,1 → 3,9 m doit être
rejetée par une porte d'innovation dépendant de l'incertitude et de la vitesse
possible, pas seulement absorbée par la régression.

### P0 — Valider spatialement la nouvelle extrinsèque et le centre du filet

Avant de retoucher la politique, déplacer une balle sur une tige ou un pendule
à plusieurs positions mesurées du volume de capture, notamment au centre et
sur le plan du filet, puis comparer `ball_state_raw` transformé et
`hoop_center`. Une caméra événementielle ne voit pas une balle parfaitement
immobile ; il faut donc un petit mouvement contrôlé. Tester au minimum quatre
profondeurs entre environ 1,0 et 2,5 m.

La cible est de séparer trois erreurs aujourd'hui confondues : profondeur
monoculaire, `T_base_camera` et TF du filet. Tant que ce test n'existe pas,
ajuster un offset à partir de lancers peut compenser la mauvaise cause.

### P0 — Terminer explicitement le vol et refuser les entrées hors domaine

Ajouter le cycle `IDLE → CANDIDATE → RELEASED → TRACKING → COASTING →
PASSED_OR_IMPACT → REFRACTORY`. La transition `PASSED_OR_IMPACT` doit utiliser
le croisement du plan `hoop_center`, le mouvement d'éloignement et, si
possible, une déflexion détectée. Dès cette transition, figer la cible et
tenir la pose au lieu de poursuivre la balle derrière le robot.

Au démarrage, comparer position et vitesse ajustées aux plages du metadata de
la politique, avec une petite marge. En cas de lancer hors domaine, conserver
la perception et la télémétrie mais ne pas armer la politique. Pendant le
débogage, lancer dans l'enveloppe existante ; pour l'usage réel, élargir la
distribution et réentraîner.

### P1 — Instrumenter puis réduire la latence

Publier séparément `measurement_stamp`, `state_stamp` et `publish_stamp`, ainsi
que l'âge de la dernière vraie mesure. Synchroniser H5, rosbag et une référence
visuelle ou lumineuse permettrait de mesurer aussi libération → première
détection.

Ensuite seulement, tester par replay une réduction de `min_span_s` de 60 vers
40 ms et/ou un démarrage en deux étapes. Le gain attendu est environ 20 ms,
mais le changement ne doit être gardé que si la vitesse et l'intersection au
plan restent stables. Le `lead_time_s` doit rester à zéro tant que le contrat de
timestamps et le modèle d'entraînement ne représentent pas explicitement un
horizon futur.

### P1 — Adapter la politique au domaine réel

Deux voies sont possibles : standardiser les lancers d'essai dans
l'enveloppe actuelle, ou réentraîner avec une enveloppe correspondant aux
lancers humains réels. Le nouvel entraînement devrait inclure le bruit de
profondeur corrélé à la distance, les dropouts, l'incertitude extrinsèque, la
latence mesurée, le retard de l'UR3e et les limites réelles. Les lancers 2 à 4
ne permettent pas d'évaluer justement la politique actuelle car leurs vitesses
sont fortement hors distribution.

### P2 — Modéliser et réduire l'erreur articulaire

Comparer systématiquement commande et articulation réelle au moment prédit de
l'intersection. Si les pics proches de 0,05 rad persistent, intégrer les
dynamiques identifiées du robot dans Isaac ou adapter l'interpolateur/les
limites. Ce poste est secondaire par rapport à la profondeur et aux 120–148 ms
antérieurs à la première commande.

### P2 — Rendre les sessions reproductibles

Chaque session devrait conserver : H5, rosbag, dump des paramètres, hashes de
calibration et de modèle, résultat humain de chaque lancer, heure de chaque
lancer et indication « lancé normalement » ou « dirigé dans le filet ».
Ajouter une vidéo latérale à haute fréquence avec un flash/LED visible par les
deux systèmes donnerait enfin une référence spatiale et temporelle indépendante.

## Prochain test recommandé

1. Robot désarmé : rejouer le H5 avec l'ancien et le nouvel estimateur de bord,
   comparer continuité de profondeur, RMS balistique et temps de première pose.
2. Robot désarmé : faire un test de balle déplacée sur des positions mesurées
   autour du plan du filet pour valider extrinsèque et `hoop_center`.
3. Robot armé à vitesse prudente : réaliser 5 à 10 lancers dans l'enveloppe
   actuelle sans viser volontairement le filet, avec étiquette réussite/échec.
4. Seulement après ces trois étapes : décider s'il faut diminuer la porte de
   60 ms ou réentraîner sur une distribution plus large.

## Voir aussi

- [Trace contre circle fitting](trace-vs-circle-benchmark.md)
- [Robustesse et cycle de vol](perception-robustness-flight-lifecycle.md)
- [Test Trace réel](real-perception-trace-test.md)
- [État courant et blocages](../live-catch/current-status-and-blockers.md)
- [Latence des observations](../sim-to-real/observation-latency-and-models.md)
