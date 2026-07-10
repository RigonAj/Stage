# Revue perception robuste et contrôle fluide — DVXplorer, Trace et UR3e

Date : 2026-07-10  
Périmètre : événements DVXplorer → Trace 3D → régression balistique → observation PPO → commande UR3e  
Statut : revue de conception et plan de validation ; aucune validation caméra + robot réel n'est revendiquée ici

## 1. Conclusion courte

Le diagnostic du premier essai réel du 2026-07-09 est convaincant : le petit
mouvement du robot et le clignotement `command ON/OFF` s'expliquent d'abord par
deux `live_catch_node` et deux producteurs de `ball_state`. Le faux générateur
de balle publiait des heartbeats `valid=false` entre les estimations réelles ;
la politique et la commande étaient donc arrêtées et réinitialisées à répétition.
Cet essai ne permet pas de conclure que la perception était mauvaise, ni qu'elle
était bonne : la boucle de commande n'est jamais restée active assez longtemps
pour la juger.

Les correctifs ajoutés ensuite vont dans la bonne direction : source unique,
régression balistique à 60 Hz, mesures Trace non extrapolées en entrée,
pondération plus faible de la profondeur monoculaire, rejeu offline et
interpolation de commande à 500 Hz. Le dépôt compile et les tests unitaires
passent. Il manque cependant encore les preuves importantes : un enregistrement
réel, une vérité terrain, une mesure de latence fiable et plusieurs lancers à
blanc reproductibles.

Mon avis principal est donc le suivant : **ne pas chercher d'abord à rendre le
robot plus rapide**. Il faut d'abord rendre explicites le temps, la qualité et
le cycle de vie du vol. Une balle ne doit pas être seulement `valid=true/false` :
la chaîne doit savoir si elle acquiert un candidat, confirme le lâcher, suit un
vol, traverse une courte occultation, passe le plan du cerceau ou termine le
vol. C'est cette continuité qui donnera ensuite un contrôle fluide.

À la suite de cette revue, le défaut de bring-up a été remis à
`lead_time_s=0.0`. La régression évalue déjà la trajectoire à l'heure courante et
compense donc naturellement l'ancienneté des mesures. L'avance supplémentaire
de 200 ms est non mesurée, représente une grande fraction d'un vol et peut
terminer la balle au sol environ 200 ms trop tôt.

## 2. Ce qui a été vérifié dans le dépôt

Revue effectuée sur le commit `5696448` (`test`, 2026-07-10), qui contient les
changements issus de l'analyse du 2026-07-09.

- `ur3e_live_catch` : 127 tests réussis, 1 ignoré.
- `ur3e_web_ui` : 51 tests réussis.
- Compilation ciblée réussie pour `ur3e_catch_msgs`, `ball_tracking_cpp`,
  `ur3e_live_catch` et `ur3e_web_ui`.
- Aucun rosbag réel `/ball_state_raw` + `/ball_state` n'est encore présent.
- Les tests de régression sont synthétiques. Le test de bruit profondeur à
  6 cm qui valide `depth_sigma_scale=8` génère ses mesures à 120 Hz ; il ne
  couvre pas encore le cas réel où la sortie Trace est au mieux à 60 Hz, avec
  pertes, biais temporels et erreurs corrélées.
- La calibration intrinsèque récente est bonne dans son propre domaine
  (environ 0,149 px RMS), mais cela ne valide ni la profondeur obtenue par
  largeur de trace, ni la transformation 3D dans tout le volume de lancer.
- La calibration main-œil utilise 6 poses. Ses résidus sont encourageants, mais
  sa validation leave-one-out atteint environ 7,7 mm et une vérification
  physique dans le volume de capture reste nécessaire.

## 3. Chaîne actuelle et rôle de chaque étage

```text
événements DVXplorer horodatés
  -> filtre d'activité + fenêtre temporelle
  -> Trace : ruban 2D, largeur apparente, trajectoire 3D caméra
  -> ball_state_raw : mesures irrégulières, camera_optical
  -> TF camera_optical -> base_link
  -> régression : x/y linéaires + z à gravité fixée
  -> ball_state : position/vitesse prédites à 60 Hz, base_link
  -> observation PPO 33-D
  -> ActionMapper incrémental + limites vitesse/accélération
  -> interpolation de position 500 Hz
  -> forward_position_controller -> UR3e
```

Cette séparation est globalement saine. Trace doit produire des **mesures et
leur qualité** ; la régression doit porter la continuité temporelle et la
prédiction. Les deux étages ne doivent pas extrapoler en même temps.

Je ne ferais pas du message de coefficients `BallisticFit` la prochaine
priorité. Évaluer un état à 60 Hz coûte peu et la double quantification n'est
pas encore le problème dominant. Il faut d'abord corriger les temps, obtenir
une covariance crédible et valider les mesures réelles. Un message de fit
deviendra utile ensuite pour évaluer exactement à l'heure du tick de contrôle.

## 4. Relecture du diagnostic du 2026-07-09

### 4.1 Cause racine très probable du « robot qui frétille »

Les symptômes forment une signature forte :

- deux télémétries à 60 Hz annonçaient des valeurs opposées de
  `command_enabled`, d'où le clignotement de l'interface ;
- le `test_ball_node` au repos publiait `valid=false` à côté de la régression ;
- chaque état invalide déclenchait un hold et réinitialisait la mémoire de la
  politique, du mapper et du filtre de vitesse ;
- la commande ne survivait donc qu'un ou quelques ticks.

La règle « un seul producteur par topic contractuel » et l'option unique
`ur3e_catch_stack --tracker` corrigent le scénario opérationnel.

### 4.2 Limites du correctif d'exclusivité

Le watchdog actuel interroge le graphe toutes les 2 s. Un conflit peut donc
exister brièvement avant d'être connu. De plus, seul un conflit de producteurs
du topic balle bloque la commande ; un doublon de `live_catch_node` ou de
télémétrie avertit, mais ne bloque pas systématiquement les deux autorités de
commande.

Pour une commande robot robuste :

1. exécuter la vérification du graphe immédiatement au démarrage et au moment
   précis où le service `enable_command=true` est demandé ;
2. refuser l'armement sur **tout** doublon de nœud de commande, de télémétrie ou
   de producteur balle ;
3. à terme, mettre l'autorité de commande derrière un arbitre ou un bail
   exclusif, plutôt que derrière une simple détection périodique.

## 5. Problèmes critiques à traiter avant de régler les performances

### 5.1 L'avance de 200 ms est une hypothèse, pas une mesure

`BallRegression.step(now)` évalue déjà le modèle à `now`. Même si les dernières
mesures ont 30 ou 50 ms, le fit les propage donc jusqu'à l'heure courante. Avec
`lead_time_s=0`, la latence perception passée est déjà compensée par le modèle.

Ajouter `lead_time_s=0.2` présente au PPO la balle à `now + 200 ms` alors que
les articulations sont encore mesurées à `now`. Cela crée une observation
désynchronisée qui n'existait pas à l'entraînement. Le PPO reçoit déjà la
vitesse de la balle et a appris à anticiper sa trajectoire ; une grande avance
peut donc faire une double anticipation.

Effets concrets :

- le vol typique entraîné dure seulement quelques centaines de millisecondes ;
- le test de sol est évalué au temps futur et coupe `valid` environ 200 ms avant
  le contact réel ;
- le live node passe alors immédiatement en hold ;
- la position future peut franchir le plan du cerceau et modifier trop tôt le
  compteur de passage ;
- la politique voit une paire incohérente « balle future, robot courant ».

Décision recommandée :

- essais de perception et premiers essais commandés avec `lead_time_s=0.0` ;
- mesurer séparément la latence source → estimateur → politique → publication
  commande → mouvement observé ;
- ne conserver qu'une petite avance additionnelle justifiée et validée en
  rejeu, puis la reproduire dans l'entraînement. L'appeler
  `extra_prediction_horizon_s` éviterait de la confondre avec la compensation
  de l'ancienneté des mesures.

Décision appliquée le 2026-07-10 : `live_catch.yaml` utilise désormais 0.0 par
défaut ; le paramètre reste réglable à chaud pour de futurs essais contrôlés.

### 5.2 Le contrat d'horodatage documenté n'est pas celui du code

Deux écarts empêchent actuellement de mesurer la vraie latence :

1. Le tracker convertit l'horloge DVXplorer en heure ROS en ancrant le premier
   événement publié après un silence sur `ROS now`. Cela conserve correctement
   les écarts temporels **dans** le vol, mais force l'âge du premier point près
   de zéro et masque la latence fixe d'acquisition/traitement/rendu.
2. La régression calcule l'état à `now + lead_time_s`, mais
   `ball_regression_node` remplit actuellement `header.stamp` avec `now`, pas
   avec le temps d'évaluation. Avec une avance de 200 ms, la position est future
   alors que le stamp ne l'est pas. L'interface doit donc afficher un âge proche
   de 0 ms, et non −200 ms comme l'indiquent certains documents.

Un unique `Header.stamp` ne suffit plus. Le contrat robuste doit distinguer :

- `measurement_stamp` : heure du dernier événement réellement utilisé ;
- `state_stamp` : heure à laquelle position/vitesse sont évaluées ;
- `publish_stamp` : heure d'émission ROS ;
- éventuellement `command_apply_stamp` côté contrôle.

Le watchdog doit surveiller `now - measurement_stamp`, pas seulement la
fraîcheur d'un prédicteur qui continue à publier. Le consommateur doit comparer
la politique à `state_stamp`. Tant que le message n'est pas étendu, il faut au
minimum publier des diagnostics distincts `source_age`, `fit_age` et
`prediction_horizon`.

### 5.3 `confidence=1.0` ne signifie pas encore « bonne mesure »

Le gate `min_input_confidence=1.0` est utile pour empêcher les points de coast
du tracker de contaminer le fit. En revanche, toute fenêtre Trace fraîche et
valide reçoit actuellement `confidence=1.0`, qu'elle soit excellente ou tout
juste au-dessus des seuils. Le gate ne filtre donc pas :

- une largeur instable ;
- une profondeur biaisée ;
- une trace mélangée avec la main ou le robot ;
- un fit soutenu par peu de bins ;
- une extrapolation de bord très incertaine.

Trace devrait fournir un score ou, mieux, une covariance issue de : nombre
d'événements/bins, longueur temporelle, résidu du ruban, dispersion des
largeurs, condition du fit, fraction d'outliers et distance d'extrapolation.
La régression publie elle aussi une confiance décroissante pendant le coast,
mais `live_catch_node` n'utilise actuellement que `valid`. Une confiance faible
ne doit pas provoquer un ON/OFF à chaque tick ; elle doit alimenter une
hystérésis et une limite d'incertitude.

### 5.4 La cadence de mesure dépend encore du rendu

L'acquisition, Trace et la publication sont sérialisées avec `gui.Update()` et
`EndDrawing()`. `SetTargetFPS(60)` plafonne le système et un rendu lourd peut le
faire descendre. Les seuils de démarrage de la régression sont exprimés en
nombre d'échantillons et en durée ; une baisse de cadence retarde directement
le premier état valide.

La cible n'est pas nécessairement « le plus de Hz possible ». Il faut :

- un thread ou un mode headless pour acquisition → Trace → publication ;
- un rendu consommateur d'un snapshot, à 20–30 Hz si nécessaire ;
- une borne sur l'intervalle entre mesures et non une moyenne trompeuse ;
- des files bornées qui abandonnent les anciens lots plutôt que d'accumuler du
  retard.

### 5.5 Le modèle de bruit profondeur reste approximatif

La profondeur Trace suit approximativement :

```text
z = f_effectif * diamètre / largeur_px
sigma_z ≈ z * sigma_largeur / largeur_px
```

Le bruit est donc anisotrope **et hétéroscédastique** : il augmente lorsque la
balle est loin et petite. `depth_sigma_scale=8` est une bonne amélioration par
rapport à un bruit isotrope, mais l'implémentation actuelle ne conserve que la
diagonale de la covariance exprimée dans `base_link`. Lorsque le rayon caméra
n'est aligné avec aucun axe de base, cette approximation affaiblit aussi une
partie des erreurs latérales et perd les corrélations entre axes.

Pour les résidus et gates, utiliser exactement :

```text
e_parallel = dot(erreur, rayon_camera)
e_perp     = erreur - e_parallel * rayon_camera
d²         = ||e_perp||² / sigma_lat² + e_parallel² / sigma_depth²
```

Pour le fit, utiliser la covariance 3×3 complète de chaque mesure dans une
petite résolution GLS robuste. `sigma_depth` devrait venir de la dispersion de
largeur observée, pas seulement d'un facteur global.

### 5.6 Le contrôle de cohérence balistique ignore ce même bruit

Après 150 ms, la régression compare le `z` brut à une parabole libre et à une
parabole à gravité fixée. Ce contrôle n'utilise ni la covariance anisotrope, ni
les poids robustes du fit principal. Il peut donc interrompre un vrai vol à
faible cadence quand le bruit profondeur se projette sur l'axe vertical.

Il faut au minimum :

- comparer des résidus blanchis par la covariance ;
- demander plusieurs violations consécutives avant d'abandonner le vol ;
- enregistrer la courbure estimée, son incertitude et la raison d'arrêt ;
- tester à 30 et 60 Hz avec dropout et biais corrélé, pas seulement à 120 Hz.

Il ne faut pas non plus supposer qu'une balle réelle suit exactement le vide :
une balle de 90 mm légère peut subir une traînée et un effet de rotation non
négligeables. Mesurer son diamètre **et sa masse**, puis vérifier sur les
rosbags si `x/y` restent linéaires et si l'accélération verticale reste proche
de −g. Si ce n'est pas le cas, préférer un modèle à accélération bornée ou un
modèle de traînée identifié, avec le modèle balistique simple comme prior.

## 6. Cycle de vie de vol recommandé

Un `bool valid` ne représente pas suffisamment les transitions. La chaîne
devrait porter un `flight_id` monotone et une phase explicite.

| Phase | Conditions principales | Sortie perception | Comportement commande |
|---|---|---|---|
| `IDLE` | scène calme ou aucun candidat | heartbeat invalide, bruit de fond mesuré | hold stable ; politique réinitialisée une seule fois |
| `CANDIDATE` | burst cohérent dans la zone de lancement, taille/vitesse image plausibles | mesures diagnostiques, pas encore de contrôle | aucun mouvement |
| `RELEASED` | séparation de la main, direction vers le volume de capture, support temporel minimal | création du `flight_id`, fit initial + incertitude | armer seulement si l'interception est faisable |
| `TRACKING` | mesures acceptées et incertitude sous seuil | état courant à 60 Hz, covariance, temps au plan | PPO + limites + commande continue |
| `COASTING` | occultation brève ou trou de mesure | fit gelé, incertitude croissante | continuer seulement pendant une borne courte et si l'interception reste certaine |
| `PASSED_OR_IMPACT` | plan du cerceau franchi, déviation/contact près du cerceau ou balle qui s'éloigne | événement terminal avec raison | hold immédiat mais sans saut de consigne |
| `ENDED` | sol, timeout, incertitude excessive, vol non faisable | heartbeat invalide + résumé du vol | hold puis retour optionnel lent entre les lancers |
| `REFRACTORY` | 300 ms environ et retour obligatoire par la zone de lancement | ignorer rebond, main et récupération | aucune réactivation |

### 6.1 Début d'envoi

Le démarrage ne doit pas dépendre uniquement de quatre points, 60 ms et une
vitesse horizontale supérieure à 0,5 m/s. Avec `require_approach=false`, un
objet mobile dans le mauvais sens peut devenir valide, puis n'être rejeté comme
non balistique que 150 ms plus tard, après que le robot a commencé à bouger.

Ajouter un gate de faisabilité dérivé du modèle chargé :

- fermeture vers le plan du cerceau, pas simplement signe de `vy` codé en dur ;
- temps prédit au plan dans une fenêtre plausible ;
- point d'intersection dans le volume atteignable ;
- position/vitesse initiales compatibles avec l'enveloppe d'entraînement ;
- largeur de balle compatible avec le rayon physique et le volume calibré ;
- pas de candidat né dans le masque robot/hoop.

Pour `latest-left`, les métadonnées locales indiquent notamment :

- position de départ : `x=[0.2,0.6]`, `y=[1.2,2.1]`, `z=[0.5,1.2]` m ;
- vitesse : `vx=[-0.6,0.7]`, `vy=[-5.0,-4.0]`, `vz=[0.2,1.5]` m/s ;
- bruit position déclaré : 0,05 m ;
- `hold_side=left`, rayon logique de passage `disk_radius_m=0.1`.

Ces bornes ne doivent pas couper brutalement au millimètre près, mais elles
doivent définir une enveloppe avec marge. Un lancer lent, latéral ou dans le
mauvais sens est hors distribution et ne doit pas commander le robot lors des
premiers essais.

### 6.2 Pendant le vol

- Conserver les timestamps événement par mesure.
- Refaire le fit sur une fenêtre glissante robuste, tout en gardant un résumé
  du vol complet pour le diagnostic.
- Publier position, vitesse, covariance, support et état de phase.
- Calculer à chaque tick le temps et le point d'intersection avec le plan du
  cerceau, avec une incertitude.
- Utiliser une hystérésis : quelques mesures faibles ne doivent pas alterner
  TRACKING/IDLE.
- Geler ou rejeter les mesures produites par le bras près du robot ; ne jamais
  les laisser redémarrer un nouveau vol.

### 6.3 Fin d'envoi

La fin actuelle repose surtout sur le sol prédit, le timeout et le coast. Il
manque la fin pertinente pour le robot : **la balle a traversé ou dépassé le
plan de capture et s'éloigne**.

Terminer aussi lorsque :

- la distance signée au plan change de signe dans le sens d'approche, puis la
  balle continue derrière le plan ;
- le temps d'intersection est devenu négatif au-delà d'une petite marge ;
- un changement brutal de vitesse près du cerceau indique un impact/catch ;
- l'incertitude d'intersection dépasse le rayon utilisable ;
- le fit n'est plus physiquement compatible et aucune nouvelle mesure
  confirmée n'arrive.

Après la fin, garder la dernière consigne sûre. Le retour vers une pose
d'attente doit être une action séparée, lente, hors vol ; il ne faut pas que la
queue d'événements, un rebond ou la main qui récupère la balle réarme le PPO.

## 7. Rendre Trace réellement robuste

### 7.1 Calibration utile à la tâche

Le RMS de mire ne suffit pas. Trois validations doivent être séparées :

1. **intrinsèques** : reprojection sur observations tenues à l'écart ;
2. **main-œil** : positions connues réparties dans le volume, exprimées dans
   `base_link` ;
3. **profondeur par largeur** : balle réelle déplacée à plusieurs profondeurs
   et vitesses, avec comparaison à une référence indépendante.

Mesurer le rayon au pied à coulisse. Une erreur de 10 % sur le rayon produit
environ 10 % d'erreur de profondeur. Valider au moins les zones proche,
médiane et lointaine du lancer, pas seulement le centre de l'image.

Une méthode pratique est de déplacer la balle sur une trajectoire connue par
le robot ou un rail fin, en masquant le bras dans l'image, puis de comparer la
sortie Trace à la cinématique. Pour la dynamique réelle, ajouter une vidéo
rapide ou un second système de référence synchronisé sur plusieurs lancers.

### 7.2 Contraste, polarité et bruit événementiel

Trace utilise actuellement la polarité négative par défaut. Le meilleur choix
dépend du contraste balle/fond et de la direction. Pour éviter un réglage
fragile :

- fitter séparément positif, négatif et éventuellement les deux ;
- sélectionner le candidat ayant la meilleure cohérence largeur/temps ;
- enregistrer le ratio de polarités et le taux d'événements ;
- figer et journaliser les biases DVXplorer, l'éclairage et le fond utilisés ;
- inclure des essais avec scintillement, reflets et faible contraste.

### 7.3 ROI et mouvement du robot

La ROI est pleine image par défaut et réglée manuellement. Elle n'est donc pas
reproductible au prochain lancement. Les options qui protègent le mieux la
profondeur (`Edge refine`, lissage de largeur) sont elles aussi désactivées par
défaut et seulement accessibles dans l'interface.

Priorités :

- exposer ROI, polarité, mémoire Trace, edge refine et lissage en paramètres
  ROS ;
- sauvegarder un profil nommé avec chaque rosbag ;
- conserver une grande zone d'acquisition, puis une ROI dynamique en tube
  autour de la trajectoire image une fois le vol acquis ;
- projeter un masque du robot depuis TF/URDF dans l'image, ou au minimum utiliser
  un masque polygonal statique couvrant le bras et le cerceau ;
- revenir à l'acquisition large seulement après fin/refractory.

### 7.4 Score qualité Trace

Un état Trace doit être refusé ou dégradé si :

- le nombre de bins valides ou la durée couverte est trop faible ;
- la largeur change plus vite que ce qu'autorise une trajectoire 3D plausible ;
- les deux bords ne sont pas parallèles/cohérents ;
- le point évalué est trop loin du domaine temporel réellement observé ;
- la profondeur saute alors que le mouvement image est continu ;
- plusieurs composantes mobiles concurrentes existent.

Le réglage `Edge refine` corrige les caps de début/fin où la section visible est
une corde et non le diamètre. Le lissage de largeur réduit le bruit amplifié par
`1/largeur`. Ils doivent être comparés A/B sur les mêmes enregistrements, puis
activés par défaut seulement s'ils réduisent l'erreur de profondeur et non
seulement si la courbe paraît plus jolie.

## 8. Contrôle fluide : ce qui est déjà bon et ce qui manque

### 8.1 Bons choix actuels

- Le mapper respecte les métadonnées du modèle et l'intégrateur incrémental
  entraîné.
- Les limites position/vitesse/accélération restent indépendantes du PPO.
- La consigne 60 Hz est interpolée à 500 Hz pour satisfaire le driver UR.
- Une fin de vol conserve la dernière consigne au lieu de demander un retour
  instantané.
- Les pertes courtes sont absorbées par la régression avant d'atteindre le PPO.

### 8.2 Pourquoi le mouvement peut encore sembler lent ou brutal

Le modèle gauche déclare les vitesses complètes du robot et des accélérations
élevées, tandis que `v_safe_scale=0.5` divise vitesses **et** accélérations lors
du bring-up. Ce choix est prudent, mais il change la dynamique vue à
l'entraînement. De plus, les observations précédentes ont montré des actions
brutes souvent saturées : la fluidité vient alors presque entièrement des
limiteurs, pas d'une politique douce.

Il ne faut pas corriger cela par un simple filtre passe-bas sur l'action : il
ajouterait une latence non entraînée. Préférer :

- mesurer la fraction de ticks où chaque action est saturée ;
- publier les flags position/rate/acceleration limités dans la télémétrie ;
- mesurer consigne 500 Hz, position réelle, vitesse, accélération et erreur de
  suivi ;
- utiliser un interpolateur 500 Hz limité aussi en accélération/jerk si les
  mesures montrent des inversions sèches ;
- réentraîner avec les vraies limites, le délai, la dynamique identifiée, le
  bruit corrélé, les dropouts et éventuellement la traînée ;
- ajouter à l'entraînement une pénalité sur variation d'action/jerk si l'objectif
  est un geste fluide et non seulement une réussite binaire.

Le passage de `v_safe_scale=0.5` à 0.7, 0.85 puis 1.0 doit se faire uniquement
après des lancers virtuels et réels enregistrés sans rejet driver ni watchdog.
Les valeurs supérieures à 1.0 ne sont pas une solution de perception et sortent
du contrat d'entraînement.

### 8.3 Continuité de commande selon la phase

- `IDLE/CANDIDATE` : hold, aucune alternance de cibles.
- `TRACKING` : action PPO normale.
- `COASTING` court : conserver la continuité seulement si l'incertitude et le
  temps restant au plan sont bornés ; sinon hold une seule fois.
- `ENDED` : hold de la dernière consigne ; réinitialiser le PPO exactement une
  fois par changement de `flight_id`/phase, pas à chaque heartbeat.
- Retour à la pose d'attente : contrôleur de trajectoire, vitesse basse, action
  explicite entre deux lancers.

## 9. Protocole recommandé avant le prochain mouvement réel

### Étape A — remettre les hypothèses à zéro

1. Un seul stack : `ur3e_catch_stop`, puis `ur3e_catch_stack --real --tracker`.
2. Commande robot OFF.
3. Vérifier un producteur sur `/ball_state` et `/catch_telemetry`.
4. Vérifier `base_link <- camera_optical` et `base_link <- hoop_center`.
5. Vérifier rayon réel, modèle `latest-left`, montage et TF côté gauche.
6. Régler :

```bash
ros2 param get /ball_regression_node lead_time_s  # attendu : 0.0
```

### Étape B — enregistrer tout ce qui permet de comprendre

Pour chaque session :

```bash
ros2 bag record \
  /ball_state_raw /ball_state /joint_states /tf /tf_static \
  /catch_telemetry /forward_position_controller/commands /parameter_events
```

En parallèle, enregistrer les événements DVXplorer/H5 afin de pouvoir rejouer
Trace, pas seulement la régression. Sauvegarder aussi :

- dump des paramètres des trois nœuds ;
- ROI, polarité et toggles Trace ;
- rayon, diamètre, masse et type de balle ;
- éclairage/fond ;
- commit Git, modèle et métadonnées ;
- heure de début de chaque lancer ou un événement de marquage.

### Étape C — matrice perception seule

Commande toujours OFF :

1. 20 lancers dans l'enveloppe `latest-left` ;
2. profondeurs proche/moyenne/lointaine ;
3. vitesses proches des `vy=[-5,-4] m/s` entraînées ;
4. 5 occultations brèves ;
5. robot qui bouge sans balle ;
6. main seule dans la zone de lancement ;
7. balle déplacée dans le mauvais sens ;
8. rebond et récupération après un vol ;
9. variations de contraste/éclairage raisonnables.

Pour chaque vol, produire un résumé : premier événement, première mesure Trace,
premier état régression valide, premier tick PPO, dernier échantillon réel,
début du coast, passage plan, fin et raison d'arrêt.

### Étape D — critères provisoires de passage

Les valeurs suivantes sont des cibles de bring-up à confirmer avec les données,
pas des garanties déjà atteintes :

- 0 faux démarrage pendant les essais robot/main sans balle ;
- temps premier événement → état contrôle valide p95 ≤ 100 ms ;
- aucun trou de mesure > 100 ms durant un vol nominal ;
- taux de vols interrompus `non_ballistic` proche de 0 sur les vrais lancers ;
- erreur p95 au point d'intersection du plan ≤ 5 cm ;
- erreur de vitesse p95 ≤ 0,3 m/s ;
- erreur de temps au plan p95 ≤ 25 ms ;
- aucune confusion de frame, aucun stamp futur/incohérent, aucun producteur en
  double ;
- sortie régression à 60 Hz sans cacher l'âge de la dernière mesure réelle.

### Étape E — commande progressive

1. Revalider d'abord la chaîne balle virtuelle sur le vrai UR3e.
2. Faire au moins 10 lancers réels en dry-run avec ghost stable.
3. Armer à `v_safe_scale=0.5`, `lead=0`, un lancer à la fois, E-stop en main.
4. Désarmer et lire le résumé/rosbag après chaque petit lot.
5. Monter 0.7 → 0.85 → 1.0 seulement si aucun rejet driver, conflit, watchdog,
   oscillation ou erreur d'intersection croissante n'apparaît.
6. Ne modifier qu'une famille de paramètres par lot afin de garder une preuve
   causale.

## 10. Ordre d'implémentation conseillé

### P0 — avant le prochain essai commandé

1. **Fait :** conserver `lead_time_s=0.0` comme défaut de validation.
2. Corriger/clarifier les timestamps et ajouter `source_age` + horizon de
   prédiction aux diagnostics.
3. Bloquer l'armement immédiatement sur tout doublon d'autorité/producteur.
4. Enregistrer rosbags + H5 et produire des résumés par `flight_id`.
5. Valider physiquement profondeur et TF dans le volume.

### P1 — robustesse perception/vol

1. Découpler Trace du rendu.
2. Exposer et sauvegarder tous les paramètres déterminants de Trace.
3. Produire une covariance/qualité réelle plutôt que `confidence=1.0` binaire.
4. Ajouter phases de vol, gate d'approche/faisabilité et fin au plan du cerceau.
5. Rendre la cohérence balistique compatible avec le bruit anisotrope et les
   faibles cadences.
6. Passer à la covariance complète le long du rayon caméra.

### P2 — performance et fluidité

1. Identifier traînée/spin ou confirmer que le modèle sans traînée suffit.
2. Ajouter masque robot dynamique et ROI de suivi.
3. Mesurer jerk/erreur de suivi de la consigne 500 Hz.
4. Réentraîner avec limites, latence, bruit, dropout et dynamique réels.
5. Envisager ensuite un message `BallisticFit`/`InterceptState` évalué au tick
   exact, avec covariance et fenêtre de validité.

## 11. Verdict sur les choix proposés hier

Je conserverais :

- l'architecture `Trace mesures -> régression base_link -> PPO` ;
- l'interdiction de deux couches d'extrapolation ;
- le producteur unique ;
- le rejeu offline ;
- la prise en compte du bruit profondeur ;
- la sortie position + vitesse à fréquence stable.

Je modifierais ou reclasserais :

- `lead_time_s=0.2` : retiré du défaut de bring-up ; ne le réintroduire que s'il
  est mesuré, validé en rejeu et représenté à l'entraînement ;
- le contrat de stamp : à corriger avant toute étude de latence ;
- `BallisticFit` : utile, mais après timestamps/covariance/cycle de vie ;
- le rejet balistique : doit intégrer incertitude, cadence et aérodynamique ;
- `confidence=1.0` : insuffisant pour sélectionner de vraies bonnes mesures ;
- la règle « pas de Kalman » : ne pas changer maintenant par principe, mais ne
  pas l'interdire. Une initialisation batch robuste suivie d'un filtre à état
  et covariance peut devenir pertinente lorsque les données réelles existent.

Le prochain succès n'est pas « le robot a beaucoup bougé ». Le prochain succès
est un lancer enregistré où l'on peut expliquer, avec des temps et des
incertitudes cohérents, quand la balle a été détectée, quand le vol a été
confirmé, où elle devait croiser le cerceau, pourquoi la commande a continué ou
s'est arrêtée, et si le robot a suivi la consigne sans rejet.
