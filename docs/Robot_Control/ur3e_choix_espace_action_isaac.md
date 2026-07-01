# UR3e Ball-Catch — Choix de l'espace d'action (position / vitesse / accélération) pour l'entraînement Isaac Lab

> Statut (2026-06-23) : **document de décision**. Compare les trois espaces d'action
> possibles pour la policy PPO « attrape-balle » entraînée dans Isaac Lab (env
> **Direct**, `firsttraining_env.py`), avec avantages, inconvénients, mise en œuvre et
> **paramétrage concret** pour chaque cas. Recommandation en §7.

Documents liés :
- `ur3e_ball_catch_sim_to_real.md` — contraintes sim/entraînement/inférence (§2.1
  actionneur, §2.2 espace d'action, §5 déploiement).
- `ur3e_sim2real_propositions.md` — propositions sim-to-real (dont §4.2
  « safety-in-the-loop », directement liée à ce choix).
- `ur3e_live_catch_architecture.md` — boucle live + interfaces de commande UR3e.

---

## 1. Le critère décisif : ce que le robot réel accepte en streaming

Avant tout argument « RL », poser la contrainte de déploiement. **On doit entraîner
dans l'espace d'action que l'on peut réellement streamer au robot.** Sur UR3e via
`ur_robot_driver` + ros2_control, les interfaces de commande **temps réel** sont :

| Interface réelle UR3e | Type de commande | Statut dans ce projet |
|---|---|---|
| `scaled_joint_trajectory_controller` | trajectoires de **position** (bufferisées) | utilisé pour l'**approche** (pas le live) |
| `forward_position_controller` | **position** articulaire, set-point par cycle | **utilisé en live catch** (`/forward_position_controller/commands`) |
| `forward_velocity_controller` / RTDE `speedj` | **vitesse** articulaire | **disponible** sur le driver, **non câblé** ici |
| (effort / couple temps réel) | **couple** articulaire | **non exposé** par le contrôleur UR standard |

**Conséquences immédiates :**
- La **position** est l'interface déjà câblée et validée (boucle live, couche safety,
  `CommandStreamer`). Coût de déploiement nul.
- La **vitesse** est techniquement déployable (le driver fournit
  `forward_velocity_controller`, et UR expose `speedj`), mais il faudrait **câbler un
  nouveau chemin de commande** et une nouvelle couche safety en vitesse.
- L'**accélération/couple** n'est **pas streamable** sur un UR3e standard : la baie de
  contrôle UR exécute sa propre boucle servo et n'accepte que des consignes de
  position ou de vitesse, **pas** de couple en temps réel. C'est le point bloquant
  de cet espace pour ce robot.

> Règle d'or sim-to-real : *l'espace d'action de l'entraînement = l'espace de la
> commande déployée*. Tout écart (entraîner en couple, déployer en position) rajoute
> une couche de conversion non triviale et une source de divergence.

---

## 2. Vue d'ensemble des trois espaces

| Espace | Sortie policy `a` (6) | Application en sim (Isaac) | Actionneur sim | Interface UR3e | Déployable tel quel |
|---|---|---|---|---|---|
| **Position** | cible articulaire (absolue ou Δ) | `set_joint_position_target(q_des)` | PD implicite (`stiffness`/`damping`) | `forward_position_controller` | ✅ **oui** (déjà fait) |
| **Vitesse** | cible de vitesse `qd_des` | `set_joint_velocity_target` **ou** intégrer `q_des = q + qd_des·dt` | damping pur (`stiffness=0`) | `forward_velocity_controller`/`speedj` | 🟡 oui, **à câbler** |
| **Accélération** | `qd̈_des` (ou couple) | dyn. inverse `τ=M·a+C+G` → `set_joint_effort_target`, **ou** double intégration → position | effort direct (`stiffness=damping=0`) | aucune (couple non exposé) | 🔴 **non** (sauf re-projection) |

Le reste du document détaille chaque ligne.

---

## 3. Espace d'action POSITION

### 3.1 Mécanisme en simulation
La policy sort une cible de position articulaire ; l'actionneur PD **implicite**
d'Isaac Lab (`ImplicitActuatorCfg`, déjà utilisé dans `ur_gripper.py`) calcule le
couple :

```
τ = stiffness · (q_des − q) + damping · (qd_des − qd),  borné par effort_limit_sim
```

avec `qd_des = 0` par défaut. La cible est posée dans `_apply_action()` via
`self.robot.set_joint_position_target(q_des)`.

**Deux variantes** (cf. `ur3e_ball_catch_sim_to_real.md §2.2`) :
- **Absolue** (ancienne) : `q_des = action · action_scale` (`action_scale = 0.5`). La
  policy peut commander n'importe quelle cible → vitesses/accélérations implicites
  énormes (le rollout actuel monte à ~169 rad/s commandés). **À éviter.**
- **Incrémentale bornée** (appliquée, recommandée) :
  `q_des = clip(q + action · Δ, q_min, q_max)`, `Δ = v_safe · dt_step`. La vitesse
  commandée est **intrinsèquement bornée** à `v_safe` quelle que soit la sortie réseau.

### 3.2 Avantages
- **Déployable sans rien ajouter** : c'est l'interface `forward_position_controller`
  déjà câblée + la couche safety (clip/rate/accel) déjà écrite autour de cibles de
  position.
- **Stable et robuste** : le PD implicite PhysX est inconditionnellement stable, peu
  sensible au pas de temps. Entraînement PPO facile, peu de divergence numérique.
- **Sûreté naturelle** en variante incrémentale : borne de vitesse gratuite via `Δ`.
- **Lisibilité** : la cible a une unité physique directe (rad), facile à clipper aux
  bornes articulaires et à visualiser (fantôme policy du web UI).

### 3.3 Inconvénients
- **Dépend de `stiffness`/`damping`** : un PD trop raide (l'actuel `stiffness = 800`)
  donne un suivi quasi-instantané **irréaliste** → mismatch sim-to-real
  (`ur3e_sim2real_propositions.md §3.4`). À corriger par **system-id** (§3.5 du plan
  sim-to-real) et/ou en mettant la **safety dans la boucle d'entraînement**
  (`ur3e_sim2real_propositions.md §4.2`).
- La policy ne « voit » pas directement la vitesse qu'elle commande : c'est le PD qui
  la réalise. Pour une tâche très dynamique, raisonner en vitesse peut être plus
  direct (cf. §4).

### 3.4 Paramétrage Isaac Lab (cas position)
- Actionneur (`ur_gripper.py`, `ImplicitActuatorCfg`) :
  - `stiffness` : **à identifier** par réponse à l'échelon réelle (départ ~100–300,
    pas 800). C'est le levier n°1 du réalisme.
  - `damping` : ~2·√(stiffness·inertie) comme point de départ, puis ajuster sur le
    dépassement mesuré.
  - `effort_limit_sim = [56, 56, 28, 12, 12, 12]` Nm, cible à aligner avec
    `ur_description` (remplace l'ancienne valeur projet `[54, 54, 28, 9, 9, 9]`).
  - `velocity_limit_sim` = limites nominales UR3e `[3.1416×3, 6.2832×3]` rad/s.
- Espace d'action (`_pre_physics_step` / `firsttraining_env_cfg.py`) :
  - `Δ = v_safe · dt_step` (incrémental), clipping dans l'environnement
    (`clip_actions: False` côté SKRL pour l'export courant).
  - bornes position `[±2π, ±2π, ±π, ±2π, ±2π, ±2π]`.
  - `a_safe = 4·v_safe` (borne d'accélération).
- **Cohérence déploiement** : `ActionMapper` mode `faithful`/incrémental côté ROS doit
  utiliser **les mêmes** `action_scale`/`Δ`/`v_safe`/`a_safe` (`action.py`, `limits.py`).

---

## 4. Espace d'action VITESSE

### 4.1 Mécanisme en simulation
La policy sort une **cible de vitesse** `qd_des`. Deux mises en œuvre :
- **(a) Actionneur en vitesse** : `stiffness = 0`, `damping > 0`, puis
  `self.robot.set_joint_velocity_target(qd_des)` → `τ = damping·(qd_des − qd)`. Le
  drive suit la vitesse.
- **(b) Intégration cinématique** (souvent plus stable et **plus proche du
  déploiement position**) : `q_des = q + qd_des · dt_step`, puis **position control**
  comme §3. C'est en réalité **identique à la position incrémentale** où l'action
  *est* une vitesse normalisée : `qd_des = action · v_safe`, `q_des = q + qd_des·dt`.

> Remarque importante : la **variante incrémentale de position (§3.1)** est
> mathématiquement une **commande de vitesse intégrée**. Choisir « vitesse » via (b)
> revient donc à ce que le projet fait déjà, à la seule différence d'**interpréter**
> et d'éventuellement **réguler** l'action comme une vitesse.

### 4.2 Avantages
- **Naturel pour une tâche réactive** : intercepter une balle, c'est surtout une
  affaire de vitesse articulaire ; la policy raisonne directement dans la grandeur
  utile.
- **Bornage trivial** : `clip(qd_des, −v_safe, v_safe)` borne la vitesse par
  construction (comme `Δ` en position incrémentale).
- **Déployable** sur UR3e via `forward_velocity_controller` / `speedj` (vraie
  commande de vitesse, sans passer par un PD de position).

### 4.3 Inconvénients
- **Dérive de position** : une commande de vitesse n'a pas de référence de position →
  intégration → dérive et besoin d'un **clamp de position** explicite (rejet hors
  bornes) en plus du clamp de vitesse. En cas de perte de perception, une vitesse
  résiduelle est plus dangereuse qu'une cible de position figée (la position « tient »
  sa consigne, la vitesse « part »). Le watchdog doit commander **vitesse nulle**, pas
  « hold ».
- **À câbler côté ROS** : nouveau chemin de commande (`forward_velocity_controller`),
  nouvelle couche safety en vitesse, nouvelle bascule de contrôleur. Travail non
  négligeable, alors que la position est déjà validée.
- **Calibration `damping`** (variante a) tout aussi sensible que `stiffness` en
  position.

### 4.4 Paramétrage Isaac Lab (cas vitesse)
- Variante (a) actionneur vitesse : `ImplicitActuatorCfg(stiffness=0.0, damping=K_d,
  effort_limit_sim=[56,56,28,12,12,12])`, `set_joint_velocity_target`. `K_d` règle la
  raideur de suivi de vitesse (à system-id).
- Variante (b) intégrée (recommandée si on veut « vitesse ») : garder l'actionneur
  position (§3.4), action = vitesse normalisée, `qd_des = action · v_safe`,
  `q_des = q + qd_des · dt_step`, **clip position** ensuite.
- Espace d'action : clipping dans l'environnement, échelle `v_safe` par joint,
  `a_safe = 4·v_safe` toujours appliquée (limiter la **variation** de vitesse).
- **Déploiement** : `forward_velocity_controller` (publier `Float64MultiArray` de
  vitesses) + watchdog → **0 rad/s** au lieu de hold.

---

## 5. Espace d'action ACCÉLÉRATION (≈ couple / effort)

### 5.1 Mécanisme en simulation
Il n'existe **pas** de « set_joint_acceleration » direct en physique. Trois voies :
- **(a) Couple direct (effort)** : la policy sort un couple `τ` (ou une accélération
  convertie en couple). `ImplicitActuatorCfg(stiffness=0, damping=0)` +
  `self.robot.set_joint_effort_target(τ)`. C'est le « vrai » contrôle dynamique.
- **(b) Couple calculé (computed-torque)** : la policy sort `a_des` (accélération),
  on calcule `τ = M(q)·a_des + C(q,qd)·qd + G(q)` via la **dynamique inverse**
  (matrice de masse + termes de Coriolis/gravité exposés par l'articulation), puis
  `set_joint_effort_target(τ)`.
- **(c) Double intégration cinématique** : `qd_des += a_des·dt` ; `q_des += qd_des·dt`
  → position control. La policy « raisonne en accélération » mais on **déploie en
  position**. Seule voie réellement transférable sur UR3e.

### 5.2 Avantages
- **Le plus physique** : commande au niveau couple, dynamique riche, mouvements
  potentiellement plus performants/agiles (recherche).
- **(c)** offre une commande **très lisse** (l'accélération bornée ⇒ jerk borné) tout
  en restant déployable en position.

### 5.3 Inconvénients
- **Non déployable en couple sur UR3e** (a/b) : pas d'interface de couple temps réel
  → l'entraînement en couple est un **cul-de-sac sim-to-real** pour ce robot.
- **Sensibilité numérique / instabilité** : le contrôle en couple est beaucoup plus
  dur à stabiliser (PPO peut diverger), exige un `sim.dt` plus fin et un **system-id
  dynamique précis** (masses, frottements, gravité) — bien plus exigeant que pour un
  PD de position.
- **(b) computed-torque** dépend d'un modèle dynamique fidèle ; toute erreur de masse
  d'inertie (charge utile hoop incluse) se traduit directement en erreur de
  mouvement.
- **(c) double intégration** : ajoute **deux états intégrateurs** (vitesse + position)
  → dérive, retard de phase, et une couche safety plus complexe (borne sur a, v **et**
  q simultanément). En perte de perception, l'arrêt sûr est plus délicat (il faut
  ramener `a` puis `v` à zéro).

### 5.4 Paramétrage Isaac Lab (cas accélération)
- Voie (a/b) : `ImplicitActuatorCfg(stiffness=0.0, damping=0.0,
  effort_limit_sim=[56,56,28,12,12,12])`, `set_joint_effort_target`. Pour (b),
  récupérer masse/biais via l'articulation (dynamique inverse) — non trivial.
- Voie (c) : actionneur position (§3.4) ; action = accélération normalisée,
  `a_des = action · a_safe` ; `qd_des = clip(qd + a_des·dt, ±v_safe)` ;
  `q_des = clip(q + qd_des·dt, q_min, q_max)`.
- Dans **tous** les cas, garder `effort_limit_sim` réaliste et, en (c), conserver
  `v_safe`/`a_safe`/bornes position.

> **Verdict accélération** : intéressant en recherche, mais (a/b) **non transférable**
> au UR3e (pas de couple), et (c) revient à une position lissée au prix de deux
> intégrateurs. À ne retenir que si la lissité du jerk devient un objectif explicite.

---

## 6. Le compromis déjà en place : position incrémentale = vitesse bornée déployable

Point clé à garder en tête : la **position incrémentale bornée** (§3.1, déjà
appliquée) cumule l'essentiel des avantages :
- elle **borne la vitesse** comme une commande de vitesse (`Δ = v_safe·dt`) ;
- avec `a_safe`, elle **borne aussi l'accélération** (lissité de l'accélération) ;
- elle se **déploie en position** (`forward_position_controller`, déjà câblé) ;
- l'arrêt sûr est trivial (la position « tient » sa dernière consigne).

Autrement dit, le schéma actuel est **« commande de position dont l'incrément encode
une vitesse, avec borne d'accélération »** — il capture les bénéfices de la vitesse et
de l'accélération **sans** leurs inconvénients de déploiement.

---

## 7. Recommandation

| Choix | Quand le retenir | Effort sim-to-real |
|---|---|---|
| **Position incrémentale bornée** ✅ (défaut recommandé) | tâche réactive sur UR3e via `forward_position_controller` ; couche safety déjà écrite | **nul** (déjà en place) |
| **Vitesse** 🟡 (alternative crédible) | si on veut que la policy raisonne explicitement en vitesse et qu'on accepte de câbler `forward_velocity_controller` + safety vitesse | moyen (nouveau chemin commande) |
| **Accélération / couple** 🔴 (déconseillé pour ce robot) | recherche pure, ou besoin explicite de jerk borné via voie (c) | élevé / bloquant (pas de couple UR3e) |

**Recommandation : garder l'espace d'action POSITION en variante incrémentale
bornée.** C'est le seul qui (1) se déploie sans nouveau chemin de commande, (2)
réutilise la couche safety existante, (3) borne nativement vitesse et accélération, et
(4) offre un arrêt sûr trivial. Concentrer l'effort sim-to-real non pas sur le
*changement* d'espace d'action mais sur le **réalisme de l'actionneur** (system-id
`stiffness`/`damping`) et sur la **safety-in-the-loop** à l'entraînement
(`ur3e_sim2real_propositions.md §4.2`), qui éliminent le vrai problème de transfert.

Si l'on veut **tester** la vitesse, le faire via la **variante intégrée (b)** (§4.1) :
elle se compare directement à la position incrémentale (même déploiement) et isole le
seul effet « raisonner en vitesse » sans introduire de nouveau risque de commande.

---

## 8. Comment trancher empiriquement (protocole)

Entraîner 2 variantes comparables (position incrémentale vs vitesse intégrée), même
reward, même domain randomization, même budget, puis comparer :

1. **Taux de succès** en éval (`play.py --eval_episodes N`).
2. **Réalisme physique** sur rollouts ré-enregistrés : `vitesse réalisée ≤ v_safe` par
   joint, `accélération ≤ a_safe`, aucune cible hors bornes
   (`ur3e_ball_catch_sim_to_real.md §6.2`).
3. **Lissité** : `Σ‖aₜ − aₜ₋₁‖²` et jerk articulaire (proxy du confort/contrainte
   mécanique réelle).
4. **Robustesse latence** : balayage `L_a`/`L_o`, dégradation du succès.
5. **Déployabilité** : la variante gagnante doit correspondre à une interface UR3e
   câblée (position = oui ; vitesse = à câbler ; couple = non).

Graver le choix dans `policy_metadata.json` (`action_semantics`) pour que le **gate
côté inférence** (`ur3e_sim2real_propositions.md §5.4`) refuse un déploiement
incohérent.

---

## 9. Récapitulatif

- Le **critère n°1 est le déploiement** : le UR3e n'accepte en temps réel que de la
  **position** (câblée) ou de la **vitesse** (à câbler), **pas de couple**.
- **Position incrémentale bornée = recommandée** : déjà en place, déployable, borne
  vitesse **et** accélération, arrêt sûr trivial.
- **Vitesse** : alternative crédible si on raisonne en vitesse et qu'on câble le
  contrôleur vitesse ; attention à la dérive de position et à l'arrêt (vitesse nulle).
- **Accélération/couple** : non transférable au UR3e (pas de couple), instable à
  entraîner ; la seule voie déployable (double intégration) revient à une position
  lissée coûteuse.
- L'effort sim-to-real le plus rentable n'est **pas** de changer d'espace d'action,
  mais de rendre l'**actionneur réaliste** (system-id) et d'entraîner **avec la safety
  dans la boucle**.
