# UR3e — Paramètres d'actionneur : valeurs de référence & sources (sim-to-real)

> Statut (2026-06-23) : **document de référence**. Regroupe les valeurs **UR3e**
> (limites d'effort/vitesse/position, raideur/amortissement) pour calibrer la
> simulation Isaac Lab et fixer les bornes safety, avec leurs sources. Sert de base au
> **system-id** et au choix de l'espace d'action.

Documents liés :
- `ur3e_ball_catch_sim_to_real.md` — §2.1 actionneur réaliste, §2.3 limites.
- `ur3e_choix_espace_action_isaac.md` — choix position/vitesse/accélération.
- `ur3e_sim2real_propositions.md` — §4.2 safety-in-the-loop, §4.7 system-id.

> ⚠️ **Source de vérité finale** : le `joint_limits.yaml` du `ur_description` du
> **driver réellement lancé** sur la machine ROS. Les valeurs ci-dessous sont les
> valeurs publiées de référence ; toujours les recouper avec le fichier embarqué.

---

## 1. Limites articulaires UR3e (les valeurs à utiliser)

Source : `Universal_Robots_ROS2_Description/config/ur3e/joint_limits.yaml`
(= ce que le driver applique réellement).

| Joint | position min/max | vitesse max | **effort max** |
|---|---:|---:|---:|
| `shoulder_pan_joint` | ±360° = **±6.2832 rad** | 180°/s = **3.1416 rad/s** | **56 Nm** |
| `shoulder_lift_joint` | ±360° = **±6.2832 rad** | 180°/s = **3.1416 rad/s** | **56 Nm** |
| `elbow_joint` | ±180° = **±3.1416 rad**¹ | 180°/s = **3.1416 rad/s** | **28 Nm** |
| `wrist_1_joint` | ±360° = **±6.2832 rad** | 360°/s = **6.2832 rad/s** | **12 Nm** |
| `wrist_2_joint` | ±360° = **±6.2832 rad** | 360°/s = **6.2832 rad/s** | **12 Nm** |
| `wrist_3_joint` | ±360° = **±6.2832 rad** | 360°/s = **6.2832 rad/s** | **12 Nm** |

**En vecteurs (ordre canonique `shoulder_pan … wrist_3`) :**
```
position_max_rad   = [ 6.2832,  6.2832,  3.1416,  6.2832,  6.2832,  6.2832]
position_min_rad   = [-6.2832, -6.2832, -3.1416, -6.2832, -6.2832, -6.2832]
velocity_max_rad_s = [ 3.1416,  3.1416,  3.1416,  6.2832,  6.2832,  6.2832]
effort_max_Nm      = [   56,       56,      28,      12,      12,      12  ]
```

¹ L'`elbow` est **artificiellement limité à ±180°** (au lieu de ±360°) dans
`ur_description`, pour éviter l'auto-collision épaule / les soucis de planification.
Cohérent avec la borne sim `±π` du projet.

Conversions : `180°/s = π = 3.1416 rad/s` · `360°/s = 2π = 6.2832 rad/s` ·
`360° = 2π = 6.2832 rad` · `180° = π = 3.1416 rad`.

**Source officielle (dans le repo)** : `UR3e_Official_doc.pdf` = *UR3e User Manual*.
§2.1 *Technical Specifications UR3e* (p. 17) confirme : vitesses **180°/s**
(épaule/coude) et **360°/s** (poignets) ; **outil ≈ 1 m/s** ; **plages articulaires
±360°** (bride d'outil illimitée) ; charge **3 kg** ; masse **11.1 kg** ; **fréquence
système 500 Hz** (fréquence système UR ; le débit ROS `/joint_states` réel est à
**mesurer** dans les logs — driver-dépendant). Conforme à `ur_description`,
sauf le coude que `ur_description` **borne logiciellement** à ±180° (le matériel
autorise ±360°).

> ⚠️ **Ne pas confondre** : la table « Maximum joint torques » du manuel
> (§5 *Assembly*, p. 36) donne les **charges de réaction sur le support/embase**
> (`Mz / Fz / Mxy / Fxy` — normal : `140 / 370 / 180 / 320` ; arrêts cat. 0/1/2 :
> `170 / 490 / 220 / 390`, worst-case × 2.5), pour **dimensionner le stand**. **Ce ne
> sont PAS** les couples max **par articulation** (`effort_limit`) : les efforts par
> joint (56/28/12 Nm, §1) viennent de `ur_description` / l'article UR « Max. joint
> torques UR-Series » (§5), pas de cette table.

---

## 2. Comparaison avec les valeurs actuelles du projet (à corriger)

Valeurs actuelles (d'après `ur3e_ball_catch_sim_to_real.md §0`,
`<ISAAC_REPO>/.../ur_gripper.py`) :

| Paramètre | Projet (`ur_gripper.py`) | UR3e officiel (`ur_description`) | Verdict |
|---|---|---|---|
| vitesse max | `[3.1416×3, 6.2832×3]` | `[3.1416×3, 6.2832×3]` | ✅ **identique** |
| position coude | `±π` | `±π` (±180°) | ✅ identique |
| effort épaules | **54 Nm** | **56 Nm** | 🔧 aligner |
| effort coude | 28 Nm | 28 Nm | ✅ identique |
| effort poignets | **9 Nm** | **12 Nm** | 🔧 **diverge — aligner** |

→ **Vitesses/positions OK.** Corriger les **efforts** : poignets `9 → 12`, épaules
`54 → 56`, en lisant le `joint_limits.yaml` **réellement embarqué** (avertissement en
tête). Les `[54,54,28,9,9,9]` du projet viennent d'anciennes specs « couple
utilisable » ; `ur_description` fait foi côté driver.

---

## 3. Raideur (`stiffness`) / amortissement (`damping`) : à identifier

**Il n'existe pas de valeur UR3e publiée** pour `stiffness`/`damping` : le vrai UR3e
est une baie fermée qui n'expose pas ses gains, et Isaac Lab **ne livre pas de config
UR3e**. Ces gains **doivent être identifiés** sur ton robot (§4).

⚠️ Les valeurs actuelles du projet `stiffness = 800, damping = 40` (uniformes) sont
**le défaut générique d'Isaac Lab** (config `UR10_CFG`, identique au Franka), **pas du
UR3e** → suivi trop rapide, cause du mismatch sim-to-real
(`ur3e_sim2real_propositions.md §3.4`). **À remplacer.**

**Seul gabarit de *structure* disponible** (faute de config UR3e) — Isaac Lab
`UR10e_CFG`, à transposer (le UR10e est **plus gros/lourd**, ne pas copier les
nombres) :

| Groupe | stiffness | damping | à retenir |
|---|---:|---:|---|
| épaules | 1320 | 72.66 | gains **par joint** |
| coude | 600 | 34.64 | **stiffness décroît** épaule→poignet |
| poignets | 216 | 29.39 | **amortissement quasi-critique** `D ≈ 2√(K·I)` |

→ Pour le UR3e : garder la **hiérarchie** `K_épaule > K_coude > K_poignet` et
`D ≈ 2√(K·I_eff)`, avec des `K` **plus faibles** (robot plus léger), puis affiner par
system-id. Ne pas garder le `800/40` uniforme.

> 📌 **Donnée terrain — IsaacLab Discussion #4124 (nov. 2025).** Un utilisateur applique
> ces **mêmes** gains `1320 / 600 / 216` (damping `72.66 / 34.64 / 29.39`, groupes
> `shoulder / elbow / wrist`, `action_scale = 0.5`) **tels quels à un UR3e** → confirme
> qu'ils servent de **défaut UR réutilisé sans ré-identification**. Le fil rapporte des
> **instabilités** (oscillations après contact) et conclut qu'**« il n'existe pas de
> règle empirique »** (le réglage dépend de la charge, des gains IK, de la masse, de la
> vitesse) → **exactement l'argument pour le system-id** (§4) plutôt que le copier-coller.
>
> Deux leviers *sim* utiles au-delà des gains : (1) **itérations du solveur PhysX**
> (position 64 / vitesse 16) pour la stabilité au contact ; (2) **raideur du joint de
> fixation** de tout ce qui est monté sur `wrist_3` — ici le **cerceau/disque** (pas une
> pince) : un montage trop souple en sim oscille. À retenir pour la config Isaac du ball-catch.

---

## 4. Méthode pour identifier les gains UR3e (system-id)

> 🛠️ **Programme détaillé** (excitation, enregistrement, fit `K/D/L`, intégration
> web UI) : `ur3e_programme_identification_gains.md`. Une implémentation locale
> existe dans `src/ur3e_sysid/` (`run_sweep`, `fit_gains`), mais le dossier n'est
> pas encore suivi par Git dans l'état courant du workspace.

Cf. `ur3e_ball_catch_sim_to_real.md §2.1` et littérature §5 :

1. **Excitation** : consignes de position **échelon** puis **sinus/chirp** par joint
   (balayage de vitesses), via `forward_position_controller` /
   `scaled_joint_trajectory_controller`.
2. **Mesure** : enregistrer `/joint_states` (le UR3e remonte ~500 Hz en RTDE ;
   commande ~125 Hz).
3. **Ajustement** : régler `stiffness`/`damping` en sim **par joint** pour matcher
   **temps de montée + dépassement** réels. Partir de la hiérarchie §3, baisser
   `stiffness` jusqu'à retrouver le retard réel.
4. **Domain randomization** : randomiser `±20–30 %` autour des valeurs trouvées
   (`ur3e_ball_catch_sim_to_real.md §3.2`).

Pour estimer `I_eff` par joint (dans `D ≈ 2√(K·I)`), utiliser les inerties du
`physical_parameters.yaml` du `ur_description` UR3e ou la doc cinématique/dynamique UR
(§5).

---

## 5. Documentation sim-to-real UR (sources)

### NVIDIA / Isaac (le plus applicable)
- **Bridging the Sim-to-Real Gap for Industrial Robotic Assembly Using Isaac Lab**
  (blog NVIDIA) — **UR10e**, PPO, **policy = delta joint position** (= espace d'action
  incrémental retenu ici) + boucle bas-niveau impédance/couple **500 Hz** côté robot.
- **Sim2Real Deployment of Policies Trained in Isaac Lab** (doc officielle).
- **Isaac ROS — UR DNN Policy tutorial** (UR10e, exemple sim2real complet).
- **Isaac Lab — actuators API** (`ImplicitActuatorCfg` : stiffness/damping → PhysX).
- **IsaacLab — Discussion #4124** (nov. 2025) — UR3e + Robotiq : config actionneur par
  groupe (`1320/600/216`), instabilité au contact, « pas de règle empirique », itérations
  solveur PhysX (64/16) + raideur du joint de fixation `wrist_3`.

### Universal Robots (specs UR3e)
- **UR3e User Manual** — `docs/Robot_Control/UR3e_Official_doc.pdf` (dans le repo,
  source officielle) : §2.1 specs (p. 17), §5 dimensionnement stand / charges embase (p. 36).
- **Technical Specifications UR3e** (vitesses 180/360°/s, outil ≈ 1 m/s).
- **Max. joint torques UR-Series** (couples par taille de joint).
- **URe-Series Cobot Kinematics & Dynamics** (Ohio Univ.) — DH + inerties (pour `I_eff`).

### Académique (gains PD & sim2real)
- **Tune to Learn: How Controller Gains Shape Robot Policy Learning** — les gains PD
  comme partie du transfert ; identification gain-spécifique (chirp).
- **Sim-to-Real for Mobile Robots: Isaac Sim → ROS 2** (arXiv 2501.02902).

---

## 6. Liens

- UR3e `joint_limits.yaml` (`Universal_Robots_ROS2_Description`) :
  <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/ros2/config/ur3e/joint_limits.yaml>
- UR3e `physical_parameters.yaml` (inerties/masses) :
  <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/ros2/config/ur3e/physical_parameters.yaml>
- `ur_description` (index ROS) : <https://index.ros.org/p/ur_description/>
- **UR3e User Manual (officiel, dans le repo)** : `docs/Robot_Control/UR3e_Official_doc.pdf`
  (§2.1 specs p. 17 ; §5 charges embase p. 36).
- UR — Technical Specifications UR3e :
  <https://www.universal-robots.com/manuals/EN/HTML/SW10_6/Content/prod-usr-man/hardware/arm_e-Series/UR3e/H_g5_sections/appendix_g5/tech_spec_data.htm>
- UR — Max. joint torques UR-Series :
  <https://www.universal-robots.com/articles/ur/robot-care-maintenance/max-joint-torques-ur-series/>
- URe-Series Cobot Kinematics & Dynamics (Ohio Univ.) :
  <https://people.ohio.edu/williams/html/PDF/UniversalRobotKinematics.pdf>
- NVIDIA Blog — Bridging the Sim-to-Real Gap (UR10e) :
  <https://developer.nvidia.com/blog/bridging-the-sim-to-real-gap-for-industrial-robotic-assembly-applications-using-nvidia-isaac-lab/>
- Isaac Lab — Sim2Real Deployment of Policies :
  <https://isaac-sim.github.io/IsaacLab/main/source/policy_deployment/index.html>
- Isaac ROS — UR DNN Policy / sim-to-real tutorial :
  <https://nvidia-isaac-ros.github.io/reference_workflows/isaac_for_manipulation/tutorials/sim_to_real/tutorial_gear_assembly.html>
- Isaac Lab — actuators API :
  <https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.actuators.html>
- IsaacLab — Discussion #4124 (UR3e gains actionneur `1320/600/216`, stabilité PhysX) :
  <https://github.com/isaac-sim/IsaacLab/discussions/4124>
- Isaac Lab — `universal_robots.py` (gabarit UR10e, à transposer) :
  <https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab_assets/isaaclab_assets/robots/universal_robots.py>
- Tune to Learn: How Controller Gains Shape Robot Policy Learning :
  <https://arxiv.org/pdf/2604.02523>
- Sim-to-Real for Mobile Robots (Isaac Sim → ROS 2), arXiv 2501.02902 :
  <https://arxiv.org/abs/2501.02902>
