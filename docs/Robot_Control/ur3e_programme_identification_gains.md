# UR3e — Programme d'identification des gains actionneur (system-id)

> Statut (2026-06-23) : **doc de conception** (build doc). Décrit comment construire
> un petit programme ROS 2 qui **excite** le vrai UR3e, **enregistre** la réponse, et
> **calcule** les paramètres actionneur Isaac Lab : `stiffness` (K), `damping` (D),
> **latence** L, **frottement** (Coulomb + visqueux), **inertie effective** I, et
> vérifie `velocity_limit` / `effort_limit`. Sortie = un `ur3e_actuator_identified.yaml`
> + plages de domain randomization. **Aucun code projet modifié par ce doc** — il
> spécifie le programme à écrire.

Documents liés :
- `ur3e_parametres_actionneur_reference.md` — §3 hiérarchie K/D, §4 méthode (ce doc en est le détail exécutable).
- `ur3e_sim2real_propositions.md` — §4.7 system-id, modélisation de latence `L_a`/`L_o`.
- `ur3e_choix_espace_action_isaac.md` — pourquoi identifier sur l'interface de déploiement.
- `ur3e_web_ui.md` — l'infra FastAPI/`RosBridge` réutilisée (§5, §7).

---

## 0. Objet

On a déjà installé : le **driver ROS 2** (`scaled_joint_trajectory_controller`,
`forward_position_controller`, `/joint_states`) et la **web UI** FastAPI
(`ur3e_web_ui`). Ce programme s'appuie dessus pour mesurer la dynamique réelle du
robot et en déduire les paramètres à injecter en sim, afin de **fermer le gap
sim-to-real** au niveau actionneur (remplacer le `stiffness=800, damping=40` générique,
cf. réf §3).

---

## 1. Ce qu'on identifie — et ce qu'on n'identifie PAS

⚠️ Point clé : le vrai UR3e **ne tourne pas** un PD `K/D` qu'on peut régler — c'est une
baie fermée avec son propre contrôleur haute bande passante. On n'identifie donc **pas
les vrais gains internes**, mais des **gains effectifs (surrogate)** tels que la
**réponse boucle-fermée `commande de position → position articulaire` de la sim** soit
identique à celle du vrai robot. Pour le transfert de policy, c'est exactement la bonne
cible : la policy ne voit que cette dynamique « commande → mouvement ».

| Cible | But | Méthode | Requis pour la policy ? |
|---|---|---|---|
| **T1 — comportementale** (principal) | matcher la réponse `commande→position` | FRF échelon + chirp → `(ωn, ζ, L)` → `(K, D)` via `I_eff` | **Oui** |
| **T2 — physique** (optionnel) | vrais frottement / inertie / couples | moindres carrés sur le couple `/joint_states.effort` | Non (utile pour DR/frottement) |

**Corollaire (à ne pas rater) — identifier sur l'interface de DÉPLOIEMENT.** Le
live-catch streame en **position à 60 Hz via `forward_position_controller`**
(`streaming.py`). L'interpolation interne du driver entre deux consignes fait partie de
la dynamique que verra la policy. → Le **chirp décisif doit passer par
`forward_position_controller` à 60 Hz**, pas seulement par le trajectory controller.
(C'est le prolongement de la discussion « formule d'action » : sim et réel doivent
partager le même chemin action→cible→mouvement.)

---

## 2. Architecture du programme (6 blocs)

```
[1 Générateur]   [2 Commandeur]            [3 Enregistreur]
 échelon/chirp -> trajectory action  ----\
 rampe vitesse   OU fwd_pos 60 Hz     ----+--> /joint_states (q, q̇, effort)
                                          \-> commande horodatée
                                                   |
                                                   v
                            [4 Estimateur]  fit (ωn, ζ, L), frottement, I_eff
                                                   |
                       [5 Validation]  rejeu Isaac, superposition, RMSE
                                                   |
                       [6 Sortie]  ur3e_actuator_identified.yaml + plages DR
```

---

## 3. Signaux d'excitation (une articulation à la fois)

| Signal | Paramètres typiques | Ce qu'il donne |
|---|---|---|
| **Échelon** | amplitude 0.05–0.20 rad, plusieurs amplitudes | temps de montée, **dépassement**, temps d'établissement → `(ωn, ζ)` |
| **Chirp** (sinus balayé) | f : 0.1 → ~10–15 Hz, amplitude 0.02–0.10 rad | **FRF**, bande passante −3 dB, phase → `(ωn, ζ, L)` |
| **Rampe vitesse** | vitesses 0.05 → `v_safe`, par paliers | **frottement** (Coulomb + visqueux) depuis le couple stationnaire |
| (option) PRBS / multisine | bande large, faible amplitude | excitation riche pour `least_squares` |

Toutes les excitations sont **centrées sur une pose sûre mi-course**, **loin des
butées** (rappel : coude `±π`), des **singularités** et de l'**auto-collision**, à
**charge/outil fixés et documentés** (les gains en dépendent), avec le **scaling
vitesse réduit** au pendant.

---

## 4. Les maths (bloc 4 — l'estimateur)

### 4.1 Modèle 2nd ordre (par articulation, petit mouvement, gravité localement constante)

`I·q̈ = K·(q_cmd − q) − D·q̇`  ⇒  `q̈ + (D/I)·q̇ + (K/I)·q = (K/I)·q_cmd`

- `ωn = √(K/I)`  ·  `ζ = D / (2·√(K·I))`
- **Inversion** (ce qu'on injecte en sim) : `K = I·ωn²`  ·  `D = 2·ζ·ωn·I = 2·ζ·√(K·I)`

### 4.2 Depuis l'échelon

- dépassement `Mp` → `ζ = −ln(Mp) / √(π² + ln²(Mp))`
- temps de pic `t_p` → `ωn = π / (t_p·√(1 − ζ²))`
(ou via temps de montée / d'établissement). Nécessite `I_eff` (§4.5).

### 4.3 Depuis le chirp (recommandé)

FRF `H(jω) = Θ(jω)/Θ_cmd(jω)` via `scipy.signal.csd` / `welch`
(`tfestimate = Sxy / Sxx`). Extraire la **bande passante −3 dB** et la **phase**, puis
ajuster un **2nd ordre + retard pur** `H(jω) = ωn² / (−ω² + 2ζωn·jω + ωn²) · e^{−jωL}`
par moindres carrés (`scipy.optimize.least_squares` sur `|H|` et `arg H`) →
`(ωn, ζ, L)`. Le **retard L** alimente la modélisation de latence (`L_a`/`L_o`,
propositions §4.7).

### 4.4 Moindres carrés direct K/D (option, si le couple est fiable)

`τ_meas = K·(q_cmd − q) − D·q̇`  →  empiler les échantillons
`[ (q_cmd−q) , −q̇ ] · [K ; D] = τ_meas`  → LS linéaire → `K, D` **sans `I_eff`**.
⚠️ `/joint_states.effort` du UR = **estimation** de couple (courant moteur), incluant
gravité + frottement → **compenser la gravité** (modèle au point `q`) d'abord, sinon
rester sur 4.3. C'est la cible T2 (couple), pas le surrogate comportemental.

### 4.5 Inertie effective `I_eff`

(a) depuis `physical_parameters.yaml` du `ur_description` UR3e → matrice d'inertie
articulaire `M(q)`, terme diagonal `i,i` **à la pose d'identification** (CRBA via
Pinocchio ou KDL) ; (b) ou depuis l'asymptote haute fréquence de la FRF.
**Config-dépendant** → documenter la pose dans la sortie.

### 4.6 Frottement

Rampes à vitesse constante : tracer `τ_stationnaire` vs `q̇` → **ordonnée à l'origine =
Coulomb `Fc`**, **pente = visqueux `Fv`**, près de 0 = **stiction**. → DR + éventuel
terme de frottement actionneur.

### 4.7 Latence `L`

Du **retard de phase** de la FRF (pente `arg H` vs `ω`) ou de la **corrélation croisée**
commande/réponse. Séparer la latence de transport (réseau/driver) de la dynamique.

---

## 5. Réutiliser l'infra déjà installée (ne rien réinventer)

| Brique existante | Où | Usage system-id |
|---|---|---|
| `build_joint_trajectory` + `DEFAULT_JOINT_NAMES` | `ur3e_rollout_replay.send` / `.replay_core` | construire les trajectoires **échelon / rampe** |
| ActionClient `FollowJointTrajectory` | `ros_interface.py` (`ACTION_NAME = /scaled_joint_trajectory_controller/follow_joint_trajectory`) | envoyer la batterie échelon/rampe proprement |
| `switch_controller` / `list_controllers` | `ros_interface.py`, aussi `live_catch_node.py` | basculer vers `forward_position_controller` pour le chirp |
| Streaming 60 Hz `forward_position_controller` | `ur3e_live_catch/streaming.py`, topic `/forward_position_controller/commands` | **chirp sur le chemin de déploiement** |
| Clamp limites + gates moteur | `app.py` (`_clamp_to_limit`, `_ensure_joint_target_within_limits`, `ensure_motion_enabled`) | sécurité d'excitation — **réutiliser, ne pas contourner** |
| Pattern service single-shot | `test_ball_node` `~/throw` (`std_srvs/Trigger`) | exposer un `~/run_sweep` analogue |
| Flux d'état | web UI `/ws` (15 Hz) | trop lent pour le fit ⇒ **souscrire `/joint_states` directement** (à la **fréquence système 500 Hz**, officiel, *UR3e User Manual* §2.1) |
| Dashboard (play/stop/e-stop) | `ros_interface.py` `DASHBOARD_COMMANDS` | arrêt d'urgence intégré |

---

## 6. Forme du programme (2 options)

- **Option A — nœud dédié `ur3e_sysid`** (nouveau package, ou dans `ur3e_live_catch`),
  miroir de `test_ball_node` : paramètres (`joint`, `signal`, `amplitude`, `f0`, `f1`,
  `duration`), service `~/run_sweep`, publie `sysid_telemetry`, écrit **rosbag2 + CSV**,
  puis un script offline `fit_gains.py` (numpy/scipy).
- **Option B — script autonome `tools/sysid/`** (rclpy minimal) : commande + record +
  fit en une passe. Plus simple pour démarrer.

**Reco : commencer par l'Option B** (rapide, ne touche pas au code des nœuds), puis
industrialiser en nœud + onglet web (§7) une fois la méthode validée sur mock hardware.

### 6.1 Pseudo-code (end-to-end, Option B)

Deux étages : `run_sweep` (en ligne — excite + enregistre) puis `fit_gains` (hors
ligne — ajuste et écrit le YAML). Les maths sont celles de §4. *Pseudo-code* :
décomposition logique, pas du code à copier tel quel.

```python
# ── run_sweep : excitation + enregistrement (robot réel ou mock) ────────────────
JOINTS         = ["shoulder_pan","shoulder_lift","elbow","wrist_1","wrist_2","wrist_3"]
EFFORT_LIMIT   = [56, 56, 28, 12, 12, 12]            # Nm  (datasheet, vérifié)
VELOCITY_LIMIT = [PI, PI, PI, 2*PI, 2*PI, 2*PI]      # rad/s
ID_POSE        = [0, -PI/2, PI/2, -PI/2, -PI/2, 0]   # pose mi-course, loin des butées

def run_sweep(joint, signal, p):                     # p = paramètres du signal
    assert motion_enabled()                          # gate moteur (cf. ensure_motion_enabled)
    assert excitation_within_limits(joint, signal, p, margin=0.1)   # ⊂ joint_limits

    goto(ID_POSE)                                    # via trajectory action
    rec = Recorder(["/joint_states"], extra=command_stream)         # horodatage monotone
    rec.start()
    if signal in ("step", "ramp"):
        traj = build_signal_trajectory(joint, signal, p)    # s'appuie sur build_joint_trajectory
        switch_controller(to="scaled_joint_trajectory_controller")
        send_trajectory(traj); wait_until_done()
    elif signal == "chirp":                          # CHEMIN DE DÉPLOIEMENT (§1)
        switch_controller(to="forward_position_controller")
        q0 = current_q(joint)
        for t in clock(rate=60):                     # 60 Hz, comme streaming.py
            if t > p.duration: break
            f  = p.f0 + (p.f1 - p.f0) * t / p.duration              # balayage linéaire
            ph = 2*PI * (p.f0*t + 0.5*(p.f1 - p.f0)*t*t/p.duration) # phase = ∫ 2πf dt
            publish_forward_command(joint, q0 + p.amplitude * sin(ph))
    rec.stop(); controlled_stop()
    rec.save_csv(f"{joint}_{signal}.csv", fields=["t","q_cmd","q","qd","effort",
                                                   "f0","f1"])
```

```python
# ── fit_gains : estimateur hors ligne → ur3e_actuator_identified.yaml ───────────
def fit_gains():
    I = effective_inertia(URDF, ID_POSE)             # CRBA (Pinocchio) → I[j] diagonale
    out = {}
    for j in JOINTS:
        wn_s, ze_s, r2_s    = fit_step (load(j, "step"))
        wn_c, ze_c, L, r2_c = fit_chirp(load(j, "chirp"))
        wn, ze = reconcile(wn_c, ze_c, wn_s, ze_s)   # chirp prioritaire, échelon en croisé
        K = I[j] * wn**2                             # raideur effective   (§4.1)
        D = 2 * ze * wn * I[j]                       # amortissement eff.  (§4.1)
        Fc, Fv = fit_friction(load_all(j, "ramp"))
        out[j] = dict(stiffness=K, damping=D, latency_s=L, inertia_eff=I[j],
                      friction_coulomb=Fc, friction_viscous=Fv,
                      effort_limit=EFFORT_LIMIT[j], velocity_limit=VELOCITY_LIMIT[j],
                      fit_r2_step=r2_s, fit_r2_chirp=r2_c)
        assert r2_c >= 0.95, f"{j}: chirp mal ajusté ({r2_c:.2f})"   # critère §12
    write_yaml("ur3e_actuator_identified.yaml", meta=run_metadata(), joints=out,
               domain_randomization=dict(stiffness_pct=0.25, damping_pct=0.25))

def fit_step(log):                                   # échelon → (wn, ζ)        (§4.2)
    A = log.q_cmd[-1] - log.q_cmd[0]
    y = (log.q - log.q[0]) / A                       # réponse normalisée 0 → 1
    if max(y) <= 1.0 + EPS:                          # pas de dépassement → sur-amorti
        return fit_overdamped(log)                   # ajuste sur le temps de montée 10–90 %
    Mp     = max(y) - 1.0                             # dépassement relatif
    t_peak = time_at(argmax(y))
    ze = -ln(Mp) / sqrt(PI**2 + ln(Mp)**2)
    wn =  PI / (t_peak * sqrt(1 - ze**2))
    r2 = r_squared(y, second_order_step(wn, ze, log.t))
    return wn, ze, r2

def fit_chirp(log):                                  # FRF → (wn, ζ, latence)   (§4.3)
    fs = 1 / mean_diff(log.t)
    f, Suu = welch(log.q_cmd, fs);  _, Suy = csd(log.q_cmd, log.q, fs)
    H = Suy / Suu                                     # tfestimate = Suy / Suu
    w = 2*PI*f;  band = (f >= log.f0) & (f <= log.f1) # bande réellement excitée

    def model(th, w):                                # 2nd ordre + retard pur
        wn, ze, L = th
        return wn**2 / (wn**2 - w**2 + 2*ze*wn*1j*w) * exp(-1j*w*L)

    def resid(th):                                   # moindres carrés complexes
        e = model(th, w[band]) - H[band]
        return concat(real(e), imag(e))

    th0 = [bandwidth_3dB(f, H), 0.7, phase_slope_delay(f, H)]       # init
    wn, ze, L = least_squares(resid, th0, bounds=([0, 0, 0], [INF, 2, 0.1])).x
    r2 = frf_r_squared(H[band], model([wn, ze, L], w[band]))
    return wn, ze, L, r2

def fit_friction(ramps):                             # rampes → Coulomb + visqueux (§4.6)
    pts = []
    for r in ramps:
        seg = constant_velocity_segment(r)           # portion à q̇ ≈ cste
        v   = median(seg.qd)
        tau = median(seg.effort - gravity_torque(seg.q))   # gravité ôtée
        pts.append((v, tau))
    Fc, Fv = fit_line(abs_v=pts.v, tau=pts.tau)      # τ = Fc·sign(v) + Fv·v
    return Fc, Fv

def reconcile(wn_c, ze_c, wn_s, ze_s):               # consolidation chirp / échelon
    if rel_gap(wn_c, wn_s) > 0.2 or rel_gap(ze_c, ze_s) > 0.3:
        warn("step vs chirp incohérents → vérifier linéarité / amplitude")
    return wn_c, ze_c                                # chirp = large bande → prioritaire
```

---

## 7. Intégration web UI (onglet « System-ID »)

Comme l'onglet **Test** pilote `~/throw` via `POST /api/catch/throw`, ajouter :

- **Endpoints FastAPI** : `POST /api/sysid/run {joint, signal, amplitude, f0, f1, ...}`,
  `GET /api/sysid/result/{joint}`, et pousser **commande vs réponse** sur le websocket
  pour un **tracé live**.
- **UI** : sélecteur d'articulation, type de signal, sliders amplitude/fréquence,
  **bouton e-stop bien visible**, courbe commande/réponse en direct, affichage des
  `K / D / L` ajustés **avec le R²**.
- **Sécurité** : réutiliser `ensure_motion_enabled`, la confirmation explicite et le
  dashboard Play — **mêmes gates que les autres mouvements**.
- Garder le **fit lourd côté backend** (scipy) ; l'UI n'affiche que les résultats.

---

## 8. Stack technique

`rclpy` ; `numpy`, `scipy` (`signal.welch`/`csd`, `optimize.least_squares`/`curve_fit`) ;
`python-control` (option : Bode/tf) ; `pandas` ; `matplotlib` (plots offline) ;
`rosbag2` (`ros2 bag record /joint_states <commande>`). `pinocchio` ou `KDL` pour
`I_eff` (CRBA). Tout est déjà en Python dans le repo.

---

## 9. Sécurité (⚠️ ça bouge le VRAI robot)

- **Une articulation à la fois**, faibles amplitudes, pose mi-course, **scaling vitesse
  réduit** au pendant.
- Vérifier que **toute l'excitation ⊂ `joint_limits` avec marge** (coude `±π`), loin des
  singularités / auto-collision.
- **E-stop matériel** à portée ; détecter le **protective stop** → abort + log ;
  watchdog par tick.
- **Charge / outil FIXES et documentés** (les gains dépendent de la configuration).
- **Montage rigide** : un support compliant ajoute un **mode basse fréquence** qui
  pollue la FRF articulaire (le *UR3e User Manual* exige une résonance de stand
  **≥ 45 Hz**). Identifier sur le **montage final**, pas sur un bâti provisoire.
- Réutiliser les **gates web** et les **clamps de limites** — défense en profondeur.
- **Roder d'abord en mock** (`use_fake_hardware:=true`) pour valider toute la chaîne
  commande→record→fit **sans robot**.

---

## 10. Procédure pas à pas

1. Driver + controllers up (`ur3e_stack`, ou mock `use_fake_hardware:=true` d'abord).
2. Aller à une **pose d'identification sûre** (Move Home ou pose dédiée).
3. Par articulation : **échelon** (via trajectory action) → **chirp** (via
   `forward_position_controller` 60 Hz) → **rampes vitesse**. Enregistrer `/joint_states`
   + la **commande**, horodatés.
4. Offline : fit `(ωn, ζ, L)` par joint avec R² ; frottement depuis les rampes ;
   `I_eff` depuis l'URDF.
5. **Valider** : rejouer la **même** excitation en Isaac avec `(K, D, L)` identifiés,
   superposer, calculer le RMSE ; **cross-validation** (fit sur échelon → valider sur
   chirp).
6. Émettre le **YAML** + plages **DR** (±20–30 %) ; brancher dans la config Isaac ;
   mettre à jour la **table §3 du doc de référence** avec les vrais nombres.

---

## 11. Format de sortie (exemple)

```yaml
# ur3e_actuator_identified.yaml — produit par le system-id
meta:
  robot_serial: "<série>"
  date: "2026-06-23"
  id_pose_rad: [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
  payload_kg: 0.0
  interface: "forward_position_controller@60Hz"   # chemin de déploiement
joints:
  shoulder_pan_joint:
    stiffness: 0.0        # K = I_eff * wn^2
    damping: 0.0          # D = 2*zeta*wn*I_eff
    effort_limit: 56.0    # vérifié, datasheet UR3e
    velocity_limit: 3.1416
    latency_s: 0.0        # L (retard FRF)
    friction_coulomb_Nm: 0.0
    friction_viscous_Nms: 0.0
    inertia_eff: 0.0      # I_eff à id_pose
    fit_r2_step: 0.0
    fit_r2_chirp: 0.0
  # ... 5 autres joints
domain_randomization:
  stiffness_pct: 0.25
  damping_pct: 0.25
  latency_s_range: [0.0, 0.0]
```

Branchement Isaac : `ImplicitActuatorCfg(stiffness=…, damping=…, effort_limit_sim=…,
velocity_limit_sim=…)` **par groupe/joint** (cf. réf §3 pour la hiérarchie) ; `L` →
modèle de latence ; frottement → DR.

---

## 12. Validation / critères d'acceptation

- **R²** échelon et chirp **≥ ~0.95** par joint.
- **RMSE sim↔réel** sur la même excitation < seuil (ex. quelques % de l'amplitude).
- **Cross-validation** cohérente (fit échelon → prédit chirp).
- **Hiérarchie** `K_épaule > K_coude > K_poignet` retrouvée (sanity, réf §3).
- **Bande passante réaliste** (et non le suivi quasi instantané du `800/40` générique).

---

## 13. Liens

- `ur3e_parametres_actionneur_reference.md` §3 (hiérarchie K/D) & §4 (méthode résumée).
- `ur3e_sim2real_propositions.md` §4.7 (system-id) + modélisation de latence.
- `ur3e_web_ui.md` (infra FastAPI / `RosBridge` réutilisée).
- UR3e `physical_parameters.yaml` (inerties pour `I_eff`) :
  <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/ros2/config/ur3e/physical_parameters.yaml>
- Pinocchio (CRBA, `M(q)`) : <https://github.com/stack-of-tasks/pinocchio>
- SciPy — estimation FRF (`signal.csd`/`welch`) & ajustement (`optimize.least_squares`) :
  <https://docs.scipy.org/doc/scipy/reference/signal.html>
- python-control (Bode / identification de fonction de transfert) :
  <https://python-control.readthedocs.io/>
- Tune to Learn: How Controller Gains Shape Robot Policy Learning (chirp / gains) :
  <https://arxiv.org/pdf/2604.02523>
