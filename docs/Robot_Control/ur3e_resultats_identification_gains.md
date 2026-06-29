# UR3e — Résultats d'identification des gains actionneur (system-id)

> Statut (2026-06-25) : **document de résultats**. Valeurs `stiffness` (K),
> `damping` (D), latence, inertie effective et amortissement réduit **mesurées sur le
> vrai UR3e** par le programme `ur3e_sysid` (`run_sweep` → `fit_gains`), avec leurs
> conditions de mesure, leur validation FRF et leurs réserves. Remplace les
> placeholders « à identifier » de `ur3e_parametres_actionneur_reference.md` §3.
> Sortie machine : `ur3e_actuator_identified.yaml` (racine du workspace).

Documents liés :
- `ur3e_programme_identification_gains.md` — le programme/méthode (excitation, fit, maths §4).
- `ur3e_parametres_actionneur_reference.md` — §3 hiérarchie K/D (ce doc fournit les vrais nombres).
- `ur3e_sim2real_propositions.md` — §4.3 latence, §4.7 system-id.
- `ur3e_choix_espace_action_isaac.md` — pourquoi identifier sur `forward_position_controller`.

---

## 1. Conditions de mesure

| Élément | Valeur |
|---|---|
| Date | 2026-06-25 |
| Robot | UR3e (192.168.0.5), PolyScope 5.12.4, driver `ros-humble-ur` 2.13.0 |
| Pose d'identification | `[0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]` rad (mi-course, loin des butées) |
| **Charge / outil** | **0 kg — bras nu, sans cerceau** (voir réserve §5.1) |
| Interface du chirp | `forward_position_controller` @ **60 Hz** (= chemin de déploiement live) |
| Interface du step | `scaled_joint_trajectory_controller` (action trajectoire) |
| Cadence d'état `/joint_states` | ~**500 Hz** (mesurée depuis les timestamps, RTDE) |
| Chirp | `f0=1.0 → f1=15 Hz`, amplitude `A=0.01 rad`, durée `30 s` |
| Step | amplitude `0.03 rad` |

> Le chirp **doit** passer par `forward_position_controller` à 60 Hz car c'est l'interface
> que la policy live utilise : on identifie la dynamique « commande → position » réellement
> vue au déploiement. La bande `0.1–3 Hz` initialement testée était **trop basse** (le robot
> suit à l'unité, FRF plate → K aberrant `17078`, ζ saturé à 2,0) ; `1–15 Hz` capte la
> coupure −3 dB et rend le fit bien conditionné.

---

## 2. Valeurs identifiées (les 6 articulations)

| Joint | **K** `stiffness` | **D** `damping` | ωn (Hz) | ζ | latence (ms) | I_eff (kg·m²) | R²_chirp |
|---|---:|---:|---:|---:|---:|---:|---:|
| `shoulder_pan`  |  **607,5** | 18,60 |  7,5 | 0,72 | 12,8 | 0,2760 | 0,997 |
| `shoulder_lift` | **2883,5** | 71,56 | 12,5 | 0,98 | 22,4 | 0,4670 | 0,990 |
| `elbow`         |  **669,5** | 18,77 |  9,4 | 0,83 | 17,6 | 0,1906 | 0,998 |
| `wrist_1`       |   **24,70**|  0,740 |  7,7 | 0,72 | 13,7 | 1,069e-2 | 0,999 |
| `wrist_2`       |    **4,93**|  0,144 |  7,7 | 0,70 | 15,1 | 2,130e-3 | 0,999 |
| `wrist_3`       |    **0,32**|  0,0096|  7,7 | 0,72 | 13,9 | 1,371e-4 | 0,999 |

Relations utilisées (doc programme §4.1) : `K = I_eff·ωn²`, `D = 2·ζ·ωn·I_eff`.
Limites recoupées (réf §1) : effort `[56,56,28,12,12,12]` Nm, vitesse `[π,π,π,2π,2π,2π]` rad/s.
Frottement (Coulomb/visqueux) **non identifié** (rampes non faites) → laissé à 0.

---

## 3. Validation

Contrôle par `scripts/sysid_frf_check.py` (gain mesuré `std(q)/std(q_cmd)` par bande vs
module du modèle 2ⁿᵈ ordre + retard ajusté) :

- **Coupure −3 dB captée dans la bande** pour les 6 joints (~7,5–8,5 Hz mesurés) → ωn
  réellement **observé**, plus extrapolé.
- **Le modèle ajusté colle à la mesure** sur 1–15 Hz (écarts de quelques % par bande).
- `ζ` **à l'intérieur** des bornes (0,70–0,98), R²_chirp **0,99–0,999**, R²_step ≥ 0,98.
- **Hiérarchie `K_épaule > K_coude > K_poignet` respectée** (sanity réf §3).

Exemple (coude) : gain 1,0 → 0,37 sur 1,5→14,5 Hz, −3 dB à ~8,5 Hz, `wn_fit`=9,4 Hz.

---

## 4. Interprétation physique (à retenir)

**Toutes les articulations partagent ~la même boucle fermée** : bande passante **~8 Hz**
(7,5–12,5) et **latence ~13–22 ms**. C'est attendu : la dynamique « commande → position »
est dominée par le **servo interne UR + le streaming 60 Hz**, communs à tous les joints, et
non par la mécanique propre de chaque axe.

C'est `I_eff` qui fait ensuite **exploser l'échelle de K** (`K = I_eff·ωn²`) : épaule_lift
(forte inertie + gravité) ≈ 2884, épaule_pan/coude ≈ 600–670, poignets de 25 à 0,3 (inertie
outboard minuscule à la pose d'ID, **sans cerceau**).

→ **Le paramètre sim‑to‑real dominant ici est la latence (~15 ms)**, pas une raideur finie
précise. En sim, modéliser : K/D par joint (ci‑dessus) **+ retard ~15 ms + ZOH 60 Hz**.

---

## 5. Réserves / limites

### 5.1 ⚠️ Poignets identifiés à vide (sans le cerceau)
Mesure faite à `payload = 0`. Le **cerceau/disque du ball‑catch se monte sur `wrist_3`** →
l'inertie effective des poignets (donc leur `K`, surtout `wrist_3` = 0,32) est
**non représentative** de la configuration de capture. **Réidentifier au moins les poignets
avec le cerceau monté et l'inertie outil réglée sur le contrôleur** avant déploiement
(doc programme §9). Épaules/coude quasi inchangés (cerceau léger devant leur inertie).

### 5.2 `shoulder_lift` = le fit le moins net
`step` ↔ `chirp` divergent de 51 % sur ωn (joint le plus chargé en gravité à la pose d'ID),
courbe de gain un peu irrégulière 9–12 Hz. R²=0,990, utilisable, mais à resserrer si besoin
(re‑sweep, éventuellement amplitude/pose ajustées). *(Le désaccord step↔chirp est en partie
normal : le step passe par le `scaled_joint_trajectory_controller`, le chirp par le
`forward_position_controller` — chemin de déploiement, prioritaire.)*

### 5.3 Ce sont des valeurs d'INITIALISATION (doc programme §12)
`K/D/L` sont un **point de départ**. Étape finale : **rejouer ces chirps en sim Isaac** avec
`(K, D, L)` et **optimiser** pour **minimiser le RMSE réel↔sim**, puis re‑vérifier la
hiérarchie et les bornes physiques.

---

## 6. Branchement Isaac Lab

Par joint, dans `ur_gripper.py` (`ImplicitActuatorCfg`) :
```python
ImplicitActuatorCfg(
    stiffness=K,            # colonne §2
    damping=D,              # colonne §2
    effort_limit_sim=[56, 56, 28, 12, 12, 12],
    velocity_limit_sim=[3.1416, 3.1416, 3.1416, 6.2832, 6.2832, 6.2832],
)
```
- Latence `L` (~15 ms) → modèle de latence d'action (`ur3e_sim2real_propositions.md` §4.3).
- Domain randomization : `±25 %` sur `stiffness`/`damping` (présent dans le YAML).
- **Ne pas** garder l'ancien `stiffness=800 / damping=40` uniforme générique.

---

## 7. Reproduire / fichiers source

Pré-requis : driver up (`ur3e_stack`), python **système** (scipy), `forward_position_controller`
spawné. Identification (E‑stop en main, une articulation à la fois) :
```bash
# par joint : chirp (chemin de déploiement) puis step
ros2 run ur3e_sysid run_sweep --joint <joint> --signal chirp --f0 1.0 --f1 15 --amplitude 0.01 --duration 30
ros2 run ur3e_sysid run_sweep --joint <joint> --signal step  --amplitude 0.03
# fit global -> YAML
ros2 run ur3e_sysid fit_gains --in-dir recordings/sysid --out ur3e_actuator_identified.yaml
# validation FRF (gain mesuré vs modèle, -3 dB)
python3 scripts/sysid_frf_check.py
```
Si un joint donne `ζ≈2,0`, un `K` énorme, ou un gros warning `reconcile` → bande trop basse,
relancer son chirp avec `--f1 20` (rester < Nyquist 30 Hz du streaming 60 Hz).

Artefacts :
- `recordings/sysid/<joint>_{chirp,step}.csv` (+ `.meta.json`) — données brutes horodatées.
- `ur3e_actuator_identified.yaml` — gains + métadonnées + plages DR.
- `scripts/sysid_frf_check.py` — contrôle FRF par joint.
