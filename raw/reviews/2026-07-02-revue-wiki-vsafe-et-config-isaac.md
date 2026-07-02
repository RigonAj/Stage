# Revue complète : wiki Stage, déploiement v_safe_scale et configuration d'entraînement Isaac

> Source: Revue par agent (Claude, session Claude Code remote), 2026-07-02
> Collected: 2026-07-02
> Published: 2026-07-02

Ce document compile une revue en trois volets, destinée à être ingérée dans le
wiki LLM (`wiki/`) et à servir de cahier des charges pour les changements de
code dans le repo Isaac `6-Dof-Ur3e-Catch-a-ball` et dans `src/ur3e_live_catch/`.

Méthode : lecture de `CLAUDE.md`/`AGENTS.md`, des 18 articles de `wiki/`, de
`wiki/log.md`, exécution de `scripts/lint_llm_wiki.py` (passe sans erreur),
lecture de `data/models/latest/policy_metadata.json`,
`src/ur3e_live_catch/ur3e_live_catch/live_catch_node.py`, et côté Isaac de
`firsttraining_env_cfg.py`, `firsttraining_env.py` (reward et
`_pre_physics_step`) et `agents/skrl_ppo_cfg.yaml`.

---

## Volet 1 — Revue du wiki LLM (repo Stage)

### 1.1 Ce qui fonctionne (à conserver tel quel)

- Structure conforme au contrat : un niveau `wiki/<topic>/<article>.md`,
  métadonnées `Sources:`/`Raw:` partout, liens `See Also`, lint déterministe
  qui passe.
- La règle de découpage par cadence de changement est respectée : pages
  d'architecture stables séparées de
  `wiki/live-catch/current-status-and-blockers.md` qui churn.
- `wiki/log.md` est discipliné (~20 entrées en 4 jours, cascades documentées).
- Le diagnostic de l'incident pendant du 2026-07-02 (wrap ±2π sur wrist_3) est
  documenté de façon exemplaire : cause racine, forensics, fixes, action
  opérateur.

### 1.2 Corrections factuelles à faire dans le wiki

1. **Stale claim `ur3e_sysid`** :
   `wiki/live-catch/current-status-and-blockers.md` (section
   « Documentation/Reproducibility Gaps ») affirme que `src/ur3e_sysid/` est
   « present locally but untracked in the current worktree ». C'est faux :
   `git ls-files` montre que le package est tracké. Supprimer ou corriger la
   phrase.
2. **Contradiction wiki ↔ cfg Isaac** : le wiki
   (`current-status-and-blockers.md`,
   `sim-to-real/policy-transfer-and-action-semantics.md`) affirme que le cfg
   Isaac `FirstTraining` du 02/07 « halves joint_velocity_safe /
   joint_acceleration_safe and keeps joints within ±π ». Le checkout actuel de
   `6-Dof-Ur3e-Catch-a-ball` (`firsttraining_env_cfg.py`, dernier commit
   « WIP: archive current Isaac training changes ») contient encore les
   **limites pleines** : `joint_velocity_safe_rad_s = (π, π, π, 2π, 2π, 2π)`,
   accélérations pleines, bornes de position ±2π (coude ±π). Soit le
   changement n'existe que sur une machine locale non commitée, soit il a été
   perdu. À trancher, puis aligner wiki et cfg. Voir Volet 3, action B4.

### 1.3 Lacunes structurelles du wiki (pages à créer)

1. **`wiki/sim-to-real/isaac-training-environment.md`** (priorité haute).
   Le trou principal : les pages sim-to-real citent constamment « Isaac
   FirstTraining » mais aucune page compilée ne décrit l'environnement
   d'entraînement. Contenu attendu :
   - définition de l'observation 33-D côté Isaac et ordre des composantes ;
   - structure du reward (`compute_rewards` : `rew_dist = exp(-2d) - d`,
     pénalité d'action per-joint avec warmup, `+400` pass, `-100`
     termination) ;
   - terminaisons (`hit_arm`, `ball_on_ground`, `reset_on_success=False`,
     respawn de balle tenue au centre du disque) ;
   - commandes train/play/evaluate/export (elles sont dans le README Isaac et
     partiellement dans `wiki/operations/testing-and-commands.md`) ;
   - résultat de référence : ~98 % de réussite en éval headless
     (`ball_position_noise_std=0.05` d'après le README ; noter que le cfg
     actuel dit 0.01 — voir Volet 3, B6) ;
   - **procédure de synchro cross-repo** : quel export part dans
     `data/models/`, comment vérifier la parité des métadonnées
     (`policy_metadata.json`) avant déploiement.
2. **`wiki/operations/real-robot-bringup-runbook.md`** (priorité haute).
   Runbook opérateur durable, extrait de la page de statut qui churn :
   - checklist avant `enable_command` (controller actif, TF hoop présent,
     `/joint_states` cohérent avec le pendant, zone dégagée, E-stop en main) ;
   - procédure quand la gate start-pose ±2π déclenche : jog/unwind du poignet
     ou reboot du bras jusqu'à ce que `/joint_states` corresponde au pendant ;
   - procédure de montée en vitesse `v_safe_scale` par paliers (voir Volet 2) ;
   - paramètres à surveiller : tracking error, protective stops, latence.
3. **Plan de mesure de latence** (page ou section dans
   `wiki/sim-to-real/observation-latency-and-models.md`) : le wiki identifie
   la latence comme risque n°1 mais ne décrit pas *comment* mesurer p50/p95/p99
   bout-en-bout avec la perception réelle (`latency_report`,
   `CatchTelemetry.perception_age_s`, procédure de collecte et seuils
   d'acceptation).

### 1.4 Maintenance du contrat et des couches

1. **`AGENTS.md` « How to Start » incomplet** : liste 15 pages, le wiki en a
   18. Manquent `wiki/overview/project-overview.md`,
   `wiki/overview/repository-map.md`, `wiki/operations/wiki-maintenance.md`.
   Recommandation : remplacer la liste dupliquée par un simple renvoi vers
   `wiki/index.md` (supprime la double maintenance).
2. **`docs/Agent_Wiki/` (couche secondaire) duplique du contenu de statut** :
   son `Current_Status.md` est maintenu en parallèle du wiki primaire (double
   coût, divergence garantie). Recommandation : le réduire à un index de
   navigation Obsidian pointant vers `wiki/`, sans contenu propre.
3. **Provenance cross-repo** : les articles wiki affirment des faits Isaac
   (ordre `_get_dones()`, limites cfg, sémantique d'action) avec des `Raw:`
   pointant uniquement vers des docs Stage. Deux options : mirrorer les docs
   Isaac clés (`docs/environment_and_frames.md`, `docs/sim2real_v1.md`,
   README) dans `raw/isaac/` selon la convention d'ingestion, ou étendre la
   convention `Raw:` pour autoriser une référence au repo sibling. `raw/` ne
   contient aujourd'hui que son README : le mécanisme n'a jamais servi.
4. À terme : archiver les incidents datés (incident pendant) vers une page
   d'archive quand ils ne seront plus opérationnels, pour éviter que
   `current-status-and-blockers.md` devienne un dépotoir.

---

## Volet 2 — Déploiement : `v_safe_scale` 0.5 → 1.0 sans réentraîner

### 2.1 Conclusion

**Passer à `v_safe_scale=1.0` ne nécessite pas de réentraînement — c'est même
le réglage le plus fidèle à l'entraînement.** Le réentraînement (Volet 3) est
une amélioration de qualité, pas un prérequis.

### 2.2 Justification (vérifiée dans le code et les métadonnées)

- La policy actuelle a été entraînée avec
  `joint_velocity_safe_rad_s = [π, π, π, 2π, 2π, 2π]` — exactement ce que
  porte `data/models/latest/policy_metadata.json`. À `v_safe_scale=1.0`,
  l'`ActionMapper` reproduit exactement le contrat d'action vu à
  l'entraînement.
- Dans `live_catch_node.py` (`_bounds_from_metadata`), `v_safe_scale` scale
  les bornes qui alimentent **à la fois** l'`ActionMapper` et le
  `SafetyLimiter`. À 0.5, chaque pas d'intégration fait la moitié de ce que la
  policy attend et l'accélération est aussi divisée par deux (double
  ralentissement). Le nœud logge lui-même : « closed-loop dynamics diverge
  from training ». La lenteur observée sur le robot réel est ce bridage, pas
  la policy.
- L'incident pendant du 02/07 était un wrap ±2π (corrigé par la gate
  start-pose), pas un problème de vitesse. Rien n'interdit 1.0.

### 2.3 Précautions obligatoires à 1.0

La policy sature ses actions (±7..±24 brutes, clippées à ±1), donc à 1.0
chaque articulation roule aux **limites dures UR3e** : 180°/s sur les joints
de base, 360°/s sur les poignets, accélérations ~12.6/25.1 rad/s².

1. Zone dégagée obligatoire, E-stop en main.
2. Vérifier la config safety du pendant (limites de vitesse TCP, mode réduit)
   sous peine de protective stop au premier throw.
3. Monter par paliers : 0.5 → 0.7 → 0.85 → 1.0, balle virtuelle à chaque
   palier, en surveillant tracking error et protective stops.
4. Le streaming 500 Hz respecte `v_safe * dt_cmd` par pas : la trajectoire
   commandée reste bornée.

### 2.4 Changement de code associé (repo Stage)

- Ajouter un contrôle opérateur `v_safe_scale` dans la Web UI (Test tab) —
  gap déjà tracé dans le wiki. Aujourd'hui il faut éditer
  `src/ur3e_live_catch/config/live_catch.yaml` (valeur actuelle 0.5) entre
  chaque essai.

---

## Volet 3 — Revue du cfg d'entraînement Isaac et changements pour le retrain

Fichiers concernés :
`source/FirstTraining/FirstTraining/tasks/direct/firsttraining/firsttraining_env_cfg.py`,
`firsttraining_env.py`, `agents/skrl_ppo_cfg.yaml` (repo
`6-Dof-Ur3e-Catch-a-ball`).

### B1 — (Critique) La pénalité d'action ne peut pas dé-saturer la policy

Constat : dans `_pre_physics_step`,
`self.actions.copy_(torch.clamp(actions, -1.0, 1.0))`, puis la pénalité
`rew_action = -Σ c_i·a_i²` s'applique aux actions **clippées**. La policy
actuelle sort des moyennes à ±7..±24 : quasiment tous les échantillons de la
gaussienne clippent à ±1, donc tous reçoivent la même pénalité → PPO n'a aucun
signal différencié pour ramener les moyennes vers zéro. **La saturation est un
équilibre stable que la pénalité clippée ne peut pas casser, quelle que soit
la taille des coefficients.** Augmenter les coefficients ne résoudra rien.

Fix : ajouter une pénalité sur l'action **brute** (avant clamp), par exemple
une pénalité-frontière :

```python
# dans _pre_physics_step, conserver l'action brute :
self.raw_actions.copy_(actions)
self.actions.copy_(torch.clamp(actions, -1.0, 1.0))

# dans compute_rewards :
raw_excess = torch.relu(raw_actions.abs() - 1.0)
rew_saturation = -0.5 * torch.sum(raw_excess ** 2, dim=-1)
```

Un coefficient modeste (0.3–1.0) suffit : la pénalité croît quadratiquement
avec |action brute| et fournit un signal différencié partout (un échantillon à
14 coûte moins qu'à 16). Garder les coefficients per-joint sur l'action
clippée pour le shaping comportemental ; la pénalité-frontière s'occupe de la
saturation. Alternative équivalente : squashing tanh de la sortie policy.

Note : l'observation (composante « previous action ») reste l'action clippée,
conforme au contrat déployé — ne pas changer.

### B2 — (Important) Hiérarchie coude/poignets inversée par rapport à l'intention

Intention déclarée : privilégier les poignets par rapport au coude pour éviter
les grosses inerties. Coefficients finaux actuels
(`joint_action_penalty_coeff_ranges`, bornes hautes) : lift 2.85 > pan 2.55 >
wrist_1 1.35 > wrist_2 1.05 > **elbow 1.0** > wrist_3 0.75. Le coude est moins
pénalisé que deux poignets — l'inverse de l'objectif : le coude déplace tout
l'avant-bras + poignets + hoop.

Recommandation : coude ≈ 1.8–2.2, poignets < 1.0 (p.ex. wrist_1 0.9,
wrist_2 0.7, wrist_3 0.5), lift/pan inchangés.

Raffinements optionnels :

- Les poignets ont un `v_safe` double (2π vs π rad/s) : une action normalisée
  de 1 produit 2× plus de vitesse articulaire sur un poignet. La pénalité en
  espace normalisé sous-estime ce facteur (acceptable vu la faible inertie des
  poignets, mais à connaître).
- Plus rigoureux pour « éviter l'inertie » : pénaliser les **vitesses
  articulaires mesurées** pondérées par joint (voire l'accélération de la
  cible commandée) plutôt que l'action².

### B3 — (Important) Ne pas augmenter les magnitudes ; ajouter une pénalité de lissage

Analyse d'échelle : pénalité saturée = Σ coeffs ≈ 9.55/step. Fenêtre de vol de
la balle (y ∈ 1.2–2.1 m à 3.5–5 m/s) ≈ 20–36 steps → coût total d'une
interception « à fond » ≈ 230–340, contre +400 le pass et ~+1/step de
`rew_dist` après le catch (balle respawn tenue au centre du disque, robot peut
rester immobile). Bilan : catch ≈ +370 net, raté ≈ −130. **Doubler les
coefficients rendrait les catches difficiles à peine rentables** et
risquerait un abandon des balles dures. Les magnitudes actuelles sont proches
du plafond sain — les garder comme point de départ.

Ajouter à la place une pénalité de lissage, plus efficace contre la brutalité
et les pics d'accélération :

```python
rew_smooth = -c_smooth * torch.sum((self.actions - self.prev_actions) ** 2, dim=-1)
# c_smooth ≈ 0.5–1.0 ; prev_actions remis à zéro au reset
```

### B4 — (Critique pour le réel) Bornes ±2π et vitesses pleines encore dans le cfg

Le cfg actuel a `joint_position_lower/upper_rad = ±2π` (coude ±π) et les
vitesses/accélérations UR3e **pleines**. Le wiki Stage affirme que ces valeurs
ont été réduites le 02/07 (moitié des vitesses, bornes ±π) — ce n'est pas dans
ce checkout. Les bornes ±2π sont exactement ce qui a rendu possible l'incident
pendant (branche +2π de wrist_3). Pour le retrain :

- bornes de position ±π (ou l'enveloppe de sécurité réelle du poste) ;
- décider si les vitesses sont réduites de moitié (comportement plus doux) ou
  gardées pleines (fidélité aux limites dures) — dans les deux cas, ce que le
  cfg exporte dans `policy_metadata.json` devient le contrat déployé, et
  `v_safe_scale=1.0` côté Stage redevient le réglage fidèle.

### B5 — (À trancher) `disk_radius` 0.1 dans le cfg vs 0.05 dans les métadonnées déployées

Le prochain entraînement se ferait sur un trigger deux fois plus large (tâche
plus facile) et l'observation « disk radius » changera. Vérifier la cohérence
avec le rayon physique du hoop réel et avec
`wiki/calibration/frames-and-transforms.md` (distinction rayon visuel/rayon
trigger), puis fixer une valeur canonique.

### B6 — (Important pour le transfert) Robustesse perception : bruit et latence

État actuel : `ball_position_noise_std = 0.01` (1 cm ; le README Isaac
annonce 0.05), aucun bruit sur la **vitesse** de balle observée, aucune
latence d'observation modélisée. En réel, la vitesse est dérivée d'un
historique de positions (très bruitée) et la latence est le risque n°1 tracé
par le wiki. Pour le retrain :

- bruit gaussien sur position **et** vitesse de balle à chaque step (pas
  seulement au spawn) ;
- délai d'observation aléatoire de 1–3 steps (randomisation de latence) ;
- remonter `ball_position_noise_std` vers une valeur cohérente avec la
  précision réelle attendue de la perception événementielle.

C'est le changement au meilleur ratio effort/gain pour le transfert.

### B7 — Paramètres jugés sains (ne pas toucher sans raison)

- `sim.dt=1/120`, `decimation=2` → policy 60 Hz : cohérent avec le nœud live.
- PPO (`skrl_ppo_cfg.yaml`) : `initial_log_std=-0.5` (σ≈0.6),
  `entropy_loss_scale=0.005`, `discount_factor=0.995` sur épisodes de
  240 steps, KLAdaptiveLR — rien à redire.
- `action_penalty_warmup_steps=150_000` vs `trainer.timesteps=400_000` :
  coefficients au régime final à ~37 % de l'entraînement, raisonnable.
- Suggestion mineure : `random_robot_reset_on_ball_reset_probability` 0.05 →
  0.1–0.2 pour plus de diversité de poses initiales ; vérifier que la pose de
  départ réelle du bring-up est dans la distribution.

---

## Volet 4 — Avis global (contexte pour le wiki)

- Le projet est bien décomposé et l'ingénierie sérieuse : sémantique d'action
  pilotée par `policy_metadata.json`, safety fail-closed sans TF hoop,
  streaming 500 Hz interpolé, gate start-pose née d'un incident réel,
  validation incrémentale balle virtuelle → fake hardware → robot réel →
  perception réelle.
- La stratégie wiki Karpathy tient ses promesses (revue complète possible via
  l'index sans scanner les sources brutes). Ses deux risques : la couche
  `docs/Agent_Wiki/` redondante et l'angle mort cross-repo Isaac — tous deux
  adressés au Volet 1.
- Les deux risques techniques dominants restent : (a) la policy saturée qui
  roule aux limites dures (adressé Volet 3 B1–B4), (b) la latence de
  perception réelle non mesurée ni modélisée (adressé Volet 1 §1.3.3 et
  Volet 3 B6).

---

## Checklist d'exécution priorisée

### Wiki / docs (repo Stage)

- [ ] Corriger le stale claim `ur3e_sysid` dans
      `wiki/live-catch/current-status-and-blockers.md` (§1.2.1).
- [ ] Trancher et documenter l'état réel des limites du cfg Isaac
      (contradiction §1.2.2 / B4).
- [ ] Créer `wiki/sim-to-real/isaac-training-environment.md` (§1.3.1).
- [ ] Créer `wiki/operations/real-robot-bringup-runbook.md` (§1.3.2).
- [ ] Ajouter le plan de mesure de latence (§1.3.3).
- [ ] Remplacer la liste « How to Start » d'`AGENTS.md` par un renvoi vers
      `wiki/index.md` (§1.4.1).
- [ ] Réduire `docs/Agent_Wiki/` à un index de navigation (§1.4.2).
- [ ] Décider de la convention de provenance cross-repo (`raw/isaac/` ou
      référence sibling) et l'appliquer (§1.4.3).
- [ ] Mettre à jour `wiki/index.md` + `wiki/log.md`, lancer
      `python3 scripts/lint_llm_wiki.py`.

### Code Isaac (repo 6-Dof-Ur3e-Catch-a-ball) — avant retrain

- [ ] B1 : pénalité-frontière sur l'action brute (dé-saturation). **Critique.**
- [ ] B2 : réordonner les coefficients (coude au-dessus des poignets).
- [ ] B3 : pénalité de lissage `Δaction²` ; ne pas augmenter les magnitudes.
- [ ] B4 : bornes ±π ; décider vitesses réduites vs pleines. **Critique réel.**
- [ ] B5 : fixer `disk_radius` canonique (0.05 vs 0.1).
- [ ] B6 : bruit position+vitesse à chaque step, latence d'observation 1–3
      steps.
- [ ] Réentraîner, réexporter (`policy_metadata.json` reflète les nouveaux
      choix), copier l'export dans `data/models/` côté Stage.

### Bring-up robot réel (repo Stage) — sans attendre le retrain

- [ ] Monter `v_safe_scale` par paliers 0.5 → 0.7 → 0.85 → 1.0 (balle
      virtuelle, zone dégagée, E-stop, surveiller tracking error et
      protective stops) — 1.0 est le réglage fidèle à l'entraînement (§2).
- [ ] Ajouter le contrôle `v_safe_scale` dans la Web UI (§2.4).
- [ ] Vérifier la config safety du pendant avant les essais à haute vitesse.
