# Revue complète : wiki Stage, déploiement v_safe_scale et configuration d'entraînement Isaac

> Source: Revue par agent (Claude, session Claude Code remote), transmise par l'utilisateur le 2026-07-03
> Collected: 2026-07-03
> Published: 2026-07-02

Note d'ingestion : extrait fourni par l'utilisateur ; seul le Volet 1 (revue du
wiki LLM) est reproduit intégralement ici. Les Volets 2 (déploiement
`v_safe_scale`, déjà appliqué au code) et 3 (configuration d'entraînement Isaac)
ne sont référencés que par leurs renvois (B4, B6).

Original content below.

---

Ce document compile une revue en trois volets, destinée à être ingérée dans le
wiki LLM (`wiki/`) et à servir de cahier des charges pour les changements de
code dans le repo Isaac `6-Dof-Ur3e-Catch-a-ball` et dans `src/ur3e_live_catch/`.

Méthode : lecture de `CLAUDE.md`/`AGENTS.md`, des 18 articles de `wiki/`, de
`wiki/log.md`, exécution de `scripts/lint_llm_wiki.py` (passe sans erreur),
lecture de `data/models/latest/policy_metadata.json`,
`src/ur3e_live_catch/ur3e_live_catch/live_catch_node.py`, et côté Isaac de
`firsttraining_env_cfg.py`, `firsttraining_env.py` (reward et
`_pre_physics_step`) et `agents/skrl_ppo_cfg.yaml`.

## Volet 1 — Revue du wiki LLM (repo Stage)

### 1.1 Ce qui fonctionne (à conserver tel quel)

- Structure conforme au contrat : un niveau `wiki/<topic>/<article>.md`,
  métadonnées `Sources:`/`Raw:` partout, liens See Also, lint déterministe qui
  passe.
- La règle de découpage par cadence de changement est respectée : pages
  d'architecture stables séparées de
  `wiki/live-catch/current-status-and-blockers.md` qui churn.
- `wiki/log.md` est discipliné (~20 entrées en 4 jours, cascades documentées).
- Le diagnostic de l'incident pendant du 2026-07-02 (wrap ±2π sur wrist_3) est
  documenté de façon exemplaire : cause racine, forensics, fixes, action
  opérateur.

### 1.2 Corrections factuelles à faire dans le wiki

- Stale claim `ur3e_sysid` : `wiki/live-catch/current-status-and-blockers.md`
  (section « Documentation Reproducibility Gaps ») affirme que
  `src/ur3e_sysid/` est « present locally but untracked in the current
  worktree ». C'est faux : `git ls-files` montre que le package est tracké.
  Supprimer ou corriger la phrase.
- Contradiction wiki ↔ cfg Isaac : le wiki (`current-status-and-blockers.md`,
  `sim-to-real/policy-transfer-and-action-semantics.md`) affirme que le cfg
  Isaac FirstTraining du 02/07 « halves joint_velocity_safe /
  joint_acceleration_safe and keeps joints within ±π ». Le checkout actuel de
  `6-Dof-Ur3e-Catch-a-ball` (`firsttraining_env_cfg.py`, dernier commit « WIP:
  archive current Isaac training changes ») contient encore les limites
  pleines : `joint_velocity_safe_rad_s = (π, π, π, 2π, 2π, 2π)`, accélérations
  pleines, bornes de position ±2π (coude ±π). Soit le changement n'existe que
  sur une machine locale non commitée, soit il a été perdu. À trancher, puis
  aligner wiki et cfg. Voir Volet 3, action B4.

### 1.3 Lacunes structurelles du wiki (pages à créer)

- `wiki/sim-to-real/isaac-training-environment.md` (priorité haute). Le trou
  principal : les pages sim-to-real citent constamment « Isaac FirstTraining »
  mais aucune page compilée ne décrit l'environnement d'entraînement. Contenu
  attendu :
  - définition de l'observation 33-D côté Isaac et ordre des composantes ;
  - structure du reward (`compute_rewards` : `rew_dist = exp(-2d) - d`,
    pénalité d'action per-joint avec warmup, +400 pass, -100 termination) ;
  - terminaisons (`hit_arm`, `ball_on_ground`, `reset_on_success=False`,
    respawn de balle tenue au centre du disque) ;
  - commandes train/play/evaluate/export (elles sont dans le README Isaac et
    partiellement dans `wiki/operations/testing-and-commands.md`) ;
  - résultat de référence : ~98 % de réussite en éval headless
    (`ball_position_noise_std=0.05` d'après le README ; noter que le cfg
    actuel dit 0.01 — voir Volet 3, B6) ;
  - procédure de synchro cross-repo : quel export part dans `data/models/`,
    comment vérifier la parité des métadonnées (`policy_metadata.json`) avant
    déploiement.
- `wiki/operations/real-robot-bringup-runbook.md` (priorité haute). Runbook
  opérateur durable, extrait de la page de statut qui churn :
  - checklist avant `enable_command` (controller actif, TF hoop présent,
    `/joint_states` cohérent avec le pendant, zone dégagée, E-stop en main) ;
  - procédure quand la gate start-pose ±2π déclenche : jog/unwind du poignet
    ou reboot du bras jusqu'à ce que `joint_states` corresponde au pendant ;
  - procédure de montée en vitesse `v_safe_scale` par paliers (voir Volet 2) ;
  - paramètres à surveiller : tracking error, protective stops, latence.
- Plan de mesure de latence (page ou section dans
  `wiki/sim-to-real/observation-latency-and-models.md`) : le wiki identifie la
  latence comme risque n°1 mais ne décrit pas comment mesurer p50/p95/p99
  bout-en-bout avec la perception réelle (`latency_report`,
  `CatchTelemetry.perception_age_s`, procédure de collecte et seuils
  d'acceptation).

### 1.4 Maintenance du contrat et des couches

- `AGENTS.md` « How to Start » incomplet : liste 15 pages, le wiki en a 18.
  Manquent `wiki/overview/project-overview.md`,
  `wiki/overview/repository-map.md`, `wiki/operations/wiki-maintenance.md`.
  Recommandation : remplacer la liste dupliquée par un simple renvoi vers
  `wiki/index.md` (supprime la double maintenance).
- `docs/Agent_Wiki/` (couche secondaire) duplique du contenu de statut : son
  `Current_Status.md` est maintenu en parallèle du wiki primaire (double coût,
  divergence garantie). Recommandation : le réduire à un index de navigation
  Obsidian pointant vers `wiki/`, sans contenu propre.
- Provenance cross-repo : les articles wiki affirment des faits Isaac (ordre
  `_get_dones()`, limites cfg, sémantique d'action) avec des `Raw:` pointant
  uniquement vers des docs Stage. Deux options : mirrorer les docs Isaac clés
  (`docs/environment_and_frames.md`, `docs/sim2real_v1.md`, README) dans
  `raw/isaac/` selon la convention d'ingestion, ou étendre la convention
  `Raw:` pour autoriser une référence au repo sibling. `raw/` ne contient
  aujourd'hui que son README : le mécanisme n'a jamais servi.
- À terme : archiver les incidents datés (incident pendant) vers une page
  d'archive quand ils ne seront plus opérationnels, pour éviter que
  `current-status-and-blockers.md` devienne un dépotoir.
