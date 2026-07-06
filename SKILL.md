---
name: dv-rosws-llm-wiki
description: "Use when maintaining this repository's Karpathy-style LLM wiki: ingesting sources into raw/wiki, querying project knowledge, linting wiki quality, or updating wiki pages after code, docs, ROS topics, robot safety, calibration, model, launch, or architecture changes."
---

# DV-Rosws LLM Wiki

This skill maintains the repository-local Karpathy-style LLM wiki.

## Required Context

When this skill triggers, read these files in order:

1. `AGENTS.md` for the full local schema and safety rules.
2. `wiki/index.md` to route to the right compiled page.
3. The relevant `wiki/<topic>/<article>.md` page.
4. Raw sources linked in that page's `Raw:` field only when needed for
   verification.

Use `docs/Agent_Wiki/` only as a secondary navigation layer.

## Directory Contract

- `raw/`: immutable external source material.
- `wiki/`: compiled knowledge pages owned by the agent.
- `wiki/index.md`: global table of contents, updated on every ingest/archive.
- `wiki/log.md`: append-only operation log.
- `AGENTS.md`: authoritative local schema for ingest/query/lint behavior.

## Operations

- **Ingest**: add external sources to `raw/` or cite existing repository files,
  update/create concept pages under `wiki/`, cascade updates to related pages,
  update `wiki/index.md`, append to `wiki/log.md`. Facts from the sibling Isaac
  training repo are snapshotted into `raw/isaac/` (see AGENTS.md); `Raw:` links
  must resolve inside this repository.
- **Query**: read `wiki/index.md`, then relevant compiled pages. Prefer wiki
  content over model memory and cite project-root paths in the answer. Do not
  write files unless asked to save/archive.
- **Lint**: run `python3 scripts/lint_llm_wiki.py`; fix deterministic issues
  only when unambiguous and report semantic issues.
- **After project edits**: update `wiki/` when code/docs/config changes affect
  architecture, ROS contracts, TF frames, safety, commands, calibration, model
  semantics, latency assumptions, package responsibilities, or current blockers.

## Validation

After editing `wiki/`, run:

```bash
python3 scripts/lint_llm_wiki.py
```

After editing wiki scripts, also run:

```bash
python3 -m py_compile scripts/update_agent_wiki.py scripts/lint_llm_wiki.py
```
