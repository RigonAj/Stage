# Wiki Maintenance

> Sources: Karpathy LLM wiki idea, 2026-04-04; Astro-Han karpathy-llm-wiki skill, 2026-06-29; local schema, 2026-06-29
> Raw: [Karpathy gist](https://gist.github.com/karpathy/442a6bf555914893e9891c11519de94f); [Astro-Han SKILL](https://github.com/Astro-Han/karpathy-llm-wiki); [AGENTS](../../AGENTS.md)

## Overview

The wiki follows a compile-and-maintain pattern: raw source material is the
source of truth, while `wiki/` is the durable compiled knowledge layer. Agents
maintain `wiki/index.md`, `wiki/log.md`, article metadata and cross-links.

## Structure

```text
raw/        external immutable sources
wiki/       compiled knowledge pages
AGENTS.md   schema and operating rules
SKILL.md    Agent Skills entry point
```

For this repository, existing project files (`README.md`, `docs/`, `src/`,
`scripts/`) are also raw source material. They are cited directly instead of
being duplicated into `raw/`.

## Operations

- Ingest: read a source, update or create relevant wiki articles, update
  `wiki/index.md`, append to `wiki/log.md`.
- Query: read `wiki/index.md`, then relevant articles, then raw sources only for
  verification.
- Lint: check index integrity, links, raw references, stale claims and missing
  cross-links.

## Page Rules

- Use one topic directory level: `wiki/<topic>/<article>.md`.
- Name pages after concepts, not raw source files.
- Keep provenance in `Sources:` and verifiable paths in `Raw:`.
- Keep `Updated` dates in `wiki/index.md` aligned with knowledge changes.
- Cascade updates to related pages when a source changes a contract, command,
  status, safety rule or architecture assumption.
- After code/config/doc changes, update `wiki/` when the compiled knowledge would
  otherwise become stale.

## Skill Entry Point

`SKILL.md` exists at the repository root for Agent Skills-compatible tools. It is
intentionally compact and delegates the full local schema to `AGENTS.md`.

## Local Lint

```bash
python3 scripts/lint_llm_wiki.py
```

## See Also

- [Testing And Commands](testing-and-commands.md)
- [Source Document Map](source-document-map.md)
- [Repository Map](../overview/repository-map.md)
