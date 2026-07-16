# AGENTS.md

## Karpathy LLM Wiki Contract

This repository uses a Karpathy-style LLM wiki:

- `raw/`: immutable external sources collected for future ingestion. Read only.
- repository sources: `README.md`, `docs/`, `src/`, `scripts/`, `data/models/README.md`.
  Treat these as raw source material too: read them for facts, do not rewrite
  them as part of wiki maintenance unless the user explicitly asks to edit docs.
- `wiki/`: compiled knowledge pages owned by the agent. The agent may create and
  update these pages during wiki operations.
- `AGENTS.md`: schema layer. It defines how agents ingest, query and lint.

Core rule: do not answer repository questions by scanning all raw docs first.
Read `wiki/index.md`, follow the relevant compiled pages, then open raw sources
only to verify details or resolve contradictions.

## How to Start

1. Read `wiki/index.md`.
2. Read `wiki/log.md` only if recent wiki history matters.
3. Pick the relevant compiled page from the `wiki/index.md` tables.
   `wiki/index.md` is the single authoritative list of articles; do not
   duplicate that list here.
4. Open raw repository docs linked in the page's `Raw:` field only when needed.
5. Use `docs/Agent_Wiki/` as a secondary Obsidian/navigation layer, not the
   primary compiled wiki.

## Project Summary

The project is a ROS 2 workspace for event-based 3D ball tracking with a
DVXplorer camera and UR3e robot interception. It combines:

- C++ event-camera perception in `src/Ball_Tracking_Cpp/`.
- ROS 2 messages in `src/ur3e_catch_msgs/`.
- A live closed-loop UR3e catch stack in `src/ur3e_live_catch/`.
- A browser UI in `src/ur3e_web_ui/`.
- System identification and rollout replay tools in `src/ur3e_sysid/` and
  `src/ur3e_rollout_replay/`.
- Technical documentation in `docs/`.

## Context Loading Rules

- Do not read generated folders unless the task explicitly needs them:
  `build/`, `install/`, `log/`, `.deps/`, `.venv/`, `__pycache__/`.
- Do not open binary or heavy data as text: `*.bin`, `*.h5`, `*.onnx`, `*.pdf`,
  `*.glb`, `*.step`, images, recordings and local sequences.
- Avoid loading `raygui.h` unless the issue is directly about raygui.
- Prefer package-local tests and docs before scanning the whole repository.
- Prefer `rg` / `rg --files` for discovery.

## Soutenance Slide Requests

When the user asks to create, modify, continue, or fix the soutenance
diaporama/slides, read `skills/soutenance-catch-slides/SKILL.md` before editing
the LaTeX deck. Follow that skill for the Catch a ball visual style, Okular
video handling, layout validation, and common slide-design errors to avoid.

## Stage Report Requests

When the user asks to create, modify, continue, review or fix the internship
report `Stage_summary.tex`, read `skills/stage-report-editor/SKILL.md` before
editing it. Preserve the report's academic narrative and surrounding context,
and describe the engineering work without source-code or software-internals
narration unless the user explicitly requests such details.

## Wiki Page Conventions

- `wiki/` uses one topic directory level only: `wiki/<topic>/<article>.md`.
- File names are kebab-case concept names, not source-document names.
- Every non-archive article must include:

```markdown
# Title

> Sources: Source name, date; Source name, date
> Raw: [source](../../path/from/article.md); [source](../../raw/topic/file.md)

## Overview
```

- `Sources:` is human-readable provenance.
- `Raw:` is a set of links to source material used to verify the article.
- `Updated` in `wiki/index.md` means the date the article knowledge changed, not
  the file modification timestamp.
- Keep `## See Also` links relative to the current wiki file.
- Archive/query-result pages use `Sources:` and `Archived:` metadata and do not
  need a `Raw:` field.

## Wiki Operations

### Ingest

Use this when the user asks to add sources, update the wiki from docs, or compile
new project knowledge.

1. Identify the source:
   - External source: save a cleaned markdown copy in
     `raw/<topic>/YYYY-MM-DD-descriptive-slug.md` with `Source`, `Collected`
     and `Published` metadata.
   - Existing repository source: do not duplicate it into `raw/`; cite the
     existing path in the article `Raw:` field.
   - Sibling-repo source (e.g. the Isaac training repo
     `/home/rigon/Documents/6-Dof-Ur3e-Catch-a-ball`): treat it as external.
     Snapshot the cited doc into `raw/isaac/YYYY-MM-DD-slug.md` with the same
     metadata header; the lint rejects `Raw:` links that resolve outside this
     repository, so never link the sibling checkout directly.
2. Read `wiki/index.md` first to find the best topic and existing article.
3. Merge into an existing article when it is the same concept. Create a new
   `wiki/<topic>/<concept-slug>.md` article only for a distinct concept.
4. Each article must use this header:

```markdown
# Title

> Sources: Source name, date; Source name, date
> Raw: [source](../../path/from/article.md); [source](../../raw/topic/file.md)
```

5. Article body should synthesize, not copy. Use sections that fit the domain.
6. Add or update `## See Also` links when related pages exist.
7. Cascade update related articles when the new source materially changes their
   content, contradictions, status, commands, topics or safety assumptions.
8. Update `wiki/index.md` for every changed article.
9. Append one entry to `wiki/log.md`:

```markdown
## [YYYY-MM-DD] ingest | Short description

- Updated: [Article](topic/article.md)
```

### Query

Use this when answering questions from the knowledge base.

1. Read `wiki/index.md`.
2. Read only the relevant wiki articles.
3. Prefer wiki content over model memory.
4. Open raw sources only to verify exact facts, dates, commands or contradictions.
5. Cite wiki pages in conversation with project-root paths, e.g.
   `[Live Catch](wiki/live-catch/live-catch-loop.md)`.
6. Do not write files during a plain query unless the user asks to save/archive
   the answer.

### Archive Query Results

When the user asks to save a synthesis:

1. Create a new page under the most relevant `wiki/<topic>/` directory.
2. Use `Sources:` links to the wiki articles cited by the answer.
3. Do not include a `Raw:` field unless the page came from raw sources.
4. Prefix the index summary with `[Archived]`.
5. Append a `query | Archived:` entry to `wiki/log.md`.

### Lint

Use this when the user asks to verify or maintain the wiki.

1. Run:

```bash
python3 scripts/lint_llm_wiki.py
```

2. Auto-fix only safe index/link issues when the fix is unambiguous.
3. Report, do not silently rewrite, these cases:
   - contradictions between wiki pages and raw sources;
   - stale claims superseded by newer docs;
   - orphan pages;
   - missing cross-topic links;
   - concepts mentioned repeatedly but missing a page;
   - missing or suspicious `Sources:` / `Raw:` metadata.
4. Append a `lint` entry to `wiki/log.md` when a maintenance pass changes files.

### After Code Or Docs Changes

At the end of any task that edits code, launch files, config or documentation,
decide whether the compiled wiki must change.

Update `wiki/` when a change affects:

- package responsibilities or architecture;
- ROS messages, topics, services, actions or TF frames;
- robot safety, command gating, controller switching or watchdog behavior;
- build, launch, test or bring-up commands;
- calibration procedure, model location, policy semantics or latency assumptions;
- current blockers/status tracked by the wiki.

If no wiki update is needed, mention that in the final response. If wiki files
changed, run `python3 scripts/lint_llm_wiki.py`.

## Commands

From the workspace root:

```bash
source env.sh
build
run
```

Useful targeted tests:

```bash
cd src/ur3e_live_catch && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
cd src/ur3e_rollout_replay && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
cd src/ur3e_web_ui && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
cd src/ur3e_sysid && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

Refresh the agent wiki inventory after adding packages or docs:

```bash
python3 scripts/update_agent_wiki.py
```

Lint the compiled LLM wiki:

```bash
python3 scripts/lint_llm_wiki.py
```

## Editing Rules

- Keep robot-safety behavior explicit and testable.
- Preserve frame names, units and timestamps when touching perception-to-robot
  interfaces.
- Do not silently assume a frame when `BallState.header.frame_id` is missing.
- Keep command emission disabled by default unless a task explicitly changes
  bring-up behavior.
- Update `wiki/` when changing architecture, commands, topics or package
  responsibilities. Update `docs/Agent_Wiki/` only if its navigation layer also
  becomes stale.
- When docs and code disagree, record the discrepancy in the relevant wiki note
  or in `docs/incoherences_code_logique.md`.
