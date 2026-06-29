# Agent Workflow

## Goal

Use the wiki as a small retrieval layer before reading source code. The agent
should move from summary to package to exact file, and only then edit.

## Retrieval Order

1. Read `wiki/index.md`.
2. Read the relevant compiled article under `wiki/<topic>/`.
3. Read `wiki/log.md` only when recent maintenance history matters.
4. Open raw sources from the article `Raw:` field for verification.
5. Use [[Index]] / [[Project_Map]] only as secondary navigation.
6. Use [[Inventory]] to locate packages, docs and tests.
7. Inspect source files with targeted `rg`, `sed` or package-local test names.
8. Edit narrowly.
9. Run the smallest meaningful verification.
10. Update `wiki/` if package roles, commands, topics or safety contracts
   changed.

## Notes for Obsidian

Open `docs/Agent_Wiki/` as a vault or keep it inside a larger vault rooted at the
repository. Wikilinks use relative note names so the graph works inside Obsidian.

Recommended Obsidian usage:

- Pin [[Index]].
- Use graph view around [[Project_Map]] to navigate responsibilities.
- Keep source-of-truth implementation details in code and package docs; keep
  this wiki as navigation and stable architecture memory.

## Maintenance

Run this after adding or removing packages or Markdown docs:

```bash
python3 scripts/update_agent_wiki.py
```

Run this after editing `wiki/`:

```bash
python3 scripts/lint_llm_wiki.py
```

Then review `docs/Agent_Wiki/Inventory.md` before committing.
