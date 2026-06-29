# Raw Sources

External sources for the LLM wiki go here.

Rules:

- Treat files in `raw/` as immutable source material.
- The agent may add new source files during an ingest operation.
- The agent must not rewrite existing raw files unless the user explicitly asks.
- Repository files such as `README.md`, `docs/`, `src/`, `scripts/` and
  `data/models/README.md` are also raw source material for this project, but they
  are not duplicated here.

Expected external-source format:

```markdown
# Source Title

> Source: URL or origin description
> Collected: YYYY-MM-DD
> Published: YYYY-MM-DD or Unknown

Original content below.
```
