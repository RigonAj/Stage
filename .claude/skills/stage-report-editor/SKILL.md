---
name: stage-report-editor
description: Edit, restructure, shorten, extend, proofread, audit, or review the French LaTeX internship report Stage_summary.tex while enforcing the official UCA/EUPI M1-M2 MTN report requirements, preserving its academic narrative, factual consistency, cross-references, and compilability. Use for any request concerning the rapport de stage or Stage_summary.tex, including adding, removing, moving, or rewriting sections, figures, tables, results, bibliography entries, the résumé, introduction, conclusion, or appendices. Keep the report at the scientific and engineering level; exclude source-code and software-internals narration unless the user explicitly requests it.
---

# Stage Report Editor

Edit `Stage_summary.tex` as a coherent French Master 1 internship report about
event-based 3D ball perception and robotic interception.

## Load Context

1. Read `AGENTS.md` and `wiki/index.md`.
2. Read the whole report outline, résumé, introduction, affected sections and
   their immediate neighbours. Also read the conclusion when one exists.
3. Read the relevant compiled wiki pages before checking repository sources.
   Open raw sources linked by the wiki only to verify an exact fact or resolve a
   contradiction.
4. Treat the user's requested changes and supplied evidence as authoritative.
   Do not use the soutenance slide skill unless the slide deck is also in scope.

Do not scan implementation files merely to make the prose sound more detailed.

## Official UCA/EUPI Requirements

Enforce the requirements summarized from
`docs/Consignes pour le rapport M1_M2MTN-1.pdf`. Treat that document as the
primary academic brief. Reopen it only to verify exact wording or the visual
cover-page model.

### Purpose and Balance

- Give the jury the evidence needed to assess the student's own activity.
- Explain the wider project to establish context and demonstrate perspective,
  but never let that general description obscure the student's contribution.
- Present the work synthetically and thematically, not as a day-by-day diary.
- Make decisions, responsibilities, reasoning, results and limits easy to
  identify.
- Deliver impeccable French: no spelling mistakes, grammatical errors or
  malformed sentences.

### Expected Length and Front Matter

- Target approximately 40 pages excluding appendices.
- Follow the supplied cover-page model and include at least: Université Clermont
  Auvergne, École Universitaire de Physique et d'Ingénierie, mémoire de stage,
  first year of Master, Mécatronique specialization, student name, stage title,
  defense date and host organization.
- Begin with one page of acknowledgements, about half a page summarizing the
  internship topic, then a paginated table of contents and an introduction.

### Part One: Host Organization and Unit

Allocate about 7 to 10 pages in total:

- About 5 to 7 pages for the host organization: identity, essential historical
  milestones, geographical presence, activities and beneficiaries or clients,
  and internal structure with an organization chart when useful.
- About 2 to 3 pages for the hosting unit or team: its name, activity, staff,
  organization, human and technical environment, and relationships with other
  units.
- Personalize and synthesize this presentation; never copy internal or public
  institutional material verbatim.

Adapt company-oriented fields to the actual public research organization. State
that capital, turnover, fax, clientele or similar fields are not applicable or
omit them with a clear rationale; never invent values to fill the template.

### Part Two: Position and Study

Allocate about 30 to 35 pages to the student's position and assigned study:

- Describe the role, responsibilities and tasks in their human, material and
  experimental environment. Here, “fonction” means the occupied professional
  role, never a software function or internal identifier.
- Structure the study according to its nature, normally covering: initial
  situation, critical analysis, proposed solutions, study and experimentation,
  implementation and follow-up, then assessment.
- Justify every important choice instead of only listing actions.
- Keep the student's contribution visible throughout the discussion.
- Move purely technical supporting documents to appendices when they would
  interrupt or overload the argument.

### Conclusion, Appendices and Sources

- End with about one page of conclusion. First synthesize the scientific and
  technical outcomes; then explain the professional and personal contributions
  of the training and internship experience.
- Reference every appendix from the main text and retain only documents useful
  for evaluating the student's activity. Never add large code listings or
  excessive collections of technical diagrams.
- Give every figure a number and caption, cite it in the prose, and credit its
  source when external.
- Keep the bibliography complete for all sources actually used.

If a requested change conflicts with these academic requirements, explain the
conflict and preserve the requirement unless the user explicitly confirms an
exception.

## Editorial Contract

Write clear, natural French suitable for an academic internship report. Preserve
the existing first-person perspective and technical level unless the user asks
for another style.

Organize technical explanations around:

- the problem and its importance;
- the physical or scientific principle;
- the chosen approach and the reasons for that choice;
- the experimental protocol;
- the observations and measured results;
- the limitations, uncertainty and next steps.

Describe what the system does and why, not how the source code is organized.
Do not insert:

- source-code excerpts or pseudocode;
- names of functions, classes, variables or internal identifiers;
- file or directory paths;
- command lines, launch commands or commit history;
- inventories of packages, nodes, scripts or implementation modules;
- a development log presented as a list of programming changes.

Names such as DVXplorer, ROS 2, C++, OpenCV, Isaac Lab and PPO are acceptable
when they help explain the experimental environment or engineering choice.
Mathematical equations, models and named scientific methods are also acceptable.
Translate all implementation knowledge into method, behaviour, inputs, outputs,
constraints or experimentally observable consequences.

Do not invent measurements, dates, equipment, completed experiments or results.
State a limitation or use an explicit `À confirmer` marker when a necessary fact
cannot be verified. Distinguish clearly between completed work, current work,
planned work and a proposed improvement.

### Results, Not an Experimental Diary

Report experimental work as a procedure, a final result and a short critical
assessment — never as a chronicle of sessions. In the main text do not narrate:

- dates of individual manipulations, calibration sessions or debug sessions;
- intermediate or discarded attempts, counts of removed acquisitions, or
  session-by-session retries;
- how many tries were needed, what went wrong on a given day, or partial
  results that the final retained result supersedes.

Instead, describe the procedure once, give the final retained result with its
quality metrics, and briefly discuss whether the obtained values are good
(compare them to expected orders of magnitude, requirements or prior results).
Keep only the principal and most pertinent content in the main text; move
detailed numerical values, session metadata and secondary measurements to an
appendix referenced from the prose. A failed attempt belongs in the main text
only when its lesson materially shapes the method or the limits of the study,
and then presented as a lesson, not as a dated anecdote. Dates may situate the
broad phases of the internship, never individual manipulations.

Apply this rule with particular care to the final chapter on real-robot
experiments, which is still being written: draft new content directly in this
synthetic, results-first form.

## Preserve Context When Changing Content

Before each addition, deletion or move, identify:

- the claim being changed;
- its role in the chapter's argument;
- earlier definitions on which it depends;
- later paragraphs, figures, tables or conclusions that depend on it;
- the source supporting any factual change.

Then update the whole dependency chain. In particular:

- introduce a concept before using it and define acronyms on first use;
- keep terminology, symbols, units, coordinate frames, chronology and tense
  consistent throughout the report;
- revise the transition before and after an edited passage;
- remove repetitions created by an addition;
- after a deletion, repair dangling transitions and unsupported conclusions;
- propagate material changes to the résumé, introduction, chapter synthesis and
  conclusion when those passages describe the same subject;
- update captions, citations and cross-references when a figure, table or
  section changes;
- preserve useful nuance, negative results and experimental limits.

Do not rewrite unrelated chapters solely to harmonize personal style. Prefer the
smallest complete edit that leaves the surrounding narrative natural.

## LaTeX Rules

- Preserve the document class, preamble and overall visual identity unless the
  user explicitly requests formatting changes.
- Preserve valid environments, balanced braces, labels, references, citations,
  image paths, equations and table structure.
- Reuse the report's existing LaTeX conventions such as `\figref`, units,
  captions and French typography.
- Never delete a label or bibliography item without checking all usages.
- Do not add a citation that is absent from the bibliography or unsupported by a
  consulted source.
- Keep figures close to the passage that explains their purpose and result.

## Workflow

1. Restate the requested changes as a private change ledger: target passage,
   dependent passages, evidence and validation needed.
2. Inspect the relevant context and verify factual claims through the wiki-first
   workflow.
3. Edit `Stage_summary.tex` directly with focused patches.
4. Reread each changed section continuously, including the paragraph before and
   after it. Perform a global consistency pass for repeated claims affected by
   the change.
5. Search the report prose for accidental code-level narration or diary-style
   session narration and replace it with scientific or engineering
   explanations following the results-first rule.
6. Audit the official structure: front matter, approximate page allocation,
   visibility of the student's contribution, conclusion, appendices, figures
   and bibliography. Report gaps even when they are outside the requested edit.
7. Compile with the repository's documented workflow, described in
   `docs/latex_compilation.md`: from the repository root run
   `./scripts/compile_stage_summary.sh` (same as the `compile-report` alias
   available after `source env.sh`). The script runs
   `latexmk -pdf -interaction=nonstopmode -halt-on-error Stage_summary.tex`
   and then deletes the LaTeX temporary files while keeping
   `Stage_summary.pdf`. The bibliography is an embedded `thebibliography`
   environment, so never run `bibtex` or `biber`. If `latexmk` is missing,
   run `pdflatex -interaction=nonstopmode -halt-on-error Stage_summary.tex`
   three times from the repository root to stabilize the table of contents,
   cross-references and citations. Keep image and PDF asset paths relative to
   the repository root. Do not install missing dependencies without
   permission; if no TeX toolchain is available, report that compilation
   could not be run instead of skipping the check silently.
8. Check compilation errors, undefined references or citations, missing assets
   and newly introduced layout warnings. Visually inspect affected pages when
   layout or figures changed.
9. Check the compiled PDF page count and flag a clear deviation from the target
   of approximately 40 pages excluding appendices.
10. Report the sections changed, important factual caveats and validation run.
   Decide whether the compiled wiki needs an update under `AGENTS.md`.

## Request Template

Use this compact request form when the user wants to batch several changes:

```text
Utilise $stage-report-editor pour modifier Stage_summary.tex.

Changements demandés :
- ...
- ...

Informations ou résultats à intégrer :
- ...

Contraintes particulières :
- ...
```
