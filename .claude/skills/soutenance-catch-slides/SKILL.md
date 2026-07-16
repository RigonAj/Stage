---
name: soutenance-catch-slides
description: "Create, extend, and refine the LaTeX/TikZ soutenance slides for the Catch a ball project. Use when working on Soutenance_Catch_a_ball.tex/PDF, adding visual defense slides, inserting Okular-friendly videos, preserving the brown-orange-red style, or fixing layout issues such as overlapping text, logos, arrows, panels, and crowded diagrams."
---

# Soutenance Catch Slides

## Goal

Build a visual, low-text LaTeX deck for the Catch a ball soutenance. Preserve the established brown, orange, red style, the 16:9 PDF output, and Okular readability.

## Before Editing

- Read the current `Soutenance_Catch_a_ball.tex` before changing style or layout.
- Read `Stage_summary.tex` for project facts, wording, organizations, and technical claims.
- Use local assets first: `images/logos/`, `images/Tuteurs/`, `images/`, and `video/`.
- Do not overwrite the repository root `SKILL.md`; it is the wiki skill. This slide skill lives under `skills/soutenance-catch-slides/`.
- For presentation-only edits, do not update `wiki/` unless the change also modifies project architecture, commands, topics, safety, calibration, or package responsibilities.

## LaTeX Format

- Keep the deck as a 16:9 `article` document unless the user explicitly asks to convert it:

```latex
\documentclass[11pt]{article}
\usepackage[paperwidth=16cm,paperheight=9cm,margin=0cm]{geometry}
\usepackage{graphicx}
\usepackage{xcolor}
\usepackage{tikz}
\usetikzlibrary{calc,arrows.meta}
\usepackage[hidelinks]{hyperref}
```

- Build each slide as one full-page `tikzpicture` with `remember picture,overlay`.
- Use absolute coordinates from `current page.south west` for stable placement.
- Compile with `pdflatex`; verify the generated PDF, not only the `.tex`.
- Keep videos Okular-friendly by default: use the `multimedia` package (`\movie`)
  with an external video file in `video/` and a poster frame on the slide. Okular
  plays this inline in the page. Do NOT use `run:` links: Okular blocks them with
  a "Security alert: this document has been prevented from opening the file" and
  offers no clean toggle to allow program launch.

```latex
\usepackage{multimedia} % from the beamer bundle; tlmgr install beamer
...
\movie[externalviewer=false,showcontrols=true]%
  {\includegraphics[width=6cm]{images/demo_poster.png}}{video/demo.mp4}
```

- The `\movie` annotation references (does not embed) the mp4, so a shared PDF
  must travel with the `video/` folder. For a truly self-contained file, `media9`
  embeds the video but targets Adobe Reader; Okular plays embedded media poorly.
- Inline playback needs an Okular multimedia backend (GStreamer/VLC) on the host.

## Style Contract

- Palette:
  - `deepbrown` `#2B120B`
  - `redbrown` `#5C1C12`
  - `rust` `#B6401F`
  - `orangehot` `#F47A1F`
  - `amber` `#F4A322`
  - `cream` `#FFF2DE`
  - `softcream` `#F8D9B2`
- Background: dark brown to red-brown base, with subtle translucent diagonal rust/orange bands. Keep gradients gentle; avoid harsh logo-background gradients.
- Typography: large serif title, very short supporting text, no long paragraphs.
- Use text sparingly: one main message per slide, then keywords, labels, or diagrams.
- Use real assets when possible: robot render, camera imagery, diagrams, logos, tutor portraits, video stills.
- Panels: dark translucent fill, orange outline, small rounded corners. Avoid nested cards.
- Lines and accents: thin orange/amber rules, curved orange trajectory on title-style slides, restrained glow/opacity.

## Slide Patterns

### Title Slide

- Use the logo band only on the first slide unless the user asks otherwise.
- Put logos on a warm solid band, centered and enlarged enough to be legible.
- Do not place logos on black or white rectangles if transparent PNGs are available.
- Keep the main title dominant. The underline should sit a few pixels below the title, not touch it.
- Put tutor photos and names in a right-side encadrement panel. If a portrait is missing, use initials in a circular badge.

### Content Slides

- Do not reuse the top logo band by default; it consumes space.
- Reserve a clean title area at the top. Do not let charts or boxes reach into it.
- Use two large panels only when comparison helps. Otherwise prefer one large visual area.
- Keep bottom notes, source lines, or captions optional and tiny. Remove them if they crowd the slide.

### Organization Charts

- Keep boxes compact and readable. Reduce font size or text width before making boxes large.
- Draw short links between adjacent hierarchy levels.
- Anchor arrows to node edges: `node.south`, `node.north`, `node.east`, `node.west`.
- Do not draw one parent-to-every-child set of long arrows if those arrows cross central boxes.
- If arrows still cross a box, change the topology, use side branches, or draw links before nodes and keep them visually secondary.
- Leave breathing room below portraits and name labels. Ensure name text stays inside any card or panel.

## Video Slides

- Place video files under `video/` and use relative paths.
- Use a poster image or a still frame as the main visual. If no still exists, create one from the video with a tool already available in the environment.
- Add a visible play affordance, but keep it small and aligned with the style.
- Prefer one video per slide.
- Provide a static fallback visual so the slide remains meaningful if the video does not play.
- Use `\movie` (multimedia package) for inline playback; never `run:` links (Okular blocks them).
- After compiling, open/test the video in Okular when possible; PDF compilation success is not enough.

## Common Errors To Avoid

- Too much text, especially explanatory paragraphs.
- Charts or diagrams overlapping the slide title.
- Text escaping from cards because no `text width` was set.
- Arrows passing over boxes or labels.
- Organization boxes made large to compensate for long labels.
- Logo backgrounds that clash with the warm palette.
- Severe gradients behind logos.
- Reusing the first-slide logo band on dense content slides.
- Source lines or captions placed so low that they feel detached or cramped.
- Validating only with LaTeX logs and not with rendered images.
- Assuming embedded videos will work in Okular without a manual check.

## Problèmes rencontrés et solutions

Concrete issues hit while extending this deck and how they were fixed. Check
here first when a slide misbehaves.

- **New / moved page renders fully white (video slide).** Adding a page shifts
  every following `remember picture,overlay` slide, so the `.aux` positions are
  stale for one run. With a `\movie` annotation this can repaint the whole page
  white. Fix: **always compile twice** (see Validation Workflow). The white page
  is a one-pass artifact, not a broken slide.
- **No `ffmpeg` in the environment to grab a poster frame.** Use OpenCV from the
  system Python (already installed as `cv2`). Pick a representative frame and
  write a PNG:

  ```bash
  python3 -c "
  import cv2
  cap = cv2.VideoCapture('video/NAME.mp4')
  n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
  cap.set(cv2.CAP_PROP_POS_FRAMES, int(n*0.45))  # ~45% in
  ok, f = cap.read()
  cv2.imwrite('images/NAME_poster.png', f)
  print(ok)
  cap.release()"
  ```

- **Panel text decalé / touche le bord droit.** The right/left panel center is
  the average of its two x edges, not eyeballed. Compute it: for a panel from
  `x_left` to `x_right`, center `= (x_left+x_right)/2`. Center every `anchor=center`
  node and every card on that value. Give card text a `text width` that leaves a
  symmetric margin on both sides (e.g. card `9.26–15.00`, text starts at `9.80`,
  `text width=4.45cm` → ~0.5–0.7 cm margin each side).
- **Accent marker pushes card text off-center.** A leading bullet/dot inside a
  card eats horizontal space and shifts the title right. Prefer a thin vertical
  colour stripe on the card's left edge (`\fill` a narrow rounded rectangle),
  then align title and subtitle to the same left x just after it.
- **Ugly hyphenation (`ro-bot`) in a narrow card.** Shorten the wording instead
  of widening the box; low-text slides should not need mid-word breaks.
- **Confirm the `\movie` annotation actually points at the mp4.** PyPDF2 can read
  the page annotation and show the referenced (not embedded) relative path:

  ```bash
  python3 -c "
  from PyPDF2 import PdfReader
  import re
  a = str(PdfReader('Soutenance_Catch_a_ball.pdf').pages[<n-1>].get('/Annots'))
  print(re.findall(r'[^\s]+\.mp4', a))"
  ```

## Validation Workflow

1. Patch the `.tex` file with minimal, localized edits.
2. Compile **twice**. The deck uses `remember picture,overlay`, so absolute
   coordinates come from the `.aux` file written by the previous run. A single
   pass after adding or moving a page places content (and video annotations)
   with stale/empty positions:

```bash
cd /home/rigon/Dv-Rosws/Dv-Rosws
pdflatex -interaction=nonstopmode Soutenance_Catch_a_ball.tex \
  && pdflatex -interaction=nonstopmode Soutenance_Catch_a_ball.tex
```

3. Render the edited slide to PNG for visual inspection (last page number =
   number of slides; check `pdfinfo Soutenance_Catch_a_ball.pdf | grep Pages`):

```bash
pdftoppm -png -f <slide-number> -singlefile -r 180 Soutenance_Catch_a_ball.pdf /tmp/soutenance_catch_slide
# fallback renderer if pdftoppm output looks wrong:
gs -q -dNOPAUSE -dBATCH -sDEVICE=png16m -r150 \
  -dFirstPage=<n> -dLastPage=<n> -sOutputFile=/tmp/slide.png Soutenance_Catch_a_ball.pdf
```

4. For video slides, PyPDF2 is available in this workspace and can be used to
   inspect PDF page annotations/media references when needed. On a fresh machine:

```bash
python3 -m pip install --user PyPDF2
```

5. Inspect the PNG before finalizing. Check title spacing, text containment, logo readability, arrow paths, panel balance, and bottom margins.
6. Iterate until the rendered slide, not just the source, looks clean.
