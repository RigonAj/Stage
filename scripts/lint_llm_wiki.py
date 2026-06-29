#!/usr/bin/env python3
"""Deterministic checks for the Karpathy-style wiki structure."""

from __future__ import annotations

import pathlib
import re
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
WIKI = ROOT / "wiki"
INDEX = WIKI / "index.md"
LOG = WIKI / "log.md"
LINK_RE = re.compile(r"(?<!!)\[[^\]]+\]\(([^)]+)\)")


def is_external(target: str) -> bool:
    return (
        "://" in target
        or target.startswith("#")
        or target.startswith("mailto:")
        or target.startswith("tel:")
    )


def article_files() -> list[pathlib.Path]:
    files: list[pathlib.Path] = []
    for path in sorted(WIKI.glob("*/*.md")):
        if path.is_file():
            files.append(path)
    return files


def check_structure(errors: list[str]) -> None:
    if not WIKI.is_dir():
        errors.append("missing wiki/ directory")
        return
    if not INDEX.is_file():
        errors.append("missing wiki/index.md")
    if not LOG.is_file():
        errors.append("missing wiki/log.md")

    for path in sorted(WIKI.rglob("*.md")):
        rel = path.relative_to(WIKI)
        if rel.parts in (("index.md",), ("log.md",)):
            continue
        if len(rel.parts) != 2:
            errors.append(f"wiki article must be one topic level deep: {path}")


def check_index(errors: list[str]) -> None:
    if not INDEX.is_file():
        return
    text = INDEX.read_text(encoding="utf-8", errors="replace")
    for article in article_files():
        rel = article.relative_to(WIKI).as_posix()
        if rel not in text:
            errors.append(f"article missing from wiki/index.md: {rel}")


def check_article_metadata(errors: list[str]) -> None:
    for article in article_files():
        text = article.read_text(encoding="utf-8", errors="replace")
        rel = article.relative_to(ROOT).as_posix()
        if not text.startswith("# "):
            errors.append(f"missing H1 title: {rel}")
        if "> Sources:" not in text:
            errors.append(f"missing Sources metadata: {rel}")
        if "> Raw:" not in text and "[Archived]" not in text:
            errors.append(f"missing Raw metadata: {rel}")


def check_links(errors: list[str]) -> None:
    for md in sorted(WIKI.rglob("*.md")):
        text = md.read_text(encoding="utf-8", errors="replace")
        for match in LINK_RE.finditer(text):
            target = match.group(1).strip()
            if is_external(target):
                continue
            target_no_anchor = target.split("#", 1)[0]
            if not target_no_anchor:
                continue
            resolved = (md.parent / target_no_anchor).resolve()
            try:
                resolved.relative_to(ROOT)
            except ValueError:
                errors.append(f"link escapes repository: {md.relative_to(ROOT)} -> {target}")
                continue
            if not resolved.exists():
                errors.append(f"broken link: {md.relative_to(ROOT)} -> {target}")


def main() -> int:
    errors: list[str] = []
    check_structure(errors)
    check_index(errors)
    check_article_metadata(errors)
    check_links(errors)

    if errors:
        print("LLM wiki lint failed:")
        for error in errors:
            print(f"- {error}")
        return 1

    print("LLM wiki lint passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
