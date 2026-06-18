#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TEX_BASENAME="Stage_summary"
TEX_FILE="${TEX_BASENAME}.tex"

cd "${REPO_ROOT}"

latexmk -pdf -interaction=nonstopmode -halt-on-error "${TEX_FILE}"

rm -f \
  "${TEX_BASENAME}.aux" \
  "${TEX_BASENAME}.bbl" \
  "${TEX_BASENAME}.bcf" \
  "${TEX_BASENAME}.blg" \
  "${TEX_BASENAME}.fdb_latexmk" \
  "${TEX_BASENAME}.fls" \
  "${TEX_BASENAME}.lof" \
  "${TEX_BASENAME}.lot" \
  "${TEX_BASENAME}.log" \
  "${TEX_BASENAME}.out" \
  "${TEX_BASENAME}.run.xml" \
  "${TEX_BASENAME}.synctex.gz" \
  "${TEX_BASENAME}.toc"

echo "Generated ${TEX_BASENAME}.pdf and removed LaTeX temporary files."
