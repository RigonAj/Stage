# Compilation du rapport LaTeX

Le rapport principal est le fichier `Stage_summary.tex`, place a la racine du depot. Il se compile en PDF avec `pdflatex`; le plus simple est d'utiliser `latexmk`, qui relance automatiquement LaTeX autant de fois que necessaire pour mettre a jour la table des matieres, les figures, les references et la bibliographie manuelle.

## Commande a privilegier

Depuis un terminal ou `env.sh` a ete source:

```bash
compile-report
```

Cet alias lance `scripts/compile_stage_summary.sh`. Il compile le rapport, genere:

```text
Stage_summary.pdf
```

Puis il supprime uniquement les fichiers temporaires LaTeX associes a `Stage_summary.tex`.

Pour charger l'alias dans un terminal:

```bash
source env.sh
```

Le document utilise une bibliographie integree avec `thebibliography`, donc il n'y a pas de commande `bibtex` ou `biber` a lancer.

## Commande directe

Le script peut aussi etre lance directement depuis n'importe quel dossier:

```bash
./scripts/compile_stage_summary.sh
```

En interne, il utilise:

```bash
latexmk -pdf -interaction=nonstopmode -halt-on-error Stage_summary.tex
```

## Nettoyage des fichiers temporaires

Le nettoyage est deja fait automatiquement par `compile-report`.

Pour supprimer manuellement les fichiers auxiliaires produits par LaTeX tout en gardant le PDF, utiliser de preference le meme nettoyage explicite que le script:

```bash
rm -f Stage_summary.aux Stage_summary.bbl Stage_summary.bcf Stage_summary.blg \
  Stage_summary.fdb_latexmk Stage_summary.fls Stage_summary.lof \
  Stage_summary.lot Stage_summary.log Stage_summary.out \
  Stage_summary.run.xml Stage_summary.synctex.gz Stage_summary.toc
```

Pour supprimer le PDF genere:

```bash
rm -f Stage_summary.pdf
```

Les fichiers temporaires LaTeX sont ignores par Git dans `.gitignore` (`*.aux`, `*.fdb_latexmk`, `*.fls`, `*.log`, `*.out`, `*.toc`, etc.).

## Alternative sans latexmk

Si `latexmk` n'est pas disponible, lancer `pdflatex` plusieurs fois depuis la racine du depot:

```bash
pdflatex -interaction=nonstopmode -halt-on-error Stage_summary.tex
pdflatex -interaction=nonstopmode -halt-on-error Stage_summary.tex
pdflatex -interaction=nonstopmode -halt-on-error Stage_summary.tex
```

Les passes multiples sont necessaires pour stabiliser la table des matieres, les references croisees et les citations.

## Dependances LaTeX utilisees

Le document a ete compile avec TeX Live 2026 et utilise notamment:

- `babel` avec l'option `french`
- `lmodern`
- `geometry`
- `graphicx`
- `float`
- `amsmath`
- `booktabs`
- `array`
- `tikz`
- `hyperref`

Les images et PDF inclus doivent rester accessibles depuis la racine du depot, car les chemins du fichier `.tex` sont relatifs a ce dossier.
