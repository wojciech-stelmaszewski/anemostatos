# Standing in the Wind — the book

The course book that goes with the simulator. One chapter per lesson, with
the same numbers and titles as the lesson panel of the app. The plan of the
whole course, the standard a chapter must meet and the current state of
this volume are in [docs/curriculum.md](../docs/curriculum.md).

Idea and editing: Wojciech Stelmaszewski. Text, figures and code: Claude
Opus 5.5.

## Build

```sh
make book           # LuaLaTeX, two passes → book/standing-in-the-wind-preview.pdf
```

It needs a TeX distribution with LuaLaTeX and the Libertinus and Fira
fonts (TeX Live has both). The build takes about ten seconds per pass. The
figures, screenshots and photographs are committed, so nothing else has to
run first.

## Regenerate the material

Only needed when a lesson, the physics or a figure changes.

| Step                 | Command                                      | Reads                | Writes                                   |
| -------------------- | -------------------------------------------- | -------------------- | ---------------------------------------- |
| Fly the experiments  | `make book-data`                             | the simulator's code | `book/data/*.csv.gz` (not committed)     |
| Draw the figures     | `make book-figures`                          | `book/data/`         | `book/figures/out/*.pdf`                 |
| Take the screenshots | `make dev`, then `node book/tools/shots.mjs` | the running app      | `book/screens/raw/*.png` (not committed) |
| Compress them        | `python3 book/tools/screens_jpg.py`          | `book/screens/raw/`  | `book/screens/*.jpg`                     |
| Fetch the photos     | `python3 book/tools/photos.py`               | Wikimedia Commons    | `book/photos/*.jpg`, `credits.tex`       |

Both scripts accept a name filter when run directly, for example
`python3 book/tools/make_figures.py windup`. The Python tools need
matplotlib, numpy, pandas and Pillow.

## Layout

```
book/
  main.tex        the document: front matter, parts, appendices
  style/          the design system (anemostatos-book.sty): colours, boxes, chapter openers
  front/          cover, title page, contents, preface, Chapter 0, Prelude
  part1/          Part I, one file per lesson
  part2/          Part II, one file per chapter (A–E)
  back/           mathematical toolbox, app map, reading list, photo credits, colophon
  tikz/           diagrams
  figures/out/    plots drawn from simulator data
  screens/        screenshots of the app
  photos/         photographs for the Out in the World pages, with their credits
  tools/          the scripts above
```

## Conventions

- Every time series is flown by the simulator. No plot is drawn from a
  formula alone unless the caption says so.
- Signal colours are the app's colours (`style/anemostatos-book.sty`).
- Exercises use `\exercise`, and their solutions go into `\answer{...}`
  right after them. The build collects the answers into
  `build/main.ans` for the companion volume; they are not printed here.
- After a build, `book/build/main.log` lists undefined references. There
  should be none once Part II and the appendices are written.
