# Anemostatos — the book

The course book that goes with the simulator. One chapter per lesson, with
the same numbers and titles as the lesson panel of the app. The plan of the
whole course, the standard a chapter must meet and the current state of
this volume are in [docs/curriculum.md](../docs/curriculum.md).

Idea and editing: Wojciech Stelmaszewski. Text, figures and code: Claude
Opus 5.5.

## Build

```sh
make book           # LuaLaTeX, three passes with the index between them → book/anemostatos.pdf
```

It needs a TeX distribution with LuaLaTeX, `upmendex` and the Libertinus and
Fira fonts (TeX Live has all three). The index is sorted by `upmendex` with
`style/index.ist` after the first and the second pass; the third pass prints
it with the page numbers of the second, which are final. The build takes
one to two minutes per pass. The
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
| Crop the portraits   | `python3 book/tools/portraits.py`            | `book/photos/`       | `book/photos/portrait-*.jpg`             |

Both scripts accept a name filter when run directly, for example
`python3 book/tools/make_figures.py windup`. The Python tools need
matplotlib, numpy, pandas and Pillow.

## Layout

```
book/
  main.tex        the document: front matter, parts, appendices
  style/          the design system (anemostatos-book.sty): colours, boxes, chapter openers;
                  the index style (index.ist) and the index forms of terms (index-terms.tex)
  front/          cover, title page, contents, technical preface, Chapter 0, Prelude
  part1/          Part I, one file per lesson
  part2/          Part II, one file per chapter (A–E)
  back/           mathematical toolbox (math.tex and one file per appendix in toolbox/),
                  app map, reading list, photo credits, index, colophon
  tikz/           diagrams
  figures/out/    plots drawn from simulator data
  screens/        screenshots of the app
  photos/         photographs for the Out in the World pages, with their credits
  tools/          the scripts above
```

## Conventions

- Every time series is flown by the simulator. No plot is drawn from a
  formula alone unless the caption says so.
- Every chapter opens with a quotation from a mathematician or physicist,
  kept in `front/quotes.tex` under the chapter's label (`\chapterquote`).
  A new lesson needs one. Only real quotations, checked word for word against
  the source text (a scan or the full text of the edition named), with the
  source given; translations say so. Famous lines are often misquoted.
- Signal colours are the app's colours (`style/anemostatos-book.sty`).
- A number attributed to the simulator comes from an experiment in
  `scripts/book-data.ts`. Figures that draw a formula over a flight live in
  `tools/figs_predict.py`.
- A chapter follows the standard of `docs/curriculum.md` §4. Lessons 1–4
  are the model: the section _The engineer's view_, the `lookahead` box
  and exercises labelled `\exkind{derive}`, `\exkind{fly}`, `\exkind{code}`.
- General claims are stated in `theorem` environments (numbered per lesson)
  with a `proof` or a cited source; sources go into `back/reading.tex`.
  Symbols follow the Prelude's notation table: `\vx` for a state vector,
  `\Id` for the identity matrix, `J` for inertia, `I` for the integral term.
- Layout rules live in the style, not in the chapters: boxes keep their title with
  their first lines (`\keeptogether`, never `\needspace`, which leaves half-empty
  pages under `\raggedbottom`); figures carry their caption in the margin
  (`\simfig`, `\sidecapfig`); the _Out in the World_ story is a float that takes
  the head of a page; short boxes and single exercises are never split.
  After a change, look at the pages: `make book` and check for short pages.
- The margin column is used on purpose: every lesson opens with a `\setupcard` beside
  its lead (what is switched on, what happens when), exercise kinds stand there
  (`\exkind`), and so do figure captions, small figures and notes. `Try this` and
  `In Anemostatos` boxes are never split; the story (`curiosity`) follows the summary box.
- Margin notes come in five kinds, each with its own mark and colour: `\tip` (a working
  rule), `\pitfall` (a mistake people make), `\didyouknow` (history, other fields),
  `\recall` (a fact from earlier, needed here) and `\innumbers` (an order of magnitude);
  `\aside` stays for a neutral remark. Aim at about one per page, put it right after the
  sentence it belongs to, and keep it to five or six short lines. In running text a note
  is a `\marginpar` and finds its own place. Inside a box it is a `\marginnote` and cannot
  see its neighbours: keep such notes to `example` boxes, and after a build run
  `python3 book/tools/check_margins.py`; a collision is fixed with the optional shift,
  `\tip[-30pt]{...}`, or by moving the anchor.
- The toolbox is a short course, not a formula sheet (`docs/curriculum.md` §5.2). An
  appendix opens with `\toolchapter{letter}{title}{subtitle}` and a `\usedcard` that
  lists the lessons using it; its equations, theorems and worked examples are numbered
  with its letter; it ends with an _at a glance_ table and has no exercises. Its figures
  are drawn from formulas in `tools/figs_math.py` (names `tb_*`), and their captions
  say so.
- Exercises use `\exercise`, and their solutions go into `\answer{...}`
  right after them. The build collects the answers into
  `build/main.ans` for the companion volume; they are not printed here.
- The index is made from two commands. `\term{...}` (a defined term: bold) also makes an
  index entry, in the form the text uses; where the index form differs everywhere — a
  plural, an adjective, a capital — map it once in `style/index-terms.tex`
  (`\termindex{poles}{pole}`), and where it differs at one place give it there,
  `\term[convex function]{convex}`. `\index{...}` marks a method, a person or an idea
  that is discussed but not defined in a box: a person as `Surname, Given name`, a
  subentry after `!` (`\index{PID controller!integral term}`), a page range with `|(` and
  `|)` (as for a whole section). A Part II lesson that is one method from start to end is
  input as `\indexedlesson{part2/04-kalman}{Kalman filter}`, which indexes the lesson as a
  page range. Entries are lower case except names and abbreviations, singular, and list
  the main discussion rather than every mention; abbreviations point to the full name
  with `|see{...}`, collected in `back/index.tex`.
- After a build, `book/build/main.log` lists undefined references. There
  should be none once Part II and the appendices are written.
