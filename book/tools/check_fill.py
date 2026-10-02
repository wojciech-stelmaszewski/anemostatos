r"""Report how full each page of some lessons is, to find short pages.

  python3 book/tools/check_fill.py [book/build/main.pdf] [II.15 II.16 ...]     (needs pymupdf)

For every page of each lesson, prints how far down the text column the lowest text, image or
drawing reaches, as a percentage of the text block, and marks pages under 80 % (a lesson's last
page may be short). With no lesson labels it checks every lesson of the book. A short page is
usually an unsplittable box that does not fit under a float: move the float after the box.
"""
import sys

import pymupdf

TOP, FOOT, COLUMN = 70.0, 755.0, 410.0  # top and foot of the text block, right edge of the column [pt]

args = sys.argv[1:]
pdf = args.pop(0) if args and args[0].endswith('.pdf') else 'book/build/main.pdf'
d = pymupdf.open(pdf)
toc = [(t, p) for lvl, t, p in d.get_toc() if lvl == 2]
lessons = args or [t.split()[0] for t, _ in toc if t[:1] in 'I' and '.' in t.split()[0]]
for les in lessons:
    hits = [k for k, (t, _) in enumerate(toc) if t.startswith(les + ' ')]
    if not hits:
        print(f'{les}: not found')
        continue
    i = hits[0]
    a, b = toc[i][1], (toc[i + 1][1] - 1 if i + 1 < len(toc) else d.page_count)
    print(f'{toc[i][0]}  (pp {a}-{b})')
    for pn in range(a, b + 1):
        pg = d[pn - 1]
        ys = [bl[3] for bl in pg.get_text('blocks') if bl[0] < COLUMN and bl[1] > 55 and bl[3] < 780]
        ys += [im['bbox'][3] for im in pg.get_image_info() if im['bbox'][0] < COLUMN and im['bbox'][3] < 780]
        ys += [dr['rect'].y1 for dr in pg.get_drawings()
               if dr['rect'].x0 < COLUMN and dr['rect'].y0 > 55 and dr['rect'].y1 < 780]
        fill = ((max(ys) if ys else TOP) - TOP) / (FOOT - TOP) * 100
        flag = '  (last)' if pn == b else ('  <-- SHORT' if fill < 80 else '')
        print(f'  p{pn}: fill {fill:5.1f}%{flag}')
