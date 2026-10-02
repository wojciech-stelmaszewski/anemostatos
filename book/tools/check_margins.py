r"""Check the margin column of the built book.

  python3 book/tools/check_margins.py [book/build/main.pdf]        (needs pymupdf)

Reports margin notes (\tip, \pitfall, \didyouknow, \recall, \innumbers) that run off the foot of
the page or collide with another note, a caption, a drawing or an image. Notes inside boxes are
\marginnote and cannot see their neighbours: move such a note with its optional argument,
e.g. \tip[-30pt]{...}, or move its anchor.
"""
import sys

import pymupdf

d = pymupdf.open(sys.argv[1] if len(sys.argv) > 1 else "book/build/main.pdf")
X0, YB = 415, 755  # left edge of the margin column, foot of the text block [pt]
LAB = {"TIP","PITFALL","DID YOU KNOW","RECALL","IN NUMBERS"}
def inter(a,b):
    w=min(a[2],b[2])-max(a[0],b[0]); h=min(a[3],b[3])-max(a[1],b[1])
    return w>1 and h>1
tot=0
for i, pg in enumerate(d):
    L = []
    for b in pg.get_text("dict")["blocks"]:
        for l in b.get("lines", []):
            x0, y0, x1, y1 = l["bbox"]
            t = "".join(s["text"] for s in l["spans"]).strip()
            if x0 >= X0 and t and y0 > 60: L.append([x0, y0, x1, y1, t])
    L.sort(key=lambda a:(a[1],a[0]))
    labs=[a for a in L if a[4] in LAB]
    notes=[]
    for a in labs:
        # note spans from its rule (6pt above label) down through lines until a vertical gap > 7pt
        top=a[1]-6; bot=a[3]
        for b in L:
            if b is a or b[1] < a[1]: continue
            if b[1]-bot < 7 and b[0] < a[0]+110: bot=max(bot,b[3])
        notes.append((a[0]-1, top, a[0]+111, bot, a[4]))
    tot+=len(notes)
    lab=pg.get_label()
    for n in notes:
        if n[3] > YB: print(f"p{i+1} ({lab}) BOTTOM {n[3]:.0f} {n[4]}")
    for j,n in enumerate(notes):
        for m in notes[j+1:]:
            if inter(n,m): print(f"p{i+1} ({lab}) NOTE/NOTE {n[4]} / {m[4]}")
        # drawings and images
        for dr in pg.get_drawings():
            r=dr["rect"]; rr=(r.x0,r.y0,r.x1,r.y1)
            if rr[1] >= n[1]-2 and rr[3] <= n[1]+16: continue   # its own rule and mark
            if (rr[2]-rr[0])*(rr[3]-rr[1]) < 6 and rr[0] >= n[0]-2: continue  # fraction bars, radicals
            if rr[2]-rr[0] > 500 and rr[3]-rr[1] > 700: continue
            if inter(n, rr): print(f"p{i+1} ({lab}) NOTE/DRAWING {n[4]} note y {n[1]:.0f}-{n[3]:.0f} drawing {tuple(round(v) for v in rr)}"); break
        for im in pg.get_image_info():
            if im["bbox"][0] < 0: continue  # a clipped crop: its reported box is not what is seen
            if inter(n, im["bbox"]): print(f"p{i+1} ({lab}) NOTE/IMAGE {n[4]}")
    # any two margin lines from different notes / captions overlapping
    for j, a in enumerate(L):
        for b in L[j+1:]:
            ov = min(a[3], b[3]) - max(a[1], b[1]); oh = min(a[2], b[2]) - max(a[0], b[0])
            if ov > 0.55 * min(a[3]-a[1], b[3]-b[1]) and oh > 2 and abs(a[1]-b[1]) > 3.5:
                print(f"p{i+1} ({lab}) TEXT '{a[4][:30]}' / '{b[4][:30]}'")
    # anything in the page body that runs below the foot of the text block: a drawing, an image
    # or a line of text that did not fit (a TikZ figure or a box placed with [h] near the foot)
    if any(dr["rect"].width > 500 and dr["rect"].height > 700 for dr in pg.get_drawings()):
        continue  # a full-page design: the cover, a part page
    low = []
    for b in pg.get_text("dict")["blocks"]:
        for l in b.get("lines", []):
            t = "".join(s["text"] for s in l["spans"]).strip()
            if l["bbox"][3] > YB + 9 and t and not t.isdigit() and not t.lower().strip("ivxlc") == "":
                low.append(f"text '{t[:30]}'")
    for dr in pg.get_drawings():
        r = dr["rect"]
        if r.y1 > YB + 8 and r.y0 > 100 and r.width > 1: low.append(f"drawing {tuple(round(v) for v in r)}")
    for im in pg.get_image_info():
        # a clipped crop (\marginscreen) reports the whole uncropped image, which starts off the page
        if im["bbox"][3] > YB + 4 and im["bbox"][0] >= 0: low.append("image")
    for what in dict.fromkeys(low):
        print(f"p{i+1} ({lab}) BELOW FOOT {what}")
print("notes:", tot)
