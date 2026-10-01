"""Finish the cover render for the book: fade its edges into the cover's black and save a JPEG.

    python3 book/tools/cover/finish.py   (book/tools/cover/render.png → book/figures/out/cover_art.jpg)

The picture runs full bleed behind the title and the name, so its top and bottom dissolve into
the page; a soft vignette keeps the eye on the drone.
"""

from pathlib import Path

import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
OUT = HERE.parent.parent / "figures" / "out" / "cover_art.jpg"
NIGHT = np.array([0x0B, 0x0E, 0x13], float)

img = np.asarray(Image.open(HERE / "render.png").convert("RGB")).astype(float)
h, w, _ = img.shape
y = np.linspace(0, 1, h)[:, None]
x = np.linspace(0, 1, w)[None, :]


def ease(t):
    t = np.clip(t, 0, 1)
    return t * t * (3 - 2 * t)


fade = ease(y / 0.22) * ease((1 - y) / 0.30)                       # top and bottom
vignette = 1 - 0.35 * ease(np.hypot((x - 0.40) / 0.75, (y - 0.45) / 0.75))
k = (fade * vignette)[..., None]
img = NIGHT + (img - NIGHT) * k
Image.fromarray(img.round().astype(np.uint8)).save(OUT, quality=92, optimize=True)
print(f"wrote {OUT} ({w}×{h})")
