"""Crop the portraits of Chapter 0 to one frame (25 mm by 31 mm) so that they stand in a row.

Usage: python3 book/tools/portraits.py
Reads book/photos/<key>.jpg and writes book/photos/portrait-<key>.jpg.
"""
from pathlib import Path

from PIL import Image

root = Path(__file__).resolve().parent.parent / "photos"
W, H = 500, 620  # 25 : 31
# horizontal and vertical centre of the crop, as fractions of the picture
FOCUS = {"ctesibius": (0.5, 0.5), "maxwell": (0.5, 0.42), "bode": (0.5, 0.4), "wiener": (0.5, 0.4),
         "pontryagin": (0.42, 0.45), "kalman": (0.55, 0.45)}
for key, (fx, fy) in FOCUS.items():
    im = Image.open(root / f"{key}.jpg").convert("RGB")
    scale = max(W / im.width, H / im.height)
    im = im.resize((round(im.width * scale), round(im.height * scale)), Image.LANCZOS)
    left = min(max(round(im.width * fx - W / 2), 0), im.width - W)
    top = min(max(round(im.height * fy - H / 2), 0), im.height - H)
    im.crop((left, top, left + W, top + H)).save(root / f"portrait-{key}.jpg", quality=88)
    print(key, im.size)
