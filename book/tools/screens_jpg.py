"""Convert the raw app screenshots (screens/raw/*.png, from shots.mjs) to compact JPEGs for the book."""
from pathlib import Path

from PIL import Image

root = Path(__file__).resolve().parent.parent / "screens"
for png in sorted((root / "raw").glob("*.png")):
    out = root / f"{png.stem}.jpg"
    if out.exists() and out.stat().st_mtime > png.stat().st_mtime:
        continue
    im = Image.open(png).convert("RGB")
    im = im.resize((2400, round(im.height * 2400 / im.width)), Image.LANCZOS)
    im.save(out, quality=86, optimize=True, progressive=True)
    print(out.name)
