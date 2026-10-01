"""Export the flight drawn on the cover (lesson I.13, book/data/cascade.csv.gz) for the 3D render.

Writes book/tools/cover/trail.json: the side-view path (x, altitude) and pitch of the take-off,
the drift downwind and the return over the set-point, every 20 ms, relative to the final hover.
"""

import json
from pathlib import Path

import pandas as pd

HERE = Path(__file__).resolve().parent
d = pd.read_csv(HERE.parent.parent / "data" / "cascade.csv.gz",
                usecols=["t", "pos.x", "pos.y", "pitch", "wind.x"])
d = d[d.t <= 15.0].iloc[::4]
end = d.iloc[-1]
out = {
    "source": "book/data/cascade.csv.gz (lesson I.13), t <= 15 s",
    "wind": round(float(d["wind.x"].mean()), 2),
    "hoverPitchDeg": round(float(end["pitch"]), 2),
    "t": [round(v, 3) for v in d.t],
    "x": [round(v - end["pos.x"], 4) for v in d["pos.x"]],
    "y": [round(v - end["pos.y"], 4) for v in d["pos.y"]],
    "pitch": [round(v, 2) for v in d["pitch"]],
}
(HERE / "trail.json").write_text(json.dumps(out, separators=(",", ":")))
print(f"wrote {HERE / 'trail.json'}: {len(out['t'])} samples, hover pitch {out['hoverPitchDeg']} deg")
