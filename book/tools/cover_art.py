"""The picture on the cover: step responses flown by the simulator, all closing on the set-point.

Every curve is a real flight from the book's lessons (book/data): the quadrotor is asked to climb
to its set-point and the altitude it actually flew is drawn, normalised so that the target is 1.
Some gains ring, some are late, one sample rate is too slow and the loop nearly loses the fight;
the orange line is what every one of them is steered towards. That is feedback, in one picture.

    python3 book/tools/cover_art.py   →   book/figures/out/cover_art.jpg
"""

from io import BytesIO
from pathlib import Path

import matplotlib

matplotlib.use("agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
from PIL import Image  # noqa: E402

BOOK = Path(__file__).resolve().parent.parent
DATA = BOOK / "data"
OUT = BOOK / "figures" / "out" / "cover_art.jpg"

NIGHT = "#0B0E13"   # the cover's black (colour `night` in the book style)
ACCENT = "#E8661A"  # the set-point, in the app's orange
# restless responses deep blue, calm ones pale: the measured-signal blue of the app (#3987E5)
CMAP = LinearSegmentedColormap.from_list("cover", ["#1F4E8C", "#3987E5", "#8DBDF2", "#E6EEF9"])

FLIGHTS = [
    "slow-3hz", "slow-4hz", "slow-5hz", "slow-7hz", "slow-10hz", "slow-50hz", "slow-250hz",
    "slow-delay200", "slow-delay170", "slow-delay150", "slow-delay130", "slow-delay100", "slow-delay50",
    "damper-kd0.8", "damper-kd3", "damper-kd6.3", "damper-kd15", "damper-kd30",
    "state-kp10-kd2", "state-kp10-kd6.3", "state-kp10-kd20", "state-kp20-kd8", "state-kp40-kd12.65",
    "noise-step-f1", "noise-step-f5", "noise-step-f20",
    "lqr-pd", "lqr-r0.1", "lqr-r1", "lqr-r10",
]
# the lessons above share one set-point schedule: from 8 s on, a square wave between 2.5 m and
# 1.5 m with a 4 s half-period. The window shows three of its edges; a flight whose set-point does
# not match the reference here is left out, so the orange line is every curve's true target.
REFERENCE = "slow-250hz"
T0, T1 = 7.0, 20.0


def load(name):
    d = pd.read_csv(DATA / f"{name}.csv.gz", usecols=["t", "pos.y", "sp.y"])
    d = d[(d.t >= T0) & (d.t <= T1)]
    return d.t.to_numpy(), d["pos.y"].to_numpy(), d["sp.y"].to_numpy()


def main():
    fig = plt.figure(figsize=(9, 6), dpi=240, facecolor=NIGHT)
    ax = fig.add_axes([0, 0, 1, 1], facecolor=NIGHT)
    ax.set_xlim(T0, T1)
    ax.set_ylim(0.55, 3.45)
    ax.axis("off")

    t_ref, _, s_ref = load(REFERENCE)
    flights = []
    for n in FLIGHTS:
        t, y, sp = load(n)
        if len(t) == len(t_ref) and np.mean(np.abs(sp - s_ref) > 0.01) < 0.02:
            flights.append((t, y, sp))
        else:
            print(f"left out {n}: its set-point differs")
    # restlessness of a flight: how far it strays from the set-point after the climb
    rest = np.array([np.abs(y - s)[t > T0 + 1].mean() for t, y, s in flights])
    rank = np.argsort(np.argsort(-rest)) / (len(flights) - 1)   # 0 = most restless
    for (t, y, _), r in sorted(zip(flights, rank), key=lambda p: p[1]):
        c = CMAP(0.15 + 0.85 * r)
        ax.plot(t, y, color=c, lw=4.0, alpha=0.07, solid_capstyle="round")   # glow
        ax.plot(t, y, color=c, lw=0.9, alpha=0.9, solid_capstyle="round")
    ax.plot(t_ref, s_ref, color=ACCENT, lw=6, alpha=0.12)
    ax.plot(t_ref, s_ref, color=ACCENT, lw=1.6)

    buf = BytesIO()
    fig.savefig(buf, format="png", facecolor=NIGHT)
    img = np.asarray(Image.open(buf).convert("RGB")).astype(float)
    # fade every edge into the cover's black, so the picture has no frame
    h, w, _ = img.shape
    fx = np.clip(np.minimum(np.arange(w), np.arange(w)[::-1]) / (0.14 * w), 0, 1)
    fy = np.clip(np.minimum(np.arange(h), np.arange(h)[::-1]) / (0.16 * h), 0, 1)
    fade = (np.sin(np.pi / 2 * np.outer(fy, fx)) ** 2)[..., None]
    night = np.array([int(NIGHT[i:i + 2], 16) for i in (1, 3, 5)], float)
    img = night + (img - night) * fade
    Image.fromarray(img.round().astype(np.uint8)).save(OUT, quality=90, optimize=True)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
