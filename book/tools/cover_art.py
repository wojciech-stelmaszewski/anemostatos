"""The picture on the cover: the quadrotor standing in the wind, from a real flight of the simulator.

The flight is lesson I.13 (book/data/cascade.csv.gz): a steady 5 m/s wind blows from the left, the
quadrotor takes off, is carried downwind, and the four-loop cascade brings it back over its
set-point at 2 m, where it stays, leaning into the wind. The trail is the path it actually flew;
the faint silhouettes are drawn every 0.35 m of the path, at the position and pitch recorded, like
a stroboscope photograph. The orange cross is the set-point. Seen from the side; the path is to
scale, the drone is drawn 1.6 times its size so that it reads on the cover. The window shows the
end of the climb, the return against the wind and the hover.

    python3 book/tools/cover_art.py   →   book/figures/out/cover_art.jpg
"""

from io import BytesIO
from pathlib import Path

import matplotlib

matplotlib.use("agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from matplotlib.patches import Ellipse, FancyBboxPatch, Rectangle  # noqa: E402
from matplotlib.transforms import Affine2D  # noqa: E402
from PIL import Image  # noqa: E402

BOOK = Path(__file__).resolve().parent.parent
FLIGHT = BOOK / "data" / "cascade.csv.gz"
OUT = BOOK / "figures" / "out" / "cover_art.jpg"

NIGHT = "#0B0E13"   # the cover's black (colour `night` in the book style)
ACCENT = "#E8661A"  # the app's orange: the drone's arms and the set-point
WIND = "#8DBDF2"    # pale measured-signal blue
INK = "#E6E9EF"     # the night text colour

X0, X1, Y0, Y1 = -0.85, 1.95, 0.45, 2.65
SIZE = 1.6          # the drone, enlarged against the path
T_END = 15.0        # the take-off, the drift and the return; by now it hovers over the set-point


def drone(ax, x, y, pitch_deg, alpha, lw, body=INK, arms=ACCENT, props=True, z=3):
    """A quadrotor seen from the side, at true size (a 0.34 m frame): body, arms, motors, discs, legs."""
    tr = Affine2D().scale(SIZE).rotate_deg(pitch_deg).translate(x, y) + ax.transData
    parts = [
        FancyBboxPatch((-0.055, -0.022), 0.11, 0.044, boxstyle="round,pad=0,rounding_size=0.016",
                       fc=body, ec="none"),
        Rectangle((-0.17, -0.006), 0.34, 0.012, fc=arms, ec="none"),
        Rectangle((-0.182, -0.004), 0.024, 0.038, fc=body, ec="none"),
        Rectangle((0.158, -0.004), 0.024, 0.038, fc=body, ec="none"),
    ]
    if props:
        parts += [Ellipse((-0.17, 0.040), 0.17, 0.012, fc=body, ec="none", alpha=0.45),
                  Ellipse((0.17, 0.040), 0.17, 0.012, fc=body, ec="none", alpha=0.45)]
    for p in parts:
        p.set_transform(tr)
        p.set_alpha(alpha * (p.get_alpha() or 1.0))
        p.set_zorder(z)
        ax.add_patch(p)
    for sx in (-1, 1):  # landing legs
        ax.plot([sx * 0.04, sx * 0.075], [-0.022, -0.06], color=body, lw=lw, alpha=alpha,
                transform=tr, zorder=z, solid_capstyle="round")


def main():
    d = pd.read_csv(FLIGHT, usecols=["t", "pos.x", "pos.y", "pitch", "sp.x", "sp.y", "wind.x"])
    d = d[d.t <= T_END]
    t = d.t.to_numpy()
    x, y, pitch = d["pos.x"].to_numpy(), d["pos.y"].to_numpy(), d["pitch"].to_numpy()
    spx, spy = d["sp.x"].iloc[-1], d["sp.y"].iloc[-1]

    w_in = 9.0
    fig = plt.figure(figsize=(w_in, w_in * (Y1 - Y0) / (X1 - X0)), dpi=240, facecolor=NIGHT)
    ax = fig.add_axes([0, 0, 1, 1], facecolor=NIGHT)
    ax.set_xlim(X0, X1)
    ax.set_ylim(Y0, Y1)
    ax.set_aspect("equal")
    ax.axis("off")

    # the wind: streamlines from the left, faint, each with a slow wobble of its own
    rng = np.random.default_rng(7)
    xs = np.linspace(X0, X1, 600)
    for yl in np.linspace(0.6, 2.55, 11):
        ph, a = rng.uniform(0, 2 * np.pi), rng.uniform(0.01, 0.03)
        yy = yl + a * np.sin(2.2 * xs + ph)
        start = rng.uniform(X0, X0 + 1.0)
        seg = xs > start
        ax.plot(xs[seg], yy[seg], color=WIND, lw=0.7, alpha=0.20, zorder=1)
        k = np.searchsorted(xs, start + rng.uniform(0.25, 0.6))
        ax.annotate("", xy=(xs[k + 6], yy[k + 6]), xytext=(xs[k], yy[k]), zorder=1,
                    arrowprops=dict(arrowstyle="-|>,head_length=0.5,head_width=0.22", color=WIND,
                                    lw=0.7, alpha=0.35))

    # the ground and the pad it took off from
    ax.plot([X0, X1], [0, 0], color=INK, lw=0.6, alpha=0.18, zorder=1)
    ax.plot([-0.12, 0.12], [0, 0], color=ACCENT, lw=1.6, alpha=0.6, zorder=1)

    # the set-point: an orange cross with a thin ring
    r = 0.30
    ax.plot([spx - r, spx + r], [spy, spy], color=ACCENT, lw=0.9, alpha=0.8, zorder=2)
    ax.plot([spx, spx], [spy - r, spy + r], color=ACCENT, lw=0.9, alpha=0.8, zorder=2)
    ax.add_patch(plt.Circle((spx, spy), 0.20, fc="none", ec=ACCENT, lw=0.9, alpha=0.8, zorder=2))

    # the trail it flew, brightening with time
    n = len(x)
    for i in range(0, n - 1, 4):
        f = i / n
        ax.plot(x[i:i + 5], y[i:i + 5], color=WIND, lw=1.5, alpha=0.35 + 0.6 * f, zorder=2,
                solid_capstyle="round")

    # the stroboscope: a silhouette every 0.35 m of the path, at the recorded position and pitch
    path = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    for s in np.arange(path[-1] - 0.50, 0.0, -0.35)[::-1]:
        i = int(np.searchsorted(path, s))
        drone(ax, x[i], y[i], pitch[i], alpha=0.22 + 0.30 * s / path[-1], lw=1.0, body=WIND,
              arms=WIND, props=False, z=3)
    drone(ax, x[-1], y[-1], pitch[-1], alpha=1.0, lw=1.6, z=5)

    buf = BytesIO()
    fig.savefig(buf, format="png", facecolor=NIGHT)
    img = np.asarray(Image.open(buf).convert("RGB")).astype(float)
    # fade the left, right and top edges into the cover's black, so the picture has no frame
    h, w, _ = img.shape
    fx = np.clip(np.minimum(np.arange(w), np.arange(w)[::-1]) / (0.12 * w), 0, 1)
    fy = np.minimum(np.clip(np.arange(h) / (0.10 * h), 0, 1),
                    np.clip(np.arange(h)[::-1] / (0.30 * h), 0, 1))
    fade = (np.sin(np.pi / 2 * np.outer(fy, fx)) ** 2)[..., None]
    night = np.array([int(NIGHT[i:i + 2], 16) for i in (1, 3, 5)], float)
    img = night + (img - night) * fade
    Image.fromarray(img.round().astype(np.uint8)).save(OUT, quality=90, optimize=True)
    print(f"wrote {OUT}  (final pitch {pitch[-1]:.1f} deg, wind {d['wind.x'].mean():.1f} m/s)")


if __name__ == "__main__":
    main()
