"""The rosette on the cover: phase portraits of a damped oscillator, the picture of feedback at work.

Every curve is the state (position, velocity) of a mass on a spring with a damper, released far
from rest and spiralling into the origin, where the controller wants it. One petal is a fan of such
trajectories (several damping ratios, several release points) together with its mirror image; the
petal is repeated eight times around the centre, like the pattern in a kaleidoscope.

    python3 book/tools/cover_rosette.py   →   book/figures/out/cover_rosette.jpg
"""

from io import BytesIO
from pathlib import Path

import matplotlib

matplotlib.use("agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
from PIL import Image  # noqa: E402

OUT = Path(__file__).resolve().parent.parent / "figures" / "out" / "cover_rosette.jpg"

PETALS = 8
NIGHT = "#0B0E13"  # the cover's black (colour `night` in the book style), so no transparency is needed
# from the rim to the centre: deep indigo, teal, sea green, pale olive, and the app's orange at rest
CMAP = LinearSegmentedColormap.from_list(
    "rosette", ["#2B2F5E", "#2E6F8E", "#3FA7A0", "#8CC9A0", "#D8E3A0", "#F2B24C", "#E8661A"])


def trajectory(zeta, phase, t_end=14.0, n=1400):
    """Free response of x'' + 2ζx' + x = 0 from unit amplitude, drawn in the (x, x') plane."""
    t = np.linspace(0.0, t_end, n)
    wd = np.sqrt(1.0 - zeta**2)
    x = np.exp(-zeta * t) * np.cos(wd * t + phase)
    v = np.gradient(x, t)
    return x, v


def envelope(th):
    """The outline of a petal: the largest radius allowed at angle th (0 = the petal's axis)."""
    return 0.74 + 0.26 * ((1 + np.cos(th * PETALS)) / 2) ** 0.5


def segments(x, y):
    pts = np.column_stack([x, y]).reshape(-1, 1, 2)
    return np.concatenate([pts[:-1], pts[1:]], axis=1)


def main():
    fig = plt.figure(figsize=(8, 8), dpi=240, facecolor=NIGHT)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(-1.08, 1.08)
    ax.set_ylim(-1.08, 1.08)
    ax.set_aspect("equal")
    ax.axis("off")

    wedge = np.pi / PETALS  # half a petal: the kaleidoscope's mirror angle
    segs, vals, widths = [], [], []
    # each family spirals into its own point of rest, set inside the wedge at a different radius
    for cx, cy, amp in [(0.30, 0.10, 0.30), (0.62, 0.16, 0.34), (0.86, 0.09, 0.20), (0.48, 0.30, 0.18), (0.10, 0.025, 0.07)]:
        for zeta in np.linspace(0.08, 0.32, 6):
            for phase in np.linspace(0.0, 2 * np.pi, 5, endpoint=False):
                x, v = trajectory(zeta, phase)
                px, py = cx + amp * x, cy + amp * 0.85 * v
                r = np.hypot(px, py)
                th = np.mod(np.arctan2(py, px), 2 * wedge)
                th = np.where(th > wedge, 2 * wedge - th, th)  # fold by reflection
                keep = r < envelope(th)
                r, th = r[keep], th[keep]
                for mirror in (1, -1):
                    for k in range(PETALS):
                        a = mirror * th + 2 * wedge * k
                        s = segments(r * np.cos(a), r * np.sin(a))
                        # drop segments that jump across a fold
                        ok = np.hypot(*(s[:, 1] - s[:, 0]).T) < 0.05
                        segs.append(s[ok])
                        vals.append(np.linspace(0, 1, len(s))[ok] ** 0.6)
                        widths.append(np.full(ok.sum(), 0.25 + 0.9 * zeta))
    lc = LineCollection(np.concatenate(segs), cmap=CMAP, alpha=0.6,
                        linewidths=np.concatenate(widths), capstyle="round")
    lc.set_array(np.concatenate(vals))
    lc.set_clim(0.0, 1.0)
    ax.add_collection(lc)
    # the rim: a few close outlines of the eight petals, as in a printed ornament
    th = np.linspace(0, 2 * np.pi, 4000)
    fold = np.abs(np.mod(th + wedge, 2 * wedge) - wedge)
    for k, c in enumerate(["#2B2F5E", "#34407A", "#2E6F8E"]):
        rr = envelope(fold) * (1.0 - 0.012 * k)
        ax.plot(rr * np.cos(th), rr * np.sin(th), color=c, lw=1.1 - 0.3 * k, alpha=0.9)
    buf = BytesIO()
    fig.savefig(buf, format="png", facecolor=NIGHT)
    Image.open(buf).convert("RGB").save(OUT, quality=90, optimize=True)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
