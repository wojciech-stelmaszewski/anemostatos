"""The art on the back cover: the wind upstream of the emblem on the front.

Unfold the cover and the back lies to the left of the front, its right edge at the spine. The wind
of the front emblem comes in from the left, so on the back it is still on its way: the same
potential flow past the same cylinder, as if the loop stood just beyond the spine. On the left the
wind is plain; towards the spine it slows and parts for the loop it has not reached yet. The
dividing streamline is the reference r, travelling towards the summing junction at the height it
has on the front.

    python3 book/tools/back_field.py   →   book/figures/out/back_field.pdf
"""

import numpy as np

from cover_emblem import INK, WIND, A, MM
from figlib import OUT, plt

W, H = 210.0, 94.0       # mm: the full width of the page, the band of the back it fills
CY = H / 2               # the streamline, at the centre of the band
CX = W + A + 24          # the loop, just beyond the spine


def main():
    fig = plt.figure(figsize=(W * MM, H * MM))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.patch.set_alpha(0)
    ax.patch.set_alpha(0)

    # the same staggered grid, the same arrows, the same flow as on the front
    step = 5.4
    pts = []
    for j, yy in enumerate(np.arange(-2, H + 2, step * np.sqrt(3) / 2)):
        for xx in np.arange(-2 + (step / 2) * (j % 2), W + 2, step):
            pts.append((xx, yy))
    P = np.array(pts)
    P = P[np.abs(P[:, 1] - CY) >= 2.4]           # the streamline runs through a clear lane
    P = P[~((np.abs(P[:, 0] - (W - 14)) < 6) & (P[:, 1] > CY) & (P[:, 1] < CY + 10))]   # and r sits clear
    z = (P[:, 0] - CX) + 1j * (P[:, 1] - CY)
    w = 1 - A**2 / z**2
    u, v = w.real, -w.imag
    length = 3.1
    # quiet on the left where the wind is plain, a little stronger towards the spine
    alpha = 0.16 + 0.50 * (P[:, 0] / W) ** 2
    edge = np.clip(np.minimum(P[:, 1], H - P[:, 1]) / 22, 0, 1)
    alpha *= edge * edge * (3 - 2 * edge)
    rgba = np.zeros((len(P), 4))
    rgba[:, :3] = [int(WIND[i:i + 2], 16) / 255 for i in (1, 3, 5)]
    rgba[:, 3] = alpha
    ax.quiver(P[:, 0], P[:, 1], u * length, v * length, color=rgba, angles="xy", scale_units="xy",
              scale=1, pivot="mid", units="xy", width=0.13, headwidth=4.2, headlength=4.6,
              headaxislength=4.0, minshaft=1.5, zorder=1)

    # the reference r: it rises out of the dark on the left and leaves at the spine
    xs = np.linspace(-1, W + 1, 160)
    for x0, x1 in zip(xs[:-1], xs[1:]):
        s = np.clip((x0 - 8) / 70, 0, 1)
        ax.plot([x0, x1], [CY, CY], color=INK, lw=0.7, alpha=s * s * (3 - 2 * s),
                solid_capstyle="butt", zorder=3)
    ax.text(W - 14, CY + 2.2, "$r$", color=INK, fontsize=15, ha="center", va="bottom", zorder=6)

    out = OUT / "back_field.pdf"
    fig.savefig(out, transparent=True)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
