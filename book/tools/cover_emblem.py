"""The emblem on the cover: a feedback loop standing in the wind.

The loop is the block diagram of the book bent into a circle: the reference r comes in from the
left to the summing junction, the forward path runs over the top through the plant (a quadrotor,
seen from above), the output y is picked off on the right and the feedback path, in orange, runs
back underneath to the minus sign. Around it the wind is drawn as the exact streamlines of
potential flow past a cylinder, psi = U y (1 - a^2 / r^2): they part ahead of the loop and close
behind it, and the dividing streamline is the signal itself, r on the way in and y on the way out.

    python3 book/tools/cover_emblem.py   →   book/figures/out/cover_emblem.pdf
"""

import contourpy
import numpy as np

from figlib import OUT, plt

W, H = 210.0, 168.0           # mm: the full width of the page, the band of the cover it fills
CX, CY = W / 2, H / 2 + 2     # the centre of the loop
R = 33.0                      # the loop
A = 45.0                      # the cylinder the wind sees: a calm ring between it and the loop
J = 3.4                       # the summing junction
MM = 1 / 25.4

INK = "#E6E9EF"
WIND = "#7D93B5"
ACCENT = "#E8661A"


def streamlines(ax):
    x = np.linspace(-6, W + 6, 900)
    y = np.linspace(-6, H + 6, 720)
    X, Y = np.meshgrid(x, y)
    xr, yr = X - CX, Y - CY
    r2 = np.maximum(xr**2 + yr**2, 1e-6)
    psi = yr * (1 - A**2 / r2)
    psi[r2 < A**2] = np.nan
    gen = contourpy.contour_generator(x, y, psi)
    step = 5.2
    for k in range(-15, 16):
        if k == 0:
            continue
        level = k * step
        for line in gen.lines(level):
            # fainter far from the loop, so the eye goes to the middle
            d = abs(level) / (15 * step)
            alpha = 0.62 * (1 - d) ** 1.6 + 0.08
            ax.plot(line[:, 0], line[:, 1], color=WIND, lw=0.42, alpha=alpha, solid_capstyle="round",
                    zorder=1)
    # the dividing streamline is the signal: r in, y out
    ax.plot([-6, CX - R - J], [CY, CY], color=INK, lw=0.7, zorder=3)
    ax.plot([CX + R, W + 6], [CY, CY], color=INK, lw=0.7, zorder=3)


def arrow_on_circle(ax, ang_deg, color, size=3.2):
    """A small open chevron on the loop at angle ang_deg, pointing clockwise."""
    a = np.radians(ang_deg)
    p = np.array([CX + R * np.cos(a), CY + R * np.sin(a)])
    t = np.array([np.sin(a), -np.cos(a)])        # clockwise tangent
    n = np.array([np.cos(a), np.sin(a)])
    tip = p + t * size * 0.5
    for s in (1, -1):
        q = tip - t * size + n * s * size * 0.55
        ax.plot([q[0], tip[0]], [q[1], tip[1]], color=color, lw=0.9, solid_capstyle="round", zorder=4)


def arc(ax, a0, a1, color, lw):
    a = np.radians(np.linspace(a0, a1, 400))
    ax.plot(CX + R * np.cos(a), CY + R * np.sin(a), color=color, lw=lw, solid_capstyle="butt", zorder=3)


def quadrotor(ax, x0, y0, s=1.0):
    """A quadrotor from above: body, an X of arms, four rotor rings, a nose mark."""
    arm = 7.6 * s
    rotor = 3.9 * s
    for ang in (45, 135, 225, 315):
        a = np.radians(ang)
        ex, ey = x0 + arm * np.cos(a), y0 + arm * np.sin(a)
        ux, uy = np.cos(a), np.sin(a)
        ax.plot([x0 + 2.1 * s * ux, ex - rotor * 0.35 * ux], [y0 + 2.1 * s * uy, ey - rotor * 0.35 * uy],
                color=INK, lw=0.9, solid_capstyle="round", zorder=5)
        ax.add_patch(plt.Circle((ex, ey), rotor, fc="#0B0E13", ec=INK, lw=0.6, zorder=5))
        ax.add_patch(plt.Circle((ex, ey), 0.55 * s, fc=INK, ec="none", zorder=6))
    ax.add_patch(plt.Rectangle((x0 - 2.3 * s, y0 - 2.3 * s), 4.6 * s, 4.6 * s, fc="#0B0E13", ec=INK,
                               lw=0.8, zorder=6))
    ax.add_patch(plt.Polygon([[x0 + 2.3 * s, y0 - 1.0 * s], [x0 + 3.6 * s, y0], [x0 + 2.3 * s, y0 + 1.0 * s]],
                             closed=True, fc=ACCENT, ec="none", zorder=6))


def main():
    fig = plt.figure(figsize=(W * MM, H * MM))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.patch.set_alpha(0)
    ax.patch.set_alpha(0)

    streamlines(ax)

    # the loop: forward path over the top (through the plant), feedback underneath in orange
    gap = np.degrees(15.0 / R)          # room for the quadrotor at the top
    jgap = np.degrees((J + 0.6) / R)    # room for the junction on the left
    arc(ax, 180 - jgap, 90 + gap, INK, 0.9)
    arc(ax, 90 - gap, 0, INK, 0.9)
    arc(ax, 0, -180 + jgap, ACCENT, 0.9)
    arrow_on_circle(ax, 135, INK)
    arrow_on_circle(ax, 45, INK)
    arrow_on_circle(ax, -90, ACCENT)
    quadrotor(ax, CX, CY + R, s=1.35)

    # the pick-off point and the summing junction
    ax.add_patch(plt.Circle((CX + R, CY), 0.95, fc=INK, ec="none", zorder=5))
    jx = CX - R
    ax.add_patch(plt.Circle((jx, CY), J, fc="#0B0E13", ec=INK, lw=0.8, zorder=5))
    d = J * 0.62
    for s in (1, -1):
        ax.plot([jx - d, jx + d], [CY - s * d, CY + s * d], color=INK, lw=0.6, zorder=6)
    # signs: + where r enters, − where the feedback comes up from below
    ax.text(jx - J - 1.0, CY + 1.4, "$+$", color=INK, fontsize=11, ha="right", va="bottom", zorder=6)
    ax.text(jx - 2.4, CY - J - 1.4, "$-$", color=ACCENT, fontsize=13, ha="right", va="top", zorder=6)
    # the signals, in the book's italics
    ax.text(CX - A - 9, CY + 2.2, "$r$", color=INK, fontsize=15, ha="center", va="bottom", zorder=6)
    ax.text(CX + A + 9, CY + 2.2, "$y$", color=INK, fontsize=15, ha="center", va="bottom", zorder=6)

    out = OUT / "cover_emblem.pdf"
    fig.savefig(out, transparent=True)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
