"""Figures of the mathematical toolbox (appendices). Drawn from formulas; no simulator data."""

import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import expm

from figlib import C, MM, TEXT_W, fig, save


@fig
def tb_phase():
    """Phase portraits of x'' + c x' + k x = 0 for four pairs (c, k)."""
    cases = [
        (7, 10, "two real poles: $-2$, $-5$"),
        (3, 10, "complex poles: $-1.5 \\pm 2.78j$"),
        (0, 10, "imaginary poles: $\\pm 3.16j$"),
        (3, -10, "poles $+2$, $-5$: a saddle"),
    ]
    f, axs = plt.subplots(2, 2, figsize=(TEXT_W, 104 * MM), gridspec_kw=dict(hspace=0.42, wspace=0.22))
    t = np.linspace(0, 6, 500)
    for a, (c, k, title) in zip(axs.flat, cases):
        A = np.array([[0.0, 1.0], [-k, -c]])
        lam, V = np.linalg.eig(A)
        if np.isreal(lam).all():
            for l, v in zip(lam.real, V.T.real):
                v = v / np.hypot(*v) * 9
                a.plot([-v[0], v[0]], [-v[1], v[1]], color=C["hair"], lw=1.0, zorder=1)
        starts = []
        if k > 0:
            for th in np.linspace(0, 2 * np.pi, 10, endpoint=False):
                r = 1.0 if c else 1.0
                starts.append((r * np.cos(th), 3.16 * r * np.sin(th)))
            if c == 0:
                starts = [(r, 0) for r in (0.3, 0.6, 0.9)]
        else:
            for x0 in (-1, -0.5, 0.5, 1):
                for v0 in (-3.0, 3.0):
                    starts.append((x0, v0))
        for x0 in starts:
            X = np.array([expm(A * ti) @ np.array(x0) for ti in (t if k > 0 else t[:60])])
            a.plot(X[:, 0], X[:, 1], color=C["meas"], lw=0.8, zorder=2)
            j = 12 if k > 0 else 8
            a.annotate("", xy=X[j + 1], xytext=X[j], arrowprops=dict(arrowstyle="-|>", color=C["meas"], lw=0.8, mutation_scale=6))
        a.plot(0, 0, "o", color=C["accent"], ms=3, zorder=3)
        a.axhline(0, color=C["faint"], lw=0.4, zorder=0)
        a.axvline(0, color=C["faint"], lw=0.4, zorder=0)
        a.set_xlim(-1.25, 1.25)
        a.set_ylim(-3.9, 3.9)
        a.grid(False)
        a.set_title(f"$c = {c}$, $k = {k}$", fontsize=7.6)
        a.text(0.02, 0.03, title, transform=a.transAxes, fontsize=6.4, color=C["muted"])
        a.set_xlabel("position $x$", loc="right")
        a.set_ylabel("velocity $\\dot x$")
    save(f, "tb_phase")
