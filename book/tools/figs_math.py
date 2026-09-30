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


@fig
def tb_locus():
    """Root loci of the altitude loop with the motor lag: Kp alone, and Kp with Kd in a fixed ratio."""
    tau, m, kd0, kp0 = 0.03, 1.0, 7.0, 10.0
    f, axs = plt.subplots(1, 2, figsize=(TEXT_W, 66 * MM), gridspec_kw=dict(wspace=0.2))

    def draw(a, polys, gains, mark, asym, title):
        R = np.array([np.sort_complex(np.roots(p)) for p in polys])
        # follow each root continuously from one gain to the next
        tr = [R[0]]
        for r in R[1:]:
            prev, left, new = tr[-1], list(r), []
            for q in prev:
                j = int(np.argmin([abs(q - x) for x in left]))
                new.append(left.pop(j))
            tr.append(np.array(new))
        tr = np.array(tr)
        for i in range(tr.shape[1]):
            a.plot(tr[:, i].real, tr[:, i].imag, color=C["meas"], lw=1.1)
        for c, ang in asym:
            for s in (1, -1):
                th = np.radians(ang) * s
                a.plot([c, c + 80 * np.cos(th)], [0, 80 * np.sin(th)], color=C["faint"], lw=0.6, ls=(0, (4, 3)), zorder=0)
        a.plot(tr[0].real, tr[0].imag, "x", color=C["ink"], ms=5, mew=1.2, zorder=4)
        k = int(np.argmin(abs(gains - mark)))
        a.plot(tr[k].real, tr[k].imag, "o", color=C["accent"], ms=3.2, zorder=5)
        a.axhline(0, color=C["faint"], lw=0.4, zorder=0)
        a.axvline(0, color=C["ink"], lw=0.5, zorder=0)
        a.axvspan(0, 8, color=C["sat"], alpha=0.5, lw=0, zorder=0)
        a.set_xlim(-40, 8)
        a.set_ylim(-24, 24)
        a.grid(False)
        a.set_title(title, fontsize=7.6)
        a.set_xlabel("Re $s$", loc="right")

    g1 = np.concatenate([np.linspace(0, 20, 400), np.geomspace(20, 900, 500)])
    draw(axs[0], [[tau * m, m, kd0, kp] for kp in g1], g1, kp0, [(-11.1, 60)], "$K_p$ grows, $K_d = 7$")
    axs[0].set_ylabel("Im $s$")
    axs[0].annotate("$K_p = 233$", xy=(0, 15.3), xytext=(-17, 20.5), fontsize=6.4, color=C["muted"],
                    arrowprops=dict(arrowstyle="-", color=C["faint"], lw=0.5))
    axs[0].annotate("$K_p = 14$", xy=(-4.35, 0), xytext=(-16, -9), fontsize=6.4, color=C["muted"],
                    arrowprops=dict(arrowstyle="-", color=C["faint"], lw=0.5))
    g2 = np.concatenate([np.linspace(0, 12, 900), np.geomspace(12, 60, 300)])
    draw(axs[1], [[tau * m, m, kd, kd * kp0 / kd0] for kd in g2], g2, kd0, [(-15.9, 90)], "$K_p$ and $K_d$ grow, ratio $10:7$")
    axs[1].plot(-kp0 / kd0, 0, "o", mfc="white", mec=C["ink"], ms=4.5, mew=1.0, zorder=4)
    axs[1].set_yticklabels([])
    axs[1].text(4, -22, "unstable", rotation=90, fontsize=6.2, color=C["err"], ha="center", va="bottom")
    save(f, "tb_locus")


def _L(w, T=0.0):
    """Loop transfer function of the altitude loop (P-D, motor lag, delay T) at s = jw."""
    s = 1j * w
    return (10 + 7 * s) * np.exp(-s * T) / (s**2 * (0.03 * s + 1))


@fig
def tb_bode():
    w = np.geomspace(0.1, 1000, 1200)
    Lw = _L(w)
    f, (a, b) = plt.subplots(2, 1, figsize=(TEXT_W, 86 * MM), sharex=True, gridspec_kw=dict(hspace=0.16, height_ratios=[1.25, 1]))
    a.loglog(w, abs(Lw), color=C["meas"], lw=1.3)
    asy = np.where(w < 10 / 7, 10 / w**2, np.where(w < 1 / 0.03, 7 / w, 7 / 0.03 / w**2))
    a.loglog(w, asy, color=C["ink"], lw=0.8, ls=(0, (4, 2.5)))
    a.axhline(1, color=C["faint"], lw=0.6)
    for wc_, lab in ((10 / 7, "lead corner\n1.43"), (1 / 0.03, "lag corner\n33.3")):
        a.axvline(wc_, color=C["hair"], lw=0.7, zorder=0)
        b.axvline(wc_, color=C["hair"], lw=0.7, zorder=0)
        a.text(wc_ * 1.12, 300, lab, fontsize=6.2, color=C["muted"], va="top")
    a.plot(7.0, 1, "o", color=C["accent"], ms=3.5, zorder=5)
    a.text(8.5, 1.7, "crossover, $\\omega_c = 7$", fontsize=6.4, color=C["accent"])
    a.text(0.13, 40, "$-40$ dB/decade", fontsize=6.2, color=C["muted"], rotation=-33)
    a.text(3.2, 0.62, "$-20$", fontsize=6.2, color=C["muted"])
    a.text(120, 0.0045, "$-40$", fontsize=6.2, color=C["muted"])
    a.set_ylim(1e-4, 1e3)
    a.set_yticks([1e-4, 1e-2, 1, 1e2])
    a.set_yticklabels(["$-80$", "$-40$", "0", "40"])
    a.set_ylabel("$|L|$  [dB]")
    a.grid(True, which="major", axis="both")
    ph = np.degrees(np.unwrap(np.angle(Lw)))
    b.semilogx(w, ph, color=C["meas"], lw=1.3)
    b.axhline(-180, color=C["err"], lw=0.7)
    b.annotate("", xy=(7, -113.4), xytext=(7, -180), arrowprops=dict(arrowstyle="<->", color=C["accent"], lw=0.8, mutation_scale=6))
    b.text(8.5, -150, "phase margin $66.6^\\circ$", fontsize=6.4, color=C["accent"])
    b.set_ylim(-190, -85)
    b.set_yticks([-180, -135, -90])
    b.set_ylabel("$\\angle L$  [degrees]")
    b.set_xlabel("$\\omega$  [rad/s]", loc="right")
    b.set_xlim(0.1, 1000)
    b.grid(True, which="major", axis="both")
    save(f, "tb_bode")


@fig
def tb_nyquist():
    w = np.geomspace(1.2, 3000, 6000)
    f, a = plt.subplots(figsize=(TEXT_W, 78 * MM))
    th = np.linspace(0, 2 * np.pi, 300)
    a.plot(np.cos(th), np.sin(th), color=C["hair"], lw=0.8, zorder=0)
    for T, col, lab in ((0.0, C["meas"], "no delay"), (0.2, C["err"], "delay 200 ms")):
        Lw = _L(w, T)
        a.plot(Lw.real, Lw.imag, color=col, lw=1.3, label=lab)
        a.plot(Lw.real, -Lw.imag, color=col, lw=0.8, alpha=0.35)
        k = int(np.argmin(abs(abs(Lw) - 1.9)))
        a.annotate("", xy=(Lw[k + 40].real, Lw[k + 40].imag), xytext=(Lw[k].real, Lw[k].imag),
                   arrowprops=dict(arrowstyle="-|>", color=col, lw=1.0, mutation_scale=7))
    a.plot(-1, 0, "+", color=C["ink"], ms=8, mew=1.3, zorder=6)
    a.text(-1.0, 0.14, "$-1$", fontsize=7, ha="center", color=C["ink"])
    a.plot(-1.228, 0, "o", color=C["err"], ms=3.2, zorder=6)
    a.annotate("$-1.23$ at $\\omega = 5.78$", xy=(-1.228, 0), xytext=(-2.75, 0.55), fontsize=6.4, color=C["err"],
               arrowprops=dict(arrowstyle="-", color=C["err"], lw=0.5))
    # phase margin arc for the loop without delay
    pm = np.radians(66.6)
    arc = np.linspace(np.pi, np.pi + pm, 40)
    a.plot(0.45 * np.cos(arc), 0.45 * np.sin(arc), color=C["accent"], lw=0.9)
    a.plot([0, np.cos(np.pi + pm)], [0, np.sin(np.pi + pm)], color=C["accent"], lw=0.6)
    a.text(-0.78, -0.42, "PM", fontsize=6.4, color=C["accent"])
    a.axhline(0, color=C["faint"], lw=0.4, zorder=0)
    a.axvline(0, color=C["faint"], lw=0.4, zorder=0)
    a.set_xlim(-3.2, 1.4)
    a.set_ylim(-1.75, 1.75)
    a.set_aspect("equal")
    a.grid(False)
    a.legend(loc="upper right")
    a.set_xlabel("Re $L(j\\omega)$", loc="right")
    a.set_ylabel("Im $L(j\\omega)$")
    save(f, "tb_nyquist")
