"""Figures of the mathematical toolbox (appendices). Drawn from formulas; no simulator data."""

import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import expm

from figlib import C, MM, TEXT_W, fig, ramp, save


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
    a.text(0.16, 12, "slope $-40$ dB/decade", fontsize=6.2, color=C["muted"])
    a.text(2.6, 0.22, "$-20$ dB/decade", fontsize=6.2, color=C["muted"])
    a.text(45, 0.0009, "$-40$ dB/decade", fontsize=6.2, color=C["muted"])
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
    a.text(-0.93, -0.2, "$-1$", fontsize=7, ha="left", color=C["ink"])
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


@fig
def tb_zplane():
    """The map z = exp(s dt): lines of constant damping and of constant natural frequency."""
    f, a = plt.subplots(figsize=(TEXT_W, 88 * MM))
    th = np.linspace(0, 2 * np.pi, 400)
    a.plot(np.cos(th), np.sin(th), color=C["ink"], lw=0.9)
    # constant damping ratio: s = wn(-zeta + j sqrt(1-zeta^2)), wn*dt from 0 to pi/sqrt(1-zeta^2)
    for z_, col in zip((0.2, 0.4, 0.6, 0.8), ramp(4, "blue")):
        wd = np.linspace(0, np.pi, 300)
        wn = wd / np.sqrt(1 - z_**2)
        zz = np.exp(-z_ * wn + 1j * wd)
        for sg in (1, -1):
            a.plot(zz.real, sg * zz.imag, color=col, lw=0.9)
        k = 120
        a.text(zz[k].real, zz[k].imag + 0.035, f"$\\zeta = {z_}$", fontsize=6.0, color=col, ha="center")
    # constant wn*dt
    for wn in np.pi * np.array([0.1, 0.2, 0.4, 0.6, 0.8]):
        zt = np.linspace(0, 1, 100)
        zz = np.exp(wn * (-zt + 1j * np.sqrt(1 - zt**2)))
        for sg in (1, -1):
            a.plot(zz.real, sg * zz.imag, color=C["hair"], lw=0.8, zorder=0)
        a.text(zz[0].real * 1.09, zz[0].imag * 1.09, f"{wn / np.pi:.1f}$\\pi$", fontsize=6.0, color=C["muted"], ha="center", va="center")
    a.text(-1.13, 0.0, "$\\pi$", fontsize=6.0, color=C["muted"], ha="center", va="center")
    a.text(0.93, 0.62, "$\\omega_n\\Delta t$", fontsize=6.4, color=C["muted"])
    # the poles -2, -5 of the default loop at three sample rates
    for hz, col, dy in ((250, C["accent"], 0.07), (10, C["I"], 0.07), (2, C["err"], 0.07)):
        p = np.exp(np.array([-2.0, -5.0]) / hz)
        a.plot(p, [0, 0], "x", color=col, ms=5, mew=1.3, zorder=5)
        a.text(p.mean(), dy, f"{hz} Hz", fontsize=6.2, color=col, ha="center")
    a.axhline(0, color=C["faint"], lw=0.4, zorder=0)
    a.axvline(0, color=C["faint"], lw=0.4, zorder=0)
    a.set_xlim(-1.3, 1.3)
    a.set_ylim(-1.12, 1.12)
    a.set_aspect("equal")
    a.grid(False)
    a.set_xlabel("Re $z$", loc="right")
    a.set_ylabel("Im $z$")
    save(f, "tb_zplane")


@fig
def tb_ou():
    """The Ornstein-Uhlenbeck turbulence: a realisation, its autocorrelation and its spectrum."""
    sig, tau, dt = 1.2, 1.5, 0.01
    rng = np.random.default_rng(4242)
    n = int(600 / dt)
    a, q = np.exp(-dt / tau), sig * np.sqrt(1 - np.exp(-2 * dt / tau))
    w = np.empty(n)
    w[0] = sig * rng.standard_normal()
    xi = rng.standard_normal(n)
    for k in range(1, n):
        w[k] = a * w[k - 1] + q * xi[k]
    f = plt.figure(figsize=(TEXT_W, 88 * MM))
    gs = f.add_gridspec(2, 2, hspace=0.62, wspace=0.3, height_ratios=[1, 1.1])
    ax = f.add_subplot(gs[0, :])
    m = int(40 / dt)
    ax.plot(np.arange(m) * dt, w[:m], color=C["wind"], lw=0.9)
    ax.axhline(0, color=C["faint"], lw=0.5)
    for s in (1, -1):
        ax.axhline(s * sig, color=C["faint"], lw=0.6, ls=(0, (4, 3)))
    ax.text(40.3, sig, "$+\\sigma_w$", fontsize=6.4, color=C["muted"], va="center")
    ax.text(40.3, -sig, "$-\\sigma_w$", fontsize=6.4, color=C["muted"], va="center")
    ax.set_xlim(0, 40)
    ax.set_title("One realisation")
    ax.set_xlabel("time  [s]", loc="right")
    ax.set_ylabel("$w$  [m/s]")
    ax = f.add_subplot(gs[1, 0])
    lags = np.arange(0, 8.01, 0.4)
    R = [np.mean(w[: n - int(l / dt)] * w[int(l / dt):]) for l in lags]
    tt = np.linspace(0, 8, 200)
    ax.plot(tt, sig**2 * np.exp(-tt / tau), color=C["ink"], lw=1.0, ls=(0, (4, 2.5)))
    ax.plot(lags, R, "o", color=C["wind"], ms=2.6)
    ax.axvline(tau, color=C["hair"], lw=0.7, zorder=0)
    ax.text(tau + 0.15, 1.25, "$\\tau_w$", fontsize=6.6, color=C["muted"])
    ax.set_title("Autocorrelation")
    ax.set_xlabel("lag $\\tau$  [s]", loc="right")
    ax.set_ylabel("$R(\\tau)$  [m²/s²]")
    ax.set_xlim(0, 8)
    ax = f.add_subplot(gs[1, 1])
    om = np.geomspace(0.02, 100, 300)
    ax.loglog(om, 2 * sig**2 * tau / (1 + om**2 * tau**2), color=C["wind"], lw=1.2)
    ax.axvline(1 / tau, color=C["hair"], lw=0.7, zorder=0)
    ax.text(1 / tau * 1.2, 6, "$1/\\tau_w$", fontsize=6.6, color=C["muted"])
    ax.set_title("Power spectral density")
    ax.set_xlabel("$\\omega$  [rad/s]", loc="right")
    ax.set_ylabel("$\\Phi(\\omega)$")
    ax.set_xlim(0.02, 100)
    ax.grid(True, which="major", axis="both")
    save(f, "tb_ou")
