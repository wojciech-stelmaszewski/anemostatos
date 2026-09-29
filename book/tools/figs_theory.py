"""Theory figures (no simulator data)."""

import matplotlib.pyplot as plt
import numpy as np

from figlib import C, FULL_W, MM, SEQ, TEXT_W, fig, note, ramp, save


def _step2(zeta, wn, t):
    """Unit step response of wn^2 / (s^2 + 2 zeta wn s + wn^2)."""
    from scipy.signal import lti, step
    _, y = step(lti([wn**2], [1, 2 * zeta * wn, wn**2]), T=t)
    return y


@fig
def zeta_family():
    zs = [0.1, 0.3, 0.5, 0.7, 1.0, 2.0]
    cols = [C["err"], C["P"], C["FF"], C["I"], C["meas"], C["D"]]
    f, ax = plt.subplots(1, 2, figsize=(FULL_W, 62 * MM), gridspec_kw=dict(width_ratios=[1, 1.5], wspace=0.25))
    a = ax[0]
    th = np.linspace(0, 2 * np.pi, 200)
    a.plot(np.cos(th), np.sin(th), color=C["hair"], lw=0.6)
    for z, c in zip(zs, cols):
        if z < 1:
            re, im = -z, np.sqrt(1 - z * z)
            a.plot([re, re], [im, -im], "x", color=c, ms=5, mew=1.3)
            if z in (0.1, 0.5):
                a.text(re * 1.25, im * 1.2, f"{z}", color=c, fontsize=6.3, ha="center", va="center")
        else:
            r = np.sqrt(z * z - 1)
            a.plot([-z + r, -z - r], [0, 0], "x", color=c, ms=5, mew=1.3)
            a.text(-z - r, 0.16, f"{z}" if z > 1 else "1.0 (double)", color=c, fontsize=6.3, ha="center")
            if z > 1:
                a.text(-z + r, -0.2, f"{z}", color=c, fontsize=6.3, ha="center")
    a.axhline(0, color=C["faint"], lw=0.5)
    a.axvline(0, color=C["faint"], lw=0.5)
    a.set_xlim(-4.2, 0.6)
    a.set_ylim(-1.25, 1.25)
    a.set_aspect("equal")
    a.grid(False)
    a.set_title("Poles $s/\\omega_n$ as $\\zeta$ grows")
    a.set_xlabel("Re", loc="right")
    a.set_ylabel("Im")
    a.text(-4.1, -1.12, "slower ←", color=C["muted"], fontsize=6.3)
    a.text(0.12, 0.35, "faster\noscillation\n↑", color=C["muted"], fontsize=6.0, va="bottom")
    a = ax[1]
    t = np.linspace(0, 14, 800)
    for z, c in zip(zs, cols):
        a.plot(t, _step2(z, 1, t), color=c, lw=1.1, label=f"$\\zeta = {z}$")
    a.axhline(1, color=C["sp"], lw=0.8, ls=(0, (4, 2.5)))
    a.axhspan(0.98, 1.02, color=C["mist"], lw=0, zorder=0)
    a.legend(loc="lower right", ncol=2)
    a.set_title("Step responses")
    a.set_xlabel("$\\omega_n t$", loc="right")
    a.set_ylabel("$y / y_\\infty$")
    a.set_xlim(0, 14)
    save(f, "zeta_family")


@fig
def spring_energy():
    """Undamped oscillator: position and the exchange of kinetic and potential energy."""
    t = np.linspace(0, 4, 400)
    w = np.pi
    y = np.cos(w * t)
    v = -w * np.sin(w * t)
    f, a = plt.subplots(figsize=(MM * 46, MM * 40))
    a.plot(t, 0.5 * w * w * y**2 / (0.5 * w * w), color=C["P"], lw=1, label="spring energy")
    a.plot(t, 0.5 * v**2 / (0.5 * w * w), color=C["meas"], lw=1, label="kinetic energy")
    a.plot(t, np.ones_like(t), color=C["ink"], lw=0.8, ls=(0, (3, 2)), label="total")
    a.set_ylim(0, 1.25)
    a.set_yticks([0, 1])
    a.set_xticks([0, 1, 2, 3, 4])
    a.legend(loc="upper center", fontsize=5.8, ncol=1, bbox_to_anchor=(0.5, 1.28))
    a.set_xlabel("$t / (T/2)$", loc="right")
    save(f, "spring_energy")


@fig
def challenge_sens():
    """|Y/F|(jw) for the altitude loop: how much each frequency of a disturbance force moves the drone."""
    w = np.logspace(-1.5, 2, 500)
    s = 1j * w
    f, a = plt.subplots(figsize=(TEXT_W, 50 * MM))
    for (kp, ki, kd, tf), lab, col in [((10, 0.8, 7, 20), "default tune", C["faint"]),
                                       ((40, 8, 12, 30), "tuned for the challenge", C["accent"])]:
        Tf = 1 / (2 * np.pi * tf)
        C_ = kp + ki / s + kd * s / (Tf * s + 1)
        G = 1 / (s**2 * (0.03 * s + 1))
        Gd = 1 / s**2
        H = Gd / (1 + C_ * G)
        a.loglog(w / (2 * np.pi), np.abs(H) * 100, color=col, lw=1.3, label=lab)
    a.loglog(w / (2 * np.pi), 100 / w**2, color=C["err"], lw=0.8, ls=(0, (3, 2)), label="no control: $1/(m\\omega^2)$")
    a.axvspan(1 / 2.5, 1 / 0.6, color=C["mist"], lw=0, zorder=0)
    a.text(0.42, 0.012, "gust\ndurations\n0.6–2.5 s", fontsize=6.3, color=C["muted"])
    a.set_ylim(0.01, 300)
    a.set_xlabel("frequency of the disturbance  [Hz]", loc="right")
    a.set_ylabel("cm of motion per N")
    a.legend(loc="upper right")
    a.grid(True, which="major", axis="both", color="#EEF0F3", lw=0.4)
    a.set_title("Disturbance response: altitude error per newton of disturbing force")
    save(f, "challenge_sens")
