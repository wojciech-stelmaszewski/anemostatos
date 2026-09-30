"""Figures for Part II."""

import matplotlib.pyplot as plt
import numpy as np

from figlib import (C, FULL_W, MM, SEQ, TEXT_W, fig, label_end, load, note, ramp, sat_shade,
                    save, step_metrics, win, xlab_time)
from scipy.linalg import expm


def _up_step(d, size=0.5):
    """Index range of the first upward setpoint step (and the samples until the next step)."""
    sp = d["sp.y"].values
    j = [i for i in range(1, len(sp)) if abs(sp[i] - sp[i - 1]) > size]
    for a in range(len(j)):
        if sp[j[a]] > sp[j[a] - 1]:
            return j[a], (j[a + 1] if a + 1 < len(j) else len(sp))
    raise ValueError("no upward step")


# ─── Lesson II.1 ───────────────────────────────────────────────────────────
@fig
def state_portraits():
    runs = [("state-kp10-kd2", 10, 2, C["err"]), ("state-kp10-kd6.3", 10, 6.3, C["I"]),
            ("state-kp10-kd20", 10, 20, C["D"]), ("state-kp20-kd8", 20, 8, C["meas"])]
    f, (a, b) = plt.subplots(1, 2, figsize=(TEXT_W, 66 * MM), gridspec_kw=dict(wspace=0.3))
    for name, kp, kd, col in runs:
        d = load(name)
        i0, i1 = _up_step(d)
        t = d.t.values[i0:i1] - d.t.values[i0]
        e = (d["sp.y"].values - d["pos.y"].values)[i0:i1]
        de = -d["vel.y"].values[i0:i1]
        a.plot(t, e, color=col, lw=1.1, label=f"$K_p = {kp}$, $K_d = {kd}$")
        b.plot(e, de, color=col, lw=1.1)
        A = np.array([[0.0, 1.0], [-kp, -kd]])
        tt = np.linspace(0, 4, 400)
        X = np.array([expm(A * ti) @ np.array([e[0], de[0]]) for ti in tt])
        b.plot(X[:, 0], X[:, 1], color=C["ink"], lw=0.6, ls=(0, (3, 2)))
    a.axhline(0, color=C["sp"], lw=0.8, ls=(0, (4, 2.5)))
    a.set_xlim(0, 4)
    a.legend(loc="upper right", handlelength=1.2)
    a.set_title("The error after a step")
    a.set_ylabel("$e$  [m]")
    xlab_time(a, "time after the step  [s]")
    ee = np.array([0, 1.05])
    b.plot(ee, -0.513 * ee, color=C["D"], lw=0.6, alpha=0.6)
    b.text(0.62, -0.17, "slow eigenvector", fontsize=6.0, color=C["D"], rotation=-9)
    b.plot(0, 0, "o", color=C["accent"], ms=3, zorder=5)
    b.axhline(0, color=C["faint"], lw=0.4, zorder=0)
    b.axvline(0, color=C["faint"], lw=0.4, zorder=0)
    b.set_xlim(-0.4, 1.08)
    b.set_ylim(-2.2, 1.0)
    b.grid(False)
    b.set_title("The same, in the phase plane")
    b.set_xlabel("$e$  [m]", loc="right")
    b.set_ylabel("$\\dot e$  [m/s]")
    save(f, "state_portraits")


@fig
def state_thrusters():
    """Formula only: a satellite with on-off thrusters in the phase plane."""
    al = 0.05 / 12
    th0 = np.radians(60)
    f, a = plt.subplots(figsize=(TEXT_W, 58 * MM))
    w = np.linspace(-0.09, 0.09, 300)
    for c in np.radians(np.arange(-150, 151, 30)):
        a.plot(np.degrees(c + w**2 / (2 * al)), np.degrees(w), color=C["hair"], lw=0.6, zorder=0)
        a.plot(np.degrees(c - w**2 / (2 * al)), np.degrees(w), color=C["hair"], lw=0.6, zorder=0)
    ws = np.linspace(0, 0.09, 200)
    a.plot(np.degrees(-ws**2 / (2 * al)), np.degrees(ws), color=C["ink"], lw=1.0, ls=(0, (4, 2.5)))
    a.plot(np.degrees(ws**2 / (2 * al)), np.degrees(-ws), color=C["ink"], lw=1.0, ls=(0, (4, 2.5)))
    a.text(38, -4.35, "switching curve", fontsize=6.4, color=C["ink"])
    wp = np.sqrt(th0 * al)
    w1 = np.linspace(0, wp, 100)
    a.plot(np.degrees(-th0 + w1**2 / (2 * al)), np.degrees(w1), color=C["P"], lw=1.4)
    a.plot(np.degrees(-w1**2 / (2 * al)), np.degrees(w1), color=C["meas"], lw=1.4)
    a.plot(-60, 0, "o", color=C["ink"], ms=3)
    a.plot(0, 0, "o", color=C["accent"], ms=3.5, zorder=5)
    a.text(-59, -0.55, "start", fontsize=6.4, color=C["muted"])
    a.text(-52, 2.3, "full torque $+$", fontsize=6.4, color=C["P"], rotation=38)
    a.text(-17, 2.9, "full torque $-$", fontsize=6.4, color=C["meas"], rotation=-38)
    a.axhline(0, color=C["faint"], lw=0.4, zorder=0)
    a.axvline(0, color=C["faint"], lw=0.4, zorder=0)
    a.set_xlim(-75, 75)
    a.set_ylim(-5, 5)
    a.grid(False)
    a.set_xlabel("pointing error  [degrees]", loc="right")
    a.set_ylabel("rate  [degrees/s]")
    save(f, "state_thrusters")
