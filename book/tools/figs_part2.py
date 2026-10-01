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


# ─── Lesson II.2 ───────────────────────────────────────────────────────────
@fig
def lqr_runs():
    runs = [("lqr-pd", "PD ghost: $K_p = 10$, $K_d = 7$", C["ghost"]), ("lqr-r10", "LQR, $r = 10$", C["FF"]),
            ("lqr-r1", "LQR, $r = 1$", C["meas"]), ("lqr-r0.1", "LQR, $r = 0.1$", C["D"])]
    f, (a, b) = plt.subplots(2, 1, figsize=(TEXT_W, 76 * MM), sharex=True, gridspec_kw=dict(hspace=0.3, height_ratios=[1.3, 1]))
    for name, lab, col in runs:
        d = win(load(name), 15.5, 20)
        a.plot(d.t - 16, d["pos.y"], color=col, lw=1.3 if "ghost" in lab else 1.1, label=lab)
        b.plot(d.t - 16, d["alt.u"], color=col, lw=1.3 if "ghost" in lab else 1.1)
    d = win(load("lqr-pd"), 15.5, 20)
    a.plot(d.t - 16, d["sp.y"], color=C["sp"], lw=0.8, ls=(0, (4, 2.5)))
    a.axhspan(2.48, 2.52, color=C["mist"], lw=0, zorder=0)
    a.legend(loc="lower right", handlelength=1.2)
    a.set_title("One upward step of 1 m")
    a.set_ylabel("altitude  [m]")
    b.axhline(24.4, color=C["err"], lw=0.6, ls=(0, (2, 2)))
    b.text(3.2, 22.6, "motor limit 24.4 N", fontsize=6.3, color=C["err"])
    b.set_title("Thrust")
    b.set_ylabel("N")
    b.set_xlim(-0.5, 4)
    xlab_time(b, "time after the step  [s]")
    save(f, "lqr_runs")


@fig
def lqr_cost():
    """Formula: the cost of a 1 m step over the plane of PD gains; LQR gains against r."""
    from scipy.linalg import solve_continuous_lyapunov as lyap
    A = np.array([[0.0, 1.0], [0.0, 0.0]])
    B = np.array([[0.0], [1.0]])
    Q = np.diag([100.0, 10.0])
    x0 = np.array([1.0, 0.0])
    kp = np.linspace(2, 30, 120)
    kd = np.linspace(1, 16, 120)
    J = np.empty((len(kd), len(kp)))
    for i, d_ in enumerate(kd):
        for j, p_ in enumerate(kp):
            K = np.array([[p_, d_]])
            Acl = A - B @ K
            PK = lyap(Acl.T, -(Q + K.T @ K))
            J[i, j] = x0 @ PK @ x0
    f, (a, b) = plt.subplots(1, 2, figsize=(TEXT_W, 62 * MM), gridspec_kw=dict(wspace=0.32))
    lev = [55, 56, 58, 62, 70, 85, 110, 150]
    cs = a.contour(kp, kd, J, levels=lev, colors=[C["faint"]], linewidths=0.6)
    a.clabel(cs, fmt="%d", fontsize=5.6, inline_spacing=2)
    a.plot(10, 5.477, "o", color=C["meas"], ms=4, zorder=5)
    a.text(10.8, 4.6, "LQR  (54.8)", fontsize=6.4, color=C["meas"])
    a.plot(10, 7, "o", color=C["ink"], ms=3.2, zorder=5)
    a.text(10.8, 7.3, "PD ghost  (56.4)", fontsize=6.4, color=C["ink"])
    a.plot(20, 8, "s", color=C["P"], ms=3, zorder=5)
    a.text(20.8, 8.2, "II.1 goal  (63.8)", fontsize=6.4, color=C["P"])
    a.grid(False)
    a.set_title("Cost of a 1 m step, $q_e = 100$, $q_v = 10$, $r = 1$")
    a.set_xlabel("$K_p$", loc="right")
    a.set_ylabel("$K_d$")
    r = np.geomspace(0.05, 30, 200)
    k1 = np.sqrt(100 / r)
    k2 = np.sqrt(10 / r + 2 * k1)
    b.loglog(r, k1, color=C["P"], lw=1.2, label="$k_1 = \\sqrt{q_e/r}$")
    b.loglog(r, k2, color=C["D"], lw=1.2, label="$k_2 = \\sqrt{q_v/r + 2k_1}$")
    for name, rr in (("lqr-r10", 10), ("lqr-r1", 1), ("lqr-r0.1", 0.1)):
        d = load(name)
        b.plot(rr, d["lqr.k1"].iloc[-1], "o", color=C["P"], ms=3)
        b.plot(rr, d["lqr.k2"].iloc[-1], "o", color=C["D"], ms=3)
    b.legend(loc="upper right")
    b.set_title("Gains against the thrust cost")
    b.set_xlabel("$r$", loc="right")
    b.grid(True, which="major", axis="both")
    save(f, "lqr_cost")
