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


# ─── Lesson II.3 ───────────────────────────────────────────────────────────
def _lqi_gains(qi, mh=0.8, dt=0.004):
    from scipy.linalg import solve_discrete_are as dare
    A = np.array([[0, 1, 0], [0, 0, 0], [1, 0, 0.0]])
    B = np.array([[0], [1 / mh], [0.0]])
    M = np.zeros((4, 4))
    M[:3, :3] = A * dt
    M[:3, 3:] = B * dt
    E_ = expm(M)
    Ad, Bd = E_[:3, :3], E_[:3, 3:]
    Q = np.diag([100, 10, qi]) * dt
    R = np.array([[dt]])
    P = dare(Ad, Bd, Q, R)
    return np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)[0]


def _lqi_model(d, k, t0=5.0, t1=30.0, m=1.0, mh=0.8, g=9.81, step_at=15.0, dt=0.001):
    """The linear loop with the true mass, started from the flown state at t0."""
    i = int(np.argmin(abs(d.t.values - t0)))
    r = d["sp.y"].values[i]
    x, v = d["pos.y"].values[i] - r, d["vel.y"].values[i]
    xi = -d["alt.part.int"].values[i] / k[2] if k[2] else 0.0
    T, Y = [], []
    t = t0
    while t <= t1:
        if abs(t - step_at) < dt / 2:
            r += 1.0
            x -= 1.0
        u = -(k[0] * x + k[1] * v + k[2] * xi)
        a = (u - (m - mh) * g) / m
        v += a * dt
        x += v * dt
        xi += x * dt
        T.append(t)
        Y.append(x + r)
        t += dt
    return np.array(T), np.array(Y)


@fig
def lqi_runs():
    f, a = plt.subplots(figsize=(TEXT_W, 64 * MM))
    runs = [("lqi-lqr", 0, C["err"], "LQR, no integral"), ("lqi-lqi", 5, C["meas"], "LQI, $q_i = 5$"),
            ("lqi-lqi-q50", 50, C["I"], "LQI, $q_i = 50$")]
    for name, qi, col, lab in runs:
        d = win(load(name), 2, 30)
        a.plot(d.t, d["pos.y"], color=col, lw=1.2, label=lab)
        k = _lqi_gains(qi) if qi else np.r_[load(name)["lqr.k1"].iloc[-1], load(name)["lqr.k2"].iloc[-1], 0.0]
        tm, ym = _lqi_model(load(name), k)
        a.plot(tm, ym, color=C["ink"], lw=0.6, ls=(0, (3, 2)))
    d = win(load("lqi-lqr"), 2, 30)
    a.plot(d.t, d["sp.y"], color=C["sp"], lw=0.8, ls=(0, (4, 2.5)))
    a.text(9.5, 1.86, "droop $1.96\\,$N$\\,/\\,k_1 = 19.8$ cm", fontsize=6.4, color=C["err"])
    a.legend(loc="upper left", handlelength=1.2)
    a.set_xlim(2, 30)
    a.set_ylim(1.7, 3.3)
    a.set_title("A model that believes $\\hat m = 0.8$ kg, a drone of 1 kg")
    a.set_ylabel("altitude  [m]")
    xlab_time(a)
    save(f, "lqi_runs")


# ─── Lesson II.4 ───────────────────────────────────────────────────────────
@fig
def kalman_runs():
    f, (a, b) = plt.subplots(2, 1, figsize=(TEXT_W, 82 * MM), sharex=True, gridspec_kw=dict(hspace=0.3, height_ratios=[1.3, 1]))
    t0, t1 = 20, 23
    d = win(load("kalman-kalman"), t0, t1)
    a.fill_between(d.t, d["est.y.lo"], d["est.y.hi"], color=C["meas"], alpha=0.15, lw=0, label="estimate $\\pm 2\\sigma$")
    a.plot(d.t, d["meas.y"], color=C["faint"], lw=0.8, drawstyle="steps-post", label="barometer (50 Hz, $\\sigma$ = 5 cm)")
    a.plot(d.t, d["pos.y"], color=C["ink"], lw=1.1, label="truth")
    a.plot(d.t, d["est.y"], color=C["meas"], lw=1.1, label="Kalman estimate")
    a.legend(loc="upper right", ncol=2, fontsize=6.2)
    a.set_ylim(1.84, 2.2)
    a.set_title("What the controller is told")
    a.set_ylabel("altitude  [m]")
    for name, col, lab in (("kalman-none", C["err"], "raw sensor"), ("kalman-kalman", C["meas"], "Kalman filter")):
        d = win(load(name), t0, t1)
        b.plot(d.t, d["alt.u"], color=col, lw=0.8, label=lab)
    b.legend(loc="upper right", ncol=2)
    b.set_title("Thrust, same PID, same gains")
    b.set_ylabel("N")
    b.set_xlim(t0, t1)
    xlab_time(b)
    save(f, "kalman_runs")


@fig
def kalman_trust():
    f, axs = plt.subplots(1, 3, figsize=(TEXT_W, 52 * MM), sharey=True, gridspec_kw=dict(wspace=0.12))
    runs = [("kalman-overtrust", "assumed $\\sigma$ = 5 mm"), ("kalman-kalman", "assumed $\\sigma$ = 5 cm (true)"),
            ("kalman-distrust", "assumed $\\sigma$ = 50 cm")]
    for a, (name, lab) in zip(axs, runs):
        d = win(load(name), 20, 30)
        s = d["est.sigma.y"]
        a.fill_between(d.t, -200 * s, 200 * s, color=C["meas"], alpha=0.15, lw=0)
        a.plot(d.t, 100 * (d["est.y"] - d["pos.y"]), color=C["meas"], lw=0.7)
        a.axhline(0, color=C["faint"], lw=0.5)
        e = d["est.y"] - d["pos.y"]
        u = d["alt.u"].values
        a.set_title(lab, fontsize=7.2)
        a.text(0.03, 0.04, f"error {100 * np.sqrt((e**2).mean()):.1f} cm RMS\nthrust jitter {np.sqrt((np.diff(u)**2).mean()):.1f} N",
               transform=a.transAxes, fontsize=6.2, color=C["ink"])
        a.set_xlim(20, 30)
        a.set_ylim(-6, 6)
        xlab_time(a)
    axs[0].set_ylabel("estimate $-$ truth  [cm]")
    save(f, "kalman_trust")


# ─── Lesson II.5 ───────────────────────────────────────────────────────────
def _adrc_payload_model(wo=15.0, wc=3.0, T=8.0, dt=1e-4):
    """Linear model of the payload event: drone of 1.3 kg, controller model of 1 kg."""
    y = v = 0.0
    z = np.zeros(3)
    L = np.array([3 * wo, 3 * wo**2, wo**3])
    out = []
    for k in range(int(T / dt)):
        u = wc * wc * (-z[0]) - 2 * wc * z[1] - z[2] + 9.81
        a = (u - 1.3 * 9.81) / 1.3
        inn = y - z[0]
        z = z + np.array([z[1] + L[0] * inn, z[2] + (u - 9.81) + L[1] * inn, L[2] * inn]) * dt
        v += a * dt
        y += v * dt
        out.append((k * dt, y, -z[2] + 0.0))
    return np.array(out)


@fig
def adrc_payload():
    f, (a, b) = plt.subplots(2, 1, figsize=(TEXT_W, 80 * MM), sharex=True, gridspec_kw=dict(hspace=0.3))
    t0 = 12
    for name, col, lab in (("adrc-pid", C["ghost"], "PID ghost"), ("adrc-adrc", C["meas"], "ADRC")):
        d = win(load(name), 10, 20)
        a.plot(d.t - t0, 100 * (d["pos.y"] - d["sp.y"]), color=col, lw=1.3 if "ghost" in lab else 1.1, label=lab)
    m = _adrc_payload_model()
    a.plot(m[:, 0], 100 * m[:, 1], color=C["ink"], lw=0.6, ls=(0, (3, 2)), label="model, calm air")
    a.axvline(0, color=C["faint"], lw=0.6)
    a.text(0.1, 7, "300 g payload lands", fontsize=6.4, color=C["muted"])
    a.legend(loc="lower right", ncol=3)
    a.set_ylabel("altitude error  [cm]")
    a.set_title("A payload, two controllers")
    d = win(load("adrc-adrc"), 10, 20)
    b.plot(d.t - t0, d["dist.y"], color=C["ink"], lw=1.0, label="true disturbance force")
    b.plot(d.t - t0, d["est.dist.y"], color=C["meas"], lw=1.1, ls=(0, (4, 2)), label="ESO estimate")
    b.legend(loc="upper right")
    b.set_ylabel("N")
    b.set_title("What the observer sees")
    b.set_xlim(-2, 8)
    xlab_time(b, "time after the payload lands  [s]")
    save(f, "adrc_payload")


# ─── Lesson II.6 ───────────────────────────────────────────────────────────
def _adrc_loop(wo, wc=3.0, tau=0.03, dt=0.004):
    """The sampled ADRC loop as matrices: plant with motor lag (ZOH), exact discrete ESO, law."""
    def c2d(A, B):
        n, m = A.shape[0], B.shape[1]
        M = np.zeros((n + m, n + m))
        M[:n, :n] = A * dt
        M[:n, n:] = B * dt
        E_ = expm(M)
        return E_[:n, :n], E_[:n, n:]
    Pd, Pb = c2d(np.array([[0, 1, 0], [0, 0, 1], [0, 0, -1 / tau]]), np.array([[0], [0], [1 / tau]]))
    L = np.array([3 * wo, 3 * wo**2, wo**3])
    Od, Ob = c2d(np.array([[-L[0], 1, 0], [-L[1], 0, 1], [-L[2], 0, 0]]),
                 np.array([[0, L[0]], [1, L[1]], [0, L[2]]]))
    return Pd, Pb, Od, Ob, np.array([wc * wc, 2 * wc, 1.0])


def _adrc_noise_jitter(wo, sigma=0.02, N=4000):
    """RMS change of the thrust over 10 ms caused by white altimeter noise, from the impulse response."""
    Pd, Pb, Od, Ob, K = _adrc_loop(wo)
    x = np.zeros(3)
    z = np.zeros(3)
    up = 0.0
    h = []
    for k in range(N):
        y = x[0] + (1.0 if k == 0 else 0.0)
        z = Od @ z + Ob[:, 0] * up + Ob[:, 1] * y
        u = -K @ z
        h.append(u)
        x = Pd @ x + Pb[:, 0] * u
        up = u
    h = np.array(h)

    def var_d(j):
        d = np.r_[h, np.zeros(j)] - np.r_[np.zeros(j), h]
        return (d**2).sum()
    return sigma * np.sqrt(0.5 * var_d(2) + 0.5 * var_d(3))


@fig
def adrcbw_score():
    wos = [4, 6, 8, 10, 15, 20, 25, 30, 40, 50, 60, 70]
    rms, jit = [], []
    for wo in wos:
        d = load(f"adrcbw-wo{wo}")
        m = d.t >= 20
        e = (d["sp.y"] - d["pos.y"])[m].values
        u = d["alt.u"][m].values
        rms.append(100 * np.sqrt((e**2).mean()))
        jit.append(np.sqrt((np.diff(u) ** 2).mean()))
    rms, jit = np.array(rms), np.array(jit)
    f, (a, b) = plt.subplots(1, 2, figsize=(TEXT_W, 62 * MM), gridspec_kw=dict(wspace=0.3))
    wm = np.geomspace(4, 45, 60)
    a.loglog(wos, jit, "o", color=C["P"], ms=3, label="thrust jitter [N]")
    a.loglog(wm, [_adrc_noise_jitter(w) for w in wm], color=C["P"], lw=0.8, ls=(0, (3, 2)))
    a.loglog(wos, rms, "o", color=C["meas"], ms=3, label="RMS error [cm]")
    a.legend(loc="upper left")
    a.set_title("The two halves of the score")
    a.set_xlabel("$\\omega_o$  [rad/s]", loc="right")
    a.grid(True, which="major", axis="both")
    score = rms + 10 * jit
    b.semilogy(wos, score, "o-", color=C["ink"], ms=3, lw=0.9)
    b.axhline(7.5, color=C["I"], lw=0.7, ls=(0, (4, 2.5)))
    b.text(45, 6.0, "goal: 7.5", fontsize=6.4, color=C["I"])
    k = int(np.argmin(score))
    b.plot(wos[k], score[k], "o", color=C["accent"], ms=4.5, zorder=5)
    b.text(wos[k] * 1.1, score[k] * 0.62, f"best: $\\omega_o = {wos[k]}$", fontsize=6.4, color=C["accent"])
    b.axvspan(35, 72, color=C["sat"], alpha=0.6, lw=0, zorder=0)
    b.text(37, 120, "noise saturates\nthe motors", fontsize=6.2, color=C["err"])
    b.set_xscale("log")
    b.set_title("Score = error [cm] + 10 $\\times$ jitter [N]")
    b.set_xlabel("$\\omega_o$  [rad/s]", loc="right")
    b.grid(True, which="major", axis="both")
    save(f, "adrcbw_score")


# ─── Lesson II.7 ───────────────────────────────────────────────────────────
def _poserr(d):
    return np.sqrt((d["sp.x"] - d["pos.x"]) ** 2 + (d["sp.y"] - d["pos.y"]) ** 2 + (d["sp.z"] - d["pos.z"]) ** 2)


@fig
def indi_prop():
    f, ax = plt.subplots(3, 1, figsize=(TEXT_W, 100 * MM), sharex=True, gridspec_kw=dict(hspace=0.35, height_ratios=[1.1, 1, 1.1]))
    for tag, col, lab in (("pid", C["ghost"], "PID rate loop"), ("rate", C["meas"], "rate INDI")):
        d = win(load(f"indi-prop-{tag}"), 11, 20)
        ax[0].plot(d.t - 12, 100 * _poserr(d), color=col, lw=1.3 if tag == "pid" else 1.1, label=lab)
        ax[1].plot(d.t - 12, d["roll"], color=col, lw=1.3 if tag == "pid" else 1.1)
    ax[0].legend(loc="upper right")
    ax[0].set_title("Position error after the prop of motor 2 loses 40 % of its thrust")
    ax[0].set_ylabel("cm")
    ax[1].set_title("Roll angle")
    ax[1].set_ylabel("degrees")
    d = win(load("indi-prop-rate"), 11, 20)
    for key, col, lab in (("held", C["I"], "$\\tau_0/\\hat J$: the motors now"), ("want", C["P"], "$\\alpha_{des}$"),
                          ("have", C["D"], "$-\\dot\\omega$: measured")):
        ax[2].plot(d.t - 12, d[f"rate.roll.part.{key}"], color=col, lw=1.0, label=lab)
    ax[2].legend(loc="lower right", ncol=3, fontsize=6.2)
    ax[2].set_title("The three parts of the INDI command, roll axis")
    ax[2].set_ylabel("deg/s²")
    ax[2].set_xlim(-0.5, 4)
    xlab_time(ax[2], "time after the fault  [s]")
    save(f, "indi_prop")


@fig
def indi_effect():
    f, (a, b) = plt.subplots(1, 2, figsize=(TEXT_W, 58 * MM), gridspec_kw=dict(wspace=0.3, width_ratios=[1.3, 1]))
    for k, col in (("8", C["err"]), ("0.5", C["FF"]), ("2", C["meas"]), ("6", C["I"])):
        d = win(load(f"indi-prop-inertia{k}"), 11.5, 14)
        a.plot(d.t - 12, d["motor.1"], color=col, lw=0.4 if k == "8" else 1.0, alpha=0.45 if k == "8" else 1.0,
               label=f"$\\hat J = {k}\\,J$", zorder=1 if k == "8" else 3)
    a.legend(loc="upper left", ncol=2, fontsize=6.2)
    a.set_title("Thrust of motor 1 around the fault")
    a.set_ylabel("N")
    a.set_xlim(-0.5, 2)
    xlab_time(a, "time after the fault  [s]")
    wf, tau = 2 * np.pi * 20, 0.03
    w = np.geomspace(10, 1000, 400)
    FG = (wf**2 / ((1j * w) ** 2 + np.sqrt(2) * wf * 1j * w + wf**2)) / (tau * 1j * w + 1) * np.exp(-1j * w * 0.001)
    ph = np.degrees(np.unwrap(np.angle(FG)))
    b.semilogx(w, 1 / abs(FG) + 1, color=C["ink"], lw=1.0)
    k180 = np.interp(-180, ph[::-1], (1 / abs(FG) + 1)[::-1])
    b.axhline(k180, color=C["err"], lw=0.7, ls=(0, (4, 2.5)))
    b.text(12, k180 * 1.12, f"edge: $\\hat J/J \\approx {k180:.1f}$", fontsize=6.4, color=C["err"])
    for k, ok in ((0.5, True), (2, True), (3, True), (4, True), (6, True), (8, False), (12, False)):
        b.plot(600, k, "o" if ok else "x", color=C["I"] if ok else C["err"], ms=3.5, mew=1.2)
    b.text(250, 14, "flown", fontsize=6.2, color=C["muted"])
    b.set_yscale("log")
    b.set_ylim(0.4, 30)
    b.set_title("$1 + 1/|F G|$")
    b.set_xlabel("$\\omega$  [rad/s]", loc="right")
    b.grid(True, which="major", axis="both")
    save(f, "indi_effect")


# ─── Lesson II.8 ───────────────────────────────────────────────────────────
@fig
def indi_gust():
    f, (a, b) = plt.subplots(2, 1, figsize=(TEXT_W, 78 * MM), gridspec_kw=dict(hspace=0.42))
    for tag, col, lab in (("pid", C["ghost"], "PID cascade: 22.2 cm RMS"), ("acc", C["meas"], "acceleration INDI: 5.9 cm RMS")):
        d = win(load(f"indi-gust-{tag}"), 8, 40)
        a.plot(d.t, 100 * _poserr(d), color=col, lw=1.2 if tag == "pid" else 0.9, label=lab)
    a.legend(loc="upper right")
    a.set_xlim(8, 40)
    a.set_title("Position error on the gusty wind, same seed")
    a.set_ylabel("cm")
    xlab_time(a)
    d = win(load("indi-gust-acc"), 13.5, 16.5)
    b.plot(d.t, d["dist.x"], color=C["ink"], lw=1.0, label="true disturbance force, $x$")
    b.plot(d.t, d["est.dist.x"], color=C["meas"], lw=1.0, ls=(0, (4, 2)), label="what INDI implies: $\\hat m a_f - F_{0,f} - \\hat m g$")
    b.legend(loc="lower left", fontsize=6.2)
    b.set_xlim(13.5, 16.5)
    b.set_title("One gust, enlarged")
    b.set_ylabel("N")
    xlab_time(b)
    save(f, "indi_gust")


def _sync_jitter(tag):
    d = load(f"indisync-{tag}")
    m = d[d.t >= 15][[f"motor.{i}" for i in range(1, 5)]].values
    return np.sqrt((np.diff(m, axis=0) ** 2).sum(1).mean())


@fig
def indisync():
    f, (a, b) = plt.subplots(1, 2, figsize=(TEXT_W, 62 * MM), gridspec_kw=dict(wspace=0.32, width_ratios=[1.25, 1]))
    for tag, col, lab in (("off", C["err"], "not synchronised"), ("on", C["meas"], "synchronised")):
        d = win(load(f"indisync-{tag}"), 16, 17.5)
        a.plot(d.t, d["motor.1"], color=col, lw=0.8, label=lab)
    a.legend(loc="upper right", fontsize=6.2)
    a.set_xlim(16, 17.5)
    a.set_ylim(0, 7.5)
    a.set_title("Motor 1, both filters at 5 Hz")
    a.set_ylabel("N")
    xlab_time(a)
    off = [5, 6, 7, 8, 10, 20, 40]
    on = [5, 10, 20]
    jo = [_sync_jitter("off" if h == 5 else f"off-{h}hz") for h in off]
    jn = [_sync_jitter("on" if h == 5 else f"on-{h}hz") for h in on]
    b.axvspan(4, 6.9, color=C["sat"], alpha=0.7, lw=0, zorder=0)
    b.text(4.25, 0.036, "unstable", fontsize=6.0, color=C["err"])
    b.loglog(off, jo, "o-", color=C["err"], ms=3, lw=0.8, label="not synchronised")
    b.loglog(on, jn, "o-", color=C["meas"], ms=3, lw=0.8, label="synchronised")
    b.axhline(0.12, color=C["I"], lw=0.7, ls=(0, (4, 2.5)))
    b.text(22, 0.095, "goal: 0.12 N", fontsize=6.2, color=C["I"])
    b.set_xlim(4, 50)
    b.set_ylim(0.03, 1.0)
    b.set_xticks([5, 10, 20, 40])
    b.set_xticklabels(["5", "10", "20", "40"])
    b.set_yticks([0.05, 0.1, 0.2, 0.5, 1.0])
    b.set_yticklabels(["0.05", "0.1", "0.2", "0.5", "1"])
    b.minorticks_off()
    b.legend(loc="lower right", fontsize=6.2)
    b.set_title("Motor jitter against the filter")
    b.set_xlabel("filter cutoff [Hz]", loc="right")
    b.set_ylabel("N")
    b.grid(True, which="major", axis="both")
    save(f, "indisync")


def _tilt_deg(d):
    r, p = np.radians(d["roll"]), np.radians(d["pitch"])
    return np.degrees(np.arccos(np.clip(np.cos(r) * np.cos(p), -1, 1)))


@fig
def geometric_flip():
    f, (a, b) = plt.subplots(1, 2, figsize=(TEXT_W, 60 * MM), gridspec_kw=dict(wspace=0.3))
    for law, col, lab in (
        ("euler", C["err"], "Euler angles"),
        ("quaternion", C["meas"], "quaternion error"),
        ("tilt", C["I"], "tilt-prioritised"),
    ):
        d = win(load(f"flip-{law}"), 9.8, 12.5)
        ls = (0, (3, 2)) if law == "tilt" else "-"
        a.plot(d.t - 10, _tilt_deg(d), color=col, lw=0.9, ls=ls, label=lab)
        b.plot(d.t - 10, d["pos.y"], color=col, lw=0.9, ls=ls, label=lab)
    a.axhline(90, color=C["faint"], lw=0.5, ls=(0, (2, 2)))
    a.text(1.0, 94, "thrust points sideways", fontsize=6.0, color=C["muted"])
    a.set_xlim(-0.2, 2.5)
    a.set_ylim(0, 180)
    a.set_yticks([0, 45, 90, 135, 180])
    a.set_title("Tilt after the flip")
    a.set_ylabel("deg")
    a.set_xlabel("time after the flip [s]", loc="right")
    a.legend(loc="upper right", fontsize=6.2)
    b.axhline(6, color=C["faint"], lw=0.5, ls=(0, (2, 2)))
    b.axhline(6 - 1.8, color=C["I"], lw=0.7, ls=(0, (4, 2.5)))
    b.text(1.4, 4.0, "goal: lose < 1.8 m", fontsize=6.2, color=C["I"])
    b.set_xlim(-0.2, 2.5)
    b.set_title("Altitude")
    b.set_ylabel("m")
    b.set_xlabel("time after the flip [s]", loc="right")
    save(f, "geometric_flip")


@fig
def flatness_eight():
    f, (a, b) = plt.subplots(1, 2, figsize=(TEXT_W, 64 * MM), gridspec_kw=dict(wspace=0.3, width_ratios=[1.15, 1]))
    t0 = 40 - 8.9
    ref = win(load("flat-geo-ff"), t0, 40)
    a.plot(ref["sp.x"], ref["sp.z"], color=C["sp"], lw=2.2, alpha=0.5, label="reference")
    runs = (
        ("pid", C["ghost"], "PID cascade"),
        ("geo", C["err"], "geometric, feedback only"),
        ("geo-ff", C["meas"], "geometric + flatness feedforward"),
    )
    for tag, col, lab in runs:
        d = win(load(f"flat-{tag}"), t0, 40)
        a.plot(d["pos.x"], d["pos.z"], color=col, lw=0.9, label=lab)
        e = np.sqrt((d["pos.x"] - d["sp.x"]) ** 2 + (d["pos.z"] - d["sp.z"]) ** 2 + (d["pos.y"] - d["sp.y"]) ** 2)
        b.semilogy(d.t - t0, 100 * e, color=col, lw=0.9)
    a.set_aspect("equal")
    a.set_xlim(-2.6, 2.6)
    a.set_ylim(-1.9, 1.9)
    a.set_title("The last lap, seen from above")
    a.set_xlabel("x [m]", loc="right")
    a.set_ylabel("z [m]")
    a.legend(loc="upper center", fontsize=5.8, ncol=2, bbox_to_anchor=(0.5, -0.2), frameon=False)
    b.axhline(5, color=C["I"], lw=0.7, ls=(0, (4, 2.5)))
    b.text(0.3, 6.2, "goal: 5 cm RMS", fontsize=6.2, color=C["I"])
    b.set_xlim(0, 8.9)
    b.set_ylim(0.3, 300)
    b.set_yticks([1, 10, 100])
    b.set_yticklabels(["1", "10", "100"])
    b.set_title("Position error over the lap")
    b.set_ylabel("cm")
    b.set_xlabel("time in the lap [s]", loc="right")
    b.grid(True, which="major", axis="both")
    save(f, "flatness_eight")


@fig
def minsnap_square():
    f, (a, b) = plt.subplots(1, 2, figsize=(TEXT_W, 62 * MM), gridspec_kw=dict(wspace=0.32))
    for tag, col, lab, t0 in (
        ("corners", C["err"], "set-point steps", 31.81),
        ("minsnap", C["meas"], "minimum-snap plan", 32.0),
    ):
        d = win(load(f"snap-{tag}"), t0 - 0.2, t0 + 8)
        e = win(d, t0 - 0.2, t0 + 2.4)
        a.plot(e.t - t0, e["sp.x"], color=col, lw=0.8, ls=(0, (3, 2)))
        a.plot(e.t - t0, e["pos.x"], color=col, lw=1.0, label=lab)
        d = win(d, t0, 40)
        rate = np.sqrt(d["rate.roll.meas"] ** 2 + d["rate.pitch.meas"] ** 2)
        b.plot(d.t - t0, rate, color=col, lw=0.9, label=lab)
    a.set_xlim(-0.2, 2.4)
    a.set_ylim(-0.3, 3.3)
    a.set_title("One side: set-point (dashed) and drone")
    a.set_ylabel("x [m]")
    a.set_xlabel("time [s]", loc="right")
    a.legend(loc="upper right", fontsize=6.2)
    b.axhline(150, color=C["I"], lw=0.7, ls=(0, (4, 2.5)))
    b.text(2.3, 158, "goal: 150 °/s", fontsize=6.2, color=C["I"])
    b.set_xlim(0, 8)
    b.set_ylim(0, 330)
    b.set_title("Body rate over the lap")
    b.set_ylabel("deg/s")
    b.set_xlabel("time in the lap [s]", loc="right")
    save(f, "minsnap_square")


@fig
def mpc_ceiling():
    f, (a, b) = plt.subplots(2, 1, figsize=(TEXT_W, 80 * MM), sharex=True, gridspec_kw=dict(hspace=0.25, height_ratios=[1.4, 1]))
    for tag, col, lab in (("pid", C["ghost"], "PID"), ("mpc", C["meas"], "MPC")):
        d = win(load(f"mpc-{tag}"), 7, 21)
        a.plot(d.t - 8, d["pos.y"], color=col, lw=1.2 if tag == "pid" else 1.0, label=lab)
        b.plot(d.t - 8, d["thrust"], color=col, lw=1.0 if tag == "pid" else 0.9, label=lab)
    a.axhspan(4, 4.3, color=C["sat"], alpha=0.8, lw=0)
    a.axhline(4, color=C["err"], lw=0.7)
    a.text(10.2, 4.06, "ceiling", fontsize=6.2, color=C["err"])
    a.axhline(3.95, color=C["sp"], lw=0.6, ls=(0, (3, 2)))
    a.set_ylim(1.9, 4.25)
    a.set_title("Altitude after the set-point jumps to 3.95 m")
    a.set_ylabel("m")
    a.legend(loc="lower right")
    ins = a.inset_axes([0.35, 0.12, 0.3, 0.5])
    for tag, col in (("pid", C["ghost"]), ("mpc", C["meas"])):
        d = win(load(f"mpc-{tag}"), 9, 21)
        ins.plot(d.t - 8, d["pos.y"], color=col, lw=0.9)
    ins.axhline(4, color=C["err"], lw=0.7)
    ins.axhline(3.95, color=C["sp"], lw=0.6, ls=(0, (3, 2)))
    ins.set_ylim(3.88, 4.06)
    ins.set_xlim(1, 13)
    ins.tick_params(labelsize=5.5)
    b.axhline(4 * 6.1, color=C["faint"], lw=0.5, ls=(0, (2, 2)))
    b.text(10.2, 22.2, "4 × 6.1 N", fontsize=6.0, color=C["muted"])
    b.set_xlim(-1, 13)
    b.set_ylim(0, 26)
    b.set_title("Total thrust")
    b.set_ylabel("N")
    b.set_xlabel("time after the jump [s]", loc="right")
    save(f, "mpc_ceiling")


def _mpc_lin_gains(H, qp=50, qv=5, r=0.05, dt=0.02, b=1.0):
    """First-move gains (kp, kd) of the unconstrained condensed altitude MPC: u0 = -kp·e - kd·v."""
    n = max(2, round(H / dt))
    k = np.arange(n)[:, None]
    j = np.arange(n)[None, :]
    gp = np.where(j <= k, b * dt * dt * (k - j + 0.5), 0.0)
    gv = np.where(j <= k, b * dt, 0.0)
    hm = 2 * (r * np.eye(n) + qp * gp.T @ gp + qv * gv.T @ gv)
    kk = (np.arange(n) + 1) * dt
    de = 2 * qp * gp.T @ np.ones(n)
    dv = 2 * (qp * gp.T @ kk + qv * gv.T @ np.ones(n))
    return np.linalg.solve(hm, de)[0], np.linalg.solve(hm, dv)[0]


def _mpc_lin_reach(H, y0=2.0, r=3.95, umin=-9.81, umax=24.4 - 9.81):
    kp, kd = _mpc_lin_gains(H)
    y, v, t = y0, 0.0, 0.0
    while t < 10:
        u = np.clip(-kp * (y - r) - kd * v, umin, umax)
        for _ in range(20):
            v += u * 0.001
            y += v * 0.001
            t += 0.001
        if abs(y - r) < 0.05:
            return t
    return np.nan


@fig
def mpc_horizon():
    f, (a, b) = plt.subplots(1, 2, figsize=(TEXT_W, 62 * MM), gridspec_kw=dict(wspace=0.3))
    hs = [0.1, 0.2, 0.3, 0.5, 1]
    cols = ramp(len(hs), "blue")
    for h, col in zip(hs, cols):
        d = win(load(f"horizon-{h}"), 7.5, 16)
        a.plot(d.t - 8, d["pos.y"], color=col, lw=0.9, label=f"{h} s")
    a.axhline(4, color=C["err"], lw=0.7)
    a.set_xlim(-0.5, 8)
    a.set_ylim(1.9, 4.1)
    a.set_title("Altitude, by horizon")
    a.set_ylabel("m")
    a.set_xlabel("time after the jump [s]", loc="right")
    a.legend(loc="lower right", fontsize=6.0, title="horizon", title_fontsize=6.0)
    hh = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1, 1.5, 2]
    meas = []
    for h in hh:
        d = win(load(f"horizon-{h}"), 8, 20)
        k = np.argmax(np.abs(d["pos.y"].values - 3.95) < 0.05)
        meas.append(d.t.values[k] - 8)
    hm = np.geomspace(0.08, 2.2, 60)
    b.loglog(hm, [_mpc_lin_reach(h) for h in hm], color=C["meas"], lw=0.8, ls=(0, (3, 2)), label="linear MPC law, saturated")
    b.loglog(hh, meas, "o", color=C["meas"], ms=3, label="flight")
    b.axhline(1.6, color=C["I"], lw=0.7, ls=(0, (4, 2.5)))
    b.axvline(0.6, color=C["I"], lw=0.7, ls=(0, (4, 2.5)))
    b.text(0.62, 4.5, "goal: horizon ≤ 0.6 s,\nreached in < 1.6 s", fontsize=6.0, color=C["I"])
    b.set_xticks([0.1, 0.2, 0.5, 1, 2])
    b.set_xticklabels(["0.1", "0.2", "0.5", "1", "2"])
    b.set_yticks([1, 2, 5])
    b.set_yticklabels(["1", "2", "5"])
    b.minorticks_off()
    b.set_title("Time to reach the set-point")
    b.set_ylabel("s")
    b.set_xlabel("horizon [s]", loc="right")
    b.legend(loc="lower left", fontsize=6.0)
    b.grid(True, which="major", axis="both")
    save(f, "mpc_horizon")
