"""Prediction next to measurement (lessons 1–4): the formulas of each chapter drawn over the
simulator's own run of the same experiment."""

import matplotlib.pyplot as plt
import numpy as np

from figlib import C, MM, TEXT_W, fig, label_end, load, save, win, xlab_time

G = 9.81
DASH = (0, (4, 2))
DOT = (0, (1, 1.6))


def _linear(a, x0, t):
    """x(t) of x' = A x from x0, by eigen-decomposition (rows of the result: states)."""
    lam, v = np.linalg.eig(np.asarray(a, dtype=float))
    c = np.linalg.solve(v, np.asarray(x0, dtype=complex))
    return np.real(v @ (c[:, None] * np.exp(np.outer(lam, t))))


def _peaks(t, x):
    """Times and values of the positive maxima of an oscillation."""
    k = np.where((x[1:-1] > x[:-2]) & (x[1:-1] >= x[2:]) & (x[1:-1] > 0.01))[0] + 1
    return t[k], x[k]


# ─── Lesson 1 ──────────────────────────────────────────────────────────────
@fig
def meet_predict():
    f, ax = plt.subplots(1, 2, figsize=(TEXT_W, 52 * MM), gridspec_kw=dict(wspace=0.3))
    m, force, cv, floor = 1.05, 0.05 * G, 0.25, 0.052
    acc, vt = force / m, np.sqrt(force / cv)

    a = ax[0]
    d = win(load("meet-wrongmass-open"), 9, 13.8)
    t = np.linspace(0, 3.8, 400)
    a.plot(d.t - 10, d["pos.y"], color=C["meas"], lw=1.3)
    a.plot(t, np.maximum(2 - 0.5 * acc * t**2, floor), color=C["ink"], lw=0.8, ls=DASH)
    a.plot(t, np.maximum(2 - vt**2 / acc * np.log(np.cosh(acc * t / vt)), floor), color=C["accent"],
           lw=0.9, ls=DOT)
    a.axvline(0, color=C["err"], lw=0.6)
    a.text(0.08, 2.17, "feedback off", color=C["err"], fontsize=6.4)
    a.text(0.12, 0.72, "model without drag", color=C["ink"], fontsize=6.4)
    a.text(2.35, 1.35, "with drag", color=C["accent"], fontsize=6.4)
    a.text(3.0, 0.9, "simulator", color=C["meas"], fontsize=6.4)
    a.set_xlim(-1, 3.8)
    a.set_ylim(-0.05, 2.3)
    a.set_title("Open loop: the drone sinks")
    a.set_ylabel("altitude  [m]")
    xlab_time(a, "time after switching  [s]")

    a = ax[1]
    d = win(load("meet-wrongmass-pd"), 9, 16)
    a.plot(d.t - 10, d["sp.y"], color=C["sp"], ls=DASH, lw=0.9)
    a.plot(d.t - 10, d["pos.y"], color=C["meas"], lw=1.3)
    rest = 2 - force / 10
    a.axhline(rest, color=C["ink"], lw=0.8, ls=DOT)
    a.axvline(0, color=C["err"], lw=0.6)
    a.text(0.12, 2.008, "integral off", color=C["err"], fontsize=6.4)
    a.text(5.9, rest + 0.004, f"predicted rest: {rest:.3f} m", color=C["ink"], fontsize=6.4, ha="right")
    a.set_xlim(-1, 6)
    a.set_ylim(1.93, 2.02)
    a.set_title("P and D remain: it rests 4.9 cm low")
    xlab_time(a, "time after switching  [s]")
    save(f, "meet_predict")


# ─── Lesson 2 ──────────────────────────────────────────────────────────────
@fig
def spring_predict():
    f, ax = plt.subplots(1, 2, figsize=(TEXT_W, 58 * MM), gridspec_kw=dict(wspace=0.32, width_ratios=[1.35, 1]))
    kp, m, tau, cv = 10.0, 1.0, 0.032, 0.25
    w = np.sqrt(kp / m)
    sigma, a_star = kp * tau / (2 * m), 3 * np.pi * kp * tau / (8 * cv * w)
    runs = [("spring-long", 3.0, C["P"], "1 m step"), ("spring-small", 2.1, C["meas"], "0.1 m step")]

    a = ax[0]
    for name, r, col, lab in runs:
        d = win(load(name), 6, 70)
        tp, xp = _peaks(d.t.values, d["pos.y"].values - r)
        a.plot(tp - 6, xp, "o", ms=1.9, color=col, mew=0)
        t = np.linspace(tp[1], 70, 300)
        a0 = xp[1]
        a.plot(t - 6, a_star / (1 + (a_star / a0 - 1) * np.exp(-sigma * (t - tp[1]))), color=col, lw=0.8, ls=DASH)
        label_end(a, tp[2] - 6 + 1.5, xp[2] + (0.05 if r > 2.5 else -0.05), lab, col)
    a.axhline(a_star, color=C["ink"], lw=0.6, ls=DOT)
    a.text(63, a_star + 0.025, f"predicted $A^* = {a_star:.2f}$ m", color=C["ink"], fontsize=6.4, ha="right")
    a.set_xlim(0, 64)
    a.set_ylim(0, 0.85)
    a.set_title("Peaks (dots), amplitude equation (dashed)")
    a.set_ylabel("amplitude  [m]")
    xlab_time(a, "time after the step  [s]")

    a = ax[1]
    for name, r, col, _ in runs:
        d = win(load(name), 6.5, 70)
        a.plot(d["pos.y"] - r, d["vel.y"], color=col, lw=0.35, alpha=0.75)
    th = np.linspace(0, 2 * np.pi, 200)
    a.plot(a_star * np.cos(th), a_star * w * np.sin(th), color=C["ink"], lw=0.9, ls=DASH)
    a.set_title("Phase plane")
    a.set_xlabel("deviation $x$  [m]", loc="right")
    a.set_ylabel("velocity  [m/s]")
    a.grid(True, axis="both")
    save(f, "spring_predict")


# ─── Lesson 3 ──────────────────────────────────────────────────────────────
@fig
def droop_exact():
    """The creep after the integral is switched on: simulator, exact third-order model, quasi-static."""
    m, kp, kd, ki = 1.0, 10.0, 7.0, 0.8
    d = win(load("droop-ion"), 6, 40)
    t = np.linspace(0, 30, 600)
    # states: e, e', w = I − m g   (D acts on the measurement, so e'' = −(kp e + kd e' + w)/m)
    e = _linear([[0, 1, 0], [-kp / m, -kd / m, -1 / m], [ki, 0, 0]], [m * G / kp, 0, -m * G], t)[0]
    f, a = plt.subplots(figsize=(TEXT_W, 46 * MM))
    a.plot(d.t - 10, d["alt.err"], color=C["err"], lw=1.4)
    a.plot(t, e, color=C["ink"], lw=0.8, ls=DASH)
    a.plot(t, m * G / kp * np.exp(-t * ki / kp), color=C["accent"], lw=0.9, ls=DOT)
    a.axvline(0, color=C["I"], lw=0.6)
    a.text(0.4, 0.06, "I switched on", color=C["I"], fontsize=6.6)
    a.text(16.5, 0.42, "quasi-static: $e_{ss}\\,e^{-t/12.5}$", color=C["accent"], fontsize=6.4)
    a.text(6.0, 0.72, "simulator, and the exact third-order solution on top of it", color=C["ink"], fontsize=6.4)
    a.set_xlim(-4, 30)
    a.set_ylim(0, 1.05)
    a.set_title("The error after the integral is switched on")
    a.set_ylabel("error  [m]")
    xlab_time(a, "time after switching  [s]")
    save(f, "droop_exact")


@fig
def droop_type():
    f, ax = plt.subplots(1, 2, figsize=(TEXT_W, 54 * MM), gridspec_kw=dict(wspace=0.3))
    kp, kd, cv, v = 10.0, 7.0, 0.25, 0.5

    a = ax[0]
    for on, col, pred, lab in [("measurement", C["D"], (kd * v + cv * v**2) / kp, "D on the measurement"),
                               ("error", C["P"], cv * v**2 / kp, "D on the error")]:
        d = win(load(f"droop-ramp-{on}"), 6, 20)
        a.plot(d.t - 8, d["alt.err"], color=col, lw=1.3)
        a.plot([0.5, 8], [pred, pred], color=C["ink"], lw=0.7, ls=DOT)
        a.text(4.4, pred + (-0.075 if pred > 0.1 else 0.085), f"{lab}\npredicted {pred:.3f} m", color=col,
               fontsize=6.0, ha="center", va="center")
    a.axvspan(0, 8, color=C["mist"], lw=0, zorder=0)
    a.text(0.3, 0.415, "setpoint climbs at 0.5 m/s", color=C["muted"], fontsize=6.4)
    a.set_xlim(-2, 12)
    a.set_ylim(-0.03, 0.45)
    a.set_title("Error while the setpoint ramps (I off)")
    a.set_ylabel("error  [m]")
    xlab_time(a, "time after the ramp starts  [s]")

    a = ax[1]
    rate = 0.05 * G
    t = np.linspace(0, 10, 200)
    for ki, col in [(0.8, C["I"]), (3.0, C["meas"])]:
        d = win(load(f"droop-burn-ki{ki:g}"), 8, 36)
        a.plot(d.t - 10, -d["alt.err"], color=col, lw=1.3)
        a.plot(t, rate / ki * (1 - np.exp(-t * ki / kp)), color=C["ink"], lw=0.7, ls=DASH)
        a.plot([10, 26], [rate / ki, rate / ki], color=col, lw=0.6, ls=DOT)
        a.text(25.8, rate / ki + 0.012, f"$K_i = {ki:g}$:  $\\dot d/K_i = {rate / ki:.2f}$ m", color=col,
               fontsize=6.2, ha="right", va="bottom")
    a.axvspan(0, 10, color=C["mist"], lw=0, zorder=0)
    a.text(0.3, 0.675, "mass falls by 50 g/s", color=C["muted"], fontsize=6.4)
    a.set_xlim(-2, 26)
    a.set_ylim(-0.03, 0.74)
    a.set_title("Height above the setpoint, load ramping")
    a.set_ylabel("$y - r$  [m]")
    xlab_time(a, "time after the burn starts  [s]")
    save(f, "droop_type")


# ─── Lesson 4 ──────────────────────────────────────────────────────────────
def _step_start(name):
    dd = win(load(name), 8, 24)
    return dd.t[dd["sp.y"].diff() > 0.5].iloc[0]


@fig
def damper_predict():
    f, ax = plt.subplots(1, 2, figsize=(TEXT_W, 58 * MM), gridspec_kw=dict(wspace=0.3, width_ratios=[1.35, 1]))
    kp, m = 10.0, 1.0
    t = np.linspace(0, 4, 400)

    a = ax[0]
    for kd, col in [("3", C["P"]), ("6.3", C["I"]), ("15", C["meas"])]:
        t0 = _step_start(f"damper-kd{kd}")
        d = win(load(f"damper-kd{kd}"), t0, t0 + 4)
        x0, v0 = d["pos.y"].iloc[0] - 2.5, d["vel.y"].iloc[0]
        x = _linear([[0, 1], [-kp / m, -float(kd) / m]], [x0, v0], t)[0]
        a.plot(d.t - t0, d["pos.y"], color=col, lw=1.3)
        a.plot(t, 2.5 + x, color=C["ink"], lw=0.7, ls=DASH)
        a.plot([], [], color=col, label=f"$K_d = {kd}$")
    a.plot([], [], color=C["ink"], lw=0.7, ls=DASH, label="formula")
    a.axhline(2.5, color=C["sp"], lw=0.8, ls=DASH)
    a.legend(loc="lower right", handlelength=1.4)
    a.set_xlim(0, 4)
    a.set_title("Step responses: simulator and formula")
    a.set_ylabel("altitude  [m]")
    xlab_time(a, "time after the step  [s]")

    a = ax[1]
    t0 = _step_start("damper-kd3")
    d = win(load("damper-kd3"), t0, t0 + 3.2)
    x, v = d["pos.y"].values - 2.5, d["vel.y"].values
    xs = _linear([[0, 1], [-kp / m, -3.0 / m]], [x[0], v[0]], t)
    a.semilogy(d.t - t0, 0.5 * m * v**2 + 0.5 * kp * x**2, color=C["D"], lw=1.3)
    a.semilogy(t, 0.5 * m * xs[1] ** 2 + 0.5 * kp * xs[0] ** 2, color=C["ink"], lw=0.7, ls=DASH)
    a.set_xlim(0, 3.2)
    a.set_ylim(2e-4, 8)
    a.set_title("Energy $V$ for $K_d = 3$")
    a.set_ylabel("J")
    xlab_time(a, "time after the step  [s]")
    save(f, "damper_predict")


@fig
def damper_spec():
    f, ax = plt.subplots(1, 2, figsize=(TEXT_W, 52 * MM), gridspec_kw=dict(wspace=0.3))
    kp, kd, m = 33.6, 8.0, 1.0
    t = np.linspace(0, 1.6, 300)
    x = _linear([[0, 1], [-kp / m, -kd / m]], [-1, 0], t)[0]
    a = ax[0]
    for step, col in [(0.4, C["meas"]), (1, C["P"])]:
        d = win(load(f"damper-spec-step{step:g}"), 6, 7.6)
        a.plot(d.t - 6, (d["pos.y"] - 2) / step, color=col, lw=1.3, label=f"{step:g} m step")
        ax[1].plot(d.t - 6, d["thrust"], color=col, lw=1.2)
    a.plot(t, 1 + x, color=C["ink"], lw=0.7, ls=DASH, label="formula")
    a.axhspan(0.98, 1.02, color=C["mist"], lw=0, zorder=0)
    a.legend(loc="lower right", handlelength=1.4)
    a.set_xlim(0, 1.6)
    a.set_title("Response, as a fraction of the step")
    xlab_time(a, "time after the step  [s]")
    a = ax[1]
    a.axhline(24.4, color=C["err"], lw=0.6, ls=(0, (2, 2)))
    a.text(1.58, 22.6, "motor limit", color=C["err"], fontsize=6.4, ha="right")
    a.set_xlim(0, 1.6)
    a.set_ylim(0, 26)
    a.set_title("Total thrust")
    a.set_ylabel("N")
    xlab_time(a, "time after the step  [s]")
    save(f, "damper_spec")


# ─── Lesson 5 ──────────────────────────────────────────────────────────────
def _pid_matrix(m, kp, ki, kd):
    """States (x, v, z): deviation, velocity, integral minus the load it must hold."""
    return [[0, 1, 0], [-kp / m, -kd / m, 1 / m], [-ki, 0, 0]]


@fig
def integral_predict():
    f, ax = plt.subplots(1, 2, figsize=(TEXT_W, 56 * MM), gridspec_kw=dict(wspace=0.34))
    m, kp, kd, load_ = 1.4, 10.0, 7.0, 0.4 * G
    a = ax[0]
    t = np.linspace(0, 20, 600)
    for name, ki, col in [("integral-ki0.8", 0.8, C["I"]), ("integral-ki5", 5.0, C["meas"])]:
        d = win(load(name), 10, 30)
        a.plot(d.t - 10, 2 - d["pos.y"], color=col, lw=1.4)
        x = _linear(_pid_matrix(m, kp, ki, kd), [-load_ / kp, 0, -load_], t)
        a.plot(t, -x[0], color=C["ink"], lw=0.8, ls=DASH)
        a.plot(t, load_ / kp * np.exp(-ki / kp * t), color=C["accent"], lw=0.8, ls=DOT)
    a.text(9.5, 0.215, "$K_i = 0.8$", color=C["I"], fontsize=6.6)
    a.text(4.2, 0.06, "$K_i = 5$", color=C["meas"], fontsize=6.6)
    a.text(19.8, 0.375, "dashed: the cubic\ndotted: $e^{-K_i t/K_p}$", color=C["muted"], fontsize=6.2, ha="right", va="top")
    a.set_xlim(0, 20)
    a.set_title("Paying back the payload")
    a.set_ylabel("error  [m]")
    xlab_time(a, "time after I is switched on  [s]")

    a = ax[1]
    for name, ki, col, sig, w in [("integral-limit-ki45", 45, C["I"], -0.0594, 2.6246),
                                  ("integral-limit-ki55", 55, C["err"], 0.0658, 2.8407)]:
        d = win(load(name), 10, 30)
        a.plot(d.t - 10, 100 * d["alt.err"], color=col, lw=1.0)
        tp, xp = _peaks(d.t.values - 10, 100 * d["alt.err"].values)
        k = 1
        te = np.linspace(tp[k], 20, 100)
        a.plot(te, xp[k] * np.exp(sig * (te - tp[k])), color=C["ink"], lw=0.8, ls=DASH)
    a.axhline(0, color=C["faint"], lw=0.5)
    a.text(0.6, 21, "$K_i = 55$: grows", color=C["err"], fontsize=6.6)
    a.text(8, -12.5, "$K_i = 45$: decays", color=C["I"], fontsize=6.6)
    a.set_xlim(0, 20)
    a.set_title("Either side of $K_i = 50$")
    a.set_ylabel("error  [cm]")
    xlab_time(a, "time after a 5 cm step  [s]")
    save(f, "integral_predict")


@fig
def integral_locus():
    """Root locus of m s^3 + Kd s^2 + Kp s + Ki in Ki (m = 1.4 kg, default Kp, Kd)."""
    m, kp, kd = 1.4, 10.0, 7.0
    kis = np.concatenate([np.linspace(0, 5, 200), np.linspace(5, 90, 500)])
    roots = np.array([np.sort_complex(np.roots([m, kd, kp, k])) for k in kis])
    f, a = plt.subplots(figsize=(TEXT_W, 56 * MM))
    a.axvspan(0, 1.2, color="#FBEDEA", lw=0)
    a.plot(roots.real, roots.imag, ".", color=C["hair"], ms=1.6)
    for k, col, lab, dx, dy in [(0.8, C["I"], "0.8", 0.0, 0.32), (5, C["meas"], "5", 0.0, 0.32),
                                (20, C["FF"], "20", 0.12, 0.2), (50, C["err"], "50", 0.3, 0.12)]:
        r = np.roots([m, kd, kp, k])
        a.plot(r.real, r.imag, "x", color=col, ms=5.5, mew=1.4)
        top = r[np.argmax(r.imag + 1e-3 * r.real)]
        a.text(top.real + dx, top.imag + dy, f"$K_i = {lab}$", color=col, fontsize=6.5, ha="center")
    a.plot([np.roots([m, kd, kp])[0].real, np.roots([m, kd, kp])[1].real, 0], [0, 0, 0], "o", mfc="white",
           color=C["ink"], ms=3.5, mew=0.8)
    a.axhline(0, color=C["faint"], lw=0.5)
    a.axvline(0, color=C["faint"], lw=0.5)
    a.text(0.15, -3.3, "unstable", color=C["err"], fontsize=6.6)
    a.text(-6.3, -3.3, "circles: $K_i = 0$", color=C["muted"], fontsize=6.4)
    a.set_xlim(-6.5, 1.2)
    a.set_ylim(-3.6, 3.6)
    a.grid(False)
    a.set_xlabel("Re $s$  [1/s]", loc="right")
    a.set_ylabel("Im $s$  [rad/s]")
    a.set_title("Where the three poles go as the integral gain grows ($m = 1.4$ kg)")
    save(f, "integral_locus")
