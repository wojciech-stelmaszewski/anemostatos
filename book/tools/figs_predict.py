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


# ─── Lesson 6 ──────────────────────────────────────────────────────────────
def windup_model(ki=3.0, kp=10.0, kd=7.0, tmax=12.0, cv=0.25, h=4.0, dt=0.0005, t_end=14.0):
    """The windup lesson as a model: climb at full thrust against drag while saturated, then the
    PID (ideal motors). Returns t, y − y0, integral, saturated flag."""
    n = int(t_end / dt)
    y = v = integ = 0.0
    out = np.zeros((n, 4))
    for k in range(n):
        e = h - y
        integ += ki * e * dt
        u = G + kp * e + integ - kd * v
        sat = u > tmax or u < 0
        thrust = min(max(u, 0.0), tmax)
        v += (thrust - G - cv * abs(v) * v) * dt
        y += v * dt
        out[k] = (k * dt, y, integ, sat)
    return out.T


@fig
def windup_predict():
    f, ax = plt.subplots(1, 2, figsize=(TEXT_W, 56 * MM), gridspec_kw=dict(wspace=0.32))
    d = win(load("windup-none"), 7, 22)
    t, y, integ, sat = windup_model()
    a_, vt = 12 - G, np.sqrt((12 - G) / 0.25)
    tc = np.linspace(0, 2.24, 200)
    a = ax[0]
    a.plot(d.t - 8, d["pos.y"], color=C["meas"], lw=1.4)
    a.plot(t, 2 + y, color=C["ink"], lw=0.8, ls=DASH)
    a.plot(tc, 2 + vt**2 / a_ * np.log(np.cosh(a_ * tc / vt)), color=C["accent"], lw=1.1, ls=DOT)
    a.axhline(6, color=C["sp"], lw=0.8, ls=DASH)
    a.text(2.5, 3.2, "full thrust against drag:\n$\\frac{v_t^2}{a}\\ln\\cosh\\frac{a t}{v_t}$", color=C["accent"], fontsize=6.4)
    a.text(6.0, 6.9, "simulator, and the\nmodel of Example 6.2", color=C["ink"], fontsize=6.4)
    a.set_xlim(-1, 14)
    a.set_title("Altitude")
    a.set_ylabel("m")
    xlab_time(a, "time after the step  [s]")
    a = ax[1]
    a.plot(d.t - 8, d["alt.part.i"], color=C["I"], lw=1.4)
    a.plot(t, integ, color=C["ink"], lw=0.8, ls=DASH)
    k = np.argmax(y >= 4)
    a.plot(t[k], integ[k], "o", color=C["accent"], ms=3.5)
    a.annotate(f"predicted at arrival: {integ[k]:.1f} N", (t[k], integ[k]), xytext=(4.2, 14.5), fontsize=6.4,
               color=C["accent"], arrowprops=dict(arrowstyle="-", color=C["accent"], lw=0.5))
    a.set_xlim(-1, 14)
    a.set_title("The integral winds up, then unwinds")
    a.set_ylabel("N")
    xlab_time(a, "time after the step  [s]")
    save(f, "windup_predict")


# ─── Lesson 7 ──────────────────────────────────────────────────────────────
def weighted_step(b, c, kp=10.0, ki=0.8, kd=7.0, m=1.0, t_end=12.0, n=4000):
    """Unit step response of m y''' + kd y'' + kp y' + ki y = c kd r'' + b kp r' + ki r (ideal loop).
    The step sets the initial conditions y(0+) = 0, y'(0+) = c kd/m, y''(0+) = b kp/m − c kd²/m²."""
    t = np.linspace(0, t_end, n)
    a = [[0, 1, 0], [0, 0, 1], [-ki / m, -kp / m, -kd / m]]
    x0 = [-1.0, c * kd / m, b * kp / m - c * kd**2 / m**2]  # in terms of y − 1
    return t, 1 + _linear(a, x0, t)[0]


@fig
def kick_predict():
    f, ax = plt.subplots(1, 2, figsize=(TEXT_W, 56 * MM), gridspec_kw=dict(wspace=0.34))
    a = ax[0]
    dt, kd = 0.004, 7.0
    for name, fc, col in [("kick-error", 0, C["D"]), ("kick-error-f5", 5, C["accent"])]:
        d = win(load(name), 11.99, 12.26)
        a.semilogy(1000 * (d.t - 12.005), d["alt.part.d"].clip(lower=0.05), "o", color=col, ms=2.6)
    tf = 1 / (2 * np.pi * 5)
    alpha = tf / (tf + dt)
    k = np.arange(0, 60)
    a.semilogy(1000 * k * dt, kd / dt * (1 - alpha) * alpha**k, color=C["ink"], lw=0.8, ls=DASH)
    a.axhline(24.4 - 19.81, color=C["err"], lw=0.6, ls=DOT)
    a.text(250, 5.6, "headroom of the motors", color=C["err"], fontsize=6.2, ha="right")
    a.text(14, 1750, "no filter: 1750 N, one sample", color=C["D"], fontsize=6.4, va="center")
    a.text(60, 60, "5 Hz filter: $195\\,\\alpha^k$ N", color=C["accent"], fontsize=6.4)
    a.set_xlim(-10, 250)
    a.set_ylim(0.5, 4000)
    a.set_title("The kick, sample by sample")
    a.set_ylabel("D term  [N]")
    a.set_xlabel("time after the step  [ms]", loc="right")
    a.grid(True, which="major", axis="y")

    a = ax[1]
    d = win(load("kick-measurement"), 11.5, 15)
    a.plot(d.t - 12.005, (d["pos.y"] - 1.5), color=C["I"], lw=1.4)
    t, y = weighted_step(1, 0, t_end=3)
    a.plot(t, y, color=C["ink"], lw=0.8, ls=DASH)
    t, y = weighted_step(0, 0, t_end=3)
    a.plot(t, y, color=C["meas"], lw=1.0, ls=DOT)
    t, y = weighted_step(0, 0, ki=8, t_end=3)
    a.plot(t, y, color=C["meas"], lw=1.0)
    a.axhline(1, color=C["sp"], lw=0.8, ls=DASH)
    a.text(1.0, 0.72, "PI-D: simulator,\nand its model", color=C["I"], fontsize=6.4)
    a.text(1.85, 0.07, "I-PD, $K_i = 0.8$", color=C["meas"], fontsize=6.4)
    a.text(1.75, 0.43, "I-PD, $K_i = 8$", color=C["meas"], fontsize=6.4)
    a.set_xlim(-0.3, 3)
    a.set_title("Where the setpoint enters")
    a.set_ylabel("altitude, fraction of the step")
    xlab_time(a, "time after the step  [s]")
    save(f, "kick_predict")


# ─── Lesson 8 ──────────────────────────────────────────────────────────────
def d_noise_std(fc, dt=0.004, sigma=0.01, kd=7.0):
    """Standard deviation of the D term fed by white altimeter noise through the first-order filter."""
    if fc <= 0:
        return kd * np.sqrt(2) * sigma / dt
    tf = 1 / (2 * np.pi * fc)
    al = tf / (tf + dt)
    return kd * sigma / dt * (1 - al) * np.sqrt(2 / (1 + al))


@fig
def noise_predict():
    f, ax = plt.subplots(1, 2, figsize=(TEXT_W, 56 * MM), gridspec_kw=dict(wspace=0.34))
    a = ax[0]
    fcs = np.logspace(-0.2, 2.35, 200)
    a.loglog(fcs, [d_noise_std(x) for x in fcs], color=C["ink"], lw=0.9, ls=DASH)
    a.axhline(d_noise_std(0), color=C["ink"], lw=0.7, ls=DOT)
    for fc in [50, 20, 10, 5, 2, 1]:
        d = win(load(f"noise-f{fc}"), 8, 20)
        a.plot(fc, d["alt.part.d"].std(), "o", color=C["D"], ms=4)
    d = win(load("noise-f0"), 8, 20)
    a.plot(180, d["alt.part.d"].std(), "o", color=C["D"], ms=4)
    a.text(170, 33, "no filter: $\\sqrt{2}K_d\\sigma/\\Delta t$", fontsize=6.3, color=C["ink"], ha="right")
    a.text(1.1, 3.2, "formula of\nExample 8.2", fontsize=6.3, color=C["ink"])
    a.text(6, 0.55, "simulator", fontsize=6.3, color=C["D"])
    a.set_xlim(0.7, 250)
    a.set_ylim(0.3, 60)
    a.set_xlabel("cutoff of the D filter  [Hz]", loc="right")
    a.set_ylabel("fluctuation of the D term  [N]")
    a.set_title("Noise in the D term")
    a.grid(True, which="major", axis="both")

    a = ax[1]
    d = win(load("noise-f0"), 3, 20)
    k = 300
    roll = lambda s: s.rolling(k, center=True).mean()
    a.plot(d.t, roll(d["alt.part.p"]), color=C["P"], lw=1.1)
    a.plot(d.t, roll(d["alt.part.i"]), color=C["I"], lw=1.1)
    a.plot(d.t, roll(d["alt.part.p"] + d["alt.part.i"]), color=C["ink"], lw=1.1)
    a.axhline(-4.0, color=C["accent"], lw=0.9, ls=DOT)
    a.text(19.8, -4.45, "predicted: −4.0 N", color=C["accent"], fontsize=6.4, ha="right")
    a.text(16.5, -1.0, "P", color=C["P"], fontsize=6.6)
    a.text(15, -2.25, "I", color=C["I"], fontsize=6.6)
    a.text(12.8, -3.0, "P + I", color=C["ink"], fontsize=6.6)
    a.set_ylim(-5, 0.3)
    a.set_title("What the clipped noise costs")
    a.set_ylabel("N  (3 s averages)")
    xlab_time(a)
    save(f, "noise_predict")


# ─── Lesson 9 ──────────────────────────────────────────────────────────────
def _pade(T, n=6):
    from math import factorial
    c = [factorial(2 * n - k) * factorial(n) / (factorial(2 * n) * factorial(k) * factorial(n - k)) for k in range(n + 1)]
    return (np.array([c[k] * (-T) ** k for k in range(n, -1, -1)]),
            np.array([c[k] * T**k for k in range(n, -1, -1)]))


def delay_poles(T, kp=10.0, ki=0.8, kd=7.0, fc=20.0, tau=0.03, m=1.0, zoh=0.002):
    """Poles of the altitude loop with a sensor delay T (Padé approximation of order six), sorted by real part."""
    P = np.polymul
    tf = 1 / (2 * np.pi * fc)
    num, den = _pade(T + zoh)
    plant = P(P([m, 0, 0, 0], [tau, 1]), [tf, 1])
    ctrl = np.polyadd(np.polyadd(P([kp, 0], [tf, 1]), P([ki], [tf, 1])), [kd, 0, 0])
    r = np.roots(np.polyadd(P(plant, den), P(ctrl, num)))
    return r[np.argsort(-r.real)]


def sampled_loop(rate, kp=10.0, ki=0.8, kd=7.0, m=1.0, tau=0.03, fc=20.0):
    """One-period transition matrix of the altitude loop sampled at `rate`, states (y, v, T, I, D, y_prev)."""
    from scipy.linalg import expm
    ts = 1 / rate
    a = np.array([[0, 1, 0], [0, 0, 1 / m], [0, 0, -1 / tau]])
    b = np.array([[0], [0], [1 / tau]])
    mm = expm(np.block([[a, b], [np.zeros((1, 4))]]) * ts)
    ad, bd = mm[:3, :3], mm[:3, 3:]
    tf = 1 / (2 * np.pi * fc)
    al = tf / (tf + ts)
    n = 6
    i_new = np.zeros(n); i_new[3] = 1; i_new[0] = -ki * ts
    d_new = np.zeros(n); d_new[4] = al; d_new[0] = -(1 - al) / ts; d_new[5] = (1 - al) / ts
    u = -kp * np.eye(n)[0] + i_new + kd * d_new
    f = np.zeros((n, n))
    f[:3, :3] = ad
    f[:3, :] += bd @ u[None, :]
    f[3, :], f[4, :], f[5, 0] = i_new, d_new, 1
    return f


@fig
def slow_predict():
    f, ax = plt.subplots(1, 2, figsize=(TEXT_W, 56 * MM), gridspec_kw=dict(wspace=0.36))
    a = ax[0]
    ts = np.linspace(0.06, 0.22, 60)
    per, re = [], []
    for T in ts:
        r = delay_poles(T)
        c = [x for x in r if x.imag > 0.5][0]
        per.append(2 * np.pi / c.imag)
        re.append(c.real)
    a.axvspan(155, 230, color="#FBEDEA", lw=0)
    a.plot(1000 * ts, per, color=C["ink"], lw=0.9, ls=DASH)
    a.plot([130, 150, 170, 200], [0.79, 0.86, 0.92, 1.08], "o", color=C["meas"], ms=4)
    a.text(158, 0.62, "unstable\nbeyond 155 ms", color=C["err"], fontsize=6.3)
    a.text(62, 0.98, "period of the ringing:\nmodel (dashed), simulator (dots)", color=C["ink"], fontsize=6.3)
    a.set_xlim(60, 225)
    a.set_ylim(0.55, 1.15)
    a.set_xlabel("sensor delay  [ms]", loc="right")
    a.set_ylabel("period  [s]")
    a.set_title("Delay: the ringing slows down")

    a = ax[1]
    rates = np.logspace(np.log10(2.5), np.log10(60), 200)
    rho = [np.abs(np.linalg.eigvals(sampled_loop(r))).max() for r in rates]
    a.axvspan(2.5, 5.9, color="#FBEDEA", lw=0)
    a.semilogx(rates, rho, color=C["ink"], lw=0.9, ls=DASH)
    a.axhline(1, color=C["err"], lw=0.6)
    for r, lab, dy in [(7, "7 Hz: settles", 0.05), (5, "5 Hz: never settles", 0.06), (3, "3 Hz", 0.04)]:
        v = np.abs(np.linalg.eigvals(sampled_loop(r))).max()
        a.plot(r, v, "o", color=C["meas"], ms=4)
        a.text(r * 1.12, v + dy, lab, fontsize=6.3, color=C["meas"])
    a.text(2.7, 0.93, "unstable below 5.9 Hz", color=C["err"], fontsize=6.3)
    a.set_xlim(2.5, 60)
    a.set_ylim(0.9, 1.6)
    a.set_xticks([3, 5, 10, 20, 50])
    a.set_xticklabels(["3", "5", "10", "20", "50"])
    a.set_xlabel("controller rate  [Hz]", loc="right")
    a.set_ylabel("largest $|$eigenvalue$|$ per sample")
    a.set_title("Sampling: the exact discrete loop")
    save(f, "slow_predict")
