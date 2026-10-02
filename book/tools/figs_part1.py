"""Figures for Part I (lessons I.1–I.14), drawn from the simulator runs in book/data."""

import matplotlib.pyplot as plt
import numpy as np

from figlib import (C, FULL_W, MM, SEQ, TEXT_W, fig, label_end, load, note, ramp, sat_shade,
                    save, step_metrics, win, xlab_time)


# ─── Lesson I.1 ──────────────────────────────────────────────────────────────
@fig
def meet_overview():
    d = win(load("meet"), 4, 40)
    f, ax = plt.subplots(4, 1, figsize=(TEXT_W, 118 * MM), sharex=True,
                         gridspec_kw=dict(height_ratios=[1.25, 1, 1.25, 0.9], hspace=0.42))
    a = ax[0]
    a.plot(d.t, d["sp.y"], color=C["sp"], ls=(0, (4, 2.5)), lw=1)
    a.plot(d.t, d["meas.y"], color=C["meas"])
    a.set_title("Altitude: setpoint and measurement", loc="left")
    a.set_ylabel("m")
    label_end(a, 40.3, d["sp.y"].iloc[-1] + 0.06, "setpoint $r$", C["sp"])
    label_end(a, 40.3, d["meas.y"].iloc[-1] - 0.07, "measured $y$", C["meas"])
    a = ax[1]
    a.plot(d.t, d["alt.err"], color=C["err"])
    a.axhline(0, color=C["faint"], lw=0.5)
    a.set_title("Error  $e = r - y$", loc="left")
    a.set_ylabel("m")
    a = ax[2]
    for k, c, lab in [("p", C["P"], "P"), ("i", C["I"], "I"), ("d", C["D"], "D")]:
        a.plot(d.t, d[f"alt.part.{k}"], color=c, lw=1.0)
        label_end(a, 40.3, d[f"alt.part.{k}"].iloc[-1], lab, c)
    a.set_title("The three terms (feedback part of the thrust, the 9.81 N feedforward not shown)", loc="left")
    a.set_ylabel("N")
    a = ax[3]
    a.plot(d.t, d["wind.y"], color=C["wind"], lw=0.9)
    a.axhline(0, color=C["faint"], lw=0.5)
    a.set_title("Vertical wind — what the controller never sees", loc="left")
    a.set_ylabel("m/s")
    xlab_time(a)
    a.set_xlim(4, 40)
    save(f, "meet_overview")


@fig
def meet_off():
    d = load("meet-off")
    d = win(d, 5, 14)
    f, a = plt.subplots(figsize=(MM * 46, MM * 42))
    a.plot(d.t, d["sp.y"], color=C["sp"], ls=(0, (4, 2.5)), lw=1)
    a.plot(d.t, d["pos.y"], color=C["meas"])
    a.axvline(10, color=C["err"], lw=0.6)
    a.text(10.2, 2.35, "controller\noff", color=C["err"], fontsize=6.4, va="top")
    a.set_ylim(-0.1, 2.5)
    a.set_xticks([6, 8, 10, 12, 14])
    a.set_ylabel("altitude  [m]")
    a.set_xlabel("time  [s]", loc="right")
    save(f, "meet_off")


def _setpoint(a, d, **kw):
    a.plot(d.t, d["sp.y"], color=C["sp"], ls=(0, (4, 2.5)), lw=0.9, **kw)


# ─── Lesson I.2 ──────────────────────────────────────────────────────────────
@fig
def spring_run():
    f, ax = plt.subplots(2, 1, figsize=(TEXT_W, 78 * MM), sharex=True,
                         gridspec_kw=dict(height_ratios=[1.5, 1], hspace=0.35))
    for name, col, lab in [("spring-kp10", C["meas"], "$K_p = 10$"), ("spring-kp40", C["P"], "$K_p = 40$")]:
        d = win(load(name), 4, 20)
        ax[0].plot(d.t, d["pos.y"], color=col, lw=1.1)
        ax[1].plot(d.t, d["thrust"], color=col, lw=0.9)
        label_end(ax[0], 20.2, d["pos.y"].iloc[-1], lab, col)
    _setpoint(ax[0], win(load("spring-kp10"), 4, 20))
    ax[0].set_title("Altitude after the setpoint jumps from 2 to 3 m at $t = 6$ s")
    ax[0].set_ylabel("m")
    note(ax[0], 9.01, 3.67, "period 2.0 s", 10.3, 4.15, color=C["meas"])
    note(ax[0], 8.41, 3.91, "period 1.4 s, not 1.0 s", 7.3, 4.45, color=C["P"], rad=-0.2)
    a = ax[1]
    a.axhline(24.4, color=C["err"], lw=0.6, ls=(0, (2, 2)))
    a.axhline(0, color=C["err"], lw=0.6, ls=(0, (2, 2)))
    a.text(4.1, 21.5, "motor limit 4 × 6.1 N", color=C["err"], fontsize=6.4)
    a.set_title("Total thrust")
    a.set_ylabel("N")
    a.set_ylim(-2, 27)
    xlab_time(a)
    save(f, "spring_run")


# ─── Lesson I.3 ──────────────────────────────────────────────────────────────
@fig
def droop_run():
    f, ax = plt.subplots(1, 2, figsize=(TEXT_W, 50 * MM), gridspec_kw=dict(wspace=0.32))
    a = ax[0]
    for name, col, kp in [("droop-kp10", C["meas"], 10), ("droop-kp40", C["P"], 40)]:
        d = win(load(name), 0, 16)
        a.plot(d.t, d["pos.y"], color=col)
        pred = 2 - 9.81 / kp
        a.axhline(pred, color=col, lw=0.6, ls=(0, (1, 1.5)))
        a.text(16.3, pred, f"$2 - mg/{kp}$\n= {pred:.2f} m", color=col, fontsize=6.3, va="center")
    _setpoint(a, win(load("droop-kp10"), 0, 16))
    a.set_title("Altitude, P only, no feedforward")
    a.set_ylabel("m")
    xlab_time(a)
    a.set_xlim(0, 16)
    a = ax[1]
    for name, col in [("droop-kp10", C["meas"]), ("droop-kp40", C["P"])]:
        d = win(load(name), 0, 16)
        a.plot(d.t, d["alt.part.p"], color=C["P"] if col == C["P"] else C["P"], lw=1.0,
               ls="-" if col == C["P"] else (0, (3, 1.5)))
    a.axhline(9.81, color=C["faint"], lw=0.6)
    a.text(0.3, 10.6, "$mg = 9.81$ N", color=C["muted"], fontsize=6.4)
    a.set_title("The P term does the feedforward's job")
    a.set_ylabel("N")
    a.set_ylim(0, 25)
    xlab_time(a)
    save(f, "droop_run")


@fig
def droop_ion():
    d = win(load("droop-ion"), 2, 40)
    f, ax = plt.subplots(2, 1, figsize=(TEXT_W, 70 * MM), sharex=True, gridspec_kw=dict(hspace=0.35))
    a = ax[0]
    _setpoint(a, d)
    a.plot(d.t, d["pos.y"], color=C["meas"])
    a.axvline(10, color=C["I"], lw=0.6)
    a.text(10.3, 1.2, "I switched on", color=C["I"], fontsize=6.6)
    a.set_title("Altitude")
    a.set_ylabel("m")
    a = ax[1]
    a.plot(d.t, d["alt.part.p"], color=C["P"])
    a.plot(d.t, d["alt.part.i"], color=C["I"])
    a.axhline(9.81, color=C["faint"], lw=0.6, ls=(0, (2, 2)))
    label_end(a, 40.4, d["alt.part.p"].iloc[-1], "P", C["P"])
    label_end(a, 40.4, d["alt.part.i"].iloc[-1], "I", C["I"])
    a.text(2.3, 10.6, "$mg$", color=C["muted"], fontsize=6.6)
    a.set_title("The integral slowly takes over the weight from P")
    a.set_ylabel("N")
    xlab_time(a)
    a.set_xlim(2, 40)
    save(f, "droop_ion")


# ─── Lesson I.4 ──────────────────────────────────────────────────────────────
@fig
def damper_sweep():
    f, a = plt.subplots(figsize=(TEXT_W, 62 * MM))
    kds = ["0.8", "3", "6.3", "15", "30"]
    cols = [C["err"], C["P"], C["I"], C["meas"], C["D"]]
    dd = win(load("damper-kd6.3"), 8, 24)
    t0 = dd.t[dd["sp.y"].diff() > 0.5].iloc[0]
    d0 = win(dd, t0 - 0.5, t0 + 4)
    for kd, col in zip(kds, cols):
        d = win(load(f"damper-kd{kd}"), t0 - 0.5, t0 + 4)
        zeta = float(kd) / (2 * np.sqrt(10))
        a.plot(d.t - t0, d["pos.y"], color=col, lw=1.1)
        a.plot([], [], color=col, label=f"$K_d = {kd}$,  $\\zeta = {zeta:.2f}$")
    a.plot(d0.t - t0, d0["sp.y"], color=C["sp"], ls=(0, (4, 2.5)), lw=0.9)
    a.set_xlim(-0.5, 4)
    a.legend(loc="lower right", handlelength=1.2)
    a.set_title("One upward step of the square wave, five dampers ($K_p = 10$)")
    a.set_ylabel("altitude  [m]")
    xlab_time(a, "time after the step  [s]")
    # fix label collisions
    save(f, "damper_sweep")


# ─── Lesson I.5 ──────────────────────────────────────────────────────────────
@fig
def integral_run():
    f, ax = plt.subplots(2, 1, figsize=(TEXT_W, 74 * MM), sharex=True, gridspec_kw=dict(hspace=0.35))
    runs = [("integral-off", C["faint"], "I off"), ("integral-ki0.8", C["I"], "$K_i = 0.8$"),
            ("integral-ki5", C["meas"], "$K_i = 5$")]
    for name, col, lab in runs:
        d = win(load(name), 4, 40)
        ax[0].plot(d.t, d["pos.y"], color=col, lw=1.1)
        ax[1].plot(d.t, d["alt.part.i"], color=col, lw=1.1)
        label_end(ax[0], 40.4, d["pos.y"].iloc[-1] + (0.03 if "5" in lab else -0.03 if "0.8" in lab else 0), lab, col)
    _setpoint(ax[0], win(load("integral-off"), 4, 40))
    ax[0].axvline(10, color=C["I"], lw=0.6)
    ax[0].text(10.3, 1.52, "I switched on", color=C["I"], fontsize=6.6)
    ax[0].set_title("Altitude of a 1.4 kg drone that the controller believes weighs 1 kg")
    ax[0].set_ylabel("m")
    ax[1].axhline(0.4 * 9.81, color=C["faint"], lw=0.6, ls=(0, (2, 2)))
    ax[1].text(30, 3.3, "missing weight 3.92 N", color=C["muted"], fontsize=6.5, ha="center")
    ax[1].set_title("The integral term")
    ax[1].set_ylabel("N")
    xlab_time(ax[1])
    ax[1].set_xlim(4, 40)
    save(f, "integral_run")


@fig
def integral_area():
    f, ax = plt.subplots(2, 1, figsize=(TEXT_W, 66 * MM), sharex=True, gridspec_kw=dict(hspace=0.4))
    for name, col, lab in [("integral-step-ki0.8", C["I"], "$K_i = 0.8$"), ("integral-step-ki3", C["meas"], "$K_i = 3$")]:
        d = win(load(name), 9, 26)
        t, e = d.t.values, d["alt.err"].values
        if "3" in name:
            ax[0].fill_between(t, 0, e, where=e > 0, color=C["I"], alpha=0.22, lw=0)
            ax[0].fill_between(t, 0, e, where=e < 0, color=C["err"], alpha=0.35, lw=0)
        ax[0].plot(t, e, color=col, lw=1.0, label=lab)
        ax[1].plot(t, d["alt.part.i"], color=col, lw=1.1)
    a = ax[0]
    a.axhline(0, color=C["faint"], lw=0.5)
    a.set_ylim(-0.25, 1.05)
    a.legend(loc="upper right")
    a.text(10.9, 0.45, "positive area:\nthe integral grows", color=C["I"], fontsize=6.5)
    note(a, 13.2, -0.1, "negative area: the integral shrinks back —\npossible only above the setpoint", 15.2, 0.35,
         color=C["err"], rad=0.25)
    a.set_title("Error after a 1 m setpoint step at $t = 10$ s")
    a.set_ylabel("$e$  [m]")
    a = ax[1]
    a.axhline(0, color=C["faint"], lw=0.5)
    a.set_title("The integral term must end where it started: the load has not changed")
    a.set_ylabel("N")
    xlab_time(a)
    a.set_xlim(9, 26)
    save(f, "integral_area")


# ─── Lesson I.6 ──────────────────────────────────────────────────────────────
@fig
def windup_compare():
    f, ax = plt.subplots(3, 1, figsize=(FULL_W, 104 * MM), sharex=True,
                         gridspec_kw=dict(height_ratios=[1.5, 1, 1], hspace=0.38))
    runs = [("windup-none", C["err"], "no anti-windup"), ("windup-clamp", C["I"], "conditional integration"),
            ("windup-back-calculation", C["meas"], "back-calculation")]
    for name, col, lab in runs:
        d = win(load(name), 6, 30)
        ax[0].plot(d.t, d["pos.y"], color=col, lw=1.15)
        ax[1].plot(d.t, d["alt.part.i"], color=col, lw=1.1)
        ax[2].plot(d.t, d["thrust"], color=col, lw=1.0)
    dn = win(load("windup-none"), 6, 30)
    sat_shade(ax[2], dn.t, dn["alt.sat"])
    _setpoint(ax[0], dn)
    for name, col, lab in runs:
        d = win(load(name), 6, 30)
        ax[0].plot([], [], color=col, label=lab + f"  (peak {d['pos.y'].max():.2f} m)")
    ax[0].legend(loc="lower right")
    ax[0].set_title("Altitude: setpoint jumps to 6 m at $t = 8$ s with motors that can barely lift the drone")
    ax[0].set_ylabel("m")
    ax[1].set_title("Integral term")
    ax[1].set_ylabel("N")
    ax[2].axhline(12, color=C["err"], lw=0.6, ls=(0, (2, 2)))
    ax[2].text(29.9, 11.4, "4 × 3 N limit", color=C["err"], fontsize=6.4, ha="right")
    ax[2].set_title("Total thrust (red shading: output saturated, no anti-windup)")
    ax[2].set_ylabel("N")
    xlab_time(ax[2])
    ax[2].set_xlim(6, 30)
    save(f, "windup_compare")


# ─── Lesson I.7 ──────────────────────────────────────────────────────────────
@fig
def kick_compare():
    f = plt.figure(figsize=(FULL_W, 62 * MM))
    gs = f.add_gridspec(2, 2, width_ratios=[1.6, 1], hspace=0.5, wspace=0.22)
    a0 = f.add_subplot(gs[0, 0])
    a1 = f.add_subplot(gs[1, 0], sharex=a0)
    a2 = f.add_subplot(gs[:, 1])
    runs = [("kick-error", C["D"], "D on error"), ("kick-measurement", C["I"], "D on measurement")]
    for name, col, lab in runs:
        d = win(load(name), 9, 17)
        a0.plot(d.t, d["pos.y"], color=col, lw=1.1, label=lab)
        a1.plot(d.t, d["alt.part.d"].clip(-40, 40), color=col, lw=1.0)
        z = win(load(name), 11.95, 12.45)
        cmd = sum(z[f"motorCmd.{i}"] for i in range(1, 5))
        a2.step(z.t, cmd, where="post", color=col, lw=1.1, label=lab + ": command")
        a2.plot(z.t, z["thrust"], color=col, lw=0.8, ls=(0, (2, 1.5)))
    _setpoint(a0, win(load("kick-error"), 9, 17))
    a0.legend(loc="upper left", ncol=2)
    a0.set_title("Altitude — the two runs are indistinguishable")
    a0.set_ylabel("m")
    a0.set_ylim(1.35, 2.75)
    a1.set_title("D term (clipped at ±40 N)")
    a1.set_ylabel("N")
    a1.annotate("+1750 N", (12.0, 40), xytext=(12.35, 30), fontsize=6.4, color=C["D"],
                arrowprops=dict(arrowstyle="-", color=C["D"], lw=0.5))
    a1.annotate("−1750 N", (15.0, -40), xytext=(15.35, -30), fontsize=6.4, color=C["D"],
                arrowprops=dict(arrowstyle="-", color=C["D"], lw=0.5))
    xlab_time(a1)
    a2.axhline(24.4, color=C["err"], lw=0.6, ls=(0, (2, 2)))
    a2.text(12.44, 23.3, "motor limit", color=C["err"], fontsize=6.3, ha="right")
    a2.set_title("Zoom on the step at 12 s: total thrust")
    a2.plot([], [], color=C["muted"], lw=0.8, ls=(0, (2, 1.5)), label="actual thrust (30 ms lag)")
    a2.set_ylabel("N")
    a2.set_ylim(5, 26)
    a2.legend(loc="lower right", fontsize=6.2)
    xlab_time(a2)
    save(f, "kick_compare")


# ─── Lesson I.8 ──────────────────────────────────────────────────────────────
@fig
def noise_filters():
    runs = [("noise-f0", "no filter", C["err"]), ("noise-f20", "20 Hz", C["P"]),
            ("noise-f5", "5 Hz", C["I"]), ("noise-f1", "1 Hz", C["meas"])]
    f, ax = plt.subplots(3, 4, figsize=(FULL_W, 92 * MM), sharex=True, sharey="row",
                         gridspec_kw=dict(hspace=0.35, wspace=0.12))
    for j, (name, lab, col) in enumerate(runs):
        d = win(load(name), 10, 14)
        ax[0, j].plot(d.t, d["alt.part.d"], color=C["D"], lw=0.6)
        ax[1, j].plot(d.t, d["thrust"], color=col, lw=0.7)
        ax[2, j].plot(d.t, d["pos.y"], color=C["meas"], lw=0.9)
        ax[2, j].plot(d.t, d["sp.y"], color=C["sp"], lw=0.8, ls=(0, (4, 2.5)))
        ax[0, j].set_title("D filter: " + lab, loc="left")
        e = win(load(name), 8, 20)
        rms = 100 * np.sqrt(((e["sp.y"] - e["pos.y"]) ** 2).mean())
        ax[2, j].text(0.03, 0.06, f"RMS error {rms:.1f} cm", transform=ax[2, j].transAxes,
                      fontsize=6.3, color=C["muted"])
    ax[0, 0].set_ylabel("D term  [N]")
    ax[1, 0].set_ylabel("thrust  [N]")
    ax[2, 0].set_ylabel("altitude  [m]")
    ax[0, 0].set_ylim(-60, 60)
    ax[1, 0].set_ylim(0, 25)
    ax[2, 0].set_ylim(1.4, 2.6)
    for a in ax[2]:
        a.set_xticks([10, 12, 14])
    xlab_time(ax[2, 3])
    save(f, "noise_filters")


@fig
def noise_bode():
    """Gain of an ideal and a filtered differentiator; the noise band."""
    w = np.logspace(-1, 3.2, 400)
    f, a = plt.subplots(figsize=(TEXT_W, 52 * MM))
    a.loglog(w / (2 * np.pi), w, color=C["err"], lw=1.1, label="ideal derivative  $|j\\omega|$")
    for fc, col in [(20, C["P"]), (5, C["I"]), (1, C["meas"])]:
        wc = 2 * np.pi * fc
        g = w / np.sqrt(1 + (w / wc) ** 2)
        a.loglog(w / (2 * np.pi), g, color=col, lw=1.1, label=f"filtered, cutoff {fc} Hz")
    a.axvspan(0.05, 1.5, color=C["mist"], lw=0, zorder=0)
    a.text(0.1, 900, "the drone's\nmotion", color=C["muted"], fontsize=6.5, va="top")
    a.axvspan(20, 125, color="#FBEDEA", lw=0, zorder=0)
    a.text(24, 900, "sensor noise\n(up to Nyquist, 125 Hz)", color=C["err"], fontsize=6.5, va="top")
    a.set_xlim(0.05, 200)
    a.set_ylim(0.1, 2000)
    a.set_xlabel("frequency  [Hz]", loc="right")
    a.set_ylabel("gain  [1/s]")
    a.legend(loc="lower right")
    a.grid(True, which="major", axis="both", color="#EEF0F3", lw=0.4)
    a.set_title("How much a differentiator amplifies each frequency")
    save(f, "noise_bode")


# ─── Lesson I.9 ──────────────────────────────────────────────────────────────
@fig
def slow_rates():
    f, ax = plt.subplots(1, 2, figsize=(FULL_W, 58 * MM), gridspec_kw=dict(wspace=0.18))
    a = ax[0]
    runs = [("slow-250hz", "250 Hz", C["faint"]), ("slow-10hz", "10 Hz", C["meas"]),
            ("slow-7hz", "7 Hz", C["I"]), ("slow-5hz", "5 Hz", C["err"])]
    ref = win(load("slow-250hz"), 8, 30)
    t0 = ref.t[ref["sp.y"].diff() > 0.5].iloc[0]
    for name, lab, col in runs:
        d = win(load(name), t0 - 0.5, t0 + 4)
        a.plot(d.t - t0, d["pos.y"], color=col, lw=1.1, label=lab)
    a.plot(ref.t - t0, ref["sp.y"], color=C["sp"], ls=(0, (4, 2.5)), lw=0.9)
    a.set_xlim(-0.5, 4)
    a.set_ylim(1.2, 2.95)
    a.legend(loc="lower right", title="controller rate", title_fontsize=6.5)
    a.set_title("Slower sampling, same gains")
    a.set_ylabel("altitude  [m]")
    xlab_time(a, "time after the step  [s]")
    a = ax[1]
    for name, lab, col in [("slow-250hz", "no delay", C["faint"]), ("slow-delay100", "100 ms", C["FF"]),
                           ("slow-delay200", "200 ms", C["err"])]:
        d = win(load(name), 8, 30)
        a.plot(d.t, d["pos.y"], color=col, lw=1.0, label=lab)
    a.plot(ref.t, ref["sp.y"], color=C["sp"], ls=(0, (4, 2.5)), lw=0.9)
    a.legend(loc="lower right", title="sensor delay", title_fontsize=6.5, ncol=3)
    a.set_title("Sensor delay at 250 Hz")
    a.set_ylim(0.9, 3.2)
    xlab_time(a)
    save(f, "slow_rates")


def _loop_pm(kp, kd, m=1.0, tau=0.03, delay=0.0, tf=0.0):
    """Crossover frequency and phase margin of L = (kp + kd s/(tf s+1)) e^{-s delay} / (m s^2 (tau s+1))."""
    w = np.logspace(-1, 3, 20000)
    s = 1j * w
    L = (kp + kd * s / (tf * s + 1)) * np.exp(-s * delay) / (m * s**2 * (tau * s + 1))
    k = np.argmin(np.abs(np.abs(L) - 1))
    return w[k], 180 + np.degrees(np.angle(L[k])) % 360 - 360 if np.degrees(np.angle(L[k])) > 0 else 180 + np.degrees(np.angle(L[k]))


@fig
def slow_margin():
    delays = np.linspace(0, 0.25, 200)
    pm = []
    for T in delays:
        w = np.logspace(-1, 3, 20000)
        s = 1j * w
        L = (10 + 7 * s / (s / (2 * np.pi * 20) + 1)) * np.exp(-s * T) / (s**2 * (0.03 * s + 1))
        k = np.argmin(np.abs(np.abs(L) - 1))
        ph = np.unwrap(np.angle(L))[k]
        pm.append(180 + np.degrees(ph))
    pm = np.array(pm)
    f, a = plt.subplots(figsize=(TEXT_W, 46 * MM))
    a.plot(delays * 1000, pm, color=C["ink"], lw=1.2)
    a.axhline(0, color=C["err"], lw=0.7)
    a.fill_between(delays * 1000, pm, 0, where=pm < 0, color=C["sat"], lw=0)
    for T, lab, dx, dy in [(0.002, "250 Hz sampling (≈ 2 ms)", 8, 2), (0.1, "+100 ms sensor delay", 8, 4),
                           (0.2, "+200 ms", 8, 4)]:
        k = np.argmin(np.abs(delays - T))
        a.plot(T * 1000, pm[k], "o", color=C["accent"], ms=3.5)
        a.text(T * 1000 + dx, pm[k] + dy, lab, fontsize=6.3, color=C["accent"])
    a.text(245, -30, "unstable", color=C["err"], fontsize=6.5, ha="right")
    a.set_xlabel("extra delay in the loop  [ms]", loc="right")
    a.set_ylabel("phase margin  [°]")
    a.set_title("Every millisecond of delay costs phase margin (default PD, $\\omega_c \\approx 7$ rad/s)")
    save(f, "slow_margin")


# ─── Lesson I.10 ─────────────────────────────────────────────────────────────
@fig
def edge_runs():
    kps = [30, 100, 200, 400]
    f, ax = plt.subplots(1, 4, figsize=(FULL_W, 42 * MM), sharey=True, gridspec_kw=dict(wspace=0.08))
    for a, kp, col in zip(ax, kps, [C["I"], C["meas"], C["P"], C["err"]]):
        d = win(load(f"edge-kp{kp}"), 8, 20)
        a.plot(d.t, d["sp.y"], color=C["sp"], ls=(0, (4, 2.5)), lw=0.8)
        a.plot(d.t, d["pos.y"], color=col, lw=0.9)
        a.set_title(f"$K_p = {kp}$", loc="left")
        a.set_xticks([8, 12, 16, 20])
    ax[0].set_ylabel("altitude  [m]")
    xlab_time(ax[-1])
    save(f, "edge_runs")


def _max_re(kp, kd, m=1.0, tau=0.03, tf=1 / (2 * np.pi * 20), T=0.002):
    """Largest real part of the closed-loop poles: m s^2 (tau s+1)(tf s+1)(1+sT/2) + (kp (tf s+1) + kd s)(1 - sT/2)."""
    P = np.polymul
    lhs = P(P(P([m, 0, 0], [tau, 1]), [tf, 1]), [T / 2, 1])
    rhs = P(np.polyadd(np.polymul([kp], [tf, 1]), [kd, 0]), [-T / 2, 1])
    return np.roots(np.polyadd(lhs, rhs)).real.max()


@fig
def edge_region():
    """Stability region in the (Kp, Kd) plane for the altitude loop with motor lag, D filter and ZOH."""
    kds = np.linspace(0.3, 30, 120)
    lim = []
    for kd in kds:
        lo, hi = 0.01, 5000.0
        for _ in range(50):
            kp = np.sqrt(lo * hi)
            lo, hi = (kp, hi) if _max_re(kp, kd) < 0 else (lo, kp)
        lim.append(lo)
    f, a = plt.subplots(figsize=(TEXT_W, 58 * MM))
    a.fill_between(kds, 0, lim, color="#E8F4EE", lw=0)
    a.plot(kds, kds / 0.03, color=C["faint"], lw=0.9, ls=(0, (3, 2)))
    a.plot(kds, lim, color=C["I"], lw=1.3)
    a.text(16.5, 16.5 / 0.03 + 40, "motor lag only:  $K_p < K_d/\\tau_m$", color=C["muted"], fontsize=6.5,
           ha="right", va="bottom")
    a.text(22, 150, "stable", color=C["I"], fontsize=7.5)
    for kp, col in [(30, C["I"]), (100, C["meas"]), (200, C["P"]), (400, C["err"])]:
        a.plot(7, kp, "o", color=col, ms=4, zorder=5)
    a.plot(7, 10, "s", color=C["ink"], ms=3.5, zorder=5)
    a.text(7.8, 10, "default", fontsize=6.3, color=C["ink"], va="center")
    a.text(7.8, 420, "the lesson's runs ($K_d = 7$)", fontsize=6.3, color=C["muted"], va="center")
    a.set_xlim(0, 30)
    a.set_ylim(0, 800)
    a.set_xlabel("$K_d$", loc="right")
    a.set_ylabel("$K_p$")
    a.set_title("Where the altitude loop is stable (motor lag, D filter, sample-and-hold)")
    save(f, "edge_region")


# ─── Lesson I.11 ─────────────────────────────────────────────────────────────
@fig
def challenge_runs():
    f, ax = plt.subplots(2, 1, figsize=(FULL_W, 70 * MM), sharex=True,
                         gridspec_kw=dict(height_ratios=[1.4, 1], hspace=0.35))
    for name, lab, col in [("challenge", "default tune", C["faint"]),
                           ("challenge-tuned", "$K_p = 40$, $K_i = 8$, $K_d = 12$, filter 30 Hz", C["accent"])]:
        d = load(name)
        rms = 100 * np.sqrt((d["alt.err"] ** 2).mean())
        ax[0].plot(d.t, d["alt.err"], color=col, lw=0.9, label=f"{lab}:  RMS {rms:.1f} cm")
    ax[0].axhline(0, color=C["faint"], lw=0.5)
    ax[0].legend(loc="lower left", ncol=2)
    ax[0].set_ylabel("error  [m]")
    ax[0].set_title("The gust challenge: altitude error, same wind for both runs (seed 4242)")
    d = load("challenge")
    ax[1].plot(d.t, d["wind.y"], color=C["wind"], lw=0.8)
    ax[1].axhline(0, color=C["faint"], lw=0.5)
    ax[1].set_ylabel("m/s")
    ax[1].set_title("Vertical wind")
    xlab_time(ax[1])
    ax[1].set_xlim(5, 65)
    save(f, "challenge_runs")


# ─── Lesson I.12 ─────────────────────────────────────────────────────────────
@fig
def tilt_compare():
    f, ax = plt.subplots(1, 2, figsize=(FULL_W, 50 * MM), gridspec_kw=dict(wspace=0.25))
    d = win(load("tilt-l2"), 2, 30)
    a = ax[0]
    for k, col, lab in [("p", C["P"], "P"), ("i", C["I"], "I"), ("d", C["D"], "D")]:
        a.plot(d.t, d[f"pos.x.part.{k}"], color=col, lw=1.0)
        label_end(a, 30.4, d[f"pos.x.part.{k}"].iloc[-60:].mean(), lab, col)
    a.axhline(0, color=C["faint"], lw=0.5)
    a.set_title("L2: horizontal force terms in a 4 m/s wind along $+x$")
    a.set_ylabel("N")
    xlab_time(a)
    a = ax[1]
    d3 = win(load("tilt-l3"), 2, 30)
    a.plot(d3.t, d3["pitch"], color=C["meas"], lw=1.0)
    th = np.degrees(np.arctan(0.12 * 16 / 9.81))
    a.axhline(th, color=C["accent"], lw=0.8, ls=(0, (3, 2)))
    a.text(29.8, th + 1.2, f"steady-wind prediction {th:.0f}°", color=C["accent"], fontsize=6.4, ha="right")
    a.set_title("L3: the quadrotor's pitch angle — it leans into the wind")
    a.set_ylabel("degrees")
    xlab_time(a)
    save(f, "tilt_compare")


# ─── Lesson I.13 ─────────────────────────────────────────────────────────────
@fig
def cascade_loops():
    d = win(load("cascade-step"), 9.5, 14)
    f, ax = plt.subplots(4, 1, figsize=(TEXT_W, 120 * MM), sharex=True, gridspec_kw=dict(hspace=0.5))
    rows = [("pos.x", "Position loop (P, 50 Hz): $x$", "m", 1),
            ("vel.x", "Velocity loop (PID, 100 Hz): $v_x$", "m/s", 1),
            ("att.pitch", "Attitude loop (P, 250 Hz): pitch", "deg", 1),
            ("rate.pitch", "Rate loop (PID, 1 kHz): pitch rate", "deg/s", 1)]
    for a, (k, title, unit, _) in zip(ax, rows):
        a.plot(d.t, d[f"{k}.sp"], color=C["sp"], lw=0.9, ls=(0, (4, 2.5)))
        a.plot(d.t, d[f"{k}.meas"], color=C["meas"], lw=1.0)
        a.set_title(title)
        a.set_ylabel(unit)
    label_end(ax[0], 14.05, d["pos.x.sp"].iloc[-1] + 0.12, "setpoint", C["sp"])
    label_end(ax[0], 14.05, d["pos.x.meas"].iloc[-1] - 0.12, "measured", C["meas"])
    xlab_time(ax[-1])
    ax[-1].set_xlim(9.5, 14)
    save(f, "cascade_loops")


@fig
def cascade_noi():
    f, a = plt.subplots(figsize=(TEXT_W, 42 * MM))
    for name, lab, col in [("cascade", "velocity PID with I", C["I"]), ("cascade-noi", "velocity loop without I", C["err"])]:
        d = win(load(name), 2, 30)
        a.plot(d.t, d["pos.x"], color=col, lw=1.0, label=lab)
    a.axhline(0, color=C["sp"], lw=0.8, ls=(0, (4, 2.5)))
    a.legend(loc="center right")
    a.set_title("Position downwind in a 5 m/s wind (setpoint $x = 0$)")
    a.set_ylabel("$x$  [m]")
    xlab_time(a)
    save(f, "cascade_noi")


# ─── Lesson I.14 ─────────────────────────────────────────────────────────────
@fig
def inversion_runs():
    f, ax = plt.subplots(2, 1, figsize=(FULL_W, 78 * MM), sharex=True, gridspec_kw=dict(hspace=0.4))
    runs = [("inversion-att250", "attitude loop 250 Hz", C["faint"]), ("inversion-att10", "10 Hz", C["meas"]),
            ("inversion-att5", "5 Hz", C["P"]), ("inversion-att3", "3 Hz", C["err"]),
            ("inversion-kp1.5", "250 Hz, $K_{att} = 1.5$", C["D"])]
    for name, lab, col in runs:
        d = win(load(name), 6, 30)
        ax[0].plot(d.t, d["pos.x"], color=col, lw=1.0, label=lab)
        if name in ("inversion-att250", "inversion-att3"):
            ax[1].plot(d.t, d["pitch"], color=col, lw=0.8, label=lab)
    d = win(load("inversion-att250"), 6, 30)
    ax[0].plot(d.t, d["sp.x"], color=C["sp"], lw=0.8, ls=(0, (4, 2.5)))
    ax[0].legend(loc="lower center", ncol=5, fontsize=6.2, bbox_to_anchor=(0.5, -0.02))
    ax[0].set_ylabel("$x$  [m]")
    ax[0].set_ylim(-3.2, 2.6)
    ax[0].set_title("A square wave of ±1.5 m in $x$: position")
    ax[1].legend(loc="upper right", ncol=2, fontsize=6.2)
    ax[1].set_ylabel("pitch  [°]")
    ax[1].set_title("Pitch angle: the slow attitude loop swings up to 80°")
    xlab_time(ax[1])
    ax[1].set_xlim(6, 30)
    save(f, "inversion_runs")
