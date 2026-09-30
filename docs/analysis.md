# Part III — Why It Works

> **Status (2026-09-30):** planned and reviewed against the code, nothing
> implemented yet. Milestones M13–M18 in [roadmap.md](roadmap.md) point here.
> Scope is decided (§7): the app only, M13 and the ten **must** lessons first.

Part I teaches PID. Part II adds the controllers people fly today. Both are
about **design**: a knob or a cost goes in, a controller comes out, and we
judge it by watching it fly. Part III covers what a control engineer does
before anything flies: **analysis**. How close to instability is this loop?
Which frequencies does it reject and which does it amplify? What can it
never see? What does it do when the plant is not what we assumed?

Most of this is textbook material that Part II skipped on purpose
(`beyond-pid.md` §1: "H∞ lives in the frequency domain, which we don't
visualise"). Part III starts visualising it. Its guiding idea mirrors Part
II's "estimate next to the truth": **prediction next to measurement**.
Every analytic result (a Bode plot, a pole, a margin, a region of
attraction) is drawn next to the same quantity **measured on the
simulator**. Where they disagree, the lesson is about the reason why.

The non-goals stand: the world stays a grid, the controller always flies.

## 1. Gap audit

What control theory Parts I–II do not cover, checked against the code
(`src/control`, `src/estimation`, `src/sim`) and the lessons.

| Area                              | What is missing                                                                               | Where the repo stands                                                                                                   | Decision                                                                             |
| --------------------------------- | --------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| **Frequency domain**              | Transfer functions, Bode, Nyquist, gain/phase margins, sensitivity $S$ and $T$, loop shaping. | Bode appears only in "Later / ideas" (`roadmap.md`). Margins are mentioned once in the primer (A.2), never shown.       | **Yes, central.** Chapter F.                                                         |
| **Poles, zeros, root locus**      | Pole maps (s- and z-plane), root locus while a gain moves.                                    | LQR shows closed-loop poles as numbers. No root-locus view, which has waited on the ideas list since Part I.            | **Yes.** Chapter F.                                                                  |
| **Fundamental limits**            | Bode's sensitivity integral (waterbed effect), delay and RHP zeros cap the bandwidth.         | The `delayMs` sensor option exists and Part I lessons show that delay destabilises, but not why or by how much.         | **Yes.** Chapter F.                                                                  |
| **Lyapunov stability**            | Lyapunov functions, $\dot V$, region of attraction, LaSalle.                                  | Zero mentions. Geometric control is called "almost-global" without explaining why.                                      | **Yes.** Chapter G.                                                                  |
| **Structural properties**         | Controllability, observability, detectability, the separation principle.                      | Zero mentions. The Kalman filter runs, but we never ask what it _cannot_ estimate.                                      | **Yes.** Chapter G.                                                                  |
| **LQG and its lost margins**      | LQR has guaranteed margins, LQG has none [Doyle 1978]; loop-transfer recovery.                | LQR and the Kalman filter both exist on L1 (`WithAltitudeKalman`), but nobody has measured the margins of the pair.     | **Yes.** Chapter G. Costs almost nothing.                                            |
| **Sliding mode, backstepping**    | The classical robust nonlinear designs.                                                       | Rejected in Part II. Only an optional chattering lesson was left open.                                                  | **Yes, small.** Chapter H, on L1 only.                                               |
| **H∞ / μ-synthesis**              | Worst-case design, mixed sensitivity, structured uncertainty.                                 | Rejected in Part II for lack of frequency-domain views. Chapter F removes that obstacle.                                | **H∞ loop shaping as a _could_** [Glover 1989]. μ-synthesis no: too heavy for a toy. |
| **Sampling and discretisation**   | z-plane, ZOH vs Tustin, loop rate vs margin, aliasing, anti-alias filters.                    | Controllers are discretised properly (ZOH in `riccati.ts`, `lqr.ts`), but the student never sees the consequences.      | **Yes.** Chapter I.                                                                  |
| **Vibration and notch filters**   | Rotor-frequency vibration on the IMU, notch and RPM-tracking notch filters.                   | Not modelled. Low-pass filters only (`filters.ts`). In practice this is the biggest tuning topic in PX4 and Betaflight. | **Yes.** Chapter I. Needs a vibration model.                                         |
| **Nonlinear estimation**          | Complementary filter, Mahony, EKF.                                                            | Linear KF on L1 only. L3 reads true attitude plus noise ("idealised perfect estimator", `physics-model.md` §6).         | **Yes.** Chapter I.                                                                  |
| **System identification**         | Sine sweep, least squares, relay auto-tuning [Åström 1984].                                   | Relay auto-tuner on the ideas list; Ziegler–Nichols dropped (`pid-primer.md` §6.2).                                     | **Yes.** Chapter I.                                                                  |
| **Fault-tolerant control**        | Flying after the _complete_ loss of a rotor [Mueller 2014, Sun 2021].                         | The plant already supports it (`motorEfficiency[i] = 0`); only partial loss is taught (II.7).                           | **Yes.** Chapter J. Ties back to controllability.                                    |
| Gain scheduling                   | Gains as a function of the operating point.                                                   | Our plant is close to LTI around hover; INDI and geometric control already cover the nonlinearity.                      | **No.** One paragraph in the primer.                                                 |
| Iterative learning control        | Learn a feedforward correction from repeated laps.                                            | Figure-8 laps exist, so it would fit.                                                                                   | **Later.** Good candidate for a Part III extra.                                      |
| Multi-agent, consensus, formation | Several drones sharing information.                                                           | One drone per simulation.                                                                                               | **No.** Changes the whole engine.                                                    |
| Others                            | Event-triggered and stochastic/dual control, reference governors, Koopman, value-based RL.    | —                                                                                                                       | **No.** Listed in the primer's further reading.                                      |

## 2. The pedagogical arc

Part III is the **third semester**: numbering restarts at III.1. As in Part
II, each chapter opens with a question the student cannot yet answer.

| Chapter                                    | The question it answers                                                                    | New ideas                                                                        |
| ------------------------------------------ | ------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------- |
| F. The loop as a filter (L1, L3 rate loop) | "Lesson 'Find the edge' found the gain where it oscillates. Can I know that before I fly?" | Frequency response, poles, root locus, margins, delay, sensitivity, loop shaping |
| G. Stability and structure (L1)            | "It recovered this time. Will it always? And what can my filter never know?"               | Lyapunov, region of attraction, observability, controllability, LQG              |
| H. Robust by construction (L1)             | "Can I design for the worst plant instead of the nominal one?"                             | Sliding mode, backstepping, H∞ loop shaping                                      |
| I. The real loop (L3)                      | "My analysis says 60° of margin. Why does the real drone buzz?"                            | Sampling, aliasing, vibration, notch filters, attitude estimation, EKF, sysid    |
| J. When things break (L3)                  | "A propeller just came off. Is the drone still controllable?"                              | Loss of controllability, reduced-attitude control, a robustness report card      |

## 3. Architecture changes

All of it respects [software-architecture.md](software-architecture.md):
`sim/`, `control/`, `estimation/`, `math/` and the new `analysis/` stay
framework-free and deterministic.

### 3.1 LTI toolbox (`src/math/`)

```
src/math/
  complex.ts     complex arithmetic (add, mul, div, abs, arg, exp); owns the `Complex` type
  lti.ts         state space {A, B, C, D} (continuous or discrete, with dt), series/feedback,
                 frequency response via (jωI − A)⁻¹ or (e^{jωT}I − A)⁻¹, step response, Tustin
  eig.ts         eigenvalues of small non-symmetric matrices (Hessenberg + shifted QR)
  margins.ts     gain/phase/delay margins and crossover from a sampled frequency response
  fft.ts         radix-2 FFT, Welch PSD (for spectra and the vibration chapter)
  care.ts        continuous ARE via the Hamiltonian's stable subspace (only needed for H∞)
```

`mat.ts` already has `expm` (Padé), `c2d` (ZOH), `toContinuous` and
`eigenvalues`. Part III reuses the first three as they are. `eigenvalues`
keeps its name and its tests but gets the QR algorithm of `eig.ts` behind
it: the current method (characteristic polynomial, then Durand–Kerner) is
documented as good for about six states, and a loop model with the PID, the
motor lag, a Padé delay and a filter has seven to ten. `mat.ts` re-exports
`Complex` from `complex.ts`, so existing imports keep working.

Every function is unit-tested against closed-form results: a double
integrator under PD has known margins, and a first-order lag has known
phase at its corner frequency.

### 3.2 Models of the loop (`src/analysis/`)

Every analysis needs an LTI model of _plant × controller_ at an operating
point. Two ways to get one, and the lessons use both:

1. **Analytic models.** Each L1 controller exposes `linearModel(params)`:
   PID, LQR/LQI, ADRC and SMC (with its boundary layer, linearised) return
   a discrete state-space model of the controller. The plant model is the
   double integrator, plus the motor lag and a Padé approximation of the
   sensor delay. For the L3 rate loop: rigid body axis + motor lag + the
   rate PID or INDI stage.
2. **Measured response.** `analysis/sweep.ts` breaks the loop at a
   **probe point** and injects a signal (see 3.3). It runs the real
   `Simulation` headlessly, then correlates at each frequency (a
   single-bin DFT per frequency) to get the complex gain. This works for
   every controller, including nonlinear ones (MPC, MPPI, the policy),
   where it measures a _describing function_. The lessons say so.

`analysis/linearize.ts` is a fallback that takes finite-difference
Jacobians of `sim/dynamics.ts` around trim. It checks the analytic plant
models in tests.

### 3.3 Probe points

A new block `probe` in the parameter schema:

```ts
probe: {
  point: 'none' | 'ref.y' | 'l1.thrust' | 'l3.torque.x' | 'l3.rate.x' | 'l3.acc.x';
  signal: 'sine' | 'chirp' | 'multisine' | 'relay';
  amp: number; // in the units of the point
  freqHz: number; // sine
  f0Hz: number;
  f1Hz: number;
  durationS: number; // chirp
}
```

There are two kinds of point:

- **Loop-break points** (`l1.thrust`, `l3.*`). The probe signal $d$ is
  added to the controller output $u_c$, so the plant receives
  $u = u_c + d$ (the classic "break the loop at the plant input"
  measurement; the loop itself stays closed). One recording gives all
  three functions: $S = U/D$, $T = -U_c/D$ and $L = T/S = -U_c/U$.
- **The reference point** (`ref.y`). The signal is added to the altitude
  setpoint and the recording gives the closed-loop response $Y/R$. Lesson
  III.1 uses it, because "the drone follows slow sines and ignores fast
  ones" is about $Y/R$ and not about $L$.

`relay` replaces the controller output with $\pm h$ around trim for the
auto-tuner. The probe is deterministic and part of the seeded run, so
measured Bode plots are reproducible. Telemetry adds `probe.in` ($d$ or
the reference signal), `probe.u` and `probe.uc`. Sweeps run with sensor
noise and wind off unless the lesson is about them.

Sweeps run many short simulations. They reuse the headless runner of
`engine/arena.ts` and run in time-sliced chunks, so the UI stays live. If
that is not enough, this is when `Simulation` moves to a Web Worker (on the
ideas list since M4).

### 3.4 Plant and sensor additions

| Addition                                                                                                                                                   | Why                                                                                                           |
| ---------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- |
| **Rotor vibration** on the IMU: one sinusoid per motor at its rotor frequency $f_i = f_{hover}\sqrt{T_i/T_{hover}}$ (`vibration.amp`, `vibration.hoverHz`) | Vibration on the gyro and accelerometer, aliasing, notch filters. Off by default so Parts I–II are unchanged. |
| **IMU sample rate** and optional analog anti-alias pole (`sensors.imuRateHz`, `sensors.aaFilterHz`)                                                        | Aliasing appears only if the sensor samples slower than the physics (1 kHz).                                  |
| **Gyro bias** (`sensors.gyroBias`, deg/s, per axis) and a slow random walk                                                                                 | Makes attitude estimation non-trivial.                                                                        |
| **Attitude source** `sensors.attitude: 'truth' \| 'complementary' \| 'mahony'`                                                                             | L3 flies on an estimate instead of noisy truth. Default `truth`, so the Part I/II regression baseline holds.  |
| **Navigation source** `sensors.nav: 'truth' \| 'ekf'` with a 10 Hz GPS-like position fix and a barometer                                                   | L3 position/velocity from an EKF.                                                                             |
| **Scripted total motor loss** (existing `motorEfficiency`, event helper `loseMotor(i, t)`)                                                                 | Chapter J. The plant supports it already.                                                                     |

Vibration is a **sensor-level** signal. It is added to the gyro and
accelerometer readings inside `sim/sensors.ts` and does not enter the
rigid-body dynamics: the motion it would cause is far below anything
visible, and the lessons are about what it does to the measurement. Each
motor keeps an accumulated phase, so the frequency can follow the thrust.
With `hoverHz` up to 250 Hz the fastest rotor stays below the 500 Hz
Nyquist frequency of the 1 ms physics step, so the step does not change.

The sensor chain, in order, at the physics rate: truth, plus vibration,
through the anti-alias pole, then sample-and-hold at `imuRateHz`. The
controller reads the held sample. `imuRateHz = 0` (the default) means no
hold: the controller reads the newest value on each of its ticks, as it
does today.

### 3.5 New controllers and filters

```
src/control/
  smc.ts          L1 sliding mode: s = ė + λe, u = û_eq − k·sat(s/φ); φ = 0 gives pure sign()
  backstepping.ts L1 with the motor lag as a state (the first plant where backstepping has two steps)
  hinf.ts         L1 H∞ loop-shaping controller (could)
  fault.ts        L3 reduced-attitude controller for three rotors [Mueller 2014], INDI inner loop [Sun 2021]
  shaping.ts      lead/lag and notch stages for any SISO loop (L1 thrust, L3 rate loops)
src/estimation/
  filters.ts      + biquad notch, + RPM-tracking notch bank (one notch per motor, tracks f_i)
  attitude.ts     complementary filter and Mahony on SO(3), with gyro-bias estimation
  ekf.ts          loosely coupled nav EKF: (p, v, b_a), attitude taken from attitude.ts
  sysid.ts        least-squares fit of (m̂, τ̂_motor, delay) from sweep data; relay auto-tuner
```

### 3.6 New visualisations

- **Bode chart** (uPlot, log-frequency axis): magnitude and phase, model
  (solid) and measured (dots), with crossover, margins and the $-180°$
  line annotated.
- **Nyquist chart** (canvas): the $-1$ point, the unit circle and the
  distance-to-$-1$ circle, which is $1/\|S\|_\infty$.
- **Pole–zero map**: s- or z-plane with the stability boundary. Poles move
  live as sliders change, and drag leaves a root-locus trail.
- **Lyapunov overlay** on the existing `PhasePortrait`: level sets of $V$,
  $\dot V$ as a colour field, the estimated region of attraction versus
  the true one. The true one is computed headlessly from a grid of initial
  states.
- **Spectrum** of any telemetry channel (Welch PSD). A spectrogram is a
  _could_.
- **Covariance view** for observability: $\sqrt{P_{ii}}$ over time, per
  state, on a log scale.
- **Robustness report card** in the arena: margins, $\|S\|_\infty$, delay
  margin and measured bandwidth per controller (Chapter J).

`ExtraChart` grows `bode`, `nyquist`, `poles`, `spectrum` and `covariance`.

**Layout.** Today the chart grid has one switchable extra slot, and several
lessons need two analysis views at once (III.2: pole map beside the step
response; III.3: Bode beside Nyquist). A lesson can therefore set
`analysis: [ExtraChart, ExtraChart?]`. The first view takes the extra
slot; the second replaces the Actuators chart for that lesson. Tracking
stays visible, so the flight is always on screen next to the analysis.
The Bode chart is one slot with two stacked panes (magnitude, phase) on a
shared frequency axis. It has a second mode for III.5: $\ln|S|$ on a
**linear** frequency axis up to the Nyquist frequency, with the areas
above and below zero shaded.

### 3.7 Lesson plumbing

- `Part` becomes `1 | 2 | 3`, and `PARTS` in `LessonPanel.tsx` gets
  `Part III · Why it works` with the prefix `III.`.
- Lessons III.4 and III.7 ask for a number before the run. `Lesson` gets
  an optional `predict: { label, unit, truth(sim), tolerance }`. The panel
  shows an input field, `truth` is measured headlessly on the same seed,
  and `goal.check` receives the student's value in `GoalContext`.

## 4. Designs (short)

Full derivations go into a new textbook, `docs/analysis-primer.md`, written
alongside the code. `modern-control-primer.md` played the same role for
Part II.

**Frequency response on L1.** Plant $G(s) = \dfrac{e^{-s\tau_d}}{m s^2 (\tau_m s + 1)}$.
The controller is discretised at `control.rateHz` and plotted up to the
Nyquist frequency. The PID loop has one crossover, so gain and phase margin
read straight off the chart. The $-180°$ crossing sets the ultimate gain
that lesson 10 ("Find the edge") found by trial.

**Delay costs phase.** A pure delay has $|e^{-j\omega\tau}| = 1$ and phase
$-\omega\tau$, so at crossover it removes $\omega_c \tau \cdot 180°/\pi$ of
phase margin. The lesson turns a delay slider and shows that the maximum
usable bandwidth drops roughly as $1/\tau$. The Padé model of the delay
has a right-half-plane zero, a first look at non-minimum-phase behaviour.

**Waterbed.** For an open loop with no poles in the right half-plane and
relative degree ≥ 2, $\int_0^\infty \ln|S(j\omega)|\,d\omega = 0$: pushing
$|S|$ down at low frequency (better gust rejection) pushes it up somewhere
else. Our loop qualifies: its integrators sit on the imaginary axis, not
to the right of it. The controller is sampled, so the integral we can
measure runs to the Nyquist frequency, where the discrete-time version of
the theorem gives the same zero for a strictly proper loop. The areas are
equal on a **linear** frequency axis, not on the logarithmic axis of a
Bode plot, so the lesson uses the linear $\ln|S|$ view (§3.6). Turbulence
at the frequency of the $|S|$ peak then gets amplified, and the student
sees it in the air.

**Lyapunov on L1.** For PD, $V = \tfrac12 k_p e^2 + \tfrac12 m v^2$ gives
$\dot V = -k_d v^2 \le 0$, and LaSalle finishes the argument. With thrust
saturation, the drone can only brake at $a_{max} = 4f_{max}/m - g$. The
region of attraction is bounded by $v^2 < 2 a_{max} h$. $V$ gives a
conservative inner estimate; the headless grid of initial states gives the
truth.

**Observability.** For the L1 Kalman filter with state $(y, v, b_a)$ and an
altimeter, the observability matrix has full rank. Add the altimeter bias
$b_y$ as a fourth state and the rank drops to 3: $y$ and $b_y$ are
indistinguishable, and $\sqrt{P}$ of that direction never shrinks. During
an altimeter dropout event, position uncertainty grows as $t^{3/2}$, and
faster with accelerometer bias.

**LQG.** Continuous single-input LQR guarantees a gain margin
$[\tfrac12, \infty)$ and phase margin ≥ 60°, for the plant it was designed
on. Our discrete LQR gets slightly less (the primer shows how much), and
the motor lag takes about 10° more if the design ignores it. The lesson
therefore runs with `lqr.lagState` on and no sensor delay, so the measured
margin is the promised one. Put the Kalman filter in front
and those guarantees vanish [Doyle 1978]. The lesson measures the margins
of LQR, then LQR + KF with an aggressive filter, then recovers them by
detuning the filter towards the loop-transfer-recovery limit
[Doyle 1981].

**Sliding mode (L1).** $s = \dot e + \lambda e$, $u = u_{eq} - k\,\mathrm{sign}(s)$.
It is robust to any matched disturbance with $|d| < k$, but chatters at the
loop rate and hits the motor lag. A boundary layer $\mathrm{sat}(s/\phi)$
trades the chatter for a small steady error. Plotted in the phase
portrait, with the sliding line drawn.

**Backstepping (L1 + motor lag).** Virtual control on thrust, then on the
motor command, with a Lyapunov function per step [Krstić 1995]. The
analytic model shows it is equivalent to a particular state feedback near
hover, and different at large errors.

**H∞ loop shaping (could).** Shape $W_2 G W_1$ with a PI weight, then
robustify with the normalised coprime-factor solution (two AREs, closed
form, `care.ts`) [Glover 1989]. The stability margin $\epsilon_{max}$ appears
next to the classical margins.

**Sampling.** The z-plane pole map versus `control.rateHz`, from 1000 Hz
down to 50 Hz, and the margin as a function of rate: ZOH costs about half
a sample of delay. Aliasing: vibration at $f_i$ sampled at `imuRateHz`
folds to $|f_i - k f_s|$. With a 180 Hz rotor sampled at 200 Hz, the gyro
reports a 20 Hz wobble that no digital filter can remove afterwards.

**Notch filters.** A biquad notch at a fixed frequency works until the
throttle changes. An RPM-tracking bank (one notch per motor at $f_i$, the
Betaflight "RPM filter" idea) keeps working. The spectrum chart shows the
peaks disappear, and the Bode chart shows the phase the notch costs at
crossover.

**Attitude estimation.** A complementary filter blends integrated gyro
(high-pass) with the accelerometer's gravity direction (low-pass). It fails
in sustained acceleration, because the accelerometer measures specific
force, not gravity. Mahony [Mahony 2008] does the same on SO(3) and
estimates gyro bias. The lesson flies a fast circle on each and plots the
estimated tilt error against the truth.

**Nav EKF.** Loosely coupled: IMU propagation, GPS-like position at 10 Hz,
barometer. Outlier gating and the normalised innovation squared are shown,
so the student sees when the filter is inconsistent.

**System identification.** Least squares on sweep data fits $\hat m$,
$\hat\tau_m$ and the delay. The fitted model feeds `control.model` and then
LQR, so the whole chain runs: identify, design, verify margins. The relay
auto-tuner [Åström 1984] finds the ultimate gain and period of the rate
loop. Its $-180°$ crossing exists only because of the delay and the
filters, which explains why Ziegler–Nichols had no answer for the pure
double integrator in Part I.

**Losing a rotor.** Four motors produce four independent quantities:
thrust and three torques. The matrix that maps motor thrusts to those four
(the control effectiveness matrix, the inverse of the mixer) has rank 4.
Remove a column and the rank is 3: thrust, roll, pitch and yaw can no
longer be commanded independently, and there is no motor setting that
hovers without rotating. The **controllability readout** of lesson III.19
is that rank, shown with the direction that was lost. The drone can still
hold position if it gives up yaw and spins about a tilted primary axis
[Mueller 2014]. Reduced-attitude control of that axis, with an INDI inner
loop [Sun 2021]. The spin settles where the yaw reaction torque meets the
rotational drag, at roughly $c_\tau m g / k_{damp}$. With today's defaults
that is about 78 rad/s, or 18° of rotation per 250 Hz controller tick,
which is too fast to control (see §7).

## 5. Milestones

Each ends with something runnable; `make check` passes at each commit, and
the Part I/II behavioural baseline stays unchanged.

**Build order.** Two passes. The first pass is M13 and then the **must**
lessons of M14, M15, M17 and M18 (III.1–4, 7, 8, 14–16, 20), with only the
code those lessons need. The second pass fills in the **should** and
**could** lessons, all of M16 (Chapter H) and the rotor-loss lesson III.19.
Lesson numbers are fixed now, so the first pass leaves gaps in the list.

### M13 — Foundations for Part III

- `complex.ts`, `lti.ts`, `eig.ts`, `margins.ts`, `fft.ts` with tests
  against closed-form results.
- `linearModel()` for PID, LQR/LQI and ADRC on L1; analytic L1 plant.
- Probe block, injection, telemetry; `sweep.ts` with time-sliced runs.
- Bode, Nyquist and pole–zero charts; `ExtraChart` additions; the
  two-view analysis layout (§3.6).
- Lesson panel shows Part III as a third course (§3.7).

**Done when** the model and measured Bode plots of the default L1 PID agree
to within 1 dB and 5° from 0.1 Hz to 20 Hz. The default loop crosses over
near 1.1 Hz, so the band covers a decade on each side. Below it the loop
gain is so high that the plant input barely moves, and the measurement is
a ratio of a large signal to a very small one; above it the 250 Hz controller and the 1 kHz physics
stop looking like one sampled system.

### M14 — Chapter F lessons (III.1–III.6)

Root-locus trail, delay margin, the `predict` field (first used by III.4),
$S$/$T$ measurement, `shaping.ts` (lead/lag, notch on L1), spectrum chart.

### M15 — Chapter G (III.7–III.9)

Lyapunov overlay and region-of-attraction grid; altimeter-bias state and
altimeter-dropout event for the Kalman filter; covariance chart; LQG margin
measurement.

### M16 — Chapter H (III.10–III.12)

`smc.ts`, `backstepping.ts`, sliding line in the phase portrait. `care.ts`
and `hinf.ts` only if time allows.

### M17 — Chapter I (III.13–III.18)

Vibration model, IMU sample rate and anti-alias pole, notch and RPM notch
bank, `attitude.ts`, `ekf.ts`, `sysid.ts`, relay probe. Linear models of
the L3 rate loop (PID and INDI).

### M18 — Chapter J and the report card (III.19–III.20)

Robustness report card in the arena, and `make bench` printing margins
next to RMS (first pass). Then the `loseMotor` event, `fault.ts` and the
controllability readout (second pass). III.19 starts with a headless
prototype of the three-rotor spin before any lesson is written (§7).

## 6. Lessons (Part III)

Priority: **must** (core story), **should**, **could**.

### Chapter F — The loop as a filter

| #   | id         | Title                  | Setup and events                                                               | What the student sees / goal                                                                                                                                                    | Prio   |
| --- | ---------- | ---------------------- | ------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------ |
| 1   | `bode`     | The drone as a filter  | L1 PID, sine probe on the setpoint; frequency slider, then an automatic sweep. | Slow sines are followed, fast ones are ignored, with a resonance in between. The measured dots land on the model curve. Goal: find the −3 dB bandwidth.                         | must   |
| 2   | `poles`    | Poles tell the story   | Pole map beside the step response; drag $K_p$, $K_d$.                          | Poles move, the step response changes with them; the root-locus trail crosses into instability at the gain lesson 10 found. Goal: place poles for ζ ≈ 0.7.                      | must   |
| 3   | `margins`  | How far from the edge? | Loop broken at the thrust; Bode and Nyquist of $L$.                            | Gain and phase margin as distances to $-1$. Goal: PM ≥ 45° and GM ≥ 6 dB with the highest possible crossover.                                                                   | must   |
| 4   | `delay`    | Delay is a phase thief | Same loop, `delayMs` slider 0 → 60 ms.                                         | The phase curve bends down, the margin disappears, and the drone oscillates at the predicted frequency. Goal: predict the critical delay within 10 %.                           | must   |
| 5   | `waterbed` | The waterbed           | Measure $\|S\|$; stronger integral action; turbulence on.                      | Better low-frequency rejection, a higher $\|S\|$ peak, and gusts at that frequency amplified in flight. On the linear-frequency view the areas above and below zero stay equal. | should |
| 6   | `shaping`  | Shape the loop         | Lead/lag and notch blocks on the L1 loop; a target loop shape drawn in.        | Design by bending the Bode plot instead of turning gains. Goal: match the target shape and hit PM ≥ 50°.                                                                        | should |

### Chapter G — Stability and structure

| #   | id           | Title                         | Setup and events                                                                                                        | What the student sees / goal                                                                                                                        | Prio   |
| --- | ------------ | ----------------------------- | ----------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- | ------ |
| 7   | `lyapunov`   | Energy that only goes down    | PD on L1, low max thrust; phase portrait with $V$ level sets; scripted dives from rising heights.                       | $V$ never increases, until the thrust saturates. The Lyapunov estimate sits inside the true region of attraction. Goal: predict which dive crashes. | must   |
| 8   | `observable` | What the filter can't see     | Kalman filter with the altimeter bias as a state; then a 5 s altimeter dropout.                                         | Bias covariance never shrinks (rank-deficient observability matrix); during the dropout, position uncertainty grows as $t^{3/2}$.                   | must   |
| 9   | `lqg`        | Optimal plus optimal ≠ robust | LQR with the lag state and no sensor delay, then LQR + aggressive KF, then a detuned KF; margins measured by the probe. | LQR: ≥ 60° as promised. LQG: a small margin, and a slightly heavier drone oscillates. Goal: recover PM ≥ 45° by detuning the filter.                | should |

### Chapter H — Robust by construction

| #   | id             | Title                | Setup and events                                                  | What the student sees / goal                                                                                                                                   | Prio   |
| --- | -------------- | -------------------- | ----------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------ |
| 10  | `smc`          | Slide to the target  | L1 SMC, payload +300 g, gusts; boundary layer φ = 0 at first.     | State slides down the line in the phase portrait, rejects the payload, and the motors chatter. Goal: remove chatter (thrust jitter < 0.3 N) with error < 3 cm. | should |
| 11  | `backstepping` | One step at a time   | L1 with the motor-lag state; backstepping vs PD on a large step.  | Near hover they match (same linear model); at large errors backstepping respects the lag and overshoots less.                                                  | could  |
| 12  | `hinf`         | Design for the worst | H∞ loop shaping vs the lesson-6 design; mass ±40 %, delay +20 ms. | The H∞ design keeps its margins across the whole family of plants; the hand-shaped one loses them at one corner.                                               | could  |

### Chapter I — The real loop (L3)

| #   | id         | Title                 | Setup and events                                                                                       | What the student sees / goal                                                                                                        | Prio   |
| --- | ---------- | --------------------- | ------------------------------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------- | ------ |
| 13  | `rate`     | How slow can you go?  | L3 rate loop; `rateHz` slider 1000 → 50 Hz; z-plane pole map.                                          | Poles creep towards the unit circle; each halving of the rate costs predictable phase. Goal: find the lowest rate with PM ≥ 45°.    | should |
| 14  | `alias`    | Ghost frequencies     | Rotor vibration on, IMU at 200 Hz, no anti-alias pole; spectrum chart.                                 | A 20 Hz wobble that isn't there; no digital filter removes it. Goal: switch on the anti-alias pole or raise the IMU rate.           | must   |
| 15  | `notch`    | Notch the noise       | Vibration at a proper IMU rate; D-term amplifies it; fixed notch, then RPM notch; throttle steps.      | The fixed notch works at hover, fails on a climb; the RPM notch follows. Goal: motor jitter below the no-vibration baseline + 10 %. | must   |
| 16  | `attitude` | Where is up?          | L3 flies on estimated attitude; gyro bias; fast 3 m circle; complementary vs Mahony.                   | In the turn the accelerometer "sees" a tilted gravity, and the complementary filter trusts it. Mahony holds. Goal: tilt error < 2°. | must   |
| 17  | `ekf`      | Fly on beliefs        | Nav EKF with 10 Hz GPS-like fixes and baro; one outlier fix at t = 8 s.                                | Innovation and NIS plot; the gated EKF ignores the outlier, the ungated one jumps. Goal: tracking RMS within 2× of flying on truth. | should |
| 18  | `sysid`    | Identify, then design | The model block starts wrong (m̂ −30 %, τ̂ ×2); run a sweep, fit, redesign LQR; relay-tune the rate PID. | Fitted values next to truth; LQR on the fitted model beats LQR on the wrong one. Relay finds $K_u, T_u$.                            | should |

### Chapter J — When things break (L3)

| #   | id        | Title                      | Setup and events                                                                      | What the student sees / goal                                                                                                                              | Prio   |
| --- | --------- | -------------------------- | ------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- | ------ |
| 19  | `3motors` | Three rotors left          | Hover at 5 m; motor 2 lost at t = 5 s; standard cascade vs fault-tolerant controller. | The cascade flips and crashes. The effectiveness matrix drops from rank 4 to 3. The fault controller gives up yaw, spins and holds position within 0.5 m. | should |
| 20  | `report`  | The robustness report card | Arena extended with probe sweeps for every L1 and L3 controller.                      | GM, PM, delay margin, $\|S\|_\infty$ next to Part II's RMS columns. Controllers that won on RMS are not always the robust ones.                           | must   |

Goals use the existing `goal.check` mechanism. Lessons that predict a
number (III.4, III.7) compare the student's input field with the value
measured headlessly on the same seed.

## 7. Risks and open questions

| Risk                                                                                                                     | Mitigation                                                                                                                                                                                            |
| ------------------------------------------------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Sweeps are slow (dozens of runs per Bode plot).                                                                          | Multisine excitation (many frequencies per run); time-sliced chunks; cache per parameter hash. Web Worker if needed.                                                                                  |
| The three-rotor spin is too fast to control: about 78 rad/s with the default `angularDamping` of 0.002 (§4).             | Prototype headlessly first. If needed, add a separate yaw-damping parameter for rotor drag in yaw, and have the lesson set it. If the prototype fails, III.19 is dropped; nothing else depends on it. |
| Low-frequency and near-Nyquist points of a measured Bode plot are unreliable.                                            | Agreement is tested only in a stated band (M13: 0.1–20 Hz); the chart greys out points outside it.                                                                                                    |
| Model and measurement disagree for reasons that are not pedagogical (e.g. drag nonlinearity, saturation during a sweep). | Small probe amplitudes, calm air during sweeps, a tolerance band drawn on the Bode chart; tests pin agreement for the default loops.                                                                  |
| `linearModel()` for every controller is a lot of work.                                                                   | Analytic models only for L1 (PID, LQR/LQI, ADRC, SMC) and the L3 rate loop; everything else is measured-only, and the report card is measured-only.                                                   |
| Scope: 20 lessons, 6 milestones.                                                                                         | The **must** set (10 lessons: III.1–4, 7, 8, 14–16, 20) is a coherent Part III on its own and is built first (§5). Chapter H is optional.                                                             |

Decisions (confirmed 2026-09-30):

1. **Numbering** — Part III as a third semester, restarting at III.1.
2. **H∞** — only as a _could_ (lesson III.12).
3. **Default attitude source** — stays `truth`; estimation is opt-in per
   lesson, so every earlier tune and test stays valid.
4. **Scope** — M13 and the ten **must** lessons first; Chapter H and
   III.19 in the second pass (§5, build order).
5. **The book** — Part III is built in the app only. Chapters of
   _Standing in the Wind_ for Part III come later and are not part of
   M13–M18.

## 8. References

- [Åström 2021] K. J. Åström, R. M. Murray, _Feedback Systems: An Introduction for Scientists and Engineers_, 2nd ed., Princeton University Press, 2021.
- [Åström 1984] K. J. Åström, T. Hägglund, "Automatic tuning of simple regulators with specifications on phase and amplitude margins", _Automatica_ 20(5), 1984.
- [Skogestad 2005] S. Skogestad, I. Postlethwaite, _Multivariable Feedback Control: Analysis and Design_, 2nd ed., Wiley, 2005.
- [Stein 2003] G. Stein, "Respect the unstable", _IEEE Control Systems Magazine_ 23(4), 2003.
- [Franklin 1998] G. F. Franklin, J. D. Powell, M. Workman, _Digital Control of Dynamic Systems_, 3rd ed., Addison-Wesley, 1998.
- [Khalil 2002] H. K. Khalil, _Nonlinear Systems_, 3rd ed., Prentice Hall, 2002.
- [Slotine 1991] J.-J. E. Slotine, W. Li, _Applied Nonlinear Control_, Prentice Hall, 1991.
- [Krstić 1995] M. Krstić, I. Kanellakopoulos, P. Kokotović, _Nonlinear and Adaptive Control Design_, Wiley, 1995.
- [Doyle 1978] J. C. Doyle, "Guaranteed margins for LQG regulators", _IEEE Trans. Automatic Control_ 23(4), 1978.
- [Doyle 1981] J. C. Doyle, G. Stein, "Multivariable feedback design: concepts for a classical/modern synthesis", _IEEE Trans. Automatic Control_ 26(1), 1981.
- [Glover 1989] K. Glover, D. McFarlane, "Robust stabilization of normalized coprime factor plant descriptions with H∞-bounded uncertainty", _IEEE Trans. Automatic Control_ 34(8), 1989.
- [Mahony 2008] R. Mahony, T. Hamel, J.-M. Pflimlin, "Nonlinear complementary filters on the special orthogonal group", _IEEE Trans. Automatic Control_ 53(5), 2008.
- [Ljung 1999] L. Ljung, _System Identification: Theory for the User_, 2nd ed., Prentice Hall, 1999.
- [Mueller 2014] M. W. Mueller, R. D'Andrea, "Stability and control of a quadrocopter despite the complete loss of one, two, or three propellers", _ICRA_ 2014.
- [Sun 2021] S. Sun, X. Wang, Q. Chu, C. de Visser, "Incremental nonlinear fault-tolerant control of a quadrotor with complete loss of two opposing rotors", _IEEE Trans. Robotics_ 37(1), 2021.
- [Betaflight] Betaflight documentation, _RPM filtering_ and _dynamic notch filter_, betaflight.com.
- [PX4] PX4 Autopilot, _Filter/Control Latency Tuning_, docs.px4.io.
