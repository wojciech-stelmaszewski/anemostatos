# Part III — Why It Works

> **Status (2026-10-01):** M13 (the foundations) and the first pass of M14
> (lessons III.1–III.5), M15 (III.7, III.9), M16 (III.11, III.12), M17
> (III.20–III.22) and M18 (III.27–III.30) are implemented, with III.10 and III.13
> of the second pass: all thirteen **must** lessons and Chapter J complete. §5 lists what was built and where
> it differs from this plan. The second passes of M14–M17 are planned.
> Milestones in [roadmap.md](roadmap.md) point here.
> Extended on 2026-09-30 from 20 to 30 lessons, so that Parts I–III together
> cover the analysis and robustness material of a graduate control course
> (see [curriculum.md](curriculum.md) for the whole programme).

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

Part III stays on the quadrotor. Other vehicles arrive in Part IV
([aerospace-gnc.md](aerospace-gnc.md)).

## 1. Gap audit

What control theory Parts I–II do not cover, checked against the code
(`src/control`, `src/estimation`, `src/sim`), the lessons and the book.

| Area                                 | What is missing                                                                               | Where the repo stands                                                                                                    | Decision                                                        |
| ------------------------------------ | --------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------- |
| **Frequency domain**                 | Transfer functions, Bode, Nyquist, gain/phase margins, sensitivity $S$ and $T$, loop shaping. | The book's lesson I.9 introduces the phase margin in one section. Nothing is plotted in the app.                         | **Yes, central.** Chapter F.                                    |
| **Poles, zeros, root locus**         | Pole maps (s- and z-plane), root locus, Routh–Hurwitz, system type and steady-state error.    | LQR shows closed-loop poles as numbers. The book uses Routh once (lesson I.5) and points to an appendix not yet written. | **Yes.** Chapter F.                                             |
| **Fundamental limits**               | Bode's sensitivity integral (waterbed effect); delay and RHP zeros cap the bandwidth.         | `delayMs` exists and Part I shows that delay destabilises, but not by how much.                                          | **Yes.** Chapter F.                                             |
| **Lyapunov stability**               | Lyapunov functions, $\dot V$, region of attraction, LaSalle.                                  | Zero mentions in the lessons. Geometric control is called "almost-global" without explaining why.                        | **Yes.** Chapter G.                                             |
| **Nonlinear loops**                  | Describing functions, limit-cycle prediction, the circle criterion.                           | Lesson I.2 meets a limit cycle and explains it by energy. One mention of describing functions.                           | **Yes.** Chapter G.                                             |
| **Structural properties**            | Controllability, observability, detectability, the separation principle.                      | Zero mentions. The Kalman filter runs, but we never ask what it _cannot_ estimate.                                       | **Yes.** Chapter G.                                             |
| **LQG and its lost margins**         | LQR has guaranteed margins, LQG has none [Doyle 1978]; loop-transfer recovery.                | LQR and the Kalman filter both exist on L1 (`WithAltitudeKalman`); nobody has measured the margins of the pair.          | **Yes.** Chapter G.                                             |
| **Uncertainty and robust stability** | Uncertainty models, the small-gain theorem, robust stability and performance.                 | Zero mentions.                                                                                                           | **Yes, central.** Chapter H.                                    |
| **Multivariable loops**              | Singular values, MIMO sensitivity, disk margins; why loop-at-a-time margins can lie.          | Zero mentions. All margins in the first plan were loop-at-a-time.                                                        | **Yes.** Chapter H.                                             |
| **H∞ and μ**                         | Worst-case design, mixed sensitivity, structured uncertainty.                                 | Rejected in Part II for lack of frequency-domain views. Chapter F removes that obstacle.                                 | **H∞ loop shaping: should. μ analysis: could.** No μ-synthesis. |
| **Sliding mode, backstepping**       | The classical robust nonlinear designs.                                                       | Rejected in Part II.                                                                                                     | **Yes, small.** Chapter H, on L1.                               |
| **Model-reference adaptive control** | MRAC, parameter drift, the Rohrs counterexample, robust modifications.                        | Part II teaches L1 adaptive control (II.18) without the MRAC it grew out of. One mention.                                | **Yes.** Chapter H.                                             |
| **Sampling and discretisation**      | z-plane, ZOH vs Tustin, loop rate vs margin, aliasing, anti-alias filters.                    | Controllers are discretised properly (`riccati.ts`, `lqr.ts`), but the student never sees the consequences.              | **Yes.** Chapter I.                                             |
| **Finite precision and rate limits** | Quantisation, fixed-point gains, actuator rate limits, timing jitter.                         | Zero mentions of quantisation. Setpoint rate limits only.                                                                | **Yes.** Chapter I.                                             |
| **Vibration and notch filters**      | Rotor-frequency vibration on the IMU, notch and RPM-tracking notch filters.                   | Not modelled. Low-pass filters only (`filters.ts`).                                                                      | **Yes.** Chapter I. Needs a vibration model.                    |
| **Nonlinear estimation**             | Complementary filter, Mahony, multiplicative EKF on quaternions, navigation EKF.              | Linear KF on L1 only. L3 reads true attitude plus noise (`physics-model.md` §6).                                         | **Yes.** Chapter I.                                             |
| **System identification**            | Least squares, relay auto-tuning, frequency-domain identification with coherence.             | Relay auto-tuner on the ideas list; Ziegler–Nichols dropped (`pid-primer.md` §6.2).                                      | **Yes.** Chapter I.                                             |
| **Fault detection and tolerance**    | Residual tests on the innovation; flying after the complete loss of a rotor.                  | The plant supports a dead motor (`motorEfficiency[i] = 0`); only partial loss is taught (II.7).                          | **Yes.** Chapter J.                                             |
| **Monte Carlo verification**         | Dispersions over parameters and seeds, pass rates with confidence, worst cases.               | Zero mentions. The arena flies each controller once per scenario.                                                        | **Yes, central.** Chapter J.                                    |
| Optimal control theory               | Pontryagin's principle, dynamic programming, trajectory optimisation.                         | The book's field map names them; no lesson teaches them.                                                                 | **Part IV**, Chapter K.                                         |
| Gain scheduling                      | Gains as a function of the operating point.                                                   | The quadrotor is close to LTI around hover, so it cannot motivate scheduling.                                            | **Part IV**, on the aircraft (Chapter M).                       |
| Other vehicles                       | Aircraft, launch vehicles, spacecraft, flexible structures.                                   | One vehicle.                                                                                                             | **Part IV.**                                                    |
| UKF, particle filters, smoothers     | Estimation when linearisation fails.                                                          | Zero mentions.                                                                                                           | **Part IV**, Chapter O.                                         |
| Iterative learning control           | Learn a feedforward correction from repeated laps.                                            | Figure-8 laps exist, so it would fit.                                                                                    | **Later.** A Part III extra.                                    |
| Multi-agent, consensus, formation    | Several drones sharing information.                                                           | One drone per simulation.                                                                                                | **No.** Changes the whole engine.                               |
| Others                               | Event-triggered and stochastic/dual control, reference governors, Koopman, value-based RL.    | —                                                                                                                        | **No.** Listed in the primer's further reading.                 |

## 2. The pedagogical arc

Part III is the **third semester**: numbering restarts at III.1. As in Part
II, each chapter opens with a question the student cannot yet answer.

| Chapter                                    | The question it answers                                                                    | New ideas                                                                                          |
| ------------------------------------------ | ------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------- |
| F. The loop as a filter (L1)               | "Lesson 'Find the edge' found the gain where it oscillates. Can I know that before I fly?" | Frequency response, poles, root locus, Routh–Hurwitz, margins, delay, sensitivity, loop shaping    |
| G. Stability and structure (L1)            | "It recovered this time. Will it always? And what can my filter never know?"               | Lyapunov, region of attraction, describing functions, observability, controllability, LQG          |
| H. Robust by construction (L1, L3)         | "My model is wrong. How wrong can it be before the loop fails?"                            | Uncertainty, small gain, singular values, disk margins, H∞, μ, sliding mode, backstepping, MRAC    |
| I. The real loop (L3)                      | "My analysis says 60° of margin. Why does the real drone buzz?"                            | Sampling, quantisation, aliasing, vibration, notch filters, attitude and navigation filters, sysid |
| J. When things break, and how we know (L3) | "One flight went well. Is that evidence?"                                                  | Loss of controllability, fault detection, Monte Carlo verification, a robustness report card       |

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
  svd.ts         singular values of small complex matrices (for MIMO loops, at most 4 × 4)
  margins.ts     gain/phase/delay margins and crossover from a sampled frequency response
  fft.ts         radix-2 FFT, Welch PSD and cross-spectra (spectra, coherence, the vibration chapter)
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
   rate PID or INDI stage; for lesson III.12, the coupled roll–pitch pair.
2. **Measured response.** `analysis/sweep.ts` breaks the loop at a
   **probe point** and injects a signal (see 3.3). It runs the real
   `Simulation` headlessly, then correlates at each frequency (a
   single-bin DFT per frequency) to get the complex gain. This works for
   every controller, including nonlinear ones (MPC, MPPI, the policy),
   where it measures a _describing function_. The lessons say so.

`analysis/linearize.ts` is a fallback that takes finite-difference
Jacobians of `sim/dynamics.ts` around trim. It checks the analytic plant
models in tests.

The other modules of `analysis/`:

```
src/analysis/
  sweep.ts        measured frequency response (above)
  linearize.ts    finite-difference Jacobians around trim
  describing.ts   describing functions of relay, saturation, dead zone, quantiser; limit-cycle solver
  uncertainty.ts  multiplicative weights from a plant family; the robust-stability test |W·T| < 1
  disk.ts         MIMO sensitivity, singular-value plots, disk margins
  mu.ts           upper bound on μ for real parametric uncertainty (could)
  montecarlo.ts   seeded dispersions, headless campaigns, pass rates with confidence bounds
```

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

For the MIMO lesson the torque probe exists on two axes
(`l3.torque.x`, `l3.torque.z`) and is run once per axis, which gives the
2 × 2 matrices $S$ and $T$ column by column.

Sweeps run many short simulations. They reuse the headless runner of
`engine/arena.ts` and run in time-sliced chunks, so the UI stays live. If
that is not enough, this is when `Simulation` moves to a Web Worker (on the
ideas list since M4). Monte Carlo campaigns (3.2) use the same runner.

### 3.4 Plant and sensor additions

| Addition                                                                                                                                                                 | Why                                                                                                           |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------- |
| **Rotor vibration** on the IMU: one sinusoid per motor at its rotor frequency $f_i = f_{hover}\sqrt{T_i/T_{hover}}$ (`vibration.amp`, `vibration.hoverHz`)               | Vibration on the gyro and accelerometer, aliasing, notch filters. Off by default so Parts I–II are unchanged. |
| **IMU sample rate** and optional analog anti-alias pole (`sensors.imuRateHz`, `sensors.aaFilterHz`)                                                                      | Aliasing appears only if the sensor samples slower than the physics (1 kHz).                                  |
| **Gyro bias** (`sensors.gyroBias`, deg/s, per axis) and a slow random walk                                                                                               | Makes attitude estimation non-trivial.                                                                        |
| **Magnetometer** (`sensors.mag`: body-frame field direction with noise and a constant disturbance)                                                                       | Heading for the attitude filters; without it yaw is unobservable from gravity alone.                          |
| **Attitude source** `sensors.attitude: 'truth' \| 'complementary' \| 'mahony' \| 'mekf'`                                                                                 | L3 flies on an estimate instead of noisy truth. Default `truth`, so the Part I/II regression baseline holds.  |
| **Navigation source** `sensors.nav: 'truth' \| 'ekf'` with a 10 Hz GPS-like position fix and a barometer                                                                 | L3 position/velocity from an EKF.                                                                             |
| **Quantisation** of sensors and motor commands (`sensors.quantBits`, `motors.quantBits`), **motor rate limit** (`drone.motorSlew`), **loop jitter** (`control.jitterMs`) | Finite precision and rate limits, lesson III.19. All off by default.                                          |
| **Scripted total motor loss** (existing `motorEfficiency`, event helper `loseMotor(i, t)`) and **sensor faults** (`stuck`, `drift`, `dropout` events)                    | Chapter J. The plant supports the motor loss already.                                                         |
| **Dispersions** (`dispersion` block: a standard deviation or a range for any numeric parameter path, plus the seed)                                                      | Monte Carlo campaigns, lessons III.11 and III.29.                                                             |

Vibration is a **sensor-level** signal. It is added to the gyro and
accelerometer readings inside `sim/sensors.ts` and does not enter the
rigid-body dynamics: the motion it would cause is far below anything
visible, and the lessons are about what it does to the measurement. Each
motor keeps an accumulated phase, so the frequency can follow the thrust.
With `hoverHz` up to 250 Hz the fastest rotor stays below the 500 Hz
Nyquist frequency of the 1 ms physics step, so the step does not change.

The sensor chain, in order, at the physics rate: truth, plus vibration,
through the anti-alias pole, then sample-and-hold at `imuRateHz`, then
quantisation. The controller reads the held sample. `imuRateHz = 0` (the
default) means no hold: the controller reads the newest value on each of
its ticks, as it does today.

### 3.5 New controllers and filters

```
src/control/
  smc.ts          L1 sliding mode: s = ė + λe, u = û_eq − k·sat(s/φ); φ = 0 gives pure sign()
  backstepping.ts L1 with the motor lag as a state (the first plant where backstepping has two steps)
  mrac.ts         L1 model-reference adaptive control: Lyapunov adaptation law, σ-modification, projection
  hinf.ts         L1 H∞ loop-shaping controller
  fault.ts        L3 reduced-attitude controller for three rotors [Mueller 2014], INDI inner loop [Sun 2021]
  shaping.ts      lead/lag and notch stages for any SISO loop (L1 thrust, L3 rate loops)
src/estimation/
  filters.ts      + biquad notch, + RPM-tracking notch bank (one notch per motor, tracks f_i)
  attitude.ts     complementary filter and Mahony on SO(3), with gyro-bias estimation
  mekf.ts         multiplicative EKF: quaternion reference, 3-vector attitude error, gyro bias
  ekf.ts          loosely coupled nav EKF: (p, v, b_a), attitude taken from attitude.ts or mekf.ts
  sysid.ts        least-squares fit of (m̂, τ̂_motor, delay) from sweep data; relay auto-tuner;
                  frequency-response estimate with coherence, and a transfer-function fit to it
  fdi.ts          innovation tests: normalised innovation squared against χ², CUSUM
```

### 3.6 New visualisations

- **Bode chart** (uPlot, log-frequency axis): magnitude and phase, model
  (solid) and measured (dots), with crossover, margins and the $-180°$
  line annotated. For MIMO loops the same chart draws the largest and
  smallest singular value. An optional third pane shows coherence.
- **Nyquist chart** (canvas): the $-1$ point, the unit circle and the
  distance-to-$-1$ circle, which is $1/\|S\|_\infty$. It can overlay the
  curve $-1/N(A)$ of a describing function and the disk of a disk margin.
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
- **Dispersion view** for Monte Carlo: all runs of a campaign as thin
  lines with the 5–95 % envelope, a histogram of the chosen metric and the
  pass rate with its confidence bound.
- **Robustness report card** in the arena: margins, $\|S\|_\infty$, delay
  margin, measured bandwidth and the Monte Carlo pass rate per controller.

`ExtraChart` grows `bode`, `nyquist`, `poles`, `spectrum`, `covariance`
and `dispersion`.

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

### Chapter F

**Frequency response on L1.** Plant $G(s) = \dfrac{e^{-s\tau_d}}{m s^2 (\tau_m s + 1)}$.
The controller is discretised at `control.rateHz` and plotted up to the
Nyquist frequency. The PID loop has one crossover, so gain and phase margin
read straight off the chart. The $-180°$ crossing sets the ultimate gain
that lesson I.10 ("Find the edge") found by trial.

**Poles, Routh–Hurwitz and system type.** The pole lesson also states the
two pencil-and-paper tools that belong with it. The Routh–Hurwitz test
decides stability from the coefficients of the characteristic polynomial;
for our third-order loop $m s^3 + K_d s^2 + K_p s + K_i$ it gives
$K_i < K_p K_d / m$, the limit the book's lesson I.5 already quotes. The
number of integrators in the loop (its type) decides which references and
disturbances leave a steady-state error, which explains lessons I.3 and I.5
in one table.

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

### Chapter G

**Lyapunov on L1.** For PD, $V = \tfrac12 k_p e^2 + \tfrac12 m v^2$ gives
$\dot V = -k_d v^2 \le 0$, and LaSalle finishes the argument. With thrust
saturation, the drone can only brake at $a_{max} = 4f_{max}/m - g$. The
region of attraction is bounded by $v^2 < 2 a_{max} h$. $V$ gives a
conservative inner estimate; the headless grid of initial states gives the
truth.

**Describing functions.** Lesson I.2 of Part I found a limit cycle (P-only
control with saturating motors) and explained it with energy. The
describing function predicts it. A static nonlinearity driven by a sine of
amplitude $A$ is replaced by its gain at the fundamental, $N(A)$: for a
relay of height $h$, $N(A) = 4h/(\pi A)$. A limit cycle is possible where
the Nyquist curve of the linear part meets $-1/N(A)$, and the crossing
gives its frequency and amplitude. The lesson draws both curves, reads
the prediction, and compares it with the flight. It also shows where the
method fails (it assumes the loop filters out the harmonics) and states
the circle criterion as the guarantee that does not rely on that
assumption [Khalil 2002].

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

### Chapter H

**Uncertainty and small gain.** The real plant is one member of a family:
$G_\Delta = G\,(1 + W\Delta)$ with $\|\Delta\|_\infty \le 1$, where
$|W(j\omega)|$ is the largest relative model error at each frequency. The
loop is stable for the whole family exactly when
$|W(j\omega)\,T(j\omega)| < 1$ at every frequency: the small-gain theorem
for this structure [Skogestad 2005]. For our loop the family is "mass
±30 %, motor time constant ×0.5–2, delay 0–20 ms". `uncertainty.ts`
computes $|W|$ from that family, and the lesson draws $1/|W|$ over
$|T|$. Where the curves touch, a plant in the family is unstable, and a
Monte Carlo campaign over the same family finds it. The test is
conservative for parametric uncertainty, and the lesson shows that too:
the curves can touch although every sampled plant is stable.

**Multivariable loops.** On L3, roll and pitch rate form a two-input,
two-output loop. While the drone spins in yaw the two axes are coupled
through the gyroscopic term of Euler's equations, the textbook example of
a plant whose loops look perfect one at a time [Skogestad 2005, §3.7].
Each loop, broken alone, shows large margins; a small gain error in both
at once destabilises the pair. The honest measures are the singular
values of $S(j\omega)$ and the **disk margin**, which bounds simultaneous
gain and phase changes in all channels [Seiler 2020]. The lesson measures
all three and ranks them. If our inertia values make the effect too weak
to see, the lesson uses a rotated IMU (a 20° mounting error) as the
coupling instead; §7 lists this as a risk.

**H∞ loop shaping.** Shape $W_2 G W_1$ with a PI weight, then
robustify with the normalised coprime-factor solution (two AREs, closed
form, `care.ts`) [Glover 1989]. The stability margin $\epsilon_{max}$ appears
next to the classical margins.

**μ analysis (could).** The small-gain test treats the uncertainty as one
unstructured block. When the uncertain quantities are known real numbers
(mass, motor time constant, delay), the structured singular value μ gives
a tighter answer. `mu.ts` computes an upper bound by diagonal scaling on a
frequency grid; the Monte Carlo worst case gives a lower bound. No
synthesis.

**Sliding mode (L1).** $s = \dot e + \lambda e$, $u = u_{eq} - k\,\mathrm{sign}(s)$.
It is robust to any matched disturbance with $|d| < k$, but chatters at the
loop rate and hits the motor lag. A boundary layer $\mathrm{sat}(s/\phi)$
trades the chatter for a small steady error. Plotted in the phase
portrait, with the sliding line drawn.

**Backstepping (L1 + motor lag).** Virtual control on thrust, then on the
motor command, with a Lyapunov function per step [Krstić 1995]. The
analytic model shows it is equivalent to a particular state feedback near
hover, and different at large errors.

**Model-reference adaptive control.** The controller adjusts its gains so
that the loop follows a reference model, with an adaptation law that comes
out of a Lyapunov function [Ioannou 1996]. On L1 with an unknown mass it
works: the tracking error goes to zero. Then the lesson adds what the
proof left out. With the motor lag unmodelled and a fast adaptation gain,
the adapted parameter drifts and the loop goes unstable, which is the
Rohrs counterexample [Rohrs 1985]. The robust fixes (σ-modification,
projection, a dead zone) stop the drift at the price of a small error. The
filter of L1 adaptive control (lesson II.18) is the same repair done
differently, and this lesson explains why II.18 needed it.

### Chapter I

**Sampling.** The z-plane pole map versus `control.rateHz`, from 1000 Hz
down to 50 Hz, and the margin as a function of rate: ZOH costs about half
a sample of delay.

**Finite precision and rate limits.** A quantiser is a nonlinearity with a
describing function, so a loop with an integrator and a coarse sensor
settles into a small limit cycle instead of resting; the lesson predicts
its amplitude with `describing.ts`. A motor rate limit does the same at
large amplitudes and is the mechanism behind pilot-induced oscillation in
aircraft (Part IV). Timing jitter is measured as lost phase margin.

**Aliasing.** Vibration at $f_i$ sampled at `imuRateHz`
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

**Multiplicative EKF.** A quaternion has four numbers and three degrees of
freedom, so an ordinary EKF on it has a singular covariance. The
multiplicative EKF keeps a reference quaternion and estimates a
three-component error rotation plus the gyro bias, then folds the error
back into the reference after each update [Markley 2003]. It is the
standard attitude filter of spacecraft. The lesson compares it with
Mahony when the magnetometer is disturbed, and shows its covariance, which
Mahony does not have.

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

**Frequency-domain identification.** The method flight-test engineers use
[Tischler 2012]. A chirp excites the vehicle; the frequency response is
estimated from averaged cross- and auto-spectra, $\hat H = S_{xy}/S_{xx}$,
together with the **coherence** $\gamma^2$, which says at each frequency
how much of the output is explained linearly by the input. A transfer
function is then fitted only where the coherence is high. With wind on,
the coherence drops where the wind dominates, and the lesson shows the
fit going wrong if those frequencies are included.

### Chapter J

**Losing a rotor.** Four motors produce four independent quantities:
thrust and three torques. The matrix that maps motor thrusts to those four
(the control effectiveness matrix, the inverse of the mixer) has rank 4.
Remove a column and the rank is 3: thrust, roll, pitch and yaw can no
longer be commanded independently, and there is no motor setting that
hovers without rotating. The **controllability readout** of lesson III.27
is that rank, shown with the direction that was lost. The drone can still
hold position if it gives up yaw and spins about a tilted primary axis
[Mueller 2014]. Reduced-attitude control of that axis, with an INDI inner
loop [Sun 2021]. The spin settles where the yaw reaction torque meets the
rotational drag, at roughly $c_\tau m g / k_{damp}$. With today's defaults
that is about 78 rad/s, or 18° of rotation per 250 Hz controller tick,
which is too fast to control (see §7).

**Fault detection.** A Kalman filter that matches reality produces
innovations that are white and have the covariance the filter predicts.
When a sensor sticks or a motor weakens, they stop doing so. The
normalised innovation squared is compared with a χ² threshold, and a CUSUM
test accumulates small persistent deviations [Basseville 1993]. The lesson
sets the threshold and watches the trade between detection delay and false
alarms, then uses the detection to switch the estimator or the controller.

**Monte Carlo verification.** A single good flight is one sample. A
campaign flies the same scenario $N$ times with parameters drawn from
their dispersions (mass, motor time constant, sensor bias, delay, the wind
seed) and reports the distribution of the result. A requirement becomes a
pass rate with a confidence bound: with no failure in $N$ runs, the
failure probability is below about $3/N$ at 95 % confidence, so "99 %
with 95 % confidence" needs roughly 300 clean runs. This is how launch
vehicles and landers are verified [Hanson 2010]. The lesson finds the
failing corner of the parameter space that the nominal run never visits.

## 5. Milestones

Each ends with something runnable; `make check` passes at each commit, and
the Part I/II behavioural baseline stays unchanged.

**Build order.** Two passes. The first pass is M13 and then the **must**
lessons of M14–M18 (III.1–4, 7, 9, 11, 12, 20–22, 29, 30), with only the
code those lessons need. The second pass fills in the **should** and
**could** lessons. Lesson numbers are fixed now, so the first pass leaves
gaps in the list.

### M13 — Foundations for Part III ✅

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
a ratio of a large signal to a very small one; above it the 250 Hz
controller and the 1 kHz physics stop looking like one sampled system.

**Built (2026-09-30).** The acceptance test passes with a wide margin:
model and measurement agree within 0.01 dB and 0.5° across the band, for
the default PID and for six other configurations (D on the error with
40 ms of delay, 50 Hz sampling, LQR, LQI with the lag state, LQR without
motor telemetry, ADRC). The predicted delay margin of 154 ms is confirmed
in flight: stable at 149 ms, unstable at 160 ms. What differs from the plan
above:

- **The plant model is discrete.** `analysis/plant.ts` writes the L1 plant
  exactly as the simulator steps it (1 ms), because the simulator's motor
  step leads the continuous model by half a step, 3.6° at 20 Hz. The
  textbook model $1/(m s^2(\tau s + 1))$ is kept beside it for comparison.
- **Controller models live in one file**, `analysis/controllers.ts`, as
  functions of the parameters, not as `linearModel()` methods: they need
  no controller instance. Each is the controller's own update rule in
  state-space form.
- **Two rates.** The loop response accounts for the controller sampling
  every $n$ physics steps and holding: $L = G \cdot K \cdot H$ with the hold
  gain $H = \frac1n (1 - e^{-j\Omega n})/(1 - e^{-j\Omega})$. Closed-loop
  poles come from lifting the plant to the controller period, with the
  sensor delay as extra states (`analysis/loop.ts`).
- **Probe points and signals.** Implemented: `l1.thrust` and `ref.y`, with
  `sine` and `chirp`. The L3 points arrive with the L3 models in M17,
  `multisine` with M14 and `relay` with M17.
- **Layout.** An analysis chart chosen as the extra view takes the whole
  right column of the chart grid (both rows, in place of the info card):
  a Bode plot needs the height. A second analysis view can replace the
  motor chart (`chart2` in a lesson, "beside it" in the chart header).
- **Charts are drawn on a canvas**, not with uPlot, like the phase
  portrait: a logarithmic axis, two panes and annotations are simpler
  that way. Each has a hover readout; the pole map keeps a trail.
- `fft.ts` is in place (FFT, Welch PSD, single-frequency correlation); the
  spectrum chart that uses it belongs to M14.

### M14 — Chapter F lessons (III.1–III.6)

**First pass built (2026-09-30): III.1–III.5.** Each has a goal that the
tests prove unmet at the start and reachable by its documented solution.

- The Bode chart has four views: the open loop $L$, the closed loop $Y/R$
  (with the −3 dB bandwidth marked), the sensitivity $S$ (with its peak),
  and the waterbed view: $|S|$ on a linear frequency axis with the areas
  of $\ln|S|$ above and below zero shaded and integrated to the Nyquist
  frequency. A lesson picks the view with `bode`.
- The `predict` field: a lesson names a quantity, the panel shows an input
  box, and `goal.check` receives the number. The right answer comes from
  the loop model (`truth(params)`), not from a headless run as first
  planned: the model already agrees with flight to a fraction of a degree.
- III.1 is judged by flight: the goal compares the drone's swing with the
  setpoint's at the probe frequency the student chose.
- III.5's goal is a trade stated in numbers ($|S| \le -14$ dB at 0.5 Hz
  with a peak below 4 dB), because a stronger integral alone hardly moves
  the peak of this loop; the bandwidth does.
- The PID model drops the states of terms that are switched off, so the
  pole map shows no poles that the loop cannot move.

**Still to build:** III.6 (loop shaping, with `shaping.ts`), the spectrum
chart and the `multisine` probe signal.

Root-locus trail, delay margin, the `predict` field (first used by III.4),
$S$/$T$ measurement, `shaping.ts` (lead/lag, notch on L1), spectrum chart.

### M15 — Chapter G (III.7–III.10)

**First pass built (2026-09-30): III.7 and III.9.**

- `analysis/attraction.ts`: the Lyapunov estimate (the largest level set of
  $V$ in which a PD command never saturates), the braking limit of the
  motors, and the true boundary, found by flying the simulator from a grid
  of initial states with bisection on the dive speed.
- The phase portrait has a Lyapunov mode that draws all three, with the
  level sets of $V$ behind them. In III.7 the proven region reaches
  0.55 m/s; the drone survives dives of 2.17 m/s.
- III.7 asks for the fastest dive survived. The hand formula
  $\sqrt{2 a_{max} h}$ gives 2.14 m/s against 2.17 m/s flown. The lesson
  flies low (0.6 m) on purpose: from higher up, air drag makes the formula
  pessimistic by more than the 10 % allowed.
- `analysis/structure.ts`: controllability and observability matrices and
  their rank. The Kalman info card shows "observable states: 3 of 4".
- The altitude filter takes an optional fourth state, the altimeter bias
  (`control.kalman.altBiasState`). Its σ stays at its prior for ever.
- `sensors.posDropout` holds the last altimeter fix; III.9 scripts five
  seconds of it.
- III.9 has a goal the plan did not: make the filter **consistent** (the
  truth inside its ±2σ band 95 % of the time). With an altimeter that
  reads 0.3 m high, the three-state filter is confident and wrong; the
  four-state filter cannot estimate the bias either, but says so.
- A chart of the filter's σ per state on a logarithmic axis (`covariance`).
- Lessons carry their plan number (`n`), so the panel shows III.7 and
  III.9 although III.6 and III.8 do not exist yet.

**Built (2026-10-01): III.10.**

- The LQR takes an option `lqr.observer`: a steady-state Kalman filter
  (current-estimator form, the dual Riccati equation of `riccati.ts`)
  estimates (y, v, T) from the altimeter and the command, with assumed
  process noise `kfQ` at the thrust input and altimeter noise `kfR`. Its
  linear model joins the loop models, so Bode, Nyquist and the margins are
  live, and it matches a flown sweep within 2 %.
- With sensors the LQR has 72° and an infinite gain margin; with the filter
  (kfQ = 0.3 N) 35° and 9 dB. Raising kfQ recovers them (LTR): 46° at 3 N,
  62° at 100 N. The plan's "slightly heavier drone oscillates" does not
  happen: the lost margin is at the plant input, and a heavier drone only
  lowers the loop gain. The lesson's trade is noise instead: with 2 cm of
  altimeter noise the thrust jitter grows from 0.01 N to 0.24 N as kfQ goes
  from 0.3 to 10 N. The goal (PM ≥ 45°, GM ≥ 10 dB, jitter < 0.15 N) leaves
  a window of kfQ ≈ 3–5 N.

**Still to build:** III.8 (describing functions).

Lyapunov overlay and region-of-attraction grid; `describing.ts` and the
$-1/N(A)$ overlay; altimeter-bias state and altimeter-dropout event for
the Kalman filter; covariance chart; LQG margin measurement.

### M16 — Chapter H (III.11–III.17)

**First pass built (2026-09-30): III.11 and III.12.**

- `analysis/uncertainty.ts`: the plant family (mass ±30 %, motor lag
  ×0.5–2, up to 20 ms of extra delay; the `uncertainty` block), its
  weight $|W|$ as the largest relative error of a 75-member grid, and the
  small-gain test $|W T| < 1$. The Bode chart has a robust-stability view
  that draws $|T|$ under the ceiling $1/|W|$.
- `analysis/montecarlo.ts`: seeded campaigns over the family, flown in
  slices ("Fly 60 members"). Flight and model agree on every trial.
- In III.11 the members that fail are the light ones with a late sensor
  (more loop gain, less phase), not the heavy ones with slow motors that
  the plan expected.
- III.12 uses the fallback of §7: the coupling is a **gyro mounted turned
  about the vertical axis** (`sensors.imuYawDeg`), which the controller
  can correct (`control.model.imuYawDeg`). No yaw spin is needed.
- The probe has two more points, `l3.torque.x` and `l3.torque.z`; two
  sweeps give the 2 × 2 sensitivity $S$ and $T$ at the torque input.
- `analysis/mimo.ts`: the model of the roll–pitch loop,
  $L = a\,R(\theta) + b\,I$ (gyro path turned, attitude path not). It
  matches the flown $S$ within 2 % from 0.5 to 20 Hz. Singular values of a
  2 × 2 complex matrix are computed in closed form, so `svd.ts` was not
  needed; the disk margin (Seiler, Packard and Gahinet 2020) lives here
  too, not in a separate `disk.ts`.
- $L$ is a normal matrix, so the pair splits into two eigen-loops
  $\lambda_\pm = a\,e^{\pm j\theta} + b$: the mounting angle is a phase
  shift of $\theta$ in the gyro path. Their margins are exact for changes
  common to both channels, and the lesson shows them next to the
  loop-at-a-time and the disk margin.
- The plan's demonstration was a 15 % gain error in both channels. The
  clearer one is delay: with the gyro at 30° the loop-at-a-time delay
  margin is 35 ms, the true one 21 ms, and the drone shakes at 22 ms. The
  student predicts the 21 ms, then repairs the loop by telling the
  controller the angle.

**Built (2026-10-01): III.13.**

- `math/care.ts` solves the continuous Riccati equation by the matrix sign
  function of the Hamiltonian; `control/hinf.ts` is the Glover–McFarlane
  design: the shaped plant G·W, the two equations, γ_min = √(1 + ρ(XZ)),
  the central controller at γ = 1.05·γ_min, discretised (Tustin) at the
  controller rate. W is a PI times an optional lead; the PI integrator is
  run explicitly so that it can be held on the ground. L1 kind `hinf`.
- The plan compared H∞ with lesson III.6, which does not exist yet; the
  lesson compares it with the PID of III.11 instead. On a 108-tune grid
  the fastest PID that passes the family test crosses at 2.84 Hz.
- A PI weight alone gives ε_max ≈ 0.31 and fails the family test from
  about 2 Hz: a −40 dB/decade shape is the wrong one. With a lead in the
  weight (5 → 50 rad/s) ε_max is 0.41 and the loop passes at 3.5 Hz. The
  lesson's goal is ≥ 3.2 Hz with |W·T| < 1. The measured phase margin is
  within a few degrees of 2·arcsin ε_max.

**Still to build:** III.14–III.17 (μ, sliding mode, backstepping, MRAC).

First pass: `uncertainty.ts`, the dispersion block and `montecarlo.ts`
(shared with M18), `svd.ts`, `disk.ts`, the two-axis torque probe and the
coupled roll–pitch model. Second pass: `care.ts` and `hinf.ts`, `mu.ts`,
`smc.ts`, `backstepping.ts`, `mrac.ts`, the sliding line in the phase
portrait.

### M17 — Chapter I (III.18–III.26)

**First pass built (2026-09-30): III.20, III.21 and III.22.**

- `sim/vibration.ts`: one tone per motor at its rotor frequency,
  $f_i = f_{hover}\sqrt{T_i/T_{hover}}$, with an amplitude that grows
  with the thrust (`vibration.gyroDeg`, `vibration.acc`,
  `vibration.hoverHz`). The default hover frequency is 110 Hz, not the
  180 Hz of §4: it suits a 1 kg drone with 8-inch propellers.
- The IMU front end runs at the physics rate inside `sim/sensors.ts`:
  truth, plus gyro bias and vibration, through the anti-alias filter
  (`sensors.aaFilterHz`, two poles rather than one), then sample-and-hold
  (`sensors.imuRateHz`). Quantisation is left for III.19.
- `control/gyrofilter.ts`: a low-pass, a fixed notch, or one notch per
  motor that follows its speed from the motor telemetry
  (`control.gyroFilter`). `estimation/filters.ts` has the biquad sections
  and their frequency response.
- The rate-loop model of `analysis/mimo.ts` includes the whole gyro chain
  (anti-alias filter, hold, low-pass, notches), so every filter's cost is
  read as phase margin. It matches the flown $S$ within 3 %.
- `engine/tap.ts` records four signals at 1 kHz for eight seconds: the
  true roll rate, the gyro reading, the reading after the filters and one
  motor command. The ordinary telemetry (200 Hz) would alias them. The
  **spectrum chart** draws them, with the rotor frequency and the IMU's
  Nyquist frequency marked.
- III.20 samples 110 Hz rotors at 100 Hz: a 10 Hz ghost. The student
  predicts the frequency. The goal measures the ghost as the part of the
  gyro reading below 60 Hz that is not motion, and asks for 40° of margin,
  which rules out a heavy anti-alias filter on a slow IMU.
- III.21 flies a sine in altitude instead of throttle steps: the tone
  moves from below 90 Hz to almost 130 Hz and the baseline motor jitter is small and
  smooth. A low-pass that reaches the goal costs 8° of margin, the fixed
  notch lets most of the tone through, the RPM notch reaches the
  no-vibration baseline for one degree.
- `estimation/attitude.ts`: the complementary filter on the rotation
  group in Mahony's explicit form. "Complementary" is that filter with a
  gain $1/\tau$ and nothing else; "Mahony" adds the bias integrator and a
  gate on the length of the accelerometer reading. The heading comes from
  an ideal compass; the magnetometer model waits for III.23.
- `sensors.attitude` selects what the controller flies on; the filter
  runs in the IMU front end. `sensors.gyroBias` is a constant bias.
- III.22 hovers for ten seconds and then flies four laps of a 3 m circle
  with the geometric controller (the PID cascade lags a circle by
  metres). The accelerometer of a multirotor reads the thrust, which
  points along the body, so in the turn it reports "level". No time
  constant of the complementary filter gets below 8° (bias error
  $b\tau$ against turn error $\theta/\sqrt{1+(\Omega\tau)^2}$). Mahony
  without the gate is fooled as well; with a gate of 1–3 % it stays
  under 2°. The student predicts the 11 % by which the reading exceeds
  1 g. A gate just under that value fails, because the reading passes
  through every length on the way into and out of the turn.

**Built (2026-10-02): III.23 and III.24.**

- `estimation/mekf.ts`: the multiplicative EKF, a reference quaternion
  plus a 6-state error filter (small rotation, gyro bias), propagated at
  1 kHz; gravity and north are read at 100 Hz, each through a χ² gate on
  its NIS. The magnetometer corrects the heading only (one degree of
  freedom about the estimated vertical): used as a full vector, a
  disturbed reading also tilted the estimate.
- The magnetometer (`sensors.magnetometer`, `magNoiseDeg`,
  `magDisturbDeg`) sees the field at 65° of dip, turned by the local
  disturbance. Mahony and the complementary filter take north from its
  horizontal part; without it they keep the ideal compass of III.22.
- With an attitude or navigation filter in the chain, the IMU's noise is
  added before the filters (the controller reads the same samples), not
  after; with the default noise of 0 nothing earlier changes.
- III.23 hovers past steel (40°, 6–16 s): Mahony 46° of heading error,
  the complementary filter 40°, the MEKF without a gate 51°, with a gate
  of 16.3 under 2°. The rejections show as shading and the filter's 2σ on
  yaw grows while it flies on the gyro.
- `estimation/navekf.ts`: position, velocity and accelerometer bias,
  propagated with the accelerometer turned by the believed attitude,
  corrected by GPS-like fixes (`sensors.gpsRateHz`, `gpsNoise`) and a
  50 Hz barometer, gated on the NIS. `sensors.nav: 'ekf'` makes the L3
  controller fly on it.
- III.24 uses 10 cm of receiver noise: with 30 cm the estimate alone
  quadruples the tracking error and the plan's "2× of flying on truth"
  cannot be met by any gate. One fix 6 m off at 8 s: ungated the
  estimate jumps over 2 m and the RMS doubles; gated (16.3) it stays
  under 2× of the truth, which the goal computes by flying the same
  setting on the true position.

**Still to build:** III.18, III.19, III.25, III.26.

First pass: vibration model, IMU sample rate and anti-alias pole, notch
and RPM notch bank, `attitude.ts`, gyro bias, magnetometer. Second pass:
quantisation, rate limit and jitter; `mekf.ts`, `ekf.ts`, `sysid.ts` with
the relay probe and the coherence estimate; linear models of the L3 rate
loop (PID and INDI).

### M18 — Chapter J and the report card (III.27–III.30)

**First pass built (2026-10-01): III.29 and III.30.**

- `analysis/dispersion.ts`: a campaign flies one scenario $N$ times with
  mass, motor lag, sensor delay, gyro bias and the wind's seed drawn
  anew, in slices. Every tune flies the same draws (common random
  numbers), so a retune is compared on the same 300 drones. The
  dispersions are fixed in code rather than a `dispersion` block of the
  parameters; one set serves both lessons.
- A flight fails if it crashes or still exceeds 300 °/s of body rate in
  its last five seconds. The plan's position limit in gusts failed most
  flights for reasons no tune could change (the wind); the rate limit sits
  in a wide gap: gusts stay below 140 °/s, unstable loops pass 500 °/s.
- Pass rates carry a one-sided Clopper–Pearson bound; `cleanRunsNeeded`
  gives 299 for 99 % at 95 %.
- The **dispersion view** places every flight by two of its drawn values,
  passed or failed. It replaces the plan's envelope and histogram: the
  lesson is about where the failures sit.
- III.29: the default cascade fails about 7 % of 300 gusty hovers, all
  where motors slower than 25 ms meet a sensor later than 30 ms; the
  rate-loop model agrees with the flights on at least 295 of 300.
  Attitude Kp 5 flies the same 300 without a failure. The goal reads the
  finished campaign of the current tune, so its test lives in
  `tests/dispersion.test.ts`.
- `analysis/report.ts` builds the **report card**: for every L1
  controller and every L3 entry of the arena, PM, GM (the smaller of up
  and down), delay margin and $\|S\|_\infty$ from a probe sweep at the
  plant input, a 50-flight campaign (the III.11 family for L1, the III.29
  dispersions for L3) and the RMS error in gusts. `make report` prints it
  and writes [report.md](report.md); lesson III.30 builds it in the
  browser (about four minutes). It is a lesson panel, not a column of the
  arena, and `make bench` is unchanged.
- The measured margins of the two PID loops match the models within
  1.5°. MPPI and the neural policy fly, yet the sweep gives them negative
  margins: they are not linear time-invariant loops, so the card shows a
  dash and only the Monte Carlo row speaks for them.

**Built (2026-10-01): III.27 and III.28.** M18 is complete.

- The prototype of §7 confirmed the risk: with `angularDamping` 0.002 a
  drone on three rotors spins at 78 rad/s and cannot be held. A separate
  `drone.yawDamping` (default 0.002, so nothing earlier changes) stands for
  the rotor drag in yaw; the lessons use 0.012, a spin of 13 rad/s, which
  the student predicts as $c_\tau m g / k_{yaw}$.
- `control/fault.ts`: the cascade until the fault, then a position PID,
  a reduced-attitude law on the direction of the thrust axis with the
  gyroscopic coupling cancelled, and an exact solve for thrust and the
  two torques on the three working motors. Yaw is given up. In the spinning
  body every lag turns the torque, so the law commands it ahead by
  $\arctan(\Omega\tau_m) + \Omega\,\tau_{delay}$; without that lead it
  falls, which is the second step of III.27. It holds within 0.15 m for any
  motor lost, survives 10 ms of sensor delay and falls at 20 ms: the
  lesson says so. INDI was not needed; a design in the spinning frame
  [Mueller 2014] would widen the margin and is left for later.
- The info card shows the rank of the control effectiveness, 4 then 3.
- `estimation/fdi.ts`: a fault monitor with two tests. The altimeter: a
  vertical Kalman filter (the L1 filter, now reporting its NIS) and the
  NIS summed over a window. The plan's one-sided test cannot see a stuck
  sensor: the filter believes it and follows it. The test is two-sided:
  a stuck sensor has stopped being noisy and its NIS sum falls below the
  lower χ² tail. The motors: the torque the gyro shows against the torque
  the rotor telemetry promises, filtered alike, and a CUSUM per motor
  along its arm, which also isolates the motor.
- `sensors.posStuck` repeats the last fix at the sensor's rate and calls it
  fresh. III.28 samples position at 20 Hz: at 1 kHz a window of fixes
  spans milliseconds, the innovations are correlated and the sum is not
  χ².
- The monitor can report the lost motor to the fault-tolerant mode
  (`control.fault.source`). The rotor loss is a cliff: told within 40 ms
  the drone loses under a metre, within 80 ms it reaches the ground. The
  goal asks for 50 ms and no drop below 3 m, and the stuck sensor within
  1 s, with no false alarm in the healthy minutes. The plan's motor at
  60 % became the propeller loss of III.27, so that the two lessons form
  one chain: detect, isolate, reconfigure.

First pass: the dispersion view, Monte Carlo campaigns in the arena, the
robustness report card, and `make bench` printing margins and pass rates
next to RMS. Second pass: `fdi.ts` and the sensor-fault events; the
`loseMotor` event, `fault.ts` and the controllability readout. III.27
starts with a headless prototype of the three-rotor spin before any lesson
is written (§7).

## 6. Lessons (Part III)

Priority: **must** (core story), **should**, **could**.

### Chapter F — The loop as a filter

| #   | id         | Title                  | Setup and events                                                               | What the student sees / goal                                                                                                                                                   | Prio   |
| --- | ---------- | ---------------------- | ------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------ |
| 1   | `bode`     | The drone as a filter  | L1 PID, sine probe on the setpoint; frequency slider, then an automatic sweep. | Slow sines are followed, fast ones are ignored, with a resonance in between. The measured dots land on the model curve. Goal: find the −3 dB bandwidth.                        | must   |
| 2   | `poles`    | Poles tell the story   | Pole map beside the step response; drag $K_p$, $K_d$, then $K_i$.              | Poles move, the step response changes with them; the root-locus trail crosses into instability at the gain Routh–Hurwitz predicts. Goal: place poles for ζ ≈ 0.7.              | must   |
| 3   | `margins`  | How far from the edge? | Loop broken at the thrust; Bode and Nyquist of $L$.                            | Gain and phase margin as distances to $-1$. Goal: PM ≥ 45° and GM ≥ 6 dB with the highest possible crossover.                                                                  | must   |
| 4   | `delay`    | Delay is a phase thief | Same loop, `delayMs` slider 0 → 60 ms.                                         | The phase curve bends down, the margin disappears, and the drone oscillates at the predicted frequency. Goal: predict the critical delay within 10 %.                          | must   |
| 5   | `waterbed` | The waterbed           | Measure $\|S\|$; stronger integral action; turbulence on.                      | Better low-frequency rejection, a higher $\|S\|$ peak, and gusts at that frequency amplified in flight. On the linear-frequency view the areas above and below zero are equal. | should |
| 6   | `shaping`  | Shape the loop         | Lead/lag and notch blocks on the L1 loop; a target loop shape drawn in.        | Design by bending the Bode plot instead of turning gains. Goal: match the target shape and hit PM ≥ 50°.                                                                       | should |

### Chapter G — Stability and structure

| #   | id           | Title                         | Setup and events                                                                                                        | What the student sees / goal                                                                                                                                        | Prio   |
| --- | ------------ | ----------------------------- | ----------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------ |
| 7   | `lyapunov`   | Energy that only goes down    | PD on L1, low max thrust; phase portrait with $V$ level sets; scripted dives from rising heights.                       | $V$ never increases, until the thrust saturates. The Lyapunov estimate sits inside the true region of attraction. Goal: predict which dive crashes.                 | must   |
| 8   | `describing` | The bounce, predicted         | The P-only loop of lesson I.2 with saturating motors; Nyquist chart with the $-1/N(A)$ curve.                           | The crossing gives the amplitude and period of the limit cycle before the flight; then a case with strong harmonics where the prediction is off. Goal: within 15 %. | should |
| 9   | `observable` | What the filter can't see     | Kalman filter with the altimeter bias as a state; then a 5 s altimeter dropout.                                         | Bias covariance never shrinks (rank-deficient observability matrix); during the dropout, position uncertainty grows as $t^{3/2}$.                                   | must   |
| 10  | `lqg`        | Optimal plus optimal ≠ robust | LQR with the lag state and no sensor delay, then LQR + aggressive KF, then a detuned KF; margins measured by the probe. | LQR: ≥ 60° as promised. LQG: a small margin, and a slightly heavier drone oscillates. Goal: recover PM ≥ 45° by detuning the filter.                                | should |

### Chapter H — Robust by construction

| #   | id             | Title                     | Setup and events                                                                                              | What the student sees / goal                                                                                                                                                  | Prio   |
| --- | -------------- | ------------------------- | ------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------ |
| 11  | `smallgain`    | How wrong can I be?       | L1 PID; a plant family (mass ±30 %, motor lag ×0.5–2, delay 0–20 ms); $1/\|W\|$ drawn over $\|T\|$.           | Where the curves touch, a Monte Carlo campaign finds an unstable plant. Goal: retune so that the whole family is stable, with the highest bandwidth the test allows.          | must   |
| 12  | `mimo`         | One loop at a time lies   | L3 roll and pitch rate loops while spinning in yaw; loop-at-a-time margins, then singular values of $S$.      | Each loop alone shows large margins; a 15 % gain error in both at once destabilises the pair. The disk margin predicted it. Goal: disk gain margin ≥ 6 dB.                    | must   |
| 13  | `hinf`         | Design for the worst      | H∞ loop shaping vs the lesson-6 design on the family of III.11.                                               | The H∞ design keeps its margins across the whole family; the hand-shaped one loses them at one corner. $\epsilon_{max}$ is shown next to the classical margins.               | should |
| 14  | `mu`           | Uncertainty has structure | The family of III.11 treated as three real parameters; μ upper bound vs the small-gain test vs Monte Carlo.   | The small-gain test rejects a design that every sampled plant flies. μ accepts it. The three answers are ordered as the theory says.                                          | could  |
| 15  | `smc`          | Slide to the target       | L1 SMC, payload +300 g, gusts; boundary layer φ = 0 at first.                                                 | State slides down the line in the phase portrait, rejects the payload, and the motors chatter. Goal: remove chatter (thrust jitter < 0.3 N) with error < 3 cm.                | should |
| 16  | `backstepping` | One step at a time        | L1 with the motor-lag state; backstepping vs PD on a large step.                                              | Near hover they match (same linear model); at large errors backstepping respects the lag and overshoots less.                                                                 | could  |
| 17  | `mrac`         | Learning can diverge      | L1 MRAC with an unknown mass; then an unmodelled motor lag, a fast adaptation gain and a sinusoidal setpoint. | First the gains converge and tracking is perfect. Then the adapted gain drifts until the loop goes unstable. Goal: stop the drift with σ-modification, tracking error < 5 cm. | should |

### Chapter I — The real loop (L3)

| #   | id         | Title                        | Setup and events                                                                                       | What the student sees / goal                                                                                                                         | Prio   |
| --- | ---------- | ---------------------------- | ------------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------- | ------ |
| 18  | `rate`     | How slow can you go?         | L3 rate loop; `rateHz` slider 1000 → 50 Hz; z-plane pole map.                                          | Poles creep towards the unit circle; each halving of the rate costs predictable phase. Goal: find the lowest rate with PM ≥ 45°.                     | should |
| 19  | `quant`    | The computer cannot count    | Coarse altimeter (8 bits), then a motor rate limit, then 2 ms of loop jitter.                          | A loop that should rest hunts between two sensor steps; the describing function predicts the amplitude. Goal: the fewest bits with ripple < 1 cm.    | should |
| 20  | `alias`    | Ghost frequencies            | Rotor vibration on, IMU at 200 Hz, no anti-alias pole; spectrum chart.                                 | A 20 Hz wobble that isn't there; no digital filter removes it. Goal: switch on the anti-alias pole or raise the IMU rate.                            | must   |
| 21  | `notch`    | Notch the noise              | Vibration at a proper IMU rate; D-term amplifies it; fixed notch, then RPM notch; throttle steps.      | The fixed notch works at hover, fails on a climb; the RPM notch follows. Goal: motor jitter below the no-vibration baseline + 10 %.                  | must   |
| 22  | `attitude` | Where is up?                 | L3 flies on estimated attitude; gyro bias; fast 3 m circle; complementary vs Mahony.                   | In the turn the accelerometer "sees" a tilted gravity, and the complementary filter trusts it. Mahony holds. Goal: tilt error < 2°.                  | must   |
| 23  | `mekf`     | Four numbers, three unknowns | MEKF vs Mahony; a magnetic disturbance at t = 6 s; covariance chart.                                   | Mahony's heading follows the disturbance; the MEKF's innovation gate rejects it, and its covariance says how sure it is. Goal: heading error < 3°.   | should |
| 24  | `ekf`      | Fly on beliefs               | Nav EKF with 10 Hz GPS-like fixes and baro; one outlier fix at t = 8 s.                                | Innovation and NIS plot; the gated EKF ignores the outlier, the ungated one jumps. Goal: tracking RMS within 2× of flying on truth.                  | should |
| 25  | `sysid`    | Identify, then design        | The model block starts wrong (m̂ −30 %, τ̂ ×2); run a sweep, fit, redesign LQR; relay-tune the rate PID. | Fitted values next to truth; LQR on the fitted model beats LQR on the wrong one. Relay finds $K_u, T_u$.                                             | should |
| 26  | `freqid`   | Trust the coherence          | Chirp on the rate loop in calm air, then in turbulence; coherence pane under the Bode chart.           | Where coherence is high the estimate matches the model; where wind dominates it does not. Goal: a fit with every parameter within 10 % of the truth. | should |

### Chapter J — When things break, and how we know (L3)

| #   | id           | Title                      | Setup and events                                                                                                      | What the student sees / goal                                                                                                                                               | Prio   |
| --- | ------------ | -------------------------- | --------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------ |
| 27  | `3motors`    | Three rotors left          | Hover at 5 m; motor 2 lost at t = 5 s; standard cascade vs fault-tolerant controller.                                 | The cascade flips and crashes. The effectiveness matrix drops from rank 4 to 3. The fault controller gives up yaw, spins and holds position within 0.5 m.                  | should |
| 28  | `fdi`        | Notice that it broke       | Altimeter sticks at t = 6 s; then a motor at 60 %; NIS and CUSUM charts with a threshold slider.                      | A low threshold raises false alarms in turbulence, a high one detects late. Goal: detect both faults within 0.5 s with no false alarm in 60 s of gusts.                    | should |
| 29  | `montecarlo` | One flight proves nothing  | The default L3 cascade on a gusty hover, flown 300 times with dispersed mass, motor lag, sensor bias, delay and wind. | The nominal run passes; a few per cent of the campaign does not, all in one corner of the parameter space. Goal: a retune with 300 clean runs (≥ 99 % at 95 % confidence). | must   |
| 30  | `report`     | The robustness report card | Arena extended with probe sweeps and Monte Carlo campaigns for every L1 and L3 controller.                            | GM, PM, delay margin, $\|S\|_\infty$ and pass rate next to Part II's RMS columns. Controllers that won on RMS are not always the robust ones.                              | must   |

Goals use the existing `goal.check` mechanism. Lessons that predict a
number (III.4, III.7, III.8) compare the student's input field with the
value measured headlessly on the same seed.

## 7. Risks and open questions

| Risk                                                                                                                     | Mitigation                                                                                                                                                                                            |
| ------------------------------------------------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Sweeps and campaigns are slow (dozens of runs per Bode plot, 300 per campaign).                                          | Multisine excitation (many frequencies per run); time-sliced chunks; cache per parameter hash. Web Worker if needed. Campaigns use short scenarios (10 s) and show partial results as they arrive.    |
| The three-rotor spin is too fast to control: about 78 rad/s with the default `angularDamping` of 0.002 (§4).             | Prototype headlessly first. If needed, add a separate yaw-damping parameter for rotor drag in yaw, and have the lesson set it. If the prototype fails, III.27 is dropped; nothing else depends on it. |
| The gyroscopic coupling of III.12 is too weak with our inertia values to make loop-at-a-time margins visibly misleading. | Compute the disk margin of the coupled pair headlessly before writing the lesson. Fallback: a rotated IMU (mounting error) as the coupling, which gives the same lesson.                              |
| The MRAC drift of III.17 needs a careful choice of adaptation gain and excitation to appear within a lesson's 60 s.      | Tune the scenario headlessly; pin it with a test that asserts divergence without σ-modification and convergence with it.                                                                              |
| Low-frequency and near-Nyquist points of a measured Bode plot are unreliable.                                            | Agreement is tested only in a stated band (M13: 0.1–20 Hz); the chart greys out points outside it.                                                                                                    |
| Model and measurement disagree for reasons that are not pedagogical (e.g. drag nonlinearity, saturation during a sweep). | Small probe amplitudes, calm air during sweeps, a tolerance band drawn on the Bode chart; tests pin agreement for the default loops.                                                                  |
| `linearModel()` for every controller is a lot of work.                                                                   | Analytic models only for L1 (PID, LQR/LQI, ADRC, SMC) and the L3 rate loop; everything else is measured-only, and the report card is measured-only.                                                   |
| Scope: 30 lessons, 6 milestones.                                                                                         | The **must** set (13 lessons: III.1–4, 7, 9, 11, 12, 20–22, 29, 30) is a coherent Part III on its own and is built first (§5).                                                                        |

Decisions (confirmed 2026-09-30):

1. **Numbering** — Part III as a third semester, restarting at III.1.
2. **H∞** — promoted from _could_ to _should_ (lesson III.13) when the
   goal of the programme became full coverage; μ analysis is a _could_.
3. **Default attitude source** — stays `truth`; estimation is opt-in per
   lesson, so every earlier tune and test stays valid.
4. **Scope** — M13 and the **must** lessons first, the rest in a second
   pass (§5, build order).
5. **The book** — Part III is built in the app first. Its chapters belong
   to the second volume of the book
   ([curriculum.md](curriculum.md) §5).

## 8. References

- [Åström 2021] K. J. Åström, R. M. Murray, _Feedback Systems: An Introduction for Scientists and Engineers_, 2nd ed., Princeton University Press, 2021.
- [Åström 1984] K. J. Åström, T. Hägglund, "Automatic tuning of simple regulators with specifications on phase and amplitude margins", _Automatica_ 20(5), 1984.
- [Skogestad 2005] S. Skogestad, I. Postlethwaite, _Multivariable Feedback Control: Analysis and Design_, 2nd ed., Wiley, 2005.
- [Seiler 2020] P. Seiler, A. Packard, P. Gahinet, "An introduction to disk margins", _IEEE Control Systems Magazine_ 40(5), 2020.
- [Stein 2003] G. Stein, "Respect the unstable", _IEEE Control Systems Magazine_ 23(4), 2003.
- [Franklin 1998] G. F. Franklin, J. D. Powell, M. Workman, _Digital Control of Dynamic Systems_, 3rd ed., Addison-Wesley, 1998.
- [Khalil 2002] H. K. Khalil, _Nonlinear Systems_, 3rd ed., Prentice Hall, 2002.
- [Slotine 1991] J.-J. E. Slotine, W. Li, _Applied Nonlinear Control_, Prentice Hall, 1991.
- [Krstić 1995] M. Krstić, I. Kanellakopoulos, P. Kokotović, _Nonlinear and Adaptive Control Design_, Wiley, 1995.
- [Ioannou 1996] P. A. Ioannou, J. Sun, _Robust Adaptive Control_, Prentice Hall, 1996.
- [Rohrs 1985] C. E. Rohrs, L. Valavani, M. Athans, G. Stein, "Robustness of continuous-time adaptive control algorithms in the presence of unmodeled dynamics", _IEEE Trans. Automatic Control_ 30(9), 1985.
- [Doyle 1978] J. C. Doyle, "Guaranteed margins for LQG regulators", _IEEE Trans. Automatic Control_ 23(4), 1978.
- [Doyle 1981] J. C. Doyle, G. Stein, "Multivariable feedback design: concepts for a classical/modern synthesis", _IEEE Trans. Automatic Control_ 26(1), 1981.
- [Glover 1989] K. Glover, D. McFarlane, "Robust stabilization of normalized coprime factor plant descriptions with H∞-bounded uncertainty", _IEEE Trans. Automatic Control_ 34(8), 1989.
- [Mahony 2008] R. Mahony, T. Hamel, J.-M. Pflimlin, "Nonlinear complementary filters on the special orthogonal group", _IEEE Trans. Automatic Control_ 53(5), 2008.
- [Markley 2003] F. L. Markley, "Attitude error representations for Kalman filtering", _Journal of Guidance, Control, and Dynamics_ 26(2), 2003.
- [Ljung 1999] L. Ljung, _System Identification: Theory for the User_, 2nd ed., Prentice Hall, 1999.
- [Tischler 2012] M. B. Tischler, R. K. Remple, _Aircraft and Rotorcraft System Identification_, 2nd ed., AIAA, 2012.
- [Basseville 1993] M. Basseville, I. V. Nikiforov, _Detection of Abrupt Changes: Theory and Application_, Prentice Hall, 1993.
- [Hanson 2010] J. M. Hanson, B. B. Beard, _Applying Monte Carlo Simulation to Launch Vehicle Design and Requirements Analysis_, NASA/TP-2010-216447, 2010.
- [Mueller 2014] M. W. Mueller, R. D'Andrea, "Stability and control of a quadrocopter despite the complete loss of one, two, or three propellers", _ICRA_ 2014.
- [Sun 2021] S. Sun, X. Wang, Q. Chu, C. de Visser, "Incremental nonlinear fault-tolerant control of a quadrotor with complete loss of two opposing rotors", _IEEE Trans. Robotics_ 37(1), 2021.
- [Betaflight] Betaflight documentation, _RPM filtering_ and _dynamic notch filter_, betaflight.com.
- [PX4] PX4 Autopilot, _Filter/Control Latency Tuning_, docs.px4.io.
