# Roadmap

> **Status (2026-09-30):** M0–M12 are implemented: Part I (PID) and Part II (beyond PID) are complete. What changed against the
> original plan is noted under each milestone. M13 (the foundations of Part III) is implemented; M14–M26 are planned.

Built in small milestones; each ends with something runnable and useful.
The order is chosen so that the **educational core (L1 altitude loop with
charts) is usable as early as possible**, and fidelity grows afterwards.

## M0 — Project skeleton ✅

- Vite + React + TypeScript (strict) + ESLint/Prettier + Vitest.
- Tailwind CSS + shadcn/ui set up; Zustand; R3F + drei installed.
- Folder layout from [software-architecture.md](software-architecture.md#2-module-layout).
- `src/math`: vec3, quaternion, seeded PRNG — with tests.
- `Makefile` as the single entry point: `make install`, `make dev`, `make test`,
  `make check` (lint + typecheck + test), `make build`, `make preview`, `make clean`.

**Done when:** `make dev` shows the empty app layout (viewport + panels) and
`make check` passes.

## M1 — Scene and drone model ✅

- R3F `<Canvas>`: camera with drei orbit controls, lights, shadows,
  environment lighting, sky gradient, fog.
- Shader grid floor with 0.1/1/5 m hierarchy, axes, labels.
- Procedural drone model with spinning props (driven by a dummy thrust
  slider), LEDs, motion-blur discs.
- Altitude aids: drop line, height label, shadow.

**Done when:** a good-looking drone can be positioned by hand above the grid
and its height is easy to read from any camera angle.

## M2 — L1 physics + PID + charts (the core) ✅

- `sim`: L1 dynamics, motor lag, gravity, vertical drag, ground contact.
- `engine` + `<SimDriver>`: fixed-step loop with accumulator in `useFrame`,
  time scale, pause/step; scene updated via refs, not React state.
- `control/pid.ts` with all features from the primer (term toggles, D on
  measurement, D filter, anti-windup modes, output limits) — heavily tested.
- L1 altitude controller with feedforward and $\hat m$.
- Telemetry ring buffers.
- Charts: Tracking, Error, PID terms, Actuators (uPlot, synced cursor).
- Parameter panel generated from schema (controller, setpoint, physics,
  simulation groups).
- Live formula widget and term arrows on the drone.
- Tests: free fall, hover equilibrium, P-only droop $= mg/K_p$, closed loop
  converges with I.

**Done when:** you can take off, change the altitude setpoint and tune
$K_p, K_i, K_d$ while watching each term on the chart.

## M3 — Wind and disturbances ✅

- Wind model: mean, Ornstein–Uhlenbeck turbulence, 1−cos gusts, seeded.
- Quadratic drag with relative air velocity (vertical in L1).
- Wind particles + HUD indicator; Disturbance chart with gust markers.
- "Gust now", poke, payload change.
- Sensor model: noise, bias, delay.

**Done when:** the drone visibly gets knocked by gusts and recovers, and
the same seed reproduces the same wind exactly (tested).

## M4 — Learning tools ✅

- Step-response analyser with on-chart markers and metrics table.
- Theory readouts ($\omega_n$, $\zeta$, predicted $e_{ss}$).
- Snapshot/ghost trace comparison, CSV export.
- Setpoint profiles (square, ramp, sine) and dragging the setpoint ghost.
- URL-shareable state. Lessons double as presets (no separate preset library).
- Lesson runner + lessons 1–11 (written as TSX with KaTeX instead of MDX:
  fewer build plugins, type-checked content).
- Tooltips with explanations for every parameter.

**Done when:** a newcomer can go through the lessons and understand PID
without any other material.

## M5 — L2: 3D point mass ✅

- L2 dynamics, three PIDs, horizontal wind.
- Loop inspector (select `pos.x`, `pos.y`, `pos.z`).
- Cosmetic tilt, force-vector arrows, camera modes (follow, top, side).

**Done when:** the drone holds a 3D point in wind from any direction.

## M6 — L3: quadrotor with cascaded control ✅

- 6-DoF rigid-body dynamics, quaternion integration, X-frame torques.
- Mixer with desaturation; motor saturation glow.
- Cascade: position P → velocity PID → thrust vector → attitude P → rate PID.
- Per-loop rates; inspector for all loops; attitude chart.
- Lessons 12–14.
- Tests: torque signs, mixer round-trip, hover stability, step convergence.

**Done when:** the full quadrotor holds position in gusty wind with a
default tune, and each loop can be inspected and retuned live.

## Part II — Beyond PID (planned)

Contemporary controllers (LQR, Kalman, ADRC, INDI, geometric + flatness,
MPC, MPPI, CBF, L1 adaptive, learned residuals and policies) and a second
semester of lessons (II.1–II.21). Full research, design and lesson plan:
[beyond-pid.md](beyond-pid.md).

| Milestone | Content                                                                                    |
| --------- | ------------------------------------------------------------------------------------------ |
| M7 ✅     | Foundations: generic loop terms, controller selection, new sensors, linear algebra, charts |
| M8 ✅     | L1: LQR/LQI, Kalman filter, ADRC — lessons II.1–II.6                                       |
| M9 ✅     | L3 stage pipeline, INDI — lessons II.7–II.9                                                |
| M10 ✅    | Geometric control, flatness feedforward, trajectories — lessons II.10–II.12                |
| M11 ✅    | QP, MPC (L1 + L3 outer), MPPI, CBF safety filter, keep-out zones — lessons II.13–II.17     |
| M12 ✅    | L1 adaptive, learned residual, learned policy, arena + `make bench` — lessons II.18–II.21  |

## Part III — Why it works (planned)

Analysis rather than design: frequency response, poles and root locus,
margins, delay and the waterbed effect, Lyapunov and regions of attraction,
describing functions, observability, LQG, uncertainty and the small-gain
theorem, multivariable margins, H∞, sliding mode, backstepping and MRAC,
sampling, quantisation, aliasing and notch filters, attitude and navigation
filters, system identification, fault detection, flight after losing a
rotor, and Monte Carlo verification. Every analytic prediction is drawn
next to the same quantity measured on the simulator. Third semester of
lessons (III.1–III.30). Gap audit, design and lesson plan:
[analysis.md](analysis.md).

| Milestone | Content                                                                                                                 |
| --------- | ----------------------------------------------------------------------------------------------------------------------- |
| M13 ✅    | Foundations: LTI toolbox, linear models, probe points and sweeps, Bode / Nyquist / pole–zero charts                     |
| M14 ◐     | Chapter F: poles, margins, delay, sensitivity, loop shaping — lessons III.1–III.6                                       |
| M15       | Chapter G: Lyapunov, describing functions, observability, LQG — lessons III.7–III.10                                    |
| M16       | Chapter H: uncertainty, small gain, MIMO margins, H∞, μ, sliding mode, backstepping, MRAC — lessons III.11–III.17       |
| M17       | Chapter I: sampling, quantisation, vibration, notch filters, attitude filters, MEKF, EKF, sysid — lessons III.18–III.26 |
| M18       | Chapter J: rotor loss, fault detection, Monte Carlo verification, robustness report card — lessons III.27–III.30        |

Build order: M13, then the thirteen **must** lessons across M14–M18
(III.1–4, 7, 9, 11, 12, 20–22, 29, 30). The remaining lessons follow in a
second pass.

## Parts IV and V — Aerospace GNC and engineering practice (planned)

Three more vehicles in the same simulator (a rocket, an aircraft, a
satellite) and the theory the quadrotor cannot motivate: optimal control
from first principles, guidance, launch-vehicle control with bending and
slosh, flight dynamics and gain scheduling, spacecraft attitude control,
advanced estimation. Part V covers requirements, verification campaigns,
controller software, redundancy and mode logic. Lessons IV.1–IV.29 and
V.1–V.6. Plan: [aerospace-gnc.md](aerospace-gnc.md).

| Milestone | Content                                                                                           |
| --------- | ------------------------------------------------------------------------------------------------- |
| M19       | Vehicle abstraction; quadrotor moved behind it with an unchanged baseline; `trim` and `linearize` |
| M20       | Chapter K on L1 and the 1D rocket: Pontryagin, bang-bang, dynamic programming — IV.1–IV.4         |
| M21       | Cone and nonlinear programs, 3D rocket, powered-descent guidance, collocation — IV.5–IV.6         |
| M22       | Chapter L: planar rocket, thrust-vector control, bending, slosh — IV.7–IV.10                      |
| M23       | Chapter M: aircraft, modes, augmentation, gain scheduling, PIO, energy control — IV.11–IV.17      |
| M24       | Chapter N: satellite, reaction wheels, thrusters, flexible panel — IV.18–IV.23                    |
| M25       | Chapter O: guidance laws, relative motion, INS, UKF, particle filter, smoother — IV.24–IV.29      |
| M26       | Part V: budgets, campaigns, fixed-point controller, voter, mode logic, design review — V.1–V.6    |

The whole course, its competence map and the order of work across the app
and the book: [curriculum.md](curriculum.md).

## Later / ideas

- Ziegler–Nichols helper (dropped from M4 — the method does not fit a
  double-integrator plant; see pid-primer.md §6.2). Could return for the
  rate loop, or with a relay auto-tuner (now planned: lesson III.25).
- Physics in a Web Worker if fast-forward on slow machines needs it.

- Iterative learning control on repeated figure-8 laps (see analysis.md §1).
- Side-by-side twin drones with different tunings on the same wind.

## Open decisions

| Topic             | Current choice        | Alternative                                         |
| ----------------- | --------------------- | --------------------------------------------------- |
| UI framework      | React + R3F (decided) | —                                                   |
| Physics in worker | Main thread           | Web Worker if L3 + fast-forward gets heavy.         |
| Chart library     | uPlot                 | Custom canvas if we need very specific annotations. |
