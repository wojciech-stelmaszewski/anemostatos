# Roadmap

Built in small milestones; each ends with something runnable and useful.
The order is chosen so that the **educational core (L1 altitude loop with
charts) is usable as early as possible**, and fidelity grows afterwards.

## M0 — Project skeleton

- Vite + React + TypeScript (strict) + ESLint/Prettier + Vitest.
- Tailwind CSS + shadcn/ui set up; Zustand; R3F + drei installed.
- Folder layout from [software-architecture.md](software-architecture.md#2-module-layout).
- `src/math`: vec3, quaternion, seeded PRNG — with tests.
- `Makefile` as the single entry point: `make install`, `make dev`, `make test`,
  `make check` (lint + typecheck + test), `make build`, `make preview`, `make clean`.

**Done when:** `make dev` shows the empty app layout (viewport + panels) and
`make check` passes.

## M1 — Scene and drone model

- R3F `<Canvas>`: camera with drei orbit controls, lights, shadows,
  environment lighting, sky gradient, fog.
- Shader grid floor with 0.1/1/5 m hierarchy, axes, labels.
- Procedural drone model with spinning props (driven by a dummy thrust
  slider), LEDs, motion-blur discs.
- Altitude aids: drop line, height label, shadow.

**Done when:** a good-looking drone can be positioned by hand above the grid
and its height is easy to read from any camera angle.

## M2 — L1 physics + PID + charts (the core)

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

## M3 — Wind and disturbances

- Wind model: mean, Ornstein–Uhlenbeck turbulence, 1−cos gusts, seeded.
- Quadratic drag with relative air velocity (vertical in L1).
- Wind particles + HUD indicator; Disturbance chart with gust markers.
- "Gust now", poke, payload change.
- Sensor model: noise, bias, delay.

**Done when:** the drone visibly gets knocked by gusts and recovers, and
the same seed reproduces the same wind exactly (tested).

## M4 — Learning tools

- Step-response analyser with on-chart markers and metrics table.
- Theory readouts ($\omega_n$, $\zeta$, predicted $e_{ss}$).
- Snapshot/ghost trace comparison, CSV export.
- Setpoint profiles (square, ramp, sine) and dragging the setpoint ghost.
- Presets + URL-shareable state.
- MDX lesson runner + lessons 1–11.
- Tooltips with explanations for every parameter.

**Done when:** a newcomer can go through the lessons and understand PID
without any other material.

## M5 — L2: 3D point mass

- L2 dynamics, three PIDs, horizontal wind.
- Loop inspector (select `pos.x`, `pos.y`, `pos.z`).
- Cosmetic tilt, force-vector arrows, camera modes (follow, top, side).

**Done when:** the drone holds a 3D point in wind from any direction.

## M6 — L3: quadrotor with cascaded control

- 6-DoF rigid-body dynamics, quaternion integration, X-frame torques.
- Mixer with desaturation; motor saturation glow.
- Cascade: position P → velocity PID → thrust vector → attitude P → rate PID.
- Per-loop rates; inspector for all loops; attitude chart.
- Lessons 12–14.
- Tests: torque signs, mixer round-trip, hover stability, step convergence.

**Done when:** the full quadrotor holds position in gusty wind with a
default tune, and each loop can be inspected and retuned live.

## Later / ideas

- Frequency response: sine sweep → Bode plot of the closed loop.
- Root-locus / pole view for the L1 linearised model.
- Auto-tuner (relay method) as a lesson.
- Side-by-side twin drones with different tunings on the same wind.
- Other controllers for comparison (LQR), once PID is well understood.

## Open decisions

| Topic | Current choice | Alternative |
| --- | --- | --- |
| UI framework | React + R3F (decided) | — |
| Physics in worker | Main thread | Web Worker if L3 + fast-forward gets heavy. |
| Chart library | uPlot | Custom canvas if we need very specific annotations. |
