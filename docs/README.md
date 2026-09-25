# Anemostatos — Documentation

> _ánemos_ (wind) + _statós_ (standing): "the one that stands in the wind".

Anemostatos is an **educational, interactive simulator of a PID controller**.
A quadcopter drone hovers above an infinite reference grid in a 3D WebGL
scene. Gravity pulls it down, random wind gusts push it around, and a PID
controller (or a cascade of them) fights back to keep it at a setpoint.

The goal is not a game or a realistic flight simulator. The goal is
**understanding**: every force, every term of the controller and every
design decision (sampling rate, saturation, derivative filtering,
anti-windup…) is visible, tweakable and plotted live.

## Goals

1. Build intuition for what the **P**, **I** and **D** terms each do — and
   what goes wrong when one of them is missing or mistuned.
2. Show the practical problems of a _real_ digital PID: discrete time,
   actuator saturation, integrator windup, derivative kick, sensor noise,
   actuator lag.
3. Show how PID controllers are **composed** (cascaded loops) to control a
   system that is more complex than a single integrator.
4. Look good enough that it is a pleasure to experiment with.

## Non-goals

- Photorealistic scenery, terrain, buildings. The world is a grid floor.
- A flight simulator you pilot manually (we may add setpoint dragging, but
  the controller always flies).
- Advanced control (LQR, MPC, Kalman filters). Possible future lessons,
  out of scope for v1.

## Document index

| Document                                             | Contents                                                                                                                                           |
| ---------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| [pid-primer.md](pid-primer.md)                       | PID theory from the ground up, discrete implementation, the practical pitfalls, tuning. The "textbook" of the project.                             |
| [physics-model.md](physics-model.md)                 | Units, frames, drone dynamics, motors, gravity, drag, the wind/gust model, numerical integration.                                                  |
| [control-architecture.md](control-architecture.md)   | How the controllers are wired for each simulation level: altitude loop, 3-axis loop, full cascade (position → velocity → attitude → rate → mixer). |
| [software-architecture.md](software-architecture.md) | Tech stack, module layout, simulation loop, data flow, telemetry, testing.                                                                         |
| [ui-and-visualization.md](ui-and-visualization.md)   | 3D scene, drone model, overlays, live charts, parameter panel, lessons/scenarios.                                                                  |
| [roadmap.md](roadmap.md)                             | The implementation plan: milestones, deliverables, acceptance criteria.                                                                            |

## At a glance

- **Runtime:** browser only, run locally (`make dev`), no backend, no deployment.
- **Workflow:** a toy project — everything on `main`, no CI/CD.
- **Stack:** TypeScript, Vite, React + React Three Fiber (Three.js/WebGL),
  Zustand, shadcn/ui + Tailwind, uPlot (live charts), Vitest (tests).
- **Simulation:** fixed-step physics (default 1 kHz) decoupled from rendering,
  controller running at its own configurable rate, seeded random wind for
  reproducible experiments.
- **Language:** everything — code, docs, UI, lessons — is in English.
- **Progression:** three simulation levels of increasing fidelity —
  1D altitude → 3D point mass → full quadrotor with cascaded control.
