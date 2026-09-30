// The L1 plant as the simulator computes it (docs/analysis.md §3.2): a discrete model at the
// physics step, exact for small motions around hover, where the quadratic drag has no linear part.
import { PHYS_DT } from '@/engine/simulation';
import { ss, type Lti } from '@/math/lti';
import type { Params } from '@/sim/params';

/** Rows of the plant's output: altitude, vertical velocity, delivered thrust. */
export const PLANT_OUT = { y: 0, v: 1, thrust: 2 } as const;

/**
 * Input: the total thrust command, N (deviation from hover). Outputs: see PLANT_OUT.
 *
 * One physics step, in the simulator's order (src/sim/dynamics.ts): the rotors move towards the
 * command, the new thrust accelerates the mass, the new velocity moves it.
 *   r⁺ = (1 − λ)·r + λ·u         λ = 1 − e^{−dt/τ}
 *   v⁺ = v + (dt/m)·η·r⁺         η = motor efficiency
 *   y⁺ = y + dt·v⁺
 */
export function l1Plant(p: Params, dt = PHYS_DT): Lti {
  const m = p.drone.mass;
  const lam = 1 - Math.exp(-dt / Math.max(p.drone.motorTau, 1e-4));
  const eta = p.drone.motorEfficiency.reduce((s, v) => s + v, 0) / 4;
  const g = (dt / m) * eta;
  return ss(
    [
      [1, dt, dt * g * (1 - lam)],
      [0, 1, g * (1 - lam)],
      [0, 0, 1 - lam],
    ],
    [[dt * g * lam], [g * lam], [lam]],
    [
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, eta],
    ],
    undefined,
    dt,
  );
}

/** The textbook continuous model, for comparison: 1/(m·s²·(τ·s + 1)) from command to altitude. */
export function l1PlantContinuous(p: Params): Lti {
  const m = p.drone.mass;
  const tau = Math.max(p.drone.motorTau, 1e-4);
  return ss(
    [
      [0, 1, 0],
      [0, 0, 1 / m],
      [0, 0, -1 / tau],
    ],
    [[0], [0], [1 / tau]],
    [
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1],
    ],
  );
}
