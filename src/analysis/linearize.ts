// Linearisation of any vehicle (docs/aerospace-gnc.md §3.1): central finite differences of the
// simulator's own step around a trim point. The result is the discrete model the simulator flies.
import { PHYS_DT } from '@/engine/simulation';
import type { Params } from '@/sim/params';
import { v3 } from '@/math/vec3';
import type { Linearization, Vehicle } from '@/sim/vehicles/types';

const CALM = { wind: v3(), external: v3() };

/**
 * A, B of the step x⁺ = f(x, u) at (`state`, `input`), by central differences with steps `hx`
 * (states) and `hu` (inputs). The state is copied before every step, so `state` is not changed.
 */
export function linearize<S, U>(
  vehicle: Vehicle<S, U>,
  p: Params,
  at: { state: S; input: U },
  dt = PHYS_DT,
  hx = 1e-5,
  hu = 1e-3,
): Linearization {
  const ref = at.state;
  const u0 = vehicle.inputToVector(at.input);
  const x0 = vehicle.toVector(ref, ref);
  const f = (x: readonly number[], u: readonly number[]): number[] => {
    const s = vehicle.fromVector(x, ref);
    vehicle.step(s, vehicle.inputFromVector(u), CALM, dt, p);
    return vehicle.toVector(s, ref);
  };
  const n = x0.length;
  const m = u0.length;
  const a = Array.from({ length: n }, () => new Array<number>(n).fill(0));
  const b = Array.from({ length: n }, () => new Array<number>(m).fill(0));
  for (let j = 0; j < n; j++) {
    const xp = [...x0];
    const xm = [...x0];
    xp[j] = xp[j]! + hx;
    xm[j] = xm[j]! - hx;
    const fp = f(xp, u0);
    const fm = f(xm, u0);
    for (let i = 0; i < n; i++) a[i]![j] = (fp[i]! - fm[i]!) / (2 * hx);
  }
  for (let j = 0; j < m; j++) {
    const up = [...u0];
    const um = [...u0];
    up[j] = up[j]! + hu;
    um[j] = um[j]! - hu;
    const fp = f(x0, up);
    const fm = f(x0, um);
    for (let i = 0; i < n; i++) b[i]![j] = (fp[i]! - fm[i]!) / (2 * hu);
  }
  return { a, b, dt, stateNames: vehicle.stateNames, inputNames: vehicle.inputNames };
}
