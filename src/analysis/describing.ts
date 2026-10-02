// Describing functions (docs/analysis.md §4, Chapter G, lesson III.8): a static nonlinearity
// driven by a sine of amplitude A is replaced by its gain at the fundamental, N(A). A loop whose
// linear part G meets −1/N(A) can hold a limit cycle at that frequency and amplitude.
import { PHYS_DT } from '@/engine/simulation';
import { C_ONE, C_ZERO, cadd, cdiv, cis, cmul, cscale, csub, type Complex } from '@/math/complex';
import { evalAt, ss, type Lti } from '@/math/lti';
import type { Params } from '@/sim/params';
import { controlPeriod, CTRL_IN, l1ControllerModel } from './controllers';

/** Saturation of slope k that clips at ±limit: N = k for A ≤ limit, less above. */
export function dfSaturation(a: number, limit: number, k = 1): number {
  if (a <= limit) return k;
  const r = limit / a;
  return ((2 * k) / Math.PI) * (Math.asin(r) + r * Math.sqrt(1 - r * r));
}

/** An ideal relay of output ±h: N = 4h/(πA). */
export const dfRelay = (a: number, h: number): number => (4 * h) / (Math.PI * Math.max(a, 1e-12));

/** A dead zone of half-width d and slope k outside it: N = k − (saturation at d). */
export const dfDeadZone = (a: number, d: number, k = 1): number =>
  a <= d ? 0 : k - dfSaturation(a, d, k);

/**
 * Quadratic drag f = c·x·|x| (air drag on a velocity x): N = 8cA/(3π). A gain that grows with
 * the amplitude: small motions see no damping at all, large ones a lot.
 */
export const dfQuadratic = (a: number, c: number): number => (8 * c * a) / (3 * Math.PI);

/** Inverse of dfQuadratic: the amplitude at which quadratic drag acts like a damping n. */
export const quadraticAmplitude = (n: number, c: number): number => (3 * Math.PI * n) / (8 * c);

/**
 * The L1 plant at the physics step with a second input, an external vertical force d, acting on
 * the mass directly (not through the motors), in the simulator's order:
 *   r⁺ = (1 − λ)·r + λ·u,  v⁺ = v + (dt/m)·(η·r⁺ + d),  y⁺ = y + dt·v⁺.
 * Outputs: altitude, velocity, delivered thrust (as l1Plant).
 */
function plantWithForce(p: Params, dt = PHYS_DT): Lti {
  const m = p.drone.mass;
  const lam = 1 - Math.exp(-dt / Math.max(p.drone.motorTau, 1e-4));
  const eta = p.drone.motorEfficiency.reduce((s, v) => s + v, 0) / 4;
  const g = (dt / m) * eta;
  const h = dt / m;
  return ss(
    [
      [1, dt, dt * g * (1 - lam)],
      [0, 1, g * (1 - lam)],
      [0, 0, 1 - lam],
    ],
    [
      [dt * g * lam, dt * h],
      [g * lam, h],
      [lam, 0],
    ],
    [
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, eta],
    ],
    undefined,
    dt,
  );
}

/** Sample-and-hold of the controller output, as in src/analysis/loop.ts. */
function holdGain(w: number, steps: number): Complex {
  const om = w * PHYS_DT;
  if (Math.abs(om) < 1e-12 || steps === 1) return C_ONE;
  return cscale(cdiv(csub(C_ONE, cis(-om * steps)), csub(C_ONE, cis(-om))), 1 / steps);
}

/**
 * Closed loop (controller in place) from an external vertical force to the vertical velocity,
 * V/D at ω: the linear part that the air drag sees. Null without a linear controller model.
 */
export function forceToVelocity(p: Params): ((w: number) => Complex) | null {
  const k = l1ControllerModel(p);
  if (!k) return null;
  const plant = plantWithForce(p);
  const { steps, T } = controlPeriod(p);
  const delay = Math.max(0, Math.round(p.sensors.delayMs / 1000 / PHYS_DT));
  const meas = [
    [CTRL_IN.y, 0],
    [CTRL_IN.v, 1],
    [CTRL_IN.thrust, 2],
  ] as const;
  return (w) => {
    const pz = evalAt(plant, cis(w * PHYS_DT)); // 3 × 2
    const kz = evalAt(k, cis(w * T))[0]!;
    const hl = cmul(holdGain(w, steps), cis(-w * PHYS_DT * delay));
    let a: Complex = C_ZERO;
    let b: Complex = C_ZERO;
    for (const [ci, pi] of meas) {
      a = cadd(a, cmul(kz[ci]!, pz[pi]![0]!));
      b = cadd(b, cmul(kz[ci]!, pz[pi]![1]!));
    }
    a = cmul(hl, a);
    b = cmul(hl, b);
    // u = a·u + b·d  ⇒  u = b·d/(1 − a);  v = Pvu·u + Pvd·d.
    const u = cdiv(b, csub(C_ONE, a));
    return cadd(cmul(pz[1]![0]!, u), pz[1]![1]!);
  };
}

export interface LimitCycle {
  /** Frequency, Hz, and the amplitude of the signal into the nonlinearity. */
  fHz: number;
  amplitude: number;
  /** The describing-function gain at the crossing, N = −1/G(jω). */
  n: number;
}

/**
 * Where G(jω) crosses the negative real axis, the locus of −1/N(A) for any nonlinearity with a
 * real, positive describing function: the frequency of the predicted limit cycle and the gain
 * N = −1/G there. Scans [f0, f1] Hz on a log grid and refines by bisection on the phase.
 */
export function negativeRealCrossings(
  g: (w: number) => Complex,
  f0 = 0.05,
  f1 = 50,
  n = 800,
): { fHz: number; n: number }[] {
  const out: { fHz: number; n: number }[] = [];
  const at = (f: number) => g(2 * Math.PI * f);
  let fa = f0;
  let ga = at(fa);
  for (let i = 1; i <= n; i++) {
    const fb = f0 * (f1 / f0) ** (i / n);
    const gb = at(fb);
    // A sign change of Im with Re < 0 on both sides.
    if (ga.im * gb.im < 0 && ga.re < 0 && gb.re < 0) {
      let lo = fa;
      let hi = fb;
      let glo = ga;
      for (let k = 0; k < 60; k++) {
        const mid = Math.sqrt(lo * hi);
        const gm = at(mid);
        if (gm.im * glo.im <= 0) hi = mid;
        else {
          lo = mid;
          glo = gm;
        }
      }
      const f = Math.sqrt(lo * hi);
      out.push({ fHz: f, n: -1 / at(f).re });
    }
    fa = fb;
    ga = gb;
  }
  return out;
}

/**
 * The limit cycle of an L1 loop held by quadratic air drag (lesson III.8): the loop is unstable on
 * its own (a P controller behind a lagging motor) and the drag, whose describing function grows
 * with the velocity amplitude, stops the growth. Returns the predicted frequency and the velocity
 * and altitude amplitudes, or null if there is no crossing.
 */
export function dragLimitCycle(
  p: Params,
): { fHz: number; velocity: number; altitude: number; n: number } | null {
  const g = forceToVelocity(p);
  if (!g) return null;
  const c = p.drone.dragV;
  const crossings = negativeRealCrossings(g).filter((x) => x.n > 0);
  if (!crossings.length || c <= 0) return null;
  // The crossing of the oscillation the loop sustains: the lowest one.
  const x = crossings[0]!;
  const velocity = quadraticAmplitude(x.n, c);
  return { fHz: x.fHz, velocity, altitude: velocity / (2 * Math.PI * x.fHz), n: x.n };
}

/** The locus −1/N(A) of quadratic drag for velocity amplitudes from a0 to a1 (on the real axis). */
export const dragLocus = (c: number, a0: number, a1: number, n = 40): Complex[] =>
  Array.from({ length: n }, (_, i) => {
    const a = a0 * (a1 / a0) ** (i / (n - 1));
    return { re: -1 / dfQuadratic(a, c), im: 0 };
  });
