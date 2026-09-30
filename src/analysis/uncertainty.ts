// A family of plants and the small-gain test for it (docs/analysis.md §4, Chapter H).
//
// The real loop is one member of a family: L_Δ = L·(1 + Δ) with |Δ(jω)| ≤ |W(jω)|. If the nominal
// loop is stable, every member is stable when |W(jω)·T(jω)| < 1 at all frequencies, where
// T = L/(1 + L). The test is sufficient, not necessary: it treats the three real parameters as
// one unstructured perturbation.
import { cabs, cdiv, csub, C_ONE } from '@/math/complex';
import { logspace } from '@/math/margins';
import type { Params } from '@/sim/params';
import { l1Loop, loopGain, sensitivities } from './loop';

/**
 * The member of the family at (a, b, c): a ∈ [−1, 1] scales the mass error, b ∈ [−1, 1] the motor
 * time constant (logarithmically), c ∈ [0, 1] the extra delay. (0, 0, 0) is the nominal plant.
 */
export function familyMember(p: Params, a: number, b: number, c: number): Params {
  const u = p.uncertainty;
  const q = structuredClone(p);
  q.drone.mass = p.drone.mass * (1 + (a * u.massPct) / 100);
  q.drone.motorTau = p.drone.motorTau * Math.max(u.tauFactor, 1) ** b;
  q.sensors.delayMs = p.sensors.delayMs + c * u.delayMs;
  return q;
}

/** A grid over the family, corners and edges included: 5 × 5 × 3 members. */
export function familyGrid(p: Params): Params[] {
  const out: Params[] = [];
  for (const a of [-1, -0.5, 0, 0.5, 1])
    for (const b of [-1, -0.5, 0, 0.5, 1])
      for (const c of [0, 0.5, 1]) out.push(familyMember(p, a, b, c));
  return out;
}

export interface RobustTest {
  fHz: number[];
  /** |W|: the largest relative deviation of any member's loop from the nominal one. */
  w: number[];
  /** |T| of the nominal loop. */
  t: number[];
  /** The largest |W·T| and where it occurs. Below 1: every member is guaranteed stable. */
  worst: number;
  fWorst: number;
}

/** The small-gain test for the L1 loop of `p` against its family. Null without a linear model. */
export function robustTest(p: Params, points = 240): RobustTest | null {
  const nominal = l1Loop(p);
  if (!nominal) return null;
  const members = familyGrid(p).map((q) => l1Loop(q)!);
  const fHz = logspace(0.05, Math.min(100, 0.49 / nominal.T), points);
  const w: number[] = [];
  const t: number[] = [];
  let worst = 0;
  let fWorst = NaN;
  for (const f of fHz) {
    const om = 2 * Math.PI * f;
    const l0 = loopGain(nominal, om);
    let dev = 0;
    for (const m of members) dev = Math.max(dev, cabs(csub(cdiv(loopGain(m, om), l0), C_ONE)));
    const tt = cabs(sensitivities(nominal, om).t);
    w.push(dev);
    t.push(tt);
    if (dev * tt > worst) {
      worst = dev * tt;
      fWorst = f;
    }
  }
  return { fHz, w, t, worst, fWorst };
}
