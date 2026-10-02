import { Rng } from '@/math/prng';
import { v3, type Vec3 } from '@/math/vec3';
import { GRAVITY, type Params } from '@/sim/params';
import type { Measurement } from '@/sim/sensors';
import { zoneDistance } from '@/sim/world';
import type { Plan } from './types';

/**
 * Model Predictive Path Integral control (Williams et al. 2017), as an L3 outer stage.
 *
 * Every re-plan: perturb the current nominal acceleration sequence with Gaussian noise K times,
 * roll each sequence out on a point-mass model, score it (tracking + effort + keep-out zones), and
 * replace the nominal sequence by the average of the perturbations weighted by exp(−cost/λ).
 * No gradients and no convexity needed — the cost may be as non-convex as a pillar in the way.
 * Seeded, so runs are reproducible.
 */
export class MppiOuter {
  private rng: Rng;
  private nominal: Vec3[] = [];
  private samples: { path: Vec3[]; cost: number }[] = [];
  private best: Vec3[] = [];
  private t0 = 0;
  private held = v3();
  /** Re-plan periods not yet shifted out of the plan, in plan steps (the period is usually
   *  shorter than a step: 20 ms against 50 ms). */
  private shiftDebt = 0;
  /** Effective number of samples that carried weight last time (1 = greedy … K = uniform). */
  effectiveSamples = 0;

  constructor(seed: number) {
    this.rng = new Rng(seed ^ 0x5bd1e995);
  }

  reset(): void {
    this.nominal = [];
    this.samples = [];
    this.best = [];
    this.held = v3();
    this.shiftDebt = 0;
  }

  /** Re-plan and return the desired acceleration (m/s², gravity excluded). */
  update(s: Measurement, preview: (tau: number) => Vec3, p: Params, t: number): Vec3 {
    const c = p.control.mppi;
    const steps = Math.max(2, Math.round(c.steps));
    const dt = c.horizon / steps;
    const K = Math.max(1, Math.round(c.samples));
    const aH = GRAVITY * Math.tan((p.control.l3.maxTiltDeg * Math.PI) / 180);
    const aUp = (4 * p.drone.maxMotorThrust) / Math.max(p.control.model.mass, 0.1) - GRAVITY;
    const limit = (a: Vec3): Vec3 => {
      const h = Math.hypot(a.x, a.z);
      const k = h > aH ? aH / h : 1;
      return v3(a.x * k, Math.min(Math.max(a.y, -0.8 * GRAVITY), aUp), a.z * k);
    };
    if (this.nominal.length !== steps) this.nominal = Array.from({ length: steps }, () => v3());
    const ref = Array.from({ length: steps }, (_, k) => preview((k + 1) * dt));
    const floor = 0.3;

    const noise: Vec3[][] = [];
    const costs: number[] = [];
    const paths: Vec3[][] = [];
    for (let i = 0; i < K; i++) {
      // Sample 0 is the unperturbed nominal sequence.
      const eps = Array.from({ length: steps }, () =>
        i === 0
          ? v3()
          : v3(
              c.sigma * this.rng.normal(),
              0.5 * c.sigma * this.rng.normal(),
              c.sigma * this.rng.normal(),
            ),
      );
      let px = s.pos.x;
      let py = s.pos.y;
      let pz = s.pos.z;
      let vx = s.vel.x;
      let vy = s.vel.y;
      let vz = s.vel.z;
      let cost = 0;
      const path: Vec3[] = [];
      for (let k = 0; k < steps; k++) {
        const n = this.nominal[k]!;
        const a = limit(v3(n.x + eps[k]!.x, n.y + eps[k]!.y, n.z + eps[k]!.z));
        vx += a.x * dt;
        vy += a.y * dt;
        vz += a.z * dt;
        px += vx * dt;
        py += vy * dt;
        pz += vz * dt;
        const r = ref[k]!;
        const e2 = (px - r.x) ** 2 + (py - r.y) ** 2 + (pz - r.z) ** 2;
        cost += c.qPos * e2 * dt + 0.01 * (a.x * a.x + a.y * a.y + a.z * a.z) * dt;
        const pk = v3(px, py, pz);
        const d = zoneDistance(pk, p.world, 3) - c.margin;
        if (d < 0) cost += c.zoneWeight * (1 + d * d * 10) * dt;
        if (py < floor) cost += c.zoneWeight * dt;
        path.push(pk);
      }
      noise.push(eps);
      costs.push(cost);
      paths.push(path);
    }

    const cMin = Math.min(...costs);
    const cMax = Math.max(...costs);
    const lambda = Math.max(c.lambda, 1e-6);
    const w = costs.map((cost) => Math.exp(-(cost - cMin) / lambda));
    const wSum = w.reduce((a, b) => a + b, 0);
    this.effectiveSamples = wSum ** 2 / w.reduce((a, b) => a + b * b, 0);
    this.nominal = this.nominal.map((n, k) => {
      let x = 0;
      let y = 0;
      let z = 0;
      for (let i = 0; i < K; i++) {
        const e = noise[i]![k]!;
        x += w[i]! * e.x;
        y += w[i]! * e.y;
        z += w[i]! * e.z;
      }
      return limit(v3(n.x + x / wSum, n.y + y / wSum, n.z + z / wSum));
    });

    // Keep a subset of the rollouts for drawing, and the resulting plan.
    const show = Math.min(K, 64);
    const stride = Math.max(1, Math.floor(K / show));
    this.samples = [];
    for (let i = 0; i < K; i += stride)
      this.samples.push({
        path: paths[i]!,
        cost: (costs[i]! - cMin) / Math.max(cMax - cMin, 1e-9),
      });
    this.best = [];
    {
      let x = s.pos.x;
      let y = s.pos.y;
      let z = s.pos.z;
      let ux = s.vel.x;
      let uy = s.vel.y;
      let uz = s.vel.z;
      for (const a of this.nominal) {
        ux += a.x * dt;
        uy += a.y * dt;
        uz += a.z * dt;
        x += ux * dt;
        y += uy * dt;
        z += uz * dt;
        this.best.push(v3(x, y, z));
      }
    }
    this.t0 = t;
    this.held = this.nominal[0]!;
    // Receding horizon: shift the plan by one re-plan period for the next warm start. The period
    // (20 ms) is usually a fraction of a plan step (50 ms), so whole steps are shifted only when
    // the periods add up to one; rounding up to a step every time ran the plan 2.5 times too fast.
    this.shiftDebt += 1 / Math.max(c.hz, 1) / dt;
    const shift = Math.min(steps, Math.floor(this.shiftDebt + 1e-9));
    this.shiftDebt -= shift;
    if (shift > 0)
      this.nominal = [
        ...this.nominal.slice(shift),
        ...Array.from({ length: shift }, () => this.nominal[steps - 1]!),
      ];
    return this.held;
  }

  get acceleration(): Vec3 {
    return this.held;
  }

  plan(): Plan {
    return { best: this.best, samples: this.samples, t0: this.t0 };
  }
}
