import { ConeBuilder, solveCone } from '@/math/socp';
import { length, v3, type Vec3 } from '@/math/vec3';
import { GRAVITY } from '@/sim/params';
import { G0 } from '@/sim/vehicles/rocket';
import type { LanderInput, LanderParams, LanderState } from '@/sim/vehicles/lander';

/**
 * Powered-descent guidance (docs/aerospace-gnc.md §4, Chapter K; Açıkmeşe & Ploen 2007,
 * Blackmore et al. 2010): the fuel-optimal landing as a second-order cone program.
 *
 * The thrust bounds ρ₁ ≤ ‖T‖ ≤ ρ₂ are not convex (the lower one cuts a hole in the ball). With
 * u = T/m, a slack σ ≥ ‖u‖ and z = ln m the problem becomes
 *   minimise Σσₖ   s.t.   ‖uₖ‖ ≤ σₖ,   ρ₁e^(−z₀ₖ)·(1 − δ + δ²/2) ≤ σₖ ≤ ρ₂e^(−z₀ₖ)·(1 − δ),
 *   δ = zₖ − z₀ₖ,   ż = −σ/(Isp·g₀),   r̈ = u + g,   glide slope,   r(t_f) = 0,   v(t_f) = 0
 * which is convex, and its optimum has ‖uₖ‖ = σₖ: the relaxation is lossless. z₀ₖ is the log of
 * the least mass possible at tₖ, around which the exponential is expanded.
 *
 * The thrust is held constant over each of N intervals, so the discrete dynamics are exact; the
 * state is eliminated (positions and velocities are linear in the u's), leaving 4N variables.
 * The flight time is not convex: it is searched over, one cone program per candidate.
 */
export interface PdgSettings {
  /** Intervals of the plan. */
  nodes: number;
  /** Fraction of the maximum thrust the plan may use (the rest is margin for re-planning). */
  throttle: number;
  /** Re-plan every this many seconds (0: plan once, then fly it open loop). */
  replanEvery: number;
  /** Stop re-planning this close to touchdown, s. */
  freezeBefore: number;
  /** The plan ends at this gate above the pad, descending at `gateSpeed`; a terminal law lands. */
  gateAltitude: number;
  gateSpeed: number;
  /** Propellant the plan leaves for the terminal descent, kg. */
  reserve: number;
}

export type PdgStatus = 'optimal' | 'infeasible';

export interface PdgPlan {
  status: PdgStatus;
  /** Flight time from the plan's start, s, and the interval. */
  tf: number;
  dt: number;
  /** Acceleration from the engine (thrust/mass) per interval, m/s², and its slack σ. */
  u: Vec3[];
  sigma: number[];
  /** Positions, velocities and masses at the N + 1 nodes. */
  r: Vec3[];
  v: Vec3[];
  m: number[];
  /** Propellant the plan burns, kg. */
  fuel: number;
  /** Largest (σ − ‖u‖)/σ over the plan: zero when the relaxation is lossless. */
  slackGap: number;
  /** Cone programs solved to make it. */
  solves: number;
  /** Newton steps of the interior-point method, over all its solves. */
  newton: number;
}

interface Start {
  r: Vec3;
  v: Vec3;
  m: number;
}

const G = v3(0, -GRAVITY, 0);

/** One cone program for a fixed flight time; null if it has no solution. */
export function solvePdg(s: Start, tf: number, l: LanderParams, set: PdgSettings): PdgPlan | null {
  const N = set.nodes;
  const dt = tf / N;
  const n = 4 * N;
  const alpha = 1 / (l.isp * G0);
  const rho1 = l.thrustMin;
  const rho2 = l.thrustMax * set.throttle;
  const z0 = Math.log(s.m);
  const iu = (k: number, c: number) => 3 * k + c;
  const is = (k: number) => 3 * N + k;
  const b = new ConeBuilder(n);
  const ax = (v: Vec3, c: number) => (c === 0 ? v.x : c === 1 ? v.y : v.z);

  // Affine expressions (coefficients, constant) of the eliminated states.
  type Aff = { c: number[]; k: number };
  const pos = (k: number, c: number): Aff => {
    const row = b.row();
    let k0 = ax(s.r, c) + ax(s.v, c) * k * dt;
    for (let j = 0; j < k; j++) {
      const w = dt * dt * (k - j - 0.5);
      row[iu(j, c)] = w;
      k0 += ax(G, c) * w;
    }
    return { c: row, k: k0 };
  };
  const vel = (k: number, c: number): Aff => {
    const row = b.row();
    let k0 = ax(s.v, c);
    for (let j = 0; j < k; j++) {
      row[iu(j, c)] = dt;
      k0 += ax(G, c) * dt;
    }
    return { c: row, k: k0 };
  };
  const logMass = (k: number): Aff => {
    const row = b.row();
    for (let j = 0; j < k; j++) row[is(j)] = -alpha * dt;
    return { c: row, k: z0 };
  };
  // A cone row from an affine expression e (its value is h − g·x = k + c·x).
  const cr = (e: Aff) => ({ g: e.c.map((v) => -v), h: e.k });
  const lin = (coef: number[], k: number): Aff => ({ c: coef, k });

  for (let k = 0; k < N; k++) {
    const tk = k * dt;
    // ‖uₖ‖ ≤ σₖ.
    const sig = b.row();
    sig[is(k)] = 1;
    const comp = (c: number) => {
      const r = b.row();
      r[iu(k, c)] = 1;
      return lin(r, 0);
    };
    b.cone([cr(lin(sig, 0)), cr(comp(0)), cr(comp(1)), cr(comp(2))]);

    const mLeast = s.m - alpha * rho2 * tk;
    if (mLeast <= 0) return null;
    const zl = Math.log(mLeast);
    const z = logMass(k);
    const delta: Aff = { c: z.c, k: z.k - zl };
    // Upper bound: σₖ ≤ ρ₂e^(−z₀ₖ)(1 − δ).
    const cu = rho2 * Math.exp(-zl);
    b.le(
      sig.map((v, i) => v + cu * delta.c[i]!),
      cu * (1 - delta.k),
    );
    // Lower bound: c·δ²/2 ≤ w := σₖ − c(1 − δ), as the cone ‖(2δ, Y − 1)‖ ≤ Y + 1, Y = 2w/c.
    const cl = rho1 * Math.exp(-zl);
    const w: Aff = { c: sig.map((v, i) => v + cl * delta.c[i]!), k: -cl * (1 - delta.k) };
    const y: Aff = { c: w.c.map((v) => (2 * v) / cl), k: (2 * w.k) / cl };
    b.cone([
      cr({ c: y.c, k: y.k + 1 }),
      cr({ c: delta.c.map((v) => 2 * v), k: 2 * delta.k }),
      cr({ c: y.c, k: y.k - 1 }),
    ]);
    if (k > 0) {
      // The mass stays between burning at the most and at the least: δ ≥ 0 and z ≤ ln(m₀ − αρ₁t).
      b.le(
        delta.c.map((v) => -v),
        delta.k,
      );
      b.le(z.c, Math.log(s.m - alpha * rho1 * tk) - z.k);
      // Glide slope: ‖(x, z)‖·tan γ ≤ y.
      const cot = 1 / Math.tan((l.glideSlopeDeg * Math.PI) / 180);
      const py = pos(k, 1);
      b.cone([cr({ c: py.c.map((v) => v * cot), k: py.k * cot }), cr(pos(k, 0)), cr(pos(k, 2))]);
    }
  }
  // Enough propellant: z_N ≥ ln(dry mass).
  const zN = logMass(N);
  b.le(
    zN.c.map((v) => -v),
    zN.k - Math.log(l.dryMass + set.reserve),
  );
  // Arrive at the gate above the pad, descending slowly.
  const gate = [v3(0, set.gateAltitude, 0), v3(0, -set.gateSpeed, 0)] as const;
  for (let c = 0; c < 3; c++) {
    const p = pos(N, c);
    b.equal(p.c, ax(gate[0], c) - p.k);
    const q = vel(N, c);
    b.equal(q.c, ax(gate[1], c) - q.k);
  }
  const cost = b.row();
  for (let k = 0; k < N; k++) cost[is(k)] = dt;
  const res = solveCone(b.build(cost), { tol: 1e-7, mu: 50 });
  if (res.status === 'infeasible') return null;
  const x = res.x;
  const u = Array.from({ length: N }, (_, k) => v3(x[iu(k, 0)]!, x[iu(k, 1)]!, x[iu(k, 2)]!));
  const sigma = Array.from({ length: N }, (_, k) => x[is(k)]!);
  const val = (e: Aff) => e.k + e.c.reduce((acc, v, i) => acc + v * x[i]!, 0);
  const r = Array.from({ length: N + 1 }, (_, k) =>
    v3(val(pos(k, 0)), val(pos(k, 1)), val(pos(k, 2))),
  );
  const v = Array.from({ length: N + 1 }, (_, k) =>
    v3(val(vel(k, 0)), val(vel(k, 1)), val(vel(k, 2))),
  );
  const m = Array.from({ length: N + 1 }, (_, k) => Math.exp(val(logMass(k))));
  let slackGap = 0;
  for (let k = 0; k < N; k++)
    slackGap = Math.max(slackGap, (sigma[k]! - length(u[k]!)) / sigma[k]!);
  return {
    status: 'optimal',
    tf,
    dt,
    u,
    sigma,
    r,
    v,
    m,
    fuel: s.m - m[N]!,
    slackGap,
    solves: 1,
    newton: res.iterations,
  };
}

/**
 * The fuel-optimal landing: a coarse scan of flight times for the feasible ones, then a
 * golden-section search on the fuel around the best of them (the fuel is unimodal in t_f).
 * The engine never stops, so no flight lasts longer than the propellant burns at the least
 * thrust. Infeasible if no time works.
 */
export function planLanding(s: Start, l: LanderParams, set: PdgSettings, tMin = 1): PdgPlan {
  const fuelLeft = s.m - l.dryMass - set.reserve;
  const tMax = Math.max(tMin + 1, fuelLeft / (l.thrustMin / (l.isp * G0)));
  let solves = 0;
  let newton = 0;
  let best: PdgPlan | null = null;
  const at = (tf: number): number => {
    solves++;
    const p = solvePdg(s, tf, l, set);
    if (!p) return Infinity;
    newton += p.newton;
    if (!best || p.fuel < best.fuel) best = p;
    return p.fuel;
  };
  const scan = 6;
  const times = Array.from({ length: scan + 1 }, (_, i) => tMin + ((tMax - tMin) * i) / scan);
  const fuels = times.map(at);
  let bi = 0;
  fuels.forEach((f, i) => {
    if (f < fuels[bi]!) bi = i;
  });
  if (!Number.isFinite(fuels[bi]!)) return infeasible(solves);
  let lo = times[Math.max(bi - 1, 0)]!;
  let hi = times[Math.min(bi + 1, scan)]!;
  const gr = (Math.sqrt(5) - 1) / 2;
  let a = hi - gr * (hi - lo);
  let c = lo + gr * (hi - lo);
  let fa = at(a);
  let fc = at(c);
  for (let i = 0; i < 5; i++) {
    if (fa < fc) {
      hi = c;
      c = a;
      fc = fa;
      a = hi - gr * (hi - lo);
      fa = at(a);
    } else {
      lo = a;
      a = c;
      fa = fc;
      c = lo + gr * (hi - lo);
      fc = at(c);
    }
  }
  return { ...(best as unknown as PdgPlan), solves, newton };
}

/**
 * Re-plan with the time left. A fuel-optimal plan ends at the planned thrust limit, so an engine
 * that delivers a little less than commanded soon makes the remaining landing infeasible within
 * that limit: then the reserve above it is released (the full thrust), and slightly different
 * flight times are tried.
 */
export function replan(s: Start, tgo: number, l: LanderParams, set: PdgSettings): PdgPlan {
  let solves = 0;
  for (const throttle of [set.throttle, 1]) {
    for (const k of [1, 0.9, 1.15]) {
      solves++;
      const p = solvePdg(s, tgo * k, l, { ...set, throttle });
      if (p) return { ...p, solves };
    }
    if (set.throttle >= 1) break;
  }
  return infeasible(solves);
}

function infeasible(solves: number): PdgPlan {
  return {
    status: 'infeasible',
    tf: NaN,
    dt: NaN,
    u: [],
    sigma: [],
    r: [],
    v: [],
    m: [],
    fuel: NaN,
    slackGap: NaN,
    solves,
    newton: 0,
  };
}

/**
 * The guidance loop: plan at the start, re-plan every `replanEvery` seconds from the measured
 * state, and between plans command thrust = mass × the planned acceleration. After the plan's end
 * (or with no plan) it settles onto the pad: hold a slow descent and cancel any drift.
 */
export class PdgGuidance {
  plan: PdgPlan | null = null;
  /** Simulation time the current plan starts at. */
  planStart = 0;
  /** The first plan, kept for the chart. */
  first: PdgPlan | null = null;
  lastReplan = 0;
  replans = 0;
  /** True once a plan reported that no landing exists. */
  infeasible = false;
  /**
   * Thrust delivered per thrust commanded, as measured in flight (an accelerometer and the known
   * mass give the delivered thrust). Plans use the engine as it is, not as the data sheet says.
   */
  efficiency = 1;
  private lastCmd = 0;

  reset(): void {
    this.efficiency = 1;
    this.lastCmd = 0;
    this.plan = null;
    this.first = null;
    this.planStart = 0;
    this.lastReplan = 0;
    this.replans = 0;
    this.infeasible = false;
  }

  tick(s: LanderState, t: number, lander: LanderParams, set: PdgSettings): LanderInput {
    const delivered = length(s.thrust);
    if (this.lastCmd > 0 && delivered > 0)
      this.efficiency += 0.05 * (delivered / this.lastCmd - this.efficiency);
    const e = this.efficiency;
    // The engine as measured: its limits scaled by the efficiency.
    const l = { ...lander, thrustMin: lander.thrustMin * e, thrustMax: lander.thrustMax * e };
    const out = this.command(s, t, l, set);
    const mag = length(out);
    this.lastCmd = mag > 0 ? Math.min(Math.max(mag / e, lander.thrustMin), lander.thrustMax) : 0;
    return { thrust: v3(out.x / e, out.y / e, out.z / e) };
  }

  /** The thrust wanted from the engine (delivered), N. */
  private command(s: LanderState, t: number, l: LanderParams, set: PdgSettings): Vec3 {
    const here = { r: s.pos, v: s.vel, m: s.mass };
    if (!this.plan) {
      this.plan = planLanding(here, l, set);
      this.first = this.plan;
      this.planStart = t;
      this.lastReplan = t;
      this.infeasible = this.plan.status === 'infeasible';
    } else if (
      set.replanEvery > 0 &&
      this.plan.status === 'optimal' &&
      t - this.lastReplan >= set.replanEvery - 1e-9
    ) {
      const tgo = this.plan.tf - (t - this.planStart);
      if (tgo > set.freezeBefore) {
        const next = replan(here, tgo, l, set);
        this.lastReplan = t;
        this.replans++;
        if (next.status === 'optimal') {
          this.plan = next;
          this.planStart = t;
        }
      }
    }
    const p = this.plan;
    if (p.status !== 'optimal') return brake(s);
    const k = Math.floor((t - this.planStart) / p.dt);
    if (k >= p.u.length) return settle(s, set);
    const u = p.u[k]!;
    return v3(u.x * s.mass, u.y * s.mass, u.z * s.mass);
  }

  /** Whether the plan is over and the terminal descent flies. */
  terminal(t: number): boolean {
    const p = this.plan;
    return !!p && p.status === 'optimal' && t - this.planStart >= p.tf;
  }
}

/** With no landing possible: brake as hard as possible against the velocity, and hold up. */
function brake(s: LanderState): Vec3 {
  const sp = length(s.vel);
  const dir = sp > 1e-6 ? v3(-s.vel.x / sp, -s.vel.y / sp, -s.vel.z / sp) : v3(0, 1, 0);
  const big = 1e9;
  return v3(dir.x * big, Math.max(dir.y, 0.5) * big, dir.z * big);
}

/**
 * The terminal descent after the gate: down at the gate speed, slowing to 0.4 m/s near the
 * ground, while a PD law centres the lander over the pad.
 */
function settle(s: LanderState, set: PdgSettings): Vec3 {
  const vDown = Math.min(set.gateSpeed, 0.4 + 0.6 * s.pos.y);
  const a = v3(
    -3 * s.vel.x - 2 * s.pos.x,
    GRAVITY + 5 * (-vDown - s.vel.y),
    -3 * s.vel.z - 2 * s.pos.z,
  );
  return v3(a.x * s.mass, a.y * s.mass, a.z * s.mass);
}
