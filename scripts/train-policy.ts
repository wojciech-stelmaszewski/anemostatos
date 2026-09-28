// Train the neural-network policy (docs/beyond-pid.md §4, lesson II.20). Run with `make train`.
//
//   1. Imitation: fly the geometric controller in randomised conditions, record what it commands
//      (collective thrust + body rates) for what it observes, and fit the MLP to it.
//   2. Evolution strategies: improve the network on the closed loop itself — the network flies,
//      and the fitness is how close it stays to its targets.
//
// Everything is seeded; no dependencies (Node's type stripping + scripts/ts-loader.mjs).
import { writeFileSync } from 'node:fs';
import type { CascadeController } from '@/control/cascade';
import { encode, observe, ACT, OBS } from '@/control/policy';
import { setPolicyWeights } from '@/control/policy-registry';
import { Adam, network } from './mlp';
import type { Setpoint } from '@/control/types';
import { Simulation } from '@/engine/simulation';
import { Rng } from '@/math/prng';
import { v3 } from '@/math/vec3';
import { defaultParams, type L3Outer, type Params } from '@/sim/params';

const QUICK = process.argv.includes('--quick');
const HIDDEN = [32, 32];
const SIZES = [OBS, ...HIDDEN, ACT];
const log = (...a: unknown[]) => console.log(...a);

// ------------------------------------------------------------------------------------------
// Scenarios: randomised mass, wind and targets (the conditions the policy is trained for).

const MASS = [0.85, 1.15] as const;
const WIND = [0, 3] as const;

function scenario(seed: number, outer: L3Outer): Params {
  const r = new Rng(seed);
  const p = defaultParams();
  p.sim.level = 3;
  p.sim.seed = seed;
  p.drone.mass = r.uniform(MASS[0], MASS[1]);
  p.wind.meanSpeed = r.uniform(WIND[0], WIND[1]);
  p.wind.meanHeading = r.uniform(0, 360);
  p.wind.gustsPerMinute = r.uniform(0, 6);
  p.wind.gustAmpMax = 6;
  p.control.l3.outer = outer;
  if (r.next() < 0.3) {
    p.setpoint.y = 3;
    p.setpoint.profile = r.next() < 0.5 ? 'figure8' : 'circle';
    p.setpoint.profileAmplitude = r.uniform(1, 2);
    p.setpoint.profilePeriod = r.uniform(7, 12);
  }
  return p;
}

/** Random target jumps every 3 s (for scenarios without a profile). */
function scheduleTargets(sim: Simulation, seed: number, seconds: number): void {
  if (sim.params.setpoint.profile !== 'none') return;
  const r = new Rng(seed ^ 0x9e3779b9);
  for (let t = 4; t < seconds; t += 3) {
    const target = v3(r.uniform(-3, 3), r.uniform(1, 4), r.uniform(-3, 3));
    sim.schedule(t, (s) => s.setTarget(target));
  }
}

/** The setpoint as the controller sees it (mirrors the engine). */
function setpointOf(sim: Simulation): Setpoint {
  const exact = sim.params.setpoint.profile !== 'none' && !sim.takingOff;
  return exact
    ? { pos: sim.setpoint, yaw: sim.yaw, vel: sim.reference.vel }
    : { pos: sim.setpoint, yaw: sim.yaw };
}

const { count, unflatten, init, backprop } = network(SIZES);

// ------------------------------------------------------------------------------------------
// 1. Imitation of the geometric controller.

function collect(episodes: number, seconds: number): { x: number[][]; y: number[][] } {
  const x: number[][] = [];
  const y: number[][] = [];
  const DEG = Math.PI / 180;
  for (let e = 0; e < episodes; e++) {
    const p = scenario(1000 + e, 'geometric');
    const sim = new Simulation(p);
    scheduleTargets(sim, 1000 + e, seconds);
    const ctl = sim.controller as CascadeController;
    for (let k = 1; k <= seconds * 1000; k++) {
      sim.step();
      if (k % 10 !== 0 || sim.t < 0.3 || !sim.measurement || sim.state.crashed) continue;
      const loops = ctl.loops();
      const rates = v3(
        loops['rate.roll']!.setpoint * DEG,
        loops['rate.yaw']!.setpoint * DEG,
        loops['rate.pitch']!.setpoint * DEG,
      );
      x.push(observe(sim.measurement, setpointOf(sim)));
      y.push(encode(ctl.extras().thrust!, rates, p.control.model.mass));
    }
  }
  return { x, y };
}

function imitate(
  theta: Float64Array,
  data: { x: number[][]; y: number[][] },
  epochs: number,
  rng: Rng,
) {
  const adam = new Adam(theta.length, 3e-3);
  const n = data.x.length;
  const order = Array.from({ length: n }, (_, i) => i);
  let loss = NaN;
  for (let ep = 1; ep <= epochs; ep++) {
    for (let i = n - 1; i > 0; i--) {
      const j = Math.floor(rng.next() * (i + 1));
      [order[i], order[j]] = [order[j]!, order[i]!];
    }
    let sum = 0;
    const batch = 256;
    for (let b = 0; b < n; b += batch) {
      const grad = new Float64Array(theta.length);
      const net = unflatten(theta);
      const idx = order.slice(b, b + batch);
      for (const i of idx) sum += backprop(net, data.x[i]!, data.y[i]!, grad);
      for (let g = 0; g < grad.length; g++) grad[g]! /= idx.length;
      adam.step(theta, grad);
    }
    loss = sum / n;
    log(`  epoch ${ep}/${epochs}  loss ${loss.toFixed(4)}`);
  }
  return loss;
}

// ------------------------------------------------------------------------------------------
// 2. Closed-loop evaluation and evolution strategies.

/** Mean position error (m) of the policy over an episode; crashes cost 5 m. Lower is better. */
function episodeCost(theta: Float64Array | null, seed: number, seconds: number): number {
  if (theta) setPolicyWeights(unflatten(theta));
  const sim = new Simulation(scenario(seed, theta ? 'policy' : 'geometric'));
  scheduleTargets(sim, seed, seconds);
  let sum = 0;
  let n = 0;
  for (let k = 1; k <= seconds * 1000; k++) {
    sim.step();
    if (sim.state.crashed) return 5;
    if (k % 20 === 0 && sim.t > 1) {
      const s = sim.state.pos;
      const sp = sim.setpoint;
      sum += Math.min(Math.hypot(s.x - sp.x, s.y - sp.y, s.z - sp.z), 5);
      n++;
    }
  }
  return sum / Math.max(n, 1);
}

const VALIDATION = [9001, 9002, 9003, 9004, 9005, 9006];
/** Validation error of a candidate (or, with null, of the geometric teacher). */
const validate = (theta: Float64Array | null) =>
  VALIDATION.reduce((s, seed) => s + episodeCost(theta, seed, 12), 0) / VALIDATION.length;

function evolve(theta: Float64Array, iterations: number, rng: Rng) {
  const pairs = QUICK ? 6 : 16;
  const sigma = 0.01;
  const adam = new Adam(theta.length, 0.002);
  let best = Float64Array.from(theta);
  let bestCost = validate(theta);
  log(`  start: validation error ${bestCost.toFixed(3)} m`);
  for (let it = 1; it <= iterations; it++) {
    const seeds = [20000 + 3 * it, 20001 + 3 * it, 20002 + 3 * it];
    const eps: Float64Array[] = [];
    const scores: number[] = [];
    for (let i = 0; i < pairs; i++) {
      const e = new Float64Array(theta.length);
      for (let j = 0; j < e.length; j++) e[j] = rng.normal();
      const plus = theta.map((t, j) => t + sigma * e[j]!);
      const minus = theta.map((t, j) => t - sigma * e[j]!);
      const cost = (th: Float64Array) =>
        seeds.reduce((s, sd) => s + episodeCost(th, sd, 8), 0) / seeds.length;
      eps.push(e);
      scores.push(cost(plus), cost(minus));
    }
    // Rank-based fitness shaping (robust to outliers such as crashes).
    const ranks = scores
      .map((c, i) => ({ c, i }))
      .sort((a, b) => a.c - b.c)
      .map((r, k) => ({ ...r, u: 0.5 - k / (scores.length - 1) }));
    const util = new Array<number>(scores.length);
    for (const r of ranks) util[r.i] = r.u;
    const grad = new Float64Array(theta.length);
    for (let i = 0; i < pairs; i++) {
      const w = util[2 * i]! - util[2 * i + 1]!;
      const e = eps[i]!;
      for (let j = 0; j < grad.length; j++) grad[j]! -= (w * e[j]!) / (pairs * sigma);
    }
    adam.step(theta, grad);
    if (it % 5 === 0 || it === iterations) {
      const c = validate(theta);
      log(
        `  iteration ${it}/${iterations}  validation error ${c.toFixed(3)} m${c < bestCost ? '  (best)' : ''}`,
      );
      if (c < bestCost) {
        bestCost = c;
        best = Float64Array.from(theta);
      }
    }
  }
  return { best, bestCost };
}

// ------------------------------------------------------------------------------------------

const t0 = performance.now();
const rng = new Rng(20260928);
log(`Policy ${SIZES.join('→')}, ${count} parameters${QUICK ? ' (quick)' : ''}`);

log('1/3 Collecting demonstrations from the geometric controller…');
const data = collect(QUICK ? 40 : 160, 15);
log(`  ${data.x.length} samples`);

const teacherCost = validate(null);
log(`  teacher (geometric controller): validation error ${teacherCost.toFixed(3)} m`);

log('2/3 Imitation learning…');
const theta = init(rng);
const bcLoss = imitate(theta, data, QUICK ? 4 : 12, rng);
const bcCost = validate(theta);
log(`  imitation only: validation error ${bcCost.toFixed(3)} m`);

log('3/3 Evolution strategies on the closed loop…');
const esIterations = QUICK ? 5 : 40;
const { best, bestCost } = evolve(theta, esIterations, rng);

const round = (x: number) => Number(x.toPrecision(5));
const meta = {
  trained: new Date().toISOString().slice(0, 10),
  demonstrations: `${data.x.length} samples of the geometric controller`,
  'training conditions': `mass ${MASS[0]}–${MASS[1]} kg, wind 0–${WIND[1]} m/s + gusts`,
  'imitation loss': round(bcLoss),
  'error after imitation': `${(bcCost * 100).toFixed(1)} cm`,
  'evolution strategies': `${esIterations} iterations`,
  'error after ES': `${(bestCost * 100).toFixed(1)} cm (validation, mean)`,
  'teacher error': `${(teacherCost * 100).toFixed(1)} cm (geometric controller, same flights)`,
};
const weights = unflatten(best.map(round), meta);
const file = new URL('../src/control/policy-weights.ts', import.meta.url);
writeFileSync(
  file,
  `// Generated by scripts/train-policy.ts (make train). Do not edit by hand.\nimport type { MlpWeights } from './policy';\n\nexport const POLICY_WEIGHTS: MlpWeights = ${JSON.stringify(weights)};\n`,
);
log(`Wrote src/control/policy-weights.ts in ${((performance.now() - t0) / 1000).toFixed(0)} s.`);
