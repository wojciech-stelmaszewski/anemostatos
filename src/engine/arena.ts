import { setIn } from '@/engine/schema';
import { v3 } from '@/math/vec3';
import { defaultParams, type Params } from '@/sim/params';
import { Simulation } from './simulation';

/**
 * The arena (lesson II.21, `make bench`): every controller configuration flies every scenario
 * on identical, seeded conditions, and the same metrics are measured for all.
 */
export interface ArenaScenario {
  id: string;
  label: string;
  /** What the scenario tests. */
  about: string;
  seconds: number;
  /** Metrics are measured from this time on. */
  from: number;
  setup: (p: Params) => void;
  events?: (sim: Simulation) => void;
}

export interface ArenaEntry {
  id: string;
  label: string;
  setup: (p: Params) => void;
}

export interface ArenaResult {
  scenario: string;
  entry: string;
  /** RMS and maximum 3D position error, m. */
  rms: number;
  max: number;
  /** RMS sample-to-sample change of the total thrust at 200 Hz, N — how hard the motors work. */
  jitter: number;
  crashed: boolean;
  /** Wall-clock cost, µs per physics step (not deterministic; for information only). */
  cpuUs: number;
}

const at = (t: number, path: string, value: unknown) => (sim: Simulation) =>
  sim.schedule(t, (s) => s.setParams(setIn(s.params, path, value)));

export const SCENARIOS: ArenaScenario[] = [
  {
    id: 'step',
    label: 'Step',
    about: '3 m sideways jump, calm air',
    seconds: 20,
    from: 10,
    setup: (p) => (p.wind.enabled = false),
    events: (sim) => sim.schedule(10, (s) => s.setTarget(v3(3, 2, 0))),
  },
  {
    id: 'gusts',
    label: 'Gusts',
    about: "Part I's gust challenge, hovering",
    seconds: 40,
    from: 8,
    setup: (p) => {
      p.sim.seed = 4242;
      p.wind.gustsPerMinute = 14;
      p.wind.gustAmpMin = 4;
      p.wind.gustAmpMax = 10;
      p.wind.turbSigma = 1.2;
    },
  },
  {
    id: 'payload',
    label: 'Payload',
    about: '+300 g at 12 s',
    seconds: 25,
    from: 12,
    setup: () => {},
    events: at(12, 'drone.mass', 1.3),
  },
  {
    id: 'fault',
    label: 'Broken prop',
    about: 'motor 2 at 60 % at 12 s, calm',
    seconds: 25,
    from: 12,
    setup: (p) => (p.wind.enabled = false),
    events: at(12, 'drone.motorEfficiency', [1, 0.6, 1, 1]),
  },
  {
    id: 'figure8',
    label: 'Figure-8',
    about: '2 m/s figure-8, light wind',
    seconds: 40,
    from: 20,
    setup: (p) => {
      p.wind.gustsOn = false;
      p.wind.turbSigma = 0.3;
      p.setpoint.y = 3;
      p.setpoint.profile = 'figure8';
      p.setpoint.profileAmplitude = 2;
      p.setpoint.profilePeriod = 8.9;
    },
  },
  {
    id: 'noise',
    label: 'Noisy sensors',
    about: 'position 3 cm, gyro 5 °/s, accelerometer 1 m/s², 20 ms delay; gusts',
    seconds: 30,
    from: 8,
    setup: (p) => {
      p.sensors.posNoise = 0.03;
      p.sensors.gyroNoise = 5;
      p.sensors.accNoise = 1;
      p.sensors.delayMs = 20;
    },
  },
];

const l3 =
  (outer: Params['control']['l3']['outer'], extra: (p: Params) => void = () => {}) =>
  (p: Params) => {
    p.control.l3.outer = outer;
    extra(p);
  };
const indi = (p: Params) => {
  p.control.l3.inner = 'indi';
  p.control.l3.compensation = 'indi';
};

export const ENTRIES: ArenaEntry[] = [
  { id: 'pid', label: 'PID cascade (Part I)', setup: l3('pid-cascade') },
  { id: 'geo', label: 'Geometric + flatness', setup: l3('geometric') },
  { id: 'geo-indi', label: 'Geometric + INDI', setup: l3('geometric', indi) },
  {
    id: 'geo-l1',
    label: 'Geometric + L1 adaptive',
    setup: l3('geometric', (p) => (p.control.l3.compensation = 'l1ac')),
  },
  {
    id: 'geo-learned',
    label: 'Geometric + learned residual',
    setup: l3('geometric', (p) => (p.control.l3.compensation = 'learned')),
  },
  { id: 'mpc', label: 'MPC', setup: l3('mpc') },
  { id: 'mpc-indi', label: 'MPC + INDI', setup: l3('mpc', indi) },
  { id: 'mppi', label: 'MPPI', setup: l3('mppi') },
  { id: 'policy', label: 'Neural policy', setup: l3('policy') },
];

/** Fly one entry through one scenario. */
export function runArena(entry: ArenaEntry, scenario: ArenaScenario): ArenaResult {
  const p = defaultParams();
  p.sim.level = 3;
  scenario.setup(p);
  entry.setup(p);
  const sim = new Simulation(p);
  scenario.events?.(sim);
  let sq = 0;
  let max = 0;
  let n = 0;
  let dT = 0;
  let nT = 0;
  let lastT = NaN;
  const t0 = performance.now();
  const steps = scenario.seconds * 1000;
  for (let k = 1; k <= steps; k++) {
    sim.step();
    if (sim.state.crashed) break;
    if (k % 5 === 0 && sim.t >= scenario.from) {
      const s = sim.state.pos;
      const sp = sim.setpoint;
      const e = Math.hypot(s.x - sp.x, s.y - sp.y, s.z - sp.z);
      sq += e * e;
      max = Math.max(max, e);
      n++;
      const T =
        sim.state.motors[0] + sim.state.motors[1] + sim.state.motors[2] + sim.state.motors[3];
      if (Number.isFinite(lastT)) {
        dT += (T - lastT) ** 2;
        nT++;
      }
      lastT = T;
    }
  }
  const cpuUs = ((performance.now() - t0) * 1000) / Math.max(sim.stepIndex, 1);
  return {
    scenario: scenario.id,
    entry: entry.id,
    rms: n ? Math.sqrt(sq / n) : NaN,
    max,
    jitter: nT ? Math.sqrt(dT / nT) : NaN,
    crashed: sim.state.crashed,
    cpuUs,
  };
}

/** Best entry per scenario by RMS error (crashes excluded). */
export function winners(results: ArenaResult[]): Record<string, string> {
  const out: Record<string, string> = {};
  for (const sc of SCENARIOS) {
    const rows = results.filter((r) => r.scenario === sc.id && !r.crashed);
    if (rows.length) out[sc.id] = rows.reduce((a, b) => (b.rms < a.rms ? b : a)).entry;
  }
  return out;
}
