import { describe, expect, it } from 'vitest';
import { l1Loop, loopGain, isLoopStable } from '@/analysis/loop';
import { runSweep } from '@/analysis/sweep';
import { lyapunovP } from '@/control/mrac';
import { Simulation } from '@/engine/simulation';
import { LESSONS } from '@/lessons/lessons';
import { cabs, csub } from '@/math/complex';
import { defaultParams, GRAVITY, type Params } from '@/sim/params';

const lessonParams = (id: string, mod: (p: Params) => void = () => {}): Params => {
  const l = LESSONS.find((x) => x.id === id)!;
  const p = defaultParams();
  p.sim.level = l.level;
  l.setup!(p);
  mod(p);
  return p;
};
const flyLesson = (id: string, p: Params, seconds: number) => {
  const l = LESSONS.find((x) => x.id === id)!;
  const sim = new Simulation(p);
  sim.script = l.events ?? null;
  sim.reset();
  for (let k = 0; k < seconds * 1000 && !sim.state.crashed; k++) sim.step();
  return sim;
};
const mean = (sim: Simulation, key: string, from: number, to: number) => {
  const { t, series } = sim.telemetry.window([key], from);
  const v = series[0]!.filter((_, i) => t[i]! <= to && !Number.isNaN(series[0]![i]!));
  return v.reduce((a, b) => a + b, 0) / v.length;
};
const rmsOf = (sim: Simulation, key: string, from: number, to: number) => {
  const { t, series } = sim.telemetry.window([key], from);
  const v = series[0]!.filter((_, i) => t[i]! <= to && !Number.isNaN(series[0]![i]!));
  return Math.sqrt(v.reduce((a, b) => a + b * b, 0) / v.length);
};
/** RMS change of the total thrust command between telemetry samples, N. */
const jitter = (sim: Simulation, from: number) => {
  const keys = ['motorCmd.1', 'motorCmd.2', 'motorCmd.3', 'motorCmd.4'];
  const { series } = sim.telemetry.window(keys, from);
  let sq = 0;
  for (let i = 1; i < series[0]!.length; i++) {
    const u = series.reduce((s, x) => s + x[i]!, 0) - series.reduce((s, x) => s + x[i - 1]!, 0);
    sq += u * u;
  }
  return Math.sqrt(sq / (series[0]!.length - 1));
};

describe('sliding mode (III.15)', () => {
  const calm = (smc: Params['control']['smc']) =>
    lessonParams('smc', (p) => {
      p.wind.enabled = false;
      p.control.smc = smc;
    });

  it('inside the boundary layer it is the PD the lesson names, and the model matches a flight', () => {
    const p = calm({ k: 6, lambda: 3, phi: 0.05 });
    const m = l1Loop(p)!;
    expect(isLoopStable(m)).toBe(true);
    for (const pt of runSweep(p, [0.5, 2], { amp: 0.05 })) {
      const model = loopGain(m, 2 * Math.PI * pt.fHz);
      expect(cabs(csub(pt.l!, model)) / cabs(model)).toBeLessThan(0.03);
    }
    // Without a layer the law is a switch: no linear model.
    expect(l1Loop(calm({ k: 6, lambda: 3, phi: 0 }))).toBeNull();
  });

  it('the payload: the sag is F·φ/(k·λ), and without a layer almost nothing', () => {
    for (const phi of [0.05, 0.2]) {
      const sim = flyLesson('smc', calm({ k: 6, lambda: 3, phi }), 25);
      const sag = mean(sim, 'alt.err', 20, 25) * 100;
      const predicted = (100 * 0.3 * GRAVITY * phi) / (6 * 3);
      expect(sag / predicted).toBeGreaterThan(0.95);
      expect(sag / predicted).toBeLessThan(1.05);
    }
    const ideal = flyLesson('smc', calm({ k: 6, lambda: 3, phi: 0 }), 25);
    expect(Math.abs(mean(ideal, 'alt.err', 20, 25))).toBeLessThan(0.004);
    expect(
      LESSONS.find((l) => l.id === 'smc')!.predict!.truth(calm({ k: 6, lambda: 3, phi: 0 })),
    ).toBeCloseTo(0.8175, 3);
  }, 300_000);

  it('a switching force below the payload weight cannot hold the line: the drone sinks', () => {
    const sim = flyLesson('smc', calm({ k: 2, lambda: 3, phi: 0 }), 25);
    expect(mean(sim, 'alt.err', 20, 25)).toBeGreaterThan(1.5);
  }, 300_000);

  it('in gusts: φ = 0 rejects everything and chatters; the layer stops the chatter for under a centimetre', () => {
    const chat = flyLesson('smc', lessonParams('smc'), 30);
    expect(jitter(chat, 15)).toBeGreaterThan(4);
    expect(rmsOf(chat, 'alt.err', 15, 30)).toBeLessThan(0.005);
    const layer = flyLesson(
      'smc',
      lessonParams('smc', (p) => (p.control.smc.phi = 0.05)),
      30,
    );
    expect(jitter(layer, 15)).toBeLessThan(0.1);
    expect(rmsOf(layer, 'alt.err', 15, 30)).toBeLessThan(0.012);
    // The default PID on the same flight: far worse.
    const pid = flyLesson(
      'smc',
      lessonParams('smc', (p) => (p.control.l1.kind = 'pid')),
      30,
    );
    expect(rmsOf(pid, 'alt.err', 15, 30)).toBeGreaterThan(0.1);
  }, 300_000);
});

describe('MRAC (III.17)', () => {
  it('the Lyapunov matrix solves AᵀP + PA = −I', () => {
    const [p11, p12, p22] = lyapunovP(9, 6);
    const a = [
      [0, 1],
      [-9, -6],
    ];
    const P = [
      [p11, p12],
      [p12, p22],
    ];
    const atp = [0, 1].map((i) => [0, 1].map((j) => a[0]![i]! * P[0]![j]! + a[1]![i]! * P[1]![j]!));
    for (const [i, j] of [
      [0, 0],
      [0, 1],
      [1, 1],
    ] as const)
      expect(atp[i]![j]! + atp[j]![i]!).toBeCloseTo(i === j ? -1 : 0, 9);
  });

  const trace = (mod: (p: Params) => void, seconds: number) => {
    const sim = flyLesson('mrac', lessonParams('mrac', mod), seconds);
    return sim;
  };

  it('the gains drift twenty times past their ideal and the motors saturate; the error grows', () => {
    const sim = trace(() => {}, 40);
    expect(sim.state.crashed).toBe(false);
    expect(rmsOf(sim, 'mrac.err', 1, 4)).toBeLessThan(0.01);
    expect(sim.controller.extras()['mrac.theta.d']!).toBeGreaterThan(20 * 6);
    expect(rmsOf(sim, 'mrac.err', 30, 40)).toBeGreaterThan(0.1);
    const sat = mean(sim, 'alt.sat', 30, 40);
    expect(sat).toBeGreaterThan(0.6);
  }, 300_000);

  it('without the delay the gain still drifts, but the motors keep up', () => {
    const sim = trace((p) => (p.sensors.delayMs = 0), 40);
    expect(sim.controller.extras()['mrac.theta.d']!).toBeGreaterThan(45);
    expect(mean(sim, 'alt.sat', 30, 40)).toBeLessThan(0.05);
  }, 300_000);

  it('σ-modification holds the gains near the first guess and the drone follows the model', () => {
    const sim = trace((p) => (p.control.mrac.sigma = 0.01), 45);
    expect(sim.controller.extras()['mrac.theta.d']!).toBeLessThan(6);
    expect(rmsOf(sim, 'mrac.err', 30, 45)).toBeLessThan(0.05);
    // The hover parameter, left free, finds m·g of the true drone.
    expect(sim.controller.extras()['mrac.theta.g']!).toBeCloseTo(GRAVITY, 0);
  }, 300_000);
});
