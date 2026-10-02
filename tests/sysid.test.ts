import { describe, expect, it } from 'vitest';
import { runSweep } from '@/analysis/sweep';
import { Simulation } from '@/engine/simulation';
import {
  finishedChirp,
  fitChirp,
  fitPlant,
  plantModel,
  relayPrediction,
  relayTest,
  runChirp,
} from '@/estimation/sysid';
import { crossSpectrum } from '@/math/fft';
import { LESSONS } from '@/lessons/lessons';
import { logspace } from '@/math/margins';
import { defaultParams, type Params } from '@/sim/params';

const lesson = (id: string) => LESSONS.find((l) => l.id === id)!;
const setup = (id: string, mod: (p: Params) => void = () => {}): Params => {
  const p = defaultParams();
  p.sim.level = 1;
  lesson(id).setup!(p);
  mod(p);
  return p;
};
const hoverOffset = (p: Params, seconds = 15) => {
  const sim = new Simulation(p);
  let sum = 0;
  let n = 0;
  for (let k = 0; k < seconds * 1000; k++) {
    sim.step();
    if (k > (seconds - 3) * 1000) {
      sum += sim.state.pos.y - sim.setpoint.y;
      n++;
    }
  }
  return sum / n;
};

describe('the fit', () => {
  it('recovers m, τ and d exactly from the model it assumes', () => {
    const truth = { mass: 1.3, tau: 0.045, delay: 0.025 };
    const pts = logspace(0.2, 6, 12).map((fHz) => ({
      fHz,
      p: plantModel(truth, 2 * Math.PI * fHz),
    }));
    const f = fitPlant(pts);
    expect(f.mass).toBeCloseTo(1.3, 9);
    expect(f.tau).toBeCloseTo(0.045, 9);
    expect(f.delay).toBeCloseTo(0.025, 9);
    expect(f.rmsDb).toBeLessThan(1e-9);
  });

  it('cross-spectra: the response and coherence of a known filter, with and without noise', () => {
    // y = x delayed by 5 samples and halved: H = 0.5·e^{−jω·5·dt}, coherence 1.
    const n = 1 << 15;
    let s = 1;
    const rnd = () => ((s = (s * 16807) % 2147483647) / 2147483647) * 2 - 1;
    const x = Float64Array.from({ length: n }, rnd);
    const y = x.map((_, i) => (i >= 5 ? 0.5 * x[i - 5]! : 0));
    const cs = crossSpectrum(x, y, 1000, 1024);
    const k = cs.f.findIndex((f) => f >= 50);
    expect(Math.hypot(cs.h[k]!.re, cs.h[k]!.im)).toBeCloseTo(0.5, 2);
    expect(cs.coherence[k]!).toBeGreaterThan(0.99);
    // Add noise as large as the signal: coherence near one half.
    const yn = y.map((v) => v + 0.5 * rnd());
    const cn = crossSpectrum(x, yn, 1000, 1024);
    const mean = cn.coherence.slice(10, 400).reduce((a, b) => a + b, 0) / 390;
    expect(mean).toBeGreaterThan(0.35);
    expect(mean).toBeLessThan(0.65);
  });
});

describe('lesson III.25: identify, then design', () => {
  it('the LQR on the wrong model hangs about 40 cm low; with the fitted one it holds', () => {
    expect(hoverOffset(setup('sysid'))).toBeLessThan(-0.3);
    // The lag alone barely matters; the mass decides it.
    expect(Math.abs(hoverOffset(setup('sysid', (p) => (p.control.model.mass = 1))))).toBeLessThan(
      0.02,
    );
    const p = setup('sysid');
    const f = fitPlant(runSweep(p, logspace(0.2, 5, 10)).map((q) => ({ fHz: q.fHz, p: q.pm! })));
    expect(Math.abs(f.mass - 1)).toBeLessThan(0.005);
    expect(Math.abs(f.tau - 0.03)).toBeLessThan(0.0005);
    expect(f.delay * 1000).toBeGreaterThan(20);
    expect(f.delay * 1000).toBeLessThan(22);
    p.control.model.mass = f.mass;
    p.control.model.motorTau = f.tau;
    expect(Math.abs(hoverOffset(p))).toBeLessThan(0.02);
  }, 300_000);

  it('the relay finds the velocity loop’s −180° point where the model puts it', () => {
    const p = setup('sysid');
    const r = relayTest(p);
    const pr = relayPrediction(p);
    expect(r.tu).toBeCloseTo(0.18, 1);
    expect(Math.abs(r.tu / pr.tu - 1)).toBeLessThan(0.05);
    expect(Math.abs(r.ku / pr.ku - 1)).toBeLessThan(0.1);
    // Without the delay only the motor lag is left, and the cycle is four times faster.
    const fast = relayTest(setup('sysid', (q) => (q.sensors.delayMs = 0)));
    expect(fast.tu).toBeLessThan(0.05);
  }, 300_000);
});

describe('lesson III.26: trust the coherence', () => {
  it('the thrust-based estimate is biased with high coherence; against the probe it is right', () => {
    const fits: string[] = [];
    for (const seed of [1, 2, 1337]) {
      const p = setup('freqid', (q) => {
        q.sim.seed = seed;
        lesson('freqid').solution!(q);
      });
      const id = runChirp(p);
      const direct = fitChirp(id, 0.8, 'direct');
      expect(direct.mass).toBeLessThan(0.75);
      // …with high coherence over most of the band.
      const high = id.direct.coherence.filter((c) => c >= 0.8).length / id.fHz.length;
      expect(high).toBeGreaterThan(0.5);
      const good = fitChirp(id, 0.75, 'viaProbe');
      expect(Math.abs(good.mass - 1)).toBeLessThan(0.1);
      expect(Math.abs(good.tau / 0.03 - 1)).toBeLessThan(0.1);
      expect(Math.abs(good.delay * 1000 - 20)).toBeLessThan(3);
      fits.push(`${good.mass.toFixed(3)}`);
    }
    expect(fits).toHaveLength(3);
  }, 600_000);

  it('the goal reads the finished chirp of the current settings', () => {
    const goal = lesson('freqid').goal!.check;
    const ctx = (p: Params) => ({ sim: new Simulation(p), metrics: null });
    const start = setup('freqid');
    expect(finishedChirp(start)).toBeNull();
    expect(goal(ctx(start))).toMatch(/Fly the chirp/);
    runChirp(start);
    expect(goal(ctx(start))).not.toBe(true);
    const solved = setup('freqid', (q) => lesson('freqid').solution!(q));
    runChirp(solved);
    expect(goal(ctx(solved))).toBe(true);
    // The probe-based estimate on the short, weak chirp of the start is not enough.
    const half = setup('freqid', (q) => {
      q.sysid.estimator = 'viaProbe';
      q.sysid.minCoherence = 0.75;
    });
    runChirp(half);
    expect(goal(ctx(half))).not.toBe(true);
  }, 600_000);
});
