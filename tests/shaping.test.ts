import { describe, expect, it } from 'vitest';
import {
  dfDeadZone,
  dfQuadratic,
  dfRelay,
  dfSaturation,
  dragLimitCycle,
  quadraticAmplitude,
} from '@/analysis/describing';
import { isLoopStable, l1Loop, loopGain } from '@/analysis/loop';
import { shapingLti } from '@/analysis/shaping';
import { runSweep } from '@/analysis/sweep';
import { Simulation } from '@/engine/simulation';
import { cabs, carg, csub } from '@/math/complex';
import { freqResp } from '@/math/lti';
import { logspace, margins } from '@/math/margins';
import { defaultParams, type Params } from '@/sim/params';

const TWO_PI = 2 * Math.PI;
const DEG = 180 / Math.PI;

describe('describing functions', () => {
  it('saturation: unity below the limit, falling as 4·limit/(π·A) far above it', () => {
    expect(dfSaturation(0.5, 1)).toBe(1);
    expect(dfSaturation(1, 1)).toBeCloseTo(1, 12);
    expect(dfSaturation(1000, 1) / ((4 * 1) / (Math.PI * 1000))).toBeCloseTo(1, 4);
    expect(dfSaturation(2, 1)).toBeLessThan(1);
  });

  it('relay and dead zone', () => {
    expect(dfRelay(2, 1)).toBeCloseTo(2 / Math.PI, 12);
    expect(dfDeadZone(0.5, 1)).toBe(0);
    expect(dfDeadZone(2, 1) + dfSaturation(2, 1)).toBeCloseTo(1, 12);
  });

  it('quadratic drag: the fundamental of c·sin|sin| is 8c/(3π), checked by integration', () => {
    const c = 0.25;
    const a = 1.7;
    const n = 20000;
    let s = 0;
    for (let k = 0; k < n; k++) {
      const th = (TWO_PI * (k + 0.5)) / n;
      const x = a * Math.sin(th);
      s += c * x * Math.abs(x) * Math.sin(th);
    }
    const fundamental = (2 * s) / n; // amplitude of the sin component
    expect(fundamental / a).toBeCloseTo(dfQuadratic(a, c), 6);
    expect(quadraticAmplitude(dfQuadratic(a, c), c)).toBeCloseTo(a, 12);
  });
});

/** Fly the P-only loop of lesson I.2 and measure the settled bounce: amplitude and frequency. */
function flyBounce(mod: (p: Params) => void, seconds = 80) {
  const p = defaultParams();
  p.wind.enabled = false;
  p.control.alt = { ...p.control.alt, iOn: false, dOn: false };
  mod(p);
  const sim = new Simulation(p);
  sim.schedule(6, (s) => s.setTarget({ x: 0, y: 3, z: 0 }));
  const ys: number[] = [];
  const ts: number[] = [];
  let clipped = 0;
  for (let k = 0; k < seconds * 1000; k++) {
    sim.step();
    if (k > (seconds - 20) * 1000 && k % 5 === 0) {
      ys.push(sim.state.pos.y);
      ts.push(sim.t);
      const u = sim.actuation.motorCmd[0];
      if (u <= 1e-9 || u >= p.drone.maxMotorThrust - 1e-9) clipped++;
    }
  }
  const hi = Math.max(...ys);
  const lo = Math.min(...ys);
  const mid = (hi + lo) / 2;
  const ups: number[] = [];
  for (let i = 1; i < ys.length; i++) if (ys[i - 1]! < mid && ys[i]! >= mid) ups.push(ts[i]!);
  return {
    p,
    amp: (hi - lo) / 2,
    fHz: (ups.length - 1) / (ups[ups.length - 1]! - ups[0]!),
    clipped: clipped / ys.length,
  };
}

describe('the drag holds the bounce of lesson I.2 (III.8)', () => {
  it('prediction and flight agree for several gains, drags and motors', () => {
    for (const mod of [
      (p: Params) => p,
      (p: Params) => (p.control.alt.kp = 5),
      (p: Params) => (p.drone.dragV = 0.5),
      (p: Params) => (p.drone.motorTau = 0.05),
    ]) {
      const f = flyBounce(mod);
      const c = dragLimitCycle(f.p)!;
      expect(f.clipped).toBe(0);
      expect(Math.abs(f.amp / c.altitude - 1)).toBeLessThan(0.02);
      expect(Math.abs(f.fHz / c.fHz - 1)).toBeLessThan(0.02);
    }
  }, 300_000);

  it('the default: 0.46 m at 0.50 Hz, to the millimetre', () => {
    const f = flyBounce((p) => p);
    const c = dragLimitCycle(f.p)!;
    expect(c.altitude).toBeCloseTo(0.46, 2);
    expect(c.fHz).toBeCloseTo(0.5, 2);
    expect(Math.abs(f.amp - c.altitude)).toBeLessThan(0.002);
  }, 120_000);

  it('at Kp = 40 the motors clip and the single describing function overestimates', () => {
    const f = flyBounce((p) => (p.control.alt.kp = 40));
    const c = dragLimitCycle(f.p)!;
    expect(c.altitude).toBeCloseTo(0.91, 1);
    expect(f.amp).toBeCloseTo(0.67, 1);
    expect(f.clipped).toBeGreaterThan(0.05);
  }, 120_000);

  it('without drag there is no limit cycle to predict', () => {
    const p = defaultParams();
    p.control.alt = { ...p.control.alt, iOn: false, dOn: false };
    p.drone.dragV = 0;
    expect(dragLimitCycle(p)).toBeNull();
  });
});

describe('loop shaping (III.6)', () => {
  const shaped = (mod: (p: Params) => void = () => {}) => {
    const p = defaultParams();
    p.wind.enabled = false;
    p.control.alt = { ...p.control.alt, kp: 20, iOn: false, dOn: false };
    mod(p);
    return p;
  };
  const sol = (p: Params) => {
    p.control.alt.kp = 22;
    p.control.shaping = {
      ...p.control.shaping,
      leadZHz: 0.25,
      leadPHz: 16,
      lagZHz: 0.15,
      lagPHz: 0.03,
    };
  };
  const report = (p: Params) => {
    const m = l1Loop(p)!;
    const w = logspace(TWO_PI * 0.02, TWO_PI * 120, 1500);
    const r = margins(
      w,
      w.map((x) => loopGain(m, x)),
    );
    return {
      stable: isLoopStable(m),
      pm: r.pmDeg,
      fc: r.wc / TWO_PI,
      low: 20 * Math.log10(cabs(loopGain(m, TWO_PI * 0.1))),
    };
  };

  it('a lead gives its textbook phase at the geometric mean of its corners', () => {
    const p = shaped((q) => (q.control.shaping = { ...q.control.shaping, leadZHz: 1, leadPHz: 9 }));
    const g = shapingLti(p, 0.004)!;
    const v = freqResp(g, TWO_PI * 3)[0]![0]!;
    // α = 9: arcsin(8/10) = 53.1°, a little less after Tustin at 250 Hz.
    expect(carg(v) * DEG).toBeGreaterThan(52);
    expect(carg(v) * DEG).toBeLessThan(53.2);
    expect(cabs(freqResp(g, TWO_PI * 0.01)[0]![0]!)).toBeCloseTo(1, 3);
  });

  it('with no stage on the PID model is unchanged', () => {
    const p = shaped();
    expect(shapingLti(p, 0.004)).toBeNull();
  });

  it('P alone is unstable; the lead and lag of the solution meet the target', () => {
    expect(report(shaped()).stable).toBe(false);
    const r = report(shaped(sol));
    expect(r.stable).toBe(true);
    expect(r.fc).toBeGreaterThan(1.8);
    expect(r.fc).toBeLessThan(2.2);
    expect(r.pm).toBeGreaterThan(45);
    expect(r.low).toBeGreaterThan(40);
    // The lag costs phase at crossover: without it the same lead gives more margin.
    const noLag = report(shaped((p) => (sol(p), (p.control.shaping.lagZHz = 0))));
    expect(noLag.pm).toBeGreaterThan(r.pm + 3);
    expect(noLag.low).toBeLessThan(40);
  });

  it('the stages fly as the model says, and the drone follows a step without offset', () => {
    const p = shaped(sol);
    const m = l1Loop(p)!;
    for (const pt of runSweep(p, [0.2, 1, 3])) {
      const md = loopGain(m, TWO_PI * pt.fHz);
      expect(cabs(csub(pt.l!, md)) / cabs(md)).toBeLessThan(0.01);
    }
    const sim = new Simulation(p);
    for (let k = 0; k < 8000; k++) sim.step();
    sim.setTarget({ x: 0, y: 2.5, z: 0 });
    for (let k = 0; k < 8000; k++) sim.step();
    expect(Math.abs(sim.state.pos.y - 2.5)).toBeLessThan(0.005);
  }, 300_000);
});
