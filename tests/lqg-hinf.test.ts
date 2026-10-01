import { describe, expect, it } from 'vitest';
import { isLoopStable, l1Loop, loopGain } from '@/analysis/loop';
import { runSweep } from '@/analysis/sweep';
import { robustTest } from '@/analysis/uncertainty';
import { designHinf, shapedPlant } from '@/control/hinf';
import { Simulation } from '@/engine/simulation';
import { care, careResidual } from '@/math/care';
import { cabs, carg, csub } from '@/math/complex';
import { eigenvalues, maxAbs, mul, transpose } from '@/math/mat';
import { logspace, margins } from '@/math/margins';
import { defaultParams, type Params } from '@/sim/params';

const TWO_PI = 2 * Math.PI;
const calm = (mod: (p: Params) => void = () => {}): Params => {
  const p = defaultParams();
  p.wind.enabled = false;
  mod(p);
  return p;
};
const marginsOf = (p: Params) => {
  const m = l1Loop(p)!;
  const w = logspace(TWO_PI * 0.02, TWO_PI * 120, 1500);
  const r = margins(
    w,
    w.map((x) => loopGain(m, x)),
  );
  return { ...r, gmDb: 20 * Math.log10(r.gm), fc: r.wc / TWO_PI };
};

describe('the continuous Riccati equation', () => {
  it('scalar: X = (a + √(a² + r·q))/r', () => {
    const x = care([[2]], [[3]], [[5]]);
    expect(x[0]![0]).toBeCloseTo((2 + Math.sqrt(4 + 15)) / 3, 10);
  });

  it('a double integrator with a lag: the residual vanishes and the closed loop is stable', () => {
    const g = shapedPlant(calm((p) => (p.control.l1.kind = 'hinf')));
    const r = mul(g.b, transpose(g.b));
    const q = mul(transpose(g.c), g.c);
    const x = care(g.a, r, q);
    expect(maxAbs(careResidual(g.a, r, q, x))).toBeLessThan(1e-8);
    // A − R·X is Hurwitz: every eigenvalue in the left half plane.
    const rx = mul(r, x);
    const acl = g.a.map((row, i) => row.map((v, j) => v - rx[i]![j]!));
    expect(eigenvalues(acl).every((z) => z.re < 0)).toBe(true);
  });
});

describe('LQG: optimal plus optimal is not robust', () => {
  const lqr = (mod: (p: Params) => void = () => {}) =>
    calm((p) => {
      p.control.l1.kind = 'lqr';
      p.control.lqr.lagState = true;
      mod(p);
    });
  const lqg = (kfQ: number) =>
    lqr((p) => {
      p.control.lqr.observer = true;
      p.control.lqr.kfQ = kfQ;
    });

  it('the LQR with its sensors keeps the margins LQR promises', () => {
    const m = marginsOf(lqr());
    expect(m.pmDeg).toBeGreaterThan(60);
    expect(m.gm).toBe(Infinity);
  });

  it('with the Kalman filter in front they shrink, and loop transfer recovery brings them back', () => {
    const tight = marginsOf(lqg(0.3));
    expect(tight.pmDeg).toBeLessThan(40);
    expect(tight.gmDb).toBeLessThan(10);
    let prev = tight.pmDeg;
    for (const q of [3, 30, 100]) {
      const m = marginsOf(lqg(q));
      expect(m.pmDeg).toBeGreaterThan(prev);
      prev = m.pmDeg;
    }
    expect(prev).toBeGreaterThan(60);
  });

  it('the model of the LQG loop is what the simulator flies', () => {
    const p = lqg(0.3);
    const m = l1Loop(p)!;
    for (const pt of runSweep(p, [0.3, 1, 3])) {
      const model = loopGain(m, TWO_PI * pt.fHz);
      expect(cabs(csub(pt.l!, model)) / cabs(model)).toBeLessThan(0.02);
      expect(Math.abs(carg(pt.l!) - carg(model))).toBeLessThan(0.02);
    }
  }, 300_000);

  it('recovery costs noise: a filter that follows the altimeter shakes the motors', () => {
    const jitter = (q: number) => {
      const p = lqg(q);
      p.sensors.posNoise = 0.02;
      const sim = new Simulation(p);
      let sq = 0;
      let n = 0;
      let prev = NaN;
      for (let k = 0; k < 16000; k++) {
        sim.step();
        if (k > 6000 && k % 5 === 0) {
          const u = sim.actuation.motorCmd.reduce((a, b) => a + b, 0);
          if (Number.isFinite(prev)) {
            sq += (u - prev) ** 2;
            n++;
          }
          prev = u;
        }
      }
      return Math.sqrt(sq / n);
    };
    const a = jitter(0.3);
    const b = jitter(4);
    const c = jitter(30);
    expect(b).toBeGreaterThan(3 * a);
    expect(c).toBeGreaterThan(3 * b);
  }, 300_000);
});

describe('H∞ loop shaping', () => {
  const hinf = (mod: (p: Params) => void = () => {}) =>
    calm((p) => {
      p.control.l1.kind = 'hinf';
      p.control.hinf = { k: 200, wi: 0.5, wz: 0, wp: 0, gammaFactor: 1.05 };
      mod(p);
    });

  it('the weight alone cannot stabilise the drone; the robustified controller can', () => {
    // A PI on a double integrator with a lag: the PID model with kd = 0 is the same thing.
    const pi = calm((p) => {
      p.control.alt.kp = 20;
      p.control.alt.dOn = false;
    });
    expect(isLoopStable(l1Loop(pi)!)).toBe(false);
    expect(isLoopStable(l1Loop(hinf())!)).toBe(true);
  });

  it('ε_max is a stability margin: the phase margin is near 2·arcsin ε', () => {
    for (const mod of [
      (p: Params) => p,
      (p: Params) => ((p.control.hinf.wz = 5), (p.control.hinf.wp = 50)),
    ]) {
      const p = hinf(mod);
      const d = designHinf(p, 0.004);
      expect(d.epsMax).toBeGreaterThan(0.25);
      const bound = (2 * Math.asin(d.epsMax) * 180) / Math.PI;
      // Sampling and the hold cost a few degrees that the continuous design does not see.
      expect(Math.abs(marginsOf(p).pmDeg - bound)).toBeLessThan(6);
    }
  });

  it('a steep shape fails the family test; with a lead it passes at 3.5 Hz', () => {
    const steep = hinf();
    expect(robustTest(steep)!.worst).toBeGreaterThan(1);
    const lead = hinf((p) => ((p.control.hinf.wz = 5), (p.control.hinf.wp = 50)));
    expect(robustTest(lead)!.worst).toBeLessThan(1);
    expect(marginsOf(lead).fc).toBeGreaterThan(3.2);
    // The PID frontier of lesson III.11: its fastest tune that passes is slower.
    const pid = calm(
      (p) => ((p.control.alt.kp = 40), (p.control.alt.kd = 20), (p.control.alt.iOn = false)),
    );
    expect(robustTest(pid)!.worst).toBeLessThan(1);
    expect(marginsOf(pid).fc).toBeLessThan(3);
  }, 300_000);

  it('it flies: steps followed without offset, and the model matches the flight', () => {
    const p = hinf((q) => ((q.control.hinf.wz = 5), (q.control.hinf.wp = 50)));
    const m = l1Loop(p)!;
    for (const pt of runSweep(p, [0.5, 2, 5])) {
      const model = loopGain(m, TWO_PI * pt.fHz);
      expect(cabs(csub(pt.l!, model)) / cabs(model)).toBeLessThan(0.02);
    }
    const sim = new Simulation(p);
    for (let k = 0; k < 8000; k++) sim.step();
    sim.setTarget({ x: 0, y: 2.5, z: 0 });
    for (let k = 0; k < 6000; k++) sim.step();
    expect(Math.abs(sim.state.pos.y - 2.5)).toBeLessThan(0.005);
  }, 300_000);
});
