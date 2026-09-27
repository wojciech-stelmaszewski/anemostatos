import { describe, expect, it } from 'vitest';
import { designLqr } from '@/control/lqr';
import { Simulation } from '@/engine/simulation';
import { Eso } from '@/estimation/eso';
import { AltitudeKalman } from '@/estimation/kalman';
import { eigenvalues } from '@/math/mat';
import { Rng } from '@/math/prng';
import { defaultParams, GRAVITY, type Params } from '@/sim/params';

const calm = (setup: (p: Params) => void = () => {}): Params => {
  const p = defaultParams();
  p.wind.enabled = false;
  setup(p);
  return p;
};
const run = (p: Params, seconds: number, onStep?: (sim: Simulation) => void) => {
  const sim = new Simulation(p);
  for (let k = 0; k < seconds * 1000; k++) {
    sim.step();
    onStep?.(sim);
  }
  return sim;
};

describe('eigenvalues', () => {
  it('finds complex and real roots of a 3×3', () => {
    // Companion matrix of (λ + 1)(λ² + 2λ + 5): roots −1, −1 ± 2j.
    const ev = eigenvalues([
      [0, 1, 0],
      [0, 0, 1],
      [-5, -7, -3],
    ]).sort((a, b) => a.im - b.im);
    expect(ev[0]!.re).toBeCloseTo(-1, 8);
    expect(ev[0]!.im).toBeCloseTo(-2, 8);
    expect(ev[1]!.re).toBeCloseTo(-1, 8);
    expect(ev[1]!.im).toBeCloseTo(0, 8);
  });
});

describe('LQR / LQI (L1)', () => {
  it('matches the continuous double-integrator gains when sampled fast', () => {
    const p = calm();
    const d = designLqr(p, 0.001);
    // m̂·ÿ = u with m̂ = 1, q = (100, 10), r = 1 ⇒ k₁ = 10, k₂ = √(10 + 2·10).
    expect(d.k[0]).toBeCloseTo(10, 1);
    expect(d.k[1]).toBeCloseTo(Math.sqrt(30), 1);
    for (const s of d.poles) expect(s.re).toBeLessThan(0);
  });

  it('flies a step without overshoot', () => {
    let peak = 0;
    const sim = run(
      calm((p) => (p.control.l1.kind = 'lqr')),
      16,
      (s) => {
        if (s.t > 8 && s.t < 8.002) s.setTarget({ x: 0, y: 3, z: 0 });
        if (s.t > 8) peak = Math.max(peak, s.state.pos.y);
      },
    );
    expect(sim.state.pos.y).toBeCloseTo(3, 3);
    expect(peak).toBeLessThan(3.01);
  });

  it('droops by (m − m̂)·g/k₁ with a wrong mass, and LQI removes it', () => {
    const wrong = (p: Params) => {
      p.control.l1.kind = 'lqr';
      p.control.model.mass = 0.8;
    };
    const lqr = run(calm(wrong), 15);
    const k1 = designLqr(lqr.params, 0.004).k[0]!;
    expect(2 - lqr.state.pos.y).toBeCloseTo((0.2 * GRAVITY) / k1, 2);
    const lqi = run(
      calm((p) => {
        wrong(p);
        p.control.lqr.integral = true;
      }),
      25,
    );
    expect(lqi.state.pos.y).toBeCloseTo(2, 2);
  });

  it('flies with the motor-lag state, with and without motor telemetry', () => {
    for (const telemetry of [true, false]) {
      const sim = run(
        calm((p) => {
          p.control.l1.kind = 'lqr';
          p.control.lqr.lagState = true;
          p.sensors.motorFeedback = telemetry;
        }),
        10,
      );
      expect(sim.state.pos.y).toBeCloseTo(2, 2);
    }
  });
});

describe('extended state observer', () => {
  it('estimates a constant disturbance on a double integrator', () => {
    // Bounded motion, as in closed loop: the input cancels f and adds a gentle wobble. (On an
    // ever-accelerating open-loop trajectory the sampled measurement lags and biases f̂.)
    const eso = new Eso();
    const dt = 0.004;
    let y = 0;
    let v = 0;
    const f = -1.5;
    for (let k = 0; k < 2500; k++) {
      const u = 1.5 + 0.2 * Math.sin(k * dt);
      eso.update(u, y, 20, 1, dt);
      const a = u + f;
      y += v * dt + 0.5 * a * dt * dt;
      v += a * dt;
    }
    expect(eso.z[2]).toBeCloseTo(f, 2);
  });
});

describe('ADRC (L1)', () => {
  it('holds the setpoint with a wrong mass, and its estimate matches the true disturbance', () => {
    const sim = run(
      calm((p) => {
        p.control.l1.kind = 'adrc';
        p.control.model.mass = 0.8;
      }),
      12,
    );
    expect(sim.state.pos.y).toBeCloseTo(2, 3);
    expect(sim.telemetry.latest('est.dist.y')).toBeCloseTo(sim.telemetry.latest('dist.y'), 2);
    expect(sim.telemetry.latest('dist.y')).toBeCloseTo(-0.2 * GRAVITY, 1);
  });

  it('works without the gravity feedforward (the observer learns gravity)', () => {
    const sim = run(
      calm((p) => {
        p.control.l1.kind = 'adrc';
        p.control.feedforward = false;
      }),
      15,
    );
    expect(sim.state.pos.y).toBeCloseTo(2, 2);
  });
});

describe('Kalman filter (L1)', () => {
  it('beats the raw sensor and learns the accelerometer bias', () => {
    const kf = new AltitudeKalman();
    const rng = new Rng(7);
    const dt = 0.004;
    const noise = { accSigma: 0.3, posSigma: 0.05, biasSigma: 0.05 };
    let y = 2;
    let v = 0;
    let rawSq = 0;
    let estSq = 0;
    let n = 0;
    for (let k = 0; k < 10000; k++) {
      const t = k * dt;
      const a = 0.5 * Math.sin(t); // some true motion
      v += a * dt;
      y += v * dt;
      const acc = a + GRAVITY + 0.2 + 0.3 * rng.normal();
      const fix = k % 5 === 0 ? y + 0.05 * rng.normal() : null;
      kf.step(k ? dt : 0, acc, fix, noise, GRAVITY);
      if (k > 2500 && fix !== null) {
        rawSq += (fix - y) ** 2;
        estSq += (kf.y - y) ** 2;
        n++;
      }
    }
    expect(Math.sqrt(estSq / n)).toBeLessThan(0.5 * Math.sqrt(rawSq / n));
    expect(kf.bias).toBeCloseTo(0.2, 1);
    expect(kf.sigma(0)).toBeLessThan(0.05);
  });

  it('can sit in front of every L1 controller', () => {
    for (const kind of ['pid', 'lqr', 'adrc'] as const) {
      const sim = run(
        calm((p) => {
          p.control.l1.kind = kind;
          p.control.l1.estimator = 'kalman';
          p.sensors.posNoise = 0.03;
          p.sensors.posRateHz = 50;
        }),
        12,
      );
      expect(Math.abs(sim.state.pos.y - 2), kind).toBeLessThan(0.05);
    }
  });
});
