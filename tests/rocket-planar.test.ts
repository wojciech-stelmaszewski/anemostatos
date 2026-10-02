import { describe, expect, it } from 'vitest';
import { linearize } from '@/analysis/linearize';
import { analyseTvc, minPitchGain, tvcLoop, spectralRadius, wrongWayZero } from '@/analysis/tvc';
import { PHYS_DT, Simulation } from '@/engine/simulation';
import { LESSONS } from '@/lessons/lessons';
import { cabs } from '@/math/complex';
import { evalAt } from '@/math/lti';
import { eigenvalues } from '@/math/mat';
import { defaultParams, type Params } from '@/sim/params';
import { modalMass, modeShape, modeSlope, muAlpha, muDelta, tvcRocket } from '@/sim/vehicles/tvc';

/** A lesson's set-up, with a change. */
const lessonParams = (id: string, mod: (p: Params) => void = () => {}): Params => {
  const lesson = LESSONS.find((l) => l.id === id)!;
  const p = defaultParams();
  p.sim.level = lesson.level;
  lesson.setup!(p);
  mod(p);
  return p;
};
const fly = (p: Params, seconds: number, events?: (sim: Simulation) => void): Simulation => {
  const sim = new Simulation(p);
  if (events) {
    sim.script = events;
    sim.reset();
  }
  for (let k = 0; k < seconds * 1000; k++) sim.step();
  return sim;
};
/** Largest |value| of a telemetry channel between two times. */
const peak = (sim: Simulation, key: string, from: number, to = Infinity): number => {
  const w = sim.telemetry.window([key], from);
  let m = 0;
  w.t.forEach((t, i) => {
    const v = w.series[0]![i]!;
    if (t <= to && Number.isFinite(v)) m = Math.max(m, Math.abs(v));
  });
  return m;
};
const scaleGains = (p: Params, k: number) => {
  p.control.tvc.kp *= k;
  p.control.tvc.kd *= k;
};

describe('the pitch-plane rocket', () => {
  it('linearises to θ̈ = μα·α − μc·δ with the airframe unstable at about √μα', () => {
    const p = defaultParams();
    const r = p.tvc;
    expect(muAlpha(r)).toBeCloseTo(4, 9);
    expect(muDelta(r)).toBeCloseTo(8.333, 3);
    const lin = linearize(tvcRocket, p, tvcRocket.trim(p));
    // State order x, u, θ, ω, δ, …: the pitch acceleration per radian of pitch is μα.
    expect(lin.a[3]![2]! / PHYS_DT).toBeCloseTo(muAlpha(r), 3);
    // The actuator moves first within a step, so the gimbal acts with a factor (1 − dt/τ).
    expect(lin.a[3]![4]! / PHYS_DT).toBeCloseTo(-muDelta(r) * (1 - PHYS_DT / r.actuatorTau), 3);
    const growth = eigenvalues(lin.a)
      .map((z) => Math.log(Math.hypot(z.re, z.im)) / PHYS_DT)
      .filter((v) => v > 1e-6)
      .sort((a, b) => b - a);
    // The airframe, and a slow drift mode of the flight path.
    expect(growth).toHaveLength(2);
    expect(Math.abs(growth[0]! / Math.sqrt(muAlpha(r)) - 1)).toBeLessThan(0.05);
    expect(growth[1]!).toBeLessThan(0.1);
  });

  it('the first bending mode: ends together, nodes at 0.224 and 0.776, slope zero in the middle', () => {
    expect(modeShape(0)).toBeCloseTo(1, 6);
    expect(modeShape(1)).toBeCloseTo(1, 6);
    expect(Math.abs(modeShape(0.2242))).toBeLessThan(2e-3);
    expect(Math.abs(modeShape(0.7758))).toBeLessThan(2e-3);
    expect(modeSlope(0.5, 12)).toBeCloseTo(0, 9);
    expect(modeSlope(0.4, 12)).toBeCloseTo(-modeSlope(0.6, 12), 6);
    expect(modeSlope(0.85, 12)).toBeGreaterThan(0);
    // A uniform beam with the tail normalised to 1 has a quarter of the mass in the mode.
    expect(modalMass(defaultParams().tvc) / 2000).toBeCloseTo(0.25, 3);
  });

  it('the analysis agrees with flight: the upper edge of the gain', () => {
    const p = lessonParams('tvc');
    p.control.tvc.kp = 1.5;
    p.control.tvc.kd = 0.8;
    p.tvc.startPitchDeg = 0.01; // small, so the gimbal stays linear
    const { high } = analyseTvc(p);
    const late = (k: number) => {
      const q = structuredClone(p);
      scaleGains(q, k);
      const sim = fly(q, 15);
      return peak(sim, 'tvc.theta', 10);
    };
    expect(late(0.92 * high)).toBeLessThan(0.005);
    expect(late(1.08 * high)).toBeGreaterThan(0.05);
  }, 120_000);
});

describe('lesson IV.7: balancing on a flame', () => {
  it('starts below the smallest gain and breaks up within seconds', () => {
    const sim = new Simulation(lessonParams('tvc'));
    let t = NaN;
    for (let k = 0; k < 15000 && Number.isNaN(t); k++) {
      sim.step();
      if (sim.state.crashed) t = sim.t;
    }
    expect(t).toBeLessThan(10);
  });

  it('the smallest kp is within 10 % of μα/μc, and the closed loop agrees', () => {
    const p = lessonParams('tvc');
    const kmin = minPitchGain(p);
    expect(Math.abs(kmin / (muAlpha(p.tvc) / muDelta(p.tvc)) - 1)).toBeLessThan(0.1);
    const late = (kp: number) => {
      const q = structuredClone(p);
      q.control.tvc.kp = kp;
      q.tvc.startPitchDeg = 0.2;
      return peak(fly(q, 30), 'tvc.theta', 25);
    };
    expect(late(1.15 * kmin)).toBeLessThan(0.2);
    expect(late(0.85 * kmin)).toBeGreaterThan(0.4);
  }, 120_000);

  it('the design keeps more than 6 dB both ways; five times its gain leaves less than 6 dB above', () => {
    const p = lessonParams('tvc');
    LESSONS.find((l) => l.id === 'tvc')!.solution!(p);
    const a = analyseTvc(p);
    expect(a.stable).toBe(true);
    expect(a.lowDb).toBeGreaterThan(6);
    expect(a.highDb).toBeGreaterThan(6);
    expect(a.margins.pmDeg).toBeGreaterThan(40);
    expect(a.margins.wc / (2 * Math.PI)).toBeGreaterThan(0.8); // crossover about 1 Hz
    expect(a.margins.wc / (2 * Math.PI)).toBeLessThan(1.2);
    scaleGains(p, 5);
    const b = analyseTvc(p);
    expect(b.stable).toBe(true);
    expect(b.highDb).toBeLessThan(6);
  });
});

describe('lesson IV.8: bending', () => {
  it('with the gyro near the nose the mode is a 26 dB spike and the loop sings at 5 Hz', () => {
    const p = lessonParams('bending');
    const a = analyseTvc(p);
    expect(a.stable).toBe(false);
    expect(a.bendDb).toBeGreaterThan(24);
    expect(a.bendDb).toBeLessThan(28);
    const sim = fly(p, 12);
    // A steady buzz, held to about a degree and a half by the rate limit.
    expect(peak(sim, 'tvc.delta', 8)).toBeGreaterThan(1);
    expect(sim.state.crashed).toBe(false);
  }, 120_000);

  it('a 40 dB notch gain-stabilises it with the phase margin intact; 30 dB is not enough', () => {
    const p = lessonParams('bending');
    LESSONS.find((l) => l.id === 'bending')!.solution!(p);
    const a = analyseTvc(p);
    expect(a.stable).toBe(true);
    expect(a.bendDb).toBeLessThan(-10);
    expect(a.margins.pmDeg).toBeGreaterThan(30);
    expect(peak(fly(p, 14), 'tvc.delta', 10)).toBeLessThan(0.1);
    p.control.tvc.notchDepthDb = 30;
    expect(analyseTvc(p).bendDb).toBeGreaterThan(-10);
  }, 120_000);

  it('at 0.4 and 0.6 the gyro sees the mode equally strongly; one sings, the other is stable', () => {
    const at = (g: number) => lessonParams('bending', (p) => (p.tvc.gyroStation = g));
    const aft = analyseTvc(at(0.4));
    const fwd = analyseTvc(at(0.6));
    expect(Math.abs(aft.bendDb - fwd.bendDb)).toBeLessThan(0.5);
    expect(fwd.bendDb).toBeGreaterThan(15);
    expect(aft.stable).toBe(false);
    expect(fwd.stable).toBe(true);
    expect(peak(fly(at(0.4), 14), 'tvc.delta', 10)).toBeGreaterThan(0.5);
    expect(peak(fly(at(0.6), 14), 'tvc.delta', 10)).toBeLessThan(0.2);
  }, 120_000);
});

describe('lesson IV.9: slosh', () => {
  it('unbaffled the slosh lobe destabilises the loop, and the oscillation grows in flight', () => {
    const p = lessonParams('slosh');
    expect(analyseTvc(p).stable).toBe(false);
    const sim = fly(p, 40);
    // After the start the wobble dies down, then comes back and grows.
    expect(peak(sim, 'tvc.theta', 35)).toBeGreaterThan(2.5 * peak(sim, 'tvc.theta', 5, 10));
  }, 120_000);

  it('a little damping restores it, and between 0.02 and 0.035 the curve clears −1 by 0.5', () => {
    const dist = (z: number) => {
      const a = analyseTvc(lessonParams('slosh', (p) => (p.tvc.sloshZeta = z)));
      return a.stable ? 1 / a.margins.ms : 0;
    };
    expect(dist(0.02)).toBeLessThan(0.5);
    expect(dist(0.035)).toBeGreaterThan(0.5);
    const p = lessonParams('slosh');
    LESSONS.find((l) => l.id === 'slosh')!.solution!(p);
    expect(1 / analyseTvc(p).margins.ms).toBeGreaterThan(0.5);
    const sim = fly(p, 30);
    expect(peak(sim, 'tvc.theta', 25)).toBeLessThan(0.3 * peak(sim, 'tvc.theta', 5, 10));
  }, 120_000);

  it('the tank behind the centre of mass, or further ahead, is stable without baffles', () => {
    for (const ahead of [-2, 4])
      expect(analyseTvc(lessonParams('slosh', (p) => (p.tvc.sloshAhead = ahead))).stable).toBe(
        true,
      );
  });

  it('a notch at the slosh frequency costs the rigid body its phase margin', () => {
    const a = analyseTvc(
      lessonParams('slosh', (p) => {
        p.tvc.sloshZeta = 0.05;
        p.control.tvc.notch = true;
        p.control.tvc.notchHz = 0.8;
        p.control.tvc.notchDepthDb = 20;
        p.control.tvc.notchWidth = 0.3;
      }),
    );
    expect(!a.stable || a.margins.pmDeg < 20).toBe(true);
  });
});

describe('lesson IV.10: first the wrong way', () => {
  it('the drift has a right-half-plane zero at √(T·l/I)', () => {
    const p = lessonParams('wrongway');
    const z = wrongWayZero(p);
    expect(z).toBeCloseTo(2.022, 2);
    // x/δ vanishes at s = z on the real axis (z = e^{z·dt} for the discrete model), not nearby.
    const plant = tvcLoop(p).plant;
    const g = (s: number) => cabs(evalAt(plant, { re: Math.exp(s * PHYS_DT), im: 0 })[2]![0]!);
    expect(g(z)).toBeLessThan(1e-3 * g(0.7 * z));
    expect(g(z)).toBeLessThan(1e-3 * g(1.4 * z));
  });

  const step = (mod: (p: Params) => void) => {
    const lesson = LESSONS.find((l) => l.id === 'wrongway')!;
    const p = lessonParams('wrongway', mod);
    const sim = fly(p, 18, lesson.events);
    const w = sim.telemetry.window(['tvc.x'], 2);
    let settle = NaN;
    let under = 0;
    w.t.forEach((t, i) => {
      const x = w.series[0]![i]!;
      under = Math.min(under, x);
      if (Math.abs(x - 2) > 0.1) settle = NaN;
      else if (Number.isNaN(settle)) settle = t - 2;
    });
    return { settle, under, crashed: sim.state.crashed };
  };

  it('it first moves the wrong way, more the faster the loop; the solution settles in time', () => {
    const slow = step(() => {});
    expect(slow.under).toBeLessThan(-0.02);
    expect(Number.isNaN(slow.settle) || slow.settle > 6).toBe(true);
    const fast = step((p) => {
      p.control.tvc.kx = 1.5;
      p.control.tvc.kv = 4;
    });
    expect(fast.settle).toBeLessThan(6);
    expect(fast.under).toBeLessThan(slow.under);
  }, 120_000);

  it('a drift gain of 5 °/m runs the gimbal into its stops and the rocket falls', () => {
    const r = step((p) => {
      p.control.tvc.kx = 5;
      p.control.tvc.kv = 4;
    });
    expect(r.crashed).toBe(true);
    // A little further and the linear loop is unstable.
    const p = lessonParams('wrongway', (q) => {
      q.control.tvc.kx = 6;
      q.control.tvc.kv = 5;
    });
    expect(spectralRadius(tvcLoop(p))).toBeGreaterThan(1);
  }, 120_000);

  it('a faster pitch loop does not help: with the same drift loop the rocket falls', () => {
    const r = step((p) => {
      p.control.tvc.kx = 1.5;
      p.control.tvc.kv = 4;
      p.control.tvc.kp = 6;
      p.control.tvc.kd = 1.6;
    });
    expect(r.crashed).toBe(true);
  }, 120_000);
});
