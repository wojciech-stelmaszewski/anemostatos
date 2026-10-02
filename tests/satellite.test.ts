import { describe, expect, it } from 'vitest';
import { linearize } from '@/analysis/linearize';
import { limitCycleFuelRate, limitCyclePeriod, minDrift, pulseTime } from '@/control/satellite';
import { PHYS_DT, Simulation } from '@/engine/simulation';
import { LESSONS } from '@/lessons/lessons';
import { v3 } from '@/math/vec3';
import { defaultParams, type Params } from '@/sim/params';
import {
  errorAngle,
  inertialMomentum,
  initialSatellite,
  satellite,
  stepSatellite,
} from '@/sim/vehicles/satellite';

const DEG = 180 / Math.PI;

/** Fly a chapter N lesson's set-up with a change for `seconds`, calling `each` after every step. */
const fly = (
  id: string,
  mod: (p: Params) => void,
  seconds: number,
  each?: (sim: Simulation) => void,
): Simulation => {
  const lesson = LESSONS.find((l) => l.id === id)!;
  const p = defaultParams();
  p.sim.level = lesson.level;
  lesson.setup!(p);
  mod(p);
  const sim = new Simulation(p);
  sim.script = lesson.events ?? null;
  sim.reset();
  for (let k = 0; k < seconds * 1000; k++) {
    sim.step();
    each?.(sim);
  }
  return sim;
};

/** The time after which the pointing error stays within `tol` degrees (NaN if it never does). */
const settleTime = (sim: Simulation, tol = 0.1): number => {
  const w = sim.telemetry.window(['sat.err'], -Infinity);
  const e = w.series[0]!;
  let k = e.length - 1;
  if (!(e[k]! <= tol)) return NaN;
  while (k > 0 && e[k - 1]! <= tol) k--;
  return w.t[k]!;
};

describe('the satellite', () => {
  it('conserves its total angular momentum when the wheels trade it with the body', () => {
    const sp = defaultParams().satellite;
    const s = initialSatellite();
    s.w = v3(0.1, -0.05, 0.2); // a tumble about no principal axis
    s.h = v3(0.1, 0.2, -0.1);
    const h0 = inertialMomentum(s, sp);
    for (let k = 0; k < 20000; k++) {
      const t = k * PHYS_DT;
      stepSatellite(
        s,
        { wheel: v3(0.05 * Math.sin(t), -0.03, 0.04), thrusters: v3() },
        PHYS_DT,
        sp,
      );
    }
    const h1 = inertialMomentum(s, sp);
    expect(Math.hypot(h1.x - h0.x, h1.y - h0.y, h1.z - h0.z)).toBeLessThan(2e-3);
  });

  it('a wheel gives no more than its torque, and nothing once it is at full speed', () => {
    const sp = defaultParams().satellite;
    const s = initialSatellite();
    stepSatellite(s, { wheel: v3(1, 0, 0), thrusters: v3() }, PHYS_DT, sp);
    expect(s.wheelTorque.x).toBe(sp.wheelTorque);
    s.h = v3(-sp.wheelMomentum, 0, 0); // full speed the way a positive body torque spins it
    stepSatellite(s, { wheel: v3(0.1, 0, 0), thrusters: v3() }, PHYS_DT, sp);
    expect(s.wheelTorque.x).toBe(0);
  });

  it('linearises to a double integrator per axis: ω⁺ = ω + τ·dt/J, θ⁺ = θ + ω⁺·dt', () => {
    const p = defaultParams();
    const lin = linearize(satellite, p, satellite.trim(p));
    const J = p.satellite.inertia;
    expect(lin.b[3]![0]).toBeCloseTo(PHYS_DT / J.x, 9);
    expect(lin.b[4]![1]).toBeCloseTo(PHYS_DT / J.y, 9);
    expect(lin.b[5]![2]).toBeCloseTo(PHYS_DT / J.z, 9);
    expect(lin.a[0]![3]).toBeCloseTo(PHYS_DT, 6);
    // The wheels take the opposite: ḣ = −τ.
    expect(lin.b[6]![0]).toBeCloseTo(-PHYS_DT, 9);
  });
});

describe('lesson IV.18: the slew', () => {
  const turned = (shortest: boolean) => {
    let most = 0;
    const sim = fly(
      'slew',
      (p) => (p.satellite.shortest = shortest),
      45,
      (s) => {
        const q = s.satellite!.q;
        most = Math.max(most, 2 * Math.acos(Math.max(-1, Math.min(1, q.w))) * DEG);
      },
    );
    return { most, sim };
  };

  it('takes the error as it comes and turns 240°; with the sign flipped it turns 120° the other way', () => {
    expect(turned(false).most).toBeCloseTo(240, 0);
    expect(turned(true).most).toBeCloseTo(120, 0);
  }, 120_000);

  /** The largest one-step rise of V, and how many steps a wheel spent at full momentum or torque. */
  const lyapunovRun = (mod: (p: Params) => void) => {
    let prev = Infinity;
    let rise = 0;
    let fullMomentum = 0;
    let fullTorque = 0;
    fly('slew', mod, 40, (s) => {
      const sp = s.params.satellite;
      const st = s.satellite!;
      const J = sp.inertia;
      const e = s.satCtl.error;
      const V =
        0.5 * (J.x * st.w.x ** 2 + J.y * st.w.y ** 2 + J.z * st.w.z ** 2) + 2 * sp.kp * (1 - e.w);
      rise = Math.max(rise, V - prev);
      prev = V;
      if (Math.abs(st.h.y) >= sp.wheelMomentum - 1e-9) fullMomentum++;
      if (Math.abs(st.wheelTorque.x) >= sp.wheelTorque - 1e-12) fullTorque++;
    });
    return { rise, fullMomentum, fullTorque };
  };

  it('V never rises with moderate gains, even with a wheel at full speed', () => {
    for (const mod of [
      (p: Params) => p,
      (p: Params) => Object.assign(p.satellite, { shortest: true, kp: 8, kd: 8 }),
    ]) {
      const r = lyapunovRun(mod);
      expect(r.rise).toBeLessThan(1e-9);
      expect(r.fullMomentum).toBeGreaterThan(0);
    }
  }, 120_000);

  it('with high gains the torque limit clips the command and V rises, slightly', () => {
    const r = lyapunovRun((p) => Object.assign(p.satellite, { shortest: true, kp: 32, kd: 8 }));
    expect(r.fullTorque).toBeGreaterThan(0);
    expect(r.rise).toBeGreaterThan(1e-6);
    expect(r.rise).toBeLessThan(1e-3); // V(0) = 2·kp·(1 − cos 60°) = 32: a millionth of it
  }, 120_000);

  it('the long way cannot settle within 20 s with any of these gains; the short way can', () => {
    for (const [kp, kd] of [
      [4, 4],
      [8, 6],
      [8, 8],
      [16, 12],
      [32, 16],
    ] as const) {
      const sim = fly('slew', (p) => Object.assign(p.satellite, { kp, kd }), 45);
      expect(settleTime(sim)).toBeGreaterThan(22);
    }
    const best = fly(
      'slew',
      (p) => Object.assign(p.satellite, { shortest: true, kp: 8, kd: 8 }),
      25,
    );
    expect(settleTime(best)).toBeLessThan(16);
  }, 300_000);

  it('the wheels cap the rate at about 16°/s', () => {
    let top = 0;
    fly(
      'slew',
      (p) => Object.assign(p.satellite, { shortest: true, kp: 32, kd: 8 }),
      25,
      (s) => {
        const w = s.satellite!.w;
        top = Math.max(top, Math.hypot(w.x, w.y, w.z) * DEG);
      },
    );
    expect(top).toBeGreaterThan(15);
    expect(top).toBeLessThan(17);
  }, 120_000);
});

describe('lesson IV.19: wheels and momentum dumping', () => {
  it('the z wheel is full at h_max/τ_d = 30 s, and then the target is lost', () => {
    let full = NaN;
    const sim = fly(
      'wheels',
      () => {},
      40,
      (s) => {
        if (Number.isNaN(full) && Math.abs(s.satellite!.h.z) >= s.params.satellite.wheelMomentum)
          full = s.t;
      },
    );
    expect(Math.abs(full / 30 - 1)).toBeLessThan(0.02);
    expect(errorAngle(sim.satCtl.error) * DEG).toBeGreaterThan(5);
  }, 120_000);

  it('before that the PD holds an offset of τ_d/(kp/2)', () => {
    const sim = fly('wheels', () => {}, 20);
    const offset = (0.02 / (8 / 2)) * DEG;
    expect(Math.abs(errorAngle(sim.satCtl.error) * DEG - offset)).toBeLessThan(0.05 * offset);
  }, 120_000);

  it('dumping alone jumps to 0.8°; told about the thrusters, the wheels hold 0.3° and never fill', () => {
    const plain = fly('wheels', (p) => (p.satellite.dump = true), 45);
    expect(plain.satPeak).toBeGreaterThan(0.7);
    expect(plain.satPeak).toBeLessThan(0.9);
    let most = 0;
    const ff = fly(
      'wheels',
      (p) => Object.assign(p.satellite, { dump: true, dumpFeedforward: true }),
      45,
      (s) => (most = Math.max(most, Math.abs(s.satellite!.h.z))),
    );
    expect(ff.satPeak).toBeLessThan(0.35);
    expect(most).toBeLessThan(0.45);
    expect(ff.satellite!.fuel).toBeGreaterThan(0); // the thrusters did fire
  }, 120_000);
});

describe('lesson IV.20: the thruster limit cycle', () => {
  /** Fly the thruster lesson with a dead band and a drift rate; fuel rate from 15 to 45 s. */
  const cycle = (deadbandDeg: number, driftRateDeg: number) => {
    let f15 = 0;
    const sim = fly(
      'thrusters',
      (p) => Object.assign(p.satellite, { deadbandDeg, driftRateDeg }),
      45,
      (s) => {
        if (Math.abs(s.t - 15) < 5e-4) f15 = s.satellite!.fuel;
      },
    );
    return {
      rate: ((sim.satellite!.fuel - f15) / 30) * 60,
      predicted: limitCycleFuelRate(sim.params.satellite) * 60,
      peak: sim.satPeak,
      sp: sim.params.satellite,
    };
  };

  it('the fuel rate follows the phase-plane geometry, pulses included', () => {
    for (const [db, w] of [
      [0.1, 1],
      [0.3, 0.5],
      [0.45, 0.5],
      [0.45, 0],
    ] as const) {
      const c = cycle(db, w);
      expect(Math.abs(c.rate / c.predicted - 1)).toBeLessThan(0.1);
    }
  }, 300_000);

  it('the slowest drift is half a minimum pulse: 0.36°/s, and the pulse is then the minimum', () => {
    const sp = { ...defaultParams().satellite, driftRateDeg: 0 };
    expect(minDrift(sp) * DEG).toBeCloseTo(0.358, 3);
    expect(pulseTime(sp)).toBeCloseTo(sp.minPulse, 9);
    expect(limitCyclePeriod({ ...sp, deadbandDeg: 0.5 })).toBeCloseTo(5.79, 2);
  });

  it('the solution points within 0.5° within 30 % of the least fuel; a faster drift or a band at the limit does not', () => {
    const least = cycle(0.5, 0).predicted;
    const best = cycle(0.45, 0);
    expect(best.peak).toBeLessThan(0.5);
    expect(best.rate).toBeLessThan(1.3 * least);
    expect(cycle(0.45, 0.5).rate).toBeGreaterThan(1.3 * least);
    expect(cycle(0.5, 0).peak).toBeGreaterThan(0.5); // the pulse overshoots the band a little
    // The starting settings burn about sixteen times the least.
    const start = cycle(0.1, 1).rate / least;
    expect(start).toBeGreaterThan(14);
    expect(start).toBeLessThan(18);
  }, 300_000);
});

describe('lesson IV.22: star tracker and gyro', () => {
  /** Knowledge error at each telemetry sample from `from` on, and the filter's 2σ there. */
  const knowledge = (mod: (p: Params) => void, from = 15, seconds = 45) => {
    const sim = fly('startracker', mod, seconds);
    const w = sim.telemetry.window(['st.err', 'st.sigma2'], from);
    return { err: w.series[0]!, sigma2: w.series[1]!, sim };
  };

  it('on the gyro alone the estimate drifts at |b|: 1 °/h is 1″/s', () => {
    const sim = fly('startracker', () => {}, 30);
    const b = sim.params.satellite.gyroBiasDegH;
    const expected = Math.hypot(b.x, b.y, b.z) * 30;
    expect(Math.abs(sim.starNav.error(sim.satellite!) / expected - 1)).toBeLessThan(0.03);
  }, 120_000);

  it('with the tracker but no bias state the filter is confidently wrong, even at 5 Hz', () => {
    for (const hz of [1, 5]) {
      const k = knowledge((p) => (p.satellite.trackerHz = hz), 15, 25);
      expect(Math.max(...k.err)).toBeGreaterThan(20);
      expect(Math.max(...k.sigma2)).toBeLessThan(8);
    }
  }, 120_000);

  it('estimating the bias keeps the attitude within 10″ through the outage, for several seeds', () => {
    for (const seed of [1, 2, 3]) {
      const k = knowledge((p) => {
        p.sim.seed = seed;
        p.satellite.trackerHz = 1;
        p.satellite.estimateBias = true;
      });
      expect(Math.max(...k.err)).toBeLessThan(10);
      expect(k.sim.starNav.biasError(k.sim.params.satellite)).toBeLessThan(1);
      // Honest: the error is within the filter's own 2σ most of the time.
      const inside = k.err.filter((e, i) => e <= k.sigma2[i]!).length / k.err.length;
      expect(inside).toBeGreaterThan(0.8);
    }
  }, 120_000);

  it('without the bias state the outage costs tens of arc-seconds', () => {
    const k = knowledge((p) => (p.satellite.trackerHz = 1));
    expect(Math.max(...k.err)).toBeGreaterThan(80);
  }, 120_000);
});
