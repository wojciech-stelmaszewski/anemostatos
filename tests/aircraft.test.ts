import { describe, expect, it } from 'vitest';
import {
  aircraftLinear,
  aircraftModes,
  envelopeMargins,
  hqPoint,
  lanchesterPeriod,
  leastDamperGain,
  meetsPitchSpec,
  pitchLoopMargins,
} from '@/analysis/aircraft';
import { Simulation } from '@/engine/simulation';
import { LESSONS } from '@/lessons/lessons';
import { v3 } from '@/math/vec3';
import { defaultParams, type Params } from '@/sim/params';
import { aircraft, trimAircraft } from '@/sim/vehicles/aircraft';

const DEG = 180 / Math.PI;

/** The default aircraft, flying, with changes. */
const airParams = (mod: (p: Params) => void = () => {}): Params => {
  const p = defaultParams();
  p.sim.vehicle = 'aircraft';
  mod(p);
  return p;
};

/** Times at which `y` crosses its mean upwards. */
const upCrossings = (t: number[], y: number[]): number[] => {
  const mean = y.reduce((a, b) => a + b, 0) / y.length;
  const out: number[] = [];
  for (let i = 1; i < y.length; i++) if (y[i - 1]! < mean && y[i]! >= mean) out.push(t[i]!);
  return out;
};

/** Local maxima of `y` (index, value). */
const maxima = (y: number[]): { i: number; v: number }[] => {
  const out: { i: number; v: number }[] = [];
  for (let i = 1; i < y.length - 1; i++)
    if (y[i]! > y[i - 1]! && y[i]! >= y[i + 1]!) out.push({ i, v: y[i]! });
  return out;
};

describe('the aircraft', () => {
  it('trims to level flight: one step leaves the trim point where it is', () => {
    const p = airParams();
    const { state, input } = aircraft.trim(p);
    const s = structuredClone(state);
    aircraft.step(s, input, { wind: v3(), external: v3() }, 0.001, p);
    const before = aircraft.toVector(state, state);
    const after = aircraft.toVector(s, state);
    for (const i of [0, 1, 2, 3, 6]) expect(after[i]).toBeCloseTo(before[i]!, 9);
    expect(after[4]).toBeCloseTo(before[4]!, 9); // level: no climb
  });

  it('the trim is feasible across the envelope of lesson IV.13, with a plausible angle of attack', () => {
    const p = airParams();
    for (const V of [80, 110, 160]) {
      const t = trimAircraft(p.aircraft, V, p.aircraft.altitude);
      expect(t.feasible).toBe(true);
      expect(t.state.alpha * DEG).toBeGreaterThan(-2);
      expect(t.state.alpha * DEG).toBeLessThan(8);
    }
  });

  it('flies untouched for a minute in the simulator without drifting', () => {
    const sim = new Simulation(airParams());
    const h0 = sim.aircraft!.h;
    for (let k = 0; k < 60000; k++) sim.step();
    expect(Math.abs(sim.aircraft!.V - 110)).toBeLessThan(0.01);
    expect(Math.abs(sim.aircraft!.h - h0)).toBeLessThan(0.1);
    expect(sim.state.pos.x).toBeCloseTo(sim.aircraft!.x, 9);
  }, 120_000);
});

describe('lesson IV.11: the modes', () => {
  const p = airParams();
  const m = aircraftModes(p, false);

  it('the bare airframe has the short period and the phugoid quoted in the lesson', () => {
    const sp = m.shortPeriod!;
    const ph = m.phugoid!;
    expect(sp.pole.re).toBeCloseTo(-0.79, 2);
    expect(sp.pole.im).toBeCloseTo(3.24, 2);
    expect(sp.period).toBeCloseTo(1.94, 2);
    expect(sp.zeta).toBeCloseTo(0.24, 2);
    expect(ph.pole.re).toBeCloseTo(-0.005, 3);
    expect(ph.pole.im).toBeCloseTo(0.127, 3);
    expect(ph.period).toBeGreaterThan(49);
    expect(ph.period).toBeLessThan(49.6);
    expect(ph.zeta).toBeLessThan(0.05);
    // A factor of 25 apart in frequency.
    expect(sp.wn / ph.wn).toBeGreaterThan(25);
    expect(sp.wn / ph.wn).toBeLessThan(28);
  });

  it("Lanchester's period is within about one per cent, on the long side", () => {
    const l = lanchesterPeriod(110);
    expect(l).toBeCloseTo(49.8, 1);
    expect(l / m.phugoid!.period - 1).toBeGreaterThan(0);
    expect(l / m.phugoid!.period - 1).toBeLessThan(0.015);
  });

  it('without the altitude in the model the phugoid is longer: the thinner air shortens it', () => {
    // The flown period (below) matches the model with altitude; Lanchester ignores it.
    expect(lanchesterPeriod(110)).toBeGreaterThan(m.phugoid!.period);
  });

  it('the flown pulse response has the periods of the poles', () => {
    const lesson = LESSONS.find((l) => l.id === 'modes')!;
    const q = defaultParams();
    q.sim.level = 1;
    lesson.setup!(q);
    const sim = new Simulation(q);
    sim.lessonStart = structuredClone(q);
    sim.script = lesson.events!;
    sim.reset();
    const t: number[] = [];
    const V: number[] = [];
    const rate: number[] = [];
    for (let k = 0; k < 160000; k++) {
      sim.step();
      if (k % 10 === 0) {
        t.push(sim.t);
        V.push(sim.aircraft!.V);
        rate.push(sim.aircraft!.q);
      }
    }
    // The phugoid in the speed, after the short period has died out.
    const from = t.findIndex((x) => x > 20);
    const c = upCrossings(t.slice(from), V.slice(from));
    expect(c.length).toBeGreaterThanOrEqual(2);
    for (let i = 1; i < c.length; i++)
      expect(Math.abs(c[i]! - c[i - 1]! - m.phugoid!.period)).toBeLessThan(0.5);
    // The short period in the pitch rate, just after the pulse ends at 6 s.
    const a = t.findIndex((x) => x > 6.05);
    const b = t.findIndex((x) => x > 11);
    const peaks = maxima(rate.slice(a, b));
    const period = (t[a + peaks[1]!.i]! - t[a + peaks[0]!.i]!) / 1;
    expect(Math.abs(period / m.shortPeriod!.period - 1)).toBeLessThan(0.03);
  }, 120_000);
});

describe('lesson IV.12: the pitch damper', () => {
  it('the bare short period is level 3 by its damping alone', () => {
    const hq = hqPoint(airParams(), false)!;
    expect(hq.level).toBe(3);
    expect(hq.zeta).toBeLessThan(0.25);
    // Its CAP is comfortably inside the level-1 band: only the damping is wrong.
    expect(hq.cap).toBeGreaterThan(0.28);
    expect(hq.cap).toBeLessThan(3.6);
  });

  it('the least damper gain is about 0.083 s; the short-period estimate gives 0.07', () => {
    const p = airParams();
    const g = leastDamperGain(p);
    expect(g).toBeGreaterThan(0.08);
    expect(g).toBeLessThan(0.086);
    // M_δe = q̄·S·c·C_mδe/I_yy, pitch acceleration per radian of elevator.
    const a = p.aircraft;
    const lin = aircraftLinear(a, 110);
    const mDe = (lin.qbar * a.wingArea * a.chord * a.cmDe) / a.inertia;
    expect(mDe).toBeCloseTo(-10.7, 1);
    // The linear model's entry is 2 % smaller: within a step the actuator moves before the moment.
    expect(lin.a[2]![5]! / mDe).toBeCloseTo(1 - 0.001 / a.servoTau, 3);
    const sp = aircraftModes(p, false).shortPeriod!;
    const estimate = (2 * sp.wn * (0.35 - sp.zeta)) / Math.abs(mDe);
    expect(estimate).toBeCloseTo(0.07, 2);
    // At the least gain the point is in level 1, just at the damping boundary.
    const at = hqPoint({ ...p, autopilot: { ...p.autopilot, gain: g } })!;
    expect(at.level).toBe(1);
    expect(at.zeta).toBeCloseTo(0.35, 3);
  });

  it('without the delay and the actuator lag the full model needs 0.079: the rest is the approximation', () => {
    const p = airParams((q) => {
      q.autopilot.delayMs = 0;
      q.aircraft.servoTau = 0.001;
    });
    expect(leastDamperGain(p)).toBeGreaterThan(0.077);
    expect(leastDamperGain(p)).toBeLessThan(0.081);
  });

  it('the damper damps the flown response as the poles say: successive pitch-rate peaks', () => {
    const ratio = (gain: number) => {
      const sim = new Simulation(airParams((q) => (q.autopilot.gain = gain)));
      sim.schedule(1, (s) =>
        s.setParams({ ...s.params, autopilot: { ...s.params.autopilot, elevatorOffset: -2 } }),
      );
      sim.schedule(1.3, (s) =>
        s.setParams({ ...s.params, autopilot: { ...s.params.autopilot, elevatorOffset: 0 } }),
      );
      const qs: number[] = [];
      for (let k = 0; k < 6000; k++) {
        sim.step();
        if (k > 1300) qs.push(sim.aircraft!.q);
      }
      // The extremes after the release; the first is still forced by the pulse.
      const ext = qs.filter(
        (v, i) => i > 0 && i < qs.length - 1 && (v - qs[i - 1]!) * (qs[i + 1]! - v) < 0,
      );
      return Math.abs(ext[2]! / ext[1]!);
    };
    const bare = ratio(0);
    const damped = ratio(0.085);
    // Successive half-cycle peaks: e^(−πζ/√(1−ζ²)) is 0.46 at ζ 0.24 and 0.31 at ζ 0.35.
    expect(bare).toBeGreaterThan(0.42);
    expect(bare).toBeLessThan(0.5);
    expect(damped).toBeGreaterThan(0.27);
    expect(damped).toBeLessThan(0.34);
  }, 120_000);
});

describe('lesson IV.13: gain scheduling', () => {
  const lesson = LESSONS.find((l) => l.id === 'schedule')!;
  const setUp = (mod: (p: Params) => void = () => {}): Params => {
    const p = defaultParams();
    p.sim.level = 1;
    lesson.setup!(p);
    mod(p);
    return p;
  };

  it('the fixed gain tuned at 80 m/s has 30° of phase margin left at 160 m/s', () => {
    const rows = envelopeMargins(setUp(), 9);
    expect(rows[0]!.pmDeg).toBeCloseTo(54.8, 0);
    expect(rows[0]!.wc).toBeGreaterThan(4.2);
    expect(rows[8]!.pmDeg).toBeCloseTo(29.6, 0);
    expect(rows[8]!.wc).toBeGreaterThan(11);
  });

  it('scheduled on q̄ the margin stays between 55° and 78° and the crossover above 4 rad/s', () => {
    const rows = envelopeMargins(
      setUp((p) => (p.autopilot.schedule = 'qbar')),
      17,
    );
    for (const r of rows) {
      expect(r.pmDeg).toBeGreaterThan(54);
      expect(r.pmDeg).toBeLessThan(79);
      expect(r.wc).toBeGreaterThan(4);
    }
    // The crossover still rises a little with speed: the airframe gets faster.
    expect(rows[16]!.wc).toBeGreaterThan(rows[0]!.wc * 1.3);
  });

  it('no fixed gain meets the requirement; the schedule does', () => {
    for (const kTheta of [0.5, 1, 2, 3, 4])
      for (const gain of [0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1, 1.5])
        expect(
          meetsPitchSpec(
            setUp((p) => {
              p.autopilot.kTheta = kTheta;
              p.autopilot.gain = gain;
            }),
          ).ok,
        ).toBe(false);
    expect(meetsPitchSpec(setUp((p) => (p.autopilot.schedule = 'qbar'))).ok).toBe(true);
  });

  it('at 160 m/s the fixed gain leaves the short period at ζ 0.25 and 2 Hz; scheduled 0.36', () => {
    const fixed = aircraftModes(setUp(), true, 160).shortPeriod!;
    const sched = aircraftModes(
      setUp((p) => (p.autopilot.schedule = 'qbar')),
      true,
      160,
    ).shortPeriod!;
    const slow = aircraftModes(setUp(), true, 80).shortPeriod!;
    expect(slow.zeta).toBeCloseTo(0.46, 2);
    expect(fixed.zeta).toBeCloseTo(0.25, 2);
    expect(sched.zeta).toBeCloseTo(0.36, 2);
    expect(fixed.period).toBeGreaterThan(0.45);
    expect(fixed.period).toBeLessThan(0.5);
  });

  it('flown at 160 m/s the nose buzzes at the frequency of that pole', () => {
    const p = setUp((q) => {
      q.aircraft.speed = 160;
      q.autopilot.speedTarget = 160;
    });
    const sim = new Simulation(p);
    sim.schedule(1, (s) =>
      s.setParams({ ...s.params, autopilot: { ...s.params.autopilot, thetaOffset: 2 } }),
    );
    const qs: number[] = [];
    for (let k = 0; k < 3000; k++) {
      sim.step();
      if (k >= 1000) qs.push(sim.aircraft!.q);
    }
    const pk = maxima(qs);
    const period = (pk[1]!.i - pk[0]!.i) / 1000;
    const model = aircraftModes(p, true, 160).shortPeriod!.period;
    expect(Math.abs(period / model - 1)).toBeLessThan(0.05);
  }, 120_000);

  it('the margins of the pitch loop agree with the flown loop: more gain at 160 m/s goes unstable', () => {
    // The upper gain margin at 160 m/s, from the analysis, against a flight at that gain.
    const p = setUp((q) => {
      q.aircraft.speed = 160;
      q.autopilot.speedTarget = 160;
    });
    const gm = pitchLoopMargins(p, 160).gm;
    expect(gm).toBeGreaterThan(1.5);
    expect(gm).toBeLessThan(3);
    const fly = (factor: number) => {
      const q = structuredClone(p);
      q.autopilot.gain *= factor;
      const sim = new Simulation(q);
      sim.schedule(1, (s) =>
        s.setParams({ ...s.params, autopilot: { ...s.params.autopilot, thetaOffset: 0.2 } }),
      );
      let late = 0;
      for (let k = 0; k < 6000; k++) {
        sim.step();
        if (k > 5000) late = Math.max(late, Math.abs(sim.aircraft!.q));
      }
      return late;
    };
    expect(fly(gm * 0.85)).toBeLessThan(0.01);
    expect(fly(gm * 1.15)).toBeGreaterThan(0.01);
  }, 120_000);
});

describe('lesson IV.17: total energy control', () => {
  const lesson = LESSONS.find((l) => l.id === 'energy')!;
  /** The largest speed error over 45 s with the lesson's 300 m climb at 5 s, and the height gained. */
  const climb = (mod: (p: Params) => void) => {
    const p = defaultParams();
    p.sim.level = 1;
    lesson.setup!(p);
    mod(p);
    const sim = new Simulation(p);
    sim.lessonStart = structuredClone(p);
    sim.script = lesson.events!;
    sim.reset();
    let worst = 0;
    for (let k = 0; k < 45000; k++) {
      sim.step();
      worst = Math.max(worst, Math.abs(sim.aircraft!.V - sim.autopilot.vRef));
    }
    return { worst, climbed: sim.aircraft!.h - p.aircraft.altitude };
  };

  it('with the separate loops as set the speed sags 2.5 m/s in the climb', () => {
    const r = climb(() => {});
    expect(r.worst).toBeGreaterThan(2.2);
    expect(r.worst).toBeLessThan(2.8);
    expect(r.climbed).toBeGreaterThan(250);
  }, 120_000);

  it('no separate tuning up to 0.2 holds 0.3 m/s; the best is about half a metre per second', () => {
    let best = Infinity;
    for (const gain of [0.05, 0.1, 0.15, 0.2])
      for (const int of [0, 0.02, 0.05, 0.08, 0.1, 0.12, 0.15, 0.18, 0.2]) {
        const r = climb((p) => {
          p.autopilot.speedGain = gain;
          p.autopilot.speedInt = int;
        });
        best = Math.min(best, r.worst);
      }
    expect(best).toBeGreaterThan(0.4);
    expect(best).toBeLessThan(0.6);
  }, 300_000);

  it('pushed beyond that, the engine lag makes the speed loop hunt', () => {
    const r = climb((p) => {
      p.autopilot.speedGain = 0.2;
      p.autopilot.speedInt = 0.2;
    });
    expect(r.worst).toBeGreaterThan(1);
  }, 120_000);

  it('total energy control holds 0.15 m/s, and 0.4 m/s on the feedforward alone', () => {
    const tecs = climb((p) => (p.autopilot.outer = 'tecs'));
    expect(tecs.worst).toBeLessThan(0.2);
    expect(tecs.climbed).toBeGreaterThan(250);
    const ff = climb((p) => {
      p.autopilot = { ...p.autopilot, outer: 'tecs', kTP: 0, kTI: 0, kEP: 0, kEI: 0 };
    });
    expect(ff.worst).toBeGreaterThan(0.3);
    expect(ff.worst).toBeLessThan(0.5);
  }, 120_000);
});
