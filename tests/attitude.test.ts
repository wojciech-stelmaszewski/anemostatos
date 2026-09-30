import { describe, expect, it } from 'vitest';
import { Simulation } from '@/engine/simulation';
import { AttitudeFilter, tiltError } from '@/estimation/attitude';
import { LESSONS } from '@/lessons/lessons';
import { qConj, qFromEuler, qIdentity, qRotate } from '@/math/quat';
import { length, sub, v3 } from '@/math/vec3';
import { defaultParams, GRAVITY, type Params } from '@/sim/params';

const DEG = Math.PI / 180;
const G = GRAVITY;

/** What an accelerometer and an ideal compass read on a body at rest with attitude q. */
const atRest = (q: ReturnType<typeof qIdentity>) => ({
  acc: qRotate(qConj(q), v3(0, G, 0)),
  north: qRotate(qConj(q), v3(1, 0, 0)),
});

describe('the attitude filter', () => {
  it('measures the angle between two up-axes', () => {
    expect(tiltError(qIdentity(), qIdentity())).toBe(0);
    expect(tiltError(qFromEuler(10 * DEG, 0, 0), qIdentity())).toBeCloseTo(10 * DEG, 9);
    // A pure heading difference is no tilt error.
    expect(tiltError(qFromEuler(0, 0, 40 * DEG), qIdentity())).toBeCloseTo(0, 9);
  });

  it('turns a wrong estimate towards the accelerometer and the compass', () => {
    const truth = qFromEuler(12 * DEG, -7 * DEG, 25 * DEG);
    const { acc, north } = atRest(truth);
    const f = new AttitudeFilter();
    const gains = { kp: 2, ki: 0, gate: 0 };
    let before = tiltError(truth, f.q);
    for (let k = 0; k < 5000; k++) {
      f.update(v3(), acc, north, gains, G, 0.001);
      if (k % 500 === 499) {
        const now = tiltError(truth, f.q);
        expect(now).toBeLessThan(before);
        before = now;
      }
    }
    expect(tiltError(truth, f.q)).toBeLessThan(0.01 * DEG);
    // Heading too: the estimated x-axis lands on the true one.
    expect(length(sub(qRotate(f.q, v3(1)), qRotate(truth, v3(1))))).toBeLessThan(1e-3);
  });

  it('a gyro bias b leaves b·τ in the complementary filter and nothing in Mahony', () => {
    const { acc, north } = atRest(qIdentity());
    const bias = v3(1 * DEG, 0, 0);
    const run = (gains: { kp: number; ki: number; gate: number }) => {
      const f = new AttitudeFilter();
      for (let k = 0; k < 60000; k++) f.update(bias, acc, north, gains, G, 0.001);
      return f;
    };
    for (const tau of [1, 4]) {
      const f = run({ kp: 1 / tau, ki: 0, gate: 0 });
      expect(tiltError(qIdentity(), f.q) / DEG).toBeCloseTo(tau, 1);
    }
    const m = run({ kp: 1, ki: 0.3, gate: 0.05 });
    expect(tiltError(qIdentity(), m.q) / DEG).toBeLessThan(0.01);
    expect(m.bias.x / DEG).toBeCloseTo(1, 2);
  });

  it('the gate shuts out a reading that is not 1 g long', () => {
    const f = new AttitudeFilter();
    const gains = { kp: 1, ki: 0.3, gate: 0.05 };
    // 1.1 g, pointing 25° off: a turn. The estimate must not follow it.
    const acc = qRotate(qFromEuler(25 * DEG, 0, 0), v3(0, 1.1 * G, 0));
    for (let k = 0; k < 5000; k++) f.update(v3(), acc, null, gains, G, 0.001);
    expect(f.trusted).toBe(false);
    expect(tiltError(qIdentity(), f.q)).toBe(0);
    for (let k = 0; k < 5000; k++) f.update(v3(), acc, null, { ...gains, gate: 0 }, G, 0.001);
    expect(tiltError(qIdentity(), f.q) / DEG).toBeGreaterThan(20);
  });
});

describe('flying on an estimated attitude', () => {
  const lesson = LESSONS.find((l) => l.id === 'attitude')!;
  /** Fly the lesson's manoeuvre and return the largest tilt error from the start of the turn on. */
  const worst = (mod?: (p: Params) => void) => {
    const p = defaultParams();
    p.sim.level = 3;
    lesson.setup!(p);
    mod?.(p);
    const sim = new Simulation(p);
    sim.script = lesson.events!;
    sim.reset();
    let err = 0;
    let excess = 0;
    for (let k = 0; k < 33000; k++) {
      sim.step();
      if (k % 5 === 0 && sim.t > 10) {
        err = Math.max(err, sim.telemetry.latest('ahrs.err'));
        // Third lap: the turn is steady.
        if (sim.t > 20 && sim.t < 25 && sim.measurement)
          excess = Math.max(excess, length(sim.measurement.acc) / G - 1);
      }
    }
    expect(sim.state.crashed).toBe(false);
    return { err, excess: excess * 100, sim };
  };

  it('with the true attitude nothing changes: the default source is the truth', () => {
    const p = defaultParams();
    p.sim.level = 3;
    const sim = new Simulation(p);
    for (let k = 0; k < 3000; k++) sim.step();
    // The reading is the state one step ago, nothing else.
    expect(tiltError(sim.state.q, sim.measurement!.q)).toBeLessThan(1e-3);
    expect(Number.isNaN(sim.telemetry.latest('ahrs.err'))).toBe(true);
  });

  it('in the turn the accelerometer points along the body, not along gravity', () => {
    const { sim } = worst((p) => (p.sensors.attitude = 'mahony'));
    // Rerun to the middle of the turn and look at the reading there.
    const again = new Simulation(sim.params);
    again.script = lesson.events!;
    again.reset();
    for (let k = 0; k < 22000; k++) again.step();
    const acc = again.measurement!.acc;
    const tilt = Math.acos(qRotate(again.state.q, v3(0, 1, 0)).y) / DEG;
    const offBody = Math.acos(acc.y / length(acc)) / DEG;
    expect(tilt).toBeGreaterThan(20);
    // A few degrees of difference are the air drag, the only other force on the drone.
    expect(offBody).toBeLessThan(tilt / 3);
  });

  it('the predicted length of the reading is what the accelerometer shows', () => {
    const predicted = lesson.predict!.truth(worst().sim.params);
    expect(predicted).toBeCloseTo(11.1, 0);
    expect(worst().excess / predicted).toBeGreaterThan(0.9);
    expect(worst().excess / predicted).toBeLessThan(1.15);
  });

  it('no time constant of the complementary filter reaches 2°', () => {
    expect(worst().err).toBeGreaterThan(15); // τ = 1 s: fooled by the turn
    for (const tau of [0.5, 2, 5, 10])
      expect(worst((p) => (p.control.ahrs.tau = tau)).err).toBeGreaterThan(7);
    // The long time constant fails on the bias alone: about b·τ.
    expect(worst((p) => (p.control.ahrs.tau = 10)).err).toBeGreaterThan(10);
  });

  it('Mahony removes the bias, and needs the gate for the turn', () => {
    const open = worst((p) => (p.sensors.attitude = 'mahony'));
    expect(open.err).toBeGreaterThan(15);
    for (const gate of [0.01, 0.02, 0.03]) {
      const gated = worst((p) => {
        p.sensors.attitude = 'mahony';
        p.control.ahrs.gate = gate;
      });
      expect(gated.err).toBeLessThan(2);
      expect(gated.sim.sensors.ahrs.bias.x / DEG).toBeCloseTo(1, 0);
    }
    // A gate just under the turn's 11 % is open while the drone speeds up and brakes…
    const loose = worst((p) => {
      p.sensors.attitude = 'mahony';
      p.control.ahrs.gate = 0.08;
    });
    expect(loose.err).toBeGreaterThan(3);
    // …and one wider than the excess lets the whole turn through.
    const wide = worst((p) => {
      p.sensors.attitude = 'mahony';
      p.control.ahrs.gate = 0.2;
    });
    expect(wide.err).toBeGreaterThan(10);
  });
});
