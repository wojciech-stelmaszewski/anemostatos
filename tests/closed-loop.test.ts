import { describe, expect, it } from 'vitest';
import { mix } from '@/control/mixer';
import { v3 } from '@/math/vec3';
import { Simulation } from '@/engine/simulation';
import { defaultParams, GRAVITY, type Params } from '@/sim/params';
import { motorTorques } from '@/sim/dynamics';

const calm = (level: 1 | 2 | 3 = 1): Params => {
  const p = defaultParams();
  p.sim.level = level;
  p.wind.enabled = false;
  return p;
};

const run = (sim: Simulation, seconds: number) => {
  for (let k = 0; k < seconds * 1000; k++) sim.step();
};

describe('closed loop — L1 altitude', () => {
  it('takes off and settles at the setpoint with PID + feedforward', () => {
    const sim = new Simulation(calm());
    run(sim, 10);
    expect(sim.state.pos.y).toBeCloseTo(2, 2);
    expect(sim.state.crashed).toBe(false);
  });

  it('P-only without feedforward droops by m·g/Kp (docs/pid-primer.md §4)', () => {
    const p = calm();
    p.control.feedforward = false;
    p.control.alt = { ...p.control.alt, kp: 20, iOn: false, kd: 8 };
    const sim = new Simulation(p);
    run(sim, 15);
    const expected = (p.drone.mass * GRAVITY) / 20;
    expect(2 - sim.state.pos.y).toBeCloseTo(expected, 2);
  });

  it('the I term removes the droop', () => {
    const p = calm();
    p.control.feedforward = false;
    p.control.alt = { ...p.control.alt, kp: 20, ki: 10, kd: 8 };
    const sim = new Simulation(p);
    run(sim, 30);
    expect(sim.state.pos.y).toBeCloseTo(2, 2);
  });

  it('recovers from gusts', () => {
    const p = defaultParams();
    const sim = new Simulation(p);
    run(sim, 60);
    expect(sim.state.crashed).toBe(false);
    expect(Math.abs(sim.state.pos.y - 2)).toBeLessThan(1);
  });

  it('same seed → identical trajectory', () => {
    const a = new Simulation(defaultParams());
    const b = new Simulation(defaultParams());
    run(a, 5);
    run(b, 5);
    expect(a.state.pos).toEqual(b.state.pos);
  });
});

describe('closed loop — L2 point mass', () => {
  it('flies to a 3D setpoint', () => {
    const p = calm(2);
    const sim = new Simulation(p);
    sim.setTarget(v3(3, 2, -2));
    run(sim, 15);
    expect(sim.state.pos.x).toBeCloseTo(3, 1);
    expect(sim.state.pos.y).toBeCloseTo(2, 1);
    expect(sim.state.pos.z).toBeCloseTo(-2, 1);
  });
});

describe('mixer', () => {
  const p = defaultParams().drone;
  it('round-trips thrust and torques when not saturated', () => {
    const r = mix(10, 0.1, 0.02, -0.15, p.torqueCoeff, p.maxMotorThrust);
    expect(r.saturated).toBe(false);
    const total = r.motors.reduce((a, b) => a + b, 0);
    const t = motorTorques(r.motors, p);
    expect(total).toBeCloseTo(10, 9);
    expect(t.x).toBeCloseTo(0.1, 9);
    expect(t.y).toBeCloseTo(0.02, 9);
    expect(t.z).toBeCloseTo(-0.15, 9);
  });

  it('keeps motors in range and sacrifices yaw first', () => {
    const r = mix(20, 0.5, 0.2, 0, p.torqueCoeff, p.maxMotorThrust);
    for (const f of r.motors) {
      expect(f).toBeGreaterThanOrEqual(0);
      expect(f).toBeLessThanOrEqual(p.maxMotorThrust);
    }
    expect(r.saturated).toBe(true);
    expect(r.yawScale).toBeLessThan(1);
  });
});

describe('closed loop — L3 quadrotor cascade', () => {
  it('takes off and hovers at the setpoint', () => {
    const sim = new Simulation(calm(3));
    run(sim, 10);
    expect(sim.state.crashed).toBe(false);
    expect(sim.state.pos.y).toBeCloseTo(2, 1);
    expect(Math.abs(sim.state.pos.x)).toBeLessThan(0.05);
  });

  it('flies to a horizontal setpoint by tilting', () => {
    const sim = new Simulation(calm(3));
    run(sim, 5);
    sim.setTarget(v3(4, 2, 3));
    let maxTilt = 0;
    for (let k = 0; k < 12000; k++) {
      sim.step();
      const up = sim.state.q;
      maxTilt = Math.max(maxTilt, Math.abs(up.x) + Math.abs(up.z));
    }
    expect(sim.state.crashed).toBe(false);
    expect(maxTilt).toBeGreaterThan(0.05);
    expect(sim.state.pos.x).toBeCloseTo(4, 1);
    expect(sim.state.pos.z).toBeCloseTo(3, 1);
    expect(sim.state.pos.y).toBeCloseTo(2, 1);
  });

  it('holds a yaw setpoint', () => {
    const p = calm(3);
    p.setpoint.yawDeg = 90;
    const sim = new Simulation(p);
    run(sim, 10);
    const e = Math.atan2(2 * (sim.state.q.w * sim.state.q.y), 1 - 2 * sim.state.q.y ** 2);
    expect(e).toBeCloseTo(Math.PI / 2, 1);
  });

  it('survives gusty wind', () => {
    const p = defaultParams();
    p.sim.level = 3;
    p.wind.gustVertical = 0.2;
    const sim = new Simulation(p);
    run(sim, 60);
    expect(sim.state.crashed).toBe(false);
    expect(Math.hypot(sim.state.pos.x, sim.state.pos.y - 2, sim.state.pos.z)).toBeLessThan(1.5);
  });
});
