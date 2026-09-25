import { describe, expect, it } from 'vitest';
import { qFromEuler, qRotate } from '@/math/quat';
import { v3 } from '@/math/vec3';
import { FOOT_HEIGHT } from '@/sim/drone';
import { idleActuation, initialState, motorTorques, stepDynamics } from '@/sim/dynamics';
import { defaultParams, GRAVITY } from '@/sim/params';

const dt = 0.001;
const params = () => defaultParams().drone;

describe('dynamics', () => {
  it('free fall without drag matches ½·g·t²', () => {
    const p = { ...params(), dragV: 0, crashSpeed: 1e9 };
    const s = initialState();
    s.pos = v3(0, 100, 0);
    s.landed = false;
    for (let k = 0; k < 1000; k++) stepDynamics(1, s, idleActuation(), v3(), v3(), p, dt);
    expect(100 - s.pos.y).toBeCloseTo(0.5 * GRAVITY * 1, 1);
  });

  it('hovers when total thrust equals weight', () => {
    const p = params();
    const s = initialState();
    s.pos = v3(0, 2, 0);
    const f = (p.mass * GRAVITY) / 4;
    s.motors = [f, f, f, f];
    const act = idleActuation();
    act.motorCmd = [f, f, f, f];
    for (let k = 0; k < 2000; k++) stepDynamics(3, s, act, v3(), v3(), p, dt);
    expect(s.pos.y).toBeCloseTo(2, 6);
    expect(Math.hypot(s.omega.x, s.omega.y, s.omega.z)).toBeLessThan(1e-9);
  });

  it('motor thrust follows a first-order lag with time constant τ', () => {
    const p = params();
    const s = initialState();
    const act = idleActuation();
    act.motorCmd = [4, 4, 4, 4];
    const steps = Math.round(p.motorTau / dt);
    for (let k = 0; k < steps; k++) stepDynamics(1, s, act, v3(), v3(), p, dt);
    expect(s.motors[0] / 4).toBeCloseTo(1 - Math.exp(-1), 2);
  });

  it('reaches drag terminal velocity in free fall', () => {
    const p = { ...params(), crashSpeed: 1e9 };
    const s = initialState();
    s.pos = v3(0, 1000, 0);
    for (let k = 0; k < 20000; k++) stepDynamics(1, s, idleActuation(), v3(), v3(), p, dt);
    expect(-s.vel.y).toBeCloseTo(Math.sqrt((p.mass * GRAVITY) / p.dragV), 2);
  });

  it('an updraft pushes a hovering drone up', () => {
    const p = params();
    const s = initialState();
    s.pos = v3(0, 2, 0);
    const f = (p.mass * GRAVITY) / 4;
    s.motors = [f, f, f, f];
    const act = idleActuation();
    act.motorCmd = [f, f, f, f];
    stepDynamics(1, s, act, v3(0, 4, 0), v3(), p, dt);
    expect(s.vel.y).toBeGreaterThan(0);
  });

  it('lands softly and crashes when hitting the ground too fast', () => {
    const p = params();
    const soft = initialState();
    soft.pos = v3(0, FOOT_HEIGHT + 0.001, 0);
    soft.vel = v3(0, -1, 0);
    stepDynamics(1, soft, idleActuation(), v3(), v3(), p, dt);
    expect(soft.crashed).toBe(false);
    expect(soft.pos.y).toBe(FOOT_HEIGHT);

    const hard = initialState();
    hard.pos = v3(0, FOOT_HEIGHT + 0.001, 0);
    hard.vel = v3(0, -5, 0);
    stepDynamics(1, hard, idleActuation(), v3(), v3(), p, dt);
    expect(hard.crashed).toBe(true);
  });

  describe('torque sign conventions (X frame, +x fwd, +y up, +z right)', () => {
    const p = params();
    it('stronger left motors (M1, M4) → positive roll torque → rolls right', () => {
      expect(motorTorques([2, 1, 1, 2], p).x).toBeGreaterThan(0);
      // Positive rotation about +x lowers the right side (+z).
      expect(qRotate(qFromEuler(0.1, 0, 0), v3(0, 0, 1)).y).toBeLessThan(0);
    });
    it('stronger front motors (M1, M2) → positive pitch torque → nose up', () => {
      expect(motorTorques([2, 2, 1, 1], p).z).toBeGreaterThan(0);
    });
    it('stronger CW motors (M1, M3) → positive yaw torque', () => {
      expect(motorTorques([2, 1, 2, 1], p).y).toBeGreaterThan(0);
    });
    it('equal thrust → no torque', () => {
      const t = motorTorques([3, 3, 3, 3], p);
      expect(Math.abs(t.x) + Math.abs(t.y) + Math.abs(t.z)).toBeLessThan(1e-12);
    });
  });
});
