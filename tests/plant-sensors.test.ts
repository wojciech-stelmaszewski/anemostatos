import { describe, expect, it } from 'vitest';
import { getIn, SCHEMA, setIn, visible } from '@/engine/schema';
import { Simulation } from '@/engine/simulation';
import { qFromEuler } from '@/math/quat';
import { v3 } from '@/math/vec3';
import { idleActuation, initialState, rotorDragForce, stepDynamics } from '@/sim/dynamics';
import { defaultParams, GRAVITY, type Params } from '@/sim/params';
import { measurementOf, Sensors } from '@/sim/sensors';
import { deepMerge, migrate } from '@/store/params';

const dt = 0.001;

const hovering = (level: 1 | 3 = 1) => {
  const p = defaultParams().drone;
  const s = initialState();
  s.pos = v3(0, 2, 0);
  s.landed = false;
  const f = (p.mass * GRAVITY) / 4;
  s.motors = [f, f, f, f];
  const act = idleActuation();
  act.motorCmd = [f, f, f, f];
  stepDynamics(level, s, act, v3(), v3(), p, dt);
  return { p, s, act };
};

describe('accelerometer', () => {
  it('reads +g along body up while hovering', () => {
    const { s } = hovering();
    const acc = measurementOf(s).acc;
    expect(acc.y).toBeCloseTo(GRAVITY, 6);
    expect(Math.hypot(acc.x, acc.z)).toBeLessThan(1e-9);
  });

  it('reads ~0 in free fall', () => {
    const p = { ...defaultParams().drone, dragV: 0 };
    const s = initialState();
    s.pos = v3(0, 50, 0);
    s.landed = false;
    stepDynamics(1, s, idleActuation(), v3(), v3(), p, dt);
    expect(Math.abs(measurementOf(s).acc.y)).toBeLessThan(1e-9);
  });

  it('measures in the body frame: a tilted hovering drone still reads thrust along its own up-axis', () => {
    const s = initialState();
    s.q = qFromEuler(0.3, 0, 0);
    s.accel = v3(0, 0, 0);
    const acc = measurementOf(s).acc;
    expect(Math.hypot(acc.x, acc.y, acc.z)).toBeCloseTo(GRAVITY, 9);
    expect(acc.y).toBeCloseTo(GRAVITY * Math.cos(0.3), 9);
  });

  it('adds bias and noise', () => {
    const sensors = new Sensors(1);
    const { s } = hovering();
    sensors.record(s);
    const sp = { ...defaultParams().sensors, accBias: 0.5 };
    expect(sensors.read(sp, dt).acc.y).toBeCloseTo(GRAVITY + 0.5, 6);
  });
});

describe('position sample rate', () => {
  it('holds the position between fixes', () => {
    const sensors = new Sensors(1);
    const sp = { ...defaultParams().sensors, posRateHz: 50 };
    const s = initialState();
    const seen: number[] = [];
    for (let k = 0; k < 60; k++) {
      s.pos = v3(0, k * 0.001, 0);
      sensors.record(s);
      seen.push(sensors.read(sp, dt, k * dt).pos.y);
    }
    // 50 Hz → new value every 20 steps.
    expect(new Set(seen).size).toBe(3);
    expect(seen[19]).toBe(seen[0]);
    expect(seen[20]).toBeCloseTo(0.02, 9);
  });
});

describe('motor feedback', () => {
  it('reports actual motor thrust, or null when disabled', () => {
    const sensors = new Sensors(1);
    const { s } = hovering();
    sensors.record(s);
    const on = sensors.read(defaultParams().sensors, dt);
    expect(on.motors).toEqual(s.motors);
    const off = sensors.read({ ...defaultParams().sensors, motorFeedback: false }, dt);
    expect(off.motors).toBeNull();
  });
});

describe('faults and aerodynamics', () => {
  it('a motor at 50 % efficiency settles at half the commanded thrust', () => {
    const p = {
      ...defaultParams().drone,
      motorEfficiency: [0.5, 1, 1, 1] as [number, number, number, number],
    };
    const s = initialState();
    const act = idleActuation();
    act.motorCmd = [4, 4, 4, 4];
    for (let k = 0; k < 500; k++) stepDynamics(1, s, act, v3(), v3(), p, dt);
    expect(s.motors[0]).toBeCloseTo(2, 3);
    expect(s.motors[1]).toBeCloseTo(4, 3);
  });

  it('rotor drag opposes in-plane air velocity and ignores motion along the thrust axis', () => {
    const s = initialState();
    s.vel = v3(2, 3, 0);
    const f = rotorDragForce(s, v3(), 10, 0.01);
    expect(f.x).toBeCloseTo(-0.2, 9);
    expect(f.y).toBeCloseTo(0, 9);
  });
});

describe('true disturbance channel', () => {
  const calm = (): Params => {
    const p = defaultParams();
    p.wind.enabled = false;
    p.drone.dragV = 0;
    return p;
  };

  it('is the unmodelled weight when the assumed mass is wrong', () => {
    const p = calm();
    p.control.model.mass = 0.8;
    const sim = new Simulation(p);
    for (let k = 0; k < 15000; k++) sim.step();
    // Hovering: m̂·0 − m·g + m̂·g = (m̂ − m)·g.
    expect(sim.disturbance().y).toBeCloseTo((0.8 - 1) * GRAVITY, 1);
  });

  it('is ~0 when the model is right and nothing pushes', () => {
    const sim = new Simulation(calm());
    for (let k = 0; k < 15000; k++) sim.step();
    expect(Math.abs(sim.disturbance().y)).toBeLessThan(0.05);
  });
});

describe('parameters', () => {
  it('setIn copies arrays instead of turning them into objects', () => {
    const p = setIn(defaultParams(), 'drone.motorEfficiency.2', 0.6);
    expect(Array.isArray(p.drone.motorEfficiency)).toBe(true);
    expect(p.drone.motorEfficiency).toEqual([1, 1, 0.6, 1]);
  });

  it('old share links with control.massEstimate still load', () => {
    const merged = deepMerge(
      defaultParams(),
      migrate({ control: { massEstimate: 1.3 } }),
    ) as Params;
    expect(merged.control.model.mass).toBe(1.3);
    expect(merged.control.model.inertiaScale).toBe(1);
  });
});

describe('schema', () => {
  it('every field points at an existing parameter of the right type', () => {
    const p = defaultParams();
    for (const g of SCHEMA)
      for (const f of g.fields) {
        const v = getIn(p, f.path);
        const expected = f.kind === 'number' ? 'number' : f.kind === 'bool' ? 'boolean' : 'string';
        expect(typeof v, f.path).toBe(expected);
      }
  });

  it('PID groups disappear when another controller is selected', () => {
    const p = defaultParams();
    const alt = SCHEMA.find((g) => g.id === 'alt')!;
    expect(visible(alt, p)).toBe(true);
    (p.control.l1 as { kind: string }).kind = 'lqr';
    expect(visible(alt, p)).toBe(false);
  });
});
