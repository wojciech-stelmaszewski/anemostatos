import { describe, expect, it } from 'vitest';
import { attitudeRates, flatnessRates } from '@/control/geometric';
import { profileDemand, profileOffset } from '@/engine/reference';
import { Simulation } from '@/engine/simulation';
import { evalSpline, minSnap } from '@/math/poly';
import { qFromAxisAngle, qFromEuler, qMul, type Quat } from '@/math/quat';
import { v3, type Vec3 } from '@/math/vec3';
import { defaultParams, type Params } from '@/sim/params';

describe('minimum-snap spline', () => {
  const sp = minSnap([0, 2, -1, 3], [1, 1.5, 2]);

  it('passes through the waypoints and starts and ends at rest', () => {
    expect(evalSpline(sp, 0)[0]).toBeCloseTo(0, 9);
    expect(evalSpline(sp, 1)[0]).toBeCloseTo(2, 9);
    expect(evalSpline(sp, 2.5)[0]).toBeCloseTo(-1, 9);
    expect(evalSpline(sp, 4.5)[0]).toBeCloseTo(3, 9);
    for (const t of [0, 4.5])
      for (let d = 1; d <= 3; d++) expect(evalSpline(sp, t)[d]).toBeCloseTo(0, 8);
  });

  it('is smooth across the waypoints (C⁶; checked up to snap)', () => {
    for (const knot of [1, 2.5]) {
      const a = evalSpline(sp, knot - 1e-9);
      const b = evalSpline(sp, knot + 1e-9);
      for (let d = 0; d <= 4; d++) expect(a[d]).toBeCloseTo(b[d]!, 4);
    }
  });

  it('has derivatives that match finite differences', () => {
    const h = 1e-5;
    for (const t of [0.3, 1.7, 3.9]) {
      const [p0, v0, a0] = evalSpline(sp, t);
      const [p1, v1] = evalSpline(sp, t + h);
      expect((p1 - p0) / h).toBeCloseTo(v0, 3);
      expect((v1 - v0) / h).toBeCloseTo(a0, 2);
    }
  });
});

describe('reference profiles', () => {
  const params = (profile: Params['setpoint']['profile']) => {
    const p = defaultParams();
    p.setpoint.profile = profile;
    p.setpoint.profileAmplitude = 1.5;
    p.setpoint.profilePeriod = 8;
    return p;
  };
  const diff = (f: (t: number) => Vec3, t: number, h = 1e-4) => {
    const a = f(t - h);
    const b = f(t + h);
    return v3((b.x - a.x) / (2 * h), (b.y - a.y) / (2 * h), (b.z - a.z) / (2 * h));
  };

  for (const profile of ['circle', 'figure8', 'minsnap'] as const) {
    it(`${profile}: derivatives are consistent and it starts at the base point`, () => {
      const p = params(profile);
      const r = (t: number) => profileOffset(p, 3, t, 0);
      expect(Math.hypot(r(0).pos.x, r(0).pos.z)).toBeLessThan(1e-9);
      for (const t of [0.7, 2.9, 5.3]) {
        const keys = ['pos', 'vel', 'acc', 'jerk'] as const;
        for (let d = 0; d < 3; d++) {
          const num = diff((s) => r(s)[keys[d]!], t);
          const ana = r(t)[keys[d + 1]!];
          expect(num.x).toBeCloseTo(ana.x, 2);
          expect(num.z).toBeCloseTo(ana.z, 2);
        }
      }
    });
  }

  it('keeps the Part I one-axis profiles unchanged', () => {
    const p = params('square');
    p.setpoint.profileAmplitude = 0.5;
    expect(profileOffset(p, 1, 1, 0).pos.y).toBe(0.5);
    expect(profileOffset(p, 1, 5, 0).pos.y).toBe(-0.5);
    p.setpoint.profile = 'triangle';
    expect(profileOffset(p, 1, 2, 0).pos.y).toBeCloseTo(0.5 * (4 * 0.25 - 1), 12);
  });

  it('reports what a trajectory demands', () => {
    const p = params('circle');
    p.setpoint.profileAmplitude = 2;
    const w = (2 * Math.PI) / 8;
    const d = profileDemand(p);
    expect(d.speed).toBeCloseTo(2 * w, 2);
    expect(d.acc).toBeCloseTo(2 * w * w, 2);
  });
});

describe('attitude laws', () => {
  const rates = (law: 'quaternion' | 'tilt' | 'euler', q: Quat, sp: Quat) =>
    attitudeRates(law, q, sp, 8, 3).raw;

  it('agree for small errors', () => {
    const q = qFromEuler(0.02, -0.03, 0.01);
    const sp = qFromEuler(0, 0, 0);
    const a = rates('quaternion', q, sp);
    for (const law of ['tilt', 'euler'] as const) {
      const b = rates(law, q, sp);
      expect(b.x).toBeCloseTo(a.x, 0);
      expect(b.z).toBeCloseTo(a.z, 0);
    }
  });

  it('tilt-prioritised: a pure yaw error commands no roll or pitch', () => {
    const q = qFromAxisAngle(v3(0, 1, 0), 0);
    const sp = qFromAxisAngle(v3(0, 1, 0), 2.5);
    const r = rates('tilt', q, sp);
    expect(Math.abs(r.x) + Math.abs(r.z)).toBeLessThan(1e-9);
    expect(r.y).toBeCloseTo((3 * 2.5 * 180) / Math.PI, 6);
  });

  it('recover from a 170° flip; the naive Euler law loses the most altitude', () => {
    const loss = (law: 'quaternion' | 'tilt' | 'euler') => {
      const p = defaultParams();
      p.sim.level = 3;
      p.wind.enabled = false;
      p.setpoint.y = 6;
      p.control.l3.attitude = law;
      const sim = new Simulation(p);
      sim.schedule(10, (s) => {
        s.state.q = qMul(qFromAxisAngle(v3(1, 0, 1), (170 * Math.PI) / 180), s.state.q);
        s.state.omega = v3();
      });
      let low = 99;
      for (let k = 0; k < 16000; k++) {
        sim.step();
        if (sim.t > 10) low = Math.min(low, sim.state.pos.y);
      }
      expect(sim.state.crashed).toBe(false);
      return 6 - low;
    };
    const euler = loss('euler');
    expect(loss('quaternion')).toBeLessThan(0.7 * euler);
    expect(loss('tilt')).toBeLessThan(0.7 * euler);
  });
});

describe('geometric tracking and flatness feedforward', () => {
  const trackingError = (ff: boolean) => {
    const p = defaultParams();
    p.sim.level = 3;
    p.wind.enabled = false;
    p.setpoint.y = 3;
    p.setpoint.profile = 'figure8';
    p.setpoint.profileAmplitude = 2;
    p.setpoint.profilePeriod = 8.9;
    p.control.l3.outer = 'geometric';
    p.control.geometric.feedforward = ff;
    const sim = new Simulation(p);
    let sq = 0;
    let n = 0;
    for (let k = 0; k < 35000; k++) {
      sim.step();
      if (sim.t > 25) {
        const e = Math.hypot(
          sim.state.pos.x - sim.setpoint.x,
          sim.state.pos.y - sim.setpoint.y,
          sim.state.pos.z - sim.setpoint.z,
        );
        sq += e * e;
        n++;
      }
    }
    return Math.sqrt(sq / n);
  };

  it('tracks a 2 m/s figure-8 within 5 cm with feedforward, and not without', () => {
    expect(trackingError(true)).toBeLessThan(0.05);
    expect(trackingError(false)).toBeGreaterThan(0.3);
  });

  it('flatness body rates vanish for constant acceleration and follow the jerk', () => {
    const f = v3(2, 9.81, 0);
    const q = qFromAxisAngle(v3(0, 0, 1), 0);
    expect(flatnessRates(f, v3(), 1, q)).toEqual(v3());
    const w = flatnessRates(v3(0, 9.81, 0), v3(1, 0, 0), 1, q);
    // Thrust tipping towards +x means rotating about −z (pitch), at j/g rad/s.
    expect(w.z).toBeCloseTo((-1 / 9.81) * (180 / Math.PI), 6);
    expect(Math.abs(w.x)).toBeLessThan(1e-9);
  });
});
