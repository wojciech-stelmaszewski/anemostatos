import { describe, expect, it } from 'vitest';
import { steerCmg } from '@/control/cmg';
import { profileTime } from '@/control/satellite';
import { Simulation } from '@/engine/simulation';
import { LESSONS } from '@/lessons/lessons';
import { defaultParams, type Params } from '@/sim/params';
import {
  cmgJacobian,
  cmgMomentum,
  envelopeX,
  internalX,
  parkedGimbals,
  type CmgParams,
} from '@/sim/vehicles/cmg';
import {
  attitudeError,
  errorAngle,
  inertialMomentum,
  targetAttitude,
} from '@/sim/vehicles/satellite';

const lesson = LESSONS.find((l) => l.id === 'cmg')!;
const setup = (mod: (p: Params) => void = () => {}): Params => {
  const p = defaultParams();
  p.sim.level = lesson.level;
  lesson.setup!(p);
  mod(p);
  return p;
};
const RAD = Math.PI / 180;
const cmg = (): CmgParams => defaultParams().satellite.cmg;

/** Take momentum along `axis` at `rate` N·m·s/s with the pseudoinverse until m < 1e-3 or 30 s. */
const sweep = (d0: number[], axis: number[], rate = 0.12) => {
  const c = cmg();
  let d = [...d0];
  let least = Infinity;
  for (let k = 0; k < 30000; k++) {
    const { rates, measure } = steerCmg(
      d,
      axis.map((a) => a * rate),
      c,
      k * 0.001,
    );
    least = Math.min(least, measure);
    if (measure < 1e-3) break;
    d = d.map((v, i) => v + rates[i]! * 0.001);
    if (Math.abs(cmgMomentum(d, c)[0]) > 2.9 || Math.abs(cmgMomentum(d, c)[1]) > 2.9) break;
  }
  return { h: cmgMomentum(d, c), least };
};

/** Fly the lesson for `seconds`: the least singularity measure, the most momentum along x, when it settled. */
const fly = (mod: (p: Params) => void, seconds = 20) => {
  const sim = new Simulation(setup(mod));
  let least = Infinity;
  let most = 0;
  let settled = NaN;
  let worstTracking = 0;
  const end = profileTime(sim.params.satellite);
  for (let k = 0; k < seconds * 1000; k++) {
    sim.step();
    const m = sim.satCtl.cmgMeasure;
    if (Number.isFinite(m)) least = Math.min(least, m);
    most = Math.max(most, Math.abs(sim.satellite!.h.x));
    const e =
      errorAngle(attitudeError(sim.satellite!.q, targetAttitude(sim.params.satellite), true)) / RAD;
    if (e > 0.1) settled = NaN;
    else if (Number.isNaN(settled)) settled = sim.t;
    if (sim.t < end) worstTracking = Math.max(worstTracking, errorAngle(sim.satCtl.error) / RAD);
  }
  return { least, most, settled, worstTracking, sim };
};

describe('control-moment gyros: the pyramid', () => {
  it('stores no momentum at δ = 0 or anywhere on the parked family (s, −s, s, −s)', () => {
    const c = cmg();
    for (const s of [0, 30, 60, 120, -75]) {
      const h = cmgMomentum(parkedGimbals({ ...c, parkDeg: s }), c);
      for (const v of h) expect(Math.abs(v)).toBeLessThan(1e-12);
    }
  });

  it('A is the derivative of the momentum', () => {
    const c = cmg();
    const d = [0.3, -1.1, 2.0, 0.7];
    const A = cmgJacobian(d, c);
    for (let n = 0; n < 4; n++) {
      const p = [...d];
      const q = [...d];
      p[n]! += 1e-6;
      q[n]! -= 1e-6;
      const hp = cmgMomentum(p, c);
      const hq = cmgMomentum(q, c);
      for (let i = 0; i < 3; i++) expect(A[i]![n]!).toBeCloseTo((hp[i]! - hq[i]!) / 2e-6, 6);
    }
  });

  it('the envelope along x is h₀(2 + 2cos β) = 3.15; the stall from δ = 0 is 2h₀cos β = 1.15', () => {
    const c = cmg();
    expect(envelopeX(c)).toBeCloseTo(3.155, 2);
    expect(internalX(c)).toBeCloseTo(1.155, 2);
    // Every rotor turned toward +x: δ = (−90°, 180°, 90°, 0°).
    expect(cmgMomentum([-90 * RAD, 180 * RAD, 90 * RAD, 0], c)[0]).toBeCloseTo(envelopeX(c), 3);
  });

  it('from δ = 0 the pseudoinverse stalls at 1.15 along x and y; parked at 60° it reaches 2.9', () => {
    const stuck = sweep([0, 0, 0, 0], [1, 0, 0]);
    expect(stuck.h[0]).toBeCloseTo(internalX(cmg()), 2);
    expect(stuck.least).toBeLessThan(1e-3);
    expect(sweep([0, 0, 0, 0], [0, 1, 0]).h[1]).toBeCloseTo(internalX(cmg()), 2);
    for (const axis of [
      [1, 0, 0],
      [-1, 0, 0],
      [0, 1, 0],
    ]) {
      const free = sweep(parkedGimbals({ ...cmg(), parkDeg: 60 }), axis);
      expect(Math.hypot(...free.h)).toBeGreaterThan(2.89);
      expect(free.least).toBeGreaterThan(0.5);
    }
  });
});

describe('lesson IV.23: the singular direction', () => {
  it('the slew needs √(Θ·τ·J) = 1.77 N·m·s at its midpoint, and the gyros hold exactly that', () => {
    const p = setup((q) => (q.satellite.cmg.parkDeg = 60));
    const need = lesson.predict!.truth(p);
    expect(need).toBeCloseTo(Math.sqrt(((90 * Math.PI) / 180) * 0.5 * 4), 6);
    expect(need).toBeCloseTo(1.772, 3);
    expect(need).toBeGreaterThan(internalX(p.satellite.cmg));
    expect(need).toBeLessThan(envelopeX(p.satellite.cmg));
    const r = fly((q) => (q.satellite.cmg.parkDeg = 60));
    expect(Math.abs(r.most / need - 1)).toBeLessThan(0.01);
  }, 120_000);

  it('from δ = 0 the gimbals stall at 1.15 and the slew is late, whatever the steering', () => {
    for (const steering of ['pinv', 'sr', 'gsr'] as const) {
      const r = fly((p) => (p.satellite.cmg.steering = steering));
      expect(r.most).toBeLessThan(internalX(cmg()) + 0.01);
      expect(r.least).toBeLessThan(0.01);
      expect(Number.isNaN(r.settled) || r.settled > 12).toBe(true);
    }
  }, 240_000);

  it('parked between 45° and 75° the same pseudoinverse slews in 7 s, far from any singularity', () => {
    for (const s of [45, 60, 75]) {
      const r = fly((p) => (p.satellite.cmg.parkDeg = s));
      expect(r.least).toBeGreaterThan(0.5);
      expect(r.settled).toBeLessThan(7.5);
      expect(r.worstTracking).toBeLessThan(0.1);
    }
  }, 240_000);

  it('the gyros only move momentum around: the total is conserved in the inertial frame', () => {
    const r = fly((p) => (p.satellite.cmg.parkDeg = 60), 10);
    const H = inertialMomentum(r.sim.satellite!, r.sim.params.satellite);
    expect(Math.hypot(H.x, H.y, H.z)).toBeLessThan(1e-3);
  }, 120_000);
});
