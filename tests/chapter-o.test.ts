import { describe, expect, it } from 'vitest';
import { reconstructAltitude } from '@/analysis/smoother';
import { RendezvousMpc } from '@/control/rendezvous';
import { Simulation } from '@/engine/simulation';
import { ParticleFilter } from '@/estimation/particle';
import { middleSigmaRatio, smoothTrack } from '@/estimation/rts';
import { LESSONS } from '@/lessons/lessons';
import { defaultParams, type Params } from '@/sim/params';
import { BEACONS } from '@/sim/sensors';
import {
  cwDiscrete,
  driftPerOrbit,
  initialChaser,
  meanMotion,
  stepChaser,
} from '@/sim/vehicles/chaser';

/** Set up a lesson as the panel does (with its scripted events), change it, and fly. */
const fly = (
  id: string,
  mod: (p: Params) => void,
  seconds: number,
  until?: (s: Simulation) => boolean,
) => {
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
    if (until && k % 100 === 0 && until(sim)) break;
  }
  return sim;
};

describe('lesson IV.25: rendezvous', () => {
  const p = defaultParams().rendezvous;
  const n = meanMotion(p);

  it('the discrete model is the Clohessy–Wiltshire solution', () => {
    const dv = 0.05;
    const t = 1234;
    const { a } = cwDiscrete(n, t);
    const z = a.map((r) => r[3]! * dv); // from rest at the origin with ẏ = dv
    expect(z[0]).toBeCloseTo(((2 * dv) / n) * (1 - Math.cos(n * t)), 6);
    expect(z[1]).toBeCloseTo(((4 * dv) / n) * Math.sin(n * t) - 3 * dv * t, 6);
  });

  it('a forward burn of 5 cm/s leaves the chaser 833 m further behind after one orbit', () => {
    expect(-driftPerOrbit(0.05, n)).toBeCloseTo(833.4, 0);
    const s = initialChaser(p);
    let top = 0;
    const dt = 0.12;
    while (s.t < p.periodMin * 60 - 1e-9) {
      stepChaser(s, { ax: 0, ay: 0 }, Math.min(dt, p.periodMin * 60 - s.t), p);
      top = Math.max(top, s.x);
    }
    expect(s.y - p.startY).toBeCloseTo(driftPerOrbit(p.burn, n), 1);
    expect(Math.abs(s.x)).toBeLessThan(0.01);
    // It climbs to 4Δv/n first: faster is higher, and higher is slower.
    expect(top).toBeCloseTo((4 * p.burn) / n, 0);
  });

  it('the MPC stays within the thruster limit', () => {
    const q = { ...p, control: 'mpc' as const };
    const s = initialChaser(q);
    const mpc = new RendezvousMpc();
    for (let k = 0; k < 2000; k++) {
      const u = mpc.tick(s, q);
      expect(Math.abs(u.ax)).toBeLessThanOrEqual(q.accelMax + 1e-12);
      expect(Math.abs(u.ay)).toBeLessThanOrEqual(q.accelMax + 1e-12);
      stepChaser(s, u, 0.12, q);
    }
  });

  const docking = (mod: (p: Params) => void) => {
    const sim = fly('rendezvous', mod, 60, (s) => s.chaser!.docked || s.chaser!.hit);
    return sim.chaser!;
  };

  it('coasting never docks', () => {
    const c = docking(() => {});
    expect(c.docked || c.hit).toBe(false);
  });

  it('the MPC docks gently but, with a small thrust weight, far over the Δv budget', () => {
    const c = docking((q) => (q.rendezvous.control = 'mpc'));
    expect(c.docked).toBe(true);
    expect(c.contactSpeed).toBeLessThan(0.05);
    expect(c.dv).toBeGreaterThan(1);
  }, 120_000);

  it('a large thrust weight docks within an orbit on less than 0.3 m/s', () => {
    const c = docking((q) => {
      q.rendezvous.control = 'mpc';
      q.rendezvous.rAcc = 1e6;
      q.rendezvous.burn = 0;
    });
    expect(c.docked).toBe(true);
    expect(c.t).toBeLessThan(q0().periodMin * 60);
    expect(c.dv).toBeLessThan(0.3);
  }, 120_000);

  it('with the naive burn still in, a weight of 10⁷ takes longer than an orbit', () => {
    const c = docking((q) => {
      q.rendezvous.control = 'mpc';
      q.rendezvous.rAcc = 1e7;
      q.rendezvous.warp = 300;
    });
    expect(c.docked).toBe(true);
    expect(c.t).toBeGreaterThan(q0().periodMin * 60);
  }, 120_000);
});
const q0 = () => defaultParams().rendezvous;

describe('lesson IV.28: particle filter', () => {
  it('while the drone hovers the cloud is a ring: the mean near the beacon, σ about the range', () => {
    const sim = fly('particle', (p) => (p.setpoint.z = 0), 14);
    const f = sim.sensors.rangeFilter as ParticleFilter;
    const s = sim.state.pos;
    const range = Math.hypot(s.x - BEACONS[0]!.x, s.z - BEACONS[0]!.z);
    expect(f.sigmaPos).toBeGreaterThan(0.8 * range);
    expect(f.sigmaPos).toBeLessThan(1.15 * range);
    expect(Math.hypot(f.x[0]! - BEACONS[0]!.x, f.x[1]! - BEACONS[0]!.z)).toBeLessThan(0.5 * range);
  }, 120_000);

  it('a straight leg along z leaves two guesses, the drone and its mirror image across x = 10', () => {
    const sim = fly('particle', () => {}, 24);
    const f = sim.sensors.rangeFilter as ParticleFilter;
    const [a, b] = f.halves(BEACONS[0]!, { x: 0, z: 1 });
    const near = a!.x < BEACONS[0]!.x ? a! : b!;
    const ghost = a!.x < BEACONS[0]!.x ? b! : a!;
    expect(near.w).toBeGreaterThan(0.15);
    expect(ghost.w).toBeGreaterThan(0.15);
    expect(Math.abs(near.x - sim.state.pos.x)).toBeLessThan(1.5);
    expect(Math.abs(ghost.x - (2 * BEACONS[0]!.x - sim.state.pos.x))).toBeLessThan(1.5);
  }, 120_000);

  it('a turn leaves one guess: within 2 m, and the cloud narrower than 2 m', () => {
    for (const seed of [1, 2, 3]) {
      const sim = fly(
        'particle',
        (p) => {
          p.sim.seed = seed;
          p.setpoint.profile = 'circle';
          p.setpoint.profileAmplitude = 1;
          p.setpoint.profilePeriod = 20;
        },
        40,
      );
      const w = sim.telemetry.window(['rng.err', 'rng.sigma2'], 30);
      expect(Math.max(...w.series[0]!)).toBeLessThan(2);
      expect(Math.max(...w.series[1]!) / 2).toBeLessThan(2);
    }
  }, 300_000);
});

describe('lesson IV.29: the RTS smoother', () => {
  it('in the middle of a record the smoother’s σ is half the filter’s for a slow model', () => {
    expect(middleSigmaRatio(0.05, 0.01, 0.005)).toBeCloseTo(0.5, 1);
    expect(Math.abs(middleSigmaRatio(0.05, 0.1, 0.005) - 0.5)).toBeLessThan(0.02);
    // A fast model gains less from the future.
    expect(middleSigmaRatio(0.05, 100, 0.005)).toBeGreaterThan(0.6);
  });

  it('the smoother’s σ is below the filter’s everywhere and equal at the last sample', () => {
    const t = Array.from({ length: 500 }, (_, i) => i * 0.01);
    const z = t.map((v) => Math.sin(v));
    const tr = smoothTrack(t, z, 0.05, 1);
    tr.smoothSigma.forEach((s, i) => expect(s).toBeLessThanOrEqual(tr.filterSigma[i]! + 1e-12));
    expect(tr.smoothSigma.at(-1)).toBeCloseTo(tr.filterSigma.at(-1)!, 12);
  });

  const recon = (accel: number) => {
    const sim = fly('smoother', (p) => (p.smoother.accelSigma = accel), 30);
    return reconstructAltitude(sim)!;
  };

  it('the starting model follows the noise: the smoother is over 1 cm RMS', () => {
    const r = recon(100);
    expect(r.rmsS).toBeGreaterThan(0.01);
    expect(r.rmsS / r.rmsF).toBeGreaterThan(0.55);
  });

  it('a slower model halves the filter’s error, under 1 cm', () => {
    const r = recon(1);
    expect(r.rmsS).toBeLessThan(0.01);
    expect(r.rmsS / r.rmsF).toBeLessThan(0.55);
  });

  it('too slow a model lags behind the steps: the filter is worse than at 1', () => {
    expect(recon(0.01).rmsF).toBeGreaterThan(recon(1).rmsF);
  });
});
