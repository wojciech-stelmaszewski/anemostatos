import { existsSync, readFileSync, writeFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import { Simulation } from '@/engine/simulation';
import { defaultParams, type Params } from '@/sim/params';

/**
 * Behavioural regression: the Part I simulator must stay bit-for-bit (well, 1e-9) identical while
 * Part II refactors the controller plumbing. Regenerate deliberately with UPDATE_BASELINE=1.
 */
const FIXTURE = new URL('./fixtures/baseline.json', import.meta.url);

const scenarios: Record<string, (p: Params) => void> = {
  l1: () => {},
  l1Noisy: (p) => {
    p.sensors.posNoise = 0.02;
    p.sensors.delayMs = 20;
    p.control.alt = { ...p.control.alt, antiWindup: 'back-calculation' };
  },
  l2: (p) => {
    p.sim.level = 2;
    p.setpoint.x = 2;
  },
  l3: (p) => {
    p.sim.level = 3;
    p.setpoint.x = 2;
    p.setpoint.z = -1;
    p.setpoint.yawDeg = 30;
    p.sensors.gyroNoise = 2;
  },
};

const record = (setup: (p: Params) => void): number[][] => {
  const p = defaultParams();
  setup(p);
  const sim = new Simulation(p);
  const rows: number[][] = [];
  for (let k = 1; k <= 20000; k++) {
    sim.step();
    if (k % 100 === 0) {
      const s = sim.state;
      rows.push([s.pos.x, s.pos.y, s.pos.z, s.vel.y, s.q.w, s.q.x, s.q.y, s.q.z, ...s.motors]);
    }
  }
  return rows;
};

describe('regression against the Part I baseline', () => {
  const current = Object.fromEntries(
    Object.entries(scenarios).map(([k, setup]) => [k, record(setup)]),
  );
  if (process.env.UPDATE_BASELINE || !existsSync(FIXTURE)) {
    writeFileSync(FIXTURE, JSON.stringify(current));
  }
  const baseline = JSON.parse(readFileSync(FIXTURE, 'utf8')) as Record<string, number[][]>;

  for (const name of Object.keys(scenarios)) {
    it(`${name} matches`, () => {
      const a = current[name]!;
      const b = baseline[name]!;
      expect(a.length).toBe(b.length);
      let worst = 0;
      for (let r = 0; r < a.length; r++)
        for (let c = 0; c < a[r]!.length; c++)
          worst = Math.max(worst, Math.abs(a[r]![c]! - b[r]![c]!));
      expect(worst).toBeLessThan(1e-9);
    });
  }
});
