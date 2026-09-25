import { describe, expect, it } from 'vitest';
import { Simulation } from '@/engine/simulation';
import { LESSONS } from '@/lessons/lessons';
import { defaultParams } from '@/sim/params';

describe('lessons', () => {
  it('have unique ids', () => {
    expect(new Set(LESSONS.map((l) => l.id)).size).toBe(LESSONS.length);
  });

  for (const lesson of LESSONS) {
    it(`"${lesson.title}" starts in a flyable state`, () => {
      const p = defaultParams();
      p.sim.level = lesson.level;
      lesson.setup?.(p);
      const sim = new Simulation(p);
      for (let k = 0; k < 15000; k++) sim.step();
      expect(sim.state.crashed).toBe(false);
      expect(sim.state.pos.y).toBeGreaterThan(0.5);
    });
  }
});
