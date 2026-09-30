import { describe, expect, it } from 'vitest';
import { analyzeLastStep } from '@/engine/metrics';
import { Simulation } from '@/engine/simulation';
import { LESSONS } from '@/lessons/lessons';
import type { Lesson } from '@/lessons/types';
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

/** Run a lesson like the panel does and report whether its goal was ever met. */
const reachesGoal = (lesson: Lesson, withSolution: boolean, seconds = 45): boolean => {
  const p = defaultParams();
  p.sim.level = lesson.level;
  lesson.setup?.(p);
  if (withSolution) lesson.solution?.(p);
  const sim = new Simulation(p);
  sim.script = lesson.events ?? null;
  sim.reset();
  lesson.onStart?.(sim);
  const loop = lesson.loop ?? 'alt';
  for (let k = 1; k <= seconds * 1000; k++) {
    sim.step();
    if (k % 400 === 0) {
      const { t, series } = sim.telemetry.window([`${loop}.sp`, `${loop}.meas`], -Infinity);
      const metrics = analyzeLastStep(t, series[0]!, series[1]!);
      // A lesson that asks for a prediction is solved by entering the right number.
      const prediction = withSolution ? (lesson.predict?.truth(sim.params) ?? null) : null;
      if (lesson.goal!.check({ sim, metrics, prediction }) === true) return true;
    }
  }
  return false;
};

describe('Part II and III lesson goals', () => {
  for (const lesson of LESSONS.filter((l) => (l.part === 2 || l.part === 3) && l.goal)) {
    it(`"${lesson.title}" needs work, and its solution reaches the goal`, () => {
      expect(lesson.solution, 'a lesson with a goal documents a solution').toBeDefined();
      expect(reachesGoal(lesson, false)).toBe(false);
      expect(reachesGoal(lesson, true)).toBe(true);
    });
  }
});

describe('lesson scripts', () => {
  it('replay after a reset, starting from the lesson’s original parameters', () => {
    const lesson = LESSONS.find((l) => l.id === 'indi')!;
    const p = defaultParams();
    p.sim.level = lesson.level;
    lesson.setup?.(p);
    const sim = new Simulation(p);
    sim.lessonStart = structuredClone(p);
    sim.script = lesson.events!;
    sim.reset();
    for (let k = 0; k < 14000; k++) sim.step();
    expect(sim.params.drone.motorEfficiency[1]).toBe(0.6); // the prop broke at 12 s
    sim.reset(); // the user presses R
    expect(sim.params.drone.motorEfficiency[1]).toBe(1); // healthy again…
    for (let k = 0; k < 14000; k++) sim.step();
    expect(sim.params.drone.motorEfficiency[1]).toBe(0.6); // …and it breaks again
  });
});
