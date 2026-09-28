import { defaultParams } from '@/sim/params';
import { useParams } from '@/store/params';
import { DEFAULT_LOOP, sim, useUi } from '@/store/sim';
import type { Lesson } from './types';

/** Load a lesson: its parameters on top of the defaults, a fresh run, and its scripted events. */
export function startLesson(lesson: Lesson): void {
  const p = defaultParams();
  p.sim.level = lesson.level;
  lesson.setup?.(p);
  useParams.getState().replace(p);
  sim.ghost = null;
  sim.lessonStart = structuredClone(p);
  const events = lesson.events;
  sim.script = events ? (s) => s.level === lesson.level && events(s) : null;
  sim.reset();
  lesson.onStart?.(sim);
  const ui = useUi.getState();
  ui.setLoop(lesson.loop ?? DEFAULT_LOOP[lesson.level]);
  ui.setExtraChart(lesson.chart ?? 'wind');
  ui.setLesson(lesson.id);
  if (ui.paused) ui.setPaused(false);
}
