import { CheckCircle2, ChevronLeft, ChevronRight, GraduationCap, Play, Target } from 'lucide-react';
import { useEffect, useState } from 'react';
import { analyzeLastStep } from '@/engine/metrics';
import { sim, useUi } from '@/store/sim';
import { Button } from '@/ui/components/button';
import { Select } from '@/ui/components/select';
import { LESSONS } from './lessons';
import { startLesson } from './runner';
import type { Lesson } from './types';

function Goal({ lesson }: { lesson: Lesson }) {
  const [status, setStatus] = useState<boolean | string>(false);
  useEffect(() => {
    if (!lesson.goal) return;
    const loop = useUi.getState().loop;
    const tick = () => {
      const { t, series } = sim.telemetry.window([`${loop}.sp`, `${loop}.meas`], -Infinity);
      setStatus(lesson.goal!.check({ sim, metrics: analyzeLastStep(t, series[0]!, series[1]!) }));
    };
    tick();
    const id = setInterval(tick, 400);
    return () => clearInterval(id);
  }, [lesson]);
  if (!lesson.goal) return null;
  const done = status === true;
  return (
    <div
      className={`mt-2 flex gap-2 rounded border px-2 py-1.5 ${done ? 'border-[#199e70] bg-[#199e70]/10' : 'border-border'}`}
    >
      {done ? (
        <CheckCircle2 className="mt-0.5 size-4 shrink-0 text-[#199e70]" />
      ) : (
        <Target className="mt-0.5 size-4 shrink-0 text-muted" />
      )}
      <div>
        <div className="font-semibold text-fg">{done ? 'Goal reached!' : 'Goal'}</div>
        <div className="text-muted">{lesson.goal.text}</div>
        {typeof status === 'string' && <div className="font-mono text-fg">{status}</div>}
      </div>
    </div>
  );
}

/** Guided lessons (docs/ui-and-visualization.md §5). The sandbox stays fully unlocked. */
export function LessonPanel() {
  const id = useUi((s) => s.lesson);
  const setLesson = useUi((s) => s.setLesson);
  const idx = Math.max(
    0,
    LESSONS.findIndex((l) => l.id === id),
  );
  const lesson = LESSONS[idx]!;
  const [open, setOpen] = useState(true);

  return (
    <div className="border-b border-border bg-panel-2/40">
      <div className="flex items-center gap-2 px-3 py-2">
        <GraduationCap className="size-4 text-accent" />
        <button
          className="text-xs font-semibold uppercase tracking-wider text-muted hover:text-fg"
          onClick={() => setOpen(!open)}
        >
          Lesson {idx + 1}/{LESSONS.length}
        </button>
        <span className="flex-1" />
        <Button
          size="icon"
          variant="ghost"
          disabled={idx === 0}
          onClick={() => setLesson(LESSONS[idx - 1]!.id)}
        >
          <ChevronLeft />
        </Button>
        <Button
          size="icon"
          variant="ghost"
          disabled={idx === LESSONS.length - 1}
          onClick={() => setLesson(LESSONS[idx + 1]!.id)}
        >
          <ChevronRight />
        </Button>
      </div>
      {open && (
        <div className="px-3 pb-3 text-xs leading-relaxed text-fg/90">
          <div className="mb-2 flex items-center gap-2">
            <Select
              className="min-w-0 flex-1"
              value={lesson.id}
              onValueChange={setLesson}
              options={LESSONS.map((l, i) => ({
                value: l.id,
                label: `${i + 1}. ${l.title} (L${l.level})`,
              }))}
            />
            <Button variant="accent" onClick={() => startLesson(lesson)}>
              <Play /> Start
            </Button>
          </div>
          <div className="max-h-[42vh] space-y-1.5 overflow-y-auto pr-1 [&_kbd]:rounded [&_kbd]:border [&_kbd]:border-border [&_kbd]:bg-panel-2 [&_kbd]:px-1 [&_kbd]:font-mono [&_kbd]:text-[10px]">
            <h3 className="text-sm font-semibold text-fg">{lesson.title}</h3>
            {lesson.body}
            <Goal key={lesson.id} lesson={lesson} />
          </div>
        </div>
      )}
    </div>
  );
}
