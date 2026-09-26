import { CheckCircle2, ChevronLeft, ChevronRight, GraduationCap, Play, Target } from 'lucide-react';
import { useEffect, useState } from 'react';
import { analyzeLastStep } from '@/engine/metrics';
import { sim, useUi } from '@/store/sim';
import { Button } from '@/ui/components/button';
import { Select } from '@/ui/components/select';
import { LESSONS } from './lessons';
import { startLesson } from './runner';
import type { Lesson, Part } from './types';

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

const PARTS: { part: Part; label: string; prefix: string }[] = [
  { part: 1, label: 'Part I · PID', prefix: '' },
  { part: 2, label: 'Part II · Beyond PID', prefix: 'II.' },
];
const partOf = (l: Lesson): Part => l.part ?? 1;

/** Guided lessons (docs/ui-and-visualization.md §5). The sandbox stays fully unlocked. */
export function LessonPanel() {
  const id = useUi((s) => s.lesson);
  const setLesson = useUi((s) => s.setLesson);
  const current = LESSONS.find((l) => l.id === id) ?? LESSONS[0]!;
  const [part, setPart] = useState<Part>(partOf(current));
  const list = LESSONS.filter((l) => partOf(l) === part);
  const idx = list.findIndex((l) => l.id === current.id);
  const lesson = idx >= 0 ? current : list[0];
  const i = Math.max(0, idx);
  const prefix = PARTS.find((p) => p.part === part)!.prefix;
  const [open, setOpen] = useState(true);

  return (
    <div className="border-b border-border bg-panel-2/40">
      <div className="flex items-center gap-2 px-3 py-2">
        <GraduationCap className="size-4 text-accent" />
        <button
          className="text-xs font-semibold uppercase tracking-wider text-muted hover:text-fg"
          onClick={() => setOpen(!open)}
        >
          {lesson ? `Lesson ${prefix}${i + 1}/${prefix}${list.length}` : 'Lessons'}
        </button>
        <span className="flex-1" />
        <Button
          size="icon"
          variant="ghost"
          disabled={!lesson || i === 0}
          onClick={() => setLesson(list[i - 1]!.id)}
        >
          <ChevronLeft />
        </Button>
        <Button
          size="icon"
          variant="ghost"
          disabled={!lesson || i >= list.length - 1}
          onClick={() => setLesson(list[i + 1]!.id)}
        >
          <ChevronRight />
        </Button>
      </div>
      {open && (
        <div className="px-3 pb-3 text-xs leading-relaxed text-fg/90">
          <div className="mb-2 flex gap-1">
            {PARTS.map((p) => (
              <button
                key={p.part}
                onClick={() => {
                  setPart(p.part);
                  const first = LESSONS.find((l) => partOf(l) === p.part);
                  if (first) setLesson(first.id);
                }}
                className={`flex-1 rounded border px-2 py-0.5 text-[11px] font-semibold transition-colors ${
                  p.part === part
                    ? 'border-accent/60 bg-accent/10 text-fg'
                    : 'border-border text-muted hover:text-fg'
                }`}
              >
                {p.label}
              </button>
            ))}
          </div>
          {lesson ? (
            <>
              <div className="mb-2 flex items-center gap-2">
                <Select
                  className="min-w-0 flex-1"
                  value={lesson.id}
                  onValueChange={setLesson}
                  options={list.map((l, k) => ({
                    value: l.id,
                    label: `${prefix}${k + 1}. ${l.chapter ? `${l.chapter} — ` : ''}${l.title} (L${l.level})`,
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
            </>
          ) : (
            <div className="space-y-1.5 text-muted">
              <p className="text-fg">The second semester is being written.</p>
              <p>
                It picks up where the PID cascade ends: state feedback and LQR, the Kalman filter,
                disturbance observers (ADRC), INDI, geometric control and trajectories, MPC and
                MPPI, safety filters, adaptive and learned controllers — and a final arena where
                they all compete on the same wind.
              </p>
              <p>
                The sandbox already has what they need: the <b>disturbance</b> and{' '}
                <b>phase portrait</b> charts (chart header → extra), motor faults and rotor drag.
              </p>
            </div>
          )}
        </div>
      )}
    </div>
  );
}
