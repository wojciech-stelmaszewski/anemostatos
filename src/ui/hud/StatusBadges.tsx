import { useRef } from 'react';
import { sim } from '@/store/sim';
import { useRaf } from './useRaf';

/** Crash / motors-off / take-off / lagging notices over the viewport. */
export function StatusBadges() {
  const el = useRef<HTMLDivElement>(null);
  useRaf(() => {
    const msgs: [string, string][] = [];
    if (sim.state.crashed)
      msgs.push([
        'Crashed! Hit the ground too hard. Press R to reset.',
        'bg-[#e66767]/90 text-black',
      ]);
    else if (!sim.armed)
      msgs.push([
        'Controller off — motors idle. Press M to switch it on.',
        'bg-[#c98500]/90 text-black',
      ]);
    else if (sim.takingOff)
      msgs.push(['Taking off… (integrators held until airborne)', 'bg-panel-2/90 text-fg']);
    if (sim.lagging) msgs.push(['Simulation slowed down to keep up', 'bg-panel-2/90 text-muted']);
    if (sim.paused)
      msgs.push(['Paused — Space to resume, . to single-step', 'bg-panel-2/90 text-fg']);
    const html = msgs
      .map(
        ([m, c]) => `<div class="rounded-md px-3 py-1 text-xs font-medium shadow ${c}">${m}</div>`,
      )
      .join('');
    if (el.current && el.current.innerHTML !== html) el.current.innerHTML = html;
  });
  return <div ref={el} className="pointer-events-none flex flex-col items-center gap-1" />;
}
