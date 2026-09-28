import { useState } from 'react';
import { ENTRIES, runArena, SCENARIOS, winners, type ArenaResult } from '@/engine/arena';
import { Button } from '@/ui/components/button';

// Kept across re-renders and lesson switches: the arena takes a while.
let saved: ArenaResult[] = [];

/** Runs the arena in the browser, one flight per tick, and shows the table (lesson II.21). */
export function ArenaPanel() {
  const [results, setResults] = useState<ArenaResult[]>(saved);
  const [running, setRunning] = useState(false);
  const total = ENTRIES.length * SCENARIOS.length;

  const run = () => {
    setRunning(true);
    const jobs = ENTRIES.flatMap((e) => SCENARIOS.map((s) => [e, s] as const));
    const out: ArenaResult[] = [];
    const next = (k: number) => {
      if (k >= jobs.length) {
        saved = out;
        setRunning(false);
        return;
      }
      const [e, s] = jobs[k]!;
      out.push(runArena(e, s));
      setResults([...out]);
      setTimeout(() => next(k + 1), 0);
    };
    setTimeout(() => next(0), 0);
  };

  const best = winners(results);
  const get = (e: string, s: string) => results.find((r) => r.entry === e && r.scenario === s);
  const done = results.length === total;
  const meanOf = (id: string, f: (r: ArenaResult) => number) => {
    const rows = results.filter((r) => r.entry === id && !r.crashed);
    return rows.length ? rows.reduce((a, r) => a + f(r), 0) / rows.length : NaN;
  };
  const bestMean = (f: (r: ArenaResult) => number) =>
    done ? ENTRIES.reduce((a, b) => (meanOf(b.id, f) < meanOf(a.id, f) ? b : a)).id : '';
  const bestJitter = bestMean((r) => r.jitter);
  const bestCpu = bestMean((r) => r.cpuUs);
  const win = 'font-semibold text-accent';

  return (
    <div className="mt-2 space-y-1.5">
      <div className="flex items-center gap-2">
        <Button variant="accent" disabled={running} onClick={run}>
          {running ? `Flying… ${results.length}/${total}` : done ? 'Run again' : 'Run the arena'}
        </Button>
        <span className="text-[11px] text-muted">
          {ENTRIES.length} controllers × {SCENARIOS.length} scenarios, about a minute.
        </span>
      </div>
      {results.length > 0 && (
        <div className="overflow-x-auto">
          <table className="w-full border-collapse font-mono text-[10px]">
            <thead>
              <tr className="text-muted">
                <th className="py-0.5 pr-1 text-left font-normal">RMS error, cm</th>
                {SCENARIOS.map((s) => (
                  <th key={s.id} className="px-1 text-right font-normal" title={s.about}>
                    {s.label}
                  </th>
                ))}
                <th
                  className="px-1 text-right font-normal"
                  title="Mean RMS thrust change between samples, N"
                >
                  jitter
                </th>
                <th
                  className="px-1 text-right font-normal"
                  title="Wall-clock µs per physics step, in your browser"
                >
                  µs
                </th>
              </tr>
            </thead>
            <tbody>
              {ENTRIES.map((e) => (
                <tr key={e.id} className="border-t border-border/50">
                  <td className="py-0.5 pr-1 font-sans text-fg">{e.label}</td>
                  {SCENARIOS.map((s) => {
                    const r = get(e.id, s.id);
                    return (
                      <td
                        key={s.id}
                        className={`px-1 text-right ${best[s.id] === e.id ? win : ''}`}
                      >
                        {!r ? '' : r.crashed ? 'crash' : (r.rms * 100).toFixed(1)}
                      </td>
                    );
                  })}
                  <td className={`px-1 text-right ${bestJitter === e.id ? win : ''}`}>
                    {Number.isFinite(meanOf(e.id, (r) => r.jitter))
                      ? meanOf(e.id, (r) => r.jitter).toFixed(3)
                      : ''}
                  </td>
                  <td className={`px-1 text-right ${bestCpu === e.id ? win : ''}`}>
                    {Number.isFinite(meanOf(e.id, (r) => r.cpuUs))
                      ? meanOf(e.id, (r) => r.cpuUs).toFixed(1)
                      : ''}
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      )}
    </div>
  );
}
