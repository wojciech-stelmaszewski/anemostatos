import { useState } from 'react';
import {
  meetsMargins,
  REPORT_ENTRIES,
  ReportCard,
  REQUIRED,
  type ReportRow,
} from '@/analysis/report';
import { Button } from '@/ui/components/button';

// Kept across re-renders and lesson switches: the card takes minutes.
let saved: ReportRow[] = [];
/** Physics steps per timer tick: the page stays responsive between them. */
const SLICE = 20000;

const fmt = (v: number, digits = 0) => (Number.isFinite(v) ? v.toFixed(digits) : '∞');

/** One table per level: margins, Monte Carlo and RMS side by side. */
function Table({ rows, level }: { rows: ReportRow[]; level: 1 | 3 }) {
  const mine = rows.filter((r) => r.level === level);
  if (!mine.length) return null;
  const finite = mine.filter((r) => !r.crashed && Number.isFinite(r.rms));
  const bestRms = finite.length ? finite.reduce((a, b) => (b.rms < a.rms ? b : a)).id : '';
  const below = 'text-[#e66767]';
  return (
    <table className="w-full border-collapse font-mono text-[10px]">
      <thead>
        <tr className="text-muted">
          <th className="py-0.5 pr-1 text-left font-normal">
            {level === 1 ? 'L1 altitude' : 'L3 quadrotor'}
          </th>
          <th className="px-1 text-right font-normal" title="Phase margin at the plant input">
            PM
          </th>
          <th className="px-1 text-right font-normal" title="Smaller of the two gain margins">
            GM
          </th>
          <th className="px-1 text-right font-normal" title="Delay margin">
            delay, ms
          </th>
          <th className="px-1 text-right font-normal" title="Peak of |S|">
            ‖S‖∞
          </th>
          <th
            className="px-1 text-right font-normal"
            title="Monte Carlo: flights passed, and the pass rate it shows with 95 % confidence"
          >
            Monte Carlo
          </th>
          <th
            className="px-1 text-right font-normal"
            title={level === 1 ? 'RMS altitude error in gusts' : 'RMS position error in gusts'}
          >
            RMS, cm
          </th>
        </tr>
      </thead>
      <tbody>
        {mine.map((r) => (
          <tr key={r.id} className="border-t border-border/50">
            <td className="py-0.5 pr-1 font-sans text-fg">
              {meetsMargins(r) ? '' : '▲ '}
              {r.label}
            </td>
            {r.linear ? (
              <>
                <td className={`px-1 text-right ${r.pmDeg < REQUIRED.pmDeg ? below : ''}`}>
                  {fmt(r.pmDeg, 1)}°
                </td>
                <td className={`px-1 text-right ${r.gmDb < REQUIRED.gmDb ? below : ''}`}>
                  {fmt(r.gmDb, 1)}
                </td>
                <td className="px-1 text-right">{fmt(r.delayMs)}</td>
                <td className="px-1 text-right">{r.ms.toFixed(2)}</td>
              </>
            ) : (
              <td
                colSpan={4}
                className="px-1 text-center text-muted"
                title="The sweep found no consistent linear loop: a flying controller cannot have a negative margin."
              >
                — not a linear loop —
              </td>
            )}
            <td className={`px-1 text-right ${r.passed < r.runs ? below : ''}`}>
              {r.passed}/{r.runs} ≥{(r.bound * 100).toFixed(0)}%
            </td>
            <td
              className={`px-1 text-right ${r.id === bestRms ? 'font-semibold text-accent' : ''}`}
            >
              {r.crashed ? 'crash' : (r.rms * 100).toFixed(1)}
            </td>
          </tr>
        ))}
      </tbody>
    </table>
  );
}

/** Builds the robustness report card in the browser, a slice per tick (lesson III.30). */
export function ReportPanel() {
  const [rows, setRows] = useState<ReportRow[]>(saved);
  const [status, setStatus] = useState<string | null>(null);

  const run = () => {
    const card = new ReportCard();
    setRows([]);
    const tick = () => {
      const done = card.advance(SLICE);
      setRows([...card.rows]);
      setStatus(done ? null : card.status);
      if (done) saved = [...card.rows];
      else setTimeout(tick, 0);
    };
    setStatus(card.status);
    setTimeout(tick, 0);
  };

  return (
    <div className="mt-2 space-y-1.5">
      <div className="flex items-center gap-2">
        <Button variant="accent" disabled={status !== null} onClick={run}>
          {status !== null
            ? `${rows.length}/${REPORT_ENTRIES.length}…`
            : rows.length
              ? 'Build again'
              : 'Build the report card'}
        </Button>
        <span className="text-[11px] text-muted">
          {status ?? `${REPORT_ENTRIES.length} controllers, a few minutes.`}
        </span>
      </div>
      {rows.length > 0 && (
        <div className="space-y-2 overflow-x-auto">
          <Table rows={rows} level={1} />
          <Table rows={rows} level={3} />
          <p className="text-[10px] text-muted">
            ▲ below {REQUIRED.gmDb} dB or {REQUIRED.pmDeg}° at the plant input (red), or no margin
            can be measured. Best RMS in colour.
          </p>
        </div>
      )}
    </div>
  );
}
