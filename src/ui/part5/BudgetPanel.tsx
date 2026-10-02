import { useState } from 'react';
import {
  BudgetRun,
  finishedBudget,
  MARGIN,
  REQUIREMENT,
  storeBudget,
  type ErrorStats,
} from '@/analysis/budget';
import { useParams } from '@/store/params';
import { Button } from '@/ui/components/button';
import { BAD, GOOD } from './style';
import { useSliced } from './useSliced';

const cm = (v: number) => (Number.isFinite(v) ? (v * 100).toFixed(2) : '—');

function Stats({ s }: { s: ErrorStats }) {
  return s.crashed ? (
    <td colSpan={3} className={`px-1 text-center ${BAD}`}>
      crashed
    </td>
  ) : (
    <>
      <td className="px-1 text-right">{cm(s.mean)}</td>
      <td className="px-1 text-right">{cm(3 * s.sd)}</td>
      <td className="px-1 text-right font-semibold">{cm(s.total)}</td>
    </>
  );
}

/** Lesson V.1: fly each error source alone, then all of them, and lay out the budget. */
export function BudgetPanel() {
  const params = useParams((s) => s.params);
  const [, setVersion] = useState(0);
  const { progress, start } = useSliced<BudgetRun>((run) => {
    storeBudget(run.p, run.result);
    setVersion((v) => v + 1);
  });
  const result = finishedBudget(params);
  const usable = REQUIREMENT * (1 - MARGIN);

  return (
    <div className="mt-2 space-y-1.5">
      <div className="flex items-center gap-2">
        <Button
          variant="accent"
          disabled={progress !== null}
          onClick={() => start(new BudgetRun(structuredClone(params)))}
        >
          {progress !== null ? `${Math.round(progress * 100)} %…` : 'Measure the budget'}
        </Button>
        <span className="text-[11px] text-muted">
          {progress !== null
            ? 'one flight per source, then all at once'
            : result
              ? 'measured for these settings'
              : 'a minute of flight per source, a second or two of computing'}
        </span>
      </div>
      {result && (
        <table className="w-full border-collapse font-mono text-[10px]">
          <thead>
            <tr className="text-muted">
              <th className="py-0.5 pr-1 text-left font-normal">source, cm</th>
              <th className="px-1 text-right font-normal">allowed</th>
              <th className="px-1 text-right font-normal">mean</th>
              <th className="px-1 text-right font-normal">3σ</th>
              <th className="px-1 text-right font-normal">|mean|+3σ</th>
            </tr>
          </thead>
          <tbody>
            {result.entries.map((e) => (
              <tr key={e.source.id} className="border-t border-border/50">
                <td className="py-0.5 pr-1 font-sans text-fg">
                  {e.within ? '✓ ' : '✗ '}
                  {e.source.label}
                  <span className="text-muted"> ({e.source.kind})</span>
                </td>
                <td className={`px-1 text-right ${e.within ? GOOD : BAD}`}>
                  {cm(e.source.allocation)}
                </td>
                <Stats s={e.stats} />
              </tr>
            ))}
            <tr className="border-t border-border">
              <td className="py-0.5 pr-1 font-sans text-fg" colSpan={4}>
                Budget from the entries: Σ|mean| + 3·√Σσ²
              </td>
              <td className="px-1 text-right font-semibold">{cm(result.predicted)}</td>
            </tr>
            <tr className="border-t border-border/50">
              <td className="py-0.5 pr-1 font-sans text-fg">Confirmation: all sources at once</td>
              <td
                className={`px-1 text-right ${result.flown.total <= usable ? GOOD : BAD}`}
                title="The requirement less the margin kept back"
              >
                {cm(usable)}
              </td>
              <Stats s={result.flown} />
            </tr>
          </tbody>
        </table>
      )}
      {result && (
        <p className="text-[10px] text-muted">
          Requirement {cm(REQUIREMENT)} cm; {Math.round(MARGIN * 100)} % kept back, so the sources
          may use {cm(usable)} cm.{' '}
          {result.closes ? (
            <span className={GOOD}>
              The budget closes with {cm(REQUIREMENT - result.flown.total)} cm left over.
            </span>
          ) : (
            <span className={BAD}>The budget does not close.</span>
          )}
        </p>
      )}
    </div>
  );
}
