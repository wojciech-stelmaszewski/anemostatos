import { useState } from 'react';
import { DesignReview, finishedReview, storeReview, type ReviewItem } from '@/analysis/review';
import { useParams } from '@/store/params';
import { Button } from '@/ui/components/button';
import { BAD, GOOD } from './style';
import { useSliced } from './useSliced';

const TITLES: Record<ReviewItem['id'], string> = {
  model: 'Model',
  margins: 'Margins',
  campaign: 'Campaign',
  faults: 'Faults',
  software: 'Software',
};

/** Lesson V.6: the design review, item by item, and its verdict. */
export function ReviewPanel() {
  const params = useParams((s) => s.params);
  const [, setVersion] = useState(0);
  const [partial, setPartial] = useState<ReviewItem[]>([]);
  const [current, setCurrent] = useState<string | null>(null);
  const { progress, start } = useSliced<DesignReview>(
    (r) => {
      storeReview(r.p, r.items);
      setPartial([]);
      setCurrent(null);
      setVersion((v) => v + 1);
    },
    (r) => {
      setPartial([...r.items]);
      setCurrent(r.current);
    },
  );
  const items = finishedReview(params) ?? (progress !== null ? partial : null);
  const open = items?.filter((i) => !i.passed).length ?? 0;
  const complete = items?.length === Object.keys(TITLES).length;

  return (
    <div className="mt-2 space-y-1.5">
      <div className="flex items-center gap-2">
        <Button
          variant="accent"
          disabled={progress !== null}
          onClick={() => start(new DesignReview(structuredClone(params)))}
        >
          {progress !== null ? `${Math.round(progress * 100)} %…` : 'Hold the review'}
        </Button>
        <span className="text-[11px] text-muted">
          {progress !== null && current
            ? `checking: ${TITLES[current as ReviewItem['id']]}`
            : 'a probe sweep, 60 dispersed flights and two scenarios: about ten seconds'}
        </span>
      </div>
      {items && items.length > 0 && (
        <ol className="space-y-1">
          {items.map((i) => (
            <li key={i.id} className="rounded border border-border/60 px-2 py-1">
              <div className="flex gap-1.5">
                <span className={i.passed ? GOOD : BAD}>{i.passed ? '✓' : '✗'}</span>
                <span className="font-semibold text-fg">{i.title}.</span>
                <span className="text-fg/90">{i.question}</span>
              </div>
              <div className="pl-4 font-mono text-[10px] text-muted">{i.evidence}</div>
            </li>
          ))}
        </ol>
      )}
      {complete && (
        <p className={`text-[11px] font-semibold ${open ? BAD : GOOD}`}>
          {open
            ? `Not cleared for flight: ${open} open item${open > 1 ? 's' : ''}.`
            : 'Cleared for flight: every item closed with evidence.'}
        </p>
      )}
    </div>
  );
}
