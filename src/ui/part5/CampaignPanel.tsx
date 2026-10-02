import { Fragment, useState } from 'react';
import {
  CAMPAIGN_RUNS,
  finishedVerification,
  L1_DISPERSIONS,
  REQUIREMENTS,
  storeVerification,
  VerificationCampaign,
  type RequirementResult,
} from '@/analysis/campaign';
import { useParams } from '@/store/params';
import { Button } from '@/ui/components/button';
import { describeRun } from './describe';
import { BAD, GOOD } from './style';
import { useSliced } from './useSliced';

const fmt = (v: number, digits: number) => (Number.isFinite(v) ? v.toFixed(digits) : '∞');

/** The dispersion table every campaign draws from. */
export function DispersionTable() {
  return (
    <table className="w-full border-collapse font-mono text-[10px]">
      <thead>
        <tr className="text-muted">
          <th className="py-0.5 pr-1 text-left font-normal">dispersed</th>
          <th className="px-1 text-right font-normal">from</th>
          <th className="px-1 text-right font-normal">to</th>
        </tr>
      </thead>
      <tbody>
        {L1_DISPERSIONS.map((d) => (
          <tr key={d.id} className="border-t border-border/50">
            <td className="py-0.5 pr-1 font-sans text-fg">
              {d.label}
              {d.log ? ' (log-uniform)' : ''}
            </td>
            {d.id === 'seed' ? (
              <td colSpan={2} className="px-1 text-right">
                any
              </td>
            ) : (
              <>
                <td className="px-1 text-right">
                  {d.lo} {d.unit}
                </td>
                <td className="px-1 text-right">
                  {d.hi} {d.unit}
                </td>
              </>
            )}
          </tr>
        ))}
      </tbody>
    </table>
  );
}

/** One row per requirement: pass count, the rate it proves, the worst case and where it was. */
export function VerificationTable({ results }: { results: RequirementResult[] }) {
  return (
    <table className="w-full border-collapse font-mono text-[10px]">
      <thead>
        <tr className="text-muted">
          <th className="py-0.5 pr-1 text-left font-normal">requirement</th>
          <th className="px-1 text-right font-normal" title="Flights that met it">
            passed
          </th>
          <th className="px-1 text-right font-normal" title="Pass rate shown with 95 % confidence">
            ≥ rate
          </th>
          <th className="px-1 text-right font-normal" title="The worst flight, against the limit">
            worst / limit
          </th>
        </tr>
      </thead>
      <tbody>
        {results.map((r) => (
          <Fragment key={r.requirement.id}>
            <tr className="border-t border-border/50">
              <td className="py-0.5 pr-1 font-sans text-fg">
                <span className={r.green ? GOOD : BAD}>●</span> {r.requirement.id}{' '}
                {r.requirement.text}
              </td>
              <td className={`px-1 text-right ${r.green ? '' : BAD}`}>
                {r.passed}/{r.runs.length}
              </td>
              <td className="px-1 text-right">{(r.bound * 100).toFixed(0)} %</td>
              <td className={`px-1 text-right ${r.worst.passed ? '' : BAD}`}>
                {fmt(r.worst.metric * r.requirement.scale, 1)} /{' '}
                {fmt(r.requirement.limit * r.requirement.scale, 0)} {r.requirement.unit}
              </td>
            </tr>
            <tr>
              <td colSpan={4} className="pb-0.5 pl-3 font-sans text-[10px] text-muted">
                worst case: {describeRun(r.worst.values)}
              </td>
            </tr>
          </Fragment>
        ))}
      </tbody>
    </table>
  );
}

/** Lesson V.2: one campaign per requirement over the dispersion table, and the report. */
export function CampaignPanel() {
  const params = useParams((s) => s.params);
  const [, setVersion] = useState(0);
  const [partial, setPartial] = useState<RequirementResult[]>([]);
  const { progress, start } = useSliced<VerificationCampaign>(
    (c) => {
      storeVerification(c.p, c.results);
      setPartial([]);
      setVersion((v) => v + 1);
    },
    // Show each requirement's row as soon as its campaign is done.
    (c) => setPartial([...c.results]),
  );
  const results = finishedVerification(params) ?? (progress !== null ? partial : null);
  const flights = REQUIREMENTS.length * CAMPAIGN_RUNS;

  return (
    <div className="mt-2 space-y-1.5">
      <DispersionTable />
      <div className="flex items-center gap-2">
        <Button
          variant="accent"
          disabled={progress !== null}
          onClick={() => start(new VerificationCampaign(structuredClone(params)))}
        >
          {progress !== null ? `${Math.round(progress * 100)} %…` : 'Fly the campaign'}
        </Button>
        <span className="text-[11px] text-muted">
          {REQUIREMENTS.length} requirements × {CAMPAIGN_RUNS} flights = {flights} flights
        </span>
      </div>
      {results && results.length > 0 && <VerificationTable results={results} />}
    </div>
  );
}
