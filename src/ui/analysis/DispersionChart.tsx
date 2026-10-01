import { useState } from 'react';
import { campaignKey, DISPERSIONS, passRateBound, type Run } from '@/analysis/dispersion';
import { useParams } from '@/store/params';
import { SIGNAL } from '@/ui/colors';
import { Button } from '@/ui/components/button';
import { Select } from '@/ui/components/select';
import { ChartCard } from './ChartCard';
import { dot, FONT, INK, niceTicks, useChartCanvas } from './canvas';
import { useDispersion } from './dispersionStore';

const LEFT = 40;
const RIGHT = 10;
const TOP = 8;
const BOTTOM = 30;
/** The axes a run can be placed on: every dispersed parameter except the wind's seed. */
const AXES = DISPERSIONS.filter((d) => d.id !== 'seed').map((d, i) => ({ d, i }));
const OPTIONS = AXES.map(({ d }) => ({ value: d.id, label: d.label }));
const COLOR = { pass: SIGNAL.measurement, fail: SIGNAL.error } as const;

/**
 * Every flight of a Monte Carlo campaign as a point in the space of the drawn parameters: passed
 * (dot) or failed (cross). Failures that sit together are a corner the nominal flight never visits.
 */
export function DispersionChart() {
  const [xId, setX] = useState('tau');
  const [yId, setY] = useState('delay');
  const store = useDispersion();
  const key = useParams((s) => campaignKey(s.params));
  const level = useParams((s) => s.params.sim.level);
  const runs: Run[] = store.key === key ? store.runs : [];
  const n = store.key === key ? store.n : 0;
  const failed = runs.filter((r) => !r.passed).length;

  const { canvas, readout } = useChartCanvas(
    ({ ctx, w, h, hover }) => {
      ctx.font = FONT;
      const ax = AXES.find((a) => a.d.id === xId)!;
      const ay = AXES.find((a) => a.d.id === yId)!;
      const x0 = LEFT;
      const x1 = w - RIGHT;
      const y0 = TOP;
      const y1 = h - BOTTOM;
      const frac = (d: (typeof AXES)[number]['d'], v: number) =>
        d.log ? Math.log(v / d.lo) / Math.log(d.hi / d.lo) : (v - d.lo) / (d.hi - d.lo);
      const X = (v: number) => x0 + frac(ax.d, v) * (x1 - x0);
      const Y = (v: number) => y1 - frac(ay.d, v) * (y1 - y0);

      ctx.lineWidth = 1;
      ctx.textBaseline = 'middle';
      for (const v of niceTicks(ay.d.lo, ay.d.hi, 5)) {
        ctx.strokeStyle = INK.grid;
        ctx.beginPath();
        ctx.moveTo(x0, Y(v));
        ctx.lineTo(x1, Y(v));
        ctx.stroke();
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'right';
        ctx.fillText(`${+v.toPrecision(3)}`, x0 - 5, Y(v));
      }
      ctx.textBaseline = 'top';
      const xt = ax.d.log ? [15, 20, 30, 40, 50, 60] : niceTicks(ax.d.lo, ax.d.hi, 6);
      for (const v of xt) {
        if (v < ax.d.lo || v > ax.d.hi) continue;
        ctx.strokeStyle = INK.grid;
        ctx.beginPath();
        ctx.moveTo(X(v), y0);
        ctx.lineTo(X(v), y1);
        ctx.stroke();
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText(`${+v.toPrecision(3)}`, X(v), y1 + 4);
      }
      ctx.fillStyle = INK.muted;
      ctx.textAlign = 'center';
      ctx.fillText(`${ax.d.label}, ${ax.d.unit}`, (x0 + x1) / 2, y1 + 16);
      ctx.save();
      ctx.translate(10, (y0 + y1) / 2);
      ctx.rotate(-Math.PI / 2);
      ctx.fillText(`${ay.d.label}, ${ay.d.unit}`, 0, -4);
      ctx.restore();

      if (level !== 3) {
        ctx.textAlign = 'center';
        ctx.fillText('Campaigns need the quadrotor (level 3).', (x0 + x1) / 2, (y0 + y1) / 2);
        return;
      }
      if (!runs.length) {
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText('Press Fly 300: each flight becomes a point.', (x0 + x1) / 2, (y0 + y1) / 2);
      }
      // Passes first, failures on top: the few must not hide under the many.
      for (const r of runs)
        if (r.passed) dot(ctx, X(r.values[ax.i]!), Y(r.values[ay.i]!), COLOR.pass, 2.5);
      ctx.lineWidth = 2;
      for (const r of runs) {
        if (r.passed) continue;
        const x = X(r.values[ax.i]!);
        const y = Y(r.values[ay.i]!);
        ctx.strokeStyle = INK.surface;
        ctx.lineWidth = 4;
        ctx.beginPath();
        ctx.moveTo(x - 5, y - 5);
        ctx.lineTo(x + 5, y + 5);
        ctx.moveTo(x - 5, y + 5);
        ctx.lineTo(x + 5, y - 5);
        ctx.stroke();
        ctx.strokeStyle = COLOR.fail;
        ctx.lineWidth = 2;
        ctx.stroke();
      }

      if (!hover || !runs.length) return;
      let best: Run | null = null;
      let d = 12;
      for (const r of runs) {
        const e = Math.hypot(X(r.values[ax.i]!) - hover.x, Y(r.values[ay.i]!) - hover.y);
        if (e < d) {
          d = e;
          best = r;
        }
      }
      if (!best) return;
      const vals = AXES.map(({ d: p, i }) => `${p.label} ${best.values[i]!.toPrecision(3)}`);
      const verdict = best.crashed
        ? 'crashed'
        : `peak rate ${best.peakRate.toFixed(0)} °/s, ${best.passed ? 'passed' : 'failed'}`;
      return `${vals.join(' · ')} — ${verdict}`;
    },
    [runs, xId, yId, level],
  );

  const bound = runs.length ? passRateBound(runs.length - failed, runs.length) : 0;
  return (
    <ChartCard
      title="Monte Carlo: every flight a point"
      readout={readout}
      legend={[
        { label: 'passed', color: COLOR.pass, mark: 'dot' },
        { label: 'failed', color: COLOR.fail, mark: 'cross' },
      ]}
      summary={
        runs.length ? (
          <span>
            {failed} of {runs.length} failed
            {store.running ? ` (flying ${runs.length} / ${n})` : ''} · pass rate ≥{' '}
            {(bound * 100).toFixed(1)} % with 95 % confidence
          </span>
        ) : (
          <span>
            mass, motor lag, sensor delay, gyro bias and the wind drawn anew for each flight
          </span>
        )
      }
      controls={
        <>
          <Select value={xId} onValueChange={setX} options={OPTIONS} />
          <Select value={yId} onValueChange={setY} options={OPTIONS} />
          <Button
            size="sm"
            className="h-5 px-2 text-[11px]"
            disabled={store.running || level !== 3}
            onClick={() => store.start(300)}
            title="Fly the scenario 300 times, each with its own draw of the uncertain parameters."
          >
            {store.running ? `flying ${runs.length} / ${n}` : 'Fly 300'}
          </Button>
        </>
      }
    >
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </ChartCard>
  );
}
