import { useMemo } from 'react';
import { muAnalysis } from '@/analysis/mu';
import { useParams } from '@/store/params';
import { SIGNAL } from '@/ui/colors';
import { ChartCard } from './ChartCard';
import { FONT, INK, niceTicks, useChartCanvas } from './canvas';
import { familyKey } from './store';

const LEFT = 40;
const RIGHT = 10;
const TOP = 8;
const BOTTOM = 18;
/** One hue per answer; the ceiling of 1 in ink. */
const COLOR = { sg: SIGNAL.measurement, mu: SIGNAL.p, mixed: SIGNAL.i } as const;

/**
 * Lesson III.14: three answers to "is every member of the family stable?", as functions of
 * frequency, against the ceiling of 1: the small-gain test with one lumped weight (III.11), μ with
 * one complex disk per parameter, and mixed μ with the mass as a real parameter.
 */
export function MuChart() {
  const key = useParams((s) => familyKey(s.params));
  const level = useParams((s) => s.params.sim.level);
  // eslint-disable-next-line react-hooks/exhaustive-deps -- the key stands for the parameters
  const data = useMemo(() => muAnalysis(useParams.getState().params, 1, 70), [key]);

  const { canvas, readout } = useChartCanvas(
    ({ ctx, w, h, hover }) => {
      ctx.font = FONT;
      if (!data || level !== 1) {
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText('μ analysis needs an L1 controller with a linear model.', w / 2, h / 3);
        return;
      }
      const x0 = LEFT;
      const x1 = w - RIGHT;
      const y0 = TOP;
      const y1 = h - BOTTOM;
      const f = data.fHz;
      const fMin = f[0]!;
      const fMax = f[f.length - 1]!;
      const X = (hz: number) => x0 + (Math.log10(hz / fMin) / Math.log10(fMax / fMin)) * (x1 - x0);
      const hi = Math.max(1.5, Math.ceil(Math.max(...data.mu) * 2) / 2);
      const Y = (v: number) => y1 - (Math.min(v, hi) / hi) * (y1 - y0);
      ctx.lineWidth = 1;
      ctx.textBaseline = 'middle';
      for (const v of niceTicks(0, hi, 5)) {
        ctx.strokeStyle = INK.grid;
        ctx.beginPath();
        ctx.moveTo(x0, Y(v));
        ctx.lineTo(x1, Y(v));
        ctx.stroke();
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'right';
        ctx.fillText(`${v}`, x0 - 5, Y(v));
      }
      ctx.textBaseline = 'top';
      for (const hz of [0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50]) {
        if (hz < fMin || hz > fMax || X(hz) > x1 - 22) continue;
        ctx.strokeStyle = INK.grid;
        ctx.beginPath();
        ctx.moveTo(X(hz), y0);
        ctx.lineTo(X(hz), y1);
        ctx.stroke();
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText(`${hz}`, X(hz), y1 + 4);
      }
      ctx.textAlign = 'right';
      ctx.fillText('Hz', x1, y1 + 4);
      // The ceiling: below 1 at every frequency, the answer is "guaranteed stable".
      ctx.strokeStyle = INK.text;
      ctx.setLineDash([5, 4]);
      ctx.beginPath();
      ctx.moveTo(x0, Y(1));
      ctx.lineTo(x1, Y(1));
      ctx.stroke();
      ctx.setLineDash([]);
      const line = (ys: number[], color: string) => {
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.lineJoin = 'round';
        ctx.beginPath();
        ys.forEach((v, i) => (i ? ctx.lineTo(X(f[i]!), Y(v)) : ctx.moveTo(X(f[i]!), Y(v))));
        ctx.stroke();
      };
      line(data.smallGain, COLOR.sg);
      line(data.mu, COLOR.mu);
      line(data.muMixed, COLOR.mixed);
      if (!hover || hover.x < x0 || hover.x > x1) return;
      const hz = fMin * (fMax / fMin) ** ((hover.x - x0) / (x1 - x0));
      let i = 0;
      while (i < f.length - 1 && f[i + 1]! < hz) i++;
      ctx.strokeStyle = INK.axis;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(X(f[i]!), y0);
      ctx.lineTo(X(f[i]!), y1);
      ctx.stroke();
      const wm = data.w[0][i]!.toFixed(2);
      const wt = data.w[1][i]!.toFixed(2);
      const wd = data.w[2][i]!.toFixed(2);
      return `${f[i]!.toPrecision(3)} Hz   |W·T| ${data.smallGain[i]!.toFixed(2)}   μ ${data.mu[i]!.toFixed(2)}   mixed ${data.muMixed[i]!.toFixed(2)}   weights m ${wm} τ ${wt} d ${wd}`;
    },
    [data, level],
  );

  const verdict = (v: number) => (v < 1 ? `${v.toFixed(2)} ●` : `${v.toFixed(2)} ▲`);
  return (
    <ChartCard
      title="Is every member stable? Three answers"
      readout={readout}
      legend={[
        { label: 'small gain, one weight', color: COLOR.sg, mark: 'line' },
        { label: 'μ, a disk per parameter', color: COLOR.mu, mark: 'line' },
        { label: 'mixed μ, the mass real', color: COLOR.mixed, mark: 'line' },
      ]}
      summary={
        data ? (
          <span>
            peaks (below 1 = guaranteed): small gain {verdict(data.peakSmallGain)} · μ{' '}
            {verdict(data.peakMu)} · mixed μ {verdict(data.peakMuMixed)}
          </span>
        ) : null
      }
    >
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </ChartCard>
  );
}
