import { singularValues } from '@/analysis/mimo';
import { cabs } from '@/math/complex';
import { useParams } from '@/store/params';
import { Button } from '@/ui/components/button';
import { ChartCard } from './ChartCard';
import { dot, FONT, INK, niceTicks, SERIES, useChartCanvas } from './canvas';
import { useMimoMeasure, useRateLoop } from './mimoStore';

const dB = (x: number) => 20 * Math.log10(x);
const LEFT = 40;
const RIGHT = 10;
const TOP = 6;
const BOTTOM = 18;
/** The margins pane under the curves: three rows of name, bar and value. */
const ROW = 15;
const BARS = 3 * ROW + 8;
const NAME_W = 178;
const VALUE_W = 106;
const PM_MAX = 60;

/**
 * Roll and pitch as one loop with two channels. Above: the sensitivity as a test of one channel
 * sees it, |S₁₁|, and as the pair really has it, the largest singular value. Below: the phase
 * margin three ways: one channel perturbed, both by the same amount, any mix of the two.
 */
export function MimoChart() {
  const { key, data } = useRateLoop();
  const measure = useMimoMeasure();
  const level = useParams((s) => s.params.sim.level);
  const measured = measure.key === key ? measure.points : [];

  const { canvas, readout } = useChartCanvas(
    ({ ctx, w, h, hover }) => {
      ctx.font = FONT;
      if (!data) {
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText('This view needs the quadrotor (level 3) with the PID cascade.', w / 2, h / 3);
        return;
      }
      const x0 = LEFT;
      const x1 = w - RIGHT;
      const y0 = TOP;
      const y1 = h - BOTTOM - BARS;
      const f = data.fHz;
      const fMin = f[0]!;
      const fMax = f[f.length - 1]!;
      const X = (hz: number) => x0 + (Math.log10(hz / fMin) / Math.log10(fMax / fMin)) * (x1 - x0);
      const s11 = data.s11.map(dB);
      const sMax = data.sMax.map(dB);
      const lo = -40;
      const hi = Math.min(30, Math.max(9, Math.ceil(Math.max(...sMax) / 3) * 3 + 3));
      const Y = (v: number) => y0 + ((hi - Math.min(Math.max(v, lo), hi)) / (hi - lo)) * (y1 - y0);

      // Grid and axes.
      ctx.lineWidth = 1;
      ctx.textBaseline = 'middle';
      for (const v of niceTicks(lo, hi, Math.max(3, Math.floor((y1 - y0) / 28)))) {
        ctx.strokeStyle = v === 0 ? INK.axis : INK.grid;
        ctx.setLineDash(v === 0 ? [4, 3] : []);
        ctx.beginPath();
        ctx.moveTo(x0, Y(v));
        ctx.lineTo(x1, Y(v));
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'right';
        ctx.fillText(`${v}`, x0 - 5, Y(v));
      }
      ctx.textBaseline = 'top';
      for (let d = Math.floor(Math.log10(fMin)); 10 ** d <= fMax * 1.001; d++) {
        for (const m of [1, 2, 5]) {
          const hz = m * 10 ** d;
          if (hz < fMin * 0.999 || hz > fMax * 1.001) continue;
          ctx.strokeStyle = INK.grid;
          ctx.beginPath();
          ctx.moveTo(X(hz), y0);
          ctx.lineTo(X(hz), y1);
          ctx.stroke();
          if (X(hz) > x1 - 22) continue; // room for the unit
          ctx.fillStyle = INK.muted;
          ctx.textAlign = 'center';
          ctx.fillText(`${hz}`, X(hz), y1 + 4);
        }
      }
      ctx.textAlign = 'right';
      ctx.fillText('Hz', x1, y1 + 4);
      ctx.save();
      ctx.translate(10, (y0 + y1) / 2);
      ctx.rotate(-Math.PI / 2);
      ctx.textAlign = 'center';
      ctx.fillText('dB', 0, -4);
      ctx.restore();

      // The model: what one channel sees (dashed) and what the pair has (solid).
      const line = (ys: number[], dash: number[]) => {
        ctx.save();
        ctx.beginPath();
        ctx.rect(x0, y0, x1 - x0, y1 - y0);
        ctx.clip();
        ctx.strokeStyle = SERIES.model;
        ctx.lineWidth = 2;
        ctx.lineJoin = 'round';
        ctx.setLineDash(dash);
        ctx.beginPath();
        ys.forEach((v, i) => (i ? ctx.lineTo(X(f[i]!), Y(v)) : ctx.moveTo(X(f[i]!), Y(v))));
        ctx.stroke();
        ctx.restore();
      };
      line(s11, [5, 4]);
      line(sMax, []);
      // The measurement: the same two quantities from the flown sweeps.
      for (const p of measured) {
        const x = X(p.fHz);
        const one = Y(dB(cabs(p.s[0][0])));
        dot(ctx, x, one, SERIES.measured, 3.5);
        ctx.beginPath();
        ctx.arc(x, one, 1.8, 0, 2 * Math.PI);
        ctx.fillStyle = INK.surface;
        ctx.fill();
        dot(ctx, x, Y(dB(singularValues(p.s)[0])), SERIES.measured);
      }
      // Which line is which: a key in the empty corner under the 0 dB line.
      const key = (text: string, dash: number[], y: number) => {
        const tw = ctx.measureText(text).width;
        const tx = x1 - tw - 6;
        ctx.strokeStyle = SERIES.model;
        ctx.lineWidth = 2;
        ctx.setLineDash(dash);
        ctx.beginPath();
        ctx.moveTo(tx - 28, y);
        ctx.lineTo(tx - 6, y);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = INK.text;
        ctx.textAlign = 'left';
        ctx.textBaseline = 'middle';
        ctx.fillText(text, tx, y);
      };
      key('both channels: σmax(S)', [], y1 - 24);
      key('one channel: |S₁₁|', [5, 4], y1 - 10);

      // Phase margin three ways: name, bar, value.
      const hz2 = (w: number) => `${(w / 2 / Math.PI).toFixed(2)} Hz`;
      const rows: [string, number, string][] = [
        ['one loop at a time', data.one.pmDeg, hz2(data.one.wc)],
        ['both channels, same change', data.both.pmDeg, hz2(data.both.wc)],
        ['both channels, any mix', data.disk.pmDeg, `${data.disk.fHz.toFixed(2)} Hz`],
      ];
      const top = h - BARS + 4;
      const b0 = x0 + NAME_W;
      const b1 = Math.max(x1 - VALUE_W, b0 + 20);
      ctx.textBaseline = 'middle';
      rows.forEach(([name, pm, where], i) => {
        const y = top + i * ROW + ROW / 2;
        ctx.textAlign = 'left';
        ctx.fillStyle = INK.muted;
        ctx.fillText(name, x0, y);
        ctx.textAlign = 'right';
        ctx.fillStyle = INK.text;
        ctx.fillText(
          !data.stable
            ? 'unstable'
            : Number.isFinite(pm)
              ? `PM ${pm.toFixed(0)}° at ${where}`
              : 'no crossover',
          x1,
          y,
        );
        // Track, then the bar with a rounded data end.
        ctx.fillStyle = INK.grid;
        ctx.fillRect(b0, y - 3, b1 - b0, 6);
        if (data.stable && pm > 0) {
          const len = (Math.min(pm, PM_MAX) / PM_MAX) * (b1 - b0);
          ctx.fillStyle = SERIES.model;
          ctx.beginPath();
          ctx.roundRect(b0, y - 3, Math.max(len, 2), 6, [0, 3, 3, 0]);
          ctx.fill();
        }
      });

      if (!hover || hover.x < x0 || hover.x > x1 || hover.y > y1) return;
      const hz = fMin * (fMax / fMin) ** ((hover.x - x0) / (x1 - x0));
      let i = 0;
      while (i < f.length - 1 && f[i + 1]! < hz) i++;
      ctx.strokeStyle = INK.axis;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(X(f[i]!), y0);
      ctx.lineTo(X(f[i]!), y1);
      ctx.stroke();
      dot(ctx, X(f[i]!), Y(sMax[i]!), SERIES.model, 3);
      dot(ctx, X(f[i]!), Y(s11[i]!), SERIES.model, 3);
      const sv = singularValues(data.points[i]!.s);
      return `${f[i]!.toPrecision(3)} Hz   both ${sMax[i]!.toFixed(1)} dB   one ${s11[i]!.toFixed(1)} dB   smallest ${dB(sv[1]).toFixed(1)} dB`;
    },
    [data, measured],
  );

  const delay = (s: number) => (Number.isFinite(s) && s > 0 ? `${(s * 1000).toFixed(0)} ms` : '—');
  return (
    <ChartCard
      title="Roll + pitch: sensitivity S"
      readout={readout}
      legend={[
        { label: 'model', color: SERIES.model, mark: 'line' },
        { label: 'measured', color: SERIES.measured, mark: 'dot' },
      ]}
      summary={
        data ? (
          <span>
            {data.stable ? '● stable' : '▲ unstable'} · delay margin, one loop at a time:{' '}
            {delay(data.one.delayMargin)} · any mix: gain ±{data.disk.gmDb.toFixed(1)} dB
          </span>
        ) : null
      }
      controls={
        <Button
          size="sm"
          className="h-5 px-2 text-[11px]"
          disabled={measure.running || level !== 3}
          onClick={() => measure.start()}
          title="Fly two sweeps with a sine on the roll torque and then on the pitch torque, and plot the 2 × 2 sensitivity that comes back."
        >
          {measure.running ? `measuring ${(measure.progress * 100).toFixed(0)} %` : 'Measure'}
        </Button>
      }
    >
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </ChartCard>
  );
}
