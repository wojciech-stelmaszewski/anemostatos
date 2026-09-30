import { cabs } from '@/math/complex';
import { ChartCard } from './ChartCard';
import { dot, FONT, INK, SERIES, useChartCanvas } from './canvas';
import { useLoopData, useMeasured } from './store';

/** The window on the complex plane: the neighbourhood of −1, where stability is decided. */
const VIEW = { cx: -0.8, cy: 0, half: 1.9 };

/**
 * Nyquist plot: L(jω) drawn in the complex plane for rising frequency. The loop is stable with
 * margins to spare while the curve keeps its distance from −1; that distance is 1/Ms.
 */
export function NyquistChart() {
  const data = useLoopData();
  const measured = useMeasured(data);
  const m = data.margins;

  const { canvas, readout } = useChartCanvas(
    ({ ctx, w, h, hover }) => {
      const s = Math.min(w, h) / (2 * VIEW.half);
      const X = (re: number) => w / 2 + (re - VIEW.cx) * s;
      const Y = (im: number) => h / 2 - (im - VIEW.cy) * s;
      ctx.font = FONT;
      ctx.lineWidth = 1;
      // Axes and the unit circle.
      ctx.strokeStyle = INK.grid;
      ctx.beginPath();
      ctx.moveTo(0, Y(0));
      ctx.lineTo(w, Y(0));
      ctx.moveTo(X(0), 0);
      ctx.lineTo(X(0), h);
      ctx.stroke();
      ctx.setLineDash([3, 3]);
      ctx.strokeStyle = INK.axis;
      ctx.beginPath();
      ctx.arc(X(0), Y(0), s, 0, 2 * Math.PI);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = INK.muted;
      ctx.textAlign = 'left';
      ctx.textBaseline = 'top';
      ctx.fillText('|L| = 1', X(0) + 0.72 * s, Y(0) - 0.72 * s - 11);
      ctx.fillText('Im', X(0) + 4, 3);
      ctx.textAlign = 'right';
      ctx.fillText('Re', w - 4, Y(0) + 3);

      // The critical point and the circle the curve must stay out of.
      if (m && m.ms > 0) {
        ctx.strokeStyle = INK.text;
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.arc(X(-1), Y(0), s / m.ms, 0, 2 * Math.PI);
        ctx.stroke();
      }
      ctx.strokeStyle = INK.text;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(X(-1) - 4, Y(0) - 4);
      ctx.lineTo(X(-1) + 4, Y(0) + 4);
      ctx.moveTo(X(-1) - 4, Y(0) + 4);
      ctx.lineTo(X(-1) + 4, Y(0) - 4);
      ctx.stroke();
      ctx.fillStyle = INK.text;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      ctx.fillText('−1', X(-1), Y(0) + 7);

      if (!data.l.length && !measured.length) {
        ctx.fillStyle = INK.muted;
        ctx.fillText('No linear model: press Measure on the Bode plot.', w / 2, h * 0.2);
        return;
      }
      // The curve, low frequencies first; it enters the window from far away.
      ctx.strokeStyle = SERIES.model;
      ctx.lineWidth = 2;
      ctx.lineJoin = 'round';
      ctx.beginPath();
      data.l.forEach((v, i) => (i ? ctx.lineTo(X(v.re), Y(v.im)) : ctx.moveTo(X(v.re), Y(v.im))));
      ctx.stroke();
      for (const p of measured) dot(ctx, X(p.l!.re), Y(p.l!.im), SERIES.measured);

      if (!hover || !data.l.length) return;
      // The nearest point of the curve: the pointer only has to be closest.
      let best = 0;
      let d = Infinity;
      data.l.forEach((v, i) => {
        const e = Math.hypot(X(v.re) - hover.x, Y(v.im) - hover.y);
        if (e < d) {
          d = e;
          best = i;
        }
      });
      const v = data.l[best]!;
      dot(ctx, X(v.re), Y(v.im), SERIES.model, 3.5);
      const dist = Math.hypot(v.re + 1, v.im);
      return `${data.fHz[best]!.toPrecision(3)} Hz   L = ${v.re.toFixed(2)} ${v.im < 0 ? '−' : '+'} ${Math.abs(v.im).toFixed(2)}j   |L| ${cabs(v).toFixed(2)}   to −1: ${dist.toFixed(2)}`;
    },
    [data, measured],
  );

  return (
    <ChartCard
      title="Nyquist plot of L"
      readout={readout}
      legend={[
        { label: 'model', color: SERIES.model, mark: 'line' },
        { label: 'measured', color: SERIES.measured, mark: 'dot' },
      ]}
      summary={
        m ? (
          <span>
            closest approach to −1: {(1 / m.ms).toFixed(2)} (circle), so the peak of |S| is{' '}
            {m.ms.toFixed(2)}
          </span>
        ) : null
      }
    >
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </ChartCard>
  );
}
