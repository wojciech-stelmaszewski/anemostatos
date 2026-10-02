import { useEffect, useMemo, useState } from 'react';
import { dfQuadratic, dragLimitCycle, forceToVelocity } from '@/analysis/describing';
import { logspace } from '@/math/margins';
import { useParams } from '@/store/params';
import { sim, useUi } from '@/store/sim';
import { ChartCard } from './ChartCard';
import { dot, FONT, INK, SERIES, useChartCanvas } from './canvas';
import { loopKey } from './store';

/** Half the peak-to-peak altitude swing over the last ten seconds, and its frequency, of the flight. */
function flownCycle(): { amp: number; fHz: number } | null {
  const { t, series } = sim.telemetry.window(['pos.y'], sim.t - 10);
  const y = series[0]!;
  const ok = y.filter((v) => !Number.isNaN(v));
  if (ok.length < 200) return null;
  const hi = Math.max(...ok);
  const lo = Math.min(...ok);
  const mid = (hi + lo) / 2;
  const ups: number[] = [];
  for (let i = 1; i < y.length; i++) if (y[i - 1]! < mid && y[i]! >= mid) ups.push(t[i]!);
  const fHz = ups.length > 2 ? (ups.length - 1) / (ups[ups.length - 1]! - ups[0]!) : NaN;
  return { amp: (hi - lo) / 2, fHz };
}

/**
 * The describing-function picture of lesson III.8: the Nyquist curve of the loop as the air drag
 * sees it (force in, vertical velocity out, controller in place) and the locus −1/N(A) of
 * quadratic drag on the negative real axis. Where they meet, a limit cycle.
 */
export function DescribingChart() {
  const key = useParams((s) => loopKey(s.params));
  const c = useParams((s) => s.params.drone.dragV);
  const level = useParams((s) => s.params.sim.level);
  const data = useMemo(() => {
    const p = useParams.getState().params;
    const g = forceToVelocity(p);
    if (!g) return null;
    const f = logspace(0.05, 20, 600);
    return { curve: f.map((x) => g(2 * Math.PI * x)), f, cycle: dragLimitCycle(p) };
    // eslint-disable-next-line react-hooks/exhaustive-deps -- the key stands for the parameters
  }, [key]);
  const [flown, setFlown] = useState(flownCycle);
  // The flight's answer is shown only once a prediction has been entered (lesson III.8).
  const predicted = useUi((st) => st.prediction !== null || st.lesson !== 'describing');
  useEffect(() => {
    const id = setInterval(() => setFlown(flownCycle()), 500);
    return () => clearInterval(id);
  }, []);

  const { canvas, readout } = useChartCanvas(
    ({ ctx, w, h, hover }) => {
      ctx.font = FONT;
      if (level !== 1 || !data) {
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText('This view needs an L1 controller with a linear model.', w / 2, h / 3);
        return;
      }
      const cross = data.cycle ? -1 / data.cycle.n : -1;
      const half = Math.max(1.4 * Math.abs(cross), 0.5);
      const cx = cross * 0.55;
      const s = Math.min(w, h) / (2 * half);
      const X = (re: number) => w / 2 + (re - cx) * s;
      const Y = (im: number) => h / 2 - im * s;
      // Axes.
      ctx.strokeStyle = INK.grid;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(0, Y(0));
      ctx.lineTo(w, Y(0));
      ctx.moveTo(X(0), 0);
      ctx.lineTo(X(0), h);
      ctx.stroke();
      ctx.fillStyle = INK.muted;
      ctx.textAlign = 'right';
      ctx.textBaseline = 'top';
      ctx.fillText('G(jω), m/s per N', w - 4, 4);
      // The locus −1/N(A): the negative real axis, labelled by the velocity amplitude.
      ctx.strokeStyle = SERIES.measured;
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(X(-half * 3), Y(0));
      ctx.lineTo(X(0), Y(0));
      ctx.stroke();
      ctx.lineWidth = 1;
      ctx.textAlign = 'center';
      // Velocity amplitudes along the locus, labels alternating below and above the axis.
      [0.5, 1, 1.5, 2, 3, 5].forEach((a, i) => {
        const re = -1 / dfQuadratic(a, c);
        if (X(re) < 10 || X(re) > X(0) - 10) return;
        ctx.strokeStyle = SERIES.measured;
        ctx.beginPath();
        ctx.moveTo(X(re), Y(0) - 4);
        ctx.lineTo(X(re), Y(0) + 4);
        ctx.stroke();
        ctx.fillStyle = INK.muted;
        ctx.textBaseline = i % 2 ? 'bottom' : 'top';
        ctx.fillText(`${a} m/s`, X(re), i % 2 ? Y(0) - 7 : Y(0) + 7);
      });
      ctx.textBaseline = 'top';
      // The linear part.
      ctx.save();
      ctx.beginPath();
      ctx.rect(0, 0, w, h);
      ctx.clip();
      ctx.strokeStyle = SERIES.model;
      ctx.lineWidth = 2;
      ctx.beginPath();
      data.curve.forEach((v, i) =>
        i ? ctx.lineTo(X(v.re), Y(v.im)) : ctx.moveTo(X(v.re), Y(v.im)),
      );
      ctx.stroke();
      ctx.restore();
      // The crossing: the predicted limit cycle.
      if (data.cycle) {
        dot(ctx, X(cross), Y(0), INK.text, 4.5);
        const text = `crossing: ${cross.toFixed(2)} at ${data.cycle.fHz.toFixed(2)} Hz`;
        const tw = ctx.measureText(text).width;
        const tx = Math.min(Math.max(X(cross) - tw / 2, 4), w - tw - 4);
        ctx.fillStyle = 'rgba(11,14,19,0.85)';
        ctx.fillRect(tx - 2, Y(0) + 22, tw + 4, 14);
        ctx.fillStyle = INK.text;
        ctx.textAlign = 'left';
        ctx.fillText(text, tx, Y(0) + 23);
      }
      if (!hover) return;
      let best = 0;
      let d = Infinity;
      data.curve.forEach((v, i) => {
        const e = Math.hypot(X(v.re) - hover.x, Y(v.im) - hover.y);
        if (e < d) {
          d = e;
          best = i;
        }
      });
      const v = data.curve[best]!;
      dot(ctx, X(v.re), Y(v.im), SERIES.model, 3.5);
      return `${data.f[best]!.toPrecision(3)} Hz   G = ${v.re.toFixed(2)} ${v.im < 0 ? '−' : '+'} ${Math.abs(v.im).toFixed(2)}j`;
    },
    [data, c, level],
  );

  return (
    <ChartCard
      title="Describing function: the loop against −1/N(A) of the drag"
      readout={readout}
      legend={[
        { label: 'loop as the drag sees it', color: SERIES.model, mark: 'line' },
        { label: '−1/N(A), quadratic drag', color: SERIES.measured, mark: 'line' },
      ]}
      summary={
        <span>
          {data?.cycle
            ? `crossing at ${(-1 / data.cycle.n).toFixed(2)} on the real axis, ${data.cycle.fHz.toFixed(2)} Hz`
            : 'no crossing: no limit cycle predicted'}
          {flown && predicted
            ? ` · flown: ±${flown.amp.toFixed(2)} m at ${Number.isFinite(flown.fHz) ? flown.fHz.toFixed(2) : '—'} Hz`
            : ''}
        </span>
      }
    >
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </ChartCard>
  );
}
