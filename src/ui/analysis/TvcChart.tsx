import { useMemo } from 'react';
import { analyseTvc, type TvcAnalysis } from '@/analysis/tvc';
import { cabs } from '@/math/complex';
import type { Params } from '@/sim/params';
import { useParams } from '@/store/params';
import { ChartCard } from './ChartCard';
import { dot, FONT, INK, SERIES, useChartCanvas } from './canvas';

/** What the thrust-vector loop depends on: the vehicle and its controller. */
const tvcKey = (p: Params): string => JSON.stringify([p.tvc, p.control.tvc]);

function useTvc(): TvcAnalysis {
  const key = useParams((s) => tvcKey(s.params));
  // eslint-disable-next-line react-hooks/exhaustive-deps -- the key stands for the parameters
  return useMemo(() => analyseTvc(useParams.getState().params), [key]);
}

const db = (v: number) => (v >= 0 ? '+' : '−') + Math.abs(v).toFixed(1) + ' dB';

/**
 * The loop of the pitch-plane rocket broken at the gimbal (Chapter L). Above, the Nyquist plot:
 * the airframe is unstable, so the curve must encircle −1 (counter-clockwise, once for each
 * unstable pole), and it crosses the negative real axis twice, once on each side of −1: a lower
 * and an upper gain margin. Below, |L| against frequency, where the bending mode is a spike.
 */
export function TvcChart() {
  const a = useTvc();
  return (
    <div className="flex h-full min-h-0 flex-col gap-1.5">
      <div className="min-h-0 flex-[3]">
        <TvcNyquist a={a} />
      </div>
      <div className="min-h-0 flex-[2]">
        <TvcMagnitude a={a} />
      </div>
    </div>
  );
}

function TvcNyquist({ a }: { a: TvcAnalysis }) {
  const { canvas, readout } = useChartCanvas(
    ({ ctx, w, h, hover }) => {
      // The window: from beyond the lower crossing to a little right of the origin.
      const left = a.stable && a.low > 0 ? Math.min(-1 / a.low - 0.6, -2.2) : -3;
      const right = 1.2;
      const half = Math.max((right - left) / 2, 1.2);
      const cx = (left + right) / 2;
      const s = Math.min(w, h) / (2 * half);
      const X = (re: number) => w / 2 + (re - cx) * s;
      const Y = (im: number) => h / 2 - im * s;
      ctx.font = FONT;
      ctx.lineWidth = 1;
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

      // The curve for positive frequencies, and its mirror image (negative frequencies) faint.
      ctx.save();
      ctx.beginPath();
      ctx.rect(0, 0, w, h);
      ctx.clip();
      ctx.lineJoin = 'round';
      for (const mirror of [1, -1]) {
        ctx.strokeStyle = SERIES.model;
        ctx.globalAlpha = mirror === 1 ? 1 : 0.35;
        ctx.lineWidth = mirror === 1 ? 2 : 1;
        ctx.setLineDash(mirror === 1 ? [] : [4, 3]);
        ctx.beginPath();
        a.l.forEach((v, i) => {
          // Far lobes are drawn to the edge of the window and no further.
          const k = Math.min(1, (4 * half) / Math.max(cabs(v), 1e-9));
          const px = X(v.re * k);
          const py = Y(mirror * v.im * k);
          if (i) ctx.lineTo(px, py);
          else ctx.moveTo(px, py);
        });
        ctx.stroke();
      }
      ctx.restore();
      ctx.globalAlpha = 1;
      ctx.setLineDash([]);

      // The critical point and the two crossings of the negative real axis.
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
      if (a.stable) {
        ctx.textBaseline = 'bottom';
        if (a.low > 0) {
          dot(ctx, X(-1 / a.low), Y(0), SERIES.measured, 3.5);
          ctx.fillText(`lower ${db(-a.lowDb)}`, X(-1 / a.low), Y(0) - 6);
        }
        if (Number.isFinite(a.high)) {
          dot(ctx, X(-1 / a.high), Y(0), SERIES.measured, 3.5);
          ctx.fillText(`upper ${db(a.highDb)}`, X(-1 / a.high), Y(0) - 18);
        }
      }

      if (!hover) return;
      let best = 0;
      let d = Infinity;
      a.l.forEach((v, i) => {
        const e = Math.hypot(X(v.re) - hover.x, Y(v.im) - hover.y);
        if (e < d) {
          d = e;
          best = i;
        }
      });
      const v = a.l[best]!;
      dot(ctx, X(v.re), Y(v.im), SERIES.model, 3.5);
      return `${a.fHz[best]!.toPrecision(3)} Hz   |L| ${cabs(v).toFixed(2)}   to −1: ${Math.hypot(v.re + 1, v.im).toFixed(2)}`;
    },
    [a],
  );
  return (
    <ChartCard
      title="Nyquist plot of the gimbal loop"
      readout={readout}
      legend={[
        { label: 'L(jω), ω > 0', color: SERIES.model, mark: 'line' },
        { label: 'gain margins', color: SERIES.measured, mark: 'dot' },
      ]}
      summary={
        a.stable ? (
          <span>
            stable for loop gains × {a.low.toFixed(2)} to ×{' '}
            {Number.isFinite(a.high) ? a.high.toFixed(2) : '∞'}: margins {db(-a.lowDb)} and{' '}
            {db(a.highDb)}; phase margin {a.margins.pmDeg.toFixed(0)}°
          </span>
        ) : (
          <span>closed loop unstable</span>
        )
      }
    >
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </ChartCard>
  );
}

const LOG_MIN = Math.log10(0.02);
const LOG_MAX = Math.log10(50);

function TvcMagnitude({ a }: { a: TvcAnalysis }) {
  const { canvas, readout } = useChartCanvas(
    ({ ctx, w, h, hover }) => {
      const m = a.l.map((v) => 20 * Math.log10(Math.max(cabs(v), 1e-12)));
      const lo = -40;
      const hi = 40;
      const pad = { l: 34, r: 8, t: 6, b: 16 };
      const X = (f: number) =>
        pad.l + ((Math.log10(f) - LOG_MIN) / (LOG_MAX - LOG_MIN)) * (w - pad.l - pad.r);
      const Y = (v: number) =>
        pad.t + ((hi - Math.min(Math.max(v, lo), hi)) / (hi - lo)) * (h - pad.t - pad.b);
      ctx.font = FONT;
      ctx.lineWidth = 1;
      ctx.fillStyle = INK.muted;
      ctx.textAlign = 'right';
      ctx.textBaseline = 'middle';
      for (let v = lo; v <= hi; v += 20) {
        ctx.strokeStyle = v === 0 ? INK.axis : INK.grid;
        ctx.beginPath();
        ctx.moveTo(pad.l, Y(v));
        ctx.lineTo(w - pad.r, Y(v));
        ctx.stroke();
        ctx.fillText(`${v}`, pad.l - 4, Y(v));
      }
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      for (const f of [0.1, 1, 10]) {
        ctx.strokeStyle = INK.grid;
        ctx.beginPath();
        ctx.moveTo(X(f), pad.t);
        ctx.lineTo(X(f), h - pad.b);
        ctx.stroke();
        ctx.fillText(`${f} Hz`, X(f), h - pad.b + 2);
      }
      // The bending frequency.
      ctx.setLineDash([3, 3]);
      ctx.strokeStyle = INK.axis;
      ctx.beginPath();
      ctx.moveTo(X(a.bendHz), pad.t);
      ctx.lineTo(X(a.bendHz), h - pad.b);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.strokeStyle = SERIES.model;
      ctx.lineWidth = 2;
      ctx.beginPath();
      a.fHz.forEach((f, i) => (i ? ctx.lineTo(X(f), Y(m[i]!)) : ctx.moveTo(X(f), Y(m[i]!))));
      ctx.stroke();
      dot(ctx, X(a.bendHz), Y(a.bendDb), SERIES.measured, 3.5);

      if (!hover) return;
      const lf = LOG_MIN + ((hover.x - pad.l) / (w - pad.l - pad.r)) * (LOG_MAX - LOG_MIN);
      let best = 0;
      a.fHz.forEach((f, i) => {
        if (Math.abs(Math.log10(f) - lf) < Math.abs(Math.log10(a.fHz[best]!) - lf)) best = i;
      });
      dot(ctx, X(a.fHz[best]!), Y(m[best]!), SERIES.model, 3.5);
      return `${a.fHz[best]!.toPrecision(3)} Hz   |L| ${m[best]!.toFixed(1)} dB`;
    },
    [a],
  );
  return (
    <ChartCard
      title="|L| in dB"
      readout={readout}
      legend={[
        { label: '|L|', color: SERIES.model, mark: 'line' },
        { label: 'at the bending mode', color: SERIES.measured, mark: 'dot' },
      ]}
      summary={
        <span>
          bending mode {a.bendHz.toFixed(1)} Hz: |L| {db(a.bendDb)}
        </span>
      }
    >
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </ChartCard>
  );
}
