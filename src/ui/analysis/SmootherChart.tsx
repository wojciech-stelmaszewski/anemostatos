import { useEffect, useState } from 'react';
import { reconstructAltitude, type Reconstruction } from '@/analysis/smoother';
import { useParams } from '@/store/params';
import { sim } from '@/store/sim';
import { SIGNAL } from '@/ui/colors';
import { ChartCard } from './ChartCard';
import { FONT, INK, niceTicks, useChartCanvas } from './canvas';

const COLOR = { filter: SIGNAL.error, smoother: SIGNAL.measurement } as const;

/**
 * Lesson IV.29: the altitude error of the forward Kalman filter and of the RTS smoother over the
 * whole record, with each one's σ. The smoother is recomputed over everything recorded so far, so
 * its last instant always equals the filter's.
 */
export function SmootherChart() {
  const accel = useParams((s) => s.params.smoother.accelSigma);
  const [r, setR] = useState<Reconstruction | null>(() => reconstructAltitude(sim));
  useEffect(() => {
    const id = setInterval(() => setR(reconstructAltitude(sim)), 500);
    return () => clearInterval(id);
  }, []);

  const { canvas, readout } = useChartCanvas(
    ({ ctx, w, h }) => {
      ctx.font = FONT;
      if (!r) {
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText('Lesson IV.29: a noisy altimeter and the reconstruction on.', w / 2, h / 2);
        return;
      }
      const m = { l: 44, r: 10, t: 10, b: 22 };
      const t0 = r.t[0]!;
      const t1 = Math.max(r.t.at(-1)!, t0 + 1);
      const lim = Math.max(3 * r.rmsF, 2 * Math.max(...r.sf.slice(10)), 1e-3);
      const X = (t: number) => m.l + ((t - t0) / (t1 - t0)) * (w - m.l - m.r);
      const Y = (v: number) =>
        m.t + (1 - (Math.max(-lim, Math.min(lim, v)) + lim) / (2 * lim)) * (h - m.t - m.b);
      ctx.lineWidth = 1;
      ctx.textBaseline = 'middle';
      for (const v of niceTicks(-lim * 100, lim * 100, 5)) {
        ctx.strokeStyle = v === 0 ? INK.axis : INK.grid;
        ctx.beginPath();
        ctx.moveTo(m.l, Y(v / 100));
        ctx.lineTo(w - m.r, Y(v / 100));
        ctx.stroke();
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'right';
        ctx.fillText(`${v} cm`, m.l - 4, Y(v / 100));
      }
      ctx.textAlign = 'center';
      for (const v of niceTicks(t0, t1, 6)) ctx.fillText(`${v}s`, X(v), h - m.b + 12);
      // Draw a series, thinned to about one point per pixel.
      const step = Math.max(1, Math.floor(r.t.length / (w - m.l - m.r)));
      const line = (ys: number[], color: string, width: number, dash: number[] = []) => {
        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        ctx.setLineDash(dash);
        ctx.beginPath();
        for (let i = 0; i < r.t.length; i += step) {
          const x = X(r.t[i]!);
          const y = Y(ys[i]!);
          if (i === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        }
        ctx.stroke();
        ctx.setLineDash([]);
      };
      line(r.ef, COLOR.filter, 1.5);
      line(r.es, COLOR.smoother, 1.5);
      line(r.sf, COLOR.filter, 1, [4, 3]);
      line(
        r.sf.map((v) => -v),
        COLOR.filter,
        1,
        [4, 3],
      );
      line(r.ss, COLOR.smoother, 1, [4, 3]);
      line(
        r.ss.map((v) => -v),
        COLOR.smoother,
        1,
        [4, 3],
      );
      return `RMS error: filter ${(r.rmsF * 100).toFixed(2)} cm, smoother ${(r.rmsS * 100).toFixed(2)} cm (${((100 * r.rmsS) / r.rmsF).toFixed(0)} %)`;
    },
    [r, accel],
  );

  return (
    <ChartCard
      title="Altitude error after the flight: filter against smoother (dashed: ±σ)"
      readout={readout}
      legend={[
        { label: 'Kalman filter (the past only)', color: COLOR.filter, mark: 'line' },
        { label: 'RTS smoother (past and future)', color: COLOR.smoother, mark: 'line' },
      ]}
      summary={
        r ? (
          <span>
            RMS {(r.rmsF * 100).toFixed(2)} → {(r.rmsS * 100).toFixed(2)} cm
          </span>
        ) : null
      }
    >
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </ChartCard>
  );
}
