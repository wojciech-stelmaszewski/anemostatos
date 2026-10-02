import { useEffect, useState } from 'react';
import { useParams } from '@/store/params';
import { sim } from '@/store/sim';
import { SIGNAL } from '@/ui/colors';
import { ChartCard } from './ChartCard';
import { dot, FONT, INK, niceTicks, useChartCanvas } from './canvas';

/** What the chart draws: the two paths seen from above, and the range now and at its least. */
interface Track {
  px: number[];
  pz: number[];
  tx: number[];
  tz: number[];
  range: number;
  minRange: number;
  over: boolean;
  losRate: number;
  closing: number;
}

const read = (): Track => {
  const w = sim.telemetry.window(['pos.x', 'pos.z', 'tgt.x', 'tgt.z'], 0);
  const keep = (a: number[]) => a.filter((v) => Number.isFinite(v));
  const e = sim.engagement;
  return {
    px: w.series[0]!,
    pz: w.series[1]!,
    tx: keep(w.series[2]!),
    tz: keep(w.series[3]!),
    range: e.last.range,
    minRange: e.minRange,
    over: e.over,
    losRate: (e.last.losRate * 180) / Math.PI,
    closing: e.last.closing,
  };
};

const COLOR = { drone: SIGNAL.measurement, target: SIGNAL.error } as const;

/**
 * The engagement of lesson IV.24 seen from above: the pursuer's path and the target's, with the
 * line of sight between them now. Pure pursuit curls in behind the target; proportional
 * navigation leads it on a nearly straight line.
 */
export function EngagementChart() {
  const [t, setT] = useState<Track>(read);
  const enabled = useParams((s) => s.params.guidance.enabled);
  const law = useParams((s) => s.params.guidance.law);
  useEffect(() => {
    const id = setInterval(() => setT(read()), 200);
    return () => clearInterval(id);
  }, []);

  const { canvas, readout } = useChartCanvas(
    ({ ctx, w, h }) => {
      ctx.font = FONT;
      const pts = [
        ...t.px.map((x, i) => [x, t.pz[i]!]),
        ...t.tx.map((x, i) => [x, t.tz[i]!]),
      ].filter(([a, b]) => Number.isFinite(a) && Number.isFinite(b));
      if (!enabled || pts.length < 2) {
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText('Switch on Guidance → Chase a target (level 3).', w / 2, h / 2);
        return;
      }
      const xs = pts.map((p) => p[0]!);
      const zs = pts.map((p) => p[1]!);
      const pad = 2;
      let x0 = Math.min(...xs) - pad;
      let x1 = Math.max(...xs) + pad;
      let z0 = Math.min(...zs) - pad;
      let z1 = Math.max(...zs) + pad;
      // Equal scales: a metre is a metre in both directions.
      const m = 30;
      const s = Math.min((w - 2 * m) / (x1 - x0), (h - 2 * m) / (z1 - z0));
      const cx = (x0 + x1) / 2;
      const cz = (z0 + z1) / 2;
      x0 = cx - (w - 2 * m) / s / 2;
      x1 = cx + (w - 2 * m) / s / 2;
      z0 = cz - (h - 2 * m) / s / 2;
      z1 = cz + (h - 2 * m) / s / 2;
      const X = (x: number) => m + (x - x0) * s;
      const Y = (z: number) => m + (z - z0) * s;
      ctx.lineWidth = 1;
      ctx.textBaseline = 'middle';
      for (const v of niceTicks(x0, x1, 6)) {
        ctx.strokeStyle = INK.grid;
        ctx.beginPath();
        ctx.moveTo(X(v), m);
        ctx.lineTo(X(v), h - m);
        ctx.stroke();
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText(`${v}`, X(v), h - m + 10);
      }
      for (const v of niceTicks(z0, z1, 5)) {
        ctx.strokeStyle = INK.grid;
        ctx.beginPath();
        ctx.moveTo(m, Y(v));
        ctx.lineTo(w - m, Y(v));
        ctx.stroke();
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'right';
        ctx.fillText(`${v}`, m - 4, Y(v));
      }
      ctx.textAlign = 'right';
      ctx.fillText('x, m', w - m, h - m + 22);
      const path = (xa: number[], za: number[], color: string) => {
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.lineJoin = 'round';
        ctx.beginPath();
        let started = false;
        xa.forEach((x, i) => {
          const z = za[i]!;
          if (!Number.isFinite(x) || !Number.isFinite(z)) return;
          if (started) ctx.lineTo(X(x), Y(z));
          else ctx.moveTo(X(x), Y(z));
          started = true;
        });
        ctx.stroke();
      };
      path(t.tx, t.tz, COLOR.target);
      path(t.px, t.pz, COLOR.drone);
      const pd = [t.px.at(-1)!, t.pz.at(-1)!];
      const td = [t.tx.at(-1)!, t.tz.at(-1)!];
      if (!t.over && Number.isFinite(td[0])) {
        ctx.strokeStyle = INK.axis;
        ctx.setLineDash([4, 3]);
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(X(pd[0]!), Y(pd[1]!));
        ctx.lineTo(X(td[0]!), Y(td[1]!));
        ctx.stroke();
        ctx.setLineDash([]);
      }
      if (Number.isFinite(td[0])) dot(ctx, X(td[0]!), Y(td[1]!), COLOR.target, 4);
      if (Number.isFinite(pd[0])) dot(ctx, X(pd[0]!), Y(pd[1]!), COLOR.drone, 4);
      return `range ${t.range.toFixed(2)} m   closing ${t.closing.toFixed(2)} m/s   line of sight turning ${t.losRate.toFixed(1)} °/s`;
    },
    [t, enabled],
  );

  return (
    <ChartCard
      title="The chase, seen from above"
      readout={readout}
      legend={[
        {
          label: `drone (${law === 'pn' ? 'proportional navigation' : 'pure pursuit'})`,
          color: COLOR.drone,
          mark: 'line',
        },
        { label: 'target', color: COLOR.target, mark: 'line' },
      ]}
      summary={
        enabled && Number.isFinite(t.minRange) ? (
          <span>
            {t.over ? 'over: ' : ''}closest approach {t.minRange.toFixed(2)} m
          </span>
        ) : null
      }
    >
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </ChartCard>
  );
}
