import { useEffect, useState } from 'react';
import { sim } from '@/store/sim';
import { SIGNAL } from '@/ui/colors';
import { ChartCard } from './ChartCard';
import { dot, FONT, INK, niceTicks, useChartCanvas } from './canvas';

interface Path {
  y: number[];
  x: number[];
  plan: { x: number; y: number }[];
  dv: number;
  speed: number;
  minutes: number;
  docked: boolean;
  hit: boolean;
  contact: number;
}

const read = (): Path | null => {
  const c = sim.chaser;
  if (!c) return null;
  const w = sim.telemetry.window(['rv.x', 'rv.y'], 0);
  return {
    x: w.series[0]!,
    y: w.series[1]!,
    plan: sim.params.rendezvous.control === 'mpc' ? sim.rvMpc.plan : [],
    dv: c.dv,
    speed: Math.hypot(c.vx, c.vy),
    minutes: c.t / 60,
    docked: c.docked,
    hit: c.hit,
    contact: c.contactSpeed,
  };
};

const COLOR = { chaser: SIGNAL.measurement, plan: SIGNAL.setpoint, target: SIGNAL.error } as const;

/**
 * Lesson IV.25 in the target's frame: along-track to the right (the direction of flight), radial
 * up. A forward burn makes the chaser climb and fall behind; the MPC's plan bends with the orbit.
 */
export function RendezvousChart() {
  const [p, setP] = useState<Path | null>(read);
  useEffect(() => {
    const id = setInterval(() => setP(read()), 200);
    return () => clearInterval(id);
  }, []);

  const { canvas, readout } = useChartCanvas(
    ({ ctx, w, h }) => {
      ctx.font = FONT;
      if (!p) {
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText('Lesson IV.25: Vehicle → chaser near a target in orbit.', w / 2, h / 2);
        return;
      }
      const ys = [...p.y, ...p.plan.map((q) => q.y), 0].filter(Number.isFinite);
      const xs = [...p.x, ...p.plan.map((q) => q.x), 0].filter(Number.isFinite);
      const pad = 10;
      let y0 = Math.min(...ys) - pad;
      let y1 = Math.max(...ys) + pad;
      let x0 = Math.min(...xs) - pad;
      let x1 = Math.max(...xs) + pad;
      // Equal scales.
      const m = 32;
      const s = Math.min((w - 2 * m) / (y1 - y0), (h - 2 * m) / (x1 - x0));
      const cy = (y0 + y1) / 2;
      const cx = (x0 + x1) / 2;
      y0 = cy - (w - 2 * m) / s / 2;
      y1 = cy + (w - 2 * m) / s / 2;
      x0 = cx - (h - 2 * m) / s / 2;
      x1 = cx + (h - 2 * m) / s / 2;
      const X = (along: number) => m + (along - y0) * s;
      const Y = (radial: number) => h - m - (radial - x0) * s;
      ctx.lineWidth = 1;
      ctx.textBaseline = 'middle';
      for (const v of niceTicks(y0, y1, 6)) {
        ctx.strokeStyle = INK.grid;
        ctx.beginPath();
        ctx.moveTo(X(v), m);
        ctx.lineTo(X(v), h - m);
        ctx.stroke();
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText(`${v}`, X(v), h - m + 10);
      }
      for (const v of niceTicks(x0, x1, 5)) {
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
      ctx.fillText('along-track, m →', w - m, h - m + 22);
      ctx.textAlign = 'left';
      ctx.fillText('radial (up), m', m + 2, m - 10);
      const line = (pts: [number, number][], color: string, width: number, dash: number[]) => {
        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        ctx.setLineDash(dash);
        ctx.beginPath();
        pts.forEach(([a, r], i) => (i ? ctx.lineTo(X(a), Y(r)) : ctx.moveTo(X(a), Y(r))));
        ctx.stroke();
        ctx.setLineDash([]);
      };
      const path: [number, number][] = [];
      p.y.forEach((a, i) => {
        if (Number.isFinite(a) && Number.isFinite(p.x[i]!)) path.push([a, p.x[i]!]);
      });
      if (p.plan.length) {
        const now = path.at(-1) ?? [0, 0];
        line([now, ...p.plan.map((q): [number, number] => [q.y, q.x])], COLOR.plan, 1.5, [4, 3]);
      }
      line(path, COLOR.chaser, 2, []);
      // The target: a square at the origin.
      ctx.fillStyle = COLOR.target;
      ctx.fillRect(X(0) - 4, Y(0) - 4, 8, 8);
      const last = path.at(-1);
      if (last) dot(ctx, X(last[0]), Y(last[1]), COLOR.chaser, 4);
      return `${p.minutes.toFixed(1)} min   speed ${(p.speed * 100).toFixed(1)} cm/s   Δv spent ${p.dv.toFixed(3)} m/s`;
    },
    [p],
  );

  return (
    <ChartCard
      title="The approach, in the target’s frame"
      readout={readout}
      legend={[
        { label: 'chaser', color: COLOR.chaser, mark: 'line' },
        { label: 'MPC plan', color: COLOR.plan, mark: 'line' },
        { label: 'target', color: COLOR.target, mark: 'dot' },
      ]}
      summary={
        p ? (
          <span>
            {p.docked
              ? `docked at ${(p.contact * 100).toFixed(1)} cm/s`
              : p.hit
                ? `hit the target at ${(p.contact * 100).toFixed(0)} cm/s`
                : `Δv ${p.dv.toFixed(3)} m/s`}
          </span>
        ) : null
      }
    >
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </ChartCard>
  );
}
