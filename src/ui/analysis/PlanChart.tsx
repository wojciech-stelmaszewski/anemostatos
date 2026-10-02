import { useEffect, useMemo, useState } from 'react';
import { DRONE_RADIUS, manoeuvreFor, manoeuvreSpec } from '@/guidance/collocation';
import { useParams } from '@/store/params';
import { sim } from '@/store/sim';
import { GRAVITY } from '@/sim/params';
import { SIGNAL } from '@/ui/colors';
import { ChartCard } from './ChartCard';
import { FONT, INK, niceTicks, useChartCanvas } from './canvas';

interface Flown {
  t: number[];
  x: number[];
  z: number[];
  ax: number[];
  az: number[];
  sat: number[];
  start: number;
}

const read = (): Flown => {
  const w = sim.telemetry.window(
    ['pos.x', 'pos.z', 'pos.x.u', 'pos.z.u', 'pos.x.sat', 'pos.z.sat'],
    0,
  );
  return {
    t: w.t,
    x: w.series[0]!,
    z: w.series[1]!,
    ax: w.series[2]!,
    az: w.series[3]!,
    sat: w.series[4]!.map((v, i) => Math.max(v, w.series[5]![i]!)),
    start: sim.profileStart + sim.params.plan.delay,
  };
};

const COLOR = { plan: SIGNAL.output, flown: SIGNAL.measurement, limit: SIGNAL.error } as const;

/**
 * Lesson IV.6: the manoeuvre from above (the pillar, its keep-out ring, the plan and the flight),
 * and the horizontal acceleration the plan asks for against what the tilt limit allows. Min-snap
 * does not know the limit; the transcription is built around it.
 */
export function PlanChart() {
  const p = useParams((s) => s.params);
  const [f, setF] = useState<Flown>(read);
  useEffect(() => {
    const id = setInterval(() => setF(read()), 200);
    return () => clearInterval(id);
  }, []);
  const plan = useMemo(() => (p.sim.level === 3 ? manoeuvreFor(p) : null), [p]);
  const spec = useMemo(() => manoeuvreSpec(p), [p]);
  const aH = GRAVITY * Math.tan((p.control.l3.maxTiltDeg * Math.PI) / 180);

  const { canvas, readout } = useChartCanvas(
    ({ ctx, w, h }) => {
      ctx.font = FONT;
      if (!plan || p.setpoint.profile !== 'plan') {
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText('Level 3, Setpoint → Automatic motion → planned manoeuvre.', w / 2, h / 2);
        return;
      }
      const bx = p.setpoint.x;
      const bz = p.setpoint.z;
      const left = 40;
      const right = 10;
      const split = Math.round(h * 0.55);

      // Top view, equal scales.
      const xs = [bx - 1, bx + spec.distance + 1];
      const zs = [bz - 2, bz + 2];
      const y0 = 8;
      const y1 = split - 20;
      const s = Math.min((w - left - right) / (xs[1]! - xs[0]!), (y1 - y0) / (zs[1]! - zs[0]!));
      const cx = (xs[0]! + xs[1]!) / 2;
      const cz = (zs[0]! + zs[1]!) / 2;
      const X = (x: number) => (left + w - right) / 2 + (x - cx) * s;
      const Z = (z: number) => (y0 + y1) / 2 + (z - cz) * s;
      ctx.lineWidth = 1;
      ctx.textBaseline = 'top';
      for (const v of niceTicks(xs[0]!, xs[1]!, 8)) {
        ctx.strokeStyle = INK.grid;
        ctx.beginPath();
        ctx.moveTo(X(v), Z(zs[0]!));
        ctx.lineTo(X(v), Z(zs[1]!));
        ctx.stroke();
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText(`${v}`, X(v), Z(zs[1]!) + 3);
      }
      ctx.textAlign = 'right';
      ctx.fillText('x, m (from above)', w - right, Z(zs[1]!) + 3);
      if (p.world.pillar) {
        ctx.fillStyle = 'rgba(230,103,103,0.25)';
        ctx.beginPath();
        ctx.arc(X(p.world.pillarX), Z(p.world.pillarZ), p.world.pillarR * s, 0, 2 * Math.PI);
        ctx.fill();
        ctx.strokeStyle = COLOR.limit;
        ctx.setLineDash([4, 3]);
        ctx.beginPath();
        ctx.arc(
          X(p.world.pillarX),
          Z(p.world.pillarZ),
          (p.world.pillarR + DRONE_RADIUS) * s,
          0,
          2 * Math.PI,
        );
        ctx.stroke();
        ctx.setLineDash([]);
      }
      const n = 120;
      ctx.strokeStyle = COLOR.plan;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      for (let i = 0; i <= n; i++) {
        const r = plan.at((i / n) * plan.duration);
        const px = X(bx + r.pos.x);
        const pz = Z(bz + r.pos.z);
        if (i) ctx.lineTo(px, pz);
        else ctx.moveTo(px, pz);
      }
      ctx.stroke();
      ctx.strokeStyle = COLOR.flown;
      ctx.lineWidth = 2;
      ctx.beginPath();
      let on = false;
      f.t.forEach((t, i) => {
        if (t < f.start - 0.2) return;
        const x = f.x[i]!;
        const z = f.z[i]!;
        if (!Number.isFinite(x) || !Number.isFinite(z)) return;
        if (on) ctx.lineTo(X(x), Z(z));
        else ctx.moveTo(X(x), Z(z));
        on = true;
      });
      ctx.stroke();

      // Horizontal acceleration: planned, commanded, and the limits.
      const t0 = split + 6;
      const t1 = h - 20;
      const tEnd = plan.duration + 1;
      const aTop = Math.max(aH, plan.peakAccel) * 1.15;
      const T = (t: number) => left + (t / tEnd) * (w - left - right);
      const A = (a: number) => t1 - (a / aTop) * (t1 - t0);
      ctx.textBaseline = 'middle';
      for (const v of niceTicks(0, aTop, 4)) {
        ctx.strokeStyle = INK.grid;
        ctx.beginPath();
        ctx.moveTo(left, A(v));
        ctx.lineTo(w - right, A(v));
        ctx.stroke();
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'right';
        ctx.fillText(`${v}`, left - 4, A(v));
      }
      ctx.textBaseline = 'top';
      for (const v of niceTicks(0, tEnd, 6)) {
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText(`${v}`, T(v), t1 + 3);
      }
      ctx.textAlign = 'right';
      ctx.fillText('time into the manoeuvre, s', w - right, t1 + 12);
      ctx.textAlign = 'left';
      ctx.fillText('|a| horizontal, m/s²', left + 4, t0);
      const hline = (a: number, dash: number[]) => {
        ctx.strokeStyle = COLOR.limit;
        ctx.lineWidth = 1;
        ctx.setLineDash(dash);
        ctx.beginPath();
        ctx.moveTo(left, A(a));
        ctx.lineTo(w - right, A(a));
        ctx.stroke();
        ctx.setLineDash([]);
      };
      hline(aH, []);
      hline(spec.accelMax, [4, 3]);
      // Saturated samples of the flight, shaded.
      ctx.fillStyle = 'rgba(230,103,103,0.18)';
      f.t.forEach((t, i) => {
        const tt = t - f.start;
        if (tt < 0 || tt > tEnd || !(f.sat[i]! > 0)) return;
        ctx.fillRect(T(tt) - 1, t0, 2, t1 - t0);
      });
      ctx.strokeStyle = COLOR.plan;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      for (let i = 0; i <= n; i++) {
        const tt = (i / n) * plan.duration;
        const r = plan.at(tt);
        const y = A(Math.hypot(r.acc.x, r.acc.z));
        if (i) ctx.lineTo(T(tt), y);
        else ctx.moveTo(T(tt), y);
      }
      ctx.stroke();
      ctx.strokeStyle = COLOR.flown;
      ctx.lineWidth = 2;
      ctx.beginPath();
      on = false;
      f.t.forEach((t, i) => {
        const tt = t - f.start;
        if (tt < 0 || tt > tEnd) return;
        const a = Math.hypot(f.ax[i]!, f.az[i]!);
        if (!Number.isFinite(a)) return;
        if (on) ctx.lineTo(T(tt), A(a));
        else ctx.moveTo(T(tt), A(a));
        on = true;
      });
      ctx.stroke();

      const what = plan.method === 'minsnap' ? 'min-snap' : 'transcription';
      if (plan.status !== 'optimal')
        return `${what}: no plan within the limits in ${plan.duration.toFixed(2)} s — the drone holds the start`;
      return `${what}: ${plan.duration.toFixed(2)} s · peak ${plan.peakAccel.toFixed(2)} m/s² (tilt limit ${aH.toFixed(2)}) · jerk ${plan.peakJerk.toFixed(1)} m/s³ · clearance ${plan.clearance.toFixed(2)} m`;
    },
    [f, plan, spec, aH, p],
  );

  return (
    <ChartCard
      title="Planned manoeuvre: from above, and the acceleration it asks for"
      readout={readout}
      legend={[
        { label: 'plan', color: COLOR.plan, mark: 'line' },
        { label: 'flown / commanded', color: COLOR.flown, mark: 'line' },
        { label: 'pillar, tilt limit, plan limit', color: COLOR.limit, mark: 'line' },
      ]}
    >
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </ChartCard>
  );
}
