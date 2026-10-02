import { useEffect, useState } from 'react';
import { landingKey, type LandingRun } from '@/guidance/pdgCampaign';
import type { PdgPlan } from '@/guidance/pdg';
import { useParams } from '@/store/params';
import { sim } from '@/store/sim';
import { SIGNAL } from '@/ui/colors';
import { Button } from '@/ui/components/button';
import { ChartCard } from './ChartCard';
import { FONT, INK, niceTicks, useChartCanvas } from './canvas';
import { useLandings } from './landingStore';

interface Flight {
  t: number[];
  range: number[];
  alt: number[];
  thrust: number[];
  first: PdgPlan | null;
  plan: PdgPlan | null;
  planStart: number;
  efficiency: number;
}

const read = (): Flight => {
  const w = sim.telemetry.window(['pdg.range', 'pos.y', 'pdg.thrust'], 0);
  return {
    t: w.t,
    range: w.series[0]!,
    alt: w.series[1]!,
    thrust: w.series[2]!,
    first: sim.pdg.first,
    plan: sim.pdg.plan,
    planStart: sim.pdg.planStart,
    efficiency: sim.pdg.efficiency,
  };
};

const COLOR = { flown: SIGNAL.measurement, plan: SIGNAL.output, first: SIGNAL.setpoint } as const;

/**
 * Lesson IV.5: the descent seen from the side (distance from the pad against altitude) inside the
 * glide-slope cone, with the first plan and the current one; below it the thrust against its
 * limits, which the fuel-optimal plan rides: full, least, full. A campaign of dispersed starts
 * is flown from the button.
 */
export function PdgChart() {
  const [f, setF] = useState<Flight>(read);
  const p = useParams((s) => s.params);
  const store = useLandings();
  const key = landingKey(p);
  const runs: LandingRun[] = store.key === key ? store.runs : [];
  useEffect(() => {
    const id = setInterval(() => setF(read()), 200);
    return () => clearInterval(id);
  }, []);
  const l = p.lander;

  const { canvas, readout } = useChartCanvas(
    ({ ctx, w, h }) => {
      ctx.font = FONT;
      if (p.sim.vehicle !== 'lander') {
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText('Select the 3D lander (Vehicle → lander).', w / 2, h / 2);
        return;
      }
      const left = 42;
      const right = 10;
      const split = Math.round(h * 0.6);
      const r0 = Math.hypot(l.start.x, l.start.z);
      const xMax = Math.max(r0 + l.dispersePos, ...f.range.filter(Number.isFinite)) * 1.05;
      const yMax = Math.max(l.start.y + l.dispersePos / 2, ...f.alt.filter(Number.isFinite)) * 1.05;

      const axes = (
        y0: number,
        y1: number,
        xlo: number,
        xhi: number,
        ylo: number,
        yhi: number,
        xlabel: string,
        ylabel: string,
      ) => {
        const X = (v: number) => left + ((v - xlo) / (xhi - xlo)) * (w - left - right);
        const Y = (v: number) => y1 - ((v - ylo) / (yhi - ylo)) * (y1 - y0);
        ctx.lineWidth = 1;
        ctx.textBaseline = 'middle';
        for (const v of niceTicks(ylo, yhi, 4)) {
          ctx.strokeStyle = INK.grid;
          ctx.beginPath();
          ctx.moveTo(left, Y(v));
          ctx.lineTo(w - right, Y(v));
          ctx.stroke();
          ctx.fillStyle = INK.muted;
          ctx.textAlign = 'right';
          ctx.fillText(`${+v.toPrecision(3)}`, left - 4, Y(v));
        }
        ctx.textBaseline = 'top';
        for (const v of niceTicks(xlo, xhi, 6)) {
          ctx.strokeStyle = INK.grid;
          ctx.beginPath();
          ctx.moveTo(X(v), y0);
          ctx.lineTo(X(v), y1);
          ctx.stroke();
          ctx.fillStyle = INK.muted;
          ctx.textAlign = 'center';
          ctx.fillText(`${+v.toPrecision(3)}`, X(v), y1 + 3);
        }
        ctx.textAlign = 'right';
        ctx.fillText(xlabel, w - right, y1 + 14);
        ctx.textAlign = 'left';
        ctx.textBaseline = 'top';
        ctx.fillText(ylabel, left + 4, y0 + 2);
        return { X, Y };
      };
      const line = (
        xs: readonly number[],
        ys: readonly number[],
        X: (v: number) => number,
        Y: (v: number) => number,
        color: string,
        width = 2,
        dash: number[] = [],
      ) => {
        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        ctx.setLineDash(dash);
        ctx.beginPath();
        let on = false;
        xs.forEach((x, i) => {
          const y = ys[i]!;
          if (!Number.isFinite(x) || !Number.isFinite(y)) return;
          if (on) ctx.lineTo(X(x), Y(y));
          else ctx.moveTo(X(x), Y(y));
          on = true;
        });
        ctx.stroke();
        ctx.setLineDash([]);
      };

      // Side view with the glide-slope boundary: altitude = tan γ × distance.
      const side = axes(6, split - 22, 0, xMax, 0, yMax, 'distance from the pad, m', 'altitude, m');
      const tg = Math.tan((l.glideSlopeDeg * Math.PI) / 180);
      ctx.fillStyle = 'rgba(230,103,103,0.08)';
      ctx.beginPath();
      ctx.moveTo(side.X(0), side.Y(0));
      ctx.lineTo(side.X(xMax), side.Y(Math.min(yMax, tg * xMax)));
      ctx.lineTo(side.X(xMax), side.Y(0));
      ctx.closePath();
      ctx.fill();
      line([0, xMax], [0, tg * xMax], side.X, side.Y, SIGNAL.error, 1, [4, 3]);
      const planSide = (pl: PdgPlan | null, color: string, dash: number[]) => {
        if (!pl || pl.status !== 'optimal') return;
        line(
          pl.r.map((r) => Math.hypot(r.x, r.z)),
          pl.r.map((r) => r.y),
          side.X,
          side.Y,
          color,
          1.5,
          dash,
        );
      };
      planSide(f.first, COLOR.first, [5, 4]);
      planSide(f.plan, COLOR.plan, []);
      line(f.range, f.alt, side.X, side.Y, COLOR.flown, 2);

      // Thrust against its limits.
      const tEnd = Math.max(f.t.at(-1) ?? 0, f.first?.tf ?? 0, 1) * 1.05;
      const th = axes(split + 6, h - 22, 0, tEnd, 0, l.thrustMax * 1.1, 'time, s', 'thrust, N');
      line([0, tEnd], [l.thrustMax, l.thrustMax], th.X, th.Y, SIGNAL.error, 1, [4, 3]);
      line([0, tEnd], [l.thrustMin, l.thrustMin], th.X, th.Y, SIGNAL.error, 1, [4, 3]);
      const fp = f.first;
      if (fp && fp.status === 'optimal') {
        const xs: number[] = [];
        const ys: number[] = [];
        fp.sigma.forEach((s, k) => {
          xs.push(k * fp.dt, (k + 1) * fp.dt);
          ys.push(s * fp.m[k]!, s * fp.m[k]!);
        });
        line(xs, ys, th.X, th.Y, COLOR.first, 1.5, [5, 4]);
      }
      line(f.t, f.thrust, th.X, th.Y, COLOR.flown, 2);

      if (f.first?.status === 'infeasible')
        return 'the guidance found no landing from this start: it is reported, and the lander brakes as best it can';
      if (!f.first) return 'press Start: the guidance plans as the flight begins';
      return `first plan: ${f.first.tf.toFixed(2)} s, ${f.first.fuel.toFixed(3)} kg · ‖u‖ = σ within ${(f.first.slackGap * 100).toExponential(1)} % · ${f.first.solves} cone programs · engine measured at ${(f.efficiency * 100).toFixed(1)} %`;
    },
    [f, l, p.sim.vehicle],
  );

  const tally = (o: LandingRun['outcome']) => runs.filter((r) => r.outcome === o).length;
  const failed = runs.filter((r) => !r.passed);
  return (
    <ChartCard
      title="Powered descent: side view and thrust"
      readout={readout}
      legend={[
        { label: 'flown', color: COLOR.flown, mark: 'line' },
        { label: 'current plan', color: COLOR.plan, mark: 'line' },
        { label: 'first plan', color: COLOR.first, mark: 'line' },
        { label: 'glide slope, thrust limits', color: SIGNAL.error, mark: 'line' },
      ]}
      summary={
        runs.length ? (
          <span>
            {runs.length} cases: {tally('landed')} landed · {tally('infeasible')} reported
            infeasible · {tally('crashed')} crashed · {tally('missed')} off the pad ·{' '}
            {tally('cone')} left the cone
            {failed.length
              ? ` (failed: ${failed
                  .slice(0, 6)
                  .map((r) => r.case)
                  .join(', ')})`
              : ''}
          </span>
        ) : (
          <span>
            dispersed starts: ±{l.dispersePos} m, ±{l.disperseVel} m/s
          </span>
        )
      }
      controls={
        <Button
          size="sm"
          className="h-5 px-2 text-[11px]"
          disabled={store.running || p.sim.vehicle !== 'lander'}
          onClick={() => store.start()}
          title="Fly the dispersed starts 1 to 20 with this guidance, engine and wind."
        >
          {store.running ? `flying ${runs.length} / 20` : 'Fly 20 cases'}
        </Button>
      }
    >
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </ChartCard>
  );
}
