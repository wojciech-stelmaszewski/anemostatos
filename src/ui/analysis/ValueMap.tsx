import { useEffect, useMemo, useState } from 'react';
import { designLqr } from '@/control/lqr';
import { dpModel } from '@/control/dp';
import { bestInput, gridX, gridY } from '@/math/grid';
import { useParams } from '@/store/params';
import { sim } from '@/store/sim';
import { ChartCard } from './ChartCard';
import { dot, FONT, INK, SERIES, useChartCanvas } from './canvas';

const LEFT = 40;
const RIGHT = 10;
const TOP = 8;
const BOTTOM = 28;

/** Sequential, one hue: low cost light, high cost dark (on the dark surface: dark → bright). */
const shade = (f: number) => {
  const t = Math.min(Math.max(f, 0), 1);
  const l = 14 + 52 * t;
  return `hsl(212, 70%, ${l}%)`;
};

/**
 * The value function V(e, v) of the L1 drone as a heat map (lesson IV.3): the cost of the best
 * possible future from each state, solved by value iteration. The policy is drawn as arrows (the
 * direction the state moves under the chosen thrust), the drone as a dot, and the two lines where
 * the LQR's command reaches the motors' limits.
 */
export function ValueMap() {
  const key = useParams((s) =>
    JSON.stringify([
      s.params.control.lqr,
      s.params.control.dp,
      s.params.control.model,
      s.params.drone,
    ]),
  );
  // eslint-disable-next-line react-hooks/exhaustive-deps -- the key stands for the parameters
  const model = useMemo(() => dpModel(useParams.getState().params), [key]);
  const lqr = useMemo(
    () => designLqr(useParams.getState().params, model.dt),
    // eslint-disable-next-line react-hooks/exhaustive-deps -- the key stands for the parameters
    [key],
  );
  // Redraw a few times a second so the dot follows the drone.
  const [tick, setTick] = useState(0);
  useEffect(() => {
    const id = setInterval(() => setTick((t) => t + 1), 250);
    return () => clearInterval(id);
  }, []);

  const { canvas, readout } = useChartCanvas(
    ({ ctx, w, h, hover }) => {
      ctx.font = FONT;
      const g = model.grid;
      const x0 = LEFT;
      const x1 = w - RIGHT;
      const y0 = TOP;
      const y1 = h - BOTTOM;
      const X = (e: number) => x0 + ((e - g.x0) / (g.x1 - g.x0)) * (x1 - x0);
      const Y = (v: number) => y1 - ((v - g.y0) / (g.y1 - g.y0)) * (y1 - y0);
      // Log scale of V: the quadratic bowl would otherwise be one colour.
      let vmax = 0;
      for (const v of g.v) vmax = Math.max(vmax, v);
      const lv = (v: number) => Math.log1p(v) / Math.log1p(vmax);
      const cw = (x1 - x0) / (g.nx - 1);
      const ch = (y1 - y0) / (g.ny - 1);
      for (let j = 0; j < g.ny; j++)
        for (let i = 0; i < g.nx; i++) {
          ctx.fillStyle = shade(lv(g.v[j * g.nx + i]!));
          ctx.fillRect(X(gridX(g, i)) - cw / 2, Y(gridY(g, j)) - ch / 2, cw + 0.5, ch + 0.5);
        }
      // Where the LQR's command −k₁e − k₂v reaches the limits: beyond these lines it saturates.
      const [k1, k2] = [lqr.k[0]!, lqr.k[1]!];
      ctx.strokeStyle = INK.text;
      ctx.setLineDash([5, 4]);
      ctx.lineWidth = 1.5;
      for (const u of [model.uMax, model.uMin]) {
        // −k1·e − k2·v = u  →  v = (−u − k1·e)/k2
        ctx.beginPath();
        ctx.moveTo(X(g.x0), Y((-u - k1 * g.x0) / k2));
        ctx.lineTo(X(g.x1), Y((-u - k1 * g.x1) / k2));
        ctx.save();
        ctx.beginPath();
        ctx.rect(x0, y0, x1 - x0, y1 - y0);
        ctx.clip();
        ctx.beginPath();
        ctx.moveTo(X(g.x0), Y((-u - k1 * g.x0) / k2));
        ctx.lineTo(X(g.x1), Y((-u - k1 * g.x1) / k2));
        ctx.stroke();
        ctx.restore();
      }
      ctx.setLineDash([]);
      // The policy: arrows along (v, u/m), scaled to the cell.
      const m = Math.max(sim.params.control.model.mass, 0.05);
      ctx.strokeStyle = 'rgba(255,255,255,0.55)';
      ctx.lineWidth = 1;
      for (let j = 4; j < g.ny; j += 8)
        for (let i = 4; i < g.nx; i += 8) {
          const e = gridX(g, i);
          const v = gridY(g, j);
          const u = bestInput(g, model.problem, e, v);
          const de = (v * (x1 - x0)) / (g.x1 - g.x0);
          const dv = (-(u / m) * (y1 - y0)) / (g.y1 - g.y0);
          const len = Math.hypot(de, dv) || 1;
          const s = 12 / len;
          const px = X(e);
          const py = Y(v);
          ctx.beginPath();
          ctx.moveTo(px, py);
          ctx.lineTo(px + de * s, py + dv * s);
          ctx.stroke();
          ctx.beginPath();
          ctx.arc(px + de * s, py + dv * s, 1.5, 0, 2 * Math.PI);
          ctx.fillStyle = 'rgba(255,255,255,0.7)';
          ctx.fill();
        }
      // Axes.
      ctx.fillStyle = INK.muted;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      for (const e of [-4, -2, 0, 2, 4]) ctx.fillText(`${e}`, X(e), y1 + 4);
      ctx.fillText('e = y − r, m', (x0 + x1) / 2, y1 + 15);
      ctx.textAlign = 'right';
      ctx.textBaseline = 'middle';
      for (const v of [-6, -3, 0, 3, 6]) ctx.fillText(`${v}`, x0 - 5, Y(v));
      ctx.save();
      ctx.translate(10, (y0 + y1) / 2);
      ctx.rotate(-Math.PI / 2);
      ctx.textAlign = 'center';
      ctx.fillText('v, m/s', 0, 0);
      ctx.restore();
      // The drone.
      const tl = sim.telemetry;
      const e = tl.latest('pos.y') - tl.latest('sp.y');
      const v = tl.latest('vel.y');
      if (Number.isFinite(e) && Number.isFinite(v)) dot(ctx, X(e), Y(v), SERIES.measured, 4.5);

      if (!hover || hover.x < x0 || hover.x > x1 || hover.y < y0 || hover.y > y1) return;
      const he = g.x0 + ((hover.x - x0) / (x1 - x0)) * (g.x1 - g.x0);
      const hv = g.y0 + ((y1 - hover.y) / (y1 - y0)) * (g.y1 - g.y0);
      const fi = Math.round(((he - g.x0) / (g.x1 - g.x0)) * (g.nx - 1));
      const fj = Math.round(((hv - g.y0) / (g.y1 - g.y0)) * (g.ny - 1));
      const vv = g.v[fj * g.nx + fi] ?? NaN;
      const pp = lqr.p;
      const quad = pp[0]![0]! * he * he + 2 * pp[0]![1]! * he * hv + pp[1]![1]! * hv * hv;
      const u = bestInput(g, model.problem, he, hv);
      const ul = Math.min(Math.max(-(k1 * he + k2 * hv), model.uMin), model.uMax);
      return `e ${he.toFixed(2)} v ${hv.toFixed(2)}   V ${vv.toFixed(1)}  xᵀPx ${quad.toFixed(1)}   u ${u.toFixed(1)} N (LQR ${ul.toFixed(1)})`;
    },
    [model, lqr, tick],
  );
  return (
    <ChartCard
      title="Value function V(e, v): the cost of the best future"
      readout={readout}
      legend={[
        { label: 'drone', color: SERIES.measured, mark: 'dot' },
        { label: 'LQR at the motor limit', color: INK.text, mark: 'line' },
      ]}
      summary={
        <span>
          dark = cheap, bright = costly (log scale) · arrows: the optimal motion · {model.grid.nx} ×{' '}
          {model.grid.ny} grid, {model.iterations} sweeps
        </span>
      }
    >
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </ChartCard>
  );
}
