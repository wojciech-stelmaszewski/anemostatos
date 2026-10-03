import { useMemo } from 'react';
import {
  aircraftModes,
  ENVELOPE,
  envelopeMargins,
  HQ_LEVELS,
  hqPoint,
  PITCH_SPEC,
  type HqPoint,
} from '@/analysis/aircraft';
import { pioOnset } from '@/analysis/pio';
import type { Complex } from '@/math/complex';
import type { Params } from '@/sim/params';
import { useParams } from '@/store/params';
import { sim } from '@/store/sim';
import { SIGNAL } from '@/ui/colors';
import { ChartCard } from './ChartCard';
import { dot, FONT, INK, niceTicks, useChartCanvas, type Frame } from './canvas';

/** What the aircraft charts depend on: the airframe and the autopilot's gains, not its offsets. */
const airKey = (p: Params): string => {
  const ap = { ...p.autopilot, elevatorOffset: 0, thetaOffset: 0 };
  return JSON.stringify([p.aircraft, ap]);
};
const current = (): Params => useParams.getState().params;

/** Axes with a light grid; returns nothing, just draws. */
function grid(
  f: Frame,
  box: { x0: number; x1: number; y0: number; y1: number },
  xs: { v: number; X: number; label: string }[],
  ys: { v: number; Y: number; label: string }[],
) {
  const { ctx } = f;
  ctx.font = FONT;
  ctx.lineWidth = 1;
  ctx.textBaseline = 'middle';
  for (const t of ys) {
    ctx.strokeStyle = INK.grid;
    ctx.beginPath();
    ctx.moveTo(box.x0, t.Y);
    ctx.lineTo(box.x1, t.Y);
    ctx.stroke();
    ctx.fillStyle = INK.muted;
    ctx.textAlign = 'right';
    ctx.fillText(t.label, box.x0 - 4, t.Y);
  }
  ctx.textBaseline = 'top';
  for (const t of xs) {
    ctx.strokeStyle = INK.grid;
    ctx.beginPath();
    ctx.moveTo(t.X, box.y0);
    ctx.lineTo(t.X, box.y1);
    ctx.stroke();
    ctx.fillStyle = INK.muted;
    ctx.textAlign = 'center';
    ctx.fillText(t.label, t.X, box.y1 + 4);
  }
}

/** One s-plane panel: the poles inside `re`/`im` ranges, crosses in the series colour. */
function polePanel(
  f: Frame,
  box: { x0: number; x1: number; y0: number; y1: number },
  poles: Complex[],
  span: number,
  title: string,
) {
  const { ctx } = f;
  const X = (re: number) => box.x1 - ((-re / span) * (box.x1 - box.x0)) / 1.05;
  const Y = (im: number) => (box.y0 + box.y1) / 2 - (im / span) * ((box.y1 - box.y0) / 2);
  const xt = niceTicks(-span, 0, 3).map((v) => ({ v, X: X(v), label: `${+v.toPrecision(3)}` }));
  const yt = niceTicks(-span, span, 4).map((v) => ({ v, Y: Y(v), label: `${+v.toPrecision(3)}` }));
  grid(f, box, xt, yt);
  ctx.strokeStyle = INK.axis;
  ctx.beginPath();
  ctx.moveTo(X(0), box.y0);
  ctx.lineTo(X(0), box.y1);
  ctx.moveTo(box.x0, Y(0));
  ctx.lineTo(box.x1, Y(0));
  ctx.stroke();
  ctx.fillStyle = INK.text;
  ctx.textAlign = 'left';
  ctx.textBaseline = 'top';
  ctx.fillText(title, box.x0 + 4, box.y0 + 2);
  ctx.strokeStyle = SIGNAL.measurement;
  ctx.lineWidth = 2;
  for (const p of poles) {
    if (-p.re > span * 1.05 || Math.abs(p.im) > span) continue;
    const x = X(p.re);
    const y = Y(p.im);
    ctx.beginPath();
    ctx.moveTo(x - 4, y - 4);
    ctx.lineTo(x + 4, y + 4);
    ctx.moveTo(x - 4, y + 4);
    ctx.lineTo(x + 4, y - 4);
    ctx.stroke();
  }
}

/**
 * Lesson IV.11: the aircraft's poles (with its autopilot), twice: the whole short-period pair,
 * and a view 25 times closer to the origin where the phugoid lives.
 */
export function AircraftPoles() {
  const key = useParams((s) => airKey(s.params));
  // eslint-disable-next-line react-hooks/exhaustive-deps -- the key stands for the parameters
  const modes = useMemo(() => aircraftModes(current()), [key]);
  const sp = modes.shortPeriod;
  const ph = modes.phugoid;
  const spanFar = Math.max(1, Math.ceil((sp ? sp.wn : 4) * 1.3));
  const spanNear = Math.max(0.05, (ph ? ph.wn : 0.15) * 1.6);
  const { canvas, readout } = useChartCanvas(
    (f) => {
      const { w, h, hover } = f;
      // Side by side: the whole short-period pair, and the phugoid near the origin.
      const mid = w / 2;
      polePanel(
        f,
        { x0: 34, x1: mid - 8, y0: 8, y1: h - 18 },
        modes.poles,
        spanFar,
        'short period',
      );
      polePanel(
        f,
        { x0: mid + 30, x1: w - 10, y0: 8, y1: h - 18 },
        modes.poles,
        spanNear,
        'phugoid (zoomed in)',
      );
      if (!hover) return;
      const fmt = (p: Complex) => `${p.re.toFixed(3)} ± ${Math.abs(p.im).toFixed(3)}j rad/s`;
      return hover.x < mid ? (sp ? fmt(sp.pole) : '') : ph ? fmt(ph.pole) : '';
    },
    [modes, spanFar, spanNear],
  );
  return (
    <ChartCard
      title="Poles of the aircraft"
      readout={readout}
      legend={[{ label: 'pole', color: SIGNAL.measurement, mark: 'cross' }]}
      summary={
        <span>
          {sp ? `short period ζ ${sp.zeta.toFixed(2)} · ` : ''}
          {ph ? `phugoid ζ ${ph.zeta.toFixed(3)}` : ''} · hover a panel for its pole
        </span>
      }
    >
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </ChartCard>
  );
}

const HQ_COLOR = { bare: SIGNAL.setpoint, locus: SIGNAL.d, now: SIGNAL.measurement } as const;

/**
 * Lesson IV.12: the handling-qualities chart of the short period [MIL-F-8785C, Category A],
 * damping ratio against the control anticipation parameter, with the level 1 and level 2 boxes.
 * The line is the path of the point as the damper gain grows from zero to twice its value.
 */
export function HqChart() {
  const key = useParams((s) => airKey(s.params));
  const data = useMemo(() => {
    const p = current();
    const bare = hqPoint(p, false);
    const now = hqPoint(p, true);
    const g = Math.max(p.autopilot.gain * 2, 0.3);
    const locus: HqPoint[] = [];
    for (let i = 0; i <= 30; i++) {
      const pt = hqPoint({ ...p, autopilot: { ...p.autopilot, gain: (g * i) / 30 } }, true);
      if (pt) locus.push(pt);
    }
    return { bare, now, locus };
    // eslint-disable-next-line react-hooks/exhaustive-deps -- the key stands for the parameters
  }, [key]);
  const { canvas, readout } = useChartCanvas(
    (f) => {
      const { ctx, w, h } = f;
      const box = { x0: 38, x1: w - 10, y0: 8, y1: h - 18 };
      const zMax = 2.2;
      const cLo = 0.1;
      const cHi = 20;
      const X = (z: number) => box.x0 + (Math.min(z, zMax) / zMax) * (box.x1 - box.x0);
      const Y = (c: number) =>
        box.y1 -
        (Math.log(Math.min(Math.max(c, cLo), cHi) / cLo) / Math.log(cHi / cLo)) * (box.y1 - box.y0);
      grid(
        f,
        box,
        [0, 0.5, 1, 1.5, 2].map((v) => ({ v, X: X(v), label: `${v}` })),
        [0.1, 0.2, 0.5, 1, 2, 5, 10, 20].map((v) => ({ v, Y: Y(v), label: `${v}` })),
      );
      const rect = (l: 1 | 2) => {
        const b = HQ_LEVELS[l];
        ctx.strokeStyle = INK.text;
        ctx.lineWidth = l === 1 ? 1.5 : 1;
        ctx.setLineDash(l === 1 ? [] : [4, 3]);
        ctx.strokeRect(
          X(b.zeta[0]),
          Y(b.cap[1]),
          X(b.zeta[1]) - X(b.zeta[0]),
          Y(b.cap[0]) - Y(b.cap[1]),
        );
        ctx.setLineDash([]);
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'left';
        ctx.textBaseline = 'top';
        ctx.fillText(`level ${l}`, X(b.zeta[0]) + 4, Y(b.cap[1]) + 3);
      };
      rect(2);
      rect(1);
      ctx.fillStyle = INK.muted;
      ctx.textAlign = 'right';
      ctx.textBaseline = 'bottom';
      ctx.fillText('damping ratio ζ', box.x1, box.y1 - 2);
      ctx.textAlign = 'left';
      ctx.fillText('CAP, 1/(g·s²)', box.x0 + 4, box.y1 - 2);
      ctx.strokeStyle = HQ_COLOR.locus;
      ctx.lineWidth = 2;
      ctx.beginPath();
      data.locus.forEach((pt, i) =>
        i ? ctx.lineTo(X(pt.zeta), Y(pt.cap)) : ctx.moveTo(X(pt.zeta), Y(pt.cap)),
      );
      ctx.stroke();
      if (data.bare) dot(ctx, X(data.bare.zeta), Y(data.bare.cap), HQ_COLOR.bare);
      if (data.now) dot(ctx, X(data.now.zeta), Y(data.now.cap), HQ_COLOR.now, 5);
      const n = data.now;
      return n
        ? `ζ ${n.zeta.toFixed(3)}   ω ${n.wn.toFixed(2)} rad/s   CAP ${n.cap.toFixed(2)}   level ${n.level}`
        : '';
    },
    [data],
  );
  return (
    <ChartCard
      title="Handling qualities of the short period"
      readout={readout}
      legend={[
        { label: 'bare airframe', color: HQ_COLOR.bare, mark: 'dot' },
        { label: 'with the damper', color: HQ_COLOR.now, mark: 'dot' },
        { label: 'as the gain grows', color: HQ_COLOR.locus, mark: 'line' },
      ]}
      summary={data.now ? <span>now: level {data.now.level}</span> : null}
    >
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </ChartCard>
  );
}

const ENV_COLOR = { now: SIGNAL.measurement, other: SIGNAL.setpoint } as const;

/**
 * Lesson IV.13: phase margin and crossover of the pitch loop across the speed envelope, for the
 * gain as set and for the other choice of schedule (dashed), against the requirement.
 */
export function EnvelopeChart() {
  const key = useParams((s) => airKey(s.params));
  const data = useMemo(() => {
    const p = current();
    const ndi = p.autopilot.law === 'ndi';
    // Against NDI: the q̄ schedule, with the same error in the elevator's strength (a gain error).
    const other = {
      ...p,
      autopilot: ndi
        ? {
            ...p.autopilot,
            law: 'classic' as const,
            schedule: 'qbar' as const,
            gain: p.autopilot.gain / (1 + p.autopilot.modelError),
          }
        : {
            ...p.autopilot,
            schedule: p.autopilot.schedule === 'qbar' ? ('none' as const) : ('qbar' as const),
          },
    };
    const label = (q: Params): string =>
      q.autopilot.law === 'ndi'
        ? 'dynamic inversion'
        : q.autopilot.schedule === 'qbar'
          ? ndi
            ? 'scheduled on q̄ (same error)'
            : 'scheduled on q̄'
          : 'fixed gain';
    // Lesson IV.14: on α, the other line is the same design as the table's point designs saw it.
    if (!ndi && p.autopilot.schedule === 'alpha')
      return {
        now: envelopeMargins(p, 17),
        other: envelopeMargins(p, 17, true),
        labels: ['scheduled on α, as flown', 'the same, as the table saw it'] as const,
      };
    return {
      now: envelopeMargins(p, 17),
      other: envelopeMargins(other, 17),
      labels: [label(p), label(other)] as const,
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps -- the key stands for the parameters
  }, [key]);
  const { canvas, readout } = useChartCanvas(
    (f) => {
      const { ctx, w, h, hover } = f;
      const half = h / 2;
      const X0 = 38;
      const X1 = w - 10;
      const X = (v: number) =>
        X0 + ((v - ENVELOPE.vMin) / (ENVELOPE.vMax - ENVELOPE.vMin)) * (X1 - X0);
      const xt = niceTicks(ENVELOPE.vMin, ENVELOPE.vMax, 4).map((v) => ({
        v,
        X: X(v),
        label: `${v}`,
      }));
      const panel = (
        y0: number,
        y1: number,
        hi: number,
        pick: (r: { pmDeg: number; wc: number }) => number,
        spec: number,
        title: string,
      ) => {
        const Y = (v: number) => y1 - (Math.min(Math.max(v, 0), hi) / hi) * (y1 - y0);
        grid(
          f,
          { x0: X0, x1: X1, y0, y1 },
          xt,
          niceTicks(0, hi, 4).map((v) => ({ v, Y: Y(v), label: `${v}` })),
        );
        ctx.strokeStyle = INK.text;
        ctx.setLineDash([5, 4]);
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(X0, Y(spec));
        ctx.lineTo(X1, Y(spec));
        ctx.stroke();
        ctx.setLineDash([]);
        const line = (rows: typeof data.now, color: string, dash: number[]) => {
          ctx.strokeStyle = color;
          ctx.lineWidth = 2;
          ctx.setLineDash(dash);
          ctx.beginPath();
          rows.forEach((r, i) => {
            const v = pick(r);
            if (i) ctx.lineTo(X(r.V), Y(Number.isFinite(v) ? v : hi));
            else ctx.moveTo(X(r.V), Y(Number.isFinite(v) ? v : hi));
          });
          ctx.stroke();
          ctx.setLineDash([]);
        };
        line(data.other, ENV_COLOR.other, [4, 3]);
        line(data.now, ENV_COLOR.now, []);
        ctx.fillStyle = INK.text;
        ctx.textAlign = 'left';
        ctx.textBaseline = 'top';
        ctx.fillText(title, X0 + 4, y0 + 2);
      };
      panel(8, half - 16, 90, (r) => r.pmDeg, PITCH_SPEC.pmDeg, 'phase margin, °');
      panel(half + 6, h - 18, 14, (r) => r.wc, PITCH_SPEC.wc, 'crossover, rad/s');
      ctx.fillStyle = INK.muted;
      ctx.textAlign = 'right';
      ctx.textBaseline = 'bottom';
      ctx.fillText('airspeed, m/s', X1, h - 20);
      // Where the aircraft is now.
      const V = sim.aircraft?.V;
      if (V != null && V >= ENVELOPE.vMin && V <= ENVELOPE.vMax) {
        ctx.strokeStyle = INK.axis;
        ctx.beginPath();
        ctx.moveTo(X(V), 8);
        ctx.lineTo(X(V), h - 18);
        ctx.stroke();
      }
      if (!hover || hover.x < X0 || hover.x > X1) return;
      const v = ENVELOPE.vMin + ((hover.x - X0) / (X1 - X0)) * (ENVELOPE.vMax - ENVELOPE.vMin);
      const r = data.now.reduce((a, b) => (Math.abs(b.V - v) < Math.abs(a.V - v) ? b : a));
      return `${r.V.toFixed(0)} m/s   q̄ ${(r.qbar / 1000).toFixed(1)} kPa   PM ${r.pmDeg.toFixed(0)}°   crossover ${r.wc.toFixed(2)} rad/s`;
    },
    [data],
  );
  const worst = Math.min(...data.now.map((r) => r.pmDeg));
  const slowest = Math.min(...data.now.map((r) => (Number.isFinite(r.wc) ? r.wc : 0)));
  return (
    <ChartCard
      title="The pitch loop across the envelope"
      readout={readout}
      legend={[
        {
          label: data.labels[0],
          color: ENV_COLOR.now,
          mark: 'line',
        },
        {
          label: data.labels[1],
          color: ENV_COLOR.other,
          mark: 'line',
        },
      ]}
      summary={
        <span>
          worst PM {worst.toFixed(0)}° · slowest crossover {slowest.toFixed(2)} rad/s (dashed: the
          requirement)
        </span>
      }
    >
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </ChartCard>
  );
}

/**
 * Lesson IV.16: harmonic balance of the pilot–aircraft loop through the rate-limited elevator. For
 * each amplitude of the elevator command, the loop gain where its phase is −180°: below 1 an
 * oscillation of that size dies out, above 1 it grows. The first crossing is the onset of the PIO.
 */
export function PioChart() {
  const key = useParams((s) => airKey(s.params));
  const data = useMemo(
    () => pioOnset(current()),
    // eslint-disable-next-line react-hooks/exhaustive-deps -- the key stands for the parameters
    [key],
  );
  const travel = current().aircraft.elevatorMax;
  const { canvas, readout } = useChartCanvas(
    (f) => {
      const { ctx, w, h, hover } = f;
      const X0 = 38;
      const X1 = w - 10;
      const Y0 = 8;
      const Y1 = h - 18;
      const hi = Math.max(1.5, ...data.curve.map((c) => c.gain)) * 1.1;
      const X = (a: number) => X0 + (a / travel) * (X1 - X0);
      const Y = (g: number) => Y1 - (Math.min(g, hi) / hi) * (Y1 - Y0);
      grid(
        f,
        { x0: X0, x1: X1, y0: Y0, y1: Y1 },
        niceTicks(0, travel, 5).map((v) => ({ v, X: X(v), label: `${v}°` })),
        niceTicks(0, hi, 4).map((v) => ({ v, Y: Y(v), label: v.toFixed(1) })),
      );
      // Gain 1: the boundary between dying out and growing.
      ctx.strokeStyle = INK.text;
      ctx.setLineDash([5, 4]);
      ctx.beginPath();
      ctx.moveTo(X0, Y(1));
      ctx.lineTo(X1, Y(1));
      ctx.stroke();
      // Where the elevator starts to rate-limit at the loop's own frequency.
      ctx.strokeStyle = INK.muted;
      ctx.beginPath();
      ctx.moveTo(X(data.rateLimitDeg), Y0);
      ctx.lineTo(X(data.rateLimitDeg), Y1);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.strokeStyle = SIGNAL.measurement;
      ctx.lineWidth = 2;
      ctx.beginPath();
      data.curve.forEach((c, i) => {
        if (i) ctx.lineTo(X(c.ampDeg), Y(c.gain));
        else ctx.moveTo(X(c.ampDeg), Y(c.gain));
      });
      ctx.stroke();
      if (Number.isFinite(data.onsetDeg)) dot(ctx, X(data.onsetDeg), Y(1), SIGNAL.error, 4);
      ctx.fillStyle = INK.muted;
      ctx.textAlign = 'right';
      ctx.textBaseline = 'bottom';
      ctx.fillText('elevator command amplitude', X1, Y1 - 2);
      if (!hover || hover.x < X0 || hover.x > X1) return;
      const a = ((hover.x - X0) / (X1 - X0)) * travel;
      const c = data.curve.reduce((p, q) =>
        Math.abs(q.ampDeg - a) < Math.abs(p.ampDeg - a) ? q : p,
      );
      return `${c.ampDeg.toFixed(1)}°   loop gain ${c.gain.toFixed(2)} at ${c.w.toFixed(2)} rad/s`;
    },
    [data],
  );
  return (
    <ChartCard
      title="Pilot and elevator: loop gain at −180° against amplitude"
      readout={readout}
      legend={[
        {
          label: 'describing function × airframe × pilot',
          color: SIGNAL.measurement,
          mark: 'line',
        },
      ]}
      summary={
        <span>
          {Number.isFinite(data.onsetDeg)
            ? `PIO above ${data.onsetDeg.toFixed(1)}° of command, at ${data.onsetW.toFixed(2)} rad/s`
            : 'no PIO within the elevator’s travel'}{' '}
          · rate-limits above {data.rateLimitDeg.toFixed(1)}° (grey) · small-signal gain margin{' '}
          {(1 / data.linearGain).toFixed(2)}
        </span>
      }
    >
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </ChartCard>
  );
}
