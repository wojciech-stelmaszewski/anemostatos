import { useMemo } from 'react';
import { sensitivities, type LoopModel } from '@/analysis/loop';
import type { SweepPoint } from '@/analysis/sweep';
import type { RobustTest } from '@/analysis/uncertainty';
import { cabs, type Complex } from '@/math/complex';
import { logspace, unwrapPhase } from '@/math/margins';
import { useParams } from '@/store/params';
import { useUi, type BodeTarget, type BodeView } from '@/store/sim';
import { Button } from '@/ui/components/button';
import { Select } from '@/ui/components/select';
import { ChartCard } from './ChartCard';
import { dot, FONT, INK, niceTicks, SERIES, useChartCanvas, type Frame } from './canvas';
import {
  F_MIN,
  familyKey,
  useCampaign,
  useLoopData,
  useMeasure,
  useMeasured,
  useRobustTest,
  type LoopData,
} from './store';

const DEG = 180 / Math.PI;
const dB = (x: number) => 20 * Math.log10(x);
const LEFT = 40;
const RIGHT = 10;
const TOP = 6;
const BOTTOM = 18;
const GAP = 12;

const VIEWS: { value: BodeView; label: string }[] = [
  { value: 'loop', label: 'open loop L' },
  { value: 'ref', label: 'following the setpoint Y/R' },
  { value: 'sens', label: 'sensitivity S' },
  { value: 'waterbed', label: 'waterbed: |S|, linear axis' },
  { value: 'robust', label: 'robust stability' },
];
const TITLE: Record<BodeView, string> = {
  loop: 'Bode plot of the open loop L',
  ref: 'Bode plot of the closed loop Y/R',
  sens: 'Bode plot of the sensitivity S',
  waterbed: 'The waterbed: |S| on a linear frequency axis',
  robust: 'Robust stability: |T| below 1/|W|',
};
const SYMBOL: Record<BodeView, string> = {
  loop: 'L',
  ref: 'Y/R',
  sens: 'S',
  waterbed: 'S',
  robust: 'T',
};

/** Phases in degrees, continuous, shifted by whole turns so that they start in (−360°, 0°]. */
function phasesDeg(l: Complex[]): number[] {
  const ph = unwrapPhase(l).map((v) => v * DEG);
  if (!ph.length) return ph;
  const shift = 360 * Math.ceil(ph[0]! / 360 - 1e-9);
  return ph.map((v) => v - shift);
}

/** Index of the value in `xs` closest to `x` on a logarithmic scale. */
function nearest(xs: number[], x: number): number {
  let best = 0;
  let d = Infinity;
  for (let i = 0; i < xs.length; i++) {
    const e = Math.abs(Math.log(xs[i]! / x));
    if (e < d) {
      d = e;
      best = i;
    }
  }
  return best;
}

/** Labels drawn last, on a backing of the surface colour, so that marks do not run through them. */
function drawNotes(ctx: CanvasRenderingContext2D, notes: [string, number, number][], xMax: number) {
  for (const [text, x, y] of notes) {
    const tw = ctx.measureText(text).width;
    const tx = Math.min(x, xMax - tw - 3);
    ctx.fillStyle = 'rgba(11,14,19,0.85)';
    ctx.fillRect(tx - 2, y - 7, tw + 4, 14);
    ctx.fillStyle = INK.text;
    ctx.textAlign = 'left';
    ctx.fillText(text, tx, y);
  }
}

/** |S| in dB on a linear frequency grid, and the areas of ln|S| above and below zero. */
function waterbed(model: LoopModel) {
  const fN = 0.5 / model.T;
  const grid = [
    ...logspace(1e-5, 1, 600),
    ...Array.from({ length: 1500 }, (_, i) => 1 + ((fN - 1) * (i + 1)) / 1500),
  ];
  const ln = grid.map((f) => Math.log(cabs(sensitivities(model, 2 * Math.PI * f).s)));
  let above = 0;
  let below = 0;
  for (let i = 1; i < grid.length; i++) {
    const a = 0.5 * (ln[i - 1]! + ln[i]!) * (grid[i]! - grid[i - 1]!);
    if (a > 0) above += a;
    else below -= a;
  }
  return { f: grid, db: ln.map((v) => (20 / Math.LN10) * v), above, below, fN };
}

function drawWaterbed({ ctx, w, h, hover }: Frame, wb: ReturnType<typeof waterbed>) {
  const x0 = LEFT;
  const x1 = w - RIGHT;
  const y0 = TOP;
  const y1 = h - BOTTOM;
  // The areas run to the Nyquist frequency; the interesting part is the first dozen hertz.
  const fMax = Math.min(wb.fN, 12);
  const lo = -30;
  const hi = Math.max(6, Math.ceil(Math.max(...wb.db) / 3) * 3 + 3);
  const X = (f: number) => x0 + (f / fMax) * (x1 - x0);
  const Y = (v: number) => y0 + ((hi - Math.max(v, lo)) / (hi - lo)) * (y1 - y0);
  ctx.font = FONT;
  ctx.lineWidth = 1;
  ctx.textBaseline = 'middle';
  for (const v of niceTicks(lo, hi, Math.max(3, Math.floor((y1 - y0) / 28)))) {
    ctx.strokeStyle = v === 0 ? INK.axis : INK.grid;
    ctx.setLineDash(v === 0 ? [4, 3] : []);
    ctx.beginPath();
    ctx.moveTo(x0, Y(v));
    ctx.lineTo(x1, Y(v));
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = INK.muted;
    ctx.textAlign = 'right';
    ctx.fillText(`${v}`, x0 - 5, Y(v));
  }
  for (const v of niceTicks(0, fMax, 6)) {
    ctx.strokeStyle = INK.grid;
    ctx.beginPath();
    ctx.moveTo(X(v), y0);
    ctx.lineTo(X(v), y1);
    ctx.stroke();
    ctx.fillStyle = INK.muted;
    ctx.textAlign = 'center';
    ctx.fillText(`${v}`, X(v), y1 + 9);
  }
  ctx.textAlign = 'right';
  ctx.fillText('Hz', x1, h - 3);
  ctx.fillText('|S|  dB', x1 - 3, y0 + 7);

  // The two areas: what the loop rejects (below zero) and what it amplifies in return (above).
  const pts = wb.f.map((f, i) => [f, wb.db[i]!] as const).filter(([f]) => f <= fMax);
  const fill = (positive: boolean, color: string) => {
    ctx.beginPath();
    ctx.moveTo(X(pts[0]![0]), Y(0));
    for (const [f, v] of pts) ctx.lineTo(X(f), Y(positive ? Math.max(v, 0) : Math.min(v, 0)));
    ctx.lineTo(X(pts[pts.length - 1]![0]), Y(0));
    ctx.closePath();
    ctx.fillStyle = color;
    ctx.fill();
  };
  fill(false, 'rgba(57,135,229,0.22)');
  fill(true, 'rgba(217,89,38,0.30)');
  ctx.strokeStyle = SERIES.model;
  ctx.lineWidth = 2;
  ctx.lineJoin = 'round';
  ctx.beginPath();
  pts.forEach(([f, v], i) => (i ? ctx.lineTo(X(f), Y(v)) : ctx.moveTo(X(f), Y(v))));
  ctx.stroke();
  const peak = wb.db.reduce((b, v, i) => (v > wb.db[b]! ? i : b), 0);
  drawNotes(
    ctx,
    [
      [`amplified: area ${wb.above.toFixed(2)}`, X(wb.f[peak]!) + 8, Y(wb.db[peak]!) - 9],
      [`rejected: area ${wb.below.toFixed(2)}`, X(0) + 14, Y(-18)],
    ],
    x1,
  );
  if (!hover || hover.x < x0 || hover.x > x1) return;
  const f = ((hover.x - x0) / (x1 - x0)) * fMax;
  let i = 0;
  for (let k = 0; k < wb.f.length; k++) if (Math.abs(wb.f[k]! - f) < Math.abs(wb.f[i]! - f)) i = k;
  ctx.strokeStyle = INK.axis;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(X(wb.f[i]!), y0);
  ctx.lineTo(X(wb.f[i]!), y1);
  ctx.stroke();
  dot(ctx, X(wb.f[i]!), Y(wb.db[i]!), SERIES.model, 3);
  return `${wb.f[i]!.toPrecision(3)} Hz   |S| ${wb.db[i]!.toFixed(1)} dB   a disturbance here is ${wb.db[i]! > 0 ? 'amplified' : 'reduced'} ×${(10 ** (wb.db[i]! / 20)).toFixed(2)}`;
}

/** The small-gain test as a picture: the nominal |T| and the ceiling 1/|W| the family sets. */
function drawRobust({ ctx, w, h, hover }: Frame, rt: RobustTest) {
  const x0 = LEFT;
  const x1 = w - RIGHT;
  const y0 = TOP;
  const y1 = h - BOTTOM;
  const f = rt.fHz;
  const fMax = f[f.length - 1]!;
  const X = (hz: number) =>
    x0 +
    ((Math.log10(hz) - Math.log10(f[0]!)) / (Math.log10(fMax) - Math.log10(f[0]!))) * (x1 - x0);
  const tDb = rt.t.map(dB);
  const limDb = rt.w.map((v) => -dB(Math.max(v, 1e-6)));
  const lo = -40;
  const hi = 30;
  const Y = (v: number) => y0 + ((hi - Math.min(Math.max(v, lo), hi)) / (hi - lo)) * (y1 - y0);
  ctx.font = FONT;
  ctx.lineWidth = 1;
  ctx.textBaseline = 'middle';
  for (const v of niceTicks(lo, hi, Math.max(3, Math.floor((y1 - y0) / 28)))) {
    ctx.strokeStyle = v === 0 ? INK.axis : INK.grid;
    ctx.beginPath();
    ctx.moveTo(x0, Y(v));
    ctx.lineTo(x1, Y(v));
    ctx.stroke();
    ctx.fillStyle = INK.muted;
    ctx.textAlign = 'right';
    ctx.fillText(`${v}`, x0 - 5, Y(v));
  }
  for (let d = -2; d <= 2; d++)
    for (const k of [1, 2, 5]) {
      const hz = k * 10 ** d;
      if (hz < f[0]! || hz > fMax * 1.0001) continue;
      ctx.strokeStyle = INK.grid;
      ctx.beginPath();
      ctx.moveTo(X(hz), y0);
      ctx.lineTo(X(hz), y1);
      ctx.stroke();
      ctx.fillStyle = INK.muted;
      ctx.textAlign = 'center';
      ctx.fillText(String(+hz.toPrecision(1)), X(hz), y1 + 9);
    }
  ctx.textAlign = 'right';
  ctx.fillText('Hz', x1, h - 3);
  ctx.fillText('dB', x1 - 3, y0 + 7);

  // Where the nominal |T| pokes through the ceiling, some member of the family may be unstable.
  ctx.fillStyle = 'rgba(230,103,103,0.30)';
  for (let i = 1; i < f.length; i++) {
    if (tDb[i]! <= limDb[i]! && tDb[i - 1]! <= limDb[i - 1]!) continue;
    ctx.beginPath();
    ctx.moveTo(X(f[i - 1]!), Y(Math.max(tDb[i - 1]!, limDb[i - 1]!)));
    ctx.lineTo(X(f[i]!), Y(Math.max(tDb[i]!, limDb[i]!)));
    ctx.lineTo(X(f[i]!), Y(limDb[i]!));
    ctx.lineTo(X(f[i - 1]!), Y(limDb[i - 1]!));
    ctx.closePath();
    ctx.fill();
  }
  const line = (ys: number[], color: string, dash: number[]) => {
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.lineJoin = 'round';
    ctx.setLineDash(dash);
    ctx.beginPath();
    ys.forEach((v, i) => (i ? ctx.lineTo(X(f[i]!), Y(v)) : ctx.moveTo(X(f[i]!), Y(v))));
    ctx.stroke();
    ctx.setLineDash([]);
  };
  line(limDb, SERIES.measured, [6, 4]);
  line(tDb, SERIES.model, []);
  const iw = f.findIndex((v) => v >= rt.fWorst);
  if (iw >= 0) {
    dot(ctx, X(f[iw]!), Y(tDb[iw]!), INK.text, 3.5);
    drawNotes(ctx, [[`|W·T| = ${rt.worst.toFixed(2)}`, X(f[iw]!) + 8, Y(tDb[iw]!) - 10]], x1);
  }
  if (!hover || hover.x < x0 || hover.x > x1) return;
  const hz = 10 ** (Math.log10(f[0]!) + ((hover.x - x0) / (x1 - x0)) * Math.log10(fMax / f[0]!));
  const i = nearest(f, hz);
  ctx.strokeStyle = INK.axis;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(X(f[i]!), y0);
  ctx.lineTo(X(f[i]!), y1);
  ctx.stroke();
  dot(ctx, X(f[i]!), Y(tDb[i]!), SERIES.model, 3);
  dot(ctx, X(f[i]!), Y(limDb[i]!), SERIES.measured, 3);
  return `${f[i]!.toPrecision(3)} Hz   |T| ${tDb[i]!.toFixed(1)} dB   ceiling ${limDb[i]!.toFixed(1)} dB   |W·T| ${(rt.w[i]! * rt.t[i]!).toFixed(2)}`;
}

const pick = (p: SweepPoint, view: BodeView) =>
  view === 'loop' ? p.l : view === 'ref' ? p.yr : p.s;

function drawBode(
  { ctx, w, h, hover }: Frame,
  data: LoopData,
  measuredAll: SweepPoint[],
  view: BodeView,
  level: number,
  probeHz: number | null,
  target: BodeTarget | null = null,
) {
  const f = data.fHz;
  const curve = view === 'loop' ? data.l : view === 'ref' ? data.yr : data.s;
  const measured = measuredAll.filter((p) => pick(p, view));
  const m = data.margins;
  const fMax = f[f.length - 1]!;
  const x0 = LEFT;
  const x1 = w - RIGHT;
  const paneH = (h - TOP - BOTTOM - GAP) / 2;
  const X = (hz: number) =>
    x0 +
    ((Math.log10(hz) - Math.log10(F_MIN)) / (Math.log10(fMax) - Math.log10(F_MIN))) * (x1 - x0);

  const mag = curve.map((v) => dB(cabs(v)));
  const ph = phasesDeg(curve);
  const mMag = measured.map((p) => dB(cabs(pick(p, view)!)));
  // The measured phases take the turn of the model's phase at the same frequency.
  const mPh = measured.map((p) => {
    const c = pick(p, view)!;
    let v = Math.atan2(c.im, c.re) * DEG;
    const near = ph.length ? ph[nearest(f, p.fHz)]! : -180;
    while (v - near > 180) v -= 360;
    while (v - near < -180) v += 360;
    return v;
  });

  const all = [...mag, ...mMag];
  const magLo = Math.max(-80, all.length ? Math.floor((Math.min(...all) - 3) / 20) * 20 : -60);
  const magHi = all.length ? Math.ceil((Math.max(...all) + 3) / 20) * 20 : 40;
  // Delay drives the phase down without end; the story is told between −360° and 0°.
  const allPh = [...ph, ...mPh, view === 'loop' ? -180 : 0];
  const phLo = Math.max(-450, Math.floor((Math.min(...allPh) - 10) / 90) * 90);
  const phHi = Math.min(180, Math.ceil((Math.max(...allPh) + 10) / 90) * 90);
  const phStep = paneH / ((phHi - phLo) / 90) >= 16 ? 90 : 180;
  const yMag = (v: number) => TOP + ((magHi - Math.max(v, magLo)) / (magHi - magLo)) * paneH;
  const yPh = (v: number) =>
    TOP + paneH + GAP + ((phHi - Math.max(v, phLo)) / (phHi - phLo)) * paneH;

  ctx.font = FONT;
  ctx.lineWidth = 1;
  ctx.textBaseline = 'middle';
  for (let d = Math.ceil(Math.log10(F_MIN)); d <= Math.log10(fMax) + 1; d++)
    for (const k of [1, 2, 5]) {
      const hz = k * 10 ** d;
      if (hz < F_MIN || hz > fMax * 1.0001) continue;
      ctx.strokeStyle = INK.grid;
      ctx.beginPath();
      ctx.moveTo(X(hz), TOP);
      ctx.lineTo(X(hz), TOP + paneH);
      ctx.moveTo(X(hz), TOP + paneH + GAP);
      ctx.lineTo(X(hz), h - BOTTOM);
      ctx.stroke();
      if (k === 1 || k === 5 || fMax / F_MIN < 300) {
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText(String(+hz.toPrecision(1)), X(hz), h - BOTTOM + 9);
      }
    }
  ctx.textAlign = 'right';
  ctx.fillStyle = INK.muted;
  ctx.fillText('Hz', x1, h - 3);
  const hLine = (y: number, strong: boolean) => {
    ctx.strokeStyle = strong ? INK.axis : INK.grid;
    ctx.setLineDash(strong ? [4, 3] : []);
    ctx.beginPath();
    ctx.moveTo(x0, y);
    ctx.lineTo(x1, y);
    ctx.stroke();
    ctx.setLineDash([]);
  };
  for (const v of niceTicks(magLo, magHi, Math.max(2, Math.floor(paneH / 26)))) {
    hLine(yMag(v), v === 0);
    ctx.fillStyle = INK.muted;
    ctx.fillText(`${v}`, x0 - 5, yMag(v));
  }
  if (view === 'ref') hLine(yMag(-3), true);
  for (let v = Math.ceil(phLo / phStep) * phStep; v <= phHi; v += phStep) {
    hLine(yPh(v), view === 'loop' && v === -180);
    ctx.fillStyle = INK.muted;
    ctx.fillText(`${v}°`, x0 - 5, yPh(v));
  }
  // The curves fall from the upper left, so the upper right of each pane is free.
  const sym = SYMBOL[view];
  ctx.fillText(`|${sym}|  dB`, x1 - 3, TOP + 7);
  ctx.fillText(`∠${sym}`, x1 - 3, TOP + paneH + GAP + 7);
  ctx.textAlign = 'left';

  if (!curve.length && !measured.length) {
    ctx.fillStyle = INK.muted;
    ctx.textAlign = 'center';
    ctx.fillText(
      level === 1
        ? 'This controller has no linear model. Press Measure to fly the response.'
        : 'The frequency-response tools work on Level 1 (altitude) for now.',
      (x0 + x1) / 2,
      TOP + paneH / 2,
    );
    return;
  }

  // The numbers of each view, drawn as the distances or positions they are.
  const notes: [string, number, number][] = [];
  ctx.lineWidth = 2;
  ctx.strokeStyle = INK.text;
  const vMark = (hz: number, ya: number, yb: number) => {
    ctx.beginPath();
    ctx.moveTo(X(hz), ya);
    ctx.lineTo(X(hz), yb);
    ctx.stroke();
  };
  if (view === 'loop' && m && ph.length) {
    if (Number.isFinite(m.pmDeg)) {
      const fc = m.wc / (2 * Math.PI);
      const at = ph[nearest(f, fc)]!;
      const base = at - m.pmDeg; // the odd multiple of −180° that the margin is measured from
      vMark(fc, yPh(base), yPh(at));
      notes.push([`PM ${m.pmDeg.toFixed(0)}°`, X(fc) + 5, (yPh(base) + yPh(at)) / 2]);
    }
    if (Number.isFinite(m.gm)) {
      const f180 = m.w180 / (2 * Math.PI);
      vMark(f180, yMag(0), yMag(-dB(m.gm)));
      notes.push([`GM ${dB(m.gm).toFixed(0)} dB`, X(f180) + 5, (yMag(0) + yMag(-dB(m.gm))) / 2]);
    }
  }
  if (view === 'ref' && Number.isFinite(data.bandwidthHz)) {
    vMark(data.bandwidthHz, yMag(-3) - 6, yMag(-3) + 6);
    notes.push([
      `bandwidth ${data.bandwidthHz.toFixed(2)} Hz`,
      X(data.bandwidthHz) + 6,
      yMag(-3) - 9,
    ]);
  }
  if (view === 'sens' && m && Number.isFinite(m.wMs)) {
    const fp = m.wMs / (2 * Math.PI);
    vMark(fp, yMag(0), yMag(dB(m.ms)));
    notes.push([`peak ${dB(m.ms).toFixed(1)} dB`, X(fp) + 6, yMag(dB(m.ms)) - 8]);
  }
  if (probeHz && probeHz >= F_MIN && probeHz <= fMax) {
    ctx.lineWidth = 1;
    ctx.setLineDash([2, 3]);
    ctx.strokeStyle = INK.text;
    vMark(probeHz, TOP, h - BOTTOM);
    ctx.setLineDash([]);
    notes.push(['probe', X(probeHz) + 4, h - BOTTOM - 8]);
  }

  // The target shape (lesson III.6): a floor at low frequency, a crossover window, a phase line.
  if (view === 'loop' && target) {
    ctx.save();
    ctx.fillStyle = 'rgba(230,103,103,0.12)';
    const xl = Math.max(x0, X(F_MIN));
    const xr = Math.min(x1, X(target.lowHz));
    // Forbidden: below the floor at low frequency.
    ctx.fillRect(xl, yMag(target.lowDb), xr - xl, TOP + paneH - yMag(target.lowDb));
    // Forbidden: crossing outside the window, and too little phase inside it.
    ctx.fillStyle = 'rgba(57,135,229,0.14)';
    ctx.fillRect(X(target.fcLoHz), TOP, X(target.fcHiHz) - X(target.fcLoHz), paneH);
    ctx.fillStyle = 'rgba(230,103,103,0.12)';
    const yLim = yPh(-180 + target.pmDeg);
    ctx.fillRect(
      X(target.fcLoHz),
      yLim,
      X(target.fcHiHz) - X(target.fcLoHz),
      TOP + 2 * paneH + GAP - yLim,
    );
    ctx.restore();
    notes.push([`|L| ≥ ${target.lowDb} dB`, xl + 4, yMag(target.lowDb) + 9]);
    notes.push([`cross here`, X(target.fcLoHz) + 3, TOP + 9]);
    notes.push([`PM ≥ ${target.pmDeg}°`, X(target.fcLoHz) + 3, yLim + 9]);
  }

  // The model.
  const line = (ys: number[], y: (v: number) => number, top: number) => {
    ctx.save();
    ctx.beginPath();
    ctx.rect(x0, top, x1 - x0, paneH);
    ctx.clip();
    ctx.strokeStyle = SERIES.model;
    ctx.lineWidth = 2;
    ctx.lineJoin = 'round';
    ctx.beginPath();
    ys.forEach((v, i) => (i ? ctx.lineTo(X(f[i]!), y(v)) : ctx.moveTo(X(f[i]!), y(v))));
    ctx.stroke();
    ctx.restore();
  };
  line(mag, yMag, TOP);
  line(ph, yPh, TOP + paneH + GAP);
  // The measurement.
  measured.forEach((p, i) => {
    dot(ctx, X(p.fHz), yMag(mMag[i]!), SERIES.measured);
    dot(ctx, X(p.fHz), yPh(mPh[i]!), SERIES.measured);
  });
  drawNotes(ctx, notes, x1);

  if (!hover || hover.x < x0 || hover.x > x1) return;
  // The crosshair finds the frequency; the readout lists every series there.
  const hz = 10 ** (Math.log10(F_MIN) + ((hover.x - x0) / (x1 - x0)) * Math.log10(fMax / F_MIN));
  const i = nearest(f, hz);
  ctx.strokeStyle = INK.axis;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(X(f[i]!), TOP);
  ctx.lineTo(X(f[i]!), h - BOTTOM);
  ctx.stroke();
  const parts = [`${f[i]!.toPrecision(3)} Hz`];
  if (mag.length) {
    dot(ctx, X(f[i]!), yMag(mag[i]!), SERIES.model, 3);
    dot(ctx, X(f[i]!), yPh(ph[i]!), SERIES.model, 3);
    parts.push(`model ${mag[i]!.toFixed(1)} dB ${ph[i]!.toFixed(0)}°`);
  }
  const j = measured.length
    ? nearest(
        measured.map((p) => p.fHz),
        hz,
      )
    : -1;
  if (j >= 0 && Math.abs(Math.log10(measured[j]!.fHz / hz)) < 0.08)
    parts.push(`measured ${mMag[j]!.toFixed(1)} dB ${mPh[j]!.toFixed(0)}°`);
  return parts.join('   ');
}

/**
 * Bode plot with the loop broken at the plant input: magnitude and phase on a shared logarithmic
 * frequency axis. The line is the model, the dots are flown by the simulator. The view selects
 * which response: the open loop, the closed loop, or the sensitivity.
 */
export function BodeChart() {
  const data = useLoopData();
  const measured = useMeasured(data);
  const running = useMeasure((s) => s.running);
  const progress = useMeasure((s) => s.progress);
  const start = useMeasure((s) => s.start);
  const level = useParams((s) => s.params.sim.level);
  const view = useUi((s) => s.bodeView);
  const setView = useUi((s) => s.setBodeView);
  const target = useUi((s) => s.bodeTarget);
  const probeHz = useParams((s) =>
    s.params.probe.point !== 'none' && s.params.probe.signal === 'sine'
      ? s.params.probe.freqHz
      : null,
  );
  const m = data.margins;
  const rt = useRobustTest(view === 'robust');
  const campaign = useCampaign();
  const famKey = useParams((s) => familyKey(s.params));
  const flown = campaign.key === famKey ? campaign.trials : [];
  const wb = useMemo(
    () => (view === 'waterbed' && data.model ? waterbed(data.model) : null),
    [view, data],
  );

  const { canvas, readout } = useChartCanvas(
    (frame) =>
      view === 'waterbed'
        ? wb
          ? drawWaterbed(frame, wb)
          : undefined
        : view === 'robust'
          ? rt
            ? drawRobust(frame, rt)
            : undefined
          : drawBode(frame, data, measured, view, level, probeHz, target),
    [data, measured, level, view, wb, probeHz, rt, target],
  );

  const failed = flown.filter((t) => !t.stable).length;
  const summary =
    view === 'robust' ? (
      rt ? (
        <span>
          {rt.worst < 1
            ? `● passed: every member is stable (largest |W·T| ${rt.worst.toFixed(2)})`
            : `▲ failed at ${rt.fWorst.toFixed(1)} Hz (|W·T| ${rt.worst.toFixed(2)}): a member may be unstable`}
          {flown.length
            ? ` · flown: ${flown.length - failed} of ${flown.length} stable${campaign.running ? '…' : ''}`
            : ''}
        </span>
      ) : null
    ) : view === 'waterbed' ? (
      wb ? (
        <span>
          area rejected {wb.below.toFixed(2)} = area amplified {wb.above.toFixed(2)} (∫ ln|S| df,
          Hz): what the loop takes out at low frequency it puts back higher up
        </span>
      ) : null
    ) : !m ? null : view === 'loop' ? (
      <span>
        {Number.isFinite(m.pmDeg)
          ? `PM ${m.pmDeg.toFixed(0)}° at ${(m.wc / 2 / Math.PI).toFixed(2)} Hz`
          : 'no gain crossover'}
        {Number.isFinite(m.gm) ? ` · GM ${dB(m.gm).toFixed(0)} dB` : ' · GM ∞'}
        {Number.isFinite(m.delayMargin)
          ? ` · delay margin ${(m.delayMargin * 1000).toFixed(0)} ms`
          : ''}
      </span>
    ) : view === 'ref' ? (
      <span>
        {Number.isFinite(data.bandwidthHz)
          ? `bandwidth (−3 dB) ${data.bandwidthHz.toFixed(2)} Hz: slower setpoint motion is followed, faster is not`
          : 'no −3 dB point below the Nyquist frequency'}
      </span>
    ) : (
      <span>
        peak of |S| {m.ms.toFixed(2)} ({dB(m.ms).toFixed(1)} dB) at{' '}
        {(m.wMs / 2 / Math.PI).toFixed(2)} Hz: disturbances there are amplified
      </span>
    );

  return (
    <ChartCard
      title={TITLE[view]}
      readout={readout}
      legend={
        view === 'waterbed'
          ? [{ label: 'model', color: SERIES.model, mark: 'line' }]
          : view === 'robust'
            ? [
                { label: '|T| nominal', color: SERIES.model, mark: 'line' },
                { label: 'ceiling 1/|W|', color: SERIES.measured, mark: 'line' },
              ]
            : [
                { label: 'model', color: SERIES.model, mark: 'line' },
                { label: 'measured', color: SERIES.measured, mark: 'dot' },
              ]
      }
      summary={summary}
      controls={
        <>
          <Select value={view} onValueChange={(v) => setView(v as BodeView)} options={VIEWS} />
          {view === 'robust' && (
            <Button
              size="sm"
              className="h-5 px-2 text-[11px]"
              disabled={campaign.running || level !== 1}
              onClick={() => campaign.start(60)}
              title="Draw sixty drones at random from the family and fly each one for ten seconds."
            >
              {campaign.running ? `flying ${flown.length} / 60` : 'Fly 60 members'}
            </Button>
          )}
          {view !== 'waterbed' && view !== 'robust' && (
            <Button
              size="sm"
              className="h-5 px-2 text-[11px]"
              disabled={running || level !== 1}
              onClick={() => start(view)}
              title="Fly the simulator with a sine on the probe at fourteen frequencies and plot what comes back."
            >
              {running ? `measuring ${(progress * 100).toFixed(0)} %` : 'Measure'}
            </Button>
          )}
        </>
      }
    >
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </ChartCard>
  );
}
