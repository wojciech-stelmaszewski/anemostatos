import { cabs } from '@/math/complex';
import { unwrapPhase } from '@/math/margins';
import { useParams } from '@/store/params';
import { Button } from '@/ui/components/button';
import { ChartCard } from './ChartCard';
import { dot, FONT, INK, niceTicks, SERIES, useChartCanvas } from './canvas';
import { F_MIN, useLoopData, useMeasure, useMeasured } from './store';

const DEG = 180 / Math.PI;
const dB = (x: number) => 20 * Math.log10(x);
const LEFT = 40;
const RIGHT = 10;
const TOP = 6;
const BOTTOM = 18;
const GAP = 12;

/** Phases in degrees, continuous, shifted by whole turns so that they start in (−360°, 0°]. */
function phasesDeg(l: { re: number; im: number }[]): number[] {
  const ph = unwrapPhase(l).map((v) => v * DEG);
  if (!ph.length) return ph;
  const shift = 360 * Math.ceil(ph[0]! / 360 - 1e-9);
  return ph.map((v) => v - shift);
}

/**
 * Bode plot of the open loop L with the loop broken at the plant input: magnitude and phase on a
 * shared logarithmic frequency axis. The line is the model, the dots are flown by the simulator.
 */
export function BodeChart() {
  const data = useLoopData();
  const measured = useMeasured(data);
  const running = useMeasure((s) => s.running);
  const progress = useMeasure((s) => s.progress);
  const start = useMeasure((s) => s.start);
  const level = useParams((s) => s.params.sim.level);
  const m = data.margins;

  const { canvas, readout } = useChartCanvas(
    ({ ctx, w, h, hover }) => {
      const f = data.fHz;
      const fMax = f[f.length - 1]!;
      const x0 = LEFT;
      const x1 = w - RIGHT;
      const paneH = (h - TOP - BOTTOM - GAP) / 2;
      const X = (hz: number) =>
        x0 +
        ((Math.log10(hz) - Math.log10(F_MIN)) / (Math.log10(fMax) - Math.log10(F_MIN))) * (x1 - x0);

      const mag = data.l.map((v) => dB(cabs(v)));
      const ph = phasesDeg(data.l);
      const mMag = measured.map((p) => dB(cabs(p.l!)));
      // The measured phases take the turn of the model's phase at the same frequency.
      const mPh = measured.map((p) => {
        let v = Math.atan2(p.l!.im, p.l!.re) * DEG;
        const near = ph.length ? ph[nearest(f, p.fHz)]! : -180;
        while (v - near > 180) v -= 360;
        while (v - near < -180) v += 360;
        return v;
      });

      const all = [...mag, ...mMag];
      const magLo = all.length ? Math.floor((Math.min(...all) - 3) / 20) * 20 : -60;
      const magHi = all.length ? Math.ceil((Math.max(...all) + 3) / 20) * 20 : 40;
      // Delay drives the phase down without end; the story is told between −360° and 0°.
      const allPh = [...ph, ...mPh, -180];
      const phLo = Math.max(-450, Math.floor((Math.min(...allPh) - 10) / 90) * 90);
      const phHi = Math.min(90, Math.ceil((Math.max(...allPh) + 10) / 90) * 90);
      const phStep = paneH / ((phHi - phLo) / 90) >= 16 ? 90 : 180;
      const yMag = (v: number) => TOP + ((magHi - v) / (magHi - magLo)) * paneH;
      const yPh = (v: number) => TOP + paneH + GAP + ((phHi - v) / (phHi - phLo)) * paneH;

      ctx.font = FONT;
      ctx.lineWidth = 1;
      // Grid and ticks.
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
      for (let v = Math.ceil(phLo / phStep) * phStep; v <= phHi; v += phStep) {
        hLine(yPh(v), v === -180);
        ctx.fillStyle = INK.muted;
        ctx.fillText(`${v}°`, x0 - 5, yPh(v));
      }
      // The curves fall from the upper left, so the upper right of each pane is free.
      ctx.fillText('|L|  dB', x1 - 3, TOP + 7);
      ctx.fillText('∠L', x1 - 3, TOP + paneH + GAP + 7);
      ctx.textAlign = 'left';

      if (!data.model && !measured.length) {
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

      // The margins, drawn as the distances they are. Their labels go on last, on a backing.
      const notes: [string, number, number][] = [];
      ctx.lineWidth = 2;
      ctx.strokeStyle = INK.text;
      ctx.fillStyle = INK.text;
      ctx.textAlign = 'left';
      if (m && Number.isFinite(m.pmDeg) && ph.length) {
        const fc = m.wc / (2 * Math.PI);
        const at = ph[nearest(f, fc)]!;
        const base = at - m.pmDeg; // the odd multiple of −180° that the margin is measured from
        ctx.beginPath();
        ctx.moveTo(X(fc), yPh(base));
        ctx.lineTo(X(fc), yPh(at));
        ctx.stroke();
        notes.push([`PM ${m.pmDeg.toFixed(0)}°`, X(fc) + 5, (yPh(base) + yPh(at)) / 2]);
      }
      if (m && Number.isFinite(m.gm) && mag.length) {
        const f180 = m.w180 / (2 * Math.PI);
        ctx.beginPath();
        ctx.moveTo(X(f180), yMag(0));
        ctx.lineTo(X(f180), yMag(-dB(m.gm)));
        ctx.stroke();
        notes.push([`GM ${dB(m.gm).toFixed(0)} dB`, X(f180) + 5, (yMag(0) + yMag(-dB(m.gm))) / 2]);
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

      for (const [text, x, y] of notes) {
        const tw = ctx.measureText(text).width;
        const tx = Math.min(x, x1 - tw - 3);
        ctx.fillStyle = 'rgba(11,14,19,0.85)';
        ctx.fillRect(tx - 2, y - 7, tw + 4, 14);
        ctx.fillStyle = INK.text;
        ctx.textAlign = 'left';
        ctx.fillText(text, tx, y);
      }

      if (!hover || hover.x < x0 || hover.x > x1) return;
      // The crosshair finds the frequency; the readout lists every series there.
      const hz =
        10 ** (Math.log10(F_MIN) + ((hover.x - x0) / (x1 - x0)) * Math.log10(fMax / F_MIN));
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
    },
    [data, measured, level],
  );

  return (
    <ChartCard
      title="Bode plot of the open loop L"
      readout={readout}
      legend={[
        { label: 'model', color: SERIES.model, mark: 'line' },
        { label: 'measured', color: SERIES.measured, mark: 'dot' },
      ]}
      summary={
        m ? (
          <span>
            {Number.isFinite(m.pmDeg)
              ? `PM ${m.pmDeg.toFixed(0)}° at ${(m.wc / 2 / Math.PI).toFixed(2)} Hz`
              : 'no gain crossover'}
            {Number.isFinite(m.gm) ? ` · GM ${dB(m.gm).toFixed(0)} dB` : ' · GM ∞'}
            {Number.isFinite(m.delayMargin)
              ? ` · delay margin ${(m.delayMargin * 1000).toFixed(0)} ms`
              : ''}
          </span>
        ) : null
      }
      controls={
        <Button
          size="sm"
          className="h-5 px-2 text-[11px]"
          disabled={running || level !== 1}
          onClick={start}
          title="Fly the simulator with a sine on the thrust command at fourteen frequencies and plot what comes back."
        >
          {running ? `measuring ${(progress * 100).toFixed(0)} %` : 'Measure'}
        </Button>
      }
    >
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </ChartCard>
  );
}

/** Index of the value in the ascending array `xs` closest to `x` on a logarithmic scale. */
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
