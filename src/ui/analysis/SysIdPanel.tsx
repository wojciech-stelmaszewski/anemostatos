import { useState } from 'react';
import { Sweep } from '@/analysis/sweep';
import {
  ChirpExperiment,
  chirpKey,
  finishedChirp,
  fitChirp,
  fitPlant,
  plantModel,
  relayPrediction,
  relayTest,
  storeChirp,
  type PlantFit,
  type RelayResult,
} from '@/estimation/sysid';
import { cabs } from '@/math/complex';
import { logspace } from '@/math/margins';
import { useParams } from '@/store/params';
import { Button } from '@/ui/components/button';
import { dot, FONT, INK, SERIES, useChartCanvas } from './canvas';

/** Physics steps per timer tick: the page stays responsive between them. */
const SLICE = 20000;
const fmt = (v: number, d = 3) => (Number.isFinite(v) ? v.toFixed(d) : '—');

function Row({
  label,
  fit,
  believed,
  truth,
}: {
  label: string;
  fit: string;
  believed?: string;
  truth: string;
}) {
  return (
    <tr className="border-t border-border/50">
      <td className="py-0.5 pr-1 font-sans text-fg">{label}</td>
      <td className="px-1 text-right text-fg">{fit}</td>
      <td className="px-1 text-right text-muted">{believed ?? ''}</td>
      <td className="px-1 text-right text-muted">{truth}</td>
    </tr>
  );
}

/**
 * Lesson III.25: fly a sweep with the probe on the thrust, fit m, τ and the delay to the measured
 * plant, put the fit into the controller's model; and the relay experiment on vertical velocity.
 */
export function SweepFitPanel() {
  const params = useParams((s) => s.params);
  const set = useParams((s) => s.set);
  const [fit, setFit] = useState<PlantFit | null>(null);
  const [progress, setProgress] = useState<number | null>(null);
  const [relay, setRelay] = useState<(RelayResult & { pku: number; ptu: number }) | null>(null);

  const sweep = () => {
    const p = useParams.getState().params;
    const s = new Sweep(p, logspace(0.2, 5, 10), { point: 'l1.thrust' });
    setFit(null);
    setProgress(0);
    const tick = () => {
      const done = s.advance(SLICE);
      setProgress(s.progress);
      if (done) {
        setFit(fitPlant(s.points.map((q) => ({ fHz: q.fHz, p: q.pm! }))));
        setProgress(null);
      } else setTimeout(tick, 0);
    };
    setTimeout(tick, 0);
  };
  const runRelay = () => {
    const p = useParams.getState().params;
    const r = relayTest(p);
    const pr = relayPrediction(p);
    setRelay({ ...r, pku: pr.ku, ptu: pr.tu });
  };

  return (
    <div className="mt-2 space-y-1.5">
      <div className="flex flex-wrap items-center gap-2">
        <Button variant="accent" disabled={progress !== null} onClick={sweep}>
          {progress !== null ? `Sweeping… ${(progress * 100).toFixed(0)} %` : 'Sweep and fit'}
        </Button>
        <Button
          disabled={!fit}
          onClick={() => {
            if (!fit) return;
            set('control.model.mass', +fit.mass.toFixed(3));
            set('control.model.motorTau', +fit.tau.toFixed(4));
          }}
        >
          Use the fitted model
        </Button>
        <Button onClick={runRelay}>Relay test</Button>
      </div>
      {fit && (
        <table className="w-full border-collapse font-mono text-[10px]">
          <thead>
            <tr className="text-muted">
              <th className="py-0.5 pr-1 text-left font-normal">parameter</th>
              <th className="px-1 text-right font-normal">fitted</th>
              <th className="px-1 text-right font-normal">model now</th>
              <th className="px-1 text-right font-normal">truth</th>
            </tr>
          </thead>
          <tbody>
            <Row
              label="mass, kg"
              fit={fmt(fit.mass)}
              believed={fmt(params.control.model.mass)}
              truth={fmt(params.drone.mass)}
            />
            <Row
              label="motor lag, ms"
              fit={fmt(fit.tau * 1000, 1)}
              believed={fmt(params.control.model.motorTau * 1000, 1)}
              truth={fmt(params.drone.motorTau * 1000, 1)}
            />
            <Row
              label="delay, ms"
              fit={fmt(fit.delay * 1000, 1)}
              truth={`${params.sensors.delayMs} + sampling`}
            />
          </tbody>
        </table>
      )}
      {fit && (
        <p className="text-[10px] text-muted">
          {fit.used} frequencies, magnitude misfit {fit.rmsDb.toFixed(2)} dB RMS.
        </p>
      )}
      {relay && (
        <p className="font-mono text-[10px] text-fg">
          relay ±0.5 N on vertical velocity: Ku {relay.ku.toFixed(1)} N·s/m, Tu{' '}
          {(relay.tu * 1000).toFixed(0)} ms · the model predicts Ku {relay.pku.toFixed(1)}, Tu{' '}
          {(relay.ptu * 1000).toFixed(0)} ms
        </p>
      )}
    </div>
  );
}

/**
 * Lesson III.26: fly a chirp in turbulence, estimate the plant two ways with its coherence, and
 * fit it on the frequencies the settings allow.
 */
export function ChirpFitPanel() {
  const params = useParams((s) => s.params);
  const key = chirpKey(params);
  const [, bump] = useState(0);
  const [progress, setProgress] = useState<number | null>(null);
  const id = finishedChirp(params);
  const s = params.sysid;
  const fit = id ? fitChirp(id, s.minCoherence, s.estimator) : null;

  const fly = () => {
    const p = useParams.getState().params;
    const e = new ChirpExperiment(p, {
      f0: 0.2,
      f1: 8,
      seconds: p.sysid.seconds,
      amp: p.sysid.amp,
    });
    setProgress(0);
    const tick = () => {
      const done = e.advance(SLICE);
      setProgress(e.progress);
      if (done) {
        storeChirp(p, e.result!);
        setProgress(null);
        bump((x) => x + 1);
      } else setTimeout(tick, 0);
    };
    setTimeout(tick, 0);
  };

  const { canvas, readout } = useChartCanvas(
    ({ ctx, w, h, hover }) => {
      ctx.font = FONT;
      const x0 = 36;
      const x1 = w - 8;
      const magH = (h - 26) * 0.68;
      const cohTop = magH + 10;
      const cohH = h - 18 - cohTop;
      if (!id) {
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText('Fly the chirp: each frequency becomes a point.', w / 2, h / 2);
        return;
      }
      const e = id[s.estimator];
      const X = (f: number) => x0 + (Math.log(f / 0.2) / Math.log(8 / 0.2)) * (x1 - x0);
      const db = (c: { re: number; im: number }) => 20 * Math.log10(cabs(c));
      const vals = e.h.map(db);
      const hi = Math.ceil(Math.max(...vals) / 10) * 10;
      const lo = hi - 70;
      const Y = (v: number) => 4 + ((hi - Math.min(Math.max(v, lo), hi)) / (hi - lo)) * magH;
      const Yc = (c: number) => cohTop + (1 - c) * cohH;
      ctx.lineWidth = 1;
      for (const f of [0.2, 0.5, 1, 2, 5, 8]) {
        ctx.strokeStyle = INK.grid;
        ctx.beginPath();
        ctx.moveTo(X(f), 4);
        ctx.lineTo(X(f), cohTop + cohH);
        ctx.stroke();
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'top';
        ctx.fillText(`${f}`, X(f), cohTop + cohH + 3);
      }
      ctx.textAlign = 'right';
      ctx.textBaseline = 'middle';
      for (let v = lo; v <= hi; v += 20) {
        ctx.fillStyle = INK.muted;
        ctx.fillText(`${v}`, x0 - 4, Y(v));
      }
      ctx.fillText('γ² 1', x0 - 4, Yc(1));
      ctx.fillText('0', x0 - 4, Yc(0));
      // The threshold.
      ctx.strokeStyle = INK.axis;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(x0, Yc(s.minCoherence));
      ctx.lineTo(x1, Yc(s.minCoherence));
      ctx.stroke();
      ctx.setLineDash([]);
      // The fitted model.
      if (fit && Number.isFinite(fit.mass)) {
        ctx.strokeStyle = SERIES.model;
        ctx.lineWidth = 2;
        ctx.beginPath();
        for (let i = 0; i <= 100; i++) {
          const f = 0.2 * (8 / 0.2) ** (i / 100);
          const v = db(plantModel(fit, 2 * Math.PI * f));
          if (i) ctx.lineTo(X(f), Y(v));
          else ctx.moveTo(X(f), Y(v));
        }
        ctx.stroke();
      }
      // The estimate, coloured by whether the fit uses it; the coherence below.
      id.fHz.forEach((f, i) => {
        const used = e.coherence[i]! >= s.minCoherence;
        dot(ctx, X(f), Y(vals[i]!), used ? SERIES.measured : INK.muted, 2.5);
        dot(ctx, X(f), Yc(e.coherence[i]!), used ? SERIES.measured : INK.muted, 2);
      });
      if (!hover) return;
      let best = 0;
      id.fHz.forEach((f, i) => {
        if (Math.abs(X(f) - hover.x) < Math.abs(X(id.fHz[best]!) - hover.x)) best = i;
      });
      return `${id.fHz[best]!.toFixed(2)} Hz   |H| ${vals[best]!.toFixed(1)} dB   γ² ${e.coherence[best]!.toFixed(2)}`;
    },
    [id, s.estimator, s.minCoherence, fit, key],
  );

  return (
    <div className="mt-2 space-y-1.5">
      <div className="flex flex-wrap items-center gap-2">
        <Button variant="accent" disabled={progress !== null} onClick={fly}>
          {progress !== null
            ? `Flying the chirp… ${(progress * 100).toFixed(0)} %`
            : 'Fly the chirp'}
        </Button>
        <span className="font-mono text-[10px] text-fg" ref={readout} />
      </div>
      <canvas ref={canvas} className="h-56 w-full rounded border border-border bg-bg/60" />
      {fit && (
        <table className="w-full border-collapse font-mono text-[10px]">
          <thead>
            <tr className="text-muted">
              <th className="py-0.5 pr-1 text-left font-normal">parameter</th>
              <th className="px-1 text-right font-normal">fitted</th>
              <th className="px-1 text-right font-normal" />
              <th className="px-1 text-right font-normal">truth</th>
            </tr>
          </thead>
          <tbody>
            <Row label="mass, kg" fit={fmt(fit.mass)} truth={fmt(params.drone.mass)} />
            <Row
              label="motor lag, ms"
              fit={fmt(fit.tau * 1000, 1)}
              truth={fmt(params.drone.motorTau * 1000, 1)}
            />
            <Row
              label="delay, ms"
              fit={fmt(fit.delay * 1000, 1)}
              truth={`${params.sensors.delayMs}`}
            />
          </tbody>
        </table>
      )}
      {fit && <p className="text-[10px] text-muted">{fit.used} frequencies fitted.</p>}
    </div>
  );
}
