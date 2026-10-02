import { useEffect, useMemo, useRef } from 'react';
import { attractionBoundary, brakingLimit, lyapunovEstimate } from '@/analysis/attraction';
import { useParams } from '@/store/params';
import { sim, useUi } from '@/store/sim';
import type { Telemetry } from '@/engine/telemetry';
import { SIGNAL } from '@/ui/colors';
import type { LoopMeta } from './loops';

const INK = '#8b94a5';
const GRID = 'rgba(255,255,255,0.06)';
/** Setpoint rates above this are steps, m/s. */
const STEP_RATE = 20;

/**
 * Error e and its rate ė for a position-like loop. Uses the true state (setpoint − true position,
 * setpoint rate − true velocity) when the loop has one, so sensor noise does not scribble over
 * the picture; otherwise differentiates the measured error.
 */
function phaseData(tl: Telemetry, meta: LoopMeta, from: number): { e: number[]; de: number[] } {
  const axis = meta.truth?.startsWith('pos.') ? meta.truth.slice(4) : null;
  if (axis) {
    // `sp.*` is the physics-rate setpoint; the loop's own `.sp` is sampled at the loop rate and
    // aliases against the telemetry rate, which would scribble over ė.
    const { t, series } = tl.window([`sp.${axis}`, meta.truth!, `vel.${axis}`], from);
    const [sp, pos, vel] = series as [number[], number[], number[]];
    const e = sp.map((s, k) => s - pos[k]!);
    const de = vel.map((v, k) => {
      const k0 = Math.max(0, k - 1);
      const dt = t[k]! - t[k0]!;
      const spRate = dt > 0 ? (sp[k]! - sp[k0]!) / dt : 0;
      // A setpoint step is a jump of e at constant ė — a horizontal hop in the phase plane,
      // not an infinite ė spike. Ramps and sines are slow enough to pass.
      return (Math.abs(spRate) > STEP_RATE ? 0 : spRate) - v;
    });
    return { e, de };
  }
  const { t, series } = tl.window([`${meta.id}.err`], from);
  const err = series[0]!;
  const de = err.map((_, k) => {
    const a = Math.max(0, k - 2);
    const b = Math.min(err.length - 1, k + 2);
    return b > a ? (err[b]! - err[a]!) / (t[b]! - t[a]!) : 0;
  });
  return { e: err, de };
}

/**
 * What is known about the region of attraction of the L1 hover, in the portrait's coordinates
 * (e = r − y, ė = −v): the largest Lyapunov level set that is proven safe, the limit set by the
 * motors alone, and the boundary the simulator actually has (docs/analysis.md §4).
 */
interface Regions {
  h: number;
  aMax: number;
  kp: number;
  m: number;
  c: number;
  flown: { e: number; de: number }[];
}

function useRegions(on: boolean): Regions | null {
  const key = useParams((s) => {
    const p = s.params;
    return on && p.sim.level === 1 && p.control.l1.kind === 'pid'
      ? JSON.stringify([
          p.control.alt,
          p.control.feedforward,
          p.control.model,
          p.control.rateHz,
          p.drone,
          p.setpoint.y,
          p.sensors,
        ])
      : null;
  });
  return useMemo(() => {
    if (key === null) return null;
    const p = useParams.getState().params;
    const { h, aMax } = brakingLimit(p);
    if (!(aMax > 0) || !(h > 0)) return null;
    return {
      h,
      aMax,
      kp: p.control.alt.kp,
      m: p.drone.mass,
      c: lyapunovEstimate(p).c,
      flown: attractionBoundary(p, 1.2 * h, 8).map((b) => ({ e: -b.x, de: -b.v })),
    };
  }, [key]);
}

const REGION = { proven: '#3987e5', flown: '#d95926', limit: '#9aa3b2' } as const;

/** Phase portrait: the loop's trajectory in the (e, ė) plane — state, not just error. */
export function PhasePortrait({ meta, lyapunov = false }: { meta: LoopMeta; lyapunov?: boolean }) {
  const regions = useRegions(lyapunov);
  const regionsRef = useRef(regions);
  regionsRef.current = regions;
  const canvas = useRef<HTMLCanvasElement>(null);
  const windowSec = useUi((s) => s.window);
  const windowRef = useRef(windowSec);
  windowRef.current = windowSec;

  useEffect(() => {
    const cv = canvas.current!;
    const ctx = cv.getContext('2d')!;
    let raf = 0;
    let seen = -1;
    let span = { e: 0.1, de: 0.1 };

    const draw = () => {
      raf = requestAnimationFrame(draw);
      const tl = sim.telemetry;
      if (tl.version === seen) return;
      seen = tl.version;
      const dpr = devicePixelRatio;
      const w = cv.clientWidth * dpr;
      const h = cv.clientHeight * dpr;
      if (cv.width !== w || cv.height !== h) {
        cv.width = w;
        cv.height = h;
      }
      const from = tl.lastTime - windowRef.current;
      const live = phaseData(tl, meta, from);
      const ghost = sim.ghost ? phaseData(sim.ghost.telemetry, meta, -Infinity) : null;

      // Symmetric scale that grows at once and shrinks slowly.
      const maxAbs = (a: number[]) =>
        a.reduce((m, v) => (Number.isNaN(v) ? m : Math.max(m, Math.abs(v))), 0);
      const rg = regionsRef.current;
      const target = {
        e: Math.max(maxAbs(live.e) * 1.15, 0.02, rg ? rg.h * 1.25 : 0),
        de: Math.max(
          maxAbs(live.de) * 1.15,
          0.02,
          rg ? maxAbs(rg.flown.map((f) => f.de)) * 1.1 : 0,
        ),
      };
      span = {
        e: target.e > span.e ? target.e : span.e + (target.e - span.e) * 0.02,
        de: target.de > span.de ? target.de : span.de + (target.de - span.de) * 0.02,
      };
      const pad = 26 * dpr;
      const cx = w / 2;
      const cy = h / 2;
      const sx = (w / 2 - pad) / span.e;
      const sy = (h / 2 - pad * 0.6) / span.de;
      const X = (e: number) => cx + e * sx;
      const Y = (de: number) => cy - de * sy;

      ctx.clearRect(0, 0, w, h);
      ctx.lineWidth = dpr;
      ctx.strokeStyle = GRID;
      ctx.beginPath();
      ctx.moveTo(pad * 0.4, cy);
      ctx.lineTo(w - pad * 0.4, cy);
      ctx.moveTo(cx, pad * 0.2);
      ctx.lineTo(cx, h - pad * 0.2);
      ctx.stroke();
      ctx.fillStyle = INK;
      ctx.font = `${10 * dpr}px ui-monospace, monospace`;
      ctx.textAlign = 'right';
      ctx.fillText(`e →  ±${span.e.toPrecision(2)} ${meta.unit}`, w - 6 * dpr, h - 6 * dpr);
      ctx.textAlign = 'left';
      ctx.fillText(`ė ↑  ±${span.de.toPrecision(2)} ${meta.unit}/s`, 6 * dpr, 12 * dpr);

      if (rg) {
        const kv = Math.sqrt(rg.kp / rg.m);
        // Level sets of V = ½·Kp·e² + ½·m·ė²: ellipses. A path that crosses them outwards gains V.
        ctx.lineWidth = dpr;
        ctx.strokeStyle = 'rgba(255,255,255,0.09)';
        for (const a of [0.25, 0.5, 0.75, 1, 1.25].map((k) => k * rg.h)) {
          ctx.beginPath();
          ctx.ellipse(cx, cy, a * sx, a * kv * sy, 0, 0, 2 * Math.PI);
          ctx.stroke();
        }
        // The floor: e = h.
        ctx.strokeStyle = REGION.limit;
        ctx.setLineDash([2 * dpr, 3 * dpr]);
        ctx.beginPath();
        ctx.moveTo(X(rg.h), pad * 0.2);
        ctx.lineTo(X(rg.h), h - pad * 0.2);
        ctx.stroke();
        // What the motors allow at best: stop within the room left, ė² = 2·a_max·(h − e).
        ctx.beginPath();
        for (let k = 0; k <= 60; k++) {
          const e = -span.e + ((rg.h + span.e) * k) / 60;
          const y = Y(Math.sqrt(2 * rg.aMax * (rg.h - e)));
          if (k) ctx.lineTo(X(e), y);
          else ctx.moveTo(X(e), y);
        }
        ctx.stroke();
        ctx.setLineDash([]);
        // What this controller does, flown by the simulator from a grid of initial states.
        ctx.strokeStyle = REGION.flown;
        ctx.lineWidth = 2 * dpr;
        ctx.beginPath();
        rg.flown.forEach((f, k) => (k ? ctx.lineTo(X(f.e), Y(f.de)) : ctx.moveTo(X(f.e), Y(f.de))));
        ctx.stroke();
        // What the Lyapunov function proves: the largest level set with no saturation in it.
        const a = Math.sqrt((2 * rg.c) / rg.kp);
        ctx.fillStyle = 'rgba(57,135,229,0.28)';
        ctx.strokeStyle = REGION.proven;
        ctx.lineWidth = 1.5 * dpr;
        ctx.beginPath();
        ctx.ellipse(cx, cy, a * sx, a * kv * sy, 0, 0, 2 * Math.PI);
        ctx.fill();
        ctx.stroke();
        ctx.fillStyle = INK;
        ctx.font = `${10 * dpr}px ui-monospace, monospace`;
        ctx.textAlign = 'right';
        ctx.fillText('floor', X(rg.h) - 4 * dpr, 24 * dpr);
      }

      // Sliding mode: the line s = ė + λ·e = 0, and the boundary layer |s| < φ around it.
      const P = sim.params;
      if (P.sim.level === 1 && P.control.l1.kind === 'smc' && meta.id === 'alt') {
        const { lambda, phi } = P.control.smc;
        const line = (off: number) => {
          ctx.beginPath();
          ctx.moveTo(X(-span.e), Y(lambda * span.e + off));
          ctx.lineTo(X(span.e), Y(-lambda * span.e + off));
          ctx.stroke();
        };
        ctx.strokeStyle = 'rgba(57,135,229,0.85)';
        ctx.lineWidth = 1.5 * dpr;
        line(0);
        if (phi > 0) {
          ctx.setLineDash([3 * dpr, 3 * dpr]);
          ctx.lineWidth = dpr;
          line(phi);
          line(-phi);
          ctx.setLineDash([]);
        }
        ctx.fillStyle = INK;
        ctx.textAlign = 'left';
        ctx.fillText('s = 0', X(-span.e * 0.92), Y(lambda * span.e * 0.92) + 12 * dpr);
      }

      if (ghost) {
        ctx.strokeStyle = SIGNAL.ghost;
        ctx.lineWidth = 1.2 * dpr;
        ctx.beginPath();
        ghost.e.forEach((e, k) => {
          if (Number.isNaN(e)) return;
          if (k === 0) ctx.moveTo(X(e), Y(ghost.de[k]!));
          else ctx.lineTo(X(e), Y(ghost.de[k]!));
        });
        ctx.stroke();
      }

      // Older samples fade out.
      const n = live.e.length;
      const chunk = Math.max(1, Math.ceil(n / 24));
      ctx.lineWidth = 1.8 * dpr;
      ctx.strokeStyle = SIGNAL.error;
      for (let s = 0; s < n - 1; s += chunk) {
        ctx.globalAlpha = 0.15 + 0.85 * (s / n);
        ctx.beginPath();
        let started = false;
        for (let k = s; k <= Math.min(n - 1, s + chunk); k++) {
          const e = live.e[k]!;
          const de = live.de[k]!;
          if (Number.isNaN(e) || Number.isNaN(de)) continue;
          if (!started) ctx.moveTo(X(e), Y(de));
          else ctx.lineTo(X(e), Y(de));
          started = true;
        }
        ctx.stroke();
      }
      ctx.globalAlpha = 1;
      if (n) {
        ctx.fillStyle = SIGNAL.output;
        ctx.beginPath();
        ctx.arc(X(live.e[n - 1]!), Y(live.de[n - 1]!), 3.5 * dpr, 0, 2 * Math.PI);
        ctx.fill();
      }
    };
    draw();
    return () => cancelAnimationFrame(raf);
  }, [meta]);

  return (
    <div className="flex h-full min-h-0 flex-col rounded-md border border-border bg-bg/60">
      <div className="flex items-center gap-x-3 px-2 pt-1.5 text-[11px]">
        <span className="font-semibold text-fg">Phase portrait (e, ė)</span>
        {regions ? (
          <>
            {(
              [
                ['proven', REGION.proven],
                ['flown limit', REGION.flown],
                ['motor limit', REGION.limit],
              ] as const
            ).map(([label, color]) => (
              <span key={label} className="flex items-center gap-1 text-muted">
                <span className="inline-block h-0.5 w-3.5 rounded" style={{ background: color }} />
                {label}
              </span>
            ))}
          </>
        ) : (
          <span className="text-muted">the loop's state; it should spiral into the origin</span>
        )}
      </div>
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </div>
  );
}
