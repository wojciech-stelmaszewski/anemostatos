import { useRef } from 'react';
import { useParams } from '@/store/params';
import { sim } from '@/store/sim';
import { SIGNAL } from '@/ui/colors';
import { useRaf } from './useRaf';

/** Compass-style wind indicator: horizontal direction (top view), vertical component, speed. */
export function WindHud() {
  const level = useParams((s) => s.params.sim.level);
  const arrow = useRef<SVGGElement>(null);
  const bar = useRef<SVGRectElement>(null);
  const text = useRef<HTMLDivElement>(null);
  const gust = useRef<HTMLDivElement>(null);

  useRaf(() => {
    const w = sim.wind.velocity;
    const h = Math.hypot(w.x, w.z);
    // Top view: +x right, +z down on screen.
    const ang = (Math.atan2(w.z, w.x) * 180) / Math.PI;
    const len = Math.min(1, h / 10);
    arrow.current?.setAttribute('transform', `rotate(${ang} 32 32) scale(1)`);
    arrow.current?.setAttribute('opacity', level === 1 ? '0.25' : String(0.35 + len * 0.65));
    const vy = Math.max(-10, Math.min(10, w.y));
    if (bar.current) {
      const hgt = Math.abs(vy) * 2.6;
      bar.current.setAttribute('y', String(vy >= 0 ? 32 - hgt : 32));
      bar.current.setAttribute('height', String(hgt));
    }
    if (text.current)
      text.current.textContent = `${Math.hypot(w.x, w.y, w.z).toFixed(1)} m/s · ↕ ${w.y >= 0 ? '+' : '−'}${Math.abs(w.y).toFixed(1)}`;
    if (gust.current)
      gust.current.style.opacity = sim.wind.gusts.some((g) => sim.t >= g.start) ? '1' : '0';
  });

  return (
    <div className="pointer-events-none flex items-center gap-2 rounded-lg border border-border/80 bg-panel/85 px-2 py-1.5 shadow-lg backdrop-blur">
      <svg width="64" height="64" viewBox="0 0 64 64" aria-label="wind direction">
        <circle cx="32" cy="32" r="26" fill="none" stroke="#242b37" />
        <text x="58" y="35" fontSize="7" fill="#8b94a5">
          x
        </text>
        <text x="30" y="62" fontSize="7" fill="#8b94a5">
          z
        </text>
        <g ref={arrow}>
          <line
            x1="12"
            y1="32"
            x2="48"
            y2="32"
            stroke={SIGNAL.wind}
            strokeWidth="2.5"
            strokeLinecap="round"
          />
          <path d="M48 32 l-7 -5 v10 z" fill={SIGNAL.wind} />
        </g>
      </svg>
      <svg width="10" height="64" viewBox="0 0 10 64">
        <rect x="4" y="6" width="2" height="52" fill="#242b37" />
        <rect ref={bar} x="1" y="32" width="8" height="0" fill={SIGNAL.wind} rx="1" />
      </svg>
      <div className="font-mono text-[10px] leading-tight">
        <div className="font-sans text-[10px] uppercase tracking-wider text-muted">Wind</div>
        <div ref={text} className="text-fg" />
        <div
          ref={gust}
          className="font-semibold transition-opacity"
          style={{ color: SIGNAL.wind, opacity: 0 }}
        >
          GUST
        </div>
      </div>
    </div>
  );
}
