import { useEffect, useRef } from 'react';
import uPlot from 'uplot';
import 'uplot/dist/uPlot.min.css';
import { sim, useUi } from '@/store/sim';

export interface SeriesSpec {
  key: string;
  label: string;
  color: string;
  dash?: number[];
  width?: number;
  /** Listed in the legend with its live value, but not drawn (keeps the y scale readable). */
  legendOnly?: boolean;
  /** Read from the frozen comparison run instead of the live one. */
  ghost?: boolean;
}

export interface TimeChartProps {
  title: string;
  unit?: string;
  series: SeriesSpec[];
  /** Channel that is 1 while the loop output is saturated; those intervals get shaded. */
  shadeKey?: string;
  /** Horizontal reference lines (e.g. actuator limits). */
  hlines?: () => { value: number; label: string }[];
  /** Vertical event markers (e.g. gust starts), in simulation time. */
  vlines?: () => number[];
  /** Always include zero in the y range. */
  includeZero?: boolean;
}

const INK = '#8b94a5';
const GRID = 'rgba(255,255,255,0.05)';
const fmt = (v: number | null | undefined) =>
  v == null || Number.isNaN(v)
    ? '—'
    : Math.abs(v) >= 100
      ? v.toFixed(1)
      : Math.abs(v) >= 10
        ? v.toFixed(2)
        : v.toFixed(3);

/**
 * Streaming time-series chart bound to the simulation telemetry. Updated imperatively every
 * animation frame; React only mounts it.
 */
export function TimeChart({
  title,
  unit,
  series,
  shadeKey,
  hlines,
  vlines,
  includeZero,
}: TimeChartProps) {
  const host = useRef<HTMLDivElement>(null);
  const valueEls = useRef<(HTMLSpanElement | null)[]>([]);
  const windowSec = useUi((s) => s.window);
  const windowRef = useRef(windowSec);
  windowRef.current = windowSec;
  const specKey =
    series.map((s) => (s.ghost ? `ghost:${s.key}` : s.key)).join('|') + (shadeKey ?? '');

  useEffect(() => {
    const el = host.current!;
    const shade: { t: number[]; v: number[] } = { t: [], v: [] };
    let cursorIdx: number | null = null;

    const opts: uPlot.Options = {
      width: el.clientWidth,
      height: el.clientHeight,
      legend: { show: false },
      cursor: {
        sync: { key: 'anemostatos' },
        points: { size: 6 },
        drag: { x: false, y: false },
      },
      scales: {
        x: { time: false },
        y: {
          range: (u) => {
            // Own min/max over the drawn series: uPlot's auto range breaks on all-NaN
            // series (e.g. the ghost trace when there is no ghost).
            let min = Infinity;
            let max = -Infinity;
            series.forEach((s, i) => {
              if (s.legendOnly) return;
              for (const v of (u.data[i + 1] ?? []) as (number | null)[]) {
                if (v == null || Number.isNaN(v)) continue;
                if (v < min) min = v;
                if (v > max) max = v;
              }
            });
            if (!Number.isFinite(min) || !Number.isFinite(max)) return [-1, 1];
            if (includeZero) {
              min = Math.min(min, 0);
              max = Math.max(max, 0);
            }
            for (const h of hlines?.() ?? []) {
              min = Math.min(min, h.value);
              max = Math.max(max, h.value);
            }
            const span = Math.max(max - min, 1e-3);
            return [min - span * 0.08, max + span * 0.08];
          },
        },
      },
      axes: [
        {
          stroke: INK,
          grid: { stroke: GRID, width: 1 },
          ticks: { show: false },
          size: 22,
          font: '10px ui-monospace, monospace',
          values: (_u, vals) => vals.map((v) => `${v.toFixed(0)}s`),
        },
        {
          stroke: INK,
          grid: { stroke: GRID, width: 1 },
          ticks: { show: false },
          size: 44,
          font: '10px ui-monospace, monospace',
        },
      ],
      series: [
        {},
        ...series.map((s) => ({
          label: s.label,
          stroke: s.color,
          width: s.width ?? 1.5,
          dash: s.dash,
          show: !s.legendOnly,
          points: { show: false },
          spanGaps: false,
        })),
      ],
      hooks: {
        drawClear: [
          (u) => {
            const ctx = u.ctx;
            const { top, height } = u.bbox;
            // Saturation intervals.
            if (shadeKey && shade.t.length) {
              ctx.save();
              ctx.fillStyle = 'rgba(230,103,103,0.13)';
              let start: number | null = null;
              for (let k = 0; k <= shade.t.length; k++) {
                const on = k < shade.t.length && shade.v[k] === 1;
                if (on && start === null) start = shade.t[k]!;
                if (!on && start !== null) {
                  const x0 = u.valToPos(start, 'x', true);
                  const x1 = u.valToPos(shade.t[k - 1]!, 'x', true);
                  ctx.fillRect(x0, top, Math.max(x1 - x0, 1), height);
                  start = null;
                }
              }
              ctx.restore();
            }
          },
        ],
        draw: [
          (u) => {
            const ctx = u.ctx;
            const { left, top, width, height } = u.bbox;
            ctx.save();
            ctx.font = `${10 * devicePixelRatio}px ui-monospace, monospace`;
            for (const h of hlines?.() ?? []) {
              const y = u.valToPos(h.value, 'y', true);
              if (y < top || y > top + height) continue;
              ctx.strokeStyle = 'rgba(230,103,103,0.6)';
              ctx.setLineDash([4 * devicePixelRatio, 4 * devicePixelRatio]);
              ctx.beginPath();
              ctx.moveTo(left, y);
              ctx.lineTo(left + width, y);
              ctx.stroke();
              ctx.fillStyle = 'rgba(230,103,103,0.9)';
              ctx.fillText(h.label, left + 4, y - 3 * devicePixelRatio);
            }
            const xs = u.scales.x;
            for (const t of vlines?.() ?? []) {
              if (t < (xs?.min ?? 0) || t > (xs?.max ?? 0)) continue;
              const x = u.valToPos(t, 'x', true);
              ctx.strokeStyle = 'rgba(63,185,80,0.45)';
              ctx.setLineDash([2 * devicePixelRatio, 3 * devicePixelRatio]);
              ctx.beginPath();
              ctx.moveTo(x, top);
              ctx.lineTo(x, top + height);
              ctx.stroke();
            }
            ctx.restore();
          },
        ],
        setCursor: [
          (u) => {
            cursorIdx = u.cursor.idx ?? null;
            updateLegend(u);
          },
        ],
      },
    };

    const updateLegend = (u: uPlot) => {
      const n = u.data[0]?.length ?? 0;
      const idx = cursorIdx ?? n - 1;
      series.forEach((_, i) => {
        const el = valueEls.current[i];
        if (el) el.textContent = n ? fmt(u.data[i + 1]?.[idx] as number) : '—';
      });
    };

    const u = new uPlot(opts, [[], ...series.map(() => [])], el);
    const ro = new ResizeObserver(() =>
      u.setSize({ width: el.clientWidth, height: el.clientHeight }),
    );
    ro.observe(el);

    let raf = 0;
    let seen = -1;
    let seenWindow = -1;
    const keys = series.map((s) => s.key);
    let ghostSeen: unknown = null;
    const loop = () => {
      raf = requestAnimationFrame(loop);
      const tl = sim.telemetry;
      if (tl.version === seen && windowRef.current === seenWindow && sim.ghost === ghostSeen)
        return;
      ghostSeen = sim.ghost;
      seen = tl.version;
      seenWindow = windowRef.current;
      const tNow = tl.lastTime;
      const from = tNow - windowRef.current;
      const names = shadeKey ? [...keys, shadeKey] : keys;
      const { t, series: data } = tl.window(names, from);
      const ghost = sim.ghost?.telemetry;
      series.forEach((s, i) => {
        if (!s.ghost) return;
        data[i] = ghost ? t.map((tk) => ghost.valueAt(s.key, tk)) : t.map(() => NaN);
      });
      if (shadeKey) {
        shade.t = t;
        shade.v = data.pop()!;
      }
      u.batch(() => {
        u.setData([t, ...data] as uPlot.AlignedData, false);
        u.setScale('x', { min: Math.max(0, from), max: Math.max(tNow, windowRef.current) });
      });
      updateLegend(u);
    };
    loop();

    return () => {
      cancelAnimationFrame(raf);
      ro.disconnect();
      u.destroy();
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [specKey]);

  return (
    <div className="flex h-full min-h-0 flex-col rounded-md border border-border bg-bg/60">
      <div className="flex flex-wrap items-center gap-x-3 gap-y-0.5 px-2 pt-1.5 text-[11px]">
        <span className="font-semibold text-fg">{title}</span>
        {unit && <span className="text-muted">[{unit}]</span>}
        <span className="flex-1" />
        {series.map((s, i) => (
          <span key={`${s.key}-${i}`} className="flex items-center gap-1 text-muted">
            <svg width="14" height="6" aria-hidden>
              <line
                x1="0"
                y1="3"
                x2="14"
                y2="3"
                stroke={s.color}
                strokeWidth="2"
                strokeDasharray={s.dash?.join(' ')}
              />
            </svg>
            {s.label}
            {s.legendOnly && <span className="text-[9px] opacity-60">(not plotted)</span>}
            <span ref={(r) => void (valueEls.current[i] = r)} className="min-w-9 font-mono text-fg">
              —
            </span>
          </span>
        ))}
      </div>
      <div ref={host} className="min-h-0 flex-1" />
    </div>
  );
}
