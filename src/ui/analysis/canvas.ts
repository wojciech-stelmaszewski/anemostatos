import { useEffect, useRef } from 'react';

/** Ink of the analysis charts, matching the time charts. Text never wears a series colour. */
export const INK = {
  text: '#e6e9ef',
  muted: '#8b94a5',
  grid: 'rgba(255,255,255,0.07)',
  axis: 'rgba(255,255,255,0.22)',
  surface: '#0b0e13',
} as const;
/** Two series, one hue each: the model (line) and the measurement (dots). Validated on INK.surface. */
export const SERIES = { model: '#3987e5', measured: '#d95926' } as const;

export interface Frame {
  ctx: CanvasRenderingContext2D;
  /** Size in CSS pixels; the context is already scaled by the device pixel ratio. */
  w: number;
  h: number;
  /** Pointer position in CSS pixels, or null when it is outside. */
  hover: { x: number; y: number } | null;
}

/**
 * A canvas that redraws when `deps` change, when it is resized and when the pointer moves.
 * `draw` may return a line of text for the readout under the pointer.
 */
export function useChartCanvas(draw: (f: Frame) => string | void, deps: unknown[]) {
  const canvas = useRef<HTMLCanvasElement>(null);
  const readout = useRef<HTMLSpanElement>(null);
  const hover = useRef<Frame['hover']>(null);
  const drawRef = useRef(draw);
  drawRef.current = draw;

  useEffect(() => {
    const cv = canvas.current!;
    const ctx = cv.getContext('2d')!;
    let pending = 0;
    const paint = () => {
      pending = 0;
      const dpr = devicePixelRatio || 1;
      const w = cv.clientWidth;
      const h = cv.clientHeight;
      if (w === 0 || h === 0) return;
      if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
        cv.width = Math.round(w * dpr);
        cv.height = Math.round(h * dpr);
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, w, h);
      const text = drawRef.current({ ctx, w, h, hover: hover.current });
      if (readout.current) readout.current.textContent = text || '';
    };
    const request = () => {
      if (!pending) pending = requestAnimationFrame(paint);
    };
    const move = (e: PointerEvent) => {
      const r = cv.getBoundingClientRect();
      hover.current = { x: e.clientX - r.left, y: e.clientY - r.top };
      request();
    };
    const leave = () => {
      hover.current = null;
      request();
    };
    const ro = new ResizeObserver(request);
    ro.observe(cv);
    cv.addEventListener('pointermove', move);
    cv.addEventListener('pointerleave', leave);
    request();
    return () => {
      cancelAnimationFrame(pending);
      ro.disconnect();
      cv.removeEventListener('pointermove', move);
      cv.removeEventListener('pointerleave', leave);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps -- the caller lists what the drawing depends on
  }, deps);

  return { canvas, readout };
}

/** Tick values for a linear axis: about `count` round numbers covering [lo, hi]. */
export function niceTicks(lo: number, hi: number, count = 5): number[] {
  const span = hi - lo;
  if (!(span > 0)) return [lo];
  const raw = span / count;
  const mag = 10 ** Math.floor(Math.log10(raw));
  const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => span / s <= count + 1)!;
  const out: number[] = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + step * 1e-9; v += step)
    out.push(Math.abs(v) < step * 1e-9 ? 0 : v);
  return out;
}

/** A marker with a ring of the surface colour, so overlapping marks stay apart. */
export function dot(ctx: CanvasRenderingContext2D, x: number, y: number, color: string, r = 4) {
  ctx.beginPath();
  ctx.arc(x, y, r + 1.5, 0, 2 * Math.PI);
  ctx.fillStyle = INK.surface;
  ctx.fill();
  ctx.beginPath();
  ctx.arc(x, y, r, 0, 2 * Math.PI);
  ctx.fillStyle = color;
  ctx.fill();
}

export const FONT = '10px ui-monospace, SFMono-Regular, Menlo, monospace';
