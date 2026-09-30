import { useEffect, useRef } from 'react';
import type { Complex } from '@/math/complex';
import { Button } from '@/ui/components/button';
import { ChartCard } from './ChartCard';
import { FONT, INK, niceTicks, SERIES, useChartCanvas } from './canvas';
import { useLoopData } from './store';

/** Poles further left than this are fast enough not to matter and would flatten the picture. */
const FAR_LEFT = -15;
const TRAIL = 80;

/**
 * Closed-loop poles in the s-plane. The vertical axis is the stability boundary. As a gain is
 * dragged the earlier positions stay as a fading trail: a root locus drawn by hand.
 */
export function PoleMap() {
  const data = useLoopData();
  const trail = useRef<Complex[][]>([]);
  const last = useRef('');
  if (last.current !== data.key) {
    last.current = data.key;
    if (data.poles.length) trail.current = [...trail.current, data.poles].slice(-TRAIL);
  }
  const clear = useRef(() => {});

  const shown = data.poles.filter((z) => z.re >= FAR_LEFT);
  const hidden = data.poles.length - shown.length;

  const { canvas, readout } = useChartCanvas(
    ({ ctx, w, h, hover }) => {
      const pts = trail.current.flat().filter((z) => z.re >= FAR_LEFT);
      const reLo = Math.min(-6, ...pts.map((z) => z.re)) * 1.12;
      const reHi = Math.max(1.5, ...pts.map((z) => z.re * 1.3), -reLo * 0.12);
      const imMax = Math.max(4, ...pts.map((z) => Math.abs(z.im))) * 1.18;
      const L = 8;
      const B = 16;
      const X = (re: number) => L + ((re - reLo) / (reHi - reLo)) * (w - 2 * L);
      const Y = (im: number) => (h - B) / 2 - (im / imMax) * ((h - B) / 2 - 4);
      ctx.font = FONT;

      // The unstable half-plane, washed; the label says what the wash means.
      ctx.fillStyle = 'rgba(230,103,103,0.08)';
      ctx.fillRect(X(0), 0, w - X(0), h - B);
      ctx.fillStyle = INK.muted;
      ctx.textAlign = 'left';
      ctx.textBaseline = 'top';
      ctx.fillText('unstable', X(0) + 4, 3);
      ctx.textAlign = 'right';
      ctx.fillText('stable', X(0) - 4, 3);

      ctx.lineWidth = 1;
      ctx.textBaseline = 'middle';
      for (const v of niceTicks(reLo, reHi, 6)) {
        ctx.strokeStyle = v === 0 ? INK.axis : INK.grid;
        ctx.beginPath();
        ctx.moveTo(X(v), 0);
        ctx.lineTo(X(v), h - B);
        ctx.stroke();
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText(`${v}`, X(v), h - B + 8);
      }
      ctx.strokeStyle = INK.grid;
      ctx.beginPath();
      ctx.moveTo(L, Y(0));
      ctx.lineTo(w - L, Y(0));
      ctx.stroke();
      ctx.fillStyle = INK.muted;
      ctx.textAlign = 'left';
      ctx.fillText(`Im ±${imMax.toPrecision(2)} rad/s`, L, Y(imMax) + 14);

      // Lines of constant damping ratio ζ = 0.7: poles between them do not ring.
      ctx.setLineDash([2, 4]);
      ctx.strokeStyle = INK.axis;
      const slope = Math.tan(Math.acos(0.7));
      ctx.beginPath();
      ctx.moveTo(X(0), Y(0));
      ctx.lineTo(X(reLo), Y(-reLo * slope));
      ctx.moveTo(X(0), Y(0));
      ctx.lineTo(X(reLo), Y(reLo * slope));
      ctx.stroke();
      ctx.setLineDash([]);
      const yz = Math.max(Y(-reLo * slope * 0.45), 12);
      ctx.fillStyle = INK.muted;
      ctx.fillText('ζ = 0.7', X(reLo * 0.45) + 4, yz);

      // The trail: where the poles were, oldest faintest.
      const sets = trail.current;
      sets.slice(0, -1).forEach((set, k) => {
        ctx.fillStyle = SERIES.model;
        ctx.globalAlpha = 0.25 + 0.5 * (k / Math.max(sets.length - 1, 1));
        for (const z of set) {
          if (z.re < FAR_LEFT) continue;
          ctx.beginPath();
          ctx.arc(X(z.re), Y(z.im), 2, 0, 2 * Math.PI);
          ctx.fill();
        }
      });
      ctx.globalAlpha = 1;

      // The poles now.
      const cross = (z: Complex, r: number, color: string, width: number) => {
        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        ctx.lineCap = 'round';
        ctx.beginPath();
        ctx.moveTo(X(z.re) - r, Y(z.im) - r);
        ctx.lineTo(X(z.re) + r, Y(z.im) + r);
        ctx.moveTo(X(z.re) - r, Y(z.im) + r);
        ctx.lineTo(X(z.re) + r, Y(z.im) - r);
        ctx.stroke();
      };
      for (const z of shown) {
        cross(z, 5, INK.surface, 5);
        cross(z, 4.5, SERIES.model, 2);
      }
      if (!shown.length) {
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText('No linear model for this controller.', w / 2, h * 0.25);
        return;
      }
      if (!hover) return;
      let best = shown[0]!;
      let d = Infinity;
      for (const z of shown) {
        const e = Math.hypot(X(z.re) - hover.x, Y(z.im) - hover.y);
        if (e < d) {
          d = e;
          best = z;
        }
      }
      cross(best, 6.5, INK.text, 2);
      const wn = Math.hypot(best.re, best.im);
      const zeta = wn > 0 ? -best.re / wn : 1;
      const im = Math.abs(best.im) > 1e-9 ? ` ± ${Math.abs(best.im).toFixed(2)}j` : '';
      const kind =
        best.re >= 0
          ? `grows, doubling every ${(Math.LN2 / Math.max(best.re, 1e-9)).toFixed(1)} s`
          : `decays with τ = ${(-1 / best.re).toFixed(2)} s`;
      return `s = ${best.re.toFixed(3)}${im}   ωn ${wn.toFixed(2)} rad/s   ζ ${zeta.toFixed(2)}   ${kind}`;
    },
    [data],
  );

  useEffect(() => {
    clear.current = () => {
      trail.current = data.poles.length ? [data.poles] : [];
      // Repaint through a resize-independent nudge: the draw reads the trail from the ref.
      canvas.current?.dispatchEvent(new PointerEvent('pointerleave'));
    };
  }, [data, canvas]);

  return (
    <ChartCard
      title="Closed-loop poles (s-plane, 1/s)"
      readout={readout}
      legend={[{ label: 'pole', color: SERIES.model, mark: 'cross' }]}
      summary={
        data.model ? (
          <span>
            {data.stable ? '● stable' : '▲ unstable'}: {data.poles.length} poles
            {hidden ? `, ${hidden} faster than ${-FAR_LEFT} 1/s not shown` : ''}
          </span>
        ) : null
      }
      controls={
        <Button
          size="sm"
          variant="ghost"
          className="h-5 px-2 text-[11px]"
          onClick={() => clear.current()}
          title="Forget the trail of earlier pole positions."
        >
          clear trail
        </Button>
      }
    >
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </ChartCard>
  );
}
