import { useEffect, useState } from 'react';
import { ParticleFilter } from '@/estimation/particle';
import { BEACONS } from '@/sim/sensors';
import { sim } from '@/store/sim';
import { SIGNAL } from '@/ui/colors';
import { ChartCard } from './ChartCard';
import { dot, FONT, INK, niceTicks, useChartCanvas } from './canvas';

/** At most this many particles are drawn (every k-th one). */
const DRAWN = 600;

interface Cloud {
  px: number[];
  pz: number[];
  mx: number;
  mz: number;
  tx: number;
  tz: number;
  sigma: number;
  ess: number;
}

const read = (): Cloud | null => {
  const f = sim.sensors.rangeFilter;
  if (!(f instanceof ParticleFilter) || !sim.params.sensors.beacons) return null;
  const every = Math.max(1, Math.floor(f.n / DRAWN));
  const px: number[] = [];
  const pz: number[] = [];
  for (let i = 0; i < f.n; i += every) {
    px.push(f.px[i]!);
    pz.push(f.pz[i]!);
  }
  return {
    px,
    pz,
    mx: f.x[0]!,
    mz: f.x[1]!,
    tx: sim.state.pos.x,
    tz: sim.state.pos.z,
    sigma: f.sigmaPos,
    ess: f.ess,
  };
};

const COLOR = { particle: SIGNAL.measurement, drone: SIGNAL.output, mean: SIGNAL.error } as const;

/**
 * The particle cloud of lesson IV.28 from above: every particle a guess of where the drone is, the
 * beacon, the truth and the cloud's mean. One range leaves a ring around the beacon; a straight leg
 * leaves two arcs, mirror images; a turn leaves one.
 */
export function ParticleCloudChart() {
  const [c, setC] = useState<Cloud | null>(read);
  useEffect(() => {
    const id = setInterval(() => setC(read()), 200);
    return () => clearInterval(id);
  }, []);

  const { canvas, readout } = useChartCanvas(
    ({ ctx, w, h }) => {
      ctx.font = FONT;
      if (!c) {
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText(
          'Switch on Sensors → Range-only beacon with the particle filter (level 3).',
          w / 2,
          h / 2,
        );
        return;
      }
      const b = BEACONS[0]!;
      // A fixed frame around the beacon: wide enough for the whole ring and its mirror image.
      const span = 26;
      const m = 30;
      const s = Math.min((w - 2 * m) / (2 * span), (h - 2 * m) / (2 * span));
      const cx = b.x;
      const cz = b.z;
      const x0 = cx - (w - 2 * m) / s / 2;
      const z0 = cz - (h - 2 * m) / s / 2;
      const X = (x: number) => m + (x - x0) * s;
      const Y = (z: number) => m + (z - z0) * s;
      ctx.lineWidth = 1;
      ctx.textBaseline = 'middle';
      for (const v of niceTicks(x0, x0 + (w - 2 * m) / s, 6)) {
        ctx.strokeStyle = INK.grid;
        ctx.beginPath();
        ctx.moveTo(X(v), m);
        ctx.lineTo(X(v), h - m);
        ctx.stroke();
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText(`${v}`, X(v), h - m + 10);
      }
      for (const v of niceTicks(z0, z0 + (h - 2 * m) / s, 5)) {
        ctx.strokeStyle = INK.grid;
        ctx.beginPath();
        ctx.moveTo(m, Y(v));
        ctx.lineTo(w - m, Y(v));
        ctx.stroke();
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'right';
        ctx.fillText(`${v}`, m - 4, Y(v));
      }
      ctx.textAlign = 'right';
      ctx.fillText('x, m', w - m, h - m + 22);
      // The circle the last range allows, through the drone.
      ctx.strokeStyle = INK.axis;
      ctx.setLineDash([4, 3]);
      ctx.beginPath();
      ctx.arc(X(b.x), Y(b.z), Math.hypot(c.tx - b.x, c.tz - b.z) * s, 0, 2 * Math.PI);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = COLOR.particle;
      ctx.globalAlpha = 0.55;
      c.px.forEach((x, i) => ctx.fillRect(X(x) - 1, Y(c.pz[i]!) - 1, 2, 2));
      ctx.globalAlpha = 1;
      // The beacon: a square.
      ctx.fillStyle = INK.text;
      ctx.fillRect(X(b.x) - 4, Y(b.z) - 4, 8, 8);
      dot(ctx, X(c.mx), Y(c.mz), COLOR.mean, 4);
      dot(ctx, X(c.tx), Y(c.tz), COLOR.drone, 4);
      return `estimate off by ${Math.hypot(c.mx - c.tx, c.mz - c.tz).toFixed(2)} m   cloud σ ${c.sigma.toFixed(2)} m   effective particles ${c.ess.toFixed(0)}`;
    },
    [c],
  );

  return (
    <ChartCard
      title="The particle cloud, seen from above"
      readout={readout}
      legend={[
        { label: 'particles', color: COLOR.particle, mark: 'dot' },
        { label: 'drone', color: COLOR.drone, mark: 'dot' },
        { label: 'mean of the cloud', color: COLOR.mean, mark: 'dot' },
      ]}
      summary={c ? <span>beacon: the white square</span> : null}
    >
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </ChartCard>
  );
}
