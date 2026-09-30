import { useEffect, useState } from 'react';
import { ghostRate, motorJitter, spectrumOf, type Spectrum } from '@/analysis/spectrum';
import { useParams } from '@/store/params';
import { sim } from '@/store/sim';
import { SIGNAL } from '@/ui/colors';
import { Select } from '@/ui/components/select';
import { ChartCard } from './ChartCard';
import { FONT, INK, useChartCanvas } from './canvas';
import { useRateLoop } from './mimoStore';

const LEFT = 44;
const RIGHT = 10;
const TOP = 6;
const BOTTOM = 18;
const F_MIN = 2;
const F_MAX = 500;
/** Seconds of signal behind each spectrum. */
const WINDOW = 4;

type View = 'gyro' | 'motor';
const VIEWS: { value: View; label: string }[] = [
  { value: 'gyro', label: 'roll rate' },
  { value: 'motor', label: 'motor command' },
];
/** Truth is neutral, the sensor has the colour of every measurement, the filter's output its own. */
const COLOR = { truth: SIGNAL.setpoint, sensor: SIGNAL.measurement, used: SIGNAL.p } as const;

interface Data {
  truth: Spectrum;
  sensor: Spectrum;
  used: Spectrum;
  motor: Spectrum;
  ghost: number;
  jitter: number;
}

const read = (): Data => {
  const n = WINDOW * 1000;
  return {
    truth: spectrumOf(sim.tap.last('gyro.true', n)),
    sensor: spectrumOf(sim.tap.last('gyro.sensor', n)),
    used: spectrumOf(sim.tap.last('gyro.used', n)),
    motor: spectrumOf(sim.tap.last('motor.cmd', n)),
    ghost: ghostRate(sim),
    jitter: motorJitter(sim),
  };
};

/**
 * Spectrum of the roll rate three ways (what the body does, what the gyro reports, what the rate
 * loop is given after its filters) or of one motor command, from the last seconds at 1 kHz.
 */
export function SpectrumChart() {
  const [view, setView] = useState<View>('gyro');
  const [data, setData] = useState<Data>(read);
  const level = useParams((s) => s.params.sim.level);
  const hoverHz = useParams((s) => s.params.vibration.hoverHz);
  const shaking = useParams((s) => s.params.vibration.gyroDeg > 0 || s.params.vibration.acc > 0);
  const imuHz = useParams((s) => s.params.sensors.imuRateHz);
  const filtered = useParams(
    (s) => s.params.control.gyroFilter.lpfHz > 0 || s.params.control.gyroFilter.notch !== 'off',
  );
  const { data: loop } = useRateLoop();

  useEffect(() => {
    const id = setInterval(() => setData(read()), 400);
    return () => clearInterval(id);
  }, []);

  const { canvas, readout } = useChartCanvas(
    ({ ctx, w, h, hover }) => {
      ctx.font = FONT;
      if (level !== 3) {
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText('The spectrum view needs the quadrotor (level 3).', w / 2, h / 3);
        return;
      }
      const x0 = LEFT;
      const x1 = w - RIGHT;
      const y0 = TOP;
      const y1 = h - BOTTOM;
      const series: { s: Spectrum; color: string; name: string }[] =
        view === 'motor'
          ? [{ s: data.motor, color: COLOR.sensor, name: 'command' }]
          : [
              { s: data.truth, color: COLOR.truth, name: 'true' },
              { s: data.sensor, color: COLOR.sensor, name: 'gyro' },
              ...(filtered ? [{ s: data.used, color: COLOR.used, name: 'filtered' }] : []),
            ];
      // Decades of amplitude: five of them, ending just above the largest line.
      let top = -Infinity;
      for (const { s } of series)
        s.fHz.forEach((f, i) => {
          if (f >= F_MIN) top = Math.max(top, s.amp[i]!);
        });
      const hi = Number.isFinite(top) && top > 0 ? Math.ceil(Math.log10(top) + 0.1) : 1;
      const lo = hi - 5;
      const X = (f: number) => x0 + (Math.log10(f / F_MIN) / Math.log10(F_MAX / F_MIN)) * (x1 - x0);
      const Y = (a: number) => {
        const v = Math.min(Math.max(Math.log10(Math.max(a, 1e-30)), lo), hi);
        return y0 + ((hi - v) / (hi - lo)) * (y1 - y0);
      };
      ctx.lineWidth = 1;
      ctx.textBaseline = 'middle';
      for (let d = lo; d <= hi; d++) {
        ctx.strokeStyle = INK.grid;
        ctx.beginPath();
        ctx.moveTo(x0, Y(10 ** d));
        ctx.lineTo(x1, Y(10 ** d));
        ctx.stroke();
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'right';
        ctx.fillText(d >= 0 ? `${10 ** d}` : `${(10 ** d).toFixed(-d)}`, x0 - 5, Y(10 ** d));
      }
      ctx.textBaseline = 'top';
      for (const f of [2, 5, 10, 20, 50, 100, 200, 500]) {
        ctx.strokeStyle = INK.grid;
        ctx.beginPath();
        ctx.moveTo(X(f), y0);
        ctx.lineTo(X(f), y1);
        ctx.stroke();
        if (X(f) > x1 - 22) continue; // room for the unit
        ctx.fillStyle = INK.muted;
        ctx.textAlign = 'center';
        ctx.fillText(`${f}`, X(f), y1 + 4);
      }
      ctx.fillStyle = INK.muted;
      ctx.textAlign = 'right';
      ctx.fillText('Hz', x1, y1 + 4);
      ctx.save();
      ctx.translate(10, (y0 + y1) / 2);
      ctx.rotate(-Math.PI / 2);
      ctx.textAlign = 'center';
      ctx.fillText(view === 'motor' ? 'N' : '°/s', 0, -4);
      ctx.restore();

      // Where the rotors turn at hover, and the highest frequency the IMU's samples can carry.
      const mark = (f: number, text: string, row: number) => {
        if (f < F_MIN || f > F_MAX) return;
        ctx.strokeStyle = INK.axis;
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        ctx.moveTo(X(f), y0);
        ctx.lineTo(X(f), y1);
        ctx.stroke();
        ctx.setLineDash([]);
        const tw = ctx.measureText(text).width;
        const tx = Math.min(X(f) + 4, x1 - tw - 2);
        ctx.fillStyle = 'rgba(11,14,19,0.85)';
        ctx.fillRect(tx - 2, y0 + 1 + row * 13, tw + 4, 13);
        ctx.fillStyle = INK.text;
        ctx.textAlign = 'left';
        ctx.textBaseline = 'top';
        ctx.fillText(text, tx, y0 + 2 + row * 13);
      };
      if (shaking) mark(hoverHz, `rotors at hover ${hoverHz} Hz`, 0);
      if (imuHz > 0 && imuHz < 1000) mark(imuHz / 2, `IMU Nyquist ${imuHz / 2} Hz`, 1);

      ctx.save();
      ctx.beginPath();
      ctx.rect(x0, y0, x1 - x0, y1 - y0);
      ctx.clip();
      ctx.lineJoin = 'round';
      for (const { s, color } of series) {
        ctx.strokeStyle = color;
        ctx.lineWidth = color === COLOR.truth ? 1.5 : 2;
        ctx.beginPath();
        let started = false;
        s.fHz.forEach((f, i) => {
          if (f < F_MIN) return;
          if (started) ctx.lineTo(X(f), Y(s.amp[i]!));
          else ctx.moveTo(X(f), Y(s.amp[i]!));
          started = true;
        });
        ctx.stroke();
      }
      ctx.restore();

      if (!hover || hover.x < x0 || hover.x > x1) return;
      const hz = F_MIN * (F_MAX / F_MIN) ** ((hover.x - x0) / (x1 - x0));
      ctx.strokeStyle = INK.axis;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(hover.x, y0);
      ctx.lineTo(hover.x, y1);
      ctx.stroke();
      // The strongest line within a few pixels of the pointer: peaks are narrow.
      const span = (F_MAX / F_MIN) ** (4 / (x1 - x0));
      const parts = [`${hz.toPrecision(3)} Hz`];
      for (const { s, name } of series) {
        let best = 0;
        s.fHz.forEach((f, i) => {
          if (f >= hz / span && f <= hz * span) best = Math.max(best, s.amp[i]!);
        });
        parts.push(`${name} ${best.toPrecision(2)}`);
      }
      return parts.join('   ');
    },
    [data, view, level, hoverHz, imuHz, shaking, filtered],
  );

  return (
    <ChartCard
      title={view === 'motor' ? 'Spectrum: motor command' : 'Spectrum: roll rate'}
      readout={readout}
      legend={
        view === 'motor'
          ? [{ label: 'motor 1', color: COLOR.sensor, mark: 'line' }]
          : [
              { label: 'true', color: COLOR.truth, mark: 'line' },
              { label: 'gyro', color: COLOR.sensor, mark: 'line' },
              ...(filtered
                ? [{ label: 'filtered', color: COLOR.used, mark: 'line' as const }]
                : []),
            ]
      }
      summary={
        <span>
          ghost below 60 Hz {data.ghost.toFixed(2)} °/s · motor jitter above 60 Hz{' '}
          {data.jitter.toFixed(4)} N
          {loop ? ` · PM ${loop.stable ? loop.both.pmDeg.toFixed(0) + '°' : 'none: unstable'}` : ''}
        </span>
      }
      controls={<Select value={view} onValueChange={(v) => setView(v as View)} options={VIEWS} />}
    >
      <canvas ref={canvas} className="min-h-0 w-full flex-1" />
    </ChartCard>
  );
}
