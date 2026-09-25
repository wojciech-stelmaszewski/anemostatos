import { v3 } from '@/math/vec3';
import { sim } from '@/store/sim';

export function gustNow(): void {
  const vertical = sim.params.wind.gustVertical;
  const heading = Math.random() * Math.PI * 2;
  const vy =
    sim.level === 1 ? (Math.random() < 0.5 ? -1 : 1) : vertical * (Math.random() < 0.5 ? -1 : 1);
  const hs = Math.sqrt(Math.max(0, 1 - vy * vy));
  sim.triggerGust(
    v3(Math.cos(heading) * hs, vy, Math.sin(heading) * hs),
    sim.params.wind.gustAmpMax,
    1.5,
  );
}

export function downloadCsv(): void {
  const blob = new Blob([sim.telemetry.toCsv()], { type: 'text/csv' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `anemostatos-L${sim.level}-seed${sim.params.sim.seed}-t${sim.t.toFixed(0)}.csv`;
  a.click();
  URL.revokeObjectURL(a.href);
}
