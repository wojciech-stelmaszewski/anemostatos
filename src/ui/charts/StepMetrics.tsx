import { useEffect, useState } from 'react';
import { analyzeLastStep, type StepMetrics as M } from '@/engine/metrics';
import { sim } from '@/store/sim';

const f = (v: number | null, digits = 2, unit = '') =>
  v === null ? '…' : `${v.toFixed(digits)}${unit}`;

/** Rise time, overshoot, settling time and steady-state error of the last setpoint step. */
/** Only position-like loops have setpoints the user steps; inner loops' setpoints move continuously. */
const isStepLoop = (id: string) => id === 'alt' || id.startsWith('pos.');

export function StepMetrics({ loopId, unit }: { loopId: string; unit: string }) {
  const [m, setM] = useState<M | null>(null);
  const applies = isStepLoop(loopId);
  useEffect(() => {
    if (!applies) return;
    const tick = () => {
      const { t, series } = sim.telemetry.window([`${loopId}.sp`, `${loopId}.meas`], -Infinity);
      setM(analyzeLastStep(t, series[0]!, series[1]!));
    };
    tick();
    const id = setInterval(tick, 250);
    return () => clearInterval(id);
  }, [loopId, applies]);

  if (!applies)
    return (
      <div className="rounded border border-border/60 p-1.5 text-muted">
        Step-response metrics apply to the position loops, where you set the setpoint. Here the
        setpoint comes from the outer loop and moves all the time.
      </div>
    );

  return (
    <div className="space-y-0.5 rounded border border-border/60 p-1.5">
      <div className="text-muted">
        Last setpoint step{' '}
        {m && (
          <span className="font-mono text-fg">
            {m.from.toFixed(2)} → {m.to.toFixed(2)} {unit} at {m.t0.toFixed(1)} s
          </span>
        )}
      </div>
      {m ? (
        <div className="grid grid-cols-2 gap-x-3 font-mono">
          <span className="text-muted">rise 10–90 %</span>
          <span className="text-right text-fg">{f(m.riseTime, 2, ' s')}</span>
          <span className="text-muted">overshoot</span>
          <span className="text-right text-fg">{f(m.overshoot, 1, ' %')}</span>
          <span className="text-muted">settling ±2 %</span>
          <span className="text-right text-fg">{f(m.settlingTime, 2, ' s')}</span>
          <span className="text-muted">steady-state error</span>
          <span className="text-right text-fg">{f(m.steadyStateError, 3, ` ${unit}`)}</span>
        </div>
      ) : (
        <div className="text-muted">
          Change the setpoint (↑ / ↓ or the slider) to measure a step response.
        </div>
      )}
    </div>
  );
}
