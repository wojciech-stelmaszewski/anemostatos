import { useShallow } from 'zustand/react/shallow';
import { getIn } from '@/engine/schema';
import { GRAVITY } from '@/sim/params';
import { useParams } from '@/store/params';
import { SIGNAL } from '@/ui/colors';
import { StepMetrics } from './StepMetrics';
import type { LoopMeta } from './loops';

const gainPath = (id: string, level: number): string | null => {
  if (id === 'alt' || (level === 2 && id === 'pos.y')) return 'control.alt';
  if (level === 2) return 'control.posH';
  if (id === 'vel.y') return 'control.l3.velV';
  if (id.startsWith('vel.')) return 'control.l3.velH';
  if (id === 'rate.yaw') return 'control.l3.rateYaw';
  if (id.startsWith('rate.')) return 'control.l3.rateRP';
  return null;
};

function Row({ label, value, hint }: { label: string; value: string; hint?: string }) {
  return (
    <div className="flex items-baseline justify-between gap-2" title={hint}>
      <span className="text-muted">{label}</span>
      <span className="font-mono text-fg">{value}</span>
    </div>
  );
}

/** Theory next to practice: what the gains predict, for the position-like loops. */
export function InfoCard({ meta }: { meta: LoopMeta }) {
  const level = useParams((s) => s.params.sim.level);
  const path = gainPath(meta.id, level);
  const v = useParams(
    useShallow((s) => ({
      kp: path ? (getIn(s.params, `${path}.kp`) as number) : 0,
      ki: path ? (getIn(s.params, `${path}.ki`) as number) : 0,
      kd: path ? (getIn(s.params, `${path}.kd`) as number) : 0,
      pOn: path ? (getIn(s.params, `${path}.pOn`) as boolean) : false,
      iOn: path ? (getIn(s.params, `${path}.iOn`) as boolean) : false,
      dOn: path ? (getIn(s.params, `${path}.dOn`) as boolean) : false,
      m: s.params.drone.mass,
      mHat: s.params.control.massEstimate,
      ff: s.params.control.feedforward,
    })),
  );
  const massLoop = level < 3 && path !== null; // force-output loops on a point mass: m·ÿ = u
  const kp = v.pOn ? v.kp : 0;
  const kd = v.dOn ? v.kd : 0;
  const wn = Math.sqrt(kp / v.m);
  const zeta = kd / (2 * Math.sqrt(kp * v.m));
  const vertical = meta.id === 'alt' || meta.id === 'pos.y';
  const load = vertical ? (v.m - (v.ff ? v.mHat : 0)) * GRAVITY : 0;
  const ess = !v.iOn && kp > 0 ? load / kp : 0;
  const character =
    zeta < 0.05
      ? 'undamped — oscillates'
      : zeta < 0.95
        ? 'underdamped — overshoots'
        : zeta <= 1.05
          ? 'critically damped'
          : 'overdamped — sluggish';

  return (
    <div className="flex h-full min-h-0 flex-col gap-1.5 overflow-auto rounded-md border border-border bg-bg/60 p-2 text-[11px]">
      <div className="font-semibold text-fg">
        {meta.name} loop{' '}
        <span className="font-normal text-muted">
          · output = {meta.outName} [{meta.outUnit}]
        </span>
      </div>
      {path && (
        <div className="grid grid-cols-3 gap-x-3 font-mono">
          <span style={{ color: SIGNAL.p }}>Kp {v.kp}</span>
          <span style={{ color: SIGNAL.i }}>Ki {v.ki}</span>
          <span style={{ color: SIGNAL.d }}>Kd {v.kd}</span>
        </div>
      )}
      {massLoop && kp > 0 && (
        <div className="space-y-0.5 rounded border border-border/60 p-1.5">
          <div className="text-muted">Theory (mass–spring–damper, PD part)</div>
          <Row
            label="ωₙ = √(Kp/m)"
            value={`${wn.toFixed(2)} rad/s`}
            hint="Natural frequency: how fast the loop wants to move."
          />
          <Row
            label="ζ = Kd / 2√(Kp·m)"
            value={`${zeta.toFixed(2)}`}
            hint="Damping ratio: <1 overshoots, 1 critical, >1 sluggish."
          />
          <Row label="character" value={character} />
          {vertical && (
            <Row
              label="predicted e_ss"
              value={v.iOn ? '0 (I term on)' : `${ess.toFixed(3)} m`}
              hint="Steady-state error without integral: the unbalanced weight divided by Kp."
            />
          )}
          {v.ki > 0 && v.kp > 0 && (
            <Row
              label="Tᵢ = Kp/Ki · T_d = Kd/Kp"
              value={`${(v.kp / v.ki).toFixed(2)} s · ${(v.kd / v.kp).toFixed(2)} s`}
              hint="The same gains in the 'standard' (industrial) form."
            />
          )}
        </div>
      )}
      {level === 3 && !path && (
        <div className="text-muted">
          A proportional loop in the cascade: its output is the setpoint of the next, faster loop.
        </div>
      )}
      <StepMetrics loopId={meta.id} unit={meta.unit} />
    </div>
  );
}
