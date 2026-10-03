import { useEffect, useState } from 'react';
import type { LoopPart } from '@/control/types';
import type { Level, Params } from '@/sim/params';
import { sim } from '@/store/sim';

export interface LoopMeta {
  id: string;
  name: string;
  /** Unit of setpoint / measurement / error. */
  unit: string;
  /** Unit of the loop output (what the PID terms add up to). */
  outUnit: string;
  /** What the output means. */
  outName: string;
  /** Telemetry channel with the true (un-noised) value, if any. */
  truth?: string;
}

const pos = (axis: 'x' | 'y' | 'z', out: string, outUnit: string): LoopMeta => ({
  id: `pos.${axis}`,
  name: `Position ${axis.toUpperCase()}`,
  unit: 'm',
  outUnit,
  outName: out,
  truth: `pos.${axis}`,
});

export const LOOPS: Record<Level, LoopMeta[]> = {
  1: [{ id: 'alt', name: 'Altitude', unit: 'm', outUnit: 'N', outName: 'thrust', truth: 'pos.y' }],
  2: [pos('x', 'force', 'N'), pos('y', 'force', 'N'), pos('z', 'force', 'N')],
  3: [
    pos('x', 'velocity sp', 'm/s'),
    pos('y', 'velocity sp', 'm/s'),
    pos('z', 'velocity sp', 'm/s'),
    {
      id: 'vel.x',
      name: 'Velocity X',
      unit: 'm/s',
      outUnit: 'm/s²',
      outName: 'accel sp',
      truth: 'vel.x',
    },
    {
      id: 'vel.y',
      name: 'Velocity Y',
      unit: 'm/s',
      outUnit: 'm/s²',
      outName: 'accel sp',
      truth: 'vel.y',
    },
    {
      id: 'vel.z',
      name: 'Velocity Z',
      unit: 'm/s',
      outUnit: 'm/s²',
      outName: 'accel sp',
      truth: 'vel.z',
    },
    { id: 'att.roll', name: 'Attitude roll', unit: '°', outUnit: '°/s', outName: 'rate sp' },
    { id: 'att.pitch', name: 'Attitude pitch', unit: '°', outUnit: '°/s', outName: 'rate sp' },
    { id: 'att.yaw', name: 'Attitude yaw', unit: '°', outUnit: '°/s', outName: 'rate sp' },
    { id: 'rate.roll', name: 'Rate roll', unit: '°/s', outUnit: '°/s²', outName: 'ang. accel' },
    { id: 'rate.pitch', name: 'Rate pitch', unit: '°/s', outUnit: '°/s²', outName: 'ang. accel' },
    { id: 'rate.yaw', name: 'Rate yaw', unit: '°/s', outUnit: '°/s²', outName: 'ang. accel' },
  ],
};

const accel = (axis: 'x' | 'y' | 'z'): LoopMeta => ({
  id: `acc.${axis}`,
  name: `Accel ${axis.toUpperCase()} (INDI)`,
  unit: 'm/s²',
  outUnit: 'm/s²',
  outName: 'force / m̂',
});
const ACCEL = [accel('x'), accel('y'), accel('z')];

/** The loops of the pitch-plane rocket (Chapter L): pitch on the gimbal, drift on the pitch. */
const TVC_LOOPS: LoopMeta[] = [
  {
    id: 'pitch',
    name: 'Pitch (TVC)',
    unit: '°',
    outUnit: '°',
    outName: 'gimbal',
    truth: 'tvc.theta',
  },
  { id: 'drift', name: 'Drift', unit: 'm', outUnit: '°', outName: 'pitch sp', truth: 'tvc.x' },
];

/** The satellite of Chapter N: the whole pointing error, and each body axis' angle. */
const satAxis = (axis: 'x' | 'y' | 'z'): LoopMeta => ({
  id: `point.${axis}`,
  name: `Angle about ${axis}`,
  unit: '°',
  outUnit: 'N·m',
  outName: 'torque',
});
const SATELLITE: LoopMeta[] = [
  { id: 'point', name: 'Pointing error', unit: '°', outUnit: 'N·m', outName: 'torque' },
  satAxis('x'),
  satAxis('y'),
  satAxis('z'),
];

/** The chaser of lesson IV.25: each axis of the relative position, the thrust as the output. */
const RENDEZVOUS: LoopMeta[] = [
  { id: 'along', name: 'Along-track', unit: 'm', outUnit: 'mm/s²', outName: 'thrust' },
  { id: 'radial', name: 'Radial', unit: 'm', outUnit: 'mm/s²', outName: 'thrust' },
];

/** Loops to offer in the inspector for these parameters. */
/** The aircraft of Chapter M flies one loop: pitch attitude, through the elevator. */
const PITCH: LoopMeta[] = [
  {
    id: 'pitch',
    name: 'Pitch attitude',
    unit: '°',
    outUnit: '°',
    outName: 'elevator',
    truth: 'air.theta',
  },
];

export const loopsFor = (p: Params): LoopMeta[] => {
  if (p.sim.vehicle === 'tvc') return TVC_LOOPS;
  if (p.sim.vehicle === 'aircraft') return PITCH;
  if (p.sim.vehicle === 'satellite') return SATELLITE;
  if (p.sim.vehicle === 'chaser') return RENDEZVOUS;
  if (p.sim.level !== 3) return LOOPS[p.sim.level];
  const geometric = p.control.l3.outer !== 'pid-cascade'; // geometric, MPC, MPPI
  // The geometric controller closes position and velocity in one law: no separate velocity loop,
  // and its position loops output an acceleration.
  const outer = geometric
    ? LOOPS[3].slice(0, 3).map((l) => ({ ...l, outName: 'accel sp', outUnit: 'm/s²' }))
    : LOOPS[3].slice(0, 6);
  const acc = p.control.l3.compensation === 'indi' ? ACCEL : [];
  return [...outer, ...acc, ...LOOPS[3].slice(6)];
};

/** Metadata of a loop as the selected controller defines it (falls back to the first loop). */
export const loopMeta = (p: Params, id: string): LoopMeta => {
  const list = loopsFor(p);
  return list.find((l) => l.id === id) ?? list[0]!;
};

const partsSignature = (parts: readonly LoopPart[]) =>
  parts.map((p) => `${p.key}:${p.label}:${p.like ?? ''}:${p.ff ? 1 : 0}`).join('|');

/**
 * The contributions the running controller reports for a loop. Polled, because the list changes
 * when the controller is swapped or reconfigured (e.g. LQI adds an integral part) — React only
 * re-renders when the list's shape changes, never for the values.
 */
export function useLoopParts(id: string): LoopPart[] {
  const read = () => sim.loops()[id]?.parts ?? [];
  const [parts, setParts] = useState(read);
  useEffect(() => {
    let sig = '';
    const poll = () => {
      const next = sim.loops()[id]?.parts ?? [];
      const s = partsSignature(next);
      if (s !== sig) {
        sig = s;
        setParts(next);
      }
    };
    poll();
    const t = setInterval(poll, 250);
    return () => clearInterval(t);
  }, [id]);
  return parts;
}

/** True when the loop's parts are the classic P, I, D (+ feedforward). */
export const isPidParts = (parts: readonly LoopPart[]): boolean =>
  parts.map((p) => p.key).join() === 'p,i,d,ff';
