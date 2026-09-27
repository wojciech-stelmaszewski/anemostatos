import { useEffect, useState } from 'react';
import type { LoopPart } from '@/control/types';
import type { Level } from '@/sim/params';
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

export const loopMeta = (level: Level, id: string): LoopMeta =>
  LOOPS[level].find((l) => l.id === id) ?? LOOPS[level][0]!;

const partsSignature = (parts: readonly LoopPart[]) =>
  parts.map((p) => `${p.key}:${p.label}:${p.like ?? ''}:${p.ff ? 1 : 0}`).join('|');

/**
 * The contributions the running controller reports for a loop. Polled, because the list changes
 * when the controller is swapped or reconfigured (e.g. LQI adds an integral part) — React only
 * re-renders when the list's shape changes, never for the values.
 */
export function useLoopParts(id: string): LoopPart[] {
  const read = () => sim.controller.loops()[id]?.parts ?? [];
  const [parts, setParts] = useState(read);
  useEffect(() => {
    let sig = '';
    const poll = () => {
      const next = sim.controller.loops()[id]?.parts ?? [];
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
