import type { Level } from '@/sim/params';

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
