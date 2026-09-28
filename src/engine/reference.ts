import { evalSpline, minSnap, type Spline } from '@/math/poly';
import { v3, type Vec3 } from '@/math/vec3';
import type { Level, Params } from '@/sim/params';

/** A reference and its time derivatives (world frame). */
export interface Reference {
  pos: Vec3;
  vel: Vec3;
  acc: Vec3;
  jerk: Vec3;
  snap: Vec3;
}

export const zeroReference = (pos: Vec3): Reference => ({
  pos,
  vel: v3(),
  acc: v3(),
  jerk: v3(),
  snap: v3(),
});

/** Profiles that move in the horizontal plane (L2/L3). */
export const PLANAR = ['circle', 'figure8', 'corners', 'minsnap'] as const;
export const isPlanar = (profile: string): boolean =>
  (PLANAR as readonly string[]).includes(profile);

/** Corners of the square the waypoint profiles visit (x, z offsets), starting and ending at 0. */
const corners = (a: number): [number, number][] => [
  [0, 0],
  [2 * a, 0],
  [2 * a, 2 * a],
  [0, 2 * a],
  [0, 0],
];

// One rest-to-rest minimum-snap segment per edge: the drone stops at every corner (like the steps
// version, but on a planned, smooth schedule) and the edges stay straight.
let splineCache: { key: string; x: Spline[]; z: Spline[] } | null = null;
const cornerSplines = (a: number, period: number) => {
  const key = `${a}|${period}`;
  if (splineCache?.key !== key) {
    const c = corners(a);
    const seg = (i: number, k: 0 | 1) => minSnap([c[i]![k], c[i + 1]![k]], [period / 4]);
    splineCache = {
      key,
      x: [0, 1, 2, 3].map((i) => seg(i, 0)),
      z: [0, 1, 2, 3].map((i) => seg(i, 1)),
    };
  }
  return splineCache;
};

/**
 * The automatic setpoint motion added to the base setpoint (docs/beyond-pid.md §3.6), with its
 * derivatives. `t` is simulation time; the planar profiles start their first lap at `start`, so
 * they begin at the base point instead of jumping into the middle of a lap.
 */
export function profileOffset(p: Params, level: Level, t: number, start: number): Reference {
  const sp = p.setpoint;
  const out = zeroReference(v3());
  if (sp.profile === 'none') return out;
  const period = Math.max(sp.profilePeriod, 0.1);
  const a = sp.profileAmplitude;
  const w = (2 * Math.PI) / period;

  if (!isPlanar(sp.profile) || level === 1) {
    // Part I profiles along one axis — kept numerically identical to Part I.
    const phase = (t / period) % 1;
    const axis = level === 1 ? 'y' : sp.profileAxis;
    const profile = isPlanar(sp.profile) ? 'sine' : sp.profile;
    if (profile === 'square') out.pos[axis] = phase < 0.5 ? a : -a;
    else if (profile === 'sine') {
      const th = 2 * Math.PI * phase;
      out.pos[axis] = a * Math.sin(th);
      out.vel[axis] = a * w * Math.cos(th);
      out.acc[axis] = -a * w * w * Math.sin(th);
      out.jerk[axis] = -a * w ** 3 * Math.cos(th);
      out.snap[axis] = a * w ** 4 * Math.sin(th);
    } else {
      out.pos[axis] = a * (phase < 0.5 ? 4 * phase - 1 : 3 - 4 * phase);
      out.vel[axis] = (phase < 0.5 ? 4 : -4) * (a / period);
    }
    return out;
  }

  const tl = Math.max(t - start, 0);
  if (sp.profile === 'circle') {
    // Circle of radius a through the base point, centre at base − (a, 0, 0).
    const th = w * tl;
    const [c, s] = [Math.cos(th), Math.sin(th)];
    out.pos = v3(a * (c - 1), 0, a * s);
    out.vel = v3(-a * w * s, 0, a * w * c);
    out.acc = v3(-a * w * w * c, 0, -a * w * w * s);
    out.jerk = v3(a * w ** 3 * s, 0, -a * w ** 3 * c);
    out.snap = v3(a * w ** 4 * c, 0, a * w ** 4 * s);
  } else if (sp.profile === 'figure8') {
    // Lissajous figure-8: x = a·sin(ωt), z = (a/2)·sin(2ωt).
    const th = w * tl;
    const f = (amp: number, k: number, d: number) => {
      const phase = k * th + (d * Math.PI) / 2;
      return amp * (k * w) ** d * Math.sin(phase);
    };
    const at = (d: number) => v3(f(a, 1, d), 0, f(a / 2, 2, d));
    [out.pos, out.vel, out.acc, out.jerk, out.snap] = [at(0), at(1), at(2), at(3), at(4)];
  } else {
    const lap = tl % period;
    if (sp.profile === 'corners') {
      // Steps: at the start of each quarter, jump to the corner that quarter should end at.
      const c = corners(a)[Math.min(4, Math.floor((4 * lap) / period) + 1)]!;
      out.pos = v3(c[0], 0, c[1]);
    } else {
      const { x, z } = cornerSplines(a, period);
      const i = Math.min(3, Math.floor((4 * lap) / period));
      const local = lap - (i * period) / 4;
      const dx = evalSpline(x[i]!, local);
      const dz = evalSpline(z[i]!, local);
      [out.pos, out.vel, out.acc, out.jerk, out.snap] = [0, 1, 2, 3, 4].map((d) =>
        v3(dx[d]!, 0, dz[d]!),
      ) as [Vec3, Vec3, Vec3, Vec3, Vec3];
    }
  }
  return out;
}

/**
 * What a planar trajectory asks of the drone over one lap: peak speed, peak horizontal
 * acceleration and the tilt needed for it (atan(a/g)) — known before flying it.
 */
export function profileDemand(p: Params): { speed: number; acc: number; tiltDeg: number } {
  let speed = 0;
  let acc = 0;
  const period = Math.max(p.setpoint.profilePeriod, 0.1);
  for (let k = 0; k <= 400; k++) {
    const r = profileOffset(p, 3, (k / 400) * period, 0);
    speed = Math.max(speed, Math.hypot(r.vel.x, r.vel.y, r.vel.z));
    acc = Math.max(acc, Math.hypot(r.acc.x, r.acc.z));
  }
  return { speed, acc, tiltDeg: (Math.atan2(acc, 9.81) * 180) / Math.PI };
}

/** Speed of a planar profile at its fastest, m/s (for readouts). */
export function profilePeakSpeed(p: Params): number {
  const sp = p.setpoint;
  const w = (2 * Math.PI) / Math.max(sp.profilePeriod, 0.1);
  const a = sp.profileAmplitude;
  if (sp.profile === 'circle') return a * w;
  if (sp.profile === 'figure8') return a * w * Math.sqrt(2); // at the crossing: (aω, aω)
  return NaN;
}
