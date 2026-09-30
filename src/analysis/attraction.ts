// Regions of attraction of the L1 hover (docs/analysis.md §4, Chapter G): what a Lyapunov
// function guarantees, what physics allows, and what the simulator actually does.
//
// Coordinates: x = y − r (height above the setpoint) and v (vertical speed, up positive).
import { Simulation } from '@/engine/simulation';
import { v3 } from '@/math/vec3';
import { FOOT_HEIGHT } from '@/sim/drone';
import { GRAVITY, type Params } from '@/sim/params';

/** The room the drone has: height of the setpoint above the floor, and the hardest it can brake. */
export function brakingLimit(p: Params): { h: number; aMax: number } {
  return {
    h: p.setpoint.y - FOOT_HEIGHT,
    aMax: (4 * p.drone.maxMotorThrust) / p.drone.mass - GRAVITY,
  };
}

/**
 * The largest level set of V = ½·Kp·x² + ½·m·v² inside which a PD controller with exact
 * feedforward never saturates and the drone never reaches the floor. Inside it V̇ = −Kd·v² ≤ 0
 * holds, so every state in it is proven to converge. Returns the level c and the semi-axes.
 */
export function lyapunovEstimate(p: Params): { c: number; ax: number; av: number } {
  const g = p.control.alt;
  const m = p.drone.mass;
  const kp = Math.max(g.kp, 1e-9);
  const { h, aMax } = brakingLimit(p);
  // On V = c the command Kp·x + Kd·v reaches at most √(2c·(Kp + Kd²/m)).
  const reach = kp + (g.kd * g.kd) / m;
  const up = m * aMax; // thrust still available above hover
  const down = m * GRAVITY; // and below it
  const cSat = Math.min(up, down) ** 2 / (2 * reach);
  const cFloor = 0.5 * kp * h * h;
  const c = Math.max(0, Math.min(cSat, cFloor));
  return { c, ax: Math.sqrt((2 * c) / kp), av: Math.sqrt((2 * c) / m) };
}

/** A simulation that begins in flight at height r + x0 with vertical speed v0, in calm air. */
export function startAt(p: Params, x0: number, v0: number): Simulation {
  const q = structuredClone(p);
  q.wind.enabled = false;
  q.probe = { ...q.probe, point: 'none' };
  q.setpoint = { ...q.setpoint, profile: 'none', rateLimit: 0 };
  const sim = new Simulation(q);
  const hover = Math.min((q.drone.mass * GRAVITY) / 4, q.drone.maxMotorThrust);
  sim.state.pos = v3(0, q.setpoint.y + x0, 0);
  sim.state.vel = v3(0, v0, 0);
  sim.state.rotors = [hover, hover, hover, hover];
  sim.state.motors = [hover, hover, hover, hover];
  sim.state.landed = false;
  sim.takingOff = false;
  sim.setpoint = v3(0, q.setpoint.y, 0);
  return sim;
}

/** Fly from (x0, v0): true if the drone touches the floor before it settles. */
export function touchesFloor(p: Params, x0: number, v0: number, seconds = 6): boolean {
  const sim = startAt(p, x0, v0);
  const r = sim.params.setpoint.y;
  for (let k = 0; k < seconds * 1000; k++) {
    sim.step();
    if (sim.state.landed || sim.state.crashed) return true;
    if (k > 200 && Math.abs(sim.state.pos.y - r) < 0.01 && Math.abs(sim.state.vel.y) < 0.01)
      return false;
  }
  return false;
}

/** The fastest downward speed at height r + x0 from which the drone still stays off the floor. */
export function criticalDiveSpeed(p: Params, x0 = 0): number {
  if (touchesFloor(p, x0, 0)) return 0;
  let lo = 0;
  let hi = 4;
  while (!touchesFloor(p, x0, -hi) && hi < 60) hi *= 2;
  for (let i = 0; i < 12; i++) {
    const mid = 0.5 * (lo + hi);
    if (touchesFloor(p, x0, -mid)) hi = mid;
    else lo = mid;
  }
  return lo;
}

/**
 * The boundary of the region of attraction on its dangerous side, flown by the simulator:
 * for heights from just above the floor to `xMax` above the setpoint, the critical dive speed.
 */
export function attractionBoundary(p: Params, xMax: number, n = 9): { x: number; v: number }[] {
  const { h } = brakingLimit(p);
  const out: { x: number; v: number }[] = [];
  for (let i = 0; i < n; i++) {
    const x = -0.85 * h + ((xMax + 0.85 * h) * i) / (n - 1);
    out.push({ x, v: -criticalDiveSpeed(p, x) });
  }
  return out;
}
