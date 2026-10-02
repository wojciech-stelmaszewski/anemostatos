import { clamp } from '@/math/util';
import { idleActuation, type Actuation } from '@/sim/dynamics';
import { GRAVITY, type Params } from '@/sim/params';
import {
  periodSteps,
  type ControlInput,
  type Controller,
  type InfoRow,
  type LoopTerms,
} from './types';

/**
 * Time-optimal altitude change (docs/aerospace-gnc.md §4, Chapter K, lesson IV.2).
 *
 * With the thrust bounded and the time as the cost, the Hamiltonian is linear in the thrust, so
 * the best thrust always sits on a bound: full thrust one way, then the other. For a climb the
 * drone accelerates at full thrust and coasts up with the motors off (gravity and drag brake it);
 * for a descent it falls and then brakes at full thrust. The switch happens on the curve of states
 * from which the brake, started now, stops the drone exactly at the target.
 *
 * Braking with deceleration a + k·v² (k = drag / mass) from speed v covers
 *   D(v) = ln(1 + k·v²/a) / (2k)          (→ v²/(2a) without drag)
 * The motors lag: a switch commanded now takes about τ to act, so the curve is moved by the
 * distance flown meanwhile, v·t_lead. Near the target, where bang-bang would chatter, a linear
 * capture law takes over.
 */

/** Distance to stop from speed |v| with constant deceleration a and quadratic drag k = c/m. */
export function brakingDistance(v: number, a: number, k: number): number {
  const v2 = v * v;
  if (!(a > 0)) return Infinity;
  return k > 1e-9 ? Math.log(1 + (k * v2) / a) / (2 * k) : v2 / (2 * a);
}

export interface BangBangLimits {
  /** Deceleration available when braking a climb (motors off): g, m/s². */
  aUp: number;
  /** Deceleration available when braking a descent (full thrust): (T_max − m·g)/m, m/s². */
  aDown: number;
  /** Drag over mass, 1/m. */
  k: number;
  tMax: number;
}

export function bangBangLimits(p: Params): BangBangLimits {
  const m = Math.max(p.control.model.mass, 0.05);
  const tMax = 4 * p.drone.maxMotorThrust;
  return {
    aUp: GRAVITY,
    aDown: tMax / m - GRAVITY,
    k: p.control.bangbang.drag ? p.drone.dragV / m : 0,
    tMax,
  };
}

/**
 * The switching function: positive means "push up", negative "push down". d = r − y is the
 * distance still to climb, v the vertical speed.
 */
export function switching(d: number, v: number, l: BangBangLimits, lead: number): number {
  if (v >= 0) return d - brakingDistance(v, l.aUp, l.k) - v * lead;
  return d + brakingDistance(-v, l.aDown, l.k) - v * lead;
}

/** The fastest rest-to-rest change of altitude d for a double integrator with these limits, s. */
export function minimumTime(d: number, aPush: number, aBrake: number): number {
  return Math.sqrt((2 * Math.abs(d) * (aPush + aBrake)) / (aPush * aBrake));
}

export class BangBangController implements Controller {
  private held: Actuation = idleActuation();
  private last: LoopTerms = {
    setpoint: 0,
    measurement: 0,
    error: 0,
    parts: [],
    unsaturated: 0,
    output: 0,
    saturated: false,
  };
  /** The switching function at the last sample, m; and whether the capture law was flying. */
  sigma = 0;
  capture = false;

  reset(): void {
    this.held = idleActuation();
    this.sigma = 0;
    this.capture = false;
  }

  resetIntegrators(): void {}

  tick(input: ControlInput): Actuation {
    const { params: p, physDt, stepIndex } = input;
    const steps = periodSteps(p.control.rateHz, physDt);
    if (stepIndex % steps !== 0) return this.held;
    const c = p.control.bangbang;
    const m = input.sense();
    const mHat = Math.max(p.control.model.mass, 0.05);
    const l = bangBangLimits(p);
    const sp = input.setpoint.pos.y;
    const d = sp - m.pos.y;
    const v = m.vel.y;
    const s = switching(d, v, l, c.lead);
    this.sigma = s;
    const ff = mHat * GRAVITY;
    // Near the target the bang-bang law would chatter: a linear capture law finishes the job.
    this.capture = Math.abs(d) < c.band && Math.abs(v) < c.band * c.wn * 2;
    let unsaturated: number;
    let parts: LoopTerms['parts'];
    if (this.capture) {
      const pd = mHat * (c.wn * c.wn * d - 2 * c.wn * v);
      unsaturated = ff + pd;
      parts = [
        { key: 'pd', label: 'capture PD', value: pd, like: 'p' },
        { key: 'ff', label: 'm̂·g', value: ff, ff: true, like: 'ff' },
      ];
    } else {
      unsaturated = s > 0 ? l.tMax : 0;
      parts = [{ key: 'bang', label: s > 0 ? 'full thrust' : 'motors off', value: unsaturated - ff, like: 'p' }];
      parts.push({ key: 'ff', label: 'm̂·g', value: ff, ff: true, like: 'ff' });
    }
    const output = clamp(unsaturated, 0, l.tMax);
    this.last = {
      setpoint: sp,
      measurement: m.pos.y,
      error: d,
      parts,
      unsaturated,
      output,
      saturated: !this.capture,
    };
    const f = output / 4;
    this.held = { motorCmd: [f, f, f, f], forceCmd: this.held.forceCmd };
    return this.held;
  }

  loops() {
    return { alt: this.last };
  }

  extras() {
    return { 'bang.sigma': this.sigma, 'bang.capture': this.capture ? 1 : 0 };
  }

  describe(p: Params): InfoRow[] {
    const l = bangBangLimits(p);
    return [
      {
        label: 'push up · brake a climb',
        value: `${(l.tMax / p.control.model.mass - GRAVITY).toFixed(1)} · ${l.aUp.toFixed(1)} m/s²`,
        hint: 'Full thrust accelerates the climb; motors off, gravity alone brakes it.',
      },
      {
        label: 'switching curve',
        value: p.control.bangbang.drag ? 'with drag: ln(1 + k·v²/a)/(2k)' : 'v²/(2a)',
        hint: 'The brake must start where the stopping distance equals the distance left.',
      },
      {
        label: 'lead for the motor lag',
        value: `${(p.control.bangbang.lead * 1000).toFixed(0)} ms`,
        hint: 'A switch takes effect only as fast as the motors spin up or down.',
      },
    ];
  }
}
