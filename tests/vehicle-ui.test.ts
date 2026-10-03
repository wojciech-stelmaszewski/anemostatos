import { describe, expect, it } from 'vitest';
import { Simulation } from '@/engine/simulation';
import { SCHEMA, visible } from '@/engine/schema';
import { LESSONS } from '@/lessons/lessons';
import { defaultParams, type Params } from '@/sim/params';
import type { VehicleId } from '@/sim/vehicles/types';
import { endNotice } from '@/ui/hud/endNotice';

/** The groups each Part IV vehicle owns. */
const OWN: Record<Exclude<VehicleId, 'quadrotor'>, string[]> = {
  rocket: ['rocket'],
  tvc: ['tvc-vehicle', 'tvc-control'],
  aircraft: ['aircraft'],
  satellite: ['satellite'],
  lander: ['lander'],
  chaser: ['rendezvous'],
};
/** Groups that belong to the drone only. */
const DRONE = ['controller', 'setpoint', 'alt', 'kalman', 'uncertainty', 'probe', 'sensors'];

const params = (vehicle: VehicleId): Params => {
  const p = defaultParams();
  p.sim.level = 1;
  p.sim.vehicle = vehicle;
  return p;
};
const shown = (p: Params) => SCHEMA.filter((g) => visible(g, p)).map((g) => g.id);

describe('the side panel follows the vehicle', () => {
  it('the drone sees its own groups and none of the vehicles', () => {
    const ids = shown(params('quadrotor'));
    for (const id of ['controller', 'setpoint', 'alt', 'physics']) expect(ids).toContain(id);
    for (const own of Object.values(OWN)) for (const id of own) expect(ids).not.toContain(id);
  });

  for (const [vehicle, own] of Object.entries(OWN)) {
    it(`${vehicle}: its own groups and the vehicle selector, no drone groups`, () => {
      const p = params(vehicle as VehicleId);
      const ids = shown(p);
      for (const id of own) expect(ids).toContain(id);
      for (const id of DRONE) expect(ids).not.toContain(id);
      const physics = SCHEMA.find((g) => g.id === 'physics')!;
      const fields = physics.fields.filter((f) => visible(f, p)).map((f) => f.path);
      expect(fields).toEqual(['sim.vehicle']);
    });
  }

  it('the pitch-plane rocket keeps the wind; the others do not', () => {
    expect(shown(params('tvc'))).toContain('wind');
    expect(shown(params('aircraft'))).not.toContain('wind');
  });

  it('every Part IV vehicle lesson opens on a panel with its own group', () => {
    for (const l of LESSONS) {
      const p = defaultParams();
      p.sim.level = l.level;
      l.setup?.(p);
      const v = p.sim.vehicle ?? 'quadrotor';
      if (v === 'quadrotor') continue;
      const ids = shown(p);
      expect(
        ids.some((id) => OWN[v].includes(id)),
        l.id,
      ).toBe(true);
    }
  });
});

describe('the end-of-flight notice speaks the vehicle', () => {
  it('a drone crash, a rocket touchdown, a broken-up rocket', () => {
    const drone = new Simulation(params('quadrotor'));
    expect(endNotice(drone)).toBeNull();
    drone.state.crashed = true;
    expect(endNotice(drone)![0]).toMatch(/Hit the ground/);

    const rocket = new Simulation(params('rocket'));
    rocket.rocket!.landed = true;
    rocket.rocket!.touchdown = -0.42;
    expect(endNotice(rocket)![0]).toBe('Landed at 0.42 m/s.');

    const tvc = new Simulation(params('tvc'));
    tvc.state.crashed = true;
    expect(endNotice(tvc)![0]).toMatch(/past 30°/);
  });
});
