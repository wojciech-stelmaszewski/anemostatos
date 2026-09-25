import { describe, expect, it } from 'vitest';
import { Rng } from '@/math/prng';
import {
  qFromAxisAngle,
  qFromEuler,
  qFromTo,
  qIdentity,
  qIntegrate,
  qRotate,
  qToEuler,
} from '@/math/quat';
import { cross, length, v3, type Vec3 } from '@/math/vec3';

const expectVec = (a: Vec3, b: Vec3, digits = 9) => {
  expect(a.x).toBeCloseTo(b.x, digits);
  expect(a.y).toBeCloseTo(b.y, digits);
  expect(a.z).toBeCloseTo(b.z, digits);
};

describe('vec3', () => {
  it('cross product is right-handed', () => {
    expectVec(cross(v3(1, 0, 0), v3(0, 1, 0)), v3(0, 0, 1));
  });
});

describe('quat', () => {
  it('rotates +x by 90° about +y onto -z (right-hand rule)', () => {
    const q = qFromAxisAngle(v3(0, 1, 0), Math.PI / 2);
    expectVec(qRotate(q, v3(1, 0, 0)), v3(0, 0, -1));
  });

  it('integrating a constant rate matches the axis-angle rotation', () => {
    let q = qIdentity();
    const omega = v3(0.3, -1.2, 0.7);
    for (let i = 0; i < 1000; i++) q = qIntegrate(q, omega, 0.001);
    const expected = qFromAxisAngle(omega, length(omega));
    expectVec(qRotate(q, v3(1, 2, 3)), qRotate(expected, v3(1, 2, 3)), 6);
  });

  it('qFromTo maps a onto b', () => {
    const a = v3(0, 1, 0);
    const b = v3(0.6, 0.8, 0);
    expectVec(qRotate(qFromTo(a, b), a), b);
    expectVec(qRotate(qFromTo(a, v3(0, -1, 0)), a), v3(0, -1, 0));
  });

  it('euler round-trip', () => {
    const e = qToEuler(qFromEuler(0.2, -0.4, 1.1));
    expect(e.roll).toBeCloseTo(0.2, 9);
    expect(e.pitch).toBeCloseTo(-0.4, 9);
    expect(e.yaw).toBeCloseTo(1.1, 9);
  });

  it('positive pitch raises the nose, positive roll lowers the right side', () => {
    expect(qRotate(qFromEuler(0, 0.3, 0), v3(1, 0, 0)).y).toBeGreaterThan(0);
    expect(qRotate(qFromEuler(0.3, 0, 0), v3(0, 0, 1)).y).toBeLessThan(0);
  });
});

describe('Rng', () => {
  it('is deterministic for a given seed', () => {
    const a = new Rng(42);
    const b = new Rng(42);
    for (let i = 0; i < 100; i++) expect(a.next()).toBe(b.next());
  });

  it('normal() has zero mean and unit variance', () => {
    const r = new Rng(7);
    const n = 200_000;
    let sum = 0;
    let sq = 0;
    for (let i = 0; i < n; i++) {
      const x = r.normal();
      sum += x;
      sq += x * x;
    }
    expect(sum / n).toBeCloseTo(0, 2);
    expect(sq / n).toBeCloseTo(1, 1);
  });
});
