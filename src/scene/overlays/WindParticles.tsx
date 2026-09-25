import { useFrame } from '@react-three/fiber';
import { useMemo } from 'react';
import * as THREE from 'three';
import { sim, useUi } from '@/store/sim';
import { SIGNAL } from '@/ui/colors';

const N = 700;
const BOX = { x: 5, y: 3, z: 5 };
const STREAK = 0.09; // seconds of travel drawn as the streak length

/** Air-tracer streaks moving with the wind around the drone; brighter when stronger. */
export function WindParticles() {
  const show = useUi((s) => s.overlays.wind);
  const { lines, pts, seg } = useMemo(() => {
    const pts = new Float32Array(N * 3);
    for (let i = 0; i < N; i++) {
      pts[i * 3] = (Math.random() * 2 - 1) * BOX.x;
      pts[i * 3 + 1] = Math.random() * BOX.y * 2;
      pts[i * 3 + 2] = (Math.random() * 2 - 1) * BOX.z;
    }
    const seg = new Float32Array(N * 6);
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(seg, 3));
    const m = new THREE.LineBasicMaterial({
      color: SIGNAL.wind,
      transparent: true,
      opacity: 0.4,
      depthWrite: false,
    });
    const lines = new THREE.LineSegments(g, m);
    lines.frustumCulled = false;
    return { lines, pts, seg };
  }, []);
  const last = useMemo(() => ({ t: 0 }), []);

  useFrame(() => {
    const dt = Math.max(0, Math.min(0.1, sim.t - last.t));
    last.t = sim.t;
    const w = sim.wind.velocity;
    const c = sim.state.pos;
    const cx = c.x;
    const cz = c.z;
    const cy = Math.max(c.y, BOX.y);
    // All components are drawn, even though in L1 the drone only feels the vertical one.
    const wy = w.y;
    for (let i = 0; i < N; i++) {
      let x = pts[i * 3]! + w.x * dt;
      let y = pts[i * 3 + 1]! + wy * dt;
      let z = pts[i * 3 + 2]! + w.z * dt;
      // Wrap into a box that follows the drone.
      const wrap = (v: number, centre: number, half: number) =>
        v < centre - half ? v + 2 * half : v > centre + half ? v - 2 * half : v;
      x = wrap(x, cx, BOX.x);
      y = wrap(y, cy, BOX.y);
      z = wrap(z, cz, BOX.z);
      pts[i * 3] = x;
      pts[i * 3 + 1] = y;
      pts[i * 3 + 2] = z;
      seg[i * 6] = x;
      seg[i * 6 + 1] = y;
      seg[i * 6 + 2] = z;
      seg[i * 6 + 3] = x - w.x * STREAK;
      seg[i * 6 + 4] = y - wy * STREAK;
      seg[i * 6 + 5] = z - w.z * STREAK;
    }
    const speed = Math.hypot(w.x, w.y, w.z);
    (lines.material as THREE.LineBasicMaterial).opacity = Math.min(0.75, 0.08 + speed * 0.08);
    lines.geometry.attributes.position!.needsUpdate = true;
  });

  if (!show) return null;
  return <primitive object={lines} />;
}
