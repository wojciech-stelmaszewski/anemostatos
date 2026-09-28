import { useFrame } from '@react-three/fiber';
import { useMemo } from 'react';
import * as THREE from 'three';
import { sim, useUi } from '@/store/sim';
import { SIGNAL } from '@/ui/colors';

const MAX_SAMPLES = 64;
const MAX_POINTS = 60;

/**
 * What a predictive controller intends: the MPC's planned path, or MPPI's cloud of sampled futures
 * (green = cheap, red = expensive) with the chosen plan on top.
 */
export function Plans() {
  const show = useUi((s) => s.overlays.setpoint);
  const { best, cloud, bestPos, cloudPos, cloudCol } = useMemo(() => {
    const bestPos = new Float32Array(MAX_POINTS * 3);
    const bg = new THREE.BufferGeometry();
    bg.setAttribute('position', new THREE.BufferAttribute(bestPos, 3));
    const best = new THREE.Line(
      bg,
      new THREE.LineBasicMaterial({ color: SIGNAL.output, transparent: true, opacity: 0.95 }),
    );
    best.frustumCulled = false;
    // Line segments: each sample path as (points − 1) segments.
    const n = MAX_SAMPLES * (MAX_POINTS - 1) * 2;
    const cloudPos = new Float32Array(n * 3);
    const cloudCol = new Float32Array(n * 3);
    const cg = new THREE.BufferGeometry();
    cg.setAttribute('position', new THREE.BufferAttribute(cloudPos, 3));
    cg.setAttribute('color', new THREE.BufferAttribute(cloudCol, 3));
    const cloud = new THREE.LineSegments(
      cg,
      new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.35 }),
    );
    cloud.frustumCulled = false;
    return { best, cloud, bestPos, cloudPos, cloudCol };
  }, []);
  const state = useMemo(() => ({ t0: -1 }), []);
  const cheap = useMemo(() => new THREE.Color(SIGNAL.i), []);
  const dear = useMemo(() => new THREE.Color(SIGNAL.error), []);

  useFrame(() => {
    const plan = sim.controller.plan?.() ?? null;
    best.visible = !!plan && plan.best.length > 1;
    cloud.visible = !!plan?.samples?.length;
    if (!plan || plan.t0 === state.t0) return;
    state.t0 = plan.t0;
    const pts = plan.best.slice(0, MAX_POINTS);
    pts.forEach((p, k) => bestPos.set([p.x, p.y, p.z], k * 3));
    best.geometry.setDrawRange(0, pts.length);
    best.geometry.attributes.position!.needsUpdate = true;
    if (plan.samples) {
      let v = 0;
      const c = new THREE.Color();
      for (const s of plan.samples.slice(0, MAX_SAMPLES)) {
        c.copy(cheap).lerp(dear, Math.sqrt(s.cost));
        const path = s.path.slice(0, MAX_POINTS);
        for (let k = 0; k + 1 < path.length; k++) {
          for (const p of [path[k]!, path[k + 1]!]) {
            cloudPos.set([p.x, p.y, p.z], v * 3);
            cloudCol.set([c.r, c.g, c.b], v * 3);
            v++;
          }
        }
      }
      cloud.geometry.setDrawRange(0, v);
      cloud.geometry.attributes.position!.needsUpdate = true;
      cloud.geometry.attributes.color!.needsUpdate = true;
    }
  });

  if (!show) return null;
  return (
    <group>
      <primitive object={cloud} />
      <primitive object={best} />
    </group>
  );
}
