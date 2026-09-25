import { useFrame } from '@react-three/fiber';
import { useMemo } from 'react';
import * as THREE from 'three';
import { sim, useUi } from '@/store/sim';
import { SCENE, SIGNAL } from '@/ui/colors';

const N = 400;

/** Fading path of the last few seconds. */
export function Trail() {
  const show = useUi((s) => s.overlays.trail);
  const { line, positions, colors } = useMemo(() => {
    const positions = new Float32Array(N * 3);
    const colors = new Float32Array(N * 3);
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    g.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    const line = new THREE.Line(
      g,
      new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.8 }),
    );
    line.frustumCulled = false;
    return { line, positions, colors };
  }, []);
  const state = useMemo(() => ({ count: 0, lastT: -1 }), []);

  useFrame(() => {
    if (sim.t < state.lastT) state.count = 0; // reset happened
    if (sim.t - state.lastT < 0.015) return;
    state.lastT = sim.t;
    const p = sim.state.pos;
    positions.copyWithin(3, 0, (N - 1) * 3);
    positions[0] = p.x;
    positions[1] = p.y;
    positions[2] = p.z;
    state.count = Math.min(N, state.count + 1);
    const head = new THREE.Color(SIGNAL.measurement);
    const tail = new THREE.Color(SCENE.background);
    const c = new THREE.Color();
    for (let k = 0; k < state.count; k++) {
      c.copy(head).lerp(tail, k / N);
      colors[k * 3] = c.r;
      colors[k * 3 + 1] = c.g;
      colors[k * 3 + 2] = c.b;
    }
    line.geometry.setDrawRange(0, state.count);
    line.geometry.attributes.position!.needsUpdate = true;
    line.geometry.attributes.color!.needsUpdate = true;
  });

  if (!show) return null;
  return <primitive object={line} />;
}
