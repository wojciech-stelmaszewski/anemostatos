import { useFrame } from '@react-three/fiber';
import { useMemo } from 'react';
import * as THREE from 'three';
import { isPlanar, profileOffset } from '@/engine/reference';
import { sim, useUi } from '@/store/sim';
import { SIGNAL } from '@/ui/colors';

const N = 240;

/** The planned path of a planar trajectory profile (one lap), drawn as a dashed line. */
export function ReferencePath() {
  const show = useUi((s) => s.overlays.setpoint);
  const line = useMemo(() => {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(N * 3), 3));
    const l = new THREE.Line(
      g,
      new THREE.LineDashedMaterial({
        color: SIGNAL.setpoint,
        dashSize: 0.12,
        gapSize: 0.08,
        transparent: true,
        opacity: 0.7,
      }),
    );
    l.frustumCulled = false;
    return l;
  }, []);
  const state = useMemo(() => ({ key: '' }), []);

  useFrame(() => {
    const p = sim.params;
    const sp = p.setpoint;
    const planar = isPlanar(sp.profile) && sim.level > 1;
    line.visible = planar;
    if (!planar) return;
    const b = sim.profileBase;
    const key = [sp.profile, sp.profileAmplitude, sp.profilePeriod, b.x, b.y, b.z].join('|');
    if (key === state.key) return;
    state.key = key;
    const pos = line.geometry.getAttribute('position') as THREE.BufferAttribute;
    for (let k = 0; k < N; k++) {
      const t = (k / (N - 1)) * sp.profilePeriod;
      const r = profileOffset(p, sim.level, t, 0);
      pos.setXYZ(k, b.x + r.pos.x, Math.max(b.y + r.pos.y, 0.2), b.z + r.pos.z);
    }
    pos.needsUpdate = true;
    line.computeLineDistances();
  });

  if (!show) return null;
  return <primitive object={line} />;
}
