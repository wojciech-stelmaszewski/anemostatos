import { OrbitControls } from '@react-three/drei';
import { useFrame, useThree } from '@react-three/fiber';
import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import type { OrbitControls as OrbitControlsImpl } from 'three-stdlib';
import { sim, useUi } from '@/store/sim';

const PRESETS = {
  side: (p: THREE.Vector3) => new THREE.Vector3(p.x, p.y + 0.3, p.z + 4.5),
  top: (p: THREE.Vector3) => new THREE.Vector3(p.x + 0.01, p.y + 7, p.z),
} as const;

/** Orbit controls plus follow / side / top modes. */
export function CameraRig() {
  const mode = useUi((s) => s.camera);
  const controls = useRef<OrbitControlsImpl>(null);
  const { camera } = useThree();
  const drone = useRef(new THREE.Vector3());
  const lastDrone = useRef(new THREE.Vector3(0, 2, 0));

  useEffect(() => {
    const c = controls.current;
    if (!c) return;
    const p = new THREE.Vector3(sim.state.pos.x, Math.max(sim.state.pos.y, 1), sim.state.pos.z);
    if (mode === 'side' || mode === 'top') {
      camera.position.copy(PRESETS[mode](p));
      c.target.copy(p);
      c.update();
    }
  }, [mode, camera]);

  useFrame(() => {
    const c = controls.current;
    if (!c) return;
    const { pos } = sim.renderPose();
    drone.current.set(pos.x, pos.y, pos.z);
    if (mode !== 'orbit') {
      // Follow: move camera and target together with the drone (smoothed).
      const target = drone.current.clone();
      if (mode !== 'top') target.y = Math.max(target.y, 0.6);
      const delta = target.clone().sub(c.target).multiplyScalar(0.08);
      c.target.add(delta);
      camera.position.add(delta);
    }
    lastDrone.current.copy(drone.current);
    c.update();
  });

  return (
    <OrbitControls
      ref={controls}
      makeDefault
      target={[0, 1.85, 0]}
      enableDamping
      dampingFactor={0.08}
      minDistance={0.4}
      maxDistance={60}
      maxPolarAngle={Math.PI * 0.495}
    />
  );
}
