import { useFrame } from '@react-three/fiber';
import { useRef } from 'react';
import * as THREE from 'three';
import { RV_HEIGHT } from '@/engine/simulation';
import { sim } from '@/store/sim';

/**
 * Lesson IV.25: the target (a station with panels, still at the origin of its own frame) and the
 * chaser, drawn at a tenth of their distance so the whole approach fits. Along-track is the
 * scene's x (the direction of flight to the right), radial is up. A flame shows the thrusters.
 */
export function RendezvousScene() {
  const chaser = useRef<THREE.Group>(null);
  const flame = useRef<THREE.Mesh>(null);
  useFrame(() => {
    const g = chaser.current;
    if (!g) return;
    const { pos } = sim.renderPose();
    g.position.set(pos.x, pos.y, pos.z);
    const c = sim.chaser;
    const f = flame.current;
    if (f && c) {
      const a = Math.hypot(c.ax, c.ay);
      f.visible = a > 1e-6;
      if (a > 1e-6) {
        // The flame points against the thrust.
        const dir = new THREE.Vector3(-c.ay, -c.ax, 0).normalize();
        f.position.copy(dir.clone().multiplyScalar(0.22));
        f.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
      }
    }
  });
  return (
    <>
      <group position={[0, RV_HEIGHT, 0]}>
        <mesh>
          <cylinderGeometry args={[0.18, 0.18, 0.9, 16]} />
          <meshStandardMaterial color="#c9ccd3" metalness={0.5} roughness={0.35} />
        </mesh>
        {[-1, 1].map((side) => (
          <mesh key={side} position={[0, 0, side * 0.75]}>
            <boxGeometry args={[0.5, 0.02, 1]} />
            <meshStandardMaterial color="#1d3a6b" metalness={0.4} roughness={0.3} />
          </mesh>
        ))}
        {/* The docking port faces the chaser, behind the target (−x). */}
        <mesh position={[-0.12, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
          <cylinderGeometry args={[0.07, 0.07, 0.1, 12]} />
          <meshStandardMaterial color="#d95926" />
        </mesh>
      </group>
      <group ref={chaser}>
        <mesh>
          <boxGeometry args={[0.24, 0.18, 0.18]} />
          <meshStandardMaterial color="#c9a227" metalness={0.6} roughness={0.35} />
        </mesh>
        <mesh ref={flame} visible={false}>
          <coneGeometry args={[0.05, 0.18, 10]} />
          <meshBasicMaterial color="#ffb347" />
        </mesh>
      </group>
    </>
  );
}
