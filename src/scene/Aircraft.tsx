import { useFrame } from '@react-three/fiber';
import { useRef } from 'react';
import * as THREE from 'three';
import { sim } from '@/store/sim';
import { SCENE } from '@/ui/colors';

const BODY = '#d9dde4';
const DARK = '#3a3f48';

/**
 * A procedural small jet of Chapter M, about 13 m long with a 15 m span: fuselage, wing,
 * tailplane with its elevator (which moves with the real deflection), fin and two engines.
 * Its origin is the centre of mass; +x is the nose, +y up, so the pitch is a rotation about z.
 */
export function AircraftRig() {
  const group = useRef<THREE.Group>(null);
  const elevator = useRef<THREE.Group>(null);
  useFrame(() => {
    const g = group.current;
    if (!g) return;
    const { pos, q } = sim.renderPose();
    g.position.set(pos.x, pos.y, pos.z);
    g.quaternion.set(q.x, q.y, q.z, q.w);
    const a = sim.aircraft;
    // Trailing edge down (positive δe) is a negative rotation about z at the tail.
    if (elevator.current && a) elevator.current.rotation.z = -a.de;
  });
  return (
    <group ref={group}>
      <mesh rotation={[0, 0, -Math.PI / 2]} castShadow>
        <cylinderGeometry args={[0.75, 0.55, 12, 20]} />
        <meshStandardMaterial color={BODY} metalness={0.3} roughness={0.45} />
      </mesh>
      <mesh position={[6.6, 0, 0]} rotation={[0, 0, -Math.PI / 2]} castShadow>
        <coneGeometry args={[0.75, 1.4, 20]} />
        <meshStandardMaterial color={BODY} metalness={0.3} roughness={0.45} />
      </mesh>
      <mesh position={[0.4, -0.35, 0]} castShadow>
        <boxGeometry args={[2.0, 0.18, 15]} />
        <meshStandardMaterial color={BODY} metalness={0.2} roughness={0.5} />
      </mesh>
      <mesh position={[-5.4, 0.9, 0]} castShadow>
        <boxGeometry args={[1.6, 2.4, 0.15]} />
        <meshStandardMaterial color={SCENE.accent} metalness={0.2} roughness={0.5} />
      </mesh>
      <mesh position={[-5.4, 0.2, 0]} castShadow>
        <boxGeometry args={[1.0, 0.12, 5.6]} />
        <meshStandardMaterial color={BODY} metalness={0.2} roughness={0.5} />
      </mesh>
      <group ref={elevator} position={[-5.9, 0.2, 0]}>
        <mesh position={[-0.35, 0, 0]} castShadow>
          <boxGeometry args={[0.7, 0.08, 5.6]} />
          <meshStandardMaterial color={SCENE.accent} metalness={0.2} roughness={0.5} />
        </mesh>
      </group>
      {[-1.4, 1.4].map((z) => (
        <mesh key={z} position={[-3.2, 0.5, z]} rotation={[0, 0, -Math.PI / 2]} castShadow>
          <cylinderGeometry args={[0.38, 0.38, 2.2, 16]} />
          <meshStandardMaterial color={DARK} metalness={0.6} roughness={0.4} />
        </mesh>
      ))}
    </group>
  );
}
