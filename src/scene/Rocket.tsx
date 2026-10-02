import { useFrame } from '@react-three/fiber';
import { useRef } from 'react';
import * as THREE from 'three';
import { modeShape } from '@/sim/vehicles/tvc';
import { sim } from '@/store/sim';
import { SCENE } from '@/ui/colors';

/** Height of the procedural rocket, m: the vehicle of lesson IV.4 is a small lander. */
const BODY = 1.6;
const RADIUS = 0.18;

/**
 * A procedural vertical lander: body, nose, three legs and a flame that scales with the thrust.
 * Its origin is the bottom of the legs, which is where `rocket.h` is measured.
 */
export function RocketRig() {
  const group = useRef<THREE.Group>(null);
  const flame = useRef<THREE.Mesh>(null);
  useFrame(() => {
    const g = group.current;
    if (!g) return;
    const { pos } = sim.renderPose();
    g.position.set(0, pos.y, 0);
    const r = sim.rocket;
    const f = flame.current;
    if (f && r) {
      const k = r.thrust / Math.max(sim.params.rocket.thrustMax, 1);
      f.visible = k > 0.01;
      f.scale.set(0.6 + 0.4 * k, 0.3 + 1.4 * k, 0.6 + 0.4 * k);
    }
  });
  const legs = [0, 1, 2].map((i) => (i * 2 * Math.PI) / 3);
  return (
    <group ref={group}>
      <mesh position={[0, 0.35 + BODY / 2, 0]} castShadow>
        <cylinderGeometry args={[RADIUS, RADIUS, BODY, 24]} />
        <meshStandardMaterial color="#d9dde4" metalness={0.3} roughness={0.45} />
      </mesh>
      <mesh position={[0, 0.35 + BODY + 0.22, 0]} castShadow>
        <coneGeometry args={[RADIUS, 0.45, 24]} />
        <meshStandardMaterial color={SCENE.accent} metalness={0.2} roughness={0.5} />
      </mesh>
      <mesh position={[0, 0.27, 0]}>
        <cylinderGeometry args={[RADIUS * 0.55, RADIUS * 0.8, 0.16, 16]} />
        <meshStandardMaterial color="#3a3f48" metalness={0.6} roughness={0.4} />
      </mesh>
      {legs.map((a) => (
        <mesh
          key={a}
          position={[Math.cos(a) * 0.26, 0.2, Math.sin(a) * 0.26]}
          rotation={[Math.sin(a) * 0.45, 0, -Math.cos(a) * 0.45]}
          castShadow
        >
          <cylinderGeometry args={[0.02, 0.02, 0.45, 8]} />
          <meshStandardMaterial color="#8b94a5" />
        </mesh>
      ))}
      <mesh ref={flame} position={[0, 0.0, 0]} rotation={[Math.PI, 0, 0]}>
        <coneGeometry args={[RADIUS * 0.6, 0.9, 16, 1, true]} />
        <meshBasicMaterial color="#ffb347" transparent opacity={0.8} />
      </mesh>
    </group>
  );
}

/** The landing pad under the rocket. */
export function LandingPad() {
  return (
    <group>
      <mesh rotation-x={-Math.PI / 2} position={[0, 0.005, 0]} receiveShadow>
        <circleGeometry args={[2.2, 48]} />
        <meshStandardMaterial color="#2b3240" roughness={0.9} />
      </mesh>
      <mesh rotation-x={-Math.PI / 2} position={[0, 0.008, 0]}>
        <ringGeometry args={[1.6, 1.75, 48]} />
        <meshBasicMaterial color={SCENE.accent} />
      </mesh>
    </group>
  );
}

/** Segments of the pitch-plane rocket's body, so the bending mode can be drawn. */
const TVC_SEGMENTS = 12;
/** Bending is millimetres; drawn this many times larger. */
const BEND_DRAWN = 40;

/**
 * The pitch-plane rocket of Chapter L at its true size: a slender body pitched by θ, its first
 * bending mode drawn larger than life, and the flame turned by the gimbal angle. Its origin is
 * the centre of mass, which is where the simulator puts `state.pos`.
 */
export function TvcRocketRig() {
  const group = useRef<THREE.Group>(null);
  const flame = useRef<THREE.Group>(null);
  const segments = useRef<(THREE.Mesh | null)[]>([]);
  const r = sim.params.tvc;
  const radius = r.length / 22;
  const seg = r.length / TVC_SEGMENTS;
  useFrame(() => {
    const g = group.current;
    const s = sim.tvc;
    if (!g || !s) return;
    const { pos, q } = sim.renderPose();
    g.position.set(pos.x, pos.y, pos.z);
    g.quaternion.set(q.x, q.y, q.z, q.w);
    segments.current.forEach((m, i) => {
      if (!m) return;
      const off = s.eta * modeShape((i + 0.5) / TVC_SEGMENTS) * BEND_DRAWN;
      m.position.x = Math.max(-1, Math.min(1, off));
    });
    const f = flame.current;
    if (f) {
      f.rotation.z = -s.delta;
      f.position.x = Math.max(-1, Math.min(1, s.eta * BEND_DRAWN));
    }
  });
  const tail = -sim.params.tvc.gimbalArm;
  return (
    <group ref={group}>
      {Array.from({ length: TVC_SEGMENTS }, (_, i) => (
        <mesh
          key={i}
          ref={(m) => {
            segments.current[i] = m;
          }}
          position={[0, tail + seg * (i + 0.5), 0]}
          castShadow
        >
          <cylinderGeometry args={[radius, radius, seg * 1.02, 24]} />
          <meshStandardMaterial
            color={i === TVC_SEGMENTS - 1 ? SCENE.accent : '#d9dde4'}
            metalness={0.3}
            roughness={0.45}
          />
        </mesh>
      ))}
      <mesh position={[0, tail + r.length + radius * 1.5, 0]} castShadow>
        <coneGeometry args={[radius, radius * 3, 24]} />
        <meshStandardMaterial color={SCENE.accent} metalness={0.2} roughness={0.5} />
      </mesh>
      <group ref={flame} position={[0, tail, 0]}>
        <mesh position={[0, -radius * 0.4, 0]}>
          <cylinderGeometry args={[radius * 0.45, radius * 0.7, radius * 0.8, 16]} />
          <meshStandardMaterial color="#3a3f48" metalness={0.6} roughness={0.4} />
        </mesh>
        <mesh position={[0, -radius * 0.8 - 2.2, 0]} rotation={[Math.PI, 0, 0]}>
          <coneGeometry args={[radius * 0.7, 4.4, 16, 1, true]} />
          <meshBasicMaterial color="#ffb347" transparent opacity={0.8} />
        </mesh>
      </group>
    </group>
  );
}
