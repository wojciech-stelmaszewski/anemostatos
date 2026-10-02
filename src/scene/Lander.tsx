import { useFrame } from '@react-three/fiber';
import { useMemo, useRef } from 'react';
import * as THREE from 'three';
import { useParams } from '@/store/params';
import { sim } from '@/store/sim';
import { SCENE, SIGNAL } from '@/ui/colors';
import { LandingPad } from './Rocket';

const BODY = 1.6;
const RADIUS = 0.18;
/** Height of the glide-slope cone drawn over the pad, m. */
const CONE_HEIGHT = 150;
const MAX_NODES = 64;

/**
 * The lander of lesson IV.5: the rocket of IV.4, now free to move in three dimensions and tilted
 * along its thrust, with the glide-slope cone it must stay inside and the guidance's current plan.
 */
export function LanderScene() {
  const glide = useParams((s) => s.params.lander.glideSlopeDeg);
  const radius = CONE_HEIGHT / Math.tan((Math.max(glide, 1) * Math.PI) / 180);
  return (
    <>
      <LandingPad />
      <mesh position={[0, CONE_HEIGHT / 2, 0]} rotation={[Math.PI, 0, 0]}>
        <coneGeometry args={[radius, CONE_HEIGHT, 64, 1, true]} />
        <meshBasicMaterial
          color={SCENE.accent}
          transparent
          opacity={0.07}
          side={THREE.DoubleSide}
          depthWrite={false}
        />
      </mesh>
      <PlanLine />
      <LanderBody />
    </>
  );
}

function LanderBody() {
  const group = useRef<THREE.Group>(null);
  const flame = useRef<THREE.Mesh>(null);
  useFrame(() => {
    const g = group.current;
    if (!g) return;
    const { pos, q } = sim.renderPose();
    g.position.set(pos.x, pos.y, pos.z);
    g.quaternion.set(q.x, q.y, q.z, q.w);
    const s = sim.pdgLander;
    const f = flame.current;
    if (f && s) {
      const t = Math.hypot(s.thrust.x, s.thrust.y, s.thrust.z);
      const k = t / Math.max(sim.params.lander.thrustMax, 1);
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
      <mesh ref={flame} position={[0, 0, 0]} rotation={[Math.PI, 0, 0]}>
        <coneGeometry args={[RADIUS * 0.6, 0.9, 16, 1, true]} />
        <meshBasicMaterial color="#ffb347" transparent opacity={0.8} />
      </mesh>
    </group>
  );
}

/** The guidance's current plan, from where it was made down to the gate above the pad. */
function PlanLine() {
  const { line, pos } = useMemo(() => {
    const pos = new Float32Array(MAX_NODES * 3);
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    const line = new THREE.Line(
      g,
      new THREE.LineBasicMaterial({ color: SIGNAL.output, transparent: true, opacity: 0.95 }),
    );
    line.frustumCulled = false;
    return { line, pos };
  }, []);
  const shown = useRef<object | null>(null);
  useFrame(() => {
    const plan = sim.pdg.plan;
    line.visible = !!plan && plan.status === 'optimal';
    if (!plan || plan === shown.current) return;
    shown.current = plan;
    const pts = plan.r.slice(0, MAX_NODES);
    pts.forEach((p, k) => pos.set([p.x, p.y, p.z], k * 3));
    line.geometry.setDrawRange(0, pts.length);
    line.geometry.attributes.position!.needsUpdate = true;
  });
  return <primitive object={line} />;
}
