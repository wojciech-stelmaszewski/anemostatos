import { useFrame } from '@react-three/fiber';
import { useRef } from 'react';
import * as THREE from 'three';
import { sim } from '@/store/sim';
import { SCENE } from '@/ui/colors';

/** Size of the procedural satellite's bus, m. */
const BUS = 0.5;
const PANEL = { length: 1.1, width: 0.42 };

/**
 * A procedural satellite for Chapter N: a bus, two solar panels along body x and a dish along
 * body y (the "telescope" it points). Pose from the simulator, which mirrors the satellite's
 * attitude into the drone's state; small rings mark the three wheel axes' thrusters.
 */
export function SatelliteRig() {
  const group = useRef<THREE.Group>(null);
  const jets = useRef<(THREE.Mesh | null)[]>([]);
  useFrame(() => {
    const g = group.current;
    if (!g) return;
    const { pos, q } = sim.renderPose();
    g.position.set(pos.x, pos.y, pos.z);
    g.quaternion.set(q.x, q.y, q.z, q.w);
    const s = sim.satellite;
    const thr = s ? [s.thrustTorque.x, s.thrustTorque.y, s.thrustTorque.z] : [0, 0, 0];
    jets.current.forEach((m, i) => {
      if (!m) return;
      const f = Math.sign(thr[Math.floor(i / 2)]!);
      // Each axis has a pair: index 2k fires for +, 2k+1 for −.
      m.visible = f !== 0 && (i % 2 === 0 ? f > 0 : f < 0);
    });
  });
  const jetAt: [number, number, number][] = [
    [0, BUS / 2 + 0.06, BUS / 2],
    [0, -BUS / 2 - 0.06, BUS / 2],
    [BUS / 2, 0, BUS / 2 + 0.06],
    [-BUS / 2, 0, BUS / 2 + 0.06],
    [BUS / 2 + 0.06, BUS / 2, 0],
    [-BUS / 2 - 0.06, BUS / 2, 0],
  ];
  return (
    <group ref={group}>
      <mesh castShadow>
        <boxGeometry args={[BUS, BUS, BUS]} />
        <meshStandardMaterial color="#c9a227" metalness={0.6} roughness={0.35} />
      </mesh>
      {[-1, 1].map((side) => (
        <group key={side} position={[side * (BUS / 2 + 0.08 + PANEL.length / 2), 0, 0]}>
          <mesh castShadow>
            <boxGeometry args={[PANEL.length, 0.02, PANEL.width]} />
            <meshStandardMaterial color="#1d3a6b" metalness={0.4} roughness={0.3} />
          </mesh>
          <mesh position={[-side * (PANEL.length / 2 + 0.04), 0, 0]}>
            <cylinderGeometry args={[0.012, 0.012, 0.08, 8]} />
            <meshStandardMaterial color="#8b94a5" />
          </mesh>
        </group>
      ))}
      <mesh position={[0, BUS / 2 + 0.1, 0]} castShadow>
        <cylinderGeometry args={[0.2, 0.06, 0.12, 24, 1, true]} />
        <meshStandardMaterial color="#e6e9ef" side={THREE.DoubleSide} />
      </mesh>
      <mesh position={[0, BUS / 2 + 0.22, 0]}>
        <sphereGeometry args={[0.025, 12, 12]} />
        <meshStandardMaterial color={SCENE.accent} />
      </mesh>
      {jetAt.map((p, i) => (
        <mesh
          key={i}
          ref={(m) => {
            jets.current[i] = m;
          }}
          position={p}
          visible={false}
        >
          <sphereGeometry args={[0.05, 10, 10]} />
          <meshBasicMaterial color="#ffb347" transparent opacity={0.85} />
        </mesh>
      ))}
    </group>
  );
}
