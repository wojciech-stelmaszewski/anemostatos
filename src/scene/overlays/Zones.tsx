import { useFrame } from '@react-three/fiber';
import { useMemo } from 'react';
import * as THREE from 'three';
import { useParams } from '@/store/params';
import { sim } from '@/store/sim';
import { zoneDistance } from '@/sim/world';
import { SIGNAL } from '@/ui/colors';

const HEIGHT = 12;

/**
 * Keep-out zones: a translucent ceiling and a pillar. Virtual — the drone can pass through, but
 * they glow while it does, and the time spent inside is counted.
 */
export function Zones() {
  const w = useParams((s) => s.params.world);
  const level = useParams((s) => s.params.sim.level);
  const mats = useMemo(
    () => ({
      ceiling: new THREE.MeshBasicMaterial({
        color: SIGNAL.error,
        transparent: true,
        opacity: 0.1,
        side: THREE.DoubleSide,
        depthWrite: false,
      }),
      pillar: new THREE.MeshBasicMaterial({
        color: SIGNAL.error,
        transparent: true,
        opacity: 0.16,
        depthWrite: false,
      }),
      edge: new THREE.LineBasicMaterial({ color: SIGNAL.error, transparent: true, opacity: 0.6 }),
    }),
    [],
  );

  useFrame(() => {
    const inside = zoneDistance(sim.state.pos, sim.params.world, sim.level) < 0;
    mats.ceiling.opacity = inside ? 0.28 : 0.1;
    mats.pillar.opacity = inside ? 0.45 : 0.16;
  });

  return (
    <group>
      {w.ceiling > 0 && (
        <group position={[0, w.ceiling, 0]}>
          <mesh rotation-x={-Math.PI / 2} material={mats.ceiling}>
            <planeGeometry args={[40, 40]} />
          </mesh>
          <gridHelper
            args={[40, 40, SIGNAL.error, SIGNAL.error]}
            material-transparent
            material-opacity={0.15}
          />
        </group>
      )}
      {w.pillar && level > 1 && (
        <group position={[w.pillarX, HEIGHT / 2, w.pillarZ]}>
          <mesh material={mats.pillar}>
            <cylinderGeometry args={[w.pillarR, w.pillarR, HEIGHT, 48, 1, true]} />
          </mesh>
          <lineSegments material={mats.edge}>
            <edgesGeometry
              args={[new THREE.CylinderGeometry(w.pillarR, w.pillarR, HEIGHT, 24, 1, true)]}
            />
          </lineSegments>
        </group>
      )}
    </group>
  );
}
