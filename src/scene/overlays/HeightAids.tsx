import { Html } from '@react-three/drei';
import { useFrame } from '@react-three/fiber';
import { useMemo, useRef } from 'react';
import * as THREE from 'three';
import { sim, useUi } from '@/store/sim';

/** Drop line to the floor, floor marker and a height label: altitude readable from any angle. */
export function HeightAids() {
  const show = useUi((s) => s.overlays.heightAids);
  const marker = useRef<THREE.Group>(null);
  const labelAnchor = useRef<THREE.Group>(null);
  const label = useRef<HTMLDivElement>(null);
  const line = useMemo(() => {
    const g = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
    const m = new THREE.LineDashedMaterial({
      color: '#8b94a5',
      dashSize: 0.05,
      gapSize: 0.05,
      transparent: true,
      opacity: 0.6,
    });
    return new THREE.Line(g, m);
  }, []);
  const ticks = useMemo(() => {
    // Half-metre ticks along the drop line.
    const pts: THREE.Vector3[] = [];
    for (let h = 0.5; h <= 20; h += 0.5) {
      const w = h % 1 === 0 ? 0.05 : 0.025;
      pts.push(new THREE.Vector3(-w, h, 0), new THREE.Vector3(w, h, 0));
    }
    const g = new THREE.BufferGeometry().setFromPoints(pts);
    return new THREE.LineSegments(
      g,
      new THREE.LineBasicMaterial({ color: '#8b94a5', transparent: true, opacity: 0.5 }),
    );
  }, []);

  useFrame(({ camera }) => {
    const { pos } = sim.renderPose();
    const attr = line.geometry.getAttribute('position') as THREE.BufferAttribute;
    attr.setXYZ(0, pos.x, pos.y - 0.05, pos.z);
    attr.setXYZ(1, pos.x, 0, pos.z);
    attr.needsUpdate = true;
    line.computeLineDistances();
    marker.current?.position.set(pos.x, 0.002, pos.z);
    ticks.position.set(pos.x, 0, pos.z);
    ticks.rotation.y = Math.atan2(camera.position.x - pos.x, camera.position.z - pos.z);
    ticks.geometry.setDrawRange(0, Math.max(0, Math.floor((pos.y - 0.05) / 0.5)) * 2);
    labelAnchor.current?.position.set(pos.x, pos.y / 2, pos.z);
    if (label.current) label.current.textContent = `${pos.y.toFixed(2)} m`;
  });

  if (!show) return null;
  return (
    <>
      <primitive object={line} />
      <primitive object={ticks} />
      <group ref={marker}>
        <mesh rotation-x={-Math.PI / 2}>
          <ringGeometry args={[0.1, 0.115, 48]} />
          <meshBasicMaterial color="#8b94a5" transparent opacity={0.7} depthWrite={false} />
        </mesh>
      </group>
      <group ref={labelAnchor}>
        <Html position={[0.06, 0, 0]} style={{ pointerEvents: 'none' }}>
          <div
            ref={label}
            className="select-none whitespace-nowrap rounded bg-black/40 px-1 font-mono text-[10px] text-fg"
          />
        </Html>
      </group>
    </>
  );
}
