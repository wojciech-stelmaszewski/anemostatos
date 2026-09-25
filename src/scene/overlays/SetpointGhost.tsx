import { Html } from '@react-three/drei';
import { useFrame, useThree, type ThreeEvent } from '@react-three/fiber';
import { useMemo, useRef, useState } from 'react';
import * as THREE from 'three';
import { useParams } from '@/store/params';
import { sim, useUi } from '@/store/sim';
import { SIGNAL } from '@/ui/colors';

const snap = (v: number) => Math.round(v * 20) / 20;

/**
 * Translucent target marker at the setpoint with a dashed error line to the drone.
 * Drag it to move the setpoint: vertically in L1, horizontally in L2/L3 (Shift = vertical).
 */
export function SetpointGhost() {
  const show = useUi((s) => s.overlays.setpoint);
  const group = useRef<THREE.Group>(null);
  const label = useRef<HTMLDivElement>(null);
  const [hover, setHover] = useState(false);
  const dragging = useRef<{ plane: THREE.Plane; vertical: boolean } | null>(null);
  const { camera, controls } = useThree() as unknown as {
    camera: THREE.Camera;
    controls: { enabled: boolean } | null;
  };

  const errorLine = useMemo(() => {
    const g = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
    const m = new THREE.LineDashedMaterial({
      color: SIGNAL.error,
      dashSize: 0.04,
      gapSize: 0.03,
      transparent: true,
      opacity: 0.9,
    });
    return new THREE.Line(g, m);
  }, []);

  useFrame(() => {
    const sp = sim.setpoint;
    group.current?.position.set(sp.x, sp.y, sp.z);
    const { pos } = sim.renderPose();
    const attr = errorLine.geometry.getAttribute('position') as THREE.BufferAttribute;
    attr.setXYZ(0, pos.x, pos.y, pos.z);
    attr.setXYZ(1, sp.x, sp.y, sp.z);
    attr.needsUpdate = true;
    errorLine.computeLineDistances();
    const e = Math.hypot(sp.x - pos.x, sp.y - pos.y, sp.z - pos.z);
    errorLine.visible = e > 0.02;
    if (label.current) label.current.textContent = `setpoint ${sp.y.toFixed(2)} m`;
  });

  const onDown = (e: ThreeEvent<PointerEvent>) => {
    e.stopPropagation();
    (e.target as Element).setPointerCapture?.(e.pointerId);
    const vertical = sim.level === 1 || e.shiftKey;
    const p = new THREE.Vector3(sim.setpoint.x, sim.setpoint.y, sim.setpoint.z);
    let normal: THREE.Vector3;
    if (vertical) {
      normal = new THREE.Vector3().subVectors(camera.position, p).setY(0).normalize();
    } else normal = new THREE.Vector3(0, 1, 0);
    dragging.current = {
      plane: new THREE.Plane().setFromNormalAndCoplanarPoint(normal, p),
      vertical,
    };
    if (controls) controls.enabled = false;
  };
  const onMove = (e: ThreeEvent<PointerEvent>) => {
    const d = dragging.current;
    if (!d) return;
    const hit = new THREE.Vector3();
    if (!e.ray.intersectPlane(d.plane, hit)) return;
    const set = useParams.getState().set;
    if (d.vertical) set('setpoint.y', Math.min(10, Math.max(0.2, snap(hit.y))));
    else {
      set('setpoint.x', Math.min(10, Math.max(-10, snap(hit.x))));
      set('setpoint.z', Math.min(10, Math.max(-10, snap(hit.z))));
    }
  };
  const onUp = (e: ThreeEvent<PointerEvent>) => {
    (e.target as Element).releasePointerCapture?.(e.pointerId);
    dragging.current = null;
    if (controls) controls.enabled = true;
  };

  if (!show) return null;
  return (
    <>
      <primitive object={errorLine} />
      <group ref={group}>
        <mesh
          onPointerDown={onDown}
          onPointerMove={onMove}
          onPointerUp={onUp}
          onPointerOver={() => {
            setHover(true);
            document.body.style.cursor = 'grab';
          }}
          onPointerOut={() => {
            setHover(false);
            document.body.style.cursor = '';
          }}
        >
          <sphereGeometry args={[0.07, 20, 14]} />
          <meshBasicMaterial
            color={SIGNAL.setpoint}
            transparent
            opacity={hover ? 0.5 : 0.22}
            depthWrite={false}
          />
        </mesh>
        <mesh rotation-x={-Math.PI / 2}>
          <ringGeometry args={[0.26, 0.275, 64]} />
          <meshBasicMaterial
            color={SIGNAL.setpoint}
            transparent
            opacity={0.6}
            side={THREE.DoubleSide}
            depthWrite={false}
          />
        </mesh>
        {[0, Math.PI / 2].map((r) => (
          <mesh key={r} rotation-y={r}>
            <boxGeometry args={[0.62, 0.002, 0.002]} />
            <meshBasicMaterial color={SIGNAL.setpoint} transparent opacity={0.5} />
          </mesh>
        ))}
        <Html position={[0.3, 0.06, 0]} style={{ pointerEvents: 'none' }}>
          <div
            ref={label}
            className="select-none whitespace-nowrap font-mono text-[10px] text-muted"
          />
        </Html>
      </group>
    </>
  );
}
