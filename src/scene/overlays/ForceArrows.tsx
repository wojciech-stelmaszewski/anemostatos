import { Html } from '@react-three/drei';
import { useFrame } from '@react-three/fiber';
import { useMemo, useRef } from 'react';
import * as THREE from 'three';
import { sim, useUi } from '@/store/sim';
import { SIGNAL } from '@/ui/colors';
import { Arrow } from './Arrow';

/** Metres of arrow per newton. */
export const N_TO_M = 0.04;

const tmpO = new THREE.Vector3();
const tmpV = new THREE.Vector3();

function useArrows<K extends string>(spec: Record<K, string>, radius?: number) {
  return useMemo(() => {
    const out = {} as Record<K, Arrow>;
    for (const k in spec) out[k] = new Arrow(spec[k], radius);
    return out;
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
}

/** Thrust, gravity and aerodynamic force at the drone (and the desired thrust in L3). */
export function ForceArrows() {
  const show = useUi((s) => s.overlays.forces);
  const arrows = useArrows({
    thrust: SIGNAL.thrust,
    gravity: SIGNAL.gravity,
    drag: SIGNAL.drag,
    desired: SIGNAL.ff,
  });

  useFrame(() => {
    const { pos } = sim.renderPose();
    const f = sim.forces;
    const set = (key: keyof typeof arrows, v: { x: number; y: number; z: number }, offset = 0) => {
      tmpO.set(pos.x + offset, pos.y, pos.z);
      tmpV.set(v.x, v.y, v.z).multiplyScalar(N_TO_M);
      arrows[key].set(tmpO, tmpV);
    };
    set('thrust', f.thrust);
    set('gravity', f.gravity);
    set('drag', f.drag, 0.02);
    const ex = sim.controller.extras();
    if (sim.level === 3 && 'fdes.x' in ex) {
      set('desired', { x: ex['fdes.x']!, y: ex['fdes.y']!, z: ex['fdes.z']! });
    } else arrows.desired.visible = false;
  });

  if (!show) return null;
  return (
    <group>
      {Object.entries(arrows).map(([k, a]) => (
        <primitive key={k} object={a} />
      ))}
    </group>
  );
}

/**
 * The PID made visible on the drone: separate arrows for the P, I, D and feedforward
 * contributions (L1: vertical; L2: 3D vectors from the three axis loops).
 */
export function TermArrows() {
  const show = useUi((s) => s.overlays.terms);
  const arrows = useArrows({ p: SIGNAL.p, i: SIGNAL.i, d: SIGNAL.d, ff: SIGNAL.ff }, 0.011);
  const order = ['ff', 'p', 'i', 'd'] as const;

  useFrame(() => {
    const { pos } = sim.renderPose();
    const loops = sim.controller.loops();
    const vec = (k: 'p' | 'i' | 'd' | 'ff') => {
      if (sim.level === 1) return tmpV.set(0, loops.alt?.[k] ?? 0, 0);
      if (sim.level === 2)
        return tmpV.set(
          loops['pos.x']?.[k] ?? 0,
          loops['pos.y']?.[k] ?? 0,
          loops['pos.z']?.[k] ?? 0,
        );
      return tmpV.set(0, 0, 0);
    };
    order.forEach((k, idx) => {
      // Side by side, left of the drone as seen from the default camera.
      tmpO.set(pos.x - 0.3 - idx * 0.06, pos.y, pos.z);
      arrows[k].set(tmpO, vec(k).multiplyScalar(N_TO_M), 0.004);
    });
  });

  if (!show || sim.level === 3) return null;
  return (
    <group>
      {order.map((k) => (
        <primitive key={k} object={arrows[k]} />
      ))}
      <TermLegend />
    </group>
  );
}

function TermLegend() {
  const ref = useRef<THREE.Group>(null);
  useFrame(() => {
    const { pos } = sim.renderPose();
    ref.current?.position.set(pos.x, pos.y, pos.z);
  });
  return (
    <group ref={ref}>
      {(['ff', 'p', 'i', 'd'] as const).map((k, idx) => (
        <group key={k} position={[-0.3 - idx * 0.06, -0.035, 0]}>
          <Html center style={{ pointerEvents: 'none' }}>
            <span
              className="select-none font-mono text-[9px] font-bold"
              style={{ color: SIGNAL[k] }}
            >
              {k.toUpperCase()}
            </span>
          </Html>
        </group>
      ))}
    </group>
  );
}
