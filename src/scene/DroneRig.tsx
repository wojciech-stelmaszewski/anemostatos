import { useFrame, type ThreeEvent } from '@react-three/fiber';
import { useRef } from 'react';
import * as THREE from 'three';
import { v3 } from '@/math/vec3';
import { sim } from '@/store/sim';
import { DroneModel, type DroneModelHandle } from './Drone';

const IMPULSE = 1.2; // N·s

/** Places the drone model at the interpolated simulation pose and animates motors. */
export function DroneRig() {
  const group = useRef<THREE.Group>(null);
  const model = useRef<DroneModelHandle>(null);
  const fractions = [0, 0, 0, 0];
  const saturated = [false, false, false, false];

  useFrame(({ clock }, delta) => {
    const g = group.current;
    if (!g) return;
    const { pos, q } = sim.renderPose();
    g.position.set(pos.x, pos.y, pos.z);
    g.quaternion.set(q.x, q.y, q.z, q.w);
    const fmax = sim.params.drone.maxMotorThrust;
    const cmd = sim.actuation.motorCmd;
    for (let i = 0; i < 4; i++) {
      fractions[i] = sim.state.motors[i]! / fmax;
      saturated[i] =
        sim.armed &&
        sim.level !== 2 &&
        (cmd[i]! >= fmax - 1e-6 || (cmd[i]! <= 1e-6 && !sim.onGround));
    }
    // Props keep their visual speed while paused/slow-mo in proportion to simulated time.
    model.current?.animate(fractions, saturated, sim.paused ? 0 : delta * sim.timeScale);
    void clock;
  });

  /** Click the drone to give it a shove (away from the camera, or downward in L1). */
  const poke = (e: ThreeEvent<MouseEvent>) => {
    e.stopPropagation();
    if (sim.level === 1) {
      sim.applyImpulse(v3(0, -IMPULSE, 0));
      return;
    }
    const d = e.ray.direction;
    const h = Math.hypot(d.x, d.z) || 1;
    sim.applyImpulse(v3((d.x / h) * IMPULSE, 0, (d.z / h) * IMPULSE));
  };

  return (
    <group
      ref={group}
      onClick={poke}
      onPointerOver={() => (document.body.style.cursor = 'pointer')}
      onPointerOut={() => (document.body.style.cursor = '')}
    >
      <DroneModel ref={model} />
    </group>
  );
}
