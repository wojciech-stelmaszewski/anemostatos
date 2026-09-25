import { RoundedBox } from '@react-three/drei';
import { useImperativeHandle, useMemo, useRef, type Ref } from 'react';
import * as THREE from 'three';
import { MOTOR_POSITIONS, MOTOR_SPIN } from '@/sim/drone';
import { SCENE } from '@/ui/colors';
import { droneMaterials } from './materials';

const PROP_RADIUS = 0.064;
const MAX_VISUAL_SPIN = 190; // rad/s at full thrust — far below reality, readable on screen
const BLUR_START = 70; // rad/s where blades start dissolving into a disc

/** Imperative API: the scene updates the model every frame without React re-renders. */
export interface DroneModelHandle {
  /** thrust fractions 0…1 per motor, saturation flags, frame dt */
  animate(fractions: ArrayLike<number>, saturated: ArrayLike<boolean>, dt: number): void;
}

function bladeGeometry(): THREE.BufferGeometry {
  // Planform of one blade in the XY plane, root at the hub, rounded tip at +x.
  const shape = new THREE.Shape();
  const root = 0.006;
  const tip = PROP_RADIUS;
  const chord = (x: number) => {
    const t = (x - root) / (tip - root);
    return 0.007 + 0.009 * Math.sin(Math.PI * Math.min(1, t * 1.25)) * (1 - 0.35 * t);
  };
  const steps = 16;
  shape.moveTo(root, -0.0035);
  for (let i = 0; i <= steps; i++) {
    const x = root + ((tip - root - 0.004) * i) / steps;
    shape.lineTo(x, -chord(x) * 0.35);
  }
  shape.quadraticCurveTo(tip + 0.001, 0, tip - 0.004, chord(tip - 0.004) * 0.65);
  for (let i = steps; i >= 0; i--) {
    const x = root + ((tip - root - 0.004) * i) / steps;
    shape.lineTo(x, chord(x) * 0.65);
  }
  shape.closePath();
  const geo = new THREE.ExtrudeGeometry(shape, {
    depth: 0.0012,
    bevelEnabled: true,
    bevelSize: 0.0004,
    bevelThickness: 0.0003,
    bevelSegments: 1,
    curveSegments: 6,
  });
  geo.rotateX(-Math.PI / 2); // lay flat in the XZ plane
  geo.computeVertexNormals();
  return geo;
}

function Propeller({
  spin,
  propRef,
  discRef,
  bladeMaterial,
}: {
  spin: 1 | -1;
  propRef: (g: THREE.Group | null) => void;
  discRef: (m: THREE.Mesh | null) => void;
  bladeMaterial: THREE.Material;
}) {
  const blade = useMemo(bladeGeometry, []);
  // CW (spin = -1 about +y) and CCW props have mirrored blade pitch.
  const pitch = 0.22 * spin;
  return (
    <group>
      <group ref={propRef}>
        <mesh geometry={blade} material={bladeMaterial} rotation-x={pitch} castShadow />
        <group rotation-y={Math.PI}>
          <mesh geometry={blade} material={bladeMaterial} rotation-x={pitch} castShadow />
        </group>
        <mesh position-y={0.002}>
          <cylinderGeometry args={[0.005, 0.006, 0.006, 16]} />
          <meshStandardMaterial color="#20242b" roughness={0.4} />
        </mesh>
      </group>
      <mesh ref={discRef} rotation-x={-Math.PI / 2} position-y={0.0006}>
        <ringGeometry args={[0.008, PROP_RADIUS, 64]} />
        <meshBasicMaterial color="#c9d4e3" transparent opacity={0} depthWrite={false} />
      </mesh>
    </group>
  );
}

export function DroneModel({ ref }: { ref?: Ref<DroneModelHandle> }) {
  const m = droneMaterials();
  const props = useRef<(THREE.Group | null)[]>([]);
  const discs = useRef<(THREE.Mesh | null)[]>([]);
  const rings = useRef<(THREE.Mesh | null)[]>([]);
  const angles = useRef([0, 0, 0, 0]);
  const speeds = useRef([0, 0, 0, 0]);

  // One transparent blade material per prop so each can fade independently.
  const bladeMaterials = useMemo(
    () =>
      [0, 1, 2, 3].map(
        () =>
          new THREE.MeshStandardMaterial({
            color: '#2d3440',
            roughness: 0.35,
            metalness: 0.05,
            transparent: true,
          }),
      ),
    [],
  );
  const ringMaterials = useMemo(
    () =>
      [0, 1, 2, 3].map(
        () =>
          new THREE.MeshStandardMaterial({ color: SCENE.accent, roughness: 0.3, metalness: 0.6 }),
      ),
    [],
  );

  useImperativeHandle(ref, () => {
    const accent = new THREE.Color(SCENE.accent);
    const alarm = new THREE.Color('#ff1f3d');
    return {
      animate(fractions, saturated, dt) {
        for (let i = 0; i < 4; i++) {
          const f = Math.max(0, fractions[i] ?? 0);
          const target = Math.sqrt(f) * MAX_VISUAL_SPIN;
          // Props have inertia too: smooth the visual spin a little.
          speeds.current[i]! += (target - speeds.current[i]!) * Math.min(1, dt * 12);
          const w = speeds.current[i]!;
          angles.current[i]! += MOTOR_SPIN[i]! * w * dt;
          props.current[i]?.rotation.set(0, angles.current[i]!, 0);
          const blur = THREE.MathUtils.smoothstep(w, BLUR_START, MAX_VISUAL_SPIN * 0.8);
          bladeMaterials[i]!.opacity = 1 - 0.8 * blur;
          const disc = discs.current[i];
          if (disc) (disc.material as THREE.MeshBasicMaterial).opacity = 0.16 * blur;
          const mat = ringMaterials[i]!;
          const sat = saturated[i] ? 1 : 0;
          mat.color.copy(accent).lerp(alarm, sat);
          mat.emissive.copy(alarm).multiplyScalar(sat * 0.8);
        }
      },
    };
  }, [bladeMaterials, ringMaterials]);

  const armLength = Math.hypot(MOTOR_POSITIONS[0]!.x, MOTOR_POSITIONS[0]!.z) * 2 + 0.03;

  return (
    <group>
      {/* Frame: two crossed carbon arms and the bottom plate */}
      {[Math.PI / 4, -Math.PI / 4].map((r) => (
        <RoundedBox
          key={r}
          args={[armLength, 0.007, 0.024]}
          radius={0.003}
          smoothness={3}
          rotation-y={r}
          position-y={0.002}
          material={m.carbon}
          castShadow
          receiveShadow
        />
      ))}
      <RoundedBox
        args={[0.13, 0.006, 0.075]}
        radius={0.0025}
        smoothness={3}
        position-y={-0.002}
        material={m.carbon}
        castShadow
      />

      {/* Body: canopy with top vent plate */}
      <RoundedBox
        args={[0.11, 0.034, 0.066]}
        radius={0.013}
        smoothness={5}
        position-y={0.022}
        material={m.canopy}
        castShadow
      />
      <RoundedBox
        args={[0.066, 0.004, 0.044]}
        radius={0.0018}
        smoothness={3}
        position={[-0.008, 0.0395, 0]}
        material={m.darkPlastic}
      />
      {[-0.012, 0, 0.012].map((z) => (
        <mesh key={z} position={[-0.008, 0.0418, z]} material={m.strap}>
          <boxGeometry args={[0.05, 0.001, 0.004]} />
        </mesh>
      ))}

      {/* Camera pod — marks the front (+x) */}
      <RoundedBox
        args={[0.026, 0.024, 0.028]}
        radius={0.005}
        smoothness={3}
        position={[0.064, 0.02, 0]}
        material={m.darkPlastic}
        castShadow
      />
      <mesh position={[0.079, 0.02, 0]} rotation-z={-Math.PI / 2} material={m.darkMetal}>
        <cylinderGeometry args={[0.0095, 0.0095, 0.008, 24]} />
      </mesh>
      <mesh position={[0.0835, 0.02, 0]} rotation-z={-Math.PI / 2} material={m.lens}>
        <sphereGeometry args={[0.0072, 24, 12, 0, Math.PI * 2, 0, Math.PI / 2]} />
      </mesh>

      {/* Battery strapped underneath */}
      <RoundedBox
        args={[0.095, 0.03, 0.046]}
        radius={0.006}
        smoothness={3}
        position-y={-0.021}
        material={m.battery}
        castShadow
      />
      {[-0.025, 0.025].map((x) => (
        <mesh key={x} position={[x, -0.021, 0]} material={m.strap}>
          <boxGeometry args={[0.012, 0.0325, 0.0485]} />
        </mesh>
      ))}

      {/* Motors, props, legs, LEDs */}
      {MOTOR_POSITIONS.map((p, i) => (
        <group key={i} position={[p.x, 0, p.z]}>
          <mesh position-y={0.0075} material={m.darkMetal} castShadow>
            <cylinderGeometry args={[0.016, 0.016, 0.004, 32]} />
          </mesh>
          <mesh position-y={0.0165} material={m.metal} castShadow>
            <cylinderGeometry args={[0.0135, 0.0142, 0.014, 32]} />
          </mesh>
          <mesh
            ref={(r) => void (rings.current[i] = r)}
            position-y={0.0245}
            material={ringMaterials[i]}
          >
            <cylinderGeometry args={[0.0138, 0.0138, 0.003, 32]} />
          </mesh>
          <mesh position-y={0.028} material={m.metal}>
            <cylinderGeometry args={[0.0022, 0.0022, 0.006, 12]} />
          </mesh>
          <group position-y={0.029}>
            <Propeller
              spin={MOTOR_SPIN[i]!}
              propRef={(g) => void (props.current[i] = g)}
              discRef={(d) => void (discs.current[i] = d)}
              bladeMaterial={bladeMaterials[i]!}
            />
          </group>
          {/* Leg with rubber foot */}
          <mesh position-y={-0.022} material={m.darkMetal} castShadow>
            <cylinderGeometry args={[0.0035, 0.0045, 0.04, 12]} />
          </mesh>
          <mesh position-y={-0.046} material={m.rubber} castShadow>
            <sphereGeometry args={[0.006, 16, 12]} />
          </mesh>
          {/* LED under the arm: white at the front, red at the back */}
          <mesh
            position={[-p.x * 0.18, -0.003, -p.z * 0.18]}
            material={p.x > 0 ? m.ledFront : m.ledRear}
          >
            <sphereGeometry args={[0.0042, 12, 8]} />
          </mesh>
        </group>
      ))}
    </group>
  );
}
