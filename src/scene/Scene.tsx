import { Environment, Lightformer } from '@react-three/drei';
import { Canvas, useFrame } from '@react-three/fiber';
import { useMemo, useRef } from 'react';
import * as THREE from 'three';
import { sim } from '@/store/sim';
import { SCENE } from '@/ui/colors';
import { CameraRig } from './CameraRig';
import { DroneRig } from './DroneRig';
import { Grid } from './Grid';
import { ForceArrows, TermArrows } from './overlays/ForceArrows';
import { HeightAids } from './overlays/HeightAids';
import { SetpointGhost } from './overlays/SetpointGhost';
import { Trail } from './overlays/Trail';
import { WindParticles } from './overlays/WindParticles';

/** Advances the simulation before anything else reads it this frame. */
function SimDriver() {
  useFrame((_, delta) => sim.advance(delta), -1);
  return null;
}

function Sky() {
  const material = useMemo(
    () =>
      new THREE.ShaderMaterial({
        side: THREE.BackSide,
        depthWrite: false,
        uniforms: {
          top: { value: new THREE.Color(SCENE.skyTop) },
          horizon: { value: new THREE.Color(SCENE.skyHorizon) },
          bottom: { value: new THREE.Color(SCENE.background) },
        },
        vertexShader: `varying vec3 vDir; void main(){ vDir = normalize(position); gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0); }`,
        fragmentShader: `uniform vec3 top; uniform vec3 horizon; uniform vec3 bottom; varying vec3 vDir;
          void main(){ float h = vDir.y; vec3 c = h > 0.0 ? mix(horizon, top, pow(h, 0.55)) : mix(horizon, bottom, pow(-h, 0.3));
          gl_FragColor = vec4(c, 1.0);
          #include <colorspace_fragment>
          }`,
      }),
    [],
  );
  const ref = useRef<THREE.Mesh>(null);
  useFrame(({ camera }) => ref.current?.position.copy(camera.position));
  return (
    <mesh ref={ref} material={material} renderOrder={-10} frustumCulled={false}>
      <sphereGeometry args={[400, 32, 16]} />
    </mesh>
  );
}

/** Key light that follows the drone so its shadow on the grid always works as a height cue. */
function KeyLight() {
  const light = useRef<THREE.DirectionalLight>(null);
  useFrame(() => {
    const l = light.current;
    if (!l) return;
    const p = sim.state.pos;
    l.position.set(p.x + 1.5, p.y + 8, p.z + 1);
    l.target.position.set(p.x, 0, p.z);
    l.target.updateMatrixWorld();
  });
  return (
    <directionalLight
      ref={light}
      intensity={2.2}
      castShadow
      shadow-mapSize={[2048, 2048]}
      shadow-camera-left={-3}
      shadow-camera-right={3}
      shadow-camera-top={3}
      shadow-camera-bottom={-3}
      shadow-camera-near={0.5}
      shadow-camera-far={30}
      shadow-bias={-0.0004}
      shadow-radius={4}
    />
  );
}

export function Scene() {
  return (
    <Canvas
      shadows="soft"
      dpr={[1, 2]}
      camera={{ position: [0.95, 2.3, 1.45], fov: 42, near: 0.02, far: 1000 }}
      gl={{ antialias: true }}
      onCreated={({ gl }) => {
        gl.toneMapping = THREE.ACESFilmicToneMapping;
        gl.toneMappingExposure = 1.05;
      }}
    >
      <SimDriver />
      <Sky />
      <fog attach="fog" args={[SCENE.background, 25, 90]} />
      <ambientLight intensity={0.25} />
      <KeyLight />
      <Environment resolution={256} frames={1}>
        <Lightformer
          form="rect"
          intensity={3}
          position={[0, 5, -2]}
          scale={[8, 3, 1]}
          color="#ffffff"
        />
        <Lightformer
          form="rect"
          intensity={1.5}
          position={[-5, 2, 1]}
          rotation-y={Math.PI / 2}
          scale={[6, 2, 1]}
          color="#9ec5f4"
        />
        <Lightformer
          form="rect"
          intensity={1.2}
          position={[5, 1, 2]}
          rotation-y={-Math.PI / 2}
          scale={[6, 2, 1]}
          color="#ffd2b0"
        />
        <Lightformer form="ring" intensity={2} position={[0, 6, 4]} scale={2} color="#ffffff" />
      </Environment>

      <Grid />
      <mesh rotation-x={-Math.PI / 2} receiveShadow>
        <planeGeometry args={[200, 200]} />
        <shadowMaterial opacity={0.45} />
      </mesh>

      <DroneRig />
      <HeightAids />
      <SetpointGhost />
      <ForceArrows />
      <TermArrows />
      <Trail />
      <WindParticles />
      <CameraRig />
    </Canvas>
  );
}
