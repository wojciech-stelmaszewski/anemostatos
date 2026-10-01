/* eslint-disable react-refresh/only-export-components -- a one-off render page, never hot-reloaded */
/**
 * The cover picture of the book, rendered with the simulator's own drone model.
 *
 * The quadrotor hovers over its set-point leaning into a 5 m/s wind, as at the end of lesson I.13;
 * the orange trail is the path it actually flew there (trail.json, exported by trail.py from
 * book/data/cascade.csv.gz), with a bead every half second. The wind is drawn as streamlines of a
 * simple flow model: the uniform wind, the flow around the airframe (a doublet) and the rotor
 * downwash along the body axis. Lines that pass through the downwash warm up in colour.
 *
 * render.mjs opens this page headlessly and saves the frame; cover.py finishes it for the book.
 */
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { useEffect, useMemo, useRef } from 'react';
import { createRoot } from 'react-dom/client';
import * as THREE from 'three';
import { EffectComposer } from 'three/examples/jsm/postprocessing/EffectComposer.js';
import { OutputPass } from 'three/examples/jsm/postprocessing/OutputPass.js';
import { RenderPass } from 'three/examples/jsm/postprocessing/RenderPass.js';
import { UnrealBloomPass } from 'three/examples/jsm/postprocessing/UnrealBloomPass.js';
import { RoomEnvironment } from 'three/examples/jsm/environments/RoomEnvironment.js';
import { DroneModel, type DroneModelHandle } from '@/scene/Drone';
import trail from './trail.json';

const NIGHT = '#0b0e13';
const ACCENT = '#ff7a1a'; // the app's orange: the warm fill light
const PITCH = THREE.MathUtils.degToRad(trail.hoverPitchDeg); // leaning into the wind
const YAW = THREE.MathUtils.degToRad(-118); // nose towards the viewer and the wind
const GROUND = Math.min(...trail.y) - 0.05; // the take-off pad, 2 m below the hover

// ------------------------------------------------------------------ the flow model ---------------
const UP = new THREE.Vector3(0, 1, 0).applyAxisAngle(new THREE.Vector3(0, 0, 1), PITCH); // body axis
const A = 0.17; // radius of the obstacle seen by the wind
function velocity(p: THREE.Vector3, out: THREE.Vector3): number {
  const r2 = p.lengthSq();
  const r = Math.sqrt(r2);
  out.set(1, 0, 0);
  if (r > 0.02) {
    // a doublet: potential flow round a sphere of radius A
    const k = (A * A * A) / 2;
    out.x += k * (1 / (r2 * r) - (3 * p.x * p.x) / (r2 * r2 * r));
    out.y += k * ((-3 * p.x * p.y) / (r2 * r2 * r));
    out.z += k * ((-3 * p.x * p.z) / (r2 * r2 * r));
  }
  // the rotor downwash: a jet along -UP below the rotor plane, widening and slowing with depth
  const s = -p.dot(UP); // depth below the rotors
  let heat = 0;
  if (s > -0.02) {
    const radial = p.clone().addScaledVector(UP, s).length();
    const width = 0.2 + 0.18 * Math.max(0, s);
    const w = 1.5 * Math.exp(-((radial / width) ** 2)) * Math.exp(-Math.max(0, s) / 1.3);
    out.addScaledVector(UP, -w);
    heat = w / 1.5;
  }
  return heat;
}

function streamline(seed: THREE.Vector3) {
  const pts: THREE.Vector3[] = [];
  const heat: number[] = [];
  const p = seed.clone();
  const v = new THREE.Vector3();
  const mid = new THREE.Vector3();
  for (let i = 0; i < 900 && p.x < 2.6 && p.y > GROUND + 0.05; i++) {
    pts.push(p.clone());
    heat.push(velocity(p, v));
    mid.copy(p).addScaledVector(v.normalize(), 0.005);
    velocity(mid, v);
    p.addScaledVector(v.normalize(), 0.01);
  }
  return { pts, heat };
}

// ------------------------------------------------------------------ glowing tubes ----------------
const glowVertex = /* glsl */ `
  attribute float heat;
  varying float vU; varying float vHeat; varying vec3 vWorld;
  void main() {
    vU = uv.x; vHeat = heat;
    vec4 w = modelMatrix * vec4(position, 1.0); vWorld = w.xyz;
    gl_Position = projectionMatrix * viewMatrix * w;
  }`;
const glowFragment = /* glsl */ `
  uniform vec3 cold; uniform vec3 warm; uniform float opacity; uniform float streaks;
  uniform float phase; uniform float length_;
  varying float vU; varying float vHeat; varying vec3 vWorld;
  void main() {
    float ends = smoothstep(0.0, 0.18, vU) * smoothstep(1.0, 0.82, vU);
    float pulse = 0.12 + 0.88 * pow(0.5 + 0.5 * sin(vU * length_ * streaks - phase), 10.0);
    float depth = exp(-max(0.0, -vWorld.z) * 0.9) * exp(-max(0.0, vWorld.z - 0.2) * 0.8);
    vec3 c = mix(cold, warm, clamp(vHeat * 1.6, 0.0, 1.0));
    gl_FragColor = vec4(c, opacity * ends * pulse * depth);
  }`;

function Tube({
  pts,
  heat,
  radius,
  cold,
  warm,
  opacity,
  streaks = 0,
  phase = 0,
}: {
  pts: THREE.Vector3[];
  heat: number[];
  radius: number;
  cold: string;
  warm: string;
  opacity: number;
  streaks?: number;
  phase?: number;
}) {
  const geometry = useMemo(() => {
    const curve = new THREE.CatmullRomCurve3(pts);
    const segs = Math.max(8, Math.floor(pts.length / 2));
    const radial = 6;
    const g = new THREE.TubeGeometry(curve, segs, radius, radial, false);
    const h = new Float32Array((segs + 1) * (radial + 1));
    for (let i = 0; i <= segs; i++) {
      const v = heat[Math.min(heat.length - 1, Math.round((i / segs) * (heat.length - 1)))] ?? 0;
      for (let j = 0; j <= radial; j++) h[i * (radial + 1) + j] = v;
    }
    g.setAttribute('heat', new THREE.BufferAttribute(h, 1));
    return g;
  }, [pts, heat, radius]);
  const length = useMemo(() => {
    let L = 0;
    for (let i = 1; i < pts.length; i++) L += pts[i]!.distanceTo(pts[i - 1]!);
    return L;
  }, [pts]);
  const material = useMemo(
    () =>
      new THREE.ShaderMaterial({
        vertexShader: glowVertex,
        fragmentShader: glowFragment,
        uniforms: {
          cold: { value: new THREE.Color(cold) },
          warm: { value: new THREE.Color(warm) },
          opacity: { value: opacity },
          streaks: { value: streaks },
          phase: { value: phase },
          length_: { value: length },
        },
        transparent: true,
        depthWrite: false,
        blending: THREE.AdditiveBlending,
      }),
    [cold, warm, opacity, streaks, phase, length],
  );
  return <mesh geometry={geometry} material={material} />;
}

function Wind() {
  const lines = useMemo(() => {
    const rng = mulberry32(11);
    const out: { pts: THREE.Vector3[]; heat: number[]; phase: number }[] = [];
    for (let i = 0; i < 80; i++) {
      const seed = new THREE.Vector3(-2.4, -0.6 + 1.05 * rng(), -0.6 + 0.9 * rng());
      // more lines near the drone, where the flow has something to say
      if (Math.abs(seed.y) > 0.45 && rng() < 0.5) continue;
      const s = streamline(seed);
      if (s.pts.length > 20) out.push({ ...s, phase: rng() * 40 });
    }
    return out;
  }, []);
  return (
    <group>
      {lines.map((l, i) => (
        <Tube
          key={i}
          pts={l.pts}
          heat={l.heat}
          radius={0.0013}
          cold="#5aa2ff"
          warm="#ffc48a"
          opacity={0.42}
          streaks={7}
          phase={l.phase}
        />
      ))}
    </group>
  );
}

function Trail() {
  const pts = useMemo(() => trail.x.map((x, i) => new THREE.Vector3(x, trail.y[i]!, 0)), []);
  const heat = useMemo(() => pts.map((_, i) => i / pts.length), [pts]);
  const beads = useMemo(
    () =>
      trail.t.flatMap((t, i) =>
        Math.abs((t % 0.5) - 0.0) < 0.011 && i < pts.length - 30 ? [pts[i]!] : [],
      ),
    [pts],
  );
  return (
    <group>
      <Tube pts={pts} heat={heat} radius={0.006} cold="#ff5a00" warm="#ffc78f" opacity={0.95} />
      <Tube pts={pts} heat={heat} radius={0.02} cold="#ff5a00" warm="#ff8a3a" opacity={0.12} />
      {beads.map((p, i) => (
        <mesh key={i} position={p}>
          <sphereGeometry args={[0.0075, 16, 12]} />
          <meshBasicMaterial color="#ffb070" toneMapped={false} />
        </mesh>
      ))}
    </group>
  );
}

function Grid() {
  const material = useMemo(
    () =>
      new THREE.ShaderMaterial({
        transparent: true,
        depthWrite: false,
        uniforms: { color: { value: new THREE.Color('#3d6aa8') } },
        vertexShader: `varying vec3 vW; void main(){ vec4 w = modelMatrix*vec4(position,1.0); vW = w.xyz; gl_Position = projectionMatrix*viewMatrix*w; }`,
        fragmentShader: `uniform vec3 color; varying vec3 vW;
          float line(float x, float w){ float d = abs(fract(x - 0.5) - 0.5) / fwidth(x); return 1.0 - min(d / w, 1.0); }
          void main(){
            float major = max(line(vW.x, 1.2), line(vW.z, 1.2));
            float minor = max(line(vW.x * 4.0, 0.8), line(vW.z * 4.0, 0.8)) * 0.35;
            float fade = exp(-length(vW.xz - vec2(0.3, -0.6)) * 0.45);
            gl_FragColor = vec4(color, max(major, minor) * fade * 0.32);
          }`,
        extensions: { derivatives: true } as never,
      }),
    [],
  );
  return (
    <mesh rotation-x={-Math.PI / 2} position-y={GROUND} material={material}>
      <planeGeometry args={[30, 30]} />
    </mesh>
  );
}

function Hero() {
  const ref = useRef<DroneModelHandle>(null);
  useEffect(() => {
    // spin the props up until their blades dissolve into discs
    for (let k = 0; k < 240; k++)
      ref.current?.animate([0.62, 0.6, 0.62, 0.6], [false, false, false, false], 1 / 60);
  }, []);
  return (
    <group rotation-z={PITCH}>
      <group rotation-y={YAW}>
        <DroneModel ref={ref} />
      </group>
    </group>
  );
}

function Post() {
  const { gl, scene, camera, size } = useThree();
  const composer = useMemo(() => {
    const c = new EffectComposer(gl);
    c.addPass(new RenderPass(scene, camera));
    c.addPass(new UnrealBloomPass(new THREE.Vector2(size.width, size.height), 0.55, 0.6, 0.86));
    c.addPass(new OutputPass());
    return c;
  }, [gl, scene, camera, size]);
  useEffect(() => composer.setSize(size.width, size.height), [composer, size]);
  const frames = useRef(0);
  useFrame(() => {
    composer.render();
    if (++frames.current === 20)
      (window as unknown as { __coverReady: boolean }).__coverReady = true;
  }, 1);
  return null;
}

function Environment() {
  const { gl, scene } = useThree();
  useEffect(() => {
    const pmrem = new THREE.PMREMGenerator(gl);
    scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
    scene.environmentIntensity = 0.55;
    scene.background = new THREE.Color(NIGHT);
    return () => pmrem.dispose();
  }, [gl, scene]);
  return null;
}

function Cover() {
  return (
    <Canvas
      dpr={[1, 2]}
      gl={{
        antialias: true,
        preserveDrawingBuffer: true,
        toneMapping: THREE.ACESFilmicToneMapping,
        toneMappingExposure: 1.05,
      }}
      camera={{ fov: 30, near: 0.05, far: 60, position: [-0.6, 0.2, 1.32] }}
      onCreated={({ camera }) => camera.lookAt(0.13, -0.07, 0)}
    >
      <Environment />
      <ambientLight intensity={0.15} />
      <directionalLight position={[-1.6, 2.2, 3]} intensity={1.7} color="#fff3e6" />
      <directionalLight position={[2.5, 0.8, -2]} intensity={6} color="#7fb6ff" />
      <directionalLight position={[-0.5, -1.5, -2]} intensity={2.5} color="#ff9a52" />
      <pointLight position={[0.0, -0.6, 0.5]} intensity={0.6} color={ACCENT} distance={2} />
      <Hero />
      <Trail />
      <Wind />
      <Grid />
      <Post />
    </Canvas>
  );
}

function mulberry32(a: number) {
  return () => {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

createRoot(document.getElementById('root')!).render(<Cover />);
