import { useMemo } from 'react';
import * as THREE from 'three';
import { SCENE } from '@/ui/colors';

const vertexShader = /* glsl */ `
  varying vec3 vWorld;
  void main() {
    vec4 world = modelMatrix * vec4(position, 1.0);
    vWorld = world.xyz;
    gl_Position = projectionMatrix * viewMatrix * world;
  }
`;

// Anti-aliased procedural grid: 0.1 m minor, 1 m major, 5 m section lines, X/Z axes,
// all fading with distance so the floor has no visible edge.
const fragmentShader = /* glsl */ `
  uniform vec3 uMinor;
  uniform vec3 uMajor;
  uniform vec3 uSection;
  uniform vec3 uAxisX;
  uniform vec3 uAxisZ;
  uniform vec3 uFog;
  uniform float uFadeDistance;
  varying vec3 vWorld;

  // 1 on the line, 0 elsewhere; width in pixels.
  float lines(vec2 p, float spacing, float width) {
    vec2 c = p / spacing;
    vec2 d = fwidth(c);
    vec2 g = abs(fract(c - 0.5) - 0.5) / d;
    float l = min(g.x, g.y);
    // Hide lines that got denser than ~4 px apart instead of letting them moiré.
    float density = 1.0 - smoothstep(0.08, 0.25, max(d.x, d.y));
    return (1.0 - min(l / width, 1.0)) * density;
  }

  float axis(float coord, float width) {
    float d = fwidth(coord);
    return 1.0 - min(abs(coord) / (d * width), 1.0);
  }

  void main() {
    vec2 p = vWorld.xz;
    float dist = length(cameraPosition.xz - p);

    float minor = lines(p, 0.1, 1.0);
    float major = lines(p, 1.0, 1.2);
    float section = lines(p, 5.0, 1.6);

    vec3 color = uFog;
    float alpha = 0.0;
    color = mix(color, uMinor, minor * 0.7); alpha = max(alpha, minor * 0.55);
    color = mix(color, uMajor, major);       alpha = max(alpha, major * 0.85);
    color = mix(color, uSection, section);   alpha = max(alpha, section);

    float ax = axis(p.y, 2.0); // line z = 0 -> X axis
    float az = axis(p.x, 2.0); // line x = 0 -> Z axis
    color = mix(color, uAxisX, ax); alpha = max(alpha, ax);
    color = mix(color, uAxisZ, az); alpha = max(alpha, az);

    float fade = 1.0 - smoothstep(uFadeDistance * 0.35, uFadeDistance, dist);
    gl_FragColor = vec4(color, alpha * fade);
    #include <colorspace_fragment>
  }
`;

export function Grid({ fadeDistance = 60 }: { fadeDistance?: number }) {
  const uniforms = useMemo(
    () => ({
      uMinor: { value: new THREE.Color(SCENE.gridMinor) },
      uMajor: { value: new THREE.Color(SCENE.gridMajor) },
      uSection: { value: new THREE.Color(SCENE.gridSection) },
      uAxisX: { value: new THREE.Color(SCENE.axisX) },
      uAxisZ: { value: new THREE.Color(SCENE.axisZ) },
      uFog: { value: new THREE.Color(SCENE.background) },
      uFadeDistance: { value: fadeDistance },
    }),
    [fadeDistance],
  );

  return (
    <mesh rotation-x={-Math.PI / 2} position-y={0.0005} renderOrder={-1}>
      <planeGeometry args={[fadeDistance * 4, fadeDistance * 4]} />
      <shaderMaterial
        vertexShader={vertexShader}
        fragmentShader={fragmentShader}
        uniforms={uniforms}
        transparent
        depthWrite={false}
      />
    </mesh>
  );
}
