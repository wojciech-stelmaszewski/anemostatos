import * as THREE from 'three';
import { SCENE } from '@/ui/colors';

/** Procedural 2×2 twill carbon weave, tiled across the frame parts. */
function carbonTexture(): THREE.Texture {
  const size = 64;
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = size;
  const g = canvas.getContext('2d')!;
  const cell = size / 4;
  for (let y = 0; y < 4; y++) {
    for (let x = 0; x < 4; x++) {
      const horizontal = (x + y) % 4 < 2;
      const grad = horizontal
        ? g.createLinearGradient(x * cell, 0, (x + 1) * cell, 0)
        : g.createLinearGradient(0, y * cell, 0, (y + 1) * cell);
      grad.addColorStop(0, '#15181d');
      grad.addColorStop(0.5, '#2c313a');
      grad.addColorStop(1, '#15181d');
      g.fillStyle = grad;
      g.fillRect(x * cell, y * cell, cell, cell);
    }
  }
  const tex = new THREE.CanvasTexture(canvas);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.repeat.set(6, 1);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 8;
  return tex;
}

let cache: ReturnType<typeof create> | null = null;

function create() {
  return {
    carbon: new THREE.MeshPhysicalMaterial({
      map: carbonTexture(),
      roughness: 0.35,
      metalness: 0.15,
      clearcoat: 0.6,
      clearcoatRoughness: 0.25,
    }),
    canopy: new THREE.MeshPhysicalMaterial({
      color: SCENE.accent,
      roughness: 0.32,
      metalness: 0.0,
      clearcoat: 1,
      clearcoatRoughness: 0.15,
    }),
    darkPlastic: new THREE.MeshStandardMaterial({ color: '#1c2027', roughness: 0.55 }),
    battery: new THREE.MeshStandardMaterial({ color: '#2a3446', roughness: 0.5, metalness: 0.1 }),
    strap: new THREE.MeshStandardMaterial({ color: '#0e1014', roughness: 0.9 }),
    metal: new THREE.MeshStandardMaterial({ color: '#c9ced6', roughness: 0.28, metalness: 0.95 }),
    darkMetal: new THREE.MeshStandardMaterial({
      color: '#3a3f47',
      roughness: 0.35,
      metalness: 0.9,
    }),
    lens: new THREE.MeshPhysicalMaterial({
      color: '#05070a',
      roughness: 0.05,
      metalness: 0.2,
      clearcoat: 1,
    }),
    rubber: new THREE.MeshStandardMaterial({ color: '#121418', roughness: 0.95 }),
    ledFront: new THREE.MeshBasicMaterial({ color: '#dff1ff', toneMapped: false }),
    ledRear: new THREE.MeshBasicMaterial({ color: '#ff2d3d', toneMapped: false }),
  };
}

export const droneMaterials = () => (cache ??= create());
