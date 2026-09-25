import * as THREE from 'three';

const shaftGeo = new THREE.CylinderGeometry(1, 1, 1, 10).translate(0, 0.5, 0);
const headGeo = new THREE.ConeGeometry(1, 1, 14).translate(0, 0.5, 0);
const UP = new THREE.Vector3(0, 1, 0);

/** A 3D arrow that can be re-aimed every frame without allocations. */
export class Arrow extends THREE.Group {
  private shaft: THREE.Mesh;
  private head: THREE.Mesh;
  readonly material: THREE.MeshBasicMaterial;
  private dir = new THREE.Vector3();

  constructor(
    color: string,
    public radius = 0.008,
  ) {
    super();
    this.material = new THREE.MeshBasicMaterial({
      color,
      transparent: true,
      opacity: 0.95,
      depthTest: true,
    });
    this.shaft = new THREE.Mesh(shaftGeo, this.material);
    this.head = new THREE.Mesh(headGeo, this.material);
    this.add(this.shaft, this.head);
    this.renderOrder = 2;
  }

  /** Point from `origin` along `vec` (world units). Hidden when shorter than `minLength`. */
  set(origin: THREE.Vector3, vec: THREE.Vector3, minLength = 0.01): void {
    const len = vec.length();
    this.visible = len >= minLength;
    if (!this.visible) return;
    this.position.copy(origin);
    this.dir.copy(vec).divideScalar(len);
    this.quaternion.setFromUnitVectors(UP, this.dir);
    const headLen = Math.min(len * 0.35, this.radius * 7);
    const r = this.radius;
    this.shaft.scale.set(r, Math.max(len - headLen, 1e-4), r);
    this.head.position.y = len - headLen;
    this.head.scale.set(r * 2.6, headLen, r * 2.6);
  }
}
