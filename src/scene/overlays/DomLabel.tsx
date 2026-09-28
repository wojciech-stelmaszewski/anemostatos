import { useFrame, useThree } from '@react-three/fiber';
import { useEffect, useMemo, useRef, type CSSProperties, type RefObject } from 'react';
import * as THREE from 'three';

const tmp = new THREE.Vector3();

interface DomLabelProps {
  /** Offset from the parent object, world units. */
  position?: [number, number, number];
  className?: string;
  style?: CSSProperties;
  /** Static text; for live text, write to `elRef.current.textContent` instead. */
  text?: string;
  /** Centre the label on its point instead of hanging it to the right/below. */
  center?: boolean;
  elRef?: RefObject<HTMLDivElement | null>;
}

/**
 * A text label pinned to a point in the 3D scene: one plain DOM element over the canvas,
 * moved every frame by projecting the point to the screen. (Replaces drei's <Html>, which mounts
 * a separate React root per label and unmounts it synchronously during rendering — React 19 warns
 * about that.) Labels never take pointer events.
 */
export function DomLabel({
  position = [0, 0, 0],
  className,
  style,
  text,
  center,
  elRef,
}: DomLabelProps) {
  const anchor = useRef<THREE.Group>(null);
  const gl = useThree((s) => s.gl);
  const camera = useThree((s) => s.camera);
  const size = useThree((s) => s.size);
  const el = useMemo(() => {
    const d = document.createElement('div');
    d.style.position = 'absolute';
    d.style.left = '0';
    d.style.top = '0';
    d.style.pointerEvents = 'none';
    d.style.willChange = 'transform';
    return d;
  }, []);

  useEffect(() => {
    const host = gl.domElement.parentElement;
    if (!host) return;
    if (getComputedStyle(host).position === 'static') host.style.position = 'relative';
    host.appendChild(el);
    if (elRef) elRef.current = el;
    return () => {
      el.remove();
      if (elRef) elRef.current = null;
    };
  }, [gl, el, elRef]);

  useEffect(() => {
    el.className = className ?? '';
    Object.assign(el.style, style);
    if (text !== undefined) el.textContent = text;
  }, [el, className, style, text]);

  useFrame(() => {
    const a = anchor.current;
    if (!a) return;
    let shown = true;
    for (let o: THREE.Object3D | null = a; o; o = o.parent) if (!o.visible) shown = false;
    a.getWorldPosition(tmp).project(camera);
    if (tmp.z > 1 || tmp.z < -1) shown = false; // behind the camera or beyond the far plane
    el.style.display = shown ? '' : 'none';
    if (!shown) return;
    const x = ((tmp.x + 1) / 2) * size.width;
    const y = ((1 - tmp.y) / 2) * size.height;
    el.style.transform = `translate(${x.toFixed(1)}px, ${y.toFixed(1)}px)${center ? ' translate(-50%, -50%)' : ''}`;
  });

  return <group ref={anchor} position={position} />;
}
