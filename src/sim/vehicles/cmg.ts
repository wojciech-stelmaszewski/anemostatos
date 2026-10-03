/**
 * A pyramid of four single-gimbal control-moment gyros (lesson IV.23) [Wie 2008, §7.3]. Each rotor
 * spins with momentum h₀; its gimbal turns that momentum in a plane tilted by the skew angle β.
 * The cluster's momentum in the body frame is
 *
 *   h = h₀ · ( −cβ·sδ₁ − cδ₂ + cβ·sδ₃ + cδ₄,
 *               cδ₁ − cβ·sδ₂ − cδ₃ + cβ·sδ₄,
 *               sβ·(sδ₁ + sδ₂ + sδ₃ + sδ₄) )
 *
 * and turning the gimbals at δ̇ changes it at ḣ = A(δ)·δ̇, A = ∂h/∂δ (3 × 4). The torque on the
 * body is −ḣ: small gimbal rates steer a large momentum, which is why CMGs give far more torque
 * than wheels. Where A loses rank (det AAᵀ = 0) no gimbal rate gives torque along one direction.
 */
export interface CmgParams {
  /** Momentum of each rotor, N·m·s. */
  h0: number;
  /** Skew of the pyramid's faces, °: 54.74° (cos β = 1/√3) makes the envelope nearly spherical. */
  skewDeg: number;
  /** Largest gimbal rate, °/s. */
  rateMaxDeg: number;
  /**
   * How gimbal rates are found from the torque wanted: the Moore–Penrose pseudoinverse
   * Aᵀ(AAᵀ)⁻¹ (`pinv`), the singularity-robust inverse Aᵀ(AAᵀ + λI)⁻¹ (`sr`), or the same with
   * a slowly turning off-diagonal dither in place of I (`gsr`), which can leave a singularity.
   */
  steering: 'pinv' | 'sr' | 'gsr';
  /**
   * Where the gimbals rest at zero momentum, °: δ = (s, −s, s, −s) holds no momentum for any s, so
   * moving along this family (null motion) changes nothing the body feels, but it changes which
   * singularities lie ahead.
   */
  parkDeg: number;
  /** SR: λ = lambda·e^(−10·m), m the singularity measure; GSR: the dither's amplitude. */
  lambda: number;
  dither: number;
}

/** The parked gimbal angles, rad: a zero-momentum configuration. */
export const parkedGimbals = (c: CmgParams): number[] => {
  const s = (c.parkDeg * Math.PI) / 180;
  return [s, -s, s, -s];
};

/** The cluster's momentum, N·m·s, body frame. */
export function cmgMomentum(d: readonly number[], c: CmgParams): [number, number, number] {
  const b = (c.skewDeg * Math.PI) / 180;
  const cb = Math.cos(b);
  const sb = Math.sin(b);
  const [c1, c2, c3, c4] = d.map(Math.cos) as [number, number, number, number];
  const [s1, s2, s3, s4] = d.map(Math.sin) as [number, number, number, number];
  return [
    c.h0 * (-cb * s1 - c2 + cb * s3 + c4),
    c.h0 * (c1 - cb * s2 - c3 + cb * s4),
    c.h0 * sb * (s1 + s2 + s3 + s4),
  ];
}

/** A = ∂h/∂δ, rows x, y, z, columns the four gimbals, N·m·s per rad. */
export function cmgJacobian(d: readonly number[], c: CmgParams): number[][] {
  const b = (c.skewDeg * Math.PI) / 180;
  const cb = Math.cos(b);
  const sb = Math.sin(b);
  const [c1, c2, c3, c4] = d.map(Math.cos) as [number, number, number, number];
  const [s1, s2, s3, s4] = d.map(Math.sin) as [number, number, number, number];
  return [
    [-cb * c1, s2, cb * c3, -s4].map((v) => v * c.h0),
    [-s1, -cb * c2, s3, cb * c4].map((v) => v * c.h0),
    [sb * c1, sb * c2, sb * c3, sb * c4].map((v) => v * c.h0),
  ];
}

/** AAᵀ, 3 × 3. */
export function gram(A: number[][]): number[][] {
  return [0, 1, 2].map((i) =>
    [0, 1, 2].map((j) => A[i]!.reduce((s, v, n) => s + v * A[j]![n]!, 0)),
  );
}

export function det3(m: number[][]): number {
  const [a, b, c] = m[0]! as [number, number, number];
  const [d, e, f] = m[1]! as [number, number, number];
  const [g, h, i] = m[2]! as [number, number, number];
  return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
}

/** Solve a 3 × 3 system by Cramer's rule. */
export function solve3(m: number[][], r: number[]): number[] {
  const d = det3(m);
  return [0, 1, 2].map(
    (j) => det3(m.map((row, i) => row.map((v, k) => (k === j ? r[i]! : v)))) / d,
  );
}

/** The singularity measure m = det(AAᵀ)/h₀⁶: 0 at a singularity, about 1 to 2 far from one. */
export const singularity = (d: readonly number[], c: CmgParams): number =>
  det3(gram(cmgJacobian(d, c))) / c.h0 ** 6;

/**
 * The most momentum the pyramid can hold along a body axis in its base plane (x or y): every
 * rotor turned as far toward it as its gimbal allows, h₀·(2 + 2·cos β). This is the envelope, a
 * saturation singularity no steering law can pass.
 */
export const envelopeX = (c: CmgParams): number =>
  c.h0 * (2 + 2 * Math.cos((c.skewDeg * Math.PI) / 180));

/**
 * The internal singularity met when momentum is taken along x from the zero-momentum state
 * δ = 0: two rotors cancel and the other two lie in a plane with x, h₀·2·cos β.
 */
export const internalX = (c: CmgParams): number => c.h0 * 2 * Math.cos((c.skewDeg * Math.PI) / 180);
