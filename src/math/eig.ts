// Eigenvalues of a real, non-symmetric matrix: reduction to Hessenberg form by elimination,
// then the shifted QR algorithm with double (Francis) steps. The classical EISPACK pair
// elmhes + hqr, which stays accurate where a characteristic polynomial does not.
import type { Complex } from './complex';

const EPS = 2.220446049250313e-16;

/** Eigenvalues of the square matrix `m` (not modified). Conjugate pairs come out adjacent. */
export function eigQr(m: readonly (readonly number[])[]): Complex[] {
  const n = m.length;
  if (n === 0) return [];
  // 1-indexed working copy, as in the reference algorithm.
  const a: number[][] = Array.from({ length: n + 1 }, (_, i) =>
    i === 0 ? [] : [0, ...m[i - 1]!.slice(0, n)],
  );
  hessenberg(a, n);
  return hqr(a, n);
}

function hessenberg(a: number[][], n: number): void {
  for (let m = 2; m < n; m++) {
    let x = 0;
    let i = m;
    for (let j = m; j <= n; j++) {
      if (Math.abs(a[j]![m - 1]!) > Math.abs(x)) {
        x = a[j]![m - 1]!;
        i = j;
      }
    }
    if (i !== m) {
      for (let j = m - 1; j <= n; j++) [a[i]![j], a[m]![j]] = [a[m]![j]!, a[i]![j]!];
      for (let j = 1; j <= n; j++) [a[j]![i], a[j]![m]] = [a[j]![m]!, a[j]![i]!];
    }
    if (x !== 0) {
      for (let r = m + 1; r <= n; r++) {
        let y = a[r]![m - 1]!;
        if (y !== 0) {
          y /= x;
          a[r]![m - 1] = y;
          for (let j = m; j <= n; j++) a[r]![j] = a[r]![j]! - y * a[m]![j]!;
          for (let j = 1; j <= n; j++) a[j]![m] = a[j]![m]! + y * a[j]![r]!;
        }
      }
    }
  }
  // The elimination left its multipliers below the subdiagonal; they are not part of the matrix.
  for (let r = 3; r <= n; r++) for (let j = 1; j <= r - 2; j++) a[r]![j] = 0;
}

const sign = (x: number, s: number): number => (s >= 0 ? Math.abs(x) : -Math.abs(x));

function hqr(a: number[][], n: number): Complex[] {
  const wr = new Array<number>(n + 1).fill(0);
  const wi = new Array<number>(n + 1).fill(0);
  let anorm = 0;
  for (let i = 1; i <= n; i++)
    for (let j = Math.max(i - 1, 1); j <= n; j++) anorm += Math.abs(a[i]![j]!);
  let nn = n;
  let t = 0;
  let p = 0;
  let q = 0;
  let r = 0;
  while (nn >= 1) {
    let its = 0;
    let l: number;
    do {
      // Look for a single small subdiagonal element.
      for (l = nn; l >= 2; l--) {
        let s = Math.abs(a[l - 1]![l - 1]!) + Math.abs(a[l]![l]!);
        if (s === 0) s = anorm;
        if (Math.abs(a[l]![l - 1]!) <= EPS * s) {
          a[l]![l - 1] = 0;
          break;
        }
      }
      let x = a[nn]![nn]!;
      if (l === nn) {
        // One real root found.
        wr[nn] = x + t;
        wi[nn--] = 0;
      } else {
        let y = a[nn - 1]![nn - 1]!;
        let w = a[nn]![nn - 1]! * a[nn - 1]![nn]!;
        if (l === nn - 1) {
          // Two roots found: a real pair or a complex pair.
          p = 0.5 * (y - x);
          q = p * p + w;
          let z = Math.sqrt(Math.abs(q));
          x += t;
          if (q >= 0) {
            z = p + sign(z, p);
            wr[nn - 1] = wr[nn] = x + z;
            if (z !== 0) wr[nn] = x - w / z;
            wi[nn - 1] = wi[nn] = 0;
          } else {
            wr[nn - 1] = wr[nn] = x + p;
            wi[nn] = -z;
            wi[nn - 1] = z;
          }
          nn -= 2;
        } else {
          if (its === 60) throw new Error('eigQr: no convergence');
          if (its === 10 || its === 20 || its === 40) {
            // Exceptional shift.
            t += x;
            for (let i = 1; i <= nn; i++) a[i]![i] = a[i]![i]! - x;
            const s = Math.abs(a[nn]![nn - 1]!) + Math.abs(a[nn - 1]![nn - 2]!);
            y = x = 0.75 * s;
            w = -0.4375 * s * s;
          }
          ++its;
          let m: number;
          let z: number;
          for (m = nn - 2; m >= l; m--) {
            z = a[m]![m]!;
            r = x - z;
            let s = y - z;
            p = (r * s - w) / a[m + 1]![m]! + a[m]![m + 1]!;
            q = a[m + 1]![m + 1]! - z - r - s;
            r = a[m + 2]![m + 1]!;
            s = Math.abs(p) + Math.abs(q) + Math.abs(r);
            p /= s;
            q /= s;
            r /= s;
            if (m === l) break;
            const u = Math.abs(a[m]![m - 1]!) * (Math.abs(q) + Math.abs(r));
            const v =
              Math.abs(p) *
              (Math.abs(a[m - 1]![m - 1]!) + Math.abs(z) + Math.abs(a[m + 1]![m + 1]!));
            if (u <= EPS * v) break;
          }
          for (let i = m + 2; i <= nn; i++) {
            a[i]![i - 2] = 0;
            if (i !== m + 2) a[i]![i - 3] = 0;
          }
          // Double QR step on rows l…nn and columns m…nn.
          for (let k = m; k <= nn - 1; k++) {
            if (k !== m) {
              p = a[k]![k - 1]!;
              q = a[k + 1]![k - 1]!;
              r = k !== nn - 1 ? a[k + 2]![k - 1]! : 0;
              x = Math.abs(p) + Math.abs(q) + Math.abs(r);
              if (x !== 0) {
                p /= x;
                q /= x;
                r /= x;
              }
            }
            const s = sign(Math.sqrt(p * p + q * q + r * r), p);
            if (s !== 0) {
              if (k === m) {
                if (l !== m) a[k]![k - 1] = -a[k]![k - 1]!;
              } else {
                a[k]![k - 1] = -s * x;
              }
              p += s;
              x = p / s;
              y = q / s;
              z = r / s;
              q /= p;
              r /= p;
              for (let j = k; j <= nn; j++) {
                p = a[k]![j]! + q * a[k + 1]![j]!;
                if (k !== nn - 1) {
                  p += r * a[k + 2]![j]!;
                  a[k + 2]![j] = a[k + 2]![j]! - p * z;
                }
                a[k + 1]![j] = a[k + 1]![j]! - p * y;
                a[k]![j] = a[k]![j]! - p * x;
              }
              const mmin = nn < k + 3 ? nn : k + 3;
              for (let i = l; i <= mmin; i++) {
                p = x * a[i]![k]! + y * a[i]![k + 1]!;
                if (k !== nn - 1) {
                  p += z * a[i]![k + 2]!;
                  a[i]![k + 2] = a[i]![k + 2]! - p * r;
                }
                a[i]![k + 1] = a[i]![k + 1]! - p * q;
                a[i]![k] = a[i]![k]! - p;
              }
            }
          }
        }
      }
    } while (l < nn - 1);
  }
  const out: Complex[] = [];
  for (let i = 1; i <= n; i++) out.push({ re: wr[i]!, im: wi[i]! });
  return out;
}
