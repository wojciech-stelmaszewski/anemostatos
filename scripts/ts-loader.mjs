// Resolve the project's TypeScript sources in plain Node (no dependencies): the `@/` alias and
// extensionless relative imports → `.ts` files. Used by `make bench` and `make train`, together
// with Node's built-in type stripping (--experimental-transform-types).
import { existsSync } from 'node:fs';
import { fileURLToPath, pathToFileURL } from 'node:url';

const src = new URL('../src/', import.meta.url);

export async function resolve(specifier, context, next) {
  let target = null;
  if (specifier.startsWith('@/')) target = new URL(specifier.slice(2), src);
  else if (
    (specifier.startsWith('./') || specifier.startsWith('../')) &&
    context.parentURL?.endsWith('.ts')
  )
    target = new URL(specifier, context.parentURL);
  if (target) {
    const path = fileURLToPath(target);
    for (const candidate of [path, `${path}.ts`, `${path}/index.ts`]) {
      if (existsSync(candidate) && !candidate.endsWith('/')) {
        if (candidate === path && !/\.[cm]?[jt]s$/.test(path)) continue;
        return next(pathToFileURL(candidate).href, context);
      }
    }
  }
  return next(specifier, context);
}
