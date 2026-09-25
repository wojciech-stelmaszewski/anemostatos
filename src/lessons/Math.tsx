import katex from 'katex';
import { useMemo } from 'react';

/** Inline (or display) KaTeX formula. */
export function M({ children, display = false }: { children: string; display?: boolean }) {
  const html = useMemo(
    () => katex.renderToString(children, { displayMode: display, throwOnError: false }),
    [children, display],
  );
  return (
    <span className={display ? 'my-1 block' : ''} dangerouslySetInnerHTML={{ __html: html }} />
  );
}
