import { ChevronRight } from 'lucide-react';
import { Collapsible } from 'radix-ui';
import { useState, type ReactNode } from 'react';

export function Section({
  title,
  children,
  defaultOpen = false,
  accent,
}: {
  title: ReactNode;
  children: ReactNode;
  defaultOpen?: boolean;
  accent?: string;
}) {
  const [open, setOpen] = useState(defaultOpen);
  return (
    <Collapsible.Root open={open} onOpenChange={setOpen} className="border-b border-border">
      <Collapsible.Trigger className="flex w-full items-center gap-2 px-3 py-2 text-left text-xs font-semibold uppercase tracking-wider text-muted hover:text-fg">
        <ChevronRight className={`size-3.5 transition-transform ${open ? 'rotate-90' : ''}`} />
        {accent && <span className="size-2 rounded-full" style={{ background: accent }} />}
        {title}
      </Collapsible.Trigger>
      <Collapsible.Content className="space-y-2 px-3 pb-3">{children}</Collapsible.Content>
    </Collapsible.Root>
  );
}
