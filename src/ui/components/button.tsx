import { cva, type VariantProps } from 'class-variance-authority';
import { Slot } from 'radix-ui';
import type { ComponentProps } from 'react';
import { cn } from '@/ui/cn';

const buttonVariants = cva(
  'inline-flex items-center justify-center gap-1.5 whitespace-nowrap rounded-md text-xs font-medium transition-colors focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-accent/60 disabled:pointer-events-none disabled:opacity-40 [&_svg]:size-3.5 [&_svg]:shrink-0',
  {
    variants: {
      variant: {
        default: 'bg-panel-2 text-fg border border-border hover:bg-border/70',
        accent: 'bg-accent text-black hover:bg-accent/85',
        ghost: 'text-muted hover:text-fg hover:bg-panel-2',
        active: 'bg-accent/15 text-accent border border-accent/40',
      },
      size: {
        sm: 'h-7 px-2.5',
        icon: 'h-7 w-7',
      },
    },
    defaultVariants: { variant: 'default', size: 'sm' },
  },
);

export function Button({
  className,
  variant,
  size,
  asChild = false,
  ...props
}: ComponentProps<'button'> & VariantProps<typeof buttonVariants> & { asChild?: boolean }) {
  const Comp = asChild ? Slot.Root : 'button';
  return <Comp className={cn(buttonVariants({ variant, size }), className)} {...props} />;
}
