export function App() {
  return (
    <div className="grid h-full grid-cols-[1fr_340px] grid-rows-[44px_1fr_300px]">
      <header className="col-span-2 flex items-center gap-3 border-b border-border bg-panel px-4">
        <span className="font-semibold tracking-wide">
          Anemostatos <span className="text-muted font-normal">· PID playground</span>
        </span>
      </header>
      <main className="relative bg-bg" />
      <aside className="row-span-2 border-l border-border bg-panel" />
      <section className="border-t border-border bg-panel" />
    </div>
  );
}
