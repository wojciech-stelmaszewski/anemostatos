import { fileURLToPath, URL } from 'node:url';
import react from '@vitejs/plugin-react';
import tailwindcss from '@tailwindcss/vite';
import { defineConfig } from 'vitest/config';

export default defineConfig({
  plugins: [react(), tailwindcss()],
  resolve: {
    alias: { '@': fileURLToPath(new URL('./src', import.meta.url)) },
  },
  // Three.js alone is ~700 kB; this is a local app, so one chunk is fine.
  build: { chunkSizeWarningLimit: 2000 },
  test: {
    include: ['tests/**/*.test.ts'],
    environment: 'node',
    // Lesson tests fly tens of simulated seconds of MPC, Monte Carlo campaigns and sweeps. Alone
    // the slowest takes about 15 s; on a loaded machine, with files in parallel, several times that.
    // A timeout is not a failed assertion, so the limit is generous.
    testTimeout: 120_000,
  },
});
