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
  },
});
