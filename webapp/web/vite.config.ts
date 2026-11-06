import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// `npm run dev` proxies to a hub running on this machine.
export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      '/api': 'http://localhost:8080',
      '/ws': { target: 'ws://localhost:8080', ws: true },
    },
  },
});
