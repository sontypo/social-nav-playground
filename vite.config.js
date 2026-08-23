import { resolve } from 'path';
import { defineConfig } from 'vite';

export default defineConfig({
  base: './', // Relative paths for GitHub Pages
  server: {
    port: 5173,
    host: '0.0.0.0',
    watch: {
      usePolling: true,
      interval: 1000,
      ignored: ['**/dist/**', '**/node_modules/**', '**/.git/**']
    }
  },
  build: {
    outDir: 'dist',
    assetsDir: 'assets',
    sourcemap: false,
    rollupOptions: {
      input: {
        main: resolve(__dirname, 'index.html'),
        live: resolve(__dirname, 'live.html')
      }
    }
  }
});
