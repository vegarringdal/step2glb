import { defineConfig } from 'vite';

// OPFS *synchronous* access handles work inside a Web Worker without
// cross-origin isolation (unlike SharedArrayBuffer), so no COOP/COEP headers
// are required. Vite serves .wasm with the right MIME type out of the box.
export default defineConfig({
  // the generated wasm-bindgen glue + .wasm live in ./pkg
  server: { fs: { allow: ['..'] } },
  optimizeDeps: { exclude: ['../pkg/step2glb_wasm.js'] },
});
