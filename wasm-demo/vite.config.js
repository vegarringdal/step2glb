import { defineConfig } from 'vite';

// OPFS *synchronous* access handles work inside a Web Worker without
// cross-origin isolation (unlike SharedArrayBuffer), so no COOP/COEP headers
// are required. Vite (and GitHub Pages) serve .wasm with the right MIME type
// out of the box, so the wasm never needs inlining.
export default defineConfig({
  // GitHub Pages serves a project site under /<repo>/, so assets, the worker
  // chunk and the .wasm it fetches all need that prefix. Set VITE_BASE at build
  // time (the Pages workflow derives it from the repo name); defaults to '/'
  // for local dev / a user-or-org root site.
  base: process.env.VITE_BASE || '/',
  // the generated wasm-bindgen glue + .wasm live in ./pkg
  server: { fs: { allow: ['..'] } },
  optimizeDeps: { exclude: ['../pkg/step2glb_wasm.js'] },
  // the worker imports the wasm-bindgen ES module, so it must be emitted as an
  // ES-module worker in the production build (its `import.meta.url` then
  // resolves the .wasm under `base` automatically — no inlining needed).
  worker: { format: 'es' },
  // the demo targets very modern Chromium only (OPFS sync handles), so skip
  // any down-levelling of the wasm glue / modern syntax.
  build: { target: 'esnext' },
});
