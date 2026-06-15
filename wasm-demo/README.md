# step2glb · wasm demo

A minimal Vite page that converts a STEP file to GLB **entirely in the browser**
and renders it, using the `step2glb-wasm` build of the core converter.

## What it shows

The data path mirrors the intended production flow:

1. **Main thread** writes the uploaded STEP to OPFS with an async writable
   stream (no sync handle needed here) under a `crypto.randomUUID()` name, then
   posts the **input + output paths** to the worker.
2. **Web Worker** — the only place sync handles exist — opens an OPFS
   `createSyncAccessHandle` on the input path, reads it **synchronously**, runs
   the **synchronous** wasm converter (`convert_step_to_glb`), and writes the
   GLB to the output path through another sync handle. It returns the output
   path plus a **JSON diagnostics report**.
3. **Main thread** reads that OPFS GLB, renders it with `<model-viewer>`, shows
   the report, and offers a download. Scratch files are deleted **on error** and
   the output is deleted **once the user has downloaded** it (UUID names avoid
   collisions; a `beforeunload` sweep drops any leftovers).

The report surfaces **issues and defaults used**: faces ok / skipped /
degenerate, color-mesh count, unsupported surface / curve / item types,
plane-approximated surfaces, parser warnings, and notably whether **no length
unit was declared (millimetres assumed)** plus the deflection used.

The bundled sample (`src/sample-box-cylinder.stp`) is a `CSG_SOLID`: a 10×10×10
block with a Ø6 cylinder drilled through it (exercises the BSP mesh boolean).
You can also pick any `.step`/`.stp` file with the file input.

## Prerequisites

- Rust with the `wasm32-unknown-unknown` target:
  `rustup target add wasm32-unknown-unknown`
- [`wasm-pack`](https://rustwasm.github.io/wasm-pack/): `cargo install wasm-pack`
- Node.js 18+ and npm.

## Run

```sh
cd wasm-demo
npm install
npm run dev      # builds the wasm (wasm-pack) then starts Vite
```

Open the printed URL, click **Convert sample**. `npm run build:wasm` alone
regenerates `pkg/` (the wasm-bindgen glue + `.wasm`) after changing the Rust.

## Notes

- OPFS sync access handles need a **Worker** and a **secure context**
  (`localhost` counts); Vite's dev server is fine. They do **not** require
  COOP/COEP headers (unlike `SharedArrayBuffer`).
- The browser build drops the meshoptimizer pass (`step2glb-core` is built with
  `--no-default-features`), so meshes are valid but not weld-optimized.
- This demo passes whole-file bytes to wasm. The deeper design — OPFS-backed
  `InputHandle` / `TempHandle` read on demand *inside* Rust, for models larger
  than tab memory — is the next streaming increment (see the repo `CLAUDE.md`).

## Status

The Rust wasm crate compiles to `wasm32-unknown-unknown`. The page/worker were
authored against the wasm-pack `--target web` output and the OPFS sync-handle
API; they have **not been run in this environment** (no Node/wasm-pack/browser
here). Run the steps above to build and serve.
