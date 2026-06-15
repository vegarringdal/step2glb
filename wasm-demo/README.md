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

### Controls

Effective in the browser build today:

- **deflection** (slider, mm) — chordal tolerance; coarser = fewer triangles = faster.
- **max-angle** (slider, 10–45°) — max chord turn angle; smaller = rounder curves.
- **Y-up** — rotate Z-up (STEP) to glTF Y-up.
- **normals** — keep per-vertex normals (off = smaller, viewer flat-shades).
- **merged** (off by default) — one node/mesh per color baked to world space vs
  the hierarchical per-part node tree (instanced, keeps the assembly structure).
  Merged accumulates one buffer per color and **can't stream**, so it always
  runs in RAM and **disables the memory slider** — leave it off for large files.
- **cleanup** — rvm-style position weld (drops normals). The mesh *simplify*
  step needs meshoptimizer, which the wasm build omits, so in the browser this
  is weld + degenerate-drop only.
- **render result** (on by default) — load the GLB into the viewer. Turn it
  **off** to measure the converter's memory in isolation: model-viewer decodes
  the glTF and uploads it to the GPU, which adds a chunk of memory *after* the
  conversion. The GLB is still produced and **Export GLB** still works.
- **memory** (slider, 100–2000 MB) — the ceiling that picks the path in
  hierarchical mode: a file **larger** than this streams through OPFS sync
  handles (input read by range, and each mesh's geometry spilled to OPFS as it
  is tessellated — low, bounded wasm memory); **smaller** files convert all in
  RAM (faster). The status line shows which path ran. Disabled under **merged**
  (which never streams).
- **progress** — during a streaming conversion Rust reports product-node
  progress (throttled ~5%); the worker `postMessage`s it and the status line
  shows `N%`.
- **Export GLB** — download the result; the OPFS scratch is then deleted.
- starting a new conversion clears the previous model from the canvas.

How the streaming path maps to the core's three sync handles: the worker builds
an `Io` object whose `read`/`writeOutput`/`writeTemp`+`readTemp`/`progress`
methods drive OPFS sync access handles, and `convert_streaming` wires them to
Rust's `InputHandle` / `OutputHandle` / `TempHandle`. (Full memory *budgeting* —
splitting the ceiling across input window / output / tessellation — is the
remaining refinement; today the choice is the binary stream-or-not above.)

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
- The picked file is **streamed to OPFS** (`blob.stream().pipeTo(writable)`),
  so it never sits whole in main-thread memory. The **streaming** path
  (`convert_streaming`, file > ceiling) then reads it back from OPFS **by range
  on demand inside Rust** — the offset index is built with a sliding window and
  entity bytes are pulled per range, so the input is never fully materialized in
  wasm memory either. The **in-RAM** path (`convert_step_to_glb`, file ≤ ceiling
  or merged) does load the whole file into wasm memory — by design, for speed on
  small files. The resident floor in the streaming path is the offset index
  (∝ entity count) plus one entity + one mesh.

## Status

The Rust wasm crate compiles to `wasm32-unknown-unknown`. The page/worker were
authored against the wasm-pack `--target web` output and the OPFS sync-handle
API; they have **not been run in this environment** (no Node/wasm-pack/browser
here). Run the steps above to build and serve.
