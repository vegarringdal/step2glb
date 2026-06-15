# Coding style

- You (Claude) are the expert in coding; Vegar is the director. Make the
  engineering calls and recommend the right approach — don't defer decisions
  that are yours to make. The director sets priorities and goals.
- Always run the tests and `cargo fmt --all`, plus a release compile/test check,
  before declaring anything done. Don't report "done" on unverified work.
- Never guess. Verify facts against the code and parameterizations against the
  authoritative specs (ISO 10303-42/-21/-11/-42/-43, NURBS conventions) and
  reputable web sources before implementing. Prefer spec-correct, structural
  solutions over tuned magic numbers.
- WASM will need a Web Worker, and must use **synchronous file handles only**
  (OPFS `createSyncAccessHandle()`), so the CPU-bound core stays synchronous and
  is never `async`-colored.
- Ship a **minimal WASM sample as an HTML page** that converts a dumb STEP
  box / CSG cylinder (etc.) in-browser end-to-end, as the smoke test for the
  wasm build.

# Work in progress

Goal: handle models that don't fit in RAM (input and output), and run the same
core in the browser. Taken in stages; the unifying idea is a small set of
**synchronous I/O handles** injected into the core so it never knows whether it
is talking to a `Vec`, an mmap, a temp file, or OPFS.

## Crate split (workspace) — stage 1, the prerequisite

| Crate | Type | Contents |
| --- | --- | --- |
| `step2glb-core` | `rlib` | everything in `src/` except `main.rs` (step, geom, model, tessellate, csg, hierarchy, styles, merge, mesh, glb, io). No `clap`. `meshopt` behind a default-on `optimize` feature so wasm/capi can drop the C++ toolchain. |
| `step2glb-cli` | `bin` | `main.rs` + `clap` + file/mmap/tempfile I/O. |
| `step2glb-wasm` | `cdylib` | wasm-bindgen shell + OPFS sync-handle glue. `core` with `default-features = false`. Runs in a Web Worker. |
| `step2glb-capi` | `cdylib` + `staticlib` | thin C ABI over `core` (kept separate so `core` stays a clean `rlib`). |

## The three sync handles (live in `core::io`)

```rust
/// Random-access read source — the STEP input. Read-only.
pub trait InputHandle {
    fn size(&self) -> u64;
    fn read_at(&self, offset: u64, buf: &mut [u8]) -> io::Result<usize>;
}

/// Append-only sink — the final GLB.
pub trait OutputHandle {
    fn write(&mut self, buf: &[u8]) -> io::Result<()>;
}

/// Random-access scratch — geometry spill for memory offload. Read + write.
pub trait TempHandle {
    fn write_at(&mut self, offset: u64, buf: &[u8]) -> io::Result<()>;
    fn read_at(&self, offset: u64, buf: &mut [u8]) -> io::Result<usize>;
    fn len(&self) -> u64;
}
```

Zero-copy nuance: today the parser borrows `&[u8]` straight out of `data`
(`EntityRec` byte ranges). Keep that fast path with a borrow-or-copy accessor —
borrow when the backing is in memory (`Vec`/mmap), read into a reusable
`scratch` when it is not (OPFS):

```rust
fn bytes<'a>(&'a self, start: u64, end: u64, scratch: &'a mut Vec<u8>) -> &'a [u8];
```

`StepFile.data: Vec<u8>` becomes `input: Box<dyn InputHandle>`; the lazy param
parser threads a small reusable `scratch`. Native stays zero-copy; OPFS pays a
tiny per-entity copy (entities are small).

## Handle impls per crate

| Crate | InputHandle | OutputHandle | TempHandle |
| --- | --- | --- | --- |
| `core` (default) | `Vec<u8>` (today's behaviour) | `Vec<u8>` | `Vec<u8>` / `None` = no spill |
| `cli` | mmap (`memmap2`) or `File` | `BufWriter<File>` | `tempfile` |
| `wasm` | OPFS `FileSystemSyncAccessHandle` | OPFS handle | OPFS handle |
| `capi` | caller fd/path → `File`, or C read/write callbacks | same | same |

## Pipeline entry point

```rust
pub fn convert(
    input:  &dyn InputHandle,
    output: &mut dyn OutputHandle,
    temp:   Option<&mut dyn TempHandle>,   // None / threshold 0 => all in RAM
    opts:   &Options,
) -> Result<Stats, Error>;
```

`temp = None` is byte-for-byte today's behaviour. `Some(_)` with
`--memory-threshold N` spills completed meshes once resident geometry crosses
`N`, keeping `[offset, len]` accessor refs. The streaming GLB writer appends BIN
to `temp`, builds the (small) JSON metadata in RAM, then at finalize writes
`header + JSON` to `output` and copies BIN back from `temp`. GLB layout
(`header | JSON chunk | BIN chunk`) cooperates: BIN is append-only and
streamable; instancing helps (a deduped mesh is written once, referenced many
times).

## STEP-specific constraint (input streaming)

STEP has forward *and* backward `#N` references, so there is no single-pass
discard-as-you-go: pass 1 builds the id→byte-offset index (streamable, small),
pass 2 seeks to bodies on demand. mmap makes pass 2 free on native; OPFS makes
it a sync `read_at` in the worker. The offset index is the irreducible memory
floor.

## Stages

1. ✅ **DONE — Workspace split + `core::io` traits** with in-memory impls only.
   `step2glb-core` (lib name `step2glb`, `meshopt` behind default `optimize`
   feature — core builds without it) + `step2glb-cli` (binary `step2glb`).
   `core::io` has `InputHandle`/`OutputHandle`/`TempHandle` + `Vec`/`MemSink`/
   `MemTemp` impls. No behaviour change; outputs byte-identical; tests green
   (59 unit + 49 integration + 8 proptest). Not yet wired into the pipeline —
   that's stages 2–3.
2. ✅ **DONE — native mmap input.** `StepFile` now holds a `Source` enum
   (`Owned(Vec)` | `Mmap`, both deref to `&[u8]`), so the zero-copy parser is
   unchanged. CLI uses `StepFile::open(path)` (memory-map; only touched pages
   resident → files > RAM; falls back to read for unmappable inputs). `mmap` is
   a default feature (native only); wasm/capi use `StepFile::from_input(&dyn
   InputHandle)` which reads the whole source into RAM. Output byte-identical.
   Note: the `bytes(scratch)` on-demand borrow-or-copy was unnecessary — the
   `Source` slice covers native mmap zero-copy, and the browser path reads to
   RAM (fine for browser-sized models). True per-entity OPFS streaming (read_at
   per body) is a future refinement, not needed for the demo.
3. ✅ **DONE — streaming GLB writer + `--memory-threshold`.** Both builders
   gained `write_stream(gen, out: &mut dyn OutputHandle, tmp: &mut dyn
   TempHandle)`: the binary chunk is appended to `tmp` and the container
   (header + JSON + BIN copied back from `tmp`) is streamed to `out` — no
   monolithic GLB `Vec` and no `pack_glb` 2× copy. `write() -> Vec<u8>` is now a
   MemSink+MemTemp wrapper (so all callers/tests are unchanged and exercise the
   streaming path). CLI: `--memory-threshold 300mb` backs `tmp` with an on-disk
   `FileTemp` (positioned read/write, removed on drop) and streams to a
   `FileSink`; `0` keeps the MemTemp/in-RAM path. Output byte-identical on both
   paths (verified on part2: RAM == spill). Note: this caps the *output*
   binary's RAM; capping the *resident per-mesh geometry during the walk*
   (serialize-at-add, hold only metadata) is the remaining increment — the
   TempHandle seam is in place for it.
4. ✅ **DONE (capi verified; wasm compiles; demo scaffolded).**
   - `core::convert` — high-level `convert(input, out, tmp, opts)` running the
     merged pipeline; the embeddable one-call API both shells use.
   - `step2glb-capi` (`cdylib`/`staticlib`, lib name `step2glb_capi`): C ABI
     `step2glb_convert(ptr,len,&out,&out_len) -> int` + `step2glb_free`. Built
     and unit-tested natively.
   - `step2glb-wasm` (`cdylib`): wasm-bindgen `convert_step_to_glb(bytes)->bytes`
     (+ `_opts`, `version`), core with `--no-default-features`. Compiles to
     `wasm32-unknown-unknown`. No JS imports (OPFS lives in the worker), so it
     also builds natively; kept out of `default-members` so plain
     `cargo test`/`build` stay native-only.
   - `wasm-demo/` — Vite page + Web Worker using OPFS **sync** access handles for
     input/output around the synchronous wasm core, rendering via
     `<model-viewer>`; bundled box−cylinder CSG sample. **Not run here** (no
     Node/wasm-pack/browser in this env) — `cargo build --target
     wasm32-unknown-unknown` is the compile check; `npm run dev` builds+serves.

   - `core::convert` returns a `ConvertReport` (geometry tally + issues
     per-type + defaults used, e.g. "no unit → mm assumed"); `to_json()` feeds
     the wasm/UI. wasm `convert_step_to_glb` returns `{ glb, info }`.
   - demo data path (as specified): main thread streams the upload to OPFS
     (async writable, `crypto.randomUUID()` name) and passes **paths** to the
     worker; the worker opens OPFS **sync** access handles by path, reads,
     converts, writes the UUID `.glb`, returns its path + report; main renders,
     shows the report, downloads on demand, deletes scratch on error / after
     download (+ `beforeunload` sweep).

   Remaining: OPFS-backed `InputHandle`/`TempHandle` *inside* Rust (read/spill on
   demand for models larger than tab memory), and capping resident per-mesh
   geometry during the walk (stage 3 note). The handle seams are in place.

Each stage is independently shippable and testable.

## Open decision

`TempHandle` shape: random-access (`write_at`/`read_at`, as above — matches OPFS
sync handles exactly, allows in-place accessor rewrites) vs. append-only write +
sequential read-back (leaner, and all the memory-offload flow strictly needs).
Leaning random-access since it costs nothing extra on `File`/OPFS.
