//! WebAssembly shell for step2glb.
//!
//! Deliberately tiny and **synchronous**: STEP bytes in, GLB bytes + a JSON
//! report out. All file I/O (reading the upload, writing the result) is done on
//! the JavaScript side — in a Web Worker using OPFS **synchronous** access
//! handles (`createSyncAccessHandle`) — so the CPU-bound core never has to
//! become `async`. See `wasm-demo/` for the worker + page that drive this.
//!
//! Build (needs the wasm-bindgen CLI / wasm-pack, not required to *compile*):
//!   wasm-pack build crates/wasm --target web --out-dir ../../wasm-demo/pkg
//! or: cargo build -p step2glb-wasm --target wasm32-unknown-unknown

use step2glb::convert::{convert, convert_with_progress, ConvertOptions};
use step2glb::io::{InputHandle, MemSink, MemTemp, OutputHandle, TempHandle};
use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;

/// The result of a conversion: the GLB bytes plus a JSON diagnostics report
/// (`facesOk`, `facesSkipped`, `unsupported*`, `unitAssumedMillimetres`, …).
#[wasm_bindgen]
pub struct ConvertResult {
    glb: Vec<u8>,
    info: String,
}

#[wasm_bindgen]
impl ConvertResult {
    /// The GLB bytes (a `Uint8Array` in JS).
    #[wasm_bindgen(getter)]
    pub fn glb(&self) -> Vec<u8> {
        self.glb.clone()
    }
    /// The JSON diagnostics report.
    #[wasm_bindgen(getter)]
    pub fn info(&self) -> String {
        self.info.clone()
    }
}

fn run(bytes: &[u8], opts: &ConvertOptions) -> Result<ConvertResult, JsValue> {
    let input = bytes.to_vec();
    let mut out = MemSink::default();
    let mut tmp = MemTemp::default();
    let report =
        convert(Box::new(input), &mut out, &mut tmp, opts).map_err(|e| JsValue::from_str(&e))?;
    Ok(ConvertResult {
        glb: out.0,
        info: report.to_json(),
    })
}

/// Convert a STEP file (raw bytes) to GLB with the default options. Returns a
/// [`ConvertResult`] (GLB bytes + JSON report), or a JS error string.
#[wasm_bindgen]
pub fn convert_step_to_glb(step_bytes: &[u8]) -> Result<ConvertResult, JsValue> {
    run(step_bytes, &ConvertOptions::default())
}

/// Convert with the knobs a viewer exposes. `cleanup` runs the rvm-style
/// position weld (+ degenerate drop; the simplify step needs meshoptimizer,
/// which the wasm build omits). `merged` selects the one-mesh-per-color world-
/// baked layout vs the hierarchical per-part node tree.
#[wasm_bindgen]
#[allow(clippy::too_many_arguments)]
pub fn convert_step_to_glb_opts(
    step_bytes: &[u8],
    deflection_mm: f64,
    max_angle_deg: f64,
    y_up: bool,
    keep_normals: bool,
    cleanup: bool,
    merged: bool,
) -> Result<ConvertResult, JsValue> {
    let opts = ConvertOptions {
        deflection_mm,
        max_angle_deg,
        rotate_z_up: y_up,
        drop_normals: !keep_normals,
        cleanup,
        merged,
        ..ConvertOptions::default()
    };
    run(step_bytes, &opts)
}

/// Version of the underlying converter, for the demo UI.
#[wasm_bindgen]
pub fn version() -> String {
    concat!("step2glb-wasm ", env!("CARGO_PKG_VERSION")).to_string()
}

// ---------------------------------------------------- streaming (callback) API

// A JS object the worker provides, backed by OPFS *synchronous* access handles.
// Rust calls these to pull input ranges, append the GLB, spill geometry to a
// temp file, and report progress — so nothing larger than a chunk is ever held
// in wasm memory. Offsets/lengths are passed as f64 (JS numbers).
#[wasm_bindgen]
extern "C" {
    pub type Io;
    #[wasm_bindgen(method)]
    fn size(this: &Io) -> f64;
    #[wasm_bindgen(method)]
    fn read(this: &Io, offset: f64, len: f64) -> Vec<u8>;
    #[wasm_bindgen(method, js_name = writeOutput)]
    fn write_output(this: &Io, bytes: &[u8]);
    #[wasm_bindgen(method, js_name = writeTemp)]
    fn write_temp(this: &Io, offset: f64, bytes: &[u8]);
    #[wasm_bindgen(method, js_name = readTemp)]
    fn read_temp(this: &Io, offset: f64, len: f64) -> Vec<u8>;
    #[wasm_bindgen(method, js_name = tempLen)]
    fn temp_len(this: &Io) -> f64;
    #[wasm_bindgen(method)]
    fn progress(this: &Io, done: f64, total: f64);
}

fn copy_in(data: &[u8], buf: &mut [u8]) -> usize {
    let n = data.len().min(buf.len());
    buf[..n].copy_from_slice(&data[..n]);
    n
}

struct OpfsInput(Io);
// SAFETY: wasm32 is single-threaded; the handle is never touched concurrently.
unsafe impl Send for OpfsInput {}
unsafe impl Sync for OpfsInput {}
impl InputHandle for OpfsInput {
    fn size(&self) -> u64 {
        self.0.size() as u64
    }
    fn read_at(&self, offset: u64, buf: &mut [u8]) -> std::io::Result<usize> {
        Ok(copy_in(&self.0.read(offset as f64, buf.len() as f64), buf))
    }
}

struct OpfsOutput(Io);
impl OutputHandle for OpfsOutput {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<()> {
        self.0.write_output(buf);
        Ok(())
    }
}

struct OpfsTemp(Io);
impl TempHandle for OpfsTemp {
    fn write_at(&mut self, offset: u64, buf: &[u8]) -> std::io::Result<()> {
        self.0.write_temp(offset as f64, buf);
        Ok(())
    }
    fn read_at(&self, offset: u64, buf: &mut [u8]) -> std::io::Result<usize> {
        Ok(copy_in(
            &self.0.read_temp(offset as f64, buf.len() as f64),
            buf,
        ))
    }
    fn len(&self) -> u64 {
        self.0.temp_len() as u64
    }
}

/// Streaming conversion: input is read **by range** from the `io` handle, the
/// GLB is written through `io.writeOutput`, geometry spills through
/// `io.writeTemp`/`readTemp`, and progress is reported via `io.progress`. The
/// whole file is never materialized in wasm memory. Returns the JSON report
/// (the GLB bytes went out through `io`, not the return value).
#[wasm_bindgen]
#[allow(clippy::too_many_arguments)]
pub fn convert_streaming(
    io: Io,
    deflection_mm: f64,
    max_angle_deg: f64,
    y_up: bool,
    keep_normals: bool,
    cleanup: bool,
    merged: bool,
) -> Result<String, JsValue> {
    let opts = ConvertOptions {
        deflection_mm,
        max_angle_deg,
        rotate_z_up: y_up,
        drop_normals: !keep_normals,
        cleanup,
        merged,
        ..ConvertOptions::default()
    };
    // three views of the same JS handle object (wasm32 is single-threaded)
    let input: Box<dyn InputHandle> = Box::new(OpfsInput(io.clone().unchecked_into()));
    let mut out = OpfsOutput(io.clone().unchecked_into());
    let mut tmp = OpfsTemp(io.clone().unchecked_into());
    let report = convert_with_progress(input, &mut out, &mut tmp, &opts, &mut |done, total| {
        io.progress(done as f64, total as f64);
    })
    .map_err(|e| JsValue::from_str(&e))?;
    Ok(report.to_json())
}
