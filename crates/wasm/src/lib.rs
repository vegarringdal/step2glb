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

use step2glb::convert::{convert, ConvertOptions};
use step2glb::io::{MemSink, MemTemp};
use wasm_bindgen::prelude::*;

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
    let report = convert(&input, &mut out, &mut tmp, opts).map_err(|e| JsValue::from_str(&e))?;
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

/// Convert with an explicit chordal deflection (millimetres) and Y-up toggle —
/// the two knobs a viewer most often wants to expose.
#[wasm_bindgen]
pub fn convert_step_to_glb_opts(
    step_bytes: &[u8],
    deflection_mm: f64,
    y_up: bool,
) -> Result<ConvertResult, JsValue> {
    let opts = ConvertOptions {
        deflection_mm,
        rotate_z_up: y_up,
        ..ConvertOptions::default()
    };
    run(step_bytes, &opts)
}

/// Version of the underlying converter, for the demo UI.
#[wasm_bindgen]
pub fn version() -> String {
    concat!("step2glb-wasm ", env!("CARGO_PKG_VERSION")).to_string()
}
