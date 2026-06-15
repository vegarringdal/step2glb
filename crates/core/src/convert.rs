//! High-level one-call conversion: STEP bytes → GLB. The embeddable entry point
//! the wasm and C-ABI shells build on (and a convenience for tests). It runs the
//! *merged* pipeline (world-baked, color-grouped) — the simplest complete path —
//! reading the input through an [`InputHandle`], spilling the binary chunk
//! through a [`TempHandle`], and streaming the container to an [`OutputHandle`].
//!
//! The CLI keeps its own richer driver (hierarchical mode, filters, `--split`,
//! cleanup passes); this is the small, dependency-light API for embedding.

use std::collections::HashMap;

use crate::io::{InputHandle, OutputHandle, TempHandle};
use crate::model::TessParams;
use crate::tessellate::{Ctx, TessStats};
use crate::{hierarchy, merge, model, step::StepFile, styles};

/// Options for [`convert`]; `Default` mirrors the CLI defaults (1 mm deflection,
/// Z-up → Y-up, scale to metres).
#[derive(Clone)]
pub struct ConvertOptions {
    /// chordal deflection in millimetres (converted to the file's unit)
    pub deflection_mm: f64,
    /// max chord turn angle in degrees
    pub max_angle_deg: f64,
    /// rotate Z-up (STEP) to glTF's Y-up
    pub rotate_z_up: bool,
    /// bake the file's length unit to metres in the output
    pub unit_scale_to_meters: bool,
    /// run the meshoptimizer pass (no-op without the `optimize` feature)
    pub optimize: bool,
    /// drop vertex normals (smaller output; viewer computes flat normals)
    pub drop_normals: bool,
    /// glTF `asset.generator` string
    pub generator: String,
}

impl Default for ConvertOptions {
    fn default() -> Self {
        ConvertOptions {
            deflection_mm: 1.0,
            max_angle_deg: 25.0,
            rotate_z_up: true,
            unit_scale_to_meters: true,
            optimize: cfg!(feature = "optimize"),
            drop_normals: true,
            generator: concat!("step2glb-core ", env!("CARGO_PKG_VERSION")).to_string(),
        }
    }
}

/// What happened during a conversion: the geometry tally, the issues worth
/// surfacing (skipped / unsupported / approximated entities) and the defaults
/// that were assumed (notably the length unit). Returned by [`convert`] and
/// serialised to JSON for the wasm/UI via [`ConvertReport::to_json`].
pub struct ConvertReport {
    pub stats: TessStats,
    /// number of color meshes emitted (0 ⇒ nothing tessellatable)
    pub color_meshes: usize,
    /// no length unit was declared, so millimetres were assumed
    pub unit_assumed_mm: bool,
    /// metres per file length unit (the scale baked into the output)
    pub unit_scale_to_meters: f64,
    /// the chordal deflection requested, in millimetres
    pub deflection_mm: f64,
    /// parser warnings (malformed records etc.), capped
    pub warnings: Vec<String>,
}

impl ConvertReport {
    /// Hand-rolled JSON (no serde dependency) for the wasm/UI: counts, the
    /// assumed-unit / deflection defaults, and the per-type issue tables.
    pub fn to_json(&self) -> String {
        let s = &self.stats;
        format!(
            "{{\"facesOk\":{},\"facesSkipped\":{},\"degenerateFaces\":{},\
             \"colorMeshes\":{},\"unitAssumedMillimetres\":{},\
             \"unitScaleToMetres\":{},\"deflectionMm\":{},\
             \"unsupportedSurfaces\":{},\"unsupportedCurves\":{},\
             \"unsupportedItems\":{},\"approximatedSurfaces\":{},\
             \"skippedSurfaces\":{},\"warnings\":[{}]}}",
            s.faces_ok,
            s.faces_failed,
            s.degenerate_faces,
            self.color_meshes,
            self.unit_assumed_mm,
            self.unit_scale_to_meters,
            self.deflection_mm,
            json_count_map(&s.unsupported_surfaces),
            json_count_map(&s.unsupported_curves),
            json_count_map(&s.unsupported_items),
            json_count_map(&s.approximated_surfaces),
            json_failed_map(&s.failed_surfaces),
            self.warnings
                .iter()
                .map(|w| json_str(w))
                .collect::<Vec<_>>()
                .join(","),
        )
    }
}

fn json_str(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

fn json_count_map(m: &HashMap<String, usize>) -> String {
    let mut entries: Vec<_> = m.iter().collect();
    entries.sort_by(|a, b| b.1.cmp(a.1).then(a.0.cmp(b.0)));
    let body = entries
        .iter()
        .map(|(k, v)| format!("{}:{}", json_str(k), v))
        .collect::<Vec<_>>()
        .join(",");
    format!("{{{}}}", body)
}

fn json_failed_map(m: &HashMap<String, (usize, Vec<u32>)>) -> String {
    let mut entries: Vec<_> = m.iter().collect();
    entries.sort_by(|a, b| b.1 .0.cmp(&a.1 .0).then(a.0.cmp(b.0)));
    let body = entries
        .iter()
        .map(|(k, (n, _))| format!("{}:{}", json_str(k), n))
        .collect::<Vec<_>>()
        .join(",");
    format!("{{{}}}", body)
}

/// Convert a STEP source to a merged GLB. Single-threaded (wasm-safe). Returns a
/// [`ConvertReport`] (stats + issues + assumed defaults), or an error string on
/// parse/IO failure.
pub fn convert(
    input: &dyn InputHandle,
    out: &mut dyn OutputHandle,
    tmp: &mut dyn TempHandle,
    opts: &ConvertOptions,
) -> Result<ConvertReport, String> {
    let sf = StepFile::from_input(input)?;

    // length unit: deflection is given in mm and converted into the file's unit
    // so the physical tolerance is unit-independent; the same scale takes the
    // output to metres. A missing unit is a *default* worth reporting.
    let detected = model::file_length_scale(&sf);
    let unit_assumed_mm = detected.is_none();
    let file_unit_scale = detected.unwrap_or(0.001);
    let mm_per_unit = file_unit_scale * 1000.0;
    let deflection_file = if (mm_per_unit - 1.0).abs() < 1e-9 {
        opts.deflection_mm
    } else {
        opts.deflection_mm / mm_per_unit
    };

    let colors = styles::build_color_map(&sf);
    let asm = hierarchy::build(&sf);
    let tp = TessParams {
        deflection: deflection_file,
        max_angle: opts.max_angle_deg.to_radians(),
    };
    let cx = Ctx {
        sf: &sf,
        tp: &tp,
        colors: &colors,
        threads: 1,
    };
    let mut stats = TessStats::default();
    let mopts = merge::MergeOptions {
        unit_scale: if opts.unit_scale_to_meters {
            file_unit_scale
        } else {
            1.0
        },
        file_unit_scale,
        rotate_z_up: opts.rotate_z_up,
        optimize: opts.optimize,
        drop_normals: opts.drop_normals,
        cleanup: None,
        simplify: None,
    };
    let (merged, _unique) = merge::build(&cx, &asm, mopts, &mut stats);
    let color_meshes = merged.bucket_count();
    merged
        .write_stream(&opts.generator, out, tmp)
        .map_err(|e| e.to_string())?;

    Ok(ConvertReport {
        stats,
        color_meshes,
        unit_assumed_mm,
        unit_scale_to_meters: file_unit_scale,
        deflection_mm: opts.deflection_mm,
        warnings: sf.warnings.iter().take(10).cloned().collect(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::{MemSink, MemTemp};

    #[test]
    fn convert_step_bytes_to_a_valid_glb_with_report() {
        // a CSG block-minus-cylinder (no product structure → merged fallback)
        let bytes = include_bytes!("../tests/fixtures/csg_block_minus_cylinder.step").to_vec();
        let mut out = MemSink::default();
        let mut tmp = MemTemp::default();
        let report = convert(&bytes, &mut out, &mut tmp, &ConvertOptions::default())
            .expect("convert succeeds");
        assert!(report.stats.faces_ok > 0, "geometry was produced");
        assert!(report.color_meshes > 0);
        assert!(
            report.unit_assumed_mm,
            "fixture declares no unit → mm assumed"
        );
        assert_eq!(&out.0[0..4], b"glTF", "valid GLB magic");
        let total = u32::from_le_bytes(out.0[8..12].try_into().unwrap()) as usize;
        assert_eq!(total, out.0.len(), "GLB total length matches");

        // the JSON report is well-formed-ish and carries the headline fields
        let json = report.to_json();
        assert!(json.contains("\"facesOk\":"));
        assert!(json.contains("\"unitAssumedMillimetres\":true"));
        assert!(json.starts_with('{') && json.ends_with('}'));
    }
}
