//! step2glb — tessellate STEP (ISO 10303-21) files and export binary glTF,
//! with assembly-hierarchy dump. Companion in spirit to rvm_parser_glb.

use step2glb::{glb, hierarchy, merge, step, styles, tessellate};

use std::collections::HashMap;
use std::path::PathBuf;
use std::time::Instant;

use clap::{Parser, ValueEnum};

use step::StepFile;
use step2glb::geom::M4;
use step2glb::mesh::MeshSet;
use step2glb::model::TessParams;
use tessellate::TessStats;

#[derive(Parser, Debug)]
#[command(
    name = "step2glb",
    version,
    about = "Tessellate STEP files to GLB and inspect assembly hierarchy"
)]
struct Args {
    /// Input .step / .stp file
    input: PathBuf,

    /// Output .glb path (default: input with .glb extension)
    #[arg(short, long)]
    output: Option<PathBuf>,

    /// Chordal deflection used for tessellation, in file units (e.g. mm)
    #[arg(short, long, default_value_t = 1.0)]
    deflection: f64,

    /// Maximum angle per segment on curved geometry, degrees. The tighter of
    /// this and --deflection wins per feature; on small radii max-angle is
    /// almost always the binding bound, so raise it to actually coarsen
    #[arg(long, default_value_t = 25.0)]
    max_angle: f64,

    /// Print the assembly hierarchy tree and exit (no GLB written)
    #[arg(long)]
    tree: bool,

    /// Print entity statistics (top types, mesh stats)
    #[arg(long)]
    stats: bool,

    /// Merge everything into one node/mesh per color, geometry baked to
    /// world space (Y-up), with per-part draw ranges and the id hierarchy in
    /// scene extras — the rvm_parser_glb output layout
    #[arg(long)]
    merged: bool,

    /// rvm-style position cleanup: weld vertices on a quantized grid,
    /// simplify with meshoptimizer and drop normals. Applies per unique mesh
    /// in the default output, per part instance with --merged
    #[arg(long)]
    cleanup_position: bool,

    /// Skip vertex normals in the output: positions weld harder and files
    /// shrink; viewers flat-shade or compute their own normals
    #[arg(long)]
    no_normals: bool,

    /// Quantization decimals for --cleanup-position, in file units
    #[arg(long, default_value_t = 3)]
    cleanup_precision: u32,

    /// meshopt_simplify target index ratio (0.75 under --cleanup-position).
    /// Given on its own, enables a simplify-only pass that keeps normals —
    /// works with and without --merged
    #[arg(long)]
    meshopt_threshold: Option<f32>,

    /// meshopt_simplify target error (0 = only lossless collapses; under
    /// --cleanup-position it defaults to 0). Given on its own, enables the
    /// simplify-only pass too
    #[arg(long)]
    meshopt_target_error: Option<f32>,

    /// Tessellation worker threads; default: auto (CPU cores, capped at 4)
    #[arg(short = 't', long)]
    threads: Option<usize>,

    /// Skip the meshoptimizer pass
    #[arg(long)]
    no_optimize: bool,

    /// Write a self-contained STEP sub-graph of the first failing face of each
    /// surface type (plus the file HEADER and the stage that failed) to
    /// <input>.debug.txt, for diagnosing skipped faces without the whole file
    #[arg(long)]
    debug_print: bool,

    /// Don't rescale to meters (keep raw file units in the GLB)
    #[arg(long)]
    no_unit_scale: bool,

    /// Up axis of the input model — STEP has no reliable up-axis field, so
    /// it can't be auto-detected. "z" (engineering convention) rotates the
    /// model to glTF's Y-up; "y" exports the axes unchanged
    #[arg(long, value_enum, default_value_t = UpAxis::Z)]
    up_axis: UpAxis,
}

#[derive(Clone, Copy, Debug, PartialEq, ValueEnum)]
enum UpAxis {
    /// input is Z-up: rotate (x, y, z) -> (x, z, -y) into glTF Y-up
    Z,
    /// input is already Y-up: no rotation
    Y,
}

fn main() {
    let args = Args::parse();
    tessellate::install_panic_guard();
    let t0 = Instant::now();

    let data = match std::fs::read(&args.input) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("error: cannot read {}: {}", args.input.display(), e);
            std::process::exit(1);
        }
    };
    let file_len = data.len();

    let sf = match StepFile::parse(data) {
        Ok(sf) => sf,
        Err(e) => {
            eprintln!("error: failed to parse STEP file: {}", e);
            std::process::exit(1);
        }
    };
    eprintln!(
        "parsed {:.1} MB, {} entities in {:.2?}",
        file_len as f64 / 1e6,
        sf.entities.len(),
        t0.elapsed()
    );
    for w in sf.warnings.iter().take(5) {
        eprintln!("warn: {}", w);
    }

    if args.stats {
        print_entity_stats(&sf);
    }

    let asm = hierarchy::build(&sf);
    if args.tree {
        if asm.roots.is_empty() {
            println!("(no product hierarchy found)");
        } else {
            hierarchy::print_tree(&asm);
        }
        if !args.stats {
            return;
        }
    }

    // ---------------------------------------------------------- tessellation
    let tp = TessParams {
        deflection: args.deflection,
        max_angle: args.max_angle.to_radians(),
    };
    let mut stats = TessStats::default();
    let mut builder = glb::GlbBuilder::default();

    // mesh cache: per product-definition, plus md5 content dedup across PDs
    let colors = styles::build_color_map(&sf);
    if !colors.is_empty() && args.stats {
        eprintln!("found {} styled (colored) items", colors.len());
    }
    let threads = args.threads.filter(|&n| n > 0).unwrap_or_else(|| {
        std::thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(1)
            .min(4)
    });
    if threads > 1 {
        eprintln!("tessellating with {} threads", threads);
    }
    let cx = tessellate::Ctx {
        sf: &sf,
        tp: &tp,
        colors: &colors,
        threads,
    };

    // ------------------------------------------------------------ merged mode
    if args.merged {
        let scale = if args.no_unit_scale {
            1.0
        } else {
            detect_length_unit(&sf)
        };
        let opts = merge::MergeOptions {
            unit_scale: scale,
            rotate_z_up: args.up_axis == UpAxis::Z,
            optimize: !args.no_optimize,
            drop_normals: args.no_normals,
            cleanup: args.cleanup_position.then_some(merge::Cleanup {
                precision: args.cleanup_precision,
                threshold: simplify_threshold(&args),
                target_error: simplify_target_error(&args),
            }),
            simplify: simplify_only(&args),
        };
        let t1 = Instant::now();
        let (merged, unique) = merge::build(&cx, &asm, opts, &mut stats);
        if merged.bucket_count() == 0 {
            eprintln!("error: no tessellatable geometry found in this file");
            report_unsupported(&stats);
            std::process::exit(2);
        }
        eprintln!(
            "merged {} parts ({} unique meshes, ~{:.1}x instance expansion) \
             into {} color meshes ({} faces ok, {} skipped) in {:.2?}",
            merged.part_count(),
            unique,
            merged.part_count() as f64 / unique.max(1) as f64,
            merged.bucket_count(),
            stats.faces_ok,
            stats.faces_failed,
            t1.elapsed()
        );
        report_unsupported(&stats);
        maybe_write_debug(&args, &sf, &stats);
        eprintln!(
            "{} verts, {} tris",
            merged.total_vertices(),
            merged.total_triangles()
        );
        let out_path = args
            .output
            .unwrap_or_else(|| args.input.with_extension("glb"));
        let bytes = merged.write(&format!("step2glb {}", env!("CARGO_PKG_VERSION")));
        write_out(&out_path, &bytes, t0);
        return;
    }

    let mut mesh_of_pd: HashMap<u32, Option<usize>> = HashMap::new();
    let mut mesh_of_hash: HashMap<[u8; 16], usize> = HashMap::new();

    let t1 = Instant::now();
    let mut get_mesh =
        |pd: u32, builder: &mut glb::GlbBuilder, stats: &mut TessStats| -> Option<usize> {
            if let Some(cached) = mesh_of_pd.get(&pd) {
                return *cached;
            }
            let node = asm.products.get(&pd);
            let mut tm = MeshSet::default();
            if let Some(node) = node {
                for &sr in &node.shape_reps {
                    // SHAPE_REPRESENTATION('', (items), context)
                    if let Some(p) = sf.params(sr) {
                        if let Some(items) = p.get(1).and_then(|v| v.as_list()) {
                            for it in items {
                                if let Some(r) = it.as_ref_id() {
                                    tessellate::tessellate_item(&cx, r, None, &mut tm, stats);
                                }
                            }
                        }
                    }
                }
            }
            prepare_mesh(&mut tm, &args);
            let result = if tm.is_empty() {
                None
            } else {
                let h = tm.content_hash();
                let idx = match mesh_of_hash.get(&h) {
                    Some(&i) => i,
                    None => {
                        let name = node
                            .map(|n| n.name.clone())
                            .unwrap_or_else(|| format!("PD#{}", pd));
                        let i = builder.add_mesh(tm, name);
                        mesh_of_hash.insert(h, i);
                        i
                    }
                };
                Some(idx)
            };
            mesh_of_pd.insert(pd, result);
            result
        };

    // -------------------------------------------------------- node expansion
    fn expand(
        asm: &hierarchy::Assembly,
        pd: u32,
        name: &str,
        transform: Option<M4>,
        builder: &mut glb::GlbBuilder,
        get_mesh: &mut dyn FnMut(u32, &mut glb::GlbBuilder, &mut TessStats) -> Option<usize>,
        stats: &mut TessStats,
        depth: usize,
        budget: &mut i64,
    ) -> Option<usize> {
        if depth > 64 || *budget <= 0 {
            return None;
        }
        *budget -= 1;
        let mesh = get_mesh(pd, builder, stats);
        let node = builder.add_node(name.to_string(), transform, mesh);
        if let Some(kids) = asm.children.get(&pd) {
            let mut children = Vec::with_capacity(kids.len());
            for k in kids {
                if let Some(c) = expand(
                    asm,
                    k.child_pd,
                    &k.name,
                    Some(k.transform),
                    builder,
                    get_mesh,
                    stats,
                    depth + 1,
                    budget,
                ) {
                    children.push(c);
                }
            }
            builder.nodes[node].children = children;
        }
        Some(node)
    }

    let mut budget: i64 = 2_000_000; // instance explosion guard
    let mut top_nodes: Vec<usize> = Vec::new();
    for &root in &asm.roots {
        let name = asm
            .products
            .get(&root)
            .map(|n| n.name.clone())
            .unwrap_or_else(|| format!("PD#{}", root));
        if let Some(n) = expand(
            &asm,
            root,
            &name,
            None,
            &mut builder,
            &mut get_mesh,
            &mut stats,
            0,
            &mut budget,
        ) {
            top_nodes.push(n);
        }
    }

    // Fallback: no product structure -> dump every standalone solid we can find
    if top_nodes.is_empty() {
        let mut tm = MeshSet::default();
        for ty in merge::FALLBACK_TYPES {
            for &id in sf.of_type(ty) {
                tessellate::tessellate_item(&cx, id, None, &mut tm, &mut stats);
            }
        }
        prepare_mesh(&mut tm, &args);
        if !tm.is_empty() {
            let mi = builder.add_mesh(tm, "geometry".into());
            let n = builder.add_node("root".into(), None, Some(mi));
            top_nodes.push(n);
        }
    }

    if top_nodes.is_empty() {
        eprintln!("error: no tessellatable geometry found in this file");
        report_unsupported(&stats);
        std::process::exit(2);
    }

    // -------------------------------------------- unit scale + up-axis root
    let scale = if args.no_unit_scale {
        1.0
    } else {
        detect_length_unit(&sf)
    };
    let mut root_m = M4::scale_uniform(scale);
    if args.up_axis == UpAxis::Z {
        root_m = M4::Z_UP_TO_Y_UP.mul(root_m);
    }
    if !root_m.is_identity(1e-12) {
        let root = builder.add_node("root_transform".into(), Some(root_m), None);
        builder.nodes[root].children = top_nodes;
        builder.root_nodes = vec![root];
    } else {
        builder.root_nodes = top_nodes;
    }

    eprintln!(
        "tessellated {} unique meshes ({} faces ok, {} skipped) in {:.2?}",
        builder.meshes.len(),
        stats.faces_ok,
        stats.faces_failed,
        t1.elapsed()
    );
    report_unsupported(&stats);
    maybe_write_debug(&args, &sf, &stats);

    let total_tris: usize = builder.total_triangles();
    let total_verts: usize = builder.total_vertices();
    eprintln!(
        "{} nodes, {} unique meshes, {} verts, {} tris",
        builder.nodes.len(),
        builder.meshes.len(),
        total_verts,
        total_tris
    );

    // ------------------------------------------------------------- write GLB
    let out_path = args
        .output
        .unwrap_or_else(|| args.input.with_extension("glb"));
    let bytes = builder.write(&format!("step2glb {}", env!("CARGO_PKG_VERSION")));
    write_out(&out_path, &bytes, t0);
}

fn write_out(out_path: &std::path::Path, bytes: &[u8], t0: Instant) {
    if let Err(e) = std::fs::write(out_path, bytes) {
        eprintln!("error: cannot write {}: {}", out_path.display(), e);
        std::process::exit(1);
    }
    eprintln!(
        "wrote {} ({:.1} MB) in {:.2?} total",
        out_path.display(),
        bytes.len() as f64 / 1e6,
        t0.elapsed()
    );
}

fn simplify_threshold(args: &Args) -> f32 {
    args.meshopt_threshold.unwrap_or(0.75)
}

fn simplify_target_error(args: &Args) -> f32 {
    args.meshopt_target_error.unwrap_or(0.0)
}

/// The meshopt knobs given without --cleanup-position select a standalone
/// simplification pass (normals kept).
fn simplify_only(args: &Args) -> Option<(f32, f32)> {
    (!args.cleanup_position
        && (args.meshopt_threshold.is_some() || args.meshopt_target_error.is_some()))
    .then(|| (simplify_threshold(args), simplify_target_error(args)))
}

/// Per-mesh pipeline for the hierarchical output: normal stripping, the
/// meshoptimizer pass and the optional rvm-style position cleanup or
/// simplify-only pass — the same steps merged mode runs per part, here once
/// per unique mesh.
fn prepare_mesh(tm: &mut MeshSet, args: &Args) {
    if tm.is_empty() {
        return;
    }
    if args.no_normals {
        tm.drop_normals();
    }
    if !args.no_optimize {
        tm.optimize();
    }
    if args.cleanup_position {
        tm.cleanup_positions(
            args.cleanup_precision,
            simplify_threshold(args),
            simplify_target_error(args),
        );
    } else if let Some((threshold, target_error)) = simplify_only(args) {
        tm.simplify(threshold, target_error);
    }
}

fn report_unsupported(stats: &TessStats) {
    if !stats.unsupported_surfaces.is_empty() {
        let mut v: Vec<_> = stats.unsupported_surfaces.iter().collect();
        v.sort_by(|a, b| b.1.cmp(a.1));
        eprintln!("unsupported surface types (faces skipped):");
        for (ty, n) in v {
            eprintln!("  {:>6}  {}", n, ty);
        }
    }
    if !stats.failed_surfaces.is_empty() {
        let mut v: Vec<_> = stats.failed_surfaces.iter().collect();
        v.sort_by(|a, b| b.1 .0.cmp(&a.1 .0));
        eprintln!("faces skipped on supported surfaces (trimming/tessellation failed):");
        for (ty, (n, samples)) in v {
            let ids = samples
                .iter()
                .map(|id| format!("#{}", id))
                .collect::<Vec<_>>()
                .join(" ");
            eprintln!("  {:>6}  {}  (e.g. {})", n, ty, ids);
            // the stage that failed for the first face of this type, so the
            // reason is visible in the console without --debug-print
            if let Some((_, reason)) = stats.debug_samples.get(ty) {
                eprintln!("          why: {}", reason);
            }
        }
        eprintln!("  (run with --debug-print to dump these faces as a shareable .step)");
    }
}

/// `--debug-print`: write a self-contained STEP excerpt for the first failing
/// face of each surface type so a vendor model's failures can be diagnosed (and
/// turned into a fixture) without shipping the whole file. Each excerpt is the
/// face entity plus the transitive closure of everything it references.
fn maybe_write_debug(args: &Args, sf: &StepFile, stats: &TessStats) {
    if !args.debug_print {
        return;
    }
    if stats.debug_samples.is_empty() {
        eprintln!("--debug-print: no failing supported faces to dump");
        return;
    }
    // Emit one self-contained, re-runnable Part-21 file: rename it to .step and
    // feed it back through step2glb to reproduce the failures in isolation.
    let mut out = String::from("ISO-10303-21;\n");
    out.push_str(
        "/* step2glb --debug-print: the first failing face of each surface type,\n\
        \x20  each as the ADVANCED_FACE plus the transitive closure of everything\n\
        \x20  it references. Geometry only. Rename to .step to reproduce. */\n",
    );
    // original file HEADER (schema + originating system) — small, valuable
    let (h0, h1) = sf.header_range;
    if h1 > h0 && h1 <= sf.data.len() {
        out.push_str(&String::from_utf8_lossy(&sf.data[h0..h1]));
        if !out.ends_with('\n') {
            out.push('\n');
        }
    } else {
        out.push_str("HEADER;\nENDSEC;\n");
    }
    // Pre-compute each sample's reference closure so we can order the excerpts
    // smallest-first: a single huge B-spline (thousands of control points) must
    // not bury the simple PLANE/CYLINDER cases past a copy-paste/scroll limit.
    // The cap is a runaway guard only — a truncated closure drops the bound
    // topology (the surface's control points expand first) and the excerpt
    // stops being re-runnable, so it must comfortably exceed any real face.
    const SUBGRAPH_CAP: usize = 200_000;
    let mut samples: Vec<(&String, u32, &'static str, Vec<u32>)> = stats
        .debug_samples
        .iter()
        .map(|(ty, (face, reason))| (ty, *face, *reason, sf.subgraph(*face, SUBGRAPH_CAP)))
        .collect();
    samples.sort_by(|a, b| a.3.len().cmp(&b.3.len()).then(a.0.cmp(b.0)));

    // Compact, geometry-free summary first: type, sample face, entity count and
    // the failing stage — one line each, so it stays readable (and pasteable)
    // no matter how large the per-face excerpts below get.
    out.push_str("/* ===== FAILURE SUMMARY =====\n");
    for (ty, face, reason, ids) in &samples {
        out.push_str(&format!(
            "   {:<28} face #{:<10} ({:>4} entities{})  {}\n",
            ty,
            face,
            ids.len(),
            if ids.len() >= SUBGRAPH_CAP {
                ", TRUNCATED"
            } else {
                ""
            },
            reason
        ));
    }
    out.push_str("   ===== END SUMMARY ===== */\n");
    out.push_str("DATA;\n");

    // dedup entities shared between faces so the single DATA section stays valid
    let mut emitted = std::collections::HashSet::new();
    let mut max_id = 0u32;
    for (ty, face, reason, ids) in &samples {
        out.push_str(&format!(
            "/* ===== {} (face #{}) -- failure stage: {} ===== */\n",
            ty, face, reason
        ));
        for &id in ids {
            max_id = max_id.max(id);
            if !emitted.insert(id) {
                continue;
            }
            if let Some(line) = sf.entity_source(id) {
                out.push_str(&line);
                out.push('\n');
            }
        }
    }
    // Synthesize a shell + solid around the sample faces so the excerpt has a
    // traversable root (the original shell/product chain isn't included) and
    // runs straight back through the converter to reproduce the skips.
    let shell = max_id + 1;
    let solid = max_id + 2;
    let faces = samples
        .iter()
        .map(|(_, f, _, _)| format!("#{}", f))
        .collect::<Vec<_>>()
        .join(",");
    out.push_str("/* synthetic root so the excerpt is self-contained */\n");
    out.push_str(&format!("#{}=CLOSED_SHELL('debug',({}));\n", shell, faces));
    out.push_str(&format!("#{}=MANIFOLD_SOLID_BREP('debug',#{});\n", solid, shell));
    out.push_str("ENDSEC;\nEND-ISO-10303-21;\n");

    let path = args.input.with_extension("debug.txt");
    match std::fs::write(&path, out) {
        Ok(()) => eprintln!(
            "--debug-print: wrote {} face excerpt(s) to {}",
            stats.debug_samples.len(),
            path.display()
        ),
        Err(e) => eprintln!("--debug-print: cannot write {}: {}", path.display(), e),
    }
}

/// Sniff the model's LENGTH_UNIT and return the scale factor to meters.
/// Looks for SI_UNIT prefixes and inch-based CONVERSION_BASED_UNITs.
fn detect_length_unit(sf: &StepFile) -> f64 {
    // SI_UNIT(.MILLI., .METRE.) usually lives inside a complex instance also
    // tagged LENGTH_UNIT.
    let candidates: Vec<u32> = sf
        .of_type("SI_UNIT")
        .iter()
        .copied()
        .chain(
            sf.by_type
                .iter()
                .filter(|(t, _)| sf.type_name(**t) == step::TYPE_COMPLEX)
                .flat_map(|(_, ids)| ids.iter().copied()),
        )
        .collect();

    for id in candidates {
        let params = if sf.is_complex(id) {
            if sf.complex_leaf(id, "LENGTH_UNIT").is_none() {
                continue;
            }
            match sf.complex_leaf(id, "SI_UNIT") {
                Some(p) => p,
                None => {
                    // inch & friends: CONVERSION_BASED_UNIT('INCH', measure)
                    if let Some(cbu) = sf.complex_leaf(id, "CONVERSION_BASED_UNIT") {
                        if let Some(f) = conversion_unit_scale(&cbu) {
                            return f;
                        }
                    }
                    continue;
                }
            }
        } else {
            match sf.params(id) {
                Some(p) => p,
                None => continue,
            }
        };
        // SI_UNIT(prefix?, name) — name must be METRE for length
        let mut prefix: Option<String> = None;
        let mut name: Option<String> = None;
        for p in &params {
            if let step::P::Enum(e) = p {
                if e == "METRE" {
                    name = Some(e.clone());
                } else if name.is_none() && e != "STERADIAN" && e != "RADIAN" {
                    prefix = Some(e.clone());
                }
            }
        }
        if name.as_deref() == Some("METRE") {
            return match prefix.as_deref() {
                Some("MILLI") => 0.001,
                Some("CENTI") => 0.01,
                Some("DECI") => 0.1,
                Some("KILO") => 1000.0,
                Some("MICRO") => 1e-6,
                _ => 1.0,
            };
        }
    }
    eprintln!("warn: no length unit found, assuming millimetres");
    0.001
}

fn conversion_unit_scale(cbu: &[step::P]) -> Option<f64> {
    let name = cbu.iter().find_map(|p| p.as_str())?.to_ascii_uppercase();
    match name.as_str() {
        "INCH" | "\"INCH\"" => Some(0.0254),
        "FOOT" => Some(0.3048),
        _ => None,
    }
}

fn print_entity_stats(sf: &StepFile) {
    let mut counts: Vec<(usize, &str)> = sf
        .by_type
        .iter()
        .map(|(t, ids)| (ids.len(), sf.type_name(*t)))
        .collect();
    counts.sort_by(|a, b| b.0.cmp(&a.0));
    println!("top entity types:");
    for (n, ty) in counts.iter().take(25) {
        println!("  {:>8}  {}", n, ty);
    }
}
