//! step2glb — tessellate STEP (ISO 10303-21) files and export binary glTF,
//! with assembly-hierarchy dump. Companion in spirit to rvm_parser_glb.

use step2glb::{glb, hierarchy, merge, model, step, styles, tessellate};

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

    /// Chordal deflection (max sag) for tessellation, in millimetres. It is
    /// converted into each representation's own modeling unit, so the same
    /// value means the same physical tolerance whether a part is in mm, inch or
    /// metre (Autodesk mixes units across one file)
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

    /// Include vertex normals in the output. Off by default — positions then
    /// weld harder (face-boundary vertices merge) and files shrink; viewers
    /// flat-shade or compute their own. --cleanup-position drops normals
    /// regardless
    #[arg(long)]
    normals: bool,

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

    /// Debug: restrict output to the element(s) matching this query plus their
    /// whole subtree — to isolate why an element is missing or wrong. Matches a
    /// PRODUCT_DEFINITION id as `#<n>`/`<n>`, else a case-insensitive substring
    /// of the product name. With --tree prints just that subtree; pair with
    /// --extract-step to dump it to a new STEP file
    #[arg(long)]
    filter: Option<String>,

    /// Debug: write the --filter selection to a new standalone STEP file at
    /// this path — the matched element plus the transitive closure of
    /// everything it references (product structure, shape/representation
    /// linkage followed multi-hop, geometry), re-runnable in isolation and
    /// small enough to share. Requires --filter; no GLB is written
    #[arg(long, value_name = "PATH")]
    extract_step: Option<PathBuf>,

    /// Debug: explode each part's geometry into separate named nodes so a bad
    /// piece can be isolated in a viewer (hierarchical output only). "solid" =
    /// one node per solid, "shell" = per CLOSED_SHELL, "face" = per
    /// ADVANCED_FACE (finest). Nodes are named <ENTITY_TYPE>#<id>, so the id
    /// feeds straight back into --filter "#<id>" --extract-step
    #[arg(long, value_enum, value_name = "LEVEL")]
    split: Option<SplitArg>,
}

#[derive(Clone, Copy, Debug, PartialEq, ValueEnum)]
enum SplitArg {
    Solid,
    Shell,
    Face,
}

impl SplitArg {
    fn level(self) -> tessellate::SplitLevel {
        match self {
            SplitArg::Solid => tessellate::SplitLevel::Solid,
            SplitArg::Shell => tessellate::SplitLevel::Shell,
            SplitArg::Face => tessellate::SplitLevel::Face,
        }
    }
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

    let threads = resolve_threads(&args);

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

    // The model's geometry (length) unit. `--deflection` is specified in mm
    // and converted into this unit for tessellation, so the same number means
    // the same physical chord tolerance regardless of how the file models
    // length. The same scale also takes the GLB to meters at export.
    let file_unit_scale = detect_length_unit(&sf);
    let mm_per_unit = file_unit_scale * 1000.0;
    let deflection_file = if (mm_per_unit - 1.0).abs() < 1e-9 {
        args.deflection // file already in mm: keep the value exactly
    } else {
        args.deflection / mm_per_unit
    };
    let output_scale = if args.no_unit_scale { 1.0 } else { file_unit_scale };
    print_settings(&args, threads, file_unit_scale);

    if args.stats {
        print_entity_stats(&sf);
    }

    let mut asm = hierarchy::build(&sf);
    if let Some(query) = &args.filter {
        let roots = hierarchy::filter_roots(&asm, query);
        if roots.is_empty() {
            eprintln!("error: --filter {:?} matched no product", query);
            std::process::exit(2);
        }
        eprintln!("filter {:?} -> {} subtree root(s):", query, roots.len());
        for &pd in &roots {
            let node = asm.products.get(&pd);
            let name = node.map(|n| n.name.as_str()).unwrap_or("?");
            let reps = node.map(|n| n.shape_reps.len()).unwrap_or(0);
            let kids = asm.children.get(&pd).map(|k| k.len()).unwrap_or(0);
            eprintln!(
                "  #{pd} {name}  ({reps} shape rep(s), {kids} child instance(s){})",
                if reps == 0 { ", no geometry of its own" } else { "" }
            );
        }
        asm.roots = roots;
    }

    // --extract-step: write the selected subtree to a standalone, re-runnable
    // STEP file (requires --filter, so it's a focused excerpt to share/inspect).
    if let Some(path) = &args.extract_step {
        if args.filter.is_none() {
            eprintln!("error: --extract-step needs --filter to select what to extract");
            std::process::exit(2);
        }
        let ids = hierarchy::subtree_entities(&sf, &asm, &asm.roots);
        write_step_excerpt(&sf, &ids, path);
        return;
    }

    // Heads-up: parts that are in the tree but have no geometry attached often
    // mean a representation linkage we didn't follow, not an empty part.
    let missing = hierarchy::parts_missing_geometry(&asm);
    if !missing.is_empty() {
        eprintln!(
            "warn: {} leaf part(s) in the tree have no geometry attached \
             (possible unfollowed representation linkage); e.g.:",
            missing.len()
        );
        for &pd in missing.iter().take(8) {
            let name: String = asm
                .products
                .get(&pd)
                .map(|n| n.name.replace(['\n', '\r', '\t'], " "))
                .unwrap_or_else(|| "?".into())
                .chars()
                .take(70)
                .collect();
            eprintln!("        #{pd} {name}");
        }
        eprintln!("        (isolate one with --filter \"<name>\" or --filter \"#<id>\")");
    }

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
        deflection: deflection_file,
        max_angle: args.max_angle.to_radians(),
    };
    let mut stats = TessStats::default();
    let mut builder = glb::GlbBuilder::default();

    // mesh cache: per product-definition, plus md5 content dedup across PDs
    let colors = styles::build_color_map(&sf);
    if !colors.is_empty() && args.stats {
        eprintln!("found {} styled (colored) items", colors.len());
    }
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
        if args.split.is_some() {
            eprintln!(
                "warning: --split is ignored with --merged (merge collapses geometry by \
                 color); drop --merged to get one node per solid/shell/face"
            );
        }
        let opts = merge::MergeOptions {
            unit_scale: output_scale,
            file_unit_scale,
            rotate_z_up: args.up_axis == UpAxis::Z,
            optimize: !args.no_optimize,
            drop_normals: !args.normals,
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
            "merged {} draw calls ({} unique meshes, ~{:.1}x expansion) \
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

    // each product's geometry as one or more (node-label, mesh-index) units:
    // one merged unit normally, or one per solid/shell/face under --split
    let mut units_of_pd: HashMap<u32, Vec<(String, usize)>> = HashMap::new();
    let mut mesh_of_hash: HashMap<[u8; 16], usize> = HashMap::new();
    let split = args.split.map(|s| s.level());

    let t1 = Instant::now();
    let mut build_units =
        |pd: u32, builder: &mut glb::GlbBuilder, stats: &mut TessStats| -> Vec<(String, usize)> {
            if let Some(cached) = units_of_pd.get(&pd) {
                return cached.clone();
            }
            let node = asm.products.get(&pd);
            // weld/prepare a finished MeshSet, then dedup it into a mesh index
            // so identical geometry (instanced parts) is stored once
            let add = |mut tm: MeshSet,
                       name: String,
                       builder: &mut glb::GlbBuilder,
                       mesh_of_hash: &mut HashMap<[u8; 16], usize>|
             -> Option<usize> {
                prepare_mesh(&mut tm, &args);
                if tm.is_empty() {
                    return None;
                }
                let h = tm.content_hash();
                Some(match mesh_of_hash.get(&h) {
                    Some(&i) => i,
                    None => {
                        let i = builder.add_mesh(tm, name);
                        mesh_of_hash.insert(h, i);
                        i
                    }
                })
            };
            let mut units: Vec<(String, usize)> = Vec::new();
            let mut merged = MeshSet::default();
            if let Some(node) = node {
                for &sr in &node.shape_reps {
                    // SHAPE_REPRESENTATION('', (items), context). Honour this
                    // representation's own length unit (Autodesk mixes mm and
                    // metre contexts in one file): tessellate in the rep's unit
                    // (deflection scaled to match, so --deflection stays in mm),
                    // then scale the geometry into the global unit.
                    let factor = model::rep_unit_factor(&sf, sr, file_unit_scale);
                    let rep_tp = TessParams {
                        deflection: cx.tp.deflection / factor,
                        max_angle: cx.tp.max_angle,
                    };
                    let rep_cx = tessellate::Ctx {
                        sf: cx.sf,
                        tp: &rep_tp,
                        colors: cx.colors,
                        threads: cx.threads,
                    };
                    let mut items: Vec<u32> = Vec::new();
                    if let Some(p) = sf.params(sr) {
                        if let Some(list) = p.get(1).and_then(|v| v.as_list()) {
                            items.extend(list.iter().filter_map(|v| v.as_ref_id()));
                        }
                    }
                    let scale = |mut sub: MeshSet| -> MeshSet {
                        if (factor - 1.0).abs() > 1e-9 {
                            sub.transform(&M4::scale_uniform(factor));
                        }
                        sub
                    };
                    match split {
                        // default: merge every item of every rep into one mesh
                        None => {
                            let mut sub = MeshSet::default();
                            for r in items {
                                tessellate::tessellate_item(&rep_cx, r, None, &mut sub, stats);
                            }
                            merged.append(&scale(sub));
                        }
                        // debug: each solid/shell/face becomes its own node,
                        // named <ENTITY_TYPE>#<id> for cross-referencing
                        Some(level) => {
                            for r in items {
                                for gid in tessellate::split_units(&sf, r, level) {
                                    let mut sub = MeshSet::default();
                                    tessellate::tessellate_item(
                                        &rep_cx, gid, None, &mut sub, stats,
                                    );
                                    let label = format!(
                                        "{}#{}",
                                        sf.entity_type(gid).unwrap_or("ENTITY"),
                                        gid
                                    );
                                    if let Some(mi) =
                                        add(scale(sub), label.clone(), builder, &mut mesh_of_hash)
                                    {
                                        units.push((label, mi));
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if split.is_none() {
                let name = node
                    .map(|n| n.name.clone())
                    .unwrap_or_else(|| format!("PD#{}", pd));
                if let Some(mi) = add(merged, name.clone(), builder, &mut mesh_of_hash) {
                    units.push((name, mi));
                }
            }
            units_of_pd.insert(pd, units.clone());
            units
        };

    // -------------------------------------------------------- node expansion
    fn expand(
        asm: &hierarchy::Assembly,
        pd: u32,
        name: &str,
        transform: Option<M4>,
        builder: &mut glb::GlbBuilder,
        build_units: &mut dyn FnMut(u32, &mut glb::GlbBuilder, &mut TessStats) -> Vec<(String, usize)>,
        stats: &mut TessStats,
        split_on: bool,
        depth: usize,
        budget: &mut i64,
    ) -> Option<usize> {
        if depth > 64 || *budget <= 0 {
            return None;
        }
        *budget -= 1;
        let units = build_units(pd, builder, stats);
        // without --split the product node carries its one merged mesh; with
        // --split it carries none and each unit hangs off it as a child node
        let node_mesh = if split_on {
            None
        } else {
            units.first().map(|(_, mi)| *mi)
        };
        let node = builder.add_node(name.to_string(), transform, node_mesh);
        let mut children: Vec<usize> = Vec::new();
        if split_on {
            for (label, mi) in &units {
                children.push(builder.add_node(label.clone(), None, Some(*mi)));
            }
        }
        if let Some(kids) = asm.children.get(&pd) {
            for k in kids {
                if let Some(c) = expand(
                    asm,
                    k.child_pd,
                    &k.name,
                    Some(k.transform),
                    builder,
                    build_units,
                    stats,
                    split_on,
                    depth + 1,
                    budget,
                ) {
                    children.push(c);
                }
            }
        }
        builder.nodes[node].children = children;
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
            &mut build_units,
            &mut stats,
            split.is_some(),
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
    let mut root_m = M4::scale_uniform(output_scale);
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

/// Effective tessellation worker count: the `--threads` value if positive,
/// else auto (CPU cores capped at 4).
fn resolve_threads(args: &Args) -> usize {
    args.threads.filter(|&n| n > 0).unwrap_or_else(|| {
        std::thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(1)
            .min(4)
    })
}

/// STEP length-unit name for a file-unit→metre scale factor (for display).
fn unit_label(scale_to_m: f64) -> &'static str {
    for (s, name) in [
        (1e-6, "µm"),
        (0.001, "mm"),
        (0.01, "cm"),
        (0.1, "dm"),
        (1.0, "m"),
        (1000.0, "km"),
        (0.0254, "inch"),
        (0.3048, "foot"),
    ] {
        if (scale_to_m / s - 1.0).abs() < 1e-6 {
            return name;
        }
    }
    "file units"
}

/// Echo the effective configuration (resolved defaults included) at startup, so
/// a run's settings are visible without re-deriving them from the command line.
/// `file_unit_scale` is the detected file-unit→metre factor.
fn print_settings(args: &Args, threads: usize, file_unit_scale: f64) {
    let onoff = |b: bool| if b { "on" } else { "off" };
    let out = args
        .output
        .clone()
        .unwrap_or_else(|| args.input.with_extension("glb"));
    let tree_only = args.tree && !args.stats;
    let unit = unit_label(file_unit_scale);
    let mm_per_unit = file_unit_scale * 1000.0;

    eprintln!("settings:");
    eprintln!("  input             {}", args.input.display());
    if tree_only {
        eprintln!("  mode              tree (print hierarchy, no GLB written)");
    } else {
        eprintln!("  output            {}", out.display());
        eprintln!(
            "  mode              {}",
            if args.merged {
                "merged (one node/mesh per color, baked to world space)"
            } else {
                "hierarchical (per-part nodes)"
            }
        );
    }
    // deflection is always given in mm; for non-mm files show what that is in
    // the file's own unit (what tessellation actually uses)
    if (mm_per_unit - 1.0).abs() < 1e-9 {
        eprintln!("  deflection        {} mm", args.deflection);
    } else {
        eprintln!(
            "  deflection        {} mm (= {:.5} {} in file units)",
            args.deflection,
            args.deflection / mm_per_unit,
            unit
        );
    }
    eprintln!("  max-angle         {}°", args.max_angle);
    eprintln!(
        "  threads           {}{}",
        threads,
        if args.threads.filter(|&n| n > 0).is_none() {
            " (auto)"
        } else {
            ""
        }
    );
    eprintln!(
        "  up-axis           {}",
        match args.up_axis {
            UpAxis::Z => "z (rotate to glTF Y-up)",
            UpAxis::Y => "y (kept as-is)",
        }
    );
    eprintln!(
        "  unit-scale        {}",
        if args.no_unit_scale {
            format!("off (output kept in {unit})")
        } else {
            format!("{unit} → meters")
        }
    );
    // normals: --cleanup-position always drops them, so show the effective state
    if args.normals && args.cleanup_position {
        eprintln!("  normals           off (dropped by --cleanup-position)");
    } else {
        eprintln!("  normals           {}", onoff(args.normals));
    }
    // meshoptimizer is applied per item — once per unique mesh in the default
    // (hierarchical) output, once per part instance in merged mode
    let item = if args.merged { "per part" } else { "per unique mesh" };
    if args.no_optimize {
        eprintln!("  optimize          off");
    } else {
        eprintln!("  optimize          on ({item})");
        eprintln!(
            "    meshopt         weld duplicates{}, vertex-cache, vertex-fetch",
            if args.normals && !args.cleanup_position {
                ""
            } else {
                " (by position)"
            }
        );
    }
    let simplify = |what: &str, th: f32, te: f32| {
        eprintln!("  {what}");
        eprintln!(
            "    meshopt         simplify → {:.0}% of indices, target-error {}, border locked",
            th * 100.0,
            te
        );
    };
    if args.cleanup_position {
        simplify(
            &format!(
                "cleanup-position  on (weld grid {} decimals, {item})",
                args.cleanup_precision
            ),
            simplify_threshold(args),
            simplify_target_error(args),
        );
    } else if let Some((th, te)) = simplify_only(args) {
        eprintln!("  cleanup-position  off");
        simplify(&format!("simplify          on ({item})"), th, te);
    } else {
        eprintln!("  cleanup-position  off");
    }
    if args.stats {
        eprintln!("  stats             on");
    }
    if args.tree && args.stats {
        eprintln!("  tree              on (with --stats: prints tree and still writes GLB)");
    }
    if args.debug_print {
        eprintln!(
            "  debug-print       on (failing-face excerpts → {})",
            args.input.with_extension("debug.txt").display()
        );
    }
    if let Some(q) = &args.filter {
        eprintln!("  filter            {q:?} (isolate matching subtree)");
    }
    if let Some(p) = &args.extract_step {
        eprintln!("  extract-step      {} (write subtree STEP, no GLB)", p.display());
    }
    if let Some(s) = args.split {
        let level = match s {
            SplitArg::Solid => "solid (one node per solid)",
            SplitArg::Shell => "shell (one node per CLOSED_SHELL)",
            SplitArg::Face => "face (one node per ADVANCED_FACE)",
        };
        eprintln!("  split             {level}");
    }
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
    if !args.normals {
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
    // print a "count  TYPE" table, most frequent first
    let table = |title: &str, m: &std::collections::HashMap<String, usize>| {
        if m.is_empty() {
            return;
        }
        let mut v: Vec<_> = m.iter().collect();
        v.sort_by(|a, b| b.1.cmp(a.1).then(a.0.cmp(b.0)));
        eprintln!("{title}");
        for (ty, n) in v {
            eprintln!("  {:>6}  {}", n, ty);
        }
    };
    table(
        "unsupported surface types (faces skipped):",
        &stats.unsupported_surfaces,
    );
    table(
        "unsupported surface types approximated as a flat plane (curvature lost):",
        &stats.approximated_surfaces,
    );
    table(
        "unsupported edge curve types (boundary straightened to a chord):",
        &stats.unsupported_curves,
    );
    table(
        "unsupported representation items (no geometry produced):",
        &stats.unsupported_items,
    );
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

/// `--filter --extract-step`: write a self-contained, re-runnable STEP excerpt
/// of the matched subtree — its product structure, shape/representation linkage
/// and geometry (see [`hierarchy::subtree_entities`]) — so a missing or wrong
/// element can be inspected or shared in isolation.
fn write_step_excerpt(sf: &StepFile, ids: &[u32], path: &std::path::Path) {
    let mut out = String::from("ISO-10303-21;\n");
    out.push_str(
        "/* step2glb --filter --extract-step: a matched element plus the transitive\n\
        \x20  closure of everything it references — product structure, shape and\n\
        \x20  representation linkage (relationships followed multi-hop), geometry.\n\
        \x20  A standalone, re-runnable STEP file. */\n",
    );
    let (h0, h1) = sf.header_range;
    if h1 > h0 && h1 <= sf.data.len() {
        out.push_str(&String::from_utf8_lossy(&sf.data[h0..h1]));
        if !out.ends_with('\n') {
            out.push('\n');
        }
    } else {
        out.push_str("HEADER;\nENDSEC;\n");
    }
    out.push_str("DATA;\n");
    for &id in ids {
        if let Some(line) = sf.entity_source(id) {
            out.push_str(&line);
            out.push('\n');
        }
    }
    out.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
    match std::fs::write(path, out) {
        Ok(()) => eprintln!("--extract-step: wrote {} entities to {}", ids.len(), path.display()),
        Err(e) => eprintln!("--extract-step: cannot write {}: {}", path.display(), e),
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

/// The model's global `LENGTH_UNIT` as a scale factor to meters (mm if the file
/// declares none). Shares [`model::file_length_scale`] with the per-instance
/// transform unit handling so geometry and placements scale consistently.
fn detect_length_unit(sf: &StepFile) -> f64 {
    model::file_length_scale(sf).unwrap_or_else(|| {
        eprintln!("warn: no length unit found, assuming millimetres");
        0.001
    })
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
