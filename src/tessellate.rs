//! B-rep traversal and face tessellation.
//!
//! Faces are triangulated in the surface's UV space with tess2 (odd winding
//! handles holes), refined by midpoint edge subdivision until both the
//! parametric step limits and a 3D chord-deviation criterion are met, then
//! mapped back to 3D with analytic/finite-difference normals.
//!
//! Surface support:
//! - plane + the quadrics (closed-form UV inversion)
//! - surfaces of linear extrusion / revolution (reduced to quadrics where
//!   possible, otherwise seeded 1D Newton inversion)
//! - (rational) B-spline surfaces, trimmed via seeded 2D Newton projection
//! - faces that wrap fully around a periodic direction (cylinder bands,
//!   closed revolutions, ...) are cut at a seam and rebuilt as band polygons
//! - boundary loops that encircle a sphere pole / cone apex are closed with a
//!   synthetic polar cap
//! - AP242 tessellated items are taken verbatim
//!
//! Colors from STYLED_ITEM are carried per item/face into per-color buckets
//! (-> one glTF primitive + material each).

use std::collections::HashMap;

use tess2_rust::tess::TESS_UNDEF;
use tess2_rust::{ElementType, Tessellator, WindingRule};

use crate::geom::*;
use crate::mesh::{MeshSet, TriMesh};
use crate::model::{self, TessParams};
use crate::step::StepFile;
use crate::styles::ColorMap;

#[derive(Default)]
pub struct TessStats {
    pub faces_ok: usize,
    pub faces_failed: usize,
    pub unsupported_surfaces: HashMap<String, usize>,
    /// supported surface types whose trimming/tessellation still failed
    /// (Newton non-convergence, multi-winding loops, degenerate bounds, …):
    /// count plus a few sample ADVANCED_FACE entity ids for diagnosis
    pub failed_surfaces: HashMap<String, (usize, Vec<u32>)>,
    /// first failing ADVANCED_FACE per surface type, plus the stage that
    /// failed — used by `--debug-print` to dump a self-contained sub-graph
    pub debug_samples: HashMap<String, (u32, &'static str)>,
}

impl TessStats {
    /// Fold another stats record in (used by the parallel face workers).
    pub fn merge(&mut self, o: &TessStats) {
        self.faces_ok += o.faces_ok;
        self.faces_failed += o.faces_failed;
        for (k, v) in &o.unsupported_surfaces {
            *self.unsupported_surfaces.entry(k.clone()).or_insert(0) += v;
        }
        for (k, (n, ids)) in &o.failed_surfaces {
            let e = self
                .failed_surfaces
                .entry(k.clone())
                .or_insert((0, Vec::new()));
            e.0 += n;
            for id in ids {
                if e.1.len() < 5 {
                    e.1.push(*id);
                }
            }
        }
        for (k, v) in &o.debug_samples {
            self.debug_samples.entry(k.clone()).or_insert(*v);
        }
    }
}

/// Shared, read-only context for a tessellation run.
pub struct Ctx<'a> {
    pub sf: &'a StepFile,
    pub tp: &'a TessParams,
    pub colors: &'a ColorMap,
    /// worker threads for per-face fan-out (1 = serial)
    pub threads: usize,
}

impl<'a> Ctx<'a> {
    pub fn color_of(&self, id: u32, inherited: Option<[f32; 4]>) -> Option<[f32; 4]> {
        self.colors.get(&id).copied().or(inherited)
    }
}

/// Tessellate any geometric representation item we understand into `out`.
/// Returns true if the item produced (or may produce) triangles.
pub fn tessellate_item(
    cx: &Ctx,
    id: u32,
    inherited: Option<[f32; 4]>,
    out: &mut MeshSet,
    stats: &mut TessStats,
) -> bool {
    let sf = cx.sf;
    let color = cx.color_of(id, inherited);
    let ty = match sf.entity_type(id) {
        Some(t) => t.to_string(),
        None => return false,
    };
    match ty.as_str() {
        "MANIFOLD_SOLID_BREP" | "FACETED_BREP" => {
            if let Some(p) = sf.params(id) {
                if let Some(shell) = p.get(1).and_then(|v| v.as_ref_id()) {
                    tessellate_shell(cx, shell, color, out, stats);
                    return true;
                }
            }
            false
        }
        "BREP_WITH_VOIDS" => {
            if let Some(p) = sf.params(id) {
                if let Some(shell) = p.get(1).and_then(|v| v.as_ref_id()) {
                    tessellate_shell(cx, shell, color, out, stats);
                }
                if let Some(voids) = p.get(2).and_then(|v| v.as_list()) {
                    for v in voids {
                        if let Some(r) = v.as_ref_id() {
                            tessellate_shell(cx, resolve_oriented_shell(sf, r), color, out, stats);
                        }
                    }
                }
                return true;
            }
            false
        }
        "SHELL_BASED_SURFACE_MODEL" | "FACE_BASED_SURFACE_MODEL" => {
            if let Some(p) = sf.params(id) {
                if let Some(shells) = p.get(1).and_then(|v| v.as_list()) {
                    for s in shells {
                        if let Some(r) = s.as_ref_id() {
                            tessellate_shell(cx, resolve_oriented_shell(sf, r), color, out, stats);
                        }
                    }
                    return true;
                }
            }
            false
        }
        "TRIANGULATED_FACE_SET" | "TRIANGULATED_SURFACE_SET" => {
            tessellate_triangulated_set(sf, id, out.bucket(color));
            true
        }
        "TESSELLATED_SOLID" | "TESSELLATED_SHELL" => {
            if let Some(p) = sf.params(id) {
                if let Some(items) = p.get(1).and_then(|v| v.as_list()) {
                    for it in items {
                        if let Some(r) = it.as_ref_id() {
                            tessellate_item(cx, r, color, out, stats);
                        }
                    }
                    return true;
                }
            }
            false
        }
        "MAPPED_ITEM" => {
            if let Some(p) = sf.params(id) {
                let map = p.get(1).and_then(|v| v.as_ref_id());
                let target = p.get(2).and_then(|v| v.as_ref_id());
                if let Some(map) = map {
                    let mut sub = MeshSet::default();
                    let mut any = false;
                    if let Some(mp) = sf.params(map) {
                        let origin = mp.first().and_then(|v| v.as_ref_id());
                        let rep = mp.get(1).and_then(|v| v.as_ref_id());
                        if let Some(rep) = rep {
                            if let Some(rp) = sf.params(rep) {
                                if let Some(items) = rp.get(1).and_then(|v| v.as_list()) {
                                    for it in items {
                                        if let Some(r) = it.as_ref_id() {
                                            any |= tessellate_item(cx, r, color, &mut sub, stats);
                                        }
                                    }
                                }
                            }
                        }
                        if any {
                            let m_origin = origin
                                .map(|o| model::axis2_matrix(sf, o))
                                .unwrap_or(M4::IDENTITY);
                            let m_target = target
                                .map(|t| model::axis2_matrix(sf, t))
                                .unwrap_or(M4::IDENTITY);
                            sub.transform(&m_target.mul(m_origin.inverse_rigid()));
                            out.append(&sub);
                        }
                    }
                    return any;
                }
            }
            false
        }
        "ADVANCED_FACE" | "FACE_SURFACE" => {
            tessellate_face(cx, id, color, out, stats);
            true
        }
        _ => false,
    }
}

fn resolve_oriented_shell(sf: &StepFile, id: u32) -> u32 {
    if sf.entity_type(id) == Some("ORIENTED_CLOSED_SHELL") {
        if let Some(p) = sf.params(id) {
            if let Some(r) = p.get(2).and_then(|v| v.as_ref_id()) {
                return r;
            }
        }
    }
    id
}

fn tessellate_shell(
    cx: &Ctx,
    shell: u32,
    inherited: Option<[f32; 4]>,
    out: &mut MeshSet,
    stats: &mut TessStats,
) {
    let p = match cx.sf.params(shell) {
        Some(p) => p,
        None => return,
    };
    let faces = match p.get(1).and_then(|v| v.as_list()) {
        Some(f) => f.to_vec(),
        None => return,
    };
    let color = cx.color_of(shell, inherited);
    let ids: Vec<u32> = faces.iter().filter_map(|f| f.as_ref_id()).collect();
    tessellate_faces(cx, &ids, color, out, stats);
}

/// Tessellate faces, fanning out over `cx.threads` workers. Each face fills
/// its own mesh/stats; results merge in face order, so the output is
/// byte-identical to the serial path regardless of thread count.
fn tessellate_faces(
    cx: &Ctx,
    faces: &[u32],
    color: Option<[f32; 4]>,
    out: &mut MeshSet,
    stats: &mut TessStats,
) {
    if cx.threads <= 1 || faces.len() < 2 {
        for &fid in faces {
            tessellate_face(cx, fid, color, out, stats);
        }
        return;
    }
    let next = std::sync::atomic::AtomicUsize::new(0);
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::scope(|s| {
        for _ in 0..cx.threads.min(faces.len()) {
            let tx = tx.clone();
            let next = &next;
            s.spawn(move || loop {
                let i = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                if i >= faces.len() {
                    break;
                }
                let mut m = MeshSet::default();
                let mut st = TessStats::default();
                tessellate_face(cx, faces[i], color, &mut m, &mut st);
                if tx.send((i, m, st)).is_err() {
                    break;
                }
            });
        }
        drop(tx);
        let mut results: Vec<Option<(MeshSet, TessStats)>> =
            (0..faces.len()).map(|_| None).collect();
        for (i, m, st) in rx {
            results[i] = Some((m, st));
        }
        for r in results.into_iter().flatten() {
            out.append(&r.0);
            stats.merge(&r.1);
        }
    });
}

// --------------------------------------------------------------------- face

/// One boundary loop in 3D plus the orientation it carries in the file.
struct Loop3 {
    /// boundary polyline with FACE_BOUND orientation already applied
    pts: Vec<V3>,
}

fn tessellate_face(
    cx: &Ctx,
    face: u32,
    inherited: Option<[f32; 4]>,
    out: &mut MeshSet,
    stats: &mut TessStats,
) {
    let sf = cx.sf;
    let tp = cx.tp;
    // ADVANCED_FACE('', (bounds), surface, same_sense)
    let p = match sf.params(face) {
        Some(p) => p,
        None => return,
    };
    let bounds = match p.get(1).and_then(|v| v.as_list()) {
        Some(b) => b.to_vec(),
        None => return,
    };
    let surf_id = match p.get(2).and_then(|v| v.as_ref_id()) {
        Some(s) => s,
        None => return,
    };
    let same_sense = p.get(3).and_then(|v| v.as_bool()).unwrap_or(true);
    let color = cx.color_of(face, inherited);
    let mesh = out.bucket(color);

    let surf = model::surface(sf, surf_id);

    let mut loops3d: Vec<Loop3> = Vec::new();
    for b in &bounds {
        let bid = match b.as_ref_id() {
            Some(b) => b,
            None => continue,
        };
        // FACE_BOUND / FACE_OUTER_BOUND ('', loop, orientation)
        let bp = match sf.params(bid) {
            Some(bp) => bp,
            None => continue,
        };
        let loop_id = match bp.get(1).and_then(|v| v.as_ref_id()) {
            Some(l) => l,
            None => continue,
        };
        let orientation = bp.get(2).and_then(|v| v.as_bool()).unwrap_or(true);
        if let Some(mut lp) = loop_polyline(sf, loop_id, tp) {
            if lp.len() >= 3 {
                if !orientation {
                    lp.reverse();
                }
                loops3d.push(Loop3 { pts: lp });
            }
        }
    }
    if loops3d.is_empty() {
        // full closed quadric with no bounds: tessellate the whole domain
        if let Some(s) = &surf {
            if tessellate_unbounded(s, tp, same_sense, mesh) {
                stats.faces_ok += 1;
                return;
            }
        }
        stats.faces_failed += 1;
        record_failed(
            stats,
            sf,
            surf_id,
            face,
            "no usable boundary loops (edge discretization failed or <3 points) \
             and not a closed quadric",
        );
        return;
    }

    let surf = match surf {
        Some(s) => s,
        None => {
            // Unknown surface: near-planar boundary fallback (POLY_LOOP breps)
            match fit_plane(&loops3d) {
                Some(pl) => pl,
                None => {
                    *stats
                        .unsupported_surfaces
                        .entry(surface_type_name(sf, surf_id))
                        .or_insert(0) += 1;
                    stats.faces_failed += 1;
                    return;
                }
            }
        }
    };

    match face_to_mesh(&surf, &loops3d, tp, same_sense, mesh) {
        Ok(()) => stats.faces_ok += 1,
        Err(reason) => {
            stats.faces_failed += 1;
            record_failed(stats, sf, surf_id, face, reason);
        }
    }
}

/// Track a failed face: per-surface-type count plus a few sample face ids so
/// the offending entities can be looked up in the STEP file. The first face of
/// each type also records the failing stage for `--debug-print`.
fn record_failed(
    stats: &mut TessStats,
    sf: &StepFile,
    surf_id: u32,
    face: u32,
    reason: &'static str,
) {
    let ty = surface_type_name(sf, surf_id);
    stats
        .debug_samples
        .entry(ty.clone())
        .or_insert((face, reason));
    let e = stats.failed_surfaces.entry(ty).or_insert((0, Vec::new()));
    e.0 += 1;
    if e.1.len() < 5 {
        e.1.push(face);
    }
}

/// STEP type name of a surface entity, resolving complex instances to their
/// most specific SURFACE leaf.
fn surface_type_name(sf: &StepFile, surf_id: u32) -> String {
    match sf.entity_type(surf_id) {
        Some(crate::step::TYPE_COMPLEX) => {
            let leaves = sf.complex_leaf_names(surf_id);
            leaves
                .iter()
                .find(|l| l.contains("SURFACE") && !l.contains("BOUNDED"))
                .cloned()
                .unwrap_or_else(|| leaves.join("+"))
        }
        Some(t) => t.to_string(),
        None => "?".to_string(),
    }
}

fn loop_polyline(sf: &StepFile, loop_id: u32, tp: &TessParams) -> Option<Vec<V3>> {
    let ty = sf.entity_type(loop_id)?;
    match ty {
        "EDGE_LOOP" => {
            let p = sf.params(loop_id)?;
            let edges = p.get(1)?.as_list()?;
            let mut pts: Vec<V3> = Vec::new();
            for e in edges {
                let eid = e.as_ref_id()?;
                let ep = model::edge_polyline(sf, eid, tp)?;
                let skip = usize::from(
                    !pts.is_empty()
                        && pts
                            .last()
                            .map(|l| l.sub(ep[0]).len() < 1e-9)
                            .unwrap_or(false),
                );
                pts.extend_from_slice(&ep[skip..]);
            }
            if pts.len() > 1 && pts[0].sub(*pts.last().unwrap()).len() < 1e-9 {
                pts.pop();
            }
            Some(pts)
        }
        "POLY_LOOP" => {
            let p = sf.params(loop_id)?;
            let pts: Vec<V3> = p
                .get(1)?
                .as_list()?
                .iter()
                .filter_map(|v| v.as_ref_id())
                .filter_map(|r| model::cartesian_point(sf, r))
                .collect();
            Some(pts)
        }
        "VERTEX_LOOP" => Some(Vec::new()),
        _ => None,
    }
}

/// Newell-plane fit fallback for faces with unknown surface types.
fn fit_plane(loops: &[Loop3]) -> Option<Surface> {
    let outer = &loops.first()?.pts;
    if outer.len() < 3 {
        return None;
    }
    let mut n = V3::ZERO;
    let mut c = V3::ZERO;
    for i in 0..outer.len() {
        let a = outer[i];
        let b = outer[(i + 1) % outer.len()];
        n = n.add(v3(
            (a.y - b.y) * (a.z + b.z),
            (a.z - b.z) * (a.x + b.x),
            (a.x - b.x) * (a.y + b.y),
        ));
        c = c.add(a);
    }
    if n.len() < 1e-12 {
        return None;
    }
    let n = n.norm();
    let c = c.scale(1.0 / outer.len() as f64);
    let mut span: f64 = 0.0;
    let mut dev: f64 = 0.0;
    for l in loops {
        for p in &l.pts {
            dev = dev.max((p.sub(c).dot(n)).abs());
            span = span.max(p.sub(c).len());
        }
    }
    if dev > 1e-3 * span.max(1e-9) + 1e-6 {
        return None;
    }
    Some(Surface::Plane(Frame::new(c, Some(n), None)))
}

// ------------------------------------------------------------ UV tess + map

/// Per-loop UV polyline plus winding bookkeeping.
struct LoopUv {
    uv: Vec<[f64; 2]>,
    /// net winding around the u period (0 for ordinary loops)
    w: i32,
    /// interior of the face is on the +v side of this (winding) loop
    interior_above: bool,
}

fn face_to_mesh(
    surf: &Surface,
    loops3d: &[Loop3],
    tp: &TessParams,
    same_sense: bool,
    mesh: &mut TriMesh,
) -> Result<(), &'static str> {
    let per_u = surf.u_period();
    let per_v = surf.v_period();

    // 3D positions of parameterization singularities (cone apex, sphere
    // poles): boundary points there carry no u information of their own.
    let eps_cap = 1e-7 * surf.approx_size().max(1.0);
    let cap_pts: Vec<crate::geom::V3> = match (per_u, surf.v_caps()) {
        (Some(_), Some((vb, vt))) => [vb, vt]
            .iter()
            .filter(|v| v.is_finite())
            .map(|&v| surf.point(0.0, v))
            .collect(),
        _ => Vec::new(),
    };
    let singular =
        |p: crate::geom::V3| cap_pts.iter().any(|c| p.sub(*c).len() < eps_cap);

    // 1) map loops to UV with periodic unwrapping; Newton surfaces are seeded
    // from the previous boundary point for continuity and speed.
    let mut loops_uv: Vec<LoopUv> = Vec::new();
    let mut u_ref: Option<f64> = None;
    for l in loops3d {
        // start each loop off-singularity, so a singular point's u (taken
        // from the uv() hint) is the u of its incoming boundary edge
        let n = l.pts.len();
        let start = (0..n).find(|&i| !singular(l.pts[i])).unwrap_or(0);
        let pts: Vec<crate::geom::V3> = (0..n).map(|i| l.pts[(start + i) % n]).collect();

        let mut uv: Vec<[f64; 2]> = Vec::with_capacity(n);
        let mut sing: Vec<bool> = Vec::with_capacity(n);
        let mut hint: Option<(f64, f64)> = None;
        for (i, p) in pts.iter().enumerate() {
            let (mut u, mut v) = surf.uv(*p, hint);
            hint = Some((u, v));
            if i > 0 {
                let pu = uv[i - 1][0];
                let pv = uv[i - 1][1];
                if let Some(per) = per_u {
                    while u - pu > per / 2.0 {
                        u -= per;
                    }
                    while pu - u > per / 2.0 {
                        u += per;
                    }
                }
                if let Some(per) = per_v {
                    while v - pv > per / 2.0 {
                        v -= per;
                    }
                    while pv - v > per / 2.0 {
                        v += per;
                    }
                }
            }
            uv.push([u, v]);
            sing.push(singular(*p));
        }
        // A singular point belongs to both adjacent meridians: walk the cap
        // line from the incoming edge's u to the outgoing edge's u (sampled
        // at the u step, like the periodic-band polar caps, so the polar fan
        // triangulates cleanly) instead of cutting diagonally across the
        // domain.
        if let Some(per) = per_u {
            if sing.iter().any(|&s| s) {
                let du = surf.u_step(tp.deflection, tp.max_angle);
                let mut out: Vec<[f64; 2]> = Vec::with_capacity(uv.len() + 8);
                for i in 0..uv.len() {
                    out.push(uv[i]);
                    if sing[i] {
                        let pu = uv[i][0];
                        let mut nu = uv[(i + 1) % uv.len()][0];
                        while nu - pu > per / 2.0 {
                            nu -= per;
                        }
                        while pu - nu > per / 2.0 {
                            nu += per;
                        }
                        let steps = (((nu - pu).abs() / du).ceil() as usize).clamp(1, 256);
                        for k in 1..=steps {
                            out.push([pu + (nu - pu) * k as f64 / steps as f64, uv[i][1]]);
                        }
                    }
                }
                uv = out;
            }
        }
        // net winding around u: unwrap the implicit closing edge back to p0
        let w = match per_u {
            Some(per) if uv.len() >= 3 => {
                let mut uc = uv[0][0];
                let last = uv[uv.len() - 1][0];
                while uc - last > per / 2.0 {
                    uc -= per;
                }
                while last - uc > per / 2.0 {
                    uc += per;
                }
                ((uc - uv[0][0]) / per).round() as i32
            }
            _ => 0,
        };
        // With the face normal = surface normal * same_sense, boundary loops
        // run counter-clockwise around the interior in UV iff same_sense.
        // For a loop winding in +u, "interior on the left" means +v.
        let interior_above = (w > 0) == same_sense;
        // align non-winding loops with each other in the periodic direction
        if let Some(per) = per_u {
            if w == 0 {
                let mean: f64 = uv.iter().map(|p| p[0]).sum::<f64>() / uv.len().max(1) as f64;
                match u_ref {
                    None => u_ref = Some(mean),
                    Some(r) => {
                        let shift = ((mean - r) / per).round() * per;
                        if shift != 0.0 {
                            for p in &mut uv {
                                p[0] -= shift;
                            }
                        }
                    }
                }
            }
        }
        loops_uv.push(LoopUv {
            uv,
            w,
            interior_above,
        });
    }

    // A boundary whose unwrapped UV polygon encloses (next to) no area is a
    // seam "slit" — an edge walked out and back. Some exporters write a full
    // closed surface as a single face bounded only by such a slit; tessellate
    // the whole domain instead. (Tested in UV, not 3D: a one-loop cylinder
    // band legitimately cancels in 3D but is a real rectangle in UV.)
    let uv_slit = |l: &LoopUv| -> bool {
        if l.w != 0 {
            return false;
        }
        let mut area = 0.0;
        let mut per = 0.0;
        for i in 0..l.uv.len() {
            let a = l.uv[i];
            let b = l.uv[(i + 1) % l.uv.len()];
            area += a[0] * b[1] - b[0] * a[1];
            per += ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
        }
        area.abs() * 0.5 < 1e-9 * per * per
    };
    if loops_uv.iter().all(uv_slit) {
        return ok_or(
            tessellate_unbounded(surf, tp, same_sense, mesh),
            "slit/full-surface tessellation failed (surface not closed-form invertible)",
        );
    }

    if loops_uv.iter().any(|l| l.w != 0) {
        return ok_or(
            tessellate_periodic_band(surf, &loops_uv, tp, same_sense, mesh),
            "periodic-band (wrap-around) tessellation failed \
             (multi-winding loop or seam reconstruction)",
        );
    }

    let contours: Vec<Vec<[f64; 2]>> = loops_uv.into_iter().map(|l| l.uv).collect();
    ok_or(
        emit_uv_region(surf, &contours, tp, same_sense, mesh),
        "UV tessellation produced no triangles \
         (tess2 failed: degenerate or self-intersecting UV contour)",
    )
}

fn ok_or(success: bool, reason: &'static str) -> Result<(), &'static str> {
    if success {
        Ok(())
    } else {
        Err(reason)
    }
}

/// Triangulate closed UV contours (odd winding -> holes), refine for
/// curvature + chord deviation, map to 3D and append to `mesh`.
fn emit_uv_region(
    surf: &Surface,
    loops_uv: &[Vec<[f64; 2]>],
    tp: &TessParams,
    same_sense: bool,
    mesh: &mut TriMesh,
) -> bool {
    // Scale UV close to the 3D metric so tess2 triangulates in (almost)
    // isometric coordinates. Anisotropic UV (e.g. a sphere strip) otherwise
    // produces needle triangles that can fold when mapped onto curvature.
    let (su, sv) = {
        let (mut umin, mut umax, mut vmin, mut vmax) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
        for l in loops_uv {
            for p in l {
                umin = umin.min(p[0]);
                umax = umax.max(p[0]);
                vmin = vmin.min(p[1]);
                vmax = vmax.max(p[1]);
            }
        }
        let du = (umax - umin).max(1e-12);
        let dv = (vmax - vmin).max(1e-12);
        let (uc, vc) = ((umin + umax) * 0.5, (vmin + vmax) * 0.5);
        let h = 1e-4;
        let lu = surf
            .point(uc + h * du, vc)
            .sub(surf.point(uc - h * du, vc))
            .len()
            / (2.0 * h * du);
        let lv = surf
            .point(uc, vc + h * dv)
            .sub(surf.point(uc, vc - h * dv))
            .len()
            / (2.0 * h * dv);
        let size = (lu * du).max(lv * dv).max(1e-12);
        // metric scale, normalized so the larger extent maps to ~1
        ((lu / size).max(1e-9 / du), (lv / size).max(1e-9 / dv))
    };

    let (verts_uv, mut tris) = match run_tess2(loops_uv, su, sv) {
        Some(r) => r,
        None => return false,
    };
    // tess2 emits zero-area triangles along collinear contour stretches
    // (e.g. a meridian boundary). They cover nothing in UV, but mapped onto
    // a curved surface their corners no longer align — phantom secant
    // triangles that refinement then multiplies. Drop them up front.
    let (mut umin, mut umax, mut vmin, mut vmax) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for p in &verts_uv {
        umin = umin.min(p[0]);
        umax = umax.max(p[0]);
        vmin = vmin.min(p[1]);
        vmax = vmax.max(p[1]);
    }
    let eps_a = 1e-10 * ((umax - umin) * (vmax - vmin)).max(1e-300);
    let uv_area2 = |t: &[u32; 3]| {
        let a = verts_uv[t[0] as usize];
        let b = verts_uv[t[1] as usize];
        let c = verts_uv[t[2] as usize];
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    };
    tris.retain(|t| uv_area2(t).abs() > eps_a);
    // tess2-rust does not guarantee CCW output: normalize each triangle so
    // that the d/du x d/dv (= surface normal) convention holds downstream.
    for t in &mut tris {
        if uv_area2(t) < 0.0 {
            t.swap(1, 2);
        }
    }
    let mut verts_uv = verts_uv;

    // refine: parametric step limits + 3D chord deviation
    let max_du = surf.u_step(tp.deflection, tp.max_angle);
    let max_dv = surf.v_step(tp.deflection, tp.max_angle);
    let needs_dev = !matches!(surf, Surface::Plane(_));
    if max_du.is_finite() || max_dv.is_finite() || needs_dev {
        refine_uv(
            surf,
            &mut verts_uv,
            &mut tris,
            max_du,
            max_dv,
            tp.deflection,
            if needs_dev { Some((su, sv)) } else { None },
        );
    }

    // map to 3D + normals; fix winding to match the face normal
    let flip = !same_sense;
    let base = mesh.positions.len() as u32 / 3;
    for p in &verts_uv {
        let pos = surf.point(p[0], p[1]);
        let mut n = surf.normal(p[0], p[1]);
        if flip {
            n = n.scale(-1.0);
        }
        mesh.push_vertex(pos, n);
    }
    for t in &tris {
        // tess2 outputs CCW in UV => geometric normal ~ du x dv = surf.normal
        let (a, b, c) = if flip {
            (t[0], t[2], t[1])
        } else {
            (t[0], t[1], t[2])
        };
        mesh.indices.push(base + a);
        mesh.indices.push(base + b);
        mesh.indices.push(base + c);
    }
    true
}

thread_local! {
    /// Set while we're inside tess2 so the global panic hook stays quiet for
    /// expected, recoverable tessellation failures.
    pub static TESS_GUARD: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Install a panic hook that suppresses output for guarded tess2 panics.
pub fn install_panic_guard() {
    let default = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        if !TESS_GUARD.with(|g| g.get()) {
            default(info);
        }
    }));
}

fn run_tess2(
    loops_uv: &[Vec<[f64; 2]>],
    su: f64,
    sv: f64,
) -> Option<(Vec<[f64; 2]>, Vec<[u32; 3]>)> {
    TESS_GUARD.with(|g| g.set(true));
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let mut tess = Tessellator::new();
        for l in loops_uv {
            if l.len() < 3 {
                continue;
            }
            let mut flat: Vec<f64> = Vec::with_capacity(l.len() * 2);
            for p in l {
                flat.push(p[0] * su);
                flat.push(p[1] * sv);
            }
            tess.add_contour(2, &flat);
        }
        if !tess.tessellate(WindingRule::Odd, ElementType::Polygons, 3, 2, None) {
            return None;
        }
        let tv = tess.vertices();
        let te = tess.elements();
        if te.is_empty() {
            return None;
        }
        let verts: Vec<[f64; 2]> = (0..tess.vertex_count())
            .map(|i| [tv[i * 2] / su, tv[i * 2 + 1] / sv])
            .collect();
        let mut tris: Vec<[u32; 3]> = Vec::with_capacity(tess.element_count());
        for e in te.chunks(3) {
            if e.len() == 3 && e[0] != TESS_UNDEF && e[1] != TESS_UNDEF && e[2] != TESS_UNDEF {
                tris.push([e[0], e[1], e[2]]);
            }
        }
        Some((verts, tris))
    }));
    TESS_GUARD.with(|g| g.set(false));
    result.ok().flatten()
}

/// Faces that wrap fully around a periodic surface. Each winding loop is cut
/// at a common seam parameter `c`, normalized to run u = c .. c+period, then
/// consecutive curves (sorted by v) pair into closed band polygons. An odd
/// curve at a capped extreme (sphere pole / cone apex) is closed against a
/// synthetic polar cap line. Non-winding loops become holes in their band.
fn tessellate_periodic_band(
    surf: &Surface,
    loops_uv: &[LoopUv],
    tp: &TessParams,
    same_sense: bool,
    mesh: &mut TriMesh,
) -> bool {
    let per = match surf.u_period() {
        Some(p) => p,
        None => return false,
    };

    struct BandCurve {
        pts: Vec<[f64; 2]>,
        interior_above: bool,
    }
    let mut curves: Vec<BandCurve> = Vec::new();
    let mut holes: Vec<Vec<[f64; 2]>> = Vec::new();
    let mut seam: Option<f64> = None;

    for l in loops_uv {
        if l.w == 0 {
            holes.push(l.uv.clone());
            continue;
        }
        if l.w.abs() != 1 || l.uv.len() < 3 {
            return false; // multi-winding loops: out of scope
        }
        let mut ext = l.uv.clone();
        ext.push([l.uv[0][0] + l.w as f64 * per, l.uv[0][1]]);

        let c = *seam.get_or_insert(ext[0][0]);
        // find a crossing of u ≡ c (mod period) along the extended polyline
        let mut cut: Option<(usize, [f64; 2])> = None;
        'outer: for i in 0..ext.len() - 1 {
            let (u0, u1) = (ext[i][0], ext[i + 1][0]);
            let k0 = ((u0.min(u1) - c) / per).floor() as i64;
            for k in k0..=k0 + 2 {
                let cv = c + k as f64 * per;
                if (u0 - cv).abs() < 1e-12 {
                    cut = Some((i, ext[i]));
                    break 'outer;
                }
                if (u0 - cv) * (u1 - cv) <= 0.0 && (u1 - u0).abs() > 1e-12 {
                    let t = (cv - u0) / (u1 - u0);
                    if (0.0..=1.0).contains(&t) {
                        let v = ext[i][1] + t * (ext[i + 1][1] - ext[i][1]);
                        cut = Some((i, [cv, v]));
                        break 'outer;
                    }
                }
            }
        }
        let (ci, cp) = match cut {
            Some(c) => c,
            None => return false,
        };
        let n = l.uv.len();
        let mut open: Vec<[f64; 2]> = Vec::with_capacity(n + 2);
        open.push(cp);
        for j in 1..=n {
            let idx = (ci + j) % n;
            let wraps = ((ci + j) / n) as f64;
            let p = l.uv[idx];
            open.push([p[0] + wraps * l.w as f64 * per, p[1]]);
        }
        open.push([cp[0] + l.w as f64 * per, cp[1]]);
        open.dedup_by(|a, b| (a[0] - b[0]).abs() < 1e-12 && (a[1] - b[1]).abs() < 1e-12);

        // normalize to +u and start exactly at u = c
        if l.w < 0 {
            open.reverse();
        }
        let shift = ((open[0][0] - c) / per).round() * per;
        if shift != 0.0 {
            for p in &mut open {
                p[0] -= shift;
            }
        }
        curves.push(BandCurve {
            pts: open,
            interior_above: l.interior_above,
        });
    }

    if curves.is_empty() {
        return false;
    }
    let mean_v = |c: &BandCurve| c.pts.iter().map(|p| p[1]).sum::<f64>() / c.pts.len() as f64;
    curves.sort_by(|a, b| {
        mean_v(a)
            .partial_cmp(&mean_v(b))
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // odd number of winding curves: close against a polar cap if available
    if curves.len() % 2 == 1 {
        let caps = match surf.v_caps() {
            Some(c) => c,
            None => return false,
        };
        let c = seam.unwrap_or(0.0);
        let top_wants_cap = curves.last().map(|t| t.interior_above).unwrap_or(false);
        let bottom_wants_cap = curves.first().map(|b| !b.interior_above).unwrap_or(false);
        // sample the cap line at the u step so the polar fan triangulates
        // cleanly instead of as long slivers
        let cap_line = |vcap: f64| {
            let n =
                ((per / surf.u_step(tp.deflection, tp.max_angle)).ceil() as usize).clamp(2, 256);
            (0..=n)
                .map(|i| [c + per * i as f64 / n as f64, vcap])
                .collect::<Vec<[f64; 2]>>()
        };
        if top_wants_cap && caps.1.is_finite() {
            curves.push(BandCurve {
                pts: cap_line(caps.1),
                interior_above: false,
            });
            curves.sort_by(|a, b| {
                mean_v(a)
                    .partial_cmp(&mean_v(b))
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
        } else if bottom_wants_cap && caps.0.is_finite() {
            curves.insert(
                0,
                BandCurve {
                    pts: cap_line(caps.0),
                    interior_above: true,
                },
            );
        } else {
            return false;
        }
    }

    let mut any = false;
    let mut i = 0;
    while i + 1 < curves.len() {
        let bottom = &curves[i];
        let top = &curves[i + 1];
        let (vb, vt) = (mean_v(bottom), mean_v(top));
        let mut poly: Vec<[f64; 2]> = Vec::with_capacity(bottom.pts.len() + top.pts.len());
        poly.extend_from_slice(&bottom.pts);
        poly.extend(top.pts.iter().rev());
        let mut contours: Vec<Vec<[f64; 2]>> = vec![poly];
        for h in &holes {
            let hv = h.iter().map(|p| p[1]).sum::<f64>() / h.len().max(1) as f64;
            if hv >= vb.min(vt) && hv <= vb.max(vt) {
                let hu = h.iter().map(|p| p[0]).sum::<f64>() / h.len().max(1) as f64;
                let c = bottom.pts[0][0];
                let shift = ((hu - (c + per / 2.0)) / per).round() * per;
                let mut hh = h.clone();
                if shift != 0.0 {
                    for p in &mut hh {
                        p[0] -= shift;
                    }
                }
                contours.push(hh);
            }
        }
        any |= emit_uv_region(surf, &contours, tp, same_sense, mesh);
        i += 2;
    }
    any
}

/// Full-domain tessellation for an unbounded closed quadric face.
fn tessellate_unbounded(
    surf: &Surface,
    tp: &TessParams,
    same_sense: bool,
    mesh: &mut TriMesh,
) -> bool {
    use std::f64::consts::{PI, TAU};
    let (u0, u1, v0, v1) = match surf {
        Surface::Sphere(_, _) => (0.0, TAU, -PI / 2.0, PI / 2.0),
        Surface::Torus(_, _, _) => (0.0, TAU, 0.0, TAU),
        _ => return false,
    };
    let du = surf.u_step(tp.deflection, tp.max_angle);
    let dv = surf.v_step(tp.deflection, tp.max_angle).min(du);
    let nu = (((u1 - u0) / du).ceil() as usize).max(4);
    let nv = (((v1 - v0) / dv).ceil() as usize).max(3);
    let base = mesh.positions.len() as u32 / 3;
    for j in 0..=nv {
        let v = v0 + (v1 - v0) * j as f64 / nv as f64;
        for i in 0..=nu {
            let u = u0 + (u1 - u0) * i as f64 / nu as f64;
            let mut n = surf.normal(u, v);
            if !same_sense {
                n = n.scale(-1.0);
            }
            mesh.push_vertex(surf.point(u, v), n);
        }
    }
    let w = (nu + 1) as u32;
    for j in 0..nv as u32 {
        for i in 0..nu as u32 {
            let a = base + j * w + i;
            let b = a + 1;
            let c = a + w;
            let d = c + 1;
            if same_sense {
                mesh.indices.extend_from_slice(&[a, b, d, a, d, c]);
            } else {
                mesh.indices.extend_from_slice(&[a, d, b, a, c, d]);
            }
        }
    }
    true
}

/// Midpoint edge subdivision in UV until every edge respects the parametric
/// step limits *and* a 3D chord-deviation bound (midpoint of the chord vs the
/// surface point at the UV midpoint). Triangles whose 3D normal disagrees
/// with the surface normal ("folds", typically slivers on curved surfaces)
/// get their longest edge split; edge marks are collected globally first so
/// neighbours split consistently and no T-junction cracks appear.
fn refine_uv(
    surf: &Surface,
    verts: &mut Vec<[f64; 2]>,
    tris: &mut Vec<[u32; 3]>,
    max_du: f64,
    max_dv: f64,
    defl: f64,
    flip_metric: Option<(f64, f64)>,
) {
    let mut pts3: Vec<V3> = verts.iter().map(|p| surf.point(p[0], p[1])).collect();
    let check_dev = !matches!(surf, Surface::Plane(_));

    // Deflection budget: meeting a sag bound of `defl` needs at most about
    // area / defl² triangles — features below the deflection scale are not
    // representable anyway. Near-cusp parameterizations (arc length
    // concentrated in tiny UV ranges, e.g. swept tubes with path kinks)
    // otherwise re-fail the sag test at every subdivision level and explode
    // toward the hard cap; the budget stops them at a sane density without
    // affecting well-parameterized faces.
    let approx_area: f64 = tris
        .iter()
        .map(|t| {
            let (a, b, c) = (
                pts3[t[0] as usize],
                pts3[t[1] as usize],
                pts3[t[2] as usize],
            );
            b.sub(a).cross(c.sub(a)).len() * 0.5
        })
        .sum();
    let budget = if defl > 0.0 {
        ((4.0 * approx_area / (defl * defl)) as usize)
            .max(tris.len() * 4)
            .clamp(2_048, 300_000)
    } else {
        300_000
    };

    for _pass in 0..12 {
        let mut mid: HashMap<(u32, u32), u32> = HashMap::new();
        let mut out: Vec<[u32; 3]> = Vec::with_capacity(tris.len());
        let mut split_any = false;

        // 0 = no, 1 = parametric, 2 = deviation
        let edge_needs_split = |i: u32, j: u32, verts: &Vec<[f64; 2]>, pts3: &Vec<V3>| -> u8 {
            let a = verts[i as usize];
            let b = verts[j as usize];
            if (b[0] - a[0]).abs() > max_du || (b[1] - a[1]).abs() > max_dv {
                return 1;
            }
            if !check_dev {
                return 0;
            }
            // sag test: perpendicular distance from the surface point at the
            // UV midpoint to the 3D chord line. Comparing against the chord
            // *midpoint* instead would also measure tangential drift from
            // non-uniform parameterization speed (ubiquitous on B-splines)
            // and over-refine without bound.
            let pa = pts3[i as usize];
            let chord = pts3[j as usize].sub(pa);
            let l2 = chord.dot(chord);
            // sub-resolution floor: a chord this short with sag > defl means
            // a curvature radius below ~2*defl — a feature the deflection
            // budget cannot represent; refining further only explodes the
            // mesh (cusps on spring/fillet B-splines do exactly that)
            if l2 < (4.0 * defl) * (4.0 * defl) {
                return 0;
            }
            let m = surf.point((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5);
            let d = m.sub(pa);
            let dev = if l2 < 1e-300 {
                d.len()
            } else {
                let t = (d.dot(chord) / l2).clamp(0.0, 1.0);
                d.sub(chord.scale(t)).len()
            };
            if dev > defl {
                2
            } else {
                0
            }
        };

        // phase 1: mark edges — per-edge criteria plus the longest edge of
        // any folded triangle (marks are global, so adjacency stays crack
        // free in phase 2)
        let mut marked: std::collections::HashSet<(u32, u32)> = std::collections::HashSet::new();
        let key = |i: u32, j: u32| (i.min(j), i.max(j));
        for t in tris.iter() {
            for (i, j) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                if !marked.contains(&key(i, j)) {
                    match edge_needs_split(i, j, verts, &pts3) {
                        0 => {}
                        _ => {
                            marked.insert(key(i, j));
                        }
                    }
                }
            }
            // Fold detection breaks up the big folded slivers that tess2
            // can emit, in the first passes only: persistently folded micro
            // triangles indicate parameterization cusps (collapsed control
            // points on fillet B-splines, degenerate tori) that subdivision
            // cannot fix — chasing them explodes the mesh instead.
            if check_dev && _pass < 3 {
                let (a, b, c) = (
                    pts3[t[0] as usize],
                    pts3[t[1] as usize],
                    pts3[t[2] as usize],
                );
                let gn = b.sub(a).cross(c.sub(a));
                let lmax = b.sub(a).len().max(c.sub(b).len()).max(a.sub(c).len());
                // Skip (near-)degenerate triangles: their geometric normal is
                // numeric noise, they contribute no area, and chasing them
                // explodes the subdivision. optimize() drops them later.
                if gn.len() > 1e-4 * lmax * lmax {
                    let (ua, ub, uc) = (
                        verts[t[0] as usize],
                        verts[t[1] as usize],
                        verts[t[2] as usize],
                    );
                    let cen = [(ua[0] + ub[0] + uc[0]) / 3.0, (ua[1] + ub[1] + uc[1]) / 3.0];
                    let sn = surf.normal(cen[0], cen[1]);
                    if gn.norm().dot(sn) < 0.5 {
                        // folded or badly warped: split the longest 3D edge
                        let l01 = b.sub(a).len();
                        let l12 = c.sub(b).len();
                        let l20 = a.sub(c).len();
                        let e = if l01 >= l12 && l01 >= l20 {
                            (t[0], t[1])
                        } else if l12 >= l20 {
                            (t[1], t[2])
                        } else {
                            (t[2], t[0])
                        };
                        let (ea, eb) = (verts[e.0 as usize], verts[e.1 as usize]);
                        let big_enough = (eb[0] - ea[0]).abs() > max_du / 8.0
                            || (eb[1] - ea[1]).abs() > max_dv / 8.0;
                        if big_enough {
                            marked.insert(key(e.0, e.1));
                        }
                    }
                }
            }
        }

        let mut midpoint = |i: u32, j: u32, verts: &mut Vec<[f64; 2]>, pts3: &mut Vec<V3>| -> u32 {
            let key = (i.min(j), i.max(j));
            *mid.entry(key).or_insert_with(|| {
                let a = verts[i as usize];
                let b = verts[j as usize];
                let m = [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5];
                verts.push(m);
                pts3.push(surf.point(m[0], m[1]));
                (verts.len() - 1) as u32
            })
        };

        // phase 2: subdivide strictly by the marked-edge set
        for t in tris.iter() {
            let s = [
                marked.contains(&key(t[0], t[1])),
                marked.contains(&key(t[1], t[2])),
                marked.contains(&key(t[2], t[0])),
            ];
            if !s[0] && !s[1] && !s[2] {
                out.push(*t);
                continue;
            }
            split_any = true;
            let m01 = if s[0] {
                Some(midpoint(t[0], t[1], verts, &mut pts3))
            } else {
                None
            };
            let m12 = if s[1] {
                Some(midpoint(t[1], t[2], verts, &mut pts3))
            } else {
                None
            };
            let m20 = if s[2] {
                Some(midpoint(t[2], t[0], verts, &mut pts3))
            } else {
                None
            };
            match (m01, m12, m20) {
                (Some(a), Some(b), Some(c)) => {
                    out.push([t[0], a, c]);
                    out.push([a, t[1], b]);
                    out.push([c, b, t[2]]);
                    out.push([a, b, c]);
                }
                (Some(a), Some(b), None) => {
                    out.push([t[0], a, t[2]]);
                    out.push([a, b, t[2]]);
                    out.push([a, t[1], b]);
                }
                (None, Some(b), Some(c)) => {
                    out.push([t[0], t[1], b]);
                    out.push([t[0], b, c]);
                    out.push([c, b, t[2]]);
                }
                (Some(a), None, Some(c)) => {
                    out.push([t[0], a, c]);
                    out.push([a, t[1], c]);
                    out.push([c, t[1], t[2]]);
                }
                (Some(a), None, None) => {
                    out.push([t[0], a, t[2]]);
                    out.push([a, t[1], t[2]]);
                }
                (None, Some(b), None) => {
                    out.push([t[0], t[1], b]);
                    out.push([t[0], b, t[2]]);
                }
                (None, None, Some(c)) => {
                    out.push([t[0], t[1], c]);
                    out.push([c, t[1], t[2]]);
                }
                (None, None, None) => unreachable!(),
            }
        }
        *tris = out;
        // reshape slivers right away so they don't propagate through the
        // next subdivision (Delaunay flips in metric UV)
        if let Some((su, sv)) = flip_metric {
            delaunay_flip(verts, tris, su, sv);
        }
        if !split_any || tris.len() > budget {
            break;
        }
    }
}

// ------------------------------------------------- AP242 tessellated geometry

fn tessellate_triangulated_set(sf: &StepFile, id: u32, mesh: &mut TriMesh) {
    let p = match sf.params(id) {
        Some(p) => p,
        None => return,
    };
    let coords = match p
        .get(1)
        .and_then(|v| v.as_ref_id())
        .and_then(|r| sf.params(r))
    {
        Some(c) => c,
        None => return,
    };
    let pts: Vec<V3> = match coords.get(2).and_then(|v| v.as_list()) {
        Some(l) => l
            .iter()
            .filter_map(|t| t.as_list())
            .filter_map(|t| {
                Some(v3(
                    t.first()?.as_f64()?,
                    t.get(1)?.as_f64()?,
                    t.get(2)?.as_f64()?,
                ))
            })
            .collect(),
        None => return,
    };
    let pnindex: Vec<u32> = p
        .get(4)
        .and_then(|v| v.as_list())
        .map(|l| {
            l.iter()
                .filter_map(|v| v.as_i64())
                .map(|v| v as u32)
                .collect()
        })
        .unwrap_or_default();
    let tris = match p.get(5).and_then(|v| v.as_list()) {
        Some(t) => t,
        None => return,
    };

    let map_idx = |i: u32| -> Option<u32> {
        let i = i.checked_sub(1)?; // STEP is 1-based
        let pi = if pnindex.is_empty() {
            i
        } else {
            pnindex.get(i as usize)?.checked_sub(1)?
        };
        if (pi as usize) < pts.len() {
            Some(pi)
        } else {
            None
        }
    };

    let base = mesh.positions.len() as u32 / 3;
    for pnt in &pts {
        mesh.push_vertex(*pnt, V3::ZERO);
    }
    let istart = mesh.indices.len();
    for t in tris {
        if let Some(t) = t.as_list() {
            if t.len() >= 3 {
                let idx: Option<Vec<u32>> = t
                    .iter()
                    .take(3)
                    .map(|v| v.as_i64().map(|v| v as u32).and_then(map_idx))
                    .collect();
                if let Some(idx) = idx {
                    mesh.indices.push(base + idx[0]);
                    mesh.indices.push(base + idx[1]);
                    mesh.indices.push(base + idx[2]);
                }
            }
        }
    }
    mesh.compute_missing_normals(base as usize, istart);
}

/// Lawson edge flips in (metric-scaled) UV. Interior edges are flipped while
/// the flip increases the smaller of the two triangles' minimum angles;
/// boundary edges (single adjacent triangle) are left alone.
fn delaunay_flip(verts: &[[f64; 2]], tris: &mut Vec<[u32; 3]>, su: f64, sv: f64) {
    let p = |i: u32| [verts[i as usize][0] * su, verts[i as usize][1] * sv];
    let min_angle = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| -> f64 {
        let l = |x: [f64; 2], y: [f64; 2]| ((x[0] - y[0]).powi(2) + (x[1] - y[1]).powi(2)).sqrt();
        let (ab, bc, ca) = (l(a, b), l(b, c), l(c, a));
        if ab < 1e-300 || bc < 1e-300 || ca < 1e-300 {
            return 0.0;
        }
        let ang = |opp: f64, s1: f64, s2: f64| {
            ((s1 * s1 + s2 * s2 - opp * opp) / (2.0 * s1 * s2))
                .clamp(-1.0, 1.0)
                .acos()
        };
        ang(bc, ab, ca).min(ang(ca, ab, bc)).min(ang(ab, bc, ca))
    };
    let area2 = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    };

    for _sweep in 0..6 {
        // interior edge -> (tri index, opposite vertex)
        let mut edge_map: HashMap<(u32, u32), Vec<(usize, u32)>> = HashMap::new();
        for (ti, t) in tris.iter().enumerate() {
            for k in 0..3 {
                let (i, j, o) = (t[k], t[(k + 1) % 3], t[(k + 2) % 3]);
                edge_map
                    .entry((i.min(j), i.max(j)))
                    .or_default()
                    .push((ti, o));
            }
        }
        let mut dirty: Vec<bool> = vec![false; tris.len()];
        let mut flips = 0usize;
        // HashMap order is randomized per process; flip in sorted edge order
        // so the triangulation (and the GLB bytes) are reproducible
        let mut edges: Vec<_> = edge_map.iter().collect();
        edges.sort_unstable_by_key(|(k, _)| **k);
        for ((i, j), adj) in edges {
            if adj.len() != 2 {
                continue;
            }
            let ((t1, o1), (t2, o2)) = (adj[0], adj[1]);
            if dirty[t1] || dirty[t2] || o1 == o2 {
                continue;
            }
            let (pi, pj, po1, po2) = (p(*i), p(*j), p(o1), p(o2));
            // candidate triangles after the flip must keep CCW orientation
            if area2(po1, po2, pj) <= 1e-18 || area2(po2, po1, pi) <= 1e-18 {
                continue;
            }
            let before = min_angle(pi, pj, po1).min(min_angle(pj, pi, po2));
            let after = min_angle(po1, po2, pj).min(min_angle(po2, po1, pi));
            if after > before + 1e-12 {
                tris[t1] = [o1, o2, *j];
                tris[t2] = [o2, o1, *i];
                dirty[t1] = true;
                dirty[t2] = true;
                flips += 1;
            }
        }
        if flips == 0 {
            break;
        }
    }
}
