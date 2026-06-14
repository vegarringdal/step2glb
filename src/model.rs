//! Typed accessors over the lazy STEP index: points, directions, placements,
//! surface construction and edge-curve discretization.

use std::collections::HashMap;

use crate::geom::*;
use crate::step::{StepFile, P, TYPE_COMPLEX};

/// SI length-unit prefix -> metres. (`SI_UNIT(prefix, .METRE.)`.)
fn si_prefix_scale(prefix: Option<&str>) -> f64 {
    match prefix {
        Some("MILLI") => 0.001,
        Some("CENTI") => 0.01,
        Some("DECI") => 0.1,
        Some("DECA") => 10.0,
        Some("HECTO") => 100.0,
        Some("KILO") => 1000.0,
        Some("MICRO") => 1e-6,
        Some("NANO") => 1e-9,
        _ => 1.0,
    }
}

/// Length scale (to metres) of one unit entity, if it is a length unit: a
/// metre `SI_UNIT(prefix,.METRE.)` (plain or inside a complex `LENGTH_UNIT`) or
/// an inch/foot `CONVERSION_BASED_UNIT`. Returns `None` for angle/other units.
fn length_unit_scale(sf: &StepFile, unit: u32) -> Option<f64> {
    // an SI metre unit is a length unit whether or not it is tagged LENGTH_UNIT
    let si = sf
        .complex_leaf(unit, "SI_UNIT")
        .or_else(|| sf.params(unit).filter(|_| sf.entity_type(unit) == Some("SI_UNIT")));
    if let Some(si) = si {
        let mut prefix = None;
        let mut metre = false;
        for v in &si {
            if let P::Enum(e) = v {
                if e == "METRE" {
                    metre = true;
                } else {
                    prefix = Some(e.clone());
                }
            }
        }
        if metre {
            return Some(si_prefix_scale(prefix.as_deref()));
        }
    }
    // CONVERSION_BASED_UNIT('INCH'|'FOOT', ...) — only when tagged a length unit
    if sf.complex_leaf(unit, "LENGTH_UNIT").is_some() {
        if let Some(cbu) = sf.complex_leaf(unit, "CONVERSION_BASED_UNIT") {
            return match cbu
                .iter()
                .find_map(|v| v.as_str())?
                .trim_matches('"')
                .to_ascii_uppercase()
                .as_str()
            {
                "INCH" => Some(0.0254),
                "FOOT" => Some(0.3048),
                _ => None,
            };
        }
    }
    None
}

/// The file's global length-unit scale to metres — the first length unit among
/// the unit entities (plain SI_UNITs, then complex units), matching the scan a
/// single-context file would use. `None` if no length unit is declared. Shared
/// by the GLB unit scaling and the per-instance transform unit handling so they
/// stay consistent.
pub fn file_length_scale(sf: &StepFile) -> Option<f64> {
    for &id in sf.of_type("SI_UNIT") {
        if let Some(s) = length_unit_scale(sf, id) {
            return Some(s);
        }
    }
    let cty = sf.type_id(TYPE_COMPLEX)?;
    sf.by_type
        .get(&cty)?
        .iter()
        .find_map(|&id| length_unit_scale(sf, id))
}

/// Length-unit scale (to metres) of a SHAPE_REPRESENTATION's own context, if it
/// carries a `GLOBAL_UNIT_ASSIGNED_CONTEXT` with a length unit. Autodesk mixes
/// mm and metre contexts in one file, so geometry must be scaled per
/// representation rather than by a single global unit.
pub fn representation_length_scale(sf: &StepFile, rep: u32) -> Option<f64> {
    // SHAPE_REPRESENTATION('name', (items), context)
    let ctx = sf.params(rep)?.get(2).and_then(|v| v.as_ref_id())?;
    let assigned = sf.complex_leaf(ctx, "GLOBAL_UNIT_ASSIGNED_CONTEXT")?;
    assigned
        .iter()
        .filter_map(|v| v.as_list())
        .flat_map(|l| l.iter().filter_map(|v| v.as_ref_id()))
        .find_map(|u| length_unit_scale(sf, u))
}

/// Factor to bring a representation's geometry into the file's global unit, so a
/// metre-context part in an otherwise-mm file is sized consistently before the
/// global unit scaling. 1.0 when the representation has no own unit or already
/// matches the global one.
pub fn rep_unit_factor(sf: &StepFile, rep: u32, global_scale: f64) -> f64 {
    match representation_length_scale(sf, rep) {
        Some(s) if global_scale.abs() > 1e-300 => s / global_scale,
        _ => 1.0,
    }
}

pub fn cartesian_point(sf: &StepFile, id: u32) -> Option<V3> {
    // CARTESIAN_POINT('', (x, y, z))
    let p = sf.params(id)?;
    let l = p.get(1)?.as_list()?;
    Some(v3(
        l.first()?.as_f64()?,
        l.get(1).and_then(|v| v.as_f64()).unwrap_or(0.0),
        l.get(2).and_then(|v| v.as_f64()).unwrap_or(0.0),
    ))
}

pub fn direction(sf: &StepFile, id: u32) -> Option<V3> {
    let p = sf.params(id)?;
    let l = p.get(1)?.as_list()?;
    Some(
        v3(
            l.first()?.as_f64()?,
            l.get(1).and_then(|v| v.as_f64()).unwrap_or(0.0),
            l.get(2).and_then(|v| v.as_f64()).unwrap_or(0.0),
        )
        .norm(),
    )
}

/// AXIS2_PLACEMENT_3D('', location, axis?, ref_direction?)
pub fn axis2_placement(sf: &StepFile, id: u32) -> Option<Frame> {
    let p = sf.params(id)?;
    let o = p
        .get(1)
        .and_then(|v| v.as_ref_id())
        .and_then(|r| cartesian_point(sf, r))
        .unwrap_or(V3::ZERO);
    let axis = p
        .get(2)
        .and_then(|v| v.as_ref_id())
        .and_then(|r| direction(sf, r));
    let refd = p
        .get(3)
        .and_then(|v| v.as_ref_id())
        .and_then(|r| direction(sf, r));
    Some(Frame::new(o, axis, refd))
}

pub fn axis2_matrix(sf: &StepFile, id: u32) -> M4 {
    axis2_placement(sf, id)
        .map(|f| f.to_m4())
        .unwrap_or(M4::IDENTITY)
}

/// Build an evaluatable surface from a STEP surface entity. Swept surfaces
/// with line/circle generatrices are reduced to the equivalent analytic
/// surface (closed-form UV inversion); everything else evaluates directly.
pub fn surface(sf: &StepFile, id: u32) -> Option<Surface> {
    let ty = sf.entity_type(id)?;
    if ty == crate::step::TYPE_COMPLEX {
        // rational B-spline surface in complex-instance form
        return bspline_surface_complex(sf, id).map(Surface::BSpline);
    }
    let p = sf.params(id)?;
    let frame = |i: usize| {
        p.get(i)
            .and_then(|v| v.as_ref_id())
            .and_then(|r| axis2_placement(sf, r))
    };
    match ty {
        "PLANE" => Some(Surface::Plane(frame(1)?)),
        "CYLINDRICAL_SURFACE" => Some(Surface::Cylinder(frame(1)?, p.get(2)?.as_f64()?)),
        "CONICAL_SURFACE" => Some(Surface::Cone(
            frame(1)?,
            p.get(2)?.as_f64()?,
            p.get(3)?.as_f64()?,
        )),
        "SPHERICAL_SURFACE" => Some(Surface::Sphere(frame(1)?, p.get(2)?.as_f64()?)),
        "TOROIDAL_SURFACE" | "DEGENERATE_TOROIDAL_SURFACE" => Some(Surface::Torus(
            frame(1)?,
            p.get(2)?.as_f64()?,
            p.get(3)?.as_f64()?,
        )),
        "SURFACE_OF_LINEAR_EXTRUSION" => {
            // ('', swept_curve, extrusion_axis VECTOR)
            let curve = curve3(sf, p.get(1)?.as_ref_id()?)?;
            let dir = vector(sf, p.get(2)?.as_ref_id()?)?;
            Some(reduce_extrusion(curve, dir))
        }
        "SURFACE_OF_REVOLUTION" => {
            // ('', swept_curve, axis AXIS1_PLACEMENT)
            let curve = curve3(sf, p.get(1)?.as_ref_id()?)?;
            let axis = axis1_placement(sf, p.get(2)?.as_ref_id()?)?;
            Some(reduce_revolution(curve, axis))
        }
        "B_SPLINE_SURFACE_WITH_KNOTS" => bspline_surface_simple(sf, &p).map(Surface::BSpline),
        _ => None,
    }
}

/// VECTOR('', direction, magnitude) -> scaled direction
pub fn vector(sf: &StepFile, id: u32) -> Option<V3> {
    let p = sf.params(id)?;
    let d = direction(sf, p.get(1)?.as_ref_id()?)?;
    let m = p.get(2).and_then(|v| v.as_f64()).unwrap_or(1.0);
    Some(d.scale(m))
}

/// AXIS1_PLACEMENT('', location, axis)
pub fn axis1_placement(sf: &StepFile, id: u32) -> Option<Frame> {
    let p = sf.params(id)?;
    let o = p
        .get(1)
        .and_then(|v| v.as_ref_id())
        .and_then(|r| cartesian_point(sf, r))
        .unwrap_or(V3::ZERO);
    let axis = p
        .get(2)
        .and_then(|v| v.as_ref_id())
        .and_then(|r| direction(sf, r));
    Some(Frame::new(o, axis, None))
}

// ------------------------------------------------------- evaluatable curves

/// Build an evaluatable `Curve3` from a STEP curve entity.
pub fn curve3(sf: &StepFile, id: u32) -> Option<Curve3> {
    let ty = sf.entity_type(id)?;
    match ty {
        "LINE" => {
            let p = sf.params(id)?;
            Some(Curve3::Line {
                p: cartesian_point(sf, p.get(1)?.as_ref_id()?)?,
                d: vector(sf, p.get(2)?.as_ref_id()?)?,
            })
        }
        "CIRCLE" => {
            let p = sf.params(id)?;
            Some(Curve3::Circle {
                f: axis2_placement(sf, p.get(1)?.as_ref_id()?)?,
                r: p.get(2)?.as_f64()?,
            })
        }
        "ELLIPSE" => {
            let p = sf.params(id)?;
            Some(Curve3::Ellipse {
                f: axis2_placement(sf, p.get(1)?.as_ref_id()?)?,
                a: p.get(2)?.as_f64()?,
                b: p.get(3)?.as_f64()?,
            })
        }
        "POLYLINE" => {
            let p = sf.params(id)?;
            let pts: Vec<V3> = p
                .get(1)?
                .as_list()?
                .iter()
                .filter_map(|v| v.as_ref_id())
                .filter_map(|r| cartesian_point(sf, r))
                .collect();
            if pts.len() >= 2 {
                Some(Curve3::Polyline(pts))
            } else {
                None
            }
        }
        "B_SPLINE_CURVE_WITH_KNOTS" => {
            let p = sf.params(id)?;
            bspline_curve3(sf, &p, None)
        }
        "TRIMMED_CURVE" | "SURFACE_CURVE" | "SEAM_CURVE" => {
            let p = sf.params(id)?;
            curve3(sf, p.get(1)?.as_ref_id()?)
        }
        crate::step::TYPE_COMPLEX => {
            let core = sf.complex_leaf(id, "B_SPLINE_CURVE_WITH_KNOTS")?;
            let base = sf.complex_leaf(id, "B_SPLINE_CURVE")?;
            let weights = sf
                .complex_leaf(id, "RATIONAL_B_SPLINE_CURVE")
                .and_then(|w| w.first().and_then(|l| l.as_list().map(|l| l.to_vec())))
                .map(|l| l.iter().filter_map(|v| v.as_f64()).collect::<Vec<f64>>());
            // merge fields of both leaves and parse
            let mut merged = base;
            merged.extend(core);
            bspline_curve3(sf, &merged, weights)
        }
        _ => None,
    }
}

/// Assemble a `Curve3::BSpline` from B_SPLINE_CURVE(_WITH_KNOTS) params
/// (field positions detected dynamically, as in `bspline_polyline_core`).
fn bspline_curve3(sf: &StepFile, params: &[P], weights: Option<Vec<f64>>) -> Option<Curve3> {
    let mut degree: Option<usize> = None;
    let mut cps: Option<Vec<V3>> = None;
    let mut num_lists: Vec<Vec<f64>> = Vec::new();
    for p in params {
        match p {
            P::I(v) if degree.is_none() => degree = Some((*v).max(1) as usize),
            P::L(l) => {
                if !l.is_empty() && l.iter().all(|e| matches!(e, P::Ref(_))) {
                    if cps.is_none() {
                        cps = Some(
                            l.iter()
                                .filter_map(|e| e.as_ref_id())
                                .filter_map(|r| cartesian_point(sf, r))
                                .collect(),
                        );
                    }
                } else if !l.is_empty() && l.iter().all(|e| matches!(e, P::I(_) | P::F(_))) {
                    num_lists.push(l.iter().filter_map(|e| e.as_f64()).collect());
                }
            }
            _ => {}
        }
    }
    let degree = degree?;
    let cps = cps?;
    if cps.len() < 2 || num_lists.len() < 2 {
        return None;
    }
    let mults = &num_lists[num_lists.len() - 2];
    let knots_u = &num_lists[num_lists.len() - 1];
    let mut knots = Vec::new();
    for (m, k) in mults.iter().zip(knots_u.iter()) {
        for _ in 0..(*m as usize) {
            knots.push(*k);
        }
    }
    if knots.len() != cps.len() + degree + 1 {
        return None;
    }
    if let Some(w) = &weights {
        if w.len() != cps.len() {
            return None;
        }
    }
    Some(Curve3::BSpline {
        degree,
        knots,
        cps,
        weights,
    })
}

// --------------------------------------------------------- swept reductions

/// Extrusion of a line is a plane; extrusion of a circle along its own axis
/// is a cylinder. Everything else stays a general extrusion surface.
pub fn reduce_extrusion(curve: Curve3, dir: V3) -> Surface {
    match &curve {
        Curve3::Line { p, d } => {
            let n = d.cross(dir);
            if n.len() > 1e-12 {
                return Surface::Plane(Frame::new(*p, Some(n.norm()), Some(d.norm())));
            }
            Surface::Extrusion { curve, dir }
        }
        Curve3::Circle { f, r } => {
            if f.z.cross(dir.norm()).len() < 1e-9 {
                let sign = if f.z.dot(dir) >= 0.0 { 1.0 } else { -1.0 };
                let frame = Frame::new(f.o, Some(f.z.scale(sign)), Some(f.x));
                return Surface::Cylinder(frame, *r);
            }
            Surface::Extrusion { curve, dir }
        }
        _ => Surface::Extrusion { curve, dir },
    }
}

/// Revolved lines coplanar with the axis reduce to cylinders/cones/planes;
/// revolved circles coplanar with the axis reduce to spheres/tori.
pub fn reduce_revolution(curve: Curve3, axis: Frame) -> Surface {
    let in_axis = |p: V3| {
        let d = p.sub(axis.o);
        (
            d.sub(axis.z.scale(d.dot(axis.z))), // radial vector
            d.dot(axis.z),                      // height
        )
    };
    match &curve {
        Curve3::Line { p, d } => {
            let (rad, _) = in_axis(*p);
            let dn = d.norm();
            // coplanar with the axis: direction lies in span(z, radial dir)
            let radial_dir = if rad.len() > 1e-12 {
                rad.norm()
            } else {
                // line starts on the axis; use the direction's radial part
                let dr = dn.sub(axis.z.scale(dn.dot(axis.z)));
                if dr.len() < 1e-12 {
                    // line *is* the axis -> degenerate
                    return Surface::Revolution { curve, axis };
                }
                dr.norm()
            };
            let out_of_plane = dn
                .sub(axis.z.scale(dn.dot(axis.z)))
                .sub(radial_dir.scale(dn.sub(axis.z.scale(dn.dot(axis.z))).dot(radial_dir)));
            if out_of_plane.len() > 1e-9 {
                return Surface::Revolution { curve, axis };
            }
            let dz = dn.dot(axis.z);
            let drad = dn.dot(radial_dir);
            if drad.abs() < 1e-12 {
                // parallel to axis -> cylinder, radius = radial distance
                let frame = Frame::new(axis.o, Some(axis.z), Some(radial_dir));
                return Surface::Cylinder(frame, rad.len());
            }
            if dz.abs() < 1e-12 {
                // perpendicular to axis -> plane (annulus)
                let (_, h) = in_axis(*p);
                let frame = Frame::new(axis.o.add(axis.z.scale(h)), Some(axis.z), Some(radial_dir));
                return Surface::Plane(frame);
            }
            // slanted -> cone: radius at the axis frame origin's height plane
            let (rad0, h0) = in_axis(*p);
            let slope = drad / dz; // d radius / d height
            let r_at_origin = rad0.len() - h0 * slope;
            let frame = Frame::new(axis.o, Some(axis.z), Some(radial_dir));
            return Surface::Cone(frame, r_at_origin, slope.atan());
        }
        Curve3::Circle { f, r } => {
            // circle plane must contain the axis direction
            if f.z.dot(axis.z).abs() < 1e-9 {
                let (rad, h) = in_axis(f.o);
                if rad.len() < 1e-9 {
                    // centered on the axis -> sphere
                    let frame = Frame::new(axis.o.add(axis.z.scale(h)), Some(axis.z), None);
                    return Surface::Sphere(frame, *r);
                }
                let frame = Frame::new(axis.o.add(axis.z.scale(h)), Some(axis.z), Some(rad.norm()));
                return Surface::Torus(frame, rad.len(), *r);
            }
            Surface::Revolution { curve, axis }
        }
        _ => Surface::Revolution { curve, axis },
    }
}

// ------------------------------------------------------- B-spline surfaces

/// B_SPLINE_SURFACE_WITH_KNOTS('name', u_deg, v_deg, ((cps)), form,
///   u_closed, v_closed, self_isect, (u_mults), (v_mults), (u_knots),
///   (v_knots), spec)
fn bspline_surface_simple(sf: &StepFile, p: &[P]) -> Option<BSplineSurface> {
    let deg_u = p.get(1)?.as_i64()? as usize;
    let deg_v = p.get(2)?.as_i64()? as usize;
    let net = parse_control_net(sf, p.get(3)?)?;
    let closed_u = p.get(5).and_then(|v| v.as_bool()).unwrap_or(false);
    let closed_v = p.get(6).and_then(|v| v.as_bool()).unwrap_or(false);
    let knots_u = expand_knots(p.get(8)?, p.get(10)?)?;
    let knots_v = expand_knots(p.get(9)?, p.get(11)?)?;
    build_bspline_surface(
        deg_u, deg_v, net, None, knots_u, knots_v, closed_u, closed_v,
    )
}

/// Complex-instance (usually rational) form: fields are split across the
/// B_SPLINE_SURFACE, B_SPLINE_SURFACE_WITH_KNOTS and RATIONAL_B_SPLINE_SURFACE
/// leaves.
pub fn bspline_surface_complex(sf: &StepFile, id: u32) -> Option<BSplineSurface> {
    let base = sf.complex_leaf(id, "B_SPLINE_SURFACE")?;
    // B_SPLINE_SURFACE(u_deg, v_deg, ((cps)), form, u_closed, v_closed, si)
    let mut ints: Vec<i64> = Vec::new();
    let mut net: Option<(usize, usize, Vec<V3>)> = None;
    let mut bools: Vec<bool> = Vec::new();
    for v in &base {
        match v {
            P::I(i) => ints.push(*i),
            P::L(_) if net.is_none() => net = parse_control_net(sf, v),
            P::Enum(e) if e == "T" || e == "F" => bools.push(e == "T"),
            _ => {}
        }
    }
    if ints.len() < 2 {
        return None;
    }
    let (deg_u, deg_v) = (ints[0].max(1) as usize, ints[1].max(1) as usize);
    let net = net?;

    let wk = sf.complex_leaf(id, "B_SPLINE_SURFACE_WITH_KNOTS")?;
    // four numeric lists: u_mults, v_mults, u_knots, v_knots
    let lists: Vec<Vec<f64>> = wk
        .iter()
        .filter_map(|v| v.as_list())
        .filter(|l| !l.is_empty() && l.iter().all(|e| matches!(e, P::I(_) | P::F(_))))
        .map(|l| l.iter().filter_map(|e| e.as_f64()).collect())
        .collect();
    if lists.len() < 4 {
        return None;
    }
    let expand = |m: &Vec<f64>, k: &Vec<f64>| -> Vec<f64> {
        let mut out = Vec::new();
        for (mm, kk) in m.iter().zip(k.iter()) {
            for _ in 0..(*mm as usize) {
                out.push(*kk);
            }
        }
        out
    };
    let knots_u = expand(&lists[0], &lists[2]);
    let knots_v = expand(&lists[1], &lists[3]);

    let weights = sf
        .complex_leaf(id, "RATIONAL_B_SPLINE_SURFACE")
        .and_then(|w| {
            let rows: Vec<Vec<f64>> = w
                .iter()
                .filter_map(|v| v.as_list())
                .flat_map(|outer| {
                    outer
                        .iter()
                        .filter_map(|row| row.as_list())
                        .map(|row| row.iter().filter_map(|e| e.as_f64()).collect::<Vec<f64>>())
                        .collect::<Vec<_>>()
                })
                .collect();
            if rows.is_empty() {
                None
            } else {
                Some(rows.into_iter().flatten().collect::<Vec<f64>>())
            }
        });

    let (cu, cv) = (
        bools.first().copied().unwrap_or(false),
        bools.get(1).copied().unwrap_or(false),
    );
    build_bspline_surface(deg_u, deg_v, net, weights, knots_u, knots_v, cu, cv)
}

fn parse_control_net(sf: &StepFile, p: &P) -> Option<(usize, usize, Vec<V3>)> {
    let rows = p.as_list()?;
    let nu = rows.len();
    let mut nv = 0usize;
    let mut cps = Vec::new();
    for row in rows {
        let row = row.as_list()?;
        if nv == 0 {
            nv = row.len();
        } else if nv != row.len() {
            return None;
        }
        for r in row {
            cps.push(cartesian_point(sf, r.as_ref_id()?)?);
        }
    }
    if nu == 0 || nv == 0 {
        None
    } else {
        Some((nu, nv, cps))
    }
}

fn expand_knots(mults: &P, knots: &P) -> Option<Vec<f64>> {
    let m = mults.as_list()?;
    let k = knots.as_list()?;
    let mut out = Vec::new();
    for (mm, kk) in m.iter().zip(k.iter()) {
        let mm = mm.as_i64()? as usize;
        let kk = kk.as_f64()?;
        for _ in 0..mm {
            out.push(kk);
        }
    }
    Some(out)
}

#[allow(clippy::too_many_arguments)]
fn build_bspline_surface(
    deg_u: usize,
    deg_v: usize,
    net: (usize, usize, Vec<V3>),
    weights: Option<Vec<f64>>,
    knots_u: Vec<f64>,
    knots_v: Vec<f64>,
    closed_u: bool,
    closed_v: bool,
) -> Option<BSplineSurface> {
    let (nu, nv, cps) = net;
    if knots_u.len() != nu + deg_u + 1 || knots_v.len() != nv + deg_v + 1 {
        return None;
    }
    if let Some(w) = &weights {
        if w.len() != nu * nv {
            return None;
        }
    }
    Some(
        BSplineSurface {
            deg_u,
            deg_v,
            nu,
            nv,
            cps,
            weights,
            knots_u,
            knots_v,
            closed_u,
            closed_v,
            size: 0.0,
        }
        .finish(),
    )
}

// ------------------------------------------------------------- edge sampling

pub struct TessParams {
    pub deflection: f64,
    pub max_angle: f64, // radians
}

/// Discretize an ORIENTED_EDGE (or EDGE_CURVE) into a 3D polyline that starts
/// at the edge start vertex and ends at the edge end vertex, honouring
/// orientation. The last point is included.
pub fn edge_polyline(
    sf: &StepFile,
    id: u32,
    tp: &TessParams,
    unsup: &mut HashMap<String, usize>,
) -> Option<Vec<V3>> {
    let ty = sf.entity_type(id)?;
    let (edge_id, flip_oriented) = if ty == "ORIENTED_EDGE" {
        // ORIENTED_EDGE('', *, *, edge, orientation)
        let p = sf.params(id)?;
        let edge = p.get(3)?.as_ref_id()?;
        let orient = p.get(4).and_then(|v| v.as_bool()).unwrap_or(true);
        (edge, !orient)
    } else {
        (id, false)
    };

    // EDGE_CURVE('', start_vertex, end_vertex, curve, same_sense)
    let p = sf.params(edge_id)?;
    let sv = vertex_point(sf, p.get(1)?.as_ref_id()?)?;
    let ev = vertex_point(sf, p.get(2)?.as_ref_id()?)?;
    // The 3D curve may be omitted ($): a legal EDGE_CURVE between two known
    // vertices with no separate geometry is a straight segment. Don't let a
    // null (or unresolved) curve drop the whole edge — and hence the loop and
    // the entire face — to None.
    let curve = p.get(3).and_then(|v| v.as_ref_id());
    let same_sense = p.get(4).and_then(|v| v.as_bool()).unwrap_or(true);

    let (a, b) = if same_sense { (sv, ev) } else { (ev, sv) };
    let mut pts = curve
        .and_then(|c| curve_polyline(sf, c, a, b, tp, unsup))
        .unwrap_or_else(|| vec![a, b]);
    if !same_sense {
        pts.reverse();
    }
    if flip_oriented {
        pts.reverse();
    }
    Some(pts)
}

fn vertex_point(sf: &StepFile, id: u32) -> Option<V3> {
    // VERTEX_POINT('', point)
    let p = sf.params(id)?;
    cartesian_point(sf, p.get(1)?.as_ref_id()?)
}

/// Discretize `curve` from `a` to `b` (3D positions of the trimming vertices).
/// `unsup` tallies curve types we don't support (the edge then falls back to a
/// straight chord), so a silently-straightened boundary is reported.
fn curve_polyline(
    sf: &StepFile,
    id: u32,
    a: V3,
    b: V3,
    tp: &TessParams,
    unsup: &mut HashMap<String, usize>,
) -> Option<Vec<V3>> {
    let ty = sf.entity_type(id)?;
    match ty {
        "LINE" => Some(vec![a, b]),
        "CIRCLE" | "ELLIPSE" => {
            let p = sf.params(id)?;
            let f = axis2_placement(sf, p.get(1)?.as_ref_id()?)?;
            let (rx, ry) = if ty == "CIRCLE" {
                let r = p.get(2)?.as_f64()?;
                (r, r)
            } else {
                (p.get(2)?.as_f64()?, p.get(3)?.as_f64()?)
            };
            let ang = |pt: V3| -> f64 {
                let d = pt.sub(f.o);
                (d.dot(f.y) / ry).atan2(d.dot(f.x) / rx)
            };
            let t0 = ang(a);
            let mut t1 = ang(b);
            let full = a.sub(b).len() < 1e-9 * (1.0 + rx.abs());
            if full {
                t1 = t0 + std::f64::consts::TAU;
            } else {
                while t1 <= t0 + 1e-12 {
                    t1 += std::f64::consts::TAU;
                }
            }
            let step = angle_step(rx.max(ry), tp.deflection, tp.max_angle);
            // at least 2 segments: a face bounded by just a chord and a
            // shallow arc (a sliver) needs the arc's interior point, or the
            // closed loop collapses to 2 points and the face loses its only
            // boundary
            let nseg = (((t1 - t0) / step).ceil() as usize).max(2);
            let mut pts = Vec::with_capacity(nseg + 1);
            for i in 0..=nseg {
                let t = t0 + (t1 - t0) * i as f64 / nseg as f64;
                pts.push(
                    f.o.add(f.x.scale(rx * t.cos()))
                        .add(f.y.scale(ry * t.sin())),
                );
            }
            // snap endpoints exactly to the topological vertices
            *pts.first_mut()? = a;
            *pts.last_mut()? = b;
            Some(pts)
        }
        "B_SPLINE_CURVE_WITH_KNOTS" | "RATIONAL_B_SPLINE_CURVE" => {
            bspline_polyline(sf, id, sf.params(id)?, None, a, b)
        }
        "POLYLINE" => {
            let p = sf.params(id)?;
            let pts: Vec<V3> = p
                .get(1)?
                .as_list()?
                .iter()
                .filter_map(|v| v.as_ref_id())
                .filter_map(|r| cartesian_point(sf, r))
                .collect();
            if pts.len() >= 2 {
                Some(pts)
            } else {
                None
            }
        }
        "SURFACE_CURVE" | "SEAM_CURVE" | "INTERSECTION_CURVE" | "BOUNDED_CURVE" => {
            // SURFACE_CURVE('', curve_3d, (pcurves), repr) -> follow 3D curve
            let p = sf.params(id)?;
            curve_polyline(sf, p.get(1)?.as_ref_id()?, a, b, tp, unsup)
        }
        "TRIMMED_CURVE" => {
            // TRIMMED_CURVE('', basis, trim1, trim2, sense, mode): endpoints are
            // already given by the edge vertices, just discretize the basis.
            let p = sf.params(id)?;
            curve_polyline(sf, p.get(1)?.as_ref_id()?, a, b, tp, unsup)
        }
        crate::step::TYPE_COMPLEX => {
            // rational b-spline curve expressed as a complex instance: the
            // degree and control points live in the B_SPLINE_CURVE leaf, the
            // knot vector in B_SPLINE_CURVE_WITH_KNOTS, the weights in
            // RATIONAL_B_SPLINE_CURVE — merge the leaves before parsing
            let core = sf.complex_leaf(id, "B_SPLINE_CURVE_WITH_KNOTS")?;
            let mut params = sf.complex_leaf(id, "B_SPLINE_CURVE").unwrap_or_default();
            params.extend(core);
            let weights = sf
                .complex_leaf(id, "RATIONAL_B_SPLINE_CURVE")
                .and_then(|w| w.first().and_then(|l| l.as_list().map(|l| l.to_vec())))
                .map(|l| l.iter().filter_map(|v| v.as_f64()).collect::<Vec<f64>>());
            bspline_polyline_core(sf, &params, weights, a, b, true)
        }
        _ => {
            // unsupported curve type -> caller falls back to a straight segment
            // between the edge vertices; tally it so the lost curvature is
            // reported rather than silently straightening the boundary
            *unsup.entry(ty.to_string()).or_insert(0) += 1;
            None
        }
    }
}

fn bspline_polyline(
    sf: &StepFile,
    _id: u32,
    params: Vec<P>,
    weights: Option<Vec<f64>>,
    a: V3,
    b: V3,
) -> Option<Vec<V3>> {
    bspline_polyline_core(sf, &params, weights, a, b, false)
}

/// B_SPLINE_CURVE_WITH_KNOTS('name', degree, (cps), form, closed, self_isect,
///                            (mults), (knots), spec)
/// In complex instances the leading 'name' belongs to BOUNDED_CURVE /
/// B_SPLINE_CURVE leaves and the B_SPLINE_CURVE_WITH_KNOTS leaf often carries
/// only (mults),(knots),spec — so we detect field layout dynamically.
fn bspline_polyline_core(
    sf: &StepFile,
    params: &[P],
    weights: Option<Vec<f64>>,
    a: V3,
    b: V3,
    complex: bool,
) -> Option<Vec<V3>> {
    // Locate degree (first integer), control point list (first list of refs),
    // and the two trailing numeric lists (multiplicities, knots).
    let mut degree: Option<usize> = None;
    let mut cps: Option<Vec<V3>> = None;
    let mut num_lists: Vec<Vec<f64>> = Vec::new();

    let scan = |params: &[P],
                degree: &mut Option<usize>,
                cps: &mut Option<Vec<V3>>,
                num_lists: &mut Vec<Vec<f64>>| {
        for p in params {
            match p {
                P::I(v) if degree.is_none() => *degree = Some((*v).max(1) as usize),
                P::L(l) => {
                    if !l.is_empty() && l.iter().all(|e| matches!(e, P::Ref(_))) {
                        if cps.is_none() {
                            *cps = Some(
                                l.iter()
                                    .filter_map(|e| e.as_ref_id())
                                    .filter_map(|r| cartesian_point(sf, r))
                                    .collect(),
                            );
                        }
                    } else if !l.is_empty() && l.iter().all(|e| matches!(e, P::I(_) | P::F(_))) {
                        num_lists.push(l.iter().filter_map(|e| e.as_f64()).collect());
                    }
                }
                _ => {}
            }
        }
    };

    scan(params, &mut degree, &mut cps, &mut num_lists);

    // In complex instances the cps may live in the B_SPLINE_CURVE leaf
    if complex && cps.is_none() {
        // params came from the WITH_KNOTS leaf only; cannot recover here
        return None;
    }

    let degree = degree?;
    let cps = cps?;
    if cps.len() < 2 || num_lists.len() < 2 {
        return None;
    }
    let mults = &num_lists[num_lists.len() - 2];
    let knots_u = &num_lists[num_lists.len() - 1];

    let mut knots: Vec<f64> = Vec::new();
    for (m, k) in mults.iter().zip(knots_u.iter()) {
        for _ in 0..(*m as usize) {
            knots.push(*k);
        }
    }
    if knots.len() != cps.len() + degree + 1 {
        // malformed; fall back to control polygon
        let mut pts = cps;
        *pts.first_mut()? = a;
        *pts.last_mut()? = b;
        return Some(pts);
    }

    let w = weights.as_deref();
    let t0 = knots[degree];
    let t1 = knots[knots.len() - 1 - degree];
    let nseg = (cps.len().max(degree + 1) * 4).clamp(8, 512);
    let mut pts = Vec::with_capacity(nseg + 1);
    for i in 0..=nseg {
        let t = t0 + (t1 - t0) * i as f64 / nseg as f64;
        pts.push(bspline_curve_point(degree, &knots, &cps, w, t));
    }
    let mut pts = align_polyline_to_vertices(pts, a, b);
    *pts.first_mut()? = a;
    *pts.last_mut()? = b;
    Some(pts)
}

/// Align a sampled basis-curve polyline with the edge's trimming vertices.
/// Exporters trim edges to interior stretches of the basis curve, and close
/// closed edges at a vertex away from the curve's own parametric seam —
/// blindly snapping the curve's natural endpoints onto such vertices folds the
/// polyline through long false chords (and the face's UV contour with it).
fn align_polyline_to_vertices(pts: Vec<V3>, a: V3, b: V3) -> Vec<V3> {
    let n = pts.len();
    if n < 3 {
        return pts;
    }
    let mut step = 0.0f64;
    let mut len = 0.0f64;
    for i in 1..n {
        let d = pts[i].sub(pts[i - 1]).len();
        step = step.max(d);
        len += d;
    }
    let tol = step.max(1e-12);
    if pts[0].sub(a).len() <= tol && pts[n - 1].sub(b).len() <= tol {
        return pts; // vertices sit on the curve endpoints (the common case)
    }
    let nearest = |q: V3, ring: &[V3]| -> usize {
        let mut bi = 0usize;
        let mut bd = f64::MAX;
        for (i, p) in ring.iter().enumerate() {
            let d = p.sub(q).len();
            if d < bd {
                bd = d;
                bi = i;
            }
        }
        bi
    };
    if pts[0].sub(pts[n - 1]).len() <= 1e-6 * len.max(1e-9) {
        // closed basis curve: walk the ring forward from a to b (an edge
        // follows increasing parameter, wrapping through the curve's seam)
        let ring = &pts[..n - 1];
        let m = ring.len();
        let ia = nearest(a, ring);
        let span = if a.sub(b).len() <= tol {
            m // closed edge: the full ring, re-seamed at the vertex
        } else {
            let ib = nearest(b, ring);
            if ia == ib {
                return pts; // degenerate trim: keep the old behaviour
            }
            (ib + m - ia) % m
        };
        (0..=span).map(|k| ring[(ia + k) % m]).collect()
    } else {
        // open curve trimmed to an interior stretch
        let ia = nearest(a, &pts);
        let ib = nearest(b, &pts);
        if ia < ib {
            pts[ia..=ib].to_vec()
        } else {
            pts // vertices against the parameter direction: leave as-is
        }
    }
}
