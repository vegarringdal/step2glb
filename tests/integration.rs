//! End-to-end tests over real STEP text: parsing, hierarchy extraction,
//! tessellation correctness (area / normals / watertight-ish checks) and
//! GLB output validity.

use step2glb::geom::{v3, V3};
use step2glb::mesh::{MeshSet, TriMesh};
use step2glb::model::TessParams;
use step2glb::step::StepFile;
use step2glb::styles::{self, ColorMap};
use step2glb::tessellate::{self, Ctx, TessStats};
use step2glb::{glb, hierarchy, merge};

fn load(name: &str) -> StepFile {
    let path = format!("{}/tests/fixtures/{}", env!("CARGO_MANIFEST_DIR"), name);
    StepFile::parse(std::fs::read(path).expect("fixture")).expect("parse")
}

fn tp() -> TessParams {
    TessParams {
        deflection: 0.05,
        max_angle: 20.0_f64.to_radians(),
    }
}

fn tessellate_all(sf: &StepFile, types: &[&str]) -> (MeshSet, TessStats) {
    let tp = tp();
    let colors = styles::build_color_map(sf);
    tessellate_with(sf, &tp, &colors, types)
}

fn tessellate_with(
    sf: &StepFile,
    tp: &TessParams,
    colors: &ColorMap,
    types: &[&str],
) -> (MeshSet, TessStats) {
    let cx = Ctx {
        sf,
        tp,
        colors,
        threads: 1,
    };
    let mut set = MeshSet::default();
    let mut stats = TessStats::default();
    for ty in types {
        for &id in sf.of_type(ty) {
            tessellate::tessellate_item(&cx, id, None, &mut set, &mut stats);
        }
    }
    (set, stats)
}

fn total_area(m: &TriMesh) -> f64 {
    let p = |i: u32| {
        v3(
            m.positions[i as usize * 3] as f64,
            m.positions[i as usize * 3 + 1] as f64,
            m.positions[i as usize * 3 + 2] as f64,
        )
    };
    m.indices
        .chunks(3)
        .map(|t| p(t[1]).sub(p(t[0])).cross(p(t[2]).sub(p(t[0]))).len() * 0.5)
        .sum()
}

// ------------------------------------------------------------- planar face

#[test]
fn planar_triangle_face_tessellates_exactly() {
    let sf = load("triangle.step");
    let (set, stats) = tessellate_all(&sf, &["MANIFOLD_SOLID_BREP"]);
    let mesh = set.merged();
    assert_eq!(stats.faces_ok, 1);
    assert_eq!(stats.faces_failed, 0);
    assert!(!mesh.is_empty());
    // right triangle with legs of 10 -> area 50
    assert!((total_area(&mesh) - 50.0).abs() < 1e-6);
    // every normal is +z (same_sense = .T., plane axis = +z)
    for n in mesh.normals.chunks(3) {
        assert!((n[2] - 1.0).abs() < 1e-6, "normal {:?}", n);
    }
    // triangle winding must agree with the normal
    let p = |i: u32| {
        v3(
            mesh.positions[i as usize * 3] as f64,
            mesh.positions[i as usize * 3 + 1] as f64,
            mesh.positions[i as usize * 3 + 2] as f64,
        )
    };
    for t in mesh.indices.chunks(3) {
        let gn = p(t[1]).sub(p(t[0])).cross(p(t[2]).sub(p(t[0])));
        assert!(gn.z > 0.0, "winding disagrees with face normal");
    }
}

// ----------------------------------------------- edge with a null 3D curve

#[test]
fn edge_with_null_curve_falls_back_to_a_segment() {
    // A loop edge whose 3D curve is `$` is a straight segment between its
    // vertices; it must not drop the edge -> loop -> whole face to a skip.
    let sf = load("null_curve_edge.step");
    let (set, stats) = tessellate_all(&sf, &["MANIFOLD_SOLID_BREP"]);
    let mesh = set.merged();
    assert_eq!(stats.faces_ok, 1, "face must tessellate, not be skipped");
    assert_eq!(stats.faces_failed, 0);
    // right triangle with legs 12 and 8 -> area 48
    assert!((total_area(&mesh) - 48.0).abs() < 1e-6);
}

// ---------------------------- hole inscribed in a curved rim -> shrink retry

#[test]
fn inscribed_hole_recovers_by_shrinking() {
    // The square hole's corners lie exactly on the circular rim, so once the
    // rim is discretized they poke through it; the face must still tessellate
    // (holes nudged inward) instead of being skipped.
    let sf = load("inscribed_hole.step");
    let (set, stats) = tessellate_all(&sf, &["MANIFOLD_SOLID_BREP"]);
    let mesh = set.merged();
    assert_eq!(stats.faces_ok, 1, "inscribed-hole face must recover");
    assert_eq!(stats.faces_failed, 0);
    // pi*100 - 200 ~= 114.16, a touch more after the hole is nudged inward
    let a = total_area(&mesh);
    assert!((110.0..120.0).contains(&a), "area {}", a);
    for p in mesh.positions.chunks(3) {
        assert!(p[2].abs() < 1e-6, "off-plane point {:?}", p);
    }
}

// ------------------------------ thin curved face -> finer-retry on tess2 fail

#[test]
fn thin_arc_band_recovers_via_finer_retry() {
    // At a coarse deflection the two near-concentric boundary arcs (r40, r40.3)
    // self-intersect once discretized; the face must still tessellate via the
    // finer-retry fallback instead of being skipped.
    let sf = load("thin_arc_band.step");
    let coarse = TessParams {
        deflection: 1.0,
        max_angle: 25.0_f64.to_radians(),
    };
    let colors = styles::build_color_map(&sf);
    let (set, stats) = tessellate_with(&sf, &coarse, &colors, &["MANIFOLD_SOLID_BREP"]);
    let mesh = set.merged();
    assert_eq!(stats.faces_ok, 1, "thin band must recover via finer retry");
    assert_eq!(stats.faces_failed, 0);
    // annular sector area ~ (pi/3)*(40.3^2 - 40^2)/2 ~= 12.6
    assert!(
        (total_area(&mesh) - 12.6).abs() < 0.6,
        "area {}",
        total_area(&mesh)
    );
    for p in mesh.positions.chunks(3) {
        assert!(p[2].abs() < 1e-6, "off-plane point {:?}", p);
    }
}

// ----------------------------------- B-spline patch with no real boundary

#[test]
fn bspline_patch_with_degenerate_bound_tessellates_full_domain() {
    // A B-spline face whose only bound is degenerate (a VERTEX_LOOP / seam
    // slit) must tessellate over its whole knot domain, not be skipped: the
    // knot domain is the patch extent, so this reproduces just the patch.
    let sf = load("bspline_unbounded.step");
    let (set, stats) = tessellate_all(&sf, &["MANIFOLD_SOLID_BREP"]);
    let mesh = set.merged();
    assert_eq!(stats.faces_ok, 1, "patch must tessellate, not be skipped");
    assert_eq!(stats.faces_failed, 0);
    // flat 10x10 patch -> area 100, independent of tessellation density
    assert!((total_area(&mesh) - 100.0).abs() < 1e-4, "area {}", total_area(&mesh));
    // planar patch in z=0: every point on the plane
    for p in mesh.positions.chunks(3) {
        assert!(p[2].abs() < 1e-6, "off-plane point {:?}", p);
    }
}

// -------------------------------------------- periodic full cylinder band

#[test]
fn full_cylinder_band_handles_the_seam() {
    let sf = load("cylinder_band.step");
    let (set, stats) = tessellate_all(&sf, &["SHELL_BASED_SURFACE_MODEL"]);
    let mesh = set.merged();
    assert_eq!(stats.faces_ok, 1, "seam face must tessellate");
    // lateral area 2*pi*r*h = 2*pi*5*10; chordal mesh slightly underestimates
    let expect = std::f64::consts::TAU * 5.0 * 10.0;
    let area = total_area(&mesh);
    assert!(
        (area - expect).abs() / expect < 0.01,
        "area {} vs {}",
        area,
        expect
    );
    // all points must lie on the cylinder
    for c in mesh.positions.chunks(3) {
        let r = ((c[0] as f64).powi(2) + (c[1] as f64).powi(2)).sqrt();
        assert!((r - 5.0).abs() < 1e-6);
        assert!((-1e-6..=10.0 + 1e-6).contains(&(c[2] as f64)));
    }
    // normals radial and outward
    for i in 0..mesh.vertex_count() {
        let p = V3 {
            x: mesh.positions[i * 3] as f64,
            y: mesh.positions[i * 3 + 1] as f64,
            z: 0.0,
        }
        .norm();
        let n = V3 {
            x: mesh.normals[i * 3] as f64,
            y: mesh.normals[i * 3 + 1] as f64,
            z: mesh.normals[i * 3 + 2] as f64,
        };
        assert!(p.dot(n) > 0.99, "normal not radial/outward");
    }
}

// ------------------------------- two-edge sliver face (line + shallow arc)

#[test]
fn two_edge_arc_sliver_face_keeps_an_arc_point() {
    // Vendor-model excerpt: a planar sliver bounded by exactly two edges, a
    // chord and a ~12.7° arc (r 7.14). At a coarse deflection the arc used to
    // discretize to a single chord, collapsing the closed loop to 2 points
    // and dropping the face; arcs must always keep an interior point.
    let sf = load("two_edge_arc_sliver.step");
    let coarse = TessParams {
        deflection: 1.0,
        max_angle: 25.0_f64.to_radians(),
    };
    let colors = styles::build_color_map(&sf);
    let (set, stats) = tessellate_with(&sf, &coarse, &colors, &["MANIFOLD_SOLID_BREP"]);
    let mesh = set.merged();
    assert_eq!(stats.faces_ok, 1, "sliver face must tessellate");
    assert_eq!(stats.faces_failed, 0);
    // chord 1.584, sagitta 0.044 -> at minimum one triangle of area ~0.035,
    // converging to the lens area ~0.047 as the arc refines
    let area = total_area(&mesh);
    assert!(
        (0.02..=0.06).contains(&area),
        "sliver area out of range: {}",
        area
    );
}

// --------------- cylinder band, closed B-spline rims with off-seam vertices

#[test]
fn closed_bspline_rim_with_offset_seam_vertex_tessellates() {
    // Vendor-model excerpt: a full cylinder band (r 2.5) whose rims are
    // *closed* B-spline edges. The edge vertex sits half-way around the basis
    // curve's own seam; the rim polylines must be re-seamed at the vertex
    // instead of snapping the curve's endpoints across the cylinder (which
    // used to self-intersect the UV contour and drop the face).
    let sf = load("cylinder_offset_seam_rims.step");
    let (set, stats) = tessellate_all(&sf, &["MANIFOLD_SOLID_BREP"]);
    let mesh = set.merged();
    assert_eq!(stats.faces_ok, 1, "band face must tessellate");
    assert_eq!(stats.faces_failed, 0);
    // wavy rims at |y| in 7.6..8.0 -> mean height ~15.6, area ~ 2*pi*2.5*15.6
    let area = total_area(&mesh);
    assert!(
        (220.0..=270.0).contains(&area),
        "band area out of range: {}",
        area
    );
    // every vertex on the cylinder x^2 + (z-958)^2 = 2.5^2, |y| <= 8
    for c in mesh.positions.chunks(3) {
        let r = ((c[0] as f64).powi(2) + (c[2] as f64 - 958.0).powi(2)).sqrt();
        assert!((r - 2.5).abs() < 1e-3, "off-cylinder point {:?}", c);
        assert!((c[1] as f64).abs() <= 8.0 + 1e-3);
    }
}

// -------------------------------------------------- hierarchy + transforms

#[test]
fn assembly_hierarchy_and_instance_transform() {
    let sf = load("assembly.step");
    let asm = hierarchy::build(&sf);

    assert_eq!(asm.roots.len(), 1);
    let root = asm.roots[0];
    assert_eq!(asm.products[&root].name, "ASM");

    let kids = &asm.children[&root];
    assert_eq!(kids.len(), 1);
    assert_eq!(asm.products[&kids[0].child_pd].name, "PART_B");
    // ITEM_DEFINED_TRANSFORMATION: identity -> (100, 0, 0)
    let m = kids[0].transform.0;
    assert!((m[12] - 100.0).abs() < 1e-9, "tx = {}", m[12]);
    assert!((m[13]).abs() < 1e-9 && (m[14]).abs() < 1e-9);

    // part B owns the brep representation
    assert!(!asm.products[&kids[0].child_pd].shape_reps.is_empty());
}

#[test]
fn per_representation_length_unit_is_read_from_its_context() {
    // Autodesk mixes units across contexts in one file: a mm context and a
    // metre context. Each SHAPE_REPRESENTATION must report its own unit, so a
    // metre-context part in an otherwise-mm file isn't scaled to nothing.
    let src = "DATA;
#1=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#2=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT($,.METRE.));
#3=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));
#10=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNIT_ASSIGNED_CONTEXT((#1,#3))REPRESENTATION_CONTEXT('','3D'));
#11=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNIT_ASSIGNED_CONTEXT((#2,#3))REPRESENTATION_CONTEXT('','3D'));
#20=SHAPE_REPRESENTATION('mm part',(),#10);
#21=SHAPE_REPRESENTATION('metre part',(),#11);
ENDSEC;";
    let sf = StepFile::parse(src.as_bytes().to_vec()).expect("parse");
    use step2glb::model::{rep_unit_factor, representation_length_scale};
    assert_eq!(representation_length_scale(&sf, 20), Some(0.001));
    assert_eq!(representation_length_scale(&sf, 21), Some(1.0));
    // against a mm global: the mm part is unchanged, the metre part scales 1000x
    assert!((rep_unit_factor(&sf, 20, 0.001) - 1.0).abs() < 1e-9);
    assert!((rep_unit_factor(&sf, 21, 0.001) - 1000.0).abs() < 1e-6);
}

#[test]
fn composite_curve_edge_is_discretized_not_chorded() {
    // A triangle whose A->B edge is a COMPOSITE_CURVE that detours through the
    // apex M=(5,5,0) (two POLYLINE segments), closed by a straight B->A edge.
    // Discretized correctly the loop is the triangle A,M,B (area 25); if the
    // composite fell back to a straight chord the loop A->B->A is degenerate
    // (zero area) and the face would be lost.
    let src = "DATA;
#1=CARTESIAN_POINT('',(0.,0.,0.));
#2=CARTESIAN_POINT('',(10.,0.,0.));
#3=CARTESIAN_POINT('',(5.,5.,0.));
#4=VERTEX_POINT('',#1);
#5=VERTEX_POINT('',#2);
#10=POLYLINE('',(#1,#3));
#11=POLYLINE('',(#3,#2));
#12=COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,.T.,#10);
#13=COMPOSITE_CURVE_SEGMENT(.DISCONTINUOUS.,.T.,#11);
#14=COMPOSITE_CURVE('',(#12,#13),.F.);
#15=EDGE_CURVE('',#4,#5,#14,.T.);
#16=DIRECTION('',(-1.,0.,0.));
#17=VECTOR('',#16,1.);
#18=LINE('',#2,#17);
#19=EDGE_CURVE('',#5,#4,#18,.T.);
#20=ORIENTED_EDGE('',*,*,#15,.T.);
#21=ORIENTED_EDGE('',*,*,#19,.T.);
#22=EDGE_LOOP('',(#20,#21));
#23=FACE_OUTER_BOUND('',#22,.T.);
#25=DIRECTION('',(0.,0.,1.));
#26=DIRECTION('',(1.,0.,0.));
#24=AXIS2_PLACEMENT_3D('',#1,#25,#26);
#27=PLANE('',#24);
#28=ADVANCED_FACE('',(#23),#27,.T.);
#29=CLOSED_SHELL('',(#28));
#30=MANIFOLD_SOLID_BREP('',#29);
ENDSEC;";
    let sf = StepFile::parse(src.as_bytes().to_vec()).expect("parse");
    let (set, stats) = tessellate_all(&sf, &["MANIFOLD_SOLID_BREP"]);
    assert_eq!(stats.faces_ok, 1);
    assert!(stats.unsupported_curves.get("COMPOSITE_CURVE").is_none(), "composite handled");
    assert!((total_area(&set.merged()) - 25.0).abs() < 1e-6, "area {}", total_area(&set.merged()));
}

#[test]
fn parabola_and_hyperbola_edges_match_their_analytic_area() {
    // PARABOLA P(u)=C+f*u^2*x+2f*u*y with f=1: the arc u in [-1,1] (from
    // (1,-2) through the vertex (0,0) to (1,2)) closed by the chord x=1 bounds
    // area integral(-2..2)(1 - y^2/4) dy = 8/3.
    let para = "DATA;
#1=CARTESIAN_POINT('',(1.,-2.,0.));
#2=CARTESIAN_POINT('',(1.,2.,0.));
#3=VERTEX_POINT('',#1);
#4=VERTEX_POINT('',#2);
#5=CARTESIAN_POINT('',(0.,0.,0.));
#6=DIRECTION('',(0.,0.,1.));
#7=DIRECTION('',(1.,0.,0.));
#8=AXIS2_PLACEMENT_3D('',#5,#6,#7);
#9=PARABOLA('',#8,1.0);
#10=EDGE_CURVE('',#3,#4,#9,.T.);
#11=DIRECTION('',(0.,-1.,0.));
#12=VECTOR('',#11,1.);
#13=LINE('',#2,#12);
#14=EDGE_CURVE('',#4,#3,#13,.T.);
#15=ORIENTED_EDGE('',*,*,#10,.T.);
#16=ORIENTED_EDGE('',*,*,#14,.T.);
#17=EDGE_LOOP('',(#15,#16));
#18=FACE_OUTER_BOUND('',#17,.T.);
#19=PLANE('',#8);
#20=ADVANCED_FACE('',(#18),#19,.T.);
#21=CLOSED_SHELL('',(#20));
#22=MANIFOLD_SOLID_BREP('',#21);
ENDSEC;";
    // a fine deflection so the inscribed-polygon area converges to the analytic
    // value — this validates the parameterization, not just that it tessellates
    let fine = TessParams { deflection: 0.002, max_angle: 5.0_f64.to_radians() };
    let cols = ColorMap::new();
    let sf = StepFile::parse(para.as_bytes().to_vec()).expect("parse");
    let (set, stats) = tessellate_with(&sf, &fine, &cols, &["MANIFOLD_SOLID_BREP"]);
    assert_eq!(stats.faces_ok, 1);
    assert!(stats.unsupported_curves.get("PARABOLA").is_none());
    assert!((total_area(&set.merged()) - 8.0 / 3.0).abs() < 0.01, "parabola area {}", total_area(&set.merged()));

    // HYPERBOLA P(u)=C+a*cosh(u)*x+b*sinh(u)*y with a=b=1: arc u in [-1,1]
    // (vertex (1,0)) closed by the chord x=cosh(1) bounds area sinh(1)*cosh(1)-1.
    let hyp = "DATA;
#1=CARTESIAN_POINT('',(1.5430806348,-1.1752011936,0.));
#2=CARTESIAN_POINT('',(1.5430806348,1.1752011936,0.));
#3=VERTEX_POINT('',#1);
#4=VERTEX_POINT('',#2);
#5=CARTESIAN_POINT('',(0.,0.,0.));
#6=DIRECTION('',(0.,0.,1.));
#7=DIRECTION('',(1.,0.,0.));
#8=AXIS2_PLACEMENT_3D('',#5,#6,#7);
#9=HYPERBOLA('',#8,1.0,1.0);
#10=EDGE_CURVE('',#3,#4,#9,.T.);
#11=DIRECTION('',(0.,-1.,0.));
#12=VECTOR('',#11,1.);
#13=LINE('',#2,#12);
#14=EDGE_CURVE('',#4,#3,#13,.T.);
#15=ORIENTED_EDGE('',*,*,#10,.T.);
#16=ORIENTED_EDGE('',*,*,#14,.T.);
#17=EDGE_LOOP('',(#15,#16));
#18=FACE_OUTER_BOUND('',#17,.T.);
#19=PLANE('',#8);
#20=ADVANCED_FACE('',(#18),#19,.T.);
#21=CLOSED_SHELL('',(#20));
#22=MANIFOLD_SOLID_BREP('',#21);
ENDSEC;";
    let sf = StepFile::parse(hyp.as_bytes().to_vec()).expect("parse");
    let (set, stats) = tessellate_with(&sf, &fine, &cols, &["MANIFOLD_SOLID_BREP"]);
    let expect = 1.0_f64.sinh() * 1.0_f64.cosh() - 1.0;
    assert_eq!(stats.faces_ok, 1);
    assert!(stats.unsupported_curves.get("HYPERBOLA").is_none());
    assert!((total_area(&set.merged()) - expect).abs() < 0.01, "hyperbola area {} (expected {expect})", total_area(&set.merged()));
}

#[test]
fn poly_loop_face_recovers_from_an_inconsistent_declared_plane() {
    // A faceted-export quirk: a FACE_SURFACE has an explicit POLY_LOOP polygon
    // (a unit square in the XZ plane, normal +Y) but its declared PLANE has an
    // orthogonal normal (+Z). Projected onto the wrong plane the polygon
    // collapses to a zero-area line and used to fail as a "slit". The polygon is
    // authoritative, so the face must recover by re-fitting the plane to it.
    let src = "DATA;
#1=CARTESIAN_POINT('',(0.,0.,0.));
#2=CARTESIAN_POINT('',(10.,0.,0.));
#3=CARTESIAN_POINT('',(10.,0.,10.));
#4=CARTESIAN_POINT('',(0.,0.,10.));
#5=POLY_LOOP('',(#1,#2,#3,#4));
#6=FACE_OUTER_BOUND('',#5,.T.);
#10=CARTESIAN_POINT('',(0.,0.,0.));
#11=DIRECTION('',(0.,0.,1.));
#12=DIRECTION('',(1.,0.,0.));
#13=AXIS2_PLACEMENT_3D('',#10,#11,#12);
#14=PLANE('',#13);
#15=FACE_SURFACE('',(#6),#14,.T.);
#16=CLOSED_SHELL('',(#15));
#17=MANIFOLD_SOLID_BREP('',#16);
ENDSEC;";
    let sf = StepFile::parse(src.as_bytes().to_vec()).expect("parse");
    let (set, stats) = tessellate_all(&sf, &["MANIFOLD_SOLID_BREP"]);
    assert_eq!(stats.faces_ok, 1, "the planar polygon is recovered, not skipped");
    assert_eq!(stats.faces_failed, 0);
    // the unit-square (10x10) area is reproduced from the polygon's own plane
    assert!(
        (total_area(&set.merged()) - 100.0).abs() < 1e-6,
        "area {}",
        total_area(&set.merged())
    );
}

#[test]
fn entity_filter_resolves_owner_and_closure() {
    // `--filter #<id>` for a geometry entity (e.g. a face from `--split`): the
    // id must resolve to the product that owns it, and its reference closure
    // must be a small self-contained fragment, not the whole file.
    let sf = load("as1_pe_203.stp");
    let asm = hierarchy::build(&sf);
    let face = *sf
        .of_type("ADVANCED_FACE")
        .first()
        .expect("the fixture has faces");

    // owning_product points at a real product whose rep reaches the face
    let (pd, rep) = hierarchy::owning_product(&sf, &asm, face).expect("face has an owning part");
    assert!(asm.products.contains_key(&pd), "owner is a known product");
    assert!(sf.entity_type(rep).is_some_and(|t| t.contains("REPRESENTATION")));

    // the closure contains the face and pulls in its geometry (surface/edges),
    // but stays far smaller than the whole file
    let closure = hierarchy::reference_closure(&sf, &[face]);
    assert!(closure.contains(&face));
    assert!(closure.len() > 3, "closure pulls in the face's geometry");
    assert!(
        closure.len() < sf.entities.len() / 2,
        "a single face is a focused fragment, not half the model ({} of {})",
        closure.len(),
        sf.entities.len()
    );
}

#[test]
fn unsupported_curve_and_item_types_are_recorded() {
    // A planar triangle whose middle edge uses an OFFSET_CURVE_3D (a curve type
    // we do not discretize) plus a standalone GEOMETRIC_CURVE_SET (an item we do
    // not tessellate). Both must be tallied so the gaps surface in the console,
    // the benign AXIS2_PLACEMENT_3D datum must NOT be flagged, and the face must
    // still tessellate (the unsupported edge falls back to a straight chord).
    let src = "DATA;
#1=CARTESIAN_POINT('',(0.,0.,0.));
#2=CARTESIAN_POINT('',(10.,0.,0.));
#3=CARTESIAN_POINT('',(0.,10.,0.));
#4=VERTEX_POINT('',#1);
#5=VERTEX_POINT('',#2);
#6=VERTEX_POINT('',#3);
#7=DIRECTION('',(1.,0.,0.));
#8=VECTOR('',#7,1.);
#9=LINE('',#1,#8);
#31=OFFSET_CURVE_3D('',#9,1.0,.F.,#7);
#11=EDGE_CURVE('',#4,#5,#9,.T.);
#12=EDGE_CURVE('',#5,#6,#31,.T.);
#13=EDGE_CURVE('',#6,#4,#9,.T.);
#14=ORIENTED_EDGE('',*,*,#11,.T.);
#15=ORIENTED_EDGE('',*,*,#12,.T.);
#16=ORIENTED_EDGE('',*,*,#13,.T.);
#17=EDGE_LOOP('',(#14,#15,#16));
#18=FACE_OUTER_BOUND('',#17,.T.);
#22=AXIS2_PLACEMENT_3D('',#1,#20,#21);
#20=DIRECTION('',(0.,0.,1.));
#21=DIRECTION('',(1.,0.,0.));
#23=PLANE('',#22);
#24=ADVANCED_FACE('',(#18),#23,.T.);
#25=CLOSED_SHELL('',(#24));
#26=MANIFOLD_SOLID_BREP('',#25);
#40=GEOMETRIC_CURVE_SET('',());
ENDSEC;";
    let sf = StepFile::parse(src.as_bytes().to_vec()).expect("parse");
    let (set, stats) = tessellate_all(&sf, &["MANIFOLD_SOLID_BREP", "GEOMETRIC_CURVE_SET"]);
    assert_eq!(stats.unsupported_curves.get("OFFSET_CURVE_3D"), Some(&1));
    assert_eq!(stats.unsupported_items.get("GEOMETRIC_CURVE_SET"), Some(&1));
    assert!(stats.unsupported_items.get("AXIS2_PLACEMENT_3D").is_none());
    // the face is not lost — the unsupported edge degrades to a chord
    assert_eq!(stats.faces_ok, 1);
    assert!(!set.merged().is_empty());
}

#[test]
fn split_units_enumerate_solids_shells_and_faces() {
    use step2glb::tessellate::{split_units, SplitLevel};
    // a brep-with-voids (outer shell + one void shell) plus a standalone
    // shell-model with two shells, to exercise each split granularity
    let src = "DATA;
#10=ADVANCED_FACE('',(),#100,.T.);
#11=ADVANCED_FACE('',(),#100,.T.);
#12=ADVANCED_FACE('',(),#100,.T.);
#20=CLOSED_SHELL('',(#10,#11));
#21=CLOSED_SHELL('',(#12));
#22=ORIENTED_CLOSED_SHELL('',*,#21,.F.);
#30=BREP_WITH_VOIDS('',#20,(#22));
#40=OPEN_SHELL('',(#10));
#41=OPEN_SHELL('',(#11));
#50=SHELL_BASED_SURFACE_MODEL('',(#40,#41));
ENDSEC;";
    let sf = StepFile::parse(src.as_bytes().to_vec()).expect("parse");

    // solid: the brep is one unit; a shell-model's shells count individually
    assert_eq!(split_units(&sf, 30, SplitLevel::Solid), vec![30]);
    assert_eq!(split_units(&sf, 50, SplitLevel::Solid), vec![40, 41]);

    // shell: brep outer shell + resolved void shell; model shells pass through
    assert_eq!(split_units(&sf, 30, SplitLevel::Shell), vec![20, 21]);
    assert_eq!(split_units(&sf, 50, SplitLevel::Shell), vec![40, 41]);

    // face: every ADVANCED_FACE under the item (outer #10,#11 + void #12)
    assert_eq!(split_units(&sf, 30, SplitLevel::Face), vec![10, 11, 12]);
    assert_eq!(split_units(&sf, 50, SplitLevel::Face), vec![10, 11]);
}

#[test]
fn filter_roots_by_name_and_id() {
    let sf = load("assembly.step");
    let asm = hierarchy::build(&sf);

    // case-insensitive substring on the product name
    let by_name = hierarchy::filter_roots(&asm, "part_b");
    assert_eq!(by_name.len(), 1, "PART_B matches once");
    let pd = by_name[0];
    assert_eq!(asm.products[&pd].name, "PART_B");

    // explicit PRODUCT_DEFINITION id, with and without '#'
    assert_eq!(hierarchy::filter_roots(&asm, &format!("#{pd}")), vec![pd]);
    assert_eq!(hierarchy::filter_roots(&asm, &pd.to_string()), vec![pd]);

    // filtering to the assembly root yields exactly the root set
    assert_eq!(hierarchy::filter_roots(&asm, "ASM"), asm.roots);

    // no match -> empty (caller turns this into an error)
    assert!(hierarchy::filter_roots(&asm, "no-such-part").is_empty());
}

#[test]
fn filter_subtree_entities_reach_geometry() {
    let sf = load("assembly.step");
    let asm = hierarchy::build(&sf);
    let roots = hierarchy::filter_roots(&asm, "PART_B");
    assert_eq!(roots.len(), 1);

    let ids = hierarchy::subtree_entities(&sf, &asm, &roots);
    assert!(ids.contains(&roots[0]), "excerpt includes the matched PD");
    let ty = |pred: &dyn Fn(&str) -> bool| {
        ids.iter()
            .any(|&id| sf.entity_type(id).is_some_and(pred))
    };
    assert!(
        ty(&|t| t.contains("SHAPE_REPRESENTATION")),
        "closure must reach the shape representation"
    );
    assert!(
        ty(&|t| t.contains("BREP")),
        "closure must reach the brep geometry"
    );

    // deterministic: the extraction must not depend on hash iteration order
    assert_eq!(
        ids,
        hierarchy::subtree_entities(&sf, &asm, &roots),
        "subtree_entities must be deterministic"
    );
}

#[test]
fn as1_real_world_assembly() {
    let sf = load("as1_pe_203.stp");
    let asm = hierarchy::build(&sf);

    assert_eq!(asm.roots.len(), 1);
    let root_name = &asm.products[&asm.roots[0]].name;
    assert_eq!(root_name, "AS1_PE_ASM");

    // canonical structure: plate + 2 bracket assemblies + rod assembly
    let top = &asm.children[&asm.roots[0]];
    assert_eq!(top.len(), 4);
    let names: Vec<&str> = top.iter().map(|i| i.name.as_str()).collect();
    assert!(names.contains(&"PLATE"));
    assert_eq!(
        names
            .iter()
            .filter(|n| **n == "L_BRACKET_ASSEMBLY_ASM")
            .count(),
        2
    );

    // 18 mesh-bearing leaf instances overall (plate, 2 brackets, rod,
    // 8 bolts/nuts in brackets x..., 2 rod nuts): count leaves
    fn leaves(asm: &hierarchy::Assembly, pd: u32) -> usize {
        match asm.children.get(&pd) {
            None => 1,
            Some(k) if k.is_empty() => 1,
            Some(k) => k.iter().map(|i| leaves(asm, i.child_pd)).sum(),
        }
    }
    assert_eq!(leaves(&asm, asm.roots[0]), 18);
}

#[test]
fn as1_tessellation_and_dedup() {
    let sf = load("as1_pe_203.stp");
    let asm = hierarchy::build(&sf);
    let mut stats = TessStats::default();

    // tessellate every product with shape reps; hash for dedup
    let mut hashes = std::collections::HashSet::new();
    let mut meshes = 0;
    for node in asm.products.values() {
        let tp = tp();
        let colors = ColorMap::new();
        let cx = Ctx {
            sf: &sf,
            tp: &tp,
            colors: &colors,
            threads: 1,
        };
        let mut m = MeshSet::default();
        for &sr in &node.shape_reps {
            if let Some(p) = sf.params(sr) {
                if let Some(items) = p.get(1).and_then(|v| v.as_list()) {
                    for it in items {
                        if let Some(r) = it.as_ref_id() {
                            tessellate::tessellate_item(&cx, r, None, &mut m, &mut stats);
                        }
                    }
                }
            }
        }
        if !m.is_empty() {
            m.optimize();
            meshes += 1;
            hashes.insert(m.content_hash());
            // optimization must keep valid index buffers
            let flat = m.merged();
            assert!(flat
                .indices
                .iter()
                .all(|&i| (i as usize) < flat.vertex_count()));
            assert_eq!(flat.indices.len() % 3, 0);
        }
    }
    assert_eq!(stats.faces_failed, 0, "as1 has only planes and cylinders");
    // 5 distinct parts carry geometry: plate, bracket, bolt, nut, rod
    assert_eq!(meshes, 5);
    assert_eq!(hashes.len(), 5);
}

// --------------------------------------------------------------- GLB output

#[test]
fn glb_roundtrip_of_fixture_geometry() {
    let sf = load("triangle.step");
    let (mut set, _) = tessellate_all(&sf, &["MANIFOLD_SOLID_BREP"]);
    set.optimize();

    let mut b = glb::GlbBuilder::default();
    let mi = b.add_mesh(set, "tri".into());
    let n = b.add_node("root".into(), None, Some(mi));
    b.root_nodes = vec![n];
    let bytes = b.write("test");

    // container sanity
    assert_eq!(&bytes[0..4], b"glTF");
    let total = u32::from_le_bytes(bytes[8..12].try_into().unwrap()) as usize;
    assert_eq!(total, bytes.len());

    // JSON chunk parses and references the binary chunk consistently
    let jlen = u32::from_le_bytes(bytes[12..16].try_into().unwrap()) as usize;
    let json: serde_json::Value = serde_json::from_slice(&bytes[20..20 + jlen]).unwrap();
    let blen = json["buffers"][0]["byteLength"].as_u64().unwrap() as usize;
    let bin_declared = u32::from_le_bytes(bytes[20 + jlen..24 + jlen].try_into().unwrap()) as usize;
    assert_eq!(blen, bin_declared);
    for view in json["bufferViews"].as_array().unwrap() {
        let off = view["byteOffset"].as_u64().unwrap_or(0) as usize;
        let len = view["byteLength"].as_u64().unwrap() as usize;
        assert!(off + len <= blen, "bufferView overruns the BIN chunk");
    }
}

// ====================================================== new surface features

#[test]
fn bspline_patch_parses_and_trims_via_newton() {
    let sf = load("bspline_patch.step");
    // parsing: surface entity becomes a B-spline surface
    let sid = sf.of_type("B_SPLINE_SURFACE_WITH_KNOTS")[0];
    let surf = step2glb::model::surface(&sf, sid).expect("parsed");
    match &surf {
        step2glb::geom::Surface::BSpline(b) => {
            assert_eq!((b.deg_u, b.deg_v, b.nu, b.nv), (1, 1, 2, 2));
            assert!(surf.uses_newton());
        }
        other => panic!("expected BSpline surface, got {:?}", other),
    }
    // tessellation: triangle trim of the flat patch -> exact area 50, +z
    let (set, stats) = tessellate_all(&sf, &["SHELL_BASED_SURFACE_MODEL"]);
    assert_eq!((stats.faces_ok, stats.faces_failed), (1, 0));
    let mesh = set.merged();
    assert!(
        (total_area(&mesh) - 50.0).abs() < 0.05,
        "area {}",
        total_area(&mesh)
    );
    for n in mesh.normals.chunks(3) {
        assert!(n[2] > 0.99, "normal {:?}", n);
    }
    for c in mesh.positions.chunks(3) {
        assert!(c[2].abs() < 1e-4, "point off the patch: {:?}", c);
        assert!(c[0] >= -1e-4 && c[1] >= -1e-4 && c[0] + c[1] <= 10.0 + 1e-3);
    }
}

#[test]
fn extrusion_surface_face() {
    let sf = load("extrusion_face.step");
    let sid = sf.of_type("SURFACE_OF_LINEAR_EXTRUSION")[0];
    let surf = step2glb::model::surface(&sf, sid).expect("parsed");
    assert!(
        matches!(surf, step2glb::geom::Surface::Extrusion { .. }),
        "B-spline directrix must stay a general extrusion, got {:?}",
        surf
    );
    // zigzag length 20 extruded by 5 -> exact area 100
    let (set, stats) = tessellate_all(&sf, &["SHELL_BASED_SURFACE_MODEL"]);
    assert_eq!((stats.faces_ok, stats.faces_failed), (1, 0));
    let mesh = set.merged();
    assert!(
        (total_area(&mesh) - 100.0).abs() < 0.2,
        "area {}",
        total_area(&mesh)
    );
    // all points on the two wall planes y=0 (x in 0..10) or x=10 (y in 0..10)
    for c in mesh.positions.chunks(3) {
        let on_wall1 = c[1].abs() < 1e-3;
        let on_wall2 = (c[0] - 10.0).abs() < 1e-3;
        assert!(on_wall1 || on_wall2, "off the extrusion: {:?}", c);
        assert!((-1e-3..=5.0 + 1e-3).contains(&c[2]));
    }
}

#[test]
fn revolution_reduces_to_cylinder_and_band_tessellates() {
    let sf = load("revolution_cylinder.step");
    let sid = sf.of_type("SURFACE_OF_REVOLUTION")[0];
    let surf = step2glb::model::surface(&sf, sid).expect("parsed");
    match &surf {
        step2glb::geom::Surface::Cylinder(_, r) => assert!((r - 5.0).abs() < 1e-9),
        other => panic!(
            "line parallel to axis must reduce to a cylinder, got {:?}",
            other
        ),
    }
    assert!(!surf.uses_newton());
    let (set, stats) = tessellate_all(&sf, &["SHELL_BASED_SURFACE_MODEL"]);
    assert_eq!((stats.faces_ok, stats.faces_failed), (1, 0));
    let mesh = set.merged();
    let expect = std::f64::consts::TAU * 5.0 * 10.0;
    let area = total_area(&mesh);
    assert!(
        (area - expect).abs() / expect < 0.01,
        "area {} vs {}",
        area,
        expect
    );
    for c in mesh.positions.chunks(3) {
        let r = ((c[0] as f64).powi(2) + (c[1] as f64).powi(2)).sqrt();
        assert!((r - 5.0).abs() < 1e-6);
    }
}

#[test]
fn sphere_cap_face_closes_at_the_pole() {
    let sf = load("sphere_cap.step");
    let (set, stats) = tessellate_all(&sf, &["SHELL_BASED_SURFACE_MODEL"]);
    assert_eq!(
        (stats.faces_ok, stats.faces_failed),
        (1, 0),
        "single-circle spherical cap must tessellate"
    );
    let mesh = set.merged();
    // spherical cap area = 2*pi*r*h with r=5, h=5-3=2
    let expect = std::f64::consts::TAU * 5.0 * 2.0;
    let area = total_area(&mesh);
    assert!(
        (area - expect).abs() / expect < 0.015,
        "cap area {} vs {}",
        area,
        expect
    );
    let mut zmax = f64::MIN;
    for c in mesh.positions.chunks(3) {
        let r = ((c[0] as f64).powi(2) + (c[1] as f64).powi(2) + (c[2] as f64).powi(2)).sqrt();
        assert!((r - 5.0).abs() < 1e-5, "off the sphere: {:?}", c);
        assert!(c[2] as f64 >= 3.0 - 1e-5, "below the cap boundary: {:?}", c);
        zmax = zmax.max(c[2] as f64);
    }
    assert!(zmax > 5.0 - 1e-6, "the pole itself must be part of the cap");
}

// ===================================== parameterization singularities on rim

#[test]
fn half_cone_with_apex_on_boundary() {
    // a 180° cone face whose boundary passes through the apex (screw tips,
    // countersinks split in half): u is undefined at the apex, the
    // tessellator must follow both meridians instead of cutting across
    let sf = load("half_cone_apex.step");
    let (set, stats) = tessellate_all(&sf, &["SHELL_BASED_SURFACE_MODEL"]);
    assert_eq!((stats.faces_ok, stats.faces_failed), (1, 0));
    let mesh = set.merged();
    // half lateral cone area = pi * r * slant / 2, r = 5, slant = 5*sqrt(2)
    let expect = std::f64::consts::PI * 5.0 * (50.0_f64).sqrt() / 2.0;
    let area = total_area(&mesh);
    assert!(
        (area - expect).abs() / expect < 0.01,
        "area {} vs {}",
        area,
        expect
    );
    // every point on the cone: radius == -z for z in [-5, 0]
    for c in mesh.positions.chunks(3) {
        let r = ((c[0] as f64).powi(2) + (c[1] as f64).powi(2)).sqrt();
        assert!((r - (c[2] as f64 + 5.0)).abs() < 1e-4, "off cone: {:?}", c);
        assert!(c[1] >= -1e-4, "wrong half: {:?}", c);
    }
}

#[test]
fn hemisphere_bounded_through_both_poles() {
    // half sphere whose boundary great circle passes through both poles
    // (dome split in two): both pole singularities sit on the boundary
    let sf = load("hemisphere_poles.step");
    let (set, stats) = tessellate_all(&sf, &["SHELL_BASED_SURFACE_MODEL"]);
    assert_eq!((stats.faces_ok, stats.faces_failed), (1, 0));
    let mesh = set.merged();
    let expect = 2.0 * std::f64::consts::PI * 25.0;
    let area = total_area(&mesh);
    assert!(
        (area - expect).abs() / expect < 0.02,
        "area {} vs {}",
        area,
        expect
    );
    for c in mesh.positions.chunks(3) {
        let r = ((c[0] as f64).powi(2) + (c[1] as f64).powi(2) + (c[2] as f64).powi(2)).sqrt();
        assert!((r - 5.0).abs() < 1e-4, "off sphere: {:?}", c);
        assert!(c[0] >= -1e-3, "wrong hemisphere: {:?}", c);
    }
}

#[test]
fn full_sphere_with_slit_boundary() {
    // a full sphere written as one face whose boundary is a single meridian
    // edge walked forward and back (a seam slit enclosing no area)
    let sf = load("sphere_slit.step");
    let (set, stats) = tessellate_all(&sf, &["SHELL_BASED_SURFACE_MODEL"]);
    assert_eq!((stats.faces_ok, stats.faces_failed), (1, 0));
    let mesh = set.merged();
    let r = 0.689462533607661_f64;
    let expect = 4.0 * std::f64::consts::PI * r * r;
    let area = total_area(&mesh);
    assert!(
        (area - expect).abs() / expect < 0.03,
        "area {} vs {}",
        area,
        expect
    );
    // every point on the sphere (center (0, 2.5, 0))
    for c in mesh.positions.chunks(3) {
        let d = ((c[0] as f64).powi(2) + (c[1] as f64 - 2.5).powi(2) + (c[2] as f64).powi(2))
            .sqrt();
        assert!((d - r).abs() < 1e-4, "off sphere: {:?}", c);
    }
}

#[test]
fn cone_face_bounded_by_complex_rational_bspline_curve() {
    // a cone sliver bounded by a circle arc and a rational B-spline conic in
    // the complex-instance form (degree + control points in the
    // B_SPLINE_CURVE leaf, knots in B_SPLINE_CURVE_WITH_KNOTS)
    let sf = load("cone_complex_curve.step");
    let (set, stats) = tessellate_all(&sf, &["SHELL_BASED_SURFACE_MODEL"]);
    assert_eq!((stats.faces_ok, stats.faces_failed), (1, 0));
    let mesh = set.merged();
    assert!(!mesh.is_empty());
    // every point on the cone: radius == z + 1.1547 (45° half-angle,
    // r = 1.1547 at z = 0), z in [apex region, 0]
    for c in mesh.positions.chunks(3) {
        let r = ((c[0] as f64).powi(2) + (c[1] as f64).powi(2)).sqrt();
        assert!(
            (r - (c[2] as f64 + 1.15470053837925)).abs() < 1e-3,
            "off cone: {:?}",
            c
        );
        assert!((-1.16..=1e-3).contains(&(c[2] as f64)), "z range: {:?}", c);
    }
}

// ============================================================ parallelism

#[test]
fn parallel_tessellation_is_byte_identical_to_serial() {
    let sf = load("as1_pe_203.stp");
    let tp = tp();
    let colors = styles::build_color_map(&sf);
    let run = |threads: usize| {
        let cx = Ctx {
            sf: &sf,
            tp: &tp,
            colors: &colors,
            threads,
        };
        let mut set = MeshSet::default();
        let mut stats = TessStats::default();
        for &id in sf.of_type("MANIFOLD_SOLID_BREP") {
            tessellate::tessellate_item(&cx, id, None, &mut set, &mut stats);
        }
        (set.content_hash(), stats.faces_ok, stats.faces_failed)
    };
    let serial = run(1);
    assert_eq!(serial, run(4), "4 threads must match serial output");
    assert_eq!(serial, run(2), "2 threads must match serial output");
    assert!(serial.1 > 0);
}

// ======================================================= styles / materials

#[test]
fn styled_item_colors_reach_mesh_buckets_and_glb_materials() {
    let sf = load("colored.step");
    let colors = styles::build_color_map(&sf);
    assert_eq!(
        colors.get(&23),
        Some(&[1.0, 0.0, 0.0, 1.0]),
        "solid #23 is red"
    );

    let tp = tp();
    let (mut set, stats) = tessellate_with(&sf, &tp, &colors, &["MANIFOLD_SOLID_BREP"]);
    assert_eq!(stats.faces_failed, 0);
    set.optimize();
    assert_eq!(set.parts.len(), 1);
    assert_eq!(set.parts[0].0, Some([1.0, 0.0, 0.0, 1.0]));

    let mut b = glb::GlbBuilder::default();
    let mi = b.add_mesh(set, "red".into());
    let n = b.add_node("root".into(), None, Some(mi));
    b.root_nodes = vec![n];
    let bytes = b.write("test");
    let jlen = u32::from_le_bytes(bytes[12..16].try_into().unwrap()) as usize;
    let json: serde_json::Value = serde_json::from_slice(&bytes[20..20 + jlen]).unwrap();
    let mat = json["meshes"][0]["primitives"][0]["material"]
        .as_u64()
        .unwrap();
    let base = &json["materials"][mat as usize]["pbrMetallicRoughness"]["baseColorFactor"];
    assert_eq!(base[0], 1.0);
    assert_eq!(base[1], 0.0);
}

// ==================================================== merged (rvm-style) GLB

fn glb_json(bytes: &[u8]) -> serde_json::Value {
    assert_eq!(&bytes[0..4], b"glTF");
    let jlen = u32::from_le_bytes(bytes[12..16].try_into().unwrap()) as usize;
    serde_json::from_slice(&bytes[20..20 + jlen]).expect("valid GLB JSON")
}

fn build_merged_with(
    name: &str,
    cleanup: Option<merge::Cleanup>,
) -> (serde_json::Value, hierarchy::Assembly) {
    let sf = load(name);
    let asm = hierarchy::build(&sf);
    let tp = tp();
    let colors = styles::build_color_map(&sf);
    let cx = Ctx {
        sf: &sf,
        tp: &tp,
        colors: &colors,
        threads: 1,
    };
    let mut stats = TessStats::default();
    let opts = merge::MergeOptions {
        unit_scale: 1.0,
        file_unit_scale: 1.0,
        rotate_z_up: true,
        optimize: true,
        drop_normals: false,
        cleanup,
        simplify: None,
    };
    let (merged, _unique) = merge::build(&cx, &asm, opts, &mut stats);
    assert!(merged.bucket_count() > 0);
    (glb_json(&merged.write("test")), asm)
}

fn build_merged(name: &str) -> (serde_json::Value, hierarchy::Assembly) {
    build_merged_with(name, None)
}

#[test]
fn merged_as1_draw_ranges_tile_the_index_buffer() {
    let (json, asm) = build_merged("as1_pe_203.stp");

    // one node + mesh + material per color bucket, nodes flat under the
    // scene with no transform (world space is baked in)
    let nodes = json["nodes"].as_array().unwrap();
    let n_colors = nodes.len();
    assert_eq!(json["meshes"].as_array().unwrap().len(), n_colors);
    assert_eq!(json["materials"].as_array().unwrap().len(), n_colors);
    for (i, n) in nodes.iter().enumerate() {
        assert_eq!(n["name"], format!("node{}", i));
        assert_eq!(n["mesh"], i);
        assert!(n.get("matrix").is_none());
        assert_eq!(json["scenes"][0]["nodes"][i], i);
    }

    // per color mesh: ranges are disjoint, 3-aligned and tile the index
    // accessor exactly; across all meshes the 18 geometry-bearing leaf
    // instances each own at least one range
    let extras = &json["scenes"][0]["extras"];
    let mut all_ids = std::collections::HashSet::new();
    for i in 0..n_colors {
        let ranges = extras[&format!("draw_ranges_node{}", i)]
            .as_object()
            .unwrap();
        assert!(!ranges.is_empty());
        all_ids.extend(ranges.keys().cloned());
        let idx_acc = json["meshes"][i]["primitives"][0]["indices"]
            .as_u64()
            .unwrap() as usize;
        let total = json["accessors"][idx_acc]["count"].as_u64().unwrap();
        let mut spans: Vec<(u64, u64)> = ranges
            .values()
            .map(|v| (v[0].as_u64().unwrap(), v[1].as_u64().unwrap()))
            .collect();
        spans.sort_unstable();
        let mut at = 0u64;
        for (start, count) in spans {
            assert_eq!(start, at, "ranges must be contiguous");
            assert!(count > 0 && count % 3 == 0);
            at = start + count;
        }
        assert_eq!(at, total, "ranges must cover the whole index buffer");
    }
    assert_eq!(all_ids.len(), 18);

    // id_hierarchy holds every expanded instance (groups included)
    fn count_instances(asm: &hierarchy::Assembly, pd: u32) -> usize {
        1 + asm
            .children
            .get(&pd)
            .map(|k| k.iter().map(|i| count_instances(asm, i.child_pd)).sum())
            .unwrap_or(0)
    }
    let expected: usize = asm
        .roots
        .iter()
        .map(|&r| count_instances(&asm, r))
        .sum();
    let idh = extras["id_hierarchy"].as_object().unwrap();
    assert_eq!(idh.len(), expected);

    // the root entry is the assembly, parent "*"; every range id and every
    // parent id resolves within id_hierarchy
    let roots: Vec<&str> = idh
        .values()
        .filter(|v| v[1] == "*")
        .map(|v| v[0].as_str().unwrap())
        .collect();
    assert_eq!(roots, ["AS1_PE_ASM"]);
    for id in &all_ids {
        assert!(idh.contains_key(id), "draw range id {} not in hierarchy", id);
    }
    for v in idh.values() {
        let p = v[1].as_str().unwrap();
        assert!(p == "*" || idh.contains_key(p), "dangling parent {}", p);
    }

    assert_eq!(json["asset"]["extras"]["web3dversion"], 2);
}

#[test]
fn merged_colored_part_gets_red_material_bucket() {
    let (json, _) = build_merged("colored.step");
    assert_eq!(json["meshes"].as_array().unwrap().len(), 1);
    let base = &json["materials"][0]["pbrMetallicRoughness"]["baseColorFactor"];
    assert_eq!(base[0], 1.0);
    assert_eq!(base[1], 0.0);
    // no product structure -> single fallback part "geometry" with id 1
    let extras = &json["scenes"][0]["extras"];
    assert!(extras["draw_ranges_node0"]["1"].is_array());
    assert_eq!(extras["id_hierarchy"]["1"][0], "geometry");
    assert_eq!(extras["id_hierarchy"]["1"][1], "*");
}

#[test]
fn merged_output_is_y_up() {
    // cylinder along +z (r=5, z in 0..10) must come out along +y
    let (json, _) = build_merged("cylinder_band.step");
    let pos_acc = json["meshes"][0]["primitives"][0]["attributes"]["POSITION"]
        .as_u64()
        .unwrap() as usize;
    let acc = &json["accessors"][pos_acc];
    let g = |k: &str, i: usize| acc[k][i].as_f64().unwrap();
    // chord vertices lie on the cylinder but not exactly at angle pi, so the
    // radial extents carry the chordal sag; the axial extent (now y) is exact
    assert!((g("min", 0) - -5.0).abs() < 0.1 && (g("max", 0) - 5.0).abs() < 0.1);
    assert!((g("min", 1) - 0.0).abs() < 1e-3 && (g("max", 1) - 10.0).abs() < 1e-3);
    assert!((g("min", 2) - -5.0).abs() < 0.1 && (g("max", 2) - 5.0).abs() < 0.1);
}

#[test]
fn merged_without_rotation_keeps_z_up() {
    // --up-axis y: the cylinder must stay along +z
    let sf = load("cylinder_band.step");
    let asm = hierarchy::build(&sf);
    let tp = tp();
    let colors = styles::build_color_map(&sf);
    let cx = Ctx {
        sf: &sf,
        tp: &tp,
        colors: &colors,
        threads: 1,
    };
    let mut stats = TessStats::default();
    let opts = merge::MergeOptions {
        unit_scale: 1.0,
        file_unit_scale: 1.0,
        rotate_z_up: false,
        optimize: true,
        drop_normals: false,
        cleanup: None,
        simplify: None,
    };
    let (merged, _) = merge::build(&cx, &asm, opts, &mut stats);
    let json = glb_json(&merged.write("test"));
    let pos_acc = json["meshes"][0]["primitives"][0]["attributes"]["POSITION"]
        .as_u64()
        .unwrap() as usize;
    let acc = &json["accessors"][pos_acc];
    let g = |k: &str, i: usize| acc[k][i].as_f64().unwrap();
    assert!((g("min", 2) - 0.0).abs() < 1e-3 && (g("max", 2) - 10.0).abs() < 1e-3);
}

#[test]
fn merged_cleanup_position_drops_normals_and_keeps_valid_ranges() {
    let cleanup = merge::Cleanup {
        precision: 3,
        threshold: 0.75,
        target_error: 0.0,
    };
    let (json, _) = build_merged_with("as1_pe_203.stp", Some(cleanup));
    let (plain, _) = build_merged("as1_pe_203.stp");

    // positions-only output, exactly like rvm_parser_glb with
    // --cleanup-position on: no NORMAL attribute anywhere
    let extras = &json["scenes"][0]["extras"];
    let meshes = json["meshes"].as_array().unwrap();
    for (i, m) in meshes.iter().enumerate() {
        let attrs = m["primitives"][0]["attributes"].as_object().unwrap();
        assert!(attrs.contains_key("POSITION"));
        assert!(!attrs.contains_key("NORMAL"), "mesh {} kept normals", i);

        // ranges still tile the (simplified) index buffer exactly
        let idx_acc = m["primitives"][0]["indices"].as_u64().unwrap() as usize;
        let total = json["accessors"][idx_acc]["count"].as_u64().unwrap();
        let mut spans: Vec<(u64, u64)> = extras[&format!("draw_ranges_node{}", i)]
            .as_object()
            .unwrap()
            .values()
            .map(|v| (v[0].as_u64().unwrap(), v[1].as_u64().unwrap()))
            .collect();
        spans.sort_unstable();
        let mut at = 0u64;
        for (start, count) in spans {
            assert_eq!(start, at);
            assert!(count > 0 && count % 3 == 0);
            at = start + count;
        }
        assert_eq!(at, total);
    }

    // the simplify + weld pass must not grow the output
    let count = |j: &serde_json::Value, ty: &str| -> u64 {
        j["accessors"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|a| a["type"] == ty)
            .map(|a| a["count"].as_u64().unwrap())
            .sum()
    };
    assert!(count(&json, "SCALAR") <= count(&plain, "SCALAR"));
    assert!(count(&json, "VEC3") < count(&plain, "VEC3"));
}

#[test]
fn hierarchical_cleanup_writes_position_only_glb() {
    // --cleanup-position without --merged: weld + simplify per unique mesh,
    // normals dropped, classic node hierarchy kept
    let sf = load("colored.step");
    let (mut set, _) = tessellate_all(&sf, &["MANIFOLD_SOLID_BREP"]);
    set.optimize();
    set.cleanup_positions(3, 0.75, 0.0);

    let mut b = glb::GlbBuilder::default();
    let mi = b.add_mesh(set, "part".into());
    let n = b.add_node("root".into(), None, Some(mi));
    b.root_nodes = vec![n];
    let json = glb_json(&b.write("test"));

    let attrs = json["meshes"][0]["primitives"][0]["attributes"]
        .as_object()
        .unwrap();
    assert!(attrs.contains_key("POSITION"));
    assert!(!attrs.contains_key("NORMAL"));
    // still the hierarchical layout: named nodes, no scene draw-range extras
    assert_eq!(json["nodes"][0]["name"], "root");
    assert!(json["scenes"][0].get("extras").is_none());
    // index buffer stays valid
    let idx_acc = json["meshes"][0]["primitives"][0]["indices"]
        .as_u64()
        .unwrap() as usize;
    assert_eq!(json["accessors"][idx_acc]["count"].as_u64().unwrap() % 3, 0);
}

#[test]
fn rational_complex_form_bspline_surface_parses() {
    // uniform weights = same geometry as the unweighted bilinear patch
    let src = "DATA;
#1=CARTESIAN_POINT('',(0.,0.,0.));
#2=CARTESIAN_POINT('',(0.,10.,0.));
#3=CARTESIAN_POINT('',(10.,0.,0.));
#4=CARTESIAN_POINT('',(10.,10.,0.));
#5=(B_SPLINE_SURFACE(1,1,((#1,#2),(#3,#4)),.UNSPECIFIED.,.F.,.F.,.F.)
B_SPLINE_SURFACE_WITH_KNOTS((2,2),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.)
BOUNDED_SURFACE()GEOMETRIC_REPRESENTATION_ITEM()
RATIONAL_B_SPLINE_SURFACE(((2.,2.),(2.,2.)))REPRESENTATION_ITEM('')SURFACE());
ENDSEC;";
    let sf = StepFile::parse(src.as_bytes().to_vec()).unwrap();
    let surf = step2glb::model::surface(&sf, 5).expect("complex rational surface parses");
    match &surf {
        step2glb::geom::Surface::BSpline(b) => {
            assert!(b.weights.is_some());
            let p = b.point(0.3, 0.7);
            assert!((p.x - 3.0).abs() < 1e-9 && (p.y - 7.0).abs() < 1e-9);
        }
        other => panic!("expected BSpline, got {:?}", other),
    }
}
