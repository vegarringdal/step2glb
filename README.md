# step2glb

> **Proof of concept.** I needed to convert a pile of STEP files to glTF and
> couldn't find a tool that handled *all* of them, so this is an experiment in
> how far AI assistance could take a small, from-scratch CLI for the job. It
> was written with the help of misc AI tools. Treat it as such — it works on
> the models I threw at it, but it is not a hardened production converter.

Tessellate STEP (ISO 10303-21 / `.step` / `.stp`) files into binary glTF
(`.glb`) and inspect the assembly hierarchy, with low memory usage.

No geometry kernel dependency (no OpenCASCADE): the parser, math, surfaces,
tessellation and GLB writer are all in this crate, on top of four small
dependencies: `clap` (CLI), `md5` (mesh dedup keys), `meshopt`
(meshoptimizer pass) and `tess2-rust` (pure-Rust libtess2 port for polygon
triangulation with holes).

## What it does

- **Reads the product structure**: `PRODUCT` / `PRODUCT_DEFINITION` /
  `NEXT_ASSEMBLY_USAGE_OCCURRENCE` graph, with per-instance transforms from
  `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` +
  `ITEM_DEFINED_TRANSFORMATION` (both simple and complex-instance forms).
- **Tessellates B-rep geometry**: planes, the quadrics (cylinder / cone /
  sphere / torus), surfaces of linear extrusion and revolution, and
  (rational) B-spline / NURBS surfaces. Trimmed faces with holes are
  triangulated in UV space by tess2 (odd winding) — analytic surfaces invert
  UV in closed form, swept and B-spline surfaces via seeded Newton
  projection — then refined by midpoint edge subdivision until both the
  parametric step limits and a perpendicular chord-sag bound are met,
  Delaunay-flipped in metric UV to remove slivers, and mapped back to 3D
  with surface normals. Swept surfaces reduce to the equivalent quadric
  where possible (revolved line ∥ axis -> cylinder, slanted -> cone,
  revolved circle -> sphere/torus, extruded line -> plane). Faces that wrap
  fully around a periodic direction are cut at a seam and rebuilt as band
  polygons; boundary loops that encircle a sphere pole or cone apex are
  closed with a sampled polar cap; boundary loops that pass *through* a
  pole/apex (half-cones with the tip on the rim, domes split through the
  poles) walk the cap line between the adjacent meridians at the
  singularity, where `u` is otherwise undefined; a face bounded only by a
  seam "slit" (an edge walked out and back, enclosing no UV area — how some
  exporters write a full sphere as a single face) is recognized and
  tessellated as the whole closed surface.
- **Reads AP242 tessellated geometry** (`TRIANGULATED_FACE_SET`,
  `TESSELLATED_SOLID`, …) verbatim, and resolves `MAPPED_ITEM` instancing.
- **Reads colors**: `STYLED_ITEM` / `OVER_RIDING_STYLED_ITEM` presentation
  chains (both `COLOUR_RGB` and named pre-defined colours) are resolved per
  solid/shell/face and become per-color glTF primitives with their own PBR
  materials (deduplicated across the file).
- **Deduplicates meshes** two ways: per `PRODUCT_DEFINITION` (one mesh shared
  by all instances of a part) and by md5 over the geometry bytes (catches
  identical geometry exported under different ids).
- **Optimizes** every mesh with meshoptimizer: vertex weld → degenerate
  triangle removal → vertex-cache → vertex-fetch.
- **Writes a single `.glb`**: full node hierarchy with instance matrices,
  shared meshes, `POSITION`/`NORMAL` + 32-bit indices, and a root transform
  node converting the file's `LENGTH_UNIT` (mm, cm, m, inch, …) to meters
  and the Z-up engineering convention to glTF's Y-up (STEP has no up-axis
  field to read; pass `--up-axis y` if a model is already Y-up).
- **Merged mode (`--merged`)**: the rvm_parser_glb output layout instead —
  one node/mesh/material per color with everything baked to world space, and
  per-part drawcall ranges + the id hierarchy in the scene `extras` (see
  below).

Output of all test models validates clean against the Khronos glTF validator
(0 errors / 0 warnings / 0 infos).

## Build

```sh
cargo build --release
# binary at target/release/step2glb
```

`meshopt` compiles the bundled meshoptimizer C++ sources, so a C++ toolchain
is required, and its current bindings need a reasonably recent stable Rust
(1.82+). Everything else is pure Rust.

## Usage

```sh
# convert; writes model.glb next to the input
step2glb model.step

# choose output and tessellation quality (deflection is in file units, e.g. mm)
step2glb model.step -o out.glb --deflection 0.05 --max-angle 15

# NOTE: the tighter of the two bounds wins per feature. A curved face with
# radius r is governed by --max-angle whenever r < deflection / (1 - cos(a/2));
# at --deflection 0.5 that is every radius under ~58 mm (a=15°) / ~15 mm
# (a=30°). So to get a genuinely coarse mesh, raise --max-angle along with
# --deflection — e.g. --deflection 0.5 --max-angle 30

# one mesh per color + draw-range metadata (rvm_parser_glb layout)
step2glb model.step --merged

# smaller files: drop normals (viewers flat-shade), or full rvm-style cleanup
# (position weld + meshopt simplify + no normals) — both work with and
# without --merged
step2glb model.step --no-normals
step2glb model.step --cleanup-position

# just print the assembly tree
step2glb model.step --tree

# entity statistics (top types by count) + conversion
step2glb model.step --stats

# keep raw file units instead of scaling to meters
step2glb model.step --no-unit-scale

# input that is already Y-up: skip the default Z-up -> Y-up rotation
step2glb model.step --up-axis y

# skip the meshoptimizer pass
step2glb model.step --no-optimize

# tessellation threads (default: auto = CPU cores, capped at 4);
# output is byte-identical regardless of thread count
step2glb model.step -t 8
```

`--tree` output looks like:

```
AS1_PE_ASM  [geometry]
├─ PLATE  [geometry]
├─ L_BRACKET_ASSEMBLY_ASM  [geometry]
│  ├─ L-BRACKET  [geometry]
│  ├─ NUT_BOLT_ASSEMBLY_ASM  [geometry]
│  │  ├─ BOLT  [geometry]
│  │  └─ NUT  [geometry]
...
```

## Merged mode (`--merged`)

Produces the same GLB layout as
[rvm_parser_glb](https://github.com/vegarringdal/rvm_parser_glb), so the same
viewer code (e.g. three.js `BatchedMesh` selection + a treeview) works for
both RVM and STEP input:

- **One node + one mesh + one material per distinct color.** All instances
  are expanded and baked to world space (meters, Y-up via the same
  `(x, y, z) → (x, z, −y)` rotation rvm_parser_glb applies — `--up-axis y`
  skips it for already-Y-up input), so nodes carry
  no transforms and are named `node0`, `node1`, … with node `N` referencing
  mesh `N` / material `N`. Unlike rvm_parser_glb, `NORMAL` is kept alongside
  `POSITION` (the tessellator produces exact analytic normals).
- **Drawcall metadata in `scenes[0].extras`** — per-part index ranges into
  each merged mesh, plus the full instance tree:

```jsonc
"scenes": [{
  "nodes": [0, 1],
  "extras": {
    // Record<PART_ID, [FIRST_INDEX, INDEX_COUNT]> per color mesh;
    // offsets are elements into that mesh's index accessor
    "draw_ranges_node0": { "2": [0, 2112], "6": [2112, 720] },
    "draw_ranges_node1": { "4": [0, 1572] },
    // Record<ID, [NAME, PARENT_ID]>, "*" marks a root
    "id_hierarchy": {
      "1": ["AS1_PE_ASM", "*"],
      "2": ["PLATE", "1"],
      "3": ["L_BRACKET_ASSEMBLY_ASM", "1"],
      "4": ["L-BRACKET", "3"]
    }
  }
}]
```

Ids are assigned depth-first over the expanded assembly (so a part that is
instanced five times gets five ids and five draw ranges). A part whose
geometry spans several colors gets one range per color mesh under the same
id. Within each merged mesh the ranges are contiguous and tile the index
buffer exactly, so a raycast hit maps back to a part id by range lookup, and
selection/recolor is a `[start, count]` group per part. `asset.extras`
carries `"web3dversion": 2` like rvm_parser_glb.

Per-part mesh optimization still runs before merging (ranges stay valid;
merged meshes are never reordered afterwards), and unit scaling to meters
applies unless `--no-unit-scale` is given.

### Position cleanup (`--cleanup-position`)

Mirrors rvm_parser_glb's cleanup pipeline. With `--merged` it runs per part
instance before merging; without it, it runs once per unique (instanced)
mesh of the hierarchical output:
positions are welded on a quantized grid, the part is simplified with
`meshopt_simplify` (border locked, so seams between parts stay closed),
degenerate triangles (repeated index / coincident positions / near-zero
area) are dropped, and the vertex pool is compacted. Draw ranges are
recorded after this pass, so they always match the final index buffer.

Like in rvm_parser_glb this produces **positions-only** primitives — a
vertex welded across faces has no single valid normal, so `NORMAL` is
dropped and the viewer flat-shades or computes its own. Skip the flag to
keep the tessellator's exact analytic normals instead.

```sh
step2glb model.step --merged --cleanup-position

# rvm_parser_glb-equivalent knobs (same defaults)
#   --cleanup-precision 3       weld grid decimals, in file units
#   --meshopt-threshold 0.75    simplify target = threshold * index count
#   --meshopt-target-error 0.0  allowed simplification error
step2glb model.step --merged --cleanup-position \
  --meshopt-threshold 0.3 --meshopt-target-error 0.05
```

With `--meshopt-target-error 0` (the default, like rvm_parser_glb) only
zero-error collapses happen regardless of the threshold; give it a small
error budget to actually decimate toward the threshold.

Both meshopt knobs also work **on their own**, with or without `--merged`:
passing either one runs a simplify-only pass that keeps normals (and the
hierarchical layout, if not merging):

```sh
step2glb model.step --meshopt-threshold 0.3 --meshopt-target-error 0.05
```

Skipped faces are reported on stderr by surface type, so you always know
what a model needed that isn't supported yet — and, separately, which
*supported* surfaces failed trimming/tessellation (Newton non-convergence,
multi-winding periodic loops, degenerate bounds, …):

```
tessellated 1 unique meshes (112 faces ok, 193 skipped) in 34.9ms
unsupported surface types (faces skipped):
     163  B_SPLINE_SURFACE_WITH_KNOTS
      26  SURFACE_OF_LINEAR_EXTRUSION
       4  SURFACE_OF_REVOLUTION
faces skipped on supported surfaces (trimming/tessellation failed):
      12  TOROIDAL_SURFACE
```

## How it stays low-memory

The file is held once as a byte buffer. A single string/comment-aware pass
builds a compact index of `#id → (interned type id, parameter byte range)`
(16 bytes per entity) plus a per-type id list. Entity parameters are parsed
*lazily*, only for entities the pipeline actually touches, and dropped right
after use — no DOM of the file is ever materialized. A 12.6 MB / 195 000
entity file indexes in ~80 ms; a 15 MB assembly converts end-to-end in
~0.7 s with a ~90 MB peak RSS (geometry output dominates, not parsing).

## Module map

```
src/step.rs        Part-21 indexer + lazy parameter parser (incl. complex instances)
src/geom.rs        V3 / M4 / frames, analytic surfaces, B-spline curve eval
src/model.rs       typed entity accessors, edge-curve discretization
src/tessellate.rs  B-rep traversal, UV tessellation, seam handling, refinement
src/hierarchy.rs   product graph, NAUO edges, instance transforms
src/styles.rs      STYLED_ITEM color chains, named pre-defined colours
src/merge.rs       --merged: world-space bake, per-color merge, draw ranges
src/mesh.rs        TriMesh / MeshSet (per-color buckets), md5 hashing, meshopt
src/glb.rs         dependency-free binary glTF writer (hierarchical + merged)
src/main.rs        CLI
```

The crate is a lib + bin, so the pipeline can be embedded:

```rust
let sf = step2glb::step::StepFile::parse(std::fs::read("a.step")?)?;
let asm = step2glb::hierarchy::build(&sf);
```

## Supported entities (geometry)

| Kind      | Supported                                                                  |
| --------- | -------------------------------------------------------------------------- |
| Solids    | `MANIFOLD_SOLID_BREP`, `BREP_WITH_VOIDS`, `FACETED_BREP`, `SHELL_BASED_SURFACE_MODEL`, `FACE_BASED_SURFACE_MODEL` |
| Surfaces  | `PLANE`, `CYLINDRICAL_SURFACE`, `CONICAL_SURFACE`, `SPHERICAL_SURFACE`, `TOROIDAL_SURFACE`, `SURFACE_OF_LINEAR_EXTRUSION`, `SURFACE_OF_REVOLUTION`, `B_SPLINE_SURFACE_WITH_KNOTS` incl. the rational complex-instance form (+ near-planar fallback via Newell plane fit) |
| Curves    | `LINE`, `CIRCLE`, `ELLIPSE`, `B_SPLINE_CURVE_WITH_KNOTS` (incl. rational complex form), `POLYLINE`, `TRIMMED_CURVE`, `SURFACE_CURVE`/`SEAM_CURVE` (via 3D curve); anything else falls back to a straight segment |
| Tessellated | `TRIANGULATED_FACE_SET`, `TRIANGULATED_SURFACE_SET`, `TESSELLATED_SOLID`, `TESSELLATED_SHELL` |
| Instancing | `MAPPED_ITEM` / `REPRESENTATION_MAP`, NAUO assembly instances             |
| Presentation | `STYLED_ITEM`, `OVER_RIDING_STYLED_ITEM` -> `COLOUR_RGB` / `DRAUGHTING_PRE_DEFINED_COLOUR` |

## Tests

```sh
cargo test
```

- **Unit tests** (in each module, 35): Part-21 lexing/param parsing edge
  cases (escaped quotes, comments, complex instances, typed params),
  frame/matrix math, closed-form surface UV round-trips, Newton inversion
  round-trips on B-spline / extrusion / revolution surfaces (cold and
  hint-seeded), (rational) B-spline curve and surface evaluation against
  known closed forms, pole-cap reporting, `Curve3` evaluation/periods,
  STYLED_ITEM color-chain resolution, mesh welding/degenerate
  removal/hashing, GLB container layout, materials and JSON content, merged
  draw-range/id-hierarchy extras.
- **Integration tests** (`tests/integration.rs`) over STEP fixtures in
  `tests/fixtures/`:
  - `triangle.step` — minimal planar `ADVANCED_FACE`: exact area, normal
    direction and winding consistency.
  - `cylinder_band.step` — a full 360° cylindrical face bounded by two
    circles (the classic periodic-seam case): area within 1 %, all points on
    the cylinder, normals radial.
  - `assembly.step` — two products + NAUO + `ITEM_DEFINED_TRANSFORMATION`:
    asserts the tree shape and the (100, 0, 0) instance translation.
  - `as1_pe_203.stp` — the canonical real-world AS1 assembly: root name,
    4 top-level children, 18 leaf instances, 5 unique deduplicated meshes,
    zero failed faces.
  - `bspline_patch.step` — a trimmed `B_SPLINE_SURFACE_WITH_KNOTS`: exact
    area through Newton UV trimming, plus a rational complex-form parse
    test.
  - `extrusion_face.step` — a `SURFACE_OF_LINEAR_EXTRUSION` over a B-spline
    directrix (not reducible): exact lateral area, all points on the walls.
  - `revolution_cylinder.step` — `SURFACE_OF_REVOLUTION` of a line parallel
    to the axis: asserts reduction to an analytic cylinder and band area.
  - `sphere_cap.step` — a spherical face bounded by a single circle: the
    polar-cap path; area within 1.5 % of 2πrh and the pole present.
  - `half_cone_apex.step` — a 180° cone face whose boundary passes through
    the apex (the parameterization singularity): exact lateral area.
  - `hemisphere_poles.step` — a half sphere bounded by a great circle
    through both poles: area within 2 %, all points on the correct half.
  - `sphere_slit.step` — a full sphere as one face bounded by a seam slit
    (one meridian edge walked out and back): full 4πr² area.
  - `cone_complex_curve.step` — a cone sliver bounded by a rational B-spline
    conic in complex-instance form: all points on the cone.
  - `colored.step` — a `STYLED_ITEM` chain: color map -> mesh bucket -> GLB
    material assertions.
  - merged mode: draw ranges tile every color mesh's index buffer exactly and
    all ids resolve in `id_hierarchy` (as1), color buckets and the fallback
    part (colored.step), the Z-up -> Y-up bake (cylinder_band.step), and
    `--cleanup-position` output (positions-only primitives, ranges still
    tiling the simplified index buffers, output never larger).

## Known limitations / TODO

- [x] ~~Tessellation density on pathological B-splines~~: refinement now
      runs under a per-face deflection budget (~`4·area/deflection²`
      triangles — features below the deflection scale aren't representable
      anyway), so near-cusp parameterizations (swept tubes with path kinks,
      helical springs) stop at a sane density instead of exploding to the
      hard cap. The sag bound may go locally unmet right at a cusp.
- [ ] `PCURVE` support: trimming currently always projects 3D edge curves
      via Newton; using the file's 2D parameter curves when present would be
      faster and more robust near surface seams.
- [ ] Multi-winding periodic loops (|w| > 1) and polar caps on general
      surfaces of revolution are skipped.
- [ ] Transparency (`SURFACE_STYLE_TRANSPARENT`) and per-vertex colors.
- [ ] Optional `EXT_mesh_gpu_instancing` instead of node-per-instance for
      huge assemblies, and meshopt simplification LODs (`--simplify`).
- [ ] Streaming/chunked index for files that don't fit in RAM (current
      design holds the file bytes once; fine into the multi-GB range).
- [x] ~~Parallel tessellation~~: `-t/--threads` fans faces out over scoped
      std threads (no new dependency), default auto = CPU cores capped at 4.
      Results merge in face order, so output is byte-identical to serial.
- [ ] `--filter`/`--subtree` to export only part of the hierarchy.

## Note on the bundled `Cargo.lock`

None is committed; `cargo build` resolves fresh. The meshopt functions used
(`generate_vertex_remap`, `remap_*`, `optimize_vertex_cache`,
`optimize_vertex_fetch`) have identical signatures across 0.5/0.6, so minor
resolver differences are harmless.

## License

MIT
