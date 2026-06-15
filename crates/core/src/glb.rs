//! Minimal GLB (binary glTF 2.0) writer — no serde, JSON is emitted by hand.
//!
//! One buffer; per primitive: a POSITION view, a NORMAL view and an index
//! view. A mesh holds one primitive per color bucket; materials are deduped
//! across the file with material 0 as the uncolored default.

use crate::geom::M4;
use crate::io::{MemSink, MemTemp, OutputHandle, TempHandle};
use crate::mesh::{as_bytes, as_bytes_u32, quantize_color, MeshSet, TriMesh};

/// Appends the GLB binary chunk to a [`TempHandle`] (spillable to disk under a
/// memory threshold), tracking the running offset. Every payload here is f32 /
/// u32, so the chunk is naturally 4-byte aligned.
struct Bin<'a> {
    tmp: &'a mut dyn TempHandle,
    off: usize,
}

impl<'a> Bin<'a> {
    fn new(tmp: &'a mut dyn TempHandle) -> Self {
        Bin { tmp, off: 0 }
    }
    /// Append bytes, returning their start offset within the buffer.
    fn append(&mut self, bytes: &[u8]) -> usize {
        let at = self.off;
        self.tmp.write_at(at as u64, bytes).expect("temp write");
        self.off += bytes.len();
        at
    }
    fn align(&mut self, to: usize) {
        while !self.off.is_multiple_of(to) {
            self.tmp
                .write_at(self.off as u64, &[0])
                .expect("temp write");
            self.off += 1;
        }
    }
    fn len(&self) -> usize {
        self.off
    }
}

/// Stream the GLB container to `out`: the 12-byte header, the padded JSON chunk,
/// then the BIN chunk copied back from `tmp`. Nothing larger than the JSON
/// metadata is held in RAM here — the geometry lives in `tmp`.
fn finish_glb(
    out: &mut dyn OutputHandle,
    json: String,
    tmp: &dyn TempHandle,
) -> std::io::Result<()> {
    let mut json_bytes = json.into_bytes();
    while !json_bytes.len().is_multiple_of(4) {
        json_bytes.push(b' ');
    }
    let bin_len = tmp.len() as usize;
    let bin_pad = (4 - bin_len % 4) % 4;
    let total = 12 + 8 + json_bytes.len() + 8 + bin_len + bin_pad;

    let mut header = Vec::with_capacity(20);
    header.extend_from_slice(b"glTF");
    header.extend_from_slice(&2u32.to_le_bytes());
    header.extend_from_slice(&(total as u32).to_le_bytes());
    header.extend_from_slice(&(json_bytes.len() as u32).to_le_bytes());
    header.extend_from_slice(b"JSON");
    out.write(&header)?;
    out.write(&json_bytes)?;

    let mut bin_header = Vec::with_capacity(8);
    bin_header.extend_from_slice(&((bin_len + bin_pad) as u32).to_le_bytes());
    bin_header.extend_from_slice(b"BIN\0");
    out.write(&bin_header)?;

    // copy the binary chunk out of the spill in 1 MiB strides
    let mut buf = vec![0u8; 1 << 20];
    let mut off = 0u64;
    while (off as usize) < bin_len {
        let n = tmp.read_at(off, &mut buf)?;
        if n == 0 {
            break;
        }
        out.write(&buf[..n])?;
        off += n as u64;
    }
    if bin_pad > 0 {
        out.write(&[0u8; 4][..bin_pad])?;
    }
    Ok(())
}

pub struct GlbNode {
    pub name: String,
    pub matrix: Option<M4>,
    pub mesh: Option<usize>,
    pub children: Vec<usize>,
}

#[derive(Default)]
pub struct GlbBuilder {
    pub nodes: Vec<GlbNode>,
    pub root_nodes: Vec<usize>,
    pub meshes: Vec<(String, MeshSet)>,
}

impl GlbBuilder {
    pub fn add_node(&mut self, name: String, matrix: Option<M4>, mesh: Option<usize>) -> usize {
        self.nodes.push(GlbNode {
            name,
            matrix: matrix.filter(|m| !m.is_identity(1e-12)),
            mesh,
            children: Vec::new(),
        });
        self.nodes.len() - 1
    }

    pub fn add_mesh(&mut self, mesh: MeshSet, name: String) -> usize {
        self.meshes.push((name, mesh));
        self.meshes.len() - 1
    }

    pub fn total_vertices(&self) -> usize {
        self.meshes.iter().map(|(_, m)| m.vertex_count()).sum()
    }

    pub fn total_triangles(&self) -> usize {
        self.meshes.iter().map(|(_, m)| m.triangle_count()).sum()
    }

    /// Convenience all-in-RAM write (used by tests and callers that want bytes).
    pub fn write(&self, generator: &str) -> Vec<u8> {
        let mut out = MemSink::default();
        let mut tmp = MemTemp::default();
        self.write_stream(generator, &mut out, &mut tmp)
            .expect("in-memory GLB write is infallible");
        out.0
    }

    /// Stream the GLB to `out`, spilling the binary chunk through `tmp` (which
    /// the caller may back with an on-disk temp file to stay under a memory
    /// ceiling). The JSON metadata is the only large thing kept in RAM here.
    pub fn write_stream(
        &self,
        generator: &str,
        out: &mut dyn OutputHandle,
        tmp: &mut dyn TempHandle,
    ) -> std::io::Result<()> {
        let mut bin = Bin::new(tmp);
        let mut views = String::new();
        let mut accessors = String::new();
        let mut meshes_json = String::new();
        let mut n_views = 0usize;
        let mut n_acc = 0usize;

        // material table: 0 = default; colored materials deduped by RGBA bits
        let mut materials_json = String::from(
            "{\"name\":\"default\",\"pbrMetallicRoughness\":{\
             \"baseColorFactor\":[0.72,0.72,0.75,1.0],\
             \"metallicFactor\":0.1,\"roughnessFactor\":0.7},\"doubleSided\":true}",
        );
        let mut mat_index: std::collections::HashMap<[u32; 4], usize> =
            std::collections::HashMap::new();
        let mut n_materials = 1usize;

        for (name, set) in &self.meshes {
            let mut prims = String::new();
            for (color, m) in &set.parts {
                if m.is_empty() {
                    continue;
                }
                let material = match color {
                    None => 0,
                    Some(c) => {
                        let key = [
                            c[0].to_bits(),
                            c[1].to_bits(),
                            c[2].to_bits(),
                            c[3].to_bits(),
                        ];
                        *mat_index.entry(key).or_insert_with(|| {
                            materials_json.push_str(&format!(
                                ",{{\"name\":\"color_{}\",\"pbrMetallicRoughness\":{{\
                                 \"baseColorFactor\":[{},{},{},{}],\
                                 \"metallicFactor\":0.1,\"roughnessFactor\":0.7}},\
                                 \"doubleSided\":true}}",
                                n_materials,
                                fmt_f32(c[0]),
                                fmt_f32(c[1]),
                                fmt_f32(c[2]),
                                fmt_f32(c[3]),
                            ));
                            let i = n_materials;
                            n_materials += 1;
                            i
                        })
                    }
                };
                let (pos_acc, nrm_acc, idx_acc) = write_primitive(
                    m,
                    &mut bin,
                    &mut views,
                    &mut accessors,
                    &mut n_views,
                    &mut n_acc,
                );
                if !prims.is_empty() {
                    prims.push(',');
                }
                prims.push_str(&format!(
                    "{{\"attributes\":{{{}}},\
                     \"indices\":{},\"mode\":{},\"material\":{}}}",
                    attributes_json(pos_acc, nrm_acc),
                    idx_acc,
                    if m.lines { 1 } else { 4 }, // glTF LINES vs TRIANGLES
                    material
                ));
            }
            if !meshes_json.is_empty() {
                meshes_json.push(',');
            }
            meshes_json.push_str(&format!(
                "{{\"name\":{},\"primitives\":[{}]}}",
                json_str(name),
                prims
            ));
        }

        // nodes
        let mut nodes_json = String::new();
        for n in &self.nodes {
            if !nodes_json.is_empty() {
                nodes_json.push(',');
            }
            nodes_json.push('{');
            nodes_json.push_str(&format!("\"name\":{}", json_str(&n.name)));
            if let Some(m) = &n.matrix {
                nodes_json.push_str(",\"matrix\":[");
                for (i, v) in m.0.iter().enumerate() {
                    if i > 0 {
                        nodes_json.push(',');
                    }
                    nodes_json.push_str(&fmt_f64(*v));
                }
                nodes_json.push(']');
            }
            if let Some(mi) = n.mesh {
                nodes_json.push_str(&format!(",\"mesh\":{}", mi));
            }
            if !n.children.is_empty() {
                nodes_json.push_str(",\"children\":[");
                for (i, c) in n.children.iter().enumerate() {
                    if i > 0 {
                        nodes_json.push(',');
                    }
                    nodes_json.push_str(&c.to_string());
                }
                nodes_json.push(']');
            }
            nodes_json.push('}');
        }

        let scene_nodes = self
            .root_nodes
            .iter()
            .map(|n| n.to_string())
            .collect::<Vec<_>>()
            .join(",");

        bin.align(4);
        let blen = bin.len();
        let json = format!(
            concat!(
                "{{\"asset\":{{\"version\":\"2.0\",\"generator\":{gen}}},",
                "\"scene\":0,\"scenes\":[{{\"nodes\":[{scene}]}}],",
                "\"nodes\":[{nodes}],",
                "\"meshes\":[{meshes}],",
                "\"materials\":[{materials}],",
                "\"accessors\":[{acc}],",
                "\"bufferViews\":[{views}],",
                "\"buffers\":[{{\"byteLength\":{blen}}}]}}"
            ),
            gen = json_str(generator),
            scene = scene_nodes,
            nodes = nodes_json,
            meshes = meshes_json,
            materials = materials_json,
            acc = accessors,
            views = views,
            blen = blen,
        );

        finish_glb(out, json, tmp)
    }
}

// -------------------------------------------------------------- merged output

/// rvm_parser_glb-compatible layout: one node + one mesh + one material per
/// distinct color, geometry already baked to world space, and per-part
/// drawcall metadata in the scene `extras`:
///
/// ```json
/// "scenes": [{
///   "extras": {
///     "draw_ranges_node0": { "<partId>": [firstIndex, indexCount], ... },
///     "id_hierarchy":      { "<id>": ["<name>", "<parentId or '*'>"], ... }
///   }
/// }]
/// ```
///
/// Nodes are named `node<N>` and reference mesh `N` / material `N`, so the
/// `draw_ranges_node<N>` key is resolvable from the node name. Ranges are
/// offsets into that mesh's index accessor (element counts, not bytes).
///
/// One id per draw call: a part's first color reuses the part's id, and each
/// further color of the same part is added to `id_hierarchy` as its own
/// numbered child node. So every id appears in exactly one `draw_ranges_node`
/// (never shared across color meshes), and a part that spans several colors is
/// the set of its own id plus those child ids.
#[derive(Default)]
pub struct MergedBuilder {
    buckets: Vec<MergedBucket>,
    /// (id, name, parent id); parent 0 means root (`"*"` in the JSON)
    hierarchy: Vec<(u32, String, u32)>,
}

struct MergedBucket {
    color: Option<[f32; 4]>,
    mesh: TriMesh,
    /// (part id, first index, index count) in emission order
    ranges: Vec<(u32, u32, u32)>,
}

impl MergedBuilder {
    pub fn add_hierarchy(&mut self, id: u32, name: &str, parent: u32) {
        self.hierarchy.push((id, name.to_string(), parent));
    }

    /// Append one draw call: a single color slice of a part, merged into its
    /// color mesh under `id`. Each `id` lands in exactly one color mesh — the
    /// caller mints a fresh id per color slice (see [`crate::merge`]), so a
    /// draw-range id is never shared across color meshes.
    pub fn add_bucket(&mut self, id: u32, color: Option<[f32; 4]>, m: &TriMesh) {
        // wireframe (line) geometry has no place in the merged triangle layout
        if m.is_empty() || m.lines {
            return;
        }
        let key = color.map(quantize_color);
        let bi = self
            .buckets
            .iter()
            .position(|b| b.color.map(quantize_color) == key)
            .unwrap_or_else(|| {
                self.buckets.push(MergedBucket {
                    color,
                    mesh: TriMesh::default(),
                    ranges: Vec::new(),
                });
                self.buckets.len() - 1
            });
        let b = &mut self.buckets[bi];
        let start = b.mesh.indices.len() as u32;
        b.mesh.append(m);
        b.ranges.push((id, start, m.indices.len() as u32));
    }

    pub fn bucket_count(&self) -> usize {
        self.buckets.len()
    }

    /// Distinct draw-call ids that own geometry (one per part color slice).
    pub fn part_count(&self) -> usize {
        let mut ids = std::collections::HashSet::new();
        for b in &self.buckets {
            for (id, _, _) in &b.ranges {
                ids.insert(*id);
            }
        }
        ids.len()
    }

    pub fn total_vertices(&self) -> usize {
        self.buckets.iter().map(|b| b.mesh.vertex_count()).sum()
    }

    pub fn total_triangles(&self) -> usize {
        self.buckets.iter().map(|b| b.mesh.triangle_count()).sum()
    }

    /// Convenience all-in-RAM write (used by tests and callers that want bytes).
    pub fn write(&self, generator: &str) -> Vec<u8> {
        let mut out = MemSink::default();
        let mut tmp = MemTemp::default();
        self.write_stream(generator, &mut out, &mut tmp)
            .expect("in-memory GLB write is infallible");
        out.0
    }

    /// Stream the merged GLB to `out`, spilling the binary chunk through `tmp`.
    pub fn write_stream(
        &self,
        generator: &str,
        out: &mut dyn OutputHandle,
        tmp: &mut dyn TempHandle,
    ) -> std::io::Result<()> {
        let mut bin = Bin::new(tmp);
        let mut views = String::new();
        let mut accessors = String::new();
        let mut meshes_json = String::new();
        let mut materials_json = String::new();
        let mut nodes_json = String::new();
        let mut extras = String::new();
        let mut n_views = 0usize;
        let mut n_acc = 0usize;

        for (i, b) in self.buckets.iter().enumerate() {
            let (pos_acc, nrm_acc, idx_acc) = write_primitive(
                &b.mesh,
                &mut bin,
                &mut views,
                &mut accessors,
                &mut n_views,
                &mut n_acc,
            );
            if i > 0 {
                meshes_json.push(',');
                materials_json.push(',');
                nodes_json.push(',');
            }
            let c = b.color.unwrap_or([0.72, 0.72, 0.75, 1.0]);
            let alpha_mode = if c[3] < 1.0 {
                ",\"alphaMode\":\"BLEND\""
            } else {
                ""
            };
            materials_json.push_str(&format!(
                "{{\"name\":\"color_{}\",\"pbrMetallicRoughness\":{{\
                 \"baseColorFactor\":[{},{},{},{}],\
                 \"metallicFactor\":0.1,\"roughnessFactor\":0.7}}{},\
                 \"doubleSided\":true}}",
                i,
                fmt_f32(c[0]),
                fmt_f32(c[1]),
                fmt_f32(c[2]),
                fmt_f32(c[3]),
                alpha_mode,
            ));
            meshes_json.push_str(&format!(
                "{{\"name\":\"node{}\",\"primitives\":[{{\
                 \"attributes\":{{{}}},\
                 \"indices\":{},\"mode\":4,\"material\":{}}}]}}",
                i,
                attributes_json(pos_acc, nrm_acc),
                idx_acc,
                i
            ));
            nodes_json.push_str(&format!("{{\"name\":\"node{}\",\"mesh\":{}}}", i, i));

            let mut rec = String::new();
            for (id, start, count) in &b.ranges {
                if !rec.is_empty() {
                    rec.push(',');
                }
                rec.push_str(&format!("\"{}\":[{},{}]", id, start, count));
            }
            extras.push_str(&format!("\"draw_ranges_node{}\":{{{}}},", i, rec));
        }

        let mut id_h = String::new();
        for (id, name, parent) in &self.hierarchy {
            if !id_h.is_empty() {
                id_h.push(',');
            }
            let parent = match parent {
                0 => "\"*\"".to_string(),
                p => format!("\"{}\"", p),
            };
            id_h.push_str(&format!("\"{}\":[{},{}]", id, json_str(name), parent));
        }
        extras.push_str(&format!("\"id_hierarchy\":{{{}}}", id_h));

        let scene_nodes = (0..self.buckets.len())
            .map(|n| n.to_string())
            .collect::<Vec<_>>()
            .join(",");

        bin.align(4);
        let blen = bin.len();
        let json = format!(
            concat!(
                "{{\"asset\":{{\"version\":\"2.0\",\"generator\":{gen},",
                "\"extras\":{{\"web3dversion\":2}}}},",
                "\"scene\":0,\"scenes\":[{{\"nodes\":[{scene}],\"extras\":{{{extras}}}}}],",
                "\"nodes\":[{nodes}],",
                "\"meshes\":[{meshes}],",
                "\"materials\":[{materials}],",
                "\"accessors\":[{acc}],",
                "\"bufferViews\":[{views}],",
                "\"buffers\":[{{\"byteLength\":{blen}}}]}}"
            ),
            gen = json_str(generator),
            scene = scene_nodes,
            extras = extras,
            nodes = nodes_json,
            meshes = meshes_json,
            materials = materials_json,
            acc = accessors,
            views = views,
            blen = blen,
        );
        finish_glb(out, json, tmp)
    }
}

fn write_primitive(
    m: &TriMesh,
    bin: &mut Bin,
    views: &mut String,
    accessors: &mut String,
    n_views: &mut usize,
    n_acc: &mut usize,
) -> (usize, Option<usize>, usize) {
    let vcount = m.vertex_count();

    bin.align(4);
    let pos_off = bin.append(as_bytes(&m.positions));
    let (mn, mx) = m.bounds();
    push_view(views, n_views, pos_off, m.positions.len() * 4, Some(34962));
    let pos_acc = push_accessor(
        accessors,
        n_acc,
        *n_views - 1,
        5126,
        vcount,
        "VEC3",
        Some((mn, mx)),
    );

    // normals are absent after the rvm-style position cleanup pass
    let nrm_acc = if m.normals.is_empty() {
        None
    } else {
        bin.align(4);
        let nrm_off = bin.append(as_bytes(&m.normals));
        push_view(views, n_views, nrm_off, m.normals.len() * 4, Some(34962));
        Some(push_accessor(
            accessors,
            n_acc,
            *n_views - 1,
            5126,
            vcount,
            "VEC3",
            None,
        ))
    };

    bin.align(4);
    let idx_off = bin.append(as_bytes_u32(&m.indices));
    push_view(views, n_views, idx_off, m.indices.len() * 4, Some(34963));
    let idx_acc = push_accessor(
        accessors,
        n_acc,
        *n_views - 1,
        5125,
        m.indices.len(),
        "SCALAR",
        None,
    );

    (pos_acc, nrm_acc, idx_acc)
}

fn attributes_json(pos_acc: usize, nrm_acc: Option<usize>) -> String {
    match nrm_acc {
        Some(n) => format!("\"POSITION\":{},\"NORMAL\":{}", pos_acc, n),
        None => format!("\"POSITION\":{}", pos_acc),
    }
}

fn push_view(views: &mut String, n: &mut usize, offset: usize, len: usize, target: Option<u32>) {
    if !views.is_empty() {
        views.push(',');
    }
    views.push_str(&format!(
        "{{\"buffer\":0,\"byteOffset\":{},\"byteLength\":{}",
        offset, len
    ));
    if let Some(t) = target {
        views.push_str(&format!(",\"target\":{}", t));
    }
    views.push('}');
    *n += 1;
}

fn push_accessor(
    acc: &mut String,
    n: &mut usize,
    view: usize,
    comp: u32,
    count: usize,
    ty: &str,
    minmax: Option<([f32; 3], [f32; 3])>,
) -> usize {
    if !acc.is_empty() {
        acc.push(',');
    }
    acc.push_str(&format!(
        "{{\"bufferView\":{},\"componentType\":{},\"count\":{},\"type\":\"{}\"",
        view, comp, count, ty
    ));
    if let Some((mn, mx)) = minmax {
        acc.push_str(&format!(
            ",\"min\":[{},{},{}],\"max\":[{},{},{}]",
            fmt_f32(mn[0]),
            fmt_f32(mn[1]),
            fmt_f32(mn[2]),
            fmt_f32(mx[0]),
            fmt_f32(mx[1]),
            fmt_f32(mx[2])
        ));
    }
    acc.push('}');
    let id = *n;
    *n += 1;
    id
}

fn fmt_f32(v: f32) -> String {
    if v.is_finite() {
        let s = format!("{}", v);
        if s.contains('.') || s.contains('e') {
            s
        } else {
            format!("{}.0", s)
        }
    } else {
        "0.0".into()
    }
}

fn fmt_f64(v: f64) -> String {
    if v.is_finite() {
        format!("{}", v)
    } else {
        "0".into()
    }
}

/// JSON string escaping for entity names.
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::v3;
    use crate::mesh::TriMesh;

    fn tri_mesh() -> TriMesh {
        let mut m = TriMesh::default();
        let n = v3(0.0, 0.0, 1.0);
        m.push_vertex(v3(0.0, 0.0, 0.0), n);
        m.push_vertex(v3(1.0, 0.0, 0.0), n);
        m.push_vertex(v3(0.0, 1.0, 0.0), n);
        m.indices.extend_from_slice(&[0, 1, 2]);
        m
    }

    fn scene() -> GlbBuilder {
        let mut set = MeshSet::default();
        set.bucket(None).append(&tri_mesh());
        set.bucket(Some([1.0, 0.0, 0.0, 1.0])).append(&tri_mesh());
        set.bucket(Some([1.0, 0.0, 0.0, 1.0])).append(&tri_mesh()); // same bucket

        let mut b = GlbBuilder::default();
        let mi = b.add_mesh(set, "tri \"quoted\" name".into());
        let child = b.add_node("child".into(), Some(M4::scale_uniform(2.0)), Some(mi));
        let root = b.add_node("root".into(), None, None);
        b.nodes[root].children = vec![child];
        b.root_nodes = vec![root];
        b
    }

    fn build() -> Vec<u8> {
        scene().write("test")
    }

    #[test]
    fn streamed_output_through_temp_spill_is_byte_identical() {
        // the streaming writer routes the binary chunk through a TempHandle (a
        // file under --memory-threshold); a MemTemp spill must reproduce the
        // exact same GLB as the all-in-RAM write().
        let b = scene();
        let in_ram = b.write("test");
        let mut out = MemSink::default();
        let mut tmp = MemTemp::default();
        b.write_stream("test", &mut out, &mut tmp).unwrap();
        assert_eq!(out.0, in_ram, "streamed bytes must equal the in-RAM write");
        assert!(
            tmp.len() > 0,
            "binary chunk was routed through the temp spill"
        );
    }

    #[test]
    fn glb_container_layout_is_valid() {
        let g = build();
        assert_eq!(&g[0..4], b"glTF");
        assert_eq!(u32::from_le_bytes(g[4..8].try_into().unwrap()), 2);
        let total = u32::from_le_bytes(g[8..12].try_into().unwrap()) as usize;
        assert_eq!(total, g.len());
        let jlen = u32::from_le_bytes(g[12..16].try_into().unwrap()) as usize;
        assert_eq!(&g[16..20], b"JSON");
        assert_eq!(jlen % 4, 0, "JSON chunk must be 4-byte aligned");
        let bin_hdr = 20 + jlen;
        let blen = u32::from_le_bytes(g[bin_hdr..bin_hdr + 4].try_into().unwrap()) as usize;
        assert_eq!(&g[bin_hdr + 4..bin_hdr + 8], b"BIN\0");
        assert_eq!(bin_hdr + 8 + blen, g.len());
    }

    #[test]
    fn glb_json_materials_and_primitives() {
        let g = build();
        let jlen = u32::from_le_bytes(g[12..16].try_into().unwrap()) as usize;
        let json: serde_json::Value =
            serde_json::from_slice(&g[20..20 + jlen]).expect("valid JSON");
        assert_eq!(json["asset"]["version"], "2.0");
        assert_eq!(json["nodes"].as_array().unwrap().len(), 2);
        assert_eq!(json["meshes"][0]["name"], "tri \"quoted\" name");

        // two primitives: uncolored (material 0) + red (material 1)
        let prims = json["meshes"][0]["primitives"].as_array().unwrap();
        assert_eq!(prims.len(), 2);
        assert_eq!(prims[0]["material"], 0);
        assert_eq!(prims[1]["material"], 1);
        let mats = json["materials"].as_array().unwrap();
        assert_eq!(mats.len(), 2, "red bucket dedupes to one material");
        assert_eq!(mats[1]["pbrMetallicRoughness"]["baseColorFactor"][0], 1.0);

        // POSITION accessor carries min/max; identity matrices omitted
        assert!(json["accessors"][0]["min"].is_array());
        assert!(json["nodes"][1].get("matrix").is_none());
        assert_eq!(json["nodes"][0]["matrix"][0], 2.0);
    }

    #[test]
    fn merged_builder_one_id_per_drawcall_and_id_hierarchy() {
        let mut mb = MergedBuilder::default();
        mb.add_hierarchy(1, "root", 0);
        // element "a" (id 2): red slice keeps id 2; its uncolored slice is its
        // own numbered child node (id 4)
        mb.add_hierarchy(2, "a", 1);
        mb.add_bucket(2, Some([1.0, 0.0, 0.0, 1.0]), &tri_mesh());
        mb.add_hierarchy(4, "a", 2);
        mb.add_bucket(4, None, &tri_mesh());
        // element "b" (id 3): red slice keeps id 3; its translucent green slice
        // is its own numbered child node (id 5)
        mb.add_hierarchy(3, "b", 1);
        mb.add_bucket(3, Some([1.0, 0.0, 0.0, 1.0]), &tri_mesh());
        mb.add_hierarchy(5, "b", 3);
        mb.add_bucket(5, Some([0.0, 1.0, 0.0, 0.5]), &tri_mesh());

        assert_eq!(mb.bucket_count(), 3, "red, uncolored, green");
        assert_eq!(mb.part_count(), 4, "four draw calls -> four ids");

        let g = mb.write("test");
        let jlen = u32::from_le_bytes(g[12..16].try_into().unwrap()) as usize;
        let json: serde_json::Value =
            serde_json::from_slice(&g[20..20 + jlen]).expect("valid JSON");

        // one node + mesh + material per color, node<N> -> mesh N -> material N
        for k in ["nodes", "meshes", "materials"] {
            assert_eq!(json[k].as_array().unwrap().len(), 3, "{}", k);
        }
        for i in 0..3 {
            assert_eq!(json["nodes"][i]["name"], format!("node{}", i));
            assert_eq!(json["nodes"][i]["mesh"], i);
            assert_eq!(json["meshes"][i]["primitives"][0]["material"], i);
        }
        assert_eq!(
            json["materials"][0]["pbrMetallicRoughness"]["baseColorFactor"][0],
            1.0
        );
        assert!(json["materials"][0].get("alphaMode").is_none());
        assert_eq!(json["materials"][2]["alphaMode"], "BLEND");

        // every draw call is one id in exactly one color mesh (never shared)
        let extras = &json["scenes"][0]["extras"];
        assert_eq!(extras["draw_ranges_node0"]["2"], serde_json::json!([0, 3]));
        assert_eq!(extras["draw_ranges_node0"]["3"], serde_json::json!([3, 3]));
        assert_eq!(extras["draw_ranges_node1"]["4"], serde_json::json!([0, 3]));
        assert_eq!(extras["draw_ranges_node2"]["5"], serde_json::json!([0, 3]));
        let mut seen = std::collections::HashSet::new();
        for i in 0..3 {
            for k in extras[&format!("draw_ranges_node{}", i)]
                .as_object()
                .unwrap()
                .keys()
            {
                assert!(seen.insert(k.clone()), "id {k} appears in >1 mesh");
            }
        }

        // id_hierarchy: [name, parent]; the extra-color slices are child nodes
        assert_eq!(
            extras["id_hierarchy"]["1"],
            serde_json::json!(["root", "*"])
        );
        assert_eq!(extras["id_hierarchy"]["2"], serde_json::json!(["a", "1"]));
        assert_eq!(extras["id_hierarchy"]["4"], serde_json::json!(["a", "2"]));
        assert_eq!(extras["id_hierarchy"]["5"], serde_json::json!(["b", "3"]));

        assert_eq!(json["asset"]["extras"]["web3dversion"], 2);
    }
}
