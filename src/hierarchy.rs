//! Product structure: PRODUCT / PRODUCT_DEFINITION graph,
//! NEXT_ASSEMBLY_USAGE_OCCURRENCE parent/child edges, and per-instance
//! transforms from CONTEXT_DEPENDENT_SHAPE_REPRESENTATION +
//! REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION.

use std::collections::{HashMap, HashSet};

use crate::geom::M4;
use crate::model;
use crate::step::{StepFile, P};

pub struct ProductNode {
    pub pd: u32,
    pub name: String,
    /// SHAPE_REPRESENTATIONs attached to this product definition (directly and
    /// via SHAPE_REPRESENTATION_RELATIONSHIP).
    pub shape_reps: Vec<u32>,
}

pub struct Instance {
    pub nauo: u32,
    pub child_pd: u32,
    pub name: String,
    pub transform: M4,
}

#[derive(Default)]
pub struct Assembly {
    pub products: HashMap<u32, ProductNode>,
    /// parent PD -> child instances
    pub children: HashMap<u32, Vec<Instance>>,
    pub roots: Vec<u32>,
}

pub fn build(sf: &StepFile) -> Assembly {
    let mut asm = Assembly::default();

    // --- products ----------------------------------------------------------
    for &pd in sf.of_type("PRODUCT_DEFINITION") {
        let name = product_name(sf, pd).unwrap_or_else(|| format!("PD#{}", pd));
        asm.products.insert(
            pd,
            ProductNode {
                pd,
                name,
                shape_reps: Vec::new(),
            },
        );
    }

    // --- shapes: PD -> PDS -> SDR -> SR ------------------------------------
    // PRODUCT_DEFINITION_SHAPE('', '', definition)
    let mut pds_of_pd: HashMap<u32, u32> = HashMap::new(); // PDS id -> PD id
    let mut pds_of_nauo: HashMap<u32, u32> = HashMap::new(); // PDS id -> NAUO id
    for &pds in sf.of_type("PRODUCT_DEFINITION_SHAPE") {
        if let Some(p) = sf.params(pds) {
            if let Some(def) = p.get(2).and_then(|v| v.as_ref_id()) {
                match sf.entity_type(def) {
                    Some("PRODUCT_DEFINITION") => {
                        pds_of_pd.insert(pds, def);
                    }
                    Some("NEXT_ASSEMBLY_USAGE_OCCURRENCE") => {
                        pds_of_nauo.insert(pds, def);
                    }
                    _ => {}
                }
            }
        }
    }

    // SHAPE_DEFINITION_REPRESENTATION(definition PDS, used_representation SR)
    let mut sr_of_pd: HashMap<u32, Vec<u32>> = HashMap::new();
    for &sdr in sf.of_type("SHAPE_DEFINITION_REPRESENTATION") {
        if let Some(p) = sf.params(sdr) {
            let pds = p.first().and_then(|v| v.as_ref_id());
            let sr = p.get(1).and_then(|v| v.as_ref_id());
            if let (Some(pds), Some(sr)) = (pds, sr) {
                if let Some(&pd) = pds_of_pd.get(&pds) {
                    sr_of_pd.entry(pd).or_default().push(sr);
                }
            }
        }
    }

    // SHAPE_REPRESENTATION_RELATIONSHIP links SR <-> ADVANCED_BREP_SHAPE_REP etc.
    let mut srr: HashMap<u32, Vec<u32>> = HashMap::new();
    for &r in sf.of_type("SHAPE_REPRESENTATION_RELATIONSHIP") {
        if let Some(p) = sf.params(r) {
            let a = p.get(2).and_then(|v| v.as_ref_id());
            let b = p.get(3).and_then(|v| v.as_ref_id());
            if let (Some(a), Some(b)) = (a, b) {
                srr.entry(a).or_default().push(b);
                srr.entry(b).or_default().push(a);
            }
        }
    }

    for (pd, srs) in &sr_of_pd {
        if let Some(node) = asm.products.get_mut(pd) {
            let mut seen = HashSet::new();
            for &sr in srs {
                if seen.insert(sr) {
                    node.shape_reps.push(sr);
                }
                if let Some(linked) = srr.get(&sr) {
                    for &l in linked {
                        if seen.insert(l) {
                            node.shape_reps.push(l);
                        }
                    }
                }
            }
        }
    }

    // --- transforms per NAUO ------------------------------------------------
    // CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(rep_relation, represented_product_relation)
    let mut xform_of_nauo: HashMap<u32, M4> = HashMap::new();
    for &cdsr in sf.of_type("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION") {
        let p = match sf.params(cdsr) {
            Some(p) => p,
            None => continue,
        };
        let rel = p.first().and_then(|v| v.as_ref_id());
        let pds = p.get(1).and_then(|v| v.as_ref_id());
        let (rel, pds) = match (rel, pds) {
            (Some(a), Some(b)) => (a, b),
            _ => continue,
        };
        let nauo = match pds_of_nauo.get(&pds) {
            Some(&n) => n,
            None => continue,
        };
        if let Some((rep1, rep2, m1, m2)) = transform_relationship(sf, rel) {
            // Which rep belongs to the child product of this NAUO?
            let child_pd = nauo_pds(sf, nauo).1;
            let child_reps: HashSet<u32> = child_pd
                .and_then(|pd| asm.products.get(&pd))
                .map(|n| n.shape_reps.iter().copied().collect())
                .unwrap_or_default();
            // The IDT maps axis1 (in rep_1 space) onto axis2 (in rep_2 space):
            // T(rep_1 -> rep_2) = M(axis2) * M(axis1)^-1.
            let fwd = m2.mul(m1.inverse_rigid());
            let t = if child_reps.contains(&rep2) && !child_reps.contains(&rep1) {
                // child geometry lives in rep_2 -> we need rep_2 -> rep_1
                fwd.inverse_rigid()
            } else {
                fwd
            };
            xform_of_nauo.insert(nauo, t);
        }
    }

    // --- assembly edges -----------------------------------------------------
    let mut is_child: HashSet<u32> = HashSet::new();
    for &nauo in sf.of_type("NEXT_ASSEMBLY_USAGE_OCCURRENCE") {
        let (parent, child) = nauo_pds(sf, nauo);
        let (parent, child) = match (parent, child) {
            (Some(a), Some(b)) => (a, b),
            _ => continue,
        };
        let p = sf.params(nauo).unwrap_or_default();
        // Instance label: the child product's name reads best in viewers;
        // fall back to the NAUO's reference designator / name.
        let name = asm
            .products
            .get(&child)
            .map(|n| n.name.clone())
            .or_else(|| {
                p.get(5)
                    .and_then(|v| v.as_str())
                    .filter(|s| !s.is_empty())
                    .map(|s| s.to_string())
            })
            .or_else(|| {
                p.get(1)
                    .and_then(|v| v.as_str())
                    .filter(|s| !s.is_empty())
                    .map(|s| s.to_string())
            })
            .unwrap_or_else(|| format!("NAUO#{}", nauo));
        let transform = xform_of_nauo.get(&nauo).copied().unwrap_or(M4::IDENTITY);
        is_child.insert(child);
        asm.children.entry(parent).or_default().push(Instance {
            nauo,
            child_pd: child,
            name,
            transform,
        });
    }

    // --- roots ---------------------------------------------------------------
    let mut roots: Vec<u32> = asm
        .products
        .keys()
        .copied()
        .filter(|pd| !is_child.contains(pd))
        .filter(|pd| {
            // only keep roots that own geometry somewhere below them
            subtree_has_shape(&asm, *pd, 0)
        })
        .collect();
    roots.sort_unstable();
    asm.roots = roots;
    asm
}

fn subtree_has_shape(asm: &Assembly, pd: u32, depth: usize) -> bool {
    if depth > 64 {
        return false;
    }
    if asm
        .products
        .get(&pd)
        .map(|n| !n.shape_reps.is_empty())
        .unwrap_or(false)
    {
        return true;
    }
    asm.children
        .get(&pd)
        .map(|kids| {
            kids.iter()
                .any(|k| subtree_has_shape(asm, k.child_pd, depth + 1))
        })
        .unwrap_or(false)
}

/// NAUO('id','name','desc', relating PD, related PD, ref_designator)
fn nauo_pds(sf: &StepFile, nauo: u32) -> (Option<u32>, Option<u32>) {
    match sf.params(nauo) {
        Some(p) => (
            p.get(3).and_then(|v| v.as_ref_id()),
            p.get(4).and_then(|v| v.as_ref_id()),
        ),
        None => (None, None),
    }
}

/// Resolve a (possibly complex) representation relationship with transform.
/// Returns (rep_1, rep_2, M(axis1), M(axis2)).
fn transform_relationship(sf: &StepFile, rel: u32) -> Option<(u32, u32, M4, M4)> {
    let (reps, idt) = if sf.is_complex(rel) {
        let rr = sf.complex_leaf(rel, "REPRESENTATION_RELATIONSHIP")?;
        // could match REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION leaf too;
        // pick the leaf that actually has the two rep refs
        let reps = extract_two_refs(&rr).or_else(|| {
            let alt = sf.complex_leaf(rel, "SHAPE_REPRESENTATION_RELATIONSHIP")?;
            extract_two_refs(&alt)
        })?;
        let with_t = sf.complex_leaf(rel, "WITH_TRANSFORMATION")?;
        let idt = with_t.iter().find_map(|v| v.as_ref_id())?;
        (reps, idt)
    } else {
        // simple REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION
        // ('name','desc', rep_1, rep_2, transformation)
        let p = sf.params(rel)?;
        let r1 = p.get(2).and_then(|v| v.as_ref_id())?;
        let r2 = p.get(3).and_then(|v| v.as_ref_id())?;
        let idt = p.get(4).and_then(|v| v.as_ref_id())?;
        ((r1, r2), idt)
    };

    // ITEM_DEFINED_TRANSFORMATION('name','desc', axis1, axis2)
    let it = sf.params(idt)?;
    let a1 = it.get(2).and_then(|v| v.as_ref_id());
    let a2 = it.get(3).and_then(|v| v.as_ref_id());
    let m1 = a1
        .map(|a| model::axis2_matrix(sf, a))
        .unwrap_or(M4::IDENTITY);
    let m2 = a2
        .map(|a| model::axis2_matrix(sf, a))
        .unwrap_or(M4::IDENTITY);
    Some((reps.0, reps.1, m1, m2))
}

fn extract_two_refs(p: &[P]) -> Option<(u32, u32)> {
    let refs: Vec<u32> = p.iter().filter_map(|v| v.as_ref_id()).collect();
    if refs.len() >= 2 {
        Some((refs[0], refs[1]))
    } else {
        None
    }
}

fn product_name(sf: &StepFile, pd: u32) -> Option<String> {
    // PRODUCT_DEFINITION(id, desc, formation, frame)
    let p = sf.params(pd)?;
    let pdf = p.get(2).and_then(|v| v.as_ref_id())?;
    // PRODUCT_DEFINITION_FORMATION[_*](id, desc, product)
    let f = sf.params(pdf)?;
    let product = f.get(2).and_then(|v| v.as_ref_id())?;
    // PRODUCT(id, name, desc, contexts)
    let pr = sf.params(product)?;
    pr.get(1)
        .and_then(|v| v.as_str())
        .filter(|s| !s.is_empty())
        .map(|s| s.to_string())
        .or_else(|| pr.first().and_then(|v| v.as_str()).map(|s| s.to_string()))
}

/// Pretty-print the assembly tree to stdout.
pub fn print_tree(asm: &Assembly) {
    fn rec(asm: &Assembly, pd: u32, name: &str, prefix: &str, last: bool, depth: usize) {
        if depth > 64 {
            return;
        }
        let node = asm.products.get(&pd);
        let geo = node
            .map(|n| {
                if n.shape_reps.is_empty() {
                    ""
                } else {
                    "  [geometry]"
                }
            })
            .unwrap_or("");
        let (tee, pad) = if depth == 0 {
            ("", "")
        } else if last {
            ("└─ ", "   ")
        } else {
            ("├─ ", "│  ")
        };
        println!("{}{}{}{}", prefix, tee, name, geo);
        if let Some(kids) = asm.children.get(&pd) {
            let n = kids.len();
            for (i, k) in kids.iter().enumerate() {
                rec(
                    asm,
                    k.child_pd,
                    &k.name,
                    &format!("{}{}", prefix, pad),
                    i + 1 == n,
                    depth + 1,
                );
            }
        }
    }
    for &r in &asm.roots {
        let name = asm
            .products
            .get(&r)
            .map(|n| n.name.clone())
            .unwrap_or_else(|| format!("PD#{}", r));
        rec(asm, r, &name, "", true, 0);
    }
}
