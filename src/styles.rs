//! Presentation styles: extract per-item colors from STYLED_ITEM chains.
//!
//! The STEP presentation model is a deep chain
//! (`STYLED_ITEM -> PRESENTATION_STYLE_ASSIGNMENT -> SURFACE_STYLE_USAGE ->
//! SURFACE_SIDE_STYLE -> SURFACE_STYLE_FILL_AREA -> FILL_AREA_STYLE ->
//! FILL_AREA_STYLE_COLOUR -> COLOUR_RGB`) with several exporter-specific
//! variations (curve styles, rendering styles, pre-defined colour names,
//! OVER_RIDING_STYLED_ITEM). Rather than enumerating every path, we walk the
//! reference graph breadth-first from each styled item until a colour is
//! found.

use std::collections::{HashMap, HashSet, VecDeque};

use crate::step::{StepFile, P};

/// item entity id (solid, shell, face, ...) -> RGBA
pub type ColorMap = HashMap<u32, [f32; 4]>;

pub fn build_color_map(sf: &StepFile) -> ColorMap {
    let mut map = ColorMap::new();
    // plain styled items first, overriding ones second so they win
    for ty in ["STYLED_ITEM", "OVER_RIDING_STYLED_ITEM"] {
        for &si in sf.of_type(ty) {
            let p = match sf.params(si) {
                Some(p) => p,
                None => continue,
            };
            // STYLED_ITEM('name', (styles), item) and
            // OVER_RIDING_STYLED_ITEM('name', (styles), item, over_ridden):
            // the styled item is parameter 2 in both forms.
            let item = p.get(2).and_then(|v| v.as_ref_id());
            let styles: Vec<u32> = p
                .iter()
                .filter_map(|v| v.as_list())
                .flat_map(|l| l.iter().filter_map(|v| v.as_ref_id()))
                .collect();
            let (item, styles) = match (item, styles.is_empty()) {
                (Some(i), false) => (i, styles),
                _ => continue,
            };
            if let Some(rgba) = find_color(sf, &styles) {
                map.insert(item, rgba);
            }
        }
    }
    map
}

/// BFS through entity references until a COLOUR_RGB or pre-defined colour
/// name is reached.
fn find_color(sf: &StepFile, roots: &[u32]) -> Option<[f32; 4]> {
    let mut queue: VecDeque<(u32, usize)> = roots.iter().map(|&r| (r, 0)).collect();
    let mut seen: HashSet<u32> = roots.iter().copied().collect();
    while let Some((id, depth)) = queue.pop_front() {
        if depth > 10 {
            continue;
        }
        match sf.entity_type(id) {
            Some("COLOUR_RGB") => {
                let p = sf.params(id)?;
                let nums: Vec<f64> = p.iter().filter_map(|v| v.as_f64()).collect();
                if nums.len() >= 3 {
                    return Some([nums[0] as f32, nums[1] as f32, nums[2] as f32, 1.0]);
                }
            }
            Some("DRAUGHTING_PRE_DEFINED_COLOUR") | Some("PRE_DEFINED_COLOUR") => {
                let p = sf.params(id)?;
                if let Some(name) = p.iter().find_map(|v| v.as_str()) {
                    if let Some(c) = named_color(name) {
                        return Some(c);
                    }
                }
            }
            Some(_) => {
                if let Some(p) = sf.params(id) {
                    enqueue_refs(&p, depth, &mut queue, &mut seen);
                }
            }
            None => {}
        }
    }
    None
}

fn enqueue_refs(
    params: &[P],
    depth: usize,
    queue: &mut VecDeque<(u32, usize)>,
    seen: &mut HashSet<u32>,
) {
    for p in params {
        match p {
            P::Ref(r) => {
                if seen.insert(*r) {
                    queue.push_back((*r, depth + 1));
                }
            }
            P::L(l) => enqueue_refs(l, depth, queue, seen),
            P::Typed(_, l) => enqueue_refs(l, depth, queue, seen),
            _ => {}
        }
    }
}

pub fn named_color(name: &str) -> Option<[f32; 4]> {
    let c = match name.to_ascii_lowercase().as_str() {
        "red" => [1.0, 0.0, 0.0, 1.0],
        "green" => [0.0, 1.0, 0.0, 1.0],
        "blue" => [0.0, 0.0, 1.0, 1.0],
        "yellow" => [1.0, 1.0, 0.0, 1.0],
        "magenta" => [1.0, 0.0, 1.0, 1.0],
        "cyan" => [0.0, 1.0, 1.0, 1.0],
        "black" => [0.0, 0.0, 0.0, 1.0],
        "white" => [1.0, 1.0, 1.0, 1.0],
        "orange" => [1.0, 0.5, 0.0, 1.0],
        "grey" | "gray" => [0.5, 0.5, 0.5, 1.0],
        _ => return None,
    };
    Some(c)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(src: &str) -> StepFile {
        StepFile::parse(src.as_bytes().to_vec()).expect("parse")
    }

    const STYLED: &str = "DATA;
#1=MANIFOLD_SOLID_BREP('part',#99);
#2=COLOUR_RGB('',0.9,0.1,0.2);
#3=FILL_AREA_STYLE_COLOUR('',#2);
#4=FILL_AREA_STYLE('',(#3));
#5=SURFACE_STYLE_FILL_AREA(#4);
#6=SURFACE_SIDE_STYLE('',(#5));
#7=SURFACE_STYLE_USAGE(.BOTH.,#6);
#8=PRESENTATION_STYLE_ASSIGNMENT((#7));
#9=STYLED_ITEM('color',(#8),#1);
#10=ADVANCED_FACE('f',(),#98,.T.);
#11=DRAUGHTING_PRE_DEFINED_COLOUR('green');
#12=CURVE_STYLE('',#97,POSITIVE_LENGTH_MEASURE(0.1),#11);
#13=PRESENTATION_STYLE_ASSIGNMENT((#12));
#14=OVER_RIDING_STYLED_ITEM('o',(#13),#10,#9);
ENDSEC;";

    #[test]
    fn full_surface_style_chain_resolves_to_rgb() {
        let sf = parse(STYLED);
        let map = build_color_map(&sf);
        let c = map.get(&1).expect("solid #1 colored");
        assert!((c[0] - 0.9).abs() < 1e-6 && (c[1] - 0.1).abs() < 1e-6);
    }

    #[test]
    fn overriding_item_with_named_color() {
        let sf = parse(STYLED);
        let map = build_color_map(&sf);
        let c = map.get(&10).expect("face #10 colored");
        assert_eq!(*c, [0.0, 1.0, 0.0, 1.0]); // 'green'
    }

    #[test]
    fn unstyled_items_have_no_entry() {
        let sf = parse(
            "DATA;
#1=MANIFOLD_SOLID_BREP('p',#2);
ENDSEC;",
        );
        assert!(build_color_map(&sf).is_empty());
    }
}
