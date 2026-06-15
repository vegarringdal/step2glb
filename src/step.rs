//! Low-memory ISO 10303-21 (STEP Part 21) reader.
//!
//! Strategy (similar in spirit to rvm_parser_glb): the file is held once as a
//! byte buffer and a single pass builds a compact index of
//! `#id -> (interned type, byte range of the parameter list)`.
//! Parameters are parsed *lazily*, only for the entities the pipeline actually
//! touches, and the resulting `P` values are dropped right after use.
//! No DOM of the whole file is ever materialized.

use std::collections::HashMap;

/// One indexed entity instance. 16 bytes.
#[derive(Clone, Copy)]
pub struct EntityRec {
    /// Interned type id (see `StepFile::type_names`). For complex (multi-leaf)
    /// instances this is `TYPE_COMPLEX`.
    pub ty: u32,
    /// Byte range (in `StepFile::data`) of the parameter list, *excluding* the
    /// outer parentheses for simple records. For complex records it covers the
    /// whole `( LEAF1(...) LEAF2(...) ... )` body excluding outer parens.
    pub start: u32,
    pub end: u32,
}

pub const TYPE_COMPLEX: &str = "<COMPLEX>";

/// Lazily-parsed parameter value.
#[derive(Clone, Debug, PartialEq)]
pub enum P {
    Ref(u32),
    F(f64),
    I(i64),
    Str(String),
    Enum(String),
    L(Vec<P>),
    Typed(String, Vec<P>),
    Null, // `$`
    Star, // `*`
}

impl P {
    pub fn as_ref_id(&self) -> Option<u32> {
        match self {
            P::Ref(r) => Some(*r),
            _ => None,
        }
    }
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            P::F(v) => Some(*v),
            P::I(v) => Some(*v as f64),
            _ => None,
        }
    }
    pub fn as_i64(&self) -> Option<i64> {
        match self {
            P::I(v) => Some(*v),
            P::F(v) => Some(*v as i64),
            _ => None,
        }
    }
    pub fn as_str(&self) -> Option<&str> {
        match self {
            P::Str(s) => Some(s),
            _ => None,
        }
    }
    pub fn as_list(&self) -> Option<&[P]> {
        match self {
            P::L(l) => Some(l),
            _ => None,
        }
    }
    /// An enumeration value (`.DIFFERENCE.` → `"DIFFERENCE"`), already uppercased.
    pub fn as_enum(&self) -> Option<&str> {
        match self {
            P::Enum(e) => Some(e),
            _ => None,
        }
    }
    /// `.T.` / `.F.` enums as bool.
    pub fn as_bool(&self) -> Option<bool> {
        match self {
            P::Enum(e) if e == "T" => Some(true),
            P::Enum(e) if e == "F" => Some(false),
            _ => None,
        }
    }
}

pub struct StepFile {
    pub data: Vec<u8>,
    pub entities: HashMap<u32, EntityRec>,
    /// type id -> entity ids, for fast "find all NAUO" style queries.
    pub by_type: HashMap<u32, Vec<u32>>,
    pub type_names: Vec<String>,
    type_lookup: HashMap<String, u32>,
    pub header_range: (usize, usize),
    pub warnings: Vec<String>,
}

impl StepFile {
    pub fn parse(data: Vec<u8>) -> Result<StepFile, String> {
        let mut sf = StepFile {
            data,
            entities: HashMap::new(),
            by_type: HashMap::new(),
            type_names: Vec::new(),
            type_lookup: HashMap::new(),
            header_range: (0, 0),
            warnings: Vec::new(),
        };
        sf.index()?;
        Ok(sf)
    }

    pub fn type_id(&self, name: &str) -> Option<u32> {
        self.type_lookup.get(name).copied()
    }

    pub fn type_name(&self, id: u32) -> &str {
        &self.type_names[id as usize]
    }

    /// All entity ids of an exact type name (empty slice if none).
    pub fn of_type(&self, name: &str) -> &[u32] {
        static EMPTY: [u32; 0] = [];
        match self.type_id(name).and_then(|t| self.by_type.get(&t)) {
            Some(v) => v,
            None => &EMPTY,
        }
    }

    pub fn entity_type(&self, id: u32) -> Option<&str> {
        self.entities.get(&id).map(|e| self.type_name(e.ty))
    }

    pub fn is_complex(&self, id: u32) -> bool {
        self.entity_type(id) == Some(TYPE_COMPLEX)
    }

    fn intern(&mut self, name: &str) -> u32 {
        if let Some(&id) = self.type_lookup.get(name) {
            return id;
        }
        let id = self.type_names.len() as u32;
        self.type_names.push(name.to_string());
        self.type_lookup.insert(name.to_string(), id);
        id
    }

    // ---------------------------------------------------------------- index

    fn index(&mut self) -> Result<(), String> {
        let data = std::mem::take(&mut self.data);
        let n = data.len();
        let mut i;

        // Find DATA; section. Also remember HEADER for unit sniffing.
        let header_start = find_kw(&data, 0, b"HEADER;").unwrap_or(0);
        let data_start = find_kw(&data, 0, b"DATA;").ok_or("no DATA; section found")?;
        self.header_range = (header_start, data_start);
        i = data_start + 5;

        while i < n {
            i = skip_ws_comments(&data, i);
            if i >= n {
                break;
            }
            if data[i] != b'#' {
                // ENDSEC; / END-ISO-10303-21; or junk -> stop at ENDSEC
                if starts_with_ci(&data, i, b"ENDSEC") {
                    break;
                }
                i += 1;
                continue;
            }
            i += 1;
            let (id, ni) = parse_uint(&data, i);
            i = skip_ws_comments(&data, ni);
            if i >= n || data[i] != b'=' {
                // malformed; resync to next ';'
                while i < n && data[i] != b';' {
                    i += 1;
                }
                continue;
            }
            i = skip_ws_comments(&data, i + 1);

            let (ty, pstart, pend, after) = if i < n && data[i] == b'(' {
                // complex instance: #5=( LEAF(...) LEAF(...) );
                let close = match_paren(&data, i)?;
                (self.intern(TYPE_COMPLEX), i + 1, close, close + 1)
            } else {
                // simple: NAME(params);
                let ts = i;
                while i < n && is_ident(data[i]) {
                    i += 1;
                }
                let tname = ascii_upper(&data[ts..i]);
                let j = skip_ws_comments(&data, i);
                if j >= n || data[j] != b'(' {
                    while i < n && data[i] != b';' {
                        i += 1;
                    }
                    self.warnings
                        .push(format!("#{}: malformed record skipped", id));
                    continue;
                }
                let close = match_paren(&data, j)?;
                (self.intern(&tname), j + 1, close, close + 1)
            };

            self.entities.insert(
                id,
                EntityRec {
                    ty,
                    start: pstart as u32,
                    end: pend as u32,
                },
            );
            self.by_type.entry(ty).or_default().push(id);

            // skip to ';'
            i = after;
            while i < n && data[i] != b';' {
                i += 1;
            }
            i += 1;
        }

        self.data = data;
        Ok(())
    }

    // ------------------------------------------------------------ accessors

    /// Lazily parse the parameter list of a simple entity.
    pub fn params(&self, id: u32) -> Option<Vec<P>> {
        let rec = self.entities.get(&id)?;
        let slice = &self.data[rec.start as usize..rec.end as usize];
        Some(parse_param_list(slice))
    }

    /// For complex (multi-leaf) instances: return the params of the leaf with
    /// the given name, e.g. `B_SPLINE_SURFACE` inside a rational surface combo.
    /// `name_contains`: leaf type must contain this substring.
    pub fn complex_leaf(&self, id: u32, name_contains: &str) -> Option<Vec<P>> {
        let rec = self.entities.get(&id)?;
        if self.type_name(rec.ty) != TYPE_COMPLEX {
            return None;
        }
        let slice = &self.data[rec.start as usize..rec.end as usize];
        let mut i = 0usize;
        let n = slice.len();
        while i < n {
            i = skip_ws_comments(slice, i);
            if i >= n {
                break;
            }
            let ts = i;
            while i < n && is_ident(slice[i]) {
                i += 1;
            }
            if i == ts {
                i += 1;
                continue;
            }
            let tname = ascii_upper(&slice[ts..i]);
            let j = skip_ws_comments(slice, i);
            if j < n && slice[j] == b'(' {
                let close = match match_paren(slice, j) {
                    Ok(c) => c,
                    Err(_) => return None,
                };
                if tname.contains(name_contains) {
                    return Some(parse_param_list(&slice[j + 1..close]));
                }
                i = close + 1;
            }
        }
        None
    }

    /// True if entity (simple or complex) has / contains the given type name.
    /// Leaf type names of a complex instance (empty for simple entities).
    pub fn complex_leaf_names(&self, id: u32) -> Vec<String> {
        let mut out = Vec::new();
        let rec = match self.entities.get(&id) {
            Some(r) if self.type_name(r.ty) == TYPE_COMPLEX => *r,
            _ => return out,
        };
        let slice = &self.data[rec.start as usize..rec.end as usize];
        let mut i = 0usize;
        let n = slice.len();
        while i < n {
            i = skip_ws_comments(slice, i);
            if i >= n {
                break;
            }
            let ts = i;
            while i < n && is_ident(slice[i]) {
                i += 1;
            }
            if i == ts {
                i += 1;
                continue;
            }
            let tname = ascii_upper(&slice[ts..i]);
            let j = skip_ws_comments(slice, i);
            if j < n && slice[j] == b'(' {
                match match_paren(slice, j) {
                    Ok(close) => {
                        out.push(tname);
                        i = close + 1;
                    }
                    Err(_) => break,
                }
            }
        }
        out
    }

    #[allow(dead_code)]
    pub fn has_type(&self, id: u32, name: &str) -> bool {
        match self.entities.get(&id) {
            None => false,
            Some(rec) => {
                let tn = self.type_name(rec.ty);
                if tn == name {
                    return true;
                }
                if tn == TYPE_COMPLEX {
                    self.complex_leaf(id, name).is_some()
                } else {
                    false
                }
            }
        }
    }

    /// Raw bytes of the HEADER section (for unit sniffing etc.)
    #[allow(dead_code)]
    pub fn header(&self) -> &[u8] {
        &self.data[self.header_range.0..self.header_range.1]
    }

    /// Reconstruct the Part-21 source line for one entity from its indexed
    /// byte range: `#id=TYPE(params);`, or `#id=(LEAF1(..) LEAF2(..));` for a
    /// complex instance. Used by `--debug-print` to re-emit failing faces.
    pub fn entity_source(&self, id: u32) -> Option<String> {
        let rec = self.entities.get(&id)?;
        let body = std::str::from_utf8(&self.data[rec.start as usize..rec.end as usize])
            .ok()?
            .trim();
        let ty = self.entity_type(id)?;
        Some(if ty == TYPE_COMPLEX {
            format!("#{}=({});", id, body)
        } else {
            format!("#{}={}({});", id, ty, body)
        })
    }

    /// Every `#id` reference inside an entity's parameter bytes. Scans for the
    /// `#<digits>` token, so it is robust across simple, typed and complex
    /// records without re-parsing the value tree.
    pub fn entity_refs(&self, id: u32) -> Vec<u32> {
        let rec = match self.entities.get(&id) {
            Some(r) => r,
            None => return Vec::new(),
        };
        let body = &self.data[rec.start as usize..rec.end as usize];
        let mut out = Vec::new();
        let mut i = 0;
        while i < body.len() {
            if body[i] == b'#' {
                let mut j = i + 1;
                let mut val: u32 = 0;
                while j < body.len() && body[j].is_ascii_digit() {
                    val = val * 10 + u32::from(body[j] - b'0');
                    j += 1;
                }
                if j > i + 1 {
                    out.push(val);
                    i = j;
                    continue;
                }
            }
            i += 1;
        }
        out
    }

    /// Transitive closure of references reachable from `root` (inclusive),
    /// capped at `limit` entities, returned in ascending id order so an
    /// excerpt reads top-to-bottom.
    pub fn subgraph(&self, root: u32, limit: usize) -> Vec<u32> {
        let mut seen = std::collections::HashSet::new();
        let mut stack = vec![root];
        while let Some(id) = stack.pop() {
            if seen.len() >= limit || !seen.insert(id) {
                continue;
            }
            for r in self.entity_refs(id) {
                if !seen.contains(&r) {
                    stack.push(r);
                }
            }
        }
        let mut ids: Vec<u32> = seen.into_iter().collect();
        ids.sort_unstable();
        ids
    }
}

// ------------------------------------------------------------------ lexing

fn is_ident(b: u8) -> bool {
    b.is_ascii_alphanumeric() || b == b'_' || b == b'-'
}

fn ascii_upper(b: &[u8]) -> String {
    b.iter()
        .map(|c| (*c as char).to_ascii_uppercase())
        .collect()
}

fn starts_with_ci(data: &[u8], i: usize, kw: &[u8]) -> bool {
    data.len() >= i + kw.len()
        && data[i..i + kw.len()]
            .iter()
            .zip(kw)
            .all(|(a, b)| a.to_ascii_uppercase() == b.to_ascii_uppercase())
}

fn find_kw(data: &[u8], from: usize, kw: &[u8]) -> Option<usize> {
    let mut i = from;
    while i + kw.len() <= data.len() {
        match data[i] {
            b'\'' => i = skip_string(data, i),
            b'/' if i + 1 < data.len() && data[i + 1] == b'*' => i = skip_comment(data, i),
            _ => {
                if starts_with_ci(data, i, kw) {
                    return Some(i);
                }
                i += 1;
            }
        }
    }
    None
}

fn skip_string(data: &[u8], mut i: usize) -> usize {
    // data[i] == '\''
    i += 1;
    while i < data.len() {
        if data[i] == b'\'' {
            if i + 1 < data.len() && data[i + 1] == b'\'' {
                i += 2; // escaped quote
            } else {
                return i + 1;
            }
        } else {
            i += 1;
        }
    }
    i
}

fn skip_comment(data: &[u8], mut i: usize) -> usize {
    i += 2;
    while i + 1 < data.len() {
        if data[i] == b'*' && data[i + 1] == b'/' {
            return i + 2;
        }
        i += 1;
    }
    data.len()
}

fn skip_ws_comments(data: &[u8], mut i: usize) -> usize {
    while i < data.len() {
        match data[i] {
            b' ' | b'\t' | b'\r' | b'\n' => i += 1,
            b'/' if i + 1 < data.len() && data[i + 1] == b'*' => i = skip_comment(data, i),
            _ => break,
        }
    }
    i
}

fn parse_uint(data: &[u8], mut i: usize) -> (u32, usize) {
    let mut v: u64 = 0;
    while i < data.len() && data[i].is_ascii_digit() {
        v = v * 10 + (data[i] - b'0') as u64;
        i += 1;
    }
    (v as u32, i)
}

/// Find the index of the `)` matching the `(` at `open`, string/comment aware.
fn match_paren(data: &[u8], open: usize) -> Result<usize, String> {
    let mut depth = 0i32;
    let mut i = open;
    while i < data.len() {
        match data[i] {
            b'(' => {
                depth += 1;
                i += 1;
            }
            b')' => {
                depth -= 1;
                if depth == 0 {
                    return Ok(i);
                }
                i += 1;
            }
            b'\'' => i = skip_string(data, i),
            b'/' if i + 1 < data.len() && data[i + 1] == b'*' => i = skip_comment(data, i),
            _ => i += 1,
        }
    }
    Err("unbalanced parentheses".into())
}

// ------------------------------------------------------------- param parse

/// Parse a comma-separated parameter list (the text *between* parens).
pub fn parse_param_list(s: &[u8]) -> Vec<P> {
    let mut out = Vec::new();
    let mut i = 0usize;
    let n = s.len();
    loop {
        i = skip_ws_comments(s, i);
        if i >= n {
            break;
        }
        let (v, ni) = parse_value(s, i);
        out.push(v);
        i = skip_ws_comments(s, ni);
        if i < n && s[i] == b',' {
            i += 1;
        } else {
            break;
        }
    }
    out
}

fn parse_value(s: &[u8], i: usize) -> (P, usize) {
    let n = s.len();
    match s[i] {
        b'#' => {
            let (id, ni) = parse_uint(s, i + 1);
            (P::Ref(id), ni)
        }
        b'$' => (P::Null, i + 1),
        b'*' => (P::Star, i + 1),
        b'\'' => {
            let end = skip_string(s, i);
            let raw = &s[i + 1..end.saturating_sub(1)];
            let mut t = String::with_capacity(raw.len());
            let mut k = 0;
            while k < raw.len() {
                if raw[k] == b'\'' && k + 1 < raw.len() && raw[k + 1] == b'\'' {
                    t.push('\'');
                    k += 2;
                } else {
                    t.push(raw[k] as char);
                    k += 1;
                }
            }
            (P::Str(t), end)
        }
        b'.' => {
            // enum .NAME.
            let mut j = i + 1;
            while j < n && s[j] != b'.' {
                j += 1;
            }
            (P::Enum(ascii_upper(&s[i + 1..j])), (j + 1).min(n))
        }
        b'(' => {
            let close = match_paren(s, i).unwrap_or(n.saturating_sub(1));
            (P::L(parse_param_list(&s[i + 1..close])), close + 1)
        }
        c if c == b'-' || c == b'+' || c.is_ascii_digit() => {
            let mut j = i;
            let mut isf = false;
            if s[j] == b'-' || s[j] == b'+' {
                j += 1;
            }
            while j < n {
                match s[j] {
                    b'0'..=b'9' => j += 1,
                    b'.' => {
                        isf = true;
                        j += 1;
                    }
                    b'E' | b'e' => {
                        isf = true;
                        j += 1;
                        if j < n && (s[j] == b'-' || s[j] == b'+') {
                            j += 1;
                        }
                    }
                    _ => break,
                }
            }
            let txt: String = s[i..j].iter().map(|b| *b as char).collect();
            if isf {
                (P::F(txt.parse().unwrap_or(0.0)), j)
            } else {
                (P::I(txt.parse().unwrap_or(0)), j)
            }
        }
        c if is_ident(c) => {
            // typed parameter: NAME(args)  (e.g. IFCPLANEANGLEMEASURE(...))
            let ts = i;
            let mut j = i;
            while j < n && is_ident(s[j]) {
                j += 1;
            }
            let name = ascii_upper(&s[ts..j]);
            let k = skip_ws_comments(s, j);
            if k < n && s[k] == b'(' {
                let close = match_paren(s, k).unwrap_or(n.saturating_sub(1));
                (
                    P::Typed(name, parse_param_list(&s[k + 1..close])),
                    close + 1,
                )
            } else {
                (P::Enum(name), j)
            }
        }
        _ => (P::Null, i + 1),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(src: &str) -> StepFile {
        StepFile::parse(src.as_bytes().to_vec()).expect("parse")
    }

    const MINI: &str = "ISO-10303-21;
HEADER;
FILE_NAME('t','',(''),(''),'','','');
ENDSEC;
DATA;
#1=CARTESIAN_POINT('a point',(1.,2.5E-1,-3.));
#2 = DIRECTION ( '' , ( 0., 0., 1. ) ) ;
#3=ADVANCED_FACE('it''s',(#10,#20),#2,.T.);
/* a comment with ; and ( inside */
#4=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#5=MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#4);
ENDSEC;
END-ISO-10303-21;
";

    #[test]
    fn indexes_entities_and_types() {
        let sf = parse(MINI);
        assert_eq!(sf.entities.len(), 5);
        assert_eq!(sf.entity_type(1), Some("CARTESIAN_POINT"));
        assert_eq!(sf.entity_type(4), Some(TYPE_COMPLEX));
        assert_eq!(sf.of_type("CARTESIAN_POINT"), &[1]);
    }

    #[test]
    fn parses_params_lazily() {
        let sf = parse(MINI);
        let p = sf.params(1).unwrap();
        assert_eq!(p[0], P::Str("a point".into()));
        let l = p[1].as_list().unwrap();
        assert_eq!(l[0].as_f64(), Some(1.0));
        assert_eq!(l[1].as_f64(), Some(0.25));
        assert_eq!(l[2].as_f64(), Some(-3.0));
    }

    #[test]
    fn handles_whitespace_escapes_enums_refs() {
        let sf = parse(MINI);
        let d = sf.params(2).unwrap();
        assert_eq!(d[1].as_list().unwrap()[2].as_f64(), Some(1.0));

        let f = sf.params(3).unwrap();
        assert_eq!(f[0].as_str(), Some("it's")); // '' escape
        let bounds = f[1].as_list().unwrap();
        assert_eq!(bounds[0].as_ref_id(), Some(10));
        assert_eq!(bounds[1].as_ref_id(), Some(20));
        assert_eq!(f[3].as_bool(), Some(true)); // .T.
    }

    #[test]
    fn complex_instances_and_typed_params() {
        let sf = parse(MINI);
        assert!(sf.is_complex(4));
        let si = sf.complex_leaf(4, "SI_UNIT").unwrap();
        assert_eq!(si[0], P::Enum("MILLI".into()));
        assert_eq!(si[1], P::Enum("METRE".into()));
        assert_eq!(
            sf.complex_leaf_names(4),
            vec!["LENGTH_UNIT", "NAMED_UNIT", "SI_UNIT"]
        );

        let m = sf.params(5).unwrap();
        match &m[0] {
            P::Typed(name, args) => {
                assert_eq!(name, "LENGTH_MEASURE");
                assert_eq!(args[0].as_f64(), Some(25.4));
            }
            other => panic!("expected typed param, got {:?}", other),
        }
    }

    #[test]
    fn entity_source_refs_and_subgraph_round_trip() {
        let sf = parse(MINI);

        // refs are scanned straight from the bytes, including inside lists
        let mut r = sf.entity_refs(3);
        r.sort_unstable();
        assert_eq!(r, vec![2, 10, 20]); // (#10,#20) bounds + #2 surface

        // a simple entity reconstructs to re-parseable Part-21
        assert!(sf
            .entity_source(1)
            .unwrap()
            .starts_with("#1=CARTESIAN_POINT("));
        // a complex instance keeps its leaf structure inside outer parens
        let cs = sf.entity_source(4).unwrap();
        assert!(cs.starts_with("#4=(") && cs.contains("SI_UNIT(.MILLI.,.METRE.)"));
        // a missing id yields nothing rather than panicking
        assert!(sf.entity_source(999).is_none());

        // subgraph from #5 (MEASURE_WITH_UNIT -> #4 unit) pulls the unit in
        assert_eq!(sf.subgraph(5, 100), vec![4, 5]);

        // round-trip: a DATA section rebuilt from the excerpt re-parses, and
        // the reconstructed entities carry the same types and references
        let mut doc = String::from("ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n");
        for id in sf.subgraph(5, 100) {
            doc.push_str(&sf.entity_source(id).unwrap());
            doc.push('\n');
        }
        doc.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
        let re = parse(&doc);
        assert_eq!(re.entities.len(), 2);
        assert!(re.is_complex(4));
        assert_eq!(re.params(5).unwrap()[1].as_ref_id(), Some(4));
    }

    #[test]
    fn strings_with_semicolons_and_parens_dont_break_indexing() {
        let sf = parse(
            "DATA;
#1=PRODUCT('a;b(c)','x','',());
#2=PRODUCT('q','r','',());
ENDSEC;",
        );
        assert_eq!(sf.entities.len(), 2);
        assert_eq!(sf.params(1).unwrap()[0].as_str(), Some("a;b(c)"));
    }
}
