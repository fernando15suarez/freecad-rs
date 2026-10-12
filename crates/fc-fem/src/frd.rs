//! CalculiX `.frd` result file parser (ASCII, the format ccx writes for `*NODE FILE` /
//! `*EL FILE`), plus the `.dat` file bits FreeCAD reads (reaction force totals, eigenvalues).

use std::collections::{BTreeMap, HashMap};

use crate::FemError;

/// One nodal result block (`-4  DISP`, `-4  STRESS`, ...).
#[derive(Debug, Clone, PartialEq)]
pub struct FrdBlock {
    /// Block name: DISP, STRESS, TOSTRAIN, NDTEMP, FLUX, ERROR, ...
    pub name: String,
    /// Step value of the `100CL` record: time for static steps, frequency in Hz for modal steps.
    pub value: f64,
    /// Load case / mode number of the `100CL` record (1-based).
    pub number: u32,
    /// Analysis kind from `100CL` (0 static, 1 time step, 2 frequency, 3 load step...).
    pub kind: u32,
    /// Component names (D1 D2 D3 / SXX SYY SZZ SXY SYZ SZX / T ...), "ALL" entries dropped.
    pub components: Vec<String>,
    /// node id -> component values.
    pub values: HashMap<u32, Vec<f64>>,
}

/// Parsed .frd file.
#[derive(Debug, Clone, Default)]
pub struct Frd {
    /// Node coordinates from the `2C` block.
    pub nodes: BTreeMap<u32, [f64; 3]>,
    pub blocks: Vec<FrdBlock>,
}

impl Frd {
    /// Last block with the given name (static results have one per name).
    pub fn last(&self, name: &str) -> Option<&FrdBlock> {
        self.blocks.iter().rev().find(|b| b.name == name)
    }
    /// All blocks with the given name, in file order (modal: one per mode).
    pub fn all<'a>(&'a self, name: &'a str) -> impl Iterator<Item = &'a FrdBlock> + 'a {
        self.blocks.iter().filter(move |b| b.name == name)
    }
}

fn field(line: &str, a: usize, b: usize) -> &str {
    let b = b.min(line.len());
    if a >= b {
        ""
    } else {
        &line[a..b]
    }
}

fn num(s: &str) -> Result<f64, FemError> {
    let t = s.trim();
    t.parse::<f64>().map_err(|_| FemError::Results(format!("frd: bad number '{t}'")))
}

/// Parse fixed-width values after the node number (12 characters each).
fn values(line: &str, start: usize, n: usize) -> Result<Vec<f64>, FemError> {
    let mut v = Vec::with_capacity(n);
    for i in 0..n {
        let s = field(line, start + 12 * i, start + 12 * (i + 1));
        if s.trim().is_empty() {
            break;
        }
        v.push(num(s)?);
    }
    Ok(v)
}

/// Parse an ASCII .frd file.
pub fn parse_frd(text: &str) -> Result<Frd, FemError> {
    let mut frd = Frd::default();
    let mut lines = text.lines().peekable();
    // node id width: 10 (long format, flag 1) or 5 (short, flag 0)
    let mut w = 10usize;
    let mut step_value = 0.0;
    let mut step_number = 0u32;
    let mut step_kind = 0u32;
    while let Some(l) = lines.next() {
        let tag = l.trim_start();
        if tag.starts_with("2C") && l.len() >= 6 && field(l, 0, 6).trim() == "2C" {
            w = if l.split_whitespace().last() == Some("0") { 5 } else { 10 };
            while let Some(&n) = lines.peek() {
                if n.starts_with(" -1") {
                    let id = field(n, 3, 3 + w).trim().parse::<u32>().map_err(|_| FemError::Results("frd: bad node id".into()))?;
                    let v = values(n, 3 + w, 3)?;
                    if v.len() == 3 {
                        frd.nodes.insert(id, [v[0], v[1], v[2]]);
                    }
                    lines.next();
                } else if n.starts_with(" -3") {
                    lines.next();
                    break;
                } else {
                    lines.next();
                }
            }
        } else if tag.starts_with("100C") {
            // " 100CL  101 1.000000000        6488                     0    1           1"
            let toks: Vec<&str> = l.split_whitespace().collect();
            step_value = toks.get(2).and_then(|s| s.parse().ok()).unwrap_or(0.0);
            step_kind = toks.get(4).and_then(|s| s.parse().ok()).unwrap_or(0);
            // the load-case number is the 6th token, possibly glued to a text ("1MODAL")
            step_number = toks
                .get(5)
                .map(|s| s.chars().take_while(|c| c.is_ascii_digit()).collect::<String>())
                .and_then(|s| s.parse().ok())
                .unwrap_or(0);
            w = if toks.last() == Some(&"0") { 5 } else { 10 };
        } else if l.starts_with(" -4") {
            let toks: Vec<&str> = l.split_whitespace().collect();
            let name = toks.get(1).copied().unwrap_or("").to_string();
            let ncomp: usize = toks.get(2).and_then(|s| s.parse().ok()).unwrap_or(0);
            let mut comps = Vec::new();
            let mut block = FrdBlock {
                name,
                value: step_value,
                number: step_number,
                kind: step_kind,
                components: Vec::new(),
                values: HashMap::new(),
            };
            while let Some(&n) = lines.peek() {
                if n.starts_with(" -5") {
                    let t: Vec<&str> = n.split_whitespace().collect();
                    let cname = t.get(1).copied().unwrap_or("");
                    // "ALL" is the derived magnitude of a vector block, not stored in the data lines
                    if cname != "ALL" {
                        comps.push(cname.to_string());
                    }
                    lines.next();
                } else {
                    break;
                }
            }
            let nvals = if comps.is_empty() { ncomp } else { comps.len() };
            block.components = comps;
            let mut last_id: Option<u32> = None;
            while let Some(&n) = lines.peek() {
                if n.starts_with(" -1") {
                    let id = field(n, 3, 3 + w).trim().parse::<u32>().map_err(|_| FemError::Results("frd: bad node id".into()))?;
                    let v = values(n, 3 + w, nvals.min(6))?;
                    block.values.insert(id, v);
                    last_id = Some(id);
                    lines.next();
                } else if n.starts_with(" -2") {
                    // continuation line of the previous node
                    if let Some(id) = last_id {
                        let more = values(n, 3 + w, nvals.saturating_sub(6))?;
                        block.values.get_mut(&id).unwrap().extend(more);
                    }
                    lines.next();
                } else {
                    break;
                }
            }
            frd.blocks.push(block);
        }
    }
    if frd.blocks.is_empty() && frd.nodes.is_empty() {
        return Err(FemError::Results("frd file has no results".into()));
    }
    Ok(frd)
}

/// Totals of `*NODE PRINT, TOTALS=ONLY` blocks: set name (upper case, as ccx prints it) ->
/// [fx, fy, fz] of the last increment.
pub fn parse_dat_totals(text: &str) -> BTreeMap<String, [f64; 3]> {
    let mut out = BTreeMap::new();
    let lines: Vec<&str> = text.lines().collect();
    for (i, l) in lines.iter().enumerate() {
        let t = l.trim();
        if let Some(rest) = t.strip_prefix("total force (fx,fy,fz) for set") {
            let name = rest.split("and time").next().unwrap_or("").trim().to_string();
            if let Some(vals) = lines[i + 1..].iter().find(|s| !s.trim().is_empty()) {
                let v: Vec<f64> = vals.split_whitespace().filter_map(|s| s.parse().ok()).collect();
                if v.len() >= 3 {
                    out.insert(name, [v[0], v[1], v[2]]);
                }
            }
        }
    }
    out
}

/// Eigenfrequencies in Hz from the "E I G E N V A L U E   O U T P U T" table.
pub fn parse_dat_frequencies(text: &str) -> Vec<f64> {
    let mut out = Vec::new();
    let mut lines = text.lines();
    while let Some(l) = lines.next() {
        if l.contains("E I G E N V A L U E   O U T P U T") {
            let mut started = false;
            for l in lines.by_ref() {
                let t: Vec<&str> = l.split_whitespace().collect();
                let row = t.len() >= 4 && t[0].parse::<u32>().is_ok() && t[1].parse::<f64>().is_ok();
                if row {
                    started = true;
                    out.push(t[3].parse().unwrap_or(0.0));
                } else if started {
                    break;
                }
            }
            break;
        }
    }
    out
}
