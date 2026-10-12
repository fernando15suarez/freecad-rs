//! Bill of materials, after FreeCAD's `Assembly::BomObject`: rows grouped by the part (or
//! sub-assembly) they place, numbered `1`, `2`, `2.1`, ..., with quantities, plus our part
//! metadata (part number, description, material, mass).

use crate::model::{AssemblyId, InstanceOf, PartId, Workspace};
use crate::structure::MAX_DEPTH;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BomOptions {
    /// List sub-assemblies' contents indented under them (FreeCAD's `detailSubAssemblies`);
    /// otherwise a sub-assembly is one row.
    pub detail_sub_assemblies: bool,
    /// One row per distinct part with its total quantity over the whole tree (no
    /// sub-assembly rows).
    pub flatten: bool,
    /// Skip hidden instances.
    pub visible_only: bool,
}

impl Default for BomOptions {
    fn default() -> Self {
        BomOptions { detail_sub_assemblies: true, flatten: false, visible_only: false }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BomItem {
    Part(PartId),
    Assembly(AssemblyId),
}

#[derive(Clone, Debug, PartialEq)]
pub struct BomRow {
    /// FreeCAD's Index column: `"2.1"` is the first child of row 2.
    pub index: String,
    /// Nesting depth (0 = directly in the assembly).
    pub level: usize,
    pub item: BomItem,
    /// Part or assembly name (FreeCAD uses the linked object's label).
    pub name: String,
    pub part_number: String,
    pub description: String,
    pub material: String,
    /// Count per parent (per one of the parent row's items); for a flattened BOM, the total.
    pub quantity: u32,
    /// Mass of one item in kg (sub-assemblies: sum of their parts, when all are known).
    pub unit_mass: Option<f64>,
}

impl BomRow {
    pub fn total_mass(&self) -> Option<f64> {
        self.unit_mass.map(|m| m * self.quantity as f64)
    }
}

impl Workspace {
    pub fn bom(&self, asm: AssemblyId, opts: &BomOptions) -> Vec<BomRow> {
        let mut rows = vec![];
        if opts.flatten {
            let mut totals: Vec<(PartId, u32)> = vec![];
            self.count_parts(asm, 1, opts, 0, &mut totals);
            for (k, (p, q)) in totals.into_iter().enumerate() {
                let mut r = self.row(BomItem::Part(p), (k + 1).to_string(), 0);
                r.quantity = q;
                rows.push(r);
            }
        } else {
            self.bom_level(asm, "", 0, opts, &mut rows);
        }
        rows
    }

    fn count_parts(&self, asm: AssemblyId, mult: u32, opts: &BomOptions, depth: usize, out: &mut Vec<(PartId, u32)>) {
        let Some(a) = self.assembly(asm).filter(|_| depth <= MAX_DEPTH) else { return };
        for i in a.instances.iter().filter(|i| i.visible || !opts.visible_only) {
            match i.of {
                InstanceOf::Part(p) => match out.iter_mut().find(|(q, _)| *q == p) {
                    Some((_, n)) => *n += mult,
                    None => out.push((p, mult)),
                },
                InstanceOf::Assembly(s) => self.count_parts(s, mult, opts, depth + 1, out),
            }
        }
    }

    fn bom_level(&self, asm: AssemblyId, prefix: &str, level: usize, opts: &BomOptions, rows: &mut Vec<BomRow>) {
        let Some(a) = self.assembly(asm).filter(|_| level <= MAX_DEPTH) else { return };
        // Group siblings by what they place, in first-seen order.
        let mut groups: Vec<(BomItem, u32)> = vec![];
        for i in a.instances.iter().filter(|i| i.visible || !opts.visible_only) {
            let item = match i.of {
                InstanceOf::Part(p) => BomItem::Part(p),
                InstanceOf::Assembly(s) => BomItem::Assembly(s),
            };
            match groups.iter_mut().find(|(g, _)| *g == item) {
                Some((_, n)) => *n += 1,
                None => groups.push((item, 1)),
            }
        }
        for (k, (item, q)) in groups.into_iter().enumerate() {
            let index = if prefix.is_empty() { (k + 1).to_string() } else { format!("{prefix}.{}", k + 1) };
            let mut r = self.row(item, index.clone(), level);
            r.quantity = q;
            rows.push(r);
            if let (BomItem::Assembly(s), true) = (item, opts.detail_sub_assemblies) {
                self.bom_level(s, &index, level + 1, opts, rows);
            }
        }
    }

    fn row(&self, item: BomItem, index: String, level: usize) -> BomRow {
        let mut r = BomRow { index, level, item, name: String::new(), part_number: String::new(), description: String::new(), material: String::new(), quantity: 1, unit_mass: None };
        match item {
            BomItem::Part(p) => {
                if let Some(e) = self.part(p) {
                    r.name = e.name.clone();
                    r.part_number = e.meta.part_number.clone();
                    r.description = e.meta.description.clone();
                    r.material = e.meta.material.clone();
                    r.unit_mass = e.mass_kg();
                }
            }
            BomItem::Assembly(a) => {
                r.name = self.assembly(a).map(|a| a.name.clone()).unwrap_or_default();
                let mut parts = vec![];
                self.count_parts(a, 1, &BomOptions::default(), 0, &mut parts);
                r.unit_mass = parts.iter().map(|(p, q)| self.part(*p).and_then(|e| e.mass_kg()).map(|m| m * *q as f64)).sum();
            }
        }
        r
    }
}

fn csv_field(s: &str) -> String {
    if s.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", s.replace('"', "\"\""))
    } else {
        s.to_string()
    }
}

/// CSV with FreeCAD's BOM columns (Index, Name, Description, Quantity) plus Part Number,
/// Material and masses (kg). Indented BOMs keep FreeCAD's dotted index.
pub fn bom_csv(rows: &[BomRow]) -> String {
    let mut out = String::from("Index,Name,Part Number,Description,Material,Quantity,Unit Mass (kg),Total Mass (kg)\n");
    let mass = |m: Option<f64>| m.map(|m| format!("{m:.6}")).unwrap_or_default();
    for r in rows {
        let cells = [
            r.index.clone(),
            r.name.clone(),
            r.part_number.clone(),
            r.description.clone(),
            r.material.clone(),
            r.quantity.to_string(),
            mass(r.unit_mass),
            mass(r.total_mass()),
        ];
        out.push_str(&cells.iter().map(|c| csv_field(c)).collect::<Vec<_>>().join(","));
        out.push('\n');
    }
    out
}
