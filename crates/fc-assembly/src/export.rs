//! Assembly data exchange: structured STEP export/import (XDE product structure: one
//! shape per part, components with placements and names) and flat STL export.

use crate::frame::Frame;
use crate::model::{AssemblyId, InstanceOf, PartId, Workspace, WorkspaceError};
use crate::structure::{compound, MAX_DEPTH};
use fc_occt::{Label, StepSchema, XdeDoc};
use std::collections::HashMap;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StepExportOptions {
    pub schema: StepSchema,
    /// Write part colours (`PartMeta::color`).
    pub colors: bool,
    /// Leave hidden instances out.
    pub visible_only: bool,
}

impl Default for StepExportOptions {
    fn default() -> Self {
        StepExportOptions { schema: StepSchema::Ap214, colors: true, visible_only: false }
    }
}

/// What an export wrote.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct StepExportStats {
    /// Distinct part shapes (STEP products with geometry).
    pub parts: usize,
    /// Assemblies, the top one included.
    pub assemblies: usize,
    /// Components (instances) over all assemblies.
    pub components: usize,
    /// Instances left out because their part has no geometry.
    pub skipped: Vec<String>,
}

/// What an import created.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct StepImport {
    /// The top assembly.
    pub assembly: AssemblyId,
    /// Sub-assemblies created for the file's nested assemblies.
    pub sub_assemblies: Vec<AssemblyId>,
    pub parts: Vec<PartId>,
}

impl Workspace {
    /// Writes an assembly as a structured STEP file (STEPCAFControl, AP214 or AP242): each
    /// part once as a product with its shape, each (sub-)assembly as a product whose
    /// components place its instances, with instance and part names (and optional colours).
    pub fn export_step(&self, asm: AssemblyId, path: &str, opts: &StepExportOptions) -> Result<StepExportStats, WorkspaceError> {
        let mut doc = XdeDoc::new()?;
        let mut st = StepExportStats::default();
        let mut parts: HashMap<PartId, Option<Label>> = HashMap::new();
        let mut asms: HashMap<AssemblyId, Label> = HashMap::new();
        self.xde_assembly(asm, &mut doc, &mut parts, &mut asms, &mut st, opts, 0)?;
        doc.write_step(path, opts.schema)?;
        st.parts = parts.values().filter(|l| l.is_some()).count();
        st.assemblies = asms.len();
        Ok(st)
    }

    #[allow(clippy::too_many_arguments)]
    fn xde_assembly(
        &self,
        id: AssemblyId,
        doc: &mut XdeDoc,
        parts: &mut HashMap<PartId, Option<Label>>,
        asms: &mut HashMap<AssemblyId, Label>,
        st: &mut StepExportStats,
        opts: &StepExportOptions,
        depth: usize,
    ) -> Result<Label, WorkspaceError> {
        if let Some(l) = asms.get(&id) {
            return Ok(*l);
        }
        if depth > MAX_DEPTH {
            return Err(WorkspaceError::Cycle(id));
        }
        let asm = self.assembly(id).ok_or(WorkspaceError::Missing(id))?;
        let label = doc.add_assembly(&asm.name)?;
        asms.insert(id, label);
        for inst in &asm.instances {
            if opts.visible_only && !inst.visible {
                continue;
            }
            let child = match inst.of {
                InstanceOf::Part(p) => {
                    if !parts.contains_key(&p) {
                        let l = match self.part(p).and_then(|e| e.shape().map(|s| (e, s))) {
                            Some((e, s)) => Some(doc.add_part(s, &e.name, e.meta.color.filter(|_| opts.colors))?),
                            None => None,
                        };
                        parts.insert(p, l);
                    }
                    match parts[&p] {
                        Some(l) => l,
                        None => {
                            st.skipped.push(inst.name.clone());
                            continue;
                        }
                    }
                }
                InstanceOf::Assembly(s) => self.xde_assembly(s, doc, parts, asms, st, opts, depth + 1)?,
            };
            doc.add_component(label, child, inst.placement.to_matrix34(), &inst.name)?;
            st.components += 1;
        }
        Ok(label)
    }

    /// Reads a STEP file's product structure into this workspace: every part becomes an
    /// imported part (solid stored as BREP), every assembly an [`crate::Assembly`] with an
    /// instance per component (sub-assemblies become [`InstanceOf::Assembly`] instances).
    /// A file without an assembly, or with several top-level shapes, gets a top assembly
    /// named after the file. Nothing is grounded and there are no joints.
    pub fn import_step(&mut self, path: &str) -> Result<StepImport, WorkspaceError> {
        let doc = XdeDoc::read_step(path)?;
        let nodes = doc.nodes()?;
        let comps = doc.components()?;
        let roots = doc.roots()?;
        let stem = std::path::Path::new(path).file_stem().and_then(|s| s.to_str()).unwrap_or("Assembly").to_string();
        let mut out = StepImport::default();
        let mut made: HashMap<usize, InstanceOf> = HashMap::new();

        // Create the definitions reachable from the roots, children first.
        fn visit(
            ws: &mut Workspace,
            doc: &XdeDoc,
            nodes: &[fc_occt::AsmNode],
            comps: &[fc_occt::AsmComp],
            n: usize,
            made: &mut HashMap<usize, InstanceOf>,
            out: &mut StepImport,
            depth: usize,
        ) -> Result<InstanceOf, WorkspaceError> {
            if let Some(o) = made.get(&n) {
                return Ok(*o);
            }
            if depth > MAX_DEPTH {
                return Err(WorkspaceError::Cycle(0));
            }
            let node = &nodes[n];
            let name = if node.name.is_empty() { format!("Part{n}") } else { node.name.clone() };
            let of = if node.is_assembly {
                let mut children = vec![];
                for c in comps.iter().filter(|c| c.parent == n) {
                    let of = visit(ws, doc, nodes, comps, c.child, made, out, depth + 1)?;
                    let cname = if c.name.is_empty() { nodes[c.child].name.clone() } else { c.name.clone() };
                    children.push((cname, of, Frame::from_matrix34(&c.matrix)));
                }
                let id = ws.add_assembly(&name);
                let a = ws.assembly_mut(id).expect("just added");
                for (cname, of, pl) in children {
                    a.add_instance(if cname.is_empty() { "Part" } else { &cname }, of, pl);
                }
                out.sub_assemblies.push(id);
                InstanceOf::Assembly(id)
            } else {
                let shape = doc.shape(n)?;
                let id = ws.add_imported_part(&name, shape);
                if node.rgba[3] >= 0.0 {
                    ws.part_mut(id).expect("just added").meta.color = Some([node.rgba[0], node.rgba[1], node.rgba[2]]);
                }
                out.parts.push(id);
                InstanceOf::Part(id)
            };
            made.insert(n, of);
            Ok(of)
        }

        let mut tops = vec![];
        for &r in &roots {
            tops.push((r, visit(self, &doc, &nodes, &comps, r, &mut made, &mut out, 0)?));
        }
        match tops.as_slice() {
            [(_, InstanceOf::Assembly(a))] => {
                out.assembly = *a;
                out.sub_assemblies.retain(|x| x != a);
            }
            _ => {
                let id = self.add_assembly(&stem);
                let a = self.assembly_mut(id).expect("just added");
                for (r, of) in &tops {
                    let n = &nodes[*r].name;
                    a.add_instance(if n.is_empty() { "Part" } else { n }, *of, Frame::IDENTITY);
                }
                out.assembly = id;
            }
        }
        Ok(out)
    }

    /// Writes the posed assembly (sub-assemblies flattened) as one binary STL mesh.
    /// Returns the number of part instances written.
    pub fn export_stl(&self, asm: AssemblyId, path: &str, linear_deflection: f64, visible_only: bool) -> Result<usize, WorkspaceError> {
        let shapes: Vec<_> = self.leaves(asm).iter().filter(|l| l.visible || !visible_only).filter_map(|l| self.leaf_shape(l)).collect();
        let c = compound(&shapes).ok_or_else(|| WorkspaceError::Occt(fc_occt::OcctError("nothing to export".into())))?;
        c.write_stl(path, linear_deflection)?;
        Ok(shapes.len())
    }
}
