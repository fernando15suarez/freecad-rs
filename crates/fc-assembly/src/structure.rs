//! Assembly structure: sub-assemblies (instances of other assemblies, posed rigidly),
//! flattening to leaf part instances, and posed shapes.

use crate::frame::Frame;
use crate::model::{AssemblyId, InstanceId, InstanceOf, PartId, Workspace, WorkspaceError};
use fc_occt::Shape;

/// Instance ids from the top assembly down (`[sub_asm_instance, part_instance]`).
pub type InstancePath = Vec<InstanceId>;

/// Sub-assemblies may nest this deep (guards against cycles in hand-edited files).
pub const MAX_DEPTH: usize = 32;

/// A part instance reached through the assembly tree.
#[derive(Clone, Debug, PartialEq)]
pub struct LeafInstance {
    pub path: InstancePath,
    /// Instance names along the path joined with `.` (`"Sub.Bolt"`), FreeCAD's sub-name style.
    pub name: String,
    pub part: PartId,
    /// Placement in the top assembly's coordinates.
    pub placement: Frame,
    /// Every instance along the path is visible.
    pub visible: bool,
}

impl Workspace {
    /// Does `outer` (transitively) instance `inner`, or is it `inner`?
    pub fn uses_assembly(&self, outer: AssemblyId, inner: AssemblyId) -> bool {
        fn go(ws: &Workspace, a: AssemblyId, target: AssemblyId, depth: usize) -> bool {
            if a == target {
                return true;
            }
            if depth > MAX_DEPTH {
                return true;
            }
            let Some(asm) = ws.assembly(a) else { return false };
            asm.instances.iter().any(|i| matches!(i.of, InstanceOf::Assembly(s) if go(ws, s, target, depth + 1)))
        }
        go(self, outer, inner, 0)
    }

    /// Adds an instance of assembly `sub` to `parent` (posed rigidly). Fails if `sub` uses `parent`.
    pub fn add_sub_assembly(&mut self, parent: AssemblyId, name: &str, sub: AssemblyId, placement: Frame) -> Result<InstanceId, WorkspaceError> {
        if self.assembly(sub).is_none() {
            return Err(WorkspaceError::Missing(sub));
        }
        if self.uses_assembly(sub, parent) {
            return Err(WorkspaceError::Cycle(parent));
        }
        let asm = self.assembly_mut(parent).ok_or(WorkspaceError::Missing(parent))?;
        Ok(asm.add_instance(name, InstanceOf::Assembly(sub), placement))
    }

    /// Every part instance of an assembly, recursing into sub-assemblies.
    pub fn leaves(&self, asm: AssemblyId) -> Vec<LeafInstance> {
        self.leaves_under(asm, &[])
    }

    /// The part instances under an instance path (all of them for an empty path).
    pub fn leaves_under(&self, asm: AssemblyId, path: &[InstanceId]) -> Vec<LeafInstance> {
        let mut out = vec![];
        let Some(a) = self.assembly(asm) else { return out };
        // Walk down the given path first.
        let mut prefix = vec![];
        let mut names = vec![];
        let mut pl = Frame::IDENTITY;
        let mut visible = true;
        let mut cur = a;
        for (k, id) in path.iter().enumerate() {
            let Some(i) = cur.instance(*id) else { return out };
            prefix.push(*id);
            names.push(i.name.clone());
            pl = pl.mul(&i.placement);
            visible &= i.visible;
            match i.of {
                InstanceOf::Part(p) => {
                    if k + 1 == path.len() {
                        out.push(LeafInstance { path: prefix, name: names.join("."), part: p, placement: pl, visible });
                    }
                    return out;
                }
                InstanceOf::Assembly(s) => match self.assembly(s) {
                    Some(s) => cur = s,
                    None => return out,
                },
            }
        }
        self.collect(cur.id, &mut prefix, &mut names, pl, visible, 0, &mut out);
        out
    }

    #[allow(clippy::too_many_arguments)]
    fn collect(&self, asm: AssemblyId, path: &mut Vec<InstanceId>, names: &mut Vec<String>, pl: Frame, visible: bool, depth: usize, out: &mut Vec<LeafInstance>) {
        if depth > MAX_DEPTH {
            return;
        }
        let Some(a) = self.assembly(asm) else { return };
        for i in &a.instances {
            path.push(i.id);
            names.push(i.name.clone());
            let p = pl.mul(&i.placement);
            match i.of {
                InstanceOf::Part(part) => out.push(LeafInstance { path: path.clone(), name: names.join("."), part, placement: p, visible: visible && i.visible }),
                InstanceOf::Assembly(s) => self.collect(s, path, names, p, visible && i.visible, depth + 1, out),
            }
            path.pop();
            names.pop();
        }
    }

    /// A part's solid in part coordinates.
    pub fn part_shape(&self, part: PartId) -> Option<&Shape> {
        self.part(part)?.shape()
    }

    /// A leaf's solid posed in the top assembly's coordinates.
    pub fn leaf_shape(&self, leaf: &LeafInstance) -> Option<Shape> {
        self.part_shape(leaf.part)?.moved(leaf.placement.to_matrix34()).ok()
    }

    /// The posed solid(s) under an instance path: the part's solid for a part instance, a
    /// compound for a sub-assembly (or the whole assembly for an empty path).
    pub fn instance_shape(&self, asm: AssemblyId, path: &[InstanceId]) -> Option<Shape> {
        let leaves = self.leaves_under(asm, path);
        let shapes: Vec<Shape> = leaves.iter().filter_map(|l| self.leaf_shape(l)).collect();
        match shapes.len() {
            0 => None,
            1 if leaves.len() == 1 && path.len() == leaves[0].path.len() => shapes.into_iter().next(),
            _ => compound(&shapes),
        }
    }
}

/// Compound of shapes (each kept as one child); `None` for no shapes.
pub fn compound(shapes: &[Shape]) -> Option<Shape> {
    if shapes.is_empty() {
        return None;
    }
    // `compound_with` copies its receiver's children and appends the argument.
    let mut c = Shape::default();
    for s in shapes {
        c = c.compound_with(s).ok()?;
    }
    Some(c)
}

impl Workspace {
    /// Evaluates exploded view `view` of an assembly at animation time `t` (0..=1) using the
    /// instances' real bounding boxes (radial steps and trail lines). Non-destructive: the
    /// assembly's placements are not changed.
    pub fn exploded(&self, asm: AssemblyId, view: usize, t: f64) -> Option<crate::model::Exploded> {
        let a = self.assembly(asm)?;
        let mut centers = std::collections::HashMap::new();
        let mut bbox: Option<[f64; 6]> = None;
        for i in &a.instances {
            let Some(s) = self.instance_shape(asm, &[i.id]) else { continue };
            let b = s.bbox_exact();
            let c = [(b[0] + b[3]) * 0.5, (b[1] + b[4]) * 0.5, (b[2] + b[5]) * 0.5];
            centers.insert(i.id, i.placement.inverse().transform_point(c));
            bbox = Some(match bbox {
                None => b,
                Some(u) => [u[0].min(b[0]), u[1].min(b[1]), u[2].min(b[2]), u[3].max(b[3]), u[4].max(b[4]), u[5].max(b[5])],
            });
        }
        Some(a.explode_with(view, t, &|id| centers.get(&id).copied(), bbox))
    }
}
