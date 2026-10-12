//! Assemblies for freecad-rs: a multi-part [`Workspace`] (the `.fcrs` v2 project file),
//! [`Assembly`] instances with grounded parts, FreeCAD-style joints / Onshape-style mates
//! ([`joint`]), and a 3D constraint solver ([`solver`]) with DOF and redundancy/conflict
//! diagnosis, minimal-movement solving and joint-respecting drag.
//!
//! Reference behaviour: FreeCAD 1.1's Assembly workbench (Ondsel solver) and Onshape's
//! mates on mate connectors. See `docs/assembly.md` for the parity results.

pub mod dual;
pub mod fcstd;
pub mod frame;
pub mod geom;
pub mod joint;
pub mod model;
pub mod solver;

pub use frame::Frame;
pub use geom::{infer_frame, ElementGeom, ImplicitPoint, RefKind};
pub use joint::{DistanceMode, Joint, JointKind, JointRef, Limits, Sense};
pub use model::{
    box_elements, Assembly, AssemblyConnector, AssemblyId, ConnectorId, ExplodeStep, ExplodedView, Instance,
    InstanceId, InstanceOf, JointId, MapResolver, NoGeometry, PartEntry, PartId, PartResolver, Workspace,
    WorkspaceError, WORKSPACE_VERSION,
};
pub use solver::{DragTarget, SolveError, SolveOptions, SolveReport, SolveStatus};

impl Workspace {
    /// Solve an assembly against this workspace's part geometry.
    pub fn solve_assembly(&mut self, id: AssemblyId, opts: &SolveOptions) -> Option<SolveReport> {
        let asm = self.assembly(id)?;
        let (rep, pl) = solver::compute_solve(asm, self, opts);
        let asm = self.assembly_mut(id)?;
        for (i, f) in asm.instances.iter_mut().zip(pl) {
            i.placement = f;
        }
        Some(rep)
    }
    /// Drag an instance of an assembly, honouring its joints.
    pub fn drag_instance(&mut self, id: AssemblyId, instance: InstanceId, target: DragTarget, opts: &SolveOptions) -> Option<Result<SolveReport, SolveError>> {
        let asm = self.assembly(id)?;
        let out = solver::compute_drag(asm, self, instance, target, opts);
        Some(out.map(|(rep, pl)| {
            let asm = self.assembly_mut(id).expect("assembly exists");
            for (i, f) in asm.instances.iter_mut().zip(pl) {
                i.placement = f;
            }
            rep
        }))
    }
}

#[cfg(test)]
mod tests;
