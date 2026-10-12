//! fc-fem: finite element analysis in the style of FreeCAD's FEM workbench.
//!
//! Like FreeCAD, the heavy lifting is done by external programs: **Gmsh** meshes the solid
//! (driven by a `.geo` script written the way FreeCAD's `GmshTools` writes it) and **CalculiX**
//! (`ccx`) solves (input written the way FreeCAD's `ccxtools` writer does). This crate owns the
//! analysis model, the mesh/constraint bookkeeping, the input writer and the result readers.
//!
//! ```no_run
//! use fc_fem::*;
//! let mut a = Analysis::new(ShapeSource::Brep("beam.brep".into()), Material::steel_generic());
//! a.mesh.max_size = 25.0;
//! a.fixed(vec![SubRef::face(1)]);
//! a.force(vec![SubRef::face(2)], 1000.0, ForceDirection::Vector([0.0, 0.0, -1.0]));
//! let sol = solve(&a, &RunOptions::default()).unwrap();
//! println!("max |u| = {} mm", sol.results.max_displacement().unwrap().value);
//! ```
//!
//! Tool lookup: `FCRS_GMSH` / `FCRS_CCX`, then `PATH`, then FreeCAD.app's bundled binaries.

pub mod fmt;
pub mod frd;
pub mod inp;
pub mod material;
pub mod mesh;
pub mod model;
pub mod resolve;
pub mod results;
pub mod solve;
pub mod tools;

pub use frd::{parse_dat_frequencies, parse_dat_totals, parse_frd, Frd, FrdBlock};
pub use inp::{write_inp, InpOptions};
pub use material::Material;
pub use mesh::{parse_msh, write_geo, FemMesh, SurfaceMesh, VolumeElement};
pub use model::{
    Analysis, AnalysisType, Constraint, ConstraintKind, ForceDirection, HeatFluxKind, MaterialAssignment, MeshRefinement, MeshSettings,
    ShapeSource, Signature, SolverSettings, SubKind, SubRef, STANDARD_GRAVITY,
};
pub use resolve::{resolve, ConstraintData, Resolved};
pub use results::{Extreme, Field, Mode, Results};
pub use solve::{mesh_analysis, prepare_geometry, read_results, run_ccx, solve, write_input, RunOptions, Solution, JOB_NAME};

/// Errors of the FEM pipeline.
#[derive(Debug, thiserror::Error)]
pub enum FemError {
    #[error("{tool} not found: {detail}")]
    ToolMissing { tool: &'static str, detail: String },
    #[error("{tool} failed: {detail}")]
    ToolFailed { tool: &'static str, detail: String },
    #[error("geometry: {0}")]
    Geometry(String),
    #[error("mesh: {0}")]
    Mesh(String),
    #[error("reference: {0}")]
    Reference(String),
    #[error("analysis: {0}")]
    Model(String),
    #[error("results: {0}")]
    Results(String),
    #[error(transparent)]
    Io(#[from] std::io::Error),
}
