//! The analysis description (what FreeCAD keeps as Analysis container children: solver settings,
//! material objects, mesh object and constraint objects). Everything here is plain serde data so
//! an app can store it inside its document.

use std::path::PathBuf;

use serde::{Deserialize, Serialize};

use crate::material::Material;

/// Where the solid to analyse comes from.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ShapeSource {
    /// An OpenCASCADE BREP file (what FreeCAD hands to Gmsh).
    Brep(PathBuf),
    /// A STEP file (read with OCCT and converted to BREP for Gmsh).
    Step(PathBuf),
    /// An in-memory shape (e.g. a Part Design body's tip). Not serialised.
    #[serde(skip)]
    Shape(fc_occt::Shape),
}

/// Kind of a referenced sub-element.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum SubKind {
    Vertex,
    Edge,
    Face,
    Solid,
}

impl SubKind {
    /// Gmsh entity dimension.
    pub fn dim(self) -> i32 {
        match self {
            SubKind::Vertex => 0,
            SubKind::Edge => 1,
            SubKind::Face => 2,
            SubKind::Solid => 3,
        }
    }
    pub fn prefix(self) -> &'static str {
        match self {
            SubKind::Vertex => "Vertex",
            SubKind::Edge => "Edge",
            SubKind::Face => "Face",
            SubKind::Solid => "Solid",
        }
    }
}

/// Geometric fingerprint of a sub-element, used to find it again when the topological index
/// (`Face3`) moved after a model change, FreeCAD's toponaming problem.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Signature {
    /// Centre of mass of the face / edge, or the vertex position.
    pub center: [f64; 3],
    /// Area for faces, length for edges, 0 for vertices.
    pub size: f64,
    /// Plane normal for planar faces (sign ignored when matching).
    #[serde(default)]
    pub normal: Option<[f64; 3]>,
}

impl Signature {
    /// Signature of a B-rep face as reported by fc-occt.
    pub fn from_face_info(fi: &fc_occt::FaceInfo) -> Self {
        Signature { center: fi.center, size: fi.area, normal: if fi.kind == 0 { Some(fi.normal) } else { None } }
    }
}

/// A FreeCAD-style sub-element reference: `Face3`, `Edge12`, `Vertex1`, `Solid1`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SubRef {
    pub kind: SubKind,
    /// 1-based index (TopExp map order, same numbering as FreeCAD and Gmsh's OCC kernel).
    pub index: usize,
    #[serde(default)]
    pub signature: Option<Signature>,
}

impl SubRef {
    pub fn face(index: usize) -> Self {
        SubRef { kind: SubKind::Face, index, signature: None }
    }
    pub fn edge(index: usize) -> Self {
        SubRef { kind: SubKind::Edge, index, signature: None }
    }
    pub fn vertex(index: usize) -> Self {
        SubRef { kind: SubKind::Vertex, index, signature: None }
    }
    pub fn solid(index: usize) -> Self {
        SubRef { kind: SubKind::Solid, index, signature: None }
    }
    pub fn with_signature(mut self, s: Signature) -> Self {
        self.signature = Some(s);
        self
    }
    /// Face reference with the signature taken from the shape (robust against renumbering).
    pub fn face_of(shape: &fc_occt::Shape, index: usize) -> Option<Self> {
        let fi = shape.face_info(index).ok()?;
        Some(SubRef::face(index).with_signature(Signature::from_face_info(&fi)))
    }
    /// Parse "Face3" / "Edge1" / "Vertex2" / "Solid1".
    pub fn parse(name: &str) -> Option<Self> {
        for kind in [SubKind::Vertex, SubKind::Edge, SubKind::Face, SubKind::Solid] {
            if let Some(rest) = name.strip_prefix(kind.prefix()) {
                if let Ok(i) = rest.parse::<usize>() {
                    if i > 0 {
                        return Some(SubRef { kind, index: i, signature: None });
                    }
                }
            }
        }
        None
    }
    /// FreeCAD element name, e.g. "Face3".
    pub fn name(&self) -> String {
        format!("{}{}", self.kind.prefix(), self.index)
    }
}

/// Mesh settings mirroring FreeCAD's `Fem::FemMeshGmsh` properties.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct MeshSettings {
    /// 1 = linear tetrahedra (C3D4), 2 = quadratic (C3D10).
    pub order: u8,
    /// CharacteristicLengthMax in mm; 0 = automatic (FreeCAD writes 1e+22).
    pub max_size: f64,
    /// CharacteristicLengthMin in mm.
    pub min_size: f64,
    /// MeshSizeFromCurvature (elements per 2*pi); 0 disables. FreeCAD default 12.
    pub size_from_curvature: u32,
    /// Local refinements (FreeCAD MeshRegion objects).
    pub refinements: Vec<MeshRefinement>,
    /// Mesh.Optimize (FreeCAD OptimizeStd, default true).
    pub optimize: bool,
    /// Mesh.OptimizeNetgen (default false).
    pub optimize_netgen: bool,
    /// Gmsh 2D algorithm number (FreeCAD "Automatic" = 2).
    pub algorithm_2d: u32,
    /// Gmsh 3D algorithm number (FreeCAD "Automatic" = 1, Delaunay).
    pub algorithm_3d: u32,
    /// Mesh.SecondOrderLinear: midside nodes on straight lines instead of on the geometry.
    pub second_order_linear: bool,
    /// General.NumThreads. FreeCAD uses all cores; we default to 1 because Gmsh's threaded
    /// meshing is not deterministic.
    pub threads: u32,
    /// Geometry.Tolerance (FreeCAD GeometryTolerance, default 1e-6).
    pub geometry_tolerance: f64,
    /// `Coherence Mesh` (FreeCAD CoherenceMesh, default true): merge duplicate mesh nodes.
    /// Separate parts joined by ties or contact are meshed without it, so nodes of touching
    /// faces stay distinct.
    pub coherence: bool,
}

impl Default for MeshSettings {
    fn default() -> Self {
        MeshSettings {
            order: 2,
            max_size: 0.0,
            min_size: 0.0,
            size_from_curvature: 12,
            refinements: Vec::new(),
            optimize: true,
            optimize_netgen: false,
            algorithm_2d: 2,
            algorithm_3d: 1,
            second_order_linear: false,
            threads: 1,
            geometry_tolerance: 1e-6,
            coherence: true,
        }
    }
}

/// Local element size on faces / edges / vertices (FreeCAD `MeshRegion`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MeshRefinement {
    pub refs: Vec<SubRef>,
    /// Characteristic length in mm.
    pub size: f64,
}

/// Direction of a force constraint.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ForceDirection {
    /// A fixed direction (normalised by the writer), like FreeCAD's DirectionVector.
    Vector([f64; 3]),
    /// Along the outward face normal (inward when `reversed`), like picking the loaded face
    /// itself as FreeCAD's Direction. Curved faces get per-element normals.
    Normal { reversed: bool },
}

/// Heat flux boundary condition (FreeCAD ConstraintHeatflux).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum HeatFluxKind {
    /// Surface heat flux in W/m^2 into the body (DFLUX).
    Flux(f64),
    /// Convection: film coefficient in W/m^2/K and ambient (sink) temperature in K (FILM).
    Convection { film_coefficient: f64, ambient_temperature: f64 },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ConstraintKind {
    /// All translations fixed (FreeCAD ConstraintFixed).
    Fixed,
    /// Total force in N spread over the referenced faces (area weighted), edges (length
    /// weighted) or vertices (equally), like FreeCAD's ConstraintForce.
    Force { force: f64, direction: ForceDirection },
    /// Pressure in MPa acting against the face normal (into the solid); `reversed` = suction.
    Pressure { pressure: f64, reversed: bool },
    /// Prescribed displacement per axis in mm; `None` = free.
    Displacement { x: Option<f64>, y: Option<f64>, z: Option<f64> },
    /// Self weight: gravity magnitude in mm/s^2 (FreeCAD default 9806.65) along `direction`.
    SelfWeight { gravity: f64, direction: [f64; 3] },
    /// Fixed temperature in K on the referenced sub-elements.
    Temperature { temperature: f64 },
    /// Concentrated heat flux in W shared by the referenced nodes (FreeCAD CFlux).
    HeatSource { power: f64 },
    /// Heat flux / convection on faces.
    HeatFlux(HeatFluxKind),
    /// Initial temperature in K of the whole body (no references).
    InitialTemperature { temperature: f64 },
}

impl ConstraintKind {
    /// FreeCAD's default object name prefix.
    pub fn default_name(&self) -> &'static str {
        match self {
            ConstraintKind::Fixed => "ConstraintFixed",
            ConstraintKind::Force { .. } => "ConstraintForce",
            ConstraintKind::Pressure { .. } => "ConstraintPressure",
            ConstraintKind::Displacement { .. } => "ConstraintDisplacement",
            ConstraintKind::SelfWeight { .. } => "ConstraintSelfWeight",
            ConstraintKind::Temperature { .. } | ConstraintKind::HeatSource { .. } => "ConstraintTemperature",
            ConstraintKind::HeatFlux(_) => "ConstraintHeatflux",
            ConstraintKind::InitialTemperature { .. } => "ConstraintInitialTemperature",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Constraint {
    /// Object name; becomes the CalculiX set name (FreeCAD: "ConstraintFixed", "ConstraintFixed001").
    pub name: String,
    #[serde(default)]
    pub refs: Vec<SubRef>,
    pub kind: ConstraintKind,
}

/// A material assigned to solids (FreeCAD MaterialSolid object).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MaterialAssignment {
    /// Object name, used as the CalculiX material name (FreeCAD default "FemMaterial").
    pub name: String,
    pub material: Material,
    /// Solids it applies to; empty = all (only valid when it is the only material).
    #[serde(default)]
    pub solids: Vec<SubRef>,
}

/// Analysis type, i.e. the CalculiX step procedure.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum AnalysisType {
    /// Linear static (*STATIC).
    Static,
    /// Eigenfrequencies (*FREQUENCY) with the number of modes and an optional band in Hz.
    Frequency { modes: u32, low_hz: f64, high_hz: f64 },
    /// Coupled temperature-displacement (*COUPLED TEMPERATURE-DISPLACEMENT).
    Thermomech { steady_state: bool },
}

/// CalculiX solver settings (subset of FreeCAD SolverCcxTools properties).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct SolverSettings {
    /// *STEP INC= (FreeCAD IncrementsMaximum, default 2000).
    pub increments_maximum: u32,
    pub time_initial_increment: f64,
    pub time_period: f64,
    pub time_minimum_increment: f64,
    pub time_maximum_increment: f64,
    /// Matrix solver: None = ccx default, or "SPOOLES", "PARDISO", "PASTIX", "ITERATIVE SCALING", ...
    pub matrix_solver: Option<String>,
    /// Threads for ccx (OMP_NUM_THREADS / CCX_NPROC_EQUATION_SOLVER); 0 = ccx default.
    pub threads: u32,
    /// Geometrical nonlinearity (FreeCAD GeometricalNonlinearity = "nonlinear": `*STEP, NLGEOM`).
    pub nonlinear_geometry: bool,
}

impl Default for SolverSettings {
    fn default() -> Self {
        SolverSettings {
            increments_maximum: 2000,
            time_initial_increment: 1.0,
            time_period: 1.0,
            time_minimum_increment: 1e-5,
            time_maximum_increment: 1.0,
            matrix_solver: None,
            threads: 0,
            nonlinear_geometry: false,
        }
    }
}

/// A complete FEM analysis.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Analysis {
    /// Analysis name (written into the .inp footer).
    pub name: String,
    /// Name of the analysed shape object, used in load comments ("node loads on shape: Body:Face2").
    #[serde(default = "default_shape_name")]
    pub shape_name: String,
    pub shape: ShapeSource,
    pub analysis_type: AnalysisType,
    pub materials: Vec<MaterialAssignment>,
    #[serde(default)]
    pub mesh: MeshSettings,
    #[serde(default)]
    pub constraints: Vec<Constraint>,
    #[serde(default)]
    pub solver: SolverSettings,
    /// Surface interactions between faces: ties (bonded) and contact (FreeCAD ConstraintTie /
    /// ConstraintContact).
    #[serde(default)]
    pub interactions: Vec<Interaction>,
    /// The parts of a multi-solid (assembly) analysis and the solids / faces of the analysed
    /// shape they became (see [`crate::assembly`]). Empty for a single part.
    #[serde(default)]
    pub parts: Vec<PartMap>,
}

impl Analysis {
    /// Static analysis of `shape` with one material, default mesh settings, no constraints.
    pub fn new(shape: ShapeSource, material: Material) -> Self {
        Analysis {
            name: "Analysis".into(),
            shape_name: default_shape_name(),
            shape,
            analysis_type: AnalysisType::Static,
            materials: vec![MaterialAssignment { name: "FemMaterial".into(), material, solids: Vec::new() }],
            mesh: MeshSettings::default(),
            constraints: Vec::new(),
            solver: SolverSettings::default(),
            interactions: Vec::new(),
            parts: Vec::new(),
        }
    }

    /// Add a surface interaction named like FreeCAD would (ConstraintTie, ConstraintTie001, ...).
    /// `slave` faces are FreeCAD's first references (dependent surface), `master` its last
    /// reference (independent surface).
    pub fn add_interaction(&mut self, kind: InteractionKind, slave: Vec<SubRef>, master: Vec<SubRef>) -> &mut Interaction {
        let base = kind.default_name();
        let mut name = base.to_string();
        let mut n = 0;
        while self.interactions.iter().any(|c| c.name == name) {
            n += 1;
            name = format!("{base}{n:03}");
        }
        self.interactions.push(Interaction { name, slave, master, kind });
        self.interactions.last_mut().unwrap()
    }
    /// Bond `slave` to `master` (CalculiX `*TIE`). `tolerance` (mm) is the largest gap that is
    /// still tied.
    pub fn tie(&mut self, slave: Vec<SubRef>, master: Vec<SubRef>, tolerance: f64) -> &mut Self {
        self.add_interaction(InteractionKind::Tie { tolerance, adjust: false }, slave, master);
        self
    }
    /// Penalty contact between `slave` and `master` (CalculiX `*CONTACT PAIR`, surface to
    /// surface) with a linear pressure-overclosure `slope` in MPa/mm and optional friction.
    pub fn contact(&mut self, slave: Vec<SubRef>, master: Vec<SubRef>, slope: f64, friction: Option<Friction>) -> &mut Self {
        self.add_interaction(InteractionKind::Contact { behavior: ContactBehavior::Linear, slope, adjust: 0.0, friction }, slave, master);
        self
    }
    /// Index of the part that owns `Solid<solid>` (multi-solid analyses).
    pub fn part_of_solid(&self, solid: usize) -> Option<usize> {
        self.parts.iter().position(|p| p.solids.contains(&solid))
    }
    /// Part by name.
    pub fn part(&self, name: &str) -> Option<&PartMap> {
        self.parts.iter().find(|p| p.name == name)
    }

    /// Add a constraint named like FreeCAD would (ConstraintFixed, ConstraintFixed001, ...).
    pub fn add(&mut self, kind: ConstraintKind, refs: Vec<SubRef>) -> &mut Constraint {
        let base = kind.default_name();
        let mut name = base.to_string();
        let mut n = 0;
        while self.constraints.iter().any(|c| c.name == name) {
            n += 1;
            name = format!("{base}{n:03}");
        }
        self.constraints.push(Constraint { name, refs, kind });
        self.constraints.last_mut().unwrap()
    }

    pub fn fixed(&mut self, refs: Vec<SubRef>) -> &mut Self {
        self.add(ConstraintKind::Fixed, refs);
        self
    }
    pub fn force(&mut self, refs: Vec<SubRef>, force: f64, direction: ForceDirection) -> &mut Self {
        self.add(ConstraintKind::Force { force, direction }, refs);
        self
    }
    pub fn pressure(&mut self, refs: Vec<SubRef>, pressure: f64, reversed: bool) -> &mut Self {
        self.add(ConstraintKind::Pressure { pressure, reversed }, refs);
        self
    }
    /// Self weight with standard gravity (9806.65 mm/s^2) along `direction`.
    pub fn self_weight(&mut self, direction: [f64; 3]) -> &mut Self {
        self.add(ConstraintKind::SelfWeight { gravity: STANDARD_GRAVITY, direction }, Vec::new());
        self
    }

    pub fn to_json(&self) -> String {
        serde_json::to_string_pretty(self).unwrap()
    }
    pub fn from_json(s: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(s)
    }
}

/// Pressure-overclosure law of a contact (FreeCAD ConstraintContact SurfaceBehavior).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ContactBehavior {
    /// Hard contact (Lagrange-like, no slope).
    Hard,
    /// Linear penalty with the given slope (FreeCAD default).
    Linear,
    /// Tied contact with the given slope.
    Tied,
}

/// Coulomb friction of a contact (FreeCAD Friction / FrictionCoefficient / StickSlope).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Friction {
    pub coefficient: f64,
    /// Stick slope in MPa/mm (FreeCAD default: slope / 10).
    pub stick_slope: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum InteractionKind {
    /// Bonded faces (FreeCAD ConstraintTie): `*TIE, POSITION TOLERANCE=..`; `adjust` moves the
    /// slave nodes onto the master surface.
    Tie { tolerance: f64, adjust: bool },
    /// Surface-to-surface penalty contact (FreeCAD ConstraintContact). `slope` in MPa/mm,
    /// `adjust` (mm, 0 = off) closes initial gaps up to that size.
    Contact { behavior: ContactBehavior, slope: f64, adjust: f64, friction: Option<Friction> },
}

impl InteractionKind {
    /// FreeCAD's default object name prefix.
    pub fn default_name(&self) -> &'static str {
        match self {
            InteractionKind::Tie { .. } => "ConstraintTie",
            InteractionKind::Contact { .. } => "ConstraintContact",
        }
    }
}

/// A tie or contact between two face sets.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Interaction {
    /// Object name; CalculiX surfaces are named `TIE_DEP<name>` / `TIE_IND<name>` (tie) and
    /// `DEP<name>` / `IND<name>` (contact), as FreeCAD writes them.
    pub name: String,
    /// Dependent (slave) faces.
    pub slave: Vec<SubRef>,
    /// Independent (master) faces.
    pub master: Vec<SubRef>,
    pub kind: InteractionKind,
}

/// One part of a multi-solid analysis: which solids and faces of the analysed shape it became.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PartMap {
    pub name: String,
    /// `Solid<n>` indices of the analysed shape.
    pub solids: Vec<usize>,
    /// (face index in the part's own shape, `Face<n>` index in the analysed shape). A part face
    /// split by BooleanFragments appears several times.
    #[serde(default)]
    pub faces: Vec<(usize, usize)>,
}

impl PartMap {
    /// Faces of the analysed shape that came from the part's own `Face<local>`.
    pub fn faces_of(&self, local: usize) -> Vec<usize> {
        self.faces.iter().filter(|f| f.0 == local).map(|f| f.1).collect()
    }
    /// All faces of the analysed shape that belong to this part.
    pub fn all_faces(&self) -> Vec<usize> {
        let mut v: Vec<usize> = self.faces.iter().map(|f| f.1).collect();
        v.sort_unstable();
        v.dedup();
        v
    }
}

/// Standard gravity in mm/s^2 (FreeCAD femtools.constants.gravity()).
pub const STANDARD_GRAVITY: f64 = 9806.65;

fn default_shape_name() -> String {
    "Geometry".into()
}
