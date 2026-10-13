//! FEM workbench (FreeCAD's FEM with Gmsh and CalculiX, through `fc-fem`) on the open part.
//!
//! - Analyses are document objects: they are saved in the `.fcrs` JSON (`fem_analyses`, next
//!   to the part or workspace data; older files simply have none) and are part of undo.
//! - Meshing and solving run on a worker thread; progress shows in the Tasks panel and the
//!   status bar, and the results replace the body in the 3D view (see `fem_ui.rs`).

use super::*;
use fc_fem::{Analysis, AnalysisType, ConstraintKind, FemMesh, Field, ForceDirection, HeatFluxKind, Material, Results, ShapeSource, SubKind, SubRef, SurfaceMesh};
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};

/// One analysis container (FreeCAD's `Fem::FemAnalysis` with its children) for a part.
#[derive(Clone, Serialize, Deserialize)]
pub struct FemDoc {
    /// Name of the analysed part (Body).
    pub part: String,
    /// Constraints, material, mesh settings and solver settings; the shape is filled in at run time.
    pub analysis: Analysis,
    /// A SolverCcxTools object exists.
    #[serde(default = "yes")]
    pub solver: bool,
    /// A FEMMeshGmsh object exists.
    #[serde(default)]
    pub mesh: bool,
    /// An analysis of an assembly (`part` is then the assembly's name): how its parts are
    /// joined and their materials. The analysed shape is rebuilt from the assembly.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub asm: Option<AsmFem>,
    #[serde(skip)]
    pub rt: FemRt,
}

/// Settings of an assembly analysis (FreeCAD: BooleanFragments or a compound of the parts,
/// a MaterialSolid per part).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AsmFem {
    pub joining: fc_fem::Joining,
    /// Material per part (instance path name, "Sub.Bolt").
    pub materials: Vec<(String, Material)>,
}

fn yes() -> bool {
    true
}

/// What a run produced (not saved; re-run after opening a file).
#[derive(Clone, Default)]
pub struct FemRt {
    pub mesh: Option<Arc<FemMesh>>,
    /// Outer surface of `mesh` (display).
    pub surface: Option<Arc<SurfaceMesh>>,
    pub results: Option<Arc<Results>>,
    /// Gmsh and CalculiX output of the last run.
    pub log: String,
    /// The last run failed with this message.
    pub error: Option<String>,
    /// Seconds the last run took.
    pub elapsed: f64,
    /// Mesh settings of `mesh` (a change asks for a new mesh).
    pub meshed_with: Option<fc_fem::MeshSettings>,
    /// Assembly analyses: the analysed shape (BooleanFragments or compound of the parts).
    pub shape: Option<fc_occt::Shape>,
    /// Assembly analyses: maxima per part of the last results.
    pub part_results: Option<Vec<fc_fem::PartResult>>,
}

/// FEM toolbar commands.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FemCmd {
    Analysis,
    Material,
    Mesh,
    ClearMesh,
    Constraint(CKind),
    Solver,
    Run,
    Results,
    Purge,
}

/// Constraint types of the toolbar.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CKind {
    Fixed,
    Displacement,
    Force,
    Pressure,
    SelfWeight,
    InitialTemperature,
    HeatFlux,
    Temperature,
}

pub const MECH: [CKind; 5] = [CKind::Fixed, CKind::Displacement, CKind::Force, CKind::Pressure, CKind::SelfWeight];
pub const THERMAL: [CKind; 3] = [CKind::InitialTemperature, CKind::HeatFlux, CKind::Temperature];

impl CKind {
    pub fn of(k: &ConstraintKind) -> CKind {
        match k {
            ConstraintKind::Fixed => CKind::Fixed,
            ConstraintKind::Force { .. } => CKind::Force,
            ConstraintKind::Pressure { .. } => CKind::Pressure,
            ConstraintKind::Displacement { .. } => CKind::Displacement,
            ConstraintKind::SelfWeight { .. } => CKind::SelfWeight,
            ConstraintKind::Temperature { .. } | ConstraintKind::HeatSource { .. } => CKind::Temperature,
            ConstraintKind::HeatFlux(_) => CKind::HeatFlux,
            ConstraintKind::InitialTemperature { .. } => CKind::InitialTemperature,
        }
    }
    /// (icon, toolbar id, title, tooltip)
    pub fn info(self) -> (&'static str, &'static str, &'static str, &'static str) {
        match self {
            CKind::Fixed => ("FEM_ConstraintFixed", "fem_fixed", "Fixed Boundary Condition", "Creates a fixed boundary condition: the selected faces, edges or vertices cannot move"),
            CKind::Displacement => ("FEM_ConstraintDisplacement", "fem_displacement", "Displacement Boundary Condition", "Creates a displacement boundary condition: prescribed or free translation per axis"),
            CKind::Force => ("FEM_ConstraintForce", "fem_force", "Force Load", "Creates a force load: a total force spread over the selected faces, edges or vertices"),
            CKind::Pressure => ("FEM_ConstraintPressure", "fem_pressure", "Pressure Load", "Creates a pressure load acting on the selected faces"),
            CKind::SelfWeight => ("FEM_ConstraintSelfWeight", "fem_selfweight", "Gravity Load", "Creates a gravity load (self weight) on the whole body"),
            CKind::InitialTemperature => ("FEM_ConstraintInitialTemperature", "fem_inittemp", "Initial Temperature", "Creates an initial temperature of the body (thermomechanical analyses)"),
            CKind::HeatFlux => ("FEM_ConstraintHeatflux", "fem_heatflux", "Heat Flux Load", "Creates a heat flux or convection on the selected faces"),
            CKind::Temperature => ("FEM_ConstraintTemperature", "fem_temperature", "Temperature Boundary Condition", "Creates a fixed temperature on the selected faces, edges or vertices"),
        }
    }
    pub fn needs_refs(self) -> bool {
        !matches!(self, CKind::SelfWeight | CKind::InitialTemperature)
    }
    /// FreeCAD's defaults for a new object.
    pub fn default_kind(self) -> ConstraintKind {
        match self {
            CKind::Fixed => ConstraintKind::Fixed,
            CKind::Displacement => ConstraintKind::Displacement { x: None, y: None, z: None },
            CKind::Force => ConstraintKind::Force { force: 1.0, direction: ForceDirection::Normal { reversed: false } },
            CKind::Pressure => ConstraintKind::Pressure { pressure: 1.0, reversed: false },
            CKind::SelfWeight => ConstraintKind::SelfWeight { gravity: fc_fem::STANDARD_GRAVITY, direction: [0.0, 0.0, -1.0] },
            CKind::InitialTemperature => ConstraintKind::InitialTemperature { temperature: 300.0 },
            CKind::HeatFlux => ConstraintKind::HeatFlux(HeatFluxKind::Convection { film_coefficient: 10.0, ambient_temperature: 300.0 }),
            CKind::Temperature => ConstraintKind::Temperature { temperature: 300.0 },
        }
    }
    /// Which sub-elements the reference selector accepts.
    pub fn accepts(self, k: SubKind) -> bool {
        match self {
            CKind::Pressure | CKind::HeatFlux => k == SubKind::Face,
            _ => true,
        }
    }
}

/// A row of an analysis in the model tree.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FemItem {
    Analysis(usize),
    Solver(usize),
    Material(usize, usize),
    Mesh(usize),
    Constraint(usize, usize),
    Results(usize),
    ResultMesh(usize),
    /// A tie or contact between parts (assembly analyses).
    Interaction(usize, usize),
}

impl FemItem {
    pub fn analysis(self) -> usize {
        match self {
            FemItem::Analysis(a) | FemItem::Solver(a) | FemItem::Material(a, _) | FemItem::Mesh(a) | FemItem::Constraint(a, _) | FemItem::Results(a) | FemItem::ResultMesh(a) | FemItem::Interaction(a, _) => a,
        }
    }
}

/// What a click in the 3D view does in a constraint task.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PickMode {
    Add,
    Remove,
    Direction,
}

/// A FEM task panel.
pub enum FemTask {
    Constraint { a: usize, c: usize, created: bool, before: String, mode: PickMode },
    Material { a: usize, m: usize, created: bool, before: String },
    Mesh { a: usize, created: bool, before: String },
    Solver { a: usize, before: String, auto: bool },
    Results { a: usize },
    /// Assembly analysis setup: joining, materials per part, contacts.
    AsmSetup { a: usize, created: bool, before: String, contacts: Option<Result<Vec<fc_fem::ContactPair>, String>>, tol: f64 },
}

impl FemTask {
    pub fn analysis(&self) -> usize {
        match self {
            FemTask::Constraint { a, .. } | FemTask::Material { a, .. } | FemTask::Mesh { a, .. } | FemTask::Solver { a, .. } | FemTask::Results { a } | FemTask::AsmSetup { a, .. } => *a,
        }
    }
    fn before(&self) -> Option<&str> {
        match self {
            FemTask::Constraint { before, .. } | FemTask::Material { before, .. } | FemTask::Mesh { before, .. } | FemTask::Solver { before, .. } | FemTask::AsmSetup { before, .. } => Some(before),
            FemTask::Results { .. } => None,
        }
    }
    fn created(&self) -> bool {
        matches!(self, FemTask::Constraint { created: true, .. } | FemTask::Material { created: true, .. } | FemTask::Mesh { created: true, .. } | FemTask::AsmSetup { created: true, .. })
    }
}

enum JobMsg {
    Stage(String),
    Line(String),
    Meshed(FemMesh, String),
    Solved(Results),
    Failed(String),
}

/// A mesh or solve running on a worker thread.
pub struct FemJob {
    pub analysis: usize,
    pub mesh_only: bool,
    rx: mpsc::Receiver<JobMsg>,
    pub cancel: Arc<AtomicBool>,
    pub started: std::time::Instant,
    pub stage: String,
}

/// Result display settings (FreeCAD's result pipeline: field, warp factor, wireframe).
#[derive(Clone)]
pub struct ResultView {
    pub field: Field,
    pub auto_scale: bool,
    /// Deformation factor when not automatic.
    pub scale: f64,
    pub show_edges: bool,
    /// Eigenmode shown (frequency analyses).
    pub mode: usize,
    pub animate: bool,
    pub show_max: bool,
}

impl Default for ResultView {
    fn default() -> Self {
        ResultView { field: Field::VonMises, auto_scale: true, scale: 1.0, show_edges: false, mode: 0, animate: false, show_max: true }
    }
}

/// What the result object on screen shows, for hover read-out and the legend.
pub struct Shown {
    /// Index of the result/mesh object in the viewport.
    pub object: usize,
    /// Mesh node index of every display vertex.
    pub node_of_vertex: Vec<u32>,
    /// Per-node scalar (None for the plain mesh).
    pub values: Option<Vec<f64>>,
    pub min: f64,
    pub max: f64,
    pub max_node: usize,
    /// Displayed (deformed) node positions.
    pub positions: Vec<[f64; 3]>,
    pub scale: f64,
}

#[derive(Default)]
pub struct FemState {
    /// The FEM workbench is active (on the open part).
    pub on: bool,
    pub docs: Vec<FemDoc>,
    /// The active analysis (FreeCAD's active analysis), index into `docs`.
    pub active: Option<usize>,
    pub sel: Option<FemItem>,
    pub task: Option<FemTask>,
    pub job: Option<FemJob>,
    pub view: ResultView,
    /// Show the results instead of the body / mesh (tree eye).
    pub hide_results: bool,
    /// Show the mesh instead of the body (tree eye).
    pub hide_mesh: bool,
    pub shown: Option<Shown>,
    /// Value under the cursor on the result.
    pub hover_text: Option<String>,
    /// Viewport index of the first FEM object (result or mesh).
    pub fem_at: Option<usize>,
    pub anim_t0: f64,
}

/// Field choices: (field, label, unit).
pub fn field_label(f: Field) -> (&'static str, &'static str) {
    match f {
        Field::DisplacementMagnitude => ("Displacement Magnitude", "mm"),
        Field::DisplacementX => ("Displacement X", "mm"),
        Field::DisplacementY => ("Displacement Y", "mm"),
        Field::DisplacementZ => ("Displacement Z", "mm"),
        Field::VonMises => ("von Mises Stress", "MPa"),
        Field::PrincipalMax => ("Max Principal Stress", "MPa"),
        Field::PrincipalMin => ("Min Principal Stress", "MPa"),
        Field::MaxShear => ("Max Shear Stress (Tresca)", "MPa"),
        Field::Temperature => ("Temperature", "K"),
        Field::EquivalentStrain => ("Equivalent Plastic Strain", ""),
    }
}

pub fn fields_for(r: &Results) -> Vec<Field> {
    let mut v = vec![Field::DisplacementMagnitude, Field::DisplacementX, Field::DisplacementY, Field::DisplacementZ];
    if r.modes.is_empty() {
        v.extend([Field::VonMises, Field::PrincipalMax, Field::PrincipalMin, Field::MaxShear]);
    }
    if r.temperature.is_some() {
        v.push(Field::Temperature);
    }
    v
}

fn is_disp(f: Field) -> bool {
    matches!(f, Field::DisplacementMagnitude | Field::DisplacementX | Field::DisplacementY | Field::DisplacementZ)
}

/// Per-node values of `field` (mode shape `mode` for frequency results).
pub fn field_values(r: &Results, field: Field, mode: usize) -> Vec<f64> {
    if let (true, Some(m)) = (is_disp(field), r.modes.get(mode)) {
        return m
            .displacement
            .iter()
            .map(|d| match field {
                Field::DisplacementX => d[0],
                Field::DisplacementY => d[1],
                Field::DisplacementZ => d[2],
                _ => (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt(),
            })
            .collect();
    }
    r.scalar(field)
}

/// Deformation factor that makes the largest displacement 5 % of the model size.
pub fn auto_scale(r: &Results, mode: usize) -> f64 {
    let disp = r.modes.get(mode).map(|m| &m.displacement).unwrap_or(&r.displacement);
    let dmax = disp.iter().map(|d| (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()).fold(0.0, f64::max);
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for p in &r.coords {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let size = (0..3).map(|k| hi[k] - lo[k]).fold(0.0, f64::max);
    if dmax > 0.0 && size > 0.0 { nice_round(0.05 * size / dmax) } else { 1.0 }
}

/// Round to 1, 2 or 5 times a power of ten (readable factors like FreeCAD's spin box).
fn nice_round(v: f64) -> f64 {
    if v <= 0.0 || !v.is_finite() {
        return 1.0;
    }
    let e = v.log10().floor();
    let b = 10f64.powf(e);
    let m = v / b;
    let n = if m < 1.5 { 1.0 } else if m < 3.5 { 2.0 } else if m < 7.5 { 5.0 } else { 10.0 };
    n * b
}

/// FreeCAD's default color bar "Blue-Cyan-Green-Yellow-Red" at t in 0..1.
pub fn colormap(t: f64) -> [f32; 3] {
    let t = if t.is_finite() { t.clamp(0.0, 1.0) as f32 } else { 0.0 };
    const S: [[f32; 3]; 5] = [[0.0, 0.0, 1.0], [0.0, 1.0, 1.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0], [1.0, 0.0, 0.0]];
    let x = t * 4.0;
    let i = (x.floor() as usize).min(3);
    let f = x - i as f32;
    let (a, b) = (S[i], S[i + 1]);
    [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f]
}

/// Display triangles split per B-rep face (flat faces shade flat): positions, normals,
/// indices, tri_face (0-based like the body) and the node of every vertex.
fn split_surface(tris: &[[u32; 3]], tri_face: &[u32], pos: &[[f64; 3]]) -> (Vec<[f32; 3]>, Vec<[f32; 3]>, Vec<u32>, Vec<u32>, Vec<u32>) {
    let mut map: HashMap<(u32, u32), u32> = HashMap::new();
    let mut positions = vec![];
    let mut normals: Vec<[f64; 3]> = vec![];
    let mut nodes = vec![];
    let mut indices = Vec::with_capacity(tris.len() * 3);
    let mut faces = Vec::with_capacity(tris.len());
    for (t, tri) in tris.iter().enumerate() {
        let f = tri_face.get(t).copied().unwrap_or(0);
        let p = tri.map(|n| pos[n as usize]);
        let (u, v) = (sub(p[1], p[0]), sub(p[2], p[0]));
        let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
        for &node in tri {
            let i = *map.entry((node, f)).or_insert_with(|| {
                positions.push(pos[node as usize].map(|x| x as f32));
                normals.push([0.0; 3]);
                nodes.push(node);
                (positions.len() - 1) as u32
            });
            for k in 0..3 {
                normals[i as usize][k] += n[k];
            }
            indices.push(i);
        }
        faces.push(f.saturating_sub(1));
    }
    let normals = normals
        .into_iter()
        .map(|n| {
            let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt().max(1e-30);
            [(n[0] / l) as f32, (n[1] / l) as f32, (n[2] / l) as f32]
        })
        .collect();
    (positions, normals, indices, faces, nodes)
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// Element edges of the display surface (quadratic triangles come split in 4: their outer
/// half-edges make up the curved element edges) as line segments.
fn wire_edges(tris: &[[u32; 3]], quadratic: bool, pos: &[[f64; 3]]) -> Vec<SceneEdge> {
    let mut seen: HashSet<(u32, u32)> = HashSet::new();
    let mut out = vec![];
    let mut push = |a: u32, b: u32, out: &mut Vec<SceneEdge>| {
        let k = if a < b { (a, b) } else { (b, a) };
        if seen.insert(k) {
            out.push(SceneEdge { id: 0, points: vec![pos[a as usize].map(|x| x as f32), pos[b as usize].map(|x| x as f32)] });
        }
    };
    if quadratic && tris.len() % 4 == 0 {
        for g in tris.chunks_exact(4) {
            // [a, ab, ca], [ab, b, bc], [ca, bc, c], [ab, bc, ca]
            let (a, ab, ca) = (g[0][0], g[0][1], g[0][2]);
            let (b, bc, c) = (g[1][1], g[1][2], g[2][2]);
            for (x, y) in [(a, ab), (ab, b), (b, bc), (bc, c), (c, ca), (ca, a)] {
                push(x, y, &mut out);
            }
        }
    } else {
        for t in tris {
            for (x, y) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                push(x, y, &mut out);
            }
        }
    }
    out
}

/// Distinct light colors for the parts of an assembly analysis.
pub fn part_color(i: usize) -> [f32; 3] {
    const P: [[f32; 3]; 8] = [[0.62, 0.72, 0.88], [0.90, 0.72, 0.50], [0.62, 0.84, 0.62], [0.86, 0.62, 0.76], [0.85, 0.83, 0.55], [0.58, 0.82, 0.84], [0.78, 0.68, 0.90], [0.80, 0.80, 0.80]];
    P[i % P.len()]
}

/// FreeCAD's FEM mesh color (light gray) and result object base.
const MESH_COLOR: [f32; 4] = [0.80, 0.80, 0.80, 1.0];

impl App {
    // ----------------------------------------------------------------- access

    /// What the FEM workbench analyses: the open part, or the open assembly.
    pub(super) fn fem_part_name(&self) -> Option<String> {
        self.active_part().map(|p| self.part_name(p)).or_else(|| self.active_assembly().and_then(|a| self.ws.assembly(a)).map(|a| a.name.clone()))
    }

    /// Analyses of the open part or assembly (indices into `fem.docs`).
    pub(super) fn fem_analyses(&self) -> Vec<usize> {
        let Some(name) = self.fem_part_name() else { return vec![] };
        let asm = self.active_assembly().is_some();
        (0..self.fem.docs.len()).filter(|&i| self.fem.docs[i].part == name && self.fem.docs[i].asm.is_some() == asm).collect()
    }

    /// The FEM workbench is on an assembly.
    pub(super) fn fem_on_assembly(&self) -> bool {
        self.fem.on && self.active_assembly().is_some()
    }

    /// The analysed shape of the active analysis as known now (part: the body's solid;
    /// assembly: the BooleanFragments / compound built for it).
    pub(super) fn fem_cur_shape(&self) -> Option<&fc_occt::Shape> {
        if self.active_assembly().is_some() {
            self.fem_cur().and_then(|a| self.fem.docs[a].rt.shape.as_ref())
        } else {
            self.doc.result()
        }
    }

    /// Material per part from the parts' data (BOM material card, else Steel-Generic).
    fn fem_default_materials(&self, asm: fc_assembly::AssemblyId) -> Vec<(String, Material)> {
        self.ws
            .leaves(asm)
            .into_iter()
            .filter(|l| l.visible)
            .filter_map(|l| self.ws.part(l.part).filter(|e| e.shape().is_some()).map(|e| (l.name.clone(), fc_fem::assembly::part_material(&e.meta))))
            .collect()
    }

    /// (Re)build the analysed shape of assembly analysis `a`: BooleanFragments (bonded) or a
    /// compound (separate), the part map and one material per distinct part material.
    pub(super) fn fem_asm_build(&mut self, a: usize) -> Result<(), String> {
        let Some(aid) = self.active_assembly() else { return Err("no assembly is open".into()) };
        let Some(cfg) = self.fem.docs.get(a).and_then(|d| d.asm.clone()) else { return Ok(()) };
        let mats: HashMap<String, Material> = cfg.materials.iter().cloned().collect();
        let parts = fc_fem::assembly_parts(&self.ws, aid, &|leaf, e| mats.get(&leaf.name).cloned().unwrap_or_else(|| fc_fem::assembly::part_material(&e.meta))).map_err(|e| e.to_string())?;
        let built = Analysis::from_parts(&parts, cfg.joining).map_err(|e| e.to_string())?;
        let d = &mut self.fem.docs[a];
        d.analysis.parts = built.parts.clone();
        d.analysis.materials = built.materials.clone();
        d.analysis.shape_name = built.shape_name.clone();
        d.analysis.mesh.coherence = built.mesh.coherence;
        d.rt.shape = built.shape_in_memory().cloned();
        Ok(())
    }

    /// Make sure an assembly analysis has its shape (after opening a file or undo).
    pub(super) fn fem_ensure_shape(&mut self, a: usize) {
        if self.fem.docs.get(a).is_some_and(|d| d.asm.is_some() && d.rt.shape.is_none()) {
            if let Err(e) = self.fem_asm_build(a) {
                self.fem.docs[a].rt.error = Some(e.clone());
                self.status = e;
            }
        }
    }

    /// The active analysis of the open part.
    pub(super) fn fem_cur(&self) -> Option<usize> {
        let list = self.fem_analyses();
        self.fem.active.filter(|a| list.contains(a)).or(list.last().copied())
    }

    pub(super) fn fem_busy(&self) -> bool {
        self.fem.job.is_some()
    }

    // ------------------------------------------------------------ persistence

    /// Analyses as JSON (empty string when there are none).
    pub(super) fn fem_json(&self) -> String {
        if self.fem.docs.is_empty() {
            return String::new();
        }
        serde_json::to_string(&self.fem.docs).unwrap_or_default()
    }

    /// Bring back analyses from `fem_json`, keeping the run data of unchanged analyses.
    pub(super) fn fem_restore_json(&mut self, s: &str) {
        let mut docs: Vec<FemDoc> = if s.trim().is_empty() { vec![] } else { serde_json::from_str(s).unwrap_or_default() };
        let old = std::mem::take(&mut self.fem.docs);
        for d in &mut docs {
            if let Some(o) = old.iter().find(|o| o.part == d.part && o.analysis.name == d.analysis.name) {
                d.rt = o.rt.clone();
                if o.asm != d.asm {
                    // other joining or materials: the analysed shape is rebuilt when needed
                    d.rt.shape = None;
                }
            }
        }
        self.fem.docs = docs;
        self.fem_validate();
    }

    /// Analyses from a saved document's text (`fem_analyses`).
    pub(super) fn fem_load_text(&mut self, text: &str) {
        self.fem = FemState { on: self.fem.on, ..Default::default() };
        let Ok(v) = serde_json::from_str::<serde_json::Value>(text) else { return };
        if let Some(a) = v.get("fem_analyses") {
            match serde_json::from_value::<Vec<FemDoc>>(a.clone()) {
                Ok(d) => self.fem.docs = d,
                Err(e) => self.status = format!("FEM analyses could not be read: {e}"),
            }
        }
    }

    /// Add the analyses to the JSON that Save writes.
    pub(super) fn fem_save_into(&self, json: String) -> String {
        if self.fem.docs.is_empty() {
            return json;
        }
        let Ok(mut v) = serde_json::from_str::<serde_json::Value>(&json) else { return json };
        if let (Some(obj), Ok(a)) = (v.as_object_mut(), serde_json::to_value(&self.fem.docs)) {
            obj.insert("fem_analyses".into(), a);
        }
        serde_json::to_string_pretty(&v).unwrap_or(json)
    }

    /// Drop selections and tasks that point at analyses or objects that no longer exist.
    fn fem_validate(&mut self) {
        let n = self.fem.docs.len();
        if self.fem.active.is_some_and(|a| a >= n) {
            self.fem.active = None;
        }
        let ok = |s: &FemState, it: FemItem| -> bool {
            let Some(d) = s.docs.get(it.analysis()) else { return false };
            match it {
                FemItem::Constraint(_, c) => c < d.analysis.constraints.len(),
                FemItem::Material(_, m) => m < d.analysis.materials.len(),
                FemItem::Mesh(_) => d.mesh,
                FemItem::Solver(_) => d.solver,
                FemItem::Results(_) | FemItem::ResultMesh(_) => d.rt.results.is_some(),
                FemItem::Interaction(_, i) => i < d.analysis.interactions.len(),
                FemItem::Analysis(_) => true,
            }
        };
        if self.fem.sel.is_some_and(|s| !ok(&self.fem, s)) {
            self.fem.sel = None;
        }
    }

    // ------------------------------------------------------------- workbench

    /// Switch to the FEM workbench (on the open part, or the last part when an assembly is open).
    pub(super) fn fem_enter(&mut self) {
        if !self.idle() {
            return;
        }
        // on an assembly, FEM analyses the assembly (multi-solid analysis)
        if self.active_part().is_none() && self.active_assembly().is_none() {
            let p = self.last_part.filter(|p| self.ws.part(*p).is_some()).or(self.ws.parts.first().map(|p| p.id));
            if let Some(p) = p {
                self.switch_tab(Tab::Part(p));
            }
        }
        if self.active_part().is_none() && self.active_assembly().is_none() {
            return;
        }
        self.asm.explode_show = None;
        self.asm.highlight = None;
        self.fem.on = true;
        self.tree_sel.clear();
        self.rebuild_scene();
        self.status = match self.fem_cur() {
            Some(a) => format!("FEM: {} of {}", self.fem.docs[a].analysis.name, self.fem.docs[a].part),
            None => "FEM: create an analysis for this part with New Analysis".into(),
        };
    }

    pub(super) fn fem_leave(&mut self) {
        if self.fem.on {
            self.fem.on = false;
            self.fem.sel = None;
            self.fem.hover_text = None;
            self.rebuild_scene();
        }
    }

    // --------------------------------------------------------------- commands

    pub(super) fn fem_cmd(&mut self, c: FemCmd) {
        if !self.fem.on {
            self.fem_enter();
        }
        match c {
            FemCmd::Analysis => self.fem_new_analysis(),
            FemCmd::Material => self.fem_new_material(),
            FemCmd::Mesh => self.fem_mesh_cmd(),
            FemCmd::ClearMesh => self.fem_clear_mesh(),
            FemCmd::Constraint(k) => self.fem_new_constraint(k),
            FemCmd::Solver => self.fem_solver_cmd(),
            FemCmd::Run => self.fem_run_cmd(),
            FemCmd::Results => {
                if let Some(a) = self.fem_cur().filter(|a| self.fem.docs[*a].rt.results.is_some()) {
                    self.fem_open_task(FemItem::Results(a));
                }
            }
            FemCmd::Purge => self.fem_purge(),
        }
    }

    /// (icon, title, description, enabled)
    pub(super) fn fem_cmd_info(&self, c: FemCmd) -> (&'static str, &'static str, &'static str, bool) {
        let idle = self.idle() && (self.active_part().is_some() || self.active_assembly().is_some());
        let cur = self.fem_cur();
        let has = cur.is_some() && idle;
        let solid = match self.active_assembly() {
            Some(a) => self.ws.leaves(a).iter().any(|l| self.ws.part(l.part).is_some_and(|e| e.shape().is_some())),
            None => self.doc.result().is_some(),
        };
        let busy = self.fem_busy();
        match c {
            FemCmd::Analysis => ("FEM_Analysis", "Analysis Container", "Creates an analysis container with the CalculiX solver for the open part or assembly", idle && solid),
            FemCmd::Material => ("FEM_MaterialSolid", "Material for Solid", "Creates a FEM material for a solid", has),
            FemCmd::Mesh => ("FEM_MeshGmshFromShape", "Mesh From Shape by Gmsh", "Creates a FEM mesh from the part with Gmsh (or edits it)", has && !busy),
            FemCmd::ClearMesh => ("FEM_MeshClear", "Clear FEM Mesh", "Clears the mesh data of the FEM mesh object", has && cur.is_some_and(|a| self.fem.docs[a].rt.mesh.is_some()) && !busy),
            FemCmd::Constraint(k) => {
                let (icon, _, title, desc) = k.info();
                (icon, title, desc, has)
            }
            FemCmd::Solver => ("FEM_SolverStandard", "Solver CalculiX", "Creates the CalculiX solver object (analysis type, eigenmodes) or opens its task", has),
            FemCmd::Run => ("FEM_SolverRun", "Run Solver", "Meshes the part with Gmsh, writes the CalculiX input, runs ccx and loads the results", has && !busy),
            FemCmd::Results => ("FEM_ResultShow", "Show Result", "Shows the result: field, deformation scale and mesh edges", has && cur.is_some_and(|a| self.fem.docs[a].rt.results.is_some())),
            FemCmd::Purge => ("FEM_ResultsPurge", "Purge Results", "Deletes all results of the active analysis", has && !busy && cur.is_some_and(|a| self.fem.docs[a].rt.results.is_some() || self.fem.docs[a].rt.mesh.is_some())),
        }
    }

    /// Unique analysis name like FreeCAD ("Analysis", "Analysis001").
    fn fem_unique_name(&self, base: &str) -> String {
        let taken = |n: &str| self.fem.docs.iter().any(|d| d.analysis.name == n);
        if !taken(base) {
            return base.to_string();
        }
        (1..).map(|k| format!("{base}{k:03}")).find(|n| !taken(n)).unwrap()
    }

    fn fem_new_analysis(&mut self) {
        let Some(part) = self.fem_part_name() else { return };
        if let Some(aid) = self.active_assembly() {
            return self.fem_new_asm_analysis(aid, part);
        }
        if self.doc.result().is_none() {
            self.dialog = Some(Dialog::Message("The part has no solid yet. Create a body (for example a Pad) first.".into()));
            return;
        }
        self.snapshot();
        let mut a = Analysis::new(ShapeSource::Brep(PathBuf::new()), Material::steel_generic());
        a.name = self.fem_unique_name("Analysis");
        a.shape_name = part.clone();
        // FreeCAD's New Analysis creates the container and the solver; material and mesh are added separately
        a.materials.clear();
        a.analysis_type = AnalysisType::Static;
        self.fem.docs.push(FemDoc { part, analysis: a, solver: true, mesh: false, asm: None, rt: FemRt::default() });
        let i = self.fem.docs.len() - 1;
        self.fem.active = Some(i);
        self.fem.sel = Some(FemItem::Analysis(i));
        self.dirty = true;
        self.rebuild_scene();
        self.status = format!("Created {} (active analysis) with SolverCcxTools", self.fem.docs[i].analysis.name);
    }

    /// New analysis of the open assembly: bonded parts with their own materials; the setup
    /// task opens to choose how the parts are joined, edit materials and find contacts.
    fn fem_new_asm_analysis(&mut self, aid: fc_assembly::AssemblyId, name: String) {
        self.snapshot();
        let before = self.undo.last().cloned().unwrap_or_default();
        let mut an = Analysis::new(ShapeSource::Brep(PathBuf::new()), Material::steel_generic());
        an.name = self.fem_unique_name("Analysis");
        an.analysis_type = AnalysisType::Static;
        let materials = self.fem_default_materials(aid);
        self.fem.docs.push(FemDoc { part: name, analysis: an, solver: true, mesh: false, asm: Some(AsmFem { joining: fc_fem::Joining::Fragments { fuzzy: 0.0 }, materials }), rt: FemRt::default() });
        let i = self.fem.docs.len() - 1;
        if let Err(e) = self.fem_asm_build(i) {
            self.fem.docs.pop();
            self.undo.pop();
            self.dialog = Some(Dialog::Message(format!("Could not set up the analysis: {e}")));
            return;
        }
        self.fem.active = Some(i);
        self.fem.sel = Some(FemItem::Analysis(i));
        self.fem.task = Some(FemTask::AsmSetup { a: i, created: true, before, contacts: None, tol: 0.1 });
        self.dirty = true;
        self.rebuild_scene();
        self.status = format!("Created {}: the parts are bonded (BooleanFragments); check materials and contacts", self.fem.docs[i].analysis.name);
    }

    fn fem_new_material(&mut self) {
        let Some(a) = self.fem_cur() else { return };
        if self.fem.docs[a].asm.is_some() {
            // assembly analyses: one material per part, in the setup task
            return self.fem_open_task(FemItem::Analysis(a));
        }
        self.snapshot();
        let before = self.undo.last().cloned().unwrap_or_default();
        let mats = &mut self.fem.docs[a].analysis.materials;
        let base = "MaterialSolid";
        let name = if mats.iter().any(|m| m.name == base) { (1..).map(|k| format!("{base}{k:03}")).find(|n| !mats.iter().any(|m| &m.name == n)).unwrap() } else { base.to_string() };
        mats.push(fc_fem::MaterialAssignment { name, material: Material::steel_generic(), solids: vec![] });
        let m = mats.len() - 1;
        self.dirty = true;
        self.fem.sel = Some(FemItem::Material(a, m));
        self.fem.task = Some(FemTask::Material { a, m, created: true, before });
        self.status = "Choose a material card or enter the values".into();
    }

    fn fem_mesh_cmd(&mut self) {
        let Some(a) = self.fem_cur() else { return };
        let created = !self.fem.docs[a].mesh;
        self.snapshot();
        let before = self.undo.last().cloned().unwrap_or_default();
        if created {
            self.fem.docs[a].mesh = true;
            self.dirty = true;
        }
        self.fem.sel = Some(FemItem::Mesh(a));
        self.fem.task = Some(FemTask::Mesh { a, created, before });
        self.status = "Set the element sizes and order, then Apply to run Gmsh".into();
    }

    fn fem_clear_mesh(&mut self) {
        let Some(a) = self.fem_cur() else { return };
        let rt = &mut self.fem.docs[a].rt;
        rt.mesh = None;
        rt.surface = None;
        rt.results = None;
        self.rebuild_scene();
        self.status = "Mesh cleared".into();
    }

    fn fem_new_constraint(&mut self, k: CKind) {
        let Some(a) = self.fem_cur() else { return };
        // geometry selected beforehand becomes the first references (FreeCAD does the same)
        let refs: Vec<SubRef> = if k.needs_refs() { self.selection.iter().filter(|p| p.object == 0).filter_map(|p| self.fem_subref(p)).filter(|r| k.accepts(r.kind)).collect() } else { vec![] };
        self.snapshot();
        let before = self.undo.last().cloned().unwrap_or_default();
        self.fem.docs[a].analysis.add(k.default_kind(), refs);
        let c = self.fem.docs[a].analysis.constraints.len() - 1;
        self.dirty = true;
        self.selection.clear();
        self.vp.set_selection(vec![]);
        self.fem.sel = Some(FemItem::Constraint(a, c));
        self.fem.task = Some(FemTask::Constraint { a, c, created: true, before, mode: PickMode::Add });
        self.rebuild_scene();
        self.status = if k.needs_refs() { "Click faces, edges or vertices of the part to add them to the references".into() } else { "Set the values and press OK".into() };
    }

    fn fem_solver_cmd(&mut self) {
        let Some(a) = self.fem_cur() else { return };
        if !self.fem.docs[a].solver {
            self.snapshot();
            self.fem.docs[a].solver = true;
            self.dirty = true;
        }
        self.fem_open_task(FemItem::Solver(a));
    }

    fn fem_run_cmd(&mut self) {
        let Some(a) = self.fem_cur() else { return };
        if self.fem.task.is_none() {
            self.snapshot();
            let before = self.undo.last().cloned().unwrap_or_default();
            self.fem.task = Some(FemTask::Solver { a, before, auto: true });
        }
        self.fem_start(a, false);
    }

    fn fem_purge(&mut self) {
        let Some(a) = self.fem_cur() else { return };
        let rt = &mut self.fem.docs[a].rt;
        rt.results = None;
        rt.error = None;
        if matches!(self.fem.sel, Some(FemItem::Results(_) | FemItem::ResultMesh(_))) {
            self.fem.sel = None;
        }
        if matches!(self.fem.task, Some(FemTask::Results { .. })) {
            self.fem.task = None;
        }
        self.rebuild_scene();
        self.status = "Results purged".into();
    }

    /// Open the task panel of a tree item (double-click / Edit).
    pub(super) fn fem_open_task(&mut self, it: FemItem) {
        if self.fem.task.is_some() || !self.idle() {
            return;
        }
        let a = it.analysis();
        self.fem.active = Some(a);
        self.fem.sel = Some(it);
        let snap = |app: &mut App| {
            app.snapshot();
            app.undo.last().cloned().unwrap_or_default()
        };
        self.fem.task = match it {
            FemItem::Constraint(a, c) => Some(FemTask::Constraint { a, c, created: false, before: snap(self), mode: PickMode::Add }),
            FemItem::Material(a, m) => Some(FemTask::Material { a, m, created: false, before: snap(self) }),
            FemItem::Mesh(a) => Some(FemTask::Mesh { a, created: false, before: snap(self) }),
            FemItem::Solver(a) => Some(FemTask::Solver { a, before: snap(self), auto: false }),
            FemItem::Results(a) | FemItem::ResultMesh(a) if self.fem.docs[a].rt.results.is_some() => Some(FemTask::Results { a }),
            FemItem::Analysis(a) | FemItem::Interaction(a, _) if self.fem.docs[a].asm.is_some() => Some(FemTask::AsmSetup { a, created: false, before: snap(self), contacts: None, tol: 0.1 }),
            _ => None,
        };
        self.fem.hide_results = false;
        self.rebuild_scene();
    }

    /// OK: keep the changes (one undo step, dropped when nothing changed).
    pub(super) fn fem_task_ok(&mut self) {
        let Some(t) = self.fem.task.take() else { return };
        if let Some(b) = t.before() {
            if !t.created() && self.state_json() == b {
                self.undo.pop();
            }
        }
        // the mesh task meshes on OK when there is no mesh yet (FreeCAD does the same)
        if let FemTask::Mesh { a, .. } = t {
            if self.fem.docs.get(a).is_some_and(|d| d.rt.mesh.is_none() || d.rt.meshed_with.as_ref() != Some(&d.analysis.mesh)) && !self.fem_busy() {
                self.fem_start(a, true);
            }
        }
        self.vp.force_hover(None);
        self.rebuild_scene();
        if !self.fem_busy() {
            self.status = "Ready".into();
        }
    }

    /// Cancel: restore the analysis as it was when the task opened (a new object is removed).
    pub(super) fn fem_task_cancel(&mut self) {
        if let Some(j) = &self.fem.job {
            if matches!(self.fem.task, Some(FemTask::Solver { .. })) {
                j.cancel.store(true, Ordering::Relaxed);
            }
        }
        let Some(t) = self.fem.task.take() else { return };
        if let Some(b) = t.before() {
            let b = b.to_string();
            self.restore(&b);
            self.undo.pop();
        }
        self.vp.force_hover(None);
        self.rebuild_scene();
        self.status = "Cancelled".into();
    }

    /// Delete the selected tree item.
    pub(super) fn fem_delete(&mut self, it: FemItem) {
        if !self.idle() || self.fem_busy() {
            return;
        }
        let a = it.analysis();
        if a >= self.fem.docs.len() {
            return;
        }
        self.snapshot();
        match it {
            FemItem::Analysis(_) => {
                self.fem.docs.remove(a);
                self.fem.active = None;
            }
            FemItem::Constraint(_, c) => {
                self.fem.docs[a].analysis.constraints.remove(c);
            }
            FemItem::Material(_, m) => {
                self.fem.docs[a].analysis.materials.remove(m);
            }
            FemItem::Mesh(_) => {
                let d = &mut self.fem.docs[a];
                d.mesh = false;
                d.rt.mesh = None;
                d.rt.surface = None;
            }
            FemItem::Solver(_) => self.fem.docs[a].solver = false,
            FemItem::Interaction(_, i) => {
                if i < self.fem.docs[a].analysis.interactions.len() {
                    self.fem.docs[a].analysis.interactions.remove(i);
                }
            }
            FemItem::Results(_) | FemItem::ResultMesh(_) => {
                self.undo.pop();
                self.fem_purge();
                return;
            }
        }
        self.fem.sel = None;
        self.dirty = true;
        self.fem_validate();
        self.rebuild_scene();
    }

    /// A pick on the body as an fc-fem reference (faces carry their geometric signature).
    pub(super) fn fem_subref(&self, p: &Pick) -> Option<SubRef> {
        let idx = p.id as usize + 1;
        Some(match p.kind {
            PickKind::Face => self.fem_cur_shape().and_then(|s| SubRef::face_of(s, idx)).unwrap_or(SubRef::face(idx)),
            PickKind::Edge => SubRef::edge(idx),
            PickKind::Vertex => SubRef::vertex(idx),
        })
    }

    /// Click on the body in a constraint task: add / remove a reference or take the direction.
    pub(super) fn fem_pick(&mut self, p: &Pick) {
        let Some(FemTask::Constraint { a, c, mode, .. }) = self.fem.task.as_ref() else { return };
        let (a, c, mode) = (*a, *c, *mode);
        let Some(r) = self.fem_subref(p) else { return };
        let shape = self.fem_cur_shape().cloned();
        let Some(con) = self.fem.docs.get_mut(a).and_then(|d| d.analysis.constraints.get_mut(c)) else { return };
        let k = CKind::of(&con.kind);
        match mode {
            PickMode::Direction => {
                let dir = match p.kind {
                    PickKind::Face => shape.as_ref().and_then(|s| s.face_info(p.id as usize + 1).ok()).filter(|f| f.kind == 0).map(|f| f.normal),
                    PickKind::Edge => shape.as_ref().and_then(|s| s.edge_info(p.id as usize + 1).ok()).filter(|e| e.kind == 0).map(|e| e.dir),
                    PickKind::Vertex => None,
                };
                match dir {
                    Some(d) => {
                        if let ConstraintKind::Force { direction, .. } = &mut con.kind {
                            *direction = ForceDirection::Vector(d);
                        }
                        self.status = format!("Direction: {} ({:.3}, {:.3}, {:.3})", r.name(), d[0], d[1], d[2]);
                        if let Some(FemTask::Constraint { mode, .. }) = self.fem.task.as_mut() {
                            *mode = PickMode::Add;
                        }
                    }
                    None => self.status = "Pick a flat face (its normal) or a straight edge for the direction".into(),
                }
            }
            PickMode::Add => {
                if !k.needs_refs() {
                    return;
                }
                if !k.accepts(r.kind) {
                    self.status = format!("{} only takes faces", k.info().2);
                    return;
                }
                if let Some(i) = con.refs.iter().position(|x| x.kind == r.kind && x.index == r.index) {
                    self.status = format!("{} is already a reference", con.refs[i].name());
                    return;
                }
                // FreeCAD: one kind of sub-element per constraint
                if con.refs.first().is_some_and(|f| f.kind != r.kind) {
                    self.status = format!("Only {}s can be added (the references already are {}s)", con.refs[0].kind.prefix().to_lowercase(), con.refs[0].kind.prefix().to_lowercase());
                    return;
                }
                self.status = format!("Added {}", r.name());
                con.refs.push(r);
            }
            PickMode::Remove => {
                if let Some(i) = con.refs.iter().position(|x| x.kind == r.kind && x.index == r.index) {
                    con.refs.remove(i);
                    self.status = format!("Removed {}", r.name());
                }
            }
        }
        self.dirty = true;
    }

    // -------------------------------------------------------------------- jobs

    /// Problems that keep a run from starting (FreeCAD's check_prerequisites).
    pub(super) fn fem_check(&self, a: usize, mesh_only: bool) -> Option<String> {
        let d = self.fem.docs.get(a)?;
        if d.asm.is_some() {
            if d.rt.shape.is_none() {
                return Some(d.rt.error.clone().unwrap_or_else(|| "The assembly has no parts with a solid to analyse.".into()));
            }
            if !mesh_only && matches!(d.asm.as_ref().map(|x| x.joining), Some(fc_fem::Joining::Separate)) && d.analysis.parts.len() > 1 && d.analysis.interactions.is_empty() {
                return Some("The parts are meshed separately but nothing joins them: add ties or contact (Detect contacts in the analysis setup), or bond them.".into());
            }
        } else if self.doc.result().is_none() {
            return Some("The part has no solid to analyse.".into());
        }
        if mesh_only {
            return None;
        }
        let an = &d.analysis;
        if !d.solver {
            return Some("No solver object defined in the analysis. Create one with Solver CalculiX.".into());
        }
        if !d.mesh {
            return Some("No mesh object defined in the analysis. Create one with Mesh From Shape by Gmsh.".into());
        }
        if an.materials.is_empty() {
            return Some("No material object defined in the analysis. Create one with Material for Solid.".into());
        }
        if let Some(c) = an.constraints.iter().find(|c| CKind::of(&c.kind).needs_refs() && c.refs.is_empty()) {
            return Some(format!("{} has no references. Select faces, edges or vertices in its task panel.", c.name));
        }
        let mech = |k: &ConstraintKind| matches!(k, ConstraintKind::Fixed | ConstraintKind::Displacement { .. });
        if !matches!(an.analysis_type, AnalysisType::Frequency { .. }) && !an.constraints.iter().any(|c| mech(&c.kind)) {
            return Some("The analysis needs a Fixed or Displacement boundary condition, else the part can move freely.".into());
        }
        if matches!(an.analysis_type, AnalysisType::Static) && !an.constraints.iter().any(|c| matches!(c.kind, ConstraintKind::Force { .. } | ConstraintKind::Pressure { .. } | ConstraintKind::SelfWeight { .. } | ConstraintKind::Displacement { .. })) {
            return Some("A static analysis needs a load: Force, Pressure, Gravity or a prescribed Displacement.".into());
        }
        if matches!(an.analysis_type, AnalysisType::Thermomech { .. }) {
            if an.materials.iter().any(|m| m.material.thermal_conductivity <= 0.0 || m.material.specific_heat <= 0.0) {
                return Some("A thermomechanical analysis needs the thermal conductivity and specific heat of the material.".into());
            }
            if !an.constraints.iter().any(|c| matches!(c.kind, ConstraintKind::InitialTemperature { .. })) {
                return Some("A thermomechanical analysis needs an Initial Temperature.".into());
            }
        }
        None
    }

    /// Mesh (and solve) analysis `a` on a worker thread.
    pub(super) fn fem_start(&mut self, a: usize, mesh_only: bool) {
        if self.fem_busy() {
            return;
        }
        self.fem_ensure_shape(a);
        if let Some(e) = self.fem_check(a, mesh_only) {
            self.fem.docs[a].rt.error = Some(e.clone());
            self.status = e;
            return;
        }
        // fail fast on missing tools, with where we looked
        let tools = if mesh_only { fc_fem::tools::gmsh().err() } else { fc_fem::tools::gmsh().err().or(fc_fem::tools::ccx().err()) };
        if let Some(e) = tools {
            let msg = format!("{e}");
            self.fem.docs[a].rt.error = Some(msg.clone());
            self.status = msg.lines().next().unwrap_or_default().to_string();
            return;
        }
        let shape = if self.fem.docs[a].asm.is_some() { self.fem.docs[a].rt.shape.clone() } else { self.doc.result().cloned() };
        let Some(shape) = shape else { return };
        let mut an = self.fem.docs[a].analysis.clone();
        an.shape = ShapeSource::Shape(shape);
        an.shape_name = self.fem.docs[a].part.clone();
        let settings = an.mesh.clone();
        let (tx, rx) = mpsc::channel();
        let cancel = Arc::new(AtomicBool::new(false));
        let cancel2 = cancel.clone();
        std::thread::spawn(move || fem_worker(an, mesh_only, tx, cancel2));
        let rt = &mut self.fem.docs[a].rt;
        rt.error = None;
        rt.log.clear();
        rt.meshed_with = Some(settings);
        if !mesh_only {
            rt.results = None;
            rt.part_results = None;
        }
        self.fem.job = Some(FemJob { analysis: a, mesh_only, rx, cancel, started: std::time::Instant::now(), stage: "Starting...".into() });
        self.status = if mesh_only { "Meshing with Gmsh...".into() } else { "Running the analysis...".into() };
        self.rebuild_scene();
    }

    /// Collect worker messages (every frame).
    pub(super) fn fem_poll(&mut self, ctx: &egui::Context) {
        let Some(job) = self.fem.job.as_mut() else { return };
        ctx.request_repaint_after(std::time::Duration::from_millis(60));
        let a = job.analysis;
        let mut finished: Option<Result<(), String>> = None;
        let mut new_mesh = false;
        let mut msgs = vec![];
        while let Ok(m) = job.rx.try_recv() {
            msgs.push(m);
        }
        let disconnected = msgs.is_empty() && matches!(job.rx.try_recv(), Err(mpsc::TryRecvError::Disconnected));
        let mesh_only = job.mesh_only;
        let elapsed = job.started.elapsed().as_secs_f64();
        for m in msgs {
            let Some(d) = self.fem.docs.get_mut(a) else { break };
            match m {
                JobMsg::Stage(s) => {
                    d.rt.log.push_str(&format!("--- {s}\n"));
                    if let Some(j) = self.fem.job.as_mut() {
                        j.stage = s;
                    }
                }
                JobMsg::Line(l) => {
                    d.rt.log.push_str(&l);
                    d.rt.log.push('\n');
                    cap_log(&mut d.rt.log, LOG_CAP);
                }
                JobMsg::Meshed(mesh, log) => {
                    let surface = mesh.boundary();
                    d.rt.log.push_str(&format!("Gmsh: {} nodes, {} elements ({})\n", mesh.node_count(), mesh.element_count(), if mesh.order == 2 { "quadratic C3D10" } else { "linear C3D4" }));
                    for l in log.lines().filter(|l| l.contains("Warning") || l.contains("Error")).take(20) {
                        d.rt.log.push_str(l);
                        d.rt.log.push('\n');
                    }
                    d.rt.mesh = Some(Arc::new(mesh));
                    d.rt.surface = Some(Arc::new(surface));
                    new_mesh = true;
                    if mesh_only {
                        finished = Some(Ok(()));
                    }
                }
                JobMsg::Solved(r) => {
                    // assembly analyses: maxima per part (fc-fem's part_results)
                    if let (true, Some(m), Some(sh)) = (d.asm.is_some(), d.rt.mesh.clone(), d.rt.shape.clone()) {
                        let mut an = d.analysis.clone();
                        an.shape = ShapeSource::Shape(sh);
                        let sol = fc_fem::Solution { mesh: (*m).clone(), results: r.clone(), work_dir: PathBuf::new(), gmsh_log: String::new(), ccx_log: String::new() };
                        d.rt.part_results = Some(fc_fem::part_results(&an, &sol));
                    }
                    d.rt.results = Some(Arc::new(r));
                    finished = Some(Ok(()));
                }
                JobMsg::Failed(e) => finished = Some(Err(e)),
            }
        }
        if finished.is_none() && disconnected {
            finished = Some(Err("the worker stopped unexpectedly".into()));
        }
        if new_mesh {
            self.rebuild_scene();
        }
        let Some(res) = finished else { return };
        let solved = res.is_ok() && !mesh_only;
        self.fem.job = None;
        let d = &mut self.fem.docs[a];
        d.rt.elapsed = elapsed;
        match res {
            Ok(()) if mesh_only => {
                let m = d.rt.mesh.clone().unwrap();
                self.status = format!("Meshed: {} nodes, {} elements in {elapsed:.1} s", m.node_count(), m.element_count());
            }
            Ok(()) => {
                d.rt.log.push_str(&format!("--- Finished in {elapsed:.1} s\n"));
                let r = d.rt.results.clone().unwrap();
                self.fem.view.mode = 0;
                if !fields_for(&r).contains(&self.fem.view.field) || (!r.modes.is_empty() && !is_disp(self.fem.view.field)) {
                    self.fem.view.field = if r.modes.is_empty() { Field::VonMises } else { Field::DisplacementMagnitude };
                }
                if r.modes.is_empty() {
                    let u = r.max_displacement().map(|e| e.value).unwrap_or(0.0);
                    let s = r.max_von_mises().map(|e| e.value).unwrap_or(0.0);
                    self.status = format!("Solved in {elapsed:.1} s: max displacement {} mm, max von Mises {} MPa", sig(u), sig(s));
                } else {
                    self.status = format!("Solved in {elapsed:.1} s: {} eigenmodes, f1 = {} Hz", r.modes.len(), sig(r.modes[0].frequency));
                }
                self.fem.hide_results = false;
                // Run from the toolbar: show the result task once solved
                if matches!(self.fem.task, Some(FemTask::Solver { auto: true, .. })) {
                    if let Some(FemTask::Solver { before, .. }) = self.fem.task.take() {
                        if self.state_json() == before {
                            self.undo.pop();
                        }
                    }
                    self.fem.task = Some(FemTask::Results { a });
                    self.fem.sel = Some(FemItem::Results(a));
                } else if self.fem.task.is_none() {
                    self.fem.sel = Some(FemItem::Results(a));
                }
            }
            Err(e) => {
                d.rt.log.push_str(&format!("--- Failed: {e}\n"));
                self.status = format!("FEM run failed: {}", e.lines().next().unwrap_or_default());
                d.rt.error = Some(e);
            }
        }
        self.rebuild_scene();
        if solved {
            // fit the result into the space left of the color bar
            let ri = self.vp.right_inset;
            self.vp.right_inset = ri + 170.0;
            self.vp.fit_all();
            self.vp.right_inset = ri;
        }
    }

    // ------------------------------------------------------------------ scene

    /// FEM objects after the part's objects: the result (colored, deformed) or the mesh, which
    /// then replace the body (object 0 stays in the list, hidden, for picking geometry).
    pub(super) fn fem_extend_scene(&mut self, objs: &mut Vec<SceneObject>) {
        self.fem.shown = None;
        self.fem.fem_at = None;
        if !self.fem.on || self.editor.is_some() {
            return;
        }
        let Some(a) = self.fem_cur() else { return };
        // picking geometry for a constraint: show the body
        if matches!(self.fem.task, Some(FemTask::Constraint { .. } | FemTask::Material { .. } | FemTask::AsmSetup { .. })) {
            return;
        }
        let rt = self.fem.docs[a].rt.clone();
        let at = objs.len();
        let t = self.ctx_time();
        if let (Some(r), false) = (&rt.results, self.fem.hide_results) {
            let v = self.fem.view.clone();
            let mode = v.mode.min(r.modes.len().saturating_sub(1));
            let mut scale = if v.auto_scale { auto_scale(r, mode) } else { v.scale };
            if !r.modes.is_empty() && v.animate {
                scale *= (2.0 * std::f64::consts::PI * (t - self.fem.anim_t0) / 1.6).sin();
            }
            let pos = if r.modes.is_empty() { r.deformed(scale) } else { r.mode_shape(mode, scale).unwrap_or_else(|| r.coords.clone()) };
            let values = field_values(r, v.field, mode);
            let (mut lo, mut hi, mut hi_i) = (f64::MAX, f64::MIN, 0);
            for (i, x) in values.iter().enumerate() {
                if r.surface.triangles.is_empty() {
                    break;
                }
                lo = lo.min(*x);
                if *x > hi {
                    hi = *x;
                    hi_i = i;
                }
            }
            if lo > hi {
                (lo, hi) = (0.0, 0.0);
            }
            let (positions, normals, indices, tri_face, nodes) = split_surface(&r.surface.triangles, &r.surface.tri_face, &pos);
            let span = (hi - lo).max(1e-30);
            let colors = nodes.iter().map(|n| colormap((values[*n as usize] - lo) / span)).collect();
            let edges = if v.show_edges { wire_edges(&r.surface.triangles, rt.mesh.as_ref().is_some_and(|m| m.order == 2), &pos) } else { vec![] };
            objs.push(SceneObject { name: "Result".into(), mesh: SceneMesh { positions, normals, indices, tri_face, colors }, edges, vertices: vec![], visible: true, color: [1.0, 1.0, 1.0, 1.0] });
            if let Some(b) = objs.first_mut() {
                b.visible = false;
            }
            self.fem.shown = Some(Shown { object: at, node_of_vertex: nodes, values: Some(values), min: lo, max: hi, max_node: hi_i, positions: pos, scale });
            self.fem.fem_at = Some(at);
        } else if let (Some(m), Some(s), false) = (&rt.mesh, &rt.surface, self.fem.hide_mesh) {
            let (positions, normals, indices, tri_face, nodes) = split_surface(&s.triangles, &s.tri_face, &m.coords);
            let edges = wire_edges(&s.triangles, m.order == 2, &m.coords);
            objs.push(SceneObject { name: "FEMMeshGmsh".into(), mesh: SceneMesh { positions, normals, indices, tri_face, colors: vec![] }, edges, vertices: vec![], visible: true, color: MESH_COLOR });
            if let Some(b) = objs.first_mut() {
                b.visible = false;
            }
            self.fem.shown = Some(Shown { object: at, node_of_vertex: nodes, values: None, min: 0.0, max: 0.0, max_node: 0, positions: m.coords.clone(), scale: 0.0 });
            self.fem.fem_at = Some(at);
        }
    }

    /// The 3D view of the FEM workbench on an assembly: the analysed shape (one object, its
    /// faces pickable for constraints, colored per part), then the mesh or results.
    pub(super) fn rebuild_fem_asm_scene(&mut self) {
        let Some(a) = self.fem_cur() else { return };
        self.fem_ensure_shape(a);
        let mut objs = vec![];
        if let Some(shape) = self.fem.docs[a].rt.shape.clone() {
            if let Some(mut o) = scene::shape_object("Analysis", &shape, scene::BODY_COLOR) {
                let d = &self.fem.docs[a];
                let face_part: HashMap<usize, usize> = d.analysis.parts.iter().enumerate().flat_map(|(pi, p)| p.faces.iter().map(move |f| (f.1, pi))).collect();
                let mut colors = vec![[0.8f32, 0.8, 0.8]; o.mesh.positions.len()];
                for (t, f) in o.mesh.tri_face.iter().enumerate() {
                    if let Some(pi) = face_part.get(&(*f as usize + 1)) {
                        let c = part_color(*pi);
                        for k in 0..3 {
                            colors[o.mesh.indices[3 * t + k] as usize] = c;
                        }
                    }
                }
                o.mesh.colors = colors;
                o.color = [1.0, 1.0, 1.0, 1.0];
                objs.push(o);
            }
        }
        self.fem_extend_scene(&mut objs);
        self.vp.pick_limit = self.fem.fem_at.unwrap_or(usize::MAX);
        self.asm.objs.clear();
        self.vp.set_objects(objs);
        self.vp.set_overlay(crate::viewport::Overlay::default());
        self.planes_at = None;
    }

    /// Part of the result triangle `t` under the cursor (assembly analyses).
    pub(super) fn fem_part_at_triangle(&self, a: usize, t: usize) -> Option<String> {
        let d = self.fem.docs.get(a)?;
        d.asm.as_ref()?;
        let r = d.rt.results.as_ref()?;
        let solid = *r.surface.tri_solid.get(t)? as usize;
        d.analysis.part_of_solid(solid).map(|p| d.analysis.parts[p].name.clone())
    }

    fn ctx_time(&self) -> f64 {
        std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0)
    }

    /// Rebuild only the FEM objects (field / scale changes, mode animation).
    pub(super) fn fem_refresh_view(&mut self) {
        let keep = self.fem.fem_at.unwrap_or(self.vp.objects().len());
        let mut objs: Vec<SceneObject> = self.vp.objects()[..keep.min(self.vp.objects().len())].to_vec();
        if let Some(b) = objs.first_mut() {
            b.visible = self.body_visible;
        }
        self.fem_extend_scene(&mut objs);
        self.vp.pick_limit = self.fem.fem_at.unwrap_or(usize::MAX);
        self.vp.set_objects(objs);
    }
}

/// The in-memory solver log keeps at most this many bytes (the newest output).
pub const LOG_CAP: usize = 1 << 20;

/// Drop the oldest lines of `log` so it stays within `cap` bytes (ccx can print gigabytes when
/// a model misbehaves); a marker line says that output was cut.
pub fn cap_log(log: &mut String, cap: usize) {
    if log.len() <= cap {
        return;
    }
    const MARK: &str = "[... earlier output dropped ...]\n";
    let mut cut = log.len() - cap + MARK.len();
    while !log.is_char_boundary(cut) {
        cut += 1;
    }
    let cut = log[cut..].find('\n').map(|i| cut + i + 1).unwrap_or(cut);
    log.replace_range(..cut, MARK);
}

/// Significant-digit formatting for values in the UI (4 significant digits).
pub fn sig(v: f64) -> String {
    if v == 0.0 || !v.is_finite() {
        return "0".into();
    }
    let a = v.abs();
    if !(1e-3..1e5).contains(&a) {
        return format!("{v:.3e}");
    }
    let d = (3 - a.log10().floor() as i32).max(0) as usize;
    let s = format!("{v:.d$}");
    if s.contains('.') { s.trim_end_matches('0').trim_end_matches('.').to_string() } else { s }
}

#[cfg(test)]
mod log_tests {
    #[test]
    fn log_is_capped() {
        let mut s = String::new();
        for i in 0..100_000 {
            s.push_str(&format!("line {i}\n"));
            super::cap_log(&mut s, 4096);
        }
        assert!(s.len() <= 4096);
        assert!(s.starts_with("[... earlier"));
        assert!(s.ends_with("line 99999\n"));
    }
}

fn fem_worker(a: Analysis, mesh_only: bool, tx: mpsc::Sender<JobMsg>, cancel: Arc<AtomicBool>) {
    let fail = |tx: &mpsc::Sender<JobMsg>, e: fc_fem::FemError| {
        let _ = tx.send(JobMsg::Failed(e.to_string()));
    };
    let dir = std::env::temp_dir().join(format!("freecad-rs-fem-{}-{}", std::process::id(), a.name));
    let _ = std::fs::remove_dir_all(&dir);
    if let Err(e) = std::fs::create_dir_all(&dir) {
        return fail(&tx, e.into());
    }
    let _ = tx.send(JobMsg::Stage(format!("Meshing with Gmsh in {}", dir.display())));
    let (mesh, areas, log) = match fc_fem::mesh_analysis(&a, &dir) {
        Ok(x) => x,
        Err(e) => return fail(&tx, e),
    };
    let _ = tx.send(JobMsg::Meshed(mesh.clone(), log));
    if mesh_only {
        return;
    }
    if cancel.load(Ordering::Relaxed) {
        return fail(&tx, fc_fem::FemError::Cancelled);
    }
    let _ = tx.send(JobMsg::Stage("Writing the CalculiX input Mesh.inp".into()));
    if let Err(e) = fc_fem::write_input(&a, &mesh, areas.as_ref(), &dir, &Default::default()) {
        return fail(&tx, e);
    }
    let _ = tx.send(JobMsg::Stage("Running CalculiX (ccx -i Mesh)".into()));
    let tx2 = tx.clone();
    if let Err(e) = fc_fem::run_ccx_streaming(&dir, a.solver.threads, &mut |l| {
        let _ = tx2.send(JobMsg::Line(l.to_string()));
    }, &cancel)
    {
        return fail(&tx, e);
    }
    let _ = tx.send(JobMsg::Stage("Reading the results (Mesh.frd, Mesh.dat)".into()));
    match fc_fem::read_results(&mesh, &dir) {
        Ok(r) => {
            let _ = tx.send(JobMsg::Solved(r));
        }
        Err(e) => fail(&tx, e),
    }
}

/// fc-fem's own numbers for the e2e FEM scripts (same geometry and settings as the UI builds):
/// `cargo test -p fc-app fem_reference -- --ignored --nocapture` (needs gmsh and ccx).
#[cfg(test)]
mod reference {
    use super::*;

    fn face_where(s: &fc_occt::Shape, f: impl Fn(&fc_occt::FaceInfo) -> bool) -> usize {
        (1..=s.face_count()).find(|&i| s.face_info(i).map(|x| f(&x)).unwrap_or(false)).expect("face")
    }

    /// The UI's analysis: New Analysis + Material for Solid (Steel-Generic, MaterialSolid).
    fn ui_analysis(shape: fc_occt::Shape, part: &str) -> Analysis {
        let mut a = Analysis::new(ShapeSource::Shape(shape), Material::steel_generic());
        a.shape_name = part.into();
        a.materials = vec![fc_fem::MaterialAssignment { name: "MaterialSolid".into(), material: Material::steel_generic(), solids: vec![] }];
        a
    }

    #[test]
    #[ignore]
    fn fem_reference_plate_hole() {
        let doc = crate::demo::plate_hole_document();
        let s = doc.result().unwrap().clone();
        let x0 = face_where(&s, |f| f.kind == 0 && f.normal[0] < -0.99);
        let x1 = face_where(&s, |f| f.kind == 0 && f.normal[0] > 0.99);
        let mut a = ui_analysis(s.clone(), "Body");
        a.fixed(vec![SubRef::face_of(&s, x0).unwrap()]);
        a.pressure(vec![SubRef::face_of(&s, x1).unwrap()], 10.0, true);
        a.mesh.max_size = 5.0;
        let sol = fc_fem::solve(&a, &Default::default()).unwrap();
        println!("plate_hole: faces {x0} {x1}, nodes {}, max|u| {:?}, max vM {:?}", sol.results.node_count(), sol.results.max_displacement().map(|e| e.value), sol.results.max_von_mises().map(|e| e.value));
    }

    /// Two bonded blocks (fc-fem's asm_stack): BooleanFragments, Lower aluminium / Upper
    /// steel from the parts' materials, bottom fixed, 10 MPa on top, 4 mm tets.
    #[test]
    #[ignore]
    fn fem_reference_asm_stack() {
        let mut ws = crate::demo::asm_test_workspace("stack").unwrap();
        ws.recompute();
        let mut a = fc_fem::analysis_from_assembly(&ws, ws.assemblies[0].id, fc_fem::Joining::Fragments { fuzzy: 0.0 }).unwrap();
        let s = a.shape_in_memory().unwrap().clone();
        let bottom = face_where(&s, |f| f.kind == 0 && f.normal[2] < -0.99 && f.center[2].abs() < 1e-6);
        let top = face_where(&s, |f| f.kind == 0 && f.normal[2] > 0.99 && (f.center[2] - 40.0).abs() < 1e-6);
        a.fixed(vec![SubRef::face_of(&s, bottom).unwrap()]);
        a.pressure(vec![SubRef::face_of(&s, top).unwrap()], 10.0, false);
        a.mesh.max_size = 4.0;
        let sol = fc_fem::solve(&a, &Default::default()).unwrap();
        for p in fc_fem::part_results(&a, &sol) {
            println!("asm_stack part {}: max|u| {:?} max vM {:?}", p.name, p.max_displacement.map(|e| e.value), p.max_von_mises.map(|e| e.value));
        }
        println!("asm_stack: faces {bottom} {top}, nodes {}, max|u| {:?}, max vM {:?}", sol.results.node_count(), sol.results.max_displacement().map(|e| e.value), sol.results.max_von_mises().map(|e| e.value));
    }
}
