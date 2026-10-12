//! Workspace / assembly data model (the `.fcrs` v2 project file).

use crate::frame::Frame;
use crate::geom::ElementGeom;
use crate::joint::{Joint, JointRef};
use fc_part::placement::{dot, normalize, sub};
use fc_part::{Document, FeatureId, V3};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

pub type PartId = u32;
pub type AssemblyId = u32;
pub type InstanceId = u32;
pub type JointId = u32;
pub type ConnectorId = u32;

/// Current project-file version (v1 = a bare fc-part `Document`).
pub const WORKSPACE_VERSION: u32 = 2;

/// A part definition: one PartDesign body, or a solid imported from a file (STEP).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PartEntry {
    pub id: PartId,
    pub name: String,
    pub doc: Document,
    /// Bill-of-materials data (part number, description, material, density/mass, colour).
    #[serde(default, skip_serializing_if = "PartMeta::is_empty")]
    pub meta: PartMeta,
    /// An imported solid (OCCT BREP text) used when the body has no features.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub brep: Option<String>,
    /// `brep` parsed (filled by [`PartEntry::set_imported_shape`], [`Workspace::recompute`]
    /// and [`Workspace::from_json`]).
    #[serde(skip)]
    pub imported: Option<fc_occt::Shape>,
}

/// Optional part metadata, as FreeCAD's BOM reads it from part properties.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PartMeta {
    #[serde(skip_serializing_if = "String::is_empty")]
    pub part_number: String,
    #[serde(skip_serializing_if = "String::is_empty")]
    pub description: String,
    #[serde(skip_serializing_if = "String::is_empty")]
    pub material: String,
    /// Density in kg/m³ (FreeCAD materials' unit); mass = volume × density.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub density: Option<f64>,
    /// Mass in kg, overriding volume × density.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mass: Option<f64>,
    /// sRGB colour (0..1) for STEP export.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<[f64; 3]>,
}

impl PartMeta {
    pub fn is_empty(&self) -> bool {
        *self == PartMeta::default()
    }
}

impl PartEntry {
    pub fn new(id: PartId, name: &str, doc: Document) -> PartEntry {
        PartEntry { id, name: name.to_string(), doc, meta: PartMeta::default(), brep: None, imported: None }
    }
    /// The part's solid: the body's result, or the imported solid.
    pub fn shape(&self) -> Option<&fc_occt::Shape> {
        self.doc.result().or(self.imported.as_ref())
    }
    /// Makes this an imported part (stored as BREP text in the project file).
    pub fn set_imported_shape(&mut self, shape: fc_occt::Shape) {
        self.brep = shape.to_brep_string().ok();
        self.imported = Some(shape);
    }
    fn load_imported(&mut self) {
        if self.imported.is_none() {
            if let Some(b) = &self.brep {
                self.imported = fc_occt::Shape::from_brep_string(b).ok();
            }
        }
    }
    /// Volume in mm³ (0 without geometry).
    pub fn volume(&self) -> f64 {
        self.shape().map(|s| s.volume()).unwrap_or(0.0)
    }
    /// Mass in kg: the explicit mass, else volume × density.
    pub fn mass_kg(&self) -> Option<f64> {
        self.meta.mass.or_else(|| self.meta.density.map(|d| self.volume() * 1e-9 * d))
    }
}

/// A multi-part project: part definitions plus assemblies that instance them.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Workspace {
    #[serde(default = "default_version")]
    pub version: u32,
    #[serde(default)]
    pub parts: Vec<PartEntry>,
    #[serde(default)]
    pub assemblies: Vec<Assembly>,
    #[serde(default)]
    pub next_id: u32,
}

fn default_version() -> u32 {
    WORKSPACE_VERSION
}

impl Default for Workspace {
    fn default() -> Self {
        Workspace { version: WORKSPACE_VERSION, parts: vec![], assemblies: vec![], next_id: 1 }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum WorkspaceError {
    #[error("json: {0}")]
    Json(#[from] serde_json::Error),
    #[error("unsupported workspace version {0}")]
    Version(u32),
    #[error("assembly {0} would contain itself")]
    Cycle(AssemblyId),
    #[error("no such part or assembly: {0}")]
    Missing(u32),
    #[error("{0}")]
    Occt(#[from] fc_occt::OcctError),
    #[error("{0}")]
    Io(#[from] std::io::Error),
}

impl Workspace {
    pub fn new() -> Self {
        Self::default()
    }
    fn alloc(&mut self) -> u32 {
        let id = self.next_id.max(1);
        self.next_id = id + 1;
        id
    }
    pub fn add_part(&mut self, name: &str, doc: Document) -> PartId {
        let id = self.alloc();
        self.parts.push(PartEntry::new(id, name, doc));
        id
    }
    /// Adds a part defined by an imported solid (no features).
    pub fn add_imported_part(&mut self, name: &str, shape: fc_occt::Shape) -> PartId {
        let id = self.alloc();
        let mut p = PartEntry::new(id, name, Document::new());
        p.set_imported_shape(shape);
        self.parts.push(p);
        id
    }
    pub fn add_assembly(&mut self, name: &str) -> AssemblyId {
        let id = self.alloc();
        let mut a = Assembly::new(name);
        a.id = id;
        self.assemblies.push(a);
        id
    }
    pub fn part(&self, id: PartId) -> Option<&PartEntry> {
        self.parts.iter().find(|p| p.id == id)
    }
    pub fn part_mut(&mut self, id: PartId) -> Option<&mut PartEntry> {
        self.parts.iter_mut().find(|p| p.id == id)
    }
    pub fn assembly(&self, id: AssemblyId) -> Option<&Assembly> {
        self.assemblies.iter().find(|a| a.id == id)
    }
    pub fn assembly_mut(&mut self, id: AssemblyId) -> Option<&mut Assembly> {
        self.assemblies.iter_mut().find(|a| a.id == id)
    }
    /// Recompute every part document (needed before geometry references resolve).
    pub fn recompute(&mut self) {
        for p in &mut self.parts {
            p.doc.recompute();
            p.load_imported();
        }
    }

    pub fn to_json(&self) -> String {
        serde_json::to_string_pretty(self).expect("workspace serializes")
    }

    /// Load a v2 workspace, or an old single-body document (v1) as a one-part workspace.
    pub fn from_json(s: &str) -> Result<Workspace, WorkspaceError> {
        let v: serde_json::Value = serde_json::from_str(s)?;
        if v.get("features").is_some() && v.get("parts").is_none() {
            let doc = Document::from_json(s)?;
            let mut ws = Workspace::new();
            ws.add_part("Body", doc);
            return Ok(ws);
        }
        let mut ws: Workspace = serde_json::from_value(v)?;
        if ws.version > WORKSPACE_VERSION {
            return Err(WorkspaceError::Version(ws.version));
        }
        ws.version = WORKSPACE_VERSION;
        for p in &mut ws.parts {
            p.load_imported();
            p.doc.next_id = p.doc.next_id.max(p.doc.features.iter().map(|f| f.id + 1).max().unwrap_or(0));
        }
        let max_id = ws
            .parts
            .iter()
            .map(|p| p.id)
            .chain(ws.assemblies.iter().map(|a| a.id))
            .max()
            .unwrap_or(0);
        ws.next_id = ws.next_id.max(max_id + 1);
        for a in &mut ws.assemblies {
            a.fix_next_id();
        }
        Ok(ws)
    }
}

/// What an instance places: a part, or (later) a whole sub-assembly as a rigid body.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum InstanceOf {
    Part(PartId),
    Assembly(AssemblyId),
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Instance {
    pub id: InstanceId,
    pub name: String,
    pub of: InstanceOf,
    pub placement: Frame,
    #[serde(default)]
    pub grounded: bool,
    #[serde(default = "yes")]
    pub visible: bool,
}

fn yes() -> bool {
    true
}

/// A mate connector created in the assembly (Onshape lets you add these there too):
/// a frame anchored to an instance, optionally remembering the element it was built on.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AssemblyConnector {
    pub id: ConnectorId,
    pub name: String,
    pub instance: InstanceId,
    /// Frame in the instance's local coordinates.
    pub local: Frame,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub anchor: Option<String>,
}

/// How an exploded-view step moves its instances (FreeCAD's `ExplodedViewStep.MoveType`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum ExplodeKind {
    /// `placement = transform * placement` (world-space move/rotation).
    #[default]
    Normal,
    /// Push each instance away from the assembly's centre: its bounding-box centre moves by
    /// `(centre - assembly centre) * 4 * distance / assembly diagonal`, where `distance` is
    /// `|transform.pos|` (FreeCAD's radial explode).
    Radial,
}

/// One step of an exploded view (FreeCAD's `ExplodedViewStep`): move some top-level
/// instances of the assembly. Steps apply in order, each starting from where the previous
/// ones left the instances.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ExplodeStep {
    pub instances: Vec<InstanceId>,
    #[serde(default)]
    pub kind: ExplodeKind,
    /// FreeCAD's `MovementTransform` (world coordinates).
    pub transform: Frame,
}

impl ExplodeStep {
    /// Move along `dir` by `distance`.
    pub fn translate(instances: Vec<InstanceId>, dir: V3, distance: f64) -> ExplodeStep {
        let d = fc_part::placement::scale(normalize(dir), distance);
        ExplodeStep { instances, kind: ExplodeKind::Normal, transform: Frame::translation(d) }
    }
    /// Rotate by `degrees` about the world axis through `center` along `axis`
    /// (FreeCAD's `setMovementAngle`: `Placement(center, rot) * Placement(center).inverse()`).
    pub fn rotate(instances: Vec<InstanceId>, center: V3, axis: V3, degrees: f64) -> ExplodeStep {
        let r = Frame { pos: center, rot: Frame::rotation(axis, degrees.to_radians()).rot };
        ExplodeStep { instances, kind: ExplodeKind::Normal, transform: r.mul(&Frame::translation(center).inverse()) }
    }
    /// Radial explode by `distance`.
    pub fn radial(instances: Vec<InstanceId>, distance: f64) -> ExplodeStep {
        ExplodeStep { instances, kind: ExplodeKind::Radial, transform: Frame::translation([distance, 0.0, 0.0]) }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ExplodedView {
    pub name: String,
    pub steps: Vec<ExplodeStep>,
}

/// An exploded view evaluated at some animation time.
#[derive(Clone, Debug, Default)]
pub struct Exploded {
    /// Display placement of every top-level instance (unchanged ones included).
    pub placements: HashMap<InstanceId, Frame>,
    /// Trail lines (start, end) of moved instances' centres, one per moved instance and step.
    pub lines: Vec<(V3, V3)>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Assembly {
    #[serde(default)]
    pub id: AssemblyId,
    pub name: String,
    #[serde(default)]
    pub instances: Vec<Instance>,
    #[serde(default)]
    pub connectors: Vec<AssemblyConnector>,
    #[serde(default)]
    pub joints: Vec<Joint>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub exploded_views: Vec<ExplodedView>,
    #[serde(default)]
    pub next_id: u32,
}

impl Assembly {
    pub fn new(name: &str) -> Assembly {
        Assembly {
            id: 0,
            name: name.to_string(),
            instances: vec![],
            connectors: vec![],
            joints: vec![],
            exploded_views: vec![],
            next_id: 1,
        }
    }
    fn alloc(&mut self) -> u32 {
        let id = self.next_id.max(1);
        self.next_id = id + 1;
        id
    }
    pub(crate) fn fix_next_id(&mut self) {
        let m = self
            .instances
            .iter()
            .map(|i| i.id)
            .chain(self.joints.iter().map(|j| j.id))
            .chain(self.connectors.iter().map(|c| c.id))
            .max()
            .unwrap_or(0);
        self.next_id = self.next_id.max(m + 1);
    }
    fn unique_name(&self, base: &str) -> String {
        let taken = |n: &str| {
            self.instances.iter().any(|i| i.name == n)
                || self.joints.iter().any(|j| j.name == n)
                || self.connectors.iter().any(|c| c.name == n)
        };
        if !taken(base) {
            return base.to_string();
        }
        (1..).map(|k| format!("{base}{k:03}")).find(|n| !taken(n)).unwrap()
    }

    pub fn add_instance(&mut self, name: &str, of: InstanceOf, placement: Frame) -> InstanceId {
        let id = self.alloc();
        let name = self.unique_name(name);
        self.instances.push(Instance { id, name, of, placement, grounded: false, visible: true });
        id
    }
    pub fn add_part_instance(&mut self, name: &str, part: PartId, placement: Frame) -> InstanceId {
        self.add_instance(name, InstanceOf::Part(part), placement)
    }
    pub fn add_connector(&mut self, name: &str, instance: InstanceId, local: Frame) -> ConnectorId {
        let id = self.alloc();
        let name = self.unique_name(name);
        self.connectors.push(AssemblyConnector { id, name, instance, local, anchor: None });
        id
    }
    /// Adds a joint (its id and a unique name are assigned here).
    pub fn add_joint(&mut self, mut joint: Joint) -> JointId {
        joint.id = self.alloc();
        joint.name = self.unique_name(&joint.name);
        let id = joint.id;
        self.joints.push(joint);
        id
    }
    pub fn remove_joint(&mut self, id: JointId) -> Option<Joint> {
        let i = self.joints.iter().position(|j| j.id == id)?;
        Some(self.joints.remove(i))
    }
    /// Removes an instance with its joints and assembly connectors.
    pub fn remove_instance(&mut self, id: InstanceId) -> Option<Instance> {
        let i = self.instances.iter().position(|x| x.id == id)?;
        let conns: Vec<ConnectorId> = self.connectors.iter().filter(|c| c.instance == id).map(|c| c.id).collect();
        let touches = |r: &JointRef| match r {
            JointRef::AssemblyConnector(c) => conns.contains(c),
            other => other.instance() == Some(id),
        };
        self.joints.retain(|j| !touches(&j.a) && !touches(&j.b));
        self.connectors.retain(|c| c.instance != id);
        Some(self.instances.remove(i))
    }
    pub fn instance(&self, id: InstanceId) -> Option<&Instance> {
        self.instances.iter().find(|i| i.id == id)
    }
    pub fn instance_mut(&mut self, id: InstanceId) -> Option<&mut Instance> {
        self.instances.iter_mut().find(|i| i.id == id)
    }
    pub fn instance_by_name(&self, name: &str) -> Option<&Instance> {
        self.instances.iter().find(|i| i.name == name)
    }
    pub fn joint(&self, id: JointId) -> Option<&Joint> {
        self.joints.iter().find(|j| j.id == id)
    }
    pub fn joint_mut(&mut self, id: JointId) -> Option<&mut Joint> {
        self.joints.iter_mut().find(|j| j.id == id)
    }
    pub fn set_grounded(&mut self, id: InstanceId, grounded: bool) {
        if let Some(i) = self.instance_mut(id) {
            i.grounded = grounded;
        }
    }
    /// Instance placements with exploded view `view` applied at animation time `t`
    /// (0 = assembled, 1 = fully exploded; the steps play one after another). The solved
    /// placements are not touched. Without geometry, instance origins stand in for their
    /// bounding-box centres (radial steps); see [`Workspace::exploded`] for the real thing.
    pub fn exploded_placements(&self, view: usize, t: f64) -> HashMap<InstanceId, Frame> {
        self.explode_with(view, t, &|_| Some([0.0; 3]), None).placements
    }

    /// Evaluates an exploded view. `center(id)` is an instance's bounding-box centre in its
    /// own coordinates, `bbox` the assembled bounding box (`[min, max]`); without one the
    /// centre is the mean of the instance centres and the size 100 (FreeCAD's fallback).
    pub fn explode_with(&self, view: usize, t: f64, center: &dyn Fn(InstanceId) -> Option<V3>, bbox: Option<[f64; 6]>) -> Exploded {
        use fc_part::placement::{add, scale};
        let mut out = Exploded { placements: self.instances.iter().map(|i| (i.id, i.placement)).collect(), lines: vec![] };
        let Some(v) = self.exploded_views.get(view) else { return out };
        let world_center = |id: InstanceId, pl: &Frame| center(id).map(|c| pl.transform_point(c)).unwrap_or(pl.pos);
        let (com, size) = match bbox {
            Some(b) => {
                let d = sub([b[3], b[4], b[5]], [b[0], b[1], b[2]]);
                (scale(add([b[0], b[1], b[2]], [b[3], b[4], b[5]]), 0.5), fc_part::placement::norm(d))
            }
            None => {
                let n = self.instances.len().max(1) as f64;
                let s = self.instances.iter().fold([0.0; 3], |a, i| add(a, world_center(i.id, &i.placement)));
                (scale(s, 1.0 / n), 100.0)
            }
        };
        let size = if size > 1e-9 { size } else { 100.0 };
        let n = v.steps.len() as f64;
        let s_time = t.clamp(0.0, 1.0) * n;
        for (k, step) in v.steps.iter().enumerate() {
            let f = (s_time - k as f64).clamp(0.0, 1.0);
            if f <= 0.0 {
                break;
            }
            for id in &step.instances {
                let Some(pl) = out.placements.get(id).copied() else { continue };
                let start = world_center(*id, &pl);
                let new = match step.kind {
                    ExplodeKind::Normal => step.transform.pow(f).mul(&pl),
                    ExplodeKind::Radial => {
                        let factor = 4.0 * fc_part::placement::norm(step.transform.pos) / size * f;
                        Frame { pos: add(pl.pos, scale(sub(start, com), factor)), rot: pl.rot }
                    }
                };
                out.lines.push((start, world_center(*id, &new)));
                out.placements.insert(*id, new);
            }
        }
        out
    }
}

/// Where joint references get their geometry from.
pub trait PartResolver {
    /// Local frame of a mate connector feature of a part.
    fn connector_frame(&self, _part: PartId, _connector: FeatureId) -> Option<Frame> {
        None
    }
    /// Geometry of a sub-element ("Face3", "Edge5", "Vertex2") in part coordinates.
    fn element(&self, _part: PartId, _element: &str) -> Option<ElementGeom> {
        None
    }
    /// Onshape-style inferred frame of a sub-element (see [`crate::ImplicitPoint::Infer`]).
    fn infer_frame(&self, _part: PartId, _element: &str, _infer: fc_part::Infer) -> Option<Frame> {
        None
    }
    /// An assembly that an instance places as a rigid sub-assembly (geometry references
    /// into it resolve through its instances).
    fn sub_assembly(&self, _id: AssemblyId) -> Option<&Assembly> {
        None
    }
}

/// fc-part's mate frame (origin + axes) as an assembly [`Frame`].
pub fn frame_from_mate(f: &fc_part::Frame) -> Frame {
    Frame::from_axes(f.origin, [f.x, f.y, f.z])
}

/// A resolver that knows nothing: only `Frame`, `Origin` and assembly-connector references work.
pub struct NoGeometry;
impl PartResolver for NoGeometry {}

/// A table-driven resolver (tests, caches, or geometry imported from elsewhere).
#[derive(Clone, Debug, Default)]
pub struct MapResolver {
    pub connectors: HashMap<(PartId, FeatureId), Frame>,
    pub elements: HashMap<(PartId, String), ElementGeom>,
}

impl MapResolver {
    pub fn new() -> Self {
        Self::default()
    }
    /// Registers the faces, edges and vertices of an axis-aligned box `[0,l]x[0,w]x[0,h]`
    /// with FreeCAD's `Part::Box` topology names.
    pub fn add_box(&mut self, part: PartId, l: f64, w: f64, h: f64) {
        for (name, g) in box_elements(l, w, h) {
            self.elements.insert((part, name), g);
        }
    }
}

impl PartResolver for MapResolver {
    fn connector_frame(&self, part: PartId, connector: FeatureId) -> Option<Frame> {
        self.connectors.get(&(part, connector)).copied()
    }
    fn element(&self, part: PartId, element: &str) -> Option<ElementGeom> {
        self.elements.get(&(part, element.to_string())).cloned()
    }
}

/// FreeCAD `Part::Box` sub-elements (Face1..6, Edge1..12, Vertex1..8), local coordinates.
/// Planes carry the *surface* frame FreeCAD uses (Face1 and Face2 both have +X normals).
pub fn box_elements(l: f64, w: f64, h: f64) -> Vec<(String, ElementGeom)> {
    let v = [
        [0.0, 0.0, h],
        [0.0, 0.0, 0.0],
        [0.0, w, h],
        [0.0, w, 0.0],
        [l, 0.0, h],
        [l, 0.0, 0.0],
        [l, w, h],
        [l, w, 0.0],
    ];
    // Start/end in the curve's parameter direction (FreeCAD's `curve.Direction`).
    let edges = [(2, 1), (1, 3), (4, 3), (2, 4), (6, 5), (5, 7), (8, 7), (6, 8), (2, 6), (1, 5), (4, 8), (3, 7)];
    let mut out = vec![];
    for (i, p) in v.iter().enumerate() {
        out.push((format!("Vertex{}", i + 1), ElementGeom::Vertex { point: *p }));
    }
    for (i, (a, b)) in edges.iter().enumerate() {
        out.push((format!("Edge{}", i + 1), ElementGeom::Line { start: v[a - 1], end: v[b - 1] }));
    }
    // (origin, normal, xdir, cog) from Part.makeBox's surfaces.
    let faces: [(V3, V3, V3, V3); 6] = [
        ([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, w / 2.0, h / 2.0]),
        ([l, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [l, w / 2.0, h / 2.0]),
        ([0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [l / 2.0, 0.0, h / 2.0]),
        ([0.0, w, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [l / 2.0, w, h / 2.0]),
        ([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [l / 2.0, w / 2.0, 0.0]),
        ([0.0, 0.0, h], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [l / 2.0, w / 2.0, h]),
    ];
    for (i, (o, n, x, c)) in faces.iter().enumerate() {
        out.push((format!("Face{}", i + 1), ElementGeom::Plane { origin: *o, normal: *n, xdir: *x, cog: *c }));
    }
    out
}

/// The workspace resolves references against its own part documents (after
/// [`Workspace::recompute`]): mate connector features via `Document::connector_frame`,
/// geometry picks on the body's tip solid via fc-occt's face/edge/vertex queries, and
/// Onshape-style inference via `fc_part::mate::infer_frame`.
impl PartResolver for Workspace {
    fn connector_frame(&self, part: PartId, connector: FeatureId) -> Option<Frame> {
        let doc = &self.part(part)?.doc;
        doc.connector_frame(connector).ok().map(|f| frame_from_mate(&f))
    }
    fn element(&self, part: PartId, element: &str) -> Option<ElementGeom> {
        element_geom(self.part(part)?.shape()?, element)
    }
    fn infer_frame(&self, part: PartId, element: &str, infer: fc_part::Infer) -> Option<Frame> {
        let shape = self.part(part)?.shape()?;
        let (kind, index) = fc_part::TopoKind::parse(element)?;
        fc_part::mate::infer_frame(shape, kind, index, infer).ok().map(|f| frame_from_mate(&f))
    }
    fn sub_assembly(&self, id: AssemblyId) -> Option<&Assembly> {
        self.assembly(id)
    }
}

/// [`ElementGeom`] of `Face<n>` / `Edge<n>` / `Vertex<n>` of an OCCT shape.
///
/// Planar faces report the outward normal (fc-occt corrects for face orientation) where
/// FreeCAD uses the underlying surface axis; they differ only on reversed faces.
pub fn element_geom(shape: &fc_occt::Shape, element: &str) -> Option<ElementGeom> {
    let (kind, n) = fc_part::TopoKind::parse(element)?;
    Some(match kind {
        fc_part::TopoKind::Face => {
            let f = shape.face_info(n).ok()?;
            match f.kind {
                0 => ElementGeom::Plane { origin: f.origin, normal: f.normal, xdir: f.xdir, cog: f.center },
                1 => ElementGeom::Cylinder { origin: f.origin, axis: f.normal, xdir: f.xdir, radius: f.radius, cog: f.center },
                2 => ElementGeom::Cone { apex: f.origin, axis: f.normal, xdir: f.xdir, cog: f.center },
                3 => ElementGeom::Sphere { center: f.origin, radius: f.radius },
                _ => ElementGeom::Surface { cog: f.center },
            }
        }
        fc_part::TopoKind::Edge => {
            let e = shape.edge_info(n).ok()?;
            match e.kind {
                0 => ElementGeom::Line { start: e.start, end: e.end },
                1 => ElementGeom::Circle { center: e.center, axis: e.axis, radius: e.radius, xdir: None },
                _ => ElementGeom::Curve { mid: e.mid },
            }
        }
        fc_part::TopoKind::Vertex => ElementGeom::Vertex { point: shape.vertex_info(n).ok()? },
    })
}

/// Small helper used by tests and the GUI: is `p` on the plane through `o` with normal `n`?
pub fn point_plane_distance(p: V3, o: V3, n: V3) -> f64 {
    dot(sub(p, o), normalize(n))
}
