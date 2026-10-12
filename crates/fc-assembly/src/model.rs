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

/// A part definition: one PartDesign body.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PartEntry {
    pub id: PartId,
    pub name: String,
    pub doc: Document,
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
        self.parts.push(PartEntry { id, name: name.to_string(), doc });
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

/// One step of an exploded view: move some instances (in world coordinates).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ExplodeStep {
    pub instances: Vec<InstanceId>,
    pub translation: V3,
    /// Optional rotation (axis, degrees) about the instances' own origins.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rotation: Option<(V3, f64)>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ExplodedView {
    pub name: String,
    pub steps: Vec<ExplodeStep>,
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
    /// Instance placements with an exploded view applied (for display only).
    pub fn exploded_placements(&self, view: usize) -> HashMap<InstanceId, Frame> {
        let mut out: HashMap<InstanceId, Frame> = self.instances.iter().map(|i| (i.id, i.placement)).collect();
        if let Some(v) = self.exploded_views.get(view) {
            for s in &v.steps {
                for id in &s.instances {
                    if let Some(f) = out.get_mut(id) {
                        let mut g = *f;
                        if let Some((axis, deg)) = s.rotation {
                            let r = Frame::rotation(axis, deg.to_radians());
                            g.rot = r.mul(&Frame { pos: [0.0; 3], rot: g.rot }).rot;
                        }
                        g.pos = fc_part::placement::add(g.pos, s.translation);
                        *f = g;
                    }
                }
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
        element_geom(self.part(part)?.doc.result()?, element)
    }
    fn infer_frame(&self, part: PartId, element: &str, infer: fc_part::Infer) -> Option<Frame> {
        let shape = self.part(part)?.doc.result()?;
        let (kind, index) = fc_part::TopoKind::parse(element)?;
        fc_part::mate::infer_frame(shape, kind, index, infer).ok().map(|f| frame_from_mate(&f))
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
