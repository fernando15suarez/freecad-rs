//! The Assembly workbench (FreeCAD 1.1's, with Onshape's mate connectors): the assembly
//! scene (part instances), inserting parts, grounding, joints with live pre-solve + solve,
//! dragging parts while the joints hold, assembly-level mate connectors, and the solver's
//! DOF / redundant / conflicting diagnosis.

use super::workspace::Tab;
use super::*;
use crate::mate_ui::{self, Look};
use crate::theme;
use crate::viewport::SceneObject;
use crate::widgets::{self, task_box, task_row};
use egui::{Color32, RichText};
use fc_assembly::{
    InstanceOf, AssemblyId, ConnectorId, DragTarget, ImplicitPoint, InstanceId, Joint, JointId, JointKind, JointRef, Limits, PartId, Sense, SolveOptions, SolveReport, SolveStatus, Workspace,
};
use fc_part::Frame as MFrame;

/// FreeCAD's selection color for whole objects.
const SEL_COLOR: [f32; 4] = [0.11, 0.68, 0.11, 1.0];
const GHOST_COLOR: [f32; 4] = [0.55, 0.62, 0.75, 0.45];
/// Parts other than an inspected interference pair.
const FADED_COLOR: [f32; 4] = [0.6, 0.6, 0.62, 0.12];
/// The two parts of an inspected interference.
const PAIR_COLOR: [f32; 4] = [0.75, 0.78, 0.85, 0.35];
/// The overlapping volume of an interference.
const INTERFERENCE_COLOR: [f32; 4] = [0.95, 0.1, 0.1, 1.0];

/// The joint tools, in FreeCAD's toolbar order, then Onshape's extra mates.
pub const JOINT_TOOLS: [(&str, &str, &str); 11] = [
    ("Fixed", "Assembly_CreateJointFixed", "Fixed joint: locks the two parts together (6 DOF removed)"),
    ("Revolute", "Assembly_CreateJointRevolute", "Revolute joint: rotation about a common axis (1 DOF left)"),
    ("Cylindrical", "Assembly_CreateJointCylindrical", "Cylindrical joint: rotation about and sliding along an axis (2 DOF left)"),
    ("Slider", "Assembly_CreateJointSlider", "Slider joint: sliding along an axis (1 DOF left)"),
    ("Ball", "Assembly_CreateJointBall", "Ball joint: the two points coincide, any rotation (3 DOF left)"),
    ("Distance", "Assembly_CreateJointDistance", "Distance joint: a distance between the two elements"),
    ("Parallel", "Assembly_CreateJointParallel", "Parallel joint: the Z axes are parallel"),
    ("Perpendicular", "Assembly_CreateJointPerpendicular", "Perpendicular joint: the Z axes are perpendicular"),
    ("Angle", "Assembly_CreateJointAngle", "Angle joint: a fixed angle between the Z axes"),
    ("Planar", "Assembly_CreateJointPlanar", "Planar mate (Onshape): the XY planes coincide, sliding and turning in it (3 DOF left)"),
    ("PinSlot", "Assembly_CreateJointSlider", "Pin slot mate (Onshape): slides along X and turns about Z (2 DOF left)"),
];

pub fn joint_kind(name: &str) -> JointKind {
    match name {
        "Revolute" => JointKind::Revolute,
        "Cylindrical" => JointKind::Cylindrical,
        "Slider" => JointKind::Slider,
        "Ball" => JointKind::Ball,
        "Distance" => JointKind::Distance { distance: 0.0, mode: Default::default() },
        "Parallel" => JointKind::Parallel,
        "Perpendicular" => JointKind::Perpendicular,
        "Angle" => JointKind::Angle { angle: 90.0 },
        "Planar" => JointKind::Planar,
        "PinSlot" => JointKind::PinSlot,
        _ => JointKind::Fastened,
    }
}

pub fn joint_icon(k: &JointKind) -> &'static str {
    match k {
        JointKind::Fastened => "Assembly_CreateJointFixed",
        JointKind::Revolute => "Assembly_CreateJointRevolute",
        JointKind::Cylindrical => "Assembly_CreateJointCylindrical",
        JointKind::Slider | JointKind::PinSlot => "Assembly_CreateJointSlider",
        JointKind::Ball => "Assembly_CreateJointBall",
        JointKind::Distance { .. } => "Assembly_CreateJointDistance",
        JointKind::Parallel => "Assembly_CreateJointParallel",
        JointKind::Perpendicular => "Assembly_CreateJointPerpendicular",
        JointKind::Angle { .. } => "Assembly_CreateJointAngle",
        JointKind::Planar => "Assembly_CreateJointPlanar",
        JointKind::RackPinion { .. } => "Assembly_CreateJointRackPinion",
        JointKind::Screw { .. } => "Assembly_CreateJointScrew",
        JointKind::Gears { .. } => "Assembly_CreateJointGears",
        JointKind::Belt { .. } => "Assembly_CreateJointPulleys",
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AsmSel {
    Inst(InstanceId),
    Joint(JointId),
    Conn(ConnectorId),
}

pub struct JointTask {
    pub kind: JointKind,
    pub refs: Vec<JointRef>,
    /// The joint in the assembly once both references are set.
    pub joint: Option<JointId>,
    /// Editing an existing joint (double-click) rather than creating one.
    pub editing: bool,
    pub before: String,
    /// Placements when the task opened: every live update re-solves from here.
    pub start: Vec<(InstanceId, fc_assembly::Frame)>,
    pub offset: f64,
    pub rotation: f64,
    pub sense: Sense,
    pub limits: Limits,
}

pub struct ConnTask {
    pub before: String,
    pub pick: Option<(InstanceId, String, fc_part::Infer)>,
    pub flip: bool,
    pub turns: i32,
    pub offset: f64,
}

pub enum AsmTask {
    Insert { before: String, added: usize },
    Joint(JointTask),
    Connector(ConnTask),
    Interference(super::asm_tools::InterfTask),
    Bom(super::asm_tools::BomTask),
    Explode(super::asm_tools::ExplodeTask),
}

/// What is under the cursor in the assembly view.
#[derive(Clone)]
pub enum Hover {
    /// A mate connector triad (part connector or assembly connector).
    Triad(JointRef, MFrame),
    /// Geometry with its snapped inference point (world frame) and the other candidate points.
    Geom { inst: InstanceId, element: String, infer: fc_part::Infer, frame: MFrame, points: Vec<[f64; 3]>, chosen: usize },
}

pub struct Drag {
    pub inst: InstanceId,
    pub local: [f64; 3],
    pub plane: crate::viewport::Plane,
    pub before: String,
    pub moved: bool,
}

#[derive(Default)]
pub struct AsmState {
    pub sel: Vec<AsmSel>,
    pub report: Option<SolveReport>,
    /// Tessellated parts in part coordinates.
    pub meshes: HashMap<PartId, Option<SceneObject>>,
    /// Scene object index -> instance.
    pub objs: Vec<InstanceId>,
    pub task: Option<AsmTask>,
    pub drag: Option<Drag>,
    pub hover: Option<Hover>,
    /// Insert: the part following the cursor until a click places it.
    pub pending: Option<InstanceOf>,
    pub cursor_world: Option<[f64; 3]>,
    /// The overlap of an interference picked in its task (drawn red).
    pub highlight: Option<super::asm_tools::Highlight>,
    /// An exploded view on display: (view index, animation time).
    pub explode_show: Option<(usize, f64)>,
    /// Bounding-box centres of the top-level instances (exploded views), with the
    /// placements they were computed for.
    pub explode_cache: Option<super::asm_tools::ExplodeCache>,
}

impl AsmState {
    /// Forget selections of things that no longer exist.
    pub fn prune(&mut self, ws: &Workspace, a: Option<AssemblyId>) {
        let Some(asm) = a.and_then(|a| ws.assembly(a)) else {
            self.sel.clear();
            return;
        };
        self.sel.retain(|s| match s {
            AsmSel::Inst(i) => asm.instance(*i).is_some(),
            AsmSel::Joint(j) => asm.joint(*j).is_some(),
            AsmSel::Conn(c) => asm.connectors.iter().any(|x| x.id == *c),
        });
    }

    pub fn problem_joints(&self) -> Vec<JointId> {
        let Some(r) = &self.report else { return vec![] };
        let mut v: Vec<JointId> = r.conflicting_joints.iter().chain(&r.unsatisfied_joints).chain(&r.redundant_joints).copied().collect();
        v.extend(r.unresolved_joints.iter().map(|(j, _)| *j));
        v
    }
}

fn transform_object(o: &SceneObject, f: &fc_assembly::Frame, name: &str, color: [f32; 4]) -> SceneObject {
    let m = f.matrix();
    let tp = |p: &[f32; 3]| {
        let q = f.transform_point([p[0] as f64, p[1] as f64, p[2] as f64]);
        [q[0] as f32, q[1] as f32, q[2] as f32]
    };
    let tn = |n: &[f32; 3]| {
        let q = fc_assembly::frame::mat_vec(&m, [n[0] as f64, n[1] as f64, n[2] as f64]);
        [q[0] as f32, q[1] as f32, q[2] as f32]
    };
    let mut out = o.clone();
    out.name = name.to_string();
    out.color = color;
    out.mesh.positions = o.mesh.positions.iter().map(tp).collect();
    out.mesh.normals = o.mesh.normals.iter().map(tn).collect();
    for e in &mut out.edges {
        e.points = e.points.iter().map(tp).collect();
    }
    out.vertices = o.vertices.iter().map(tp).collect();
    out
}

fn world(pl: &fc_assembly::Frame, local: &MFrame) -> MFrame {
    mate_ui::from_asm(&pl.mul(&mate_ui::to_asm(local)))
}

impl App {
    pub(super) fn asm_ref(&self) -> Option<&fc_assembly::Assembly> {
        self.active_assembly().and_then(|a| self.ws.assembly(a))
    }

    /// Open what an instance places: its part in Part Design, or its assembly.
    fn open_instance(&mut self, inst: InstanceId) {
        match self.asm_ref().and_then(|a| a.instance(inst)).map(|i| i.of) {
            Some(InstanceOf::Part(p)) => self.switch_tab(Tab::Part(p)),
            Some(InstanceOf::Assembly(a)) => self.switch_tab(Tab::Assembly(a)),
            None => {}
        }
    }

    fn inst_part(&self, inst: InstanceId) -> Option<PartId> {
        match self.asm_ref()?.instance(inst)?.of {
            fc_assembly::InstanceOf::Part(p) => Some(p),
            _ => None,
        }
    }

    pub(super) fn inst_name(&self, inst: InstanceId) -> String {
        self.asm_ref().and_then(|a| a.instance(inst)).map(|i| i.name.clone()).unwrap_or_else(|| "?".into())
    }

    /// Solve the open assembly (FreeCAD solves after every change) and keep the report.
    pub(super) fn solve_active(&mut self, rebuild: bool) {
        let Some(a) = self.active_assembly() else {
            self.asm.report = None;
            return;
        };
        let rep = self.ws.solve_assembly(a, &SolveOptions::default());
        self.asm.report = rep;
        if rebuild {
            self.rebuild_scene();
        }
    }

    /// Run `f` on the open assembly as one undo step, then solve.
    fn asm_change(&mut self, f: impl FnOnce(&mut fc_assembly::Assembly)) {
        let Some(a) = self.active_assembly() else { return };
        if self.asm.task.is_none() {
            self.snapshot();
        }
        if let Some(asm) = self.ws.assembly_mut(a) {
            f(asm);
        }
        self.dirty = true;
        self.asm.prune(&self.ws, Some(a));
        self.solve_active(true);
    }

    // ------------------------------------------------------------------ scene

    pub(super) fn rebuild_asm_scene(&mut self) {
        let Some(a) = self.active_assembly() else { return };
        let Some(asm) = self.ws.assembly(a).cloned() else { return };
        let mut objs = vec![];
        let mut map = vec![];
        let selected: HashSet<InstanceId> = self.asm.sel.iter().filter_map(|s| if let AsmSel::Inst(i) = s { Some(*i) } else { None }).collect();
        // the exploded view on display moves top-level instances (and what they contain)
        let shown = self.asm_display_placements();
        // an interference picked in its task: the two parts stay opaque, the others fade
        let focus: Option<(Vec<InstanceId>, Vec<InstanceId>)> = self.asm.highlight.as_ref().map(|h| (h.a.clone(), h.b.clone()));
        for leaf in self.ws.leaves(a) {
            if !leaf.visible {
                continue;
            }
            let top = leaf.path[0];
            let p = leaf.part;
            if !self.asm.meshes.contains_key(&p) {
                let m = self.ws.part(p).and_then(|e| e.shape()).and_then(|s| scene::shape_object("", s, scene::BODY_COLOR));
                self.asm.meshes.insert(p, m);
            }
            let pl = match (shown.get(&top), asm.instance(top)) {
                (Some(new), Some(old)) => new.mul(&old.placement.inverse()).mul(&leaf.placement),
                _ => leaf.placement,
            };
            if let Some(Some(m)) = self.asm.meshes.get(&p) {
                let mut color = if selected.contains(&top) { SEL_COLOR } else { scene::BODY_COLOR };
                if let Some((fa, fb)) = &focus {
                    // the pair turns see-through so the red overlap inside shows
                    color = if leaf.path == *fa || leaf.path == *fb { PAIR_COLOR } else { FADED_COLOR };
                }
                objs.push(transform_object(m, &pl, &leaf.name, color));
                map.push(top);
            }
        }
        // a part or sub-assembly being inserted follows the cursor
        if let (Some(p), Some(w)) = (self.asm.pending, self.asm.cursor_world) {
            let pl = self.insert_placement(p, w);
            for (part, local) in self.pending_leaves(p) {
                if !self.asm.meshes.contains_key(&part) {
                    let m = self.ws.part(part).and_then(|e| e.shape()).and_then(|s| scene::shape_object("", s, scene::BODY_COLOR));
                    self.asm.meshes.insert(part, m);
                }
                if let Some(Some(m)) = self.asm.meshes.get(&part) {
                    let mut g = transform_object(m, &pl.mul(&local), "ghost", GHOST_COLOR);
                    g.vertices.clear();
                    objs.push(g);
                }
            }
        }
        self.vp.pick_limit = map.len();
        // the overlap of the picked interference, in red over everything
        if let Some(h) = &self.asm.highlight {
            if let Some(mut o) = scene::shape_object("Interference", &h.shape, INTERFERENCE_COLOR) {
                o.vertices.clear();
                // pulled in a hair so its faces do not z-fight with the parts' faces around it
                let n = o.mesh.positions.len().max(1) as f32;
                let c = o.mesh.positions.iter().fold([0.0f32; 3], |a, p| [a[0] + p[0] / n, a[1] + p[1] / n, a[2] + p[2] / n]);
                let shrink = |p: &mut [f32; 3]| (0..3).for_each(|k| p[k] = c[k] + (p[k] - c[k]) * 0.995);
                o.mesh.positions.iter_mut().for_each(shrink);
                o.edges.iter_mut().for_each(|e| e.points.iter_mut().for_each(shrink));
                objs.push(o);
            }
        }
        self.asm.objs = map;
        self.vp.set_objects(objs);
        self.vp.set_overlay(crate::viewport::Overlay::default());
        self.planes_at = None;
    }

    /// Placement of a new instance whose bounding-box center (in XY, bottom at z = 0) lands on `w`.
    fn insert_placement(&self, of: InstanceOf, w: [f64; 3]) -> fc_assembly::Frame {
        let b = match of {
            InstanceOf::Part(p) => self.ws.part(p).and_then(|e| e.shape()).map(|s| s.bbox()),
            InstanceOf::Assembly(a) => self.ws.instance_shape(a, &[]).map(|s| s.bbox()),
        }
        .unwrap_or([0.0; 6]);
        fc_assembly::Frame::translation([w[0] - 0.5 * (b[0] + b[3]), w[1] - 0.5 * (b[1] + b[4]), w[2] - b[2]])
    }

    /// The parts a pending insert shows, with their placements inside it.
    fn pending_leaves(&self, of: InstanceOf) -> Vec<(PartId, fc_assembly::Frame)> {
        match of {
            InstanceOf::Part(p) => vec![(p, fc_assembly::Frame::IDENTITY)],
            InstanceOf::Assembly(a) => self.ws.leaves(a).into_iter().filter(|l| l.visible).map(|l| (l.part, l.placement)).collect(),
        }
    }

    pub(super) fn obj_inst(&self, object: usize) -> Option<InstanceId> {
        self.asm.objs.get(object).copied()
    }

    // ------------------------------------------------------------- connectors

    /// Part mate connectors of every visible instance: (reference, world frame, label).
    fn part_connectors(&self) -> Vec<(JointRef, MFrame, String)> {
        let Some(asm) = self.asm_ref() else { return vec![] };
        let mut out = vec![];
        for inst in asm.instances.iter().filter(|i| i.visible) {
            let fc_assembly::InstanceOf::Part(p) = inst.of else { continue };
            let Some(doc) = self.ws.part(p).map(|e| &e.doc) else { continue };
            for f in doc.features.iter().filter(|f| matches!(f.kind, FeatureKind::MateConnector(_))) {
                if let Ok(local) = doc.connector_frame(f.id) {
                    out.push((JointRef::PartConnector { instance: inst.id, connector: f.id }, world(&inst.placement, &local), format!("{}.{}", inst.name, f.name)));
                }
            }
        }
        out
    }

    /// Assembly-level mate connectors: (reference, world frame, label).
    fn asm_connectors(&self) -> Vec<(JointRef, MFrame, String)> {
        let Some(asm) = self.asm_ref() else { return vec![] };
        asm.connectors
            .iter()
            .filter_map(|c| {
                let inst = asm.instance(c.instance)?;
                Some((JointRef::AssemblyConnector(c.id), mate_ui::from_asm(&inst.placement.mul(&c.local)), c.name.clone()))
            })
            .collect()
    }

    /// World frame of a joint reference (without joint offsets).
    fn ref_frame(&self, r: &JointRef) -> Option<MFrame> {
        let asm = self.asm_ref()?;
        match r {
            JointRef::PartConnector { instance, connector } => {
                let inst = asm.instance(*instance)?;
                let fc_assembly::InstanceOf::Part(p) = inst.of else { return None };
                let local = self.ws.part(p)?.doc.connector_frame(*connector).ok()?;
                Some(world(&inst.placement, &local))
            }
            JointRef::AssemblyConnector(c) => {
                let c = asm.connectors.iter().find(|x| x.id == *c)?;
                Some(mate_ui::from_asm(&asm.instance(c.instance)?.placement.mul(&c.local)))
            }
            JointRef::Geometry { instance, element, implicit } => {
                let inst = asm.instance(*instance)?;
                let fc_assembly::InstanceOf::Part(p) = inst.of else { return None };
                let shape = self.ws.part(p)?.shape()?;
                let (k, n) = fc_part::TopoKind::parse(element)?;
                let infer = match implicit {
                    ImplicitPoint::Infer(i) => *i,
                    _ => fc_part::Infer::Auto,
                };
                Some(world(&inst.placement, &fc_part::mate::infer_frame(shape, k, n, infer).ok()?))
            }
            JointRef::Frame { instance, local } => Some(mate_ui::from_asm(&asm.instance(*instance)?.placement.mul(local))),
            JointRef::Origin { instance } => Some(mate_ui::from_asm(&asm.instance(*instance)?.placement)),
        }
    }

    pub(super) fn ref_label(&self, r: &JointRef) -> String {
        let asm = self.asm_ref();
        match r {
            JointRef::PartConnector { instance, connector } => {
                let n = self.inst_part(*instance).and_then(|p| self.ws.part(p)).and_then(|e| e.doc.get(*connector)).map(|f| f.label.clone()).unwrap_or_else(|| "?".into());
                format!("{}.{n}", self.inst_name(*instance))
            }
            JointRef::AssemblyConnector(c) => asm.and_then(|a| a.connectors.iter().find(|x| x.id == *c)).map(|c| format!("{}.{}", self.inst_name(c.instance), c.name)).unwrap_or_else(|| "?".into()),
            JointRef::Geometry { instance, element, implicit } => {
                let how = match implicit {
                    ImplicitPoint::Infer(i) => {
                        let (k, _) = fc_part::TopoKind::parse(element).unwrap_or((fc_part::TopoKind::Face, 0));
                        let geom = self.inst_part(*instance).and_then(|p| self.ws.part(p)).and_then(|e| e.shape()).and_then(|s| fc_part::TopoKind::parse(element).map(|(k, n)| mate_ui::element_geom(s, k, n))).unwrap_or(0);
                        format!(" ({})", mate_ui::infer_label(k, geom, *i).to_lowercase())
                    }
                    _ => String::new(),
                };
                format!("{}.{element}{how}", self.inst_name(*instance))
            }
            JointRef::Frame { instance, .. } => format!("{} (placement)", self.inst_name(*instance)),
            JointRef::Origin { instance } => format!("{} (origin)", self.inst_name(*instance)),
        }
    }

    /// Test hook: screen position of a triad by name (see `input_script`).
    pub(super) fn triad_screen(&self, name: &str) -> Option<egui::Pos2> {
        let f = if self.active_assembly().is_some() {
            self.asm_connectors().into_iter().chain(self.part_connectors()).find(|(_, _, l)| l == name).map(|x| x.1)
        } else {
            self.doc.by_name(name).and_then(|f| self.doc.connector_frame(f.id).ok())
        }?;
        // a bit along Z so the click lands on the triad rather than on the disc's center geometry
        self.vp.world_to_screen(f.origin)
    }

    /// Is a pickable connector triad under `pos` (either workbench)?
    pub(super) fn triad_under(&self, pos: egui::Pos2) -> bool {
        if self.active_assembly().is_some() {
            let mut t = self.asm_connectors();
            if matches!(self.asm.task, Some(AsmTask::Joint(_))) {
                t.extend(self.part_connectors());
            }
            t.iter().any(|(_, f, _)| mate_ui::hit(&self.vp, f, pos).is_some())
        } else {
            self.mate_task().is_none() && self.editor.is_none() && !self.picking && self.connector_at(pos).is_some()
        }
    }

    // ------------------------------------------------------------------ hover

    /// What the cursor is over: a connector triad (when they are pickable) or snapped geometry.
    fn asm_hover(&self, ptr: Option<egui::Pos2>, hovered: Option<&Pick>) -> Option<Hover> {
        let pos = ptr?;
        let picking_refs = matches!(self.asm.task, Some(AsmTask::Joint(_)));
        let mut triads = self.asm_connectors();
        if picking_refs {
            triads.extend(self.part_connectors());
        }
        let best = triads.into_iter().filter_map(|(r, f, _)| mate_ui::hit(&self.vp, &f, pos).map(|d| (d, r, f))).min_by(|a, b| a.0.total_cmp(&b.0));
        if let Some((_, r, f)) = best {
            return Some(Hover::Triad(r, f));
        }
        if !matches!(self.asm.task, Some(AsmTask::Joint(_)) | Some(AsmTask::Connector(_))) {
            return None;
        }
        let h = hovered?;
        let inst = self.obj_inst(h.object)?;
        let asm = self.asm_ref()?;
        let pl = asm.instance(inst)?.placement;
        let shape = self.ws.part(self.inst_part(inst)?)?.shape()?;
        let kind = mate_ui::topo_kind(h.kind);
        let index = h.id as usize + 1;
        let cands = mate_ui::candidates(shape, kind, index);
        let to_world = |f: &MFrame| world(&pl, f);
        let i = mate_ui::snap(&self.vp, &cands, pos, &to_world)?;
        let (infer, f) = cands[i];
        Some(Hover::Geom {
            inst,
            element: format!("{}{index}", kind.prefix()),
            infer,
            frame: to_world(&f),
            points: cands.iter().map(|(_, f)| to_world(f).origin).collect(),
            chosen: i,
        })
    }

    /// Before the 3D view handles input: start dragging a part when the left button goes
    /// down on a free instance (no task open).
    pub(super) fn asm_pre_view(&mut self, ctx: &egui::Context) {
        let (pressed, pos, released) = ctx.input(|i| (i.pointer.primary_pressed(), i.pointer.interact_pos(), i.pointer.primary_released()));
        if self.asm.drag.is_some() && released {
            self.end_drag();
        }
        if pressed && self.explode_pre_view(ctx) {
            return;
        }
        if !pressed || self.asm.task.is_some() || self.asm.drag.is_some() {
            return;
        }
        let Some(pos) = pos else { return };
        let Some(p) = self.vp.pick(pos) else { return };
        // a triad under the cursor is clicked, not dragged
        if self.asm_connectors().iter().any(|(_, f, _)| mate_ui::hit(&self.vp, f, pos).is_some()) {
            return;
        }
        let Some(inst) = self.obj_inst(p.object) else { return };
        let Some(i) = self.asm_ref().and_then(|a| a.instance(inst)).cloned() else { return };
        if i.grounded {
            return;
        }
        let local = i.placement.inverse().transform_point(p.point);
        let n = self.vp.view_dir();
        let plane = crate::viewport::Plane { origin: p.point, normal: n, xdir: [1.0, 0.0, 0.0] };
        let before = self.state_json();
        // dragging works on the assembled placements
        self.asm.explode_show = None;
        self.asm.drag = Some(Drag { inst, local, plane, before, moved: false });
        self.vp.capture_primary = true;
    }

    fn end_drag(&mut self) {
        self.vp.capture_primary = false;
        let Some(d) = self.asm.drag.take() else { return };
        if d.moved {
            self.undo.push(d.before);
            self.redo.clear();
            self.dirty = true;
            self.solve_active(true);
            self.status = format!("Moved {}: {}", self.inst_name(d.inst), self.solver_message().0);
        }
    }

    /// Mouse interaction with the assembly view.
    pub(super) fn asm_view_input(&mut self, ui: &mut egui::Ui, ctx: &egui::Context, r: &crate::viewport::ViewportResponse) {
        let Some(aid) = self.active_assembly() else { return };
        let ptr = ctx.input(|i| i.pointer.hover_pos()).filter(|p| r.response.rect.contains(*p) && p.x < r.response.rect.right() - tasks::TASK_W);
        // dragging a part
        if let Some(d) = &self.asm.drag {
            let delta = ctx.input(|i| i.pointer.delta());
            if delta != egui::Vec2::ZERO {
                if let Some(pos) = ctx.input(|i| i.pointer.interact_pos()) {
                    if let Some(w) = self.vp.screen_to_plane(pos, &d.plane) {
                        let (inst, local) = (d.inst, d.local);
                        let opts = SolveOptions { diagnose: false, ..Default::default() };
                        if let Some(Ok(_)) = self.ws.drag_instance(aid, inst, DragTarget::Point { local, world: w }, &opts) {
                            if let Some(d) = self.asm.drag.as_mut() {
                                d.moved = true;
                            }
                            self.rebuild_scene();
                        }
                    }
                }
            }
            ctx.request_repaint();
            return;
        }
        // exploded view task: dragging an axis of the dragger
        if self.explode_view_input(ctx) {
            self.draw_asm_overlay(ui);
            return;
        }
        // insert: the pending part follows the cursor on the XY plane; click places it
        if let Some(p) = self.asm.pending {
            let w = ptr.and_then(|pos| self.vp.screen_to_plane(pos, &crate::viewport::Plane { origin: [0.0; 3], normal: [0.0, 0.0, 1.0], xdir: [1.0, 0.0, 0.0] }));
            if w != self.asm.cursor_world {
                self.asm.cursor_world = w;
                self.rebuild_scene();
            }
            if r.response.clicked() {
                if let Some(w) = w {
                    self.place_instance(p, self.insert_placement(p, w));
                }
            }
            return;
        }
        let hover = self.asm_hover(ptr, r.hovered.as_ref());
        self.asm.hover = hover.clone();
        if r.response.clicked() {
            match self.asm.task {
                Some(AsmTask::Joint(_)) => {
                    if let Some(h) = hover {
                        self.joint_pick(h);
                    }
                }
                Some(AsmTask::Connector(_)) => {
                    if let Some(Hover::Geom { inst, element, infer, .. }) = hover {
                        if let Some(AsmTask::Connector(t)) = self.asm.task.as_mut() {
                            t.pick = Some((inst, element, infer));
                        }
                    }
                }
                Some(AsmTask::Insert { .. }) => {}
                Some(AsmTask::Interference(_)) => {
                    let inst = r.clicked.as_ref().and_then(|p| self.obj_inst(p.object));
                    self.interf_pick(inst);
                }
                Some(AsmTask::Explode(_)) | Some(AsmTask::Bom(_)) | None => {
                    let multi = ctx.input(|i| i.modifiers.ctrl || i.modifiers.command);
                    let s = match &hover {
                        Some(Hover::Triad(JointRef::AssemblyConnector(c), _)) => Some(AsmSel::Conn(*c)),
                        _ => r.clicked.as_ref().and_then(|p| self.obj_inst(p.object)).map(AsmSel::Inst),
                    };
                    match s {
                        Some(s) if multi => {
                            if let Some(i) = self.asm.sel.iter().position(|x| *x == s) {
                                self.asm.sel.remove(i);
                            } else {
                                self.asm.sel.push(s);
                            }
                        }
                        Some(s) => self.asm.sel = vec![s],
                        None if !multi => self.asm.sel.clear(),
                        None => {}
                    }
                    if let Some(AsmSel::Inst(i)) = self.asm.sel.last() {
                        self.status = format!("Selected: {}", self.inst_name(*i));
                    }
                    self.rebuild_scene();
                }
            }
        }
        if r.response.double_clicked() && self.asm.task.is_none() {
            if let Some(AsmSel::Inst(i)) = self.asm.sel.last().copied() {
                // like FreeCAD, double-clicking a part opens it for editing
                self.open_instance(i);
            }
        }
        self.draw_asm_overlay(ui);
    }

    /// Triads over the view: assembly connectors, part connectors while picking joint
    /// references, the selected joints' frames, and the hover preview.
    fn draw_asm_overlay(&self, ui: &egui::Ui) {
        let vr = self.view_rect;
        let clip = egui::Rect::from_min_max(vr.min, egui::pos2(vr.right() - tasks::TASK_W, vr.bottom()));
        let p = ui.painter().with_clip_rect(clip);
        let hovered_ref = match &self.asm.hover {
            Some(Hover::Triad(r, _)) => Some(r.clone()),
            _ => None,
        };
        let picked: Vec<JointRef> = match &self.asm.task {
            Some(AsmTask::Joint(t)) => t.refs.clone(),
            _ => vec![],
        };
        let look = |r: &JointRef, sel: bool| if hovered_ref.as_ref() == Some(r) { Look::Hover } else if sel || picked.contains(r) { Look::Selected } else { Look::Normal };
        if matches!(self.asm.task, Some(AsmTask::Joint(_))) {
            for (r, f, _) in self.part_connectors() {
                mate_ui::draw(&p, &self.vp, &f, look(&r, false));
            }
        }
        for (r, f, _) in self.asm_connectors() {
            let sel = matches!(r, JointRef::AssemblyConnector(c) if self.asm.sel.contains(&AsmSel::Conn(c)));
            mate_ui::draw(&p, &self.vp, &f, look(&r, sel));
        }
        // picked geometry references of the joint being made
        for r in &picked {
            if matches!(r, JointRef::Geometry { .. }) {
                if let Some(f) = self.ref_frame(r) {
                    mate_ui::draw(&p, &self.vp, &f, Look::Selected);
                }
            }
        }
        // the selected joints' two frames (FreeCAD shows the JCS of a selected joint)
        if let Some(asm) = self.asm_ref() {
            for s in &self.asm.sel {
                if let AsmSel::Joint(j) = s {
                    if let Ok((a, b)) = asm.joint_frames(&self.ws, *j) {
                        mate_ui::draw(&p, &self.vp, &mate_ui::from_asm(&a), Look::Selected);
                        mate_ui::draw(&p, &self.vp, &mate_ui::from_asm(&b), Look::Normal);
                    }
                }
            }
        }
        if let Some(asm) = self.asm_ref() {
            for j in self.asm.problem_joints() {
                if self.asm.report.as_ref().is_some_and(|r| r.status == SolveStatus::Redundant) {
                    continue;
                }
                if let Ok((a, _)) = asm.joint_frames(&self.ws, j) {
                    mate_ui::draw(&p, &self.vp, &mate_ui::from_asm(&a), Look::Error);
                }
            }
        }
        if let Some(Hover::Triad(_, f)) = &self.asm.hover {
            // the hovered triad on top of the others
            mate_ui::draw(&p, &self.vp, f, Look::Hover);
        }
        if let Some(Hover::Geom { frame, points, chosen, inst, element, infer }) = &self.asm.hover {
            mate_ui::draw_points(&p, &self.vp, points, Some(*chosen));
            mate_ui::draw(&p, &self.vp, frame, Look::Ghost);
            let r = JointRef::Geometry { instance: *inst, element: element.clone(), implicit: ImplicitPoint::Infer(*infer) };
            mate_ui::label(&p, &self.vp, frame, &self.ref_label(&r));
        }
        if let Some(Hover::Triad(r, f)) = &self.asm.hover {
            mate_ui::label(&p, &self.vp, f, &self.ref_label(r));
        }
        if let Some(AsmTask::Connector(t)) = &self.asm.task {
            if let Some(f) = self.conn_task_frame(t) {
                mate_ui::draw(&p, &self.vp, &f, Look::Selected);
            }
        }
        self.draw_tools_overlay(&p);
    }

    // ------------------------------------------------------------- commands

    pub(super) fn asm_insert(&mut self) {
        if self.active_assembly().is_none() || !self.idle() {
            return;
        }
        let before = self.state_json();
        self.asm.task = Some(AsmTask::Insert { before, added: 0 });
        self.status = "Pick a part in the list, then click in the 3D view to place it".into();
    }

    fn place_instance(&mut self, of: InstanceOf, pl: fc_assembly::Frame) {
        let Some(a) = self.active_assembly() else { return };
        let first = self.asm_ref().is_some_and(|x| x.instances.is_empty());
        let name = match of {
            InstanceOf::Part(p) => self.part_name(p),
            InstanceOf::Assembly(s) => self.ws.assembly(s).map(|x| x.name.clone()).unwrap_or_default(),
        };
        let id = match of {
            InstanceOf::Part(p) => self.ws.assembly_mut(a).map(|asm| asm.add_part_instance(&name, p, pl)),
            // FreeCAD 1.1 inserts an assembly as a (rigid) sub-assembly link
            InstanceOf::Assembly(s) => match self.ws.add_sub_assembly(a, &name, s, pl) {
                Ok(id) => Some(id),
                Err(e) => {
                    self.status = format!("Cannot insert {name}: {e}");
                    self.asm.pending = None;
                    return;
                }
            },
        };
        // FreeCAD grounds the first part inserted into an assembly
        if let (true, Some(id), Some(asm)) = (first, id, self.ws.assembly_mut(a)) {
            asm.set_grounded(id, true);
        }
        if let Some(AsmTask::Insert { added, .. }) = self.asm.task.as_mut() {
            *added += 1;
        }
        self.dirty = true;
        self.asm.pending = None;
        self.asm.cursor_world = None;
        self.solve_active(true);
        self.status = if first { format!("Inserted {name} (grounded, as the first part)") } else { format!("Inserted {name}") };
    }

    /// Insert task: choose a part or an assembly. The first instance goes to the origin; later
    /// ones follow the cursor until a click places them.
    fn insert_choose(&mut self, of: InstanceOf) {
        let empty = self.asm_ref().is_some_and(|a| a.instances.is_empty());
        if empty {
            self.place_instance(of, fc_assembly::Frame::IDENTITY);
            // leave room around the first part for placing the next ones
            self.vp.fit_all();
            self.vp.zoom(0.55);
        } else {
            self.asm.pending = Some(of);
            let name = match of {
                InstanceOf::Part(p) => self.part_name(p),
                InstanceOf::Assembly(s) => self.ws.assembly(s).map(|x| x.name.clone()).unwrap_or_default(),
            };
            self.status = format!("Click in the 3D view to place {name}");
        }
    }

    pub(super) fn toggle_grounded(&mut self) {
        let insts: Vec<InstanceId> = self.asm.sel.iter().filter_map(|s| if let AsmSel::Inst(i) = s { Some(*i) } else { None }).collect();
        if insts.is_empty() {
            self.dialog = Some(Dialog::Message("Select a part of the assembly first (in the tree or the 3D view).".into()));
            return;
        }
        self.asm_change(|asm| {
            for i in insts {
                let g = asm.instance(i).is_some_and(|x| x.grounded);
                asm.set_grounded(i, !g);
            }
        });
    }

    pub(super) fn asm_delete_selected(&mut self) {
        let sel = self.asm.sel.clone();
        if sel.is_empty() {
            return;
        }
        self.asm_change(|asm| {
            for s in sel {
                match s {
                    AsmSel::Inst(i) => {
                        asm.remove_instance(i);
                    }
                    AsmSel::Joint(j) => {
                        asm.remove_joint(j);
                    }
                    AsmSel::Conn(c) => {
                        asm.joints.retain(|j| j.a != JointRef::AssemblyConnector(c) && j.b != JointRef::AssemblyConnector(c));
                        asm.connectors.retain(|x| x.id != c);
                    }
                }
            }
        });
        self.asm.sel.clear();
        self.rebuild_scene();
    }

    pub(super) fn placements(&self) -> Vec<(InstanceId, fc_assembly::Frame)> {
        self.asm_ref().map(|a| a.instances.iter().map(|i| (i.id, i.placement)).collect()).unwrap_or_default()
    }

    /// Start a joint tool. Two references already selected (connectors) are used directly.
    pub(super) fn start_joint(&mut self, kind: JointKind) {
        if self.active_assembly().is_none() || !self.idle() {
            return;
        }
        let before = self.state_json();
        let t = JointTask { kind, refs: vec![], joint: None, editing: false, before, start: self.placements(), offset: 0.0, rotation: 0.0, sense: Sense::Auto, limits: Limits::default() };
        self.asm.explode_show = None;
        self.asm.task = Some(AsmTask::Joint(t));
        self.asm.sel.clear();
        self.fields.clear();
        self.status = "Select two elements: click a mate connector or a face, edge or vertex of each part".into();
        self.rebuild_scene();
    }

    pub(super) fn edit_joint(&mut self, id: JointId) {
        if !self.idle() {
            return;
        }
        let Some(j) = self.asm_ref().and_then(|a| a.joint(id)).cloned() else { return };
        let before = self.state_json();
        let rot = j.offset_b.rotation_vector();
        let t = JointTask {
            kind: j.kind,
            refs: vec![j.a.clone(), j.b.clone()],
            joint: Some(id),
            editing: true,
            before,
            start: self.placements(),
            offset: j.offset_b.pos[2],
            rotation: rot[2].to_degrees(),
            sense: j.sense,
            limits: j.limits,
        };
        self.asm.task = Some(AsmTask::Joint(t));
        self.asm.sel = vec![AsmSel::Joint(id)];
        self.fields.clear();
        self.status = format!("Editing {}", j.name);
    }

    fn joint_pick(&mut self, h: Hover) {
        let r = match h {
            Hover::Triad(r, _) => r,
            Hover::Geom { inst, element, infer, .. } => JointRef::Geometry { instance: inst, element, implicit: ImplicitPoint::Infer(infer) },
        };
        let inst_of = |app: &App, r: &JointRef| match r {
            JointRef::AssemblyConnector(c) => app.asm_ref().and_then(|a| a.connectors.iter().find(|x| x.id == *c)).map(|x| x.instance),
            other => other.instance(),
        };
        let ri = inst_of(self, &r);
        let Some(AsmTask::Joint(t)) = self.asm.task.as_ref() else { return };
        let mut refs = t.refs.clone();
        if let Some(i) = refs.iter().position(|x| *x == r) {
            refs.remove(i);
        } else if refs.len() == 1 && inst_of(self, &refs[0]) == ri {
            self.status = "Pick the second element on another part".into();
            return;
        } else {
            if refs.len() == 2 {
                refs.pop();
            }
            refs.push(r);
        }
        if let Some(AsmTask::Joint(t)) = self.asm.task.as_mut() {
            t.refs = refs;
        }
        self.apply_joint_task();
    }

    /// Put the joint being edited into the assembly (or take it out while it has fewer than
    /// two references), pre-solve it from the task's start placements and solve: the parts
    /// snap into place as you pick and type.
    fn apply_joint_task(&mut self) {
        let Some(a) = self.active_assembly() else { return };
        let Some(AsmTask::Joint(t)) = self.asm.task.as_ref() else { return };
        let (kind, refs, jid, start, offset, rotation, sense, limits) = (t.kind, t.refs.clone(), t.joint, t.start.clone(), t.offset, t.rotation, t.sense, t.limits);
        let Some(asm) = self.ws.assembly_mut(a) else { return };
        for (i, pl) in &start {
            if let Some(x) = asm.instance_mut(*i) {
                x.placement = *pl;
            }
        }
        let mut new_id = jid;
        if refs.len() == 2 {
            let mut j = Joint::new(kind, refs[0].clone(), refs[1].clone()).with_offset(offset).with_rotation(rotation).with_sense(sense).with_limits(limits);
            match jid.and_then(|id| asm.joint_mut(id)) {
                Some(old) => {
                    j.id = old.id;
                    j.name = old.name.clone();
                    j.offset_a = old.offset_a;
                    j.suppressed = old.suppressed;
                    *old = j;
                }
                None => new_id = Some(asm.add_joint(j)),
            }
        } else if let Some(id) = jid {
            asm.remove_joint(id);
            new_id = None;
        }
        if let Some(AsmTask::Joint(t)) = self.asm.task.as_mut() {
            t.joint = new_id;
        }
        // FreeCAD's pre-solve ("match JCS") moves the free side onto the other one
        if let Some(id) = new_id {
            if kind.uses_presolve() {
                let mut asm = self.ws.assembly(a).cloned().unwrap();
                let _ = asm.presolve_joint(&self.ws, id);
                *self.ws.assembly_mut(a).unwrap() = asm;
            }
        }
        self.dirty = true;
        self.solve_active(true);
        if let Some(id) = new_id {
            if let Some(name) = self.asm_ref().and_then(|x| x.joint(id)).map(|j| j.name.clone()) {
                self.status = format!("{name}: {}", self.solver_message().0);
            }
        }
    }

    fn joint_task_ok(&mut self) {
        self.commit_focused_asm_field();
        let Some(AsmTask::Joint(t)) = self.asm.task.take() else { return };
        if t.refs.len() < 2 {
            self.restore(&t.before);
            self.status = "Joint not created: it needs two elements".into();
            return;
        }
        self.undo.push(t.before);
        self.redo.clear();
        if let Some(j) = t.joint {
            self.asm.sel = vec![AsmSel::Joint(j)];
        }
        self.fields.clear();
        self.solve_active(true);
        self.status = self.solver_message().0;
    }

    pub(super) fn asm_task_cancel(&mut self) {
        self.asm.pending = None;
        self.asm.cursor_world = None;
        match self.asm.task.take() {
            Some(AsmTask::Joint(t)) => self.restore(&t.before),
            Some(AsmTask::Insert { before, added }) => {
                if added > 0 {
                    self.restore(&before);
                }
            }
            Some(AsmTask::Connector(t)) => self.restore(&t.before),
            Some(AsmTask::Bom(t)) => self.restore(&t.before),
            Some(AsmTask::Explode(t)) => self.restore(&t.before),
            Some(AsmTask::Interference(_)) => {}
            None => {}
        }
        self.asm.highlight = None;
        self.fields.clear();
        self.rebuild_scene();
        self.status = "Cancelled".into();
    }

    fn asm_task_ok(&mut self) {
        match self.asm.task {
            Some(AsmTask::Joint(_)) => self.joint_task_ok(),
            Some(AsmTask::Insert { .. }) => {
                if let Some(AsmTask::Insert { before, added }) = self.asm.task.take() {
                    if added > 0 {
                        self.undo.push(before);
                        self.redo.clear();
                    }
                }
                self.asm.pending = None;
                self.asm.cursor_world = None;
                self.rebuild_scene();
            }
            Some(AsmTask::Connector(_)) => self.conn_task_ok(),
            Some(AsmTask::Interference(_)) | Some(AsmTask::Bom(_)) | Some(AsmTask::Explode(_)) => self.tool_task_ok(),
            None => {}
        }
    }

    /// Escape / Enter in the assembly workbench.
    pub(super) fn asm_key(&mut self, ok: bool) -> bool {
        if self.asm.task.is_none() {
            return false;
        }
        if ok {
            self.asm_task_ok();
        } else if self.asm.pending.is_some() {
            self.asm.pending = None;
            self.asm.cursor_world = None;
            self.rebuild_scene();
        } else {
            self.asm_task_cancel();
        }
        true
    }

    // ---------------------------------------------------- assembly connectors

    pub(super) fn start_asm_connector(&mut self) {
        if self.active_assembly().is_none() || !self.idle() {
            return;
        }
        let before = self.state_json();
        self.asm.task = Some(AsmTask::Connector(ConnTask { before, pick: None, flip: false, turns: 0, offset: 0.0 }));
        self.status = "Click a face, edge or vertex of a part: the connector snaps to its inferred point".into();
    }

    fn conn_task_local(&self, t: &ConnTask) -> Option<(InstanceId, MFrame)> {
        let (inst, el, infer) = t.pick.as_ref()?;
        let shape = self.ws.part(self.inst_part(*inst)?)?.shape()?;
        let (k, n) = fc_part::TopoKind::parse(el)?;
        let base = fc_part::mate::infer_frame(shape, k, n, *infer).ok()?;
        let mc = fc_part::MateConnector { anchor: fc_part::Anchor::Origin, flip_z: t.flip, x_turns: t.turns, offset: [0.0, 0.0, t.offset], rotation: [0.0; 3] };
        Some((*inst, mc.adjust(base)))
    }

    fn conn_task_frame(&self, t: &ConnTask) -> Option<MFrame> {
        let (inst, local) = self.conn_task_local(t)?;
        Some(world(&self.asm_ref()?.instance(inst)?.placement, &local))
    }

    fn conn_task_ok(&mut self) {
        self.commit_focused_asm_field();
        let Some(AsmTask::Connector(t)) = self.asm.task.take() else { return };
        let Some((inst, local)) = self.conn_task_local(&t) else {
            self.status = "No connector created: pick a face, edge or vertex first".into();
            return;
        };
        let el = t.pick.as_ref().map(|p| p.1.clone());
        let Some(a) = self.active_assembly() else { return };
        self.undo.push(t.before);
        self.redo.clear();
        if let Some(asm) = self.ws.assembly_mut(a) {
            let id = asm.add_connector("MateConnector", inst, mate_ui::to_asm(&local));
            if let Some(c) = asm.connectors.iter_mut().find(|c| c.id == id) {
                c.anchor = el;
            }
            self.asm.sel = vec![AsmSel::Conn(id)];
        }
        self.dirty = true;
        self.solve_active(true);
        self.status = "Mate connector added to the assembly".into();
    }

    // ------------------------------------------------------------ messages

    /// FreeCAD-style solver message and whether it is a problem.
    pub(super) fn solver_message(&self) -> (String, Option<Color32>) {
        let Some(asm) = self.asm_ref() else { return (String::new(), None) };
        if asm.instances.is_empty() {
            return ("Empty assembly: insert parts with Insert Component".into(), None);
        }
        let Some(r) = &self.asm.report else { return ("Not solved yet".into(), None) };
        let names = |ids: &[JointId]| ids.iter().filter_map(|j| asm.joint(*j)).map(|j| j.name.clone()).collect::<Vec<_>>().join(", ");
        if r.no_ground {
            return ("No grounded part: ground one part (Toggle Grounded) to solve the assembly".into(), Some(theme::WARNING));
        }
        if let Some((j, why)) = r.unresolved_joints.first() {
            return (format!("{}: {why}", asm.joint(*j).map(|x| x.name.clone()).unwrap_or_default()), Some(theme::ERROR));
        }
        match r.status {
            SolveStatus::Conflicting => (format!("Conflicting joints: {} contradict each other. Delete or suppress one of them.", names(&r.conflicting_joints)), Some(theme::ERROR)),
            SolveStatus::Failed => (format!("The solver failed{}", if r.unsatisfied_joints.is_empty() { String::new() } else { format!(": {} cannot be satisfied", names(&r.unsatisfied_joints)) }), Some(theme::ERROR)),
            SolveStatus::Redundant => (format!("Redundant joints: {} (implied by the others) · {}", names(&r.redundant_joints), dof_text(r.dof)), Some(theme::WARNING)),
            SolveStatus::Solved if r.dof == 0 => (dof_text(0), Some(theme::GREEN)),
            SolveStatus::Solved => (dof_text(r.dof), Some(UNDER)),
        }
    }
}

/// Under-constrained (amber, like the sketcher's under-constrained state).
const UNDER: Color32 = Color32::from_rgb(240, 190, 90);

fn dof_text(dof: usize) -> String {
    if dof == 0 { "Fully constrained".into() } else { format!("Under-constrained: {dof} degree{} of freedom", if dof == 1 { "" } else { "s" }) }
}

// ------------------------------------------------------------------ task panels

impl App {
    /// A plain number field for assembly values (mm or °); applies live while typing.
    pub(super) fn num_field(&mut self, ui: &mut egui::Ui, id: &str, v: &mut f64, unit: &str) -> bool {
        let key = format!("asm:{id}");
        let shown = format!("{} {unit}", fmt_num(*v)).trim().to_string();
        let buf = self.fields.entry(key.clone()).or_insert_with(|| shown.clone());
        let w = ui.available_width() - 4.0;
        let resp = ui.add(egui::TextEdit::singleline(buf).desired_width(w).margin(egui::Margin::symmetric(5, 2)));
        widgets::reg(format!("field:{id}"), resp.rect);
        let parse = |t: &str| fc_part::expr::parse(t.trim().trim_end_matches('°')).ok().and_then(|e| e.eval(&mut |n| Err(fc_part::expr::ExprError(format!("unknown {n}")))).ok());
        let mut changed = false;
        if resp.changed() {
            if let Some(x) = parse(buf) {
                if (x - *v).abs() > 1e-12 {
                    *v = x;
                    changed = true;
                }
            }
        }
        if resp.lost_focus() {
            self.fields.remove(&key);
        }
        changed
    }

    pub(super) fn commit_focused_asm_field(&mut self) {
        self.fields.retain(|k, _| !k.starts_with("asm:"));
    }

    /// The Tasks panel while an assembly is open.
    pub(super) fn asm_task_panel(&mut self, ui: &mut egui::Ui) {
        match &self.asm.task {
            Some(AsmTask::Joint(_)) => self.joint_panel(ui),
            Some(AsmTask::Insert { .. }) => self.insert_panel(ui),
            Some(AsmTask::Connector(_)) => self.conn_panel(ui),
            Some(AsmTask::Interference(_)) => self.interf_panel(ui),
            Some(AsmTask::Bom(_)) => self.bom_panel(ui),
            Some(AsmTask::Explode(_)) => self.explode_panel(ui),
            None => self.asm_overview_panel(ui),
        }
    }

    pub(super) fn ok_cancel(&mut self, ui: &mut egui::Ui) -> (bool, bool) {
        let bw = 118.0;
        let mut out = (false, false);
        ui.horizontal(|ui| {
            ui.add_space(((ui.available_width() - bw * 2.0 - 8.0) * 0.5).max(0.0));
            out.0 = widgets::push_button(ui, "task:cancel", "Cancel", bw, false).clicked();
            out.1 = widgets::push_button(ui, "task:ok", "OK", bw, true).clicked();
        });
        ui.add_space(8.0);
        out
    }

    pub(super) fn link_row(ui: &mut egui::Ui, reg: &str, icon: &str, text: &str, tip: &str) -> bool {
        let w = ui.available_width();
        let (rect, resp) = ui.allocate_exact_size(egui::vec2(w, 22.0), egui::Sense::click());
        widgets::reg(reg.to_string(), rect);
        if resp.hovered() {
            ui.painter().rect_filled(rect, 3.0, theme::HOVER_ROW);
        }
        crate::icons::paint(ui.painter(), icon, egui::Rect::from_center_size(egui::pos2(rect.left() + 11.0, rect.center().y), egui::vec2(16.0, 16.0)), Color32::WHITE);
        ui.painter().text(egui::pos2(rect.left() + 26.0, rect.center().y), egui::Align2::LEFT_CENTER, text, egui::FontId::proportional(13.0), theme::TEXT);
        let resp = if tip.is_empty() { resp } else { resp.on_hover_text(tip) };
        resp.clicked()
    }

    /// Free parts and their remaining DOF in plain words.
    fn free_parts_text(&self) -> Option<String> {
        let asm = self.asm_ref()?;
        let r = self.asm.report.as_ref()?;
        if !r.ok() || r.no_ground {
            return None;
        }
        let free: Vec<&str> = asm.instances.iter().filter(|i| !i.grounded).map(|i| i.name.as_str()).collect();
        if free.is_empty() {
            return Some("Every part is grounded".into());
        }
        Some(if r.dof == 0 { format!("{} held in place by the joints", free.join(", ")) } else { format!("{} can still move: drag {} in the 3D view", free.join(", "), if free.len() == 1 { "it" } else { "them" }) })
    }

    fn solver_box(&mut self, ui: &mut egui::Ui) {
        let (msg, col) = self.solver_message();
        let detail = self.free_parts_text();
        let problems = self.asm.problem_joints();
        let mut select = false;
        task_box(ui, "solver", "Assembly_SolveAssembly", "Solver", |ui| {
            ui.horizontal_wrapped(|ui| {
                let icon = if col == Some(theme::ERROR) {
                    "overlay_error"
                } else if col == Some(theme::GREEN) {
                    "edit_OK"
                } else {
                    "Warning"
                };
                if col.is_some() {
                    ui.add(crate::icons::image(ui.ctx(), icon, 16.0));
                }
                ui.label(RichText::new(&msg).color(col.unwrap_or(theme::TEXT)));
            });
            if let Some(d) = &detail {
                ui.add_space(2.0);
                ui.label(RichText::new(d).color(theme::TEXT_DIM));
            }
            if !problems.is_empty() {
                ui.add_space(2.0);
                let r = ui.link(RichText::new("Select the joints involved").color(Color32::from_rgb(110, 170, 255)));
                widgets::reg("solver:select", r.rect);
                select = r.clicked();
            }
        });
        if select {
            self.asm.sel = problems.into_iter().map(AsmSel::Joint).collect();
            self.rebuild_scene();
        }
    }

    fn asm_overview_panel(&mut self, ui: &mut egui::Ui) {
        let mut act: Option<chrome::Act> = None;
        egui::ScrollArea::vertical().id_salt("asm_overview").auto_shrink([false, true]).show(ui, |ui| {
            self.solver_box(ui);
            task_box(ui, "asm_insert", "Assembly_InsertLink", "Insert", |ui| {
                if Self::link_row(ui, "taskact:insert", "Assembly_InsertLink", "Component", "Insert a part of this document into the assembly") {
                    act = Some(chrome::Act::AsmInsert);
                }
                if Self::link_row(ui, "taskact:newpart", "PartDesign_Body", "New Part", "Create a new empty part (Body) and open it in Part Design") {
                    act = Some(chrome::Act::NewPart);
                }
            });
            task_box(ui, "asm_constraints", "Assembly_CreateJointFixed", "Constraints", |ui| {
                for (name, icon, tip) in JOINT_TOOLS {
                    let label = if name == "PinSlot" { "Pin Slot Mate".to_string() } else if name == "Planar" { "Planar Mate".to_string() } else { format!("{name} Joint") };
                    if Self::link_row(ui, &format!("taskact:{}", name.to_lowercase()), icon, &label, tip) {
                        act = Some(chrome::Act::Joint(name));
                    }
                }
            });
            task_box(ui, "asm_tools", "Assembly_SolveAssembly", "Tools", |ui| {
                if Self::link_row(ui, "taskact:ground", "Assembly_ToggleGrounded", "Toggle Grounded", "Fix the selected part in place (or release it)") {
                    act = Some(chrome::Act::Ground);
                }
                if Self::link_row(ui, "taskact:connector", "PartDesign_CoordinateSystem", "Mate Connector", "Add an Onshape-style mate connector anchored to a part's face, edge or vertex") {
                    act = Some(chrome::Act::AsmConnector);
                }
                if Self::link_row(ui, "taskact:solve", "Assembly_SolveAssembly", "Solve Assembly", "Solve the joints again") {
                    act = Some(chrome::Act::Solve);
                }
                if Self::link_row(ui, "taskact:explode", "Assembly_ExplodedView", "Exploded View", "Create an exploded view: move parts in steps, animate") {
                    act = Some(chrome::Act::AsmExplode);
                }
                if Self::link_row(ui, "taskact:bom", "Assembly_BillOfMaterials", "Bill of Materials", "Parts, quantities, part numbers, materials and masses") {
                    act = Some(chrome::Act::AsmBom);
                }
                if Self::link_row(ui, "taskact:interference", "Part_Common", "Check Interference", "Find overlapping parts and measure clearances") {
                    act = Some(chrome::Act::AsmInterference);
                }
            });
            task_box(ui, "asm_exchange", "Std_Export", "Exchange", |ui| {
                if Self::link_row(ui, "taskact:exportstep", "Std_Export", "Export STEP...", "Write the assembly as STEP (AP214 or AP242) with its structure") {
                    act = Some(chrome::Act::ExportStep);
                }
                if Self::link_row(ui, "taskact:exportstl", "Std_Export", "Export STL...", "Write the posed parts as one STL mesh") {
                    act = Some(chrome::Act::ExportStl);
                }
                if Self::link_row(ui, "taskact:importstep", "Std_Import", "Import STEP...", "Read a STEP assembly into the document") {
                    act = Some(chrome::Act::ImportStep);
                }
            });
        });
        if let Some(a) = act {
            self.run(a);
        }
    }

    fn insert_panel(&mut self, ui: &mut egui::Ui) {
        let (cancel, ok) = self.ok_cancel(ui);
        let filter = self.fields.get("asm:insert_search").cloned().unwrap_or_default().to_lowercase();
        let cur = self.active_assembly();
        // parts, then the other assemblies that can go in as sub-assemblies (no cycles)
        let mut items: Vec<(InstanceOf, String, bool)> = self.ws.parts.iter().map(|p| (InstanceOf::Part(p.id), p.name.clone(), p.shape().is_some())).collect();
        items.extend(self.ws.assemblies.iter().filter(|x| Some(x.id) != cur && !cur.is_some_and(|c| self.ws.uses_assembly(x.id, c))).map(|x| (InstanceOf::Assembly(x.id), x.name.clone(), !x.instances.is_empty())));
        items.retain(|(_, n, _)| n.to_lowercase().contains(filter.trim()));
        let pending = self.asm.pending;
        let mut chosen = None;
        let mut search = self.fields.get("asm:insert_search").cloned().unwrap_or_default();
        task_box(ui, "insert", "Assembly_InsertLink", "Insert", |ui| {
            let r = ui.add(egui::TextEdit::singleline(&mut search).hint_text("Search parts and assemblies...").desired_width(ui.available_width() - 4.0).margin(egui::Margin::symmetric(5, 3)));
            widgets::reg("insert:search", r.rect);
            ui.add_space(4.0);
            egui::Frame::new().fill(theme::INPUT).stroke(egui::Stroke::new(1.0, theme::BORDER)).corner_radius(3.0).inner_margin(3.0).show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.set_min_height(120.0);
                for (of, name, solid) in &items {
                    let w = ui.available_width();
                    let (rect, resp) = ui.allocate_exact_size(egui::vec2(w, 24.0), if *solid { egui::Sense::click() } else { egui::Sense::hover() });
                    widgets::reg(format!("insert:{name}"), rect);
                    let fill = if pending == Some(*of) { theme::SELECTION } else if resp.hovered() { theme::HOVER_ROW } else { Color32::TRANSPARENT };
                    ui.painter().rect_filled(rect, 2.0, fill);
                    let icon = if matches!(of, InstanceOf::Assembly(_)) { "Geoassembly" } else { "PartDesign_Body" };
                    crate::icons::paint_ex(ui.painter(), icon, egui::Rect::from_center_size(egui::pos2(rect.left() + 13.0, rect.center().y), egui::vec2(16.0, 16.0)), Color32::WHITE, !*solid);
                    ui.painter().text(egui::pos2(rect.left() + 28.0, rect.center().y), egui::Align2::LEFT_CENTER, name, egui::FontId::proportional(13.0), if *solid { theme::TEXT } else { theme::TEXT_DIM });
                    let tip = match (of, solid) {
                        (InstanceOf::Assembly(_), true) => "Click to insert this assembly as a rigid sub-assembly; then click in the 3D view to place it",
                        (InstanceOf::Assembly(_), false) => "This assembly is empty",
                        (_, true) => "Click to insert; then click in the 3D view to place it",
                        _ => "This part has no solid yet",
                    };
                    if resp.on_hover_text(tip).clicked() {
                        chosen = Some(*of);
                    }
                }
            });
            ui.add_space(6.0);
            for t in ["Click a part or an assembly to insert it", "The first part goes to the origin, grounded", "Others follow the cursor: click to place", "Insert as many as needed, then OK"] {
                ui.label(RichText::new(format!("•  {t}")).color(Color32::from_gray(190)));
            }
        });
        self.fields.insert("asm:insert_search".into(), search);
        if let Some(p) = chosen {
            self.insert_choose(p);
        }
        if ok {
            self.asm_task_ok();
        } else if cancel {
            self.asm_task_cancel();
        }
    }

    fn joint_panel(&mut self, ui: &mut egui::Ui) {
        let (cancel, ok) = self.ok_cancel(ui);
        let Some(AsmTask::Joint(t)) = self.asm.task.as_ref() else { return };
        let (kind, refs, jid, editing) = (t.kind, t.refs.clone(), t.joint, t.editing);
        let (mut offset, mut rotation, mut limits) = (t.offset, t.rotation, t.limits);
        let mut new_kind = kind;
        let mut remove: Option<usize> = None;
        let mut reverse = false;
        let mut changed = false;
        let labels: Vec<String> = refs.iter().map(|r| self.ref_label(r)).collect();
        let jname = jid.and_then(|j| self.asm_ref().and_then(|a| a.joint(j))).map(|j| j.name.clone());
        egui::ScrollArea::vertical().id_salt("joint_scroll").auto_shrink([false, true]).show(ui, |ui| {
            self.solver_box(ui);
            let title = match (&jname, editing) {
                (Some(n), true) => format!("Edit {n}"),
                _ => format!("Create {} joint", kind.type_name()),
            };
            task_box(ui, "joint", joint_icon(&kind), &title, |ui| {
                ui.spacing_mut().item_spacing.y = 6.0;
                // joint type combo with icons
                let cur = JOINT_TOOLS.iter().find(|(n, _, _)| joint_kind(n).type_name() == kind.type_name()).map(|x| x.0).unwrap_or("Fixed");
                let r = egui::ComboBox::from_id_salt("joint_type").selected_text(cur).width(ui.available_width() - 14.0).show_ui(ui, |ui| {
                    for (name, icon, tip) in JOINT_TOOLS {
                        let r = ui.add(egui::Button::image_and_text(crate::icons::image(ui.ctx(), icon, 16.0), name).selected(name == cur).frame(false)).on_hover_text(tip);
                        widgets::reg(format!("opt:joint_type:{name}"), r.rect);
                        if r.clicked() && name != cur {
                            new_kind = joint_kind(name);
                        }
                    }
                });
                widgets::reg("combo:joint_type", r.response.rect);
                // the two references (FreeCAD's selection list)
                egui::Frame::new().fill(theme::INPUT).stroke(egui::Stroke::new(1.0, theme::BORDER)).corner_radius(3.0).inner_margin(4.0).show(ui, |ui| {
                    ui.set_width(ui.available_width() - 10.0);
                    ui.set_min_height(56.0);
                    for i in 0..2 {
                        ui.horizontal(|ui| {
                            ui.set_min_height(22.0);
                            match labels.get(i) {
                                Some(l) => {
                                    let icon = match refs[i] {
                                        JointRef::PartConnector { .. } | JointRef::AssemblyConnector(_) => "PartDesign_CoordinateSystem",
                                        _ => "PartDesign_Body",
                                    };
                                    ui.add(crate::icons::image(ui.ctx(), icon, 16.0));
                                    ui.label(RichText::new(l).color(theme::TEXT));
                                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                        let r = ui.add(egui::Button::image(crate::icons::image(ui.ctx(), "edit-delete", 14.0)).frame(false)).on_hover_text("Remove this reference");
                                        widgets::reg(format!("joint:remove:{i}"), r.rect);
                                        if r.clicked() {
                                            remove = Some(i);
                                        }
                                    });
                                }
                                None => {
                                    ui.label(RichText::new(if i == 0 { "1. Click a mate connector or a face, edge or vertex" } else { "2. Then one on another part" }).color(Color32::from_gray(190)));
                                }
                            }
                        });
                    }
                });
                match kind {
                    JointKind::Distance { distance, mode } => {
                        let mut d = distance;
                        if task_row(ui, "Distance", |ui| self.num_field(ui, "joint.distance", &mut d, "mm")) {
                            new_kind = JointKind::Distance { distance: d, mode };
                        }
                    }
                    JointKind::Angle { angle } => {
                        let mut d = angle;
                        if task_row(ui, "Angle", |ui| self.num_field(ui, "joint.angle", &mut d, "°")) {
                            new_kind = JointKind::Angle { angle: d };
                        }
                    }
                    _ => {}
                }
                if !matches!(kind, JointKind::Distance { .. } | JointKind::Angle { .. } | JointKind::Parallel | JointKind::Perpendicular) {
                    changed |= task_row(ui, "Offset", |ui| self.num_field(ui, "joint.offset", &mut offset, "mm"));
                    changed |= task_row(ui, "Rotation", |ui| self.num_field(ui, "joint.rotation", &mut rotation, "°"));
                }
                ui.horizontal(|ui| {
                    let r = ui.add(egui::Button::image_and_text(crate::icons::image(ui.ctx(), "view-rotate-left", 16.0), "Reverse")).on_hover_text("Flip the direction of the joint (FreeCAD's Reverse, Onshape's flip primary axis)");
                    widgets::reg("joint:reverse", r.rect);
                    reverse = r.clicked();
                });
                let angular = matches!(kind, JointKind::Revolute | JointKind::Cylindrical);
                let linear = matches!(kind, JointKind::Slider | JointKind::Cylindrical);
                if angular || linear {
                    ui.add_space(4.0);
                    ui.separator();
                    ui.label(RichText::new("Limits").font(theme::bold_font(13.0)).color(theme::TEXT));
                    let mut row = |app: &mut App, ui: &mut egui::Ui, label: &str, id: &str, v: &mut Option<f64>, unit: &str| {
                        ui.horizontal(|ui| {
                            let mut on = v.is_some();
                            let r = ui.checkbox(&mut on, label);
                            widgets::reg(format!("check:{label}"), r.rect);
                            if r.changed() {
                                *v = if on { Some(0.0) } else { None };
                                changed = true;
                            }
                            if let Some(x) = v.as_mut() {
                                if app.num_field(ui, id, x, unit) {
                                    changed = true;
                                }
                            }
                        });
                    };
                    if linear {
                        row(self, ui, "Min length", "joint.lmin", &mut limits.length_min, "mm");
                        row(self, ui, "Max length", "joint.lmax", &mut limits.length_max, "mm");
                    }
                    if angular {
                        row(self, ui, "Min angle", "joint.amin", &mut limits.angle_min, "°");
                        row(self, ui, "Max angle", "joint.amax", &mut limits.angle_max, "°");
                    }
                }
            });
        });
        let mut apply = changed || new_kind != kind || remove.is_some();
        if reverse {
            // flip relative to how the joint is solved now
            let aligned = jid.and_then(|j| self.asm_ref().and_then(|a| a.joint_frames(&self.ws, j).ok())).map(|(a, b)| fc_part::placement::dot(a.z_axis(), b.z_axis()) > 0.0).unwrap_or(true);
            if let Some(AsmTask::Joint(t)) = self.asm.task.as_mut() {
                t.sense = if aligned { Sense::Flipped } else { Sense::Aligned };
            }
            apply = true;
        }
        if let Some(AsmTask::Joint(t)) = self.asm.task.as_mut() {
            t.kind = new_kind;
            t.offset = offset;
            t.rotation = rotation;
            t.limits = limits;
            if let Some(i) = remove {
                t.refs.remove(i);
            }
        }
        if apply {
            self.apply_joint_task();
        }
        if ok {
            self.joint_task_ok();
        } else if cancel {
            self.asm_task_cancel();
        }
    }

    fn conn_panel(&mut self, ui: &mut egui::Ui) {
        let (cancel, ok) = self.ok_cancel(ui);
        let Some(AsmTask::Connector(t)) = self.asm.task.as_ref() else { return };
        let label = t.pick.as_ref().map(|(i, e, inf)| {
            let (k, _) = fc_part::TopoKind::parse(e).unwrap_or((fc_part::TopoKind::Face, 0));
            format!("{}.{e} ({})", self.inst_name(*i), mate_ui::infer_label(k, -1, *inf).to_lowercase())
        });
        let mut offset = t.offset;
        let (mut flip, mut turn) = (false, false);
        let mut changed = false;
        task_box(ui, "asm_conn", "PartDesign_CoordinateSystem", "Mate connector", |ui| {
            ui.spacing_mut().item_spacing.y = 6.0;
            task_row(ui, "Origin", |ui| match &label {
                Some(l) => {
                    ui.label(RichText::new(l).color(theme::TEXT));
                }
                None => {
                    ui.label(RichText::new("Click geometry in the 3D view").color(theme::TEXT_DIM).italics());
                }
            });
            changed |= task_row(ui, "Offset Z", |ui| self.num_field(ui, "conn.offset", &mut offset, "mm"));
            ui.horizontal(|ui| {
                let r = ui.button("Flip primary axis").on_hover_text("Reverse the connector's Z axis");
                widgets::reg("conn:flip", r.rect);
                flip = r.clicked();
                let r = ui.button("Reorient secondary axis").on_hover_text("Turn the X axis by 90° about Z");
                widgets::reg("conn:turn", r.rect);
                turn = r.clicked();
            });
        });
        if let Some(AsmTask::Connector(t)) = self.asm.task.as_mut() {
            t.offset = offset;
            if flip {
                t.flip = !t.flip;
            }
            if turn {
                t.turns += 1;
            }
        }
        let _ = changed;
        if ok {
            self.conn_task_ok();
        } else if cancel {
            self.asm_task_cancel();
        }
    }
}

// --------------------------------------------------------------------- tree

impl App {
    /// Assembly > Joints (grounded parts, joints) and the part instances, like FreeCAD's tree.
    pub(super) fn assembly_tree_rows(&mut self, ui: &mut egui::Ui, a: AssemblyId, depth: usize, interactive: bool) {
        use super::tree::{is_open, toggle_open, tree_row_pub, RowSpecPub};
        let Some(asm) = self.ws.assembly(a).cloned() else { return };
        let active = self.active_assembly() == Some(a);
        // rows of other assemblies get their assembly's name in the test-hook name
        let pre = if active { String::new() } else { format!("{}/", asm.name) };
        let problems = if active { self.asm.problem_joints() } else { vec![] };
        let key = format!("asm{a}");
        let open = is_open(ui, &key, true);
        let r = tree_row_pub(
            ui,
            RowSpecPub {
                reg: format!("tree:{}", asm.name),
                depth,
                expander: Some(open),
                eye: None,
                icon: "Geoassembly",
                badge: if !problems.is_empty() { Some("Warning") } else { None },
                label: if active { RichText::new(&asm.name).font(theme::bold_font(13.0)) } else { RichText::new(&asm.name) },
                selected: false,
                tip: format!("{}\nDouble-click to open the assembly", asm.name),
            },
        );
        if r.expand {
            toggle_open(ui, &key, true);
        }
        if r.double && interactive {
            self.switch_tab(Tab::Assembly(a));
        }
        if !open {
            return;
        }
        let jkey = format!("asmj{a}");
        let jopen = is_open(ui, &jkey, true);
        let r = tree_row_pub(ui, RowSpecPub { reg: format!("tree:{}:Joints", asm.name), depth: depth + 1, expander: Some(jopen), eye: None, icon: "Assembly_JointGroup", badge: None, label: RichText::new("Joints"), selected: false, tip: "The assembly's joints".into() });
        if r.expand || (r.clicked && interactive) {
            toggle_open(ui, &jkey, true);
        }
        let mut act: Option<(AsmSel, bool)> = None;
        let mut menu_act: Option<(&'static str, AsmSel)> = None;
        if jopen {
            for inst in asm.instances.iter().filter(|i| i.grounded) {
                let s = AsmSel::Inst(inst.id);
                let r = tree_row_pub(
                    ui,
                    RowSpecPub {
                        reg: format!("tree:{pre}Grounded:{}", inst.name),
                        depth: depth + 2,
                        expander: None,
                        eye: None,
                        icon: "Assembly_ToggleGrounded",
                        badge: None,
                        label: RichText::new(format!("GroundedJoint ({})", inst.name)),
                        selected: false,
                        tip: format!("{} is grounded: it does not move", inst.name),
                    },
                );
                if r.clicked && interactive && active {
                    act = Some((s, false));
                }
            }
            for j in &asm.joints {
                let s = AsmSel::Joint(j.id);
                let bad = problems.contains(&j.id);
                let redundant = self.asm.report.as_ref().is_some_and(|r| r.redundant_joints.contains(&j.id) && r.status == SolveStatus::Redundant);
                let mut label = RichText::new(&j.name);
                if bad && !redundant {
                    label = label.color(theme::ERROR);
                } else if redundant {
                    label = label.color(theme::WARNING);
                }
                if j.suppressed {
                    label = label.strikethrough();
                }
                let tip = format!("{} ({} joint)\n{} ↔ {}{}\nDouble-click to edit", j.name, j.kind.type_name(), self.ref_label(&j.a), self.ref_label(&j.b), if bad { if redundant { "\nRedundant: implied by other joints" } else { "\nConflicting or not satisfied" } } else { "" });
                let r = tree_row_pub(
                    ui,
                    RowSpecPub {
                        reg: format!("tree:{pre}{}", j.name),
                        depth: depth + 2,
                        expander: None,
                        eye: None,
                        icon: joint_icon(&j.kind),
                        badge: if bad { Some(if redundant { "Warning" } else { "overlay_error" }) } else { None },
                        label,
                        selected: active && self.asm.sel.contains(&s),
                        tip,
                    },
                );
                if interactive && active {
                    if r.double {
                        act = Some((s, true));
                    } else if r.clicked {
                        act = Some((s, false));
                    }
                    r.resp.context_menu(|ui| {
                        if ui.button("Edit joint").clicked() {
                            menu_act = Some(("edit", s));
                            ui.close();
                        }
                        if ui.button(if j.suppressed { "Unsuppress" } else { "Suppress" }).clicked() {
                            menu_act = Some(("suppress", s));
                            ui.close();
                        }
                        if ui.button("Delete").clicked() {
                            menu_act = Some(("delete", s));
                            ui.close();
                        }
                    });
                }
            }
        }
        for c in &asm.connectors {
            let s = AsmSel::Conn(c.id);
            let r = tree_row_pub(
                ui,
                RowSpecPub {
                    reg: format!("tree:{pre}{}", c.name),
                    depth: depth + 1,
                    expander: None,
                    eye: None,
                    icon: "PartDesign_CoordinateSystem",
                    badge: None,
                    label: RichText::new(&c.name),
                    selected: active && self.asm.sel.contains(&s),
                    tip: format!("Mate connector on {}{}", asm.instance(c.instance).map(|i| i.name.clone()).unwrap_or_default(), c.anchor.as_ref().map(|e| format!(".{e}")).unwrap_or_default()),
                },
            );
            if r.clicked && interactive && active {
                act = Some((s, false));
            }
        }
        for inst in &asm.instances {
            let s = AsmSel::Inst(inst.id);
            let label = if inst.visible { RichText::new(&inst.name) } else { RichText::new(&inst.name).color(Color32::from_gray(155)) };
            let r = tree_row_pub(
                ui,
                RowSpecPub {
                    reg: format!("tree:{pre}{}", inst.name),
                    depth: depth + 1,
                    expander: None,
                    eye: Some(inst.visible),
                    icon: if matches!(inst.of, InstanceOf::Assembly(_)) { "Geoassembly" } else { "PartDesign_Body" },
                    badge: Some("LinkOverlay"),
                    label,
                    selected: active && self.asm.sel.contains(&s),
                    tip: format!("{} (instance of {}){}\nDouble-click to edit the part", inst.name, match inst.of {
                        fc_assembly::InstanceOf::Part(p) => self.part_name(p),
                        fc_assembly::InstanceOf::Assembly(a) => format!("the assembly {}, rigid", self.ws.assembly(a).map(|x| x.name.clone()).unwrap_or_default()),
                    }, if inst.grounded { "\nGrounded" } else { "" }),
                },
            );
            if interactive && active {
                if r.eye {
                    menu_act = Some(("eye", s));
                } else if r.double {
                    act = Some((s, true));
                } else if r.clicked {
                    act = Some((s, false));
                }
                r.resp.context_menu(|ui| {
                    if ui.button(if inst.grounded { "Unground" } else { "Ground" }).clicked() {
                        menu_act = Some(("ground", s));
                        ui.close();
                    }
                    if ui.button("Delete").clicked() {
                        menu_act = Some(("delete", s));
                        ui.close();
                    }
                });
            }
        }
        self.exploded_tree_rows(ui, a, depth + 1, interactive);
        if let Some((s, double)) = act {
            let multi = ui.input(|i| i.modifiers.command || i.modifiers.ctrl);
            if double {
                match s {
                    AsmSel::Joint(j) => self.edit_joint(j),
                    AsmSel::Inst(i) => self.open_instance(i),
                    _ => {}
                }
            } else {
                if multi {
                    if let Some(i) = self.asm.sel.iter().position(|x| *x == s) {
                        self.asm.sel.remove(i);
                    } else {
                        self.asm.sel.push(s);
                    }
                } else {
                    self.asm.sel = vec![s];
                }
                self.rebuild_scene();
            }
        }
        if let Some((what, s)) = menu_act {
            match (what, s) {
                ("edit", AsmSel::Joint(j)) => self.edit_joint(j),
                ("suppress", AsmSel::Joint(j)) => self.asm_change(|asm| {
                    if let Some(x) = asm.joint_mut(j) {
                        x.suppressed = !x.suppressed;
                    }
                }),
                ("eye", AsmSel::Inst(i)) => self.asm_change(|asm| {
                    if let Some(x) = asm.instance_mut(i) {
                        x.visible = !x.visible;
                    }
                }),
                ("ground", AsmSel::Inst(_)) => {
                    self.asm.sel = vec![s];
                    self.toggle_grounded();
                }
                ("delete", _) => {
                    self.asm.sel = vec![s];
                    self.asm_delete_selected();
                }
                _ => {}
            }
        }
    }

    /// Properties of the selected assembly item in the Data tab.
    pub(super) fn asm_data_tab(&mut self, ui: &mut egui::Ui) {
        let mut g = widgets::PropGrid::new(ui);
        let Some(asm) = self.asm_ref().cloned() else { return };
        match self.asm.sel.last().copied() {
            Some(AsmSel::Inst(i)) => {
                let Some(inst) = asm.instance(i) else { return };
                g.group(ui, "Base");
                g.row(ui, "Label", "", |ui| widgets::ro_value(ui, &inst.name));
                let of = match inst.of {
                    fc_assembly::InstanceOf::Part(p) => self.part_name(p),
                    fc_assembly::InstanceOf::Assembly(a) => format!("{} (sub-assembly)", self.ws.assembly(a).map(|x| x.name.clone()).unwrap_or_default()),
                };
                g.row(ui, "Linked part", "", |ui| widgets::ro_value(ui, &of));
                let mut gr = inst.grounded;
                if g.row(ui, "Grounded", "A grounded part does not move", |ui| widgets::yes_no(ui, &mut gr)) {
                    self.asm_change(|a| a.set_grounded(i, gr));
                }
                let p = inst.placement;
                g.group(ui, "Placement");
                g.row(ui, "Position", "", |ui| widgets::ro_value(ui, &format!("({}, {}, {}) mm", fmt_num(p.pos[0]), fmt_num(p.pos[1]), fmt_num(p.pos[2]))));
                let rv = p.rotation_vector();
                let ang = fc_part::placement::norm(rv);
                let ax = if ang > 1e-9 { fc_part::placement::scale(rv, 1.0 / ang) } else { [0.0, 0.0, 1.0] };
                g.row(ui, "Axis", "", |ui| widgets::ro_value(ui, &format!("({}, {}, {})", fmt_num(ax[0]), fmt_num(ax[1]), fmt_num(ax[2]))));
                g.row(ui, "Angle", "", |ui| widgets::ro_value(ui, &format!("{} °", fmt_num(ang.to_degrees()))));
            }
            Some(AsmSel::Joint(j)) => {
                let Some(jt) = asm.joint(j) else { return };
                g.group(ui, "Base");
                g.row(ui, "Label", "", |ui| widgets::ro_value(ui, &jt.name));
                g.row(ui, "Suppressed", "", |ui| widgets::ro_value(ui, if jt.suppressed { "Yes" } else { "No" }));
                g.group(ui, "Joint");
                g.row(ui, "Joint Type", "", |ui| widgets::ro_value(ui, jt.kind.type_name()));
                g.row(ui, "Reference1", "", |ui| widgets::ro_value(ui, &self.ref_label(&jt.a)));
                g.row(ui, "Reference2", "", |ui| widgets::ro_value(ui, &self.ref_label(&jt.b)));
                g.row(ui, "Offset", "", |ui| widgets::ro_value(ui, &format!("{} mm", fmt_num(jt.offset_b.pos[2]))));
                g.row(ui, "Rotation", "", |ui| widgets::ro_value(ui, &format!("{} °", fmt_num(jt.offset_b.rotation_vector()[2].to_degrees()))));
                g.row(ui, "Reversed", "", |ui| widgets::ro_value(ui, &format!("{:?}", jt.sense)));
            }
            Some(AsmSel::Conn(c)) => {
                let Some(x) = asm.connectors.iter().find(|x| x.id == c) else { return };
                g.group(ui, "Mate connector");
                g.row(ui, "Label", "", |ui| widgets::ro_value(ui, &x.name));
                g.row(ui, "Part", "", |ui| widgets::ro_value(ui, &asm.instance(x.instance).map(|i| i.name.clone()).unwrap_or_default()));
                g.row(ui, "Anchor", "", |ui| widgets::ro_value(ui, x.anchor.as_deref().unwrap_or("-")));
            }
            None => {
                g.group(ui, "Assembly");
                g.row(ui, "Label", "", |ui| widgets::ro_value(ui, &asm.name));
                g.row(ui, "Parts", "", |ui| widgets::ro_value(ui, &asm.instances.len().to_string()));
                g.row(ui, "Joints", "", |ui| widgets::ro_value(ui, &asm.joints.len().to_string()));
                let (m, _) = self.solver_message();
                g.row(ui, "Solver", "", |ui| widgets::ro_value(ui, &m));
            }
        }
    }
}
