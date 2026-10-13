//! Application shell: Part Design document, file I/O, undo, test hooks and the window
//! layout (FreeCAD 1.1 look: menu + toolbars, Model dock with the tree and property view on
//! the left, 3D view with navigation cube, Tasks panel over the right side of the view).
//!
//! - `chrome.rs`: menu bar, toolbars, status bar, keyboard shortcuts
//! - `tree.rs`: Model dock (tree, property grid, variables, measurements)
//! - `tasks.rs`: Tasks panel (feature parameters, sketch attachment, sketch edit)

mod asm_tools;
mod assembly;
mod chrome;
mod fem;
mod fem_asm_ui;
mod fem_ui;
mod mates;
mod tasks;
mod tree;
pub mod workspace;

use workspace::Tab;

use crate::scene;
use crate::sketcher::{EditorExit, SketchEditor};
use crate::viewport::{Pick, PickKind, SceneEdge, SceneMesh, SceneObject, StandardView, Viewport};
use crate::widgets;
use eframe::egui;
use fc_part::params::OnUsers;
use fc_part::*;
use std::collections::{HashMap, HashSet};
use std::path::PathBuf;

/// Something waiting on the user in a modal window.
enum Dialog {
    ConfirmDelete { what: DeleteWhat, users: Vec<String> },
    Message(String),
    About,
    /// Assembly STEP / STL export options.
    AsmExport(asm_tools::AsmExportOpts),
}

#[derive(Clone)]
enum DeleteWhat {
    Feature(FeatureId),
    Param(String),
}

/// What the "Add measurement" control will create.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum MeasKind {
    Volume,
    Area,
    BBoxX,
    BBoxY,
    BBoxZ,
    Value,
    FaceArea,
    FaceDiameter,
    FaceDistance,
}

impl MeasKind {
    const ALL: [MeasKind; 9] = [MeasKind::Volume, MeasKind::Area, MeasKind::BBoxX, MeasKind::BBoxY, MeasKind::BBoxZ, MeasKind::Value, MeasKind::FaceArea, MeasKind::FaceDiameter, MeasKind::FaceDistance];
    fn label(self) -> &'static str {
        match self {
            MeasKind::Volume => "Volume",
            MeasKind::Area => "Surface area",
            MeasKind::BBoxX => "Bounding box X",
            MeasKind::BBoxY => "Bounding box Y",
            MeasKind::BBoxZ => "Bounding box Z",
            MeasKind::Value => "Value of...",
            MeasKind::FaceArea => "Face area (pick 1 face)",
            MeasKind::FaceDiameter => "Face diameter (pick 1 round face)",
            MeasKind::FaceDistance => "Distance (pick 2 faces)",
        }
    }
    fn faces_needed(self) -> usize {
        match self {
            MeasKind::FaceArea | MeasKind::FaceDiameter => 1,
            MeasKind::FaceDistance => 2,
            _ => 0,
        }
    }
}

fn dim_unit(d: fc_part::measure::Dim) -> &'static str {
    use fc_part::measure::Dim;
    match d {
        Dim::Length => "mm",
        Dim::Area => "mm²",
        Dim::Volume => "mm³",
        Dim::Angle => "°",
        Dim::Number => "",
    }
}

/// Modification time and size of the open file: changes when something else rewrites it.
type FileSig = (std::time::SystemTime, u64);

fn file_sig(p: &std::path::Path) -> Option<FileSig> {
    let m = std::fs::metadata(p).ok()?;
    Some((m.modified().ok()?, m.len()))
}

/// A feature being created or edited in the Tasks panel (FreeCAD's TaskDlgFeatureParameters).
struct FeatureTask {
    id: FeatureId,
    /// Created by this task: Cancel deletes it again.
    created: bool,
    /// Document before the task (also the undo snapshot it pushed).
    before: String,
}

/// Origin elements shown under Body > Origin: X/Y/Z axes, then the XY/XZ/YZ planes.
pub const ORIGIN_ITEMS: [(&str, &str); 6] = [("X_Axis", "Std_Axis"), ("Y_Axis", "Std_Axis"), ("Z_Axis", "Std_Axis"), ("XY_Plane", "Std_Plane"), ("XZ_Plane", "Std_Plane"), ("YZ_Plane", "Std_Plane")];

pub struct App {
    /// The open part's document (see `workspace.rs`); empty while an assembly is open.
    doc: Document,
    /// All parts and assemblies of the document.
    ws: fc_assembly::Workspace,
    /// What the 3D view shows.
    tab: Tab,
    /// `doc` currently lives in `ws` (always true while an assembly is open).
    doc_in_ws: bool,
    /// The part last open in Part Design (the workbench switcher returns to it).
    last_part: Option<fc_assembly::PartId>,
    asm: assembly::AsmState,
    /// FEM workbench: analyses of the document, tasks, runs and result display.
    fem: fem::FemState,
    /// Mate connector task: the hovered element and its snapped inference point.
    mate_hover: Option<mates::MateHover>,
    meas_kind: MeasKind,
    meas_value_key: String,
    /// Signature of the open file as we last read or wrote it.
    disk_sig: Option<FileSig>,
    last_poll: std::time::Instant,
    /// The open file changed on disk while we have unsaved edits: ask what to do.
    disk_changed: bool,
    path: Option<PathBuf>,
    dirty: bool,
    vp: Viewport,
    selection: Vec<Pick>,
    hovered: Option<Pick>,
    /// Features selected in the tree (ctrl/cmd-click adds).
    tree_sel: Vec<FeatureId>,
    /// Origin element selected in the tree (index into `ORIGIN_ITEMS`).
    origin_sel: Option<usize>,
    editor: Option<SketchEditor>,
    task: Option<FeatureTask>,
    undo: Vec<String>,
    redo: Vec<String>,
    status: String,
    dialog: Option<Dialog>,
    /// Text buffers of numeric fields being edited, keyed by canonical target.
    fields: HashMap<String, String>,
    new_param: (String, String),
    /// Test hook: (path, frames rendered)
    screenshot: Option<(PathBuf, u32)>,
    first_frame: bool,
    /// Test hook: scripted raw input (`FCRS_INPUT`).
    input: Option<crate::input_script::Runner>,
    /// Choosing the plane/face for a new sketch ("Select attachment").
    picking: bool,
    /// Plane highlighted in the attachment list (index into `plane_picker::planes()`).
    plane_choice: Option<usize>,
    /// Index of the first origin plane object in the view, when planes are shown.
    planes_at: Option<usize>,
    show_origin: bool,
    body_visible: bool,
    hidden_sketches: HashSet<FeatureId>,
    /// Property view tab: 0 Data, 1 Variables, 2 Measurements.
    props_tab: usize,
    /// FreeCAD's two-key shortcuts ("V, F"): first key and when it was pressed.
    chord: Option<(egui::Key, f64)>,
    /// The 3D view's rect last frame (Tasks panel placement).
    view_rect: egui::Rect,
}

fn pick_name(vp: &Viewport, p: &Pick, planes_at: Option<usize>) -> String {
    let obj = vp.objects().get(p.object).map(|o| o.name.as_str()).unwrap_or("?");
    if planes_at.is_some_and(|f| p.object >= f) {
        return obj.to_string();
    }
    let kind = match p.kind {
        PickKind::Face => "Face",
        PickKind::Edge => "Edge",
        PickKind::Vertex => "Vertex",
    };
    format!("{obj}.{kind}{}", p.id + 1)
}

fn parse_pick(s: &str) -> Option<Pick> {
    let (k, id) = s.split_once(':')?;
    let kind = match k {
        "face" => PickKind::Face,
        "edge" => PickKind::Edge,
        "vertex" => PickKind::Vertex,
        _ => return None,
    };
    Some(Pick { object: 0, kind, id: id.parse().ok()?, point: [0.0; 3] })
}

/// FreeCAD icon of a feature type.
pub fn feature_icon(k: &FeatureKind) -> &'static str {
    match k {
        FeatureKind::Sketch(_) => "Sketcher_Sketch",
        FeatureKind::Pad(_) => "PartDesign_Pad",
        FeatureKind::Pocket(_) => "PartDesign_Pocket",
        FeatureKind::Revolution(_) => "PartDesign_Revolution",
        FeatureKind::Groove(_) => "PartDesign_Groove",
        FeatureKind::AdditiveLoft(_) => "PartDesign_AdditiveLoft",
        FeatureKind::SubtractiveLoft(_) => "PartDesign_SubtractiveLoft",
        FeatureKind::AdditivePipe(_) => "PartDesign_AdditivePipe",
        FeatureKind::SubtractivePipe(_) => "PartDesign_SubtractivePipe",
        FeatureKind::Hole(_) => "PartDesign_Hole",
        FeatureKind::MateConnector(_) => "PartDesign_CoordinateSystem",
    }
}

/// User-facing feature type name ("Additive loft").
pub fn feature_title(k: &FeatureKind) -> &'static str {
    match k {
        FeatureKind::Sketch(_) => "Sketch",
        FeatureKind::Pad(_) => "Pad",
        FeatureKind::Pocket(_) => "Pocket",
        FeatureKind::Revolution(_) => "Revolution",
        FeatureKind::Groove(_) => "Groove",
        FeatureKind::AdditiveLoft(_) => "Additive Loft",
        FeatureKind::SubtractiveLoft(_) => "Subtractive Loft",
        FeatureKind::AdditivePipe(_) => "Additive Pipe",
        FeatureKind::SubtractivePipe(_) => "Subtractive Pipe",
        FeatureKind::Hole(_) => "Hole",
        FeatureKind::MateConnector(_) => "Mate Connector",
    }
}

impl App {
    pub fn new(cc: &eframe::CreationContext<'_>, screenshot: Option<PathBuf>) -> Self {
        crate::theme::apply(&cc.egui_ctx);
        let mut vp = Viewport::new(cc);
        vp.instant = screenshot.is_some() || std::env::var_os("FCRS_INPUT").is_some();
        let (ws, first_part) = workspace::fresh_workspace();
        let mut app = App {
            doc: Document::new(),
            ws,
            tab: Tab::Part(first_part),
            doc_in_ws: false,
            last_part: None,
            asm: assembly::AsmState::default(),
            fem: fem::FemState::default(),
            mate_hover: None,
            meas_kind: MeasKind::Volume,
            meas_value_key: String::new(),
            disk_sig: None,
            last_poll: std::time::Instant::now(),
            disk_changed: false,
            path: None,
            dirty: false,
            vp,
            selection: vec![],
            hovered: None,
            tree_sel: vec![],
            origin_sel: None,
            editor: None,
            task: None,
            undo: vec![],
            redo: vec![],
            status: "Ready".into(),
            dialog: None,
            fields: HashMap::new(),
            new_param: (String::new(), String::new()),
            screenshot: screenshot.map(|p| (p, 0)),
            first_frame: true,
            input: None,
            picking: false,
            plane_choice: None,
            planes_at: None,
            show_origin: false,
            body_visible: true,
            hidden_sketches: HashSet::new(),
            props_tab: 0,
            chord: None,
            view_rect: egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(900.0, 600.0)),
        };
        if let Ok(p) = std::env::var("FCRS_INPUT") {
            match std::fs::read_to_string(&p).map_err(|e| e.to_string()).and_then(|s| crate::input_script::parse(&s)) {
                Ok(steps) => app.input = Some(crate::input_script::Runner::new(steps)),
                Err(e) => eprintln!("[input] {e}"),
            }
        }
        if let Ok(p) = std::env::var("FCRS_OPEN") {
            app.open_path(PathBuf::from(p));
        } else if std::env::var_os("FCRS_DEMO").is_some() {
            app.doc = crate::demo::demo_document();
        }
        if std::env::var_os("FCRS_ASM_DEMO").is_some() {
            app.load_workspace(crate::demo::assembly_workspace(true));
        }
        app.doc.recompute();
        app.rebuild_scene();
        let view = match std::env::var("FCRS_VIEW").as_deref() {
            Ok("front") => StandardView::Front,
            Ok("top") => StandardView::Top,
            Ok("right") => StandardView::Right,
            Ok("rear") => StandardView::Rear,
            Ok("bottom") => StandardView::Bottom,
            Ok("left") => StandardView::Left,
            _ => StandardView::Isometric,
        };
        app.vp.set_standard_view(view);
        if std::env::var_os("FCRS_PERSP").is_some() {
            app.vp.set_perspective(true);
            app.vp.fit_all();
        }
        if let Ok(s) = std::env::var("FCRS_SELECT") {
            app.selection = s.split(',').filter_map(parse_pick).collect();
            app.vp.set_selection(app.selection.clone());
        }
        if let Some(h) = std::env::var("FCRS_HOVER").ok().and_then(|s| parse_pick(&s)) {
            app.vp.force_hover(Some(h));
        }
        if let Ok(names) = std::env::var("FCRS_TREESEL") {
            app.tree_sel = names.split(',').filter_map(|n| app.doc.by_name(n).map(|f| f.id)).collect();
            app.rebuild_scene();
        }
        if let Some(id) = std::env::var("FCRS_EDIT").ok().and_then(|n| app.doc.by_name(&n).map(|f| f.id)) {
            app.edit_feature(id);
        }
        if std::env::var_os("FCRS_NEWSKETCH").is_some() {
            app.start_plane_pick();
        }
        if let Ok(t) = std::env::var("FCRS_TAB") {
            app.props_tab = t.parse().unwrap_or(0);
        }
        app
    }

    // ------------------------------------------------------------------ model ops

    fn rebuild_scene(&mut self) {
        if self.active_assembly().is_some() {
            if self.fem_on_assembly() && self.fem_cur().is_some() {
                self.rebuild_fem_asm_scene();
            } else {
                self.rebuild_asm_scene();
            }
            return;
        }
        let mut objs = if !self.body_visible {
            vec![]
        } else if let Some(ed) = &self.editor {
            // Like FreeCAD, features that depend on the edited sketch are hidden: show the
            // solid as it was before the sketch (the face it sits on, if any).
            let at = self.doc.index_of(ed.feature()).unwrap_or(0);
            (0..at)
                .rev()
                .find(|&i| self.doc.features[i].kind.is_solid_feature() && !self.doc.features[i].suppressed && self.doc.states.get(i).is_some_and(|s| s.error.is_none() && s.shape.is_some()))
                .and_then(|i| self.doc.states[i].shape.as_ref())
                .and_then(|s| scene::shape_object("Body", s, scene::BODY_COLOR))
                .into_iter()
                .collect()
        } else if self.doc.features.is_empty() {
            // a part imported from STEP: its solid, no features
            self.active_part().and_then(|p| self.ws.part(p)).and_then(|e| e.imported.as_ref()).and_then(|s| scene::shape_object("Body", s, scene::BODY_COLOR)).into_iter().collect()
        } else {
            scene::body_objects(&self.doc)
        };
        if self.picking || self.show_origin {
            self.planes_at = Some(objs.len());
            let half = crate::plane_picker::plane_half_size(self.doc.result().map(|s| s.bbox()));
            objs.extend(crate::plane_picker::plane_objects(half));
        } else {
            self.planes_at = None;
        }
        // FEM: the mesh or the result replaces the body
        self.fem_extend_scene(&mut objs);
        self.vp.pick_limit = self.fem.fem_at.unwrap_or(usize::MAX);
        self.vp.set_objects(objs);
        if self.editor.is_none() {
            let hl = self.tree_sel.iter().copied().find(|id| matches!(self.doc.get(*id).map(|f| &f.kind), Some(FeatureKind::Sketch(_))));
            self.vp.set_overlay(scene::sketch_overlay(&self.doc, hl, &self.hidden_sketches));
        }
        self.sync_origin_highlight();
    }

    /// Highlight the origin plane selected in the tree (or chosen in the attachment list).
    fn sync_origin_highlight(&mut self) {
        let Some(at) = self.planes_at else { return };
        let plane = if self.picking { self.plane_choice } else { self.origin_sel.filter(|i| *i >= 3).map(|i| i - 3) };
        let mut sel: Vec<Pick> = self.selection.iter().filter(|p| p.object < at).cloned().collect();
        if let Some(i) = plane {
            sel.push(Pick { object: at + i, kind: PickKind::Face, id: 0, point: [0.0; 3] });
        }
        self.vp.set_selection(sel);
    }

    /// Snapshot for undo (unless a task dialog owns the undo step), apply a change, recompute.
    fn change(&mut self, f: impl FnOnce(&mut Document)) {
        self.begin_step();
        f(&mut self.doc);
        self.after_change();
    }

    /// Start an undoable step; inside a task the task's own snapshot covers everything.
    fn begin_step(&mut self) {
        if self.task.is_none() {
            self.snapshot();
        }
    }

    fn snapshot(&mut self) {
        let s = self.state_json();
        self.undo.push(s);
        if self.undo.len() > 200 {
            self.undo.remove(0);
        }
        self.redo.clear();
    }

    fn after_change(&mut self) {
        self.doc.recompute();
        self.dirty = true;
        self.fields.clear();
        self.rebuild_scene();
        self.report_errors();
    }

    fn report_errors(&mut self) {
        for (f, st) in self.doc.features.iter().zip(&self.doc.states) {
            if let Some(e) = &st.error {
                self.status = format!("{}: {e}", f.label);
                return;
            }
        }
        if let Some((k, e)) = self.doc.expression_errors.iter().next() {
            self.status = format!("Expression {}: {e}", self.doc.display_name(k));
        }
    }

    fn can_undo(&self) -> bool {
        match &self.editor {
            Some(ed) => ed.can_undo(),
            None => self.task.is_none() && self.asm.task.is_none() && self.fem.task.is_none() && !self.undo.is_empty(),
        }
    }

    fn can_redo(&self) -> bool {
        match &self.editor {
            Some(ed) => ed.can_redo(),
            None => self.task.is_none() && self.asm.task.is_none() && self.fem.task.is_none() && !self.redo.is_empty(),
        }
    }

    fn undo(&mut self) {
        if let Some(ed) = self.editor.as_mut() {
            ed.undo(&mut self.doc);
            return;
        }
        if self.task.is_some() || self.asm.task.is_some() || self.fem.task.is_some() {
            return;
        }
        if let Some(prev) = self.undo.pop() {
            let cur = self.state_json();
            self.redo.push(cur);
            self.restore(&prev);
            self.status = "Undo".into();
        }
    }

    fn redo(&mut self) {
        if let Some(ed) = self.editor.as_mut() {
            ed.redo(&mut self.doc);
            return;
        }
        if self.task.is_some() || self.asm.task.is_some() || self.fem.task.is_some() {
            return;
        }
        if let Some(next) = self.redo.pop() {
            let cur = self.state_json();
            self.undo.push(cur);
            self.restore(&next);
            self.status = "Redo".into();
        }
    }

    /// No dialog, sketch edit or task running: Part Design commands are available.
    fn idle(&self) -> bool {
        self.editor.is_none() && self.task.is_none() && !self.picking && self.asm.task.is_none() && self.asm.drag.is_none() && self.fem.task.is_none()
    }

    /// The feature whose output is the current body solid (for sketch-on-face references).
    fn tip_solid_feature(&self) -> Option<FeatureId> {
        let tip = self.doc.tip.and_then(|t| self.doc.index_of(t)).unwrap_or(self.doc.features.len().saturating_sub(1));
        (0..=tip.min(self.doc.features.len().saturating_sub(1)))
            .rev()
            .find(|&i| {
                self.doc.features.get(i).map(|f| f.kind.is_solid_feature()).unwrap_or(false)
                    && self.doc.states.get(i).map(|s| s.error.is_none() && s.shape.is_some()).unwrap_or(false)
            })
            .map(|i| self.doc.features[i].id)
    }

    /// The sketch a new feature should use: the selected one, else the newest unused sketch.
    fn profile_sketch(&self) -> Option<FeatureId> {
        let is_sketch = |id: &FeatureId| matches!(self.doc.get(*id).map(|f| &f.kind), Some(FeatureKind::Sketch(_)));
        if let Some(id) = self.tree_sel.iter().rev().copied().find(is_sketch) {
            return Some(id);
        }
        self.doc
            .features
            .iter()
            .rev()
            .filter(|f| matches!(f.kind, FeatureKind::Sketch(_)))
            .find(|f| self.doc.dependents(f.id).iter().all(|d| !self.doc.get(*d).map(|g| g.kind.is_solid_feature()).unwrap_or(false)))
            .map(|f| f.id)
    }

    fn selected_sketches(&self) -> Vec<FeatureId> {
        self.tree_sel.iter().copied().filter(|id| matches!(self.doc.get(*id).map(|f| &f.kind), Some(FeatureKind::Sketch(_)))).collect()
    }

    fn new_sketch(&mut self) {
        // An origin plane selected in the tree or the view: sketch on it directly (FreeCAD).
        if let Some(i) = self.origin_sel.filter(|i| *i >= 3) {
            if let Some(pl) = crate::plane_picker::planes().into_iter().nth(i - 3) {
                self.create_sketch(pl.support);
                return;
            }
        }
        // A selected planar face of the body becomes the support (sketch on face).
        let face = self.selection.iter().find(|p| p.kind == PickKind::Face && p.object == 0 && self.body_visible).map(|p| p.id as usize + 1);
        if let (Some(face), Some(fid)) = (face, self.tip_solid_feature()) {
            let shape = self.doc.state(fid).and_then(|s| s.shape.clone());
            if let Some(shape) = shape {
                match shape.face_info(face) {
                    Ok(fi) if fi.kind == 0 => {
                        if let Ok(fr) = face_ref(fid, &shape, face) {
                            self.create_sketch(Support::Face(fr));
                            return;
                        }
                    }
                    _ => {
                        self.dialog = Some(Dialog::Message("The selected face is not planar. Select a flat face or pick a base plane.".into()));
                        return;
                    }
                }
            }
        }
        self.start_plane_pick();
    }

    /// FreeCAD-style "Select attachment": show the origin planes and let the user click one,
    /// or a flat face of the part.
    fn start_plane_pick(&mut self) {
        self.picking = true;
        self.plane_choice = None;
        self.selection.clear();
        self.vp.set_selection(vec![]);
        self.rebuild_scene();
        if self.doc.result().is_none() {
            self.vp.right_inset = tasks::TASK_W;
            self.vp.set_standard_view(StandardView::Isometric);
        }
        self.status = "Select a plane or a flat face in the 3D view, or pick a base plane from the list".into();
    }

    fn end_plane_pick(&mut self) {
        self.picking = false;
        self.plane_choice = None;
        self.rebuild_scene();
    }

    /// Handle a click while picking: true if it chose a support.
    fn pick_support(&mut self, p: &Pick) -> bool {
        let first = self.planes_at.unwrap_or(usize::MAX);
        if p.object >= first {
            let i = p.object - first;
            if let Some(pl) = crate::plane_picker::planes().into_iter().nth(i) {
                self.end_plane_pick();
                self.create_sketch(pl.support);
                return true;
            }
        } else if p.kind == PickKind::Face {
            if let Some(fid) = self.tip_solid_feature() {
                if let Some(shape) = self.doc.state(fid).and_then(|s| s.shape.clone()) {
                    let face = p.id as usize + 1;
                    match shape.face_info(face) {
                        Ok(fi) if fi.kind == 0 => {
                            if let Ok(fr) = face_ref(fid, &shape, face) {
                                self.end_plane_pick();
                                self.create_sketch(Support::Face(fr));
                                return true;
                            }
                        }
                        _ => self.status = "That face is curved; pick a flat face or a plane".into(),
                    }
                }
            }
        }
        false
    }

    /// Plane names next to their corners, like FreeCAD's origin labels.
    fn draw_plane_labels(&self, ui: &egui::Ui, ctx: &egui::Context) {
        let half = crate::plane_picker::plane_half_size(self.doc.result().map(|s| s.bbox()));
        let painter = ctx.layer_painter(egui::LayerId::new(egui::Order::Background, egui::Id::new("plane_labels"))).with_clip_rect(ui.clip_rect());
        for (i, p) in crate::plane_picker::planes().into_iter().enumerate() {
            if let Some(pos) = self.vp.world_to_screen(p.placement.to_world(half, half)) {
                let c = crate::plane_picker::plane_color(i);
                let col = egui::Color32::from_rgb((110.0 + c[0] * 145.0) as u8, (110.0 + c[1] * 145.0) as u8, (110.0 + c[2] * 145.0) as u8);
                // leader line from the plane's corner to its label
                let end = pos + egui::vec2(18.0, -16.0);
                painter.line_segment([pos, end], egui::Stroke::new(1.0, col));
                painter.circle_filled(pos, 2.5, col);
                let g = painter.layout_no_wrap(p.name.to_string(), egui::FontId::proportional(13.0), col);
                let r = egui::Rect::from_min_size(end + egui::vec2(2.0, -g.size().y * 0.5 - 2.0), g.size() + egui::vec2(8.0, 4.0));
                painter.rect_filled(r, 3.0, egui::Color32::from_rgba_unmultiplied(20, 20, 20, 200));
                painter.galley(r.min + egui::vec2(4.0, 2.0), g, col);
            }
        }
    }

    fn create_sketch(&mut self, support: Support) {
        self.snapshot();
        let id = self.doc.add_sketch(support, fc_sketch::Sketch::default());
        self.after_change();
        self.tree_sel = vec![id];
        self.origin_sel = None;
        self.selection.clear();
        self.vp.set_selection(vec![]);
        self.start_edit(id);
    }

    /// Double-click / "Edit": sketches open the sketcher, features their task panel.
    fn edit_feature(&mut self, id: FeatureId) {
        match self.doc.get(id).map(|f| matches!(f.kind, FeatureKind::Sketch(_))) {
            Some(false) => self.open_task(id, false),
            Some(true) => self.start_edit(id),
            None => {}
        }
    }

    fn start_edit(&mut self, id: FeatureId) {
        if self.editor.is_some() || self.task.is_some() {
            return;
        }
        if let Some(ed) = SketchEditor::new(&self.doc, id) {
            self.snapshot();
            self.vp.align_to_plane(&scene::plane(&ed.placement()));
            let mut ed = ed;
            let has_geometry = matches!(self.doc.get(id).map(|f| &f.kind), Some(FeatureKind::Sketch(s)) if !s.sketch.geometry.is_empty());
            if has_geometry {
                ed.fit();
            }
            let base_plane = matches!(self.doc.get(id).map(|f| &f.kind), Some(FeatureKind::Sketch(s)) if s.sketch.geometry.is_empty() && !matches!(s.support, Support::Face(_)));
            if base_plane {
                // a new sketch on a base plane: origin in the middle of the free part of the view
                let half = crate::plane_picker::plane_half_size(self.doc.result().map(|s| s.bbox()));
                let pl = ed.placement();
                let (a, b) = (pl.to_world(-half, -half), pl.to_world(half, half));
                self.vp.right_inset = tasks::TASK_W;
                self.vp.fit_box([a[0].min(b[0]), a[1].min(b[1]), a[2].min(b[2])], [a[0].max(b[0]), a[1].max(b[1]), a[2].max(b[2])]);
            }
            self.status = format!("Editing {}", self.doc.get(id).map(|f| f.label.clone()).unwrap_or_default());
            self.editor = Some(ed);
            self.tree_sel = vec![id];
            self.rebuild_scene();
        } else {
            self.status = "Cannot edit this sketch (its support failed to compute)".into();
        }
    }

    fn finish_edit(&mut self, exit: EditorExit) {
        let Some(ed) = self.editor.take() else { return };
        if exit == EditorExit::Cancel {
            if let Some(sk) = self.doc.sketch_mut(ed.feature()) {
                *sk = ed.original().clone();
            }
            // Drop the snapshot taken when editing started.
            self.undo.pop();
            self.doc.recompute();
            self.rebuild_scene();
            self.status = "Edit cancelled".into();
        } else {
            self.after_change();
            if self.status.starts_with("Editing") {
                self.status = "Sketch updated".into();
            }
        }
        self.vp.set_standard_view(StandardView::Isometric);
    }

    fn add_feature(&mut self, make: impl FnOnce(FeatureId) -> FeatureKind) {
        let Some(sk) = self.profile_sketch() else {
            self.dialog = Some(Dialog::Message("Select a sketch first (or create one).".into()));
            return;
        };
        self.snapshot();
        let id = self.doc.add(make(sk));
        self.after_change();
        self.tree_sel = vec![id];
        self.rebuild_scene();
        self.begin_task(id, true);
    }

    fn add_loft_or_pipe(&mut self, subtractive: bool, pipe: bool) {
        let sel = self.selected_sketches();
        if sel.len() < 2 {
            let what = if pipe { "a profile sketch and then a path sketch" } else { "two or more section sketches, in order" };
            self.dialog = Some(Dialog::Message(format!("Ctrl/Cmd-click {what} in the model tree first.")));
            return;
        }
        let kind = if pipe {
            let p = Pipe { profile: sel[0], spine: sel[1], frenet: true, transition: Transition::Transformed };
            if subtractive { FeatureKind::SubtractivePipe(p) } else { FeatureKind::AdditivePipe(p) }
        } else {
            let l = Loft { profile: sel[0], sections: sel[1..].to_vec(), ruled: false };
            if subtractive { FeatureKind::SubtractiveLoft(l) } else { FeatureKind::AdditiveLoft(l) }
        };
        self.snapshot();
        let id = self.doc.add(kind);
        self.after_change();
        self.tree_sel = vec![id];
        self.begin_task(id, true);
    }

    /// Open the task panel for feature `id`; `created` = the caller just added it (and its
    /// undo snapshot is on top of the stack).
    fn begin_task(&mut self, id: FeatureId, created: bool) {
        if !created {
            self.snapshot();
        }
        let before = self.undo.last().cloned().unwrap_or_default();
        self.task = Some(FeatureTask { id, created, before });
        self.tree_sel = vec![id];
        self.fields.clear();
        self.status = format!("Editing {}", self.doc.get(id).map(|f| f.label.clone()).unwrap_or_default());
    }

    fn open_task(&mut self, id: FeatureId, created: bool) {
        if self.editor.is_some() || self.task.is_some() || self.picking {
            return;
        }
        self.begin_task(id, created);
    }

    /// OK: keep the result as one undo step.
    fn task_ok(&mut self) {
        self.commit_focused_field();
        let Some(t) = self.task.take() else { return };
        if !t.created && self.state_json() == t.before {
            self.undo.pop();
        }
        self.fields.clear();
        self.status = format!("{} updated", self.doc.get(t.id).map(|f| f.label.clone()).unwrap_or_default());
    }

    /// Cancel: drop a just-created feature, or restore the old values.
    fn task_cancel(&mut self) {
        let Some(t) = self.task.take() else { return };
        self.restore(&t.before);
        self.undo.pop();
        if t.created {
            self.tree_sel.clear();
        }
        self.status = "Cancelled".into();
    }

    /// A field still being typed in when OK is pressed counts (Qt commits on focus loss).
    fn commit_focused_field(&mut self) {
        let keys: Vec<(String, String)> = self.fields.iter().map(|(k, v)| (k.clone(), v.clone())).collect();
        for (k, text) in keys {
            if k.contains(':') {
                continue;
            }
            let cur = self.doc.expressions.get(&k).map(|e| format!("={e}"));
            let val = self.doc.read_value(&k).map(fmt_num);
            let t = text.trim();
            let same = cur.as_deref() == Some(t) || val.as_deref().is_some_and(|v| t == v || t.split_whitespace().next() == Some(v));
            if !same && !t.is_empty() {
                self.apply_field(&k, t);
            }
        }
    }

    fn delete_selected(&mut self) {
        let Some(&id) = self.tree_sel.last() else { return };
        self.try_delete(DeleteWhat::Feature(id), OnUsers::Refuse);
    }

    fn try_delete(&mut self, what: DeleteWhat, policy: OnUsers) {
        let before = self.state_json();
        let res = match &what {
            DeleteWhat::Feature(id) => self.doc.delete_feature(*id, policy),
            DeleteWhat::Param(n) => self.doc.remove_parameter(n, policy),
        };
        match res {
            Ok(()) => {
                self.undo.push(before);
                self.redo.clear();
                self.tree_sel.retain(|id| self.doc.get(*id).is_some());
                self.after_change();
            }
            Err(users) => {
                let structural = users.iter().any(|u| u.ends_with("(uses it as input)"));
                self.dialog = Some(if structural {
                    Dialog::Message(format!("Can't delete: {} depends on it. Delete that first.", users.join(", ")))
                } else {
                    Dialog::ConfirmDelete { what, users }
                });
            }
        }
    }

    fn toggle_suppressed(&mut self, id: FeatureId) {
        let sup = self.doc.get(id).map(|f| !f.suppressed).unwrap_or(false);
        self.change(|d| d.get_mut(id).unwrap().suppressed = sup);
    }

    fn toggle_tip(&mut self, id: FeatureId) {
        let is_tip = self.doc.tip == Some(id);
        self.change(|d| d.tip = if is_tip { None } else { Some(id) });
    }

    // ------------------------------------------------------------------ files

    fn new_document(&mut self) {
        self.snapshot();
        self.load_workspace(workspace::fresh_workspace().0);
        self.path = None;
        self.after_change();
        self.dirty = false;
        self.vp.set_standard_view(StandardView::Isometric);
    }

    fn open_dialog(&mut self) {
        if let Some(p) = rfd::FileDialog::new().add_filter("freecad-rs or FreeCAD document", &["fcrs", "FCStd", "fcstd"]).pick_file() {
            self.open_path(p);
        }
    }

    fn open_path(&mut self, p: PathBuf) {
        let is_fcstd = p.extension().is_some_and(|e| e.eq_ignore_ascii_case("fcstd"));
        // FreeCAD assemblies (and the bodies they use) open as a workspace; v2 .fcrs too
        let text = if is_fcstd { None } else { std::fs::read_to_string(&p).ok() };
        let ws = if is_fcstd {
            self.import_fcstd_assembly(&p)
        } else {
            text.as_ref().map(|s| fc_assembly::Workspace::from_json(s).map_err(|e| e.to_string()))
        };
        if let Some(ws) = ws {
            match ws {
                Ok(ws) => {
                    self.load_workspace(ws);
                    if let Some(t) = &text {
                        self.fem_load_text(t);
                    }
                    self.disk_sig = if is_fcstd { None } else { file_sig(&p) };
                    self.disk_changed = false;
                    self.path = Some(if is_fcstd { p.with_extension("fcrs") } else { p });
                    self.dirty = is_fcstd;
                    self.undo.clear();
                    self.redo.clear();
                    self.vp.fit_all();
                }
                Err(e) => self.dialog = Some(Dialog::Message(format!("Could not open {}: {e}", p.display()))),
            }
            return;
        }
        let loaded = if is_fcstd {
            fc_part::fcstd::import_with_report(&p).map_err(|e| e.to_string()).map(|(d, rep)| {
                let skipped: Vec<String> = rep.notes.iter().filter(|n| !matches!(n.kind, fc_part::fcstd::NoteKind::Info)).map(|n| format!("{}: {}", n.object, n.message)).collect();
                if !skipped.is_empty() {
                    self.dialog = Some(Dialog::Message(format!("Imported from FreeCAD {} with notes:\n{}", rep.program_version, skipped.join("\n"))));
                }
                d
            })
        } else {
            std::fs::read_to_string(&p).map_err(|e| e.to_string()).and_then(|s| Document::from_json(&s).map_err(|e| e.to_string()))
        };
        match loaded {
            Ok(d) => {
                let mut ws = fc_assembly::Workspace::new();
                ws.add_part("Body", d);
                self.load_workspace(ws);
                // FreeCAD files are imported; saving writes a .fcrs next to them.
                self.disk_sig = if is_fcstd { None } else { file_sig(&p) };
                self.disk_changed = false;
                self.path = Some(if is_fcstd { p.with_extension("fcrs") } else { p });
                self.dirty = is_fcstd;
                self.undo.clear();
                self.redo.clear();
                self.tree_sel.clear();
                self.rebuild_scene();
                self.vp.fit_all();
            }
            Err(e) => self.dialog = Some(Dialog::Message(format!("Could not open {}: {e}", p.display()))),
        }
    }

    /// Re-read the open .fcrs in place: camera and (still valid) selection stay.
    fn reload_from_disk(&mut self) {
        let Some(p) = self.path.clone() else { return };
        let sig = file_sig(&p);
        let text = std::fs::read_to_string(&p).unwrap_or_default();
        match fc_assembly::Workspace::from_json(&text).map_err(|e| e.to_string()) {
            Ok(ws) => {
                self.editor = None;
                self.task = None;
                self.asm.task = None;
                self.fem.task = None;
                self.snapshot();
                let tab = self.tab;
                self.load_workspace(ws);
                self.fem_load_text(&text);
                if self.ws.part(match tab { Tab::Part(p) => p, _ => u32::MAX }).is_some() || self.ws.assembly(match tab { Tab::Assembly(a) => a, _ => u32::MAX }).is_some() {
                    self.park();
                    self.tab = tab;
                    self.unpark();
                    self.solve_active(false);
                }
                self.dirty = false;
                self.disk_sig = sig;
                self.disk_changed = false;
                self.tree_sel.retain(|id| self.doc.get(*id).is_some());
                self.fields.clear();
                self.rebuild_scene();
                self.vp.set_selection(self.selection.clone());
                self.status = format!("Reloaded {}", p.display());
            }
            // Probably caught mid-write: try again at the next poll.
            Err(e) => {
                self.disk_sig = None;
                self.status = format!("Could not reload {}: {e}", p.display());
            }
        }
    }

    /// Poll the open file every ~500 ms; our own saves update `disk_sig` and are ignored.
    fn poll_disk(&mut self, ctx: &egui::Context) {
        ctx.request_repaint_after(std::time::Duration::from_millis(500));
        if self.last_poll.elapsed() < std::time::Duration::from_millis(500) {
            return;
        }
        self.last_poll = std::time::Instant::now();
        let Some(p) = self.path.clone() else { return };
        let Some(now) = file_sig(&p) else { return };
        if self.disk_sig == Some(now) || self.disk_changed {
            return;
        }
        if self.dirty || self.editor.is_some() || self.task.is_some() || self.asm.task.is_some() {
            self.disk_changed = true;
        } else {
            self.reload_from_disk();
        }
    }

    fn disk_banner(&mut self, ui: &mut egui::Ui) {
        let name = self.path.as_ref().and_then(|p| p.file_name()).map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
        egui::Panel::top("disk_banner").frame(egui::Frame::new().fill(egui::Color32::from_rgb(70, 55, 20)).inner_margin(egui::Margin::symmetric(10, 5))).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.add(crate::icons::image(ui.ctx(), "Warning", 16.0));
                ui.label(egui::RichText::new(format!("{name} was changed on disk by another program.")).color(egui::Color32::WHITE));
                if ui.button("Reload").on_hover_text("Discard your unsaved changes and load the file from disk").clicked() {
                    self.reload_from_disk();
                }
                if ui.button("Keep mine").on_hover_text("Ignore the change on disk; your next save overwrites it").clicked() {
                    self.disk_sig = self.path.as_deref().and_then(file_sig);
                    self.disk_changed = false;
                }
            });
        });
    }

    fn save(&mut self, as_new: bool) {
        let path = match (&self.path, as_new) {
            (Some(p), false) => Some(p.clone()),
            _ => rfd::FileDialog::new().add_filter("freecad-rs document", &["fcrs"]).set_file_name("Unnamed.fcrs").save_file(),
        };
        if let Some(p) = path {
            let json = self.save_json();
            match std::fs::write(&p, json) {
                Ok(()) => {
                    self.status = format!("Saved {}", p.display());
                    self.disk_sig = file_sig(&p);
                    self.disk_changed = false;
                    self.path = Some(p);
                    self.dirty = false;
                }
                Err(e) => self.dialog = Some(Dialog::Message(format!("Save failed: {e}"))),
            }
        }
    }

    fn export(&mut self, stl: bool) {
        let Some(shape) = self.doc.result().cloned() else {
            self.dialog = Some(Dialog::Message("Nothing to export yet.".into()));
            return;
        };
        let (name, ext) = if stl { ("STL mesh", "stl") } else { ("STEP", "step") };
        let stem = self.path.as_ref().and_then(|p| p.file_stem()).map(|s| s.to_string_lossy().into_owned()).unwrap_or_else(|| "Body".into());
        let Some(p) = rfd::FileDialog::new().add_filter(name, &[ext]).set_file_name(format!("{stem}.{ext}")).save_file() else { return };
        let ps = p.to_string_lossy().into_owned();
        let r = if stl { shape.write_stl(&ps, scene::deflection(&shape) * 0.5) } else { shape.write_step(&ps) };
        match r {
            Ok(()) => self.status = format!("Exported {}", p.display()),
            Err(e) => self.dialog = Some(Dialog::Message(format!("Export failed: {e}"))),
        }
    }

    fn doc_title(&self) -> String {
        self.path.as_ref().and_then(|p| p.file_stem()).map(|s| s.to_string_lossy().into_owned()).unwrap_or_else(|| "Unnamed".into())
    }

    // ------------------------------------------------------------- value fields

    /// Numeric property editor that also takes units and formulas (`=width/2`), with
    /// FreeCAD's fx button and spin arrows. A value driven by a formula shows its result in
    /// blue; fx (or typing '=') edits the formula. Inside a task, plain numbers apply live
    /// as you type. Returns true if the document changed.
    fn value_field(&mut self, ui: &mut egui::Ui, key: &str, unit: &str, width: f32) -> bool {
        let Some(canon) = self.doc.canonical_name(key) else {
            widgets::ro_value(ui, "—");
            return false;
        };
        let value = self.doc.read_value(&canon).unwrap_or(0.0);
        let bound = self.doc.expressions.get(&canon).cloned();
        let locked = bound.as_ref().is_some_and(|e| self.doc.driven.contains_key(e.trim()));
        let mut changed = false;
        let id = ui.id().with(("field", &canon));
        let initial = format!("{} {unit}", fmt_num(value)).trim().to_string();
        let buf = self.fields.entry(canon.clone()).or_insert_with(|| initial.clone());
        let (fx_w, spin_w) = (24.0, 18.0);
        let mut te = egui::TextEdit::singleline(buf).id(id).desired_width((width - fx_w - spin_w - 14.0).max(40.0)).margin(egui::Margin::symmetric(5, 2));
        if locked {
            te = te.text_color(crate::theme::TEXT_DIM);
        }
        let resp = ui.add_enabled(!locked, te);
        widgets::reg(format!("field:{canon}"), resp.rect);
        let resp = match &bound {
            Some(e) => resp.on_hover_text(format!("= {e}\nDriven by a formula (fx edits it). Typing a plain number removes the formula.")),
            None => resp.on_hover_text("A number with units (25 mm, 30 deg) or a formula starting with '=' (=width/2)"),
        };
        // live preview while typing a plain number inside a task
        if resp.changed() && self.task.is_some() {
            let t = buf.trim().to_string();
            if !t.starts_with('=') && !t.is_empty() && t != initial {
                if let Ok(e) = fc_part::expr::parse(&t) {
                    let mut refs = Vec::new();
                    e.refs(&mut refs);
                    refs.retain(|r| r != "pi" && r != "e");
                    if refs.is_empty() {
                        if let Ok(v) = e.eval(&mut |n| Err(fc_part::expr::ExprError(format!("unknown {n}")))) {
                            if (v - value).abs() > 1e-12 || bound.is_some() {
                                let keep = buf.clone();
                                let _ = self.doc.set_expression(&canon, "");
                                self.doc.write_value(&canon, v);
                                self.after_change();
                                self.fields.insert(canon.clone(), keep);
                                changed = true;
                            }
                        }
                    }
                }
            }
        }
        let commit = resp.lost_focus() && !ui.input(|i| i.key_pressed(egui::Key::Escape));
        if commit {
            let text = self.fields.get(&canon).cloned().unwrap_or_default().trim().to_string();
            if text != initial {
                changed |= self.apply_field(&canon, &text);
            }
            self.fields.remove(&canon);
        }
        if resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Escape)) {
            self.fields.remove(&canon);
        }
        // fx button
        let icon = if bound.is_some() { "bound-expression" } else { "bound-expression-unset" };
        ui.add_space(2.0);
        let (r, fx) = ui.allocate_exact_size(egui::vec2(fx_w, fx_w), egui::Sense::click());
        widgets::reg(format!("fx:{canon}"), r);
        if fx.hovered() {
            ui.painter().rect_filled(r, 3.0, crate::theme::BUTTON_HOVER);
        }
        crate::icons::paint(ui.painter(), icon, r.shrink(2.0), egui::Color32::WHITE);
        let fx = fx.on_hover_text(if bound.is_some() { "Edit the formula" } else { "Enter a formula (expression) for this value" });
        if fx.clicked() && !locked {
            let f = bound.clone().unwrap_or_else(|| fmt_num(value));
            self.fields.insert(canon.clone(), format!("={f}"));
            ui.memory_mut(|m| m.request_focus(id));
        }
        // spin arrows (step 1; Shift: 0.1)
        ui.add_space(2.0);
        let (sr, _) = ui.allocate_exact_size(egui::vec2(spin_w, fx_w), egui::Sense::hover());
        let enabled = bound.is_none() && !locked && ui.is_enabled();
        ui.painter().rect_stroke(sr.shrink2(egui::vec2(0.0, 1.0)), 3.0, egui::Stroke::new(1.0, egui::Color32::from_rgb(80, 80, 80)), egui::StrokeKind::Inside);
        ui.painter().line_segment([egui::pos2(sr.left() + 2.0, sr.center().y), egui::pos2(sr.right() - 2.0, sr.center().y)], egui::Stroke::new(1.0, egui::Color32::from_rgb(70, 70, 70)));
        let fine = ui.input(|i| i.modifiers.shift);
        for (up, half) in [(true, egui::Rect::from_min_max(sr.min, egui::pos2(sr.right(), sr.center().y))), (false, egui::Rect::from_min_max(egui::pos2(sr.left(), sr.center().y), sr.max))] {
            let resp = ui.interact(half, id.with(up), if enabled { egui::Sense::click() } else { egui::Sense::hover() });
            let c = half.center();
            let col = if !enabled { egui::Color32::from_gray(85) } else if resp.hovered() { egui::Color32::WHITE } else { egui::Color32::from_gray(190) };
            if resp.hovered() && enabled {
                ui.painter().rect_filled(half.shrink(0.5), 2.0, crate::theme::BUTTON_HOVER);
            }
            let pts = if up { vec![c + egui::vec2(-4.0, 2.0), c + egui::vec2(4.0, 2.0), c + egui::vec2(0.0, -2.5)] } else { vec![c + egui::vec2(-4.0, -2.0), c + egui::vec2(4.0, -2.0), c + egui::vec2(0.0, 2.5)] };
            ui.painter().add(egui::Shape::convex_polygon(pts, col, egui::Stroke::NONE));
            if resp.clicked() {
                let step = if fine { 0.1 } else { 1.0 };
                let v = if up { value + step } else { value - step };
                changed |= self.apply_field(&canon, &fmt_num(v));
                self.fields.remove(&canon);
            }
        }
        changed
    }

    fn apply_field(&mut self, canon: &str, text: &str) -> bool {
        let src = text.strip_prefix('=').unwrap_or(text).trim().to_string();
        let parsed = match fc_part::expr::parse(&src) {
            Ok(e) => e,
            Err(e) => {
                self.status = format!("Invalid value: {e}");
                return false;
            }
        };
        let mut refs = Vec::new();
        parsed.refs(&mut refs);
        refs.retain(|r| r != "pi" && r != "e");
        let before = self.state_json();
        let result = if text.starts_with('=') || !refs.is_empty() {
            self.doc.set_expression(canon, &src).map_err(|e| e.to_string())
        } else {
            match parsed.eval(&mut |n| Err(fc_part::expr::ExprError(format!("unknown {n}")))) {
                Ok(v) => {
                    let _ = self.doc.set_expression(canon, "");
                    self.doc.write_value(canon, v);
                    Ok(())
                }
                Err(e) => Err(e.to_string()),
            }
        };
        match result {
            Ok(()) => {
                if self.state_json() == before {
                    return false;
                }
                if self.task.is_none() {
                    self.undo.push(before);
                    self.redo.clear();
                }
                self.after_change();
                true
            }
            Err(e) => {
                self.status = format!("{e}");
                false
            }
        }
    }

    fn add_measurement(&mut self, faces: &[u32]) {
        let kind = self.meas_kind;
        let Some(fid) = self.tip_solid_feature() else {
            self.status = "Nothing to measure yet".into();
            return;
        };
        let shape = self.doc.state(fid).and_then(|s| s.shape.clone());
        let mut refs = vec![];
        if kind.faces_needed() > 0 {
            let Some(shape) = shape else { return };
            for &f in faces {
                match face_ref(fid, &shape, f as usize) {
                    Ok(r) => refs.push(r),
                    Err(e) => {
                        self.status = format!("Face{f}: {e}");
                        return;
                    }
                }
            }
        }
        let (spec, base) = match kind {
            MeasKind::Volume => (fc_part::measure::MeasureSpec::Volume, "Volume".to_string()),
            MeasKind::Area => (fc_part::measure::MeasureSpec::Area, "Area".to_string()),
            MeasKind::BBoxX => (fc_part::measure::MeasureSpec::BBoxX, "BBoxX".to_string()),
            MeasKind::BBoxY => (fc_part::measure::MeasureSpec::BBoxY, "BBoxY".to_string()),
            MeasKind::BBoxZ => (fc_part::measure::MeasureSpec::BBoxZ, "BBoxZ".to_string()),
            MeasKind::Value => {
                if self.meas_value_key.is_empty() {
                    return;
                }
                let b: String = self.meas_value_key.chars().map(|c| if c.is_ascii_alphanumeric() { c } else { '_' }).collect();
                (fc_part::measure::MeasureSpec::Value { key: self.meas_value_key.clone() }, b)
            }
            MeasKind::FaceArea => (fc_part::measure::MeasureSpec::FaceArea { face: refs[0].clone() }, format!("FaceArea{}", faces[0])),
            MeasKind::FaceDiameter => (fc_part::measure::MeasureSpec::FaceDiameter { face: refs[0].clone() }, format!("FaceDiameter{}", faces[0])),
            MeasKind::FaceDistance => (fc_part::measure::MeasureSpec::FaceDistance { a: refs[0].clone(), b: refs[1].clone() }, format!("Distance{}_{}", faces[0], faces[1])),
        };
        let mut name = base.clone();
        let mut n = 2;
        while self.doc.measurements.contains_key(&name) || self.doc.parameters.contains_key(&name) {
            name = format!("{base}{n}");
            n += 1;
        }
        let before = self.state_json();
        match self.doc.set_measurement(&name, spec) {
            Ok(()) => {
                self.undo.push(before);
                self.redo.clear();
                self.dirty = true;
                self.status = format!("Added measurement {name}");
            }
            Err(e) => self.status = e,
        }
    }

    fn dialogs(&mut self, ctx: &egui::Context) {
        let Some(d) = self.dialog.take() else { return };
        if let Dialog::AsmExport(o) = &d {
            let mut o = o.clone();
            if self.asm_export_window(ctx, &mut o) && self.dialog.is_none() {
                self.dialog = Some(Dialog::AsmExport(o));
            }
            return;
        }
        let mut keep = true;
        let modal = |title: &str| egui::Window::new(title.to_string()).collapsible(false).resizable(false).anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0]).min_width(320.0);
        match &d {
            Dialog::ConfirmDelete { what, users } => {
                let what = what.clone();
                modal("Delete").show(ctx, |ui| {
                    ui.horizontal(|ui| {
                        ui.add(crate::icons::image(ui.ctx(), "Warning", 32.0));
                        ui.vertical(|ui| {
                            ui.label("These formulas use it:");
                            for u in users {
                                ui.label(egui::RichText::new(format!("• {u}")).monospace());
                            }
                            ui.label("Delete it anyway and keep their current values as plain numbers?");
                        });
                    });
                    ui.add_space(8.0);
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if widgets::push_button(ui, "dlg:delete", "Delete and keep values", 160.0, true).clicked() {
                            keep = false;
                            self.try_delete(what.clone(), OnUsers::Freeze);
                        }
                        if widgets::push_button(ui, "dlg:cancel", "Cancel", 90.0, false).clicked() {
                            keep = false;
                        }
                    });
                });
            }
            Dialog::Message(m) => {
                modal("freecad-rs").show(ctx, |ui| {
                    ui.horizontal(|ui| {
                        ui.add(crate::icons::image(ui.ctx(), "Warning", 32.0));
                        ui.label(m);
                    });
                    ui.add_space(8.0);
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if widgets::push_button(ui, "dlg:ok", "OK", 90.0, true).clicked() {
                            keep = false;
                        }
                    });
                });
                if ctx.input(|i| i.key_pressed(egui::Key::Enter) || i.key_pressed(egui::Key::Escape)) {
                    keep = false;
                }
            }
            Dialog::AsmExport(_) => {}
            Dialog::About => {
                modal("About freecad-rs").show(ctx, |ui| {
                    ui.horizontal(|ui| {
                        ui.add(crate::icons::image(ui.ctx(), "PartDesignWorkbench", 48.0));
                        ui.vertical(|ui| {
                            ui.label(egui::RichText::new("freecad-rs").font(crate::theme::bold_font(16.0)));
                            ui.label("A Rust reimplementation of FreeCAD's Sketcher and Part Design.");
                            ui.label(egui::RichText::new("Icons © FreeCAD contributors, LGPL-2.1-or-later.").color(crate::theme::TEXT_DIM).small());
                        });
                    });
                    ui.add_space(8.0);
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if widgets::push_button(ui, "dlg:ok", "OK", 90.0, true).clicked() {
                            keep = false;
                        }
                    });
                });
            }
        }
        if keep && self.dialog.is_none() {
            self.dialog = Some(d);
        }
    }
}

pub fn fmt_num(v: f64) -> String {
    let s = format!("{v:.4}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s == "-0" { "0".into() } else { s.to_string() }
}

impl App {
    /// Feed one step of the input script into egui's raw input.
    fn drive_input(&mut self, ctx: &egui::Context, raw: &mut egui::RawInput) {
        use crate::input_script::Step;
        use egui::Event;
        let Some(mut run) = self.input.take() else { return };
        raw.focused = true;
        raw.events.append(&mut run.pending);
        if run.wait > 0 {
            run.wait -= 1;
            self.input = Some(run);
            return;
        }
        let Some(step) = run.steps.pop_front() else {
            eprintln!("[input] done, {} failure(s)", run.failures);
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            return;
        };
        let mut moved = |run: &mut crate::input_script::Runner, p: Option<egui::Pos2>, what: &str| match p {
            Some(p) => {
                run.pointer = p;
                raw.events.push(Event::PointerMoved(p));
            }
            None => {
                eprintln!("[input] FAIL: {what} is not on screen");
                run.failures += 1;
            }
        };
        match step {
            Step::AtSketch(x, y) => {
                let p = self.editor.as_ref().map(|e| e.placement().to_world(x, y)).and_then(|w| self.vp.world_to_screen(w));
                moved(&mut run, p, "sketch point");
            }
            Step::AtWorld(w) => {
                let p = self.vp.world_to_screen(w);
                moved(&mut run, p, "world point");
            }
            Step::AtScreen(x, y) => moved(&mut run, Some(egui::pos2(x, y)), "screen point"),
            Step::AtTriad(name) => {
                let p = self.triad_screen(&name);
                moved(&mut run, p, &format!("triad '{name}'"));
            }
            Step::AtInst(name, l) => {
                let p = self.asm_ref().and_then(|a| a.instance_by_name(&name)).map(|i| i.placement.transform_point(l)).and_then(|w| self.vp.world_to_screen(w));
                moved(&mut run, p, &format!("point of '{name}'"));
            }
            Step::AtUi(name) => {
                let p = widgets::lookup(&name).map(|r| r.center());
                moved(&mut run, p, &format!("ui '{name}'"));
            }
            Step::Down(b) => raw.events.push(run.button_event(b, true)),
            Step::Up(b) => raw.events.push(run.button_event(b, false)),
            Step::Click(b) => {
                raw.events.push(run.button_event(b, true));
                run.pending.push(run.button_event(b, false));
                run.wait = 1;
            }
            Step::DoubleClick => {
                let b = egui::PointerButton::Primary;
                // First click now, second click on the next frame (egui measures the gap).
                raw.events.push(run.button_event(b, true));
                raw.events.push(run.button_event(b, false));
                run.pending.push(run.button_event(b, true));
                run.pending.push(run.button_event(b, false));
                run.wait = 2;
            }
            Step::Key(key, modifiers) => {
                raw.events.push(Event::ModifiersChanged(modifiers));
                raw.events.push(Event::Key { key, physical_key: Some(key), pressed: true, repeat: false, modifiers });
                run.pending.push(Event::Key { key, physical_key: Some(key), pressed: false, repeat: false, modifiers });
                run.pending.push(Event::ModifiersChanged(egui::Modifiers::default()));
                run.wait = 1;
            }
            Step::Type(t) => raw.events.push(Event::Text(t)),
            Step::Scroll(dx, dy) => raw.events.push(Event::MouseWheel {
                unit: egui::MouseWheelUnit::Point,
                delta: egui::vec2(dx, dy),
                phase: egui::TouchPhase::Move,
                modifiers: egui::Modifiers::default(),
            }),
            Step::Wait(n) => run.wait = n,
            Step::Expect(what, val) => {
                let got = match what.as_str() {
                    "dof" if self.active_assembly().is_some() => self.asm.report.as_ref().map(|r| r.dof.to_string()),
                    "dof" => self.editor.as_ref().and_then(|e| self.doc.get(e.feature())).and_then(|f| match &f.kind {
                        FeatureKind::Sketch(s) => Some(s.sketch.diagnose().dof.to_string()),
                        _ => None,
                    }),
                    "asmtask" => Some(self.asm.task.is_some().to_string()),
                    "refs" => self.asm_ref().and_then(|a| a.joints.last()).map(|j| {
                        let k = |r: &fc_assembly::JointRef| match r {
                            fc_assembly::JointRef::PartConnector { .. } => "PartConnector",
                            fc_assembly::JointRef::AssemblyConnector(_) => "AssemblyConnector",
                            fc_assembly::JointRef::Geometry { .. } => "Geometry",
                            fc_assembly::JointRef::Frame { .. } => "Frame",
                            fc_assembly::JointRef::Origin { .. } => "Origin",
                        };
                        format!("{} {}", k(&j.a), k(&j.b))
                    }),
                    "tab" => Some(match self.tab {
                        Tab::Part(p) => format!("part {}", self.part_name(p)),
                        Tab::Assembly(a) => format!("assembly {}", self.ws.assembly(a).map(|x| x.name.clone()).unwrap_or_default()),
                    }),
                    "instances" => self.asm_ref().map(|a| a.instances.len().to_string()),
                    "joints" => self.asm_ref().map(|a| a.joints.len().to_string()),
                    "connectors" => self.asm_ref().map(|a| a.connectors.len().to_string()),
                    "parts" => Some(self.ws.parts.len().to_string()),
                    "joint" => self.asm_ref().map(|a| a.joints.iter().map(|j| format!("{} {}", j.kind.type_name(), j.name)).collect::<Vec<_>>().join(", ")).map(|s| if s.split(", ").any(|x| x == val) { val.clone() } else { s }),
                    "grounded" => {
                        let (n, want) = val.split_once(' ').unwrap_or((&val, "true"));
                        self.asm_ref().and_then(|a| a.instance_by_name(n)).map(|i| if i.grounded.to_string() == want { val.clone() } else { format!("{n} {}", i.grounded) })
                    }
                    "solved" => self.asm.report.as_ref().map(|r| (r.ok() && r.unsatisfied_joints.is_empty()).to_string()),
                    "moved" => {
                        let (n, want) = val.split_once(' ').unwrap_or((&val, "true"));
                        let cur = self.asm_ref().and_then(|a| a.instance_by_name(n)).map(|i| i.placement);
                        let old = run.marks.get(n).copied();
                        cur.zip(old).map(|(c, o)| {
                            let (d, r) = c.distance_to(&o);
                            let moved = d > 1e-3 || r > 1e-3;
                            if moved.to_string() == want { val.clone() } else { format!("{n} {moved} (moved {d:.4} mm, turned {:.3}°)", r.to_degrees()) }
                        })
                    }
                    "fixed_point" => {
                        // NAME X Y Z: that local point of the instance did not move since `mark`
                        let w: Vec<&str> = val.split_whitespace().collect();
                        let n = w.first().copied().unwrap_or("");
                        let lp: Vec<f64> = w[1..].iter().filter_map(|x| x.parse().ok()).collect();
                        let cur = self.asm_ref().and_then(|a| a.instance_by_name(n)).map(|i| i.placement);
                        let old = run.marks.get(n).copied();
                        cur.zip(old).filter(|_| lp.len() == 3).map(|(c, o)| {
                            let l = [lp[0], lp[1], lp[2]];
                            let d = fc_part::placement::norm(fc_part::placement::sub(c.transform_point(l), o.transform_point(l)));
                            if d < 1e-4 { val.clone() } else { format!("moved by {d:.5} mm") }
                        })
                    }
                    "connector" => {
                        // NAME X Y Z: origin of a part mate connector (Part Design)
                        let w: Vec<&str> = val.split_whitespace().collect();
                        let v: Vec<f64> = w[1..].iter().filter_map(|x| x.parse().ok()).collect();
                        self.doc.by_name(w.first().copied().unwrap_or("")).and_then(|f| self.doc.connector_frame(f.id).ok()).map(|f| {
                            let ok = v.len() == 3 && (0..3).all(|i| (f.origin[i] - v[i]).abs() < 1e-6);
                            if ok { val.clone() } else { format!("{:?} z={:?}", f.origin, f.z) }
                        })
                    }
                    "anchor" => self.doc.by_name(&val).or_else(|| self.doc.features.iter().rev().find(|f| matches!(f.kind, FeatureKind::MateConnector(_)))).and_then(|f| match &f.kind {
                        FeatureKind::MateConnector(m) => Some(match &m.anchor {
                            fc_part::Anchor::Origin => "Origin".to_string(),
                            fc_part::Anchor::Topo { at, infer } => format!("{} {infer:?}", at.name()),
                        }),
                        _ => None,
                    }).map(|s| if s.starts_with(&val) { val.clone() } else { s }),
                    "fem_maxdisp" | "fem_maxvm" | "fem_f1" => {
                        // within 1 % of VALUE
                        let r = self.fem_cur().and_then(|a| self.fem.docs[a].rt.results.clone());
                        let got = r.and_then(|r| match what.as_str() {
                            "fem_maxdisp" => r.max_displacement().map(|e| e.value),
                            "fem_maxvm" => r.max_von_mises().map(|e| e.value),
                            _ => r.modes.first().map(|m| m.frequency),
                        });
                        let ok = got.zip(val.parse::<f64>().ok()).is_some_and(|(a, b)| (a - b).abs() <= 0.01 * b.abs());
                        Some(if ok { val.clone() } else { format!("{got:?}") })
                    }
                    "leaves" => self.active_assembly().map(|a| self.ws.leaves(a).len().to_string()),
                "interferences" | "interference_volume" => {
                    let vols: Vec<f64> = match &self.asm.task {
                        Some(assembly::AsmTask::Interference(t)) if t.results.is_some() => t.results.as_ref().unwrap().iter().map(|x| x.volume).collect(),
                        _ => self.active_assembly().map(|a| self.ws.interferences(a, &Default::default()).iter().map(|x| x.volume).collect()).unwrap_or_default(),
                    };
                    if what == "interferences" {
                        Some(vols.len().to_string())
                    } else {
                        let ok = vols.first().zip(val.parse::<f64>().ok()).is_some_and(|(a, b)| (a - b).abs() <= 1e-3 * b.abs().max(1.0));
                        Some(if ok { val.clone() } else { format!("{vols:?}") })
                    }
                }
                "clearance" => match &self.asm.task {
                    Some(assembly::AsmTask::Interference(t)) => t.clearance.as_ref().map(|r| match r {
                        Ok(d) if val.parse::<f64>().is_ok_and(|b| (d.value - b).abs() <= 1e-6 * b.abs().max(1.0)) => val.clone(),
                        Ok(d) => d.value.to_string(),
                        Err(e) => e.clone(),
                    }),
                    _ => None,
                },
                "highlight" => Some(self.asm.highlight.is_some().to_string()),
                "bom" | "bom_mass" => {
                    let (n, want) = val.rsplit_once(' ').unwrap_or((&val, ""));
                    let rows = self.active_assembly().map(|a| self.ws.bom(a, &self.bom_opts())).unwrap_or_default();
                    rows.iter().find(|r| r.name == n).map(|r| {
                        let ok = if what == "bom" { r.quantity.to_string() == want } else { r.unit_mass.zip(want.parse::<f64>().ok()).is_some_and(|(a, b)| (a - b).abs() <= 1e-3 * b.abs()) };
                        if ok { val.clone() } else { format!("{n} qty {} mass {:?}", r.quantity, r.unit_mass) }
                    })
                }
                "meta" => {
                    // PART FIELD VALUE...
                    let w: Vec<&str> = val.splitn(3, ' ').collect();
                    let e = w.first().and_then(|n| self.ws.parts.iter().find(|p| p.name == *n));
                    e.map(|e| {
                        let got = match w.get(1).copied().unwrap_or("") {
                            "part_number" => e.meta.part_number.clone(),
                            "description" => e.meta.description.clone(),
                            "material" => e.meta.material.clone(),
                            _ => "?".into(),
                        };
                        if got == w.get(2).copied().unwrap_or("") { val.clone() } else { got }
                    })
                }
                "explode_pos" => {
                    let w: Vec<&str> = val.split_whitespace().collect();
                    let v: Vec<f64> = w[1..].iter().filter_map(|x| x.parse().ok()).collect();
                    let asm = self.asm_ref();
                    asm.and_then(|a| a.instance_by_name(w.first().copied().unwrap_or(""))).map(|i| {
                        let pl = self.asm.explode_cache.as_ref().and_then(|c| c.shown.get(&i.id).copied()).unwrap_or(i.placement);
                        let ok = v.len() == 3 && (0..3).all(|k| (pl.pos[k] - v[k]).abs() < 1e-6);
                        if ok { val.clone() } else { format!("{:?}", pl.pos) }
                    })
                }
                "explode_steps" => self.asm_ref().map(|a| a.exploded_views.iter().map(|v| v.steps.len().to_string()).collect::<Vec<_>>().join(",")),
                "file" => Some(std::fs::metadata(&val).map(|m| m.len() > 0).unwrap_or(false).to_string()).map(|s| if s == "true" { val.clone() } else { s }),
                "fem_mode" => Some((self.fem.view.mode + 1).to_string()),
                "fem_kind" => self.fem_cur().and_then(|a| self.fem.docs[a].analysis.constraints.last().map(|c| c.kind.default_name().to_string())),
                "fem_analyses" => Some(self.fem.docs.len().to_string()),
                    "fem_constraints" => self.fem_cur().map(|a| self.fem.docs[a].analysis.constraints.len().to_string()),
                    "fem_refs" => self.fem_cur().and_then(|a| self.fem.docs[a].analysis.constraints.last().map(|c| c.refs.iter().map(|r| r.name()).collect::<Vec<_>>().join(","))),
                    "fem_mesh" => self.fem_cur().map(|a| self.fem.docs[a].rt.mesh.is_some().to_string()),
                    "fem_result" => self.fem_cur().map(|a| self.fem.docs[a].rt.results.is_some().to_string()),
                    "fem_error" => self.fem_cur().map(|a| self.fem.docs[a].rt.error.clone().unwrap_or_default()).map(|e| if !val.is_empty() && e.contains(val.as_str()) { val.clone() } else { e }),
                    "fem_task" => Some(self.fem.task.is_some().to_string()),
                    "fem_on" => Some(self.fem.on.to_string()),
                "fem_parts" => self.fem_cur().map(|a| self.fem.docs[a].analysis.parts.len().to_string()),
                "fem_interactions" => self.fem_cur().map(|a| self.fem.docs[a].analysis.interactions.len().to_string()),
                "fem_contacts" => match &self.fem.task {
                    Some(fem::FemTask::AsmSetup { contacts: Some(Ok(c)), .. }) => Some(c.len().to_string()),
                    _ => None,
                },
                "fem_hover" => Some(self.fem.hover_text.clone().unwrap_or_default()).map(|t| if t.contains(val.as_str()) { val.clone() } else { t }),
                "fem_part_max" => {
                    // PART MAXDISP (within 1 %)
                    let (n, want) = val.rsplit_once(' ').unwrap_or((&val, ""));
                    self.fem_cur().and_then(|a| self.fem.docs[a].rt.part_results.clone()).and_then(|v| v.into_iter().find(|p| p.name == n)).map(|p| {
                        let got = p.max_displacement.map(|e| e.value);
                        let ok = got.zip(want.parse::<f64>().ok()).is_some_and(|(a, b)| (a - b).abs() <= 0.01 * b.abs());
                        if ok { val.clone() } else { format!("{got:?}") }
                    })
                }
                    "editing" => Some(self.editor.is_some().to_string()),
                    "task" => Some(self.task.is_some().to_string()),
                    "picking" => Some(self.picking.to_string()),
                    "features" => Some(self.doc.features.len().to_string()),
                    "support" => self.editor.as_ref().and_then(|e| self.doc.get(e.feature())).and_then(|f| match &f.kind {
                        FeatureKind::Sketch(s) => Some(match &s.support {
                            Support::XY => "XY".to_string(),
                            Support::XZ => "XZ".to_string(),
                            Support::YZ => "YZ".to_string(),
                            Support::Face(fr) => format!("Face{}", fr.index),
                        }),
                        _ => None,
                    }),
                    "measure" | "param" => {
                        let (n, want) = val.split_once(' ').unwrap_or((&val, ""));
                        let got = if what == "measure" { self.doc.measure(n).ok().map(|m| m.value) } else { self.doc.evaluate_expressions().0.get(&format!("param:{n}")).copied() };
                        let ok = got.zip(want.parse::<f64>().ok()).is_some_and(|(a, b)| (a - b).abs() <= 1e-3 * b.abs().max(1.0));
                        Some(if ok { val.clone() } else { format!("{got:?}") })
                    }
                    "banner" => Some(self.disk_changed.to_string()),
                    "dirty" => Some(self.dirty.to_string()),
                    "undo" => Some(self.undo.len().to_string()),
                    "driven" => Some(self.doc.driven.contains_key(&val).to_string()).map(|s| if s == "true" { val.clone() } else { s }),
                    "volume" => self.doc.result().map(|s| format!("{:.3}", s.volume())),
                    "geometry" => self.editor.as_ref().and_then(|e| self.doc.get(e.feature())).and_then(|f| match &f.kind {
                        FeatureKind::Sketch(s) => Some(s.sketch.geometry.len().to_string()),
                        _ => None,
                    }),
                    "constraints" => self.editor.as_ref().and_then(|e| self.doc.get(e.feature())).and_then(|f| match &f.kind {
                        FeatureKind::Sketch(s) => Some(s.sketch.constraints.len().to_string()),
                        _ => None,
                    }),
                    _ => None,
                };
                let ok = match (&got, what.as_str()) {
                    (Some(g), "volume") => g.parse::<f64>().ok().zip(val.parse::<f64>().ok()).is_some_and(|(a, b)| (a - b).abs() <= 1e-3 * b.abs().max(1.0)),
                    (Some(g), _) => *g == val,
                    _ => false,
                };
                if ok {
                    eprintln!("[input] ok: {what} = {val}");
                } else {
                    eprintln!("[input] FAIL: expected {what} = {val}, got {got:?} (status: {})", self.status);
                    run.failures += 1;
                }
            }
            Step::Do(cmd, arg) => match cmd.as_str() {
                "open" => {
                    self.open_path(PathBuf::from(&arg));
                }
                "asmdemo" => {
                    self.load_workspace(crate::demo::assembly_workspace(arg == "assembled"));
                    self.path = None;
                    self.dirty = false;
                    self.undo.clear();
                    self.vp.set_standard_view(StandardView::Isometric);
                }
                "femwait" => {
                    // hold the script until the FEM run finishes (at most N frames)
                    let left: u32 = arg.trim().parse().unwrap_or(3000);
                    if self.fem.job.is_some() && left > 0 {
                        run.steps.push_front(Step::Do("femwait".into(), (left - 1).to_string()));
                        run.wait = 1;
                    } else if self.fem.job.is_some() {
                        eprintln!("[input] FAIL: FEM run still busy");
                        run.failures += 1;
                    }
                }
                "animwait" => {
                    // hold the script until the exploded view animation ends (at most N frames)
                    let left: u32 = arg.trim().parse().unwrap_or(3000);
                    let playing = matches!(&self.asm.task, Some(assembly::AsmTask::Explode(t)) if t.play.is_some());
                    if playing && left > 0 {
                        run.steps.push_front(Step::Do("animwait".into(), (left - 1).to_string()));
                        run.wait = 1;
                    }
                }
                "mark" => {
                    run.marks = self.asm_ref().map(|a| a.instances.iter().map(|i| (i.name.clone(), i.placement)).collect()).unwrap_or_default();
                }
                "asmtest" => match crate::demo::asm_test_workspace(arg.trim()) {
                    Some(ws) => {
                        self.load_workspace(ws);
                        self.path = None;
                        self.dirty = false;
                        self.undo.clear();
                        self.vp.set_standard_view(StandardView::Isometric);
                        self.vp.fit_all();
                    }
                    None => {
                        eprintln!("[input] FAIL: no test assembly {arg}");
                        run.failures += 1;
                    }
                },
                "exportstep" | "exportstl" => {
                    let (path, opt) = arg.split_once(' ').unwrap_or((&arg, ""));
                    let o = asm_tools::AsmExportOpts { stl: cmd == "exportstl", ap242: opt.eq_ignore_ascii_case("ap242"), colors: true, visible_only: false };
                    match self.do_asm_export(std::path::Path::new(path), &o) {
                        Ok(m) => self.status = m,
                        Err(e) => {
                            eprintln!("[input] FAIL: export: {e}");
                            run.failures += 1;
                        }
                    }
                }
                "importstep" => self.import_step_file(std::path::Path::new(arg.trim())),
                "bomcsv" => {
                    if let Err(e) = self.bom_write_csv(std::path::Path::new(arg.trim())) {
                        eprintln!("[input] FAIL: bom csv: {e}");
                        run.failures += 1;
                    }
                }
                "select" => {
                    let ids: Vec<assembly::AsmSel> = arg.split(',').filter_map(|n| self.asm_ref().and_then(|a| a.instance_by_name(n.trim())).map(|i| assembly::AsmSel::Inst(i.id))).collect();
                    self.asm.sel = ids;
                    self.rebuild_scene();
                }
                "platedemo" => {
                    let mut ws = fc_assembly::Workspace::new();
                    ws.add_part("Body", crate::demo::plate_hole_document());
                    self.load_workspace(ws);
                    self.path = None;
                    self.dirty = false;
                    self.undo.clear();
                    self.vp.set_standard_view(StandardView::Isometric);
                    self.vp.fit_all();
                }
                "demo" => {
                    self.doc = crate::demo::demo_document();
                    self.path = None;
                    self.after_change();
                    self.dirty = false;
                    self.vp.set_standard_view(StandardView::Isometric);
                }
                "saveto" => {
                    let p = PathBuf::from(&arg);
                    let json = self.save_json();
                    if std::fs::write(&p, json).is_ok() {
                        self.path = Some(p.clone());
                        self.disk_sig = file_sig(&p);
                        self.dirty = false;
                    }
                }
                "external" => {
                    // Rewrite the open file as another program would; sleep so the mtime moves.
                    std::thread::sleep(std::time::Duration::from_millis(1100));
                    let (name, expr) = arg.split_once(' ').unwrap_or((&arg, "0"));
                    let ok = self.path.clone().is_some_and(|p| {
                        let mut d = std::fs::read_to_string(&p).ok().and_then(|s| Document::from_json(&s).ok()).expect("open file");
                        d.set_driven_parameters(&[(name.to_string(), expr.to_string())], "Calc Canvas: Plate").is_ok() && std::fs::write(&p, d.to_json()).is_ok()
                    });
                    if !ok {
                        eprintln!("[input] FAIL: external write");
                        run.failures += 1;
                    }
                }
                _ => {}
            },
            Step::Shot(p) => {
                ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(egui::UserData::new(p)));
                run.wait = 3;
            }
            Step::Quit => {
                eprintln!("[input] done, {} failure(s)", run.failures);
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                return;
            }
        }
        self.input = Some(run);
    }

    /// Clicks in the 3D view outside of sketch editing and attachment picking.
    fn view_clicked(&mut self, ctx: &egui::Context, clicked: Option<Pick>) {
        let multi = ctx.input(|i| i.modifiers.ctrl || i.modifiers.command);
        let plane = |p: &Pick| self.planes_at.filter(|f| p.object >= *f).map(|f| p.object - f);
        match clicked {
            Some(p) if plane(&p).is_some() => {
                // origin plane: select it like in the tree
                self.origin_sel = Some(3 + plane(&p).unwrap());
                self.tree_sel.clear();
                self.selection.clear();
            }
            Some(p) => {
                self.origin_sel = None;
                if multi {
                    if let Some(i) = self.selection.iter().position(|s| s.kind == p.kind && s.id == p.id && s.object == p.object) {
                        self.selection.remove(i);
                    } else {
                        self.selection.push(p);
                    }
                } else {
                    self.selection = vec![p];
                }
            }
            None => {
                if !multi {
                    self.selection.clear();
                    self.origin_sel = None;
                }
            }
        }
        self.vp.set_selection(self.selection.clone());
        self.sync_origin_highlight();
        if let Some(p) = self.selection.last() {
            self.status = format!("Selected: {}", pick_name(&self.vp, p, self.planes_at));
        } else if let Some(i) = self.origin_sel {
            self.status = format!("Selected: {}", ORIGIN_ITEMS[i].0);
        }
    }
}

impl App {
    /// Mouse interaction with the 3D view for the current mode.
    fn view_input(&mut self, ui: &mut egui::Ui, ctx: &egui::Context, r: &crate::viewport::ViewportResponse) {
        if self.fem_on_assembly() {
            // FEM on an assembly: constraint picking, result read-out
            if !self.fem_view_input(ui, ctx, r) && r.response.clicked() {
                self.view_clicked(ctx, r.clicked.clone());
            }
            return;
        }
        if self.active_assembly().is_some() {
            self.asm_view_input(ui, ctx, r);
            return;
        }
        if let Some(mut ed) = self.editor.take() {
            ed.handle_viewport(&mut self.doc, &mut self.vp, r, ctx);
            self.editor = Some(ed);
            return;
        }
        if self.picking {
            if let Some(p) = r.clicked.clone() {
                self.pick_support(&p);
            }
            self.draw_plane_labels(ui, ctx);
            return;
        }
        if self.show_origin {
            self.draw_plane_labels(ui, ctx);
        }
        if self.fem_view_input(ui, ctx, r) {
            return;
        }
        if self.mate_view_input(ui, ctx, r) {
            return;
        }
        if r.response.clicked() {
            self.view_clicked(ctx, r.clicked.clone());
        }
        self.draw_part_connectors(ui);
    }
}

impl eframe::App for App {
    fn raw_input_hook(&mut self, ctx: &egui::Context, raw_input: &mut egui::RawInput) {
        self.drive_input(ctx, raw_input);
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        let ctx = &ctx;
        widgets::clear_registry();
        if self.first_frame {
            self.first_frame = false;
            if self.editor.is_none() {
                self.vp.fit_all();
            }
        }
        self.shortcuts(ctx);
        self.poll_disk(ctx);
        self.fem_poll(ctx);
        let title = format!("{}{} - freecad-rs", self.doc_title(), if self.dirty { " •" } else { "" });
        ctx.send_viewport_cmd(egui::ViewportCommand::Title(title));

        egui::Panel::top("menubar").frame(egui::Frame::new().fill(crate::theme::WINDOW).inner_margin(egui::Margin::symmetric(6, 2))).show(ui, |ui| self.menu_bar(ui));
        egui::Panel::top("toolbar").frame(egui::Frame::new().fill(crate::theme::WINDOW).inner_margin(egui::Margin { left: 4, right: 4, top: 1, bottom: 3 })).show(ui, |ui| self.toolbars(ui));
        if self.disk_changed {
            self.disk_banner(ui);
        }
        egui::Panel::bottom("status").frame(egui::Frame::new().fill(crate::theme::WINDOW).inner_margin(egui::Margin::symmetric(8, 3))).show(ui, |ui| self.status_bar(ui));
        egui::Panel::left("model")
            .resizable(true)
            .default_size(300.0)
            .size_range(220.0..=520.0)
            .frame(egui::Frame::new().fill(crate::theme::PANEL).inner_margin(egui::Margin { left: 0, right: 2, top: 0, bottom: 0 }))
            .show(ui, |ui| self.model_dock(ui));

        let task_open = self.editor.is_some() || self.task.is_some() || self.picking || self.active_assembly().is_some() || self.fem.task.is_some();
        egui::CentralPanel::default().frame(egui::Frame::NONE).show(ui, |ui| {
            egui::Panel::bottom("doc_tabs").exact_size(28.0).frame(egui::Frame::new().fill(crate::theme::WINDOW)).show(ui, |ui| self.doc_tabs(ui));
            self.vp.right_inset = if task_open { tasks::TASK_W } else { 0.0 };
            if self.active_assembly().is_some() && !self.fem_on_assembly() {
                self.asm_pre_view(ctx);
            }
            // a connector triad under the cursor takes the hover from the geometry behind it
            self.vp.suppress_hover = !self.fem_on_assembly() && ctx.input(|i| i.pointer.hover_pos()).is_some_and(|p| self.triad_under(p));
            let r = self.vp.ui(ui);
            self.view_rect = r.response.rect;
            self.hovered = r.hovered.clone();
            self.view_input(ui, ctx, &r);
            if task_open {
                self.task_panel(ui);
            }
        });

        self.dialogs(ctx);

        // Screenshots requested by the input script carry their path as user data.
        let shots: Vec<(String, std::sync::Arc<egui::ColorImage>)> = ctx.input(|i| {
            i.raw
                .events
                .iter()
                .filter_map(|e| match e {
                    egui::Event::Screenshot { image, user_data, .. } => user_data.data.as_ref().and_then(|d| d.downcast_ref::<String>()).map(|p| (p.clone(), image.clone())),
                    _ => None,
                })
                .collect()
        });
        for (p, img) in shots {
            let [w, h] = img.size;
            let bytes: Vec<u8> = img.pixels.iter().flat_map(|c| c.to_array()).collect();
            let _ = image::save_buffer(&p, &bytes, w as u32, h as u32, image::ExtendedColorType::Rgba8);
            eprintln!("[input] saved {p}");
        }
        if self.input.is_some() {
            ctx.request_repaint();
            return;
        }
        let editor_busy = self.editor.as_ref().is_some_and(|e| e.busy());
        if let Some((path, frames)) = &mut self.screenshot {
            if !editor_busy {
                *frames += 1;
            }
            if *frames == 8 {
                ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(Default::default()));
            }
            let shot = ctx.input(|i| {
                i.raw.events.iter().find_map(|e| match e {
                    egui::Event::Screenshot { image, user_data, .. } if user_data.data.is_none() => Some(image.clone()),
                    _ => None,
                })
            });
            if let Some(img) = shot {
                let [w, h] = img.size;
                let bytes: Vec<u8> = img.pixels.iter().flat_map(|c| c.to_array()).collect();
                let _ = image::save_buffer(&*path, &bytes, w as u32, h as u32, image::ExtendedColorType::Rgba8);
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
            ctx.request_repaint();
        }
    }
}
