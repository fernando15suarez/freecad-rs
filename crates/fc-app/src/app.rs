//! Application shell: Part Design document, model tree, property panel, variables,
//! file I/O and hosting of the sketch editor.

use crate::scene;
use crate::sketcher::{EditorExit, SketchEditor};
use crate::viewport::{Pick, PickKind, StandardView, Viewport};
use eframe::egui;
use fc_part::measure::MeasureSpec;
use fc_part::params::OnUsers;
use fc_part::*;
use std::collections::HashMap;
use std::path::PathBuf;

/// Something waiting on the user in a modal window.
enum Dialog {
    ConfirmDelete { what: DeleteWhat, users: Vec<String> },
    Message(String),
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

pub struct App {
    doc: Document,
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
    editor: Option<SketchEditor>,
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
    /// Choosing the plane/face for a new sketch: index of the first plane object in the view.
    picking_plane: Option<usize>,
}

fn pick_name(vp: &Viewport, p: &Pick) -> String {
    let obj = vp.objects().get(p.object).map(|o| o.name.as_str()).unwrap_or("?");
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
        _ => return None,
    };
    Some(Pick { object: 0, kind, id: id.parse().ok()?, point: [0.0; 3] })
}

fn icon(k: &FeatureKind) -> &'static str {
    match k {
        FeatureKind::Sketch(_) => "✏",
        FeatureKind::Pad(_) => "⬆",
        FeatureKind::Pocket(_) => "⬇",
        FeatureKind::Revolution(_) => "⟳",
        FeatureKind::Groove(_) => "⟲",
        FeatureKind::AdditiveLoft(_) | FeatureKind::SubtractiveLoft(_) => "◇",
        FeatureKind::AdditivePipe(_) | FeatureKind::SubtractivePipe(_) => "〰",
        FeatureKind::Hole(_) => "◎",
    }
}

impl App {
    pub fn new(cc: &eframe::CreationContext<'_>, screenshot: Option<PathBuf>) -> Self {
        let vp = Viewport::new(cc);
        let mut app = App {
            doc: Document::new(),
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
            editor: None,
            undo: vec![],
            redo: vec![],
            status: "Ready".into(),
            dialog: None,
            fields: HashMap::new(),
            new_param: (String::new(), String::new()),
            screenshot: screenshot.map(|p| (p, 0)),
            first_frame: true,
            input: None,
            picking_plane: None,
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
        }
        if let Some(id) = std::env::var("FCRS_EDIT").ok().and_then(|n| app.doc.by_name(&n).map(|f| f.id)) {
            app.start_edit(id);
        }
        app
    }

    // ------------------------------------------------------------------ model ops

    fn rebuild_scene(&mut self) {
        let mut objs = scene::body_objects(&self.doc);
        if self.picking_plane.is_some() {
            self.picking_plane = Some(objs.len());
            let half = crate::plane_picker::plane_half_size(self.doc.result().map(|s| s.bbox()));
            objs.extend(crate::plane_picker::plane_objects(half));
        }
        self.vp.set_objects(objs);
        if self.editor.is_none() {
            let hl = self.tree_sel.iter().copied().find(|id| matches!(self.doc.get(*id).map(|f| &f.kind), Some(FeatureKind::Sketch(_))));
            self.vp.set_overlay(scene::sketch_overlay(&self.doc, hl));
        }
    }

    /// Snapshot for undo, apply a change, recompute and redraw.
    fn change(&mut self, f: impl FnOnce(&mut Document)) {
        self.snapshot();
        f(&mut self.doc);
        self.after_change();
    }

    fn snapshot(&mut self) {
        self.undo.push(self.doc.to_json());
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

    fn restore(&mut self, json: &str) {
        if let Ok(d) = Document::from_json(json) {
            self.doc = d;
            self.doc.recompute();
            self.tree_sel.retain(|id| self.doc.get(*id).is_some());
            self.fields.clear();
            self.rebuild_scene();
        }
    }

    fn undo(&mut self) {
        if self.editor.is_some() {
            return;
        }
        if let Some(prev) = self.undo.pop() {
            self.redo.push(self.doc.to_json());
            self.restore(&prev);
            self.status = "Undo".into();
        }
    }

    fn redo(&mut self) {
        if self.editor.is_some() {
            return;
        }
        if let Some(next) = self.redo.pop() {
            self.undo.push(self.doc.to_json());
            self.restore(&next);
            self.status = "Redo".into();
        }
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
        // A selected planar face of the body becomes the support (sketch on face).
        let face = self.selection.iter().find(|p| p.kind == PickKind::Face).map(|p| p.id as usize + 1);
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

    /// FreeCAD-style "Select feature": show the origin planes and let the user click one,
    /// or a flat face of the part.
    fn start_plane_pick(&mut self) {
        self.picking_plane = Some(0);
        self.selection.clear();
        self.vp.set_selection(vec![]);
        self.rebuild_scene();
        if self.doc.result().is_none() {
            self.vp.set_standard_view(StandardView::Isometric);
        }
        self.status = "Click a plane or a flat face of the part to sketch on (Esc cancels)".into();
    }

    fn end_plane_pick(&mut self) {
        self.picking_plane = None;
        self.rebuild_scene();
    }

    /// Handle a click while picking: true if it chose a support.
    fn pick_support(&mut self, p: &Pick) -> bool {
        let Some(first) = self.picking_plane else { return false };
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
        let painter = ctx.layer_painter(egui::LayerId::new(egui::Order::Foreground, egui::Id::new("plane_labels")));
        let clip = ui.clip_rect();
        for p in crate::plane_picker::planes() {
            if let Some(pos) = self.vp.world_to_screen(p.placement.to_world(half, half)) {
                if clip.contains(pos) {
                    painter.text(pos + egui::vec2(4.0, -4.0), egui::Align2::LEFT_BOTTOM, p.name, egui::FontId::proportional(14.0), egui::Color32::from_rgb(40, 40, 90));
                }
            }
        }
    }

    fn plane_pick_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("New sketch");
        ui.label("Pick where to draw: click one of the planes or a flat face of the part in the 3D view, or choose a plane here.");
        ui.add_space(6.0);
        let first = self.picking_plane.unwrap_or(0);
        let hovered_plane = self.hovered.as_ref().filter(|h| h.object >= first).map(|h| h.object - first);
        let mut chosen = None;
        let mut any_hovered = false;
        for (i, p) in crate::plane_picker::planes().into_iter().enumerate() {
            let r = ui.add_sized([ui.available_width(), 34.0], egui::Button::new(egui::RichText::new(format!("▱  {}", p.name)).size(15.0)).selected(hovered_plane == Some(i)));
            let r = r.on_hover_text(p.hint);
            if r.hovered() {
                any_hovered = true;
                self.vp.force_hover(Some(Pick { object: first + i, kind: PickKind::Face, id: 0, point: [0.0; 3] }));
            }
            if r.clicked() {
                chosen = Some(p.support);
            }
            ui.weak(p.hint);
            ui.add_space(4.0);
        }
        if !any_hovered {
            self.vp.force_hover(None);
        }
        ui.add_space(8.0);
        if ui.button("Cancel").clicked() {
            self.end_plane_pick();
            self.status = "Ready".into();
        }
        if let Some(s) = chosen {
            self.end_plane_pick();
            self.create_sketch(s);
        }
    }

    fn create_sketch(&mut self, support: Support) {
        self.snapshot();
        let id = self.doc.add_sketch(support, fc_sketch::Sketch::default());
        self.after_change();
        self.tree_sel = vec![id];
        self.selection.clear();
        self.vp.set_selection(vec![]);
        self.start_edit(id);
    }

    fn start_edit(&mut self, id: FeatureId) {
        if self.editor.is_some() {
            return;
        }
        if let Some(ed) = SketchEditor::new(&self.doc, id) {
            self.snapshot();
            self.vp.align_to_plane(&scene::plane(&ed.placement()));
            self.status = format!("Editing {}", self.doc.get(id).map(|f| f.label.clone()).unwrap_or_default());
            self.editor = Some(ed);
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
    }

    fn delete_selected(&mut self) {
        let Some(&id) = self.tree_sel.last() else { return };
        self.try_delete(DeleteWhat::Feature(id), OnUsers::Refuse);
    }

    fn try_delete(&mut self, what: DeleteWhat, policy: OnUsers) {
        let before = self.doc.to_json();
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

    // ------------------------------------------------------------------ files

    fn open_path(&mut self, p: PathBuf) {
        let is_fcstd = p.extension().is_some_and(|e| e.eq_ignore_ascii_case("fcstd"));
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
                self.doc = d;
                self.doc.recompute();
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
        match std::fs::read_to_string(&p).map_err(|e| e.to_string()).and_then(|s| Document::from_json(&s).map_err(|e| e.to_string())) {
            Ok(d) => {
                self.editor = None;
                self.snapshot();
                self.doc = d;
                self.doc.recompute();
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
        if self.dirty || self.editor.is_some() {
            self.disk_changed = true;
        } else {
            self.reload_from_disk();
        }
    }

    fn disk_banner(&mut self, ui: &mut egui::Ui) {
        let name = self.path.as_ref().and_then(|p| p.file_name()).map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
        egui::Panel::top("disk_banner").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.colored_label(egui::Color32::from_rgb(230, 170, 50), format!("⚠ {name} changed on disk"));
                if ui.button("Reload").clicked() {
                    self.reload_from_disk();
                }
                if ui.button("Keep mine").clicked() {
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
            match std::fs::write(&p, self.doc.to_json()) {
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

    // ------------------------------------------------------------------ panels

    fn toolbar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            let editing = self.editor.is_some();
            ui.add_enabled_ui(!editing, |ui| {
                if ui.button("New").clicked() {
                    self.snapshot();
                    self.doc = Document::new();
                    self.path = None;
                    self.tree_sel.clear();
                    self.after_change();
                    self.dirty = false;
                }
                if ui.button("Open").clicked() {
                    if let Some(p) = rfd::FileDialog::new().add_filter("freecad-rs or FreeCAD document", &["fcrs", "FCStd", "fcstd"]).pick_file() {
                        self.open_path(p);
                    }
                }
                if ui.button("Save").clicked() {
                    self.save(false);
                }
                if ui.button("Export STEP").clicked() {
                    self.export(false);
                }
                if ui.button("Export STL").clicked() {
                    self.export(true);
                }
                ui.separator();
                if ui.add_enabled(!self.undo.is_empty(), egui::Button::new("Undo")).clicked() {
                    self.undo();
                }
                if ui.add_enabled(!self.redo.is_empty(), egui::Button::new("Redo")).clicked() {
                    self.redo();
                }
            });
            ui.separator();
            for (name, v) in [
                ("Iso", StandardView::Isometric),
                ("Front", StandardView::Front),
                ("Top", StandardView::Top),
                ("Right", StandardView::Right),
                ("Rear", StandardView::Rear),
                ("Bottom", StandardView::Bottom),
                ("Left", StandardView::Left),
            ] {
                if ui.button(name).clicked() {
                    self.vp.set_standard_view(v);
                }
            }
            if ui.button("Fit").clicked() {
                self.vp.fit_all();
            }
            let mut persp = self.vp.perspective();
            if ui.checkbox(&mut persp, "Perspective").changed() {
                self.vp.set_perspective(persp);
                self.vp.fit_all();
            }
            ui.separator();
            if editing {
                let mut ed = self.editor.take();
                if let Some(e) = ed.as_mut() {
                    e.toolbar_ui(&mut self.doc, ui);
                }
                self.editor = ed;
                return;
            }
            if ui.button("✏ Sketch").on_hover_text("New sketch on a selected face, or on a base plane").clicked() {
                self.new_sketch();
            }
            if ui.button("⬆ Pad").clicked() {
                self.add_feature(|s| FeatureKind::Pad(Extrude::new(s, 10.0)));
            }
            if ui.button("⬇ Pocket").clicked() {
                self.add_feature(|s| FeatureKind::Pocket(Extrude::new(s, 5.0)));
            }
            if ui.button("⟳ Revolution").clicked() {
                self.add_feature(|s| FeatureKind::Revolution(Revolve::new(s)));
            }
            if ui.button("⟲ Groove").clicked() {
                self.add_feature(|s| FeatureKind::Groove(Revolve::new(s)));
            }
            if ui.button("◎ Hole").clicked() {
                self.add_feature(|s| FeatureKind::Hole(Hole::new(s, 6.0, 10.0)));
            }
            if ui.button("◇ Loft").clicked() {
                self.add_loft_or_pipe(false, false);
            }
            if ui.button("◇ Sub. Loft").clicked() {
                self.add_loft_or_pipe(true, false);
            }
            if ui.button("〰 Sweep").clicked() {
                self.add_loft_or_pipe(false, true);
            }
            if ui.button("〰 Sub. Sweep").clicked() {
                self.add_loft_or_pipe(true, true);
            }
        });
    }

    fn tree(&mut self, ui: &mut egui::Ui) {
        ui.heading("Model");
        let title = self.path.as_ref().and_then(|p| p.file_stem()).map(|s| s.to_string_lossy().into_owned()).unwrap_or_else(|| "Unnamed".into());
        ui.label(egui::RichText::new(format!("{title}{}", if self.dirty { " •" } else { "" })).strong());
        // Sketches consumed by a solid feature are shown under it, like FreeCAD.
        let consumed: std::collections::HashSet<FeatureId> = self
            .doc
            .features
            .iter()
            .filter(|f| f.kind.is_solid_feature())
            .flat_map(|f| f.kind.inputs())
            .filter(|i| matches!(self.doc.get(*i).map(|f| &f.kind), Some(FeatureKind::Sketch(_))))
            .collect();
        let mut clicked: Option<(FeatureId, bool, bool)> = None;
        let feats: Vec<(FeatureId, String, bool)> = self.doc.features.iter().map(|f| (f.id, f.name.clone(), consumed.contains(&f.id))).collect();
        egui::ScrollArea::vertical().id_salt("tree").max_height(ui.available_height() * 0.45).show(ui, |ui| {
            egui::collapsing_header::CollapsingState::load_with_default_open(ui.ctx(), "tree_body".into(), true)
                .show_header(ui, |ui| ui.label(egui::RichText::new("📦 Body").strong()))
                .body(|ui| {
                    for (id, _, is_consumed) in &feats {
                        if *is_consumed {
                            continue;
                        }
                        let inputs: Vec<FeatureId> = self.doc.get(*id).map(|f| f.kind.inputs()).unwrap_or_default().into_iter().filter(|i| consumed.contains(i)).collect();
                        if inputs.is_empty() || !self.doc.get(*id).map(|f| f.kind.is_solid_feature()).unwrap_or(false) {
                            self.tree_row(ui, *id, &mut clicked);
                        } else {
                            let r = egui::collapsing_header::CollapsingState::load_with_default_open(ui.ctx(), egui::Id::new(("feat", *id)), false)
                                .show_header(ui, |ui| self.tree_row(ui, *id, &mut clicked));
                            r.body(|ui| {
                                for i in inputs {
                                    self.tree_row(ui, i, &mut clicked);
                                }
                            });
                        }
                    }
                });
        });
        if let Some((id, multi, double)) = clicked {
            if double {
                if matches!(self.doc.get(id).map(|f| &f.kind), Some(FeatureKind::Sketch(_))) {
                    self.tree_sel = vec![id];
                    self.start_edit(id);
                    return;
                }
            }
            if multi {
                if let Some(i) = self.tree_sel.iter().position(|x| *x == id) {
                    self.tree_sel.remove(i);
                } else {
                    self.tree_sel.push(id);
                }
            } else {
                self.tree_sel = vec![id];
            }
            self.fields.clear();
            self.rebuild_scene();
        }
    }

    fn tree_row(&self, ui: &mut egui::Ui, id: FeatureId, clicked: &mut Option<(FeatureId, bool, bool)>) {
        let Some(f) = self.doc.get(id) else { return };
        let st = self.doc.state(id);
        let mut text = format!("{} {}", icon(&f.kind), f.label);
        if let Some(rep) = st.and_then(|s| s.solve.as_ref()) {
            if rep.dof > 0 {
                text.push_str(&format!("  ({} DoF)", rep.dof));
            }
        }
        if f.suppressed {
            text.push_str("  (suppressed)");
        }
        let warn = st.and_then(|s| s.warning.clone());
        if warn.is_some() {
            text.push_str("  ⚠");
        }
        let mut rt = egui::RichText::new(text);
        let err = st.and_then(|s| s.error.as_ref());
        if err.is_some() {
            rt = rt.color(egui::Color32::from_rgb(230, 80, 60));
        } else if warn.is_some() {
            rt = rt.color(egui::Color32::from_rgb(230, 170, 40));
        }
        if self.doc.tip == Some(id) {
            rt = rt.underline();
        }
        let r = ui.selectable_label(self.tree_sel.contains(&id), rt);
        let r = match (err, warn) {
            (Some(e), _) => r.on_hover_text(e.to_string()),
            (None, Some(w)) => r.on_hover_text(w),
            _ => r,
        };
        if r.double_clicked() {
            *clicked = Some((id, false, true));
        } else if r.clicked() {
            let multi = ui.input(|i| i.modifiers.command || i.modifiers.ctrl);
            *clicked = Some((id, multi, false));
        }
    }

    /// A numeric property that can hold a formula. Returns true if the document changed.
    fn value_field(&mut self, ui: &mut egui::Ui, label: &str, key: &str, unit: &str) -> bool {
        let Some(canon) = self.doc.canonical_name(key) else { return false };
        let value = self.doc.read_value(&canon).unwrap_or(0.0);
        let bound = self.doc.expressions.get(&canon).cloned();
        let mut changed = false;
        ui.label(label);
        ui.horizontal(|ui| {
            let id = egui::Id::new(("field", &canon));
            let buf = self.fields.entry(canon.clone()).or_insert_with(|| match &bound {
                Some(e) => format!("={e}"),
                None => format!("{} {unit}", fmt_num(value)),
            });
            let resp = ui.add(egui::TextEdit::singleline(buf).id(id).desired_width(110.0));
            if bound.is_some() {
                ui.label(egui::RichText::new(format!("ƒx = {} {unit}", fmt_num(value))).weak()).on_hover_text("Driven by a formula. Type a plain number to remove the formula.");
            }
            let commit = resp.lost_focus() && !ui.input(|i| i.key_pressed(egui::Key::Escape));
            if commit {
                let text = buf.trim().to_string();
                changed = self.apply_field(&canon, &text);
                self.fields.remove(&canon);
            }
        });
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
        let before = self.doc.to_json();
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
                self.undo.push(before);
                self.redo.clear();
                self.after_change();
                true
            }
            Err(e) => {
                self.status = format!("{e}");
                false
            }
        }
    }

    fn properties(&mut self, ui: &mut egui::Ui) {
        ui.heading("Properties");
        let Some(&id) = self.tree_sel.last() else {
            if self.selection.is_empty() {
                ui.weak("Select a feature in the tree.");
            }
            for p in &self.selection {
                ui.label(pick_name(&self.vp, p));
            }
            return;
        };
        let Some(f) = self.doc.get(id).cloned() else { return };
        let name = f.name.clone();
        egui::Grid::new("props_head").num_columns(2).show(ui, |ui| {
            ui.label("Type");
            ui.label(f.kind.type_name());
            ui.end_row();
            ui.label("Label");
            let mut label = f.label.clone();
            if ui.text_edit_singleline(&mut label).lost_focus() && label != f.label && !label.trim().is_empty() {
                self.change(|d| d.get_mut(id).unwrap().label = label.trim().to_string());
            }
            ui.end_row();
        });
        if let Some(e) = self.doc.state(id).and_then(|s| s.error.as_ref()) {
            ui.colored_label(egui::Color32::from_rgb(230, 80, 60), format!("⚠ {e}"));
        }
        let sketch_name = |doc: &Document, s: FeatureId| doc.get(s).map(|f| f.label.clone()).unwrap_or("?".into());
        match f.kind.clone() {
            FeatureKind::Sketch(sf) => {
                ui.label(format!(
                    "Attached to: {}",
                    match &sf.support {
                        Support::XY => "XY plane".to_string(),
                        Support::XZ => "XZ plane".to_string(),
                        Support::YZ => "YZ plane".to_string(),
                        Support::Face(fr) => format!("{}.Face{}", sketch_name(&self.doc, fr.feature), fr.index),
                    }
                ));
                self.value_field(ui, "Attachment offset", &format!("{name}.AttachmentOffset"), "mm");
                let mut rev = sf.reversed;
                if ui.checkbox(&mut rev, "Reverse side").changed() {
                    self.change(|d| {
                        if let FeatureKind::Sketch(s) = &mut d.get_mut(id).unwrap().kind {
                            s.reversed = rev;
                        }
                    });
                }
                if let Some(rep) = self.doc.state(id).and_then(|s| s.solve.clone()) {
                    ui.label(format!("Degrees of freedom: {}", rep.dof));
                }
                if ui.button("Edit sketch").clicked() {
                    self.start_edit(id);
                }
            }
            FeatureKind::Pad(e) | FeatureKind::Pocket(e) => {
                let pocket = matches!(f.kind, FeatureKind::Pocket(_));
                ui.label(format!("Profile: {}", sketch_name(&self.doc, e.profile)));
                let mut ne = e.clone();
                egui::ComboBox::from_label("Side").selected_text(format!("{:?}", e.side)).show_ui(ui, |ui| {
                    ui.selectable_value(&mut ne.side, SideType::OneSide, "One side");
                    ui.selectable_value(&mut ne.side, SideType::TwoSides, "Two sides");
                    ui.selectable_value(&mut ne.side, SideType::Symmetric, "Symmetric");
                });
                egui::ComboBox::from_label("Type").selected_text(format!("{:?}", e.kind)).show_ui(ui, |ui| {
                    ui.selectable_value(&mut ne.kind, ExtentType::Length, "Dimension");
                    ui.selectable_value(&mut ne.kind, ExtentType::ThroughAll, "Through all");
                    ui.selectable_value(&mut ne.kind, ExtentType::UpToFirst, "Up to first");
                });
                if e.kind == ExtentType::Length {
                    self.value_field(ui, "Length", &format!("{name}.Length"), "mm");
                }
                if e.side == SideType::TwoSides {
                    egui::ComboBox::from_label("Type 2").selected_text(format!("{:?}", e.kind2)).show_ui(ui, |ui| {
                        ui.selectable_value(&mut ne.kind2, ExtentType::Length, "Dimension");
                        ui.selectable_value(&mut ne.kind2, ExtentType::ThroughAll, "Through all");
                        ui.selectable_value(&mut ne.kind2, ExtentType::UpToFirst, "Up to first");
                    });
                    if e.kind2 == ExtentType::Length {
                        self.value_field(ui, "Length 2", &format!("{name}.Length2"), "mm");
                    }
                }
                ui.checkbox(&mut ne.reversed, "Reversed");
                if ne.side != e.side || ne.kind != e.kind || ne.kind2 != e.kind2 || ne.reversed != e.reversed {
                    self.change(|d| {
                        if let Some(FeatureKind::Pad(x) | FeatureKind::Pocket(x)) = d.get_mut(id).map(|f| &mut f.kind) {
                            x.side = ne.side;
                            x.kind = ne.kind;
                            x.kind2 = ne.kind2;
                            x.reversed = ne.reversed;
                        }
                    });
                }
                let _ = pocket;
            }
            FeatureKind::Revolution(r) | FeatureKind::Groove(r) => {
                ui.label(format!("Profile: {}", sketch_name(&self.doc, r.profile)));
                let mut nr = r.clone();
                let axis_name = |a: &AxisRef| match a {
                    AxisRef::SketchV => "Vertical sketch axis".to_string(),
                    AxisRef::SketchH => "Horizontal sketch axis".to_string(),
                    AxisRef::SketchLine(g) => format!("Sketch line {}", g + 1),
                    AxisRef::X => "X axis".to_string(),
                    AxisRef::Y => "Y axis".to_string(),
                    AxisRef::Z => "Z axis".to_string(),
                };
                let lines: Vec<usize> = match self.doc.get(r.profile).map(|f| &f.kind) {
                    Some(FeatureKind::Sketch(s)) => s.sketch.geometry.iter().enumerate().filter(|(_, g)| g.construction && matches!(g.curve, fc_sketch::Curve::Line { .. })).map(|(i, _)| i).collect(),
                    _ => vec![],
                };
                egui::ComboBox::from_label("Axis").selected_text(axis_name(&r.axis)).show_ui(ui, |ui| {
                    for a in [AxisRef::SketchV, AxisRef::SketchH, AxisRef::X, AxisRef::Y, AxisRef::Z].into_iter().chain(lines.into_iter().map(AxisRef::SketchLine)) {
                        let n = axis_name(&a);
                        ui.selectable_value(&mut nr.axis, a, n);
                    }
                });
                egui::ComboBox::from_label("Side").selected_text(format!("{:?}", r.side)).show_ui(ui, |ui| {
                    ui.selectable_value(&mut nr.side, SideType::OneSide, "One side");
                    ui.selectable_value(&mut nr.side, SideType::TwoSides, "Two sides");
                    ui.selectable_value(&mut nr.side, SideType::Symmetric, "Symmetric");
                });
                self.value_field(ui, "Angle", &format!("{name}.Angle"), "°");
                if r.side == SideType::TwoSides {
                    self.value_field(ui, "Angle 2", &format!("{name}.Angle2"), "°");
                }
                ui.checkbox(&mut nr.reversed, "Reversed");
                if nr.axis != r.axis || nr.side != r.side || nr.reversed != r.reversed {
                    self.change(|d| {
                        if let Some(FeatureKind::Revolution(x) | FeatureKind::Groove(x)) = d.get_mut(id).map(|f| &mut f.kind) {
                            x.axis = nr.axis.clone();
                            x.side = nr.side;
                            x.reversed = nr.reversed;
                        }
                    });
                }
            }
            FeatureKind::Hole(h) => {
                ui.label(format!("Positions from: {}", sketch_name(&self.doc, h.profile)));
                self.value_field(ui, "Diameter", &format!("{name}.Diameter"), "mm");
                let mut nh = h.clone();
                ui.checkbox(&mut nh.through_all, "Through all");
                if !h.through_all {
                    self.value_field(ui, "Depth", &format!("{name}.Depth"), "mm");
                    self.value_field(ui, "Drill point angle (0 = flat)", &format!("{name}.DrillPointAngle"), "°");
                }
                egui::ComboBox::from_label("Head").selected_text(format!("{:?}", h.cut)).show_ui(ui, |ui| {
                    ui.selectable_value(&mut nh.cut, HoleCut::None, "None");
                    ui.selectable_value(&mut nh.cut, HoleCut::Counterbore, "Counterbore");
                    ui.selectable_value(&mut nh.cut, HoleCut::Countersink, "Countersink");
                });
                if h.cut != HoleCut::None {
                    self.value_field(ui, "Head diameter", &format!("{name}.CutDiameter"), "mm");
                    if h.cut == HoleCut::Counterbore {
                        self.value_field(ui, "Head depth", &format!("{name}.CutDepth"), "mm");
                    } else {
                        self.value_field(ui, "Countersink angle", &format!("{name}.CountersinkAngle"), "°");
                    }
                }
                if nh.through_all != h.through_all || nh.cut != h.cut {
                    self.change(|d| {
                        if let Some(FeatureKind::Hole(x)) = d.get_mut(id).map(|f| &mut f.kind) {
                            x.through_all = nh.through_all;
                            x.cut = nh.cut;
                        }
                    });
                }
            }
            FeatureKind::AdditiveLoft(l) | FeatureKind::SubtractiveLoft(l) => {
                ui.label(format!("Sections: {}", std::iter::once(l.profile).chain(l.sections.iter().copied()).map(|s| sketch_name(&self.doc, s)).collect::<Vec<_>>().join(" → ")));
                let mut ruled = l.ruled;
                if ui.checkbox(&mut ruled, "Ruled").changed() {
                    self.change(|d| {
                        if let Some(FeatureKind::AdditiveLoft(x) | FeatureKind::SubtractiveLoft(x)) = d.get_mut(id).map(|f| &mut f.kind) {
                            x.ruled = ruled;
                        }
                    });
                }
            }
            FeatureKind::AdditivePipe(p) | FeatureKind::SubtractivePipe(p) => {
                ui.label(format!("Profile: {}", sketch_name(&self.doc, p.profile)));
                ui.label(format!("Path: {}", sketch_name(&self.doc, p.spine)));
                let mut fr = p.frenet;
                if ui.checkbox(&mut fr, "Frenet").changed() {
                    self.change(|d| {
                        if let Some(FeatureKind::AdditivePipe(x) | FeatureKind::SubtractivePipe(x)) = d.get_mut(id).map(|f| &mut f.kind) {
                            x.frenet = fr;
                        }
                    });
                }
            }
        }
        ui.separator();
        ui.horizontal(|ui| {
            let mut sup = f.suppressed;
            if ui.checkbox(&mut sup, "Suppressed").changed() {
                self.change(|d| d.get_mut(id).unwrap().suppressed = sup);
            }
            if f.kind.is_solid_feature() {
                let is_tip = self.doc.tip == Some(id);
                if ui.selectable_label(is_tip, "Tip").on_hover_text("Stop the body after this feature").clicked() {
                    self.change(|d| d.tip = if is_tip { None } else { Some(id) });
                }
            }
            if ui.button("🗑 Delete").clicked() {
                self.try_delete(DeleteWhat::Feature(id), OnUsers::Refuse);
            }
        });
    }

    fn variables(&mut self, ui: &mut egui::Ui) {
        ui.heading("Variables");
        let params: Vec<(String, String)> = self.doc.parameters.iter().map(|(k, v)| (k.clone(), v.clone())).collect();
        let (vals, errs) = self.doc.evaluate_expressions();
        let mut delete: Option<String> = None;
        let mut edit: Option<(String, String)> = None;
        let mut release: Option<String> = None;
        egui::Grid::new("vars").num_columns(4).striped(true).show(ui, |ui| {
            for (k, v) in &params {
                ui.label(egui::RichText::new(k).monospace());
                let key = format!("param:{k}");
                let driven = self.doc.driven.get(k).cloned();
                if driven.is_some() {
                    ui.add_enabled(false, egui::Label::new(egui::RichText::new(format!("🔒 {v}")).monospace()));
                } else {
                    let buf = self.fields.entry(key.clone()).or_insert_with(|| v.clone());
                    if ui.add(egui::TextEdit::singleline(buf).desired_width(110.0)).lost_focus() && *buf != *v {
                        edit = Some((k.clone(), buf.clone()));
                    }
                }
                match (vals.get(&key), errs.get(&key)) {
                    (_, Some(e)) => {
                        ui.colored_label(egui::Color32::from_rgb(230, 80, 60), "⚠").on_hover_text(e.to_string());
                    }
                    (Some(x), _) => {
                        ui.weak(fmt_num(*x));
                    }
                    _ => {
                        ui.label("");
                    }
                }
                if let Some(src) = &driven {
                    if ui.small_button("Release").on_hover_text(format!("Driven by {src}. Release to edit it here.")).clicked() {
                        release = Some(k.clone());
                    }
                } else if ui.small_button("🗑").on_hover_text("Delete variable").clicked() {
                    delete = Some(k.clone());
                }
                ui.end_row();
            }
        });
        for (k, src) in &self.doc.driven {
            if self.doc.parameters.contains_key(k) {
                ui.weak(format!("🔒 {k}: Driven by {src}"));
            }
        }
        if let Some(k) = release {
            self.change(|d| {
                d.release_driven(&k);
            });
        }
        ui.horizontal(|ui| {
            ui.add(egui::TextEdit::singleline(&mut self.new_param.0).hint_text("name").desired_width(70.0));
            ui.add(egui::TextEdit::singleline(&mut self.new_param.1).hint_text("value or formula").desired_width(100.0));
            if ui.button("Add").clicked() && !self.new_param.0.trim().is_empty() {
                let (n, v) = (self.new_param.0.trim().to_string(), self.new_param.1.trim().to_string());
                let v = if v.is_empty() { "0".to_string() } else { v };
                let before = self.doc.to_json();
                match self.doc.set_parameter(&n, &v) {
                    Ok(()) => {
                        self.undo.push(before);
                        self.new_param = (String::new(), String::new());
                        self.after_change();
                    }
                    Err(e) => self.status = e.to_string(),
                }
            }
        });
        if let Some((k, v)) = edit {
            let before = self.doc.to_json();
            match self.doc.set_parameter(&k, &v) {
                Ok(()) => {
                    self.undo.push(before);
                    self.after_change();
                }
                Err(e) => {
                    self.status = e.to_string();
                    self.fields.remove(&format!("param:{k}"));
                }
            }
        }
        if let Some(k) = delete {
            self.try_delete(DeleteWhat::Param(k), OnUsers::Refuse);
        }
        let named: Vec<(String, f64)> = self.doc.available_names().into_iter().filter(|(n, _)| n.contains('.') && !n.ends_with("Length") && !n.ends_with("Length2") && !n.contains("Angle") && !n.ends_with("Offset") && !n.contains("Diameter") && !n.contains("Depth")).collect();
        if !named.is_empty() {
            ui.add_space(4.0);
            ui.weak("Named dimensions");
            for (n, v) in named {
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new(&n).monospace());
                    ui.weak(fmt_num(v));
                });
            }
        }
    }

    fn measurements(&mut self, ui: &mut egui::Ui) {
        ui.heading("Measurements");
        let items = self.doc.measure_all();
        let red = egui::Color32::from_rgb(230, 80, 60);
        let mut delete: Option<String> = None;
        let mut rename: Option<(String, String)> = None;
        if items.is_empty() {
            ui.weak("None yet. Add one below.");
        }
        egui::Grid::new("meas").num_columns(3).striped(true).show(ui, |ui| {
            for (name, res) in &items {
                let key = format!("meas:{name}");
                let buf = self.fields.entry(key).or_insert_with(|| name.clone());
                let resp = ui.add_sized([100.0, 20.0], egui::TextEdit::singleline(buf).font(egui::TextStyle::Monospace));
                if resp.lost_focus() && *buf != *name {
                    rename = Some((name.clone(), buf.trim().to_string()));
                }
                match res {
                    Ok(m) => {
                        let unit = dim_unit(m.dim);
                        ui.label(format!("{} {unit}", fmt_num(m.value)));
                    }
                    Err(e) => {
                        ui.colored_label(red, format!("⚠ {e}")).on_hover_text(e.to_string());
                    }
                }
                if ui.small_button("🗑").on_hover_text("Delete measurement").clicked() {
                    delete = Some(name.clone());
                }
                ui.end_row();
            }
        });
        if let Some((from, to)) = rename {
            let before = self.doc.to_json();
            match self.doc.rename_measurement(&from, &to) {
                Ok(()) => {
                    self.undo.push(before);
                    self.redo.clear();
                    self.dirty = true;
                }
                Err(e) => self.status = e,
            }
            self.fields.retain(|k, _| !k.starts_with("meas:"));
        }
        if let Some(n) = delete {
            self.change(|d| {
                d.remove_measurement(&n);
            });
            self.fields.retain(|k, _| !k.starts_with("meas:"));
        }

        ui.add_space(4.0);
        ui.weak("Add measurement");
        egui::ComboBox::from_id_salt("meas_kind").selected_text(self.meas_kind.label()).width(210.0).show_ui(ui, |ui| {
            for k in MeasKind::ALL {
                ui.selectable_value(&mut self.meas_kind, k, k.label());
            }
        });
        if self.meas_kind == MeasKind::Value {
            let names = self.doc.available_names();
            if !names.iter().any(|(n, _)| *n == self.meas_value_key) {
                self.meas_value_key = names.first().map(|(n, _)| n.clone()).unwrap_or_default();
            }
            egui::ComboBox::from_id_salt("meas_value").selected_text(self.meas_value_key.clone()).width(210.0).show_ui(ui, |ui| {
                for (n, v) in &names {
                    ui.selectable_value(&mut self.meas_value_key, n.clone(), format!("{n}  ({})", fmt_num(*v)));
                }
            });
        }
        let need = self.meas_kind.faces_needed();
        let faces: Vec<u32> = self.selection.iter().filter(|p| p.kind == PickKind::Face).map(|p| p.id + 1).collect();
        if need > 0 {
            let hint = if faces.len() == need { format!("Using selected face(s): {}", faces.iter().map(|f| format!("Face{f}")).collect::<Vec<_>>().join(", ")) } else { format!("Select {need} face(s) in the 3D view (ctrl-click for the second).") };
            ui.weak(hint);
        }
        let ready = need == 0 || faces.len() == need;
        if ui.add_enabled(ready, egui::Button::new("Add")).clicked() {
            self.add_measurement(&faces);
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
            MeasKind::Volume => (MeasureSpec::Volume, "Volume".to_string()),
            MeasKind::Area => (MeasureSpec::Area, "Area".to_string()),
            MeasKind::BBoxX => (MeasureSpec::BBoxX, "BBoxX".to_string()),
            MeasKind::BBoxY => (MeasureSpec::BBoxY, "BBoxY".to_string()),
            MeasKind::BBoxZ => (MeasureSpec::BBoxZ, "BBoxZ".to_string()),
            MeasKind::Value => {
                if self.meas_value_key.is_empty() {
                    return;
                }
                let b: String = self.meas_value_key.chars().map(|c| if c.is_ascii_alphanumeric() { c } else { '_' }).collect();
                (MeasureSpec::Value { key: self.meas_value_key.clone() }, b)
            }
            MeasKind::FaceArea => (MeasureSpec::FaceArea { face: refs[0].clone() }, format!("FaceArea{}", faces[0])),
            MeasKind::FaceDiameter => (MeasureSpec::FaceDiameter { face: refs[0].clone() }, format!("FaceDiameter{}", faces[0])),
            MeasKind::FaceDistance => (MeasureSpec::FaceDistance { a: refs[0].clone(), b: refs[1].clone() }, format!("Distance{}_{}", faces[0], faces[1])),
        };
        let mut name = base.clone();
        let mut n = 2;
        while self.doc.measurements.contains_key(&name) || self.doc.parameters.contains_key(&name) {
            name = format!("{base}{n}");
            n += 1;
        }
        let before = self.doc.to_json();
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
        let mut keep = true;
        match &d {
            Dialog::ConfirmDelete { what, users } => {
                let what = what.clone();
                egui::Window::new("Delete").collapsible(false).resizable(false).anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0]).show(ctx, |ui| {
                    ui.label("These formulas use it:");
                    for u in users {
                        ui.label(egui::RichText::new(format!("• {u}")).monospace());
                    }
                    ui.label("Delete it anyway and keep their current values as plain numbers?");
                    ui.horizontal(|ui| {
                        if ui.button("Delete and keep values").clicked() {
                            keep = false;
                            self.try_delete(what.clone(), OnUsers::Freeze);
                        }
                        if ui.button("Cancel").clicked() {
                            keep = false;
                        }
                    });
                });
            }
            Dialog::Message(m) => {
                egui::Window::new("freecad-rs").collapsible(false).resizable(false).anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0]).show(ctx, |ui| {
                    ui.label(m);
                    if ui.button("OK").clicked() {
                        keep = false;
                    }
                });
            }
        }
        if keep && self.dialog.is_none() {
            self.dialog = Some(d);
        }
    }

    fn shortcuts(&mut self, ctx: &egui::Context) {
        if ctx.egui_wants_keyboard_input() {
            return;
        }
        let (undo, redo, save, del) = ctx.input(|i| {
            let cmd = i.modifiers.command;
            (
                cmd && !i.modifiers.shift && i.key_pressed(egui::Key::Z),
                cmd && i.modifiers.shift && i.key_pressed(egui::Key::Z),
                cmd && i.key_pressed(egui::Key::S),
                i.key_pressed(egui::Key::Delete) || i.key_pressed(egui::Key::Backspace),
            )
        });
        if self.editor.is_some() {
            return;
        }
        if undo {
            self.undo();
        }
        if redo {
            self.redo();
        }
        if save {
            self.save(false);
        }
        if del && !self.tree_sel.is_empty() {
            self.delete_selected();
        }
    }
}

fn fmt_num(v: f64) -> String {
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
            Step::Down(b) => raw.events.push(run.button_event(b, true)),
            Step::Up(b) => raw.events.push(run.button_event(b, false)),
            Step::Click(b) => {
                raw.events.push(run.button_event(b, true));
                run.pending.push(run.button_event(b, false));
                run.wait = 1;
            }
            Step::DoubleClick => {
                let b = egui::PointerButton::Primary;
                raw.events.push(run.button_event(b, true));
                raw.events.push(run.button_event(b, false));
                raw.events.push(run.button_event(b, true));
                run.pending.push(run.button_event(b, false));
                run.wait = 1;
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
                    "dof" => self.editor.as_ref().and_then(|e| self.doc.get(e.feature())).and_then(|f| match &f.kind {
                        FeatureKind::Sketch(s) => Some(s.sketch.diagnose().dof.to_string()),
                        _ => None,
                    }),
                    "editing" => Some(self.editor.is_some().to_string()),
                    "features" => Some(self.doc.features.len().to_string()),
                    "measure" | "param" => {
                        let (n, want) = val.split_once(' ').unwrap_or((&val, ""));
                        let got = if what == "measure" { self.doc.measure(n).ok().map(|m| m.value) } else { self.doc.evaluate_expressions().0.get(&format!("param:{n}")).copied() };
                        let ok = got.zip(want.parse::<f64>().ok()).is_some_and(|(a, b)| (a - b).abs() <= 1e-3 * b.abs().max(1.0));
                        Some(if ok { val.clone() } else { format!("{got:?}") })
                    }
                    "banner" => Some(self.disk_changed.to_string()),
                    "dirty" => Some(self.dirty.to_string()),
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
                "demo" => {
                    self.doc = crate::demo::demo_document();
                    self.path = None;
                    self.after_change();
                    self.dirty = false;
                }
                "saveto" => {
                    let p = PathBuf::from(&arg);
                    if std::fs::write(&p, self.doc.to_json()).is_ok() {
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
}

impl eframe::App for App {
    fn raw_input_hook(&mut self, ctx: &egui::Context, raw_input: &mut egui::RawInput) {
        self.drive_input(ctx, raw_input);
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        let ctx = &ctx;
        if self.first_frame {
            self.first_frame = false;
            if self.editor.is_none() {
                self.vp.fit_all();
            }
        }
        self.shortcuts(ctx);
        self.poll_disk(ctx);
        egui::Panel::top("toolbar").show(ui, |ui| self.toolbar(ui));
        if self.disk_changed {
            self.disk_banner(ui);
        }

        egui::Panel::bottom("status").show(ui, |ui| {
            ui.horizontal(|ui| {
                if let Some(ed) = &self.editor {
                    ui.label(ed.status());
                } else if let Some(h) = &self.hovered {
                    ui.label(format!("Preselected: {} ({:.2}, {:.2}, {:.2})", pick_name(&self.vp, h), h.point[0], h.point[1], h.point[2]));
                } else {
                    ui.label(&self.status);
                }
            });
        });

        egui::Panel::left("model").default_size(280.0).show(ui, |ui| {
            if self.editor.is_some() {
                let mut ed = self.editor.take().unwrap();
                let exit = egui::ScrollArea::vertical().show(ui, |ui| ed.panel_ui(&mut self.doc, ui)).inner;
                self.editor = Some(ed);
                if exit != EditorExit::None {
                    self.finish_edit(exit);
                }
                return;
            }
            if self.picking_plane.is_some() {
                self.plane_pick_panel(ui);
                return;
            }
            self.tree(ui);
            ui.separator();
            egui::ScrollArea::vertical().id_salt("props").show(ui, |ui| {
                self.properties(ui);
                ui.separator();
                self.variables(ui);
                ui.separator();
                self.measurements(ui);
            });
        });

        egui::CentralPanel::default().frame(egui::Frame::NONE).show(ui, |ui| {
            let r = self.vp.ui(ui);
            self.hovered = r.hovered.clone();
            if let Some(mut ed) = self.editor.take() {
                ed.handle_viewport(&mut self.doc, &mut self.vp, &r, ctx);
                self.editor = Some(ed);
                return;
            }
            if self.picking_plane.is_some() {
                if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
                    self.end_plane_pick();
                    self.status = "Ready".into();
                } else if let Some(p) = r.clicked.clone() {
                    self.pick_support(&p);
                }
                self.draw_plane_labels(ui, ctx);
                return;
            }
            if r.response.clicked() {
                let multi = ctx.input(|i| i.modifiers.ctrl || i.modifiers.command);
                match r.clicked {
                    Some(p) => {
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
                        }
                    }
                }
                self.vp.set_selection(self.selection.clone());
                if let Some(p) = self.selection.last() {
                    self.status = format!("Selected: {}", pick_name(&self.vp, p));
                }
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
            if *frames == 6 {
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
