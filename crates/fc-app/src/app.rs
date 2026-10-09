//! Application shell: Part Design document, model tree, property panel, variables,
//! file I/O and hosting of the sketch editor.

use crate::scene;
use crate::sketcher::{EditorExit, SketchEditor};
use crate::viewport::{Pick, PickKind, StandardView, Viewport};
use eframe::egui;
use fc_part::params::OnUsers;
use fc_part::*;
use std::collections::HashMap;
use std::path::PathBuf;

/// Something waiting on the user in a modal window.
enum Dialog {
    NewSketch,
    ConfirmDelete { what: DeleteWhat, users: Vec<String> },
    Message(String),
}

#[derive(Clone)]
enum DeleteWhat {
    Feature(FeatureId),
    Param(String),
}

pub struct App {
    doc: Document,
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
        let objs = scene::body_objects(&self.doc);
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
        self.dialog = Some(Dialog::NewSketch);
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

    fn save(&mut self, as_new: bool) {
        let path = match (&self.path, as_new) {
            (Some(p), false) => Some(p.clone()),
            _ => rfd::FileDialog::new().add_filter("freecad-rs document", &["fcrs"]).set_file_name("Unnamed.fcrs").save_file(),
        };
        if let Some(p) = path {
            match std::fs::write(&p, self.doc.to_json()) {
                Ok(()) => {
                    self.status = format!("Saved {}", p.display());
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
        egui::Grid::new("vars").num_columns(4).striped(true).show(ui, |ui| {
            for (k, v) in &params {
                ui.label(egui::RichText::new(k).monospace());
                let key = format!("param:{k}");
                let buf = self.fields.entry(key.clone()).or_insert_with(|| v.clone());
                if ui.add(egui::TextEdit::singleline(buf).desired_width(110.0)).lost_focus() && *buf != *v {
                    edit = Some((k.clone(), buf.clone()));
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
                if ui.small_button("🗑").on_hover_text("Delete variable").clicked() {
                    delete = Some(k.clone());
                }
                ui.end_row();
            }
        });
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

    fn dialogs(&mut self, ctx: &egui::Context) {
        let Some(d) = self.dialog.take() else { return };
        let mut keep = true;
        match &d {
            Dialog::NewSketch => {
                egui::Window::new("New sketch").collapsible(false).resizable(false).anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0]).show(ctx, |ui| {
                    ui.label("Attach the sketch to a base plane (or cancel, select a flat face and try again):");
                    for (name, s) in [("XY plane", Support::XY), ("XZ plane", Support::XZ), ("YZ plane", Support::YZ)] {
                        if ui.button(name).clicked() {
                            keep = false;
                            self.create_sketch(s);
                        }
                    }
                    if ui.button("Cancel").clicked() {
                        keep = false;
                    }
                });
            }
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
        egui::Panel::top("toolbar").show(ui, |ui| self.toolbar(ui));

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
            self.tree(ui);
            ui.separator();
            egui::ScrollArea::vertical().id_salt("props").show(ui, |ui| {
                self.properties(ui);
                ui.separator();
                self.variables(ui);
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
