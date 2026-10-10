//! Menu bar, toolbars, status bar and keyboard shortcuts.

use super::*;
use crate::theme;
use crate::widgets::{tb_grip, tb_separator, tool_button, Action};
use egui::{Align2, Color32, RichText};

/// Every command reachable from the menus, toolbars and shortcuts.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Act {
    New,
    Open,
    Save,
    SaveAs,
    ExportStep,
    ExportStl,
    Undo,
    Redo,
    Delete,
    FitAll,
    View(StandardView),
    Ortho,
    Persp,
    ToggleOrigin,
    NewSketch,
    Pad,
    Pocket,
    Revolution,
    Groove,
    Hole,
    AddLoft,
    SubLoft,
    AddPipe,
    SubPipe,
    Edit,
    LeaveSketch,
    ViewSketch,
    About,
}

const VIEWS: [(StandardView, &str, &str, &str, &str); 7] = [
    (StandardView::Isometric, "view-isometric", "Isometric", "Sets the camera to the isometric view", "0"),
    (StandardView::Front, "view-front", "Front", "Sets the camera to the front view", "1"),
    (StandardView::Top, "view-top", "Top", "Sets the camera to the top view", "2"),
    (StandardView::Right, "view-right", "Right", "Sets the camera to the right view", "3"),
    (StandardView::Rear, "view-rear", "Rear", "Sets the camera to the rear view", "4"),
    (StandardView::Bottom, "view-bottom", "Bottom", "Sets the camera to the bottom view", "5"),
    (StandardView::Left, "view-left", "Left", "Sets the camera to the left view", "6"),
];

fn cmd_key(k: &str) -> String {
    if cfg!(target_os = "macos") { k.replace("Ctrl+", "⌘").replace("Shift+", "⇧") } else { k.to_string() }
}

impl App {
    pub(super) fn run(&mut self, a: Act) {
        match a {
            Act::New => self.new_document(),
            Act::Open => self.open_dialog(),
            Act::Save => self.save(false),
            Act::SaveAs => self.save(true),
            Act::ExportStep => self.export(false),
            Act::ExportStl => self.export(true),
            Act::Undo => self.undo(),
            Act::Redo => self.redo(),
            Act::Delete => {
                if self.editor.is_none() && self.task.is_none() {
                    self.delete_selected();
                }
            }
            Act::FitAll => {
                if let Some(ed) = self.editor.as_mut() {
                    ed.fit();
                } else {
                    self.vp.animate_fit();
                }
            }
            Act::View(v) => self.vp.animate_standard_view(v),
            Act::Ortho => {
                self.vp.set_perspective(false);
                self.vp.fit_all();
            }
            Act::Persp => {
                self.vp.set_perspective(true);
                self.vp.fit_all();
            }
            Act::ToggleOrigin => {
                self.show_origin = !self.show_origin;
                self.rebuild_scene();
            }
            Act::NewSketch => self.new_sketch(),
            Act::Pad => self.add_feature(|s| FeatureKind::Pad(Extrude::new(s, 10.0))),
            Act::Pocket => self.add_feature(|s| FeatureKind::Pocket(Extrude::new(s, 5.0))),
            Act::Revolution | Act::Groove => {
                let axis = self.origin_sel.filter(|i| *i < 3).map(|i| [AxisRef::X, AxisRef::Y, AxisRef::Z][i].clone());
                let groove = a == Act::Groove;
                self.add_feature(move |s| {
                    let mut r = Revolve::new(s);
                    if let Some(ax) = axis {
                        r.axis = ax;
                    }
                    if groove { FeatureKind::Groove(r) } else { FeatureKind::Revolution(r) }
                })
            }
            Act::Hole => self.add_feature(|s| FeatureKind::Hole(Hole::new(s, 6.0, 10.0))),
            Act::AddLoft => self.add_loft_or_pipe(false, false),
            Act::SubLoft => self.add_loft_or_pipe(true, false),
            Act::AddPipe => self.add_loft_or_pipe(false, true),
            Act::SubPipe => self.add_loft_or_pipe(true, true),
            Act::Edit => {
                if let Some(&id) = self.tree_sel.last() {
                    self.edit_feature(id);
                }
            }
            Act::LeaveSketch => self.finish_edit(EditorExit::Close),
            Act::ViewSketch => {
                if let Some(ed) = self.editor.as_ref() {
                    let pl = ed.placement();
                    self.vp.align_to_plane(&scene::plane(&pl));
                }
                if let Some(ed) = self.editor.as_mut() {
                    ed.fit();
                }
            }
            Act::About => self.dialog = Some(Dialog::About),
        }
    }

    /// (icon, title, description, shortcut, enabled) of an action.
    pub(super) fn act_info(&self, a: Act) -> (&'static str, &'static str, &'static str, String, bool) {
        let idle = self.idle();
        // like FreeCAD, feature commands stay enabled and explain what to select when needed
        let has_profile = idle;
        let two_sketches = idle;
        let k = |s: &str| cmd_key(s);
        match a {
            Act::New => ("document-new", "New", "Creates a new empty document", k("Ctrl+N"), idle),
            Act::Open => ("document-open", "Open...", "Opens a freecad-rs document or imports a FreeCAD .FCStd file", k("Ctrl+O"), idle),
            Act::Save => ("document-save", "Save", "Saves the active document", k("Ctrl+S"), idle),
            Act::SaveAs => ("document-save-as", "Save As...", "Saves the active document under a new file name", k("Ctrl+Shift+S"), idle),
            Act::ExportStep => ("Std_Export", "Export STEP...", "Exports the body as a STEP file", String::new(), idle && self.doc.result().is_some()),
            Act::ExportStl => ("Std_Export", "Export STL...", "Exports the body as an STL mesh", String::new(), idle && self.doc.result().is_some()),
            Act::Undo => ("edit-undo", "Undo", "Undoes the last change", k("Ctrl+Z"), self.can_undo()),
            Act::Redo => ("edit-redo", "Redo", "Redoes a previously undone change", k("Ctrl+Shift+Z"), self.can_redo()),
            Act::Delete => ("edit-delete", "Delete", "Deletes the selected feature", "Del".into(), idle && !self.tree_sel.is_empty()),
            Act::FitAll => ("zoom-all", "Fit All", "Fits the whole content on the screen", "V, F".into(), true),
            Act::View(v) => {
                let (_, icon, title, desc, key) = VIEWS.iter().find(|x| x.0 == v).copied().unwrap();
                (icon, title, desc, key.to_string(), true)
            }
            Act::Ortho => ("view-axonometric", "Orthographic View", "Switches to an orthographic projection", "V, O".into(), true),
            Act::Persp => ("view-perspective", "Perspective View", "Switches to a perspective projection", "V, P".into(), true),
            Act::ToggleOrigin => ("Std_CoordinateSystem", "Show Origin", "Shows or hides the body's origin planes in the 3D view", String::new(), self.editor.is_none() && !self.picking),
            Act::NewSketch => ("Sketcher_NewSketch", "New Sketch", "Creates a new sketch on a selected face or plane. Without a selection, choose a base plane in the 3D view.", String::new(), idle),
            Act::Pad => ("PartDesign_Pad", "Pad", "Extrudes the selected sketch (or the newest unused one)", String::new(), has_profile),
            Act::Pocket => ("PartDesign_Pocket", "Pocket", "Creates a pocket with the selected sketch", String::new(), has_profile),
            Act::Revolution => ("PartDesign_Revolution", "Revolution", "Revolves the selected sketch about an axis (select an origin axis first to use it)", String::new(), has_profile),
            Act::Groove => ("PartDesign_Groove", "Groove", "Cuts by revolving the selected sketch about an axis", String::new(), has_profile),
            Act::Hole => ("PartDesign_Hole", "Hole", "Creates holes at the circle centers of the selected sketch", String::new(), has_profile),
            Act::AddLoft => ("PartDesign_AdditiveLoft", "Additive Loft", "Lofts through two or more sketches (Ctrl/Cmd-click them in order)", String::new(), two_sketches),
            Act::SubLoft => ("PartDesign_SubtractiveLoft", "Subtractive Loft", "Cuts a loft through two or more sketches", String::new(), two_sketches),
            Act::AddPipe => ("PartDesign_AdditivePipe", "Additive Pipe", "Sweeps a profile sketch along a path sketch (select profile, then path)", String::new(), two_sketches),
            Act::SubPipe => ("PartDesign_SubtractivePipe", "Subtractive Pipe", "Cuts by sweeping a profile along a path", String::new(), two_sketches),
            Act::Edit => ("edit-edit", "Edit", "Edits the selected feature or sketch", String::new(), idle && !self.tree_sel.is_empty()),
            Act::LeaveSketch => ("Sketcher_LeaveSketch", "Leave Sketch", "Finishes editing the active sketch", String::new(), self.editor.is_some()),
            Act::ViewSketch => ("Sketcher_ViewSketch", "View Sketch", "Sets the camera orientation perpendicular to the sketch plane and fits it", "Q, P".into(), self.editor.is_some()),
            Act::About => ("help-browser", "About freecad-rs", "", String::new(), true),
        }
    }

    fn tb(&mut self, ui: &mut egui::Ui, id: &str, a: Act, checked: bool) {
        let (icon, title, desc, key, enabled) = self.act_info(a);
        if tool_button(ui, &Action::new(id, icon, title, desc, &key).enabled(enabled).checked(checked)) {
            self.run(a);
        }
    }

    fn menu_item(&mut self, ui: &mut egui::Ui, a: Act) {
        let (icon, title, desc, key, enabled) = self.act_info(a);
        let mut b = egui::Button::image_and_text(crate::icons::image(ui.ctx(), icon, 16.0), title);
        if !key.is_empty() {
            b = b.shortcut_text(key);
        }
        let r = ui.add_enabled(enabled, b);
        let r = if desc.is_empty() { r } else { r.on_hover_text(desc) };
        if r.clicked() {
            self.run(a);
            ui.close();
        }
    }

    pub(super) fn menu_bar(&mut self, ui: &mut egui::Ui) {
        egui::MenuBar::new().ui(ui, |ui| {
            ui.menu_button("File", |ui| {
                ui.set_min_width(220.0);
                for a in [Act::New, Act::Open, Act::Save, Act::SaveAs] {
                    self.menu_item(ui, a);
                }
                ui.separator();
                self.menu_item(ui, Act::ExportStep);
                self.menu_item(ui, Act::ExportStl);
            });
            ui.menu_button("Edit", |ui| {
                ui.set_min_width(200.0);
                self.menu_item(ui, Act::Undo);
                self.menu_item(ui, Act::Redo);
                ui.separator();
                self.menu_item(ui, Act::Edit);
                self.menu_item(ui, Act::Delete);
            });
            ui.menu_button("View", |ui| {
                ui.set_min_width(220.0);
                ui.menu_button("Standard views", |ui| {
                    ui.set_min_width(180.0);
                    for v in VIEWS {
                        self.menu_item(ui, Act::View(v.0));
                    }
                });
                self.menu_item(ui, Act::FitAll);
                ui.separator();
                self.menu_item(ui, Act::Ortho);
                self.menu_item(ui, Act::Persp);
                ui.separator();
                self.menu_item(ui, Act::ToggleOrigin);
            });
            if self.editor.is_some() {
                ui.menu_button("Sketch", |ui| {
                    ui.set_min_width(220.0);
                    self.menu_item(ui, Act::ViewSketch);
                    self.menu_item(ui, Act::LeaveSketch);
                });
            } else {
                ui.menu_button("Part Design", |ui| {
                    ui.set_min_width(220.0);
                    self.menu_item(ui, Act::NewSketch);
                    ui.separator();
                    ui.menu_button("Create an additive feature", |ui| {
                        ui.set_min_width(200.0);
                        for a in [Act::Pad, Act::Revolution, Act::AddLoft, Act::AddPipe] {
                            self.menu_item(ui, a);
                        }
                    });
                    ui.menu_button("Create a subtractive feature", |ui| {
                        ui.set_min_width(200.0);
                        for a in [Act::Pocket, Act::Hole, Act::Groove, Act::SubLoft, Act::SubPipe] {
                            self.menu_item(ui, a);
                        }
                    });
                });
            }
            ui.menu_button("Help", |ui| {
                ui.set_min_width(180.0);
                self.menu_item(ui, Act::About);
            });
        });
    }

    /// Fake workbench selector (FreeCAD's combo box showing the active workbench).
    fn workbench_box(&self, ui: &mut egui::Ui) {
        let (icon, name) = if self.editor.is_some() { ("SketcherWorkbench", "Sketcher") } else { ("PartDesignWorkbench", "Part Design") };
        let (rect, resp) = ui.allocate_exact_size(egui::vec2(170.0, 24.0), egui::Sense::hover());
        let y = rect.center().y;
        let p = ui.painter();
        p.rect(rect.shrink2(egui::vec2(0.0, 0.0)), 4.0, Color32::from_rgb(32, 32, 32), egui::Stroke::new(1.0, theme::BORDER), egui::StrokeKind::Inside);
        crate::icons::paint(p, icon, egui::Rect::from_center_size(egui::pos2(rect.left() + 14.0, y), egui::vec2(16.0, 16.0)), Color32::WHITE);
        p.text(egui::pos2(rect.left() + 28.0, y), Align2::LEFT_CENTER, name, egui::FontId::proportional(13.0), theme::TEXT);
        let c = egui::pos2(rect.right() - 12.0, y);
        p.add(egui::Shape::convex_polygon(vec![c + egui::vec2(-4.0, -2.0), c + egui::vec2(4.0, -2.0), c + egui::vec2(0.0, 2.5)], Color32::from_rgb(150, 150, 150), egui::Stroke::NONE));
        resp.on_hover_ui(|ui| crate::widgets::tooltip(ui, "Workbench", "Switches with the task: Sketcher while a sketch is being edited, Part Design otherwise.", ""));
    }

    pub(super) fn toolbars(&mut self, ui: &mut egui::Ui) {
        ui.spacing_mut().item_spacing = egui::vec2(1.0, 0.0);
        ui.horizontal_wrapped(|ui| {
            tb_grip(ui);
            self.tb(ui, "new", Act::New, false);
            self.tb(ui, "open", Act::Open, false);
            self.tb(ui, "save", Act::Save, false);
            tb_separator(ui);
            tb_grip(ui);
            self.tb(ui, "undo", Act::Undo, false);
            self.tb(ui, "redo", Act::Redo, false);
            tb_separator(ui);
            tb_grip(ui);
            ui.add_space(2.0);
            self.workbench_box(ui);
            ui.add_space(4.0);
            tb_separator(ui);
            tb_grip(ui);
            self.tb(ui, "fit", Act::FitAll, false);
            // standard views: isometric with a dropdown of all views, then the common three
            let mem = egui::Id::new("std_view_choice");
            let cur = ui.data(|d| d.get_temp::<usize>(mem)).unwrap_or(0).min(VIEWS.len() - 1);
            let variants: Vec<Action> = VIEWS.iter().map(|v| Action::new(v.2, v.1, v.2, v.3, v.4)).collect();
            let main = VIEWS[cur];
            let id = format!("view:{:?}", main.0).to_lowercase();
            let (clicked, chosen) = crate::widgets::split_tool_button(ui, &Action::new(&id, main.1, main.2, main.3, main.4), &variants);
            if let Some(i) = chosen {
                ui.data_mut(|d| d.insert_temp(mem, i));
                self.run(Act::View(VIEWS[i].0));
            } else if clicked {
                self.run(Act::View(main.0));
            }
            for v in &VIEWS[1..4] {
                let id = format!("view:{:?}", v.0).to_lowercase();
                self.tb(ui, &id, Act::View(v.0), false);
            }
            tb_separator(ui);
            // projection: one button showing the current one, dropdown to switch
            let persp = self.vp.perspective();
            let (oi, ot, od, ok, _) = self.act_info(Act::Ortho);
            let (pi, pt, pd, pk, _) = self.act_info(Act::Persp);
            let variants = [Action::new("ortho", oi, ot, od, &ok).checked(!persp), Action::new("persp", pi, pt, pd, &pk).checked(persp)];
            let main = if persp { Action::new("projection", pi, "Perspective view (click for orthographic)", pd, &pk) } else { Action::new("projection", oi, "Orthographic view (click for perspective)", od, &ok) };
            let (clicked, chosen) = crate::widgets::split_tool_button(ui, &main, &variants);
            match (clicked, chosen) {
                (_, Some(0)) => self.run(Act::Ortho),
                (_, Some(_)) => self.run(Act::Persp),
                (true, None) => self.run(if persp { Act::Ortho } else { Act::Persp }),
                _ => {}
            }
            tb_separator(ui);
            self.tb(ui, "origin", Act::ToggleOrigin, self.show_origin);
            // export dropdown
            let (_, _, _, _, en) = self.act_info(Act::ExportStep);
            let r = crate::widgets::tool_button_resp(ui, &Action::new("export", "Std_Export", "Export", "Exports the body as STEP or STL", "").enabled(en));
            egui::Popup::menu(&r).show(|ui| {
                ui.set_min_width(170.0);
                self.menu_item(ui, Act::ExportStep);
                self.menu_item(ui, Act::ExportStl);
            });
        });
        ui.add_space(2.0);
        ui.horizontal_wrapped(|ui| {
            if self.editor.is_some() {
                tb_grip(ui);
                self.tb(ui, "leave", Act::LeaveSketch, false);
                self.tb(ui, "viewsketch", Act::ViewSketch, false);
                tb_separator(ui);
                let mut ed = self.editor.take();
                if let Some(e) = ed.as_mut() {
                    e.toolbar_ui(&mut self.doc, ui);
                }
                self.editor = ed;
            } else {
                tb_grip(ui);
                self.tb(ui, "sketch", Act::NewSketch, false);
                tb_separator(ui);
                tb_grip(ui);
                for (id, a) in [("pad", Act::Pad), ("revolution", Act::Revolution), ("addloft", Act::AddLoft), ("addpipe", Act::AddPipe)] {
                    self.tb(ui, id, a, false);
                }
                tb_separator(ui);
                tb_grip(ui);
                for (id, a) in [("pocket", Act::Pocket), ("hole", Act::Hole), ("groove", Act::Groove), ("subloft", Act::SubLoft), ("subpipe", Act::SubPipe)] {
                    self.tb(ui, id, a, false);
                }
                tb_separator(ui);
                tb_grip(ui);
                self.tb(ui, "edit", Act::Edit, false);
                self.tb(ui, "delete", Act::Delete, false);
            }
        });
    }

    /// FreeCAD's document tab under the 3D view ("Unnamed : 1*").
    pub(super) fn doc_tabs(&mut self, ui: &mut egui::Ui) {
        let text = format!("{} : 1{}", self.doc_title(), if self.dirty { "*" } else { "" });
        let g = ui.painter().layout_no_wrap(text.clone(), theme::bold_font(13.0), Color32::WHITE);
        let top = ui.max_rect().top();
        let rect = egui::Rect::from_min_size(egui::pos2(ui.max_rect().left() + 2.0, top), egui::vec2(g.size().x + 40.0, 26.0));
        let resp = ui.interact(rect, egui::Id::new("doc_tab"), egui::Sense::hover());
        let p = ui.painter();
        p.rect_filled(rect, egui::CornerRadius { nw: 0, ne: 0, sw: 4, se: 4 }, Color32::from_rgb(40, 40, 40));
        p.line_segment([rect.left_top(), rect.right_top()], egui::Stroke::new(2.0, theme::ACCENT));
        crate::icons::paint(p, "Document", egui::Rect::from_center_size(egui::pos2(rect.left() + 15.0, rect.center().y), egui::vec2(16.0, 16.0)), Color32::WHITE);
        p.galley(egui::pos2(rect.left() + 30.0, rect.center().y - g.size().y * 0.5), g, Color32::WHITE);
        resp.on_hover_text(self.path.as_ref().map(|p| p.display().to_string()).unwrap_or_else(|| "Not saved yet".into()));
    }

    pub(super) fn status_bar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.set_min_height(18.0);
            let (w, h) = self.vp.view_size();
            let right = format!("{} mm x {} mm", fmt2(w), fmt2(h));
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                // fixed-width segments, like Qt's status bar widgets
                let seg = |ui: &mut egui::Ui, w: f32, add: &mut dyn FnMut(&mut egui::Ui)| {
                    ui.allocate_ui_with_layout(egui::vec2(w, 18.0), egui::Layout::left_to_right(egui::Align::Center), |ui| {
                        ui.set_min_width(w);
                        add(ui)
                    });
                    ui.add(egui::Separator::default().vertical().spacing(12.0));
                };
                seg(ui, 170.0, &mut |ui| {
                    ui.label(RichText::new(&right).color(theme::TEXT)).on_hover_text("Size of the visible area of the 3D view");
                });
                seg(ui, 34.0, &mut |ui| {
                    ui.label(RichText::new("mm").color(theme::TEXT)).on_hover_text("Unit system: Standard (mm, kg, s, degree)");
                });
                seg(ui, 64.0, &mut |ui| {
                    ui.add(crate::icons::image(ui.ctx(), "view-rotate-right", 14.0));
                    ui.label(RichText::new("CAD").color(theme::TEXT)).on_hover_text(
                        "Navigation style: CAD\n• Left click: select\n• Left or right drag: rotate (Shift + drag: pan)\n• Middle drag: pan\n• Wheel / pinch: zoom at the cursor\n• Two-finger scroll: rotate (Shift: pan)\n• Sketch: right drag rotates, middle drag pans",
                    );
                });
                ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                    let text = if let Some(ed) = &self.editor {
                        ed.status().to_string()
                    } else if let Some(h) = &self.hovered {
                        format!("Preselected: {} ({:.2} mm, {:.2} mm, {:.2} mm)", pick_name(&self.vp, h, self.planes_at), h.point[0], h.point[1], h.point[2])
                    } else {
                        self.status.clone()
                    };
                    ui.add(egui::Label::new(RichText::new(text).color(theme::TEXT)).truncate());
                });
            });
        });
    }

    pub(super) fn shortcuts(&mut self, ctx: &egui::Context) {
        if ctx.egui_wants_keyboard_input() {
            return;
        }
        let (keys, mods, now) = ctx.input(|i| {
            let keys: Vec<egui::Key> = i.events.iter().filter_map(|e| match e {
                egui::Event::Key { key, pressed: true, repeat: false, .. } => Some(*key),
                _ => None,
            }).collect();
            (keys, i.modifiers, i.time)
        });
        if keys.is_empty() {
            return;
        }
        use egui::Key;
        for k in keys {
            if mods.command {
                if self.editor.is_some() {
                    // the sketch editor handles its own undo/redo
                    continue;
                }
                match k {
                    Key::Z if mods.shift => self.redo(),
                    Key::Z => self.undo(),
                    Key::Y => self.redo(),
                    Key::S if mods.shift && self.idle() => self.run(Act::SaveAs),
                    Key::S if self.idle() => self.run(Act::Save),
                    Key::N if self.idle() => self.run(Act::New),
                    Key::O if self.idle() => self.run(Act::Open),
                    _ => {}
                }
                continue;
            }
            if self.dialog.is_some() {
                continue;
            }
            // FreeCAD's two-key shortcuts ("V, F")
            if let Some((first, t)) = self.chord.take() {
                if now - t < 1.5 {
                    match (first, k) {
                        (Key::V, Key::F) => self.run(Act::FitAll),
                        (Key::V, Key::O) => self.run(Act::Ortho),
                        (Key::V, Key::P) => self.run(Act::Persp),
                        _ => {}
                    }
                    continue;
                }
            }
            if self.editor.is_some() {
                continue;
            }
            match k {
                Key::V => self.chord = Some((k, now)),
                Key::Escape if self.task.is_some() => self.task_cancel(),
                Key::Escape if self.picking => {
                    self.end_plane_pick();
                    self.status = "Ready".into();
                }
                Key::Enter if self.task.is_some() => self.task_ok(),
                Key::Enter if self.picking => self.plane_pick_ok(),
                Key::Delete | Key::Backspace if self.idle() && !self.tree_sel.is_empty() => self.delete_selected(),
                Key::Num0 => self.run(Act::View(StandardView::Isometric)),
                Key::Num1 => self.run(Act::View(StandardView::Front)),
                Key::Num2 => self.run(Act::View(StandardView::Top)),
                Key::Num3 => self.run(Act::View(StandardView::Right)),
                Key::Num4 => self.run(Act::View(StandardView::Rear)),
                Key::Num5 => self.run(Act::View(StandardView::Bottom)),
                Key::Num6 => self.run(Act::View(StandardView::Left)),
                _ => {}
            }
        }
    }
}

fn fmt2(v: f64) -> String {
    format!("{v:.2}")
}
