//! Menu bar, toolbars, status bar and keyboard shortcuts.

use super::*;
use crate::theme;
use crate::widgets::{tb_grip, tb_separator, tool_button, Action};
use super::fem::{FemCmd, MECH, THERMAL};
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
    ImportStep,
    AsmInterference,
    AsmBom,
    AsmExplode,
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
    MateConnector,
    NewPart,
    NewAssembly,
    AsmInsert,
    Ground,
    Joint(&'static str),
    AsmConnector,
    Solve,
    Workbench(bool),
    FemWorkbench,
    Fem(super::fem::FemCmd),
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
            Act::ExportStep | Act::ExportStl if self.active_assembly().is_some() => self.asm_export_dialog(a == Act::ExportStl),
            Act::ExportStep => self.export(false),
            Act::ExportStl => self.export(true),
            Act::ImportStep => self.import_step_dialog(),
            Act::AsmInterference => self.start_interference(),
            Act::AsmBom => self.start_bom(),
            Act::AsmExplode => self.start_explode(None),
            Act::Undo => self.undo(),
            Act::Redo => self.redo(),
            Act::Delete => {
                if let (Some(it), true) = (self.fem.sel, self.fem.on && self.tree_sel.is_empty()) {
                    self.fem_delete(it);
                } else if self.active_assembly().is_some() {
                    if self.idle() {
                        self.asm_delete_selected();
                    }
                } else if self.editor.is_none() && self.task.is_none() {
                    self.delete_selected();
                }
            }
            Act::MateConnector => self.add_mate_connector(),
            Act::NewPart => self.new_part(),
            Act::NewAssembly => self.new_assembly(),
            Act::AsmInsert => self.asm_insert(),
            Act::Ground => self.toggle_grounded(),
            Act::Joint(name) => self.start_joint(super::assembly::joint_kind(name)),
            Act::AsmConnector => self.start_asm_connector(),
            Act::Solve => {
                self.solve_active(true);
                self.status = self.solver_message().0;
            }
            Act::Workbench(asm) => self.switch_workbench(asm),
            Act::FemWorkbench => self.fem_enter(),
            Act::Fem(c) => self.fem_cmd(c),
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
                if let (Some(it), true) = (self.fem.sel, self.fem.on && self.tree_sel.is_empty()) {
                    self.fem_open_task(it);
                } else if self.active_assembly().is_some() {
                    if let Some(super::assembly::AsmSel::Joint(j)) = self.asm.sel.last().copied() {
                        self.edit_joint(j);
                    }
                } else if let Some(&id) = self.tree_sel.last() {
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
            Act::ExportStep if self.active_assembly().is_some() => ("Std_Export", "Export STEP...", "Exports the assembly as a structured STEP file (AP214 or AP242): parts, sub-assemblies, names and placements", String::new(), idle && self.asm_ref().is_some_and(|a| !a.instances.is_empty())),
            Act::ExportStl if self.active_assembly().is_some() => ("Std_Export", "Export STL...", "Exports the posed parts of the assembly as one STL mesh", String::new(), idle && self.asm_ref().is_some_and(|a| !a.instances.is_empty())),
            Act::ExportStep => ("Std_Export", "Export STEP...", "Exports the body as a STEP file", String::new(), idle && self.doc.result().is_some()),
            Act::ExportStl => ("Std_Export", "Export STL...", "Exports the body as an STL mesh", String::new(), idle && self.doc.result().is_some()),
            Act::ImportStep => ("Std_Import", "Import STEP...", "Imports a STEP file with its assembly structure: its parts and (sub-)assemblies are added to the document", String::new(), idle),
            Act::AsmInterference => ("Part_Common", "Check Interference", "Lists the parts that overlap and shows the overlapping volume; measures the clearance between two parts", String::new(), idle && self.active_assembly().is_some()),
            Act::AsmBom => ("Assembly_BillOfMaterials", "Create Bill of Materials", "Lists the parts of the assembly with quantities, part numbers, materials and masses; exports CSV", String::new(), idle && self.active_assembly().is_some()),
            Act::AsmExplode => ("Assembly_ExplodedView", "Create Exploded View", "Creates an exploded view: steps that move selected parts, with a dragger, radial explosion and animation", String::new(), idle && self.active_assembly().is_some()),
            Act::Undo => ("edit-undo", "Undo", "Undoes the last change", k("Ctrl+Z"), self.can_undo()),
            Act::Redo => ("edit-redo", "Redo", "Redoes a previously undone change", k("Ctrl+Shift+Z"), self.can_redo()),
            Act::Delete => ("edit-delete", "Delete", "Deletes the selected item", "Del".into(), idle && (!self.tree_sel.is_empty() || !self.asm.sel.is_empty() || (self.fem.on && self.fem.sel.is_some()))),
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
            Act::Edit => ("edit-edit", "Edit", "Edits the selected feature, sketch or joint", String::new(), idle && (!self.tree_sel.is_empty() || matches!(self.asm.sel.last(), Some(super::assembly::AsmSel::Joint(_))) || (self.fem.on && self.fem.sel.is_some()))),
            Act::LeaveSketch => ("Sketcher_LeaveSketch", "Leave Sketch", "Finishes editing the active sketch", String::new(), self.editor.is_some()),
            Act::ViewSketch => ("Sketcher_ViewSketch", "View Sketch", "Sets the camera orientation perpendicular to the sketch plane and fits it", "Q, P".into(), self.editor.is_some()),
            Act::About => ("help-browser", "About freecad-rs", "", String::new(), true),
            Act::MateConnector => ("PartDesign_CoordinateSystem", "Mate Connector", "Creates an Onshape-style mate connector: pick a face, edge or vertex; it snaps to the inferred center, midpoint or vertex. Assemblies join parts by their connectors.", String::new(), idle && self.active_part().is_some()),
            Act::NewPart => ("PartDesign_Body", "New Part", "Creates a new part (Body) in the document and opens it", String::new(), idle),
            Act::NewAssembly => ("Geoassembly", "Create Assembly", "Creates a new assembly in the document and opens it", String::new(), idle),
            Act::AsmInsert => ("Assembly_InsertLink", "Insert Component", "Inserts a part of the document into the assembly (several times if needed)", "I".into(), idle && self.active_assembly().is_some()),
            Act::Ground => ("Assembly_ToggleGrounded", "Toggle Grounded", "Grounds the selected part (it stays fixed) or releases it", "G".into(), idle && self.active_assembly().is_some()),
            Act::Joint(name) => {
                let (_, icon, tip) = super::assembly::JOINT_TOOLS.iter().find(|x| x.0 == name).copied().unwrap_or(("", "Assembly_CreateJointFixed", ""));
                let title = match name {
                    "Fixed" => "Create a Fixed Joint",
                    "Revolute" => "Create a Revolute Joint",
                    "Cylindrical" => "Create a Cylindrical Joint",
                    "Slider" => "Create a Slider Joint",
                    "Ball" => "Create a Ball Joint",
                    "Distance" => "Create a Distance Joint",
                    "Parallel" => "Create a Parallel Joint",
                    "Perpendicular" => "Create a Perpendicular Joint",
                    "Angle" => "Create an Angle Joint",
                    "Planar" => "Create a Planar Mate",
                    _ => "Create a Pin Slot Mate",
                };
                (icon, title, tip, String::new(), idle && self.active_assembly().is_some())
            }
            Act::AsmConnector => ("PartDesign_CoordinateSystem", "Assembly Mate Connector", "Adds a mate connector to the assembly, anchored to a part's face, edge or vertex", String::new(), idle && self.active_assembly().is_some()),
            Act::Solve => ("Assembly_SolveAssembly", "Solve Assembly", "Solves the assembly's joints again", String::new(), idle && self.active_assembly().is_some()),
            Act::Workbench(true) => ("AssemblyWorkbench", "Assembly", "Assembly workbench: parts, joints and mate connectors", String::new(), idle),
            Act::Workbench(false) => ("PartDesignWorkbench", "Part Design", "Part Design workbench: sketches and features of a part", String::new(), idle),
            Act::FemWorkbench => ("FemWorkbench", "FEM", "FEM workbench: finite element analysis of a part with Gmsh and CalculiX", String::new(), idle),
            Act::Fem(c) => {
                let (icon, title, desc, en) = self.fem_cmd_info(c);
                (icon, title, desc, String::new(), en)
            }
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
        crate::widgets::reg(format!("menu:{title}"), r.rect);
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
                self.menu_item(ui, Act::NewPart);
                self.menu_item(ui, Act::NewAssembly);
                ui.separator();
                self.menu_item(ui, Act::ImportStep);
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
            if self.active_assembly().is_some() {
                ui.menu_button("Assembly", |ui| {
                    ui.set_min_width(240.0);
                    self.menu_item(ui, Act::NewAssembly);
                    self.menu_item(ui, Act::AsmInsert);
                    self.menu_item(ui, Act::Solve);
                    ui.separator();
                    self.menu_item(ui, Act::Ground);
                    ui.menu_button("Joints", |ui| {
                        ui.set_min_width(220.0);
                        for (n, _, _) in super::assembly::JOINT_TOOLS {
                            self.menu_item(ui, Act::Joint(n));
                        }
                    });
                    self.menu_item(ui, Act::AsmConnector);
                    ui.separator();
                    self.menu_item(ui, Act::AsmExplode);
                    self.menu_item(ui, Act::AsmBom);
                    self.menu_item(ui, Act::AsmInterference);
                    ui.separator();
                    self.menu_item(ui, Act::ExportStep);
                    self.menu_item(ui, Act::ExportStl);
                });
            } else if self.fem.on && self.editor.is_none() {
                ui.menu_button("Model", |ui| {
                    ui.set_min_width(260.0);
                    self.menu_item(ui, Act::Fem(FemCmd::Analysis));
                    self.menu_item(ui, Act::Fem(FemCmd::Material));
                    ui.menu_button("Mechanical boundary conditions and loads", |ui| {
                        ui.set_min_width(260.0);
                        for k in MECH {
                            self.menu_item(ui, Act::Fem(FemCmd::Constraint(k)));
                        }
                    });
                    ui.menu_button("Thermal boundary conditions and loads", |ui| {
                        ui.set_min_width(260.0);
                        for k in THERMAL {
                            self.menu_item(ui, Act::Fem(FemCmd::Constraint(k)));
                        }
                    });
                });
                ui.menu_button("Mesh", |ui| {
                    ui.set_min_width(240.0);
                    self.menu_item(ui, Act::Fem(FemCmd::Mesh));
                    self.menu_item(ui, Act::Fem(FemCmd::ClearMesh));
                });
                ui.menu_button("Solve", |ui| {
                    ui.set_min_width(220.0);
                    self.menu_item(ui, Act::Fem(FemCmd::Solver));
                    self.menu_item(ui, Act::Fem(FemCmd::Run));
                });
                ui.menu_button("Results", |ui| {
                    ui.set_min_width(220.0);
                    self.menu_item(ui, Act::Fem(FemCmd::Results));
                    self.menu_item(ui, Act::Fem(FemCmd::Purge));
                });
            } else if self.editor.is_some() {
                ui.menu_button("Sketch", |ui| {
                    ui.set_min_width(220.0);
                    self.menu_item(ui, Act::ViewSketch);
                    self.menu_item(ui, Act::LeaveSketch);
                });
            } else {
                ui.menu_button("Part Design", |ui| {
                    ui.set_min_width(220.0);
                    self.menu_item(ui, Act::NewSketch);
                    self.menu_item(ui, Act::MateConnector);
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
    fn workbench_box(&mut self, ui: &mut egui::Ui) {
        let (icon, name) = if self.editor.is_some() {
            ("SketcherWorkbench", "Sketcher")
        } else if self.active_assembly().is_some() {
            ("AssemblyWorkbench", "Assembly")
        } else if self.fem.on {
            ("FemWorkbench", "FEM")
        } else {
            ("PartDesignWorkbench", "Part Design")
        };
        let (rect, resp) = ui.allocate_exact_size(egui::vec2(170.0, 24.0), egui::Sense::click());
        crate::widgets::reg("workbench", rect);
        let y = rect.center().y;
        let p = ui.painter();
        p.rect(rect.shrink2(egui::vec2(0.0, 0.0)), 4.0, Color32::from_rgb(32, 32, 32), egui::Stroke::new(1.0, theme::BORDER), egui::StrokeKind::Inside);
        crate::icons::paint(p, icon, egui::Rect::from_center_size(egui::pos2(rect.left() + 14.0, y), egui::vec2(16.0, 16.0)), Color32::WHITE);
        p.text(egui::pos2(rect.left() + 28.0, y), Align2::LEFT_CENTER, name, egui::FontId::proportional(13.0), theme::TEXT);
        let c = egui::pos2(rect.right() - 12.0, y);
        p.add(egui::Shape::convex_polygon(vec![c + egui::vec2(-4.0, -2.0), c + egui::vec2(4.0, -2.0), c + egui::vec2(0.0, 2.5)], Color32::from_rgb(150, 150, 150), egui::Stroke::NONE));
        let resp = resp.on_hover_ui(|ui| crate::widgets::tooltip(ui, "Workbench", "Part Design edits parts, Assembly joins them, FEM analyses a part. Sketcher is active while a sketch is being edited.", ""));
        egui::Popup::menu(&resp).show(|ui| {
            ui.set_min_width(170.0);
            let cur = if self.active_assembly().is_some() { 1 } else if self.fem.on { 2 } else { 0 };
            for (i, a) in [Act::Workbench(false), Act::Workbench(true), Act::FemWorkbench].into_iter().enumerate() {
                let (icon, title, desc, _, en) = self.act_info(a);
                let r = ui.add_enabled(en, egui::Button::image_and_text(crate::icons::image(ui.ctx(), icon, 16.0), title).selected(cur == i));
                crate::widgets::reg(format!("wb:{title}"), r.rect);
                if r.on_hover_text(desc).clicked() {
                    self.run(a);
                    ui.close();
                }
            }
        });
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
            let tip = if self.active_assembly().is_some() { "Exports the assembly as STEP (with its structure) or STL" } else { "Exports the body as STEP or STL" };
            let r = crate::widgets::tool_button_resp(ui, &Action::new("export", "Std_Export", "Export", tip, "").enabled(en));
            egui::Popup::menu(&r).show(|ui| {
                ui.set_min_width(170.0);
                self.menu_item(ui, Act::ExportStep);
                self.menu_item(ui, Act::ExportStl);
            });
        });
        ui.add_space(2.0);
        ui.horizontal_wrapped(|ui| {
            if self.active_assembly().is_some() {
                // FreeCAD's Assembly toolbar order
                tb_grip(ui);
                self.tb(ui, "newasm", Act::NewAssembly, false);
                self.tb(ui, "insert", Act::AsmInsert, matches!(self.asm.task, Some(super::assembly::AsmTask::Insert { .. })));
                self.tb(ui, "solve", Act::Solve, false);
                tb_separator(ui);
                tb_grip(ui);
                self.tb(ui, "ground", Act::Ground, false);
                for n in ["Fixed", "Revolute", "Cylindrical", "Slider", "Ball"] {
                    self.tb(ui, &format!("joint:{}", n.to_lowercase()), Act::Joint(n), false);
                }
                tb_separator(ui);
                tb_grip(ui);
                for n in ["Distance", "Parallel", "Perpendicular", "Angle"] {
                    self.tb(ui, &format!("joint:{}", n.to_lowercase()), Act::Joint(n), false);
                }
                tb_separator(ui);
                tb_grip(ui);
                for n in ["Planar", "PinSlot"] {
                    self.tb(ui, &format!("joint:{}", n.to_lowercase()), Act::Joint(n), false);
                }
                self.tb(ui, "asmconnector", Act::AsmConnector, false);
                tb_separator(ui);
                tb_grip(ui);
                self.tb(ui, "explode", Act::AsmExplode, matches!(self.asm.task, Some(super::assembly::AsmTask::Explode(_))));
                self.tb(ui, "bom", Act::AsmBom, matches!(self.asm.task, Some(super::assembly::AsmTask::Bom(_))));
                self.tb(ui, "interference", Act::AsmInterference, matches!(self.asm.task, Some(super::assembly::AsmTask::Interference(_))));
                tb_separator(ui);
                tb_grip(ui);
                self.tb(ui, "edit", Act::Edit, false);
                self.tb(ui, "delete", Act::Delete, false);
            } else if self.fem.on && self.editor.is_none() {
                // FreeCAD's FEM toolbars: Model, Mechanical, Thermal, Mesh, Solve, Results
                tb_grip(ui);
                self.tb(ui, "fem_analysis", Act::Fem(FemCmd::Analysis), false);
                self.tb(ui, "fem_material", Act::Fem(FemCmd::Material), matches!(self.fem.task, Some(super::fem::FemTask::Material { .. })));
                tb_separator(ui);
                tb_grip(ui);
                for k in MECH {
                    self.tb(ui, k.info().1, Act::Fem(FemCmd::Constraint(k)), false);
                }
                tb_separator(ui);
                tb_grip(ui);
                for k in THERMAL {
                    self.tb(ui, k.info().1, Act::Fem(FemCmd::Constraint(k)), false);
                }
                tb_separator(ui);
                tb_grip(ui);
                self.tb(ui, "fem_mesh", Act::Fem(FemCmd::Mesh), matches!(self.fem.task, Some(super::fem::FemTask::Mesh { .. })));
                self.tb(ui, "fem_clearmesh", Act::Fem(FemCmd::ClearMesh), false);
                tb_separator(ui);
                tb_grip(ui);
                self.tb(ui, "fem_solver", Act::Fem(FemCmd::Solver), matches!(self.fem.task, Some(super::fem::FemTask::Solver { .. })));
                self.tb(ui, "fem_run", Act::Fem(FemCmd::Run), self.fem.job.is_some());
                tb_separator(ui);
                tb_grip(ui);
                self.tb(ui, "fem_results", Act::Fem(FemCmd::Results), matches!(self.fem.task, Some(super::fem::FemTask::Results { .. })));
                self.tb(ui, "fem_purge", Act::Fem(FemCmd::Purge), false);
                tb_separator(ui);
                tb_grip(ui);
                self.tb(ui, "edit", Act::Edit, false);
                self.tb(ui, "delete", Act::Delete, false);
            } else if self.editor.is_some() {
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
                self.tb(ui, "part", Act::NewPart, false);
                self.tb(ui, "sketch", Act::NewSketch, false);
                self.tb(ui, "mate", Act::MateConnector, self.mate_task().is_some());
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

    /// FreeCAD's document tab under the 3D view ("Unnamed : 1*"), followed (once the
    /// document has more than one part or an assembly) by Onshape-style tabs for its parts
    /// and assemblies, and a "+" to add one.
    pub(super) fn doc_tabs(&mut self, ui: &mut egui::Ui) {
        let text = format!("{} : 1{}", self.doc_title(), if self.dirty { "*" } else { "" });
        let top = ui.max_rect().top();
        let mut x = ui.max_rect().left() + 2.0;
        let tab = |ui: &mut egui::Ui, x: &mut f32, reg: &str, icon: &str, text: &str, active: bool, tip: &str| -> egui::Response {
            let font = if active { theme::bold_font(13.0) } else { egui::FontId::proportional(13.0) };
            let g = ui.painter().layout_no_wrap(text.to_string(), font, Color32::WHITE);
            let rect = egui::Rect::from_min_size(egui::pos2(*x, top), egui::vec2(g.size().x + 40.0, 26.0));
            *x = rect.right() + 2.0;
            let resp = ui.interact(rect, egui::Id::new(("doc_tab", reg)), egui::Sense::click());
            crate::widgets::reg(reg.to_string(), rect);
            let p = ui.painter();
            let fill = if active { Color32::from_rgb(40, 40, 40) } else if resp.hovered() { Color32::from_rgb(34, 34, 34) } else { Color32::from_rgb(28, 28, 28) };
            p.rect_filled(rect, egui::CornerRadius { nw: 0, ne: 0, sw: 4, se: 4 }, fill);
            if active {
                p.line_segment([rect.left_top(), rect.right_top()], egui::Stroke::new(2.0, theme::ACCENT));
            }
            crate::icons::paint(p, icon, egui::Rect::from_center_size(egui::pos2(rect.left() + 15.0, rect.center().y), egui::vec2(16.0, 16.0)), Color32::WHITE);
            p.galley(egui::pos2(rect.left() + 30.0, rect.center().y - g.size().y * 0.5), g, if active { Color32::WHITE } else { theme::TEXT_DIM });
            resp.on_hover_text(tip)
        };
        let path_tip = self.path.as_ref().map(|p| p.display().to_string()).unwrap_or_else(|| "Not saved yet".into());
        tab(ui, &mut x, "doc_tab", "Document", &text, true, &path_tip);
        let multi = self.ws.parts.len() > 1 || !self.ws.assemblies.is_empty();
        if !multi {
            return;
        }
        x += 10.0;
        let items: Vec<(Tab, String)> = self.ws.parts.iter().map(|p| (Tab::Part(p.id), p.name.clone())).chain(self.ws.assemblies.iter().map(|a| (Tab::Assembly(a.id), a.name.clone()))).collect();
        let mut go = None;
        for (t, name) in items {
            let icon = if matches!(t, Tab::Part(_)) { "PartDesign_Body" } else { "Geoassembly" };
            let tip = if matches!(t, Tab::Part(_)) { format!("Part {name}: open in Part Design") } else { format!("Assembly {name}: open in the Assembly workbench") };
            if tab(ui, &mut x, &format!("tab:{name}"), icon, &name, self.tab == t, &tip).clicked() {
                go = Some(t);
            }
        }
        let r = tab(ui, &mut x, "tab:+", "list-add", "", false, "Add a part or an assembly");
        egui::Popup::menu(&r).show(|ui| {
            ui.set_min_width(170.0);
            self.menu_item(ui, Act::NewPart);
            self.menu_item(ui, Act::NewAssembly);
        });
        if let Some(t) = go {
            self.switch_tab(t);
        }
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
                    } else if let Some(j) = &self.fem.job {
                        format!("{}  ({:.1} s)", j.stage, j.started.elapsed().as_secs_f64())
                    } else if let Some(t) = &self.fem.hover_text {
                        t.clone()
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
                Key::Escape if self.asm.task.is_some() => {
                    self.asm_key(false);
                }
                Key::Enter if self.asm.task.is_some() => {
                    self.asm_key(true);
                }
                Key::Delete | Key::Backspace if self.idle() && self.active_assembly().is_some() => self.asm_delete_selected(),
                Key::G if self.idle() && self.active_assembly().is_some() => self.run(Act::Ground),
                Key::I if self.idle() && self.active_assembly().is_some() => self.run(Act::AsmInsert),
                Key::Escape if self.fem.task.is_some() => self.fem_task_cancel(),
                Key::Enter if self.fem.task.is_some() => self.fem_task_ok(),
                Key::Delete | Key::Backspace if self.idle() && self.fem.on && self.fem.sel.is_some() && self.tree_sel.is_empty() => self.run(Act::Delete),
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
