//! FEM workbench UI: the analysis in the model tree, the task panels (constraints, material,
//! mesh, solver, results), constraint glyphs, the result legend, max marker and the value
//! read-out under the cursor.

use super::fem::*;
use super::tree::{tree_row_pub, RowSpecPub};
use super::*;
use crate::theme;
use crate::widgets::{self, task_box, task_row, PropGrid};
use egui::{Color32, Pos2, Rect, RichText, Stroke, Vec2};
use fc_fem::{AnalysisType, ConstraintKind, Field, ForceDirection, HeatFluxKind, Material, SubKind};

// ------------------------------------------------------------------ quantities

/// Unit choices accepted when typing a value with `unit` as its base unit: (unit, factor).
fn unit_factors(unit: &str) -> &'static [(&'static str, f64)] {
    match unit {
        "N" => &[("N", 1.0), ("kN", 1e3), ("mN", 1e-3), ("MN", 1e6)],
        "MPa" => &[("MPa", 1.0), ("kPa", 1e-3), ("Pa", 1e-6), ("GPa", 1e3), ("N/mm^2", 1.0), ("N/mm²", 1.0), ("bar", 0.1)],
        "mm" => &[("mm", 1.0), ("m", 1e3), ("cm", 10.0), ("um", 1e-3), ("µm", 1e-3)],
        "mm/s²" => &[("mm/s²", 1.0), ("mm/s^2", 1.0), ("m/s^2", 1e3), ("m/s²", 1e3)],
        _ => &[],
    }
}

/// Parse "1000", "1 kN", "25 mm", "20 °C" (for K) into the base unit.
pub(super) fn parse_qty(text: &str, unit: &str) -> Option<f64> {
    let t = text.trim();
    let bytes: Vec<char> = t.chars().collect();
    let mut cut = bytes.len();
    for (i, c) in bytes.iter().enumerate() {
        let exp = (*c == 'e' || *c == 'E') && i > 0 && bytes.get(i + 1).is_some_and(|n| n.is_ascii_digit() || *n == '-' || *n == '+');
        if (c.is_alphabetic() && !exp) || *c == '°' || *c == 'µ' || *c == '/' {
            cut = i;
            break;
        }
    }
    let num: String = bytes[..cut].iter().collect();
    let u: String = bytes[cut..].iter().collect::<String>().trim().to_string();
    let v: f64 = fc_part::expr::parse(num.trim()).ok()?.eval(&mut |n| Err(fc_part::expr::ExprError(format!("unknown {n}")))).ok()?;
    if u.is_empty() || u == unit {
        return Some(v);
    }
    if unit == "K" && (u == "°C" || u == "C" || u == "degC") {
        return Some(v + 273.15);
    }
    unit_factors(unit).iter().find(|(n, _)| *n == u).map(|(_, f)| v * f)
}

/// Compact general number formatting ("200000", "0.3", "1.2e-5").
pub(super) fn fmt_g(v: f64) -> String {
    if v == 0.0 || !v.is_finite() {
        return "0".into();
    }
    let a = v.abs();
    if (1e-3..1e9).contains(&a) {
        let s = format!("{v:.6}");
        let s = s.trim_end_matches('0').trim_end_matches('.');
        return s.to_string();
    }
    format!("{v:e}")
}

/// A quantity spin box like FreeCAD's (`1000 N`); `name` registers it as `fem:<name>`.
/// Applies on Enter or when it loses focus; returns true when the value changed.
pub(super) fn qty(ui: &mut egui::Ui, name: &str, v: &mut f64, unit: &str, enabled: bool) -> bool {
    let id = egui::Id::new(("fem_qty", name));
    let shown = if unit.is_empty() { fmt_g(*v) } else { format!("{} {unit}", fmt_g(*v)) };
    let mut buf = ui.data(|d| d.get_temp::<String>(id)).unwrap_or_else(|| shown.clone());
    let w = ui.available_width() - 2.0;
    let r = ui.add_enabled(enabled, egui::TextEdit::singleline(&mut buf).id(id.with("te")).desired_width(w).margin(egui::Margin::symmetric(5, 2)));
    widgets::reg(format!("fem:{name}"), r.rect);
    let r = r.on_hover_text(if unit.is_empty() { "A number".to_string() } else { format!("A value in {unit} (other units are converted, e.g. {})", unit_factors(unit).get(1).map(|u| u.0).unwrap_or(unit)) });
    let mut changed = false;
    if r.lost_focus() {
        if !ui.input(|i| i.key_pressed(egui::Key::Escape)) {
            match parse_qty(&buf, unit) {
                Some(x) if (x - *v).abs() > 1e-15 * x.abs().max(1.0) => {
                    *v = x;
                    changed = true;
                }
                _ => {}
            }
        }
        ui.data_mut(|d| d.remove::<String>(id));
    } else if r.has_focus() {
        ui.data_mut(|d| d.insert_temp(id, buf));
    } else {
        ui.data_mut(|d| d.remove::<String>(id));
    }
    changed
}

fn check(ui: &mut egui::Ui, name: &str, v: &mut bool, text: &str, tip: &str) -> bool {
    let r = ui.checkbox(v, text);
    widgets::reg(format!("check:{name}"), r.rect);
    let r = if tip.is_empty() { r } else { r.on_hover_text(tip) };
    r.changed()
}

/// Small toggle push button (FreeCAD's checkable "Add" / "Remove" buttons).
fn toggle_button(ui: &mut egui::Ui, id: &str, text: &str, on: bool, w: f32, tip: &str) -> bool {
    let mut b = egui::Button::new(RichText::new(text).size(13.0)).min_size(egui::vec2(w, 24.0)).corner_radius(4.0);
    if on {
        b = b.fill(Color32::from_rgb(40, 62, 88)).stroke(Stroke::new(1.0, theme::ACCENT));
    }
    let r = ui.add(b);
    widgets::reg(id.to_string(), r.rect);
    r.on_hover_text(tip).clicked()
}

fn error_box(ui: &mut egui::Ui, text: &str) {
    egui::Frame::new().fill(Color32::from_rgb(70, 32, 30)).corner_radius(4.0).inner_margin(8.0).show(ui, |ui| {
        ui.set_width(ui.available_width());
        ui.horizontal_wrapped(|ui| {
            ui.add(crate::icons::image(ui.ctx(), "overlay_error", 16.0));
            ui.label(RichText::new(text).color(Color32::from_rgb(255, 190, 180)));
        });
    });
    ui.add_space(8.0);
}

fn info_text(ui: &mut egui::Ui, text: &str) {
    ui.label(RichText::new(text).color(Color32::from_gray(185)));
}

/// Monospace log box that sticks to its end (FreeCAD's solver output view).
fn log_box(ui: &mut egui::Ui, id: &str, text: &str, height: f32) {
    egui::Frame::new().fill(theme::INPUT).stroke(Stroke::new(1.0, theme::BORDER)).corner_radius(3.0).inner_margin(4.0).show(ui, |ui| {
        ui.set_width(ui.available_width());
        egui::ScrollArea::vertical().id_salt(id).max_height(height).auto_shrink([false, true]).stick_to_bottom(true).show(ui, |ui| {
            let lines: Vec<&str> = text.lines().collect();
            let tail = &lines[lines.len().saturating_sub(400)..];
            ui.label(RichText::new(tail.join("\n")).monospace().size(11.0).color(Color32::from_gray(200)));
        });
    });
}

const SEL_GREEN: Color32 = Color32::from_rgb(28, 179, 28);
const GLYPH_RED: Color32 = Color32::from_rgb(255, 0, 0);
const GLYPH_THERMAL: Color32 = Color32::from_rgb(255, 120, 0);

impl App {
    // ---------------------------------------------------------------- tree

    /// The open part's analyses below its Body (FreeCAD lists them at the document level).
    pub(super) fn fem_tree(&mut self, ui: &mut egui::Ui, interactive: bool) {
        let list = self.fem_analyses();
        if list.is_empty() {
            return;
        }
        let cur = self.fem_cur();
        let mut clicked: Option<FemItem> = None;
        let mut double: Option<FemItem> = None;
        let mut delete: Option<FemItem> = None;
        let mut toggle_mesh = false;
        let mut toggle_res = false;
        let mut activate: Option<usize> = None;
        let sel = self.fem.sel;
        let task_item = self.fem.task.as_ref().map(|t| t.analysis());
        let _ = task_item;
        for a in list {
            let d = self.fem.docs[a].clone();
            let key = format!("fem{a}:{}", d.analysis.name);
            let open = super::tree::is_open(ui, &key, true);
            let label = if Some(a) == cur { RichText::new(&d.analysis.name).font(theme::bold_font(13.0)) } else { RichText::new(&d.analysis.name) };
            let badge = d.rt.error.as_ref().map(|_| "overlay_error");
            let tip = format!("{}: FEM analysis of {}{}\n{}", d.analysis.name, d.part, if Some(a) == cur { " (active analysis)" } else { "" }, d.rt.error.clone().unwrap_or_default());
            let it = FemItem::Analysis(a);
            let mut row = |app: &mut App, ui: &mut egui::Ui, reg: String, depth: usize, expander: Option<bool>, eye: Option<bool>, icon: &str, badge: Option<&str>, label: RichText, it: FemItem, tip: String| {
                let r = tree_row_pub(ui, RowSpecPub { reg, depth, expander, eye, icon, badge, label, selected: sel == Some(it), tip });
                if interactive {
                    if r.clicked {
                        clicked = Some(it);
                    }
                    if r.double {
                        double = Some(it);
                    }
                    let editable = !matches!(it, FemItem::Analysis(_) | FemItem::ResultMesh(_));
                    r.resp.context_menu(|ui| {
                        if editable && ui.button("Edit").clicked() {
                            double = Some(it);
                            ui.close();
                        }
                        if let FemItem::Analysis(a) = it {
                            if ui.button("Activate analysis").clicked() {
                                activate = Some(a);
                                ui.close();
                            }
                        }
                        if ui.button("Delete").clicked() {
                            delete = Some(it);
                            ui.close();
                        }
                    });
                }
                let _ = app;
                r
            };
            let r = row(self, ui, format!("tree:{}", d.analysis.name), 1, Some(open), None, "FEM_Analysis", badge, label, it, tip);
            if r.expand {
                super::tree::toggle_open(ui, &key, true);
            }
            if !open {
                continue;
            }
            if d.solver {
                let at = match d.analysis.analysis_type {
                    AnalysisType::Static => "static".to_string(),
                    AnalysisType::Frequency { modes, .. } => format!("frequency, {modes} modes"),
                    AnalysisType::Thermomech { .. } => "thermomechanical".to_string(),
                };
                row(self, ui, "tree:SolverCcxTools".into(), 2, None, None, "FEM_SolverStandard", None, RichText::new("SolverCcxTools"), FemItem::Solver(a), format!("CalculiX solver ({at})\nDouble-click to run it"));
            }
            for (m, ma) in d.analysis.materials.iter().enumerate() {
                let tip = format!("{}: {} (E = {} MPa, ν = {}, ρ = {} kg/m³)", ma.name, ma.material.name, fmt_g(ma.material.youngs_modulus), fmt_g(ma.material.poisson_ratio), fmt_g(ma.material.density));
                row(self, ui, format!("tree:{}", ma.name), 2, None, None, "FEM_MaterialSolid", None, RichText::new(&ma.name), FemItem::Material(a, m), tip);
            }
            if d.mesh {
                let (tip, label) = match &d.rt.mesh {
                    Some(m) => (format!("FEMMeshGmsh: {} nodes, {} elements", m.node_count(), m.element_count()), RichText::new("FEMMeshGmsh")),
                    None => ("FEMMeshGmsh: not meshed yet (double-click, then Apply)".to_string(), RichText::new("FEMMeshGmsh").color(theme::TEXT_DIM)),
                };
                let eye = d.rt.mesh.as_ref().map(|_| !self.fem.hide_mesh);
                let r = row(self, ui, "tree:FEMMeshGmsh".into(), 2, None, eye, "FEM_MeshGmshFromShape", None, label, FemItem::Mesh(a), tip);
                toggle_mesh |= r.eye;
            }
            for (c, con) in d.analysis.constraints.iter().enumerate() {
                let k = CKind::of(&con.kind);
                let missing = k.needs_refs() && con.refs.is_empty();
                let refs = con.refs.iter().map(|r| r.name()).collect::<Vec<_>>().join(", ");
                let tip = format!("{}: {}\n{}", con.name, k.info().2, if missing { "No references: double-click and pick geometry".to_string() } else if refs.is_empty() { String::new() } else { format!("References: {refs}") });
                row(self, ui, format!("tree:{}", con.name), 2, None, None, k.info().0, missing.then_some("overlay_error"), RichText::new(&con.name), FemItem::Constraint(a, c), tip);
            }
            if let Some(r) = &d.rt.results {
                let label = if r.modes.is_empty() { "CCX_Results".to_string() } else { format!("CCX_Results ({} modes)", r.modes.len()) };
                let res = row(self, ui, "tree:CCX_Results".into(), 2, None, Some(!self.fem.hide_results), "FEM_ResultShow", None, RichText::new(label), FemItem::Results(a), "Results of the last CalculiX run\nDouble-click to choose the field and the deformation".into());
                toggle_res |= res.eye;
                row(self, ui, "tree:ResultMesh".into(), 2, None, None, "FEM_MeshResult", None, RichText::new("ResultMesh"), FemItem::ResultMesh(a), "The mesh the results belong to".into());
            }
        }
        if toggle_mesh {
            self.fem.hide_mesh = !self.fem.hide_mesh;
            self.rebuild_scene();
        }
        if toggle_res {
            self.fem.hide_results = !self.fem.hide_results;
            self.rebuild_scene();
        }
        if let Some(a) = activate {
            self.fem.active = Some(a);
            self.rebuild_scene();
        }
        if let Some(it) = clicked {
            self.fem.sel = Some(it);
            self.fem.active = Some(it.analysis());
            self.tree_sel.clear();
            self.origin_sel = None;
            if !self.fem.on {
                self.fem_enter();
            } else {
                self.rebuild_scene();
            }
        }
        if let Some(it) = double {
            if !self.fem.on {
                self.fem_enter();
            }
            self.fem_open_task(it);
        }
        if let Some(it) = delete {
            self.fem_delete(it);
        }
    }

    // ------------------------------------------------------------ data tab

    /// Property view for the selected analysis item (read-only summary).
    pub(super) fn fem_data_tab(&mut self, ui: &mut egui::Ui, it: FemItem) {
        let Some(d) = self.fem.docs.get(it.analysis()).cloned() else { return };
        let mut g = PropGrid::new(ui);
        let ro = |g: &mut PropGrid, ui: &mut egui::Ui, k: &str, v: String| {
            g.row(ui, k, "", |ui| widgets::ro_value(ui, &v));
        };
        match it {
            FemItem::Analysis(_) => {
                g.group(ui, "Base");
                ro(&mut g, ui, "Label", d.analysis.name.clone());
                ro(&mut g, ui, "Part", d.part.clone());
                ro(&mut g, ui, "Constraints", d.analysis.constraints.len().to_string());
            }
            FemItem::Solver(_) => {
                g.group(ui, "Solver");
                let t = match d.analysis.analysis_type {
                    AnalysisType::Static => "static".to_string(),
                    AnalysisType::Frequency { .. } => "frequency".to_string(),
                    AnalysisType::Thermomech { steady_state } => format!("thermomech{}", if steady_state { " (steady state)" } else { "" }),
                };
                ro(&mut g, ui, "Analysis Type", t);
                if let AnalysisType::Frequency { modes, low_hz, high_hz } = d.analysis.analysis_type {
                    ro(&mut g, ui, "Eigenmodes Count", modes.to_string());
                    ro(&mut g, ui, "Eigenmode Low Limit", format!("{} Hz", fmt_g(low_hz)));
                    ro(&mut g, ui, "Eigenmode High Limit", format!("{} Hz", fmt_g(high_hz)));
                }
                ro(&mut g, ui, "Increments Maximum", d.analysis.solver.increments_maximum.to_string());
                ro(&mut g, ui, "Matrix Solver", d.analysis.solver.matrix_solver.clone().unwrap_or("default".into()));
                ro(&mut g, ui, "Executable", fc_fem::tools::ccx().map(|p| p.display().to_string()).unwrap_or_else(|_| "not found".into()));
            }
            FemItem::Material(_, m) => {
                let Some(ma) = d.analysis.materials.get(m) else { return };
                let mm = &ma.material;
                g.group(ui, "Material");
                ro(&mut g, ui, "Card", mm.name.clone());
                ro(&mut g, ui, "Young's Modulus", format!("{} MPa", fmt_g(mm.youngs_modulus)));
                ro(&mut g, ui, "Poisson Ratio", fmt_g(mm.poisson_ratio));
                ro(&mut g, ui, "Density", format!("{} kg/m³", fmt_g(mm.density)));
                ro(&mut g, ui, "Thermal Conductivity", format!("{} W/m/K", fmt_g(mm.thermal_conductivity)));
                ro(&mut g, ui, "Thermal Expansion", format!("{} 1/K", fmt_g(mm.thermal_expansion)));
                ro(&mut g, ui, "Specific Heat", format!("{} J/kg/K", fmt_g(mm.specific_heat)));
            }
            FemItem::Mesh(_) => {
                let ms = &d.analysis.mesh;
                g.group(ui, "FEM Mesh");
                ro(&mut g, ui, "Characteristic Length Max", if ms.max_size > 0.0 { format!("{} mm", fmt_g(ms.max_size)) } else { "0 mm (automatic)".into() });
                ro(&mut g, ui, "Characteristic Length Min", format!("{} mm", fmt_g(ms.min_size)));
                ro(&mut g, ui, "Element Order", if ms.order == 2 { "2nd".into() } else { "1st".into() });
                ro(&mut g, ui, "Mesh Size From Curvature", ms.size_from_curvature.to_string());
                if let Some(m) = &d.rt.mesh {
                    ro(&mut g, ui, "Nodes", m.node_count().to_string());
                    ro(&mut g, ui, "Volume elements", m.element_count().to_string());
                }
            }
            FemItem::Constraint(_, c) => {
                let Some(con) = d.analysis.constraints.get(c) else { return };
                g.group(ui, "Constraint");
                ro(&mut g, ui, "Label", con.name.clone());
                ro(&mut g, ui, "References", con.refs.iter().map(|r| r.name()).collect::<Vec<_>>().join(", "));
                match &con.kind {
                    ConstraintKind::Force { force, direction } => {
                        ro(&mut g, ui, "Force", format!("{} N", fmt_g(*force)));
                        ro(&mut g, ui, "Direction", dir_text(direction));
                    }
                    ConstraintKind::Pressure { pressure, reversed } => {
                        ro(&mut g, ui, "Pressure", format!("{} MPa", fmt_g(*pressure)));
                        ro(&mut g, ui, "Reversed", reversed.to_string());
                    }
                    ConstraintKind::Displacement { x, y, z } => {
                        for (n, v) in [("Displacement X", x), ("Displacement Y", y), ("Displacement Z", z)] {
                            ro(&mut g, ui, n, v.map(|v| format!("{} mm", fmt_g(v))).unwrap_or("free".into()));
                        }
                    }
                    ConstraintKind::SelfWeight { gravity, direction } => {
                        ro(&mut g, ui, "Gravity Acceleration", format!("{} mm/s²", fmt_g(*gravity)));
                        ro(&mut g, ui, "Gravity Direction", format!("({}, {}, {})", fmt_g(direction[0]), fmt_g(direction[1]), fmt_g(direction[2])));
                    }
                    ConstraintKind::Temperature { temperature } | ConstraintKind::InitialTemperature { temperature } => ro(&mut g, ui, "Temperature", format!("{} K", fmt_g(*temperature))),
                    _ => {}
                }
            }
            FemItem::Results(_) | FemItem::ResultMesh(_) => {
                let Some(r) = &d.rt.results else { return };
                g.group(ui, "Result");
                ro(&mut g, ui, "Nodes", r.node_count().to_string());
                if r.modes.is_empty() {
                    if let Some(e) = r.max_displacement() {
                        ro(&mut g, ui, "Max displacement", format!("{} mm (node {})", sig(e.value), e.node));
                    }
                    if let Some(e) = r.max_von_mises() {
                        ro(&mut g, ui, "Max von Mises", format!("{} MPa (node {})", sig(e.value), e.node));
                    }
                    for (k, v) in &r.reactions {
                        ro(&mut g, ui, &format!("Reaction {k}"), format!("({}, {}, {}) N", sig(v[0]), sig(v[1]), sig(v[2])));
                    }
                } else {
                    for m in &r.modes {
                        ro(&mut g, ui, &format!("Mode {}", m.number), format!("{} Hz", sig(m.frequency)));
                    }
                }
                ro(&mut g, ui, "Run time", format!("{:.1} s", d.rt.elapsed));
            }
        }
    }

    // ------------------------------------------------------------ task panel

    pub(super) fn fem_task_panel(&mut self, ui: &mut egui::Ui) {
        let Some(t) = self.fem.task.as_ref() else { return };
        let a = t.analysis();
        if a >= self.fem.docs.len() {
            self.fem.task = None;
            return;
        }
        match t {
            FemTask::Constraint { c, mode, .. } => {
                let (c, mode) = (*c, *mode);
                self.fem_constraint_task(ui, a, c, mode)
            }
            FemTask::Material { m, .. } => {
                let m = *m;
                self.fem_material_task(ui, a, m)
            }
            FemTask::Mesh { .. } => self.fem_mesh_task(ui, a),
            FemTask::Solver { .. } => self.fem_solver_task(ui, a),
            FemTask::Results { .. } => self.fem_results_task(ui, a),
        }
    }

    fn fem_constraint_task(&mut self, ui: &mut egui::Ui, a: usize, c: usize, mode: PickMode) {
        // OK first: a value typed in a field applies when the field loses focus this frame
        let (cancel, ok) = self.task_buttons(ui, ("task:cancel", "Cancel"), Some(("task:ok", "OK")));
        let Some(con) = self.fem.docs[a].analysis.constraints.get(c).cloned() else {
            self.fem.task = None;
            return;
        };
        let k = CKind::of(&con.kind);
        let (icon, _, title, _) = k.info();
        let mut new = con.clone();
        let mut set_mode = None;
        egui::ScrollArea::vertical().id_salt("fem_task_scroll").auto_shrink([false, true]).show(ui, |ui| {
            if k.needs_refs() {
                task_box(ui, "fem_refs", icon, title, |ui| {
                    ui.spacing_mut().item_spacing.y = 6.0;
                    info_text(ui, if matches!(k, CKind::Pressure | CKind::HeatFlux) { "Click faces of the part to add them." } else { "Click faces, edges or vertices of the part to add them." });
                    ui.horizontal(|ui| {
                        let w = (ui.available_width() - 8.0) / 2.0;
                        if toggle_button(ui, "fem:add", "Add", mode == PickMode::Add, w, "Clicks in the 3D view add references") {
                            set_mode = Some(PickMode::Add);
                        }
                        if toggle_button(ui, "fem:remove", "Remove", mode == PickMode::Remove, w, "Clicks in the 3D view remove references") {
                            set_mode = Some(PickMode::Remove);
                        }
                    });
                    egui::Frame::new().fill(theme::INPUT).stroke(Stroke::new(1.0, theme::BORDER)).corner_radius(3.0).inner_margin(3.0).show(ui, |ui| {
                        ui.set_width(ui.available_width());
                        ui.set_min_height(66.0);
                        if new.refs.is_empty() {
                            ui.label(RichText::new("No references yet").color(theme::TEXT_DIM).italics());
                        }
                        let mut remove = None;
                        for (i, r) in new.refs.iter().enumerate() {
                            ui.horizontal(|ui| {
                                ui.label(RichText::new(format!("{}:{}", self.fem.docs[a].part, r.name())).color(theme::TEXT));
                                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                    let b = ui.add(egui::Button::image(crate::icons::image(ui.ctx(), "list-remove", 14.0)).frame(false));
                                    widgets::reg(format!("fem:ref:{}", r.name()), b.rect);
                                    if b.on_hover_text("Remove this reference").clicked() {
                                        remove = Some(i);
                                    }
                                });
                            });
                        }
                        if let Some(i) = remove {
                            new.refs.remove(i);
                        }
                    });
                });
            }
            task_box(ui, "fem_params", icon, "Parameters", |ui| {
                ui.spacing_mut().item_spacing.y = 6.0;
                self.fem_constraint_params(ui, &mut new.kind, mode, &mut set_mode);
            });
        });
        if new != con {
            self.fem.docs[a].analysis.constraints[c] = new;
            self.dirty = true;
        }
        if let Some(m) = set_mode {
            if let Some(FemTask::Constraint { mode, .. }) = self.fem.task.as_mut() {
                *mode = m;
            }
            self.status = match m {
                PickMode::Add => "Click geometry to add it".into(),
                PickMode::Remove => "Click a reference to remove it".into(),
                PickMode::Direction => "Click a flat face (its normal) or a straight edge to set the direction".into(),
            };
        }
        if ok {
            self.fem_task_ok();
        } else if cancel {
            self.fem_task_cancel();
        }
    }

    fn fem_constraint_params(&mut self, ui: &mut egui::Ui, kind: &mut ConstraintKind, mode: PickMode, set_mode: &mut Option<PickMode>) {
        match kind {
            ConstraintKind::Fixed => info_text(ui, "The references cannot move (all translations are zero)."),
            ConstraintKind::Force { force, direction } => {
                task_row(ui, "Force", |ui| qty(ui, "force", force, "N", true));
                // direction: face normal, a global axis or a picked direction, then reverse
                let (choice, rev) = dir_choice(direction);
                let mut ch = choice;
                task_row(ui, "Direction", |ui| widgets::combo(ui, "fem_dir", &mut ch, &[(0, "Normal to face"), (1, "Along X"), (2, "Along Y"), (3, "Along Z"), (4, "Picked direction")], ui.available_width() - 2.0));
                ui.horizontal(|ui| {
                    if toggle_button(ui, "fem:seldir", "Select direction", mode == PickMode::Direction, ui.available_width() - 2.0, "Then click a flat face (normal) or a straight edge in the 3D view") {
                        *set_mode = Some(if mode == PickMode::Direction { PickMode::Add } else { PickMode::Direction });
                    }
                });
                let mut r2 = rev;
                check(ui, "Reverse direction", &mut r2, "Reverse direction", "Flip the force direction");
                if ch != choice || r2 != rev {
                    *direction = make_dir(ch, r2, direction);
                }
                info_text(ui, &format!("Acts along {}", dir_text(direction)));
            }
            ConstraintKind::Pressure { pressure, reversed } => {
                task_row(ui, "Pressure", |ui| qty(ui, "pressure", pressure, "MPa", true));
                check(ui, "Reverse direction", reversed, "Reverse direction", "Pull on the face instead of pushing (suction)");
            }
            ConstraintKind::Displacement { x, y, z } => {
                for (n, v) in [("X", x), ("Y", y), ("Z", z)] {
                    let mut free = v.is_none();
                    let mut val = v.unwrap_or(0.0);
                    task_row(ui, &format!("Displacement {n}"), |ui| {
                        let r = ui.checkbox(&mut free, "Free");
                        widgets::reg(format!("check:free{n}"), r.rect);
                        qty(ui, &format!("disp{n}"), &mut val, "mm", !free)
                    });
                    *v = if free { None } else { Some(val) };
                }
            }
            ConstraintKind::SelfWeight { gravity, direction } => {
                task_row(ui, "Gravity", |ui| qty(ui, "gravity", gravity, "mm/s²", true));
                for (i, n) in ["X", "Y", "Z"].iter().enumerate() {
                    task_row(ui, &format!("Direction {n}"), |ui| qty(ui, &format!("gdir{n}"), &mut direction[i], "", true));
                }
            }
            ConstraintKind::Temperature { temperature } => {
                task_row(ui, "Temperature", |ui| qty(ui, "temperature", temperature, "K", true));
            }
            ConstraintKind::InitialTemperature { temperature } => {
                task_row(ui, "Initial temperature", |ui| qty(ui, "temperature", temperature, "K", true));
                info_text(ui, "Temperature of the whole body at the start of a thermomechanical analysis.");
            }
            ConstraintKind::HeatSource { power } => {
                task_row(ui, "Heat flux", |ui| qty(ui, "power", power, "W", true));
            }
            ConstraintKind::HeatFlux(hf) => {
                let mut conv = matches!(hf, HeatFluxKind::Convection { .. });
                let before = conv;
                task_row(ui, "Type", |ui| widgets::combo(ui, "fem_hf", &mut conv, &[(false, "Heat flux"), (true, "Convection")], ui.available_width() - 2.0));
                if conv != before {
                    *hf = if conv { HeatFluxKind::Convection { film_coefficient: 10.0, ambient_temperature: 300.0 } } else { HeatFluxKind::Flux(0.0) };
                }
                match hf {
                    HeatFluxKind::Flux(q) => {
                        task_row(ui, "Surface heat flux", |ui| qty(ui, "flux", q, "W/m²", true));
                    }
                    HeatFluxKind::Convection { film_coefficient, ambient_temperature } => {
                        task_row(ui, "Film coefficient", |ui| qty(ui, "film", film_coefficient, "W/m²/K", true));
                        task_row(ui, "Ambient temperature", |ui| qty(ui, "ambient", ambient_temperature, "K", true));
                    }
                }
            }
        }
    }

    fn fem_material_task(&mut self, ui: &mut egui::Ui, a: usize, m: usize) {
        let (cancel, ok) = self.task_buttons(ui, ("task:cancel", "Cancel"), Some(("task:ok", "OK")));
        let Some(ma) = self.fem.docs[a].analysis.materials.get(m).cloned() else {
            self.fem.task = None;
            return;
        };
        let mut mat = ma.material.clone();
        egui::ScrollArea::vertical().id_salt("fem_task_scroll").auto_shrink([false, true]).show(ui, |ui| {
            task_box(ui, "fem_mat", "FEM_MaterialSolid", "Material", |ui| {
                ui.spacing_mut().item_spacing.y = 6.0;
                let lib = Material::library();
                let names: Vec<String> = lib.iter().map(|m| m.name.clone()).collect();
                let mut cur = names.iter().position(|n| *n == mat.name).unwrap_or(usize::MAX);
                let mut opts: Vec<(usize, &str)> = names.iter().enumerate().map(|(i, n)| (i, n.as_str())).collect();
                if cur == usize::MAX {
                    opts.push((usize::MAX, mat.name.as_str()));
                }
                let before = cur;
                task_row(ui, "Material card", |ui| widgets::combo(ui, "fem_card", &mut cur, &opts, ui.available_width() - 2.0));
                if cur != before {
                    if let Some(l) = lib.get(cur) {
                        mat = l.clone();
                    }
                }
                ui.add_space(2.0);
                let mut edited = false;
                edited |= task_row(ui, "Young's modulus", |ui| qty(ui, "youngs", &mut mat.youngs_modulus, "MPa", true));
                edited |= task_row(ui, "Poisson ratio", |ui| qty(ui, "poisson", &mut mat.poisson_ratio, "", true));
                edited |= task_row(ui, "Density", |ui| qty(ui, "density", &mut mat.density, "kg/m³", true));
                if edited && !mat.name.ends_with("(edited)") {
                    mat.name = format!("{} (edited)", mat.name);
                }
            });
            task_box(ui, "fem_mat_th", "FEM_ConstraintTemperature", "Thermal properties", |ui| {
                ui.spacing_mut().item_spacing.y = 6.0;
                task_row(ui, "Thermal conductivity", |ui| qty(ui, "conductivity", &mut mat.thermal_conductivity, "W/m/K", true));
                task_row(ui, "Expansion coefficient", |ui| qty(ui, "expansion", &mut mat.thermal_expansion, "1/K", true));
                task_row(ui, "Specific heat", |ui| qty(ui, "specific_heat", &mut mat.specific_heat, "J/kg/K", true));
                info_text(ui, "Used by thermomechanical analyses.");
            });
        });
        if mat != ma.material {
            self.fem.docs[a].analysis.materials[m].material = mat;
            self.dirty = true;
        }
        if ok {
            self.fem_task_ok();
        } else if cancel {
            self.fem_task_cancel();
        }
    }

    fn fem_mesh_task(&mut self, ui: &mut egui::Ui, a: usize) {
        let (cancel, ok) = self.task_buttons(ui, ("task:cancel", "Cancel"), Some(("task:ok", "OK")));
        let mut ms = self.fem.docs[a].analysis.mesh.clone();
        let busy = self.fem_busy();
        let mut apply = false;
        let mut stop = false;
        egui::ScrollArea::vertical().id_salt("fem_task_scroll").auto_shrink([false, true]).show(ui, |ui| {
            task_box(ui, "fem_mesh", "FEM_MeshGmshFromShape", "FEM Mesh by Gmsh", |ui| {
                ui.spacing_mut().item_spacing.y = 6.0;
                task_row(ui, "Element dimension", |ui| widgets::ro_value(ui, "3D (tetrahedra)"));
                task_row(ui, "Max element size", |ui| qty(ui, "max_size", &mut ms.max_size, "mm", !busy));
                task_row(ui, "Min element size", |ui| qty(ui, "min_size", &mut ms.min_size, "mm", !busy));
                task_row(ui, "Element order", |ui| widgets::combo(ui, "fem_order", &mut ms.order, &[(1u8, "1st (linear)"), (2u8, "2nd (quadratic)")], ui.available_width() - 2.0));
                info_text(ui, "Size 0 = automatic. Gmsh refines curved faces (12 elements per circle).");
            });
            task_box(ui, "fem_gmsh", "FEM_MeshDisplayInfo", "Gmsh", |ui| {
                ui.spacing_mut().item_spacing.y = 6.0;
                ui.horizontal(|ui| {
                    let w = (ui.available_width() - 8.0) / 2.0;
                    apply = widgets::push_button_enabled(ui, "fem:apply", "Apply", w, false, !busy).on_hover_text("Run Gmsh with these settings").clicked();
                    stop = widgets::push_button_enabled(ui, "fem:stop", "Stop", w, false, busy).clicked();
                });
                let d = &self.fem.docs[a];
                if let Some(j) = self.fem.job.as_ref().filter(|j| j.analysis == a) {
                    ui.horizontal(|ui| {
                        ui.add(egui::Spinner::new().size(14.0));
                        ui.label(format!("{} ({:.1} s)", j.stage, j.started.elapsed().as_secs_f64()));
                    });
                } else if let Some(m) = &d.rt.mesh {
                    let stale = d.rt.meshed_with.as_ref() != Some(&ms);
                    ui.label(RichText::new(format!("Nodes: {}    Elements: {}", m.node_count(), m.element_count())).color(theme::TEXT));
                    if stale {
                        ui.label(RichText::new("The settings changed: Apply to remesh").color(theme::WARNING));
                    }
                } else {
                    info_text(ui, "Not meshed yet.");
                }
                if let Some(e) = &d.rt.error {
                    error_box(ui, e);
                }
                if !d.rt.log.is_empty() {
                    log_box(ui, "fem_mesh_log", &d.rt.log, 140.0);
                }
            });
        });
        if ms != self.fem.docs[a].analysis.mesh {
            self.fem.docs[a].analysis.mesh = ms;
            self.dirty = true;
        }
        if apply {
            self.fem_start(a, true);
        }
        if stop {
            if let Some(j) = &self.fem.job {
                j.cancel.store(true, std::sync::atomic::Ordering::Relaxed);
            }
        }
        if ok {
            ui.ctx().memory_mut(|m| m.stop_text_input());
            self.fem_task_ok();
        } else if cancel {
            self.fem_task_cancel();
        }
    }

    fn fem_solver_task(&mut self, ui: &mut egui::Ui, a: usize) {
        let (close, _) = self.task_buttons(ui, ("task:close", "Close"), None);
        let busy = self.fem_busy();
        let mut at = self.fem.docs[a].analysis.analysis_type.clone();
        let mut run = false;
        let mut stop = false;
        egui::ScrollArea::vertical().id_salt("fem_task_scroll").auto_shrink([false, true]).show(ui, |ui| {
            task_box(ui, "fem_solver", "FEM_SolverControl", "CalculiX", |ui| {
                ui.spacing_mut().item_spacing.y = 6.0;
                ui.label(RichText::new("Analysis type").color(theme::TEXT));
                let mut kind = match at {
                    AnalysisType::Static => 0,
                    AnalysisType::Frequency { .. } => 1,
                    AnalysisType::Thermomech { .. } => 2,
                };
                let k0 = kind;
                ui.add_enabled_ui(!busy, |ui| {
                    for (i, (icon, label)) in [("fem-solver-analysis-static", "Static"), ("fem-solver-analysis-frequency", "Frequency"), ("fem-solver-analysis-thermomechanical", "Thermo mechanical")].iter().enumerate() {
                        ui.horizontal(|ui| {
                            let r = ui.radio_value(&mut kind, i, "");
                            widgets::reg(format!("fem:type:{label}"), r.rect);
                            ui.add(crate::icons::image(ui.ctx(), icon, 16.0));
                            ui.label(*label);
                        });
                    }
                });
                if kind != k0 {
                    at = match kind {
                        0 => AnalysisType::Static,
                        1 => AnalysisType::Frequency { modes: 10, low_hz: 0.0, high_hz: 1_000_000.0 },
                        _ => AnalysisType::Thermomech { steady_state: true },
                    };
                }
                match &mut at {
                    AnalysisType::Frequency { modes, low_hz, high_hz } => {
                        let mut n = *modes as f64;
                        task_row(ui, "Eigenmodes", |ui| qty(ui, "modes", &mut n, "", !busy));
                        *modes = n.round().clamp(1.0, 100.0) as u32;
                        task_row(ui, "Low limit", |ui| qty(ui, "low_hz", low_hz, "Hz", !busy));
                        task_row(ui, "High limit", |ui| qty(ui, "high_hz", high_hz, "Hz", !busy));
                    }
                    AnalysisType::Thermomech { steady_state } => {
                        check(ui, "Steady state", steady_state, "Steady state", "Solve for the final temperature distribution");
                    }
                    AnalysisType::Static => {}
                }
                ui.add_space(4.0);
                ui.horizontal(|ui| {
                    let w = (ui.available_width() - 8.0) / 2.0;
                    run = widgets::push_button_enabled(ui, "fem:run", "Run CalculiX", w, true, !busy).on_hover_text("Mesh with Gmsh, write Mesh.inp, run ccx and load the results").clicked();
                    stop = widgets::push_button_enabled(ui, "fem:stop", "Stop", w, false, busy).on_hover_text("Stop the running solver").clicked();
                });
            });
            task_box(ui, "fem_output", "FEM_SolverRun", "Output", |ui| {
                ui.spacing_mut().item_spacing.y = 6.0;
                let d = &self.fem.docs[a];
                if let Some(j) = self.fem.job.as_ref().filter(|j| j.analysis == a) {
                    ui.horizontal(|ui| {
                        ui.add(egui::Spinner::new().size(14.0));
                        ui.label(RichText::new(format!("{}  {:.1} s", j.stage, j.started.elapsed().as_secs_f64())).color(theme::TEXT));
                    });
                } else if let Some(e) = &d.rt.error {
                    error_box(ui, e);
                } else if d.rt.results.is_some() {
                    ui.label(RichText::new(format!("Finished in {:.1} s", d.rt.elapsed)).color(theme::GREEN));
                } else {
                    info_text(ui, "Run CalculiX to solve the analysis.");
                }
                log_box(ui, "fem_solver_log", if d.rt.log.is_empty() { "" } else { &d.rt.log }, 260.0);
            });
        });
        if at != self.fem.docs[a].analysis.analysis_type {
            self.fem.docs[a].analysis.analysis_type = at;
            self.dirty = true;
        }
        if run {
            ui.ctx().memory_mut(|m| m.stop_text_input());
            if let Some(FemTask::Solver { auto, .. }) = self.fem.task.as_mut() {
                *auto = false;
            }
            self.fem_start(a, false);
        }
        if stop {
            if let Some(j) = &self.fem.job {
                j.cancel.store(true, std::sync::atomic::Ordering::Relaxed);
            }
        }
        if close {
            self.fem_task_ok();
        }
    }

    fn fem_results_task(&mut self, ui: &mut egui::Ui, a: usize) {
        let (close, _) = self.task_buttons(ui, ("task:close", "Close"), None);
        let Some(r) = self.fem.docs[a].rt.results.clone() else {
            self.fem.task = None;
            return;
        };
        let mut v = self.fem.view.clone();
        let fields = fields_for(&r);
        egui::ScrollArea::vertical().id_salt("fem_task_scroll").auto_shrink([false, true]).show(ui, |ui| {
            task_box(ui, "fem_result", "FEM_ResultShow", "Result", |ui| {
                ui.spacing_mut().item_spacing.y = 6.0;
                if !r.modes.is_empty() {
                    let labels: Vec<String> = r.modes.iter().map(|m| format!("Mode {}: {} Hz", m.number, sig(m.frequency))).collect();
                    let opts: Vec<(usize, &str)> = labels.iter().enumerate().map(|(i, s)| (i, s.as_str())).collect();
                    task_row(ui, "Eigenmode", |ui| widgets::combo(ui, "fem_mode", &mut v.mode, &opts, ui.available_width() - 2.0));
                    check(ui, "Animate", &mut v.animate, "Animate the mode shape", "Oscillate the mode shape");
                }
                let opts: Vec<(Field, &str)> = fields.iter().map(|f| (*f, field_label(*f).0)).collect();
                task_row(ui, "Field", |ui| widgets::combo(ui, "fem_field", &mut v.field, &opts, ui.available_width() - 2.0));
                if let Some(s) = &self.fem.shown {
                    let unit = field_label(v.field).1;
                    ui.label(RichText::new(format!("Min: {} {unit}    Max: {} {unit}", sig(s.min), sig(s.max))).color(theme::TEXT));
                }
                if r.modes.is_empty() {
                    if let Some(e) = r.max_displacement() {
                        info_text(ui, &format!("Max displacement {} mm at node {}", sig(e.value), e.node));
                    }
                    if let Some(e) = r.max_von_mises() {
                        info_text(ui, &format!("Max von Mises {} MPa at node {}", sig(e.value), e.node));
                    }
                }
            });
            task_box(ui, "fem_warp", "FEM_PostFilterWarp", "Deformation", |ui| {
                ui.spacing_mut().item_spacing.y = 6.0;
                check(ui, "Automatic scale", &mut v.auto_scale, "Automatic scale", "Scale the displacements so the largest is 5 % of the part size");
                let auto = auto_scale(&r, v.mode);
                if v.auto_scale {
                    v.scale = auto;
                }
                task_row(ui, "Factor", |ui| {
                    let max = (auto * 20.0).max(1.0);
                    let rr = ui.add_enabled(!v.auto_scale, egui::Slider::new(&mut v.scale, 0.0..=max).logarithmic(true).smallest_positive(auto * 0.01).custom_formatter(|x, _| fmt_g(x)));
                    widgets::reg("fem:scale", rr.rect);
                });
                ui.horizontal(|ui| {
                    if ui.add_enabled(!v.auto_scale, egui::Button::new("True scale (1×)")).clicked() {
                        v.scale = 1.0;
                    }
                    if ui.add_enabled(!v.auto_scale, egui::Button::new("Undeformed")).clicked() {
                        v.scale = 0.0;
                    }
                });
            });
            task_box(ui, "fem_display", "FEM_MeshResult", "Display", |ui| {
                ui.spacing_mut().item_spacing.y = 6.0;
                check(ui, "Show mesh", &mut v.show_edges, "Show mesh edges", "Draw the element edges over the color map");
                check(ui, "Show max", &mut v.show_max, "Mark the maximum", "Mark the node with the largest value");
            });
        });
        if v.animate && !self.fem.view.animate {
            self.fem.anim_t0 = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0);
        }
        let changed = v.field != self.fem.view.field || v.auto_scale != self.fem.view.auto_scale || (v.scale - self.fem.view.scale).abs() > 0.0 || v.show_edges != self.fem.view.show_edges || v.mode != self.fem.view.mode || v.animate != self.fem.view.animate;
        self.fem.view = v;
        if changed {
            self.fem_refresh_view();
        }
        if close {
            self.fem_task_ok();
        }
    }

    // ------------------------------------------------------------ 3D view

    /// FEM mouse handling and drawing in the 3D view; true when it handled the input.
    pub(super) fn fem_view_input(&mut self, ui: &mut egui::Ui, ctx: &egui::Context, r: &crate::viewport::ViewportResponse) -> bool {
        if !self.fem.on || self.active_part().is_none() {
            return false;
        }
        self.fem.hover_text = None;
        let clip = Rect::from_min_max(self.view_rect.min, Pos2::new(self.view_rect.right() - self.vp.right_inset, self.view_rect.bottom()));
        let painter = ui.painter().with_clip_rect(clip);
        let showing_result = self.fem.shown.as_ref().is_some_and(|s| s.values.is_some());
        if !showing_result {
            self.fem_draw_glyphs(&painter);
        }
        if let Some(FemTask::Constraint { a, c, .. }) = self.fem.task.as_ref() {
            let (a, c) = (*a, *c);
            if r.response.clicked() {
                if let Some(p) = r.clicked.clone().filter(|p| p.object == 0) {
                    self.fem_pick(&p);
                }
            }
            // the references show as the selection
            let sel: Vec<Pick> = self.fem.docs.get(a).and_then(|d| d.analysis.constraints.get(c)).map(|con| con.refs.iter().filter_map(|rf| self.ref_pick(rf)).collect()).unwrap_or_default();
            self.vp.set_selection(sel);
            return true;
        }
        if self.fem.shown.is_some() {
            // the result / mesh replaces the body
            if showing_result {
                let modal = self.fem_cur().and_then(|a| self.fem.docs[a].rt.results.as_ref()).is_some_and(|r| !r.modes.is_empty());
                if modal && self.fem.view.animate {
                    self.fem_refresh_view();
                    ctx.request_repaint();
                }
                self.fem_draw_legend(&painter);
            }
            if let Some(p) = ctx.input(|i| i.pointer.hover_pos()).filter(|p| clip.contains(*p) && r.response.hovered()) {
                self.fem_hover(&painter, p);
            }
            if r.response.clicked() {
                self.selection.clear();
                self.vp.set_selection(vec![]);
            }
            return true;
        }
        false
    }

    /// A reference as a viewport pick on the body (for highlighting).
    fn ref_pick(&self, r: &fc_fem::SubRef) -> Option<Pick> {
        let body = self.vp.objects().first()?;
        let id = r.index.checked_sub(1)? as u32;
        Some(match r.kind {
            SubKind::Face => Pick { object: 0, kind: PickKind::Face, id, point: [0.0; 3] },
            SubKind::Edge => Pick { object: 0, kind: PickKind::Edge, id, point: [0.0; 3] },
            SubKind::Vertex => {
                let v = body.vertices.get(id as usize)?;
                Pick { object: 0, kind: PickKind::Vertex, id, point: [v[0] as f64, v[1] as f64, v[2] as f64] }
            }
            SubKind::Solid => return None,
        })
    }

    /// Value of the result at the node under the cursor (tooltip + status bar).
    fn fem_hover(&mut self, painter: &egui::Painter, p: Pos2) {
        let Some(s) = &self.fem.shown else { return };
        let Some((t, hit)) = self.vp.pick_triangle(p, s.object) else { return };
        let Some(obj) = self.vp.objects().get(s.object) else { return };
        let vs = [obj.mesh.indices[3 * t], obj.mesh.indices[3 * t + 1], obj.mesh.indices[3 * t + 2]];
        let d2 = |n: usize| {
            let q = s.positions[n];
            (q[0] - hit[0]).powi(2) + (q[1] - hit[1]).powi(2) + (q[2] - hit[2]).powi(2)
        };
        let node = vs.iter().map(|v| s.node_of_vertex[*v as usize] as usize).min_by(|a, b| d2(*a).total_cmp(&d2(*b))).unwrap();
        let Some(a) = self.fem_cur() else { return };
        let d = &self.fem.docs[a];
        let id = d.rt.results.as_ref().map(|r| r.node_ids[node]).or(d.rt.mesh.as_ref().map(|m| m.node_ids[node])).unwrap_or(0);
        let text = match &s.values {
            Some(v) => {
                let (name, unit) = field_label(self.fem.view.field);
                format!("{name}: {} {unit}\nNode {id}", sig(v[node]))
            }
            None => {
                let q = s.positions[node];
                format!("Node {id}\n({}, {}, {}) mm", sig(q[0]), sig(q[1]), sig(q[2]))
            }
        };
        if let Some(sp) = self.vp.world_to_screen(s.positions[node]) {
            painter.circle(sp, 4.0, Color32::WHITE, Stroke::new(1.5, Color32::BLACK));
        }
        let g = painter.layout_no_wrap(text.clone(), egui::FontId::proportional(13.0), Color32::WHITE);
        let size = g.size() + Vec2::new(12.0, 8.0);
        let clip = painter.clip_rect();
        let x = if p.x + 16.0 + size.x > clip.right() - 4.0 { p.x - 16.0 - size.x } else { p.x + 16.0 };
        let y = if p.y + 14.0 + size.y > clip.bottom() - 4.0 { p.y - 14.0 - size.y } else { p.y + 14.0 };
        let r = Rect::from_min_size(Pos2::new(x, y), size);
        painter.rect(r, 3.0, Color32::from_rgba_unmultiplied(20, 20, 20, 225), Stroke::new(1.0, theme::BORDER), egui::StrokeKind::Inside);
        painter.galley(r.min + Vec2::new(6.0, 4.0), g, Color32::WHITE);
        self.fem.hover_text = Some(text.replace('\n', "  "));
    }

    /// FreeCAD-style color bar on the right of the view: title, colors, value labels, and the
    /// max marker on the model.
    fn fem_draw_legend(&self, painter: &egui::Painter) {
        let Some(s) = &self.fem.shown else { return };
        let (name, unit) = field_label(self.fem.view.field);
        let vr = self.view_rect;
        let right = vr.right() - self.vp.right_inset - 78.0;
        let top = vr.top() + 215.0;
        let h = (vr.bottom() - 90.0 - top).clamp(120.0, 460.0);
        let bar = Rect::from_min_max(Pos2::new(right - 16.0, top), Pos2::new(right, top + h));
        let n = 64;
        for i in 0..n {
            let t0 = i as f32 / n as f32;
            let y0 = bar.bottom() - t0 * h;
            let y1 = bar.bottom() - (i + 1) as f32 / n as f32 * h;
            let c = colormap((i as f64 + 0.5) / n as f64);
            painter.rect_filled(Rect::from_min_max(Pos2::new(bar.left(), y1), Pos2::new(bar.right(), y0 + 0.5)), 0.0, Color32::from_rgb((c[0] * 255.0) as u8, (c[1] * 255.0) as u8, (c[2] * 255.0) as u8));
        }
        painter.rect_stroke(bar, 0.0, Stroke::new(1.0, Color32::from_gray(20)), egui::StrokeKind::Outside);
        let ticks = 10;
        let font = egui::FontId::proportional(12.0);
        let span = s.max - s.min;
        let exp = span.abs().max(s.max.abs()).max(s.min.abs());
        let sci = !(1e-2..1e5).contains(&exp) && exp > 0.0;
        for i in 0..=ticks {
            let t = i as f64 / ticks as f64;
            let v = s.min + span * t;
            let y = bar.bottom() - t as f32 * h;
            painter.line_segment([Pos2::new(bar.right(), y), Pos2::new(bar.right() + 4.0, y)], Stroke::new(1.0, Color32::from_gray(200)));
            let label = if sci { format!("{v:.3e}") } else { format!("{v:.4}") };
            painter.text(Pos2::new(bar.right() + 7.0, y), egui::Align2::LEFT_CENTER, label, font.clone(), Color32::WHITE);
        }
        let title = if unit.is_empty() { name.to_string() } else { format!("{name} ({unit})") };
        painter.text(Pos2::new(bar.right() + 60.0, bar.top() - 14.0), egui::Align2::RIGHT_BOTTOM, title, theme::bold_font(13.0), Color32::WHITE);
        if self.fem.view.mode > 0 || self.fem_cur().and_then(|a| self.fem.docs[a].rt.results.as_ref()).is_some_and(|r| !r.modes.is_empty()) {
            if let Some(m) = self.fem_cur().and_then(|a| self.fem.docs[a].rt.results.as_ref()).and_then(|r| r.modes.get(self.fem.view.mode)) {
                painter.text(Pos2::new(bar.right() + 60.0, bar.bottom() + 14.0), egui::Align2::RIGHT_TOP, format!("Mode {}: {} Hz", m.number, sig(m.frequency)), font.clone(), Color32::WHITE);
            }
        } else if s.scale != 1.0 {
            painter.text(Pos2::new(bar.right() + 60.0, bar.bottom() + 14.0), egui::Align2::RIGHT_TOP, format!("Deformation ×{}", fmt_g(s.scale)), font.clone(), Color32::from_gray(200));
        }
        // max marker
        if self.fem.view.show_max {
            if let Some(p) = s.positions.get(s.max_node).and_then(|w| self.vp.world_to_screen(*w)) {
                painter.circle(p, 5.0, Color32::from_rgb(255, 40, 40), Stroke::new(1.5, Color32::WHITE));
                let text = format!("Max {} {unit}", sig(s.max));
                let g = painter.layout_no_wrap(text, font, Color32::WHITE);
                let r = Rect::from_min_size(p + Vec2::new(10.0, -g.size().y - 12.0), g.size() + Vec2::new(10.0, 6.0));
                painter.line_segment([p, r.left_bottom()], Stroke::new(1.0, Color32::WHITE));
                painter.rect(r, 3.0, Color32::from_rgba_unmultiplied(20, 20, 20, 210), Stroke::new(1.0, Color32::from_gray(90)), egui::StrokeKind::Inside);
                painter.galley(r.min + Vec2::new(5.0, 3.0), g, Color32::WHITE);
            }
        }
    }

    /// Sample points (position, outward normal) of a reference on the body tessellation.
    fn glyph_points(&self, r: &fc_fem::SubRef, want: usize) -> Vec<([f64; 3], [f64; 3])> {
        let Some(body) = self.vp.objects().first() else { return vec![] };
        let id = r.index.saturating_sub(1) as u32;
        let p3 = |p: [f32; 3]| [p[0] as f64, p[1] as f64, p[2] as f64];
        match r.kind {
            SubKind::Face => {
                let m = &body.mesh;
                let tris: Vec<[[f64; 3]; 3]> = m.tri_face.iter().enumerate().filter(|(_, f)| **f == id).map(|(t, _)| [0, 1, 2].map(|k| p3(m.positions[m.indices[3 * t + k] as usize]))).collect();
                if tris.is_empty() {
                    return vec![];
                }
                let area = |t: &[[f64; 3]; 3]| {
                    let (u, v) = (sub3(t[1], t[0]), sub3(t[2], t[0]));
                    let c = cross(u, v);
                    0.5 * (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt()
                };
                let total: f64 = tris.iter().map(area).sum();
                // candidates: centroids of triangles split until they are small
                let mut cand: Vec<([f64; 3], [f64; 3])> = vec![];
                let mut stack: Vec<([[f64; 3]; 3], u32)> = tris.iter().map(|t| (*t, 0)).collect();
                while let Some((t, depth)) = stack.pop() {
                    let n = cross(sub3(t[1], t[0]), sub3(t[2], t[0]));
                    let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                    if l <= 1e-30 {
                        continue;
                    }
                    if area(&t) > total / 120.0 && depth < 6 && cand.len() < 4000 {
                        let m = |a: [f64; 3], b: [f64; 3]| [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5];
                        let (ab, bc, ca) = (m(t[0], t[1]), m(t[1], t[2]), m(t[2], t[0]));
                        for s in [[t[0], ab, ca], [ab, t[1], bc], [ca, bc, t[2]], [ab, bc, ca]] {
                            stack.push((s, depth + 1));
                        }
                        continue;
                    }
                    let c = [(t[0][0] + t[1][0] + t[2][0]) / 3.0, (t[0][1] + t[1][1] + t[2][1]) / 3.0, (t[0][2] + t[1][2] + t[2][2]) / 3.0];
                    cand.push((c, [n[0] / l, n[1] / l, n[2] / l]));
                }
                farthest(cand, want)
            }
            SubKind::Edge => {
                let pts: Vec<[f64; 3]> = body.edges.iter().filter(|e| e.id == id).flat_map(|e| e.points.iter().map(|p| p3(*p))).collect();
                let cand = pts.into_iter().map(|p| (p, [0.0; 3])).collect();
                farthest(cand, want.min(6))
            }
            SubKind::Vertex => body.vertices.get(id as usize).map(|v| vec![(p3(*v), [0.0; 3])]).unwrap_or_default(),
            SubKind::Solid => vec![],
        }
    }

    /// Constraint glyphs on the model like FreeCAD's: fixed = cones with blocks, forces and
    /// pressures = arrows, temperatures = small thermometers.
    fn fem_draw_glyphs(&self, painter: &egui::Painter) {
        let Some(a) = self.fem_cur() else { return };
        let d = &self.fem.docs[a];
        let view = self.vp.view_dir();
        let up = {
            // screen-up direction in world space, for edge and vertex glyphs
            let f = view;
            let z = [0.0, 0.0, 1.0];
            let r = cross(f, z);
            let rl = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt();
            if rl < 1e-6 { [0.0, 1.0, 0.0] } else { normalize(cross(r, f)) }
        };
        let task_c = match &self.fem.task {
            Some(FemTask::Constraint { c, .. }) => Some(*c),
            _ => None,
        };
        let sel_c = match self.fem.sel {
            Some(FemItem::Constraint(x, c)) if x == a => Some(c),
            _ => None,
        };
        for (ci, con) in d.analysis.constraints.iter().enumerate() {
            let hl = task_c == Some(ci) || sel_c == Some(ci);
            let k = CKind::of(&con.kind);
            let base = if matches!(k, CKind::Temperature | CKind::HeatFlux | CKind::InitialTemperature) { GLYPH_THERMAL } else { GLYPH_RED };
            let col = if hl { SEL_GREEN } else { base };
            if let ConstraintKind::SelfWeight { direction, .. } = &con.kind {
                // one large arrow at the middle of the part
                if let Some(bb) = self.doc.result().map(|s| s.bbox()) {
                    let c = [(bb[0] + bb[3]) * 0.5, (bb[1] + bb[4]) * 0.5, (bb[2] + bb[5]) * 0.5];
                    let len = 70.0 / self.vp.pixels_per_unit_at(c);
                    let dn = normalize(*direction);
                    let tail = [c[0] - dn[0] * len * 0.5, c[1] - dn[1] * len * 0.5, c[2] - dn[2] * len * 0.5];
                    self.arrow(painter, tail, dn, len, col);
                }
                continue;
            }
            for r in &con.refs {
                let want = match r.kind {
                    SubKind::Face => if matches!(k, CKind::Fixed) { 9 } else { 12 },
                    _ => 5,
                };
                for (p, n0) in self.glyph_points(r, want) {
                    let on_face = r.kind == SubKind::Face;
                    if on_face && dot(n0, view) > 0.2 {
                        continue; // faces turned away from the camera
                    }
                    let n = if on_face { n0 } else { up };
                    let s = 1.0 / self.vp.pixels_per_unit_at(p);
                    match &con.kind {
                        ConstraintKind::Fixed => {
                            // cone with its tip on the face and a block on its base
                            let l = 16.0 * s;
                            self.cone(painter, p, n, l, 6.0 * s, col);
                            let c = add(p, scale(n, l + 3.5 * s));
                            self.block(painter, c, n, 7.0 * s, col);
                        }
                        ConstraintKind::Displacement { x, y, z } => {
                            for (i, v) in [x, y, z].iter().enumerate() {
                                if v.is_some() {
                                    let mut ax = [0.0; 3];
                                    ax[i] = if dot(n, [(i == 0) as u8 as f64, (i == 1) as u8 as f64, (i == 2) as u8 as f64]) >= 0.0 { 1.0 } else { -1.0 };
                                    self.cone(painter, p, ax, 14.0 * s, 5.0 * s, col);
                                }
                            }
                        }
                        ConstraintKind::Force { direction, .. } => {
                            let dir = match direction {
                                ForceDirection::Vector(v) => normalize(*v),
                                ForceDirection::Normal { reversed } => {
                                    if *reversed { scale(n, -1.0) } else { n }
                                }
                            };
                            let l = 30.0 * s;
                            // arrows end on the face when they push into it, else start there
                            if on_face && dot(dir, n) < 0.0 {
                                self.arrow(painter, add(p, scale(dir, -l)), dir, l, col);
                            } else {
                                self.arrow(painter, p, dir, l, col);
                            }
                        }
                        ConstraintKind::Pressure { reversed, .. } => {
                            let l = 26.0 * s;
                            if *reversed {
                                self.arrow(painter, p, n, l, col);
                            } else {
                                let dir = scale(n, -1.0);
                                self.arrow(painter, add(p, scale(n, l)), dir, l, col);
                            }
                        }
                        ConstraintKind::HeatFlux(_) => {
                            let l = 24.0 * s;
                            let dir = scale(n, -1.0);
                            self.arrow(painter, add(p, scale(n, l)), dir, l, col);
                        }
                        ConstraintKind::SelfWeight { .. } => {}
                        ConstraintKind::Temperature { .. } | ConstraintKind::HeatSource { .. } | ConstraintKind::InitialTemperature { .. } => {
                            // thermometer: stem and bulb
                            let top = add(p, scale(n, 18.0 * s));
                            if let (Some(a), Some(b)) = (self.vp.world_to_screen(p), self.vp.world_to_screen(top)) {
                                painter.line_segment([a, b], Stroke::new(3.0, col));
                                painter.circle(b, 4.5, col, Stroke::new(1.0, Color32::from_gray(40)));
                            }
                        }
                    }
                }
            }
        }
    }

    fn proj(&self, w: [f64; 3]) -> Option<Pos2> {
        self.vp.world_to_screen(w)
    }

    /// Cone with its apex at `apex`, opening along `dir` (shaded silhouette).
    fn cone(&self, painter: &egui::Painter, apex: [f64; 3], dir: [f64; 3], len: f64, radius: f64, col: Color32) {
        let d = normalize(dir);
        let (u, v) = basis(d);
        let base = add(apex, scale(d, len));
        let mut pts = vec![];
        let mut ring = vec![];
        for i in 0..16 {
            let t = i as f64 / 16.0 * std::f64::consts::TAU;
            let q = add(base, add(scale(u, radius * t.cos()), scale(v, radius * t.sin())));
            if let Some(s) = self.proj(q) {
                pts.push(s);
                ring.push(s);
            }
        }
        let Some(a) = self.proj(apex) else { return };
        pts.push(a);
        let hull = convex_hull(pts);
        if hull.len() < 3 {
            return;
        }
        painter.add(egui::Shape::convex_polygon(hull, col, Stroke::new(1.0, darker(col))));
        // the base disc, lighter, when it faces the camera
        let view = self.vp.view_dir();
        if dot(d, view) < 0.0 && ring.len() >= 3 {
            painter.add(egui::Shape::convex_polygon(convex_hull(ring), lighter(col), Stroke::new(1.0, darker(col))));
        }
    }

    /// Small cube centered at `c`, aligned with `n`.
    fn block(&self, painter: &egui::Painter, c: [f64; 3], n: [f64; 3], size: f64, col: Color32) {
        let (u, v) = basis(n);
        let h = size * 0.5;
        let mut pts = vec![];
        for (a, b, z) in [(-1.0, -1.0, -1.0), (1.0, -1.0, -1.0), (1.0, 1.0, -1.0), (-1.0, 1.0, -1.0), (-1.0, -1.0, 1.0), (1.0, -1.0, 1.0), (1.0, 1.0, 1.0), (-1.0, 1.0, 1.0)] {
            let q = add(c, add(add(scale(u, a * h), scale(v, b * h)), scale(n, z * h)));
            if let Some(s) = self.proj(q) {
                pts.push(s);
            }
        }
        let hull = convex_hull(pts);
        if hull.len() >= 3 {
            painter.add(egui::Shape::convex_polygon(hull, darker(col), Stroke::new(1.0, Color32::from_gray(30))));
        }
    }

    /// Arrow from `tail` along `dir` with length `len` (shaft + cone head).
    fn arrow(&self, painter: &egui::Painter, tail: [f64; 3], dir: [f64; 3], len: f64, col: Color32) {
        let d = normalize(dir);
        let tip = add(tail, scale(d, len));
        let head = len * 0.36;
        let neck = add(tip, scale(d, -head));
        if let (Some(a), Some(b)) = (self.proj(tail), self.proj(neck)) {
            painter.line_segment([a, b], Stroke::new(2.2, col));
        }
        self.cone(painter, tip, scale(d, -1.0), head, head * 0.34, col);
    }
}

fn dir_choice(d: &ForceDirection) -> (u8, bool) {
    match d {
        ForceDirection::Normal { reversed } => (0, *reversed),
        ForceDirection::Vector(v) => {
            let n = normalize(*v);
            for i in 0..3 {
                if (n[i].abs() - 1.0).abs() < 1e-9 {
                    return (i as u8 + 1, n[i] < 0.0);
                }
            }
            (4, false)
        }
    }
}

fn make_dir(choice: u8, rev: bool, old: &ForceDirection) -> ForceDirection {
    let s = if rev { -1.0 } else { 1.0 };
    match choice {
        0 => ForceDirection::Normal { reversed: rev },
        1 => ForceDirection::Vector([s, 0.0, 0.0]),
        2 => ForceDirection::Vector([0.0, s, 0.0]),
        3 => ForceDirection::Vector([0.0, 0.0, s]),
        _ => match old {
            // reverse a picked direction
            ForceDirection::Vector(v) if dir_choice(old).0 == 4 && rev => ForceDirection::Vector([-v[0], -v[1], -v[2]]),
            ForceDirection::Vector(v) => ForceDirection::Vector(*v),
            _ => ForceDirection::Vector([0.0, 0.0, -1.0]),
        },
    }
}

pub(super) fn dir_text(d: &ForceDirection) -> String {
    match d {
        ForceDirection::Normal { reversed: false } => "the face normal (outward)".into(),
        ForceDirection::Normal { reversed: true } => "the face normal (inward)".into(),
        ForceDirection::Vector(v) => {
            let n = normalize(*v);
            format!("({}, {}, {})", fmt_g((n[0] * 1e4).round() / 1e4), fmt_g((n[1] * 1e4).round() / 1e4), fmt_g((n[2] * 1e4).round() / 1e4))
        }
    }
}

/// Up to `k` well spread points: start nearest the middle, then always the farthest one.
fn farthest(cand: Vec<([f64; 3], [f64; 3])>, k: usize) -> Vec<([f64; 3], [f64; 3])> {
    if cand.len() <= k {
        return cand;
    }
    let n = cand.len() as f64;
    let mid = cand.iter().fold([0.0; 3], |a, (p, _)| [a[0] + p[0] / n, a[1] + p[1] / n, a[2] + p[2] / n]);
    let d2 = |a: [f64; 3], b: [f64; 3]| (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2);
    let first = (0..cand.len()).min_by(|&i, &j| d2(cand[i].0, mid).total_cmp(&d2(cand[j].0, mid))).unwrap();
    let mut out = vec![cand[first]];
    let mut best: Vec<f64> = cand.iter().map(|c| d2(c.0, cand[first].0)).collect();
    while out.len() < k {
        let (i, _) = best.iter().enumerate().max_by(|a, b| a.1.total_cmp(b.1)).unwrap();
        out.push(cand[i]);
        let p = cand[i].0;
        for (j, c) in cand.iter().enumerate() {
            best[j] = best[j].min(d2(c.0, p));
        }
    }
    out
}

fn convex_hull(mut pts: Vec<Pos2>) -> Vec<Pos2> {
    pts.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    pts.dedup_by(|a, b| (a.x - b.x).abs() < 1e-3 && (a.y - b.y).abs() < 1e-3);
    if pts.len() < 3 {
        return pts;
    }
    let cr = |o: Pos2, a: Pos2, b: Pos2| (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    let mut lower: Vec<Pos2> = vec![];
    for &p in &pts {
        while lower.len() >= 2 && cr(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 0.0 {
            lower.pop();
        }
        lower.push(p);
    }
    let mut upper: Vec<Pos2> = vec![];
    for &p in pts.iter().rev() {
        while upper.len() >= 2 && cr(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 0.0 {
            upper.pop();
        }
        upper.push(p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

fn darker(c: Color32) -> Color32 {
    Color32::from_rgb((c.r() as f32 * 0.55) as u8, (c.g() as f32 * 0.55) as u8, (c.b() as f32 * 0.55) as u8)
}
fn lighter(c: Color32) -> Color32 {
    Color32::from_rgb(c.r().saturating_add(60), c.g().saturating_add(60), c.b().saturating_add(60))
}
fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn normalize(a: [f64; 3]) -> [f64; 3] {
    let l = dot(a, a).sqrt();
    if l < 1e-30 { [0.0, 0.0, 1.0] } else { scale(a, 1.0 / l) }
}
/// Two unit vectors perpendicular to `n` and each other.
fn basis(n: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let a = if n[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    let u = normalize(cross(n, a));
    (u, cross(n, u))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn quantities() {
        assert_eq!(parse_qty("1000", "N"), Some(1000.0));
        assert_eq!(parse_qty("1 kN", "N"), Some(1000.0));
        assert_eq!(parse_qty("2.5kN", "N"), Some(2500.0));
        assert_eq!(parse_qty("25 mm", "mm"), Some(25.0));
        assert_eq!(parse_qty("0.1 MPa", "MPa"), Some(0.1));
        assert_eq!(parse_qty("100 kPa", "MPa"), Some(0.1));
        assert_eq!(parse_qty("1.2e-5", "1/K"), Some(1.2e-5));
        assert_eq!(parse_qty("20 °C", "K"), Some(293.15));
        assert_eq!(parse_qty("abc", "N"), None);
        assert_eq!(fmt_g(200000.0), "200000");
        assert_eq!(fmt_g(0.3), "0.3");
        assert_eq!(fmt_g(1.2e-5), "1.2e-5");
    }
    #[test]
    fn colors_and_hull() {
        assert_eq!(colormap(0.0), [0.0, 0.0, 1.0]);
        assert_eq!(colormap(1.0), [1.0, 0.0, 0.0]);
        assert_eq!(colormap(0.5), [0.0, 1.0, 0.0]);
        let h = convex_hull(vec![Pos2::new(0.0, 0.0), Pos2::new(1.0, 0.0), Pos2::new(1.0, 1.0), Pos2::new(0.0, 1.0), Pos2::new(0.5, 0.5)]);
        assert_eq!(h.len(), 4);
        assert_eq!(sig(0.2005172), "0.2005");
        assert_eq!(sig(5.91221), "5.912");
        assert_eq!(sig(1234.6), "1235");
    }
}
