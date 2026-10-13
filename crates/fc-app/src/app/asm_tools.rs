//! Assembly tools beyond joints, after FreeCAD 1.1's Assembly workbench (and Onshape where
//! FreeCAD has nothing): structured STEP export/import and STL export, interference and
//! clearance checks, the bill of materials (FreeCAD's `Assembly_CreateBom`) and exploded
//! views (FreeCAD's `Assembly_CreateView`: steps that move selected parts, a dragger, radial
//! explosion, animation and trails).

use super::assembly::{AsmSel, AsmTask};
use super::workspace::Tab;
use super::*;
use crate::theme;
use crate::widgets::{self, task_box, task_row};
use egui::{Color32, Pos2, RichText, Stroke};
use fc_assembly::{bom_csv, AssemblyId, BomItem, BomOptions, ExplodeKind, ExplodeStep, ExplodedView, Frame, InstanceId, Interference, InterferenceOptions, StepExportOptions, StepSchema};

/// The overlap of an interference shown in the 3D view.
pub struct Highlight {
    pub a: Vec<InstanceId>,
    pub b: Vec<InstanceId>,
    pub shape: fc_occt::Shape,
}

/// Bounding-box centres of the top-level instances for exploded views, valid for `placements`.
pub struct ExplodeCache {
    pub asm: AssemblyId,
    pub placements: Vec<(InstanceId, Frame)>,
    pub centers: HashMap<InstanceId, [f64; 3]>,
    pub bbox: Option<[f64; 6]>,
    /// Display placements and trail lines of the exploded view last shown.
    pub shown: HashMap<InstanceId, Frame>,
    pub lines: Vec<([f64; 3], [f64; 3])>,
}

pub struct InterfTask {
    /// Check hidden parts too.
    pub hidden: bool,
    pub results: Option<Vec<Interference>>,
    pub picked: Option<usize>,
    pub secs: f64,
    /// Clearance between two instances.
    pub a: Option<InstanceId>,
    pub b: Option<InstanceId>,
    pub clearance: Option<Result<fc_occt::Distance, String>>,
}

pub struct BomTask {
    pub before: String,
    pub opts: BomOptions,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExMode {
    Move,
    Rotate,
    Radial,
}

pub struct ExDrag {
    pub axis: usize,
    pub press: Pos2,
    pub start: f64,
}

pub struct ExplodeTask {
    pub before: String,
    pub view: usize,
    pub mode: ExMode,
    pub axis: usize,
    pub distance: f64,
    pub angle: f64,
    /// Animation time shown (0 = assembled, 1 = exploded).
    pub t: f64,
    /// Playing since this time.
    pub play: Option<f64>,
    pub trails: bool,
    pub drag: Option<ExDrag>,
}

/// Options of the assembly export dialog.
#[derive(Clone, Debug)]
pub struct AsmExportOpts {
    pub stl: bool,
    pub ap242: bool,
    pub colors: bool,
    pub visible_only: bool,
}

const AXES: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
const AXIS_COLORS: [Color32; 3] = [Color32::from_rgb(230, 60, 60), Color32::from_rgb(80, 200, 80), Color32::from_rgb(70, 120, 240)];
const AXIS_NAMES: [&str; 3] = ["X", "Y", "Z"];

fn now() -> f64 {
    std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0)
}

impl App {
    // ------------------------------------------------------------ commands

    fn tool_start(&mut self) -> Option<AssemblyId> {
        let a = self.active_assembly()?;
        if !self.idle() {
            return None;
        }
        self.asm.highlight = None;
        Some(a)
    }

    pub(super) fn start_interference(&mut self) {
        if self.tool_start().is_none() {
            return;
        }
        self.asm.task = Some(AsmTask::Interference(InterfTask { hidden: false, results: None, picked: None, secs: 0.0, a: None, b: None, clearance: None }));
        self.interf_run();
    }

    pub(super) fn start_bom(&mut self) {
        if self.tool_start().is_none() {
            return;
        }
        let before = self.state_json();
        self.asm.task = Some(AsmTask::Bom(BomTask { before, opts: BomOptions::default() }));
        self.status = "Bill of materials: edit part numbers, descriptions and materials in the table".into();
    }

    /// Exploded view task on view `view` (a new view when None).
    pub(super) fn start_explode(&mut self, view: Option<usize>) {
        let Some(a) = self.tool_start() else { return };
        let before = self.state_json();
        let Some(asm) = self.ws.assembly_mut(a) else { return };
        let view = match view.filter(|v| *v < asm.exploded_views.len()) {
            Some(v) => v,
            None => {
                let n = asm.exploded_views.len();
                let name = if n == 0 { "ExplodedView".to_string() } else { format!("ExplodedView{n:03}") };
                asm.exploded_views.push(ExplodedView { name, steps: vec![] });
                n
            }
        };
        self.asm.explode_show = None;
        self.asm.task = Some(AsmTask::Explode(ExplodeTask { before, view, mode: ExMode::Move, axis: 2, distance: 0.0, angle: 0.0, t: 1.0, play: None, trails: true, drag: None }));
        self.fields.clear();
        self.status = "Select parts (Ctrl/Cmd for several), then drag an arrow of the dragger or type a distance and Add step".into();
        self.rebuild_scene();
    }

    /// OK / Close of the tool tasks.
    pub(super) fn tool_task_ok(&mut self) {
        self.commit_focused_asm_field();
        match self.asm.task.take() {
            Some(AsmTask::Bom(t)) => {
                if self.state_json() != t.before {
                    self.undo.push(t.before);
                    self.redo.clear();
                }
            }
            Some(AsmTask::Explode(t)) => {
                if self.state_json() != t.before {
                    self.undo.push(t.before);
                    self.redo.clear();
                }
                // the new view stays on display (FreeCAD shows an exploded view while it is visible)
                let has_steps = self.asm_ref().and_then(|a| a.exploded_views.get(t.view)).is_some_and(|v| !v.steps.is_empty());
                self.asm.explode_show = has_steps.then_some((t.view, 1.0));
                self.rebuild_scene();
                self.vp.fit_all();
            }
            _ => {}
        }
        self.asm.highlight = None;
        self.vp.capture_primary = false;
        self.fields.clear();
        self.rebuild_scene();
        self.status = self.solver_message().0;
    }

    // ------------------------------------------------------ exchange

    pub(super) fn asm_export_dialog(&mut self, stl: bool) {
        if self.active_assembly().is_none() {
            return;
        }
        self.dialog = Some(Dialog::AsmExport(AsmExportOpts { stl, ap242: false, colors: true, visible_only: false }));
    }

    /// The export options window (FreeCAD asks for the STEP scheme in its export settings).
    pub(super) fn asm_export_window(&mut self, ctx: &egui::Context, o: &mut AsmExportOpts) -> bool {
        let mut keep = true;
        let mut go = false;
        let title = if o.stl { "Export assembly as STL" } else { "Export assembly as STEP" };
        egui::Window::new(title).collapsible(false).resizable(false).anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0]).min_width(340.0).show(ctx, |ui| {
            ui.spacing_mut().item_spacing.y = 6.0;
            if !o.stl {
                ui.label(RichText::new("Scheme").strong());
                for (v, label, tip) in [(false, "AP214 (automotive design)", "FreeCAD's default STEP scheme"), (true, "AP242 (managed model-based 3D engineering)", "The newer scheme with PMI support")] {
                    let r = ui.radio_value(&mut o.ap242, v, label).on_hover_text(tip);
                    widgets::reg(format!("export:{}", if v { "AP242" } else { "AP214" }), r.rect);
                }
                ui.add_space(4.0);
                let r = ui.checkbox(&mut o.colors, "Write part colors");
                widgets::reg("export:colors", r.rect);
                ui.label(RichText::new("Parts and sub-assemblies are written as STEP products with their instance names and placements.").color(theme::TEXT_DIM).small());
            } else {
                ui.label(RichText::new("The posed parts of the assembly (sub-assemblies included) as one binary STL mesh.").color(theme::TEXT_DIM));
            }
            let r = ui.checkbox(&mut o.visible_only, "Only visible parts");
            widgets::reg("export:visible", r.rect);
            ui.add_space(8.0);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if widgets::push_button(ui, "dlg:export", "Export...", 110.0, true).clicked() {
                    go = true;
                }
                if widgets::push_button(ui, "dlg:cancel", "Cancel", 90.0, false).clicked() {
                    keep = false;
                }
            });
        });
        if go {
            keep = false;
            let (name, ext) = if o.stl { ("STL mesh", "stl") } else { ("STEP with assembly structure", "step") };
            let stem = self.asm_ref().map(|a| a.name.clone()).unwrap_or_else(|| "Assembly".into());
            if let Some(p) = rfd::FileDialog::new().add_filter(name, &[ext]).set_file_name(format!("{stem}.{ext}")).save_file() {
                match self.do_asm_export(&p, o) {
                    Ok(m) => self.status = m,
                    Err(e) => self.dialog = Some(Dialog::Message(format!("Export failed: {e}"))),
                }
            }
        }
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            keep = false;
        }
        keep
    }

    /// Write the open assembly to `path` (STEP with structure, or STL).
    pub(super) fn do_asm_export(&mut self, path: &std::path::Path, o: &AsmExportOpts) -> Result<String, String> {
        let a = self.active_assembly().ok_or("no assembly is open")?;
        let ps = path.to_string_lossy().into_owned();
        if o.stl {
            let n = self.ws.export_stl(a, &ps, 0.05, o.visible_only).map_err(|e| e.to_string())?;
            Ok(format!("Exported {n} parts to {}", path.display()))
        } else {
            let opts = StepExportOptions { schema: if o.ap242 { StepSchema::Ap242 } else { StepSchema::Ap214 }, colors: o.colors, visible_only: o.visible_only };
            let st = self.ws.export_step(a, &ps, &opts).map_err(|e| e.to_string())?;
            let skipped = if st.skipped.is_empty() { String::new() } else { format!(" ({} without geometry skipped)", st.skipped.len()) };
            Ok(format!("Exported {} ({}): {} parts, {} assemblies, {} instances{skipped}", path.display(), if o.ap242 { "AP242" } else { "AP214" }, st.parts, st.assemblies, st.components))
        }
    }

    pub(super) fn import_step_dialog(&mut self) {
        if !self.idle() {
            return;
        }
        if let Some(p) = rfd::FileDialog::new().add_filter("STEP", &["step", "stp", "STEP", "STP"]).pick_file() {
            self.import_step_file(&p);
        }
    }

    /// Read a STEP file's product structure into the document and open its assembly.
    pub(super) fn import_step_file(&mut self, p: &std::path::Path) {
        if !self.idle() {
            return;
        }
        self.snapshot();
        let was = self.doc_in_ws;
        self.park();
        let r = self.ws.import_step(&p.to_string_lossy());
        if !was {
            self.unpark();
        }
        match r {
            Ok(imp) => {
                // names stay unique in the document (FreeCAD numbers clashing labels too)
                for id in imp.parts.iter() {
                    if let Some(n) = self.ws.part(*id).map(|e| e.name.clone()) {
                        self.ws.part_mut(*id).unwrap().name = String::new();
                        let u = self.unique_part_name(&n);
                        self.ws.part_mut(*id).unwrap().name = u;
                    }
                }
                for id in imp.sub_assemblies.iter().chain([&imp.assembly]) {
                    if let Some(n) = self.ws.assembly(*id).map(|e| e.name.clone()) {
                        self.ws.assembly_mut(*id).unwrap().name = String::new();
                        let u = self.unique_part_name(&n);
                        self.ws.assembly_mut(*id).unwrap().name = u;
                    }
                }
                self.dirty = true;
                self.fem_leave();
                self.switch_tab(Tab::Assembly(imp.assembly));
                self.asm.meshes.clear();
                self.rebuild_scene();
                self.vp.fit_all();
                self.status = format!("Imported {}: {} parts, {} sub-assemblies", p.display(), imp.parts.len(), imp.sub_assemblies.len());
            }
            Err(e) => {
                self.undo.pop();
                self.dialog = Some(Dialog::Message(format!("Could not import {}: {e}", p.display())));
            }
        }
    }

    // --------------------------------------------------- interference

    fn interf_run(&mut self) {
        let Some(a) = self.active_assembly() else { return };
        let hidden = matches!(&self.asm.task, Some(AsmTask::Interference(t)) if t.hidden);
        let t0 = std::time::Instant::now();
        let list = self.ws.interferences(a, &InterferenceOptions { visible_only: !hidden, ..Default::default() });
        let secs = t0.elapsed().as_secs_f64();
        self.status = match list.len() {
            0 => "No interference: no two parts overlap".into(),
            n => format!("{n} interfering pair{}: click one to show the overlap", if n == 1 { "" } else { "s" }),
        };
        if let Some(AsmTask::Interference(t)) = self.asm.task.as_mut() {
            t.results = Some(list);
            t.picked = None;
            t.secs = secs;
        }
        self.asm.highlight = None;
        self.rebuild_scene();
    }

    fn interf_show(&mut self, i: usize) {
        let Some(AsmTask::Interference(t)) = self.asm.task.as_mut() else { return };
        let Some(x) = t.results.as_ref().and_then(|r| r.get(i)) else { return };
        t.picked = Some(i);
        self.asm.highlight = Some(Highlight { a: x.a.path.clone(), b: x.b.path.clone(), shape: x.shape.clone() });
        self.status = format!("{} and {} overlap by {} mm³", x.a.name, x.b.name, super::fem::sig(x.volume));
        self.rebuild_scene();
    }

    /// A click on a part in the interference task: the two parts of the clearance check.
    pub(super) fn interf_pick(&mut self, inst: Option<InstanceId>) {
        let Some(inst) = inst else { return };
        // measuring: the parts show selected again instead of the overlap
        self.asm.highlight = None;
        let Some(AsmTask::Interference(t)) = self.asm.task.as_mut() else { return };
        t.picked = None;
        if t.a.is_none() || t.b.is_some() {
            t.a = Some(inst);
            t.b = None;
            t.clearance = None;
            self.asm.sel = vec![AsmSel::Inst(inst)];
            self.status = format!("Clearance from {}: click a second part", self.inst_name(inst));
        } else if t.a != Some(inst) {
            t.b = Some(inst);
            let a = t.a.unwrap();
            self.asm.sel = vec![AsmSel::Inst(a), AsmSel::Inst(inst)];
            self.clearance_run();
        }
        self.rebuild_scene();
    }

    fn clearance_run(&mut self) {
        let Some(asm) = self.active_assembly() else { return };
        let Some(AsmTask::Interference(t)) = self.asm.task.as_ref() else { return };
        let (Some(a), Some(b)) = (t.a, t.b) else { return };
        let r = self.ws.clearance(asm, &[a], &[b]).ok_or_else(|| "no geometry to measure".to_string());
        self.status = match &r {
            Ok(d) if d.value <= 1e-9 => format!("{} and {} touch or overlap (clearance 0)", self.inst_name(a), self.inst_name(b)),
            Ok(d) => format!("Clearance between {} and {}: {} mm", self.inst_name(a), self.inst_name(b), super::fem::sig(d.value)),
            Err(e) => e.clone(),
        };
        if let Some(AsmTask::Interference(t)) = self.asm.task.as_mut() {
            t.clearance = Some(r);
        }
    }

    pub(super) fn interf_panel(&mut self, ui: &mut egui::Ui) {
        let (close, _) = self.task_buttons(ui, ("task:close", "Close"), None);
        let Some(AsmTask::Interference(t)) = self.asm.task.as_ref() else { return };
        let (mut hidden, picked, secs, a, b) = (t.hidden, t.picked, t.secs, t.a, t.b);
        let rows: Option<Vec<(String, String, f64)>> = t.results.as_ref().map(|r| r.iter().map(|x| (x.a.name.clone(), x.b.name.clone(), x.volume)).collect());
        let clr = t.clearance.as_ref().map(|r| r.as_ref().map(|d| d.value).map_err(|e| e.clone()));
        let insts: Vec<(InstanceId, String)> = self.asm_ref().map(|x| x.instances.iter().map(|i| (i.id, i.name.clone())).collect()).unwrap_or_default();
        let (mut ca, mut cb) = (a.unwrap_or(0), b.unwrap_or(0));
        let (mut run, mut show, mut measure) = (false, None, false);
        egui::ScrollArea::vertical().id_salt("interf_scroll").auto_shrink([false, true]).show(ui, |ui| {
            task_box(ui, "interf", "Part_Common", "Interference", |ui| {
                ui.spacing_mut().item_spacing.y = 6.0;
                let r = ui.checkbox(&mut hidden, "Include hidden parts");
                widgets::reg("check:interf_hidden", r.rect);
                if r.changed() {
                    run = true;
                }
                run |= widgets::push_button(ui, "interf:check", "Check interferences", ui.available_width() - 4.0, false).on_hover_text("Intersect every pair of parts (sub-assemblies included) and list the overlapping ones").clicked();
                match &rows {
                    None => {
                        ui.label(RichText::new("Not checked yet").color(theme::TEXT_DIM));
                    }
                    Some(rows) if rows.is_empty() => {
                        ui.horizontal_wrapped(|ui| {
                            ui.add(crate::icons::image(ui.ctx(), "edit_OK", 16.0));
                            ui.label(RichText::new("No interference: no two parts overlap").color(theme::GREEN));
                        });
                    }
                    Some(rows) => {
                        ui.horizontal_wrapped(|ui| {
                            ui.add(crate::icons::image(ui.ctx(), "Warning", 16.0));
                            ui.label(RichText::new(format!("{} interfering pair{} ({secs:.2} s)", rows.len(), if rows.len() == 1 { "" } else { "s" })).color(theme::WARNING));
                        });
                        egui::Frame::new().fill(theme::INPUT).stroke(Stroke::new(1.0, theme::BORDER)).corner_radius(3.0).inner_margin(3.0).show(ui, |ui| {
                            ui.set_width(ui.available_width());
                            for (i, (na, nb, v)) in rows.iter().enumerate() {
                                let w = ui.available_width();
                                let (rect, resp) = ui.allocate_exact_size(egui::vec2(w, 22.0), egui::Sense::click());
                                widgets::reg(format!("interf:{i}"), rect);
                                let fill = if picked == Some(i) { theme::SELECTION } else if resp.hovered() { theme::HOVER_ROW } else { Color32::TRANSPARENT };
                                ui.painter().rect_filled(rect, 2.0, fill);
                                ui.painter().circle_filled(egui::pos2(rect.left() + 9.0, rect.center().y), 4.0, Color32::from_rgb(240, 40, 40));
                                ui.painter().text(egui::pos2(rect.left() + 20.0, rect.center().y), egui::Align2::LEFT_CENTER, format!("{na} ↔ {nb}"), egui::FontId::proportional(13.0), theme::TEXT);
                                ui.painter().text(egui::pos2(rect.right() - 4.0, rect.center().y), egui::Align2::RIGHT_CENTER, format!("{} mm³", super::fem::sig(*v)), egui::FontId::proportional(13.0), theme::TEXT_DIM);
                                if resp.on_hover_text("Show the overlapping volume in red").clicked() {
                                    show = Some(i);
                                }
                            }
                        });
                    }
                }
            });
            task_box(ui, "clearance", "Assembly_CreateJointDistance", "Clearance", |ui| {
                ui.spacing_mut().item_spacing.y = 6.0;
                ui.label(RichText::new("Click two parts in the 3D view, or choose them:").color(Color32::from_gray(185)));
                let mut opts: Vec<(InstanceId, &str)> = vec![(0, "—")];
                opts.extend(insts.iter().map(|(i, n)| (*i, n.as_str())));
                measure |= task_row(ui, "Part 1", |ui| widgets::combo(ui, "clr_a", &mut ca, &opts, ui.available_width() - 2.0));
                measure |= task_row(ui, "Part 2", |ui| widgets::combo(ui, "clr_b", &mut cb, &opts, ui.available_width() - 2.0));
                match &clr {
                    Some(Ok(v)) if *v <= 1e-9 => {
                        ui.label(RichText::new("Touching or overlapping: clearance 0 mm").color(theme::WARNING));
                    }
                    Some(Ok(v)) => {
                        ui.label(RichText::new(format!("Minimum distance: {} mm", super::fem::sig(*v))).color(theme::TEXT).strong());
                    }
                    Some(Err(e)) => {
                        ui.label(RichText::new(e).color(theme::ERROR));
                    }
                    None => {}
                }
            });
        });
        if let Some(AsmTask::Interference(t)) = self.asm.task.as_mut() {
            t.hidden = hidden;
            if measure {
                t.a = (ca != 0).then_some(ca);
                t.b = (cb != 0).then_some(cb);
                t.clearance = None;
            }
        }
        if measure {
            self.asm.sel = [ca, cb].iter().filter(|x| **x != 0).map(|x| AsmSel::Inst(*x)).collect();
            self.clearance_run();
            self.rebuild_scene();
        }
        if run {
            self.interf_run();
        }
        if let Some(i) = show {
            self.interf_show(i);
        }
        if close {
            self.tool_task_ok();
        }
    }

    // ------------------------------------------------------------- BOM

    pub(super) fn bom_opts(&self) -> BomOptions {
        match &self.asm.task {
            Some(AsmTask::Bom(t)) => t.opts,
            _ => BomOptions::default(),
        }
    }

    /// Write the bill of materials as CSV.
    pub(super) fn bom_write_csv(&self, path: &std::path::Path) -> Result<usize, String> {
        let a = self.active_assembly().ok_or("no assembly is open")?;
        let rows = self.ws.bom(a, &self.bom_opts());
        std::fs::write(path, bom_csv(&rows)).map_err(|e| e.to_string())?;
        Ok(rows.len())
    }

    pub(super) fn bom_panel(&mut self, ui: &mut egui::Ui) {
        let (cancel, ok) = self.ok_cancel(ui);
        let Some(AsmTask::Bom(t)) = self.asm.task.as_ref() else { return };
        let mut o = t.opts;
        let mut csv = false;
        task_box(ui, "bom", "Assembly_BillOfMaterials", "Bill of Materials", |ui| {
            ui.spacing_mut().item_spacing.y = 6.0;
            let r = ui.checkbox(&mut o.detail_sub_assemblies, "Sub-assemblies children").on_hover_text("Includes children of sub-assemblies in the bill of materials");
            widgets::reg("check:bom_children", r.rect);
            let r = ui.checkbox(&mut o.flatten, "Total per part (flat)").on_hover_text("One row per distinct part with its total quantity in the whole assembly");
            widgets::reg("check:bom_flat", r.rect);
            let r = ui.checkbox(&mut o.visible_only, "Only visible parts");
            widgets::reg("check:bom_visible", r.rect);
            ui.add_space(4.0);
            csv = widgets::push_button(ui, "bom:csv", "Export CSV...", ui.available_width() - 4.0, false).on_hover_text("Save the table as a CSV spreadsheet").clicked();
            ui.add_space(4.0);
            ui.label(RichText::new("Part number, description and material are part properties: edit them in the table. The mass comes from the material's density.").color(Color32::from_gray(185)));
        });
        if let Some(AsmTask::Bom(t)) = self.asm.task.as_mut() {
            t.opts = o;
        }
        self.bom_window(ui.ctx());
        if csv {
            let stem = self.asm_ref().map(|a| a.name.clone()).unwrap_or_default();
            if let Some(p) = rfd::FileDialog::new().add_filter("CSV", &["csv"]).set_file_name(format!("{stem}_BOM.csv")).save_file() {
                match self.bom_write_csv(&p) {
                    Ok(n) => self.status = format!("Wrote {n} rows to {}", p.display()),
                    Err(e) => self.dialog = Some(Dialog::Message(format!("Could not write the CSV: {e}"))),
                }
            }
        }
        if ok {
            self.tool_task_ok();
        } else if cancel {
            self.asm_task_cancel();
        }
    }

    /// The table (FreeCAD shows the BOM as a spreadsheet): a window over the 3D view.
    fn bom_window(&mut self, ctx: &egui::Context) {
        let Some(a) = self.active_assembly() else { return };
        let rows = self.ws.bom(a, &self.bom_opts());
        let mats: Vec<String> = fc_fem::Material::library().into_iter().map(|m| m.name).collect();
        let vr = self.view_rect;
        let mut edits: Vec<(fc_assembly::PartId, &'static str, String)> = vec![];
        let total: f64 = rows.iter().filter(|r| r.level == 0).filter_map(|r| r.total_mass()).sum();
        let all_known = rows.iter().filter(|r| r.level == 0).all(|r| r.unit_mass.is_some());
        egui::Window::new("Bill of Materials")
            .id(egui::Id::new("bom_window"))
            .collapsible(false)
            .resizable(true)
            .default_pos(egui::pos2(vr.left() + 12.0, vr.bottom() - 300.0))
            .default_size(egui::vec2((vr.width() - tasks::TASK_W - 24.0).max(400.0), 250.0))
            .show(ctx, |ui| {
                egui::ScrollArea::both().auto_shrink([false, false]).show(ui, |ui| {
                    let mut seen = std::collections::HashSet::new();
                    egui::Grid::new("bom_grid").striped(true).spacing(egui::vec2(10.0, 4.0)).show(ui, |ui| {
                        for h in ["Index", "Name", "Part Number", "Description", "Material", "Qty", "Mass (kg)", "Total (kg)"] {
                            ui.label(RichText::new(h).strong().color(theme::TEXT));
                        }
                        ui.end_row();
                        for r in &rows {
                            ui.label(RichText::new(&r.index).color(theme::TEXT_DIM));
                            let icon = if matches!(r.item, BomItem::Assembly(_)) { "Geoassembly" } else { "PartDesign_Body" };
                            ui.horizontal(|ui| {
                                ui.add_space(14.0 * r.level as f32);
                                ui.add(crate::icons::image(ui.ctx(), icon, 16.0));
                                ui.label(RichText::new(&r.name).color(theme::TEXT));
                            });
                            let first = match r.item {
                                BomItem::Part(p) => seen.insert(p),
                                _ => false,
                            };
                            match r.item {
                                // a part listed again (inside a sub-assembly) shows its values read-only
                                BomItem::Part(_) if !first => {
                                    ui.label(RichText::new(&r.part_number).color(theme::TEXT_DIM));
                                    ui.label(RichText::new(&r.description).color(theme::TEXT_DIM));
                                    ui.label(RichText::new(&r.material).color(theme::TEXT_DIM));
                                }
                                BomItem::Part(p) => {
                                    for (field, val, w) in [("part_number", &r.part_number, 90.0), ("description", &r.description, 130.0)] {
                                        let mut buf = val.clone();
                                        let resp = ui.add(egui::TextEdit::singleline(&mut buf).hint_text("—").desired_width(w).margin(egui::Margin::symmetric(4, 1)));
                                        widgets::reg(format!("bom:{}:{field}", r.name), resp.rect);
                                        if resp.changed() {
                                            edits.push((p, field, buf));
                                        }
                                    }
                                    let mut m = r.material.clone();
                                    let mut names: Vec<String> = mats.clone();
                                    if !m.is_empty() && !names.contains(&m) {
                                        names.push(m.clone());
                                    }
                                    let mut opts2: Vec<(String, &str)> = vec![(String::new(), "—")];
                                    opts2.extend(names.iter().map(|n| (n.clone(), n.as_str())));
                                    if widgets::combo(ui, &format!("bom_mat:{}", r.name), &mut m, &opts2, 140.0) {
                                        edits.push((p, "material", m));
                                    }
                                }
                                BomItem::Assembly(_) => {
                                    ui.label("");
                                    ui.label(RichText::new("Sub-assembly").color(theme::TEXT_DIM));
                                    ui.label("");
                                }
                            }
                            ui.label(RichText::new(r.quantity.to_string()).color(theme::TEXT));
                            ui.label(RichText::new(r.unit_mass.map(|m| format!("{m:.4}")).unwrap_or_else(|| "—".into())).color(theme::TEXT));
                            ui.label(RichText::new(r.total_mass().map(|m| format!("{m:.4}")).unwrap_or_else(|| "—".into())).color(theme::TEXT));
                            ui.end_row();
                        }
                        if !rows.is_empty() {
                            for _ in 0..6 {
                                ui.label("");
                            }
                            ui.label(RichText::new("Total").strong());
                            ui.label(RichText::new(if all_known { format!("{total:.4}") } else { format!("≥ {total:.4}") }).strong());
                            ui.end_row();
                        }
                    });
                });
            });
        if !edits.is_empty() {
            for (p, field, v) in edits {
                if let Some(e) = self.ws.part_mut(p) {
                    match field {
                        "part_number" => e.meta.part_number = v,
                        "description" => e.meta.description = v,
                        _ => {
                            // the card's density gives the mass (FreeCAD materials carry it too)
                            e.meta.density = fc_fem::Material::by_name(&v).map(|m| m.density);
                            e.meta.material = v;
                        }
                    }
                }
            }
            self.dirty = true;
        }
    }

    // ---------------------------------------------------- exploded views

    /// Bounding-box centres (instance coordinates) and the assembled box, cached.
    fn explode_geom(&mut self, a: AssemblyId) -> (HashMap<InstanceId, [f64; 3]>, Option<[f64; 6]>) {
        let pls = self.placements();
        if let Some(c) = &self.asm.explode_cache {
            if c.asm == a && c.placements == pls {
                return (c.centers.clone(), c.bbox);
            }
        }
        let mut centers = HashMap::new();
        let mut bbox: Option<[f64; 6]> = None;
        for (id, pl) in &pls {
            let Some(s) = self.ws.instance_shape(a, &[*id]) else { continue };
            let b = s.bbox_exact();
            centers.insert(*id, pl.inverse().transform_point([(b[0] + b[3]) * 0.5, (b[1] + b[4]) * 0.5, (b[2] + b[5]) * 0.5]));
            bbox = Some(match bbox {
                None => b,
                Some(u) => [u[0].min(b[0]), u[1].min(b[1]), u[2].min(b[2]), u[3].max(b[3]), u[4].max(b[4]), u[5].max(b[5])],
            });
        }
        self.asm.explode_cache = Some(ExplodeCache { asm: a, placements: pls, centers: centers.clone(), bbox, shown: HashMap::new(), lines: vec![] });
        (centers, bbox)
    }

    fn selected_insts(&self) -> Vec<InstanceId> {
        self.asm.sel.iter().filter_map(|s| if let AsmSel::Inst(i) = s { Some(*i) } else { None }).collect()
    }

    /// The step the explode task is about to add (typed or being dragged).
    fn pending_step(&self) -> Option<ExplodeStep> {
        let Some(AsmTask::Explode(t)) = &self.asm.task else { return None };
        let ids = self.selected_insts();
        if ids.is_empty() {
            return None;
        }
        let ax = AXES[t.axis];
        match t.mode {
            ExMode::Move if t.distance.abs() > 1e-9 => Some(ExplodeStep::translate(ids, ax, t.distance)),
            ExMode::Radial if t.distance.abs() > 1e-9 => Some(ExplodeStep::radial(ids, t.distance)),
            ExMode::Rotate if t.angle.abs() > 1e-9 => {
                let c = self.selection_center()?;
                Some(ExplodeStep::rotate(ids, c, ax, t.angle))
            }
            _ => None,
        }
    }

    /// World centre of the selected instances as currently shown.
    fn selection_center(&self) -> Option<[f64; 3]> {
        let c = self.asm.explode_cache.as_ref()?;
        let asm = self.asm_ref()?;
        let pts: Vec<[f64; 3]> = self
            .selected_insts()
            .iter()
            .filter_map(|id| {
                let pl = c.shown.get(id).copied().or_else(|| asm.instance(*id).map(|i| i.placement))?;
                Some(pl.transform_point(c.centers.get(id).copied().unwrap_or([0.0; 3])))
            })
            .collect();
        if pts.is_empty() {
            return None;
        }
        let n = pts.len() as f64;
        Some(pts.iter().fold([0.0; 3], |a, p| [a[0] + p[0] / n, a[1] + p[1] / n, a[2] + p[2] / n]))
    }

    /// Display placements of the top-level instances: the exploded view being edited (with
    /// the pending step at full travel) or the one on display; empty when assembled.
    pub(super) fn asm_display_placements(&mut self) -> HashMap<InstanceId, Frame> {
        let Some(a) = self.active_assembly() else { return HashMap::new() };
        let Some(mut asm) = self.ws.assembly(a).cloned() else { return HashMap::new() };
        let (view, t) = match &self.asm.task {
            Some(AsmTask::Explode(t)) => {
                let pending = self.pending_step();
                let tt = if pending.is_some() { 1.0 } else { t.t };
                if let (Some(s), Some(v)) = (pending, asm.exploded_views.get_mut(t.view)) {
                    v.steps.push(s);
                }
                (t.view, tt)
            }
            _ => match self.asm.explode_show {
                Some(x) => x,
                None => {
                    if let Some(c) = self.asm.explode_cache.as_mut() {
                        c.shown.clear();
                        c.lines.clear();
                    }
                    return HashMap::new();
                }
            },
        };
        let (centers, bbox) = self.explode_geom(a);
        let ex = asm.explode_with(view, t, &|id| centers.get(&id).copied(), bbox);
        if let Some(c) = self.asm.explode_cache.as_mut() {
            c.shown = ex.placements.clone();
            c.lines = ex.lines.clone();
        }
        ex.placements
    }

    /// Dragger axes: (centre, tip of X, Y, Z) on screen.
    fn dragger(&self) -> Option<(Pos2, [Pos2; 3])> {
        if !matches!(self.asm.task, Some(AsmTask::Explode(_))) {
            return None;
        }
        let c = self.selection_center()?;
        let len = 75.0 / self.vp.pixels_per_unit_at(c);
        let p0 = self.vp.world_to_screen(c)?;
        let tips = [0, 1, 2].map(|k| self.vp.world_to_screen([c[0] + AXES[k][0] * len, c[1] + AXES[k][1] * len, c[2] + AXES[k][2] * len]).unwrap_or(p0));
        Some((p0, tips))
    }

    /// Button press in the 3D view during the explode task: grab a dragger arrow.
    pub(super) fn explode_pre_view(&mut self, ctx: &egui::Context) -> bool {
        let Some((p0, tips)) = self.dragger() else { return false };
        let Some(pos) = ctx.input(|i| i.pointer.interact_pos()) else { return false };
        let hit = (0..3).filter(|k| (tips[*k] - p0).length() > 4.0).find(|k| (tips[*k] - pos).length() < 12.0 || seg_dist(pos, p0 + (tips[*k] - p0) * 0.35, tips[*k]) < 5.0);
        let Some(axis) = hit else { return false };
        if let Some(AsmTask::Explode(t)) = self.asm.task.as_mut() {
            if t.mode != ExMode::Move || t.axis != axis {
                t.distance = 0.0;
            }
            t.mode = ExMode::Move;
            t.axis = axis;
            t.drag = Some(ExDrag { axis, press: pos, start: t.distance });
        }
        self.fields.retain(|k, _| !k.starts_with("asm:explode"));
        self.vp.capture_primary = true;
        true
    }

    /// Dragging along a dragger axis (true while it owns the mouse); also runs the animation.
    pub(super) fn explode_view_input(&mut self, ctx: &egui::Context) -> bool {
        let Some(AsmTask::Explode(t)) = self.asm.task.as_ref() else { return false };
        if let Some(start) = t.play {
            let n = self.asm_ref().and_then(|a| a.exploded_views.get(t.view)).map(|v| v.steps.len()).unwrap_or(1).max(1) as f64;
            let tt = ((now() - start) / (0.8 * n)).min(1.0);
            if let Some(AsmTask::Explode(t)) = self.asm.task.as_mut() {
                t.t = tt;
                if tt >= 1.0 {
                    t.play = None;
                }
            }
            self.rebuild_scene();
            ctx.request_repaint();
        }
        let Some(AsmTask::Explode(t)) = self.asm.task.as_ref() else { return false };
        let Some(d) = &t.drag else { return false };
        let (axis, press, start) = (d.axis, d.press, d.start);
        if ctx.input(|i| i.pointer.primary_released() || !i.pointer.primary_down()) {
            self.vp.capture_primary = false;
            if let Some(AsmTask::Explode(t)) = self.asm.task.as_mut() {
                t.drag = None;
            }
            // releasing the dragger makes the step (FreeCAD does the same)
            self.explode_add_step();
            return true;
        }
        if let (Some(pos), Some((p0, tips))) = (ctx.input(|i| i.pointer.interact_pos()), self.dragger()) {
            let v = tips[axis] - p0;
            let l = v.length();
            if l > 1.0 {
                let c = self.selection_center().unwrap_or([0.0; 3]);
                let len = 75.0 / self.vp.pixels_per_unit_at(c);
                // the dragger moves with the parts, so measure from where the press started
                let along = (pos - press).dot(v / l) as f64 / l as f64 * len;
                // whole millimetres, like a snapping dragger
                let d = (start + along).round();
                if let Some(AsmTask::Explode(t)) = self.asm.task.as_mut() {
                    t.distance = d;
                }
            }
        }
        self.rebuild_scene();
        ctx.request_repaint();
        true
    }

    /// Append the pending step to the view.
    pub(super) fn explode_add_step(&mut self) {
        let Some(step) = self.pending_step() else { return };
        let Some(a) = self.active_assembly() else { return };
        let Some(AsmTask::Explode(t)) = self.asm.task.as_mut() else { return };
        let view = t.view;
        t.distance = 0.0;
        t.angle = 0.0;
        t.t = 1.0;
        if let Some(v) = self.ws.assembly_mut(a).and_then(|x| x.exploded_views.get_mut(view)) {
            v.steps.push(step);
        }
        self.fields.retain(|k, _| !k.starts_with("asm:explode"));
        self.dirty = true;
        self.status = "Step added".into();
        self.rebuild_scene();
        // frame the exploded parts (FreeCAD fits after moving them too)
        self.vp.fit_all();
    }

    fn step_text(&self, s: &ExplodeStep) -> String {
        let names: Vec<String> = s.instances.iter().map(|i| self.inst_name(*i)).collect();
        let who = if names.len() > 2 { format!("{}, {} +{}", names[0], names[1], names.len() - 2) } else { names.join(", ") };
        match s.kind {
            ExplodeKind::Radial => format!("Radial {} mm: {who}", fmt_num(fc_part::placement::norm(s.transform.pos))),
            ExplodeKind::Normal => {
                let rv = s.transform.rotation_vector();
                let ang = fc_part::placement::norm(rv);
                if ang > 1e-9 {
                    let ax = fc_part::placement::scale(rv, 1.0 / ang);
                    let k = (0..3).max_by(|a, b| ax[*a].abs().total_cmp(&ax[*b].abs())).unwrap();
                    format!("Rotate {}° about {}: {who}", fmt_num(ang.to_degrees() * ax[k].signum()), AXIS_NAMES[k])
                } else {
                    let p = s.transform.pos;
                    let d = fc_part::placement::norm(p);
                    let k = (0..3).max_by(|a, b| p[*a].abs().total_cmp(&p[*b].abs())).unwrap();
                    if (p[k].abs() - d).abs() < 1e-9 { format!("Move {} mm along {}: {who}", fmt_num(p[k]), AXIS_NAMES[k]) } else { format!("Move ({}, {}, {}) mm: {who}", fmt_num(p[0]), fmt_num(p[1]), fmt_num(p[2])) }
                }
            }
        }
    }

    pub(super) fn explode_panel(&mut self, ui: &mut egui::Ui) {
        let (cancel, ok) = self.ok_cancel(ui);
        let Some(a) = self.active_assembly() else { return };
        let Some(AsmTask::Explode(t)) = self.asm.task.as_ref() else { return };
        let (view, mut mode, mut axis, mut distance, mut angle, mut tt, mut trails, playing) = (t.view, t.mode, t.axis, t.distance, t.angle, t.t, t.trails, t.play.is_some());
        let steps: Vec<String> = self.asm_ref().and_then(|x| x.exploded_views.get(view)).map(|v| v.steps.iter().map(|s| self.step_text(s)).collect()).unwrap_or_default();
        let vname = self.asm_ref().and_then(|x| x.exploded_views.get(view)).map(|v| v.name.clone()).unwrap_or_default();
        let sel: Vec<String> = self.selected_insts().iter().map(|i| self.inst_name(*i)).collect();
        let (mut add, mut del, mut play, mut changed, mut slid) = (false, None, false, false, false);
        egui::ScrollArea::vertical().id_salt("explode_scroll").auto_shrink([false, true]).show(ui, |ui| {
            task_box(ui, "explode", "Assembly_ExplodedView", &format!("Exploded View: {vname}"), |ui| {
                ui.spacing_mut().item_spacing.y = 4.0;
                egui::Frame::new().fill(theme::INPUT).stroke(Stroke::new(1.0, theme::BORDER)).corner_radius(3.0).inner_margin(3.0).show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    ui.set_min_height(60.0);
                    if steps.is_empty() {
                        ui.label(RichText::new("No steps yet").color(theme::TEXT_DIM).italics());
                    }
                    for (i, s) in steps.iter().enumerate() {
                        ui.horizontal(|ui| {
                            ui.add(crate::icons::image(ui.ctx(), "Assembly_ExplodedView", 14.0));
                            ui.label(RichText::new(format!("{}. {s}", i + 1)).color(theme::TEXT));
                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                let b = ui.add(egui::Button::image(crate::icons::image(ui.ctx(), "edit-delete", 14.0)).frame(false)).on_hover_text("Delete this step");
                                widgets::reg(format!("explode:del:{i}"), b.rect);
                                if b.clicked() {
                                    del = Some(i);
                                }
                            });
                        });
                    }
                });
            });
            task_box(ui, "explode_step", "Assembly_ExplodedViewGroup", "Add a step", |ui| {
                ui.spacing_mut().item_spacing.y = 6.0;
                let text = if sel.is_empty() { "Click parts in the 3D view (Ctrl/Cmd for several)".to_string() } else { format!("Parts: {}", sel.join(", ")) };
                ui.label(RichText::new(text).color(if sel.is_empty() { Color32::from_gray(185) } else { theme::TEXT }));
                ui.horizontal(|ui| {
                    for (m, label, tip) in [(ExMode::Move, "Move", "Translate along an axis"), (ExMode::Rotate, "Rotate", "Turn about an axis through the parts' centre"), (ExMode::Radial, "Radial", "Push the parts away from the assembly's centre (FreeCAD's Explode Radially)")] {
                        let r = ui.radio_value(&mut mode, m, label).on_hover_text(tip);
                        widgets::reg(format!("explode:mode:{label}"), r.rect);
                        changed |= r.changed();
                    }
                });
                if mode != ExMode::Radial {
                    let opts = [(0usize, "X"), (1, "Y"), (2, "Z")];
                    changed |= task_row(ui, "Axis", |ui| widgets::combo(ui, "explode_axis", &mut axis, &opts, ui.available_width() - 2.0));
                }
                if mode == ExMode::Rotate {
                    changed |= task_row(ui, "Angle", |ui| self.num_field(ui, "explode.angle", &mut angle, "°"));
                } else {
                    changed |= task_row(ui, "Distance", |ui| self.num_field(ui, "explode.distance", &mut distance, "mm"));
                }
                let can = !sel.is_empty() && if mode == ExMode::Rotate { angle.abs() > 1e-9 } else { distance.abs() > 1e-9 };
                add = widgets::push_button_enabled(ui, "explode:add", "Add step", ui.available_width() - 4.0, true, can).on_hover_text("Add the move to the view").on_disabled_hover_text("Select parts and enter a distance (or an angle) first").clicked();
                ui.label(RichText::new("Or drag an arrow of the dragger in the 3D view: releasing it adds the step.").color(Color32::from_gray(185)).small());
            });
            task_box(ui, "explode_anim", "media-playback-start", "Animation", |ui| {
                ui.spacing_mut().item_spacing.y = 6.0;
                task_row(ui, "Explode", |ui| {
                    // disabled while playing: the slider would round the running value
                    let r = ui.add_enabled(!playing, egui::Slider::new(&mut tt, 0.0..=1.0).show_value(true).custom_formatter(|v, _| format!("{v:.2}")));
                    widgets::reg("explode:t", r.rect);
                    slid |= r.changed();
                });
                ui.horizontal(|ui| {
                    if widgets::push_button(ui, "explode:t0", "Assembled", 90.0, false).on_hover_text("Show the parts assembled (t = 0)").clicked() {
                        tt = 0.0;
                        slid = true;
                    }
                    if widgets::push_button(ui, "explode:t1", "Exploded", 90.0, false).on_hover_text("Show every step done (t = 1)").clicked() {
                        tt = 1.0;
                        slid = true;
                    }
                });
                ui.horizontal(|ui| {
                    play = widgets::push_button_enabled(ui, "explode:play", if playing { "Playing..." } else { "Play" }, 90.0, false, !steps.is_empty() && !playing).on_hover_text("Animate the steps from assembled to exploded").clicked();
                    let r = ui.checkbox(&mut trails, "Trails");
                    widgets::reg("check:explode_trails", r.rect);
                });
            });
        });
        if let Some(AsmTask::Explode(t)) = self.asm.task.as_mut() {
            t.mode = mode;
            t.axis = axis;
            t.distance = distance;
            t.angle = angle;
            t.trails = trails;
            if slid {
                t.t = tt;
                t.play = None;
                changed = true;
            }
            if play {
                t.play = Some(now());
                t.t = 0.0;
            }
        }
        if let Some(i) = del {
            if let Some(v) = self.ws.assembly_mut(a).and_then(|x| x.exploded_views.get_mut(view)) {
                if i < v.steps.len() {
                    v.steps.remove(i);
                }
            }
            self.dirty = true;
            changed = true;
        }
        if add {
            self.explode_add_step();
        } else if changed || play {
            self.rebuild_scene();
        }
        if ok {
            self.tool_task_ok();
        } else if cancel {
            self.asm_task_cancel();
        }
    }

    /// Clearance line, dragger and trails over the assembly view.
    pub(super) fn draw_tools_overlay(&self, p: &egui::Painter) {
        // trails of the exploded parts' centres (dashed, like FreeCAD's explode lines)
        let trails = match &self.asm.task {
            Some(AsmTask::Explode(t)) => t.trails,
            _ => self.asm.explode_show.is_some(),
        };
        if trails {
            if let Some(c) = &self.asm.explode_cache {
                for (a, b) in &c.lines {
                    if let (Some(sa), Some(sb)) = (self.vp.world_to_screen(*a), self.vp.world_to_screen(*b)) {
                        let shapes = egui::Shape::dashed_line(&[sa, sb], Stroke::new(1.5, Color32::from_rgb(255, 200, 60)), 6.0, 4.0);
                        p.extend(shapes);
                    }
                }
            }
        }
        if let Some((p0, tips)) = self.dragger() {
            let active = match &self.asm.task {
                Some(AsmTask::Explode(t)) => t.drag.as_ref().map(|d| d.axis),
                _ => None,
            };
            p.circle_filled(p0, 4.0, Color32::from_gray(230));
            for k in 0..3 {
                let v = tips[k] - p0;
                if v.length() < 4.0 {
                    continue;
                }
                let col = if active == Some(k) { Color32::from_rgb(255, 230, 80) } else { AXIS_COLORS[k] };
                let w = if active == Some(k) { 3.5 } else { 2.5 };
                p.line_segment([p0, tips[k]], Stroke::new(w, col));
                let u = v / v.length();
                let n = egui::vec2(-u.y, u.x);
                p.add(egui::Shape::convex_polygon(vec![tips[k] + u * 12.0, tips[k] + n * 6.0, tips[k] - n * 6.0], col, Stroke::new(1.0, Color32::from_gray(30))));
                p.text(tips[k] + u * 22.0, egui::Align2::CENTER_CENTER, AXIS_NAMES[k], egui::FontId::proportional(12.0), col);
                widgets::reg(format!("dragger:{}", AXIS_NAMES[k]), egui::Rect::from_center_size(tips[k], egui::vec2(8.0, 8.0)));
            }
            if let Some(AsmTask::Explode(t)) = &self.asm.task {
                if t.drag.is_some() {
                    p.text(p0 + egui::vec2(12.0, 14.0), egui::Align2::LEFT_TOP, format!("{} mm", fmt_num(t.distance)), egui::FontId::proportional(13.0), Color32::WHITE);
                }
            }
        }
        if let Some(AsmTask::Interference(t)) = &self.asm.task {
            if let Some(Ok(d)) = &t.clearance {
                if let (Some(a), Some(b)) = (self.vp.world_to_screen(d.on_a), self.vp.world_to_screen(d.on_b)) {
                    let col = Color32::from_rgb(255, 210, 60);
                    p.line_segment([a, b], Stroke::new(2.0, col));
                    p.circle_filled(a, 4.0, col);
                    p.circle_filled(b, 4.0, col);
                    let m = a + (b - a) * 0.5;
                    let g = p.layout_no_wrap(format!("{} mm", super::fem::sig(d.value)), egui::FontId::proportional(13.0), Color32::WHITE);
                    let r = egui::Rect::from_min_size(m + egui::vec2(8.0, -g.size().y - 8.0), g.size() + egui::vec2(10.0, 6.0));
                    p.rect(r, 3.0, Color32::from_rgba_unmultiplied(20, 20, 20, 220), Stroke::new(1.0, col), egui::StrokeKind::Inside);
                    p.galley(r.min + egui::vec2(5.0, 3.0), g, Color32::WHITE);
                }
            }
        }
    }

    // ----------------------------------------------------------- tree

    /// The assembly's exploded views in the tree (eye: show it exploded).
    pub(super) fn exploded_tree_rows(&mut self, ui: &mut egui::Ui, a: AssemblyId, depth: usize, interactive: bool) {
        use super::tree::{tree_row_pub, RowSpecPub};
        let Some(views) = self.ws.assembly(a).map(|x| x.exploded_views.iter().map(|v| (v.name.clone(), v.steps.len())).collect::<Vec<_>>()) else { return };
        if views.is_empty() {
            return;
        }
        let active = self.active_assembly() == Some(a);
        let r = tree_row_pub(ui, RowSpecPub { reg: "tree:Exploded Views".into(), depth, expander: None, eye: None, icon: "Assembly_ExplodedViewGroup", badge: None, label: RichText::new("Exploded Views"), selected: false, tip: "Exploded views of the assembly".into() });
        let _ = r;
        let mut toggle = None;
        let mut edit = None;
        for (i, (name, n)) in views.iter().enumerate() {
            let shown = active && self.asm.explode_show.is_some_and(|x| x.0 == i);
            let r = tree_row_pub(ui, RowSpecPub { reg: format!("tree:{name}"), depth: depth + 1, expander: None, eye: Some(shown), icon: "Assembly_ExplodedView", badge: None, label: RichText::new(name), selected: false, tip: format!("{name}: {n} step{}\nEye: show the assembly exploded · double-click: edit", if *n == 1 { "" } else { "s" }) });
            if interactive && active {
                if r.eye {
                    toggle = Some(i);
                } else if r.double {
                    edit = Some(i);
                }
            }
        }
        if let Some(i) = toggle {
            self.asm.explode_show = if self.asm.explode_show.is_some_and(|x| x.0 == i) { None } else { Some((i, 1.0)) };
            self.rebuild_scene();
        }
        if let Some(i) = edit {
            self.start_explode(Some(i));
        }
    }
}

fn seg_dist(p: Pos2, a: Pos2, b: Pos2) -> f32 {
    let ab = b - a;
    let l2 = ab.length_sq();
    if l2 < 1e-6 {
        return (p - a).length();
    }
    let t = ((p - a).dot(ab) / l2).clamp(0.0, 1.0);
    (p - (a + ab * t)).length()
}
