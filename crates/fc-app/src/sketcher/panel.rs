//! Task panel (solver messages, constraint and element lists), toolbar and the datum popup.

use super::render::{COL_CONFLICT, COL_REDUNDANT};
use super::{geom, Cmd, EditorExit, Sel, SketchEditor, Tool};
use eframe::egui::{self, Color32, RichText};
use fc_part::params::OnUsers;
use fc_part::Document;
use fc_sketch::SolveStatus;

#[derive(Default)]
pub struct PanelState {
    /// Constraint shown in the property editor, with its edit buffers (name, value).
    editing: Option<(usize, String, String)>,
    edit_error: Option<String>,
}

fn cmd_glyph(c: Cmd) -> Option<super::render::Glyph> {
    use super::render::Glyph;
    Some(match c {
        Cmd::Coincident => Glyph::Coincident,
        Cmd::PointOnObject => Glyph::PointOn,
        Cmd::Horizontal => Glyph::H,
        Cmd::Vertical => Glyph::V,
        Cmd::Parallel => Glyph::Parallel,
        Cmd::Perpendicular => Glyph::Perpendicular,
        Cmd::Tangent => Glyph::Tangent,
        Cmd::Equal => Glyph::Equal,
        Cmd::Symmetric => Glyph::Symmetric,
        Cmd::Block => Glyph::Block,
        _ => return None,
    })
}

const GREEN: Color32 = Color32::from_rgb(60, 200, 60);

fn list_text(ints: &[usize]) -> String {
    // FreeCAD numbers constraints from 1
    if ints.len() < 8 {
        ints.iter().map(|i| (i + 1).to_string()).collect::<Vec<_>>().join(", ")
    } else {
        let head: Vec<String> = ints[..3].iter().map(|i| (i + 1).to_string()).collect();
        format!("{}, and {} more", head.join(", "), ints.len() - 3)
    }
}

impl SketchEditor {
    pub fn toolbar_ui(&mut self, doc: &mut Document, ui: &mut egui::Ui) {
        for (t, label, key) in Tool::ALL {
            if ui.selectable_label(self.tool.tool == t, label).on_hover_text(format!("Shortcut: {key}")).clicked() {
                self.set_tool(t);
            }
        }
        if ui.selectable_label(self.construction, "Construction").on_hover_text("Toggle construction geometry: for the selected curves, or the creation mode (G, N)").clicked() {
            self.toggle_construction(doc);
        }
        ui.separator();
        for (c, label, key, tip) in Cmd::ALL {
            let hover = format!("{label}: {tip}\nShortcut: {key}");
            let clicked = match cmd_glyph(c) {
                Some(g) => {
                    let (r, resp) = ui.allocate_exact_size(egui::vec2(22.0, 22.0), egui::Sense::click());
                    let fill = if resp.hovered() { Color32::from_rgb(255, 110, 70) } else { super::render::COL_DIM };
                    super::render::badge(ui.painter(), r.center(), g, fill, None);
                    resp.on_hover_text(hover).clicked()
                }
                None => ui.button(label).on_hover_text(hover).clicked(),
            };
            if clicked {
                self.run_cmd_ui(doc, c);
            }
        }
        if ui.button("Driving/Ref.").on_hover_text("Toggle the selected dimensions between driving and reference").clicked() {
            self.toggle_driving(doc);
        }
        ui.separator();
        if ui.button("Delete").on_hover_text("Delete the selection (Del)").clicked() {
            self.delete_selection(doc, OnUsers::Refuse);
        }
        if ui.add_enabled(!self.undo.is_empty(), egui::Button::new("Undo")).on_hover_text("Cmd+Z").clicked() {
            self.undo_step(doc);
        }
        if ui.add_enabled(!self.redo.is_empty(), egui::Button::new("Redo")).on_hover_text("Cmd+Shift+Z").clicked() {
            self.redo_step(doc);
        }
        if ui.button("View sketch").on_hover_text("Fit the view to the sketch").clicked() {
            self.want_fit = true;
        }
    }

    fn solver_messages(&mut self, doc: &Document, ui: &mut egui::Ui) {
        let Some(r) = self.report.clone() else { return };
        let sk = self.sk(doc);
        ui.label(RichText::new("Solver messages").strong());
        let mut select: Option<Vec<usize>> = None;
        if sk.geometry.is_empty() {
            ui.label("Empty sketch");
        } else if !r.conflicting.is_empty() || r.status == SolveStatus::Conflicting {
            ui.horizontal_wrapped(|ui| {
                ui.label(RichText::new("Over-constrained:").color(COL_CONFLICT).strong());
                if ui.link(RichText::new(format!("({})", list_text(&r.conflicting))).color(COL_CONFLICT)).on_hover_text("Click to select these constraints").clicked() {
                    select = Some(r.conflicting.clone());
                }
            });
            ui.label(RichText::new(format!("Remove at least one of the following constraints: {}", list_text(&r.conflicting))).color(COL_CONFLICT).small());
        } else if !r.malformed.is_empty() {
            ui.horizontal_wrapped(|ui| {
                ui.label(RichText::new("Malformed constraints:").color(COL_CONFLICT).strong());
                if ui.link(format!("({})", list_text(&r.malformed))).clicked() {
                    select = Some(r.malformed.clone());
                }
            });
        } else if !r.redundant.is_empty() {
            ui.horizontal_wrapped(|ui| {
                ui.label(RichText::new("Redundant constraints:").color(COL_REDUNDANT).strong());
                if ui.link(RichText::new(format!("({})", list_text(&r.redundant))).color(COL_REDUNDANT)).on_hover_text("Click to select these constraints").clicked() {
                    select = Some(r.redundant.clone());
                }
            });
            ui.label(RichText::new(format!("Remove the following redundant constraint(s): {}", list_text(&r.redundant))).color(COL_REDUNDANT).small());
        } else if !r.partially_redundant.is_empty() {
            ui.horizontal_wrapped(|ui| {
                ui.label(RichText::new("Partially redundant:").color(COL_REDUNDANT).strong());
                if ui.link(format!("({})", list_text(&r.partially_redundant))).clicked() {
                    select = Some(r.partially_redundant.clone());
                }
            });
        } else if r.status == SolveStatus::Failed {
            ui.label(RichText::new("Solver failed to converge").color(COL_CONFLICT));
        } else if r.dof > 0 {
            ui.horizontal_wrapped(|ui| {
                ui.label("Under-constrained:");
                if ui.link(format!("{} Degree{} of Freedom", r.dof, if r.dof == 1 { "" } else { "s" })).on_hover_text("Click to select the elements that can still move").clicked() {
                    let free: Vec<Sel> = (0..sk.geometry.len()).filter(|g| !self.fully.get(*g).copied().unwrap_or(false)).map(Sel::Edge).collect();
                    self.sel = free;
                }
            });
        } else {
            ui.label(RichText::new("Fully constrained").color(GREEN).strong());
        }
        if let Some(cs) = select {
            self.sel = cs.into_iter().map(Sel::Constraint).collect();
        }
        for e in &self.expr_errors {
            ui.label(RichText::new(format!("Expression error: {e}")).color(COL_CONFLICT).small());
        }
    }

    pub fn panel_ui(&mut self, doc: &mut Document, ui: &mut egui::Ui) -> EditorExit {
        let mut exit = EditorExit::None;
        self.panel_hover = None;
        ui.horizontal(|ui| {
            if ui.button(RichText::new("Close").strong()).on_hover_text("Leave the sketch, keeping the changes").clicked() {
                exit = EditorExit::Close;
            }
            if ui.button("Cancel").on_hover_text("Leave the sketch, discarding all changes").clicked() {
                exit = EditorExit::Cancel;
            }
        });
        ui.separator();
        self.solver_messages(doc, ui);
        if let Some((m, err)) = &self.message {
            ui.label(RichText::new(m).color(if *err { COL_CONFLICT } else { ui.visuals().weak_text_color() }).small());
        }

        // refused delete
        let mut freeze = false;
        let mut drop_pending = false;
        if let Some(pd) = &self.pending_delete {
            egui::Frame::group(ui.style()).show(ui, |ui| {
                ui.label(RichText::new("These dimensions are used by formulas:").color(COL_CONFLICT));
                for u in &pd.users {
                    ui.label(format!("  • {u}"));
                }
                ui.horizontal(|ui| {
                    if ui.button("Delete and keep their values").on_hover_text("The formulas are replaced by their current values").clicked() {
                        freeze = true;
                    }
                    if ui.button("Cancel").clicked() {
                        drop_pending = true;
                    }
                });
            });
        }
        if freeze {
            if let Some(pd) = self.pending_delete.take() {
                self.delete_items(doc, pd.constraints, pd.geometry, OnUsers::Freeze);
            }
        }
        if drop_pending {
            self.pending_delete = None;
        }

        ui.separator();
        ui.horizontal_wrapped(|ui| {
            ui.checkbox(&mut self.auto_constraints, "Auto constraints").on_hover_text("Snap to points/curves/axes and add coincident, point-on-object, horizontal and vertical constraints while drawing");
            ui.checkbox(&mut self.grid, "Grid snap");
            ui.checkbox(&mut self.show_constraints, "Show constraints");
        });

        ui.separator();
        self.constraint_list(doc, ui);
        ui.separator();
        self.element_list(doc, ui);
        if exit == EditorExit::Cancel {
            // the host restores the sketch; formulas and names edited meanwhile go back too
            doc.expressions = self.original_exprs.0.clone();
            doc.parameters = self.original_exprs.1.clone();
        }
        exit
    }

    fn constraint_list(&mut self, doc: &mut Document, ui: &mut egui::Ui) {
        let n = self.sk(doc).constraints.len();
        ui.label(RichText::new(format!("Constraints ({n})")).strong());
        let rep = self.report.clone();
        let multi = ui.input(|i| i.modifiers.command || i.modifiers.ctrl || i.modifiers.shift);
        let mut clicked: Option<usize> = None;
        for ci in 0..n {
            let sk = self.sk(doc);
            let c = &sk.constraints[ci];
            let name = c.name.clone().unwrap_or_else(|| format!("Constraint{}", ci + 1));
            let mut text = format!("{:>2}  {name}  ·  {}", ci + 1, geom::kind_name(&c.kind));
            if let Some(v) = geom::display_value(&c.kind) {
                let val = format!("{} {}", geom::fmt_num(v), geom::unit_of(&c.kind));
                match doc.expressions.get(&self.ckey(doc, ci)) {
                    Some(f) => text.push_str(&format!("  ƒx {f} = {val}")),
                    None if !c.driving => text.push_str(&format!("  ({val})")),
                    None => text.push_str(&format!("  {val}")),
                }
            }
            let mut rt = RichText::new(text);
            if let Some(r) = &rep {
                if r.conflicting.contains(&ci) || r.malformed.contains(&ci) {
                    rt = rt.color(COL_CONFLICT);
                } else if r.redundant.contains(&ci) || r.partially_redundant.contains(&ci) {
                    rt = rt.color(COL_REDUNDANT);
                }
            }
            if !c.driving {
                rt = rt.italics();
            }
            let selected = self.is_sel(Sel::Constraint(ci));
            let resp = ui.selectable_label(selected, rt);
            if resp.hovered() {
                self.panel_hover = Some(Sel::Constraint(ci));
            }
            if resp.clicked() {
                clicked = Some(ci);
            }
            if resp.double_clicked() && c.kind.is_dimensional() {
                self.open_dim_edit(doc, ci, false, None);
            }
        }
        if let Some(ci) = clicked {
            if multi {
                self.toggle_sel(Sel::Constraint(ci));
            } else {
                self.sel = vec![Sel::Constraint(ci)];
            }
        }

        // property editor for a single selected constraint
        let one: Vec<usize> = self.parts().cons;
        if one.len() != 1 {
            self.panel.editing = None;
            return;
        }
        let ci = one[0];
        let sk = self.sk(doc);
        let Some(c) = sk.constraints.get(ci).cloned() else { return };
        let key = self.ckey(doc, ci);
        if self.panel.editing.as_ref().map(|e| e.0) != Some(ci) {
            let value = match doc.expressions.get(&key) {
                Some(f) => format!("={f}"),
                None => geom::display_value(&c.kind).map(|v| format!("{} {}", geom::fmt_num(v), geom::unit_of(&c.kind))).unwrap_or_default(),
            };
            self.panel.editing = Some((ci, c.name.clone().unwrap_or_default(), value));
            self.panel.edit_error = None;
        }
        let mut apply = false;
        let mut toggle = false;
        let mut delete = false;
        egui::Frame::group(ui.style()).show(ui, |ui| {
            ui.label(RichText::new(format!("Constraint{}: {}", ci + 1, geom::kind_name(&c.kind))).strong());
            let (_, name, value) = self.panel.editing.as_mut().unwrap();
            if c.kind.is_dimensional() {
                egui::Grid::new("sk_cprops").num_columns(2).show(ui, |ui| {
                    ui.label("Name");
                    let r1 = ui.add(egui::TextEdit::singleline(name).desired_width(140.0).hint_text("e.g. width"));
                    ui.end_row();
                    ui.label("Value");
                    let r2 = ui.add(egui::TextEdit::singleline(value).desired_width(140.0)).on_hover_text("A number with units (25 mm, 30 deg), or a formula starting with '=' (=width/2)");
                    ui.end_row();
                    if (r1.lost_focus() || r2.lost_focus()) && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                        apply = true;
                    }
                });
                ui.horizontal(|ui| {
                    if ui.button("Apply").clicked() {
                        apply = true;
                    }
                    let mut driving = c.driving;
                    if ui.checkbox(&mut driving, "Driving").on_hover_text("Unchecked: reference dimension (only measures)").changed() {
                        toggle = true;
                    }
                });
            }
            if let Some(e) = &self.panel.edit_error {
                ui.label(RichText::new(e).color(COL_CONFLICT).small());
            }
            if ui.button("Delete constraint").clicked() {
                delete = true;
            }
        });
        if apply {
            let (_, name, value) = self.panel.editing.clone().unwrap();
            self.push_undo(doc);
            let value = if c.driving { value } else { String::new() };
            match self.apply_dimension(doc, ci, &value, Some(&name)) {
                Ok(()) => {
                    self.panel.editing = None;
                    self.panel.edit_error = None;
                }
                Err(e) => self.panel.edit_error = Some(e),
            }
        }
        if toggle {
            self.toggle_driving(doc);
            self.panel.editing = None;
        }
        if delete {
            self.delete_items(doc, vec![ci], vec![], OnUsers::Refuse);
            self.panel.editing = None;
        }
    }

    fn element_list(&mut self, doc: &mut Document, ui: &mut egui::Ui) {
        let sk = self.sk(doc);
        ui.label(RichText::new(format!("Elements ({})", sk.geometry.len())).strong());
        let multi = ui.input(|i| i.modifiers.command || i.modifiers.ctrl || i.modifiers.shift);
        let mut clicked = None;
        for (gi, g) in sk.geometry.iter().enumerate() {
            let full = self.fully.get(gi).copied().unwrap_or(false);
            let mut t = format!("{:>2}  {}  {}", gi + 1, geom::geo_name(gi), geom::curve_name(&g.curve));
            if g.construction {
                t.push_str("  (construction)");
            }
            let mut rt = RichText::new(t);
            if full && !self.invalid() {
                rt = rt.color(GREEN);
            }
            let sel = if matches!(g.curve, fc_sketch::Curve::Point { .. }) { Sel::Point(fc_sketch::PointRef::start(gi)) } else { Sel::Edge(gi) };
            let resp = ui.selectable_label(self.is_sel(sel), rt);
            if resp.hovered() {
                self.panel_hover = Some(sel);
            }
            if resp.clicked() {
                clicked = Some(sel);
            }
        }
        if let Some(s) = clicked {
            if multi {
                self.toggle_sel(s);
            } else {
                self.sel = vec![s];
            }
        }
    }

    // ------------------------------------------------------------- datum popup

    pub(crate) fn dim_edit_ui(&mut self, doc: &mut Document, ctx: &egui::Context) {
        let Some(ci) = self.dim_edit.as_ref().map(|d| d.ci) else { return };
        let kind = self.sk(doc).constraints.get(ci).map(|c| c.kind.clone());
        let Some(kind) = kind else {
            self.dim_edit = None;
            return;
        };
        let de = self.dim_edit.as_mut().unwrap();
        let title = match kind {
            fc_sketch::ConstraintKind::Radius(..) => "Insert radius",
            fc_sketch::ConstraintKind::Diameter(..) => "Insert diameter",
            fc_sketch::ConstraintKind::Angle(..) | fc_sketch::ConstraintKind::AngleBetween(..) => "Insert angle",
            _ => "Insert length",
        };
        let names: Vec<(String, f64)> = if de.value.trim_start().starts_with('=') { doc.available_names() } else { vec![] };
        let mut ok = false;
        let mut cancel = false;
        let pos = de.pos;
        egui::Area::new(egui::Id::new("sk_dim_edit")).order(egui::Order::Foreground).fixed_pos(pos).show(ctx, |ui| {
            egui::Frame::popup(ui.style()).show(ui, |ui| {
                ui.set_max_width(260.0);
                ui.label(RichText::new(title).strong());
                let vid = egui::Id::new("sk_dim_value");
                let r = ui.add(egui::TextEdit::singleline(&mut de.value).id(vid).desired_width(220.0)).on_hover_text("Number with units (25 mm, 30 deg) or a formula: =width/2");
                if de.focus {
                    de.focus = false;
                    r.request_focus();
                    if let Some(mut st) = egui::TextEdit::load_state(ctx, vid) {
                        let n = de.value.chars().count();
                        st.cursor.set_char_range(Some(egui::text::CCursorRange::two(egui::text::CCursor::new(0), egui::text::CCursor::new(n))));
                        st.store(ctx, vid);
                    }
                }
                // name completion for formulas
                if let Some(f) = de.value.trim_start().strip_prefix('=') {
                    let tail: String = f.chars().rev().take_while(|c| c.is_ascii_alphanumeric() || *c == '_' || *c == '.').collect::<Vec<_>>().into_iter().rev().collect();
                    let hits: Vec<&(String, f64)> = names.iter().filter(|(n, _)| tail.is_empty() || n.starts_with(&tail) || n.rsplit('.').next().is_some_and(|s| s.starts_with(&tail))).take(6).collect();
                    if !hits.is_empty() {
                        ui.horizontal_wrapped(|ui| {
                            for (n, v) in hits {
                                let short = n.rsplit('.').next().unwrap_or(n).to_string();
                                let ins = if names.iter().filter(|(m, _)| m.rsplit('.').next() == Some(short.as_str())).count() == 1 { short.clone() } else { n.clone() };
                                if ui.small_button(format!("{ins} = {}", geom::fmt_num(*v))).clicked() {
                                    let cut = de.value.len() - tail.len();
                                    de.value.truncate(cut);
                                    de.value.push_str(&ins);
                                    r.request_focus();
                                }
                            }
                        });
                    }
                }
                ui.horizontal(|ui| {
                    ui.label("Name");
                    let rn = ui.add(egui::TextEdit::singleline(&mut de.name).desired_width(120.0).hint_text("optional"));
                    if rn.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                        ok = true;
                    }
                    ui.checkbox(&mut de.reference, "Reference");
                });
                if let Some(e) = &de.error {
                    ui.label(RichText::new(e).color(COL_CONFLICT).small());
                }
                ui.horizontal(|ui| {
                    if ui.button("OK").clicked() {
                        ok = true;
                    }
                    if ui.button("Cancel").clicked() {
                        cancel = true;
                    }
                });
                if r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    ok = true;
                }
                if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                    cancel = true;
                }
            });
        });
        if ok {
            self.commit_dim_edit(doc);
        } else if cancel {
            self.cancel_dim_edit(doc);
        }
    }

    /// Apply the datum popup. Returns false (popup stays open with the error) on failure.
    pub(crate) fn commit_dim_edit(&mut self, doc: &mut Document) -> bool {
        let Some(mut de) = self.dim_edit.take() else { return false };
        if !de.created {
            self.push_undo(doc);
        }
        let driving_now = self.sk(doc).constraints.get(de.ci).map(|c| c.driving).unwrap_or(true);
        if driving_now == de.reference {
            if let Some(s) = doc.sketch_mut(self.feature) {
                s.constraints[de.ci].driving = !de.reference;
            }
            if de.reference {
                let key = self.ckey(doc, de.ci);
                if doc.expressions.contains_key(&key) {
                    let _ = doc.set_expression(&key, "");
                }
            }
        }
        let value = if de.reference { String::new() } else { de.value.clone() };
        if !value.is_empty() {
            for &c in &de.also {
                if let Err(e) = self.apply_dimension(doc, c, &value, None) {
                    de.error = Some(e);
                    self.dim_edit = Some(de);
                    return false;
                }
            }
        }
        match self.apply_dimension(doc, de.ci, &value, Some(&de.name)) {
            Ok(()) => {
                self.panel.editing = None;
                true
            }
            Err(e) => {
                de.error = Some(e);
                self.dim_edit = Some(de);
                false
            }
        }
    }

    /// Cancel the popup; a dimension created for it is removed again (FreeCAD aborts the command).
    pub(crate) fn cancel_dim_edit(&mut self, doc: &mut Document) {
        let Some(de) = self.dim_edit.take() else { return };
        if de.created {
            self.undo_step(doc);
            self.redo.pop();
            self.message = None;
        }
    }
}
