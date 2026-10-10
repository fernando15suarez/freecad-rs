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
    /// Constraint list filter (FreeCAD's "Filter" combo).
    filter: Filter,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
enum Filter {
    #[default]
    All,
    Geometric,
    Datums,
    Named,
    Selected,
}

const GREEN: Color32 = crate::theme::GREEN;

fn list_text(ints: &[usize]) -> String {
    // FreeCAD numbers constraints from 1
    if ints.len() < 8 {
        ints.iter().map(|i| (i + 1).to_string()).collect::<Vec<_>>().join(", ")
    } else {
        let head: Vec<String> = ints[..3].iter().map(|i| (i + 1).to_string()).collect();
        format!("{}, and {} more", head.join(", "), ints.len() - 3)
    }
}

/// FreeCAD icon of a creation tool (construction-mode variant when `constr`).
pub fn tool_icon(t: Tool, constr: bool) -> &'static str {
    match (t, constr) {
        (Tool::Select, _) => "Sketcher_EditSketch",
        (Tool::Point, _) => "Sketcher_CreatePoint",
        (Tool::Line, false) => "Sketcher_CreateLine",
        (Tool::Line, true) => "Sketcher_CreateLine_Constr",
        (Tool::Polyline, false) => "Sketcher_CreatePolyline",
        (Tool::Polyline, true) => "Sketcher_CreatePolyline_Constr",
        (Tool::Rectangle, false) => "Sketcher_CreateRectangle",
        (Tool::Rectangle, true) => "Sketcher_CreateRectangle_Constr",
        (Tool::CenterRectangle, false) => "Sketcher_CreateRectangle_Center",
        (Tool::CenterRectangle, true) => "Sketcher_CreateRectangle_Center_Constr",
        (Tool::Circle, false) => "Sketcher_CreateCircle",
        (Tool::Circle, true) => "Sketcher_CreateCircle_Constr",
        (Tool::Arc, false) => "Sketcher_CreateArc",
        (Tool::Arc, true) => "Sketcher_CreateArc_Constr",
        (Tool::Arc3, false) => "Sketcher_Create3PointArc",
        (Tool::Arc3, true) => "Sketcher_Create3PointArc_Constr",
        (Tool::Slot, false) => "Sketcher_CreateSlot",
        (Tool::Slot, true) => "Sketcher_CreateSlot_Constr",
    }
}

fn tool_title(t: Tool) -> (&'static str, &'static str) {
    match t {
        Tool::Select => ("Select", "Select and drag elements"),
        Tool::Point => ("Point", "Creates a point"),
        Tool::Line => ("Line", "Creates a line"),
        Tool::Polyline => ("Polyline", "Creates a continuous polyline (right-click or Esc ends it)"),
        Tool::Rectangle => ("Rectangle", "Creates a rectangle from two corners"),
        Tool::CenterRectangle => ("Centered Rectangle", "Creates a rectangle from its center and a corner"),
        Tool::Circle => ("Circle", "Creates a circle from its center and a rim point"),
        Tool::Arc => ("Arc From Center", "Creates an arc from its center and its end points"),
        Tool::Arc3 => ("Arc From 3 Points", "Creates an arc from its end points and a point on it"),
        Tool::Slot => ("Slot", "Creates a slot (oblong) from two centers and the width"),
    }
}

/// FreeCAD icon of a constraint command.
pub fn cmd_icon(c: Cmd) -> &'static str {
    match c {
        Cmd::Coincident => "Constraint_PointOnPoint",
        Cmd::PointOnObject => "Constraint_PointOnObject",
        Cmd::Horizontal => "Constraint_Horizontal",
        Cmd::Vertical => "Constraint_Vertical",
        Cmd::Parallel => "Constraint_Parallel",
        Cmd::Perpendicular => "Constraint_Perpendicular",
        Cmd::Tangent => "Constraint_Tangent",
        Cmd::Equal => "Constraint_EqualLength",
        Cmd::Symmetric => "Constraint_Symmetric",
        Cmd::Block => "Constraint_Block",
        Cmd::Lock => "Constraint_Lock",
        Cmd::DistanceX => "Constraint_HorizontalDistance",
        Cmd::DistanceY => "Constraint_VerticalDistance",
        Cmd::Distance => "Constraint_Length",
        Cmd::Radius => "Constraint_Radius",
        Cmd::Diameter => "Constraint_Diameter",
        Cmd::Angle => "Constraint_InternalAngle",
    }
}

fn cmd_title(c: Cmd) -> &'static str {
    match c {
        Cmd::Coincident => "Constrain Coincident",
        Cmd::PointOnObject => "Constrain Point on Object",
        Cmd::Horizontal => "Constrain Horizontal",
        Cmd::Vertical => "Constrain Vertical",
        Cmd::Parallel => "Constrain Parallel",
        Cmd::Perpendicular => "Constrain Perpendicular",
        Cmd::Tangent => "Constrain Tangent",
        Cmd::Equal => "Constrain Equal",
        Cmd::Symmetric => "Constrain Symmetric",
        Cmd::Block => "Constrain Block",
        Cmd::Lock => "Constrain Lock",
        Cmd::DistanceX => "Constrain Horizontal Distance",
        Cmd::DistanceY => "Constrain Vertical Distance",
        Cmd::Distance => "Constrain Distance",
        Cmd::Radius => "Constrain Radius",
        Cmd::Diameter => "Constrain Diameter",
        Cmd::Angle => "Constrain Angle",
    }
}

/// Icon of a constraint in the constraint list (reference dimensions use the blue variants).
pub fn constraint_icon(k: &fc_sketch::ConstraintKind, driving: bool) -> &'static str {
    use fc_sketch::ConstraintKind as K;
    let (icon, driven) = match k {
        K::Coincident(..) => ("Constraint_PointOnPoint", ""),
        K::Horizontal(_) | K::HorizontalPoints(..) => ("Constraint_Horizontal", ""),
        K::Vertical(_) | K::VerticalPoints(..) => ("Constraint_Vertical", ""),
        K::Parallel(..) => ("Constraint_Parallel", ""),
        K::Perpendicular(..) => ("Constraint_Perpendicular", ""),
        K::Tangent(..) | K::TangentAt(..) => ("Constraint_Tangent", ""),
        K::Equal(..) => ("Constraint_EqualLength", ""),
        K::PointOnObject(..) => ("Constraint_PointOnObject", ""),
        K::Symmetric(..) | K::SymmetricLine(..) => ("Constraint_Symmetric", ""),
        K::Distance(..) | K::DistanceLine(..) | K::PointLineDistance(..) => ("Constraint_Length", "Constraint_Length_Driven"),
        K::DistanceX(..) => ("Constraint_HorizontalDistance", "Constraint_HorizontalDistance_Driven"),
        K::DistanceY(..) => ("Constraint_VerticalDistance", "Constraint_VerticalDistance_Driven"),
        K::LockX(..) | K::LockY(..) => ("Constraint_Lock", "Constraint_Lock_Driven"),
        K::Radius(..) => ("Constraint_Radius", "Constraint_Radius_Driven"),
        K::Diameter(..) => ("Constraint_Diameter", "Constraint_Diameter_Driven"),
        K::Angle(..) | K::AngleBetween(..) => ("Constraint_InternalAngle", "Constraint_InternalAngle_Driven"),
        K::Block(_) => ("Constraint_Block", ""),
    };
    if !driving && !driven.is_empty() { driven } else { icon }
}

fn element_icon(c: &fc_sketch::Curve) -> &'static str {
    match c {
        fc_sketch::Curve::Point { .. } => "Sketcher_Element_Point_StartingPoint",
        fc_sketch::Curve::Line { .. } => "Sketcher_Element_Line_Edge",
        fc_sketch::Curve::Circle { .. } => "Sketcher_Element_Circle_Edge",
        fc_sketch::Curve::Arc { .. } => "Sketcher_Element_Arc_Edge",
    }
}

/// One selectable list row: icon, text, optional right-aligned value. Returns the response.
fn list_row(ui: &mut egui::Ui, reg: String, icon: &str, text: RichText, value: Option<RichText>, selected: bool, hovered_ext: bool) -> egui::Response {
    let w = ui.available_width();
    let (rect, resp) = ui.allocate_exact_size(egui::vec2(w, 22.0), egui::Sense::click());
    crate::widgets::reg(reg, rect);
    let p = ui.painter();
    if selected {
        p.rect_filled(rect, 2.0, crate::theme::SELECTION);
    } else if resp.hovered() || hovered_ext {
        p.rect_filled(rect, 2.0, crate::theme::HOVER_ROW);
    }
    crate::icons::paint(p, icon, egui::Rect::from_center_size(egui::pos2(rect.left() + 13.0, rect.center().y), egui::vec2(16.0, 16.0)), Color32::WHITE);
    let mut right = rect.right() - 6.0;
    if let Some(v) = value {
        let g = egui::WidgetText::from(v).into_galley(ui, Some(egui::TextWrapMode::Extend), f32::INFINITY, egui::TextStyle::Body);
        let pos = egui::pos2(rect.right() - 6.0 - g.size().x, rect.center().y - g.size().y * 0.5);
        right = pos.x - 8.0;
        ui.painter().galley(pos, g, crate::theme::TEXT);
    }
    let g = egui::WidgetText::from(text).into_galley(ui, Some(egui::TextWrapMode::Truncate), (right - rect.left() - 28.0).max(10.0), egui::TextStyle::Body);
    ui.painter().galley(egui::pos2(rect.left() + 28.0, rect.center().y - g.size().y * 0.5), g, crate::theme::TEXT);
    resp
}

impl SketchEditor {
    pub fn can_undo(&self) -> bool {
        !self.undo.is_empty()
    }
    pub fn can_redo(&self) -> bool {
        !self.redo.is_empty()
    }
    pub fn undo(&mut self, doc: &mut Document) {
        self.undo_step(doc);
    }
    pub fn redo(&mut self, doc: &mut Document) {
        self.redo_step(doc);
    }
    pub fn fit(&mut self) {
        self.want_fit = true;
    }

    /// The Sketcher workbench toolbar: geometry, construction toggle, constraints.
    pub fn toolbar_ui(&mut self, doc: &mut Document, ui: &mut egui::Ui) {
        use crate::widgets::{tb_grip, tb_separator, tool_button, Action};
        tb_grip(ui);
        let constr = self.construction;
        let action = move |t: Tool, checked: bool| {
            let key = Tool::ALL.iter().find(|(x, _, _)| *x == t).map(|(_, _, k)| *k).unwrap_or("");
            let (title, desc) = tool_title(t);
            (format!("sk:{}", format!("{t:?}").to_lowercase()), tool_icon(t, constr), title, desc, key, checked)
        };
        // single tools and FreeCAD-style dropdown groups (arcs, rectangles)
        let groups: [&[Tool]; 6] = [&[Tool::Point], &[Tool::Polyline], &[Tool::Line], &[Tool::Arc, Tool::Arc3], &[Tool::Circle], &[Tool::Rectangle, Tool::CenterRectangle]];
        for group in groups {
            if group.len() == 1 {
                let t = group[0];
                let (id, icon, title, desc, key, _) = action(t, false);
                if tool_button(ui, &Action::new(&id, icon, title, desc, key).checked(self.tool.tool == t)) {
                    self.set_tool(if self.tool.tool == t { Tool::Select } else { t });
                }
                continue;
            }
            let mem = egui::Id::new(("sk_group", group[0] as u8));
            let cur = ui.data(|d| d.get_temp::<u8>(mem)).map(|i| i as usize).unwrap_or(0).min(group.len() - 1);
            let cur = group.iter().position(|t| *t == self.tool.tool).unwrap_or(cur);
            let infos: Vec<_> = group.iter().map(|t| action(*t, self.tool.tool == *t)).collect();
            let variants: Vec<Action> = infos.iter().map(|(id, icon, title, desc, key, ch)| Action::new(id, icon, title, desc, key).checked(*ch)).collect();
            let (id, icon, title, desc, key, _) = &infos[cur];
            let main = Action::new(id, icon, title, desc, key).checked(group.contains(&self.tool.tool));
            let (clicked, chosen) = crate::widgets::split_tool_button(ui, &main, &variants);
            if let Some(i) = chosen {
                ui.data_mut(|d| d.insert_temp(mem, i as u8));
                self.set_tool(group[i]);
            } else if clicked {
                let t = group[cur];
                self.set_tool(if self.tool.tool == t { Tool::Select } else { t });
            }
        }
        let (id, icon, title, desc, key, _) = action(Tool::Slot, false);
        if tool_button(ui, &Action::new(&id, icon, title, desc, key).checked(self.tool.tool == Tool::Slot)) {
            self.set_tool(if self.tool.tool == Tool::Slot { Tool::Select } else { Tool::Slot });
        }
        tb_separator(ui);
        let ci = if self.construction { "Sketcher_ToggleConstruction_Constr" } else { "Sketcher_ToggleConstruction" };
        if tool_button(ui, &Action::new("sk:construction", ci, "Toggle Construction Geometry", "Toggles the selected geometry between normal and construction, or the creation mode when nothing is selected", "G, N").checked(self.construction)) {
            self.toggle_construction(doc);
        }
        tb_separator(ui);
        tb_grip(ui);
        for (c, _label, key, tip) in Cmd::ALL {
            if c == Cmd::Lock {
                tb_separator(ui);
            }
            let id = format!("sk:{}", format!("{c:?}").to_lowercase());
            if tool_button(ui, &Action::new(&id, cmd_icon(c), cmd_title(c), tip, key)) {
                self.run_cmd_ui(doc, c);
            }
        }
        tb_separator(ui);
        if tool_button(ui, &Action::new("sk:driving", "Sketcher_ToggleConstraint", "Toggle Driving/Reference Constraint", "Toggles the selected dimensions between driving and reference (measuring only)", "")) {
            self.toggle_driving(doc);
        }
        if tool_button(ui, &Action::new("sk:delete", "edit-delete", "Delete", "Deletes the selected elements and constraints", "Del").enabled(!self.sel.is_empty())) {
            self.delete_selection(doc, OnUsers::Refuse);
        }
    }

    fn solver_messages(&mut self, doc: &Document, ui: &mut egui::Ui) {
        let Some(r) = self.report.clone() else { return };
        let sk = self.sk(doc);
        let mut select: Option<Vec<usize>> = None;
        let big = |t: &str, c: Color32| RichText::new(t).color(c).font(crate::theme::bold_font(14.0));
        if sk.geometry.is_empty() {
            ui.label(big("Empty sketch", crate::theme::TEXT));
        } else if !r.conflicting.is_empty() || r.status == SolveStatus::Conflicting {
            ui.horizontal_wrapped(|ui| {
                ui.label(big("Over-constrained", COL_CONFLICT));
                if ui.link(RichText::new(format!("({})", list_text(&r.conflicting))).color(COL_CONFLICT)).on_hover_text("Click to select these constraints").clicked() {
                    select = Some(r.conflicting.clone());
                }
            });
            ui.label(RichText::new(format!("Remove at least one of the following constraints: {}", list_text(&r.conflicting))).color(COL_CONFLICT).small());
        } else if !r.malformed.is_empty() {
            ui.horizontal_wrapped(|ui| {
                ui.label(big("Malformed constraints", COL_CONFLICT));
                if ui.link(format!("({})", list_text(&r.malformed))).clicked() {
                    select = Some(r.malformed.clone());
                }
            });
        } else if !r.redundant.is_empty() {
            ui.horizontal_wrapped(|ui| {
                ui.label(big("Redundant constraints", COL_REDUNDANT));
                if ui.link(RichText::new(format!("({})", list_text(&r.redundant))).color(COL_REDUNDANT)).on_hover_text("Click to select these constraints").clicked() {
                    select = Some(r.redundant.clone());
                }
            });
            ui.label(RichText::new(format!("Remove the following redundant constraint(s): {}", list_text(&r.redundant))).color(COL_REDUNDANT).small());
        } else if !r.partially_redundant.is_empty() {
            ui.horizontal_wrapped(|ui| {
                ui.label(big("Partially redundant", COL_REDUNDANT));
                if ui.link(format!("({})", list_text(&r.partially_redundant))).clicked() {
                    select = Some(r.partially_redundant.clone());
                }
            });
        } else if r.status == SolveStatus::Failed {
            ui.label(big("Solver failed to converge", COL_CONFLICT));
        } else if r.dof > 0 {
            ui.horizontal_wrapped(|ui| {
                ui.label(big("Under-constrained:", crate::theme::TEXT));
                if ui.link(RichText::new(format!("{} Degree{} of Freedom", r.dof, if r.dof == 1 { "" } else { "s" })).font(crate::theme::bold_font(14.0))).on_hover_text("Click to select the elements that can still move").clicked() {
                    let free: Vec<Sel> = (0..sk.geometry.len()).filter(|g| !self.fully.get(*g).copied().unwrap_or(false)).map(Sel::Edge).collect();
                    self.sel = free;
                }
            });
        } else {
            ui.label(big("Fully constrained", GREEN));
        }
        if let Some(cs) = select {
            self.sel = cs.into_iter().map(Sel::Constraint).collect();
        }
        for e in &self.expr_errors {
            ui.label(RichText::new(format!("Expression error: {e}")).color(COL_CONFLICT).small());
        }
    }

    /// The sketch edit task boxes (the host draws the Close/Cancel buttons and passes the
    /// button pressed in `exit`). Returns how the editor should exit.
    pub fn panel_ui(&mut self, doc: &mut Document, ui: &mut egui::Ui, exit: EditorExit) -> EditorExit {
        use crate::widgets::task_box;
        self.panel_hover = None;
        task_box(ui, "sketch_edit", "Sketcher_Sketch", "Sketch Edit", |ui| {
            self.solver_messages(doc, ui);
            if let Some((m, err)) = &self.message {
                ui.label(RichText::new(m).color(if *err { COL_CONFLICT } else { crate::theme::TEXT_DIM }).small());
            }
            // refused delete
            let mut freeze = false;
            let mut drop_pending = false;
            if let Some(pd) = &self.pending_delete {
                ui.add_space(4.0);
                egui::Frame::group(ui.style()).fill(Color32::from_rgb(60, 36, 36)).show(ui, |ui| {
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
        });
        task_box(ui, "sketch_controls", "Sketcher_EditSketch", "Edit Controls", |ui| {
            ui.checkbox(&mut self.auto_constraints, "Auto constraints").on_hover_text("Snap to points/curves/axes and add coincident, point-on-object, horizontal and vertical constraints while drawing");
            ui.checkbox(&mut self.grid, "Grid snap").on_hover_text("Snap new points to the grid");
            ui.checkbox(&mut self.show_constraints, "Show constraints").on_hover_text("Show constraint icons and dimensions in the view");
        });
        let n = self.sk(doc).constraints.len();
        task_box(ui, "sketch_constraints", "Constraint_Dimension", &format!("Constraints ({n})"), |ui| {
            self.constraint_list(doc, ui);
        });
        let n = self.sk(doc).geometry.len();
        task_box(ui, "sketch_elements", "Sketcher_Element_Line_Edge", &format!("Elements ({n})"), |ui| {
            self.element_list(doc, ui);
        });
        if exit == EditorExit::Cancel {
            // the host restores the sketch; formulas and names edited meanwhile go back too
            doc.expressions = self.original_exprs.0.clone();
            doc.parameters = self.original_exprs.1.clone();
        }
        exit
    }

    fn constraint_list(&mut self, doc: &mut Document, ui: &mut egui::Ui) {
        let n = self.sk(doc).constraints.len();
        ui.horizontal(|ui| {
            ui.label("Filter");
            let opts = [(Filter::All, "All"), (Filter::Geometric, "Geometric"), (Filter::Datums, "Datums"), (Filter::Named, "Named"), (Filter::Selected, "Selected")];
            crate::widgets::combo(ui, "sk_filter", &mut self.panel.filter, &opts, ui.available_width() - 8.0);
        });
        ui.add_space(2.0);
        let rep = self.report.clone();
        let multi = ui.input(|i| i.modifiers.command || i.modifiers.ctrl || i.modifiers.shift);
        let mut clicked: Option<usize> = None;
        let filter = self.panel.filter;
        let sel_geo: Vec<usize> = self.sel.iter().filter_map(|s| match s {
            Sel::Edge(g) => Some(*g),
            Sel::Point(p) => Some(p.geo),
            _ => None,
        }).collect();
        egui::Frame::new().fill(crate::theme::INPUT).stroke(egui::Stroke::new(1.0, crate::theme::BORDER)).corner_radius(3.0).inner_margin(2.0).show(ui, |ui| {
            egui::ScrollArea::vertical().id_salt("sk_clist").max_height(260.0).auto_shrink([false, true]).show(ui, |ui| {
                if n == 0 {
                    ui.label(RichText::new("  No constraints").color(crate::theme::TEXT_DIM));
                }
                for ci in 0..n {
                    let sk = self.sk(doc);
                    let c = &sk.constraints[ci];
                    let dimensional = c.kind.is_dimensional();
                    let show = match filter {
                        Filter::All => true,
                        Filter::Geometric => !dimensional,
                        Filter::Datums => dimensional,
                        Filter::Named => c.name.is_some(),
                        Filter::Selected => self.is_sel(Sel::Constraint(ci)) || geom::point_refs(&c.kind).iter().any(|p| sel_geo.contains(&p.geo)),
                    };
                    if !show {
                        continue;
                    }
                    let name = c.name.clone().unwrap_or_else(|| format!("Constraint{}", ci + 1));
                    let value = geom::display_value(&c.kind).map(|v| {
                        let val = format!("{} {}", geom::fmt_num(v), geom::unit_of(&c.kind));
                        match doc.expressions.get(&self.ckey(doc, ci)) {
                            Some(f) => RichText::new(format!("ƒx {f} = {val}")).color(Color32::from_rgb(130, 180, 255)),
                            None if !c.driving => RichText::new(format!("({val})")).color(Color32::from_rgb(120, 160, 255)),
                            None => RichText::new(val),
                        }
                    });
                    let mut rt = RichText::new(name);
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
                    let icon = constraint_icon(&c.kind, c.driving);
                    let selected = self.is_sel(Sel::Constraint(ci));
                    let hover_ext = self.hover == Some(Sel::Constraint(ci));
                    let resp = list_row(ui, format!("constraint:{}", ci + 1), icon, rt, value, selected, hover_ext);
                    let resp = resp.on_hover_text(format!("{} {}", geom::kind_name(&c.kind), if dimensional { "· double-click to edit" } else { "" }));
                    if resp.hovered() {
                        self.panel_hover = Some(Sel::Constraint(ci));
                    }
                    if resp.clicked() {
                        clicked = Some(ci);
                    }
                    if resp.double_clicked() && dimensional {
                        self.open_dim_edit(doc, ci, false, None);
                    }
                }
            });
        });
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
        ui.add_space(6.0);
        egui::Frame::new().fill(crate::theme::ROW).stroke(egui::Stroke::new(1.0, crate::theme::BORDER)).corner_radius(4.0).inner_margin(8.0).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.add(crate::icons::image(ui.ctx(), constraint_icon(&c.kind, c.driving), 16.0));
                ui.label(RichText::new(format!("Constraint{}: {}", ci + 1, geom::kind_name(&c.kind))).font(crate::theme::bold_font(13.0)));
            });
            let (_, name, value) = self.panel.editing.as_mut().unwrap();
            if c.kind.is_dimensional() {
                let r1 = crate::widgets::task_row(ui, "Name", |ui| ui.add(egui::TextEdit::singleline(name).desired_width(ui.available_width() - 4.0).hint_text("e.g. width")));
                crate::widgets::reg("cprop:name", r1.rect);
                let r2 = crate::widgets::task_row(ui, "Value", |ui| ui.add(egui::TextEdit::singleline(value).desired_width(ui.available_width() - 4.0)).on_hover_text("A number with units (25 mm, 30 deg), or a formula starting with '=' (=width/2)"));
                crate::widgets::reg("cprop:value", r2.rect);
                if (r1.lost_focus() || r2.lost_focus()) && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    apply = true;
                }
                ui.horizontal(|ui| {
                    let mut driving = c.driving;
                    if ui.checkbox(&mut driving, "Driving").on_hover_text("Unchecked: reference dimension (only measures)").changed() {
                        toggle = true;
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.button("Apply").clicked() {
                            apply = true;
                        }
                    });
                });
            }
            if let Some(e) = &self.panel.edit_error {
                ui.label(RichText::new(e).color(COL_CONFLICT).small());
            }
            if ui.add(egui::Button::image_and_text(crate::icons::image(ui.ctx(), "edit-delete", 14.0), "Delete constraint")).clicked() {
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
        let sk = self.sk(doc).clone();
        let multi = ui.input(|i| i.modifiers.command || i.modifiers.ctrl || i.modifiers.shift);
        let mut clicked = None;
        egui::Frame::new().fill(crate::theme::INPUT).stroke(egui::Stroke::new(1.0, crate::theme::BORDER)).corner_radius(3.0).inner_margin(2.0).show(ui, |ui| {
            egui::ScrollArea::vertical().id_salt("sk_elist").max_height(200.0).auto_shrink([false, true]).show(ui, |ui| {
                if sk.geometry.is_empty() {
                    ui.label(RichText::new("  No elements").color(crate::theme::TEXT_DIM));
                }
                for (gi, g) in sk.geometry.iter().enumerate() {
                    let full = self.fully.get(gi).copied().unwrap_or(false);
                    let mut rt = RichText::new(format!("{}-{}", geom::geo_name(gi), geom::curve_name(&g.curve)));
                    if g.construction {
                        rt = RichText::new(format!("{}-{} (construction)", geom::geo_name(gi), geom::curve_name(&g.curve))).color(Color32::from_rgb(120, 160, 255));
                    } else if full && !self.invalid() {
                        rt = rt.color(GREEN);
                    }
                    let sel = if matches!(g.curve, fc_sketch::Curve::Point { .. }) { Sel::Point(fc_sketch::PointRef::start(gi)) } else { Sel::Edge(gi) };
                    let hover_ext = self.hover == Some(sel);
                    let resp = list_row(ui, format!("element:{}", gi + 1), element_icon(&g.curve), rt, None, self.is_sel(sel), hover_ext);
                    if resp.hovered() {
                        self.panel_hover = Some(sel);
                    }
                    if resp.clicked() {
                        clicked = Some(sel);
                    }
                }
            });
        });
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
        // keep the popup out from under the Tasks panel
        let pos = egui::pos2(de.pos.x.min(self.free_right - 280.0).max(self.rect.left() + 4.0), de.pos.y);
        egui::Area::new(egui::Id::new("sk_dim_edit")).order(egui::Order::Foreground).fade_in(false).fixed_pos(pos).show(ctx, |ui| {
            egui::Frame::popup(ui.style()).fill(Color32::from_rgb(40, 40, 40)).show(ui, |ui| {
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
