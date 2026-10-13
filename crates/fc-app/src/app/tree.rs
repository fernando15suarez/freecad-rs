//! The Model dock: tree view (document > Body > Origin + features) on top, and the
//! property view below with tabs for Data (property grid), Variables and Measurements.

use super::tasks::Lay;
use super::*;
use crate::theme;
use crate::widgets::{self, PropGrid};
use egui::{Color32, Pos2, Rect, RichText, Sense, Stroke, Vec2};

/// What happened on a tree row this frame.
#[derive(Default)]
pub(super) struct RowOut {
    pub clicked: bool,
    pub double: bool,
    pub eye: bool,
    pub expand: bool,
}

/// A tree row for other modules (assembly items): the response is returned for menus.
pub(super) struct RowSpecPub<'a> {
    pub reg: String,
    pub depth: usize,
    pub expander: Option<bool>,
    pub eye: Option<bool>,
    pub icon: &'a str,
    pub badge: Option<&'a str>,
    pub label: RichText,
    pub selected: bool,
    pub tip: String,
}

pub(super) struct RowOutPub {
    pub clicked: bool,
    pub double: bool,
    pub eye: bool,
    pub expand: bool,
    pub resp: egui::Response,
}

pub(super) fn tree_row_pub(ui: &mut egui::Ui, s: RowSpecPub) -> RowOutPub {
    let mut resp = None;
    let o = tree_row_ex(ui, RowSpec { reg: s.reg, depth: s.depth, expander: s.expander, eye: s.eye, eye_space: true, icon: s.icon, badge: s.badge, label: s.label, selected: s.selected, tip: s.tip }, None, Some(&mut resp));
    RowOutPub { clicked: o.clicked, double: o.double, eye: o.eye, expand: o.expand, resp: resp.unwrap() }
}

struct RowSpec<'a> {
    reg: String,
    depth: usize,
    /// Some(open) when the row has children.
    expander: Option<bool>,
    /// Some(visible) when the row has a visibility eye.
    eye: Option<bool>,
    /// Keep the eye column even without an eye (aligns sibling rows).
    eye_space: bool,
    icon: &'a str,
    badge: Option<&'a str>,
    label: RichText,
    selected: bool,
    tip: String,
}

const INDENT: f32 = 16.0;

fn tree_row(ui: &mut egui::Ui, s: RowSpec, menu: Option<&mut dyn FnMut(&mut egui::Ui)>) -> RowOut {
    tree_row_ex(ui, s, menu, None)
}

fn tree_row_ex(ui: &mut egui::Ui, s: RowSpec, menu: Option<&mut dyn FnMut(&mut egui::Ui)>, out_resp: Option<&mut Option<egui::Response>>) -> RowOut {
    let w = ui.available_width();
    let (rect, resp) = ui.allocate_exact_size(Vec2::new(w, 22.0), Sense::click());
    widgets::reg(s.reg.clone(), rect);
    let p = ui.painter();
    let x0 = rect.left() + 4.0 + s.depth as f32 * INDENT;
    let y = rect.center().y;
    let label_x = x0 + 14.0 + if s.eye.is_some() || s.eye_space { 20.0 } else { 0.0 } + 20.0;
    let galley = egui::WidgetText::from(s.label.clone()).into_galley(ui, Some(egui::TextWrapMode::Truncate), (rect.right() - label_x - 4.0).max(10.0), egui::TextStyle::Body);
    let label_w = galley.size().x;
    // FreeCAD highlights the label region of the selected item
    let hl = Rect::from_min_max(Pos2::new(x0 + 14.0, rect.top() + 1.0), Pos2::new((label_x + label_w + 6.0).min(rect.right()), rect.bottom() - 1.0));
    let label = if s.selected { s.label.clone().color(Color32::WHITE) } else { s.label.clone() };
    if s.selected {
        p.rect_filled(hl, 2.0, theme::SELECTION);
    } else if resp.hovered() {
        p.rect_filled(hl, 2.0, theme::HOVER_ROW);
    }
    let mut out = RowOut::default();
    // expander
    if let Some(open) = s.expander {
        let c = Pos2::new(x0 + 5.0, y);
        let pts = if open { vec![c + Vec2::new(-4.0, -2.5), c + Vec2::new(4.0, -2.5), c + Vec2::new(0.0, 3.0)] } else { vec![c + Vec2::new(-2.5, -4.0), c + Vec2::new(3.0, 0.0), c + Vec2::new(-2.5, 4.0)] };
        let er = Rect::from_center_size(c, Vec2::splat(14.0));
        let eresp = ui.interact(er, ui.id().with((&s.reg, "exp", rect.top() as i32)), Sense::click());
        let col = if eresp.hovered() { Color32::WHITE } else { Color32::from_rgb(170, 170, 170) };
        ui.painter().add(egui::Shape::convex_polygon(pts, col, Stroke::NONE));
        out.expand = eresp.clicked();
    }
    let mut x = x0 + 14.0;
    if let Some(vis) = s.eye {
        let er = Rect::from_center_size(Pos2::new(x + 9.0, y), Vec2::splat(16.0));
        let eresp = ui.interact(er, ui.id().with((&s.reg, "eye", rect.top() as i32)), Sense::click());
        widgets::reg(format!("{}:eye", s.reg), er);
        crate::icons::paint(ui.painter(), if vis { "TreeItemVisible" } else { "TreeItemInvisible" }, er, if eresp.hovered() { Color32::WHITE } else { Color32::from_gray(215) });
        out.eye = eresp.on_hover_text(if vis { "Hide" } else { "Show" }).clicked();
        x += 20.0;
    } else if s.eye_space {
        x += 20.0;
    }
    let ir = Rect::from_center_size(Pos2::new(x + 9.0, y), Vec2::splat(16.0));
    crate::icons::paint(ui.painter(), s.icon, ir, Color32::WHITE);
    if let Some(b) = s.badge {
        crate::icons::paint(ui.painter(), b, Rect::from_center_size(ir.right_bottom() - Vec2::new(3.0, 3.0), Vec2::splat(10.0)), Color32::WHITE);
    }
    let galley = egui::WidgetText::from(label).into_galley(ui, Some(egui::TextWrapMode::Truncate), (rect.right() - label_x - 4.0).max(10.0), egui::TextStyle::Body);
    ui.painter().galley(Pos2::new(label_x, y - galley.size().y * 0.5), galley, theme::TEXT);
    let resp = if s.tip.is_empty() { resp } else { resp.on_hover_text(&s.tip) };
    if let Some(m) = menu {
        resp.context_menu(|ui| m(ui));
    }
    out.double = resp.double_clicked();
    out.clicked = resp.clicked() && !out.eye && !out.expand;
    if let Some(r) = out_resp {
        *r = Some(resp);
    }
    out
}

pub(super) fn is_open(ui: &egui::Ui, key: &str, default: bool) -> bool {
    ui.data(|d| d.get_temp::<bool>(egui::Id::new(("tree_open", key)))).unwrap_or(default)
}

pub(super) fn toggle_open(ui: &egui::Ui, key: &str, default: bool) {
    let v = !is_open(ui, key, default);
    ui.data_mut(|d| d.insert_temp(egui::Id::new(("tree_open", key)), v));
}

#[derive(Clone, Copy)]
enum TreeAct {
    Select(FeatureId),
    Edit(FeatureId),
    Origin(usize),
    ToggleBody,
    ToggleOrigin,
    ToggleSketch(FeatureId),
    Delete(FeatureId),
    Suppress(FeatureId),
    Tip(FeatureId),
    OpenPart(fc_assembly::PartId),
}

impl App {
    pub(super) fn model_dock(&mut self, ui: &mut egui::Ui) {
        widgets::dock_title(ui, "Model");
        let h = ui.available_height();
        egui::Panel::bottom("prop_tabs").frame(egui::Frame::new().inner_margin(egui::Margin { left: 0, right: 0, top: 2, bottom: 4 })).show(ui, |ui| {
            let r = ui.max_rect();
            ui.painter().line_segment([r.left_top(), r.right_top()], Stroke::new(1.0, theme::BORDER));
            if let Some(t) = widgets::tab_bar(ui, &["Data", "Variables", "Measure"], self.props_tab) {
                self.props_tab = t;
            }
        });
        let tree = egui::Panel::top("tree_split")
            .resizable(true)
            .default_size((h * 0.55).max(120.0))
            .size_range(80.0..=(h - 140.0).max(100.0))
            .frame(egui::Frame::new().fill(theme::PANEL))
            .show(ui, |ui| {
                egui::ScrollArea::vertical().id_salt("tree").auto_shrink([false, false]).show(ui, |ui| {
                    ui.spacing_mut().item_spacing.y = 0.0;
                    self.tree(ui);
                });
            });
        // splitter grip between the tree and the property view
        let gb = tree.response.rect.center_bottom();
        for dx in [-8.0, -4.0, 0.0, 4.0, 8.0] {
            ui.painter().circle_filled(gb + egui::vec2(dx, -2.0), 1.1, Color32::from_gray(110));
        }
        egui::CentralPanel::default().frame(egui::Frame::new().fill(theme::ROW).stroke(Stroke::new(1.0, theme::BORDER)).inner_margin(egui::Margin::same(1))).show(ui, |ui| {
            egui::ScrollArea::vertical().id_salt("props").auto_shrink([false, false]).show(ui, |ui| {
                ui.spacing_mut().item_spacing.y = 0.0;
                match self.props_tab {
                    1 => self.variables(ui),
                    2 => self.measurements(ui),
                    _ if self.task.is_some() || self.editor.is_some() || self.picking || self.fem.task.is_some() => {
                        // FreeCAD blanks the property view while a task dialog is open
                        let h = ui.available_height();
                        ui.add_space((h * 0.35).max(10.0));
                        ui.vertical_centered(|ui| {
                            ui.add(crate::icons::image(ui.ctx(), "document-properties", 32.0).tint(Color32::from_gray(150)));
                            ui.add_space(6.0);
                            ui.label(RichText::new("Editing in the Tasks panel").color(Color32::from_gray(170)).font(theme::bold_font(13.0)));
                            ui.label(RichText::new("Properties return when the task is closed").color(theme::TEXT_DIM));
                        });
                    }
                    _ if self.asm.task.is_some() => {
                        ui.add_space(12.0);
                        ui.vertical_centered(|ui| {
                            ui.label(RichText::new("Editing in the Tasks panel").color(Color32::from_gray(170)).font(theme::bold_font(13.0)));
                        });
                    }
                    _ if self.fem.sel.is_some() && self.tree_sel.is_empty() && (self.fem.on || self.active_assembly().is_none()) => {
                        let it = self.fem.sel.unwrap();
                        self.fem_data_tab(ui, it)
                    }
                    _ if self.active_assembly().is_some() => self.asm_data_tab(ui),
                    _ => self.data_tab(ui),
                }
            });
        });
    }

    fn tree(&mut self, ui: &mut egui::Ui) {
        let interactive = self.editor.is_none() && self.task.is_none() && !self.picking;
        let mut acts: Vec<TreeAct> = vec![];
        let title = self.doc_title();
        tree_row(ui, RowSpec { reg: "tree:document".into(), depth: 0, expander: Some(true), eye: None, eye_space: false, icon: "Document", badge: None, label: RichText::new(title).font(theme::bold_font(13.0)), selected: false, tip: self.path.as_ref().map(|p| p.display().to_string()).unwrap_or_else(|| "Not saved yet".into()) }, None);
        let parts: Vec<(fc_assembly::PartId, String)> = self.ws.parts.iter().map(|p| (p.id, p.name.clone())).collect();
        let several = parts.len() > 1 || !self.ws.assemblies.is_empty();
        let all_idle = self.idle();
        for (pid, pname) in parts {
            if self.active_part() == Some(pid) {
                self.body_tree(ui, &pname, several, interactive, &mut acts);
                let fem_interactive = self.editor.is_none() && self.task.is_none() && !self.picking && self.fem.task.is_none();
                self.fem_tree(ui, fem_interactive);
                continue;
            }
            // another part: a closed Body row; double-click opens it in Part Design
            let n = self.ws.part(pid).map(|p| p.doc.features.len()).unwrap_or(0);
            let r = tree_row(ui, RowSpec { reg: format!("tree:part:{pname}"), depth: 1, expander: Some(false), eye: None, eye_space: true, icon: "PartDesign_Body", badge: None, label: RichText::new(&pname), selected: false, tip: format!("{pname}: part with {n} features\nDouble-click to open it in Part Design") }, None);
            if (r.double || r.expand) && all_idle {
                acts.push(TreeAct::OpenPart(pid));
            }
        }
        let asms: Vec<fc_assembly::AssemblyId> = self.ws.assemblies.iter().map(|a| a.id).collect();
        for a in asms {
            self.assembly_tree_rows(ui, a, 1, all_idle || self.asm.task.is_none() && self.task.is_none() && self.editor.is_none());
        }
        self.tree_acts(ui, acts, interactive);
    }

    fn body_tree(&mut self, ui: &mut egui::Ui, name: &str, several: bool, interactive: bool, acts: &mut Vec<TreeAct>) {
        let body_open = is_open(ui, "body", true);
        let label = if several { RichText::new(name).font(theme::bold_font(13.0)) } else { RichText::new(name) };
        let r = tree_row(ui, RowSpec { reg: "tree:Body".into(), depth: 1, expander: Some(body_open), eye: Some(self.body_visible), eye_space: true, icon: "PartDesign_Body", badge: None, label, selected: false, tip: format!("{name}: the solid built by the features below, in order{}", if several { "\nThe active part (open in Part Design)" } else { "" }) }, None);
        if r.expand {
            toggle_open(ui, "body", true);
        }
        if r.eye {
            acts.push(TreeAct::ToggleBody);
        }
        if body_open {
            // Origin
            let o_open = is_open(ui, "origin", false);
            let dim = |t: &str, d: bool| if d { RichText::new(t).color(theme::TEXT_DIM) } else { RichText::new(t) };
            let r = tree_row(ui, RowSpec { reg: "tree:Origin".into(), depth: 2, expander: Some(o_open), eye: Some(self.show_origin), eye_space: true, icon: "Std_CoordinateSystem", badge: None, label: dim("Origin", !self.show_origin), selected: false, tip: "The body's origin: base planes and axes".into() }, None);
            if r.expand || (r.clicked && interactive) {
                toggle_open(ui, "origin", false);
            }
            if r.eye {
                acts.push(TreeAct::ToggleOrigin);
            }
            if o_open {
                for (i, (name, icon)) in ORIGIN_ITEMS.iter().enumerate() {
                    let tip = if i >= 3 { "Select, then New Sketch to sketch on this plane" } else { "Select, then Revolution/Groove to revolve about this axis" };
                    let r = tree_row(ui, RowSpec { reg: format!("tree:{name}"), depth: 3, expander: None, eye: None, eye_space: true, icon, badge: None, label: dim(name, !self.show_origin), selected: self.origin_sel == Some(i), tip: tip.into() }, None);
                    if r.clicked && interactive {
                        acts.push(TreeAct::Origin(i));
                    }
                }
            }
            // Sketches consumed by a solid feature are shown under it, like FreeCAD.
            let consumed: HashSet<FeatureId> = self
                .doc
                .features
                .iter()
                .filter(|f| f.kind.is_solid_feature())
                .flat_map(|f| f.kind.inputs())
                .filter(|i| matches!(self.doc.get(*i).map(|f| &f.kind), Some(FeatureKind::Sketch(_))))
                .collect();
            let tip_solid = self.tip_solid_feature();
            let ids: Vec<FeatureId> = self.doc.features.iter().map(|f| f.id).collect();
            for id in ids {
                if consumed.contains(&id) {
                    continue;
                }
                let inputs: Vec<FeatureId> = self.doc.get(id).map(|f| f.kind.inputs()).unwrap_or_default().into_iter().filter(|i| consumed.contains(i)).collect();
                let solid = self.doc.get(id).is_some_and(|f| f.kind.is_solid_feature());
                let has_children = solid && !inputs.is_empty();
                let key = format!("feat{id}");
                // a selected (or edited) consumed sketch shows its parent expanded
                let open = has_children && (is_open(ui, &key, false) || inputs.iter().any(|i| self.tree_sel.contains(i)));
                let r = self.feature_row(ui, id, 2, if has_children { Some(open) } else { None }, tip_solid, &consumed, interactive, acts);
                if r.expand {
                    toggle_open(ui, &key, false);
                }
                if open {
                    for i in inputs {
                        self.feature_row(ui, i, 3, None, tip_solid, &consumed, interactive, acts);
                    }
                }
            }
        }
    }

    fn tree_acts(&mut self, ui: &mut egui::Ui, acts: Vec<TreeAct>, interactive: bool) {
        // empty area below the rows clears the selection
        let rest = ui.available_rect_before_wrap();
        if rest.height() > 4.0 && interactive {
            let r = ui.interact(rest, ui.id().with("tree_empty"), Sense::click());
            if r.clicked() {
                self.tree_sel.clear();
                self.fem.sel = None;
                self.asm.sel.clear();
                self.origin_sel = None;
                self.fields.clear();
                self.rebuild_scene();
            }
        }
        let multi = ui.input(|i| i.modifiers.command || i.modifiers.ctrl);
        for a in acts {
            match a {
                TreeAct::Select(id) => {
                    if multi {
                        if let Some(i) = self.tree_sel.iter().position(|x| *x == id) {
                            self.tree_sel.remove(i);
                        } else {
                            self.tree_sel.push(id);
                        }
                    } else {
                        self.tree_sel = vec![id];
                    }
                    self.fem.sel = None;
                    self.origin_sel = None;
                    self.fields.clear();
                    self.rebuild_scene();
                }
                TreeAct::Edit(id) => {
                    self.tree_sel = vec![id];
                    self.edit_feature(id);
                }
                TreeAct::Origin(i) => {
                    self.origin_sel = if self.origin_sel == Some(i) && multi { None } else { Some(i) };
                    self.tree_sel.clear();
                    self.selection.clear();
                    if i >= 3 && !self.show_origin {
                        // show the planes so the selection is visible
                        self.show_origin = true;
                    }
                    self.rebuild_scene();
                }
                TreeAct::ToggleBody => {
                    self.body_visible = !self.body_visible;
                    self.selection.clear();
                    self.rebuild_scene();
                }
                TreeAct::ToggleOrigin => {
                    self.show_origin = !self.show_origin;
                    self.rebuild_scene();
                }
                TreeAct::ToggleSketch(id) => {
                    if !self.hidden_sketches.remove(&id) {
                        self.hidden_sketches.insert(id);
                    }
                    self.rebuild_scene();
                }
                TreeAct::Delete(id) => self.try_delete(DeleteWhat::Feature(id), OnUsers::Refuse),
                TreeAct::Suppress(id) => self.toggle_suppressed(id),
                TreeAct::Tip(id) => self.toggle_tip(id),
                TreeAct::OpenPart(p) => self.switch_tab(super::workspace::Tab::Part(p)),
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn feature_row(&self, ui: &mut egui::Ui, id: FeatureId, depth: usize, expander: Option<bool>, tip_solid: Option<FeatureId>, consumed: &HashSet<FeatureId>, interactive: bool, acts: &mut Vec<TreeAct>) -> RowOut {
        let Some(f) = self.doc.get(id) else { return RowOut::default() };
        let st = self.doc.state(id);
        let solid = f.kind.is_solid_feature();
        // the body shows its tip's result; other solid features have no visibility of their own
        let is_tip = tip_solid == Some(id);
        let visible = if solid { !is_tip || self.body_visible } else { !consumed.contains(&id) && !self.hidden_sketches.contains(&id) };
        let eye = if solid { if is_tip { Some(self.body_visible) } else { None } } else { Some(visible) };
        let err = st.and_then(|s| s.error.as_ref()).map(|e| e.to_string());
        let warn = st.and_then(|s| s.warning.clone());
        let dof = st.and_then(|s| s.solve.as_ref()).map(|r| r.dof).unwrap_or(0);
        let mut label = RichText::new(&f.label);
        if !visible {
            label = label.color(Color32::from_gray(155));
        }
        if err.is_some() {
            label = label.color(theme::ERROR);
        }
        if f.suppressed {
            label = label.strikethrough();
        }
        let badge = if err.is_some() {
            Some("overlay_error")
        } else if f.suppressed {
            Some("feature_suppressed")
        } else if warn.is_some() {
            Some("Warning")
        } else if self.doc.tip == Some(id) {
            Some("PartDesign_Overlay_Tip")
        } else {
            None
        };
        let icon = if matches!(f.kind, FeatureKind::Sketch(_)) && dof > 0 { "Sketcher_NotFullyConstrained" } else { feature_icon(&f.kind) };
        let mut tip = format!("{} ({})", f.label, feature_title(&f.kind));
        if let Some(e) = &err {
            tip.push_str(&format!("\nError: {e}"));
        } else if let Some(w) = &warn {
            tip.push_str(&format!("\nWarning: {w}"));
        }
        if matches!(f.kind, FeatureKind::Sketch(_)) {
            tip.push_str(&if dof > 0 { format!("\nUnder-constrained: {dof} degrees of freedom") } else { "\nFully constrained".into() });
        }
        if self.doc.tip == Some(id) {
            tip.push_str("\nTip: the body ends here");
        }
        tip.push_str("\nDouble-click to edit · right-click for more");
        let explicit_tip = self.doc.tip == Some(id);
        let suppressed = f.suppressed;
        let mut menu_acts: Vec<TreeAct> = vec![];
        let mut menu = |ui: &mut egui::Ui| {
            ui.set_min_width(170.0);
            let item = |ui: &mut egui::Ui, icon: &str, text: &str, en: bool| ui.add_enabled(en, egui::Button::image_and_text(crate::icons::image(ui.ctx(), icon, 16.0), text)).clicked();
            if item(ui, "edit-edit", if solid { "Edit parameters" } else { "Edit sketch" }, interactive) {
                menu_acts.push(TreeAct::Edit(id));
                ui.close();
            }
            if !solid && item(ui, if visible { "TreeItemInvisible" } else { "TreeItemVisible" }, if visible { "Hide" } else { "Show" }, !consumed.contains(&id)) {
                menu_acts.push(TreeAct::ToggleSketch(id));
                ui.close();
            }
            if solid && item(ui, "PartDesign_MoveTip", if explicit_tip { "Unset tip" } else { "Set tip" }, interactive) {
                menu_acts.push(TreeAct::Tip(id));
                ui.close();
            }
            if item(ui, "feature_suppressed", if suppressed { "Unsuppress" } else { "Suppress" }, interactive) {
                menu_acts.push(TreeAct::Suppress(id));
                ui.close();
            }
            ui.separator();
            if item(ui, "edit-delete", "Delete", interactive) {
                menu_acts.push(TreeAct::Delete(id));
                ui.close();
            }
        };
        let r = tree_row(
            ui,
            RowSpec { reg: format!("tree:{}", f.name), depth, expander, eye, eye_space: true, icon, badge, label, selected: self.tree_sel.contains(&id), tip },
            Some(&mut menu),
        );
        acts.extend(menu_acts);
        if r.eye && !solid && !consumed.contains(&id) {
            acts.push(TreeAct::ToggleSketch(id));
        } else if r.eye && solid && is_tip {
            acts.push(TreeAct::ToggleBody);
        }
        if interactive {
            if r.double {
                acts.push(TreeAct::Edit(id));
            } else if r.clicked {
                acts.push(TreeAct::Select(id));
            }
        }
        r
    }

    // --------------------------------------------------------------- Data tab

    fn data_tab(&mut self, ui: &mut egui::Ui) {
        let mut g = PropGrid::new(ui);
        if let Some(i) = self.origin_sel {
            let (name, _) = ORIGIN_ITEMS[i];
            g.group(ui, "Base");
            g.row(ui, "Label", "", |ui| widgets::ro_value(ui, name));
            g.row(ui, "Role", "", |ui| widgets::ro_value(ui, if i < 3 { "Axis" } else { "Plane" }));
            let place = match i {
                0 => "Direction (1, 0, 0)",
                1 => "Direction (0, 1, 0)",
                2 => "Direction (0, 0, 1)",
                3 => "Normal (0, 0, 1)",
                4 => "Normal (0, -1, 0)",
                _ => "Normal (1, 0, 0)",
            };
            g.row(ui, "Placement", "", |ui| widgets::ro_value(ui, place));
            return;
        }
        let Some(&id) = self.tree_sel.last() else {
            if self.selection.is_empty() {
                ui.add_space(8.0);
                egui::Frame::new().inner_margin(egui::Margin::symmetric(8, 0)).show(ui, |ui| {
                    ui.add(egui::Label::new(RichText::new("Select an item in the tree or the 3D view to see its properties.").color(theme::TEXT_DIM)).wrap());
                });
                if self.doc.result().is_some() {
                    ui.add_space(8.0);
                    let s = self.doc.result().cloned().unwrap();
                    let b = s.bbox();
                    g.group(ui, "Body");
                    g.row(ui, "Volume", "", |ui| widgets::ro_value(ui, &format!("{} mm³", fmt_num(s.volume()))));
                    g.row(ui, "Bounding box", "", |ui| widgets::ro_value(ui, &format!("{:.2} x {:.2} x {:.2} mm", b[3] - b[0], b[4] - b[1], b[5] - b[2])));
                    g.row(ui, "Features", "", |ui| widgets::ro_value(ui, &self.doc.features.len().to_string()));
                }
            } else {
                g.group(ui, "Selection");
                for p in self.selection.clone() {
                    let n = pick_name(&self.vp, &p, self.planes_at);
                    g.row(ui, &n, "", |ui| widgets::ro_value(ui, &format!("({}, {}, {}) mm", fmt_num(p.point[0]), fmt_num(p.point[1]), fmt_num(p.point[2]))));
                }
            }
            return;
        };
        let Some(f) = self.doc.get(id).cloned() else { return };
        g.group(ui, "Base");
        g.row(ui, "Label", "User name of the feature", |ui| {
            let key = format!("label:{}", f.id);
            let buf = self.fields.entry(key.clone()).or_insert_with(|| f.label.clone());
            let r = ui.add(egui::TextEdit::singleline(buf).desired_width(ui.available_width() - 2.0).margin(egui::Margin::symmetric(4, 1)));
            if r.lost_focus() {
                let label = buf.trim().to_string();
                self.fields.remove(&key);
                if label != f.label && !label.is_empty() {
                    self.change(|d| d.get_mut(id).unwrap().label = label);
                }
            }
        });
        g.row(ui, "Type", "", |ui| widgets::ro_value(ui, feature_title(&f.kind)));
        let mut sup = f.suppressed;
        if g.row(ui, "Suppressed", "A suppressed feature is skipped when the body is computed", |ui| widgets::yes_no(ui, &mut sup)) {
            self.change(|d| d.get_mut(id).unwrap().suppressed = sup);
        }
        if f.kind.is_solid_feature() {
            let mut tip = self.doc.tip == Some(id);
            if g.row(ui, "Tip", "The body ends at its tip feature", |ui| widgets::yes_no(ui, &mut tip)) {
                self.change(|d| d.tip = if tip { Some(id) } else { None });
            }
        }
        if let Some(e) = self.doc.state(id).and_then(|s| s.error.as_ref()).map(|e| e.to_string()) {
            g.row(ui, "Error", &e, |ui| {
                ui.add(egui::Label::new(RichText::new(&e).color(theme::ERROR)).truncate()).on_hover_text(&e);
            });
        }
        g.group(ui, feature_title(&f.kind));
        self.feature_params(ui, id, &mut Lay::Grid(&mut g));
    }

    // ---------------------------------------------------------- Variables tab

    fn variables(&mut self, ui: &mut egui::Ui) {
        let mut g = PropGrid::new(ui);
        g.group(ui, "Variables");
        let params: Vec<(String, String)> = self.doc.parameters.iter().map(|(k, v)| (k.clone(), v.clone())).collect();
        let (vals, errs) = self.doc.evaluate_expressions();
        let mut delete: Option<String> = None;
        let mut edit: Option<(String, String)> = None;
        let mut release: Option<String> = None;
        if params.is_empty() {
            g.row(ui, "(none)", "", |ui| widgets::ro_value(ui, "Add one below; use it in any value as =name"));
        }
        for (k, v) in &params {
            let key = format!("param:{k}");
            let driven = self.doc.driven.get(k).cloned();
            g.row_with(
                ui,
                &format!("var:{k}"),
                |ui| {
                    ui.label(RichText::new(k).monospace().color(theme::TEXT));
                },
                |ui| {
                    let w = ui.available_width();
                    if let Some(src) = &driven {
                        ui.add(crate::icons::image(ui.ctx(), "Std_Placement", 14.0)).on_hover_text(format!("Driven by {src}"));
                        ui.add(egui::Label::new(RichText::new(v).monospace().color(theme::TEXT_DIM)).truncate());
                    } else {
                        let buf = self.fields.entry(key.clone()).or_insert_with(|| v.clone());
                        if ui.add(egui::TextEdit::singleline(buf).desired_width((w - 90.0).max(50.0)).margin(egui::Margin::symmetric(4, 1))).lost_focus() && *buf != *v {
                            edit = Some((k.clone(), buf.clone()));
                        }
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if driven.is_some() {
                            if ui.small_button("Release").on_hover_text("Stop following the external value and edit it here").clicked() {
                                release = Some(k.clone());
                            }
                        } else if ui.add(egui::Button::image(crate::icons::image(ui.ctx(), "edit-delete", 14.0)).frame(false)).on_hover_text("Delete variable").clicked() {
                            delete = Some(k.clone());
                        }
                        match (vals.get(&key), errs.get(&key)) {
                            (_, Some(e)) => {
                                ui.add(crate::icons::image(ui.ctx(), "Warning", 14.0)).on_hover_text(e.to_string());
                            }
                            (Some(x), _) => {
                                ui.label(RichText::new(fmt_num(*x)).color(theme::TEXT_DIM));
                            }
                            _ => {}
                        }
                    });
                },
            );
        }
        if let Some(k) = release {
            self.change(|d| {
                d.release_driven(&k);
            });
        }
        g.group(ui, "New variable");
        let mut add = false;
        let (new_name, new_value) = (&mut self.new_param.0, &mut self.new_param.1);
        g.row_with(
            ui,
            "var:new",
            |ui| {
                ui.add(egui::TextEdit::singleline(new_name).hint_text("name").desired_width(ui.available_width() - 2.0).margin(egui::Margin::symmetric(4, 1)));
            },
            |ui| {
                let w = ui.available_width();
                ui.add(egui::TextEdit::singleline(new_value).hint_text("value or formula").desired_width((w - 56.0).max(40.0)).margin(egui::Margin::symmetric(4, 1)));
                let r = ui.add(egui::Button::image_and_text(crate::icons::image(ui.ctx(), "list-add", 14.0), "Add"));
                widgets::reg("var:add", r.rect);
                add = r.clicked();
            },
        );
        if add && !self.new_param.0.trim().is_empty() {
            let (n, v) = (self.new_param.0.trim().to_string(), self.new_param.1.trim().to_string());
            let v = if v.is_empty() { "0".to_string() } else { v };
            let before = self.state_json();
            match self.doc.set_parameter(&n, &v) {
                Ok(()) => {
                    self.undo.push(before);
                    self.new_param = (String::new(), String::new());
                    self.after_change();
                }
                Err(e) => self.status = e.to_string(),
            }
        }
        if let Some((k, v)) = edit {
            let before = self.state_json();
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
            g.group(ui, "Named dimensions");
            for (n, v) in named {
                g.row(ui, &n, "Use it in formulas by this name", |ui| widgets::ro_value(ui, &fmt_num(v)));
            }
        }
    }

    // ------------------------------------------------------- Measurements tab

    fn measurements(&mut self, ui: &mut egui::Ui) {
        let mut g = PropGrid::new(ui);
        g.group(ui, "Measurements");
        let items = self.doc.measure_all();
        let mut delete: Option<String> = None;
        let mut rename: Option<(String, String)> = None;
        if items.is_empty() {
            g.row(ui, "(none)", "", |ui| widgets::ro_value(ui, "Add one below"));
        }
        for (name, res) in &items {
            let key = format!("meas:{name}");
            g.row_with(
                ui,
                &format!("meas:{name}"),
                |ui| {
                    let buf = self.fields.entry(key.clone()).or_insert_with(|| name.clone());
                    let resp = ui.add(egui::TextEdit::singleline(buf).font(egui::TextStyle::Monospace).frame(egui::Frame::NONE).desired_width(ui.available_width() - 2.0)).on_hover_text("Measurement name (usable in formulas); click to rename");
                    if resp.lost_focus() && *buf != *name {
                        rename = Some((name.clone(), buf.trim().to_string()));
                    }
                },
                |ui| {
                    match res {
                        Ok(m) => {
                            ui.label(RichText::new(format!("{} {}", fmt_num(m.value), dim_unit(m.dim))).color(theme::TEXT));
                        }
                        Err(e) => {
                            ui.add(crate::icons::image(ui.ctx(), "Warning", 14.0));
                            ui.add(egui::Label::new(RichText::new(e.to_string()).color(theme::ERROR)).truncate()).on_hover_text(e.to_string());
                        }
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        let r = ui.add(egui::Button::image(crate::icons::image(ui.ctx(), "edit-delete", 14.0)).frame(false)).on_hover_text("Delete measurement");
                        widgets::reg(format!("meas:del:{name}"), r.rect);
                        if r.clicked() {
                            delete = Some(name.clone());
                        }
                    });
                },
            );
        }
        if let Some((from, to)) = rename {
            let before = self.state_json();
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

        g.group(ui, "Add measurement");
        let w = ui.available_width() - g.name_w - 10.0;
        g.row(ui, "Kind", "What to measure", |ui| {
            let opts: Vec<(MeasKind, &str)> = MeasKind::ALL.iter().map(|k| (*k, k.label())).collect();
            widgets::combo(ui, "meas_kind", &mut self.meas_kind, &opts, w);
        });
        if self.meas_kind == MeasKind::Value {
            let names = self.doc.available_names();
            if !names.iter().any(|(n, _)| *n == self.meas_value_key) {
                self.meas_value_key = names.first().map(|(n, _)| n.clone()).unwrap_or_default();
            }
            g.row(ui, "Of", "The named value to report", |ui| {
                let labels: Vec<String> = names.iter().map(|(n, v)| format!("{n}  ({})", fmt_num(*v))).collect();
                let opts: Vec<(String, &str)> = names.iter().zip(&labels).map(|((n, _), l)| (n.clone(), l.as_str())).collect();
                widgets::combo(ui, "meas_value", &mut self.meas_value_key, &opts, w);
            });
        }
        let need = self.meas_kind.faces_needed();
        let faces: Vec<u32> = self.selection.iter().filter(|p| p.kind == PickKind::Face && p.object == 0).map(|p| p.id + 1).collect();
        if need > 0 {
            let hint = if faces.len() == need { format!("Face{}", faces.iter().map(|f| f.to_string()).collect::<Vec<_>>().join(", Face")) } else { format!("Select {need} face(s) in the 3D view") };
            g.row(ui, "Faces", "Ctrl/Cmd-click to select a second face", |ui| widgets::ro_value(ui, &hint));
        }
        let ready = need == 0 || faces.len() == need;
        ui.add_space(6.0);
        ui.horizontal(|ui| {
            ui.add_space(8.0);
            let r = ui.add_enabled(ready, egui::Button::image_and_text(crate::icons::image(ui.ctx(), "list-add", 14.0), "Add measurement"));
            widgets::reg("meas:add", r.rect);
            if r.clicked() {
                self.add_measurement(&faces);
            }
        });
        ui.add_space(6.0);
    }
}

