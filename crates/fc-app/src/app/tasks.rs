//! The Tasks panel over the right side of the 3D view (FreeCAD 1.1's overlay dock):
//! feature parameters with OK/Cancel, "Select attachment" for new sketches, and the
//! sketch editor's task boxes.

use super::*;
use crate::theme;
use crate::widgets::{self, task_box, task_row, PropGrid};
use egui::{Color32, RichText};

pub const TASK_W: f32 = 330.0;

/// Where feature parameters are drawn: the Data tab's property grid or a task box.
pub(super) enum Lay<'a> {
    Grid(&'a mut PropGrid),
    Task,
}

const SIDE_OPTS: [(SideType, &str); 3] = [(SideType::OneSide, "One sided"), (SideType::TwoSides, "Two sided"), (SideType::Symmetric, "Symmetric to plane")];
const EXTENT_OPTS: [(ExtentType, &str); 3] = [(ExtentType::Length, "Dimension"), (ExtentType::ThroughAll, "Through all"), (ExtentType::UpToFirst, "To first")];
const CUT_OPTS: [(HoleCut, &str); 3] = [(HoleCut::None, "None"), (HoleCut::Counterbore, "Counterbore"), (HoleCut::Countersink, "Countersink")];

impl App {
    /// A labeled row in either layout.
    fn prow<R: Default>(&mut self, ui: &mut egui::Ui, lay: &mut Lay, label: &str, tip: &str, f: impl FnOnce(&mut Self, &mut egui::Ui) -> R) -> R {
        match lay {
            Lay::Grid(g) => g.row(ui, label, tip, |ui| f(self, ui)),
            Lay::Task => task_row(ui, label, |ui| f(self, ui)),
        }
    }

    fn pbool(&mut self, ui: &mut egui::Ui, lay: &mut Lay, label: &str, tip: &str, v: &mut bool) -> bool {
        match lay {
            Lay::Grid(g) => g.row(ui, label, tip, |ui| widgets::yes_no(ui, v)),
            Lay::Task => task_row(ui, label, |ui| {
                let r = ui.checkbox(v, "");
                widgets::reg(format!("check:{label}"), r.rect);
                let r = if tip.is_empty() { r } else { r.on_hover_text(tip) };
                r.changed()
            }),
        }
    }

    fn pcombo<T: PartialEq + Clone>(&mut self, ui: &mut egui::Ui, lay: &mut Lay, id: &str, label: &str, tip: &str, v: &mut T, opts: &[(T, &str)]) -> bool {
        let task = matches!(lay, Lay::Task);
        self.prow(ui, lay, label, tip, |_, ui| {
            let w = if task { ui.available_width() - 2.0 } else { ui.available_width() - 2.0 };
            widgets::combo(ui, id, v, opts, w)
        })
    }

    fn pvalue(&mut self, ui: &mut egui::Ui, lay: &mut Lay, label: &str, tip: &str, key: &str, unit: &str) -> bool {
        self.prow(ui, lay, label, tip, |app, ui| {
            let w = ui.available_width() - 2.0;
            app.value_field(ui, key, unit, w)
        })
    }

    fn pro(&mut self, ui: &mut egui::Ui, lay: &mut Lay, label: &str, text: &str) {
        self.prow(ui, lay, label, "", |_, ui| widgets::ro_value(ui, text));
    }

    /// Kind-specific parameters of feature `id` (shared by the Data tab and the task panel).
    pub(super) fn feature_params(&mut self, ui: &mut egui::Ui, id: FeatureId, lay: &mut Lay) {
        let Some(f) = self.doc.get(id).cloned() else { return };
        let name = f.name.clone();
        let sketch_name = |doc: &Document, s: FeatureId| doc.get(s).map(|f| f.label.clone()).unwrap_or("?".into());
        match f.kind.clone() {
            FeatureKind::Sketch(sf) => {
                let support = match &sf.support {
                    Support::XY => "XY_Plane".to_string(),
                    Support::XZ => "XZ_Plane".to_string(),
                    Support::YZ => "YZ_Plane".to_string(),
                    Support::Face(fr) => format!("{}.Face{}", sketch_name(&self.doc, fr.feature), fr.index),
                };
                self.pro(ui, lay, "Attachment", &support);
                self.pvalue(ui, lay, "Attachment Offset", "Offset along the plane normal", &format!("{name}.AttachmentOffset"), "mm");
                let mut rev = sf.reversed;
                if self.pbool(ui, lay, "Reverse side", "Flip the sketch to the other side of its plane", &mut rev) {
                    self.change(|d| {
                        if let FeatureKind::Sketch(s) = &mut d.get_mut(id).unwrap().kind {
                            s.reversed = rev;
                        }
                    });
                }
                if let Some(rep) = self.doc.state(id).and_then(|s| s.solve.clone()) {
                    self.pro(ui, lay, "Degrees of freedom", &rep.dof.to_string());
                }
                self.pro(ui, lay, "Geometry", &sf.sketch.geometry.len().to_string());
                self.pro(ui, lay, "Constraints", &sf.sketch.constraints.len().to_string());
            }
            FeatureKind::Pad(e) | FeatureKind::Pocket(e) => {
                let mut ne = e.clone();
                self.pro(ui, lay, "Profile", &sketch_name(&self.doc, e.profile));
                self.pcombo(ui, lay, "ext_side", "Mode", "One sided, two sided (two lengths) or symmetric to the sketch plane", &mut ne.side, &SIDE_OPTS);
                self.pcombo(ui, lay, "ext_type", "Type", "How far to extrude", &mut ne.kind, &EXTENT_OPTS);
                if e.kind == ExtentType::Length {
                    self.pvalue(ui, lay, "Length", "Extrusion length", &format!("{name}.Length"), "mm");
                }
                if e.side == SideType::TwoSides {
                    self.pcombo(ui, lay, "ext_type2", "Type 2", "How far to extrude the second side", &mut ne.kind2, &EXTENT_OPTS);
                    if e.kind2 == ExtentType::Length {
                        self.pvalue(ui, lay, "Length 2", "Second side length", &format!("{name}.Length2"), "mm");
                    }
                }
                self.pbool(ui, lay, "Reversed", "Extrude to the other side of the sketch", &mut ne.reversed);
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
            }
            FeatureKind::Revolution(r) | FeatureKind::Groove(r) => {
                let mut nr = r.clone();
                let axis_name = |a: &AxisRef| match a {
                    AxisRef::SketchV => "Vertical sketch axis".to_string(),
                    AxisRef::SketchH => "Horizontal sketch axis".to_string(),
                    AxisRef::SketchLine(g) => format!("Construction line {}", g + 1),
                    AxisRef::X => "Base X axis".to_string(),
                    AxisRef::Y => "Base Y axis".to_string(),
                    AxisRef::Z => "Base Z axis".to_string(),
                };
                let lines: Vec<usize> = match self.doc.get(r.profile).map(|f| &f.kind) {
                    Some(FeatureKind::Sketch(s)) => s.sketch.geometry.iter().enumerate().filter(|(_, g)| g.construction && matches!(g.curve, fc_sketch::Curve::Line { .. })).map(|(i, _)| i).collect(),
                    _ => vec![],
                };
                let axes: Vec<AxisRef> = [AxisRef::SketchV, AxisRef::SketchH, AxisRef::X, AxisRef::Y, AxisRef::Z].into_iter().chain(lines.into_iter().map(AxisRef::SketchLine)).collect();
                let names: Vec<String> = axes.iter().map(axis_name).collect();
                let opts: Vec<(AxisRef, &str)> = axes.iter().cloned().zip(names.iter().map(|s| s.as_str())).collect();
                self.pro(ui, lay, "Profile", &sketch_name(&self.doc, r.profile));
                self.pcombo(ui, lay, "rev_axis", "Axis", "Axis to revolve about", &mut nr.axis, &opts);
                self.pcombo(ui, lay, "rev_side", "Mode", "One sided, two sided or symmetric to the sketch plane", &mut nr.side, &SIDE_OPTS);
                self.pvalue(ui, lay, "Angle", "Revolution angle", &format!("{name}.Angle"), "°");
                if r.side == SideType::TwoSides {
                    self.pvalue(ui, lay, "Angle 2", "Second side angle", &format!("{name}.Angle2"), "°");
                }
                self.pbool(ui, lay, "Reversed", "Revolve in the other direction", &mut nr.reversed);
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
                let mut nh = h.clone();
                self.pro(ui, lay, "Positions", &format!("Circle centers of {}", sketch_name(&self.doc, h.profile)));
                self.pvalue(ui, lay, "Diameter", "Hole diameter", &format!("{name}.Diameter"), "mm");
                let opts = [(false, "Dimension"), (true, "Through all")];
                self.pcombo(ui, lay, "hole_depth_type", "Depth type", "", &mut nh.through_all, &opts);
                if !h.through_all {
                    self.pvalue(ui, lay, "Depth", "Hole depth", &format!("{name}.Depth"), "mm");
                    self.pvalue(ui, lay, "Drill point angle", "0 = flat bottom", &format!("{name}.DrillPointAngle"), "°");
                }
                self.pcombo(ui, lay, "hole_cut", "Head type", "Counterbore or countersink", &mut nh.cut, &CUT_OPTS);
                if h.cut != HoleCut::None {
                    self.pvalue(ui, lay, "Head diameter", "", &format!("{name}.CutDiameter"), "mm");
                    if h.cut == HoleCut::Counterbore {
                        self.pvalue(ui, lay, "Head depth", "", &format!("{name}.CutDepth"), "mm");
                    } else {
                        self.pvalue(ui, lay, "Countersink angle", "", &format!("{name}.CountersinkAngle"), "°");
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
            FeatureKind::MateConnector(m) => {
                let anchor = match &m.anchor {
                    fc_part::Anchor::Origin => "Body origin".to_string(),
                    fc_part::Anchor::Topo { at, .. } => format!("{}.{}", sketch_name(&self.doc, at.feature), at.name()),
                };
                let task = matches!(lay, Lay::Task);
                if !task {
                    // the task panel shows these in its Origin box
                    self.pro(ui, lay, "Anchor", &anchor);
                }
                for (label, prop, unit) in [("Offset X", "OffsetX", "mm"), ("Offset Y", "OffsetY", "mm"), ("Offset Z", "OffsetZ", "mm"), ("Rotation X", "RotationX", "°"), ("Rotation Y", "RotationY", "°"), ("Rotation Z", "RotationZ", "°")] {
                    self.pvalue(ui, lay, label, "", &format!("{name}.{prop}"), unit);
                }
                let mut flip = m.flip_z;
                if !task && self.pbool(ui, lay, "Flip primary axis", "Reverse the connector's Z axis", &mut flip) {
                    self.change(|d| {
                        if let Some(FeatureKind::MateConnector(x)) = d.get_mut(id).map(|f| &mut f.kind) {
                            x.flip_z = flip;
                        }
                    });
                }
            }
            FeatureKind::AdditiveLoft(l) | FeatureKind::SubtractiveLoft(l) => {
                self.pro(ui, lay, "Profile", &sketch_name(&self.doc, l.profile));
                for (i, s) in l.sections.iter().enumerate() {
                    let n = sketch_name(&self.doc, *s);
                    self.pro(ui, lay, &format!("Section {}", i + 1), &n);
                }
                let mut ruled = l.ruled;
                if self.pbool(ui, lay, "Ruled surface", "Straight faces between the sections", &mut ruled) {
                    self.change(|d| {
                        if let Some(FeatureKind::AdditiveLoft(x) | FeatureKind::SubtractiveLoft(x)) = d.get_mut(id).map(|f| &mut f.kind) {
                            x.ruled = ruled;
                        }
                    });
                }
            }
            FeatureKind::AdditivePipe(p) | FeatureKind::SubtractivePipe(p) => {
                self.pro(ui, lay, "Profile", &sketch_name(&self.doc, p.profile));
                self.pro(ui, lay, "Path", &sketch_name(&self.doc, p.spine));
                let mut fr = p.frenet;
                if self.pbool(ui, lay, "Frenet", "Keep the profile's orientation with the Frenet frame of the path", &mut fr) {
                    self.change(|d| {
                        if let Some(FeatureKind::AdditivePipe(x) | FeatureKind::SubtractivePipe(x)) = d.get_mut(id).map(|f| &mut f.kind) {
                            x.frenet = fr;
                        }
                    });
                }
            }
        }
    }

    /// Accept the plane highlighted in the attachment list.
    pub(super) fn plane_pick_ok(&mut self) {
        let Some(i) = self.plane_choice else {
            self.status = "Select a plane from the list or click one in the 3D view".into();
            return;
        };
        if let Some(pl) = crate::plane_picker::planes().into_iter().nth(i) {
            self.end_plane_pick();
            self.create_sketch(pl.support);
        }
    }

    /// Docked column over the right edge of the 3D view (drawn in the view's layer after the
    /// view, so it takes the pointer there; the view keeps its size, so the camera does not
    /// jump, and fits and the navigation cube use the free part of the view).
    pub(super) fn task_panel(&mut self, ui: &mut egui::Ui) {
        let vr = self.view_rect;
        let rect = egui::Rect::from_min_max(egui::pos2(vr.right() - TASK_W, vr.top()), vr.max);
        // swallow clicks/drags/scrolls on the panel background
        ui.interact(rect, egui::Id::new("tasks_bg"), egui::Sense::click_and_drag());
        ui.painter().rect_filled(rect, 0.0, theme::PANEL);
        ui.painter().line_segment([rect.left_top(), rect.left_bottom()], egui::Stroke::new(1.0, theme::BORDER));
        let inner = egui::Rect::from_min_max(rect.min + egui::vec2(10.0, 0.0), rect.max - egui::vec2(10.0, 4.0));
        ui.scope_builder(egui::UiBuilder::new().max_rect(inner).layout(egui::Layout::top_down(egui::Align::Min)), |ui| {
            ui.set_clip_rect(inner.expand2(egui::vec2(4.0, 0.0)));
            ui.spacing_mut().scroll.floating = false;
            ui.spacing_mut().scroll.bar_width = 9.0;
            widgets::dock_title(ui, "Tasks");
            if self.editor.is_some() {
                self.sketch_task(ui);
            } else if self.active_assembly().is_some() {
                self.asm_task_panel(ui);
            } else if self.picking {
                self.attachment_task(ui);
            } else if self.task.is_some() {
                self.feature_task(ui);
            }
        });
    }

    /// Centered dialog buttons; returns (left clicked, right clicked).
    fn task_buttons(&mut self, ui: &mut egui::Ui, left: (&str, &str), right: Option<(&str, &str)>) -> (bool, bool) {
        self.task_buttons_ex(ui, left, right, true)
    }

    fn task_buttons_ex(&mut self, ui: &mut egui::Ui, left: (&str, &str), right: Option<(&str, &str)>, right_enabled: bool) -> (bool, bool) {
        let mut out = (false, false);
        let bw = 118.0;
        let total = if right.is_some() { bw * 2.0 + 8.0 } else { bw };
        ui.horizontal(|ui| {
            ui.add_space(((ui.available_width() - total) * 0.5).max(0.0));
            out.0 = widgets::push_button(ui, left.0, left.1, bw, right.is_none()).clicked();
            if let Some((id, text)) = right {
                let r = widgets::push_button_enabled(ui, id, text, bw, true, right_enabled);
                let r = if right_enabled { r } else { r.on_disabled_hover_text("Select a plane in the list or click one in the 3D view first") };
                out.1 = r.clicked();
            }
        });
        ui.add_space(8.0);
        out
    }

    fn sketch_task(&mut self, ui: &mut egui::Ui) {
        let (cancel, close) = self.task_buttons(ui, ("task:cancel", "Cancel"), Some(("task:close", "Close")));
        let mut exit = EditorExit::None;
        if close {
            exit = EditorExit::Close;
        }
        if cancel {
            exit = EditorExit::Cancel;
        }
        let mut ed = self.editor.take().unwrap();
        let exit = egui::ScrollArea::vertical().id_salt("sk_task_scroll").auto_shrink([false, true]).show(ui, |ui| ed.panel_ui(&mut self.doc, ui, exit)).inner;
        self.editor = Some(ed);
        if exit != EditorExit::None {
            self.finish_edit(exit);
        }
    }

    fn attachment_task(&mut self, ui: &mut egui::Ui) {
        let ok_enabled = self.plane_choice.is_some();
        let (cancel, ok) = self.task_buttons_ex(ui, ("task:cancel", "Cancel"), Some(("task:ok", "OK")), ok_enabled);
        let first = self.planes_at.unwrap_or(usize::MAX);
        let hovered_plane = self.hovered.as_ref().filter(|h| h.object >= first).map(|h| h.object - first);
        let mut accept = None;
        let mut any_hover = false;
        task_box(ui, "attach", "PartDesign_Plane", "Select attachment", |ui| {
            egui::Frame::new().fill(theme::INPUT).stroke(egui::Stroke::new(1.0, theme::BORDER)).corner_radius(3.0).inner_margin(3.0).show(ui, |ui| {
                ui.set_width(ui.available_width());
                for (i, p) in crate::plane_picker::planes().into_iter().enumerate() {
                    let label = format!("{}-plane (Base plane)", &p.name[..2]);
                    let sel = self.plane_choice == Some(i);
                    let w = ui.available_width();
                    let (rect, resp) = ui.allocate_exact_size(egui::vec2(w, 24.0), egui::Sense::click());
                    widgets::reg(format!("plane:{}", &p.name[..2]), rect);
                    let fill = if sel { theme::SELECTION } else if resp.hovered() || hovered_plane == Some(i) { theme::HOVER_ROW } else { Color32::TRANSPARENT };
                    ui.painter().rect_filled(rect, 2.0, fill);
                    let c = crate::plane_picker::plane_color(i);
                    let sw = egui::Rect::from_center_size(egui::pos2(rect.left() + 13.0, rect.center().y), egui::vec2(12.0, 12.0));
                    ui.painter().rect(sw, 2.0, Color32::from_rgba_unmultiplied((c[0] * 255.0) as u8, (c[1] * 255.0) as u8, (c[2] * 255.0) as u8, 150), egui::Stroke::new(1.0, Color32::from_rgb((c[0] * 255.0) as u8, (c[1] * 255.0) as u8, (c[2] * 255.0) as u8)), egui::StrokeKind::Inside);
                    ui.painter().text(egui::pos2(rect.left() + 28.0, rect.center().y), egui::Align2::LEFT_CENTER, &label, egui::FontId::proportional(13.0), theme::TEXT);
                    let resp = resp.on_hover_text(p.hint);
                    if resp.hovered() {
                        any_hover = true;
                        self.vp.force_hover(Some(Pick { object: first + i, kind: PickKind::Face, id: 0, point: [0.0; 3] }));
                    }
                    if resp.double_clicked() {
                        accept = Some(i);
                    } else if resp.clicked() {
                        self.plane_choice = Some(i);
                        self.sync_origin_highlight();
                    }
                }
            });
            ui.add_space(6.0);
            ui.label(RichText::new("Click a plane or a flat face of the part in the 3D view to sketch on it, or pick a base plane here and press OK.").color(Color32::from_gray(185)));
        });
        if !any_hover && hovered_plane.is_none() {
            self.vp.force_hover(None);
        }
        if let Some(i) = accept {
            self.plane_choice = Some(i);
            self.plane_pick_ok();
        } else if ok {
            self.plane_pick_ok();
        } else if cancel {
            self.end_plane_pick();
            self.vp.force_hover(None);
            self.status = "Ready".into();
        }
    }

    fn feature_task(&mut self, ui: &mut egui::Ui) {
        let (cancel, ok) = self.task_buttons(ui, ("task:cancel", "Cancel"), Some(("task:ok", "OK")));
        let Some(id) = self.task.as_ref().map(|t| t.id) else { return };
        let Some(f) = self.doc.get(id).cloned() else {
            self.task = None;
            return;
        };
        egui::ScrollArea::vertical().id_salt("feat_task_scroll").auto_shrink([false, true]).show(ui, |ui| {
            if let Some(e) = self.doc.state(id).and_then(|s| s.error.as_ref()).map(|e| e.to_string()) {
                egui::Frame::new().fill(Color32::from_rgb(70, 32, 30)).corner_radius(4.0).inner_margin(8.0).show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    ui.horizontal_wrapped(|ui| {
                        ui.add(crate::icons::image(ui.ctx(), "overlay_error", 16.0));
                        ui.label(RichText::new(e).color(Color32::from_rgb(255, 190, 180)));
                    });
                });
                ui.add_space(8.0);
            }
            if matches!(f.kind, FeatureKind::MateConnector(_)) {
                task_box(ui, "mate_ref", "Std_CoordinateSystem", "Origin", |ui| {
                    ui.spacing_mut().item_spacing.y = 6.0;
                    self.mate_ref_box(ui, id);
                });
            }
            let title = if matches!(f.kind, FeatureKind::MateConnector(_)) { "Offsets and rotation".to_string() } else { format!("{} parameters", feature_title(&f.kind)) };
            task_box(ui, "feature", feature_icon(&f.kind), &title, |ui| {
                ui.spacing_mut().item_spacing.y = 6.0;
                self.feature_params(ui, id, &mut Lay::Task);
            });
        });
        if ok {
            self.task_ok();
        } else if cancel {
            self.task_cancel();
        }
    }
}
