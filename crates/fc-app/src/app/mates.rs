//! Part Design mate connectors (Onshape style): the "Mate connector" command, its task
//! panel (pick a face/edge/vertex with a live triad preview at the inferred point,
//! inference choice, flip / reorient, offsets and rotations), and the connector triads in
//! the 3D view, which can be clicked to select the connector.

use super::*;
use crate::mate_ui::{self, Look};
use crate::theme;
use crate::widgets::{self, task_row};
use egui::{Color32, RichText};
use fc_part::{Anchor, Frame as MFrame, Infer, MateConnector, TopoKind};

/// The hovered element in the mate connector task: anchor and the snapped candidate.
#[derive(Clone)]
pub struct MateHover {
    pub kind: TopoKind,
    pub index: usize,
    pub infer: Infer,
    pub frame: MFrame,
    pub points: Vec<[f64; 3]>,
    pub chosen: usize,
}

impl App {
    /// The mate connector being created/edited in the Tasks panel.
    pub(super) fn mate_task(&self) -> Option<FeatureId> {
        let id = self.task.as_ref()?.id;
        matches!(self.doc.get(id)?.kind, FeatureKind::MateConnector(_)).then_some(id)
    }

    /// The solid feature a connector at tree position `id` anchors to (the last one before it).
    fn solid_before(&self, id: Option<FeatureId>) -> Option<(FeatureId, fc_occt::Shape)> {
        let end = id.and_then(|i| self.doc.index_of(i)).unwrap_or(self.doc.features.len());
        (0..end).rev().find_map(|i| {
            let f = &self.doc.features[i];
            if !f.kind.is_solid_feature() || f.suppressed {
                return None;
            }
            let st = self.doc.states.get(i)?;
            st.shape.clone().filter(|_| st.error.is_none()).map(|s| (f.id, s))
        })
    }

    /// "Mate connector": anchored to the selected face/edge/vertex if there is one, else at
    /// the body origin; the task panel then lets you pick or change the anchor.
    pub(super) fn add_mate_connector(&mut self) {
        if !self.idle() || self.active_part().is_none() {
            return;
        }
        let anchor = self.selection.iter().rev().find(|p| p.object == 0).and_then(|p| {
            let (fid, shape) = self.solid_before(None)?;
            let kind = mate_ui::topo_kind(p.kind);
            let index = p.id as usize + 1;
            let cands = mate_ui::candidates(&shape, kind, index);
            let i = self.vp.world_to_screen(p.point).and_then(|c| mate_ui::snap(&self.vp, &cands, c, &|f| *f)).unwrap_or(0);
            let infer = cands.get(i).map(|c| c.0).unwrap_or(Infer::Auto);
            fc_part::mate::topo_ref(fid, &shape, kind, index).ok().map(|at| Anchor::Topo { at, infer })
        });
        self.snapshot();
        let id = self.doc.add_mate_connector(MateConnector::new(anchor.unwrap_or_default()));
        self.after_change();
        self.selection.clear();
        self.vp.set_selection(vec![]);
        self.begin_task(id, true);
        self.status = "Click a face, edge or vertex: the connector snaps to the nearest inferred point".into();
    }

    fn set_mate_anchor(&mut self, id: FeatureId, h: &MateHover) {
        let Some((fid, shape)) = self.solid_before(Some(id)) else { return };
        let Ok(at) = fc_part::mate::topo_ref(fid, &shape, h.kind, h.index) else { return };
        let infer = h.infer;
        self.change(|d| {
            if let Some(FeatureKind::MateConnector(m)) = d.get_mut(id).map(|f| &mut f.kind) {
                m.anchor = Anchor::Topo { at, infer };
            }
        });
        self.status = format!("Anchored to {}{} ({})", h.kind.prefix(), h.index, mate_ui::infer_label(h.kind, mate_ui::element_geom(&shape, h.kind, h.index), infer).to_lowercase());
    }

    /// 3D view input in Part Design: connector task picking, triad clicks.
    pub(super) fn mate_view_input(&mut self, ui: &mut egui::Ui, ctx: &egui::Context, r: &crate::viewport::ViewportResponse) -> bool {
        let ptr = ctx.input(|i| i.pointer.hover_pos()).filter(|p| r.response.rect.contains(*p));
        if let Some(id) = self.mate_task() {
            self.mate_hover = None;
            if let (Some(h), Some(pos)) = (r.hovered.as_ref(), ptr) {
                if h.object == 0 && self.planes_at.is_none_or(|f| h.object < f) {
                    if let Some((_, shape)) = self.solid_before(Some(id)) {
                        let kind = mate_ui::topo_kind(h.kind);
                        let index = h.id as usize + 1;
                        let cands = mate_ui::candidates(&shape, kind, index);
                        if let Some(i) = mate_ui::snap(&self.vp, &cands, pos, &|f| *f) {
                            self.mate_hover = Some(MateHover { kind, index, infer: cands[i].0, frame: cands[i].1, points: cands.iter().map(|c| c.1.origin).collect(), chosen: i });
                        }
                    }
                }
            }
            if r.response.clicked() {
                if let Some(h) = self.mate_hover.clone() {
                    self.set_mate_anchor(id, &h);
                }
            }
            self.draw_part_connectors(ui);
            return true;
        }
        // clicking a triad selects its connector
        if r.response.clicked() {
            if let Some(pos) = r.response.interact_pointer_pos() {
                if let Some(id) = self.connector_at(pos) {
                    self.tree_sel = vec![id];
                    self.selection.clear();
                    self.vp.set_selection(vec![]);
                    self.status = format!("Selected: {}", self.doc.get(id).map(|f| f.label.clone()).unwrap_or_default());
                    self.draw_part_connectors(ui);
                    return true;
                }
            }
        }
        if r.response.double_clicked() {
            if let Some(id) = r.response.interact_pointer_pos().and_then(|p| self.connector_at(p)) {
                self.edit_feature(id);
                return true;
            }
        }
        false
    }

    /// Visible connectors of the open part: (feature, frame).
    fn part_triads(&self) -> Vec<(FeatureId, MFrame)> {
        if !self.body_visible {
            return vec![];
        }
        self.doc
            .features
            .iter()
            .filter(|f| matches!(f.kind, FeatureKind::MateConnector(_)) && !self.hidden_sketches.contains(&f.id))
            .filter_map(|f| self.doc.connector_frame(f.id).ok().map(|fr| (f.id, fr)))
            .collect()
    }

    fn connector_at(&self, pos: egui::Pos2) -> Option<FeatureId> {
        self.part_triads().into_iter().filter_map(|(id, f)| mate_ui::hit(&self.vp, &f, pos).map(|d| (d, id))).min_by(|a, b| a.0.total_cmp(&b.0)).map(|x| x.1)
    }

    /// Triads of the open part's connectors, plus the task's hover preview.
    pub(super) fn draw_part_connectors(&self, ui: &egui::Ui) {
        let vr = self.view_rect;
        let right = if self.task.is_some() || self.editor.is_some() || self.picking { vr.right() - tasks::TASK_W } else { vr.right() };
        let p = ui.painter().with_clip_rect(egui::Rect::from_min_max(vr.min, egui::pos2(right, vr.bottom())));
        let hover = ui.input(|i| i.pointer.hover_pos()).and_then(|pos| self.connector_at(pos));
        let editing = self.mate_task();
        for (id, f) in self.part_triads() {
            let look = if Some(id) == editing || self.tree_sel.contains(&id) { Look::Selected } else if Some(id) == hover && editing.is_none() { Look::Hover } else { Look::Normal };
            mate_ui::draw(&p, &self.vp, &f, look);
        }
        if let (Some(h), Some(_)) = (&self.mate_hover, editing) {
            mate_ui::draw_points(&p, &self.vp, &h.points, Some(h.chosen));
            mate_ui::draw(&p, &self.vp, &h.frame, Look::Ghost);
        }
    }

    /// The anchor part of the connector task: element, inference, flip and reorient.
    pub(super) fn mate_ref_box(&mut self, ui: &mut egui::Ui, id: FeatureId) {
        let Some(FeatureKind::MateConnector(m)) = self.doc.get(id).map(|f| f.kind.clone()) else { return };
        let shape = self.solid_before(Some(id));
        let (anchor_text, modes, cur_infer) = match &m.anchor {
            Anchor::Origin => ("Body origin".to_string(), vec![], Infer::Auto),
            Anchor::Topo { at, infer } => {
                let owner = self.doc.get(at.feature).map(|f| f.label.clone()).unwrap_or_else(|| "?".into());
                let modes: Vec<(Infer, String)> = shape
                    .as_ref()
                    .and_then(|(_, s)| fc_part::mate::find_topo(s, at).map(|i| (s, i)))
                    .map(|(s, i)| mate_ui::candidates(s, at.kind, i).into_iter().map(|(m, _)| (m, mate_ui::infer_label(at.kind, at.geom, m).to_string())).collect())
                    .unwrap_or_default();
                (format!("{owner}.{}", at.name()), modes, *infer)
            }
        };
        ui.horizontal_wrapped(|ui| {
            ui.add(crate::icons::image(ui.ctx(), "Std_CoordinateSystem", 16.0));
            ui.label(RichText::new("Click a face, edge or vertex in the 3D view. The connector snaps to the nearest inferred point (dots).").color(Color32::from_gray(185)));
        });
        ui.add_space(4.0);
        task_row(ui, "Origin entity", |ui| {
            let r = egui::Frame::new().fill(theme::INPUT).stroke(egui::Stroke::new(1.0, theme::BORDER)).corner_radius(3.0).inner_margin(egui::Margin::symmetric(6, 3)).show(ui, |ui| {
                ui.set_width(ui.available_width() - 4.0);
                ui.label(RichText::new(&anchor_text).color(theme::TEXT));
            });
            widgets::reg("mate:anchor", r.response.rect);
        });
        if !modes.is_empty() {
            let mut infer = cur_infer;
            let opts: Vec<(Infer, &str)> = modes.iter().map(|(m, l)| (*m, l.as_str())).collect();
            let mut v = if opts.iter().any(|(m, _)| *m == infer) { infer } else { opts[0].0 };
            let before = v;
            task_row(ui, "Origin type", |ui| {
                let w = ui.available_width() - 4.0;
                widgets::combo(ui, "mate_infer", &mut v, &opts, w)
            });
            if v != before {
                infer = v;
                self.change(|d| {
                    if let Some(FeatureKind::MateConnector(x)) = d.get_mut(id).map(|f| &mut f.kind) {
                        if let Anchor::Topo { infer: i, .. } = &mut x.anchor {
                            *i = infer;
                        }
                    }
                });
            }
        }
        ui.add_space(2.0);
        let (mut flip, mut turn) = (false, false);
        ui.horizontal(|ui| {
            let r = ui.add(egui::Button::image_and_text(crate::icons::image(ui.ctx(), "view-rotate-left", 14.0), "Flip primary axis").selected(m.flip_z)).on_hover_text("Reverse the connector's Z axis (a half turn about X)");
            widgets::reg("mate:flip", r.rect);
            flip = r.clicked();
            let r = ui.add(egui::Button::image_and_text(crate::icons::image(ui.ctx(), "view-rotate-right", 14.0), "Reorient secondary")).on_hover_text("Turn the X axis by 90° about Z");
            widgets::reg("mate:turn", r.rect);
            turn = r.clicked();
        });
        if flip || turn {
            self.change(|d| {
                if let Some(FeatureKind::MateConnector(x)) = d.get_mut(id).map(|f| &mut f.kind) {
                    if flip {
                        x.flip_z = !x.flip_z;
                    }
                    if turn {
                        x.x_turns = (x.x_turns + 1).rem_euclid(4);
                    }
                }
            });
        }
        ui.add_space(4.0);
    }
}
