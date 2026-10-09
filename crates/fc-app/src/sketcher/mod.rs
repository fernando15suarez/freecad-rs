//! Sketch edit mode (FreeCAD's Sketcher workbench while a sketch is open).
//!
//! The host (`app.rs`) owns the document; the editor edits the sketch feature in place and
//! keeps a snapshot so Cancel can restore it.

use crate::viewport::{Overlay, Viewport, ViewportResponse};
use eframe::egui;
use fc_part::{Document, FeatureId, Placement};
use fc_sketch::Sketch;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EditorExit {
    None,
    /// Keep changes (FreeCAD "Close").
    Close,
    /// Restore the sketch as it was when editing started.
    Cancel,
}

pub struct SketchEditor {
    feature: FeatureId,
    placement: Placement,
    original: Sketch,
    status: String,
}

impl SketchEditor {
    pub fn new(doc: &Document, feature: FeatureId) -> Option<Self> {
        let placement = doc.state(feature)?.placement?;
        let original = match &doc.get(feature)?.kind {
            fc_part::FeatureKind::Sketch(s) => s.sketch.clone(),
            _ => return None,
        };
        Some(SketchEditor { feature, placement, original, status: String::new() })
    }

    pub fn feature(&self) -> FeatureId {
        self.feature
    }

    pub fn placement(&self) -> Placement {
        self.placement
    }

    /// The sketch as it was when editing started (for Cancel).
    pub fn original(&self) -> &Sketch {
        &self.original
    }

    pub fn status(&self) -> &str {
        &self.status
    }

    /// Per-frame input handling after `vp.ui()`; also refreshes the overlay.
    pub fn handle_viewport(&mut self, doc: &mut Document, vp: &mut Viewport, _resp: &ViewportResponse, _ctx: &egui::Context) {
        let Some(sk) = doc.sketch_mut(self.feature) else { return };
        let fully = sk.fully_constrained_geometry();
        let mut o = Overlay::default();
        for (gi, (pts, constr)) in crate::scene::sketch_polylines(sk, &self.placement, true).into_iter().enumerate() {
            let c = if constr {
                [0.2, 0.4, 1.0, 1.0]
            } else if fully.get(gi).copied().unwrap_or(false) {
                [0.0, 0.8, 0.0, 1.0]
            } else {
                [1.0, 1.0, 1.0, 1.0]
            };
            for w in pts.windows(2) {
                o.lines.push((w[0], w[1], c, 2.0));
            }
        }
        vp.set_overlay(o);
    }

    pub fn toolbar_ui(&mut self, _doc: &mut Document, _ui: &mut egui::Ui) {}

    pub fn panel_ui(&mut self, doc: &mut Document, ui: &mut egui::Ui) -> EditorExit {
        let mut exit = EditorExit::None;
        ui.horizontal(|ui| {
            if ui.button("Close").clicked() {
                exit = EditorExit::Close;
            }
            if ui.button("Cancel").clicked() {
                exit = EditorExit::Cancel;
            }
        });
        if let Some(sk) = doc.sketch_mut(self.feature) {
            let rep = sk.diagnose();
            ui.label(format!("Degrees of freedom: {}", rep.dof));
        }
        exit
    }
}
