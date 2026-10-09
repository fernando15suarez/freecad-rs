//! Sketch edit mode (FreeCAD's Sketcher workbench while a sketch is open).
//!
//! The host (`app.rs`) owns the document; the editor edits the sketch feature in place and
//! keeps a snapshot so Cancel can restore it.
//!
//! Layout of this module:
//! - `mod.rs`: editor state, input plumbing (real pointer/keys and the test script feed the
//!   same event handlers), selection, solving, undo
//! - `tools.rs`: geometry creation tools with snapping and auto-constraints
//! - `commands.rs`: constraint commands acting on the selection, delete, toggles
//! - `render.rs`: drawing geometry, constraint icons and dimensions (egui painter on top of
//!   the 3D viewport)
//! - `panel.rs`: task panel (solver messages, constraint/element lists), toolbar, datum popup
//! - `script.rs`: `FCRS_SKETCH_SCRIPT` scripted-input test hook

mod commands;
mod geom;
mod panel;
mod pick;
mod render;
mod script;
mod tools;

use crate::viewport::{Overlay, Viewport, ViewportResponse};
use eframe::egui::{self, Pos2, Rect};
use fc_part::{Document, FeatureId, FeatureKind, Placement};
use fc_sketch::{Curve, GeoId, PointRef, Sketch, SolveReport, SolveStatus, Vec2};
use std::collections::{BTreeMap, HashMap};

pub use commands::Cmd;
pub use tools::Tool;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EditorExit {
    None,
    /// Keep changes (FreeCAD "Close").
    Close,
    /// Restore the sketch as it was when editing started.
    Cancel,
}

/// A selectable sketch element.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Sel {
    Point(PointRef),
    /// A curve, or the H/V axis.
    Edge(GeoId),
    Constraint(usize),
}

/// Sketch-plane <-> screen projection for one frame.
pub(crate) struct Proj<'a> {
    vp: &'a Viewport,
    pl: Placement,
}

impl Proj<'_> {
    /// Sketch point to screen.
    pub fn s(&self, p: Vec2) -> Pos2 {
        self.vp.world_to_screen(self.pl.to_world(p.x, p.y)).unwrap_or(Pos2::new(-1e5, -1e5))
    }
    /// Screen to sketch point.
    pub fn w(&self, s: Pos2) -> Option<Vec2> {
        let w = self.vp.screen_to_plane(s, &crate::scene::plane(&self.pl))?;
        let l = self.pl.to_local(w);
        Some(Vec2::new(l[0], l[1]))
    }
    /// Screen points per sketch unit.
    pub fn ppu(&self) -> f64 {
        self.vp.pixels_per_unit_at(self.pl.origin).max(1e-9)
    }
}

/// Input event, from the real pointer/keyboard or from the test script.
#[derive(Clone, Copy, Debug)]
pub(crate) enum Ev {
    Move(Pos2),
    /// Primary button pressed (drawing tools act on press, like FreeCAD).
    Press(Pos2),
    /// Primary click without drag (selection).
    Click(Pos2),
    DoubleClick(Pos2),
    RightClick(Pos2),
    DragStart(Pos2),
    DragMove(Pos2),
    DragEnd(Pos2),
    Key(egui::Key, egui::Modifiers),
}

#[derive(Clone, Debug)]
enum Drag {
    None,
    /// Dragging geometry through the solver; `offset` = grab point - dragged point.
    Geo { pr: PointRef, offset: Vec2 },
    /// Moving a dimension label.
    Label(usize),
    /// Rubber-band selection.
    Box { start: Pos2, cur: Pos2 },
}

/// Datum popup (FreeCAD's "Insert length" dialog).
pub(crate) struct DimEdit {
    ci: usize,
    /// Other dimensions created by the same command (they get the same value).
    also: Vec<usize>,
    value: String,
    name: String,
    reference: bool,
    created: bool,
    pos: Pos2,
    error: Option<String>,
    focus: bool,
}

/// Deletion refused because formulas use the dimensions being deleted.
pub(crate) struct PendingDelete {
    constraints: Vec<usize>,
    geometry: Vec<GeoId>,
    users: Vec<String>,
}

#[derive(Clone)]
struct Snapshot {
    sketch: Sketch,
    expressions: BTreeMap<String, String>,
    parameters: BTreeMap<String, String>,
}

pub struct SketchEditor {
    feature: FeatureId,
    placement: Placement,
    original: Sketch,
    /// Formulas/parameters when editing started (Cancel restores them too).
    original_exprs: (BTreeMap<String, String>, BTreeMap<String, String>),
    status: String,
    tool: tools::ToolState,
    sel: Vec<Sel>,
    hover: Option<Sel>,
    /// Element hovered in the panel lists (highlighted in the view).
    panel_hover: Option<Sel>,
    /// New geometry is construction geometry.
    construction: bool,
    auto_constraints: bool,
    grid: bool,
    show_constraints: bool,
    report: Option<SolveReport>,
    fully: Vec<bool>,
    expr_errors: Vec<String>,
    drag: Drag,
    dim_edit: Option<DimEdit>,
    pending_delete: Option<PendingDelete>,
    /// Last notification (text, is_error).
    message: Option<(String, bool)>,
    undo: Vec<Snapshot>,
    redo: Vec<Snapshot>,
    /// Dimension label placement overrides (see `render::DimFrame`), per constraint index.
    label_off: HashMap<usize, f64>,
    cursor: Option<Vec2>,
    cursor_px: Option<Pos2>,
    /// Script-driven pointer position (when the real pointer is not over the viewport).
    virtual_cursor: Option<Pos2>,
    /// Constraint icon / dimension label hit regions from the last render.
    hits: Vec<(usize, Rect)>,
    rect: Rect,
    script: Option<script::Script>,
    key_prefix: Option<(egui::Key, f64)>,
    overlay_cleared: bool,
    panel: panel::PanelState,
    /// Fit the view to the sketch on the next frame.
    want_fit: bool,
    frame: u64,
}

static EMPTY: std::sync::LazyLock<Sketch> = std::sync::LazyLock::new(Sketch::default);

impl SketchEditor {
    pub fn new(doc: &Document, feature: FeatureId) -> Option<Self> {
        let placement = doc.state(feature)?.placement?;
        let original = match &doc.get(feature)?.kind {
            FeatureKind::Sketch(s) => s.sketch.clone(),
            _ => return None,
        };
        let script = std::env::var("FCRS_SKETCH_SCRIPT").ok().and_then(|p| script::Script::load(&p));
        Some(SketchEditor {
            feature,
            placement,
            original,
            original_exprs: (doc.expressions.clone(), doc.parameters.clone()),
            status: String::new(),
            tool: tools::ToolState::new(Tool::Select),
            sel: vec![],
            hover: None,
            panel_hover: None,
            construction: false,
            auto_constraints: true,
            grid: false,
            show_constraints: true,
            report: None,
            fully: vec![],
            expr_errors: vec![],
            drag: Drag::None,
            dim_edit: None,
            pending_delete: None,
            message: None,
            undo: vec![],
            redo: vec![],
            label_off: HashMap::new(),
            cursor: None,
            cursor_px: None,
            virtual_cursor: None,
            hits: vec![],
            rect: Rect::from_min_size(Pos2::ZERO, egui::vec2(800.0, 600.0)),
            script,
            key_prefix: None,
            overlay_cleared: false,
            panel: panel::PanelState::default(),
            want_fit: false,
            frame: 0,
        })
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

    /// True while the test script still has steps to run (the screenshot hook waits).
    pub fn busy(&self) -> bool {
        self.script.as_ref().is_some_and(|s| s.busy())
    }

    // ------------------------------------------------------------------ helpers

    fn sk<'a>(&self, doc: &'a Document) -> &'a Sketch {
        match doc.get(self.feature).map(|f| &f.kind) {
            Some(FeatureKind::Sketch(s)) => &s.sketch,
            _ => &EMPTY,
        }
    }

    fn sketch_name(&self, doc: &Document) -> String {
        doc.get(self.feature).map(|f| f.name.clone()).unwrap_or_default()
    }

    /// Canonical expression key of a constraint (`Sketch.Constraints[i]`).
    fn ckey(&self, doc: &Document, ci: usize) -> String {
        format!("{}.Constraints[{ci}]", self.sketch_name(doc))
    }

    fn is_sel(&self, s: Sel) -> bool {
        self.sel.contains(&s)
    }

    fn toggle_sel(&mut self, s: Sel) {
        if let Some(i) = self.sel.iter().position(|x| *x == s) {
            self.sel.remove(i);
        } else {
            self.sel.push(s);
        }
    }

    fn notify(&mut self, msg: impl Into<String>, error: bool) {
        self.message = Some((msg.into(), error));
    }

    fn push_undo(&mut self, doc: &Document) {
        self.undo.push(Snapshot { sketch: self.sk(doc).clone(), expressions: doc.expressions.clone(), parameters: doc.parameters.clone() });
        if self.undo.len() > 200 {
            self.undo.remove(0);
        }
        self.redo.clear();
    }

    fn restore(&mut self, doc: &mut Document, s: Snapshot) {
        if let Some(sk) = doc.sketch_mut(self.feature) {
            *sk = s.sketch;
        }
        doc.expressions = s.expressions;
        doc.parameters = s.parameters;
        self.sel.clear();
        self.hover = None;
        self.label_off.clear();
        self.resolve(doc);
    }

    fn undo_step(&mut self, doc: &mut Document) {
        if let Some(s) = self.undo.pop() {
            let cur = Snapshot { sketch: self.sk(doc).clone(), expressions: doc.expressions.clone(), parameters: doc.parameters.clone() };
            self.redo.push(cur);
            self.restore(doc, s);
        }
    }

    fn redo_step(&mut self, doc: &mut Document) {
        if let Some(s) = self.redo.pop() {
            let cur = Snapshot { sketch: self.sk(doc).clone(), expressions: doc.expressions.clone(), parameters: doc.parameters.clone() };
            self.undo.push(cur);
            self.restore(doc, s);
        }
    }

    /// Re-evaluate formulas and re-solve the sketch (every edit goes through here).
    fn resolve(&mut self, doc: &mut Document) {
        let errs = doc.apply_expressions();
        let has_expr = !doc.expressions.is_empty();
        let Some(sk) = doc.sketch_mut(self.feature) else { return };
        let mut rep = sk.solve();
        if has_expr && rep.is_ok() {
            // formulas may read reference dimensions measured by the solve
            doc.apply_expressions();
            if let Some(sk) = doc.sketch_mut(self.feature) {
                rep = sk.solve();
            }
        }
        self.fully = self.sk(doc).fully_constrained_geometry();
        self.report = Some(rep);
        self.expr_errors = errs.iter().map(|(k, e)| format!("{}: {e}", doc.display_name(k))).collect();
        // drop stale selection
        let sk = self.sk(doc);
        let (ng, nc) = (sk.geometry.len(), sk.constraints.len());
        self.sel.retain(|s| match s {
            Sel::Point(p) => fc_sketch::is_external(p.geo) || p.geo < ng,
            Sel::Edge(g) => fc_sketch::is_external(*g) || *g < ng,
            Sel::Constraint(c) => *c < nc,
        });
        self.label_off.retain(|c, _| *c < nc);
    }

    fn invalid(&self) -> bool {
        self.report.as_ref().is_some_and(|r| !r.conflicting.is_empty() || !r.redundant.is_empty() || !r.malformed.is_empty() || r.status == SolveStatus::Conflicting)
    }

    fn fully_constrained(&self, doc: &Document) -> bool {
        self.report.as_ref().is_some_and(|r| r.dof == 0 && r.is_ok() && !self.sk(doc).geometry.is_empty())
    }

    /// Element under a screen position: points first, then constraint labels, then curves.
    fn pick(&self, sk: &Sketch, pj: &Proj, pos: Pos2) -> Option<Sel> {
        if let Some(p) = pick::nearest_point(sk, pj, pos, pick::POINT_PX) {
            return Some(Sel::Point(p));
        }
        if self.show_constraints {
            if let Some((ci, _)) = self.hits.iter().rev().find(|(_, r)| r.expand(2.0).contains(pos)) {
                return Some(Sel::Constraint(*ci));
            }
        }
        pick::nearest_edge(sk, pj, pos, pick::EDGE_PX, true).map(|(g, _)| Sel::Edge(g))
    }

    // -------------------------------------------------------------- per frame

    /// Per-frame input handling after `vp.ui()`; draws the sketch over the viewport.
    pub fn handle_viewport(&mut self, doc: &mut Document, vp: &mut Viewport, resp: &ViewportResponse, ctx: &egui::Context) {
        self.frame += 1;
        vp.set_sketch_mode();
        if !self.overlay_cleared {
            // the editor draws everything itself; drop the non-edit sketch overlay
            vp.set_overlay(Overlay::default());
            self.overlay_cleared = true;
        }
        if self.report.is_none() {
            self.resolve(doc);
        }
        if self.want_fit {
            self.want_fit = false;
            self.fit_view(doc, vp);
        }
        self.rect = resp.response.rect;

        // scripted input (one step per frame)
        if self.script.is_some() {
            self.script_step(doc, vp, ctx);
        }

        let pj = Proj { vp, pl: self.placement };
        let evs = self.collect_events(resp, ctx);
        for ev in evs {
            self.handle_event(doc, &pj, ev);
        }
        let cur = resp.response.hover_pos().or(self.virtual_cursor);
        self.cursor_px = cur;
        self.cursor = cur.and_then(|p| pj.w(p));
        if matches!(self.drag, Drag::None) && self.tool.tool == Tool::Select && self.dim_edit.is_none() {
            self.hover = cur.and_then(|p| self.pick(self.sk(doc), &pj, p)).or(self.panel_hover);
        } else if self.tool.tool != Tool::Select {
            self.hover = None;
        }

        let painter = ctx.layer_painter(resp.response.layer_id).with_clip_rect(self.rect);
        self.render(doc, &pj, &painter);
        self.dim_edit_ui(doc, ctx);
        self.update_status(doc);
        if self.script.is_some() || !matches!(self.drag, Drag::None) {
            ctx.request_repaint();
        }
    }

    fn collect_events(&mut self, resp: &ViewportResponse, ctx: &egui::Context) -> Vec<Ev> {
        let r = &resp.response;
        let mut evs = Vec::new();
        let ptr = r.hover_pos();
        if let Some(p) = ptr {
            evs.push(Ev::Move(p));
            self.virtual_cursor = None;
        }
        let popup = self.dim_edit.is_some();
        let (pressed, origin) = ctx.input(|i| (i.pointer.primary_pressed(), i.pointer.press_origin()));
        if !popup {
            if self.tool.tool != Tool::Select {
                if pressed && r.hovered() {
                    if let Some(p) = origin {
                        evs.push(Ev::Press(p));
                    }
                }
            } else {
                let ip = r.interact_pointer_pos().or(ptr);
                if r.drag_started_by(egui::PointerButton::Primary) {
                    if let Some(p) = origin.or(ip) {
                        evs.push(Ev::DragStart(p));
                    }
                }
                if r.dragged_by(egui::PointerButton::Primary) {
                    if let Some(p) = ip {
                        evs.push(Ev::DragMove(p));
                    }
                }
                if r.drag_stopped_by(egui::PointerButton::Primary) {
                    if let Some(p) = ip {
                        evs.push(Ev::DragEnd(p));
                    }
                }
                if r.clicked() {
                    if let Some(p) = ip {
                        evs.push(Ev::Click(p));
                    }
                }
                if r.double_clicked() {
                    if let Some(p) = ip {
                        evs.push(Ev::DoubleClick(p));
                    }
                }
            }
            if r.secondary_clicked() {
                if let Some(p) = r.interact_pointer_pos().or(ptr) {
                    evs.push(Ev::RightClick(p));
                }
            }
        }
        if !ctx.egui_wants_keyboard_input() && !popup {
            let keys: Vec<(egui::Key, egui::Modifiers)> = ctx.input(|i| {
                i.events
                    .iter()
                    .filter_map(|e| match e {
                        egui::Event::Key { key, pressed: true, repeat: false, modifiers, .. } => Some((*key, *modifiers)),
                        _ => None,
                    })
                    .collect()
            });
            for (k, m) in keys {
                evs.push(Ev::Key(k, m));
            }
        }
        if let Some((_, t)) = self.key_prefix {
            if now() - t > 1.5 {
                self.key_prefix = None;
            }
        }
        evs
    }

    fn handle_event(&mut self, doc: &mut Document, pj: &Proj, ev: Ev) {
        match ev {
            Ev::Move(p) => {
                if self.tool.tool != Tool::Select {
                    self.tool_move(doc, pj, p);
                }
            }
            Ev::Press(p) => self.tool_press(doc, pj, p),
            Ev::Click(p) => {
                let hit = self.pick(self.sk(doc), pj, p);
                match hit {
                    Some(s) => self.toggle_sel(s),
                    None => self.sel.clear(),
                }
            }
            Ev::DoubleClick(p) => {
                if let Some(Sel::Constraint(ci)) = self.pick(self.sk(doc), pj, p) {
                    if self.sk(doc).constraints.get(ci).is_some_and(|c| c.kind.is_dimensional()) {
                        self.sel = vec![Sel::Constraint(ci)];
                        self.open_dim_edit(doc, ci, false, Some(p));
                    }
                }
            }
            Ev::RightClick(p) => self.tool_right_click(doc, pj, p),
            Ev::DragStart(p) => self.drag_start(doc, pj, p),
            Ev::DragMove(p) => self.drag_move(doc, pj, p),
            Ev::DragEnd(p) => self.drag_end(doc, pj, p),
            Ev::Key(k, m) => self.key(doc, k, m),
        }
    }

    // --------------------------------------------------------------- dragging

    fn drag_start(&mut self, doc: &mut Document, pj: &Proj, pos: Pos2) {
        let sk = self.sk(doc);
        let raw = pj.w(pos).unwrap_or(Vec2::ZERO);
        self.drag = match self.pick(sk, pj, pos) {
            Some(Sel::Point(pr)) if !fc_sketch::is_external(pr.geo) => Drag::Geo { pr, offset: Vec2::ZERO },
            Some(Sel::Edge(g)) if !fc_sketch::is_external(g) => match &sk.geometry[g].curve {
                Curve::Line { a, b } => Drag::Geo { pr: PointRef::edge(g), offset: raw - a.lerp(*b, 0.5) },
                Curve::Point { .. } => Drag::Geo { pr: PointRef::start(g), offset: Vec2::ZERO },
                _ => Drag::Geo { pr: PointRef::edge(g), offset: Vec2::ZERO },
            },
            Some(Sel::Constraint(ci)) if sk.constraints[ci].kind.is_dimensional() => Drag::Label(ci),
            _ => Drag::Box { start: pos, cur: pos },
        };
        if matches!(self.drag, Drag::Geo { .. }) {
            self.push_undo(doc);
        }
    }

    fn drag_move(&mut self, doc: &mut Document, pj: &Proj, pos: Pos2) {
        let Some(w) = pj.w(pos) else { return };
        match self.drag.clone() {
            Drag::Geo { pr, offset } => {
                if let Some(sk) = doc.sketch_mut(self.feature) {
                    let rep = sk.drag_point(pr, w - offset);
                    if rep.is_ok() {
                        self.report = Some(rep);
                    }
                }
            }
            Drag::Label(ci) => {
                let sk = self.sk(doc);
                if let Some(f) = self.dim_frame(sk, ci) {
                    let v = f.param_from_cursor(w);
                    self.label_off.insert(ci, v);
                }
            }
            Drag::Box { start, .. } => self.drag = Drag::Box { start, cur: pos },
            Drag::None => {}
        }
    }

    fn drag_end(&mut self, doc: &mut Document, pj: &Proj, pos: Pos2) {
        self.drag_move(doc, pj, pos);
        match std::mem::replace(&mut self.drag, Drag::None) {
            Drag::Geo { .. } => self.resolve(doc),
            Drag::Box { start, cur } => {
                let r = Rect::from_two_pos(start, cur);
                let sk = self.sk(doc);
                let mut add = vec![];
                for pr in geom::all_points(sk) {
                    if pr.geo == fc_sketch::ROOT {
                        continue;
                    }
                    if r.contains(pj.s(sk.point(pr))) {
                        add.push(Sel::Point(pr));
                    }
                }
                for (gi, g) in sk.geometry.iter().enumerate() {
                    if matches!(g.curve, Curve::Point { .. }) {
                        continue;
                    }
                    if pick::screen_polyline(pj, &g.curve).iter().all(|p| r.contains(*p)) {
                        add.push(Sel::Edge(gi));
                    }
                }
                for s in add {
                    if !self.sel.contains(&s) {
                        self.sel.push(s);
                    }
                }
            }
            _ => {}
        }
    }

    // ---------------------------------------------------------------- keyboard

    fn key(&mut self, doc: &mut Document, k: egui::Key, m: egui::Modifiers) {
        use egui::Key;
        if m.command {
            match k {
                Key::Z if m.shift => self.redo_step(doc),
                Key::Z => self.undo_step(doc),
                Key::Y => self.redo_step(doc),
                _ => {}
            }
            return;
        }
        match k {
            Key::Escape => {
                self.key_prefix = None;
                self.escape(doc);
                return;
            }
            Key::Delete | Key::Backspace => {
                self.delete_selection(doc, fc_part::params::OnUsers::Refuse);
                return;
            }
            _ => {}
        }
        if let Some((pre, _)) = self.key_prefix.take() {
            if pre == Key::G {
                let t = match k {
                    Key::L => Some(Tool::Line),
                    Key::M => Some(Tool::Polyline),
                    Key::R => Some(Tool::Rectangle),
                    Key::V => Some(Tool::CenterRectangle),
                    Key::C => Some(Tool::Circle),
                    Key::A => Some(Tool::Arc),
                    Key::Num3 => Some(Tool::Arc3),
                    Key::S => Some(Tool::Slot),
                    Key::Y => Some(Tool::Point),
                    Key::N => {
                        self.toggle_construction(doc);
                        None
                    }
                    _ => None,
                };
                if let Some(t) = t {
                    self.set_tool(t);
                }
            } else if pre == Key::K {
                let c = match k {
                    Key::D => Some(Cmd::Distance),
                    Key::R => Some(Cmd::Radius),
                    Key::O => Some(Cmd::Diameter),
                    Key::A => Some(Cmd::Angle),
                    Key::L => Some(Cmd::Lock),
                    Key::B => Some(Cmd::Block),
                    Key::H => Some(Cmd::DistanceX),
                    Key::V => Some(Cmd::DistanceY),
                    _ => None,
                };
                if let Some(c) = c {
                    self.run_cmd_ui(doc, c);
                }
            }
            return;
        }
        let cmd = match k {
            Key::G | Key::K => {
                self.key_prefix = Some((k, now()));
                return;
            }
            Key::H => Some(Cmd::Horizontal),
            Key::V => Some(Cmd::Vertical),
            Key::C => Some(Cmd::Coincident),
            Key::O => Some(Cmd::PointOnObject),
            Key::P => Some(Cmd::Parallel),
            Key::N => Some(Cmd::Perpendicular),
            Key::T => Some(Cmd::Tangent),
            Key::E => Some(Cmd::Equal),
            Key::S => Some(Cmd::Symmetric),
            Key::L => Some(Cmd::DistanceX),
            Key::I => Some(Cmd::DistanceY),
            Key::D => Some(Cmd::Distance),
            Key::R => Some(Cmd::Radius),
            Key::A => Some(Cmd::Angle),
            _ => None,
        };
        if let Some(c) = cmd {
            self.run_cmd_ui(doc, c);
        }
    }

    /// Esc: abort the current operation, then leave the tool, then clear the selection.
    fn escape(&mut self, doc: &mut Document) {
        if !self.tool.pts.is_empty() || self.tool.poly_prev.is_some() {
            self.tool.reset();
        } else if self.tool.tool != Tool::Select {
            self.set_tool(Tool::Select);
        } else if !matches!(self.drag, Drag::None) {
            self.drag = Drag::None;
            self.resolve(doc);
        } else {
            self.sel.clear();
        }
    }

    pub(crate) fn set_tool(&mut self, t: Tool) {
        self.tool = tools::ToolState::new(t);
        self.hover = None;
        if t != Tool::Select {
            // FreeCAD clears the selection when a creation tool starts
            self.sel.clear();
        }
    }

    fn fit_view(&mut self, doc: &Document, vp: &mut Viewport) {
        let (mn, mx) = geom::bbox(self.sk(doc));
        let pad = ((mx - mn).length() * 0.18).max(5.0);
        let pl = self.placement;
        let a = pl.to_world(mn.x - pad, mn.y - pad);
        let b = pl.to_world(mx.x + pad, mx.y + pad);
        vp.fit_box([a[0].min(b[0]), a[1].min(b[1]), a[2].min(b[2])], [a[0].max(b[0]), a[1].max(b[1]), a[2].max(b[2])]);
    }

    fn update_status(&mut self, doc: &Document) {
        let hint = match self.tool.tool {
            Tool::Select => match (&self.drag, &self.hover) {
                (Drag::Geo { .. }, _) => "Dragging (the solver keeps the constraints)".to_string(),
                (Drag::Label(_), _) => "Moving dimension label".to_string(),
                (_, Some(h)) => format!("Preselected: {}", self.sel_name(doc, *h)),
                _ => "Select elements (click toggles, drag a box); drag points/edges to move; double-click a dimension to edit".to_string(),
            },
            t => t.hint(self.tool.pts.len(), self.tool.poly_prev.is_some()).to_string(),
        };
        let mode = if self.construction { "  [construction]" } else { "" };
        self.status = match self.cursor {
            Some(c) => format!("{hint}{mode}    ({}, {})", geom::fmt_num(c.x), geom::fmt_num(c.y)),
            None => format!("{hint}{mode}"),
        };
    }

    fn sel_name(&self, doc: &Document, s: Sel) -> String {
        let sk = self.sk(doc);
        match s {
            Sel::Point(p) if p.geo == fc_sketch::ROOT => "RootPoint".into(),
            Sel::Point(p) => format!("Vertex of {} ({})", geom::geo_name(p.geo), geom::pos_name(p.pos)),
            Sel::Edge(g) => match sk.geometry.get(g) {
                Some(gg) => format!("{} ({})", geom::geo_name(g), geom::curve_name(&gg.curve)),
                None => geom::geo_name(g),
            },
            Sel::Constraint(c) => match sk.constraints.get(c) {
                Some(cc) => format!("Constraint{} ({})", c + 1, geom::kind_name(&cc.kind)),
                None => format!("Constraint{}", c + 1),
            },
        }
    }
}

fn now() -> f64 {
    std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0)
}
