#![allow(dead_code)] // public API surface for the editor; not all used by the demo shell yet
//! 3D viewport: camera, picking, GL painting via egui paint callback.
use crate::camera::{self, Camera, FOV_Y};
use crate::gpu::{self, Frame, Gpu, HlData, ObjUpload};
use eframe::egui::{self, Color32, Pos2, Rect, Sense, Vec2};
use glam::{DMat4, DVec3, DVec4};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------- public data

pub struct SceneMesh {
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
    /// face id per triangle
    pub tri_face: Vec<u32>,
}
/// polyline per B-rep edge
pub struct SceneEdge {
    pub id: u32,
    pub points: Vec<[f32; 3]>,
}
pub struct SceneObject {
    pub name: String,
    pub mesh: SceneMesh,
    pub edges: Vec<SceneEdge>,
    pub visible: bool,
    pub color: [f32; 4],
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PickKind {
    Face,
    Edge,
    Vertex,
}
#[derive(Clone, Debug, PartialEq)]
pub struct Pick {
    pub object: usize,
    pub kind: PickKind,
    pub id: u32,
    pub point: [f64; 3],
}
/// Drawn on top (no depth test), for the sketch editor.
#[derive(Clone, Debug, Default)]
pub struct Overlay {
    /// (a, b, rgba, width in px)
    pub lines: Vec<([f32; 3], [f32; 3], [f32; 4], f32)>,
    /// (position, rgba, size in px)
    pub points: Vec<([f32; 3], [f32; 4], f32)>,
}
#[derive(Clone, Debug)]
pub struct Plane {
    pub origin: [f64; 3],
    pub normal: [f64; 3],
    pub xdir: [f64; 3],
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum StandardView {
    Isometric,
    Front,
    Top,
    Right,
    Rear,
    Bottom,
    Left,
}
pub struct ViewportResponse {
    pub hovered: Option<Pick>,
    /// Some when the primary button was clicked over a pick. If `response.clicked()`
    /// is true and this is None, the click hit empty space.
    pub clicked: Option<Pick>,
    pub response: egui::Response,
}

// ------------------------------------------------------------------- viewport

const BG_TOP: [f32; 3] = [0.62, 0.71, 0.84];
const BG_BOT: [f32; 3] = [0.89, 0.91, 0.95];
const EDGE_COLOR: [f32; 4] = [0.0, 0.0, 0.0, 1.0];
const EDGE_WIDTH: f32 = 1.5;
const HL_EDGE_WIDTH: f32 = 3.5;
const EDGE_PICK_PX: f64 = 5.0;

struct ObjCache {
    min: DVec3,
    max: DVec3,
}

pub struct Viewport {
    cam: Camera,
    objects: Vec<SceneObject>,
    cache: Vec<ObjCache>,
    uploads: Arc<Vec<ObjUpload>>,
    obj_gen: u64,
    overlay: Arc<Vec<f32>>,
    ov_gen: u64,
    hl: Arc<HlData>,
    hl_gen: u64,
    selection: Vec<Pick>,
    hover: Option<Pick>,
    forced_hover: Option<Pick>,
    rect: Rect,
    ppp: f32,
    pending_fit: bool,
    rect_known: bool,
    gpu: Arc<Mutex<Option<Gpu>>>,
}

impl Viewport {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        Viewport {
            cam: Camera::new(),
            objects: vec![],
            cache: vec![],
            uploads: Arc::new(vec![]),
            obj_gen: 0,
            overlay: Arc::new(vec![]),
            ov_gen: 0,
            hl: Arc::new(HlData::default()),
            hl_gen: 0,
            selection: vec![],
            hover: None,
            forced_hover: None,
            rect: Rect::from_min_size(Pos2::ZERO, Vec2::new(900.0, 600.0)),
            ppp: 1.0,
            pending_fit: true,
            rect_known: false,
            gpu: Arc::new(Mutex::new(None)),
        }
    }

    pub fn objects(&self) -> &[SceneObject] {
        &self.objects
    }
    pub fn perspective(&self) -> bool {
        self.cam.perspective
    }
    pub fn set_perspective(&mut self, on: bool) {
        self.cam.perspective = on;
    }

    /// Replaces the scene and re-uploads GPU buffers.
    pub fn set_objects(&mut self, objs: Vec<SceneObject>) {
        let first = self.objects.is_empty();
        self.cache = objs
            .iter()
            .map(|o| {
                let mut mn = DVec3::splat(f64::MAX);
                let mut mx = DVec3::splat(f64::MIN);
                for p in &o.mesh.positions {
                    let v = DVec3::new(p[0] as f64, p[1] as f64, p[2] as f64);
                    mn = mn.min(v);
                    mx = mx.max(v);
                }
                for e in &o.edges {
                    for p in &e.points {
                        let v = DVec3::new(p[0] as f64, p[1] as f64, p[2] as f64);
                        mn = mn.min(v);
                        mx = mx.max(v);
                    }
                }
                if mn.x > mx.x {
                    mn = DVec3::ZERO;
                    mx = DVec3::ZERO;
                }
                ObjCache { min: mn, max: mx }
            })
            .collect();
        self.uploads = Arc::new(
            objs.iter()
                .map(|o| {
                    let mut verts = Vec::with_capacity(o.mesh.positions.len() * 6);
                    for (p, n) in o.mesh.positions.iter().zip(&o.mesh.normals) {
                        verts.extend_from_slice(p);
                        verts.extend_from_slice(n);
                    }
                    let mut edge_verts = vec![];
                    for e in &o.edges {
                        for w in e.points.windows(2) {
                            gpu::push_segment(&mut edge_verts, w[0], w[1], EDGE_COLOR, EDGE_WIDTH);
                        }
                    }
                    ObjUpload { verts, indices: o.mesh.indices.clone(), edge_verts }
                })
                .collect(),
        );
        self.obj_gen += 1;
        self.objects = objs;
        self.hover = None;
        self.selection.retain(|p| p.object < self.objects.len());
        self.rebuild_highlights();
        if first {
            self.pending_fit = true;
        }
    }

    pub fn set_overlay(&mut self, o: Overlay) {
        let mut v = Vec::new();
        for (a, b, c, w) in &o.lines {
            gpu::push_segment(&mut v, *a, *b, *c, *w);
        }
        for (p, c, s) in &o.points {
            gpu::push_segment(&mut v, *p, *p, *c, *s);
        }
        self.overlay = Arc::new(v);
        self.ov_gen += 1;
    }

    pub fn set_selection(&mut self, sel: Vec<Pick>) {
        if sel != self.selection {
            self.selection = sel;
            self.rebuild_highlights();
        }
    }

    /// Test hook: pretend the pointer hovers this pick (cleared by real pointer hover).
    pub fn force_hover(&mut self, p: Option<Pick>) {
        self.forced_hover = p.clone();
        self.set_hover(p);
    }

    fn set_hover(&mut self, h: Option<Pick>) {
        let same = match (&self.hover, &h) {
            (Some(a), Some(b)) => a.object == b.object && a.kind == b.kind && a.id == b.id,
            (None, None) => true,
            _ => false,
        };
        if !same {
            self.hover = h;
            self.rebuild_highlights();
        } else {
            self.hover = h;
        }
    }

    fn rebuild_highlights(&mut self) {
        let mut hl = HlData::default();
        let mut add = |p: &Pick, hover: bool, objs: &[SceneObject]| {
            let Some(o) = objs.get(p.object) else { return };
            match p.kind {
                PickKind::Face => {
                    let out = if hover { &mut hl.hover_tris } else { &mut hl.sel_tris };
                    for (t, f) in o.mesh.tri_face.iter().enumerate() {
                        if *f == p.id {
                            for k in 0..3 {
                                let i = o.mesh.indices[t * 3 + k] as usize;
                                out.extend_from_slice(&o.mesh.positions[i]);
                                out.extend_from_slice(&o.mesh.normals[i]);
                            }
                        }
                    }
                }
                PickKind::Edge => {
                    let col = if hover { gpu::HOVER_COLOR } else { gpu::SEL_COLOR };
                    for e in o.edges.iter().filter(|e| e.id == p.id) {
                        for w in e.points.windows(2) {
                            gpu::push_segment(&mut hl.lines, w[0], w[1], col, HL_EDGE_WIDTH);
                        }
                    }
                }
                PickKind::Vertex => {
                    let col = if hover { gpu::HOVER_COLOR } else { gpu::SEL_COLOR };
                    let q = [p.point[0] as f32, p.point[1] as f32, p.point[2] as f32];
                    gpu::push_segment(&mut hl.lines, q, q, col, 8.0);
                }
            }
        };
        for p in &self.selection {
            add(p, false, &self.objects);
        }
        if let Some(h) = &self.hover {
            add(h, true, &self.objects);
        }
        self.hl = Arc::new(hl);
        self.hl_gen += 1;
    }

    // ------------------------------------------------------------- camera API

    fn scene_sphere(&self) -> (DVec3, f64) {
        let (mn, mx) = self.scene_bounds().unwrap_or((DVec3::splat(-1.0), DVec3::splat(1.0)));
        ((mn + mx) * 0.5, ((mx - mn).length() * 0.5).max(1e-3))
    }
    fn scene_bounds(&self) -> Option<(DVec3, DVec3)> {
        let mut r: Option<(DVec3, DVec3)> = None;
        for (o, c) in self.objects.iter().zip(&self.cache) {
            if o.visible {
                r = Some(match r {
                    None => (c.min, c.max),
                    Some((a, b)) => (a.min(c.min), b.max(c.max)),
                });
            }
        }
        r
    }

    fn aspect(&self) -> f64 {
        (self.rect.width() / self.rect.height().max(1.0)).max(1e-3) as f64
    }
    fn matrices(&self) -> (DMat4, DMat4) {
        let (c, r) = self.scene_sphere();
        self.cam.matrices(self.aspect(), c, r)
    }
    fn vp(&self) -> DMat4 {
        let (v, p) = self.matrices();
        p * v
    }

    pub fn fit_all(&mut self) {
        let Some((mn, mx)) = self.scene_bounds() else { return };
        let aspect = self.aspect();
        let (right, up) = (self.cam.right(), self.cam.up());
        let center = (mn + mx) * 0.5;
        let (mut x0, mut x1, mut y0, mut y1) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
        for i in 0..8 {
            let c = DVec3::new(
                if i & 1 == 0 { mn.x } else { mx.x },
                if i & 2 == 0 { mn.y } else { mx.y },
                if i & 4 == 0 { mn.z } else { mx.z },
            ) - center;
            let (x, y) = (c.dot(right), c.dot(up));
            x0 = x0.min(x);
            x1 = x1.max(x);
            y0 = y0.min(y);
            y1 = y1.max(y);
        }
        self.cam.target = center + right * (x0 + x1) * 0.5 + up * (y0 + y1) * 0.5;
        let half_h = ((y1 - y0) * 0.5).max((x1 - x0) * 0.5 / aspect).max(1e-6) * 1.12;
        self.cam.dist = half_h / (FOV_Y * 0.5).tan();
        if self.cam.perspective {
            self.cam.dist += (mx - mn).length() * 0.5;
        }
        self.pending_fit = false;
    }

    pub fn set_standard_view(&mut self, v: StandardView) {
        let (f, u) = match v {
            StandardView::Isometric => (DVec3::new(-1.0, 1.0, -1.0), DVec3::Z),
            StandardView::Front => (DVec3::Y, DVec3::Z),
            StandardView::Rear => (DVec3::NEG_Y, DVec3::Z),
            StandardView::Top => (DVec3::NEG_Z, DVec3::Y),
            StandardView::Bottom => (DVec3::Z, DVec3::NEG_Y),
            StandardView::Right => (DVec3::NEG_X, DVec3::Z),
            StandardView::Left => (DVec3::X, DVec3::Z),
        };
        self.cam.orient = camera::orient_from_dir(f, u);
        self.fit_all();
    }

    pub fn align_to_plane(&mut self, p: &Plane) {
        let n = DVec3::from(p.normal).normalize();
        let x = DVec3::from(p.xdir).normalize();
        let up = n.cross(x);
        self.cam.orient = camera::orient_from_dir(-n, up);
        let o = DVec3::from(p.origin);
        // keep lateral position but sit on the plane
        self.cam.target -= n * (self.cam.target - o).dot(n);
    }

    fn ray(&self, pos: Pos2) -> Option<(DVec3, DVec3)> {
        let r = self.rect;
        if r.width() < 1.0 || r.height() < 1.0 {
            return None;
        }
        let nx = ((pos.x - r.left()) / r.width()) as f64 * 2.0 - 1.0;
        let ny = 1.0 - ((pos.y - r.top()) / r.height()) as f64 * 2.0;
        let inv = self.vp().inverse();
        let a = inv.project_point3(DVec3::new(nx, ny, -1.0));
        let b = inv.project_point3(DVec3::new(nx, ny, 1.0));
        Some((a, (b - a).normalize()))
    }

    pub fn screen_to_plane(&self, screen: Pos2, p: &Plane) -> Option<[f64; 3]> {
        let (o, d) = self.ray(screen)?;
        let n = DVec3::from(p.normal);
        let denom = n.dot(d);
        if denom.abs() < 1e-12 {
            return None;
        }
        let t = n.dot(DVec3::from(p.origin) - o) / denom;
        Some((o + d * t).to_array())
    }

    pub fn world_to_screen(&self, w: [f64; 3]) -> Option<Pos2> {
        let c = self.vp() * DVec4::new(w[0], w[1], w[2], 1.0);
        if c.w <= 1e-12 {
            return None;
        }
        let (x, y) = (c.x / c.w, c.y / c.w);
        let r = self.rect;
        Some(Pos2::new(
            r.left() + ((x * 0.5 + 0.5) as f32) * r.width(),
            r.top() + ((0.5 - y * 0.5) as f32) * r.height(),
        ))
    }

    pub fn pixels_per_unit_at(&self, w: [f64; 3]) -> f64 {
        let h = self.rect.height() as f64;
        if self.cam.perspective {
            let depth = (DVec3::from(w) - self.cam.eye()).dot(self.cam.fwd()).max(1e-9);
            h / (2.0 * (FOV_Y * 0.5).tan() * depth)
        } else {
            h / (2.0 * self.cam.half_height())
        }
    }

    // ---------------------------------------------------------------- picking

    fn pick_at(&self, pos: Pos2) -> Option<Pick> {
        let (o, d) = self.ray(pos)?;
        let fwd = self.cam.fwd();
        let eye = self.cam.eye();
        // nearest face triangle
        let mut best: Option<(f64, usize, u32)> = None;
        for (oi, (obj, c)) in self.objects.iter().zip(&self.cache).enumerate() {
            if !obj.visible || !ray_aabb(o, d, c.min, c.max) {
                continue;
            }
            let m = &obj.mesh;
            for (t, tri) in m.indices.chunks_exact(3).enumerate() {
                let v = |i: u32| {
                    let p = m.positions[i as usize];
                    DVec3::new(p[0] as f64, p[1] as f64, p[2] as f64)
                };
                if let Some(tt) = ray_tri(o, d, v(tri[0]), v(tri[1]), v(tri[2])) {
                    if best.map_or(true, |b| tt < b.0) {
                        best = Some((tt, oi, m.tri_face.get(t).copied().unwrap_or(0)));
                    }
                }
            }
        }
        let face_depth = best.map(|(t, _, _)| (o + d * t - eye).dot(fwd));

        // nearest edge in screen space
        let mut best_e: Option<(f64, usize, u32, DVec3)> = None;
        for (oi, obj) in self.objects.iter().enumerate() {
            if !obj.visible {
                continue;
            }
            for e in &obj.edges {
                for w in e.points.windows(2) {
                    let a = DVec3::new(w[0][0] as f64, w[0][1] as f64, w[0][2] as f64);
                    let b = DVec3::new(w[1][0] as f64, w[1][1] as f64, w[1][2] as f64);
                    let (Some(sa), Some(sb)) = (self.world_to_screen(a.to_array()), self.world_to_screen(b.to_array())) else {
                        continue;
                    };
                    let (pa, pb, pp) = (sa.to_vec2(), sb.to_vec2(), pos.to_vec2());
                    let ab = pb - pa;
                    let l2 = ab.length_sq();
                    let t = if l2 > 1e-9 { ((pp - pa).dot(ab) / l2).clamp(0.0, 1.0) } else { 0.0 };
                    let dist = (pa + ab * t - pp).length() as f64;
                    if dist > EDGE_PICK_PX || best_e.map_or(false, |b| dist >= b.0) {
                        continue;
                    }
                    let p3 = a + (b - a) * t as f64;
                    // hidden behind a face? (tolerance: a few pixels)
                    if let Some(fd) = face_depth {
                        let tol = (6.0 / self.pixels_per_unit_at(p3.to_array())).max(1e-6);
                        if (p3 - eye).dot(fwd) > fd + tol {
                            continue;
                        }
                    }
                    best_e = Some((dist, oi, e.id, p3));
                }
            }
        }
        if let Some((_, oi, id, p)) = best_e {
            return Some(Pick { object: oi, kind: PickKind::Edge, id, point: p.to_array() });
        }
        best.map(|(t, oi, id)| Pick { object: oi, kind: PickKind::Face, id, point: (o + d * t).to_array() })
    }

    // ------------------------------------------------------------------- input

    fn zoom_at(&mut self, factor: f64, cursor: Option<Pos2>) {
        if !(factor.is_finite() && factor > 0.0) {
            return;
        }
        let plane_hit = |s: &Self| -> Option<DVec3> {
            let c = cursor?;
            let (o, d) = s.ray(c)?;
            let f = s.cam.fwd();
            let denom = d.dot(f);
            if denom.abs() < 1e-9 {
                return None;
            }
            Some(o + d * ((s.cam.target - o).dot(f) / denom))
        };
        let p0 = plane_hit(self);
        self.cam.dist = (self.cam.dist / factor).clamp(1e-4, 1e7);
        if let (Some(p0), Some(p1)) = (p0, plane_hit(self)) {
            self.cam.target += p0 - p1;
        }
    }

    fn orbit(&mut self, dx: f32, dy: f32) {
        let s = 0.01f64;
        let (up, right) = (self.cam.up(), self.cam.right());
        let q = glam::DQuat::from_axis_angle(up, -dx as f64 * s) * glam::DQuat::from_axis_angle(right, -dy as f64 * s);
        self.cam.orient = (q * self.cam.orient).normalize();
    }

    fn pan(&mut self, dx: f32, dy: f32) {
        let upp = 1.0 / self.pixels_per_unit_at(self.cam.target.to_array());
        self.cam.target += self.cam.up() * dy as f64 * upp - self.cam.right() * dx as f64 * upp;
    }

    fn handle_input(&mut self, ui: &egui::Ui, resp: &egui::Response) {
        let mods = ui.input(|i| i.modifiers);
        let d = resp.drag_delta();
        if d != Vec2::ZERO {
            let middle = resp.dragged_by(egui::PointerButton::Middle);
            let prim = resp.dragged_by(egui::PointerButton::Primary);
            let sec = resp.dragged_by(egui::PointerButton::Secondary);
            if middle || (prim && mods.shift) {
                self.pan(d.x, d.y);
            } else if prim || sec {
                self.orbit(d.x, d.y);
            }
        }
        if resp.hovered() {
            let hover = ui.input(|i| i.pointer.hover_pos());
            let (wheel, zoom) = ui.input(|i| {
                let w: Vec<_> = i
                    .events
                    .iter()
                    .filter_map(|e| match e {
                        egui::Event::MouseWheel { unit, delta, modifiers, .. } => Some((*unit, *delta, *modifiers)),
                        _ => None,
                    })
                    .collect();
                (w, i.zoom_delta())
            });
            for (unit, delta, m) in wheel {
                match unit {
                    egui::MouseWheelUnit::Line | egui::MouseWheelUnit::Page => {
                        self.zoom_at((delta.y as f64 * 0.12).exp(), hover);
                    }
                    egui::MouseWheelUnit::Point => {
                        if m.ctrl {
                            // handled through zoom_delta
                        } else if m.command {
                            self.zoom_at((delta.y as f64 * 0.01).exp(), hover);
                        } else if m.shift || m.alt {
                            self.pan(-delta.x, -delta.y);
                        } else {
                            self.orbit(-delta.x, -delta.y);
                        }
                    }
                }
            }
            if (zoom - 1.0).abs() > 1e-6 {
                self.zoom_at(zoom as f64, hover);
            }
        }
    }

    // ---------------------------------------------------------------------- ui

    pub fn ui(&mut self, ui: &mut egui::Ui) -> ViewportResponse {
        let size = ui.available_size().max(Vec2::splat(8.0));
        let (rect, response) = ui.allocate_exact_size(size, Sense::click_and_drag());
        self.rect = rect;
        self.ppp = ui.ctx().pixels_per_point();
        if !self.rect_known {
            self.rect_known = true;
            self.pending_fit = true;
        }
        if self.pending_fit {
            self.fit_all();
        }
        self.handle_input(ui, &response);

        // hover / click picking
        let dragging = response.dragged();
        let ptr = ui.input(|i| i.pointer.hover_pos());
        let mut hovered = None;
        if response.hovered() && !dragging {
            if let Some(p) = ptr.filter(|p| rect.contains(*p)) {
                hovered = self.pick_at(p);
                self.forced_hover = None;
            }
            self.set_hover(hovered.clone());
        } else if self.forced_hover.is_none() && (dragging || !response.hovered()) {
            self.set_hover(None);
        }
        if self.forced_hover.is_some() {
            hovered = self.hover.clone();
        }
        let clicked = if response.clicked() {
            response.interact_pointer_pos().and_then(|p| self.pick_at(p))
        } else {
            None
        };

        // GL
        let (view, proj) = self.matrices();
        let vp = proj * view;
        let vp32: Vec<f32> = vp.to_cols_array().iter().map(|v| *v as f32).collect();
        let r = glam::DMat3::from_mat4(view).to_cols_array();
        let frame = Frame {
            vp: vp32.try_into().unwrap(),
            view3: r.map(|v| v as f32),
            bg: (BG_TOP, BG_BOT),
            objs: self.uploads.clone(),
            obj_gen: self.obj_gen,
            vis: self.objects.iter().map(|o| (o.visible, o.color)).collect(),
            hl: self.hl.clone(),
            hl_gen: self.hl_gen,
            overlay: self.overlay.clone(),
            ov_gen: self.ov_gen,
        };
        let gpu = self.gpu.clone();
        ui.painter().add(egui::PaintCallback {
            rect,
            callback: Arc::new(eframe::egui_glow::CallbackFn::new(move |info, painter| {
                let gl = painter.gl();
                let mut g = gpu.lock().unwrap();
                let g = g.get_or_insert_with(|| Gpu::new(gl));
                let vp = info.viewport_in_pixels();
                g.paint(gl, &frame, [vp.width_px as f32, vp.height_px as f32], info.pixels_per_point);
            })),
        });

        self.draw_gizmo(ui, rect);
        ViewportResponse { hovered, clicked, response }
    }

    fn draw_gizmo(&self, ui: &egui::Ui, rect: Rect) {
        let p = ui.painter_at(rect);
        let c = Pos2::new(rect.right() - 56.0, rect.bottom() - 56.0);
        let len = 30.0;
        let (r, u, f) = (self.cam.right(), self.cam.up(), self.cam.fwd());
        let mut axes = [
            (DVec3::X, Color32::from_rgb(220, 40, 40), "X"),
            (DVec3::Y, Color32::from_rgb(40, 170, 40), "Y"),
            (DVec3::Z, Color32::from_rgb(50, 90, 230), "Z"),
        ];
        // far axes first
        axes.sort_by(|a, b| a.0.dot(f).partial_cmp(&b.0.dot(f)).unwrap().reverse());
        p.circle_filled(c, 40.0, Color32::from_white_alpha(40));
        for (a, col, name) in axes {
            let tip = c + Vec2::new(a.dot(r) as f32, -a.dot(u) as f32) * len;
            p.line_segment([c, tip], egui::Stroke::new(2.5, col));
            p.circle_filled(tip, 6.5, col);
            p.text(tip, egui::Align2::CENTER_CENTER, name, egui::FontId::proportional(10.0), Color32::WHITE);
        }
    }
}

fn ray_aabb(o: DVec3, d: DVec3, mn: DVec3, mx: DVec3) -> bool {
    let (mut t0, mut t1) = (f64::MIN, f64::MAX);
    for i in 0..3 {
        if d[i].abs() < 1e-14 {
            if o[i] < mn[i] - 1e-9 || o[i] > mx[i] + 1e-9 {
                return false;
            }
        } else {
            let (a, b) = ((mn[i] - o[i]) / d[i], (mx[i] - o[i]) / d[i]);
            t0 = t0.max(a.min(b));
            t1 = t1.min(a.max(b));
        }
    }
    t0 <= t1 + 1e-9
}

/// Moller-Trumbore (two-sided). Returns distance along the ray.
fn ray_tri(o: DVec3, d: DVec3, a: DVec3, b: DVec3, c: DVec3) -> Option<f64> {
    let (e1, e2) = (b - a, c - a);
    let p = d.cross(e2);
    let det = e1.dot(p);
    if det.abs() < 1e-14 {
        return None;
    }
    let inv = 1.0 / det;
    let t = o - a;
    let u = t.dot(p) * inv;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = t.cross(e1);
    let v = d.dot(q) * inv;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    Some(e2.dot(q) * inv)
}
