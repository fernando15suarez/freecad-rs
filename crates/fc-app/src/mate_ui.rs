//! Mate connector helpers shared by Part Design and the Assembly workbench: Onshape-style
//! inference points of a picked face/edge/vertex, and the connector triad drawn over the
//! 3D view (FreeCAD's JCS marker: a translucent disc in the XY plane and red/green/blue
//! axes, at a constant size on screen, plus small arrowheads like Onshape's triad).

use crate::viewport::{PickKind, Viewport};
use eframe::egui::{self, Color32, Pos2, Stroke};
use fc_part::placement::{add, scale};
use fc_part::{Frame, Infer, TopoKind};

/// Axis colors (FreeCAD's AxisXColor/AxisYColor/AxisZColor defaults).
pub const X_COLOR: Color32 = Color32::from_rgb(0xCC, 0x33, 0x33);
pub const Y_COLOR: Color32 = Color32::from_rgb(0x33, 0xCC, 0x33);
pub const Z_COLOR: Color32 = Color32::from_rgb(0x33, 0x55, 0xEE);
/// FreeCAD's preselection (yellow) and selection (green) colors.
pub const PRESELECT: Color32 = Color32::from_rgb(225, 225, 20);
pub const SELECT: Color32 = Color32::from_rgb(28, 173, 28);

/// Axis length on screen (FreeCAD's JCS scale factor).
pub const TRIAD_PX: f32 = 40.0;

pub fn topo_kind(k: PickKind) -> TopoKind {
    match k {
        PickKind::Face => TopoKind::Face,
        PickKind::Edge => TopoKind::Edge,
        PickKind::Vertex => TopoKind::Vertex,
    }
}

/// User-facing name of an inference mode for an element kind.
pub fn infer_label(kind: TopoKind, geom: i32, infer: Infer) -> &'static str {
    let axis = kind == TopoKind::Face && (geom == 1 || geom == 2);
    match infer {
        Infer::Auto => "Auto",
        Infer::Centroid => "Centroid",
        Infer::Center => "Center",
        Infer::Midpoint if axis => "Axis middle",
        Infer::Midpoint => "Midpoint",
        Infer::Start if axis => "Axis start",
        Infer::Start => "Start",
        Infer::End if axis => "Axis end",
        Infer::End => "End",
        Infer::Closest(_) => "Closest point",
        Infer::Normal(_) => "Point",
    }
}

/// Geometry type of an element (FaceInfo/EdgeInfo kind; 0 for vertices).
pub fn element_geom(shape: &fc_occt::Shape, kind: TopoKind, index: usize) -> i32 {
    match kind {
        TopoKind::Face => shape.face_info(index).map(|f| f.kind).unwrap_or(-1),
        TopoKind::Edge => shape.edge_info(index).map(|e| e.kind).unwrap_or(-1),
        TopoKind::Vertex => 0,
    }
}

/// The inference points Onshape offers on an element, with their frames (shape coordinates).
pub fn candidates(shape: &fc_occt::Shape, kind: TopoKind, index: usize) -> Vec<(Infer, Frame)> {
    let modes: Vec<Infer> = match kind {
        TopoKind::Face => match shape.face_info(index).map(|f| f.kind).unwrap_or(-1) {
            0 => vec![Infer::Centroid, Infer::Center],
            1 | 2 => vec![Infer::Midpoint, Infer::Start, Infer::End],
            3 => vec![Infer::Center],
            _ => vec![Infer::Centroid],
        },
        TopoKind::Edge => match shape.edge_info(index) {
            Ok(e) if (e.kind == 1 || e.kind == 2) && e.closed => vec![Infer::Center],
            Ok(e) if e.kind == 1 || e.kind == 2 => vec![Infer::Center, Infer::Midpoint, Infer::Start, Infer::End],
            _ => vec![Infer::Midpoint, Infer::Start, Infer::End],
        },
        TopoKind::Vertex => vec![Infer::Auto],
    };
    let mut out: Vec<(Infer, Frame)> = vec![];
    for m in modes {
        if let Ok(f) = fc_part::mate::infer_frame(shape, kind, index, m) {
            if !out.iter().any(|(_, g)| fc_part::placement::norm(fc_part::placement::sub(g.origin, f.origin)) < 1e-6) {
                out.push((m, f));
            }
        }
    }
    out
}

/// The candidate whose point is nearest to the cursor on screen. `to_world` maps shape
/// coordinates to the view (identity in Part Design, the instance placement in assemblies).
pub fn snap(vp: &Viewport, cands: &[(Infer, Frame)], cursor: Pos2, to_world: &dyn Fn(&Frame) -> Frame) -> Option<usize> {
    cands
        .iter()
        .enumerate()
        .filter_map(|(i, (_, f))| vp.world_to_screen(to_world(f).origin).map(|p| (i, (p - cursor).length())))
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(i, _)| i)
}

#[derive(Clone, Copy, PartialEq)]
pub enum Look {
    Normal,
    Hover,
    Selected,
    /// Preview of what a click would create.
    Ghost,
}

/// Screen-space size: world units per pixel at `p`.
fn upp(vp: &Viewport, p: [f64; 3]) -> f64 {
    1.0 / vp.pixels_per_unit_at(p).max(1e-12)
}

/// Draw a connector triad for the world frame `f`.
pub fn draw(p: &egui::Painter, vp: &Viewport, f: &Frame, look: Look) {
    let Some(o) = vp.world_to_screen(f.origin) else { return };
    let len = TRIAD_PX as f64 * upp(vp, f.origin);
    let proj = |v: [f64; 3], t: f64| vp.world_to_screen(add(f.origin, scale(v, t * len)));
    let alpha = if look == Look::Ghost { 170 } else { 255 };
    let fade = |c: Color32| Color32::from_rgba_unmultiplied(c.r(), c.g(), c.b(), alpha);
    // disc in the XY plane
    let disc: Vec<Pos2> = (0..20)
        .filter_map(|i| {
            let a = i as f64 / 20.0 * std::f64::consts::TAU;
            vp.world_to_screen(add(f.origin, add(scale(f.x, a.cos() * 0.4 * len), scale(f.y, a.sin() * 0.4 * len))))
        })
        .collect();
    let (disc_fill, disc_line) = match look {
        Look::Hover => (Color32::from_rgba_unmultiplied(225, 225, 20, 90), PRESELECT),
        Look::Selected => (Color32::from_rgba_unmultiplied(28, 173, 28, 100), SELECT),
        Look::Ghost => (Color32::from_rgba_unmultiplied(128, 128, 128, 70), Color32::from_rgba_unmultiplied(200, 200, 200, 120)),
        Look::Normal => (Color32::from_rgba_unmultiplied(128, 128, 128, 110), Color32::from_rgba_unmultiplied(170, 170, 170, 160)),
    };
    if disc.len() == 20 {
        p.add(egui::Shape::convex_polygon(disc.clone(), disc_fill, Stroke::NONE));
        let mut ring = disc;
        ring.push(ring[0]);
        p.add(egui::Shape::line(ring, Stroke::new(1.0, disc_line)));
    }
    let glow = match look {
        Look::Hover => Some(PRESELECT),
        Look::Selected => Some(SELECT),
        _ => None,
    };
    for (axis, col) in [(f.x, X_COLOR), (f.y, Y_COLOR), (f.z, Z_COLOR)] {
        let Some(tip) = proj(axis, 1.0) else { continue };
        let d = tip - o;
        if d.length() < 1.0 {
            continue;
        }
        if let Some(g) = glow {
            p.line_segment([o, tip], Stroke::new(6.0, g));
        }
        p.line_segment([o, tip - d.normalized() * 6.0], Stroke::new(3.0, fade(col)));
        let n = d.normalized();
        let side = egui::vec2(-n.y, n.x) * 4.0;
        p.add(egui::Shape::convex_polygon(vec![tip, tip - n * 9.0 + side, tip - n * 9.0 - side], fade(col), Stroke::NONE));
    }
    p.circle(o, 3.5, if look == Look::Ghost { Color32::from_gray(200) } else { Color32::WHITE }, Stroke::new(1.0, Color32::from_gray(40)));
}

/// Small dots at the other inference points of the hovered element (Onshape shows them).
pub fn draw_points(p: &egui::Painter, vp: &Viewport, pts: &[[f64; 3]], chosen: Option<usize>) {
    for (i, w) in pts.iter().enumerate() {
        if let Some(s) = vp.world_to_screen(*w) {
            let c = if Some(i) == chosen { PRESELECT } else { Color32::from_gray(235) };
            p.circle(s, 3.0, c, Stroke::new(1.0, Color32::from_gray(30)));
        }
    }
}

/// Distance in pixels from `pos` to the triad (origin or an axis), if close enough to pick.
pub fn hit(vp: &Viewport, f: &Frame, pos: Pos2) -> Option<f32> {
    let o = vp.world_to_screen(f.origin)?;
    let len = TRIAD_PX as f64 * upp(vp, f.origin);
    let mut best = (o - pos).length();
    for axis in [f.x, f.y, f.z] {
        if let Some(t) = vp.world_to_screen(add(f.origin, scale(axis, len))) {
            let ab = t - o;
            let l2 = ab.length_sq();
            let s = if l2 > 1e-6 { ((pos - o).dot(ab) / l2).clamp(0.0, 1.0) } else { 0.0 };
            best = best.min((o + ab * s - pos).length());
        }
    }
    (best <= 8.0).then_some(best)
}

/// fc-assembly's quaternion frame as fc-part's axes frame.
pub fn from_asm(f: &fc_assembly::Frame) -> Frame {
    let m = f.matrix();
    Frame { origin: f.pos, x: m[0], y: m[1], z: m[2] }
}

pub fn to_asm(f: &Frame) -> fc_assembly::Frame {
    fc_assembly::Frame::from_axes(f.origin, [f.x, f.y, f.z])
}
