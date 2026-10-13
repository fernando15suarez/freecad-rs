//! FreeCAD's navigation cube: a chamfered cube in the top-right corner of the 3D view
//! whose 6 faces, 12 edges and 8 corners snap the view, plus rotate arrows around it.

use crate::widgets;
use eframe::egui::{self, epaint, Align2, Color32, Pos2, Rect, Sense, Shape, Stroke, Vec2};
use glam::DVec3;

const CHAMFER: f64 = 0.24;

pub struct Facet {
    pub normal: DVec3,
    pub poly: Vec<DVec3>,
    /// Main faces carry a label and the text's up direction.
    pub label: Option<(&'static str, DVec3)>,
}

pub fn facets() -> Vec<Facet> {
    let k = 1.0 - CHAMFER;
    let axes = [DVec3::X, DVec3::Y, DVec3::Z];
    let mut out = Vec::new();
    let labels = |n: DVec3| -> (&'static str, DVec3) {
        if n.z > 0.5 {
            ("TOP", DVec3::Y)
        } else if n.z < -0.5 {
            ("BOTTOM", DVec3::NEG_Y)
        } else if n.y < -0.5 {
            ("FRONT", DVec3::Z)
        } else if n.y > 0.5 {
            ("REAR", DVec3::Z)
        } else if n.x > 0.5 {
            ("RIGHT", DVec3::Z)
        } else {
            ("LEFT", DVec3::Z)
        }
    };
    // main faces
    for i in 0..3 {
        for s in [1.0, -1.0] {
            let n = axes[i] * s;
            let (u, v) = (axes[(i + 1) % 3], axes[(i + 2) % 3]);
            let poly = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)].iter().map(|(a, b)| n + u * (a * k) + v * (b * k)).collect();
            out.push(Facet { normal: n, poly, label: Some(labels(n)) });
        }
    }
    // edges
    for i in 0..3 {
        for j in (i + 1)..3 {
            let t = axes[3 - i - j];
            for sa in [1.0, -1.0] {
                for sb in [1.0, -1.0] {
                    let (a, b) = (axes[i] * sa, axes[j] * sb);
                    let poly = vec![a + b * k + t * k, a * k + b + t * k, a * k + b - t * k, a + b * k - t * k];
                    out.push(Facet { normal: (a + b).normalize(), poly, label: None });
                }
            }
        }
    }
    // corners
    for sx in [1.0, -1.0] {
        for sy in [1.0, -1.0] {
            for sz in [1.0, -1.0] {
                let (x, y, z) = (DVec3::X * sx, DVec3::Y * sy, DVec3::Z * sz);
                let poly = vec![x + (y + z) * k, y + (x + z) * k, z + (x + y) * k];
                out.push(Facet { normal: (x + y + z).normalize(), poly, label: None });
            }
        }
    }
    out
}

/// What a click on the cube asks for.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum NavAction {
    /// Look along `fwd` with `up` up.
    View { fwd: DVec3, up: DVec3 },
    /// Orbit by this many degrees around the screen's up (x) / right (y) axes.
    Orbit { yaw: f64, pitch: f64 },
    /// Roll around the view axis (degrees, positive = counter-clockwise).
    Roll(f64),
    /// Drag on the cube: orbit by screen delta.
    Drag(Vec2),
}

const STEP_DEG: f64 = 15.0;

/// Pick the up vector for looking at a facet with normal `n`.
fn up_for(n: DVec3, cur_up: DVec3) -> DVec3 {
    if n.z.abs() > 0.99 {
        // top / bottom: keep the screen's current "up" if it lies in the XY plane, like FreeCAD
        let u = DVec3::new(cur_up.x, cur_up.y, 0.0);
        let u = if u.length() > 1e-3 { u.normalize() } else { DVec3::Y };
        // snap to the nearest axis
        let cands = [DVec3::X, DVec3::NEG_X, DVec3::Y, DVec3::NEG_Y];
        cands.into_iter().max_by(|a, b| a.dot(u).partial_cmp(&b.dot(u)).unwrap()).unwrap()
    } else {
        DVec3::Z
    }
}

/// Side of the cube's square area and its margin from the view's top-right corner.
pub const SIZE: f32 = 140.0;
pub const MARGIN: f32 = 6.0;

/// The screen area the cube (with its rotate arrows) covers in `view`: overlays such as the
/// FEM color bar stay out of it.
pub fn area(view: Rect) -> Rect {
    Rect::from_min_size(Pos2::new(view.right() - SIZE - MARGIN, view.top() + MARGIN), Vec2::splat(SIZE)).expand2(Vec2::new(4.0, 10.0))
}

/// Draw the cube for a camera with basis (right, up, fwd) into the top-right of `view`.
/// Returns the requested navigation, if any.
pub fn ui(ui: &mut egui::Ui, view: Rect, right: DVec3, up: DVec3, fwd: DVec3) -> Option<NavAction> {
    let area = Rect::from_min_size(Pos2::new(view.right() - SIZE - MARGIN, view.top() + MARGIN), Vec2::splat(SIZE));
    let center = area.center();
    let s = 34.0f32; // pixels per cube unit
    let proj = |p: DVec3| center + Vec2::new(p.dot(right) as f32, -p.dot(up) as f32) * s;
    let resp = ui.interact(area, ui.id().with("navcube"), Sense::click_and_drag());
    widgets::reg("navcube", area);
    let ptr = ui.input(|i| i.pointer.hover_pos()).filter(|p| area.contains(*p));
    let painter = ui.painter_at(area);

    let facets = facets();
    let visible: Vec<&Facet> = facets.iter().filter(|f| f.normal.dot(fwd) < -1e-3).collect();
    let polys: Vec<Vec<Pos2>> = visible.iter().map(|f| f.poly.iter().map(|p| proj(*p)).collect()).collect();
    let hover_face = ptr.and_then(|p| polys.iter().position(|poly| point_in_poly(p, poly)));

    // shadow / backdrop
    painter.circle_filled(center, 1.55 * s, Color32::from_black_alpha(40));
    for (i, (f, poly)) in visible.iter().zip(&polys).enumerate() {
        let facing = (-f.normal.dot(fwd)) as f32;
        let main = f.label.is_some();
        let base = if main { [218.0, 222.0, 230.0] } else { [176.0, 182.0, 192.0] };
        let shade = 0.72 + 0.28 * facing;
        let mut col = Color32::from_rgb((base[0] * shade) as u8, (base[1] * shade) as u8, (base[2] * shade) as u8);
        if hover_face == Some(i) {
            col = Color32::from_rgb(64, 150, 235);
        }
        painter.add(Shape::convex_polygon(poly.clone(), col, Stroke::new(1.2, Color32::from_rgb(58, 62, 68))));
        if let Some((label, text_up)) = f.label {
            if facing > 0.12 {
                let tc = if hover_face == Some(i) { Color32::WHITE } else { Color32::from_rgb(35, 38, 45) };
                face_text(ui, &painter, label, f.normal, text_up, &proj, tc.gamma_multiply(((facing - 0.12) * 3.0).min(1.0)));
            }
        }
    }

    // rotate arrows (FreeCAD: triangles on 4 sides, curved roll arrows at the top corners)
    let arrow_col = |hit: bool| if hit { Color32::from_rgb(64, 150, 235) } else { Color32::from_rgba_unmultiplied(200, 205, 215, 170) };
    let r = 1.72 * s;
    let mut action = None;
    let tri = |dir: Vec2| {
        let tip = center + dir * (r + 9.0);
        let n = Vec2::new(-dir.y, dir.x);
        vec![tip, center + dir * r + n * 7.0, center + dir * r - n * 7.0]
    };
    let arrows = [(Vec2::new(0.0, -1.0), NavAction::Orbit { yaw: 0.0, pitch: STEP_DEG }), (Vec2::new(0.0, 1.0), NavAction::Orbit { yaw: 0.0, pitch: -STEP_DEG }), (Vec2::new(-1.0, 0.0), NavAction::Orbit { yaw: STEP_DEG, pitch: 0.0 }), (Vec2::new(1.0, 0.0), NavAction::Orbit { yaw: -STEP_DEG, pitch: 0.0 })];
    let mut hovered_arrow = false;
    for (dir, act) in arrows {
        let pts = tri(dir);
        let hit = ptr.is_some_and(|p| (p - (center + dir * (r + 3.0))).length() < 10.0);
        hovered_arrow |= hit;
        painter.add(Shape::convex_polygon(pts, arrow_col(hit), Stroke::NONE));
        if hit && resp.clicked() {
            action = Some(act);
        }
    }
    for (side, roll) in [(-1.0f32, STEP_DEG), (1.0, -STEP_DEG)] {
        let c0 = center + Vec2::new(side * r * 0.78, -r * 0.78);
        let hit = ptr.is_some_and(|p| (p - c0).length() < 11.0);
        hovered_arrow |= hit;
        let col = arrow_col(hit);
        // arc around the cube with an arrow head
        let a0 = if side < 0.0 { 200.0f32 } else { 340.0 };
        let a1 = if side < 0.0 { 250.0f32 } else { 290.0 };
        let pts: Vec<Pos2> = (0..=8).map(|k| {
            let a = (a0 + (a1 - a0) * k as f32 / 8.0).to_radians();
            center + Vec2::new(a.cos(), a.sin()) * (r + 2.0)
        }).collect();
        painter.add(Shape::line(pts.clone(), Stroke::new(2.5, col)));
        let end = pts[8];
        let dir = (pts[8] - pts[7]).normalized();
        let n = Vec2::new(-dir.y, dir.x);
        painter.add(Shape::convex_polygon(vec![end + dir * 7.0, end + n * 5.0, end - n * 5.0], col, Stroke::NONE));
        if hit && resp.clicked() {
            action = Some(NavAction::Roll(roll));
        }
    }

    if resp.dragged() {
        let d = resp.drag_delta();
        if d != Vec2::ZERO {
            return Some(NavAction::Drag(d));
        }
    }
    if action.is_none() && resp.clicked() {
        if let Some(i) = hover_face {
            let n = visible[i].normal;
            action = Some(NavAction::View { fwd: -n, up: up_for(n, up) });
        }
    }
    if hover_face.is_some() || hovered_arrow {
        ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
    }
    action
}

/// Lay out `text`, tessellate it, and map the mesh onto the cube face so it skews with it.
fn face_text(ui: &egui::Ui, painter: &egui::Painter, text: &str, n: DVec3, text_up: DVec3, proj: &dyn Fn(DVec3) -> Pos2, color: Color32) {
    let font = crate::theme::bold_font(18.0);
    let galley = painter.layout_no_wrap(text.to_string(), font, color);
    let gs = galley.size();
    let ppp = ui.ctx().pixels_per_point();
    let tex = ui.ctx().fonts(|f| f.font_image_size());
    let mut tess = epaint::Tessellator::new(ppp, Default::default(), tex, vec![]);
    let mut mesh = epaint::Mesh::default();
    tess.tessellate_text(&epaint::TextShape::new(Pos2::ZERO, galley, color), &mut mesh);
    let text_right = text_up.cross(n);
    let k = (1.32 / gs.x as f64).min(0.42 / gs.y as f64);
    for v in &mut mesh.vertices {
        let u = (v.pos.x - gs.x * 0.5) as f64 * k;
        let w = (gs.y * 0.5 - v.pos.y) as f64 * k;
        v.pos = proj(n * 1.001 + text_right * u + text_up * w);
    }
    painter.add(Shape::mesh(mesh));
}

fn point_in_poly(p: Pos2, poly: &[Pos2]) -> bool {
    let mut inside = false;
    let n = poly.len();
    for i in 0..n {
        let (a, b) = (poly[i], poly[(i + 1) % n]);
        if (a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x {
            inside = !inside;
        }
    }
    inside
}

/// Small XYZ axis cross for the bottom-left corner of the view.
pub fn axis_cross(painter: &egui::Painter, view: Rect, right: DVec3, up: DVec3, fwd: DVec3) {
    let c = Pos2::new(view.left() + 42.0, view.bottom() - 42.0);
    let len = 26.0;
    let mut axes = [(DVec3::X, Color32::from_rgb(230, 60, 60), "X"), (DVec3::Y, Color32::from_rgb(70, 200, 70), "Y"), (DVec3::Z, Color32::from_rgb(70, 120, 240), "Z")];
    axes.sort_by(|a, b| a.0.dot(fwd).partial_cmp(&b.0.dot(fwd)).unwrap().reverse());
    for (a, col, name) in axes {
        let d = Vec2::new(a.dot(right) as f32, -a.dot(up) as f32);
        let tip = c + d * len;
        painter.line_segment([c, tip], Stroke::new(2.0, col));
        if d.length() > 0.15 {
            painter.text(c + d * (len + 9.0), Align2::CENTER_CENTER, name, crate::theme::bold_font(11.0), col);
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn facets_are_outward() {
        let f = super::facets();
        assert_eq!(f.len(), 26);
        for x in &f {
            let c = x.poly.iter().fold(glam::DVec3::ZERO, |a, b| a + *b) / x.poly.len() as f64;
            assert!(c.dot(x.normal) > 0.5);
            assert!((x.normal.length() - 1.0).abs() < 1e-9);
        }
    }
}
