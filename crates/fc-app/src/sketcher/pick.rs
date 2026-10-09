//! Screen-space hit testing of sketch elements.

use super::geom;
use super::Proj;
use eframe::egui::{Pos2, Vec2 as V};
use fc_sketch::{Curve, GeoId, PointRef, Sketch, Vec2, H_AXIS, V_AXIS};

/// Pick tolerances in screen points.
pub const POINT_PX: f32 = 9.0;
pub const EDGE_PX: f32 = 6.0;

pub fn seg_dist(p: Pos2, a: Pos2, b: Pos2) -> (f32, f32) {
    let ab = b - a;
    let l2 = ab.length_sq();
    let t = if l2 > 1e-9 { ((p - a).dot(ab) / l2).clamp(0.0, 1.0) } else { 0.0 };
    ((a + ab * t - p).length(), t)
}

/// Screen polyline of a curve.
pub fn screen_polyline(pj: &Proj, c: &Curve) -> Vec<Pos2> {
    let base = |r: f64| ((r.abs() * pj.ppu()).sqrt() * 6.0).clamp(48.0, 256.0);
    let n = match c {
        Curve::Point { .. } => 0,
        Curve::Line { .. } => 1,
        Curve::Circle { r, .. } => base(*r) as usize,
        Curve::Arc { r, .. } => (base(*r) * c.sweep() / std::f64::consts::TAU).ceil().max(8.0) as usize,
    };
    fc_sketch::tessellate_curve(c, n.max(1)).into_iter().map(|p| pj.s(p)).collect()
}

/// Nearest vertex (incl. the origin) within `tol` pixels.
pub fn nearest_point(sk: &Sketch, pj: &Proj, pos: Pos2, tol: f32) -> Option<PointRef> {
    let mut best: Option<(f32, PointRef)> = None;
    for pr in geom::all_points(sk) {
        let Some(p) = sk.try_point(pr) else { continue };
        let d = (pj.s(p) - pos).length();
        // Prefer later geometry slightly less than nearer points; ties go to the first.
        if d <= tol && best.map_or(true, |b| d < b.0 - 0.01) {
            best = Some((d, pr));
        }
    }
    best.map(|b| b.1)
}

/// Nearest curve (optionally incl. the axes) within `tol` pixels; returns the projected
/// sketch point on it.
pub fn nearest_edge(sk: &Sketch, pj: &Proj, pos: Pos2, tol: f32, axes: bool) -> Option<(GeoId, Vec2)> {
    let mut best: Option<(f32, GeoId)> = None;
    for (gi, g) in sk.geometry.iter().enumerate() {
        if matches!(g.curve, Curve::Point { .. }) {
            continue;
        }
        let pl = screen_polyline(pj, &g.curve);
        for w in pl.windows(2) {
            let (d, _) = seg_dist(pos, w[0], w[1]);
            if d <= tol && best.map_or(true, |b| d < b.0) {
                best = Some((d, gi));
            }
        }
    }
    if axes {
        let o = pj.s(Vec2::ZERO);
        for (g, dir) in [(H_AXIS, Vec2::new(1.0, 0.0)), (V_AXIS, Vec2::new(0.0, 1.0))] {
            let e = pj.s(dir);
            let u: V = (e - o).normalized();
            let d = ((pos - o) - u * (pos - o).dot(u)).length();
            if d <= tol && best.map_or(true, |b| d < b.0) {
                best = Some((d, g));
            }
        }
    }
    let (_, g) = best?;
    let raw = pj.w(pos)?;
    let p = match g {
        H_AXIS => Vec2::new(raw.x, 0.0),
        V_AXIS => Vec2::new(0.0, raw.y),
        _ => geom::nearest_on_curve(&sk.geometry[g].curve, raw),
    };
    Some((g, p))
}
