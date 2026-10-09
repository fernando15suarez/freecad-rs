//! Small 2D helpers for the sketch editor.

use fc_sketch::{ConstraintKind as K, Curve, GeoId, PointPos, PointRef, Sketch, Vec2, H_AXIS, ROOT, V_AXIS};
use std::f64::consts::PI;

/// Selectable points of a curve (FreeCAD vertices).
pub fn point_positions(c: &Curve) -> &'static [PointPos] {
    match c {
        Curve::Point { .. } => &[PointPos::Start],
        Curve::Line { .. } => &[PointPos::Start, PointPos::End],
        Curve::Circle { .. } => &[PointPos::Mid],
        Curve::Arc { .. } => &[PointPos::Start, PointPos::End, PointPos::Mid],
    }
}

/// Every vertex of the sketch plus the origin.
pub fn all_points(sk: &Sketch) -> Vec<PointRef> {
    let mut v = vec![PointRef::root()];
    for (gi, g) in sk.geometry.iter().enumerate() {
        for &pos in point_positions(&g.curve) {
            v.push(PointRef::new(gi, pos));
        }
    }
    v
}

/// Normalise a point reference for comparisons (a Point geometry's vertex is always `Start`).
pub fn canon(sk: &Sketch, p: PointRef) -> PointRef {
    if p.geo == ROOT {
        return PointRef::root();
    }
    match sk.geometry.get(p.geo).map(|g| &g.curve) {
        Some(Curve::Point { .. }) if p.pos != PointPos::Edge => PointRef::new(p.geo, PointPos::Start),
        _ => p,
    }
}

/// Point references (vertex-level, not `Edge`) a constraint uses.
pub fn point_refs(k: &K) -> Vec<PointRef> {
    let v = match k {
        K::Coincident(a, b) | K::HorizontalPoints(a, b) | K::VerticalPoints(a, b) | K::TangentAt(a, b) | K::Distance(a, b, _) | K::DistanceX(a, b, _) | K::DistanceY(a, b, _) => vec![*a, *b],
        K::PointOnObject(p, _) | K::PointLineDistance(p, _, _) | K::LockX(p, _) | K::LockY(p, _) => vec![*p],
        K::Symmetric(a, b, c) => vec![*a, *b, *c],
        K::SymmetricLine(a, b, _) => vec![*a, *b],
        _ => vec![],
    };
    v.into_iter().filter(|p| p.pos != PointPos::Edge).collect()
}

pub fn is_line(sk: &Sketch, g: GeoId) -> bool {
    g == H_AXIS || g == V_AXIS || matches!(sk.geometry.get(g).map(|g| &g.curve), Some(Curve::Line { .. }))
}
pub fn is_round(sk: &Sketch, g: GeoId) -> bool {
    matches!(sk.geometry.get(g).map(|g| &g.curve), Some(Curve::Circle { .. } | Curve::Arc { .. }))
}

/// Line endpoints (axes are unit segments from the origin).
pub fn line_of(sk: &Sketch, g: GeoId) -> Option<(Vec2, Vec2)> {
    match g {
        H_AXIS => Some((Vec2::ZERO, Vec2::new(1.0, 0.0))),
        V_AXIS => Some((Vec2::ZERO, Vec2::new(0.0, 1.0))),
        _ => match sk.geometry.get(g).map(|g| &g.curve) {
            Some(Curve::Line { a, b }) => Some((*a, *b)),
            _ => None,
        },
    }
}

pub fn circle_of(sk: &Sketch, g: GeoId) -> Option<(Vec2, f64)> {
    match sk.geometry.get(g).map(|g| &g.curve) {
        Some(Curve::Circle { c, r }) | Some(Curve::Arc { c, r, .. }) => Some((*c, *r)),
        _ => None,
    }
}

pub fn norm_angle(a: f64) -> f64 {
    a.rem_euclid(2.0 * PI)
}

/// Nearest point on a curve (arcs clamp to their endpoints).
pub fn nearest_on_curve(c: &Curve, p: Vec2) -> Vec2 {
    match c {
        Curve::Point { p: q } => *q,
        Curve::Line { a, b } => {
            let d = *b - *a;
            let l2 = d.dot(d);
            let t = if l2 > 1e-18 { ((p - *a).dot(d) / l2).clamp(0.0, 1.0) } else { 0.0 };
            *a + d * t
        }
        Curve::Circle { c, r } => {
            let d = p - *c;
            let l = d.length();
            if l < 1e-12 { *c + Vec2::new(*r, 0.0) } else { *c + d * (*r / l) }
        }
        Curve::Arc { c: cc, r, start, end } => {
            let ang = (p - *cc).y.atan2((p - *cc).x);
            let sweep = norm_angle(end - start);
            let rel = norm_angle(ang - start);
            if rel <= sweep {
                *cc + Vec2::polar(ang) * *r
            } else {
                let s = *cc + Vec2::polar(*start) * *r;
                let e = *cc + Vec2::polar(*end) * *r;
                if s.dist(p) < e.dist(p) { s } else { e }
            }
        }
    }
}

/// Circle through three points.
pub fn circle_through(a: Vec2, b: Vec2, c: Vec2) -> Option<(Vec2, f64)> {
    let d = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
    if d.abs() < 1e-12 {
        return None;
    }
    let (a2, b2, c2) = (a.dot(a), b.dot(b), c.dot(c));
    let ux = (a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) / d;
    let uy = (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) / d;
    let cen = Vec2::new(ux, uy);
    Some((cen, cen.dist(a)))
}

/// Intersection of two infinite lines (point + direction).
pub fn line_intersection(p1: Vec2, d1: Vec2, p2: Vec2, d2: Vec2) -> Option<Vec2> {
    let den = d1.cross(d2);
    if den.abs() < 1e-12 {
        return None;
    }
    let t = (p2 - p1).cross(d2) / den;
    Some(p1 + d1 * t)
}

/// Number formatting like FreeCAD's quantity spin boxes (up to 2 decimals, trailing zeros trimmed).
pub fn fmt_num(v: f64) -> String {
    let s = format!("{v:.2}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s == "-0" { "0".into() } else { s.to_string() }
}

/// FreeCAD's constraint type names.
pub fn kind_name(k: &K) -> &'static str {
    match k {
        K::Coincident(..) => "Coincident",
        K::Horizontal(..) | K::HorizontalPoints(..) => "Horizontal",
        K::Vertical(..) | K::VerticalPoints(..) => "Vertical",
        K::Parallel(..) => "Parallel",
        K::Perpendicular(..) => "Perpendicular",
        K::Tangent(..) | K::TangentAt(..) => "Tangent",
        K::Equal(..) => "Equal",
        K::PointOnObject(..) => "Point on object",
        K::Symmetric(..) | K::SymmetricLine(..) => "Symmetric",
        K::Distance(..) | K::DistanceLine(..) | K::PointLineDistance(..) => "Distance",
        K::DistanceX(..) => "Horizontal distance",
        K::DistanceY(..) => "Vertical distance",
        K::LockX(..) => "Lock X",
        K::LockY(..) => "Lock Y",
        K::Radius(..) => "Radius",
        K::Diameter(..) => "Diameter",
        K::Angle(..) | K::AngleBetween(..) => "Angle",
        K::Block(..) => "Block",
    }
}

pub fn is_angle(k: &K) -> bool {
    matches!(k, K::Angle(..) | K::AngleBetween(..))
}

/// Display value of a dimension in user units (degrees for angles, |v| for signed distances).
pub fn display_value(k: &K) -> Option<f64> {
    let v = k.value()?;
    Some(match k {
        K::Angle(..) | K::AngleBetween(..) => v.to_degrees(),
        K::DistanceX(..) | K::DistanceY(..) | K::LockX(..) | K::LockY(..) => v.abs(),
        _ => v,
    })
}

pub fn unit_of(k: &K) -> &'static str {
    if is_angle(k) { "°" } else { "mm" }
}

pub fn geo_name(g: GeoId) -> String {
    match g {
        H_AXIS => "H_Axis".into(),
        V_AXIS => "V_Axis".into(),
        ROOT => "RootPoint".into(),
        g => format!("Edge{}", g + 1),
    }
}

pub fn curve_name(c: &Curve) -> &'static str {
    match c {
        Curve::Point { .. } => "Point",
        Curve::Line { .. } => "Line",
        Curve::Circle { .. } => "Circle",
        Curve::Arc { .. } => "Arc",
    }
}

pub fn pos_name(p: PointPos) -> &'static str {
    match p {
        PointPos::Start => "start",
        PointPos::End => "end",
        PointPos::Mid => "center",
        PointPos::Edge => "edge",
    }
}

/// Bounding box of the sketch geometry (origin included).
pub fn bbox(sk: &Sketch) -> (Vec2, Vec2) {
    let mut mn = Vec2::ZERO;
    let mut mx = Vec2::ZERO;
    for g in &sk.geometry {
        let pts: Vec<Vec2> = match &g.curve {
            Curve::Circle { c, r } | Curve::Arc { c, r, .. } => vec![*c - Vec2::new(*r, *r), *c + Vec2::new(*r, *r)],
            c => fc_sketch::tessellate_curve(c, 1),
        };
        for p in pts {
            mn = Vec2::new(mn.x.min(p.x), mn.y.min(p.y));
            mx = Vec2::new(mx.x.max(p.x), mx.y.max(p.y));
        }
    }
    (mn, mx)
}
