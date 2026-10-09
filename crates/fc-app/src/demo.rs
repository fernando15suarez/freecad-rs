//! Demo document used by `FCRS_DEMO=1` (screenshots and first launch checks).

use fc_part::*;
use fc_sketch::{Constraint, ConstraintKind as K, Curve, Geometry, PointPos, PointRef, Sketch, Vec2};

fn v(x: f64, y: f64) -> Vec2 {
    Vec2 { x, y }
}
fn pr(geo: usize, pos: PointPos) -> PointRef {
    PointRef { geo, pos }
}

/// Fully constrained rectangle; returns (sketch, index of width constraint).
pub fn rect_sketch(x: f64, y: f64, w: f64, h: f64) -> (Sketch, usize) {
    let mut s = Sketch::default();
    let c = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)];
    let mut l = Vec::new();
    for i in 0..4 {
        let (a, b) = (c[i], c[(i + 1) % 4]);
        l.push(s.add_geometry(Geometry { curve: Curve::Line { a: v(a.0, a.1), b: v(b.0, b.1) }, construction: false }));
    }
    for i in 0..4 {
        s.add_constraint(Constraint::new(K::Coincident(pr(l[i], PointPos::End), pr(l[(i + 1) % 4], PointPos::Start))));
    }
    s.add_constraint(Constraint::new(K::Horizontal(l[0])));
    s.add_constraint(Constraint::new(K::Horizontal(l[2])));
    s.add_constraint(Constraint::new(K::Vertical(l[1])));
    s.add_constraint(Constraint::new(K::Vertical(l[3])));
    let wc = s.add_constraint(Constraint::new(K::DistanceX(pr(l[0], PointPos::Start), pr(l[0], PointPos::End), w)));
    s.add_constraint(Constraint::new(K::DistanceY(pr(l[1], PointPos::Start), pr(l[1], PointPos::End), h)));
    s.add_constraint(Constraint::new(K::LockX(pr(l[0], PointPos::Start), x)));
    s.add_constraint(Constraint::new(K::LockY(pr(l[0], PointPos::Start), y)));
    (s, wc)
}

/// A plate with a pocket, a counterbored hole and a revolved boss.
pub fn demo_document() -> Document {
    let mut doc = Document::new();
    let (sk, wc) = rect_sketch(-30.0, -20.0, 60.0, 40.0);
    let s = doc.add_sketch(Support::XY, sk);
    let pad = doc.add(FeatureKind::Pad(Extrude::new(s, 10.0)));
    let _ = doc.name_constraint(s, wc, "width");
    let _ = doc.set_parameter("thickness", "10 mm");
    let _ = doc.set_expression("Pad.Length", "thickness");
    doc.recompute();

    let body = doc.state(pad).and_then(|s| s.shape.clone()).expect("pad");
    let top = (1..=body.face_count()).find(|&i| body.face_info(i).map(|f| f.normal[2] > 0.99).unwrap_or(false)).unwrap();
    let fr = face_ref(pad, &body, top).unwrap();
    let (pk, _) = rect_sketch(-22.0, -12.0, 20.0, 24.0);
    let s2 = doc.add_sketch(Support::Face(fr.clone()), pk);
    doc.add(FeatureKind::Pocket(Extrude::new(s2, 5.0)));

    let mut hs = Sketch::default();
    hs.add_geometry(Geometry { curve: Curve::Circle { c: v(15.0, 0.0), r: 3.0 }, construction: false });
    let s3 = doc.add_sketch(Support::Face(fr), hs);
    let mut h = Hole::new(s3, 6.0, 10.0);
    h.through_all = true;
    h.cut = HoleCut::Counterbore;
    h.cut_diameter = 11.0;
    h.cut_depth = 4.0;
    doc.add(FeatureKind::Hole(h));
    doc.recompute();
    doc
}
