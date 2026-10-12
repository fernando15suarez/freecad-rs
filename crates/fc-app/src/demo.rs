//! Demo document used by `FCRS_DEMO=1` (screenshots and first launch checks).

use fc_part::*;
use fc_part::{Anchor, Infer, MateConnector, TopoKind};
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

/// A pad of `sk` with through holes at `holes` (diameter `d`), on the top face.
fn plate_with_holes(sk: Sketch, height: f64, holes: &[(f64, f64)], d: f64) -> (Document, FeatureId, FeatureId) {
    let mut doc = Document::new();
    let s = doc.add_sketch(Support::XY, sk);
    let pad = doc.add(FeatureKind::Pad(Extrude::new(s, height)));
    doc.recompute();
    let body = doc.state(pad).and_then(|s| s.shape.clone()).expect("pad");
    let top = (1..=body.face_count()).find(|&i| body.face_info(i).map(|f| f.normal[2] > 0.99).unwrap_or(false)).unwrap();
    let fr = face_ref(pad, &body, top).unwrap();
    let mut hs = Sketch::default();
    for (x, y) in holes {
        hs.add_geometry(Geometry { curve: Curve::Circle { c: v(*x, *y), r: d * 0.5 }, construction: false });
    }
    let s2 = doc.add_sketch(Support::Face(fr), hs);
    let mut h = Hole::new(s2, d, 10.0);
    h.through_all = true;
    let hole = doc.add(FeatureKind::Hole(h));
    doc.recompute();
    (doc, pad, hole)
}

/// A mate connector on the circular edge of `feature`'s solid centered at `c`.
fn rim_connector(doc: &mut Document, feature: FeatureId, c: [f64; 3]) -> FeatureId {
    let shape = doc.state(feature).and_then(|s| s.shape.clone()).expect("solid");
    let e = (1..=shape.edge_count())
        .find(|&i| shape.edge_info(i).map(|e| e.kind == 1 && (0..3).all(|k| (e.center[k] - c[k]).abs() < 1e-6)).unwrap_or(false))
        .expect("rim edge");
    let at = fc_part::mate::topo_ref(feature, &shape, TopoKind::Edge, e).unwrap();
    let id = doc.add_mate_connector(MateConnector::new(Anchor::Topo { at, infer: Infer::Center }));
    doc.recompute();
    id
}

/// Two parts with mate connectors on hole rims, for assembly tests and screenshots:
/// "Plate" (60 x 40 x 10, hole Ø6 at (15, 0), connector on the top rim) and "Arm"
/// (50 x 10 x 5 bar, hole Ø6 at its origin, connector on the bottom rim). With
/// `assembled`, also an assembly with the plate grounded and the arm on a revolute joint.
pub fn assembly_workspace(assembled: bool) -> fc_assembly::Workspace {
    let (mut plate, _, ph) = plate_with_holes(rect_sketch(-30.0, -20.0, 60.0, 40.0).0, 10.0, &[(15.0, 0.0)], 6.0);
    let pc = rim_connector(&mut plate, ph, [15.0, 0.0, 10.0]);
    let (mut arm, _, ah) = plate_with_holes(rect_sketch(-5.0, -5.0, 50.0, 10.0).0, 5.0, &[(0.0, 0.0)], 6.0);
    let ac = rim_connector(&mut arm, ah, [0.0, 0.0, 0.0]);
    let mut ws = fc_assembly::Workspace::new();
    let p = ws.add_part("Plate", plate);
    let a = ws.add_part("Arm", arm);
    if assembled {
        let id = ws.add_assembly("Assembly");
        let asm = ws.assembly_mut(id).unwrap();
        let ip = asm.add_part_instance("Plate", p, fc_assembly::Frame::IDENTITY);
        asm.set_grounded(ip, true);
        let ia = asm.add_part_instance("Arm", a, fc_assembly::Frame::from_ypr([10.0, 40.0, 0.0], 30.0, 0.0, 0.0));
        let j = fc_assembly::Joint::new(
            fc_assembly::JointKind::Revolute,
            fc_assembly::JointRef::PartConnector { instance: ip, connector: pc },
            fc_assembly::JointRef::PartConnector { instance: ia, connector: ac },
        );
        let jid = asm.add_joint(j);
        let mut asm2 = ws.assembly(id).unwrap().clone();
        ws.recompute();
        let _ = asm2.presolve_joint(&ws, jid);
        *ws.assembly_mut(id).unwrap() = asm2;
    }
    ws
}
