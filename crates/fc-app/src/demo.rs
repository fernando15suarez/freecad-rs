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

/// fc-fem's plate_hole parity geometry: a 200 x 100 x 10 plate (corner at the origin) with a
/// Ø20 hole in its middle, as one sketch (rectangle + circle) padded 10 mm.
pub fn plate_hole_document() -> Document {
    let mut doc = Document::new();
    let (mut sk, _) = rect_sketch(0.0, 0.0, 200.0, 100.0);
    let c = sk.add_geometry(Geometry { curve: Curve::Circle { c: v(100.0, 50.0), r: 10.0 }, construction: false });
    sk.add_constraint(Constraint::new(K::Diameter(c, 20.0)));
    sk.add_constraint(Constraint::new(K::LockX(pr(c, PointPos::Mid), 100.0)));
    sk.add_constraint(Constraint::new(K::LockY(pr(c, PointPos::Mid), 50.0)));
    let s = doc.add_sketch(Support::XY, sk);
    doc.add(FeatureKind::Pad(Extrude::new(s, 10.0)));
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

/// A box `[x0, x0 + w] x [y0, y0 + d] x [0, h]` as a part document.
fn box_part(x0: f64, y0: f64, w: f64, d: f64, h: f64) -> Document {
    let mut doc = Document::new();
    let s = doc.add_sketch(Support::XY, rect_sketch(x0, y0, w, d).0);
    doc.add(FeatureKind::Pad(Extrude::new(s, h)));
    doc.recompute();
    doc
}

/// A cylinder of radius `r` and height `h` on the origin.
fn pin_part(r: f64, h: f64) -> Document {
    let mut doc = Document::new();
    let mut sk = Sketch::default();
    let c = sk.add_geometry(Geometry { curve: Curve::Circle { c: v(0.0, 0.0), r }, construction: false });
    sk.add_constraint(Constraint::new(K::Diameter(c, 2.0 * r)));
    sk.add_constraint(Constraint::new(K::LockX(pr(c, PointPos::Mid), 0.0)));
    sk.add_constraint(Constraint::new(K::LockY(pr(c, PointPos::Mid), 0.0)));
    let s = doc.add_sketch(Support::XY, sk);
    doc.add(FeatureKind::Pad(Extrude::new(s, h)));
    doc.recompute();
    doc
}

fn meta(material: &str, part_number: &str, description: &str) -> fc_assembly::PartMeta {
    fc_assembly::PartMeta {
        part_number: part_number.into(),
        description: description.into(),
        material: material.into(),
        density: fc_fem::Material::by_name(material).map(|m| m.density),
        ..Default::default()
    }
}

/// Test assemblies for the e2e scripts (`asmtest NAME`):
/// - `overlap`: 20 mm cubes BlockA (grounded, at the origin), BlockB at x = 10 (overlaps A by
///   10 x 20 x 20 = 4000 mm³) and BlockC at x = 50 (20 mm from B);
/// - `bom`: "Main" = Plate (100 x 60 x 10) + 3 Bolts (Ø6 x 20) + the sub-assembly "BoltPair"
///   (2 Bolts); materials Aluminum 6061-T6 and Steel-Generic;
/// - `stack`: fc-fem's asm_stack: a 40 x 40 x 20 aluminium block "Lower" under a steel
///   block "Upper" of the same size.
pub fn asm_test_workspace(name: &str) -> Option<fc_assembly::Workspace> {
    use fc_assembly::Frame;
    let mut ws = fc_assembly::Workspace::new();
    match name {
        "overlap" => {
            let ids: Vec<_> = ["BlockA", "BlockB", "BlockC"].iter().map(|n| ws.add_part(n, box_part(0.0, 0.0, 20.0, 20.0, 20.0))).collect();
            let a = ws.add_assembly("Assembly");
            let asm = ws.assembly_mut(a).unwrap();
            let ia = asm.add_part_instance("BlockA", ids[0], Frame::IDENTITY);
            asm.set_grounded(ia, true);
            asm.add_part_instance("BlockB", ids[1], Frame::translation([10.0, 0.0, 0.0]));
            asm.add_part_instance("BlockC", ids[2], Frame::translation([50.0, 0.0, 0.0]));
        }
        "bom" => {
            let plate = ws.add_part("Plate", box_part(0.0, 0.0, 100.0, 60.0, 10.0));
            let bolt = ws.add_part("Bolt", pin_part(3.0, 20.0));
            ws.part_mut(plate).unwrap().meta = meta("Aluminum 6061-T6", "PL-100", "Base plate");
            ws.part_mut(bolt).unwrap().meta = meta("Steel-Generic", "", "Pin Ø6 x 20");
            let main = ws.add_assembly("Main");
            let pair = ws.add_assembly("BoltPair");
            {
                let p = ws.assembly_mut(pair).unwrap();
                p.add_part_instance("Bolt", bolt, Frame::IDENTITY);
                p.add_part_instance("Bolt", bolt, Frame::translation([12.0, 0.0, 0.0]));
            }
            let m = ws.assembly_mut(main).unwrap();
            let ip = m.add_part_instance("Plate", plate, Frame::IDENTITY);
            m.set_grounded(ip, true);
            for x in [15.0, 35.0, 55.0] {
                m.add_part_instance("Bolt", bolt, Frame::translation([x, 15.0, 10.0]));
            }
            ws.add_sub_assembly(main, "BoltPair", pair, Frame::translation([70.0, 45.0, 10.0])).unwrap();
        }
        "stack" => {
            let lo = ws.add_part("Lower", box_part(0.0, 0.0, 40.0, 40.0, 20.0));
            let up = ws.add_part("Upper", box_part(0.0, 0.0, 40.0, 40.0, 20.0));
            ws.part_mut(lo).unwrap().meta = meta("Aluminum 6061-T6", "", "");
            ws.part_mut(up).unwrap().meta = meta("Steel-Generic", "", "");
            let a = ws.add_assembly("Stack");
            let asm = ws.assembly_mut(a).unwrap();
            let il = asm.add_part_instance("Lower", lo, Frame::IDENTITY);
            asm.set_grounded(il, true);
            asm.add_part_instance("Upper", up, Frame::translation([0.0, 0.0, 20.0]));
        }
        _ => return None,
    }
    Some(ws)
}
