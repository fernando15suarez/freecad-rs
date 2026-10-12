//! Mate connectors: inference on faces/edges/vertices, offsets, rebuild persistence,
//! expressions and JSON round trip.

use fc_part::*;
use fc_sketch::{Constraint, ConstraintKind as K, Curve, Geometry, PointPos, PointRef, Sketch, Vec2};

fn v(x: f64, y: f64) -> Vec2 {
    Vec2 { x, y }
}
fn pr(geo: usize, pos: PointPos) -> PointRef {
    PointRef { geo, pos }
}
fn line(s: &mut Sketch, a: (f64, f64), b: (f64, f64)) -> usize {
    s.add_geometry(Geometry { curve: Curve::Line { a: v(a.0, a.1), b: v(b.0, b.1) }, construction: false })
}

/// Fully constrained w×h rectangle at the origin. Returns (sketch, width constraint index).
fn rect(w: f64, h: f64) -> (Sketch, usize) {
    let mut s = Sketch::default();
    let l0 = line(&mut s, (0.0, 0.0), (w, 0.0));
    let l1 = line(&mut s, (w, 0.0), (w, h));
    let l2 = line(&mut s, (w, h), (0.0, h));
    let l3 = line(&mut s, (0.0, h), (0.0, 0.0));
    for (a, b) in [(l0, l1), (l1, l2), (l2, l3), (l3, l0)] {
        s.add_constraint(Constraint::new(K::Coincident(pr(a, PointPos::End), pr(b, PointPos::Start))));
    }
    s.add_constraint(Constraint::new(K::Horizontal(l0)));
    s.add_constraint(Constraint::new(K::Horizontal(l2)));
    s.add_constraint(Constraint::new(K::Vertical(l1)));
    s.add_constraint(Constraint::new(K::Vertical(l3)));
    let wc = s.add_constraint(Constraint::new(K::DistanceX(pr(l0, PointPos::Start), pr(l0, PointPos::End), w)));
    s.add_constraint(Constraint::new(K::DistanceY(pr(l1, PointPos::Start), pr(l1, PointPos::End), h)));
    s.add_constraint(Constraint::new(K::LockX(pr(l0, PointPos::Start), 0.0)));
    s.add_constraint(Constraint::new(K::LockY(pr(l0, PointPos::Start), 0.0)));
    (s, wc)
}

fn near(a: V3, b: V3) -> bool {
    (0..3).all(|k| (a[k] - b[k]).abs() < 1e-6)
}
#[track_caller]
fn assert_v(a: V3, b: V3) {
    assert!(near(a, b), "{a:?} != {b:?}");
}
fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
#[track_caller]
fn assert_frame(f: &Frame) {
    assert_v(cross(f.x, f.y), f.z);
}

/// 20×10×5 pad on XY. Returns (doc, sketch, pad, width constraint).
fn block() -> (Document, FeatureId, FeatureId, usize) {
    let mut doc = Document::new();
    let (sk, wc) = rect(20.0, 10.0);
    let s = doc.add_sketch(Support::XY, sk);
    let pad = doc.add(FeatureKind::Pad(Extrude::new(s, 5.0)));
    doc.recompute();
    (doc, s, pad, wc)
}

fn connector(doc: &mut Document, feature: FeatureId, name: &str, infer: Infer) -> FeatureId {
    let at = doc.topo_ref(feature, name).unwrap();
    doc.add_mate_connector(MateConnector::new(Anchor::Topo { at, infer }))
}

fn frame(doc: &Document, id: FeatureId) -> Frame {
    let f = doc.connector_frame(id).unwrap_or_else(|e| panic!("connector failed: {e}"));
    assert_frame(&f);
    assert_eq!(doc.state(id).unwrap().frame, Some(f), "frame lives in FeatureState");
    f
}

#[test]
fn planar_face_center() {
    let (mut doc, _, pad, _) = block();
    let top = doc.state(pad).unwrap().shape.as_ref().unwrap().face_info(6).unwrap();
    assert!(top.normal[2] > 0.99, "Pad.Face6 is the top face");
    let mc = connector(&mut doc, pad, "Face6", Infer::Auto);
    doc.recompute();
    let f = frame(&doc, mc);
    assert_v(f.origin, [10.0, 5.0, 5.0]);
    assert_v(f.z, [0.0, 0.0, 1.0]);
    assert_v(f.x, [1.0, 0.0, 0.0]);
    // A connector does not change the body.
    assert!(!doc.get(mc).unwrap().kind.is_solid_feature());
    assert!(doc.state(mc).unwrap().shape.is_none());
    assert!((doc.result().unwrap().volume() - 1000.0).abs() < 1e-6);
    // Closest point to a pick.
    let at = doc.topo_ref(pad, "Face6").unwrap();
    let pick = doc.add_mate_connector(MateConnector::new(Anchor::Topo { at, infer: Infer::Closest([3.0, 4.0, 99.0]) }));
    doc.recompute();
    assert_v(frame(&doc, pick).origin, [3.0, 4.0, 5.0]);
    // A side face: Z is its outward normal.
    let side = (1..=6).find(|&i| doc.result().unwrap().face_info(i).unwrap().normal[0] > 0.99).unwrap();
    let sc = connector(&mut doc, pad, &format!("Face{side}"), Infer::Auto);
    doc.recompute();
    let f = frame(&doc, sc);
    assert_v(f.origin, [20.0, 5.0, 2.5]);
    assert_v(f.z, [1.0, 0.0, 0.0]);
}

/// Block 40×30×10 with a Ø6 × 5 hole at (20, 15) drilled from the top.
fn drilled() -> (Document, FeatureId, FeatureId, FeatureId) {
    let mut doc = Document::new();
    let (sk, _) = rect(40.0, 30.0);
    let s = doc.add_sketch(Support::XY, sk);
    let pad = doc.add(FeatureKind::Pad(Extrude::new(s, 10.0)));
    doc.recompute();
    let body = doc.state(pad).unwrap().shape.clone().unwrap();
    let top = (1..=body.face_count()).find(|&i| body.face_info(i).unwrap().normal[2] > 0.99).unwrap();
    let mut hs = Sketch::default();
    hs.add_geometry(Geometry { curve: Curve::Circle { c: v(20.0, 15.0), r: 3.0 }, construction: false });
    let s2 = doc.add_sketch(Support::Face(face_ref(pad, &body, top).unwrap()), hs);
    let hole = doc.add(FeatureKind::Hole(Hole::new(s2, 6.0, 5.0)));
    doc.recompute();
    assert!(doc.state(hole).unwrap().error.is_none());
    (doc, pad, s2, hole)
}

#[test]
fn hole_edge_and_cylinder_axis() {
    let (mut doc, _, _, hole) = drilled();
    let shape = doc.state(hole).unwrap().shape.clone().unwrap();
    let rim = (1..=shape.edge_count())
        .find(|&i| {
            let e = shape.edge_info(i).unwrap();
            e.kind == 1 && (e.radius - 3.0).abs() < 1e-6 && (e.center[2] - 10.0).abs() < 1e-6
        })
        .expect("hole rim edge");
    let ec = connector(&mut doc, hole, &format!("Edge{rim}"), Infer::Auto);
    let cyl = (1..=shape.face_count()).find(|&i| shape.face_info(i).unwrap().kind == 1).expect("hole cylinder");
    let cc = connector(&mut doc, hole, &format!("Face{cyl}"), Infer::Auto);
    let cs = connector(&mut doc, hole, &format!("Face{cyl}"), Infer::Start);
    let ce = connector(&mut doc, hole, &format!("Face{cyl}"), Infer::End);
    let cp = connector(&mut doc, hole, &format!("Face{cyl}"), Infer::Closest([0.0, 0.0, 99.0]));
    doc.recompute();
    // Rim: origin at the hole center on the top face, Z along the hole axis out of the part.
    let f = frame(&doc, ec);
    assert_v(f.origin, [20.0, 15.0, 10.0]);
    assert_v(f.z, [0.0, 0.0, 1.0]);
    // Cylinder face: on the axis, midway along the 5 mm cylindrical part.
    let f = frame(&doc, cc);
    assert_v(f.origin, [20.0, 15.0, 7.5]);
    assert_v(f.z, [0.0, 0.0, 1.0]);
    assert_v(frame(&doc, cs).origin, [20.0, 15.0, 5.0]);
    assert_v(frame(&doc, ce).origin, [20.0, 15.0, 10.0]);
    assert_v(frame(&doc, cp).origin, [20.0, 15.0, 10.0]);
}

#[test]
fn planar_face_circular_boundary_center() {
    // Top face of a plate with a through pocket-free hole: Center infers the hole center.
    let (mut doc, _, _, hole) = drilled();
    let shape = doc.state(hole).unwrap().shape.clone().unwrap();
    let top = (1..=shape.face_count()).find(|&i| { let f = shape.face_info(i).unwrap(); f.kind == 0 && f.normal[2] > 0.99 }).unwrap();
    let c = connector(&mut doc, hole, &format!("Face{top}"), Infer::Center);
    let bottom = (1..=shape.face_count()).find(|&i| { let f = shape.face_info(i).unwrap(); f.kind == 0 && f.normal[2] < -0.99 }).unwrap();
    let b = connector(&mut doc, hole, &format!("Face{bottom}"), Infer::Center);
    doc.recompute();
    let f = frame(&doc, c);
    assert_v(f.origin, [20.0, 15.0, 10.0]);
    assert_v(f.z, [0.0, 0.0, 1.0]);
    let e = doc.connector_frame(b).unwrap_err();
    assert!(e.to_string().contains("no circular boundary"), "{e}");
}

#[test]
fn vertex_connector() {
    let (mut doc, _, pad, _) = block();
    let shape = doc.state(pad).unwrap().shape.clone().unwrap();
    let vi = shape.vertices().iter().position(|p| near(*p, [20.0, 10.0, 5.0])).unwrap() + 1;
    let a = connector(&mut doc, pad, &format!("Vertex{vi}"), Infer::Auto);
    let b = connector(&mut doc, pad, &format!("Vertex{vi}"), Infer::Normal([1.0, 0.2, 0.1]));
    let c = connector(&mut doc, pad, &format!("Vertex{vi}"), Infer::Normal([0.0, 1.0, 0.0]));
    doc.recompute();
    let f = frame(&doc, a);
    assert_v(f.origin, [20.0, 10.0, 5.0]);
    assert_v(f.z, [0.0, 0.0, 1.0]);
    assert_v(frame(&doc, b).z, [1.0, 0.0, 0.0]);
    assert_v(frame(&doc, c).z, [0.0, 1.0, 0.0]);
}

#[test]
fn line_edge_connector() {
    let (mut doc, _, pad, _) = block();
    let shape = doc.state(pad).unwrap().shape.clone().unwrap();
    // The top edge along X at y = 0.
    let ei = (1..=shape.edge_count())
        .find(|&i| {
            let e = shape.edge_info(i).unwrap();
            e.kind == 0 && near(e.mid, [10.0, 0.0, 5.0])
        })
        .unwrap();
    let m = connector(&mut doc, pad, &format!("Edge{ei}"), Infer::Auto);
    let p = connector(&mut doc, pad, &format!("Edge{ei}"), Infer::Closest([19.0, -1.0, 5.0]));
    doc.recompute();
    let f = frame(&doc, m);
    assert_v(f.origin, [10.0, 0.0, 5.0]);
    assert!((f.z[0].abs() - 1.0).abs() < 1e-9, "Z along the edge: {:?}", f.z);
    assert_v(frame(&doc, p).origin, [20.0, 0.0, 5.0]);
}

#[test]
fn offsets_rotations_and_flips() {
    let (mut doc, _, pad, _) = block();
    let at = doc.topo_ref(pad, "Face6").unwrap();
    let anchor = Anchor::Topo { at, infer: Infer::Auto };
    let mk = |doc: &mut Document, f: &dyn Fn(&mut MateConnector)| {
        let mut m = MateConnector::new(anchor.clone());
        f(&mut m);
        doc.add_mate_connector(m)
    };
    let moved = mk(&mut doc, &|m| m.offset = [1.0, 2.0, 3.0]);
    let rz = mk(&mut doc, &|m| m.rotation = [0.0, 0.0, 90.0]);
    let flip = mk(&mut doc, &|m| m.flip_z = true);
    let turn = mk(&mut doc, &|m| m.x_turns = 1);
    let turn_neg = mk(&mut doc, &|m| m.x_turns = -1);
    let rx = mk(&mut doc, &|m| m.rotation = [90.0, 0.0, 0.0]);
    let ry = mk(&mut doc, &|m| m.rotation = [0.0, 90.0, 0.0]);
    // Translation happens in the flipped frame, rotation after translation.
    let combo = mk(&mut doc, &|m| {
        m.flip_z = true;
        m.offset = [0.0, 0.0, 2.0];
        m.rotation = [0.0, 0.0, 90.0];
    });
    let origin = doc.add_mate_connector(MateConnector { anchor: Anchor::Origin, offset: [1.0, 0.0, 0.0], rotation: [0.0, 0.0, 30.0], ..Default::default() });
    doc.recompute();
    assert_v(frame(&doc, moved).origin, [11.0, 7.0, 8.0]);
    let f = frame(&doc, rz);
    assert_v(f.x, [0.0, 1.0, 0.0]);
    assert_v(f.z, [0.0, 0.0, 1.0]);
    let f = frame(&doc, flip);
    assert_v(f.z, [0.0, 0.0, -1.0]);
    assert_v(f.x, [1.0, 0.0, 0.0]);
    assert_v(f.y, [0.0, -1.0, 0.0]);
    assert_v(frame(&doc, turn).x, [0.0, 1.0, 0.0]);
    assert_v(frame(&doc, turn_neg).x, [0.0, -1.0, 0.0]);
    let f = frame(&doc, rx);
    assert_v(f.z, [0.0, -1.0, 0.0]);
    assert_v(f.x, [1.0, 0.0, 0.0]);
    let f = frame(&doc, ry);
    assert_v(f.z, [1.0, 0.0, 0.0]);
    assert_v(f.y, [0.0, 1.0, 0.0]);
    let f = frame(&doc, combo);
    assert_v(f.origin, [10.0, 5.0, 3.0]);
    assert_v(f.z, [0.0, 0.0, -1.0]);
    assert_v(f.x, [0.0, -1.0, 0.0]);
    let f = frame(&doc, origin);
    assert_v(f.origin, [1.0, 0.0, 0.0]);
    let (s, c) = 30f64.to_radians().sin_cos();
    assert_v(f.x, [c, s, 0.0]);
    // Frame helpers.
    let f = frame(&doc, combo);
    assert_v(f.to_world(f.to_local([1.0, 2.0, 3.0])), [1.0, 2.0, 3.0]);
    assert_v(f.placement().normal(), f.z);
}

#[test]
fn survives_rebuilds() {
    let (mut doc, s, pad, wc) = block();
    let mc = connector(&mut doc, pad, "Face6", Infer::Auto);
    doc.recompute();
    assert_v(frame(&doc, mc).origin, [10.0, 5.0, 5.0]);
    // Pad length change.
    let FeatureKind::Pad(e) = &mut doc.get_mut(pad).unwrap().kind else { panic!() };
    e.length = 12.0;
    doc.recompute();
    let f = frame(&doc, mc);
    assert_v(f.origin, [10.0, 5.0, 12.0]);
    assert_v(f.z, [0.0, 0.0, 1.0]);
    // Sketch dimension change.
    let sname = doc.get(s).unwrap().name.clone();
    assert!(doc.write_value(&format!("{sname}.Constraints[{wc}]"), 30.0));
    doc.recompute();
    assert_v(frame(&doc, mc).origin, [15.0, 5.0, 12.0]);
    // Renumbered topology: a stale index is re-found by its signature.
    if let FeatureKind::MateConnector(m) = &mut doc.get_mut(mc).unwrap().kind {
        let Anchor::Topo { at, .. } = &mut m.anchor else { panic!() };
        at.index = 1;
    }
    doc.recompute();
    assert_v(frame(&doc, mc).origin, [15.0, 5.0, 12.0]);
    // The anchor feature cannot be deleted while the connector uses it.
    assert!(doc.dependents(pad).contains(&mc));
    assert!(doc.remove(pad).is_err());
}

#[test]
fn dangling_anchor_is_reported() {
    let (mut doc, _, _, hole) = drilled();
    let shape = doc.state(hole).unwrap().shape.clone().unwrap();
    let cyl = (1..=shape.face_count()).find(|&i| shape.face_info(i).unwrap().kind == 1).unwrap();
    let mc = connector(&mut doc, hole, &format!("Face{cyl}"), Infer::Auto);
    doc.recompute();
    assert!(doc.connector_frame(mc).is_ok());
    doc.get_mut(hole).unwrap().suppressed = true;
    doc.recompute();
    let err = doc.connector_frame(mc).unwrap_err();
    assert!(matches!(err, PartError::Dangling(_)), "{err:?}");
    assert!(err.to_string().contains("dangling"), "{err}");
    assert!(doc.state(mc).unwrap().frame.is_none());
    // The rest of the body is unaffected.
    assert!(doc.result().is_some());
    doc.get_mut(hole).unwrap().suppressed = false;
    doc.recompute();
    assert_v(frame(&doc, mc).origin, [20.0, 15.0, 7.5]);
}

#[test]
fn expression_driven_offset() {
    let (mut doc, _, pad, _) = block();
    let mc = connector(&mut doc, pad, "Face6", Infer::Auto);
    let name = doc.get(mc).unwrap().name.clone();
    assert_eq!(name, "MateConnector");
    doc.set_parameter("lift", "4 mm").unwrap();
    doc.set_expression("MateConnector.OffsetZ", "lift * 2").unwrap();
    doc.set_expression("MateConnector.RotationZ", "45 + 45").unwrap();
    doc.set_expression("Pad.Length", "lift + 1").unwrap();
    doc.recompute();
    assert!(doc.expression_errors.is_empty(), "{:?}", doc.expression_errors);
    let f = frame(&doc, mc);
    assert_v(f.origin, [10.0, 5.0, 5.0 + 8.0]);
    assert_v(f.x, [0.0, 1.0, 0.0]);
    assert_eq!(doc.read_value("MateConnector.OffsetZ"), Some(8.0));
    assert!(doc.available_names().iter().any(|(n, _)| n == "MateConnector.RotationZ"));
    doc.set_parameter("lift", "1").unwrap();
    doc.recompute();
    assert_v(frame(&doc, mc).origin, [10.0, 5.0, 2.0 + 2.0]);
}

#[test]
fn json_round_trip() {
    let (mut doc, _, _, hole) = drilled();
    let shape = doc.state(hole).unwrap().shape.clone().unwrap();
    let cyl = (1..=shape.face_count()).find(|&i| shape.face_info(i).unwrap().kind == 1).unwrap();
    let at = doc.topo_ref(hole, &format!("Face{cyl}")).unwrap();
    let mc = doc.add_mate_connector(MateConnector {
        anchor: Anchor::Topo { at, infer: Infer::Closest([1.0, 2.0, 3.0]) },
        flip_z: true,
        x_turns: 3,
        offset: [1.0, 2.0, 3.0],
        rotation: [10.0, 20.0, 30.0],
    });
    let o = doc.add_mate_connector(MateConnector::new(Anchor::Origin));
    doc.recompute();
    let before = frame(&doc, mc);
    let json = doc.to_json();
    let mut back = Document::from_json(&json).unwrap();
    // serde_json may change a float's last digit; compare the connectors field by field.
    let (FeatureKind::MateConnector(m0), FeatureKind::MateConnector(m1)) = (&doc.get(mc).unwrap().kind, &back.get(mc).unwrap().kind) else { panic!() };
    assert_eq!((m0.flip_z, m0.x_turns, m0.offset, m0.rotation), (m1.flip_z, m1.x_turns, m1.offset, m1.rotation));
    let (Anchor::Topo { at: a0, infer: i0 }, Anchor::Topo { at: a1, infer: i1 }) = (&m0.anchor, &m1.anchor) else { panic!() };
    assert_eq!((i0, a0.feature, a0.kind, a0.index, a0.geom), (i1, a1.feature, a1.kind, a1.index, a1.geom));
    assert!(near(a0.center, a1.center) && near(a0.dir, a1.dir) && (a0.size - a1.size).abs() < 1e-9);
    assert_eq!(back.get(o).unwrap().kind, doc.get(o).unwrap().kind);
    back.recompute();
    let after = back.connector_frame(mc).unwrap();
    assert!(near(before.origin, after.origin) && near(before.x, after.x) && near(before.z, after.z));
    assert_eq!(back.connector_frame(o).unwrap(), Frame::WORLD);
    // Errors for non-connectors.
    assert!(back.connector_frame(hole).is_err());
    assert_eq!(TopoKind::parse("Edge12"), Some((TopoKind::Edge, 12)));
    assert_eq!(TopoKind::parse("Face0"), None);
}
