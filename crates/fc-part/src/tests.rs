use super::*;
use crate::params::OnUsers;
use fc_sketch::{Constraint, ConstraintKind as K, Curve, Geometry, PointPos, PointRef, Vec2};
use std::f64::consts::PI;

fn v(x: f64, y: f64) -> Vec2 {
    Vec2 { x, y }
}
fn pr(geo: usize, pos: PointPos) -> PointRef {
    PointRef { geo, pos }
}
fn line(s: &mut Sketch, a: (f64, f64), b: (f64, f64)) -> usize {
    s.add_geometry(Geometry { curve: Curve::Line { a: v(a.0, a.1), b: v(b.0, b.1) }, construction: false })
}

/// Fully constrained w×h rectangle with its lower-left corner at (x, y). Returns (sketch, width constraint index).
fn rect(x: f64, y: f64, w: f64, h: f64) -> (Sketch, usize) {
    let mut s = Sketch::default();
    // Drawn slightly off so the solver has work to do.
    let l0 = line(&mut s, (x, y), (x + w * 0.9, y + 0.1));
    let l1 = line(&mut s, (x + w * 0.9, y + 0.1), (x + w, y + h));
    let l2 = line(&mut s, (x + w, y + h), (x - 0.2, y + h * 1.1));
    let l3 = line(&mut s, (x - 0.2, y + h * 1.1), (x, y));
    for (a, b) in [(l0, l1), (l1, l2), (l2, l3), (l3, l0)] {
        s.add_constraint(Constraint::new(K::Coincident(pr(a, PointPos::End), pr(b, PointPos::Start))));
    }
    s.add_constraint(Constraint::new(K::Horizontal(l0)));
    s.add_constraint(Constraint::new(K::Horizontal(l2)));
    s.add_constraint(Constraint::new(K::Vertical(l1)));
    s.add_constraint(Constraint::new(K::Vertical(l3)));
    let wc = s.add_constraint(Constraint::new(K::DistanceX(pr(l0, PointPos::Start), pr(l0, PointPos::End), w)));
    s.add_constraint(Constraint::new(K::DistanceY(pr(l1, PointPos::Start), pr(l1, PointPos::End), h)));
    s.add_constraint(Constraint::new(K::LockX(pr(l0, PointPos::Start), x)));
    s.add_constraint(Constraint::new(K::LockY(pr(l0, PointPos::Start), y)));
    (s, wc)
}

fn circle(s: &mut Sketch, c: (f64, f64), r: f64) -> usize {
    s.add_geometry(Geometry { curve: Curve::Circle { c: v(c.0, c.1), r }, construction: false })
}

fn ok(doc: &Document, id: FeatureId) -> &Shape {
    let st = doc.state(id).unwrap();
    assert!(st.error.is_none(), "{} failed: {:?}", doc.get(id).unwrap().name, st.error);
    st.shape.as_ref().unwrap()
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-3 * b.abs().max(1.0)
}

#[test]
fn pad_is_parametric() {
    let mut doc = Document::new();
    let (sk, wc) = rect(0.0, 0.0, 20.0, 10.0);
    let s = doc.add_sketch(Support::XY, sk);
    let pad = doc.add(FeatureKind::Pad(Extrude::new(s, 5.0)));
    doc.recompute();
    let st = doc.state(s).unwrap();
    assert_eq!(st.solve.as_ref().unwrap().dof, 0);
    assert!(close(ok(&doc, pad).volume(), 1000.0));
    let bb = ok(&doc, pad).bbox();
    assert!(close(bb[5], 5.0) && close(bb[2], 0.0));

    // Change the width dimension: the pad follows.
    let sk = doc.sketch_mut(s).unwrap();
    match &mut sk.constraints[wc].kind {
        K::DistanceX(_, _, d) => *d = 30.0,
        _ => unreachable!(),
    }
    doc.recompute();
    assert!(close(ok(&doc, pad).volume(), 1500.0));
}

#[test]
fn pad_options() {
    let mut doc = Document::new();
    let s = doc.add_sketch(Support::XY, rect(0.0, 0.0, 10.0, 10.0).0);
    let mut e = Extrude::new(s, 8.0);
    e.side = SideType::Symmetric;
    let pad = doc.add(FeatureKind::Pad(e));
    doc.recompute();
    let bb = ok(&doc, pad).bbox();
    assert!(close(bb[2], -4.0) && close(bb[5], 4.0));

    let mut e2 = Extrude::new(s, 3.0);
    e2.side = SideType::TwoSides;
    e2.length2 = 2.0;
    e2.reversed = true;
    doc.get_mut(pad).unwrap().kind = FeatureKind::Pad(e2);
    doc.recompute();
    let bb = ok(&doc, pad).bbox();
    assert!(close(bb[2], -3.0) && close(bb[5], 2.0), "{bb:?}");
}

#[test]
fn pocket_on_top_face_with_hole_profile() {
    let mut doc = Document::new();
    let s = doc.add_sketch(Support::XY, rect(0.0, 0.0, 40.0, 30.0).0);
    let pad = doc.add(FeatureKind::Pad(Extrude::new(s, 10.0)));
    doc.recompute();
    let body = ok(&doc, pad).clone();
    // Find the top face (normal +Z).
    let top = (1..=body.face_count()).find(|&i| body.face_info(i).unwrap().normal[2] > 0.99).unwrap();
    let fr = face_ref(pad, &body, top).unwrap();

    let mut sk = Sketch::default();
    circle(&mut sk, (5.0, 5.0), 3.0);
    let s2 = doc.add_sketch(Support::Face(fr), sk);
    let pocket = doc.add(FeatureKind::Pocket(Extrude::new(s2, 4.0)));
    doc.recompute();
    let pl = doc.state(s2).unwrap().placement.unwrap();
    assert!(close(pl.origin[2], 10.0));
    let vol = ok(&doc, pocket).volume();
    assert!(close(vol, 40.0 * 30.0 * 10.0 - PI * 9.0 * 4.0), "{vol}");

    // Through-all pocket goes all the way through.
    let mut e = Extrude::new(s2, 0.0);
    e.kind = ExtentType::ThroughAll;
    doc.get_mut(pocket).unwrap().kind = FeatureKind::Pocket(e);
    doc.recompute();
    assert!(close(ok(&doc, pocket).volume(), 40.0 * 30.0 * 10.0 - PI * 9.0 * 10.0));
}

#[test]
fn sketch_on_face_survives_renumbering() {
    let mut doc = Document::new();
    let s = doc.add_sketch(Support::XY, rect(0.0, 0.0, 20.0, 20.0).0);
    let pad = doc.add(FeatureKind::Pad(Extrude::new(s, 10.0)));
    doc.recompute();
    let body = ok(&doc, pad).clone();
    let top = (1..=body.face_count()).find(|&i| body.face_info(i).unwrap().normal[2] > 0.99).unwrap();
    let mut fr = face_ref(pad, &body, top).unwrap();
    fr.index = 1; // simulate a stale index
    let mut sk = Sketch::default();
    circle(&mut sk, (10.0, 10.0), 2.0);
    let s2 = doc.add_sketch(Support::Face(fr), sk);
    let pad2 = doc.add(FeatureKind::Pad(Extrude::new(s2, 5.0)));
    doc.recompute();
    let bb = ok(&doc, pad2).bbox();
    assert!(close(bb[5], 15.0), "{bb:?}");
}

#[test]
fn revolution_and_groove() {
    let mut doc = Document::new();
    // Rectangle in the XZ plane from x=5..10, z=0..20, revolved about the sketch V axis (= Z).
    let s = doc.add_sketch(Support::XZ, rect(5.0, 0.0, 5.0, 20.0).0);
    let rev = doc.add(FeatureKind::Revolution(Revolve::new(s)));
    doc.recompute();
    let tube = ok(&doc, rev);
    assert!(close(tube.volume(), PI * (100.0 - 25.0) * 20.0), "{}", tube.volume());

    // Groove a 1×1 ring channel into the outer wall.
    let s2 = doc.add_sketch(Support::XZ, rect(9.0, 9.0, 1.0, 2.0).0);
    let gr = doc.add(FeatureKind::Groove(Revolve::new(s2)));
    doc.recompute();
    let expect = PI * (100.0 - 25.0) * 20.0 - PI * (100.0 - 81.0) * 2.0;
    assert!(close(ok(&doc, gr).volume(), expect), "{}", ok(&doc, gr).volume());

    // Partial revolution.
    if let FeatureKind::Revolution(r) = &mut doc.get_mut(rev).unwrap().kind {
        r.angle = 90.0;
    }
    doc.recompute();
    assert!(close(ok(&doc, rev).volume(), PI * (100.0 - 25.0) * 20.0 / 4.0));
}

#[test]
fn hole_and_loft() {
    let mut doc = Document::new();
    let s = doc.add_sketch(Support::XY, rect(0.0, 0.0, 30.0, 30.0).0);
    let pad = doc.add(FeatureKind::Pad(Extrude::new(s, 10.0)));
    doc.recompute();
    let body = ok(&doc, pad).clone();
    let top = (1..=body.face_count()).find(|&i| body.face_info(i).unwrap().normal[2] > 0.99).unwrap();
    let mut sk = Sketch::default();
    circle(&mut sk, (15.0, 15.0), 2.0);
    let s2 = doc.add_sketch(Support::Face(face_ref(pad, &body, top).unwrap()), sk);
    let mut h = Hole::new(s2, 6.0, 5.0);
    h.drill_point_angle = 0.0;
    let hole = doc.add(FeatureKind::Hole(h));
    doc.recompute();
    let vol = ok(&doc, hole).volume();
    assert!(close(vol, 9000.0 - PI * 9.0 * 5.0), "{vol}");

    // Loft from a 10x10 square at z=10 to a circle 10 above it.
    let mut d2 = Document::new();
    let a = d2.add_sketch(Support::XY, rect(-5.0, -5.0, 10.0, 10.0).0);
    let mut sk = Sketch::default();
    circle(&mut sk, (0.0, 0.0), 3.0);
    let b = d2.add(FeatureKind::Sketch(SketchFeature { sketch: sk, support: Support::XY, offset: 10.0, reversed: false }));
    let loft = d2.add(FeatureKind::AdditiveLoft(Loft { profile: a, sections: vec![b], ruled: false }));
    d2.recompute();
    let l = ok(&d2, loft);
    assert!(l.volume() > PI * 9.0 * 10.0 && l.volume() < 1000.0, "{}", l.volume());
}

#[test]
fn errors_and_json() {
    let mut doc = Document::new();
    let mut sk = Sketch::default();
    line(&mut sk, (0.0, 0.0), (10.0, 0.0));
    let s = doc.add_sketch(Support::XY, sk);
    let pad = doc.add(FeatureKind::Pad(Extrude::new(s, 5.0)));
    doc.recompute();
    assert!(doc.state(pad).unwrap().error.is_some());
    assert!(doc.remove(s).is_err());

    let s2 = doc.add_sketch(Support::XY, rect(0.0, 0.0, 10.0, 10.0).0);
    if let FeatureKind::Pad(e) = &mut doc.get_mut(pad).unwrap().kind {
        e.profile = s2;
    }
    let json = doc.to_json();
    let mut back = Document::from_json(&json).unwrap();
    back.recompute();
    // Pad references a sketch that now comes after it in the tree -> error, not a panic.
    assert!(back.state(pad).unwrap().error.is_some());
    assert_eq!(back.features.len(), 3);
}

#[test]
fn named_dimensions_drive_the_model() {
    let mut doc = Document::new();
    let (sk, wc) = rect(0.0, 0.0, 20.0, 10.0);
    let s = doc.add_sketch(Support::XY, sk);
    let pad = doc.add(FeatureKind::Pad(Extrude::new(s, 5.0)));
    doc.name_constraint(s, wc, "width").unwrap();
    doc.set_parameter("thickness", "4 mm").unwrap();
    doc.set_expression("Pad.Length", "thickness * 2").unwrap();
    // The height (constraint wc+1) follows the width by name.
    doc.set_expression(&format!("Sketch.Constraints[{}]", wc + 1), "width / 2 + 3").unwrap();
    doc.recompute();
    assert!(doc.expression_errors.is_empty(), "{:?}", doc.expression_errors);
    assert!(close(ok(&doc, pad).volume(), 20.0 * 13.0 * 8.0));

    // Changing the named sketch dimension updates everything that uses it.
    doc.set_expression("Sketch.width", "30").unwrap();
    doc.recompute();
    assert!(close(ok(&doc, pad).volume(), 30.0 * 18.0 * 8.0));
    assert!(close(doc.read_value("Pad.Length").unwrap(), 8.0));
    assert!(doc.available_names().iter().any(|(n, v)| n == "Sketch.width" && close(*v, 30.0)));

    // Cycles are rejected and leave the document unchanged.
    assert!(doc.set_expression("Sketch.width", &format!("Sketch.Constraints[{}] * 2", wc + 1)).is_err());
    assert!(doc.set_expression("Pad.Nope", "1").is_err());
    assert!(doc.name_constraint(s, wc + 1, "width").is_err());

    // Survives save/load.
    let mut back = Document::from_json(&doc.to_json()).unwrap();
    back.recompute();
    assert!(close(ok(&back, pad).volume(), 30.0 * 18.0 * 8.0));
}


#[test]
fn deleting_named_values() {
    let mut doc = Document::new();
    let (sk, wc) = rect(0.0, 0.0, 20.0, 10.0);
    let s = doc.add_sketch(Support::XY, sk);
    let pad = doc.add(FeatureKind::Pad(Extrude::new(s, 5.0)));
    doc.set_parameter("thickness", "4").unwrap();
    doc.set_expression("Pad.Length", "thickness * 2").unwrap();
    doc.name_constraint(s, wc, "width").unwrap();
    doc.set_expression(&format!("Sketch.Constraints[{}]", wc + 1), "Sketch.Constraints[8] / 2").unwrap();
    doc.recompute();

    // Refused while used, with a readable list of users.
    let users = doc.remove_parameter("thickness", OnUsers::Refuse).unwrap_err();
    assert_eq!(users, vec!["Pad.Length".to_string()]);
    // Freeze: Pad.Length keeps 8 as a plain value.
    doc.remove_parameter("thickness", OnUsers::Freeze).unwrap();
    assert!(doc.parameters.is_empty() && !doc.expressions.contains_key("Pad.Length"));
    doc.recompute();
    assert!(close(ok(&doc, pad).volume(), 20.0 * 10.0 * 8.0));

    // Deleting an earlier constraint renumbers index references (8 -> 7, 9 -> 8).
    assert_eq!(wc, 8);
    doc.delete_constraints(s, &[0], OnUsers::Refuse).unwrap();
    assert_eq!(doc.expressions.get("Sketch.Constraints[8]").map(String::as_str), Some("Sketch.Constraints[7] / 2"));
    assert_eq!(doc.canonical_name("width").as_deref(), Some("Sketch.Constraints[7]"));

    // Deleting the width dimension itself is refused while the height formula uses it...
    assert!(doc.delete_constraints(s, &[7], OnUsers::Refuse).is_err());
    // ...and frozen on request.
    doc.delete_constraints(s, &[7], OnUsers::Freeze).unwrap();
    assert!(doc.expressions.is_empty(), "{:?}", doc.expressions);
    assert!(doc.canonical_name("width").is_none());
}
