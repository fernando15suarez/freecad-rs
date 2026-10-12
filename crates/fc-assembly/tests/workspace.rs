//! Assemblies over real fc-part bodies: mate connector features, FreeCAD-style geometry
//! picks and Onshape-style inference resolved through the workspace.

use fc_assembly::*;
use fc_part::{Anchor, Document, Extrude, FeatureId, FeatureKind, Infer, MateConnector, Support};
use fc_sketch::{Constraint, ConstraintKind as K, Curve, Geometry, PointPos, PointRef, Sketch, Vec2};

fn line(s: &mut Sketch, a: (f64, f64), b: (f64, f64)) -> usize {
    let v = |x, y| Vec2 { x, y };
    s.add_geometry(Geometry { curve: Curve::Line { a: v(a.0, a.1), b: v(b.0, b.1) }, construction: false })
}

fn rect(w: f64, h: f64) -> Sketch {
    let pr = |geo, pos| PointRef { geo, pos };
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
    s.add_constraint(Constraint::new(K::DistanceX(pr(l0, PointPos::Start), pr(l0, PointPos::End), w)));
    s.add_constraint(Constraint::new(K::DistanceY(pr(l1, PointPos::Start), pr(l1, PointPos::End), h)));
    s.add_constraint(Constraint::new(K::LockX(pr(l0, PointPos::Start), 0.0)));
    s.add_constraint(Constraint::new(K::LockY(pr(l0, PointPos::Start), 0.0)));
    s
}

/// 20×10×5 block with mate connectors at the centres of its top and bottom faces.
fn block() -> (Document, FeatureId, FeatureId, FeatureId, String) {
    let mut doc = Document::new();
    let s = doc.add_sketch(Support::XY, rect(20.0, 10.0));
    let pad = doc.add(FeatureKind::Pad(Extrude::new(s, 5.0)));
    doc.recompute();
    let shape = doc.state(pad).unwrap().shape.as_ref().unwrap();
    let face = |zsign: f64| {
        (1..=shape.face_count())
            .find(|&i| shape.face_info(i).map(|f| f.normal[2] * zsign > 0.99).unwrap_or(false))
            .map(|i| format!("Face{i}"))
            .unwrap()
    };
    let (top, bottom) = (face(1.0), face(-1.0));
    let at = doc.topo_ref(pad, &top).unwrap();
    let mt = doc.add_mate_connector(MateConnector::new(Anchor::Topo { at, infer: Infer::Auto }));
    let at = doc.topo_ref(pad, &bottom).unwrap();
    let mb = doc.add_mate_connector(MateConnector::new(Anchor::Topo { at, infer: Infer::Auto }));
    doc.recompute();
    (doc, pad, mt, mb, top)
}

fn near(a: [f64; 3], b: [f64; 3], tol: f64) -> bool {
    (0..3).all(|k| (a[k] - b[k]).abs() < tol)
}

#[test]
fn mate_connectors_stack_blocks() {
    let (doc, _, mt, mb, _) = block();
    let mut ws = Workspace::new();
    let p = ws.add_part("Block", doc);
    let id = ws.add_assembly("Stack");
    let asm = ws.assembly_mut(id).unwrap();
    let a = asm.add_part_instance("A", p, Frame::IDENTITY);
    asm.set_grounded(a, true);
    let b = asm.add_part_instance("B", p, Frame::from_ypr([3.0, 1.0, 9.0], 10.0, 4.0, 3.0));
    asm.add_joint(Joint::new(
        JointKind::Fastened,
        JointRef::PartConnector { instance: a, connector: mt },
        JointRef::PartConnector { instance: b, connector: mb },
    ));
    let rep = ws.solve_assembly(id, &SolveOptions::default()).unwrap();
    assert!(rep.ok(), "{rep:?}");
    assert_eq!(rep.dof, 0);
    // Auto sense picks the face-to-face pose nearest the start: B sits on A, upright.
    let pl = ws.assembly(id).unwrap().instance_by_name("B").unwrap().placement;
    assert!(near(pl.pos, [0.0, 0.0, 5.0], 1e-7), "{pl:?}");
    assert!(pl.angle() < 1e-7, "{pl:?}");

    // The JSON round trip keeps everything (and the reloaded workspace still solves).
    let s = ws.to_json();
    let mut back = Workspace::from_json(&s).unwrap();
    back.recompute();
    let rep = back.solve_assembly(id, &SolveOptions::default()).unwrap();
    assert!(rep.ok() && rep.moved.is_empty(), "{rep:?}");
}

#[test]
fn freecad_picks_and_onshape_inference_on_a_body() {
    let (doc, _, _, _, top) = block();
    let mut ws = Workspace::new();
    let p = ws.add_part("Block", doc);
    let id = ws.add_assembly("Hinge");
    let asm = ws.assembly_mut(id).unwrap();
    let a = asm.add_part_instance("A", p, Frame::IDENTITY);
    asm.set_grounded(a, true);
    let b = asm.add_part_instance("B", p, Frame::from_ypr([1.0, 2.0, 7.0], 30.0, 0.0, 0.0));
    // FreeCAD-style pick on A (top face centre), Onshape inference on B (bottom face via
    // a vertex-free inference): a revolute about the vertical axis through A's top centre.
    let jid = asm.add_joint(Joint::new(
        JointKind::Revolute,
        JointRef::Geometry { instance: a, element: top.clone(), implicit: ImplicitPoint::Center },
        JointRef::Geometry { instance: b, element: top.clone(), implicit: ImplicitPoint::Infer(Infer::Auto) },
    ));
    let (fa, _) = ws.assembly(id).unwrap().joint_frames(&ws, jid).unwrap();
    assert!(near(fa.pos, [10.0, 5.0, 5.0], 1e-7), "{fa:?}");
    let rep = ws.solve_assembly(id, &SolveOptions::default()).unwrap();
    assert!(rep.ok(), "{rep:?}");
    assert_eq!(rep.dof, 1);
    let asm = ws.assembly(id).unwrap();
    let (fa, fb) = asm.joint_frames(&ws, jid).unwrap();
    assert!(near(fa.pos, fb.pos, 1e-7));
    // B's own top face centre now coincides with A's: B's body sits inside A's footprint,
    // rotated about the vertical axis (the free DOF), never tilted.
    let pl = asm.instance(b).unwrap().placement;
    assert!(pl.z_axis()[2] > 1.0 - 1e-9, "{pl:?}");
}

/// A FreeCAD assembly of App::Links to PartDesign Bodies (tools/parity/assembly/make_pd_hinge.py)
/// imports the bodies' geometry, not placeholders, and solves to FreeCAD's 1 DOF.
#[test]
fn fcstd_assembly_imports_partdesign_bodies() {
    let p = concat!(env!("CARGO_MANIFEST_DIR"), "/../../tools/parity/assembly/pd_hinge.FCStd");
    let (mut ws, notes) = fc_assembly::fcstd::import(p).expect("import");
    assert!(notes.iter().all(|n| !n.contains("placeholder")), "{notes:?}");
    let names: Vec<&str> = ws.parts.iter().map(|p| p.name.as_str()).collect();
    assert_eq!(names, ["Base", "Lever"]);
    ws.recompute();
    assert!(ws.parts.iter().all(|p| p.doc.result().is_some()));
    let id = ws.assemblies[0].id;
    let rep = ws.solve_assembly(id, &fc_assembly::SolveOptions::default()).unwrap();
    assert!(rep.ok(), "{rep:?}");
    assert_eq!(rep.dof, 1);
}
