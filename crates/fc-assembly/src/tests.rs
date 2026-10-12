use super::*;
use crate::frame::occt_frame;
use fc_part::placement::{dot, norm, sub};

const TOL: f64 = 1e-7;

fn box_res(parts: &[(PartId, [f64; 3])]) -> MapResolver {
    let mut r = MapResolver::new();
    for (p, d) in parts {
        r.add_box(*p, d[0], d[1], d[2]);
    }
    r
}

/// Assembly with a grounded base and one free part, both boxes of part `1`/`2`.
fn two_parts(free_at: Frame) -> (Assembly, InstanceId, InstanceId) {
    let mut a = Assembly::new("A");
    let g = a.add_part_instance("Base", 1, Frame::IDENTITY);
    a.set_grounded(g, true);
    let f = a.add_part_instance("Part", 2, free_at);
    (a, g, f)
}

fn world(a: &Assembly, inst: InstanceId, local: Frame) -> Frame {
    a.instance(inst).unwrap().placement.mul(&local)
}

fn frame_joint(kind: JointKind, g: InstanceId, la: Frame, f: InstanceId, lb: Frame) -> Joint {
    Joint::new(kind, JointRef::frame(g, la), JointRef::frame(f, lb))
}

fn off_pose() -> Frame {
    Frame::from_ypr([3.0, -2.0, 4.0], 12.0, -7.0, 5.0)
}

fn dof_of(kind: JointKind) -> SolveReport {
    let (mut a, g, f) = two_parts(off_pose());
    let la = Frame::from_ypr([10.0, 5.0, 2.0], 0.0, 0.0, 0.0);
    let lb = Frame::from_ypr([1.0, 2.0, 0.0], 0.0, 0.0, 0.0);
    a.add_joint(frame_joint(kind, g, la, f, lb));
    let rep = a.solve(&NoGeometry, &SolveOptions::default());
    assert!(rep.ok(), "{kind:?}: {rep:?}");
    rep
}

#[test]
fn dof_counts() {
    // A free part alone has 6 DOF.
    let (mut a, _, _) = two_parts(off_pose());
    let rep = a.solve(&NoGeometry, &SolveOptions::default());
    assert_eq!(rep.dof, 6);
    assert_eq!(rep.status, SolveStatus::Solved);

    let cases = [
        (JointKind::Fastened, 0),
        (JointKind::Revolute, 1),
        (JointKind::Slider, 1),
        (JointKind::Cylindrical, 2),
        (JointKind::PinSlot, 2),
        (JointKind::Ball, 3),
        (JointKind::Planar, 3),
        (JointKind::Parallel, 4),
        (JointKind::Perpendicular, 5),
        (JointKind::Angle { angle: 30.0 }, 5),
        (JointKind::Angle { angle: 0.0 }, 4),
        (JointKind::Distance { distance: 5.0, mode: DistanceMode::PointPoint }, 5),
        (JointKind::Distance { distance: 0.0, mode: DistanceMode::PointPoint }, 3),
        (JointKind::Distance { distance: 5.0, mode: DistanceMode::PlanePlane }, 3),
        (JointKind::Distance { distance: 5.0, mode: DistanceMode::PointPlane }, 5),
        (JointKind::Distance { distance: 5.0, mode: DistanceMode::LinePlane }, 4),
        (JointKind::Distance { distance: 5.0, mode: DistanceMode::PointLine }, 5),
        (JointKind::Distance { distance: 5.0, mode: DistanceMode::LineLine }, 3),
    ];
    for (k, dof) in cases {
        let rep = dof_of(k);
        assert_eq!(rep.dof, dof, "{k:?}");
        assert_eq!(rep.equations, 6 - dof, "{k:?}");
        assert_eq!(rep.status, SolveStatus::Solved, "{k:?}");
    }
}

#[test]
fn joint_semantics_hold_after_solve() {
    let la = Frame::from_ypr([10.0, 5.0, 2.0], 30.0, 0.0, 0.0);
    let lb = Frame::from_ypr([1.0, 2.0, 0.0], 0.0, 20.0, 0.0);
    let check = |kind: JointKind, f: &dyn Fn(&Frame, &Frame)| {
        let (mut a, g, p) = two_parts(off_pose());
        a.add_joint(frame_joint(kind, g, la, p, lb).with_sense(Sense::Aligned));
        let rep = a.solve(&NoGeometry, &SolveOptions::default());
        assert!(rep.ok(), "{kind:?} {rep:?}");
        let (i, j) = (world(&a, g, la), world(&a, p, lb));
        f(&i, &j);
    };
    check(JointKind::Fastened, &|i, j| assert!(i.is_same(j, TOL)));
    check(JointKind::Revolute, &|i, j| {
        assert!(norm(sub(i.pos, j.pos)) < TOL);
        assert!(dot(i.z_axis(), j.z_axis()) > 1.0 - TOL);
    });
    check(JointKind::Slider, &|i, j| {
        let r = sub(j.pos, i.pos);
        assert!(dot(r, i.x_axis()).abs() < TOL && dot(r, i.y_axis()).abs() < TOL);
        assert!(i.inverse().mul(j).angle() < TOL);
    });
    check(JointKind::Cylindrical, &|i, j| {
        let r = sub(j.pos, i.pos);
        assert!(dot(r, i.x_axis()).abs() < TOL && dot(r, i.y_axis()).abs() < TOL);
        assert!(dot(i.z_axis(), j.z_axis()) > 1.0 - TOL);
    });
    check(JointKind::Planar, &|i, j| {
        assert!(dot(sub(j.pos, i.pos), i.z_axis()).abs() < TOL);
        assert!(dot(i.z_axis(), j.z_axis()) > 1.0 - TOL);
    });
    check(JointKind::PinSlot, &|i, j| {
        let r = sub(j.pos, i.pos);
        assert!(dot(r, i.y_axis()).abs() < TOL && dot(r, i.z_axis()).abs() < TOL);
        assert!(dot(i.z_axis(), j.z_axis()) > 1.0 - TOL);
    });
    check(JointKind::Ball, &|i, j| assert!(norm(sub(i.pos, j.pos)) < TOL));
    check(JointKind::Perpendicular, &|i, j| assert!(dot(i.z_axis(), j.z_axis()).abs() < TOL));
    check(JointKind::Angle { angle: 40.0 }, &|i, j| {
        assert!((dot(i.z_axis(), j.z_axis()) - 40f64.to_radians().cos()).abs() < TOL)
    });
    check(JointKind::Distance { distance: 7.0, mode: DistanceMode::PointPoint }, &|i, j| {
        assert!((norm(sub(i.pos, j.pos)) - 7.0).abs() < TOL)
    });
    check(JointKind::Distance { distance: 7.0, mode: DistanceMode::PlanePlane }, &|i, j| {
        assert!((dot(sub(j.pos, i.pos), i.z_axis()) - 7.0).abs() < TOL);
        assert!(dot(i.z_axis(), j.z_axis()) > 1.0 - TOL);
    });
}

#[test]
fn sense_flip_and_auto() {
    let la = Frame::translation([0.0, 0.0, 10.0]);
    let lb = Frame::IDENTITY;
    // Start upside down: Auto keeps the antiparallel root (Ondsel), Aligned turns it over.
    let start = Frame::from_ypr([1.0, 1.0, 12.0], 0.0, 0.0, 175.0);
    for (sense, expect) in [(Sense::Auto, -1.0), (Sense::Aligned, 1.0), (Sense::Flipped, -1.0)] {
        let (mut a, g, p) = two_parts(start);
        a.add_joint(frame_joint(JointKind::Revolute, g, la, p, lb).with_sense(sense));
        let rep = a.solve(&NoGeometry, &SolveOptions::default());
        assert!(rep.ok(), "{sense:?} {rep:?}");
        let (i, j) = (world(&a, g, la), world(&a, p, lb));
        assert!((dot(i.z_axis(), j.z_axis()) - expect).abs() < 1e-9, "{sense:?}");
    }
}

#[test]
fn offsets_like_freecad() {
    // Fixed joint with Offset2 = 2 mm along Z and 30° about Z: J ends 2 mm *below* I
    // (Offset applies to J's JCS, the part is moved so the offset frame meets I).
    let (mut a, g, p) = two_parts(off_pose());
    let la = Frame::translation([0.0, 0.0, 10.0]);
    a.add_joint(frame_joint(JointKind::Fastened, g, la, p, Frame::IDENTITY).with_offset(2.0).with_rotation(30.0));
    let rep = a.solve(&NoGeometry, &SolveOptions::default());
    assert!(rep.ok());
    let pl = a.instance(p).unwrap().placement;
    assert!(norm(sub(pl.pos, [0.0, 0.0, 8.0])) < TOL, "{:?}", pl.pos);
    let expect = Frame::rotation([0.0, 0.0, 1.0], (-30f64).to_radians());
    assert!(pl.inverse().mul(&Frame { pos: pl.pos, rot: expect.rot }).angle() < TOL);
}

#[test]
fn redundancy_and_conflict() {
    // Two identical fixed joints: redundant, still solved, 0 DOF.
    let (mut a, g, p) = two_parts(off_pose());
    let la = Frame::translation([5.0, 0.0, 0.0]);
    a.add_joint(frame_joint(JointKind::Fastened, g, la, p, Frame::IDENTITY));
    let j2 = a.add_joint(frame_joint(JointKind::Fastened, g, la, p, Frame::IDENTITY));
    let rep = a.solve(&NoGeometry, &SolveOptions::default());
    assert_eq!(rep.status, SolveStatus::Redundant, "{rep:?}");
    assert_eq!(rep.dof, 0);
    assert_eq!(rep.redundant_joints, vec![j2]);

    // Revolute + a contradicting fixed joint (different origin): conflicting; the part stays.
    let (mut a, g, p) = two_parts(off_pose());
    let j1 = a.add_joint(frame_joint(JointKind::Revolute, g, la, p, Frame::IDENTITY));
    let j2 = a.add_joint(frame_joint(JointKind::Fastened, g, Frame::translation([9.0, 0.0, 0.0]), p, Frame::IDENTITY));
    let rep = a.solve(&NoGeometry, &SolveOptions::default());
    assert_eq!(rep.status, SolveStatus::Conflicting, "{rep:?}");
    assert!(rep.conflicting_joints.contains(&j1) && rep.conflicting_joints.contains(&j2));
    assert!(a.instance(p).unwrap().placement.is_same(&off_pose(), 1e-12));

    // Revolute + slider on the same axis: redundant orientation equations, 0 DOF.
    let (mut a, g, p) = two_parts(off_pose());
    a.add_joint(frame_joint(JointKind::Revolute, g, la, p, Frame::IDENTITY));
    a.add_joint(frame_joint(JointKind::Slider, g, la, p, Frame::IDENTITY));
    let rep = a.solve(&NoGeometry, &SolveOptions::default());
    assert_eq!(rep.status, SolveStatus::Redundant, "{rep:?}");
    assert_eq!(rep.dof, 0);
}

/// Planar 4-bar: ground (60), crank (20), coupler (50), rocker (40), revolutes at the
/// link ends, all axes = world Z. Returns the assembly and the instance ids.
fn four_bar(crank_deg: f64) -> (Assembly, [InstanceId; 4]) {
    let mut a = Assembly::new("4bar");
    let gr = a.add_part_instance("Ground", 1, Frame::IDENTITY);
    a.set_grounded(gr, true);
    let cr = a.add_part_instance("Crank", 2, Frame::from_ypr([0.0; 3], crank_deg, 0.0, 0.0));
    let co = a.add_part_instance("Coupler", 3, Frame::from_ypr([10.0, 17.32, 0.0], 27.0, 0.0, 0.0));
    let ro = a.add_part_instance("Rocker", 4, Frame::from_ypr([60.0, 0.0, 0.0], 100.0, 0.0, 0.0));
    let t = |x: f64| Frame::translation([x, 0.0, 0.0]);
    a.add_joint(Joint::new(JointKind::Revolute, JointRef::frame(gr, t(0.0)), JointRef::frame(cr, t(0.0))));
    a.add_joint(Joint::new(JointKind::Revolute, JointRef::frame(cr, t(20.0)), JointRef::frame(co, t(0.0))));
    a.add_joint(Joint::new(JointKind::Revolute, JointRef::frame(co, t(50.0)), JointRef::frame(ro, t(40.0))));
    a.add_joint(Joint::new(JointKind::Revolute, JointRef::frame(ro, t(0.0)), JointRef::frame(gr, t(60.0))));
    (a, [gr, cr, co, ro])
}

fn check_four_bar(a: &Assembly, ids: [InstanceId; 4]) {
    let p = |i: InstanceId, x: f64| a.instance(i).unwrap().placement.transform_point([x, 0.0, 0.0]);
    let [gr, cr, co, ro] = ids;
    assert!(norm(sub(p(gr, 0.0), p(cr, 0.0))) < TOL);
    assert!(norm(sub(p(cr, 20.0), p(co, 0.0))) < TOL);
    assert!(norm(sub(p(co, 50.0), p(ro, 40.0))) < TOL);
    assert!(norm(sub(p(ro, 0.0), p(gr, 60.0))) < TOL);
    for i in ids {
        assert!(dot(a.instance(i).unwrap().placement.z_axis(), [0.0, 0.0, 1.0]) > 1.0 - TOL);
    }
}

#[test]
fn four_bar_linkage() {
    let (mut a, ids) = four_bar(60.0);
    let rep = a.solve(&NoGeometry, &SolveOptions::default());
    assert_eq!(rep.status, SolveStatus::Redundant, "{rep:?}");
    // 18 unknowns, 20 equations of rank 17: one DOF and 3 redundant equations.
    assert_eq!(rep.dof, 1);
    assert_eq!(rep.equations, 20);
    assert_eq!(rep.rank, 17);
    check_four_bar(&a, ids);
    // Minimal movement: the crank barely turns from 60°.
    let crank = a.instance(ids[1]).unwrap().placement;
    let ang = crank.x_axis()[1].atan2(crank.x_axis()[0]).to_degrees();
    assert!((ang - 60.0).abs() < 3.0, "crank {ang}");
}

#[test]
fn drag_respects_joints() {
    // Drag the crank of the 4-bar: it turns, the loop stays closed.
    let (mut a, ids) = four_bar(60.0);
    a.solve(&NoGeometry, &SolveOptions::default());
    let target_tip = [20.0 * 80f64.to_radians().cos(), 20.0 * 80f64.to_radians().sin(), 0.0];
    let rep = a
        .drag(&NoGeometry, ids[1], DragTarget::Point { local: [20.0, 0.0, 0.0], world: target_tip }, &SolveOptions::default())
        .unwrap();
    assert!(rep.ok(), "{rep:?}");
    check_four_bar(&a, ids);
    let crank = a.instance(ids[1]).unwrap().placement;
    let ang = crank.x_axis()[1].atan2(crank.x_axis()[0]).to_degrees();
    assert!((ang - 80.0).abs() < 1e-3, "crank {ang}");

    // A hinge: dragging the plate toward a point off its circle ends on the circle, in
    // the plane of rotation, nearest to the cursor.
    let (mut a, g, p) = two_parts(Frame::IDENTITY);
    a.add_joint(frame_joint(JointKind::Revolute, g, Frame::IDENTITY, p, Frame::IDENTITY));
    let rep = a
        .drag(&NoGeometry, p, DragTarget::Point { local: [10.0, 0.0, 0.0], world: [0.0, 30.0, 5.0] }, &SolveOptions::default())
        .unwrap();
    assert!(rep.ok());
    let tip = a.instance(p).unwrap().placement.transform_point([10.0, 0.0, 0.0]);
    assert!(norm(sub(tip, [0.0, 10.0, 0.0])) < 1e-6, "{tip:?}");

    // Slider: a frame target projects onto the slide axis.
    let (mut a, g, p) = two_parts(Frame::IDENTITY);
    a.add_joint(frame_joint(JointKind::Slider, g, Frame::IDENTITY, p, Frame::IDENTITY));
    a.drag(&NoGeometry, p, DragTarget::Frame(Frame::from_ypr([4.0, 5.0, 6.0], 20.0, 0.0, 0.0)), &SolveOptions::default()).unwrap();
    let pl = a.instance(p).unwrap().placement;
    assert!(pl.is_same(&Frame::translation([0.0, 0.0, 6.0]), 1e-6), "{pl:?}");

    // Grounded parts do not move; free parts follow exactly.
    let (mut a, g, p) = two_parts(Frame::IDENTITY);
    let rep = a.drag(&NoGeometry, g, DragTarget::Frame(off_pose()), &SolveOptions::default()).unwrap();
    assert!(rep.moved.is_empty());
    a.drag(&NoGeometry, p, DragTarget::Frame(off_pose()), &SolveOptions::default()).unwrap();
    assert!(a.instance(p).unwrap().placement.is_same(&off_pose(), 1e-12));
}

#[test]
fn limits_are_enforced() {
    // Slider with max length 5: a start at z = 8 is pulled back to 5; z = 3 is untouched.
    for (z, expect, limited) in [(8.0, 5.0, true), (3.0, 3.0, false)] {
        let (mut a, g, p) = two_parts(Frame::translation([0.5, 0.2, z]));
        let j = a.add_joint(
            frame_joint(JointKind::Slider, g, Frame::IDENTITY, p, Frame::IDENTITY)
                .with_limits(Limits { length_max: Some(5.0), ..Default::default() }),
        );
        let rep = a.solve(&NoGeometry, &SolveOptions::default());
        assert!(rep.ok(), "{rep:?}");
        assert!((a.instance(p).unwrap().placement.pos[2] - expect).abs() < TOL);
        assert_eq!(rep.limited_joints.contains(&j), limited);
    }
    // Revolute with an angle range of [-10°, 45°]: a start at 70° stops at 45°.
    let (mut a, g, p) = two_parts(Frame::from_ypr([0.0; 3], 70.0, 0.0, 0.0));
    a.add_joint(
        frame_joint(JointKind::Revolute, g, Frame::IDENTITY, p, Frame::IDENTITY)
            .with_limits(Limits { angle_min: Some(-10.0), angle_max: Some(45.0), ..Default::default() }),
    );
    a.solve(&NoGeometry, &SolveOptions::default());
    let x = a.instance(p).unwrap().placement.x_axis();
    assert!((x[1].atan2(x[0]).to_degrees() - 45.0).abs() < 1e-6);
}

#[test]
fn coupling_joints_follow_drag() {
    // Gears: two wheels on revolutes; turning one turns the other by -r1/r2.
    let mut a = Assembly::new("gears");
    let g = a.add_part_instance("Frame", 1, Frame::IDENTITY);
    a.set_grounded(g, true);
    let w1 = a.add_part_instance("W1", 2, Frame::IDENTITY);
    let w2 = a.add_part_instance("W2", 3, Frame::translation([30.0, 0.0, 0.0]));
    a.add_joint(frame_joint(JointKind::Revolute, g, Frame::IDENTITY, w1, Frame::IDENTITY));
    a.add_joint(frame_joint(JointKind::Revolute, g, Frame::translation([30.0, 0.0, 0.0]), w2, Frame::IDENTITY));
    a.add_joint(frame_joint(JointKind::Gears { radius1: 10.0, radius2: 20.0 }, w1, Frame::IDENTITY, w2, Frame::IDENTITY));
    let rep = a.solve(&NoGeometry, &SolveOptions::default());
    assert_eq!(rep.dof, 1, "{rep:?}");
    let target = Frame::from_ypr([0.0; 3], 40.0, 0.0, 0.0);
    a.drag(&NoGeometry, w1, DragTarget::Frame(target), &SolveOptions::default()).unwrap();
    let ang = |i: InstanceId| {
        let x = a.instance(i).unwrap().placement.x_axis();
        x[1].atan2(x[0]).to_degrees()
    };
    assert!((ang(w1) - 40.0).abs() < 1e-5, "{}", ang(w1));
    assert!((ang(w2) + 20.0).abs() < 1e-5, "{}", ang(w2));

    // Screw: rotation drives translation (pitch 2 mm/turn).
    let mut a = Assembly::new("screw");
    let g = a.add_part_instance("Nut", 1, Frame::IDENTITY);
    a.set_grounded(g, true);
    let s = a.add_part_instance("Screw", 2, Frame::IDENTITY);
    a.add_joint(frame_joint(JointKind::Cylindrical, g, Frame::IDENTITY, s, Frame::IDENTITY));
    a.add_joint(frame_joint(JointKind::Screw { pitch: 2.0 }, g, Frame::IDENTITY, s, Frame::IDENTITY));
    assert_eq!(a.solve(&NoGeometry, &SolveOptions::default()).dof, 1);
    a.drag(&NoGeometry, s, DragTarget::Frame(Frame::from_ypr([0.0; 3], 90.0, 0.0, 0.0)), &SolveOptions::default()).unwrap();
    let pl = a.instance(s).unwrap().placement;
    let x = pl.x_axis();
    let turn = x[1].atan2(x[0]);
    assert!(turn > 0.1, "screw did not turn: {pl:?}");
    assert!((pl.pos[2] - 2.0 * turn / (2.0 * std::f64::consts::PI)).abs() < 1e-6, "{pl:?}");
}

#[test]
fn geometry_refs_like_freecad() {
    // FreeCAD's TestCore.test_find_placement expectations (box 2 x 3 x 7).
    let (l, w, h) = (2.0, 3.0, 7.0);
    let mut r = MapResolver::new();
    r.add_box(1, l, w, h);
    let mut a = Assembly::new("A");
    let b = a.add_part_instance("Box", 1, Frame::IDENTITY);
    let side = |rf: JointRef, distance: bool| solver::resolve_ref(&a, &r, &rf, distance).unwrap().local;
    let f = side(JointRef::picked(b, "Face6", "Vertex7"), false);
    assert!(f.is_same(&Frame::translation([l, w, h]), 1e-9));
    let f = side(JointRef::picked(b, "Edge8", "Vertex8"), false);
    assert!(f.is_same(&Frame::from_ypr([l, w, 0.0], 0.0, -90.0, 270.0), 1e-9), "{f:?}");
    let f = side(JointRef::picked(b, "Vertex3", "Vertex3"), false);
    assert!(f.is_same(&Frame::translation([0.0, w, h]), 1e-9));
    let f = side(JointRef::picked(b, "Face2", "Face2"), false);
    assert!(f.is_same(&Frame::from_ypr([l, w / 2.0, h / 2.0], 0.0, -90.0, 180.0), 1e-9), "{f:?}");
    // OCCT's automatic X for a few directions.
    let z = occt_frame([0.0; 3], [0.0, -1.0, 0.0]);
    assert!(norm(sub(z.x_axis(), [0.0, 0.0, -1.0])) < 1e-12);
}

#[test]
fn presolve_matches_jcs() {
    let r = box_res(&[(1, [40.0, 20.0, 2.0]), (2, [40.0, 20.0, 2.0])]);
    let (mut a, g, p) = two_parts(Frame::from_ypr([5.0, 30.0, 4.0], 0.0, 0.0, 20.0));
    let j = a.add_joint(Joint::new(JointKind::Revolute, JointRef::picked(g, "Edge12", "Vertex7"), JointRef::picked(p, "Edge10", "Vertex5")));
    assert!(a.presolve_joint(&r, j).unwrap());
    let (fi, fj) = a.joint_frames(&r, j).unwrap();
    assert!(fi.is_same(&fj, 1e-9));
    let rep = a.solve(&r, &SolveOptions::default());
    assert_eq!(rep.dof, 1);
    assert!(rep.moved.is_empty());
}

#[test]
fn serialization_roundtrip() {
    let mut ws = Workspace::new();
    let p1 = ws.add_part("Plate", fc_part::Document::new());
    let id = ws.add_assembly("Main");
    let asm = ws.assembly_mut(id).unwrap();
    let a = asm.add_part_instance("Plate", p1, Frame::IDENTITY);
    asm.set_grounded(a, true);
    let b = asm.add_part_instance("Plate", p1, off_pose());
    let c = asm.add_connector("MC", b, Frame::translation([1.0, 2.0, 3.0]));
    asm.add_joint(Joint::new(JointKind::Revolute, JointRef::picked(a, "Edge12", "Vertex7"), JointRef::AssemblyConnector(c)).with_offset(2.0));
    asm.add_joint(
        Joint::new(JointKind::Distance { distance: 3.0, mode: DistanceMode::Auto }, JointRef::Origin { instance: a }, JointRef::PartConnector { instance: b, connector: 4 })
            .with_limits(Limits { length_min: Some(1.0), ..Default::default() }),
    );
    asm.exploded_views.push(ExplodedView { name: "E".into(), steps: vec![ExplodeStep::translate(vec![b], [0.0, 0.0, 1.0], 10.0), ExplodeStep::radial(vec![a, b], 5.0)] });
    let s = ws.to_json();
    let back = Workspace::from_json(&s).unwrap();
    assert_eq!(back.to_json(), s);
    assert_eq!(back.assemblies[0], ws.assemblies[0]);
    assert_eq!(back.assemblies[0].instances[1].name, "Plate001");
    assert!(back.next_id > p1);

    // FreeCAD's "Fixed" spelling is accepted.
    let k: JointKind = serde_json::from_str("\"Fixed\"").unwrap();
    assert_eq!(k, JointKind::Fastened);

    // An old single-body document loads as a one-part workspace.
    let mut doc = fc_part::Document::new();
    doc.parameters.insert("L".into(), "10".into());
    let old = Workspace::from_json(&doc.to_json()).unwrap();
    assert_eq!(old.parts.len(), 1);
    assert_eq!(old.parts[0].doc.parameters["L"], "10");
    assert!(old.assemblies.is_empty());
}

#[test]
fn many_parts_are_fast() {
    // A chain of 100 links on revolutes (one island, 600 unknowns) plus 100 independent
    // fastened parts.
    let mut a = Assembly::new("chain");
    let g = a.add_part_instance("Base", 1, Frame::IDENTITY);
    a.set_grounded(g, true);
    let mut prev = g;
    for k in 0..100 {
        let i = a.add_part_instance("Link", 2, Frame::from_ypr([10.0 * k as f64 + 1.0, 0.5, 0.3], 3.0, 2.0, 1.0));
        a.add_joint(Joint::new(JointKind::Revolute, JointRef::frame(prev, Frame::translation([if k == 0 { 0.0 } else { 10.0 }, 0.0, 0.0])), JointRef::frame(i, Frame::IDENTITY)));
        prev = i;
    }
    for k in 0..100 {
        let i = a.add_part_instance("Bolt", 3, Frame::from_ypr([k as f64, 50.0, 1.0], 10.0, 0.0, 0.0));
        a.add_joint(Joint::new(JointKind::Fastened, JointRef::frame(g, Frame::translation([k as f64, 50.0, 0.0])), JointRef::frame(i, Frame::IDENTITY)));
    }
    let t = std::time::Instant::now();
    let rep = a.solve(&NoGeometry, &SolveOptions::default());
    let el = t.elapsed();
    assert!(rep.ok(), "{rep:?}");
    assert_eq!(rep.dof, 100);
    eprintln!("200 parts: {el:?}, {} iterations", rep.iterations);
    assert!(el.as_secs_f64() < if cfg!(debug_assertions) { 30.0 } else { 2.0 });

    // Drag the tip of the 100-link chain.
    let tip = a.instance(prev).unwrap().placement.transform_point([10.0, 0.0, 0.0]);
    let t = std::time::Instant::now();
    let rep = a
        .drag(&NoGeometry, prev, DragTarget::Point { local: [10.0, 0.0, 0.0], world: [tip[0] - 5.0, tip[1] + 20.0, tip[2]] }, &SolveOptions::default())
        .unwrap();
    let el = t.elapsed();
    assert!(rep.ok(), "{rep:?}");
    eprintln!("drag 100-link chain: {el:?}");
    assert!(el.as_secs_f64() < if cfg!(debug_assertions) { 120.0 } else { 3.0 });
}
