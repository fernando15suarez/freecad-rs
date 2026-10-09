use super::*;
use std::f64::consts::PI;
use ConstraintKind::*;

fn v(x: f64, y: f64) -> Vec2 {
    Vec2::new(x, y)
}
fn s(g: GeoId) -> PointRef {
    PointRef::start(g)
}
fn e(g: GeoId) -> PointRef {
    PointRef::end(g)
}
fn m(g: GeoId) -> PointRef {
    PointRef::mid(g)
}
fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-7
}
fn vclose(a: Vec2, b: Vec2) -> bool {
    a.dist(b) < 1e-7
}

/// Max residual of all driving constraints at the current geometry.
fn residual(sk: &Sketch) -> f64 {
    let lay = equations::Layout::new(sk);
    let x = lay.pack(sk);
    let sys = equations::System::new(sk, &lay, &x, |c| c.driving);
    sys.residual(&x).iter().fold(0.0f64, |a, b| a.max(b.abs()))
}

/// A roughly drawn rectangle (corners slightly off) with 4 coincidents + H/V.
/// Returns line ids [bottom, right, top, left].
fn rough_rect(sk: &mut Sketch) -> [GeoId; 4] {
    let l0 = sk.add_geometry(Geometry::line(v(0.3, -0.2), v(9.1, 0.4)));
    let l1 = sk.add_geometry(Geometry::line(v(9.4, 0.1), v(10.2, 6.3)));
    let l2 = sk.add_geometry(Geometry::line(v(9.7, 5.6), v(-0.4, 4.9)));
    let l3 = sk.add_geometry(Geometry::line(v(0.2, 5.3), v(0.5, 0.6)));
    sk.add_constraint(Coincident(e(l0), s(l1)));
    sk.add_constraint(Coincident(e(l1), s(l2)));
    sk.add_constraint(Coincident(e(l2), s(l3)));
    sk.add_constraint(Coincident(e(l3), s(l0)));
    sk.add_constraint(Horizontal(l0));
    sk.add_constraint(Horizontal(l2));
    sk.add_constraint(Vertical(l1));
    sk.add_constraint(Vertical(l3));
    [l0, l1, l2, l3]
}

fn full_rect() -> (Sketch, [GeoId; 4]) {
    let mut sk = Sketch::new();
    let l = rough_rect(&mut sk);
    sk.add_constraint(DistanceLine(l[0], 10.0));
    sk.add_constraint(DistanceLine(l[1], 5.0));
    sk.add_constraint(Coincident(s(l[0]), PointRef::root()));
    (sk, l)
}

#[test]
fn geometry_dofs() {
    let mut sk = Sketch::new();
    assert_eq!(sk.diagnose().dof, 0);
    sk.add_geometry(Geometry::point(v(1.0, 2.0)));
    assert_eq!(sk.diagnose().dof, 2);
    let mut sk = Sketch::new();
    sk.add_geometry(Geometry::line(v(0.0, 0.0), v(1.0, 2.0)));
    assert_eq!(sk.diagnose().dof, 4);
    let mut sk = Sketch::new();
    sk.add_geometry(Geometry::circle(v(1.0, 1.0), 3.0));
    assert_eq!(sk.diagnose().dof, 3);
    let mut sk = Sketch::new();
    sk.add_geometry(Geometry::arc(v(1.0, 1.0), 3.0, 0.1, 2.0));
    let r = sk.diagnose();
    assert_eq!(r.dof, 5);
    assert_eq!(r.status, SolveStatus::Solved);
}

#[test]
fn rectangle_fully_constrained() {
    let (mut sk, l) = full_rect();
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Solved, "{r:?}");
    assert_eq!(r.dof, 0);
    assert!(residual(&sk) < 1e-9);
    assert!(vclose(sk.point(s(l[0])), v(0.0, 0.0)));
    assert!(vclose(sk.point(e(l[0])), v(10.0, 0.0)));
    assert!(vclose(sk.point(e(l[1])), v(10.0, 5.0)));
    assert!(vclose(sk.point(s(l[3])), v(0.0, 5.0)));
    assert!(sk.fully_constrained_geometry().iter().all(|&b| b));
}

#[test]
fn rectangle_missing_distance_has_one_dof() {
    let (mut sk, l) = full_rect();
    sk.remove_constraint(8); // width
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Solved);
    assert_eq!(r.dof, 1);
    // left side is pinned, bottom/top/right can still move with the width
    let fc = sk.fully_constrained_geometry();
    assert_eq!(fc, vec![false, false, false, true]);
    assert!(close(sk.point(e(l[1])).y, 5.0));
}

#[test]
fn rough_rectangle_without_dimensions_has_four_dof() {
    let mut sk = Sketch::new();
    rough_rect(&mut sk);
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Solved);
    assert_eq!(r.dof, 4);
    assert!(residual(&sk) < 1e-9);
}

#[test]
fn duplicate_horizontal_is_redundant() {
    let (mut sk, l) = full_rect();
    let dup = sk.add_constraint(Horizontal(l[0]));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Redundant, "{r:?}");
    assert_eq!(r.redundant, vec![dup]);
    assert!(r.conflicting.is_empty());
    assert_eq!(r.dof, 0);
    assert!(residual(&sk) < 1e-9);
}

#[test]
fn parallel_on_two_horizontals_is_redundant() {
    let (mut sk, l) = full_rect();
    let p = sk.add_constraint(Parallel(l[0], l[2]));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Redundant, "{r:?}");
    assert_eq!(r.redundant, vec![p]);
    assert_eq!(r.dof, 0);
}

#[test]
fn redundant_vertical_on_rectangle_with_perpendiculars() {
    // rectangle closed with H on bottom, perpendiculars at corners, and then an extra
    // vertical on the right side which is implied by H + perpendicular.
    let mut sk = Sketch::new();
    let l0 = sk.add_geometry(Geometry::line(v(0.0, 0.0), v(10.0, 0.3)));
    let l1 = sk.add_geometry(Geometry::line(v(10.0, 0.3), v(10.2, 5.0)));
    let l2 = sk.add_geometry(Geometry::line(v(10.2, 5.0), v(0.1, 5.2)));
    let l3 = sk.add_geometry(Geometry::line(v(0.1, 5.2), v(0.0, 0.0)));
    for (a, b) in [(l0, l1), (l1, l2), (l2, l3), (l3, l0)] {
        sk.add_constraint(Coincident(e(a), s(b)));
    }
    sk.add_constraint(Horizontal(l0));
    sk.add_constraint(Perpendicular(l0, l1));
    sk.add_constraint(Perpendicular(l1, l2));
    sk.add_constraint(Perpendicular(l2, l3));
    let extra = sk.add_constraint(Vertical(l1));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Redundant, "{r:?}");
    assert_eq!(r.redundant, vec![extra]);
    assert_eq!(r.dof, 4);
}

#[test]
fn conflicting_distances() {
    let (mut sk, l) = full_rect();
    let before = sk.geometry.clone();
    let c = sk.add_constraint(Distance(s(l[0]), e(l[0]), 12.0));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Conflicting, "{r:?}");
    assert!(r.conflicting.contains(&8) && r.conflicting.contains(&c), "{r:?}");
    assert!(r.redundant.is_empty());
    assert_eq!(sk.geometry, before, "geometry must not change on conflict");
}

#[test]
fn consistent_duplicate_distance_is_redundant() {
    let (mut sk, l) = full_rect();
    let c = sk.add_constraint(Distance(s(l[0]), e(l[0]), 10.0));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Redundant, "{r:?}");
    assert_eq!(r.redundant, vec![c]);
}

#[test]
fn partially_redundant_coincident() {
    // two points with horizontal + vertical alignment, then a coincident: both rows of
    // the coincident are dependent; FreeCAD drops the constraints with fewer solver
    // equations first (H, V), so they are the redundant ones.
    let mut sk = Sketch::new();
    let a = sk.add_geometry(Geometry::point(v(0.0, 0.0)));
    let b = sk.add_geometry(Geometry::point(v(1.0, 0.5)));
    let h = sk.add_constraint(HorizontalPoints(s(a), s(b)));
    let c = sk.add_constraint(Coincident(s(a), s(b)));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Redundant, "{r:?}");
    assert_eq!(r.redundant, vec![h]);
    assert!(r.partially_redundant.is_empty());
    assert_eq!(r.dof, 2);
    let _ = c;
    // Now with two coincidents sharing a row: Coincident(a,b) + PointOnObject + Coincident
    // where the second coincident is only half-dependent.
    let mut sk = Sketch::new();
    let a = sk.add_geometry(Geometry::point(v(0.0, 0.0)));
    let b = sk.add_geometry(Geometry::point(v(1.0, 0.5)));
    let c1 = sk.add_constraint(LockY(s(a), 0.0));
    let c2 = sk.add_constraint(LockY(s(b), 0.0));
    let c3 = sk.add_constraint(Coincident(s(a), s(b)));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Redundant, "{r:?}");
    // the y-row of the coincident is in the dependency group with both locks; the lock
    // with the highest index is dropped (equal popularity and multiplicity)
    assert_eq!(r.redundant, vec![c2], "{r:?}");
    let _ = (c1, c3);
    assert_eq!(r.dof, 1);
}

#[test]
fn circle_and_arc_dimensions() {
    let mut sk = Sketch::new();
    let c = sk.add_geometry(Geometry::circle(v(1.0, 1.0), 3.0));
    sk.add_constraint(Coincident(m(c), PointRef::root()));
    sk.add_constraint(Diameter(c, 10.0));
    let a = sk.add_geometry(Geometry::arc(v(20.0, 0.0), 2.0, 0.2, 2.0));
    sk.add_constraint(Radius(a, 4.0));
    sk.add_constraint(Angle(a, PI / 2.0));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Solved);
    assert_eq!(r.dof, 3); // arc center + start angle
    match sk.geometry[c].curve {
        Curve::Circle { c, r } => {
            assert!(vclose(c, Vec2::ZERO));
            assert!(close(r, 5.0));
        }
        _ => unreachable!(),
    }
    match sk.geometry[a].curve {
        Curve::Arc { r, start, end, .. } => {
            assert!(close(r, 4.0));
            assert!(close(end - start, PI / 2.0));
        }
        _ => unreachable!(),
    }
}

#[test]
fn tangent_line_arc_edge() {
    let mut sk = Sketch::new();
    let l = sk.add_geometry(Geometry::line(v(0.0, 0.0), v(10.0, 0.5)));
    let a = sk.add_geometry(Geometry::arc(v(5.0, 3.0), 2.5, 0.5, 3.0));
    let t = sk.add_constraint(Tangent(l, a));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Solved, "{r:?}");
    assert_eq!(r.dof, 4 + 5 - 1);
    let (p, q) = (sk.point(s(l)), sk.point(e(l)));
    let (c, rad) = match sk.geometry[a].curve {
        Curve::Arc { c, r, .. } => (c, r),
        _ => unreachable!(),
    };
    let d = ((q - p).cross(c - p) / (q - p).length()).abs();
    assert!(close(d, rad));
    // arc stays on the same side
    assert!((q - p).cross(c - p) > 0.0);
    let _ = t;
}

#[test]
fn tangent_arc_arc_external_and_internal() {
    let mut sk = Sketch::new();
    let a = sk.add_geometry(Geometry::arc(v(0.0, 0.0), 2.0, 0.0, 3.0));
    let b = sk.add_geometry(Geometry::arc(v(5.0, 0.3), 2.5, 1.0, 4.0));
    sk.add_constraint(Tangent(a, b));
    let big = sk.add_geometry(Geometry::circle(v(20.0, 0.0), 5.0));
    let small = sk.add_geometry(Geometry::circle(v(22.5, 0.2), 2.0));
    sk.add_constraint(Tangent(big, small));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Solved, "{r:?}");
    let ctr = |g: GeoId| match sk.geometry[g].curve {
        Curve::Arc { c, r, .. } | Curve::Circle { c, r } => (c, r),
        _ => unreachable!(),
    };
    let ((c1, r1), (c2, r2)) = (ctr(a), ctr(b));
    assert!(close(c1.dist(c2), r1 + r2));
    let ((c3, r3), (c4, r4)) = (ctr(big), ctr(small));
    assert!(close(c3.dist(c4), (r3 - r4).abs()));
}

#[test]
fn equal_radius_and_length() {
    let mut sk = Sketch::new();
    let c1 = sk.add_geometry(Geometry::circle(v(0.0, 0.0), 2.0));
    let a1 = sk.add_geometry(Geometry::arc(v(10.0, 0.0), 3.0, 0.0, 1.0));
    sk.add_constraint(Equal(c1, a1));
    sk.add_constraint(Radius(c1, 4.0));
    let l1 = sk.add_geometry(Geometry::line(v(0.0, 10.0), v(3.0, 10.0)));
    let l2 = sk.add_geometry(Geometry::line(v(0.0, 12.0), v(5.0, 13.0)));
    sk.add_constraint(Equal(l1, l2));
    sk.add_constraint(DistanceLine(l2, 7.0));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Solved, "{r:?}");
    assert!(close(sk.point(s(l1)).dist(sk.point(e(l1))), 7.0));
    match sk.geometry[a1].curve {
        Curve::Arc { r, .. } => assert!(close(r, 4.0)),
        _ => unreachable!(),
    }
}

#[test]
fn symmetric_constraints() {
    let mut sk = Sketch::new();
    let p1 = sk.add_geometry(Geometry::point(v(1.0, 2.0)));
    let p2 = sk.add_geometry(Geometry::point(v(-3.0, 1.0)));
    let c = sk.add_geometry(Geometry::point(v(0.5, 0.5)));
    sk.add_constraint(Symmetric(s(p1), s(p2), s(c)));
    // about a line
    let l = sk.add_geometry(Geometry::line(v(10.0, 0.0), v(12.0, 3.0)));
    let q1 = sk.add_geometry(Geometry::point(v(9.0, 3.0)));
    let q2 = sk.add_geometry(Geometry::point(v(13.0, 1.0)));
    sk.add_constraint(SymmetricLine(s(q1), s(q2), l));
    // about the horizontal axis (Symmetric with Edge third ref)
    let r1 = sk.add_geometry(Geometry::point(v(3.0, 4.0)));
    let r2 = sk.add_geometry(Geometry::point(v(3.5, -3.0)));
    sk.add_constraint(Symmetric(s(r1), s(r2), PointRef::edge(H_AXIS)));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Solved, "{r:?}");
    let (a, b, cc) = (sk.point(s(p1)), sk.point(s(p2)), sk.point(s(c)));
    assert!(vclose((a + b) * 0.5, cc));
    let (la, lb) = (sk.point(s(l)), sk.point(e(l)));
    let (x1, x2) = (sk.point(s(q1)), sk.point(s(q2)));
    let d = lb - la;
    assert!(close(d.dot(x2 - x1), 0.0));
    assert!(close(d.cross((x1 + x2) * 0.5 - la), 0.0));
    let (y1, y2) = (sk.point(s(r1)), sk.point(s(r2)));
    assert!(close(y1.x, y2.x) && close(y1.y, -y2.y));
    assert_eq!(r.dof, 6 + 8 + 4 - 2 - 2 - 2);
}

#[test]
fn angle_constraints() {
    let mut sk = Sketch::new();
    let l1 = sk.add_geometry(Geometry::line(v(0.0, 0.0), v(5.0, 1.0)));
    sk.add_constraint(Angle(l1, PI / 6.0));
    let l2 = sk.add_geometry(Geometry::line(v(0.0, 0.0), v(1.0, 5.0)));
    sk.add_constraint(AngleBetween(l1, l2, PI / 3.0));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Solved, "{r:?}");
    let d1 = sk.point(e(l1)) - sk.point(s(l1));
    let d2 = sk.point(e(l2)) - sk.point(s(l2));
    assert!(close(d1.y.atan2(d1.x), PI / 6.0));
    assert!(close(d2.y.atan2(d2.x), PI / 2.0));
    assert_eq!(r.dof, 6);
}

#[test]
fn perpendicular_and_parallel() {
    let mut sk = Sketch::new();
    let a = sk.add_geometry(Geometry::line(v(0.0, 0.0), v(4.0, 1.0)));
    let b = sk.add_geometry(Geometry::line(v(1.0, 1.0), v(2.0, 4.0)));
    let c = sk.add_geometry(Geometry::line(v(5.0, 0.0), v(9.0, 2.0)));
    sk.add_constraint(Perpendicular(a, b));
    sk.add_constraint(Parallel(a, c));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Solved);
    let d = |g| sk.point(e(g)) - sk.point(s(g));
    assert!(close(d(a).dot(d(b)), 0.0));
    assert!(close(d(a).cross(d(c)), 0.0));
}

#[test]
fn external_geometry_references() {
    let mut sk = Sketch::new();
    let p = sk.add_geometry(Geometry::point(v(1.0, 2.0)));
    sk.add_constraint(PointOnObject(s(p), V_AXIS));
    let q = sk.add_geometry(Geometry::point(v(3.0, 0.2)));
    sk.add_constraint(PointOnObject(s(q), H_AXIS));
    let l = sk.add_geometry(Geometry::line(v(0.1, 0.1), v(2.0, 3.0)));
    sk.add_constraint(Coincident(s(l), PointRef::root()));
    sk.add_constraint(Parallel(l, V_AXIS));
    sk.add_constraint(LockX(s(q), 7.0));
    sk.add_constraint(DistanceY(PointRef::root(), s(p), -2.5));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Solved, "{r:?}");
    assert!(vclose(sk.point(s(p)), v(0.0, -2.5)));
    assert!(vclose(sk.point(s(q)), v(7.0, 0.0)));
    assert!(vclose(sk.point(s(l)), Vec2::ZERO));
    assert!(close(sk.point(e(l)).x, 0.0));
    assert_eq!(r.dof, 1);
}

#[test]
fn coincident_with_root_and_axis_is_conflicting_when_impossible() {
    let mut sk = Sketch::new();
    let p = sk.add_geometry(Geometry::point(v(1.0, 2.0)));
    sk.add_constraint(Coincident(s(p), PointRef::root()));
    sk.add_constraint(LockX(s(p), 3.0));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Conflicting, "{r:?}");
    assert_eq!(r.conflicting, vec![0, 1]);
}

#[test]
fn tangent_at_endpoints_slot_profile() {
    let mut sk = Sketch::new();
    // roughly drawn slot: bottom line, right arc, top line (right to left), left arc
    let bot = sk.add_geometry(Geometry::line(v(0.2, -2.1), v(9.7, -1.8)));
    let ra = sk.add_geometry(Geometry::arc(v(10.1, 0.1), 2.2, -PI / 2.0 + 0.1, PI / 2.0 - 0.05));
    let top = sk.add_geometry(Geometry::line(v(10.3, 2.1), v(-0.2, 1.9)));
    let la = sk.add_geometry(Geometry::arc(v(-0.1, -0.2), 1.9, PI / 2.0, 3.0 * PI / 2.0 + 0.1));
    sk.add_constraint(TangentAt(e(bot), s(ra)));
    sk.add_constraint(TangentAt(e(ra), s(top)));
    sk.add_constraint(TangentAt(e(top), s(la)));
    sk.add_constraint(TangentAt(e(la), s(bot)));
    sk.add_constraint(Horizontal(bot));
    sk.add_constraint(Equal(ra, la));
    sk.add_constraint(Radius(ra, 2.0));
    sk.add_constraint(DistanceLine(bot, 10.0));
    sk.add_constraint(Coincident(m(la), PointRef::root()));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Solved, "{r:?}");
    assert_eq!(r.dof, 0, "{r:?}");
    assert!(vclose(sk.point(s(bot)), v(0.0, -2.0)));
    assert!(vclose(sk.point(e(bot)), v(10.0, -2.0)));
    assert!(vclose(sk.point(m(ra)), v(10.0, 0.0)));
    assert!(vclose(sk.point(e(top)), v(0.0, 2.0)));
    let loops = sk.profile_loops().unwrap();
    assert_eq!(loops.len(), 1);
    assert_eq!(loops[0].edges.len(), 4);
    // head-to-tail connectivity
    let ep = |le: &LoopEdge, head: bool| {
        let c = &sk.geometry[le.geo].curve;
        let (a, b) = (c.start_point().unwrap(), c.end_point().unwrap());
        if le.reversed != head { a } else { b }
    };
    let l = &loops[0];
    for i in 0..l.edges.len() {
        let nx = &l.edges[(i + 1) % l.edges.len()];
        assert!(ep(&l.edges[i], false).dist(ep(nx, true)) < 1e-6);
    }
    let area = sk.loop_signed_area(l).abs();
    assert!((area - (10.0 * 4.0 + PI * 4.0)).abs() < 1e-6, "area {area}");
    let faces = sk.profile_faces().unwrap();
    assert_eq!(faces.len(), 1);
    assert!(faces[0].holes.is_empty());
}

#[test]
fn tangent_at_endpoint_to_edge() {
    let mut sk = Sketch::new();
    let c = sk.add_geometry(Geometry::circle(v(0.0, 0.0), 5.0));
    let l = sk.add_geometry(Geometry::line(v(-3.0, 5.5), v(-10.0, 6.0)));
    sk.add_constraint(TangentAt(s(l), PointRef::edge(c)));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Solved, "{r:?}");
    let p = sk.point(s(l));
    let ctr = sk.point(m(c));
    let rad = match sk.geometry[c].curve {
        Curve::Circle { r, .. } => r,
        _ => unreachable!(),
    };
    assert!(close(p.dist(ctr), rad));
    let d = sk.point(e(l)) - p;
    assert!(close(d.dot(p - ctr), 0.0));
    assert_eq!(r.dof, 3 + 4 - 2);
}

#[test]
fn rectangle_with_hole_nests() {
    let (mut sk, _) = full_rect();
    let h = sk.add_geometry(Geometry::circle(v(5.0, 2.5), 1.0));
    let construction = sk.add_geometry(Geometry::construction(Curve::Line { a: v(-5.0, -5.0), b: v(20.0, 20.0) }));
    let _pt = sk.add_geometry(Geometry::point(v(3.0, 3.0)));
    let _ = construction;
    assert!(sk.solve().is_ok());
    let loops = sk.profile_loops().unwrap();
    assert_eq!(loops.len(), 2);
    let nest = sk.nest_loops(&loops);
    assert_eq!(nest.len(), 1);
    let (outer, holes) = &nest[0];
    assert_eq!(loops[*outer].edges.len(), 4);
    assert_eq!(holes.len(), 1);
    assert_eq!(loops[holes[0]].edges, vec![LoopEdge { geo: h, reversed: false }]);
}

#[test]
fn island_inside_hole_is_new_face() {
    let mut sk = Sketch::new();
    sk.add_geometry(Geometry::circle(v(0.0, 0.0), 10.0));
    sk.add_geometry(Geometry::circle(v(0.0, 0.0), 5.0));
    sk.add_geometry(Geometry::circle(v(1.0, 0.0), 2.0));
    sk.add_geometry(Geometry::circle(v(30.0, 0.0), 2.0));
    let loops = sk.profile_loops().unwrap();
    let nest = sk.nest_loops(&loops);
    assert_eq!(nest, vec![(0, vec![1]), (2, vec![]), (3, vec![])]);
}

#[test]
fn profile_errors() {
    let mut sk = Sketch::new();
    let a = sk.add_geometry(Geometry::line(v(0.0, 0.0), v(1.0, 0.0)));
    sk.add_geometry(Geometry::line(v(1.0, 0.0), v(1.0, 1.0)));
    assert_eq!(sk.profile_loops(), Err(ProfileError::OpenWire { geo: a }));
    // crossing bow-tie
    let mut sk = Sketch::new();
    sk.add_geometry(Geometry::line(v(0.0, 0.0), v(1.0, 1.0)));
    sk.add_geometry(Geometry::line(v(1.0, 1.0), v(1.0, 0.0)));
    sk.add_geometry(Geometry::line(v(1.0, 0.0), v(0.0, 1.0)));
    sk.add_geometry(Geometry::line(v(0.0, 1.0), v(0.0, 0.0)));
    assert!(matches!(sk.profile_loops(), Err(ProfileError::Intersecting { .. })));
    // circle crossing a rectangle edge
    let (mut sk, _) = full_rect();
    sk.solve();
    sk.add_geometry(Geometry::circle(v(10.0, 2.5), 1.0));
    assert!(matches!(sk.profile_loops(), Err(ProfileError::Intersecting { .. })));
    assert!(matches!(Sketch::new().profile_loops(), Err(ProfileError::Empty)));
    // branching
    let mut sk = Sketch::new();
    sk.add_geometry(Geometry::line(v(0.0, 0.0), v(1.0, 0.0)));
    sk.add_geometry(Geometry::line(v(1.0, 0.0), v(0.0, 1.0)));
    sk.add_geometry(Geometry::line(v(0.0, 1.0), v(0.0, 0.0)));
    sk.add_geometry(Geometry::line(v(0.0, 0.0), v(-1.0, -1.0)));
    sk.add_geometry(Geometry::line(v(-1.0, -1.0), v(-1.0, 0.0)));
    sk.add_geometry(Geometry::line(v(-1.0, 0.0), v(0.0, 0.0)));
    assert!(matches!(sk.profile_loops(), Err(ProfileError::BranchingWire { .. })));
    assert!(!format!("{}", ProfileError::OpenWire { geo: 3 }).is_empty());
}

#[test]
fn reversed_edges_in_loop() {
    // triangle drawn with inconsistent directions
    let mut sk = Sketch::new();
    sk.add_geometry(Geometry::line(v(0.0, 0.0), v(4.0, 0.0)));
    sk.add_geometry(Geometry::line(v(0.0, 3.0), v(4.0, 0.0)));
    sk.add_geometry(Geometry::line(v(0.0, 3.0), v(0.0, 0.0)));
    let loops = sk.profile_loops().unwrap();
    assert_eq!(loops.len(), 1);
    let l = &loops[0];
    assert_eq!(l.edges.len(), 3);
    assert!(l.edges[1].reversed);
    assert!(close(sk.loop_signed_area(l).abs(), 6.0));
    let poly = sk.loop_polygon(l, 64);
    assert_eq!(poly.len(), 4);
    assert!(vclose(poly[0], poly[3]));
}

#[test]
fn drag_keeps_constraints() {
    let (mut sk, l) = full_rect();
    sk.remove_constraint(8); // free width
    assert!(sk.solve().is_ok());
    let r = sk.drag_point(e(l[1]), v(15.0, 20.0));
    assert!(r.is_ok(), "{r:?}");
    assert!(residual(&sk) < 1e-9);
    // width follows the drag, height is fixed at 5, origin corner stays
    assert!(close(sk.point(e(l[1])).x, 15.0));
    assert!(close(sk.point(e(l[1])).y, 5.0));
    assert!(vclose(sk.point(s(l[0])), Vec2::ZERO));
}

#[test]
fn drag_free_geometry_reaches_target() {
    let mut sk = Sketch::new();
    let p = sk.add_geometry(Geometry::point(v(1.0, 1.0)));
    let l = sk.add_geometry(Geometry::line(v(0.0, 0.0), v(3.0, 0.0)));
    sk.add_constraint(Horizontal(l));
    sk.add_constraint(DistanceLine(l, 3.0));
    let r = sk.drag_point(s(p), v(-4.0, 7.0));
    assert!(r.is_ok());
    assert!(vclose(sk.point(s(p)), v(-4.0, 7.0)));
    // drag the line end diagonally: it stays horizontal with length 3
    sk.drag_point(e(l), v(10.0, 4.0));
    assert!(vclose(sk.point(e(l)), v(10.0, 4.0)));
    assert!(close(sk.point(s(l)).y, 4.0));
    assert!(close(sk.point(s(l)).dist(sk.point(e(l))), 3.0));
    // drag a circle edge changes its radius
    let c = sk.add_geometry(Geometry::circle(v(0.0, 0.0), 1.0));
    sk.add_constraint(Coincident(m(c), PointRef::root()));
    sk.drag_point(PointRef::edge(c), v(0.0, 4.0));
    match sk.geometry[c].curve {
        Curve::Circle { r, c } => {
            assert!(close(r, 4.0));
            assert!(vclose(c, Vec2::ZERO));
        }
        _ => unreachable!(),
    }
}

#[test]
fn drag_arc_endpoint_on_constrained_arc() {
    let mut sk = Sketch::new();
    let a = sk.add_geometry(Geometry::arc(v(0.0, 0.0), 2.0, 0.0, PI / 2.0));
    sk.add_constraint(Coincident(m(a), PointRef::root()));
    sk.add_constraint(Radius(a, 2.0));
    let r = sk.drag_point(e(a), v(-5.0, 0.1));
    assert!(r.is_ok());
    assert!(residual(&sk) < 1e-9);
    let p = sk.point(e(a));
    assert!(close(p.length(), 2.0));
    assert!(p.x < -1.9);
    assert!(vclose(sk.point(s(a)), v(2.0, 0.0)));
}

#[test]
fn reference_constraints_measure() {
    let (mut sk, l) = full_rect();
    let refd = sk.add_constraint(Constraint::reference(Distance(s(l[0]), e(l[1]), 1.0)));
    let refa = sk.add_constraint(Constraint::reference(AngleBetween(l[0], l[1], 0.0)));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Solved, "{r:?}");
    assert_eq!(r.dof, 0);
    assert!(close(sk.constraints[refd].kind.value().unwrap(), 125f64.sqrt()));
    assert!(close(sk.constraints[refa].kind.value().unwrap(), PI / 2.0));
}

#[test]
fn malformed_constraints() {
    let mut sk = Sketch::new();
    let c = sk.add_geometry(Geometry::circle(v(0.0, 0.0), 1.0));
    let l = sk.add_geometry(Geometry::line(v(0.0, 0.0), v(1.0, 0.0)));
    sk.add_constraint(Horizontal(c));
    sk.add_constraint(Coincident(s(c), s(l)));
    sk.add_constraint(Radius(l, 2.0));
    sk.add_constraint(Horizontal(42));
    sk.add_constraint(Equal(c, l));
    sk.add_constraint(DistanceLine(l, 2.0));
    let before = sk.geometry.clone();
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Failed);
    assert_eq!(r.malformed, vec![0, 1, 2, 3, 4]);
    assert_eq!(sk.geometry, before);
}

#[test]
fn block_constraint() {
    let mut sk = Sketch::new();
    let l = sk.add_geometry(Geometry::line(v(1.0, 1.0), v(4.0, 5.0)));
    sk.add_constraint(Block(l));
    let r = sk.solve();
    assert_eq!(r.dof, 0);
    assert_eq!(r.status, SolveStatus::Solved);
    let d = sk.add_constraint(DistanceLine(l, 5.0));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Redundant, "{r:?}");
    assert_eq!(r.redundant, vec![d]);
    sk.constraints[d].kind.set_value(6.0);
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Conflicting, "{r:?}");
    assert_eq!(r.conflicting, vec![0, d]);
    // a line attached to the blocked one keeps it fixed
    sk.remove_constraint(d);
    let l2 = sk.add_geometry(Geometry::line(v(4.0, 5.0), v(8.0, 5.0)));
    sk.add_constraint(Coincident(e(l), s(l2)));
    sk.add_constraint(LockX(s(l2), 0.0));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Conflicting, "{r:?}");
}

#[test]
fn point_on_object_and_point_line_distance() {
    let mut sk = Sketch::new();
    let c = sk.add_geometry(Geometry::circle(v(0.0, 0.0), 3.0));
    let p = sk.add_geometry(Geometry::point(v(4.0, 1.0)));
    sk.add_constraint(PointOnObject(s(p), c));
    let l = sk.add_geometry(Geometry::line(v(-5.0, -5.0), v(5.0, -4.0)));
    let q = sk.add_geometry(Geometry::point(v(0.0, 0.0)));
    sk.add_constraint(PointLineDistance(s(q), l, 2.0));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Solved);
    let pp = sk.point(s(p));
    let rad = match sk.geometry[c].curve {
        Curve::Circle { r, .. } => r,
        _ => unreachable!(),
    };
    assert!(close(pp.dist(sk.point(m(c))), rad));
    let (a, b) = (sk.point(s(l)), sk.point(e(l)));
    let qq = sk.point(s(q));
    assert!(close(((b - a).cross(qq - a) / (b - a).length()).abs(), 2.0));
}

#[test]
fn vertical_horizontal_points_and_distance_xy() {
    let mut sk = Sketch::new();
    let a = sk.add_geometry(Geometry::point(v(0.0, 0.0)));
    let b = sk.add_geometry(Geometry::point(v(1.0, 2.0)));
    let c = sk.add_geometry(Geometry::point(v(2.0, 1.0)));
    sk.add_constraint(Coincident(s(a), PointRef::root()));
    sk.add_constraint(VerticalPoints(s(a), s(b)));
    sk.add_constraint(HorizontalPoints(s(b), s(c)));
    sk.add_constraint(DistanceY(s(a), s(b), 3.0));
    sk.add_constraint(DistanceX(s(b), s(c), -4.0));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Solved);
    assert_eq!(r.dof, 0);
    assert!(vclose(sk.point(s(b)), v(0.0, 3.0)));
    assert!(vclose(sk.point(s(c)), v(-4.0, 3.0)));
}

#[test]
fn rough_sketch_converges() {
    // a roughly drawn L-shaped profile with many constraints
    let mut sk = Sketch::new();
    let pts = [v(0.3, -0.4), v(20.5, 0.7), v(19.6, 9.2), v(11.0, 10.4), v(9.4, 30.3), v(-0.6, 29.1)];
    let mut ls = vec![];
    for i in 0..6 {
        let a = pts[i];
        let b = pts[(i + 1) % 6] + v(0.3, -0.2);
        ls.push(sk.add_geometry(Geometry::line(a, b)));
    }
    for i in 0..6 {
        sk.add_constraint(Coincident(e(ls[i]), s(ls[(i + 1) % 6])));
    }
    sk.add_constraint(Horizontal(ls[0]));
    sk.add_constraint(Vertical(ls[1]));
    sk.add_constraint(Horizontal(ls[2]));
    sk.add_constraint(Vertical(ls[3]));
    sk.add_constraint(Horizontal(ls[4]));
    sk.add_constraint(Vertical(ls[5]));
    sk.add_constraint(Coincident(s(ls[0]), PointRef::root()));
    sk.add_constraint(DistanceLine(ls[0], 20.0));
    sk.add_constraint(DistanceLine(ls[1], 10.0));
    sk.add_constraint(DistanceLine(ls[5], 30.0));
    sk.add_constraint(DistanceLine(ls[4], 10.0));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Solved, "{r:?}");
    assert_eq!(r.dof, 0);
    assert!(vclose(sk.point(s(ls[3])), v(10.0, 10.0)));
    assert!(vclose(sk.point(s(ls[5])), v(0.0, 30.0)));
    let loops = sk.profile_loops().unwrap();
    assert_eq!(loops.len(), 1);
    assert!(close(sk.loop_signed_area(&loops[0]), 20.0 * 10.0 + 10.0 * 20.0));
}

#[test]
fn rough_triangle_with_angles_converges() {
    let mut sk = Sketch::new();
    let a = sk.add_geometry(Geometry::line(v(0.0, 0.0), v(10.0, 1.0)));
    let b = sk.add_geometry(Geometry::line(v(10.5, 0.5), v(4.0, 8.0)));
    let c = sk.add_geometry(Geometry::line(v(5.0, 7.0), v(0.5, -0.5)));
    sk.add_constraint(Coincident(e(a), s(b)));
    sk.add_constraint(Coincident(e(b), s(c)));
    sk.add_constraint(Coincident(e(c), s(a)));
    sk.add_constraint(Horizontal(a));
    sk.add_constraint(DistanceLine(a, 12.0));
    sk.add_constraint(Equal(a, b));
    sk.add_constraint(Equal(b, c));
    sk.add_constraint(Coincident(s(a), PointRef::root()));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Solved, "{r:?}");
    assert_eq!(r.dof, 0);
    let top = sk.point(e(b));
    assert!(vclose(top, v(6.0, 12.0 * (3f64).sqrt() / 2.0)));
}

#[test]
fn line_line_tangent_is_collinear() {
    let mut sk = Sketch::new();
    let a = sk.add_geometry(Geometry::line(v(0.0, 0.0), v(5.0, 0.2)));
    let b = sk.add_geometry(Geometry::line(v(6.0, 1.0), v(9.0, 1.5)));
    sk.add_constraint(Tangent(a, b));
    assert!(sk.solve().is_ok());
    let (p, q) = (sk.point(s(a)), sk.point(e(a)));
    for x in [sk.point(s(b)), sk.point(e(b))] {
        assert!(close((q - p).cross(x - p), 0.0));
    }
}

#[test]
fn arc_normalisation_after_solve() {
    let mut sk = Sketch::new();
    let a = sk.add_geometry(Geometry::arc(v(0.0, 0.0), 2.0, 5.0 * PI, 5.0 * PI + 1.0));
    sk.add_constraint(Radius(a, 3.0));
    assert!(sk.solve().is_ok());
    match sk.geometry[a].curve {
        Curve::Arc { r, start, end, .. } => {
            assert!(close(r, 3.0));
            assert!((0.0..2.0 * PI).contains(&start));
            assert!(close(end - start, 1.0));
            assert!(close(start, PI));
        }
        _ => unreachable!(),
    }
}

#[test]
fn diagnose_does_not_modify() {
    let (sk, _) = full_rect();
    let before = sk.clone();
    let r = sk.diagnose();
    assert_eq!(r.dof, 0);
    assert_eq!(sk, before);
}

#[test]
fn free_parameters_of_arc() {
    let mut sk = Sketch::new();
    let a = sk.add_geometry(Geometry::arc(v(0.0, 0.0), 2.0, 0.0, 1.0));
    sk.add_constraint(Coincident(m(a), PointRef::root()));
    sk.add_constraint(Radius(a, 2.0));
    assert!(sk.solve().is_ok());
    assert_eq!(sk.free_parameters()[0], vec![false, false, false, true, true]);
    sk.add_constraint(LockY(s(a), 0.0));
    sk.add_constraint(Angle(a, 1.0));
    let r = sk.solve();
    // LockY on the start point at y=0 with the centre at the origin pins the start angle
    // to 0 or pi
    assert_eq!(r.dof, 0, "{r:?}");
    assert_eq!(sk.fully_constrained_geometry(), vec![true]);
}

#[test]
fn remove_geometry_renumbers() {
    let (mut sk, l) = full_rect();
    let extra = sk.add_geometry(Geometry::point(v(1.0, 1.0)));
    sk.add_constraint(Coincident(s(extra), e(l[3])));
    sk.remove_geometry(l[1]);
    // constraints touching l1 are gone; the last coincident now refers to geometry 3
    assert!(sk.constraints.iter().all(|c| !c.kind.geometries().contains(&4)));
    assert_eq!(sk.constraints.last().unwrap().kind, Coincident(s(3), e(2)));
}

#[test]
fn tessellation() {
    let c = Curve::Circle { c: v(1.0, 1.0), r: 2.0 };
    let pts = tessellate_curve(&c, 16);
    assert_eq!(pts.len(), 17);
    assert!(vclose(pts[0], pts[16]));
    assert!(pts.iter().all(|p| close(p.dist(v(1.0, 1.0)), 2.0)));
    let a = Curve::Arc { c: v(0.0, 0.0), r: 1.0, start: 0.0, end: PI / 2.0 };
    let pts = tessellate_curve(&a, 4);
    assert!(vclose(pts[0], v(1.0, 0.0)));
    assert!(vclose(pts[4], v(0.0, 1.0)));
    let l = Curve::Line { a: v(0.0, 0.0), b: v(1.0, 0.0) };
    assert_eq!(tessellate_curve(&l, 10).len(), 2);
    let r = tessellate_edge(&a, true, 4);
    assert!(vclose(r[0], v(0.0, 1.0)));
}

#[test]
fn serde_roundtrip() {
    let (mut sk, _) = full_rect();
    sk.add_constraint(Constraint::reference(Angle(0, 0.0)).named("ref"));
    sk.add_constraint(PointOnObject(PointRef::start(1), H_AXIS));
    let js = serde_json::to_string(&sk).unwrap();
    let back: Sketch = serde_json::from_str(&js).unwrap();
    assert_eq!(back, sk);
    let r = sk.diagnose();
    let js = serde_json::to_string(&r).unwrap();
    let r2: SolveReport = serde_json::from_str(&js).unwrap();
    assert_eq!(r, r2);
    let loops = vec![ProfileLoop { edges: vec![LoopEdge { geo: 1, reversed: true }] }];
    let js = serde_json::to_string(&loops).unwrap();
    assert_eq!(serde_json::from_str::<Vec<ProfileLoop>>(&js).unwrap(), loops);
}

#[test]
fn conflicting_group_matches_freecad() {
    // FreeCAD 1.1 reports constraints 5, 9, 12 (1-based) as conflicting here.
    let (mut sk, l) = full_rect();
    let c = sk.add_constraint(DistanceX(s(l[0]), e(l[0]), 12.0));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Conflicting, "{r:?}");
    assert_eq!(r.conflicting, vec![4, 8, c]);
}

#[test]
fn arc_arc_endpoint_tangency_s_curve() {
    let mut sk = Sketch::new();
    let a = sk.add_geometry(Geometry::arc(v(0.0, 0.0), 2.0, -PI / 2.0, 0.1));
    let b = sk.add_geometry(Geometry::arc(v(4.2, 0.3), 2.1, PI - 0.2, 3.0 * PI / 2.0));
    sk.add_constraint(TangentAt(e(a), s(b)));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Solved, "{r:?}");
    assert_eq!(r.dof, 10 - 3);
    let p = sk.point(e(a));
    assert!(vclose(p, sk.point(s(b))));
    let (ca, cb) = (sk.point(m(a)), sk.point(m(b)));
    // tangent at the joint: both centres on the normal line through the joint
    assert!(close((p - ca).cross(cb - p), 0.0));
}

#[test]
fn fillet_line_arc_line() {
    let mut sk = Sketch::new();
    let l1 = sk.add_geometry(Geometry::line(v(0.0, 0.0), v(8.2, 0.3)));
    let arc = sk.add_geometry(Geometry::arc(v(7.8, 2.1), 1.9, -PI / 2.0 + 0.1, 0.05));
    let l2 = sk.add_geometry(Geometry::line(v(10.1, 2.2), v(9.8, 10.0)));
    sk.add_constraint(TangentAt(e(l1), s(arc)));
    sk.add_constraint(TangentAt(e(arc), s(l2)));
    sk.add_constraint(Horizontal(l1));
    sk.add_constraint(Vertical(l2));
    sk.add_constraint(Radius(arc, 2.0));
    sk.add_constraint(Coincident(s(l1), PointRef::root()));
    sk.add_constraint(DistanceX(s(l1), s(l2), 10.0));
    sk.add_constraint(DistanceLine(l2, 8.0));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Solved, "{r:?}");
    assert_eq!(r.dof, 0);
    assert!(vclose(sk.point(m(arc)), v(8.0, 2.0)));
    assert!(vclose(sk.point(e(l1)), v(8.0, 0.0)));
    assert!(vclose(sk.point(s(l2)), v(10.0, 2.0)));
    assert!(vclose(sk.point(e(l2)), v(10.0, 10.0)));
}

#[test]
fn drag_on_conflicting_sketch_does_nothing() {
    let mut sk = Sketch::new();
    let p = sk.add_geometry(Geometry::point(v(1.0, 2.0)));
    sk.add_constraint(LockX(s(p), 1.0));
    sk.add_constraint(LockX(s(p), 3.0));
    let before = sk.clone();
    let r = sk.drag_point(s(p), v(5.0, 5.0));
    assert_eq!(r.status, SolveStatus::Conflicting);
    assert_eq!(sk, before);
}

#[test]
fn constraint_on_external_geometry_only() {
    let mut sk = Sketch::new();
    sk.add_geometry(Geometry::point(v(1.0, 2.0)));
    let c = sk.add_constraint(Perpendicular(H_AXIS, V_AXIS));
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Redundant, "{r:?}");
    assert_eq!(r.redundant, vec![c]);
    sk.constraints[c].kind = Parallel(H_AXIS, V_AXIS);
    let r = sk.solve();
    assert_eq!(r.status, SolveStatus::Conflicting, "{r:?}");
    assert_eq!(r.dof, 2);
}
