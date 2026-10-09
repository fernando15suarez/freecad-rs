//! Rough performance check: a grid of fully constrained rectangles.
use fc_sketch::*;
use std::time::Instant;

fn main() {
    let n: usize = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(20);
    let chained = std::env::args().nth(2).is_some();
    let mut sk = Sketch::new();
    for k in 0..n {
        let ox = k as f64 * 15.0;
        let v = |x: f64, y: f64| Vec2::new(ox + x, y);
        let l0 = sk.add_geometry(Geometry::line(v(0.3, -0.2), v(9.1, 0.4)));
        let l1 = sk.add_geometry(Geometry::line(v(9.4, 0.1), v(10.2, 6.3)));
        let l2 = sk.add_geometry(Geometry::line(v(9.7, 5.6), v(-0.4, 4.9)));
        let l3 = sk.add_geometry(Geometry::line(v(0.2, 5.3), v(0.5, 0.6)));
        use ConstraintKind::*;
        for (a, b) in [(l0, l1), (l1, l2), (l2, l3), (l3, l0)] {
            sk.add_constraint(Coincident(PointRef::end(a), PointRef::start(b)));
        }
        sk.add_constraint(Horizontal(l0));
        sk.add_constraint(Horizontal(l2));
        sk.add_constraint(Vertical(l1));
        sk.add_constraint(Vertical(l3));
        sk.add_constraint(DistanceLine(l0, 10.0));
        sk.add_constraint(DistanceLine(l1, 5.0));
        if chained && k > 0 {
            let prev = l0 - 4;
            sk.add_constraint(DistanceX(PointRef::start(prev), PointRef::start(l0), 15.0));
            sk.add_constraint(HorizontalPoints(PointRef::start(prev), PointRef::start(l0)));
        } else {
            sk.add_constraint(LockX(PointRef::start(l0), ox));
            sk.add_constraint(LockY(PointRef::start(l0), 0.0));
        }
    }
    let t = Instant::now();
    let r = sk.solve();
    println!("{} params: solve {:?} -> {:?} dof {} iters {}", n * 16, t.elapsed(), r.status, r.dof, r.iterations);
    let t = Instant::now();
    let r = sk.drag_point(PointRef::end(1), Vec2::new(30.0, 30.0));
    println!("drag {:?} -> {:?}", t.elapsed(), r.status);
}
