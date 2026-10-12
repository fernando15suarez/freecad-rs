//! Shared helpers for the fc-fem integration tests.
#![allow(dead_code)]

use std::path::PathBuf;

use fc_part::{Document, Extrude, ExtentType, FeatureKind, SideType, Support};
use fc_sketch::{Curve, Geometry, Sketch, Vec2};

pub fn parity_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tools/parity/fem")
}

pub fn reference(case: &str) -> serde_json::Value {
    let p = parity_dir().join(format!("{case}.json"));
    serde_json::from_str(&std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{}: {e}", p.display()))).unwrap()
}

/// True when gmsh and ccx are available; otherwise prints why the test is skipped.
pub fn tools_or_skip(test: &str) -> bool {
    match (fc_fem::tools::gmsh(), fc_fem::tools::ccx()) {
        (Ok(_), Ok(_)) => true,
        (g, c) => {
            eprintln!("SKIP {test}: {}", g.err().or(c.err()).unwrap());
            false
        }
    }
}

pub fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-30)
}

fn polyline(s: &mut Sketch, pts: &[(f64, f64)]) {
    for i in 0..pts.len() {
        let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
        s.add_geometry(Geometry { curve: Curve::Line { a: Vec2 { x: a.0, y: a.1 }, b: Vec2 { x: b.0, y: b.1 } }, construction: false });
    }
}

/// An L-shaped bracket made with Part Design features (Pad of an L sketch on XZ, then a
/// through-all Pocket of a circle): 8 mm thick, 60 mm arm, 50 mm upright, 30 mm wide, 10 mm hole.
pub fn bracket() -> fc_occt::Shape {
    let mut doc = Document::new();
    let mut sk = Sketch::default();
    polyline(&mut sk, &[(0.0, 0.0), (60.0, 0.0), (60.0, 8.0), (8.0, 8.0), (8.0, 50.0), (0.0, 50.0)]);
    let s = doc.add_sketch(Support::XZ, sk);
    let mut e = Extrude::new(s, 30.0);
    e.side = SideType::Symmetric;
    doc.add(FeatureKind::Pad(e));
    let mut hole = Sketch::default();
    hole.add_geometry(Geometry { curve: Curve::Circle { c: Vec2 { x: 35.0, y: 0.0 }, r: 5.0 }, construction: false });
    let h = doc.add_sketch(Support::XY, hole);
    let mut p = Extrude::new(h, 0.0);
    p.kind = ExtentType::ThroughAll;
    p.side = SideType::Symmetric;
    let pk = doc.add(FeatureKind::Pocket(p));
    doc.recompute();
    let st = doc.state(pk).unwrap();
    assert!(st.error.is_none(), "{:?}", st.error);
    doc.result().unwrap().clone()
}
