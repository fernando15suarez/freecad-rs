//! FreeCAD parity suite: reference models built and measured by FreeCAD
//! (`tools/parity/make_models.py`, regenerate with `tools/parity/run.sh`) are imported,
//! recomputed with fc-part, and compared with FreeCAD's numbers.
//!
//! Tolerances: volume and area 1e-6 relative, bounding box 1e-6 mm; face/edge counts,
//! sketch DOF and conflicting/redundant lists exact (in FreeCAD's constraint numbering).
//!
//! `cargo test -p fc-part --test parity -- --nocapture parity_table` prints a table of
//! ours vs FreeCAD for every model (it never fails).
//! Known mismatches are `#[ignore = "see docs/parity.md"]`.

use fc_part::{fcstd, FeatureKind};
use fc_sketch::{Curve, Sketch};
use serde_json::Value;
use std::path::PathBuf;

const VOL_REL: f64 = 1e-6;
const BBOX_ABS: f64 = 1e-6;

fn models_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tools/parity/models")
}

#[derive(Default)]
struct Outcome {
    lines: Vec<String>,
    failures: Vec<String>,
    ours_volume: Option<f64>,
    fc_volume: Option<f64>,
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-12)
}

fn usize_list(v: &Value) -> Vec<usize> {
    v.as_array().map(|a| a.iter().filter_map(|x| x.as_u64()).map(|x| x as usize).collect()).unwrap_or_default()
}

fn compare(name: &str) -> Outcome {
    let dir = models_dir();
    let reference: Value = serde_json::from_str(&std::fs::read_to_string(dir.join(format!("{name}.json"))).expect("reference json"))
        .expect("valid json");
    let (mut doc, report) = fcstd::import_with_report(dir.join(format!("{name}.FCStd"))).expect("import");
    let as_saved = doc.clone();
    doc.recompute();
    let mut out = Outcome::default();
    for n in &report.notes {
        out.lines.push(format!("note [{:?}] {}: {}", n.kind, n.object, n.message));
        if n.kind == fcstd::NoteKind::Skipped {
            out.failures.push(format!("import skipped something: {}: {}", n.object, n.message));
        }
    }
    // Feature errors: FreeCAD's vs ours.
    let fc_errors: Vec<(String, Option<String>)> = reference["features"]
        .as_array()
        .unwrap()
        .iter()
        .map(|f| (f["name"].as_str().unwrap().to_string(), f["error"].as_str().map(str::to_string)))
        .collect();
    for (fname, fc_err) in &fc_errors {
        let Some(f) = doc.by_name(fname) else {
            out.failures.push(format!("{fname} was not imported"));
            continue;
        };
        let st = doc.state(f.id).unwrap();
        let ours = st.error.as_ref().map(|e| e.to_string());
        match (fc_err, &ours) {
            (None, Some(e)) => out.failures.push(format!("{fname}: we fail ({e}), FreeCAD does not")),
            // FreeCAD refuses sketches with redundant constraints; fc-part solves them.
            (Some(e), None) if e.to_lowercase().contains("redundant") => {
                out.lines.push(format!("{fname}: FreeCAD error '{e}', ours none (fc-part accepts redundant sketches)"))
            }
            (Some(e), None) => out.failures.push(format!("{fname}: FreeCAD fails ({e}), we do not")),
            (Some(e), Some(o)) => out.lines.push(format!("{fname}: both fail: FreeCAD '{e}', ours '{o}'")),
            (None, None) => {}
        }
        if let (Some(v), Some(shape)) = (f.kind.is_solid_feature().then(|| fc_feature_volume(&reference, fname)).flatten(), &st.shape) {
            if st.error.is_none() && rel(shape.volume(), v) > VOL_REL {
                out.lines.push(format!("{fname}: feature volume ours {:.6} FreeCAD {:.6}", shape.volume(), v));
            }
        }
    }
    // Sketch diagnostics.
    for (sname, fs) in reference["sketches"].as_object().unwrap() {
        let Some(f) = doc.by_name(sname) else { continue };
        let Some(rep) = doc.state(f.id).and_then(|s| s.solve.clone()) else {
            out.failures.push(format!("{sname}: no solve report"));
            continue;
        };
        let map = &report.sketches[sname];
        let fc_dof = fs["dof"].as_i64().unwrap();
        // FreeCAD reports a negative DOF for over-constrained sketches; fc-part's is
        // unsigned (free params - rank), only compare when FreeCAD's is meaningful.
        if fc_dof >= 0 && rep.dof as i64 != fc_dof {
            out.failures.push(format!("{sname}: DOF ours {} FreeCAD {fc_dof}", rep.dof));
        }
        for (what, ours, theirs) in [
            ("conflicting", &rep.conflicting, usize_list(&fs["conflicting"])),
            ("redundant", &rep.redundant, usize_list(&fs["redundant"])),
            ("partially redundant", &rep.partially_redundant, usize_list(&fs["partially_redundant"])),
            ("malformed", &rep.malformed, usize_list(&fs["malformed"])),
        ] {
            let ours_fc = map.to_freecad(ours);
            if ours_fc != theirs {
                out.failures.push(format!("{sname}: {what} ours {ours_fc:?} (our #{ours:?}) FreeCAD {theirs:?}"));
            }
        }
        out.lines.push(format!("{sname}: dof {} (FreeCAD {fc_dof}) {:?}", rep.dof, rep.status));
        // FreeCAD saved a solved sketch: solving our translation of its constraints must
        // not move anything (catches mistranslated constraint values/orientations).
        if rep.is_ok() {
            let (FeatureKind::Sketch(a), Some(FeatureKind::Sketch(b))) = (&f.kind, as_saved.by_name(sname).map(|f| &f.kind)) else { continue };
            let moved = max_geometry_delta(&a.sketch, &b.sketch);
            if moved > 1e-7 {
                let which: Vec<String> = (0..a.sketch.geometry.len())
                    .filter_map(|g| {
                        let one = |s: &Sketch| Sketch { geometry: vec![s.geometry[g].clone()], constraints: vec![] };
                        let d = max_geometry_delta(&one(&a.sketch), &one(&b.sketch));
                        (d > 1e-7).then(|| format!("geo {g}: {:?} -> {:?}", b.sketch.geometry[g].curve, a.sketch.geometry[g].curve))
                    })
                    .collect();
                out.failures.push(format!("{sname}: solving moved FreeCAD's solved geometry by {moved:.3e}: {}", which.join("; ")));
            }
        }
    }
    // Body solid.
    let fc_solid = &reference["solid"];
    let ours = doc.result();
    match (fc_solid.is_null(), ours) {
        (true, None) => {}
        (true, Some(s)) => {
            // FreeCAD has no solid (e.g. redundant sketch refused); report only.
            out.lines.push(format!("FreeCAD has no solid; ours volume {:.6}", s.volume()));
            out.ours_volume = Some(s.volume());
        }
        (false, None) => out.failures.push("no solid, FreeCAD has one".into()),
        (false, Some(s)) => {
            let v = fc_solid["volume"].as_f64().unwrap();
            let a = fc_solid["area"].as_f64().unwrap();
            out.ours_volume = Some(s.volume());
            out.fc_volume = Some(v);
            if rel(s.volume(), v) > VOL_REL {
                out.failures.push(format!("volume ours {:.9} FreeCAD {v:.9} (rel {:.2e})", s.volume(), rel(s.volume(), v)));
            }
            if rel(s.area(), a) > VOL_REL {
                out.failures.push(format!("area ours {:.9} FreeCAD {a:.9} (rel {:.2e})", s.area(), rel(s.area(), a)));
            }
            let fb: Vec<f64> = fc_solid["bbox"].as_array().unwrap().iter().map(|x| x.as_f64().unwrap()).collect();
            let ob = s.bbox();
            // OCCT bounding boxes of curved faces are enlarged by tolerances; use the
            // optimal box on both sides (FreeCAD's BoundBox is the precise one).
            if (0..6).any(|k| (ob[k] - fb[k]).abs() > BBOX_ABS) {
                out.failures.push(format!("bbox ours {:?} FreeCAD {:?}", round6(&ob), round6(&fb)));
            }
            for (what, ours, theirs) in [
                ("faces", s.face_count(), fc_solid["faces"].as_u64().unwrap() as usize),
                ("edges", s.edge_count(), fc_solid["edges"].as_u64().unwrap() as usize),
                ("solids", s.solid_count(), fc_solid["solids"].as_u64().unwrap() as usize),
            ] {
                if ours != theirs {
                    out.failures.push(format!("{what} ours {ours} FreeCAD {theirs}"));
                }
            }
        }
    }
    out
}

/// Largest parameter difference between two sketches with the same geometry layout.
fn max_geometry_delta(a: &Sketch, b: &Sketch) -> f64 {
    fn params(c: &Curve) -> Vec<f64> {
        match c {
            Curve::Point { p } => vec![p.x, p.y],
            Curve::Line { a, b } => vec![a.x, a.y, b.x, b.y],
            Curve::Circle { c, r } => vec![c.x, c.y, *r],
            // Compare arcs by their points (angles may differ by 2pi).
            Curve::Arc { .. } => {
                let (s, e, m) = (c.eval(0.0), c.eval(1.0), c.eval(0.5));
                vec![s.x, s.y, e.x, e.y, m.x, m.y]
            }
        }
    }
    a.geometry
        .iter()
        .zip(&b.geometry)
        .flat_map(|(x, y)| {
            let (p, q) = (params(&x.curve), params(&y.curve));
            if p.len() != q.len() {
                return vec![f64::INFINITY];
            }
            p.iter().zip(&q).map(|(u, v)| (u - v).abs()).collect()
        })
        .fold(0.0, f64::max)
}

/// Fully constrained sketches, nudged away from FreeCAD's solution, must solve back to it.
#[test]
fn perturbed_sketches_resolve_to_freecad_geometry() {
    let mut failures = Vec::new();
    let mut checked = 0;
    for name in ALL {
        let (doc, _) = fcstd::import_with_report(models_dir().join(format!("{name}.FCStd"))).unwrap();
        for f in &doc.features {
            let FeatureKind::Sketch(sf) = &f.kind else { continue };
            let rep = sf.sketch.diagnose();
            if rep.dof != 0 || !rep.conflicting.is_empty() || !rep.redundant.is_empty() || !rep.malformed.is_empty() {
                continue;
            }
            let mut sk = sf.sketch.clone();
            // Blocked geometry is fixed wherever it is: leave it alone.
            let blocked: Vec<usize> = sk.constraints.iter().filter_map(|c| match c.kind {
                fc_sketch::ConstraintKind::Block(g) => Some(g),
                _ => None,
            }).collect();
            for (i, g) in sk.geometry.iter_mut().enumerate() {
                if blocked.contains(&i) {
                    continue;
                }
                let d = 0.05 * (1.0 + (i % 3) as f64);
                let v = fc_sketch::Vec2::new(d, -0.7 * d);
                g.curve = match g.curve.clone() {
                    Curve::Point { p } => Curve::Point { p: p + v },
                    Curve::Line { a, b } => Curve::Line { a: a + v, b: b - v },
                    Curve::Circle { c, r } => Curve::Circle { c: c + v, r: r + d },
                    Curve::Arc { c, r, start, end } => Curve::Arc { c: c + v, r: r + d, start: start + 0.01, end: end - 0.01 },
                };
            }
            let r = sk.solve();
            checked += 1;
            let moved = max_geometry_delta(&sk, &sf.sketch);
            if !r.is_ok() || moved > 1e-6 {
                failures.push(format!("{name}/{}: {:?}, off FreeCAD's solution by {moved:.3e}", f.name, r.status));
            }
        }
    }
    println!("{checked} fully constrained sketches re-solved");
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

fn fc_feature_volume(reference: &Value, name: &str) -> Option<f64> {
    reference["features"].as_array()?.iter().find(|f| f["name"] == name)?["volume"].as_f64()
}

fn round6(b: &[f64]) -> Vec<f64> {
    b.iter().map(|x| (x * 1e6).round() / 1e6).collect()
}

fn run(name: &str) {
    let o = compare(name);
    for l in &o.lines {
        println!("{name}: {l}");
    }
    assert!(o.failures.is_empty(), "{name} differs from FreeCAD:\n  {}", o.failures.join("\n  "));
}

macro_rules! parity {
    ($($(#[$m:meta])* $name:ident),* $(,)?) => {
        $( $(#[$m])* #[test] fn $name() { run(stringify!($name)); } )*
        const ALL: &[&str] = &[$(stringify!($name)),*];
    };
}

parity! {
    pad_rect,
    pad_two_lengths,
    pad_symmetric_xz,
    pad_reversed_yz,
    pad_map_reversed,
    pad_up_to_first,
    pocket_two_lengths,
    revolution_two_angles,
    groove_symmetric_partial,
    pocket_through_all_face,
    pocket_length_offset_face,
    face_frames,
    revolution_v_axis,
    revolution_construction_line,
    revolution_symmetric_h_axis,
    groove_ring,
    hole_counterbore,
    hole_countersink,
    hole_through_flat,
    loft_rect_to_circle,
    loft_ruled_rects,
    pipe_l_path,
    #[ignore = "see docs/parity.md (F2: OCCT 'Transformed' transition at a sharp corner gives a degenerate solid; FreeCAD's own result is only the first leg)"]
    pipe_l_path_transformed,
    pipe_arc_path,
    slot_tangent_arcs,
    sketch_hole_loop,
    expressions_named,
    sketch_constraint_zoo,
    sketch_conflicting,
    sketch_redundant,
    sketch_underconstrained,
    multi_feature_part,
    multi_solid_body,
}

/// Every reference model must have a test above.
#[test]
fn all_models_listed() {
    let mut missing = Vec::new();
    for e in std::fs::read_dir(models_dir()).unwrap() {
        let p = e.unwrap().path();
        if p.extension().is_some_and(|x| x == "FCStd") {
            let stem = p.file_stem().unwrap().to_string_lossy().to_string();
            if !ALL.contains(&stem.as_str()) {
                missing.push(stem);
            }
        }
    }
    assert!(missing.is_empty(), "models without a parity test: {missing:?}");
}

/// Prints ours vs FreeCAD for every model (never fails).
#[test]
fn parity_table() {
    println!("| model | FreeCAD volume | ours | result |");
    println!("|---|---:|---:|---|");
    for name in ALL {
        let o = std::panic::catch_unwind(|| compare(name));
        match o {
            Ok(o) => {
                let f = |v: Option<f64>| v.map(|v| format!("{v:.6}")).unwrap_or_else(|| "-".into());
                let res = if o.failures.is_empty() { "match".to_string() } else { o.failures.join("; ") };
                println!("| {name} | {} | {} | {res} |", f(o.fc_volume), f(o.ours_volume));
            }
            Err(_) => println!("| {name} | | | panicked |"),
        }
    }
}
