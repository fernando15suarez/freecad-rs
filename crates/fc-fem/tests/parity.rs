//! End-to-end parity with FreeCAD 1.1.3's FEM workbench (Gmsh + CalculiX) on the reference
//! cases of tools/parity/fem (see freecad_reference.py), plus classical theory checks.
//! Solver tests are skipped (not failed) when gmsh / ccx are not installed.
mod common;

use common::*;
use fc_fem::*;
use serde_json::Value;

fn refs(v: &Value) -> Vec<SubRef> {
    v.as_array().unwrap().iter().map(|s| SubRef::parse(s.as_str().unwrap()).unwrap()).collect()
}
fn f(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn v3(v: &Value) -> [f64; 3] {
    let a = v.as_array().unwrap();
    [f(&a[0]), f(&a[1]), f(&a[2])]
}

/// The analysis FreeCAD solved, rebuilt from the case JSON on the BREP FreeCAD meshed.
fn analysis_for(case: &str, j: &Value) -> Analysis {
    let mut a = Analysis::new(ShapeSource::Brep(parity_dir().join(format!("{case}.brep"))), Material::steel_generic());
    let m = &j["mesh"];
    a.mesh.max_size = f(&m["max"]);
    a.mesh.min_size = f(&m["min"]);
    a.mesh.order = m["order"].as_u64().unwrap() as u8;
    if let Some(r) = m.get("refine") {
        a.mesh.refinements.push(MeshRefinement { refs: refs(&r["faces"]), size: f(&r["size"]) });
    }
    a.analysis_type = match j["analysis_type"].as_str().unwrap() {
        "frequency" => AnalysisType::Frequency { modes: 6, low_hz: 0.0, high_hz: 1e6 },
        "thermomech" => AnalysisType::Thermomech { steady_state: true },
        _ => AnalysisType::Static,
    };
    a.fixed(refs(&j["fixed"]));
    if let Some(ff) = j.get("force_faces") {
        let dir = if case == "plate_hole" { ForceDirection::Normal { reversed: false } } else { ForceDirection::Vector(v3(&j["force_dir"])) };
        a.force(refs(ff), f(&j["force"]), dir);
    }
    if let Some(p) = j.get("pressure_faces") {
        a.pressure(refs(p), f(&j["pressure"]), false);
    }
    if let Some(g) = j.get("gravity") {
        a.self_weight(v3(g));
    }
    if let Some(t) = j.get("initial_temperature") {
        a.add(ConstraintKind::InitialTemperature { temperature: f(t) }, vec![]);
        for key in ["cold", "hot"] {
            let e = &j["temperatures"][key];
            a.add(ConstraintKind::Temperature { temperature: f(&e[1]) }, refs(&e[0]));
        }
    }
    a
}

struct Row {
    nodes: (usize, u64),
    disp: (f64, f64),
    vm: (f64, f64),
}

fn run_case(case: &str) -> Option<(Solution, Value)> {
    if !tools_or_skip(case) {
        return None;
    }
    let j = reference(case);
    let a = analysis_for(case, &j);
    let sol = solve(&a, &RunOptions::default()).unwrap_or_else(|e| panic!("{case}: {e}"));
    Some((sol, j))
}

/// Same mesh as FreeCAD (same Gmsh, same .geo), so counts must be identical and results equal
/// to the precision of the .frd file.
fn compare(case: &str, sol: &Solution, j: &Value) -> Row {
    let r = &sol.results;
    assert_eq!(sol.mesh.node_count() as u64, j["nodes"].as_u64().unwrap(), "{case} nodes");
    assert_eq!(sol.mesh.element_count() as u64, j["volume_elements"].as_u64().unwrap(), "{case} elements");
    let mut row = Row { nodes: (sol.mesh.node_count(), j["nodes"].as_u64().unwrap()), disp: (0.0, 0.0), vm: (0.0, 0.0) };
    if let Some(d) = j.get("max_disp") {
        let ours = r.max_displacement().unwrap();
        assert!(rel(ours.value, f(d)) < 1e-4, "{case} max disp {} vs FreeCAD {}", ours.value, f(d));
        assert_eq!(ours.node as u64, j["max_disp_node"].as_u64().unwrap(), "{case} max disp node");
        row.disp = (ours.value, f(d));
    }
    if let Some(v) = j.get("max_von_mises") {
        let ours = r.max_von_mises().unwrap();
        assert!(rel(ours.value, f(v)) < 1e-4, "{case} max von Mises {} vs FreeCAD {}", ours.value, f(v));
        row.vm = (ours.value, f(v));
        let pmax = r.max(Field::PrincipalMax).unwrap().value;
        let pmin = r.min(Field::PrincipalMin).unwrap().value;
        assert!(rel(pmax, f(&j["max_principal"])) < 1e-3, "{case} principal max {pmax}");
        assert!(rel(pmin, f(&j["min_principal"])) < 1e-3, "{case} principal min {pmin}");
    }
    for (name, v) in j["reactions"].as_object().unwrap() {
        if j["analysis_type"] == "frequency" {
            break;
        }
        let ours = r.reactions.get(name).unwrap_or_else(|| panic!("{case}: no reaction {name}"));
        let theirs = v3(v);
        let scale = theirs.iter().fold(1.0f64, |m, x| m.max(x.abs()));
        for k in 0..3 {
            assert!((ours[k] - theirs[k]).abs() < 1e-4 * scale, "{case} reaction {name} {ours:?} vs {theirs:?}");
        }
    }
    if let Some(t) = j.get("max_temperature") {
        let temp = r.temperature.as_ref().unwrap();
        let mx = temp.iter().cloned().fold(f64::MIN, f64::max);
        assert!(rel(mx, f(t)) < 1e-5, "{case} max temperature {mx}");
    }
    println!(
        "PARITY {case}: nodes {} / {}, max|u| {:.6} / {:.6} mm, max vM {:.4} / {:.4} MPa",
        row.nodes.0, row.nodes.1, row.disp.0, row.disp.1, row.vm.0, row.vm.1
    );
    row
}

const E: f64 = 200_000.0;
const NU: f64 = 0.3;

#[test]
fn cantilever_matches_freecad_and_beam_theory() {
    let Some((sol, j)) = run_case("cantilever") else { return };
    compare("cantilever", &sol, &j);
    // Timoshenko beam: F L^3 / (3 E I) + F L / (k G A), k = 5/6 for a rectangle
    let (l, b, h, force) = (1000.0f64, 100.0f64, 100.0f64, 1000.0f64);
    let i = b * h.powi(3) / 12.0;
    let g = E / (2.0 * (1.0 + NU));
    let euler = force * l.powi(3) / (3.0 * E * i);
    let timo = euler + force * l / (5.0 / 6.0 * g * b * h);
    // tip deflection: mean uz of the loaded end face
    let tip = sol.mesh.nodes_on(SubKind::Face, 2);
    let uz: f64 = tip.iter().map(|n| sol.results.displacement[sol.mesh.node_index(*n).unwrap()][2]).sum::<f64>() / tip.len() as f64;
    println!("THEORY cantilever tip uz {:.6} mm, Euler-Bernoulli {euler:.6}, Timoshenko {timo:.6}", -uz);
    assert!(rel(-uz, timo) < 0.02, "tip {uz} vs Timoshenko {timo}");
    // bending stress at mid span (away from the clamp singularity): sigma = M c / I
    let x = 500.0;
    let sigma = force * (l - x) * (h / 2.0) / i;
    let top: Vec<usize> = (0..sol.mesh.node_count())
        .filter(|&k| {
            let p = sol.mesh.coords[k];
            (p[0] - x).abs() < 1e-6 && (p[2] - h).abs() < 1e-6
        })
        .collect();
    assert!(!top.is_empty());
    let sxx = top.iter().map(|&k| sol.results.stress[k][0]).sum::<f64>() / top.len() as f64;
    println!("THEORY cantilever sigma_xx(top, x=500) {sxx:.4} MPa, beam theory {sigma:.4}");
    assert!(rel(sxx, sigma) < 0.03, "{sxx} vs {sigma}");
}

#[test]
fn selfweight_matches_freecad_and_weight() {
    let Some((sol, j)) = run_case("selfweight") else { return };
    compare("selfweight", &sol, &j);
    let weight = 7900e-12 * STANDARD_GRAVITY * 1000.0 * 100.0 * 100.0;
    let rz = sol.results.reaction_sum()[2];
    println!("THEORY selfweight reaction {rz:.3} N, rho g V {weight:.3} N");
    // loads on constrained nodes do not show up as reactions: within 1 %
    assert!(rel(rz, weight) < 0.01);
}

#[test]
fn frequency_matches_freecad_and_beam_theory() {
    let Some((sol, j)) = run_case("frequency") else { return };
    compare("frequency", &sol, &j);
    let theirs: Vec<f64> = j["frequencies"].as_array().unwrap().iter().map(f).collect();
    assert_eq!(sol.results.modes.len(), theirs.len());
    for (m, t) in sol.results.modes.iter().zip(&theirs) {
        assert!(rel(m.frequency, *t) < 1e-5, "mode {} {} vs {t}", m.number, m.frequency);
    }
    // first bending mode of a clamped-free beam: (1.8751^2 / 2 pi) sqrt(E I / (rho A L^4))
    let (l, b, h) = (1.0f64, 0.1f64, 0.1f64);
    let f1 = 1.875104_f64.powi(2) / (2.0 * std::f64::consts::PI) * (E * 1e6 * b * h.powi(3) / 12.0 / (7900.0 * b * h * l.powi(4))).sqrt();
    println!("THEORY frequency f1 {:.3} Hz (FreeCAD {:.3}), Euler-Bernoulli {f1:.3}", sol.results.modes[0].frequency, theirs[0]);
    assert!(rel(sol.results.modes[0].frequency, f1) < 0.02);
}

#[test]
fn plate_with_hole_matches_freecad_and_peterson() {
    let Some((sol, j)) = run_case("plate_hole") else { return };
    compare("plate_hole", &sol, &j);
    // Peterson / Heywood: Kt(net) = 2 + (1 - d/W)^3 = 2.512 for d/W = 0.2
    let (w, d, t, force) = (100.0f64, 20.0f64, 10.0f64, 10000.0f64);
    let kt = 2.0 + (1.0 - d / w).powi(3);
    let sigma_max = kt * force / ((w - d) * t);
    // max sigma_xx on the hole
    let hole = sol.mesh.nodes_on(SubKind::Face, 7);
    let sxx = hole.iter().map(|n| sol.results.stress[sol.mesh.node_index(*n).unwrap()][0]).fold(f64::MIN, f64::max);
    println!("THEORY plate_hole max sigma_xx at hole {sxx:.3} MPa, Kt*sigma_net {sigma_max:.3} MPa (Kt {kt:.3})");
    assert!(rel(sxx, sigma_max) < 0.06);
}

#[test]
fn pressure_matches_freecad() {
    let Some((sol, j)) = run_case("pressure") else { return };
    compare("pressure", &sol, &j);
}

#[test]
fn thermal_bar_matches_freecad_and_expansion() {
    let Some((sol, j)) = run_case("thermal_bar") else { return };
    compare("thermal_bar", &sol, &j);
    // linear temperature 293 -> 393 K along 100 mm, free end: u = alpha * mean(dT) * L
    let u = 12e-6 * 50.0 * 100.0;
    let end = sol.mesh.nodes_on(SubKind::Face, 2);
    let ux = end.iter().map(|n| sol.results.displacement[sol.mesh.node_index(*n).unwrap()][0]).sum::<f64>() / end.len() as f64;
    println!("THEORY thermal_bar end ux {ux:.6} mm, alpha dT L {u:.6}");
    assert!(rel(ux, u) < 0.02);
}

#[test]
fn bracket_matches_freecad() {
    let Some((sol, j)) = run_case("bracket") else { return };
    compare("bracket", &sol, &j);
}

/// The bracket built natively with fc-part features (no BREP from FreeCAD), constraints picked
/// by geometric signature: the results must agree with FreeCAD's run on the STEP export.
#[test]
fn native_bracket_with_signatures_matches_freecad() {
    if !tools_or_skip("native bracket") {
        return;
    }
    let j = reference("bracket");
    let shape = common::bracket();
    let faces = shape.faces();
    let pick = |pred: &dyn Fn(&fc_occt::FaceInfo) -> bool| -> SubRef {
        let i = faces.iter().position(|f| pred(f)).expect("face") + 1;
        SubRef::face_of(&shape, i).unwrap()
    };
    let back = pick(&|f| f.kind == 0 && f.center[0].abs() < 1e-6);
    let end = pick(&|f| f.kind == 0 && (f.center[0] - 60.0).abs() < 1e-6);
    // deliberately wrong indices: the signature must find the faces again
    let mut back_wrong = back.clone();
    back_wrong.index = if back.index == 1 { 2 } else { 1 };
    let mut a = Analysis::new(ShapeSource::Shape(shape), Material::steel_generic());
    a.mesh.max_size = 4.0;
    a.fixed(vec![back_wrong]);
    a.force(vec![end], 500.0, ForceDirection::Vector([0.0, 0.0, -1.0]));
    let sol = solve(&a, &RunOptions::default()).unwrap();
    let r = &sol.results;
    let d = r.max_displacement().unwrap().value;
    let vm = r.max_von_mises().unwrap().value;
    println!(
        "PARITY native bracket: nodes {} / {}, max|u| {d:.6} / {:.6}, max vM {vm:.3} / {:.3}",
        sol.mesh.node_count(),
        j["nodes"],
        f(&j["max_disp"]),
        f(&j["max_von_mises"])
    );
    assert!(rel(d, f(&j["max_disp"])) < 0.02, "disp {d}");
    assert!(rel(vm, f(&j["max_von_mises"])) < 0.10, "vM {vm}");
    assert!((r.reaction_sum()[2] - 500.0).abs() < 1e-3);
}

/// The cantilever built natively from an fc-occt prism with faces picked by signature.
#[test]
fn native_cantilever_matches_freecad() {
    if !tools_or_skip("native cantilever") {
        return;
    }
    let j = reference("cantilever");
    let mut pb = fc_occt::ProfileBuilder::new();
    pb.begin_face();
    let pts = [[0.0, 0.0, 0.0], [0.0, 100.0, 0.0], [0.0, 100.0, 100.0], [0.0, 0.0, 100.0]];
    for i in 0..4 {
        pb.line(pts[i], pts[(i + 1) % 4]).unwrap();
    }
    pb.end_wire().unwrap();
    let shape = pb.build_faces().unwrap().prism([1000.0, 0.0, 0.0]).unwrap();
    let faces = shape.faces();
    let at_x = |x: f64| SubRef::face_of(&shape, faces.iter().position(|f| (f.center[0] - x).abs() < 1e-6).unwrap() + 1).unwrap();
    let (fixed, loaded) = (at_x(0.0), at_x(1000.0));
    let mut a = Analysis::new(ShapeSource::Shape(shape), Material::steel_generic());
    a.mesh.max_size = 25.0;
    a.fixed(vec![fixed]);
    a.force(vec![loaded], 1000.0, ForceDirection::Vector([0.0, 0.0, -1.0]));
    let sol = solve(&a, &RunOptions::default()).unwrap();
    let d = sol.results.max_displacement().unwrap().value;
    println!("PARITY native cantilever: nodes {} / {}, max|u| {d:.6} / {:.6}", sol.mesh.node_count(), j["nodes"], f(&j["max_disp"]));
    assert!(rel(d, f(&j["max_disp"])) < 0.01);
    assert!((sol.results.reaction_sum()[2] - 1000.0).abs() < 1e-3);
}
