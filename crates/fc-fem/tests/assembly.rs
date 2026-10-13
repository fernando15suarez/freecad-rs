//! Multi-solid (assembly) analyses: BooleanFragments (bonded, per-part materials), ties and
//! contact between separately meshed parts, contact detection, per-part results, and parity
//! with FreeCAD 1.1.3 on the cases of tools/parity/fem/freecad_assembly_reference.py.
//! Solver tests are skipped (not failed) when gmsh / ccx are not installed.
mod common;

use std::path::PathBuf;

use common::*;
use fc_fem::*;
use serde_json::Value;

fn block(o: [f64; 3], s: [f64; 3]) -> fc_occt::Shape {
    let mut b = fc_occt::ProfileBuilder::new();
    b.begin_face();
    let p = [[o[0], o[1], o[2]], [o[0] + s[0], o[1], o[2]], [o[0] + s[0], o[1] + s[1], o[2]], [o[0], o[1] + s[1], o[2]]];
    for i in 0..4 {
        b.line(p[i], p[(i + 1) % 4]).unwrap();
    }
    b.end_wire().unwrap();
    b.build_faces().unwrap().prism([0.0, 0.0, s[2]]).unwrap()
}

fn f(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn refs(v: &Value) -> Vec<SubRef> {
    v.as_array().unwrap().iter().map(|s| SubRef::parse(s.as_str().unwrap()).unwrap()).collect()
}

/// Planar faces of the analysed shape whose centre satisfies `pred`.
fn faces_where(a: &Analysis, pred: impl Fn(&fc_occt::FaceInfo) -> bool) -> Vec<SubRef> {
    let s = a.shape_in_memory().unwrap();
    s.faces().iter().enumerate().filter(|(_, fi)| fi.kind == 0 && pred(fi)).map(|(i, _)| a.face_ref(i + 1)).collect()
}

fn work_dir(case: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("fc-fem-asm-{case}-{}", std::process::id()));
    std::fs::create_dir_all(&d).unwrap();
    d
}

/// Mesh.inp without node / element data lines (as the reference script stores FreeCAD's).
fn trim_inp(text: &str) -> String {
    let mut out = String::new();
    let mut data = false;
    for line in text.split_inclusive('\n') {
        if line.starts_with('*') {
            data = line.starts_with("*Node, NSET=Nall") || line.starts_with("*Element, TYPE=");
            out.push_str(line);
        } else if data && !line.trim().is_empty() {
            continue;
        } else {
            out.push_str(line);
        }
    }
    out
}

fn assert_same_inp(ours: &str, theirs: &str, what: &str) {
    let skip = |l: &&str| l.starts_with("**   written by") || l.starts_with("**   written on") || l.starts_with("**   file name");
    let a: Vec<&str> = ours.lines().filter(|l| !skip(l)).collect();
    let b: Vec<&str> = theirs.lines().filter(|l| !skip(l)).collect();
    for (i, (x, y)) in a.iter().zip(&b).enumerate() {
        assert_eq!(x, y, "{what}: first difference at (filtered) line {}", i + 1);
    }
    assert_eq!(a.len(), b.len(), "{what}: line count");
}

/// Exact parity on FreeCAD's BREP: same mesh, same input file, same results.
fn compare_exact(case: &str, a: &Analysis) -> Solution {
    let j = reference(case);
    let dir = work_dir(case);
    let sol = solve(a, &RunOptions { work_dir: Some(dir.clone()), ..Default::default() }).unwrap_or_else(|e| panic!("{case}: {e}"));
    let ours = trim_inp(&std::fs::read_to_string(dir.join("Mesh.inp")).unwrap());
    let theirs = std::fs::read_to_string(parity_dir().join(format!("{case}.inp"))).unwrap();
    assert_same_inp(&ours, &theirs, case);
    let r = &sol.results;
    assert_eq!(sol.mesh.node_count() as u64, j["nodes"].as_u64().unwrap(), "{case} nodes");
    assert_eq!(sol.mesh.element_count() as u64, j["volume_elements"].as_u64().unwrap(), "{case} elements");
    let d = r.max_displacement().unwrap();
    assert!(rel(d.value, f(&j["max_disp"])) < 1e-4, "{case} max disp {} vs {}", d.value, j["max_disp"]);
    assert_eq!(d.node as u64, j["max_disp_node"].as_u64().unwrap(), "{case} max disp node");
    let v = r.max_von_mises().unwrap();
    assert!(rel(v.value, f(&j["max_von_mises"])) < 1e-4, "{case} max vM {} vs {}", v.value, j["max_von_mises"]);
    for (name, rf) in j["reactions"].as_object().unwrap() {
        let ours = r.reactions[name];
        let theirs: Vec<f64> = rf.as_array().unwrap().iter().map(f).collect();
        let scale = theirs.iter().fold(1.0f64, |m, x| m.max(x.abs()));
        for k in 0..3 {
            assert!((ours[k] - theirs[k]).abs() < 1e-4 * scale, "{case} reaction {name}: {ours:?} vs {theirs:?}");
        }
    }
    // per solid
    let parts = part_results(a, &sol);
    for (k, s) in j["solids"].as_array().unwrap().iter().enumerate() {
        let p = parts.iter().find(|p| p.solids == vec![k + 1]).unwrap();
        assert_eq!(p.node_count as u64, s["nodes"].as_u64().unwrap(), "{case} solid {} nodes", k + 1);
        assert!(rel(p.max_von_mises.unwrap().value, f(&s["max_von_mises"])) < 1e-4, "{case} solid {} vM", k + 1);
        assert!(rel(p.max_displacement.unwrap().value, f(&s["max_disp"])) < 1e-4, "{case} solid {} disp", k + 1);
    }
    println!(
        "PARITY {case}: nodes {} / {}, max|u| {:.6} / {:.6} mm, max vM {:.4} / {:.4} MPa, per part vM {:?}",
        sol.mesh.node_count(),
        j["nodes"],
        d.value,
        f(&j["max_disp"]),
        v.value,
        f(&j["max_von_mises"]),
        parts.iter().map(|p| (p.name.clone(), p.max_von_mises.unwrap().value)).collect::<Vec<_>>()
    );
    sol
}

fn brep_analysis(case: &str, j: &Value, shape_name: &str) -> Analysis {
    let mut a = Analysis::new(ShapeSource::Brep(parity_dir().join(format!("{case}.brep"))), Material::steel_generic());
    a.shape_name = shape_name.into();
    a.mesh.max_size = f(&j["mesh"]["max"]);
    a.mesh.coherence = j["mesh"]["coherence"].as_bool().unwrap();
    a
}

// ---------------------------------------------------------------------------------------------
// stacked blocks, BooleanFragments, two materials

fn stack_brep() -> Analysis {
    let j = reference("asm_stack");
    let mut a = brep_analysis("asm_stack", &j, "BooleanFragments");
    let (lo, up) = (j["solid_lower"].as_u64().unwrap() as usize, j["solid_upper"].as_u64().unwrap() as usize);
    a.materials = vec![
        MaterialAssignment { name: "FemMaterial".into(), material: Material::aluminium_6061_t6(), solids: vec![SubRef::solid(lo)] },
        MaterialAssignment { name: "FemMaterial001".into(), material: Material::steel_generic(), solids: vec![SubRef::solid(up)] },
    ];
    a.parts = vec![
        PartMap { name: "Lower".into(), solids: vec![lo], faces: vec![] },
        PartMap { name: "Upper".into(), solids: vec![up], faces: vec![] },
    ];
    a.fixed(refs(&j["fixed"]));
    a.pressure(refs(&j["pressure_faces"]), f(&j["pressure"]), false);
    a
}

fn stack_native() -> Analysis {
    let parts = [
        AssemblyPart::new("Lower", block([0.0; 3], [40.0, 40.0, 20.0]), Material::aluminium_6061_t6()),
        AssemblyPart::new("Upper", block([0.0, 0.0, 20.0], [40.0, 40.0, 20.0]), Material::steel_generic()),
    ];
    let mut a = Analysis::from_parts(&parts, Joining::Fragments { fuzzy: 0.0 }).unwrap();
    a.mesh.max_size = 4.0;
    let bottom = faces_where(&a, |fi| fi.center[2].abs() < 1e-9);
    let top = faces_where(&a, |fi| (fi.center[2] - 40.0).abs() < 1e-9);
    a.fixed(bottom);
    a.pressure(top, 10.0, false);
    a
}

#[test]
fn from_parts_fragments_maps_parts_and_materials() {
    let a = stack_native();
    let s = a.shape_in_memory().unwrap();
    assert_eq!(s.solid_count(), 2);
    assert_eq!(s.face_count(), 11, "the contact face is shared");
    assert_eq!(a.shape_name, "BooleanFragments");
    assert!(a.mesh.coherence);
    assert_eq!(a.parts.len(), 2);
    assert_eq!(a.parts[0].solids, vec![1]);
    assert_eq!(a.parts[1].solids, vec![2]);
    assert_eq!(a.materials.len(), 2);
    assert_eq!(a.materials[0].name, "FemMaterial");
    assert_eq!(a.materials[0].material.name, "Aluminum 6061-T6");
    assert_eq!(a.materials[1].name, "FemMaterial001");
    assert_eq!(a.materials[1].solids, vec![SubRef::solid(2)]);
    // the shared face is reported as already bonded
    let pairs = detect_contacts(&a, 0.01).unwrap();
    assert_eq!(pairs.len(), 1);
    assert!(pairs[0].shared);
    assert!((pairs[0].area - 1600.0).abs() < 1e-6);
    // every part face maps to the result
    for p in &a.parts {
        for local in 1..=6 {
            assert!(!p.faces_of(local).is_empty(), "{} Face{local}", p.name);
        }
    }
    let r = a.part_face("Upper", 1).unwrap();
    assert!(r[0].signature.is_some());
    // equal materials share one assignment
    let same = [
        AssemblyPart::new("A", block([0.0; 3], [10.0, 10.0, 10.0]), Material::steel_generic()),
        AssemblyPart::new("B", block([10.0, 0.0, 0.0], [10.0, 10.0, 10.0]), Material::steel_generic()),
    ];
    let b = Analysis::from_parts(&same, Joining::Separate).unwrap();
    assert_eq!(b.materials.len(), 1);
    assert!(b.materials[0].solids.is_empty());
    assert!(!b.mesh.coherence);
    assert_eq!(b.shape_in_memory().unwrap().face_count(), 12);
}

#[test]
fn stack_bonded_by_fragments_matches_freecad() {
    if !tools_or_skip("stack_bonded_by_fragments_matches_freecad") {
        return;
    }
    let sol = compare_exact("asm_stack", &stack_brep());
    // conformal: the interface nodes belong to elements of both solids
    let s1 = sol.mesh.nodes_on(SubKind::Solid, 1);
    let s2 = sol.mesh.nodes_on(SubKind::Solid, 2);
    assert!(s1.intersection(&s2).count() > 50);
    // equilibrium: 10 MPa on 40 x 40
    assert!(rel(sol.results.reaction_sum()[2], 16000.0) < 1e-6);
    // surfaces: outer boundary without the interface, per-solid boundary with it twice
    let outer = &sol.results.surface;
    assert_eq!(outer.tri_solid.len(), outer.triangles.len());
    assert!(outer.tri_solid.iter().all(|&s| s == 1 || s == 2));
    let per = sol.mesh.boundary_by_solid();
    assert!(per.triangles.len() > outer.triangles.len());
}

#[test]
fn stack_native_close_to_freecad() {
    if !tools_or_skip("stack_native_close_to_freecad") {
        return;
    }
    let j = reference("asm_stack");
    let a = stack_native();
    let sol = solve(&a, &RunOptions::default()).unwrap();
    let d = sol.results.max_displacement().unwrap().value;
    let parts = part_results(&a, &sol);
    println!(
        "NATIVE asm_stack: nodes {} / {}, max|u| {d:.6} / {:.6}, vM {:.4} / {:.4}, parts {:?}",
        sol.mesh.node_count(),
        j["nodes"],
        f(&j["max_disp"]),
        sol.results.max_von_mises().unwrap().value,
        f(&j["max_von_mises"]),
        parts.iter().map(|p| (p.name.clone(), p.max_von_mises.unwrap().value, p.reaction)).collect::<Vec<_>>()
    );
    assert!(rel(d, f(&j["max_disp"])) < 0.01);
    // the fixed face is on the lower part: its reaction is attributed to it
    assert!(rel(parts[0].reaction[2], 16000.0) < 1e-6);
    assert!((parts[0].volume - 32000.0).abs() < 1.0);
    // 1D estimate (lateral strain free): u = p (h1/E1 + h2/E2)
    let est = 10.0 * (20.0 / 68_900.0 + 20.0 / 200_000.0);
    println!("THEORY asm_stack top uz ~ {est:.6} mm (uniaxial stress), max |u| {d:.6}");
    assert!(rel(d, est) < 0.15);
}

// ---------------------------------------------------------------------------------------------
// lap joint with a tie

fn lap_brep() -> Analysis {
    let j = reference("asm_lap_tie");
    let mut a = brep_analysis("asm_lap_tie", &j, "Compound");
    a.parts = vec![PartMap { name: "PlateA".into(), solids: vec![1], faces: vec![] }, PartMap { name: "PlateB".into(), solids: vec![2], faces: vec![] }];
    a.fixed(refs(&j["fixed"]));
    a.force(refs(&j["force_faces"]), f(&j["force"]), ForceDirection::Normal { reversed: false });
    a.tie(refs(&j["tie_slave"]), refs(&j["tie_master"]), f(&j["tie_tolerance"]));
    a
}

fn lap_parts() -> [AssemblyPart; 2] {
    [
        AssemblyPart::new("PlateA", block([0.0; 3], [100.0, 40.0, 5.0]), Material::steel_generic()),
        AssemblyPart::new("PlateB", block([61.7, 4.3, 5.05], [98.3, 31.4, 5.0]), Material::steel_generic()),
    ]
}

#[test]
fn detect_contacts_finds_the_lap_faces() {
    let a = Analysis::from_parts(&lap_parts(), Joining::Separate).unwrap();
    let pairs = detect_contacts(&a, 0.1).unwrap();
    assert_eq!(pairs.len(), 1, "{pairs:?}");
    let p = &pairs[0];
    assert!(!p.shared);
    assert_eq!(p.kind, ContactKind::Planar);
    assert_eq!((p.slave_part, p.master_part), (1, 0), "the smaller face (plate B) is the slave");
    let overlap = (100.0 - 61.7) * 31.4;
    assert!(rel(p.area, overlap) < 0.05, "area {} vs {overlap}", p.area);
    assert!((p.gap - 0.05).abs() < 1e-6, "gap {}", p.gap);
    // too far apart for a smaller tolerance
    assert!(detect_contacts(&a, 0.01).unwrap().is_empty());
}

#[test]
fn detect_contacts_finds_a_pin_in_a_hole() {
    // plate with a 10 mm hole and a 10 mm pin through it
    let mut b = fc_occt::ProfileBuilder::new();
    b.begin_face();
    let p = [[0.0, 0.0, 0.0], [40.0, 0.0, 0.0], [40.0, 30.0, 0.0], [0.0, 30.0, 0.0]];
    for i in 0..4 {
        b.line(p[i], p[(i + 1) % 4]).unwrap();
    }
    b.end_wire().unwrap();
    b.circle([20.0, 15.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 5.0).unwrap();
    b.end_wire().unwrap();
    let plate = b.build_faces().unwrap().prism([0.0, 0.0, 10.0]).unwrap();
    let mut c = fc_occt::ProfileBuilder::new();
    c.begin_face();
    c.circle([20.0, 15.0, -5.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 5.0).unwrap();
    c.end_wire().unwrap();
    let pin = c.build_faces().unwrap().prism([0.0, 0.0, 20.0]).unwrap();
    let parts = [AssemblyPart::new("Plate", plate, Material::steel_generic()), AssemblyPart::new("Pin", pin, Material::steel_generic())];
    let a = Analysis::from_parts(&parts, Joining::Separate).unwrap();
    let pairs = detect_contacts(&a, 0.01).unwrap();
    assert_eq!(pairs.len(), 1, "{pairs:?}");
    assert_eq!(pairs[0].kind, ContactKind::Cylindrical);
    let bore = std::f64::consts::PI * 10.0 * 10.0;
    assert!(rel(pairs[0].area, bore) < 0.05, "area {} vs {bore}", pairs[0].area);
}

#[test]
fn lap_joint_tie_matches_freecad() {
    if !tools_or_skip("lap_joint_tie_matches_freecad") {
        return;
    }
    let sol = compare_exact("asm_lap_tie", &lap_brep());
    assert!(rel(sol.results.reaction_sum()[0], -2000.0) < 1e-6);
}

#[test]
fn lap_joint_native_ties_close_to_freecad() {
    if !tools_or_skip("lap_joint_native_ties_close_to_freecad") {
        return;
    }
    let j = reference("asm_lap_tie");
    let mut a = Analysis::from_parts(&lap_parts(), Joining::Separate).unwrap();
    a.mesh.max_size = 2.5;
    let pairs = detect_contacts(&a, 0.1).unwrap();
    assert_eq!(a.tie_contacts(&pairs, 0.1), 1);
    let fixed = faces_where(&a, |fi| fi.center[0].abs() < 1e-9);
    let pulled = faces_where(&a, |fi| (fi.center[0] - 160.0).abs() < 1e-9);
    a.fixed(fixed);
    a.force(pulled, 2000.0, ForceDirection::Vector([1.0, 0.0, 0.0]));
    let sol = solve(&a, &RunOptions::default()).unwrap();
    let d = sol.results.max_displacement().unwrap().value;
    let parts = part_results(&a, &sol);
    println!(
        "NATIVE asm_lap_tie: nodes {} / {}, max|u| {d:.6} / {:.6}, vM {:.4} / {:.4}, parts {:?}",
        sol.mesh.node_count(),
        j["nodes"],
        f(&j["max_disp"]),
        sol.results.max_von_mises().unwrap().value,
        f(&j["max_von_mises"]),
        parts.iter().map(|p| (p.name.clone(), p.max_von_mises.unwrap().value, p.reaction)).collect::<Vec<_>>()
    );
    assert!(rel(d, f(&j["max_disp"])) < 0.02);
    assert!(rel(parts[0].reaction[0], -2000.0) < 1e-6);
}

// ---------------------------------------------------------------------------------------------
// punch pressed into a block through penalty contact

fn contact_brep() -> Analysis {
    let j = reference("asm_contact");
    let mut a = brep_analysis("asm_contact", &j, "Compound");
    a.parts = vec![PartMap { name: "Block".into(), solids: vec![1], faces: vec![] }, PartMap { name: "Punch".into(), solids: vec![2], faces: vec![] }];
    a.fixed(refs(&j["fixed"]));
    a.add(ConstraintKind::Displacement { x: Some(0.0), y: Some(0.0), z: None }, refs(&j["top"]));
    a.pressure(refs(&j["top"]), f(&j["pressure"]), false);
    a.add_interaction(
        InteractionKind::Contact { behavior: ContactBehavior::Linear, slope: f(&j["contact_slope"]), adjust: f(&j["contact_adjust"]), friction: None },
        refs(&j["contact_slave"]),
        refs(&j["contact_master"]),
    );
    a
}

#[test]
fn punch_contact_matches_freecad() {
    if !tools_or_skip("punch_contact_matches_freecad") {
        return;
    }
    let sol = compare_exact("asm_contact", &contact_brep());
    // the punch is pressed into the block: compressive contact stresses under it
    let block_top_mid = (0..sol.mesh.node_count())
        .filter(|&k| {
            let p = sol.mesh.coords[k];
            (p[2] - 10.0).abs() < 1e-9 && (p[0] - 20.0).abs() < 3.0 && (p[1] - 20.0).abs() < 3.0
        })
        .map(|k| sol.results.stress[k][2])
        .collect::<Vec<_>>();
    assert!(!block_top_mid.is_empty());
    let mean = block_top_mid.iter().sum::<f64>() / block_top_mid.len() as f64;
    println!("THEORY asm_contact mean sigma_zz under the punch {mean:.2} MPa (nominal -50)");
    assert!(mean < -10.0);
}

#[test]
fn punch_native_contact_close_to_freecad() {
    if !tools_or_skip("punch_native_contact_close_to_freecad") {
        return;
    }
    let j = reference("asm_contact");
    let parts = [
        AssemblyPart::new("Block", block([0.0; 3], [40.0, 40.0, 10.0]), Material::steel_generic()),
        AssemblyPart::new("Punch", block([10.0, 10.0, 10.05], [20.0, 20.0, 10.0]), Material::steel_generic()),
    ];
    let mut a = Analysis::from_parts(&parts, Joining::Separate).unwrap();
    a.mesh.max_size = 2.5;
    let pairs = detect_contacts(&a, 0.1).unwrap();
    assert_eq!(pairs.len(), 1);
    let (s, m) = (a.face_ref(pairs[0].slave_face), a.face_ref(pairs[0].master_face));
    a.add_interaction(InteractionKind::Contact { behavior: ContactBehavior::Linear, slope: 1e6, adjust: 0.1, friction: None }, vec![s], vec![m]);
    let bottom = faces_where(&a, |fi| fi.center[2].abs() < 1e-9);
    let top = faces_where(&a, |fi| (fi.center[2] - 20.05).abs() < 1e-9);
    a.fixed(bottom);
    a.add(ConstraintKind::Displacement { x: Some(0.0), y: Some(0.0), z: None }, top.clone());
    a.pressure(top, 50.0, false);
    let sol = solve(&a, &RunOptions::default()).unwrap();
    let d = sol.results.max_displacement().unwrap().value;
    println!(
        "NATIVE asm_contact: nodes {} / {}, max|u| {d:.6} / {:.6}, vM {:.4} / {:.4}",
        sol.mesh.node_count(),
        j["nodes"],
        f(&j["max_disp"]),
        sol.results.max_von_mises().unwrap().value,
        f(&j["max_von_mises"])
    );
    assert!(rel(d, f(&j["max_disp"])) < 0.05);
}

// ---------------------------------------------------------------------------------------------
// fc-assembly workspace

#[test]
fn analysis_from_an_fc_assembly_workspace() {
    use fc_assembly::{Frame, Workspace};
    let mut ws = Workspace::new();
    let base = ws.add_imported_part("Base", block([0.0; 3], [40.0, 40.0, 20.0]));
    let cap = ws.add_imported_part("Cap", block([0.0; 3], [40.0, 40.0, 20.0]));
    ws.part_mut(cap).unwrap().meta.material = "Aluminum 6061-T6".into();
    let asm = ws.add_assembly("Stack");
    {
        let a = ws.assembly_mut(asm).unwrap();
        a.add_part_instance("Base", base, Frame::IDENTITY);
        a.add_part_instance("Cap", cap, Frame::translation([0.0, 0.0, 20.0]));
    }
    let a = analysis_from_assembly(&ws, asm, Joining::Fragments { fuzzy: 0.0 }).unwrap();
    assert_eq!(a.parts.iter().map(|p| p.name.as_str()).collect::<Vec<_>>(), vec!["Base", "Cap"]);
    let s = a.shape_in_memory().unwrap();
    assert_eq!(s.solid_count(), 2);
    assert_eq!(s.face_count(), 11);
    assert_eq!(a.materials.len(), 2);
    assert_eq!(a.materials[1].material.name, "Aluminum 6061-T6");
    // the posed cap sits on top
    let cap_solid = a.parts[1].solids[0];
    let top = s.solid_faces(cap_solid).unwrap().into_iter().map(|i| s.face_info(i).unwrap()).map(|fi| fi.center[2]).fold(f64::MIN, f64::max);
    assert!((top - 40.0).abs() < 1e-9);
}

#[test]
fn analysis_json_round_trip_with_interactions() {
    let mut a = Analysis::new(ShapeSource::Brep("x.brep".into()), Material::steel_generic());
    a.tie(vec![SubRef::face(11)], vec![SubRef::face(6)], 0.1);
    a.contact(vec![SubRef::face(2)], vec![SubRef::face(1)], 1e6, Some(Friction { coefficient: 0.2, stick_slope: 1e5 }));
    a.parts.push(PartMap { name: "A".into(), solids: vec![1], faces: vec![(1, 1)] });
    let b = Analysis::from_json(&a.to_json()).unwrap();
    assert_eq!(b.interactions, a.interactions);
    assert_eq!(b.parts, a.parts);
    assert_eq!(b.interactions[0].name, "ConstraintTie");
    assert_eq!(b.interactions[1].name, "ConstraintContact");
    // old files without the new fields still load
    let old = a.to_json().replace("\"interactions\"", "\"x_interactions\"").replace("\"parts\"", "\"x_parts\"");
    let c = Analysis::from_json(&old).unwrap();
    assert!(c.interactions.is_empty() && c.parts.is_empty());
}
