//! FreeCAD parity of the Gmsh .geo and CalculiX .inp writers, and the .frd/.dat readers, on
//! small reference files produced by FreeCAD 1.1.3 (tools/parity/fem/freecad_reference.py).
mod common;

use common::*;
use fc_fem::*;

pub fn snapshot_static_analysis() -> Analysis {
    let p = parity_dir();
    let mut a = Analysis::new(ShapeSource::Brep(p.join("snapshot_static.brep")), Material::steel_generic());
    a.mesh.max_size = 10.0;
    a.fixed(vec![SubRef::face(1)]);
    a.add(ConstraintKind::Displacement { x: None, y: Some(0.0), z: None }, vec![SubRef::face(3)]);
    a.force(vec![SubRef::face(2)], 500.0, ForceDirection::Vector([0.0, 0.0, -1.0]));
    a.pressure(vec![SubRef::face(6)], 2.0, false);
    a.self_weight([0.0, 0.0, -1.0]);
    a
}

pub fn snapshot_thermo_analysis() -> Analysis {
    let p = parity_dir();
    let mut a = Analysis::new(ShapeSource::Brep(p.join("snapshot_thermo.brep")), Material::steel_generic());
    a.analysis_type = AnalysisType::Thermomech { steady_state: true };
    a.mesh.max_size = 5.0;
    a.fixed(vec![SubRef::face(1)]);
    a.add(ConstraintKind::InitialTemperature { temperature: 293.0 }, vec![]);
    a.add(ConstraintKind::Temperature { temperature: 293.0 }, vec![SubRef::face(1)]);
    a.add(ConstraintKind::HeatFlux(HeatFluxKind::Flux(5000.0)), vec![SubRef::face(2)]);
    a.add(
        ConstraintKind::HeatFlux(HeatFluxKind::Convection { film_coefficient: 10.0, ambient_temperature: 293.0 }),
        vec![SubRef::face(6)],
    );
    a
}

/// Compare two texts line by line ignoring the footer lines that carry dates / program names.
fn assert_same_text(ours: &str, theirs: &str, what: &str) {
    let skip = |l: &&str| {
        l.starts_with("**   written by") || l.starts_with("**   written on") || l.starts_with("**   file name")
    };
    let a: Vec<&str> = ours.lines().filter(|l| !skip(l)).collect();
    let b: Vec<&str> = theirs.lines().filter(|l| !skip(l)).collect();
    for (i, (x, y)) in a.iter().zip(&b).enumerate() {
        assert_eq!(x, y, "{what}: first difference at (filtered) line {}", i + 1);
    }
    assert_eq!(a.len(), b.len(), "{what}: line count");
    assert_eq!(ours.ends_with('\n'), theirs.ends_with('\n'), "{what}: trailing newline");
}

fn geo_core(s: &str) -> Vec<String> {
    // FreeCAD writes its own temp file names
    // and the UNV-only SaveGroupsOfNodes; we save .msh 4.1 instead. Everything else must match.
    s.lines()
        .filter(|l| {
            !(l.starts_with("Merge ")
                || l.starts_with("Save ")
                || l.starts_with("// group data")
                || l.contains("SaveGroupsOfNodes")
                || l.starts_with("// For each group")
                || l.starts_with("// Needed for Group")
                || l.starts_with("Mesh.MshFileVersion")
                || l.starts_with("Mesh.Binary")
                || l.starts_with("// ")
                    && (l.contains("Gmsh documentation")
                        || l.contains("https://gmsh")
                        || l.contains("We do not check")
                        || l.contains("to see full")
                        || l.contains("to run Gmsh")
                        || l.contains("/gmsh ")
                        || l.contains("*****"))
                || l.trim() == "//")
        })
        .map(|l| l.to_string())
        .filter(|l| !l.is_empty())
        .collect()
}

#[test]
fn geo_matches_freecad() {
    for (case, a) in [("snapshot_static", snapshot_static_analysis()), ("snapshot_thermo", snapshot_thermo_analysis())] {
        let theirs = std::fs::read_to_string(parity_dir().join(format!("{case}.geo"))).unwrap();
        let brep = std::fs::read_to_string(parity_dir().join(format!("{case}.brep"))).unwrap();
        let ours = write_geo(&a.mesh, "shape.brep", "mesh.msh", &fc_fem::mesh::all_entity_groups(&brep));
        assert_eq!(geo_core(&ours), geo_core(&theirs), "{case}.geo");
    }
}

#[test]
fn geo_refinement_uses_points_of_faces() {
    let mut s = MeshSettings::default();
    s.refinements.push(MeshRefinement { refs: vec![SubRef::face(7)], size: 1.5 });
    let g = write_geo(&s, "a.brep", "m.msh", &[SubRef::face(2)]);
    assert!(g.contains("Characteristic Length { PointsOf { Surface{7}; } } = 1.5;"), "{g}");
    assert!(g.contains("Physical Surface(\"Face2\") = {2};"));
    assert!(g.contains("Mesh.CharacteristicLengthMax = 1e+22;"));
}

/// Mesh the reference BREP with the bundled Gmsh (single-threaded, same .geo settings as
/// FreeCAD) and write the input file: it must equal FreeCAD's Mesh.inp line for line.
fn inp_parity(case: &str, a: &Analysis) {
    if !tools_or_skip(case) {
        return;
    }
    let dir = std::env::temp_dir().join(format!("fc-fem-snapshot-{case}-{}", std::process::id()));
    let (mesh, areas, _) = mesh_analysis(a, &dir).unwrap();
    let inp = write_input(a, &mesh, areas.as_ref(), &dir, &InpOptions::default()).unwrap();
    let ours = std::fs::read_to_string(&inp).unwrap();
    let theirs = std::fs::read_to_string(parity_dir().join(format!("{case}.inp"))).unwrap();
    let _ = std::fs::remove_dir_all(&dir);
    assert_same_text(&ours, &theirs, case);
}

#[test]
fn inp_matches_freecad_static() {
    inp_parity("snapshot_static", &snapshot_static_analysis());
}

#[test]
fn inp_matches_freecad_thermomech() {
    inp_parity("snapshot_thermo", &snapshot_thermo_analysis());
}
