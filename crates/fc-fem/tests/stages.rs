//! Stage tests: mesh parsing / classification, input writing from a stored mesh, result reading
//! from stored FreeCAD/ccx output. Only `write_snapshot_msh` (ignored) needs Gmsh.
mod common;

use common::*;
use fc_fem::mesh::{tet_volume_pub, TET_FACES};
use fc_fem::*;

#[path = "snapshot.rs"]
#[allow(dead_code)]
mod snapshot;

fn snapshot_mesh() -> FemMesh {
    parse_msh(&std::fs::read_to_string(parity_dir().join("snapshot_static.msh")).unwrap()).unwrap()
}

/// Regenerates tools/parity/fem/snapshot_static.msh with the bundled Gmsh.
#[test]
#[ignore]
fn write_snapshot_msh() {
    let a = snapshot::snapshot_static_analysis();
    let dir = std::env::temp_dir().join("fc-fem-write-msh");
    mesh_analysis(&a, &dir).unwrap();
    std::fs::copy(dir.join("mesh.msh"), parity_dir().join("snapshot_static.msh")).unwrap();
}

#[test]
fn msh_classification_and_node_order() {
    let m = snapshot_mesh();
    assert_eq!(m.order, 2);
    assert_eq!(m.node_count(), 622);
    assert_eq!(m.element_count(), 287);
    assert_eq!(m.surfaces.len(), 6);
    assert_eq!(m.curves.len(), 12);
    assert_eq!(m.points.len(), 8);
    // the tets fill the 100 x 20 x 20 box
    assert!((m.volume() - 40000.0).abs() < 1e-6);
    for e in &m.elements {
        let p: Vec<[f64; 3]> = e.nodes.iter().map(|&n| m.coord(n)).collect();
        // positive orientation in CalculiX order
        assert!(tet_volume_pub(p[0], p[1], p[2], p[3]) > 0.0, "element {} inverted", e.id);
        // C3D10 midside nodes: 5=(1,2) 6=(2,3) 7=(3,1) 8=(1,4) 9=(2,4) 10=(3,4)
        for (mid, (a, b)) in [(4, (0, 1)), (5, (1, 2)), (6, (2, 0)), (7, (0, 3)), (8, (1, 3)), (9, (2, 3))] {
            for k in 0..3 {
                assert!((p[mid][k] - 0.5 * (p[a][k] + p[b][k])).abs() < 1e-9, "element {} node {}", e.id, mid + 1);
            }
        }
    }
    // FreeCAD Box faces: Face1 x=0, Face2 x=100, Face3 y=0, Face4 y=20, Face5 z=0, Face6 z=20
    let s = m.signature(SubKind::Face, 2).unwrap();
    assert!((s.center[0] - 100.0).abs() < 1e-9 && (s.size - 400.0).abs() < 1e-9);
    assert!(s.normal.unwrap()[0].abs() > 0.999_999);
    for n in m.nodes_on(SubKind::Face, 6) {
        assert!((m.coord(n)[2] - 20.0).abs() < 1e-9);
    }
    let e = m.signature(SubKind::Edge, 1).unwrap();
    assert!(e.size > 0.0);
}

#[test]
fn boundary_is_closed_and_outward() {
    let m = snapshot_mesh();
    let s = m.boundary();
    // quadratic faces are split in 4: total = 4 x boundary faces
    assert_eq!(s.triangles.len(), s.tri_face.len());
    assert!(s.tri_face.iter().all(|&f| (1..=6).contains(&f)));
    // divergence theorem: sum over outward triangles of (p0 . n) A / 3 = volume
    let mut vol = 0.0;
    for t in &s.triangles {
        let p = t.map(|i| m.coords[i as usize]);
        vol += tet_volume_pub([0.0; 3], p[0], p[1], p[2]);
    }
    assert!((vol - 40000.0).abs() < 1e-6, "enclosed volume {vol}");
}

#[test]
fn signature_finds_renumbered_faces() {
    let m = snapshot_mesh();
    let real = m.signature(SubKind::Face, 4).unwrap();
    // claims to be Face1 but carries Face4's geometry
    let r = SubRef::face(1).with_signature(real.clone());
    assert_eq!(m.resolve(&r).unwrap(), 4);
    assert_eq!(m.resolve(&SubRef::face(4).with_signature(real)).unwrap(), 4);
    assert_eq!(m.resolve(&SubRef::face(3)).unwrap(), 3);
    assert!(m.resolve(&SubRef::face(9)).is_err());
    let bogus = Signature { center: [500.0, 0.0, 0.0], size: 1.0, normal: None };
    assert!(m.resolve(&SubRef::face(1).with_signature(bogus)).is_err());
}

#[test]
fn element_faces_lie_on_their_face() {
    let m = snapshot_mesh();
    let lookup = m.face_lookup();
    let faces = m.element_faces_on(6, &lookup);
    assert!(!faces.is_empty());
    let el: std::collections::HashMap<u32, &VolumeElement> = m.elements.iter().map(|e| (e.id, e)).collect();
    let mut area = 0.0;
    for (e, f) in faces {
        let c = TET_FACES[(f - 1) as usize];
        let p = c.map(|i| m.coord(el[&e].nodes[i]));
        assert!(p.iter().all(|q| (q[2] - 20.0).abs() < 1e-9));
        let u = [p[1][0] - p[0][0], p[1][1] - p[0][1], 0.0];
        let v = [p[2][0] - p[0][0], p[2][1] - p[0][1], 0.0];
        area += 0.5 * (u[0] * v[1] - u[1] * v[0]).abs();
    }
    assert!((area - 2000.0).abs() < 1e-6);
}

/// Input writer snapshot without any external tool: the stored Gmsh mesh written as .inp must
/// equal FreeCAD's Mesh.inp for the same analysis.
#[test]
fn inp_writer_snapshot() {
    let a = snapshot::snapshot_static_analysis();
    let m = snapshot_mesh();
    let r = resolve(&a, &m, None).unwrap();
    // force distribution sums to the total force
    if let ConstraintData::NodeLoads { tables, .. } = &r.data[2] {
        let s: f64 = tables.iter().flat_map(|t| t.1.values()).sum();
        assert!((s - 500.0).abs() < 1e-9);
    } else {
        panic!("force not resolved to node loads");
    }
    let ours = write_inp(&a, &m, &r, &InpOptions::default());
    let theirs = std::fs::read_to_string(parity_dir().join("snapshot_static.inp")).unwrap();
    let keep = |l: &&str| !l.starts_with("**   written") && !l.starts_with("**   file name");
    let x: Vec<&str> = ours.lines().filter(keep).collect();
    let y: Vec<&str> = theirs.lines().filter(keep).collect();
    for (i, (p, q)) in x.iter().zip(&y).enumerate() {
        assert_eq!(p, q, "line {}", i + 1);
    }
    assert_eq!(x.len(), y.len());
}

#[test]
fn frd_and_dat_parsing() {
    let m = snapshot_mesh();
    let j = reference("snapshot_static");
    let frd = parse_frd(&std::fs::read_to_string(parity_dir().join("snapshot_static.frd")).unwrap()).unwrap();
    assert_eq!(frd.nodes.len(), 622);
    let names: Vec<&str> = frd.blocks.iter().map(|b| b.name.as_str()).collect();
    assert_eq!(names, ["DISP", "STRESS", "TOSTRAIN", "ERROR"]);
    assert_eq!(frd.last("STRESS").unwrap().components, ["SXX", "SYY", "SZZ", "SXY", "SYZ", "SZX"]);
    for (id, p) in &frd.nodes {
        let q = m.coord(*id);
        assert!((0..3).all(|k| (p[k] - q[k]).abs() < 1e-3 * (1.0 + q[k].abs())));
    }
    let dat = std::fs::read_to_string(parity_dir().join("snapshot_static.dat")).unwrap();
    let totals = parse_dat_totals(&dat);
    assert_eq!(totals.keys().collect::<Vec<_>>(), ["CONSTRAINTDISPLACEMENT", "CONSTRAINTFIXED"]);
    let r = Results::from_frd(&m, &frd, totals, &[]).unwrap();
    let d = r.max_displacement().unwrap();
    assert!(rel(d.value, j["max_disp"].as_f64().unwrap()) < 1e-6);
    assert_eq!(d.node as u64, j["max_disp_node"].as_u64().unwrap());
    let vm = r.max_von_mises().unwrap();
    assert!(rel(vm.value, j["max_von_mises"].as_f64().unwrap()) < 1e-6, "{}", vm.value);
    assert!(rel(r.max(Field::PrincipalMax).unwrap().value, j["max_principal"].as_f64().unwrap()) < 1e-6);
    assert!(rel(r.min(Field::PrincipalMin).unwrap().value, j["min_principal"].as_f64().unwrap()) < 1e-6);
    let fixed = r.reactions["CONSTRAINTFIXED"];
    assert!((fixed[2] - j["reactions"]["CONSTRAINTFIXED"][2].as_f64().unwrap()).abs() < 1e-3);
    // results API
    let def = r.deformed(10.0);
    let i = d.index;
    assert!((def[i][2] - (r.coords[i][2] + 10.0 * r.displacement[i][2])).abs() < 1e-12);
    let (pos, tris, scal) = r.surface_mesh(Field::VonMises, 1.0);
    assert_eq!(pos.len(), 622);
    assert_eq!(scal.len(), 622);
    assert!(!tris.is_empty());
    assert!(r.auto_scale(0.1) > 1.0);
}

#[test]
fn frd_thermomech_has_temperatures() {
    let frd = parse_frd(&std::fs::read_to_string(parity_dir().join("snapshot_thermo.frd")).unwrap()).unwrap();
    let t = frd.last("NDTEMP").unwrap();
    let mx = t.values.values().map(|v| v[0]).fold(f64::MIN, f64::max);
    let j = reference("snapshot_thermo");
    assert!(rel(mx, j["max_temperature"].as_f64().unwrap()) < 1e-5);
    assert!(frd.last("FLUX").is_some());
}

#[test]
fn dat_eigenvalues() {
    let dat = "                        S T E P       1\n\n\n     E I G E N V A L U E   O U T P U T\n\n MODE NO    EIGENVALUE                       FREQUENCY   \n                                     REAL PART            IMAGINARY PART\n                           (RAD/TIME)      (CYCLES/TIME     (RAD/TIME)\n\n      1   0.2594603E+06   0.5093725E+03   0.8106915E+02   0.0000000E+00\n      2   0.2594836E+06   0.5093953E+03   0.8107278E+02   0.0000000E+00\n\n     P A R T I C I P A T I O N   F A C T O R S\n";
    assert_eq!(parse_dat_frequencies(dat), vec![81.06915, 81.07278]);
}

#[test]
fn analysis_json_roundtrip() {
    let a = snapshot::snapshot_thermo_analysis();
    let b = Analysis::from_json(&a.to_json()).unwrap();
    assert_eq!(b.constraints, a.constraints);
    assert_eq!(b.mesh, a.mesh);
    assert_eq!(b.analysis_type, a.analysis_type);
    assert_eq!(b.constraints[1].name, "ConstraintInitialTemperature");
    assert_eq!(b.constraints[3].name, "ConstraintHeatflux");
    assert_eq!(b.constraints[4].name, "ConstraintHeatflux001");
}
