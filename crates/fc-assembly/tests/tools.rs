//! Assembly tools: structured STEP export/import, STL, interference/clearance, BOM,
//! exploded views and sub-assemblies.

use fc_assembly::*;
use fc_occt::{ProfileBuilder, Shape};

fn boxs(l: f64, w: f64, h: f64) -> Shape {
    let mut b = ProfileBuilder::new();
    b.begin_face();
    let p = [[0.0, 0.0, 0.0], [l, 0.0, 0.0], [l, w, 0.0], [0.0, w, 0.0]];
    for i in 0..4 {
        b.line(p[i], p[(i + 1) % 4]).unwrap();
    }
    b.end_wire().unwrap();
    b.build_faces().unwrap().prism([0.0, 0.0, h]).unwrap()
}

fn near(a: [f64; 3], b: [f64; 3], tol: f64) -> bool {
    (0..3).all(|k| (a[k] - b[k]).abs() < tol)
}

fn tmp(name: &str) -> String {
    let d = std::env::temp_dir().join("fc_assembly_tools");
    std::fs::create_dir_all(&d).unwrap();
    d.join(name).to_str().unwrap().to_string()
}

struct Fixture {
    ws: Workspace,
    main: AssemblyId,
    pair: AssemblyId,
    plate: PartId,
    bolt: PartId,
}

/// Main: a 100x50x5 plate (grounded), two "BoltPair" sub-assemblies (two 5x5x20 bolts
/// each), one rotated 90° about Z, and one loose bolt.
fn fixture() -> Fixture {
    let mut ws = Workspace::new();
    let plate = ws.add_imported_part("Plate", boxs(100.0, 50.0, 5.0));
    let bolt = ws.add_imported_part("Bolt", boxs(5.0, 5.0, 20.0));
    {
        let p = ws.part_mut(plate).unwrap();
        p.meta = PartMeta { part_number: "PL-001".into(), description: "Base plate, 5 mm".into(), material: "Steel".into(), density: Some(7850.0), color: Some([0.2, 0.4, 0.8]), ..Default::default() };
        let b = ws.part_mut(bolt).unwrap();
        b.meta = PartMeta { part_number: "B-M5".into(), description: "Bolt".into(), material: "Steel".into(), mass: Some(0.004), ..Default::default() };
    }
    let pair = ws.add_assembly("BoltPair");
    {
        let a = ws.assembly_mut(pair).unwrap();
        a.add_part_instance("Bolt", bolt, Frame::IDENTITY);
        a.add_part_instance("Bolt", bolt, Frame::translation([10.0, 0.0, 0.0]));
    }
    let main = ws.add_assembly("Main");
    let g = ws.assembly_mut(main).unwrap().add_part_instance("Plate", plate, Frame::IDENTITY);
    ws.assembly_mut(main).unwrap().set_grounded(g, true);
    ws.add_sub_assembly(main, "PairA", pair, Frame::translation([10.0, 10.0, 5.0])).unwrap();
    ws.add_sub_assembly(main, "PairB", pair, Frame::new([80.0, 10.0, 5.0], fc_assembly::frame::quat_axis_angle([0.0, 0.0, 1.0], std::f64::consts::FRAC_PI_2))).unwrap();
    ws.assembly_mut(main).unwrap().add_part_instance("Loose", bolt, Frame::translation([50.0, 40.0, 5.0]));
    Fixture { ws, main, pair, plate, bolt }
}

#[test]
fn sub_assembly_structure() {
    let mut f = fixture();
    let leaves = f.ws.leaves(f.main);
    let names: Vec<&str> = leaves.iter().map(|l| l.name.as_str()).collect();
    assert_eq!(names, ["Plate", "PairA.Bolt", "PairA.Bolt001", "PairB.Bolt", "PairB.Bolt001", "Loose"]);
    // PairB is turned 90° about Z: its second bolt (x = 10 inside) lands at y = +10.
    let l = leaves.iter().find(|l| l.name == "PairB.Bolt001").unwrap();
    assert!(near(l.placement.pos, [80.0, 20.0, 5.0], 1e-12), "{:?}", l.placement);
    let s = f.ws.instance_shape(f.main, &l.path).unwrap();
    let b = s.bbox_exact();
    assert!(near([b[0], b[1], b[2]], [75.0, 20.0, 5.0], 1e-9) && near([b[3], b[4], b[5]], [80.0, 25.0, 25.0], 1e-9), "{b:?}");
    // A sub-assembly instance's shape is the compound of its parts.
    let pair_a = f.ws.assembly(f.main).unwrap().instance_by_name("PairA").unwrap().id;
    let s = f.ws.instance_shape(f.main, &[pair_a]).unwrap();
    assert_eq!(s.solid_count(), 2);
    assert!((s.volume() - 1000.0).abs() < 1e-9);
    // Cycles are refused.
    let r = f.ws.add_sub_assembly(f.pair, "Main", f.main, Frame::IDENTITY);
    assert!(matches!(r, Err(WorkspaceError::Cycle(_))), "{r:?}");
    assert!(f.ws.add_sub_assembly(f.pair, "Self", f.pair, Frame::IDENTITY).is_err());
    assert!(f.ws.uses_assembly(f.main, f.pair) && !f.ws.uses_assembly(f.pair, f.main));
    // The workspace (with imported BREP parts and sub-assembly instances) round-trips.
    let back = Workspace::from_json(&f.ws.to_json()).unwrap();
    assert_eq!(back.leaves(f.main), leaves);
    assert!((back.part(f.plate).unwrap().volume() - 25000.0).abs() < 1e-9);
    assert_eq!(back.part(f.bolt).unwrap().meta.part_number, "B-M5");
}

#[test]
fn joints_reference_geometry_inside_sub_assemblies() {
    let mut f = fixture();
    // Put a lid on PairA's first bolt: Fastened between the lid's bottom face and the bolt's
    // top face, picked through the sub-assembly ("Bolt.Face<n>").
    let lid = f.ws.add_imported_part("Lid", boxs(20.0, 20.0, 2.0));
    let top_face = |s: &Shape, z: f64| (1..=s.face_count()).find(|&i| s.face_info(i).unwrap().normal[2] * z > 0.99).map(|i| format!("Face{i}")).unwrap();
    let bolt_top = top_face(f.ws.part_shape(f.bolt).unwrap(), 1.0);
    let lid_bottom = top_face(f.ws.part_shape(lid).unwrap(), -1.0);
    let asm = f.ws.assembly_mut(f.main).unwrap();
    let pair_a = asm.instance_by_name("PairA").unwrap().id;
    asm.set_grounded(pair_a, true);
    let l = asm.add_part_instance("Lid", lid, Frame::from_ypr([3.0, 4.0, 60.0], 20.0, 3.0, -2.0));
    asm.add_joint(Joint::new(JointKind::Fastened, JointRef::geometry(pair_a, &format!("Bolt.{bolt_top}")), JointRef::geometry(l, &lid_bottom)));
    let rep = f.ws.solve_assembly(f.main, &SolveOptions::default()).unwrap();
    assert!(rep.ok(), "{rep:?}");
    // The lid's bottom face centre sits on the bolt's top face centre: (12.5, 12.5, 25).
    let pl = f.ws.assembly(f.main).unwrap().instance(l).unwrap().placement;
    let c = pl.transform_point([10.0, 10.0, 0.0]);
    assert!(near(c, [12.5, 12.5, 25.0], 1e-7), "{c:?}");
    assert!(pl.z_axis()[2] < -0.999 || pl.z_axis()[2] > 0.999);
    // A bad path is reported, not panicked on.
    let asm = f.ws.assembly_mut(f.main).unwrap();
    asm.add_joint(Joint::new(JointKind::Fastened, JointRef::geometry(pair_a, "Nope.Face1"), JointRef::geometry(l, &lid_bottom)));
    let rep = f.ws.solve_assembly(f.main, &SolveOptions::default()).unwrap();
    assert_eq!(rep.unresolved_joints.len(), 1);
    assert!(rep.unresolved_joints[0].1.contains("no instance Nope"), "{rep:?}");
}

#[test]
fn step_export_and_import_keep_the_structure() {
    let f = fixture();
    for (schema, file) in [(StepSchema::Ap214, "main_ap214.step"), (StepSchema::Ap242, "main_ap242.step")] {
        let path = tmp(file);
        let st = f.ws.export_step(f.main, &path, &StepExportOptions { schema, ..Default::default() }).unwrap();
        assert_eq!((st.parts, st.assemblies, st.components), (2, 2, 6), "{st:?}");
        let text = std::fs::read_to_string(&path).unwrap();
        assert!(text.contains(if schema == StepSchema::Ap242 { "AP242" } else { "AUTOMOTIVE_DESIGN" }));
        assert_eq!(text.matches("NEXT_ASSEMBLY_USAGE_OCCURRENCE").count(), 6);

        let mut ws = Workspace::new();
        let imp = ws.import_step(&path).unwrap();
        assert_eq!(imp.parts.len(), 2);
        assert_eq!(imp.sub_assemblies.len(), 1);
        let main = ws.assembly(imp.assembly).unwrap();
        assert_eq!(main.name, "Main");
        let names: Vec<&str> = main.instances.iter().map(|i| i.name.as_str()).collect();
        assert_eq!(names, ["Plate", "PairA", "PairB", "Loose"]);
        let pair = ws.assembly(imp.sub_assemblies[0]).unwrap();
        assert_eq!(pair.name, "BoltPair");
        assert_eq!(pair.instances.len(), 2);
        // Same leaves, same placements, same solids.
        let a = f.ws.leaves(f.main);
        let b = ws.leaves(imp.assembly);
        assert_eq!(a.len(), b.len());
        for (x, y) in a.iter().zip(&b) {
            assert_eq!(x.name, y.name);
            assert!(x.placement.is_same(&y.placement, 1e-9), "{}: {:?} vs {:?}", x.name, x.placement, y.placement);
            assert_eq!(f.ws.part(x.part).unwrap().name, ws.part(y.part).unwrap().name);
            let (vx, vy) = (f.ws.leaf_shape(x).unwrap(), ws.leaf_shape(y).unwrap());
            assert!((vx.volume() - vy.volume()).abs() < 1e-6);
            let (bx, by) = (vx.bbox_exact(), vy.bbox_exact());
            assert!((0..6).all(|k| (bx[k] - by[k]).abs() < 1e-6), "{bx:?} {by:?}");
        }
        let plate = ws.parts.iter().find(|p| p.name == "Plate").unwrap();
        let c = plate.meta.color.unwrap();
        assert!(near(c, [0.2, 0.4, 0.8], 1e-3), "{c:?}");
    }
}

#[test]
fn step_import_of_a_single_part_wraps_it() {
    let path = tmp("single.step");
    boxs(1.0, 2.0, 3.0).write_step(&path).unwrap();
    let mut ws = Workspace::new();
    let imp = ws.import_step(&path).unwrap();
    let a = ws.assembly(imp.assembly).unwrap();
    assert_eq!(a.name, "single");
    assert_eq!(a.instances.len(), 1);
    assert!((ws.part(imp.parts[0]).unwrap().volume() - 6.0).abs() < 1e-9);
}

#[test]
fn stl_export_of_the_posed_assembly() {
    let f = fixture();
    let path = tmp("main.stl");
    assert_eq!(f.ws.export_stl(f.main, &path, 0.1, false).unwrap(), 6);
    let bytes = std::fs::read(&path).unwrap();
    let n = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
    assert_eq!(bytes.len(), 84 + 50 * n);
    assert_eq!(n, 6 * 12, "six boxes, 12 triangles each");
    // Every vertex lies inside the posed assembly's box [0,100]x[0,50]x[0,25].
    for t in 0..n {
        for v in 0..3 {
            let o = 84 + 50 * t + 12 + 12 * v;
            let p: Vec<f32> = (0..3).map(|k| f32::from_le_bytes(bytes[o + 4 * k..o + 4 * k + 4].try_into().unwrap())).collect();
            assert!(p[0] >= -1e-3 && p[0] <= 100.001 && p[1] >= -1e-3 && p[1] <= 50.001 && p[2] >= -1e-3 && p[2] <= 25.001, "{p:?}");
        }
    }
}

#[test]
fn interference_and_clearance() {
    let mut ws = Workspace::new();
    let cube = ws.add_imported_part("Cube", boxs(10.0, 10.0, 10.0));
    let pair = ws.add_assembly("Pair");
    {
        let a = ws.assembly_mut(pair).unwrap();
        a.add_part_instance("P1", cube, Frame::IDENTITY);
        a.add_part_instance("P2", cube, Frame::translation([8.0, 0.0, 0.0])); // overlaps P1: 2x10x10
    }
    let id = ws.add_assembly("Main");
    let a = ws.assembly_mut(id).unwrap();
    let c0 = a.add_part_instance("A", cube, Frame::IDENTITY);
    let c1 = a.add_part_instance("B", cube, Frame::translation([5.0, 5.0, 5.0])); // overlaps A: 5³
    let c2 = a.add_part_instance("C", cube, Frame::translation([0.0, 0.0, -10.0])); // touches A face to face
    let c3 = a.add_part_instance("D", cube, Frame::new([30.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])); // 20 mm from A
    let s = ws.add_sub_assembly(id, "S", pair, Frame::translation([100.0, 0.0, 0.0])).unwrap();
    let hits = ws.interferences(id, &InterferenceOptions::default());
    let pairs: Vec<(String, String, f64)> = hits.iter().map(|h| (h.a.name.clone(), h.b.name.clone(), h.volume)).collect();
    assert_eq!(pairs.len(), 2, "{pairs:?}");
    assert_eq!((pairs[0].0.as_str(), pairs[0].1.as_str()), ("A", "B"));
    assert!((pairs[0].2 - 125.0).abs() < 1e-6);
    assert_eq!((pairs[1].0.as_str(), pairs[1].1.as_str()), ("S.P1", "S.P2"));
    assert!((pairs[1].2 - 200.0).abs() < 1e-6);
    // The intersection shape is in assembly coordinates (for highlighting).
    let b = hits[1].shape.bbox_exact();
    assert!(near([b[0], b[3], b[5]], [108.0, 110.0, 10.0], 1e-9), "{b:?}");
    // A bigger volume threshold drops both.
    assert!(ws.interferences(id, &InterferenceOptions { min_volume: 500.0, ..Default::default() }).is_empty());

    let d = ws.clearance(id, &[c0], &[c3]).unwrap();
    assert!((d.value - 20.0).abs() < 1e-9);
    assert!((d.on_a[0] - 10.0).abs() < 1e-9 && (d.on_b[0] - 30.0).abs() < 1e-9);
    assert!(ws.clearance(id, &[c0], &[c2]).unwrap().value < 1e-9);
    assert!(ws.clearance(id, &[c0], &[c1]).unwrap().value < 1e-9);
    // Clearance to a whole sub-assembly instance, and to one part inside it.
    assert!((ws.clearance(id, &[c3], &[s]).unwrap().value - 60.0).abs() < 1e-9);
    let p2 = ws.assembly(pair).unwrap().instance_by_name("P2").unwrap().id;
    assert!((ws.clearance(id, &[c3], &[s, p2]).unwrap().value - 68.0).abs() < 1e-9);
}

#[test]
fn bill_of_materials() {
    let f = fixture();
    let rows = f.ws.bom(f.main, &BomOptions::default());
    let summary: Vec<(String, String, u32, usize)> = rows.iter().map(|r| (r.index.clone(), r.name.clone(), r.quantity, r.level)).collect();
    assert_eq!(
        summary,
        [("1".into(), "Plate".into(), 1, 0), ("2".into(), "BoltPair".into(), 2, 0), ("2.1".into(), "Bolt".into(), 2, 1), ("3".into(), "Bolt".into(), 1, 0)]
    );
    let plate = &rows[0];
    assert_eq!(plate.part_number, "PL-001");
    // 100x50x5 mm³ of steel at 7850 kg/m³ = 0.19625 kg.
    assert!((plate.unit_mass.unwrap() - 0.19625).abs() < 1e-9);
    assert!((rows[1].unit_mass.unwrap() - 0.008).abs() < 1e-12);
    assert!((rows[1].total_mass().unwrap() - 0.016).abs() < 1e-12);

    let flat = f.ws.bom(f.main, &BomOptions { flatten: true, ..Default::default() });
    let s: Vec<(&str, u32)> = flat.iter().map(|r| (r.name.as_str(), r.quantity)).collect();
    assert_eq!(s, [("Plate", 1), ("Bolt", 5)]);
    let short = f.ws.bom(f.main, &BomOptions { detail_sub_assemblies: false, ..Default::default() });
    assert_eq!(short.len(), 3);

    let csv = bom_csv(&rows);
    let lines: Vec<&str> = csv.lines().collect();
    assert_eq!(lines[0], "Index,Name,Part Number,Description,Material,Quantity,Unit Mass (kg),Total Mass (kg)");
    assert_eq!(lines[1], "1,Plate,PL-001,\"Base plate, 5 mm\",Steel,1,0.196250,0.196250");
    assert_eq!(lines[3], "2.1,Bolt,B-M5,Bolt,Steel,2,0.004000,0.008000");
    assert_eq!(lines.len(), 5);
}

#[test]
fn exploded_views() {
    let mut f = fixture();
    let solved: Vec<Frame> = f.ws.assembly(f.main).unwrap().instances.iter().map(|i| i.placement).collect();
    let (pair_a, loose, plate) = {
        let a = f.ws.assembly(f.main).unwrap();
        (a.instance_by_name("PairA").unwrap().id, a.instance_by_name("Loose").unwrap().id, a.instance_by_name("Plate").unwrap().id)
    };
    let asm = f.ws.assembly_mut(f.main).unwrap();
    asm.exploded_views.push(ExplodedView {
        name: "Exploded".into(),
        steps: vec![
            ExplodeStep::translate(vec![pair_a, loose], [0.0, 0.0, 2.0], 30.0),
            ExplodeStep::rotate(vec![loose], [50.0, 40.0, 0.0], [0.0, 0.0, 1.0], 90.0),
        ],
    });
    let pose = |ws: &Workspace, t: f64| ws.exploded(f.main, 0, t).unwrap();
    let e0 = pose(&f.ws, 0.0);
    assert!(e0.lines.is_empty());
    for (i, s) in f.ws.assembly(f.main).unwrap().instances.iter().zip(&solved) {
        assert_eq!(e0.placements[&i.id], *s);
    }
    // Half way: step 1 done, step 2 not started. A quarter: half of step 1.
    let e = pose(&f.ws, 0.25);
    assert!(near(e.placements[&pair_a].pos, [10.0, 10.0, 20.0], 1e-12));
    let e = pose(&f.ws, 0.5);
    assert!(near(e.placements[&pair_a].pos, [10.0, 10.0, 35.0], 1e-12));
    assert!(near(e.placements[&loose].pos, [50.0, 40.0, 35.0], 1e-12));
    assert_eq!(e.lines.len(), 2);
    // The loose bolt's centre (52.5, 42.5) turns 90° about (50, 40): to (47.5, 42.5).
    let e = pose(&f.ws, 1.0);
    let c = e.placements[&loose].transform_point([2.5, 2.5, 10.0]);
    assert!(near(c, [47.5, 42.5, 45.0], 1e-9), "{c:?}");
    // At 3/4 it has turned 45° about the same axis (screw interpolation keeps the pivot).
    let e = pose(&f.ws, 0.75);
    let c = e.placements[&loose].transform_point([2.5, 2.5, 10.0]);
    assert!(near(c, [50.0, 40.0 + 2.5 * 2f64.sqrt(), 45.0], 1e-9), "{c:?}");
    assert_eq!(e.placements[&plate], solved[0]);
    // Non-destructive.
    let now: Vec<Frame> = f.ws.assembly(f.main).unwrap().instances.iter().map(|i| i.placement).collect();
    assert_eq!(now, solved);

    // Radial: everything moves away from the assembly's bbox centre (50, 25, 12.5),
    // by (centre - com) * 4 * d / diagonal.
    let asm = f.ws.assembly_mut(f.main).unwrap();
    asm.exploded_views.push(ExplodedView { name: "Radial".into(), steps: vec![ExplodeStep::radial(vec![loose, plate], 30.0)] });
    let e = f.ws.exploded(f.main, 1, 1.0).unwrap();
    let diag = (100.0f64 * 100.0 + 50.0 * 50.0 + 25.0 * 25.0).sqrt();
    let k = 4.0 * 30.0 / diag;
    let lc = [52.5, 42.5, 15.0];
    let want = [50.0 + (lc[0] - 50.0) * k, 40.0 + (lc[1] - 25.0) * k, 5.0 + (lc[2] - 12.5) * k];
    assert!(near(e.placements[&loose].pos, want, 1e-9), "{:?} {want:?}", e.placements[&loose].pos);
    // The plate's centre (50, 25, 2.5) moves straight down.
    let p = e.placements[&plate].pos;
    assert!(near(p, [0.0, 0.0, -10.0 * k], 1e-9), "{p:?}");
    // The geometry-free variant still works (origins stand in for centres).
    let pl = f.ws.assembly(f.main).unwrap().exploded_placements(0, 1.0);
    assert!(near(pl[&pair_a].pos, [10.0, 10.0, 35.0], 1e-12));
}

#[test]
fn screw_interpolation() {
    let t = Frame::from_ypr([3.0, -4.0, 7.0], 70.0, 20.0, -35.0);
    assert!(t.pow(1.0).is_same(&t, 1e-12));
    assert!(t.pow(0.0).is_same(&Frame::IDENTITY, 1e-12));
    let h = t.pow(0.5);
    assert!(h.mul(&h).is_same(&t, 1e-12));
    let q = t.pow(0.25);
    assert!(q.mul(&q).mul(&q).mul(&q).is_same(&t, 1e-12));
    let m = Frame::from_matrix34(&t.to_matrix34());
    assert!(m.is_same(&t, 1e-14));
}

#[test]
fn freecad_step_import() {
    // Written by FreeCAD 1.1.3 (tools/parity/assembly/step_assembly.py): App::Part "Top" with
    // a 60x40x4 box and two links to App::Part "PinPair" (two links to one r3 h15 cylinder),
    // Pair2 turned 90° about Z.
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../tools/parity/assembly/freecad_asm.step");
    let mut ws = Workspace::new();
    let imp = ws.import_step(path).unwrap();
    let top = ws.assembly(imp.assembly).unwrap();
    assert_eq!(top.name, "Top");
    let names: Vec<&str> = top.instances.iter().map(|i| i.name.as_str()).collect();
    assert_eq!(names, ["Plate", "Pair1", "Pair2"]);
    assert_eq!(imp.sub_assemblies.len(), 1);
    let pair = ws.assembly(imp.sub_assemblies[0]).unwrap();
    assert_eq!(pair.name, "PinPair");
    assert_eq!(pair.instances.iter().map(|i| i.name.as_str()).collect::<Vec<_>>(), ["PinA", "PinB"]);
    // One shared Pin part, one Plate.
    assert_eq!(imp.parts.len(), 2);
    let leaves = ws.leaves(imp.assembly);
    let pin_v = std::f64::consts::PI * 9.0 * 15.0;
    let want: [(&str, f64, [f64; 6]); 5] = [
        ("Plate", 9600.0, [0.0, 0.0, 0.0, 60.0, 40.0, 4.0]),
        ("Pair1.PinA", pin_v, [7.0, 7.0, 4.0, 13.0, 13.0, 19.0]),
        ("Pair1.PinB", pin_v, [19.0, 7.0, 4.0, 25.0, 13.0, 19.0]),
        ("Pair2.PinA", pin_v, [42.0, 7.0, 4.0, 48.0, 13.0, 19.0]),
        ("Pair2.PinB", pin_v, [42.0, 19.0, 4.0, 48.0, 25.0, 19.0]),
    ];
    assert_eq!(leaves.len(), want.len());
    for (l, (name, v, bb)) in leaves.iter().zip(want) {
        assert_eq!(l.name, name);
        let s = ws.leaf_shape(l).unwrap();
        assert!((s.volume() - v).abs() < 1e-6, "{name}: {}", s.volume());
        let b = s.bbox_exact();
        assert!((0..6).all(|k| (b[k] - bb[k]).abs() < 1e-6), "{name}: {b:?}");
    }
    let rows = ws.bom(imp.assembly, &BomOptions { flatten: true, ..Default::default() });
    assert_eq!(rows.iter().map(|r| (r.name.as_str(), r.quantity)).collect::<Vec<_>>(), [("Plate", 1), ("Pin", 4)]);
    // Pins of Pair1 and the plate touch (pins stand on it) but do not interfere.
    assert!(ws.interferences(imp.assembly, &InterferenceOptions::default()).is_empty());
}
