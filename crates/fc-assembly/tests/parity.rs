//! Parity with FreeCAD 1.1's Assembly workbench (Ondsel solver).
//!
//! `tools/parity/assembly/make_assemblies.py` builds the reference assemblies in
//! FreeCAD and records, per case, the parts, each joint's JCS (`Placement1/2`, local to
//! the part, offsets included), the referenced element geometry, the placements the final
//! solve started from, and FreeCAD's solved placements. Here we
//! 1. re-derive every JCS from the element geometry with our port of `findPlacement`, and
//! 2. solve the same joints from the same start and compare with FreeCAD's placements.
//! Determined cases (0 DOF) must match exactly; for mechanisms with free DOF both
//! solvers return *a* nearby solution, so we check the joints and report the difference.

use fc_assembly::*;
use serde_json::Value;

fn v3(v: &Value) -> [f64; 3] {
    [v[0].as_f64().unwrap(), v[1].as_f64().unwrap(), v[2].as_f64().unwrap()]
}
fn frame(v: &Value) -> Frame {
    let r = &v["rot"];
    Frame::new(v3(&v["pos"]), [r[0].as_f64().unwrap(), r[1].as_f64().unwrap(), r[2].as_f64().unwrap(), r[3].as_f64().unwrap()])
}
fn rot_axes(v: &Value) -> ([f64; 3], [f64; 3]) {
    let f = Frame::new([0.0; 3], [v[0].as_f64().unwrap(), v[1].as_f64().unwrap(), v[2].as_f64().unwrap(), v[3].as_f64().unwrap()]);
    (f.x_axis(), f.z_axis())
}

fn geom(v: &Value) -> Option<ElementGeom> {
    if v.is_null() {
        return None;
    }
    Some(match v["type"].as_str().unwrap() {
        "vertex" => ElementGeom::Vertex { point: v3(&v["point"]) },
        "line" => ElementGeom::Line { start: v3(&v["start"]), end: v3(&v["end"]) },
        "circle" => {
            let (x, _) = rot_axes(&v["rot"]);
            ElementGeom::Circle { center: v3(&v["center"]), axis: v3(&v["axis"]), radius: v["radius"].as_f64().unwrap(), xdir: Some(x) }
        }
        "plane" => {
            let (x, z) = rot_axes(&v["rot"]);
            ElementGeom::Plane { origin: v3(&v["origin"]), normal: z, xdir: x, cog: v3(&v["cog"]) }
        }
        "cylinder" => {
            let (x, z) = rot_axes(&v["rot"]);
            ElementGeom::Cylinder { origin: v3(&v["origin"]), axis: z, xdir: x, radius: v["radius"].as_f64().unwrap(), cog: v3(&v["cog"]) }
        }
        t => panic!("unhandled geometry {t}"),
    })
}

fn kind_of(j: &Value) -> JointKind {
    let ty = j["type"].as_str().unwrap();
    let d = j["distance"].as_f64().unwrap();
    match ty {
        "Fixed" => JointKind::Fastened,
        "Revolute" => JointKind::Revolute,
        "Cylindrical" => JointKind::Cylindrical,
        "Slider" => JointKind::Slider,
        "Ball" => JointKind::Ball,
        "Parallel" => JointKind::Parallel,
        "Perpendicular" => JointKind::Perpendicular,
        "Angle" => JointKind::Angle { angle: j["angle"].as_f64().unwrap() },
        "Distance" => {
            let t1 = j["geom1"]["type"].as_str().unwrap();
            let t2 = j["geom2"]["type"].as_str().unwrap();
            let mode = match (t1, t2) {
                ("vertex", "vertex") => DistanceMode::PointPoint,
                ("plane", "plane") => DistanceMode::PlanePlane,
                ("plane", "vertex") => DistanceMode::PointPlane,
                other => panic!("distance {other:?}"),
            };
            JointKind::Distance { distance: d, mode }
        }
        t => panic!("unhandled joint type {t}"),
    }
}

struct Row {
    case: String,
    dof: usize,
    worst_pos: f64,
    worst_rot_deg: f64,
}

#[test]
fn freecad_assembly_parity() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../tools/parity/assembly/assemblies.json");
    let cases: Vec<Value> = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    // Expected remaining DOF, and whether the case is determined (must match FreeCAD).
    let expect = |name: &str| match name {
        "hinge" => (1, false),
        "slider" => (1, false),
        "fourbar" => (1, false),
        "ball" => (3, false),
        "fixed_stack" => (0, true),
        "angle" => (0, true),
        "distance" => (2, false),
        "cylindrical" => (2, false),
        n => panic!("unknown case {n}"),
    };
    let mut rows = vec![];
    for c in &cases {
        let name = c["name"].as_str().unwrap();
        let mut asm = Assembly::new(name);
        let mut ids = std::collections::HashMap::new();
        for (k, p) in c["parts"].as_array().unwrap().iter().enumerate() {
            let n = p["name"].as_str().unwrap();
            let id = asm.add_part_instance(n, k as u32 + 1, frame(&c["start"][n]));
            asm.set_grounded(id, p["grounded"].as_bool().unwrap());
            ids.insert(n.to_string(), id);
        }
        for j in c["joints"].as_array().unwrap() {
            let kind = kind_of(j);
            // 1. JCS inference parity.
            let distance = matches!(kind, JointKind::Distance { .. });
            for (s, g, vtx, plc, off) in [("1", "geom1", "vtx1", "placement1", "offset1"), ("2", "geom2", "vtx2", "placement2", "offset2")] {
                let r = &j[format!("ref{s}")];
                let same = r[1] == r[2];
                let eg = geom(&j[g]).unwrap();
                let vg = if same { None } else { geom(&j[vtx]) };
                let ours = infer_frame(&eg, vg.as_ref(), distance).mul(&frame(&j[off]));
                let fc = frame(&j[plc]);
                assert!(ours.is_same(&fc, 1e-9), "{name} {} side {s}: ours {ours:?} FreeCAD {fc:?}", j["name"]);
            }
            // 2. Same joint on the recorded JCS.
            let ia = ids[j["ref1"][0].as_str().unwrap()];
            let ib = ids[j["ref2"][0].as_str().unwrap()];
            asm.add_joint(Joint::new(kind, JointRef::frame(ia, frame(&j["placement1"])), JointRef::frame(ib, frame(&j["placement2"]))));
        }
        let rep = asm.solve(&NoGeometry, &SolveOptions::default());
        assert!(rep.ok(), "{name}: {rep:?}");
        let (dof, determined) = expect(name);
        assert_eq!(rep.dof, dof, "{name}: {rep:?}");
        let (mut wp, mut wr) = (0.0f64, 0.0f64);
        for inst in &asm.instances {
            let fc = frame(&c["solved"][&inst.name]);
            let (dp, dr) = inst.placement.distance_to(&fc);
            wp = wp.max(dp);
            wr = wr.max(dr.to_degrees());
            if determined {
                assert!(dp < 1e-6 && dr < 1e-8, "{name}/{}: ours {:?} FreeCAD {fc:?}", inst.name, inst.placement);
            }
        }
        // Mechanisms whose free DOF Ondsel and we resolve the same way (pure projections)
        // must match too; hinge and 4-bar end at a slightly different angle of the free
        // rotation (documented in docs/assembly.md).
        if matches!(name, "slider" | "ball" | "distance" | "cylindrical") {
            assert!(wp < 1e-6 && wr < 0.01, "{name}: Δpos {wp} Δrot {wr}°");
        }
        rows.push(Row { case: name.to_string(), dof: rep.dof, worst_pos: wp, worst_rot_deg: wr });
    }
    eprintln!("case          DOF  max |Δpos| mm  max |Δrot| deg");
    for r in &rows {
        eprintln!("{:<13} {:>3}  {:>13.3e}  {:>14.3e}", r.case, r.dof, r.worst_pos, r.worst_rot_deg);
    }
}

/// The saved FreeCAD documents import (parts, grounding, joints on their JCS) and are
/// already solved for us: solving moves nothing and gives FreeCAD's DOF.
#[test]
fn freecad_fcstd_import() {
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../../tools/parity/assembly");
    let cases: Vec<Value> = serde_json::from_str(&std::fs::read_to_string(format!("{dir}/assemblies.json")).unwrap()).unwrap();
    for c in &cases {
        let name = c["name"].as_str().unwrap();
        let (mut ws, _notes) = fcstd::import(format!("{dir}/{name}.FCStd")).unwrap();
        assert_eq!(ws.assemblies.len(), 1);
        let asm = &ws.assemblies[0];
        assert_eq!(asm.instances.len(), c["parts"].as_array().unwrap().len(), "{name}");
        assert_eq!(asm.joints.len(), c["joints"].as_array().unwrap().len(), "{name}");
        for p in c["parts"].as_array().unwrap() {
            let inst = asm.instance_by_name(p["name"].as_str().unwrap()).unwrap();
            assert_eq!(inst.grounded, p["grounded"].as_bool().unwrap());
            assert!(inst.placement.is_same(&frame(&c["solved"][inst.name.as_str()]), 1e-9), "{name}/{}", inst.name);
        }
        let id = asm.id;
        let rep = ws.solve_assembly(id, &SolveOptions::default()).unwrap();
        assert!(rep.ok(), "{name}: {rep:?}");
        assert!(rep.residual < 1e-6, "{name}: {rep:?}");
        let moved = ws.assemblies[0].instances.iter().any(|i| !i.placement.is_same(&frame(&c["solved"][i.name.as_str()]), 1e-7));
        assert!(!moved, "{name}: {rep:?}");
    }
}
