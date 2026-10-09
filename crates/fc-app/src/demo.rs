//! Procedural demo scene: a 40x30x20 box with a through hole along Z.
use crate::viewport::{SceneEdge, SceneMesh, SceneObject};

pub fn box_with_hole() -> SceneObject {
    let (hw, hh, h, r) = (20.0f32, 15.0f32, 20.0f32, 7.0f32);
    let mut m = SceneMesh { positions: vec![], normals: vec![], indices: vec![], tri_face: vec![] };

    // angles around the hole, including the rectangle's corner angles
    let n = 72;
    let mut angles: Vec<f32> = (0..n).map(|i| i as f32 / n as f32 * std::f32::consts::TAU).collect();
    for (sx, sy) in [(1.0f32, 1.0f32), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)] {
        let a = (sy * hh).atan2(sx * hw);
        angles.push(if a < 0.0 { a + std::f32::consts::TAU } else { a });
    }
    angles.sort_by(|a, b| a.partial_cmp(b).unwrap());
    angles.dedup_by(|a, b| (*a - *b).abs() < 1e-5);
    let outer = |t: f32| -> [f32; 2] {
        let (c, s) = (t.cos(), t.sin());
        let k = (hw / c.abs().max(1e-6)).min(hh / s.abs().max(1e-6));
        [c * k, s * k]
    };
    let inner = |t: f32| -> [f32; 2] { [t.cos() * r, t.sin() * r] };

    let push = |m: &mut SceneMesh, p: [f32; 3], nrm: [f32; 3]| -> u32 {
        m.positions.push(p);
        m.normals.push(nrm);
        (m.positions.len() - 1) as u32
    };
    let tri = |m: &mut SceneMesh, a: u32, b: u32, c: u32, f: u32| {
        m.indices.extend([a, b, c]);
        m.tri_face.push(f);
    };

    // top (face 0, +Z) and bottom (face 1, -Z) with hole
    for (face, z, nz) in [(0u32, h, 1.0f32), (1, 0.0, -1.0)] {
        let nrm = [0.0, 0.0, nz];
        let k = angles.len();
        let o: Vec<u32> = angles.iter().map(|&t| { let p = outer(t); push(&mut m, [p[0], p[1], z], nrm) }).collect();
        let i: Vec<u32> = angles.iter().map(|&t| { let p = inner(t); push(&mut m, [p[0], p[1], z], nrm) }).collect();
        for j in 0..k {
            let j2 = (j + 1) % k;
            if nz > 0.0 {
                tri(&mut m, i[j], o[j], o[j2], face);
                tri(&mut m, i[j], o[j2], i[j2], face);
            } else {
                tri(&mut m, i[j], o[j2], o[j], face);
                tri(&mut m, i[j], i[j2], o[j2], face);
            }
        }
    }
    // four side faces (2..=5)
    let c = [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]];
    let sn = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]];
    for s in 0..4 {
        let (a, b) = (c[s], c[(s + 1) % 4]);
        let v0 = push(&mut m, [a[0], a[1], 0.0], sn[s]);
        let v1 = push(&mut m, [b[0], b[1], 0.0], sn[s]);
        let v2 = push(&mut m, [b[0], b[1], h], sn[s]);
        let v3 = push(&mut m, [a[0], a[1], h], sn[s]);
        tri(&mut m, v0, v1, v2, 2 + s as u32);
        tri(&mut m, v0, v2, v3, 2 + s as u32);
    }
    // hole cylinder (face 6), normals point into the hole
    let nc = 96;
    let mut ring = vec![];
    for j in 0..=nc {
        let t = j as f32 / nc as f32 * std::f32::consts::TAU;
        let nrm = [-t.cos(), -t.sin(), 0.0];
        let a = push(&mut m, [t.cos() * r, t.sin() * r, 0.0], nrm);
        let b = push(&mut m, [t.cos() * r, t.sin() * r, h], nrm);
        ring.push((a, b));
    }
    for j in 0..nc {
        let (a0, b0) = ring[j];
        let (a1, b1) = ring[j + 1];
        tri(&mut m, a0, b1, a1, 6);
        tri(&mut m, a0, b0, b1, 6);
    }

    // B-rep edges
    let mut edges = vec![];
    let mut id = 0;
    let mut add = |pts: Vec<[f32; 3]>| {
        edges.push(SceneEdge { id, points: pts });
        id += 1;
    };
    for z in [h, 0.0] {
        for s in 0..4 {
            let (a, b) = (c[s], c[(s + 1) % 4]);
            add(vec![[a[0], a[1], z], [b[0], b[1], z]]);
        }
    }
    for s in 0..4 {
        add(vec![[c[s][0], c[s][1], 0.0], [c[s][0], c[s][1], h]]);
    }
    for z in [h, 0.0] {
        add((0..=nc).map(|j| { let t = j as f32 / nc as f32 * std::f32::consts::TAU; [t.cos() * r, t.sin() * r, z] }).collect());
    }
    SceneObject { name: "Body".into(), mesh: m, edges, visible: true, color: [0.66, 0.70, 0.76, 1.0] }
}
