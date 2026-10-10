//! Choosing where a new sketch goes, like FreeCAD's "Select feature" step: the body's
//! origin planes appear in the 3D view (and in a list), and a click on a plane or on a
//! flat face of the part attaches the sketch there.

use crate::viewport::{SceneEdge, SceneMesh, SceneObject};
use fc_part::{Placement, Support};

pub struct Plane {
    pub name: &'static str,
    pub hint: &'static str,
    pub support: Support,
    pub placement: Placement,
}

pub fn planes() -> [Plane; 3] {
    [
        Plane { name: "XY_Plane", hint: "Top view, sketch X = X, Y = Y", support: Support::XY, placement: Placement::XY },
        Plane { name: "XZ_Plane", hint: "Front view, sketch X = X, Y = Z", support: Support::XZ, placement: Placement::XZ },
        Plane { name: "YZ_Plane", hint: "Right view, sketch X = Y, Y = Z", support: Support::YZ, placement: Placement::YZ },
    ]
}

/// Translucent plane colors, by normal axis (XY: Z blue, XZ: Y green, YZ: X red).
pub fn plane_color(i: usize) -> [f32; 4] {
    match i {
        0 => [0.35, 0.55, 1.0, 0.24],
        1 => [0.35, 0.85, 0.45, 0.24],
        _ => [1.0, 0.42, 0.40, 0.24],
    }
}

/// Translucent square for each origin plane, `half` mm from the origin to each side.
pub fn plane_objects(half: f64) -> Vec<SceneObject> {
    planes()
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let pl = p.placement;
            let c = |u: f64, v: f64| {
                let w = pl.to_world(u, v);
                [w[0] as f32, w[1] as f32, w[2] as f32]
            };
            let corners = [c(-half, -half), c(half, -half), c(half, half), c(-half, half)];
            let n = pl.normal();
            let n = [n[0] as f32, n[1] as f32, n[2] as f32];
            let nb = [-n[0], -n[1], -n[2]];
            // Two-sided: front and back triangles so it can be picked from either side.
            let mut positions = corners.to_vec();
            positions.extend_from_slice(&corners);
            let mut normals = vec![n; 4];
            normals.extend(vec![nb; 4]);
            let indices = vec![0, 1, 2, 0, 2, 3, 4, 6, 5, 4, 7, 6];
            let mut pts = corners.to_vec();
            pts.push(corners[0]);
            SceneObject {
                name: p.name.to_string(),
                mesh: SceneMesh { positions, normals, indices, tri_face: vec![0; 4] },
                edges: vec![SceneEdge { id: 0, points: pts }],
                visible: true,
                color: plane_color(i),
            }
        })
        .collect()
}

/// Half-size of the plane squares: a bit larger than the part, like FreeCAD's origin.
pub fn plane_half_size(bbox: Option<[f64; 6]>) -> f64 {
    match bbox {
        Some(b) => {
            let m = b.iter().map(|v| v.abs()).fold(0.0, f64::max);
            (m * 1.2).max(10.0)
        }
        None => 50.0,
    }
}
