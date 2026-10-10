//! Converting the document's recompute results into viewport objects and overlays.

use crate::viewport::{Overlay, Plane, SceneEdge, SceneMesh, SceneObject};
use fc_occt::Shape;
use fc_part::{Document, FeatureKind, Placement};
use fc_sketch::{Curve, Sketch};

/// FreeCAD's default shape color (steel blue-gray).
pub const BODY_COLOR: [f32; 4] = [0.60, 0.64, 0.68, 1.0];

pub fn plane(p: &Placement) -> Plane {
    Plane { origin: p.origin, normal: p.normal(), xdir: p.xdir }
}

/// Mesh deflection scaled to the part size, like FreeCAD's relative deviation.
pub fn deflection(shape: &Shape) -> f64 {
    let b = shape.bbox();
    let d = ((b[3] - b[0]).powi(2) + (b[4] - b[1]).powi(2) + (b[5] - b[2]).powi(2)).sqrt();
    (d * 0.0015).clamp(0.005, 2.0)
}

/// Tessellate a shape. Face and edge ids are 0-based (FreeCAD's Face<n> is id + 1).
pub fn shape_object(name: &str, shape: &Shape, color: [f32; 4]) -> Option<SceneObject> {
    let m = shape.mesh(deflection(shape), 0.1).ok()?;
    let positions = m.positions.chunks_exact(3).map(|c| [c[0], c[1], c[2]]).collect();
    let normals = m.normals.chunks_exact(3).map(|c| [c[0], c[1], c[2]]).collect();
    let tri_face = m.tri_face.iter().map(|f| f.saturating_sub(1)).collect();
    let mut edges = Vec::new();
    for i in 0..m.edge_offsets.len().saturating_sub(1) {
        let (a, b) = (m.edge_offsets[i] as usize, m.edge_offsets[i + 1] as usize);
        if b > a {
            let pts = (a..b).map(|k| [m.edge_points[3 * k], m.edge_points[3 * k + 1], m.edge_points[3 * k + 2]]).collect();
            edges.push(SceneEdge { id: i as u32, points: pts });
        }
    }
    Some(SceneObject { name: name.to_string(), mesh: SceneMesh { positions, normals, indices: m.indices, tri_face }, edges, visible: true, color })
}

/// The body's tip solid as the single pickable object (index 0).
pub fn body_objects(doc: &Document) -> Vec<SceneObject> {
    doc.result().and_then(|s| shape_object("Body", s, BODY_COLOR)).into_iter().collect()
}

fn f3(p: [f64; 3]) -> [f32; 3] {
    [p[0] as f32, p[1] as f32, p[2] as f32]
}

/// Polylines of a sketch's curves, in world space.
pub fn sketch_polylines(sk: &Sketch, pl: &Placement, include_construction: bool) -> Vec<(Vec<[f32; 3]>, bool)> {
    let mut out = Vec::new();
    for g in &sk.geometry {
        if g.construction && !include_construction {
            continue;
        }
        let pts = match &g.curve {
            Curve::Point { .. } => continue,
            Curve::Line { .. } => fc_sketch::tessellate_curve(&g.curve, 1),
            _ => fc_sketch::tessellate_curve(&g.curve, 64),
        };
        out.push((pts.iter().map(|p| f3(pl.to_world(p.x, p.y))).collect(), g.construction));
    }
    out
}

/// Visible, not-being-edited sketches drawn as white lines (FreeCAD shows unused sketches).
pub fn sketch_overlay(doc: &Document, highlight: Option<fc_part::FeatureId>, hidden: &std::collections::HashSet<fc_part::FeatureId>) -> Overlay {
    let mut o = Overlay::default();
    for f in &doc.features {
        let FeatureKind::Sketch(sf) = &f.kind else { continue };
        if hidden.contains(&f.id) && highlight != Some(f.id) {
            continue;
        }
        let consumed = !doc.dependents(f.id).iter().all(|d| matches!(doc.get(*d).map(|g| &g.kind), Some(FeatureKind::Sketch(_))));
        let hl = highlight == Some(f.id);
        if consumed && !hl {
            continue;
        }
        let Some(pl) = doc.state(f.id).and_then(|s| s.placement) else { continue };
        let color = if hl { [0.11, 0.68, 0.11, 1.0] } else { [1.0, 1.0, 1.0, 1.0] };
        for (pts, constr) in sketch_polylines(&sf.sketch, &pl, false) {
            for w in pts.windows(2) {
                o.lines.push((w[0], w[1], if constr { [0.2, 0.4, 1.0, 1.0] } else { color }, 2.0));
            }
        }
    }
    o
}
