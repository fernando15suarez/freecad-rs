//! Turning solved sketches into OCCT faces and wires.

use crate::placement::Placement;
use crate::PartError;
use fc_occt::{ProfileBuilder, Shape};
use fc_sketch::{Curve, Sketch};

fn add_edge(b: &mut ProfileBuilder, pl: &Placement, curve: &Curve) -> Result<(), PartError> {
    let n = pl.normal();
    match curve {
        Curve::Point { .. } => {}
        Curve::Line { a, b: e } => b.line(pl.to_world(a.x, a.y), pl.to_world(e.x, e.y))?,
        Curve::Circle { c, r } => b.circle(pl.to_world(c.x, c.y), n, pl.xdir, *r)?,
        Curve::Arc { c, r, start, end } => b.arc(pl.to_world(c.x, c.y), n, pl.xdir, *r, *start, *end)?,
    }
    Ok(())
}

/// Planar faces (outer boundary + holes) from the closed loops of a sketch's
/// non-construction geometry, placed in 3D. Mirrors FreeCAD's FaceMakerBullseye.
pub fn sketch_faces(sketch: &Sketch, pl: &Placement) -> Result<Shape, PartError> {
    let loops = sketch.profile_loops().map_err(|e| PartError::Profile(e.to_string()))?;
    if loops.is_empty() {
        return Err(PartError::Profile("sketch has no closed profile".into()));
    }
    let nests = sketch.nest_loops(&loops);
    let mut b = ProfileBuilder::new();
    for (outer, holes) in nests {
        b.begin_face();
        for li in std::iter::once(outer).chain(holes) {
            for e in &loops[li].edges {
                add_edge(&mut b, pl, &sketch.geometry[e.geo].curve)?;
            }
            b.end_wire()?;
        }
    }
    Ok(b.build_faces()?)
}

/// All closed loops as separate wires (for loft sections).
pub fn sketch_wires(sketch: &Sketch, pl: &Placement) -> Result<Shape, PartError> {
    let loops = sketch.profile_loops().map_err(|e| PartError::Profile(e.to_string()))?;
    let mut b = ProfileBuilder::new();
    for lp in &loops {
        for e in &lp.edges {
            add_edge(&mut b, pl, &sketch.geometry[e.geo].curve)?;
        }
        b.end_wire()?;
    }
    Ok(b.build_wires()?)
}

/// Non-construction edges chained head-to-tail into one (possibly open) wire, for sweep
/// spines. Edges are reordered and flipped as needed, like FreeCAD's spine building.
pub fn sketch_path(sketch: &Sketch, pl: &Placement) -> Result<Shape, PartError> {
    let edges: Vec<&Curve> = sketch
        .geometry
        .iter()
        .filter(|g| !g.construction && !matches!(g.curve, Curve::Point { .. }))
        .map(|g| &g.curve)
        .collect();
    if edges.is_empty() {
        return Err(PartError::Profile("path sketch has no edges".into()));
    }
    let mut b = ProfileBuilder::new();
    if edges.len() == 1 {
        add_edge(&mut b, pl, edges[0])?;
    } else {
        let ends = |c: &Curve| (c.start_point(), c.end_point());
        let tol = 1e-6;
        let near = |a: Option<fc_sketch::Vec2>, b: Option<fc_sketch::Vec2>| matches!((a, b), (Some(a), Some(b)) if a.dist(b) < tol);
        // Start from an edge whose start (or end) touches no other edge, if any.
        let degree = |p: Option<fc_sketch::Vec2>| edges.iter().filter(|e| near(ends(e).0, p) || near(ends(e).1, p)).count();
        let mut start = 0;
        let mut flip0 = false;
        for (i, e) in edges.iter().enumerate() {
            let (s, t) = ends(e);
            if degree(s) == 1 {
                start = i;
                break;
            }
            if degree(t) == 1 {
                start = i;
                flip0 = true;
                break;
            }
        }
        let mut used = vec![false; edges.len()];
        let mut order: Vec<(usize, bool)> = vec![(start, flip0)];
        used[start] = true;
        let tail = |i: usize, flip: bool| if flip { ends(edges[i]).0 } else { ends(edges[i]).1 };
        let mut cur = tail(start, flip0);
        while let Some((i, flip)) = (0..edges.len()).filter(|i| !used[*i]).find_map(|i| {
            let (s, t) = ends(edges[i]);
            if near(s, cur) { Some((i, false)) } else if near(t, cur) { Some((i, true)) } else { None }
        }) {
            used[i] = true;
            order.push((i, flip));
            cur = tail(i, flip);
        }
        if used.iter().any(|u| !u) {
            return Err(PartError::Profile("path edges do not form a single chain".into()));
        }
        for (i, _) in order {
            // MakeWire accepts edges in either direction as long as they connect in sequence.
            add_edge(&mut b, pl, edges[i])?;
        }
    }
    b.end_wire()?;
    Ok(b.build_wires()?)
}
