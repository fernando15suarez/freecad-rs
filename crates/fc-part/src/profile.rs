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

/// Every non-construction edge chained into one (possibly open) wire, for sweep spines.
pub fn sketch_path(sketch: &Sketch, pl: &Placement) -> Result<Shape, PartError> {
    let mut b = ProfileBuilder::new();
    let mut any = false;
    for g in sketch.geometry.iter().filter(|g| !g.construction) {
        if matches!(g.curve, Curve::Point { .. }) {
            continue;
        }
        add_edge(&mut b, pl, &g.curve)?;
        any = true;
    }
    if !any {
        return Err(PartError::Profile("path sketch has no edges".into()));
    }
    b.end_wire()?;
    Ok(b.build_wires()?)
}
