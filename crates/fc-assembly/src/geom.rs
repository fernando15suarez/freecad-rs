//! Picking geometry for a joint: FreeCAD's way of turning "Face6 + Vertex7" into a joint
//! coordinate system (JCS), ported from `UtilsAssembly.findPlacement` (FreeCAD 1.1).
//!
//! All coordinates are local to the part (the part's own shape, without the instance
//! placement).

use crate::frame::{occt_frame, Frame};
use fc_part::placement::{add, dot, normalize, scale, sub};
use fc_part::V3;
use serde::{Deserialize, Serialize};

/// Geometry of one B-rep sub-element, as much as the JCS inference needs.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ElementGeom {
    Vertex { point: V3 },
    /// Straight edge from `start` to `end`.
    Line { start: V3, end: V3 },
    /// Circle or arc. `xdir` is the curve's X axis when known (FreeCAD's `curve.Rotation`);
    /// otherwise OCCT's automatic X for `axis` is used.
    Circle { center: V3, axis: V3, radius: f64, xdir: Option<V3> },
    /// Any other curve: only its midpoint is used.
    Curve { mid: V3 },
    /// Planar face: the surface's position/axis/X (FreeCAD uses the *surface* rotation),
    /// and the face's centre of gravity.
    Plane { origin: V3, normal: V3, xdir: V3, cog: V3 },
    /// Cylindrical face: a point on the axis, the axis, X, radius and centre of gravity.
    Cylinder { origin: V3, axis: V3, xdir: V3, radius: f64, cog: V3 },
    Cone { apex: V3, axis: V3, xdir: V3, cog: V3 },
    Sphere { center: V3, radius: f64 },
    Torus { center: V3, axis: V3, xdir: V3, major: f64, minor: f64 },
    /// Free-form face: only the centre of gravity is used.
    Surface { cog: V3 },
}

/// The geometric nature of a joint reference; selects the Distance joint variant.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum RefKind {
    Point,
    Line,
    Circle { radius: f64 },
    Curve,
    Plane,
    Cylinder { radius: f64 },
    Sphere { radius: f64 },
    Cone,
    Torus,
    Surface,
    /// A whole frame (mate connector, detached JCS, whole object).
    Frame,
}

impl ElementGeom {
    pub fn kind(&self) -> RefKind {
        match self {
            ElementGeom::Vertex { .. } => RefKind::Point,
            ElementGeom::Line { .. } => RefKind::Line,
            ElementGeom::Circle { radius, .. } => RefKind::Circle { radius: *radius },
            ElementGeom::Curve { .. } => RefKind::Curve,
            ElementGeom::Plane { .. } => RefKind::Plane,
            ElementGeom::Cylinder { radius, .. } => RefKind::Cylinder { radius: *radius },
            ElementGeom::Cone { .. } => RefKind::Cone,
            ElementGeom::Sphere { radius, .. } => RefKind::Sphere { radius: *radius },
            ElementGeom::Torus { .. } => RefKind::Torus,
            ElementGeom::Surface { .. } => RefKind::Surface,
        }
    }
}

/// Which point of the picked element the JCS sits on (FreeCAD's second sub-element).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum ImplicitPoint {
    /// The element itself was picked twice ("Face6", "Face6"): its natural centre —
    /// line midpoint, circle centre, plane centre of gravity, cylinder axis point at the
    /// centre of gravity, sphere/torus centre, cone apex.
    #[default]
    Center,
    /// A vertex of the element ("Face6", "Vertex7").
    Vertex(String),
    /// A circular edge of a face: its centre ("Face1", "Edge3").
    EdgeCenter(String),
    /// Onshape-style inference, shared with fc-part's mate connectors
    /// (`fc_part::mate::infer_frame`): Z = outward normal / axis, X from FreeCAD's
    /// flat-face rule, origin chosen by the [`fc_part::Infer`] mode.
    Infer(fc_part::Infer),
}

/// FreeCAD's `findPlacement`: the JCS for `elem`, positioned at `point`.
/// `point_geom` is the geometry of the vertex/edge named by `point` (if any).
/// `ignore_vertex` is FreeCAD's Distance-joint rule (always use the element's centre).
pub fn infer_frame(elem: &ElementGeom, point_geom: Option<&ElementGeom>, ignore_vertex: bool) -> Frame {
    let explicit = if ignore_vertex { None } else { point_geom };
    match elem {
        ElementGeom::Vertex { point } => Frame::translation(*point),
        ElementGeom::Line { start, end } => {
            let pos = match explicit {
                Some(ElementGeom::Vertex { point }) => *point,
                _ => scale(add(*start, *end), 0.5),
            };
            // FreeCAD: Part.Plane(origin, round(direction)).Rotation (OCCT auto X).
            let d = normalize(sub(*end, *start));
            occt_frame(pos, [round10(d[0]), round10(d[1]), round10(d[2])])
        }
        ElementGeom::Circle { center, axis, xdir, .. } => {
            let pos = match explicit {
                Some(ElementGeom::Vertex { point }) => *point,
                _ => *center,
            };
            match xdir {
                Some(x) => Frame::from_z_x(pos, *axis, *x),
                None => occt_frame(pos, *axis),
            }
        }
        ElementGeom::Curve { mid } => match explicit {
            Some(ElementGeom::Vertex { point }) => Frame::translation(*point),
            _ => Frame::translation(*mid),
        },
        _ => {
            // Faces.
            let center = match elem {
                ElementGeom::Plane { cog, .. } => *cog,
                ElementGeom::Cylinder { origin, axis, cog, .. } => {
                    let a = normalize(*axis);
                    add(*origin, scale(a, dot(sub(*cog, *origin), a)))
                }
                ElementGeom::Cone { apex, .. } => *apex,
                ElementGeom::Sphere { center, .. } | ElementGeom::Torus { center, .. } => *center,
                ElementGeom::Surface { cog } => *cog,
                _ => unreachable!(),
            };
            let pos = match explicit {
                Some(ElementGeom::Vertex { point }) => *point,
                Some(ElementGeom::Circle { center, .. }) => *center,
                _ => center,
            };
            match elem {
                ElementGeom::Plane { normal, xdir, .. } => Frame::from_z_x(pos, *normal, *xdir),
                ElementGeom::Cylinder { axis, xdir, .. }
                | ElementGeom::Cone { axis, xdir, .. }
                | ElementGeom::Torus { axis, xdir, .. } => Frame::from_z_x(pos, *axis, *xdir),
                _ => Frame::translation(pos),
            }
        }
    }
}

fn round10(v: f64) -> f64 {
    (v * 1e10).round() / 1e10
}

/// Parse a FreeCAD element name: "Face12" -> ("Face", 12).
pub fn parse_element(name: &str) -> Option<(&str, usize)> {
    let i = name.find(|c: char| c.is_ascii_digit())?;
    let (t, n) = name.split_at(i);
    Some((t, n.parse().ok()?))
}
