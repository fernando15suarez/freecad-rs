//! Mate connectors (Onshape semantics): a coordinate frame anchored to a face, edge or
//! vertex of a feature's solid (or to the body origin), inferred from the anchor's
//! geometry and then adjusted by flips, quarter turns, a translation and rotations.
//!
//! The anchor is a [`TopoRef`]: FreeCAD's `Face<n>` / `Edge<n>` / `Vertex<n>` plus a
//! geometric signature, so the connector re-finds "the same" entity after a rebuild
//! renumbers the topology, and reports [`PartError::Dangling`] when it is gone.

use crate::placement::*;
use crate::{flat_face_xhint, Document, FeatureId, FeatureKind, FeatureState, PartError};
use fc_occt::Shape;
use serde::{Deserialize, Serialize};

/// Right-handed orthonormal frame (`z = x × y`) in body coordinates.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Frame {
    pub origin: V3,
    pub x: V3,
    pub y: V3,
    pub z: V3,
}

impl Default for Frame {
    fn default() -> Self {
        Frame::WORLD
    }
}

impl Frame {
    pub const WORLD: Frame = Frame { origin: [0.0; 3], x: [1.0, 0.0, 0.0], y: [0.0, 1.0, 0.0], z: [0.0, 0.0, 1.0] };

    /// Frame with the given Z, and X as close to `xhint` as possible.
    pub fn from_z(origin: V3, z: V3, xhint: V3) -> Frame {
        let p = Placement::from_normal(origin, z, xhint);
        Frame { origin, x: p.xdir, y: p.ydir, z: normalize(z) }
    }

    /// Local (x, y, z) -> body coordinates.
    pub fn to_world(&self, p: V3) -> V3 {
        add(self.origin, add(add(scale(self.x, p[0]), scale(self.y, p[1])), scale(self.z, p[2])))
    }
    /// Body coordinates -> local (x, y, z).
    pub fn to_local(&self, p: V3) -> V3 {
        let d = sub(p, self.origin);
        [dot(d, self.x), dot(d, self.y), dot(d, self.z)]
    }
    /// The XY plane of this frame as a sketch placement.
    pub fn placement(&self) -> Placement {
        Placement { origin: self.origin, xdir: self.x, ydir: self.y }
    }

    /// Half turn about the frame's X axis: Z (primary axis) and Y reverse.
    pub fn flipped(&self) -> Frame {
        Frame { y: scale(self.y, -1.0), z: scale(self.z, -1.0), ..*self }
    }
    /// Rotate about the frame's own X (0), Y (1) or Z (2) axis by `deg` degrees.
    pub fn rotated_local(&self, axis: usize, deg: f64) -> Frame {
        let (s, c) = deg.to_radians().sin_cos();
        // Rotating the two axes perpendicular to `axis` within their plane.
        let rot = |a: V3, b: V3| (add(scale(a, c), scale(b, s)), add(scale(a, -s), scale(b, c)));
        match axis {
            0 => {
                let (y, z) = rot(self.y, self.z);
                Frame { y, z, ..*self }
            }
            1 => {
                let (z, x) = rot(self.z, self.x);
                Frame { z, x, ..*self }
            }
            _ => {
                let (x, y) = rot(self.x, self.y);
                Frame { x, y, ..*self }
            }
        }
    }
    /// Move by `d` expressed in this frame's axes.
    pub fn translated_local(&self, d: V3) -> Frame {
        Frame { origin: self.to_world(d), ..*self }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum TopoKind {
    Face,
    Edge,
    Vertex,
}

impl TopoKind {
    pub fn prefix(self) -> &'static str {
        match self {
            TopoKind::Face => "Face",
            TopoKind::Edge => "Edge",
            TopoKind::Vertex => "Vertex",
        }
    }
    /// Parse FreeCAD sub-element names: "Face6" -> (Face, 6).
    pub fn parse(name: &str) -> Option<(TopoKind, usize)> {
        for k in [TopoKind::Face, TopoKind::Edge, TopoKind::Vertex] {
            if let Some(n) = name.strip_prefix(k.prefix()).and_then(|n| n.parse::<usize>().ok()) {
                return (n >= 1).then_some((k, n));
            }
        }
        None
    }
}

/// Reference to a face, edge or vertex of a feature's solid. `index` is FreeCAD's 1-based
/// `Face<n>`/`Edge<n>`/`Vertex<n>`; the rest is a signature used to re-find the entity
/// when numbering shifts (generalizes [`crate::FaceRef`]).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TopoRef {
    pub feature: FeatureId,
    pub kind: TopoKind,
    pub index: usize,
    /// Geometry type: `FaceInfo::kind` for faces, `EdgeInfo::kind` for edges, 0 for vertices.
    pub geom: i32,
    /// Face centroid, edge midpoint (circle/ellipse center) or vertex position.
    pub center: V3,
    /// Plane normal, cylinder/cone axis, line direction or circle axis; zero for vertices.
    pub dir: V3,
    /// Area for faces, length for edges, 0 for vertices.
    pub size: f64,
}

impl TopoRef {
    /// "Face6", "Edge3", "Vertex1".
    pub fn name(&self) -> String {
        format!("{}{}", self.kind.prefix(), self.index)
    }
}

fn signature(shape: &Shape, kind: TopoKind, index: usize) -> Result<(i32, V3, V3, f64), PartError> {
    Ok(match kind {
        TopoKind::Face => {
            let f = shape.face_info(index)?;
            let dir = if matches!(f.kind, 0..=2) { normalize(f.normal) } else { [0.0; 3] };
            (f.kind, f.center, dir, f.area)
        }
        TopoKind::Edge => {
            let e = shape.edge_info(index)?;
            match e.kind {
                1 | 2 => (e.kind, e.center, normalize(e.axis), e.length),
                _ => (e.kind, e.mid, normalize(e.dir), e.length),
            }
        }
        TopoKind::Vertex => (0, shape.vertex_info(index)?, [0.0; 3], 0.0),
    })
}

fn count(shape: &Shape, kind: TopoKind) -> usize {
    match kind {
        TopoKind::Face => shape.face_count(),
        TopoKind::Edge => shape.edge_count(),
        TopoKind::Vertex => shape.vertex_count(),
    }
}

/// Build a [`TopoRef`] for `<kind><index>` of `shape` (the solid after `feature`).
pub fn topo_ref(feature: FeatureId, shape: &Shape, kind: TopoKind, index: usize) -> Result<TopoRef, PartError> {
    let (geom, center, dir, size) = signature(shape, kind, index)?;
    Ok(TopoRef { feature, kind, index, geom, center, dir, size })
}

/// Re-find a referenced entity: the same index if its signature still matches exactly,
/// else the best match of the same geometry type whose direction agrees (planes: same
/// outward normal within ~40°; axes and edge directions: up to sign). `None` = dangling.
pub fn find_topo(shape: &Shape, r: &TopoRef) -> Option<usize> {
    // Plane normals are oriented (outward); axes and edge directions are not.
    let oriented = r.kind == TopoKind::Face && r.geom == 0;
    let has_dir = norm(r.dir) > 0.5;
    let score = |i: usize| -> Option<f64> {
        let (geom, center, dir, size) = signature(shape, r.kind, i).ok()?;
        if geom != r.geom {
            return None;
        }
        let dn = if !has_dir {
            0.0
        } else if oriented {
            1.0 - dot(dir, r.dir)
        } else {
            1.0 - dot(dir, r.dir).abs()
        };
        if dn > 0.25 {
            return None;
        }
        let dc = norm(sub(center, r.center));
        let ds = if r.size > 1e-12 { (size - r.size).abs() / r.size } else { 0.0 };
        Some(dn * 100.0 + dc + ds * 10.0)
    };
    let n = count(shape, r.kind);
    if r.index >= 1 && r.index <= n {
        if let Some(s) = score(r.index) {
            if s < 1e-6 {
                return Some(r.index);
            }
        }
    }
    (1..=n).filter_map(|i| score(i).map(|s| (i, s))).min_by(|a, b| a.1.total_cmp(&b.1)).map(|(i, _)| i)
}

/// Which point of the anchor the connector sits on (Onshape's mate connector inference).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, Default)]
pub enum Infer {
    /// Planar face: centroid. Cylinder/cone: axis midpoint. Line: midpoint.
    /// Circle/ellipse edge: center. Vertex: the point.
    #[default]
    Auto,
    /// Face centroid (cylinder/cone: axis midpoint).
    Centroid,
    /// Planar face: center of its circular boundary (hole or round edge) nearest the centroid.
    /// Circle/arc edge, sphere face: center. Cylinder/cone: axis midpoint. Line: midpoint.
    Center,
    /// Middle of an edge or of a cylinder/cone axis.
    Midpoint,
    /// Edge start / lower end of a cylinder/cone axis (along the connector Z).
    Start,
    /// Edge end / upper end of a cylinder/cone axis.
    End,
    /// Closest inference point to a pick: on faces the closest point of the face
    /// (cylinder/cone: of the axis segment); on edges the nearest of start/mid/end/center.
    Closest(V3),
    /// Vertex: Z = outward normal of the adjacent face closest to this direction
    /// (without it a vertex connector's Z is world Z). Ignored for other anchors.
    Normal(V3),
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Default)]
pub enum Anchor {
    /// Body origin (world frame).
    #[default]
    Origin,
    /// A face, edge or vertex of a feature's solid.
    Topo { at: TopoRef, infer: Infer },
}

/// A mate connector feature. The resolved frame is
/// `inferred -> flip_z -> x_turns × 90° about Z -> translate(offset) -> rotate Z, X, Y`,
/// every step in the connector's current (local) axes, like Onshape's dialog.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Default)]
pub struct MateConnector {
    pub anchor: Anchor,
    /// Flip the primary axis (Z), a half turn about X.
    #[serde(default)]
    pub flip_z: bool,
    /// Reorient the secondary axis (X): quarter turns about Z (any integer, taken mod 4).
    #[serde(default)]
    pub x_turns: i32,
    /// Translation along the connector's X, Y, Z (mm).
    #[serde(default)]
    pub offset: V3,
    /// Rotations in degrees about the connector's X, Y, Z; applied Z first, then X, then Y.
    #[serde(default)]
    pub rotation: V3,
}

impl MateConnector {
    pub fn new(anchor: Anchor) -> Self {
        MateConnector { anchor, ..Default::default() }
    }
    /// Apply flip / reorientation / offsets to an inferred frame.
    pub fn adjust(&self, base: Frame) -> Frame {
        let mut f = if self.flip_z { base.flipped() } else { base };
        let turns = self.x_turns.rem_euclid(4);
        if turns != 0 {
            f = f.rotated_local(2, 90.0 * turns as f64);
        }
        f = f.translated_local(self.offset);
        for axis in [2, 0, 1] {
            if self.rotation[axis] != 0.0 {
                f = f.rotated_local(axis, self.rotation[axis]);
            }
        }
        f
    }
}

/// Canonical sign for an unoriented axis: its largest component positive.
fn canonical_axis(a: V3) -> V3 {
    let a = normalize(a);
    let k = (0..3).max_by(|&i, &j| a[i].abs().total_cmp(&a[j].abs())).unwrap();
    if a[k] < 0.0 { scale(a, -1.0) } else { a }
}

fn nearest(p: V3, cands: &[V3]) -> V3 {
    *cands.iter().min_by(|a, b| norm(sub(**a, p)).total_cmp(&norm(sub(**b, p)))).unwrap()
}

fn not_applicable(infer: Infer, what: &str) -> PartError {
    PartError::Invalid(format!("{infer:?} does not apply to {what}"))
}

/// The inferred frame (before offsets) of `<kind><index>` of `shape`.
pub fn infer_frame(shape: &Shape, kind: TopoKind, index: usize, infer: Infer) -> Result<Frame, PartError> {
    let (origin, z) = match kind {
        TopoKind::Face => infer_face(shape, index, infer)?,
        TopoKind::Edge => infer_edge(shape, index, infer)?,
        TopoKind::Vertex => {
            let p = shape.vertex_info(index)?;
            let z = match infer {
                Infer::Normal(hint) => {
                    let mut best: Option<(f64, V3)> = None;
                    for fi in shape.vertex_faces(index)? {
                        if let Ok((_, n)) = shape.face_point_normal(fi, p) {
                            let s = dot(n, normalize(hint));
                            if best.is_none_or(|(b, _)| s > b) {
                                best = Some((s, n));
                            }
                        }
                    }
                    best.map(|b| b.1).unwrap_or([0.0, 0.0, 1.0])
                }
                Infer::Auto | Infer::Centroid | Infer::Center | Infer::Midpoint | Infer::Closest(_) => [0.0, 0.0, 1.0],
                Infer::Start | Infer::End => return Err(not_applicable(infer, "a vertex")),
            };
            (p, z)
        }
    };
    let z = normalize(z);
    Ok(Frame::from_z(origin, z, flat_face_xhint(z)))
}

fn infer_face(shape: &Shape, index: usize, infer: Infer) -> Result<(V3, V3), PartError> {
    let f = shape.face_info(index)?;
    match f.kind {
        0 => {
            let n = normalize(f.normal);
            let o = match infer {
                Infer::Auto | Infer::Centroid | Infer::Midpoint | Infer::Normal(_) => f.center,
                Infer::Center => {
                    let mut best: Option<(f64, V3)> = None;
                    for ei in shape.face_edges(index)? {
                        let e = shape.edge_info(ei)?;
                        if e.kind == 1 || e.kind == 2 {
                            let d = norm(sub(e.center, f.center));
                            if best.is_none_or(|(b, _)| d < b - 1e-9) {
                                best = Some((d, e.center));
                            }
                        }
                    }
                    best.ok_or_else(|| PartError::Invalid(format!("Face{index} has no circular boundary")))?.1
                }
                Infer::Closest(p) => shape.face_point_normal(index, p)?.0,
                Infer::Start | Infer::End => return Err(not_applicable(infer, "a planar face")),
            };
            Ok((o, n))
        }
        1 | 2 => {
            let a = normalize(f.normal);
            let r = shape.face_axis_range(index)?;
            let at = |t: f64| add(f.origin, scale(a, t));
            let z = canonical_axis(a);
            // Ends ordered along the connector Z.
            let (lo, hi) = if dot(a, z) > 0.0 { (at(r[0]), at(r[1])) } else { (at(r[1]), at(r[0])) };
            let o = match infer {
                Infer::Auto | Infer::Centroid | Infer::Center | Infer::Midpoint | Infer::Normal(_) => at(0.5 * (r[0] + r[1])),
                Infer::Start => lo,
                Infer::End => hi,
                Infer::Closest(p) => at(dot(sub(p, f.origin), a).clamp(r[0], r[1])),
            };
            Ok((o, z))
        }
        _ => {
            let o = match infer {
                Infer::Center if f.kind == 3 => return Ok((f.origin, [0.0, 0.0, 1.0])),
                Infer::Closest(p) => p,
                Infer::Start | Infer::End => return Err(not_applicable(infer, "this face")),
                _ => f.center,
            };
            let (p, n) = shape.face_point_normal(index, o)?;
            Ok((p, n))
        }
    }
}

fn infer_edge(shape: &Shape, index: usize, infer: Infer) -> Result<(V3, V3), PartError> {
    let e = shape.edge_info(index)?;
    if e.degenerated {
        return Err(PartError::Invalid(format!("Edge{index} is degenerated")));
    }
    if e.kind == 1 || e.kind == 2 {
        // Z = circle axis, pointing out of the adjacent planar face when there is one
        // (a hole rim's connector points out of the part, like Onshape).
        let axis = normalize(e.axis);
        let mut z = canonical_axis(axis);
        for fi in shape.edge_faces(index)? {
            let f = shape.face_info(fi)?;
            if f.kind == 0 && dot(normalize(f.normal), axis).abs() > 1.0 - 1e-6 {
                z = normalize(f.normal);
                break;
            }
        }
        let o = match infer {
            Infer::Auto | Infer::Center | Infer::Centroid | Infer::Normal(_) => e.center,
            Infer::Midpoint => e.mid,
            Infer::Start => e.start,
            Infer::End => e.end,
            Infer::Closest(p) => {
                if e.closed { nearest(p, &[e.center]) } else { nearest(p, &[e.center, e.start, e.mid, e.end]) }
            }
        };
        return Ok((o, z));
    }
    let o = match infer {
        Infer::Auto | Infer::Midpoint | Infer::Center | Infer::Centroid | Infer::Normal(_) => e.mid,
        Infer::Start => e.start,
        Infer::End => e.end,
        Infer::Closest(p) => nearest(p, &[e.start, e.mid, e.end]),
    };
    Ok((o, normalize(e.dir)))
}

impl Document {
    /// A [`TopoRef`] for `name` ("Face6", "Edge3", "Vertex1") of the solid after `feature`
    /// (needs a recompute so the feature has a shape).
    pub fn topo_ref(&self, feature: FeatureId, name: &str) -> Result<TopoRef, PartError> {
        let (kind, index) = TopoKind::parse(name).ok_or_else(|| PartError::Invalid(format!("'{name}' is not a Face/Edge/Vertex name")))?;
        let shape = self.state(feature).and_then(|s| s.shape.as_ref()).ok_or_else(|| PartError::Missing(format!("shape of feature #{feature}")))?;
        topo_ref(feature, shape, kind, index)
    }

    /// Add a mate connector feature.
    pub fn add_mate_connector(&mut self, mc: MateConnector) -> FeatureId {
        self.add(FeatureKind::MateConnector(mc))
    }

    /// Resolved frame of a mate connector (after [`Document::recompute`]).
    pub fn connector_frame(&self, id: FeatureId) -> Result<Frame, PartError> {
        let f = self.get(id).ok_or_else(|| PartError::Missing(format!("feature #{id}")))?;
        if !matches!(f.kind, FeatureKind::MateConnector(_)) {
            return Err(PartError::Invalid(format!("{} is not a mate connector", f.name)));
        }
        let st = self.state(id).ok_or_else(|| PartError::Missing(format!("{} has not been recomputed", f.name)))?;
        if let Some(e) = &st.error {
            return Err(e.clone());
        }
        if f.suppressed {
            return Err(PartError::Invalid(format!("{} is suppressed", f.name)));
        }
        st.frame.ok_or_else(|| PartError::Missing(format!("{} has not been recomputed", f.name)))
    }

    /// Recompute step for a connector at tree index `i`.
    pub(crate) fn compute_connector(&self, i: usize, mc: &MateConnector, states: &[FeatureState], st: &mut FeatureState) -> Result<(), PartError> {
        let base = match &mc.anchor {
            Anchor::Origin => Frame::WORLD,
            Anchor::Topo { at, infer } => {
                let j = self.index_of(at.feature).ok_or_else(|| PartError::Dangling(format!("{} of a deleted feature", at.name())))?;
                let owner = &self.features[j].name;
                if j >= i {
                    return Err(PartError::Invalid(format!("{owner} comes after {} in the tree", self.features[i].name)));
                }
                let fs = &states[j];
                if let Some(e) = &fs.error {
                    return Err(PartError::Invalid(format!("{owner} has an error: {e}")));
                }
                let shape = fs.shape.as_ref().ok_or_else(|| PartError::Dangling(format!("{owner} has no solid")))?;
                let index = find_topo(shape, at).ok_or_else(|| PartError::Dangling(format!("{owner}.{} no longer exists", at.name())))?;
                infer_frame(shape, at.kind, index, *infer)?
            }
        };
        st.frame = Some(mc.adjust(base));
        Ok(())
    }
}
