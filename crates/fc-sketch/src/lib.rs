//! fc-sketch: 2D sketch geometry and geometric constraint solver.
//!
//! A pure-Rust reimplementation of FreeCAD's Sketcher solving core (planegcs):
//!
//! * geometry is parametrised like FreeCAD (point = 2 DOF, line = 4, circle = 3,
//!   arc = 5: center, radius, start angle, end angle; arc endpoints are derived),
//! * sketcher constraints are lowered to scalar solver equations ("solver
//!   constraints", several per sketcher constraint, e.g. coincident = 2),
//! * the system is solved with Powell's DogLeg trust-region method (FreeCAD's
//!   default) using minimum-norm Gauss-Newton steps,
//! * diagnosis follows FreeCAD's `GCS::System::diagnose`: full-pivot Householder
//!   QR of the transposed Jacobian, DOF = free params - rank, dependent solver
//!   constraints grouped into conflict groups, FreeCAD's popularity heuristic to
//!   pick the constraints to drop, a re-solve to tell redundant from conflicting,
//!   and "partially redundant" sketcher constraints.
//!
//! External geometry follows FreeCAD's negative GeoIds via the constants
//! [`H_AXIS`], [`V_AXIS`] and [`ROOT`].

#![allow(clippy::needless_range_loop, clippy::neg_cmp_op_on_partial_ord)]

mod diagnose;
pub mod dual;
mod equations;
pub mod profile;
mod solver;

use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

pub use profile::{
    loop_contains_point, nest_loop_polygons, tessellate_curve, tessellate_edge, LoopEdge,
    ProfileError, ProfileFace, ProfileLoop,
};

/// Index into [`Sketch::geometry`], or one of the external ids [`H_AXIS`], [`V_AXIS`], [`ROOT`].
pub type GeoId = usize;

/// FreeCAD GeoId -1: the horizontal axis, a fixed line through the origin along +X.
pub const H_AXIS: GeoId = usize::MAX;
/// FreeCAD GeoId -2: the vertical axis, a fixed line through the origin along +Y.
pub const V_AXIS: GeoId = usize::MAX - 1;
/// The sketch origin as a fixed point (use with any `PointPos` except `Edge`, usually `Start`).
pub const ROOT: GeoId = usize::MAX - 2;

/// True for the fixed external geometries ([`H_AXIS`], [`V_AXIS`], [`ROOT`]).
pub fn is_external(g: GeoId) -> bool {
    g >= ROOT
}

/// Which point of a geometry a constraint refers to (FreeCAD `PointPos` none/start/end/mid).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PointPos {
    /// The curve itself (FreeCAD `none`).
    Edge,
    Start,
    End,
    /// Center of a circle/arc. (A `Point` geometry accepts any non-Edge position.)
    Mid,
}

#[derive(Clone, Copy, Debug, PartialEq, Default, Serialize, Deserialize)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub const ZERO: Vec2 = Vec2 { x: 0.0, y: 0.0 };
    pub const fn new(x: f64, y: f64) -> Vec2 {
        Vec2 { x, y }
    }
    pub fn dot(self, o: Vec2) -> f64 {
        self.x * o.x + self.y * o.y
    }
    pub fn cross(self, o: Vec2) -> f64 {
        self.x * o.y - self.y * o.x
    }
    pub fn length(self) -> f64 {
        self.x.hypot(self.y)
    }
    pub fn dist(self, o: Vec2) -> f64 {
        (self - o).length()
    }
    pub fn normalized(self) -> Vec2 {
        let l = self.length();
        if l == 0.0 {
            self
        } else {
            self * (1.0 / l)
        }
    }
    /// Rotate 90 degrees counter-clockwise.
    pub fn perp(self) -> Vec2 {
        Vec2::new(-self.y, self.x)
    }
    pub fn polar(angle: f64) -> Vec2 {
        Vec2::new(angle.cos(), angle.sin())
    }
    pub fn lerp(self, o: Vec2, t: f64) -> Vec2 {
        self + (o - self) * t
    }
}

impl std::ops::Add for Vec2 {
    type Output = Vec2;
    fn add(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x + o.x, self.y + o.y)
    }
}
impl std::ops::Sub for Vec2 {
    type Output = Vec2;
    fn sub(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x - o.x, self.y - o.y)
    }
}
impl std::ops::Mul<f64> for Vec2 {
    type Output = Vec2;
    fn mul(self, s: f64) -> Vec2 {
        Vec2::new(self.x * s, self.y * s)
    }
}
impl std::ops::Neg for Vec2 {
    type Output = Vec2;
    fn neg(self) -> Vec2 {
        Vec2::new(-self.x, -self.y)
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Curve {
    Point { p: Vec2 },
    Line { a: Vec2, b: Vec2 },
    Circle { c: Vec2, r: f64 },
    /// Angles in radians, CCW from `start` to `end` (like FreeCAD's ArcOfCircle).
    Arc { c: Vec2, r: f64, start: f64, end: f64 },
}

impl Curve {
    /// Number of solver parameters (= DOF of the free geometry).
    pub fn param_count(&self) -> usize {
        match self {
            Curve::Point { .. } => 2,
            Curve::Line { .. } => 4,
            Curve::Circle { .. } => 3,
            Curve::Arc { .. } => 5,
        }
    }
    /// The point at `pos`, if this curve has one there.
    pub fn point(&self, pos: PointPos) -> Option<Vec2> {
        match (self, pos) {
            (Curve::Point { p }, PointPos::Start | PointPos::End | PointPos::Mid) => Some(*p),
            (Curve::Line { a, .. }, PointPos::Start) => Some(*a),
            (Curve::Line { b, .. }, PointPos::End) => Some(*b),
            (Curve::Circle { c, .. }, PointPos::Mid) => Some(*c),
            (Curve::Arc { c, .. }, PointPos::Mid) => Some(*c),
            (Curve::Arc { c, r, start, .. }, PointPos::Start) => Some(*c + Vec2::polar(*start) * *r),
            (Curve::Arc { c, r, end, .. }, PointPos::End) => Some(*c + Vec2::polar(*end) * *r),
            _ => None,
        }
    }
    pub fn start_point(&self) -> Option<Vec2> {
        self.point(PointPos::Start)
    }
    pub fn end_point(&self) -> Option<Vec2> {
        self.point(PointPos::End)
    }
    /// CCW sweep of an arc in (0, 2pi]; 2pi for circles; 0 otherwise.
    pub fn sweep(&self) -> f64 {
        match self {
            Curve::Arc { start, end, .. } => normalize_sweep(end - start),
            Curve::Circle { .. } => 2.0 * PI,
            _ => 0.0,
        }
    }
    /// Point at normalized parameter t in [0,1] along the curve (start to end).
    pub fn eval(&self, t: f64) -> Vec2 {
        match self {
            Curve::Point { p } => *p,
            Curve::Line { a, b } => a.lerp(*b, t),
            Curve::Circle { c, r } => *c + Vec2::polar(2.0 * PI * t) * *r,
            Curve::Arc { c, r, start, .. } => *c + Vec2::polar(start + self.sweep() * t) * *r,
        }
    }
    /// Curve length (0 for points).
    pub fn length(&self) -> f64 {
        match self {
            Curve::Point { .. } => 0.0,
            Curve::Line { a, b } => a.dist(*b),
            Curve::Circle { r, .. } => 2.0 * PI * r.abs(),
            Curve::Arc { r, .. } => self.sweep() * r.abs(),
        }
    }
}

/// Bring an angle difference into (0, 2pi].
pub(crate) fn normalize_sweep(mut s: f64) -> f64 {
    let tau = 2.0 * PI;
    if s.is_finite() {
        s %= tau;
        if s <= 1e-14 {
            s += tau;
        }
    }
    s
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Geometry {
    pub curve: Curve,
    pub construction: bool,
}

impl Geometry {
    pub fn new(curve: Curve) -> Geometry {
        Geometry { curve, construction: false }
    }
    pub fn construction(curve: Curve) -> Geometry {
        Geometry { curve, construction: true }
    }
    pub fn point(p: Vec2) -> Geometry {
        Geometry::new(Curve::Point { p })
    }
    pub fn line(a: Vec2, b: Vec2) -> Geometry {
        Geometry::new(Curve::Line { a, b })
    }
    pub fn circle(c: Vec2, r: f64) -> Geometry {
        Geometry::new(Curve::Circle { c, r })
    }
    pub fn arc(c: Vec2, r: f64, start: f64, end: f64) -> Geometry {
        Geometry::new(Curve::Arc { c, r, start, end })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PointRef {
    pub geo: GeoId,
    pub pos: PointPos,
}

impl PointRef {
    pub fn new(geo: GeoId, pos: PointPos) -> PointRef {
        PointRef { geo, pos }
    }
    pub fn start(geo: GeoId) -> PointRef {
        PointRef::new(geo, PointPos::Start)
    }
    pub fn end(geo: GeoId) -> PointRef {
        PointRef::new(geo, PointPos::End)
    }
    pub fn mid(geo: GeoId) -> PointRef {
        PointRef::new(geo, PointPos::Mid)
    }
    pub fn edge(geo: GeoId) -> PointRef {
        PointRef::new(geo, PointPos::Edge)
    }
    /// The sketch origin.
    pub fn root() -> PointRef {
        PointRef::new(ROOT, PointPos::Start)
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ConstraintKind {
    Coincident(PointRef, PointRef),
    Horizontal(GeoId),
    Vertical(GeoId),
    HorizontalPoints(PointRef, PointRef),
    VerticalPoints(PointRef, PointRef),
    /// Lines.
    Parallel(GeoId, GeoId),
    /// Line-line; also line-circle (line through center) and circle-circle (orthogonal circles).
    Perpendicular(GeoId, GeoId),
    /// Edge-edge tangency: line-circle/arc, circle/arc-circle/arc (internal/external is
    /// chosen from the initial configuration like FreeCAD), line-line (= collinear).
    Tangent(GeoId, GeoId),
    /// Endpoint-to-endpoint tangency (coincident + tangent). The second ref may also be
    /// `PointPos::Edge` for endpoint-to-curve tangency (point on object + tangent).
    TangentAt(PointRef, PointRef),
    /// Equal length (lines) or equal radius (circles/arcs).
    Equal(GeoId, GeoId),
    PointOnObject(PointRef, GeoId),
    /// p1 and p2 symmetric about point p3. If p3.pos is `Edge` on a line, about that line.
    Symmetric(PointRef, PointRef, PointRef),
    SymmetricLine(PointRef, PointRef, GeoId),
    Distance(PointRef, PointRef, f64),
    /// Line length.
    DistanceLine(GeoId, f64),
    /// Point to line distance (or point to circle/arc circumference).
    PointLineDistance(PointRef, GeoId, f64),
    /// Signed: p2.x - p1.x = d.
    DistanceX(PointRef, PointRef, f64),
    /// Signed: p2.y - p1.y = d.
    DistanceY(PointRef, PointRef, f64),
    LockX(PointRef, f64),
    LockY(PointRef, f64),
    Radius(GeoId, f64),
    Diameter(GeoId, f64),
    /// Line: angle to the horizontal axis (CCW, radians). Arc: its sweep angle.
    Angle(GeoId, f64),
    /// CCW angle from line 1 to line 2 (directions start->end), radians.
    AngleBetween(GeoId, GeoId, f64),
    /// Fix all parameters of a geometry at their current values.
    Block(GeoId),
}

impl ConstraintKind {
    /// The dimensional value, if any.
    pub fn value(&self) -> Option<f64> {
        use ConstraintKind::*;
        match self {
            Distance(_, _, v) | DistanceLine(_, v) | PointLineDistance(_, _, v) | DistanceX(_, _, v)
            | DistanceY(_, _, v) | LockX(_, v) | LockY(_, v) | Radius(_, v) | Diameter(_, v)
            | Angle(_, v) | AngleBetween(_, _, v) => Some(*v),
            _ => None,
        }
    }
    /// Set the dimensional value; returns false if the constraint has none.
    pub fn set_value(&mut self, nv: f64) -> bool {
        use ConstraintKind::*;
        match self {
            Distance(_, _, v) | DistanceLine(_, v) | PointLineDistance(_, _, v) | DistanceX(_, _, v)
            | DistanceY(_, _, v) | LockX(_, v) | LockY(_, v) | Radius(_, v) | Diameter(_, v)
            | Angle(_, v) | AngleBetween(_, _, v) => {
                *v = nv;
                true
            }
            _ => false,
        }
    }
    /// True for constraints carrying a value (these may be made non-driving / reference).
    pub fn is_dimensional(&self) -> bool {
        self.value().is_some()
    }
    /// All geometries referenced by this constraint.
    pub fn geometries(&self) -> Vec<GeoId> {
        use ConstraintKind::*;
        match self {
            Coincident(a, b) | HorizontalPoints(a, b) | VerticalPoints(a, b) | TangentAt(a, b)
            | Distance(a, b, _) | DistanceX(a, b, _) | DistanceY(a, b, _) => vec![a.geo, b.geo],
            Horizontal(g) | Vertical(g) | DistanceLine(g, _) | Radius(g, _) | Diameter(g, _)
            | Angle(g, _) | Block(g) => vec![*g],
            Parallel(a, b) | Perpendicular(a, b) | Tangent(a, b) | Equal(a, b)
            | AngleBetween(a, b, _) => vec![*a, *b],
            PointOnObject(p, g) | PointLineDistance(p, g, _) => vec![p.geo, *g],
            Symmetric(a, b, c) => vec![a.geo, b.geo, c.geo],
            SymmetricLine(a, b, g) => vec![a.geo, b.geo, *g],
            LockX(p, _) | LockY(p, _) => vec![p.geo],
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Constraint {
    pub kind: ConstraintKind,
    /// Driving constraints are solved for; non-driving (reference) dimensional constraints
    /// only measure and get their value updated after each successful solve.
    pub driving: bool,
    pub name: Option<String>,
}

impl Constraint {
    pub fn new(kind: ConstraintKind) -> Constraint {
        Constraint { kind, driving: true, name: None }
    }
    /// A non-driving (reference) constraint.
    pub fn reference(kind: ConstraintKind) -> Constraint {
        Constraint { kind, driving: false, name: None }
    }
    pub fn named(mut self, name: impl Into<String>) -> Constraint {
        self.name = Some(name.into());
        self
    }
}

impl From<ConstraintKind> for Constraint {
    fn from(k: ConstraintKind) -> Constraint {
        Constraint::new(k)
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum SolveStatus {
    Solved,
    Failed,
    Conflicting,
    /// Solved, but some constraints are redundant.
    Redundant,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SolveReport {
    pub status: SolveStatus,
    pub dof: usize,
    /// Constraint indices (into `Sketch::constraints`), sorted ascending.
    pub conflicting: Vec<usize>,
    pub redundant: Vec<usize>,
    pub partially_redundant: Vec<usize>,
    pub malformed: Vec<usize>,
    pub iterations: usize,
}

impl SolveReport {
    pub fn is_ok(&self) -> bool {
        matches!(self.status, SolveStatus::Solved | SolveStatus::Redundant)
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Sketch {
    pub geometry: Vec<Geometry>,
    pub constraints: Vec<Constraint>,
}

impl Sketch {
    pub fn new() -> Sketch {
        Sketch::default()
    }

    pub fn add_geometry(&mut self, g: Geometry) -> GeoId {
        self.geometry.push(g);
        self.geometry.len() - 1
    }

    /// Accepts a `Constraint` or a bare `ConstraintKind` (driving).
    pub fn add_constraint(&mut self, c: impl Into<Constraint>) -> usize {
        self.constraints.push(c.into());
        self.constraints.len() - 1
    }

    /// Solve all driving constraints. On success (Solved/Redundant) the geometry is
    /// updated in place and non-driving dimensional constraints get their measured values.
    /// On failure/conflict the geometry is left untouched.
    pub fn solve(&mut self) -> SolveReport {
        let out = solver::run(self, None);
        if let Some(x) = &out.solution {
            out.layout.unpack(self, x);
            for (ci, v) in &out.measured {
                self.constraints[*ci].kind.set_value(*v);
            }
        }
        out.report
    }

    /// Solve-and-diagnose without modifying the sketch.
    pub fn diagnose(&self) -> SolveReport {
        solver::run(self, None).report
    }

    /// Interactive drag (FreeCAD's movePoint): move `p` toward `target` as far as the
    /// constraints allow, moving everything else as little as possible.
    /// `PointPos::Edge` on a line drags its midpoint (translating it when possible); on
    /// a circle/arc it drags the circumference (changing the radius when possible).
    pub fn drag_point(&mut self, p: PointRef, target: Vec2) -> SolveReport {
        let out = solver::run(self, Some((p, target)));
        if let Some(x) = &out.solution {
            out.layout.unpack(self, x);
            for (ci, v) in &out.measured {
                self.constraints[*ci].kind.set_value(*v);
            }
        }
        out.report
    }

    /// Position of a point reference. Panics on an invalid reference; see [`Sketch::try_point`].
    pub fn point(&self, p: PointRef) -> Vec2 {
        self.try_point(p).unwrap_or_else(|| panic!("invalid point reference {p:?}"))
    }

    /// Position of a point reference, `None` if the geometry has no such point.
    /// For a line `Edge` returns the midpoint.
    pub fn try_point(&self, p: PointRef) -> Option<Vec2> {
        match p.geo {
            ROOT => (p.pos != PointPos::Edge).then_some(Vec2::ZERO),
            H_AXIS => match p.pos {
                PointPos::Start | PointPos::Mid => Some(Vec2::ZERO),
                PointPos::End => Some(Vec2::new(1.0, 0.0)),
                PointPos::Edge => None,
            },
            V_AXIS => match p.pos {
                PointPos::Start | PointPos::Mid => Some(Vec2::ZERO),
                PointPos::End => Some(Vec2::new(0.0, 1.0)),
                PointPos::Edge => None,
            },
            g => {
                let c = &self.geometry.get(g)?.curve;
                match (c, p.pos) {
                    (Curve::Line { a, b }, PointPos::Edge) => Some(a.lerp(*b, 0.5)),
                    _ => c.point(p.pos),
                }
            }
        }
    }

    /// Per-geometry "fully constrained" flags (FreeCAD's green geometry): a geometry is
    /// fully constrained when none of its parameters can move in the null space of the
    /// driving constraints' Jacobian at the current configuration.
    pub fn fully_constrained_geometry(&self) -> Vec<bool> {
        solver::fully_constrained(self)
    }

    /// Per-parameter freedom: for each geometry, one flag per solver parameter
    /// (point: x,y; line: ax,ay,bx,by; circle: cx,cy,r; arc: cx,cy,r,start,end),
    /// true when the parameter is still free.
    pub fn free_parameters(&self) -> Vec<Vec<bool>> {
        solver::free_parameters(self)
    }

    /// Remove a constraint, returning it.
    pub fn remove_constraint(&mut self, idx: usize) -> Constraint {
        self.constraints.remove(idx)
    }

    /// Remove a geometry, deleting constraints that reference it and renumbering the
    /// GeoIds of the remaining constraints (like FreeCAD's delGeometry).
    pub fn remove_geometry(&mut self, g: GeoId) -> Geometry {
        let removed = self.geometry.remove(g);
        self.constraints.retain(|c| !c.kind.geometries().contains(&g));
        let fix = |id: &mut GeoId| {
            if !is_external(*id) && *id > g {
                *id -= 1;
            }
        };
        for c in &mut self.constraints {
            remap_geos(&mut c.kind, fix);
        }
        removed
    }

    /// Closed profile loops from non-construction lines/arcs/circles (points and
    /// construction geometry are ignored). Edges are ordered head-to-tail.
    pub fn profile_loops(&self) -> Result<Vec<ProfileLoop>, ProfileError> {
        profile::profile_loops(self)
    }

    /// Even-odd ("bullseye") nesting: returns (outer loop index, hole loop indices).
    /// Loops nested inside holes become new outers (islands).
    pub fn nest_loops(&self, loops: &[ProfileLoop]) -> Vec<(usize, Vec<usize>)> {
        profile::nest_loops(self, loops)
    }

    /// `profile_loops` + `nest_loops` in one go: faces with an outer wire and holes.
    pub fn profile_faces(&self) -> Result<Vec<ProfileFace>, ProfileError> {
        let loops = self.profile_loops()?;
        Ok(self
            .nest_loops(&loops)
            .into_iter()
            .map(|(o, hs)| ProfileFace {
                outer: loops[o].clone(),
                holes: hs.into_iter().map(|h| loops[h].clone()).collect(),
            })
            .collect())
    }

    /// Ordered polyline of a loop (closed: last point equals the first).
    /// `segments_per_turn` controls arc/circle tessellation density.
    pub fn loop_polygon(&self, l: &ProfileLoop, segments_per_turn: usize) -> Vec<Vec2> {
        profile::loop_polygon(self, l, segments_per_turn)
    }

    /// Exact signed area of a loop (positive = counter-clockwise).
    pub fn loop_signed_area(&self, l: &ProfileLoop) -> f64 {
        profile::loop_signed_area(self, l)
    }
}

fn remap_geos(k: &mut ConstraintKind, f: impl Fn(&mut GeoId)) {
    use ConstraintKind::*;
    match k {
        Coincident(a, b) | HorizontalPoints(a, b) | VerticalPoints(a, b) | TangentAt(a, b)
        | Distance(a, b, _) | DistanceX(a, b, _) | DistanceY(a, b, _) => {
            f(&mut a.geo);
            f(&mut b.geo)
        }
        Horizontal(g) | Vertical(g) | DistanceLine(g, _) | Radius(g, _) | Diameter(g, _)
        | Angle(g, _) | Block(g) => f(g),
        Parallel(a, b) | Perpendicular(a, b) | Tangent(a, b) | Equal(a, b)
        | AngleBetween(a, b, _) => {
            f(a);
            f(b)
        }
        PointOnObject(p, g) | PointLineDistance(p, g, _) => {
            f(&mut p.geo);
            f(g)
        }
        Symmetric(a, b, c) => {
            f(&mut a.geo);
            f(&mut b.geo);
            f(&mut c.geo)
        }
        SymmetricLine(a, b, g) => {
            f(&mut a.geo);
            f(&mut b.geo);
            f(g)
        }
        LockX(p, _) | LockY(p, _) => f(&mut p.geo),
    }
}

#[cfg(test)]
mod tests;
