//! Lowering of sketcher constraints to scalar solver equations (residual rows),
//! evaluated with forward-mode AD for exact Jacobians.

use crate::dual::{D, NL};
use crate::{Constraint, ConstraintKind, Curve, GeoId, PointPos, PointRef, Sketch, Vec2, H_AXIS, ROOT, V_AXIS};
use nalgebra::{DMatrix, DVector};
use std::cell::Cell;
use std::f64::consts::PI;

/// Parameter layout: offset of each geometry's parameters in the global vector.
#[derive(Clone, Debug)]
pub(crate) struct Layout {
    pub offsets: Vec<usize>,
    pub counts: Vec<usize>,
    pub n: usize,
}

impl Layout {
    pub fn new(sk: &Sketch) -> Layout {
        let mut offsets = Vec::with_capacity(sk.geometry.len());
        let mut counts = Vec::with_capacity(sk.geometry.len());
        let mut n = 0;
        for g in &sk.geometry {
            offsets.push(n);
            let c = g.curve.param_count();
            counts.push(c);
            n += c;
        }
        Layout { offsets, counts, n }
    }

    pub fn pack(&self, sk: &Sketch) -> DVector<f64> {
        let mut x = Vec::with_capacity(self.n);
        for g in &sk.geometry {
            match &g.curve {
                Curve::Point { p } => x.extend([p.x, p.y]),
                Curve::Line { a, b } => x.extend([a.x, a.y, b.x, b.y]),
                Curve::Circle { c, r } => x.extend([c.x, c.y, *r]),
                Curve::Arc { c, r, start, end } => x.extend([c.x, c.y, *r, *start, *end]),
            }
        }
        DVector::from_vec(x)
    }

    /// Write parameters back, normalising circles/arcs (positive radius, start angle in
    /// [0, 2pi), sweep in (0, 2pi]).
    pub fn unpack(&self, sk: &mut Sketch, x: &DVector<f64>) {
        for (i, g) in sk.geometry.iter_mut().enumerate() {
            let o = self.offsets[i];
            match &mut g.curve {
                Curve::Point { p } => *p = Vec2::new(x[o], x[o + 1]),
                Curve::Line { a, b } => {
                    *a = Vec2::new(x[o], x[o + 1]);
                    *b = Vec2::new(x[o + 2], x[o + 3]);
                }
                Curve::Circle { c, r } => {
                    *c = Vec2::new(x[o], x[o + 1]);
                    *r = x[o + 2].abs();
                }
                Curve::Arc { c, r, start, end } => {
                    *c = Vec2::new(x[o], x[o + 1]);
                    let (mut rr, mut s, mut e) = (x[o + 2], x[o + 3], x[o + 4]);
                    if rr < 0.0 {
                        rr = -rr;
                        s += PI;
                        e += PI;
                    }
                    let sweep = crate::normalize_sweep(e - s);
                    let tau = 2.0 * PI;
                    s = s.rem_euclid(tau);
                    if s >= tau - 1e-15 {
                        s = 0.0;
                    }
                    *r = rr;
                    *start = s;
                    *end = s + sweep;
                }
            }
        }
    }
}

/// Per-constraint choices fixed when the system is set up (from the initial geometry),
/// e.g. which side of a line a tangent circle lies on.
#[derive(Clone, Debug, Default)]
pub(crate) struct Setup {
    pub s1: Cell<Option<f64>>,
    pub s2: Cell<Option<f64>>,
    pub block: Vec<f64>,
}

fn pick(cell: &Cell<Option<f64>>, f: impl FnOnce() -> f64) -> f64 {
    match cell.get() {
        Some(v) => v,
        None => {
            let v = f();
            cell.set(Some(v));
            v
        }
    }
}

fn sgn(v: f64) -> f64 {
    if v < 0.0 {
        -1.0
    } else {
        1.0
    }
}

#[derive(Debug)]
pub(crate) struct Bad;
type R<T> = Result<T, Bad>;

#[derive(Clone, Copy)]
pub(crate) struct P {
    pub x: D,
    pub y: D,
}
impl P {
    fn c(v: Vec2) -> P {
        P { x: D::c(v.x), y: D::c(v.y) }
    }
    fn sub(self, o: P) -> P {
        P { x: self.x - o.x, y: self.y - o.y }
    }
    fn add(self, o: P) -> P {
        P { x: self.x + o.x, y: self.y + o.y }
    }
    fn scale(self, s: D) -> P {
        P { x: self.x * s, y: self.y * s }
    }
    fn dot(self, o: P) -> D {
        self.x * o.x + self.y * o.y
    }
    fn cross(self, o: P) -> D {
        self.x * o.y - self.y * o.x
    }
    fn len(self) -> D {
        (self.x.sq() + self.y.sq() + 1e-300).sqrt()
    }
    fn unit(self) -> P {
        let l = self.len();
        P { x: self.x / l, y: self.y / l }
    }
    fn perp(self) -> P {
        P { x: -self.y, y: self.x }
    }
}

#[derive(Clone, Copy)]
pub(crate) enum G {
    Point(P),
    Line(P, P),
    Circle(P, D),
    Arc(P, D, D, D),
}

pub(crate) struct Ev<'a> {
    pub sk: &'a Sketch,
    pub lay: &'a Layout,
    pub x: &'a [f64],
    pub slots: Vec<usize>,
}

impl<'a> Ev<'a> {
    pub fn new(sk: &'a Sketch, lay: &'a Layout, x: &'a [f64]) -> Ev<'a> {
        Ev { sk, lay, x, slots: Vec::new() }
    }

    pub fn var(&mut self, gi: usize) -> D {
        let k = match self.slots.iter().position(|&s| s == gi) {
            Some(k) => k,
            None => {
                assert!(self.slots.len() < NL, "constraint touches too many parameters");
                self.slots.push(gi);
                self.slots.len() - 1
            }
        };
        D::var(self.x[gi], k)
    }

    pub fn geo(&mut self, id: GeoId) -> R<G> {
        match id {
            H_AXIS => return Ok(G::Line(P::c(Vec2::ZERO), P::c(Vec2::new(1.0, 0.0)))),
            V_AXIS => return Ok(G::Line(P::c(Vec2::ZERO), P::c(Vec2::new(0.0, 1.0)))),
            ROOT => return Ok(G::Point(P::c(Vec2::ZERO))),
            _ => {}
        }
        let g = self.sk.geometry.get(id).ok_or(Bad)?;
        let o = self.lay.offsets[id];
        Ok(match g.curve {
            Curve::Point { .. } => G::Point(P { x: self.var(o), y: self.var(o + 1) }),
            Curve::Line { .. } => G::Line(
                P { x: self.var(o), y: self.var(o + 1) },
                P { x: self.var(o + 2), y: self.var(o + 3) },
            ),
            Curve::Circle { .. } => G::Circle(P { x: self.var(o), y: self.var(o + 1) }, self.var(o + 2)),
            Curve::Arc { .. } => G::Arc(
                P { x: self.var(o), y: self.var(o + 1) },
                self.var(o + 2),
                self.var(o + 3),
                self.var(o + 4),
            ),
        })
    }

    pub fn point(&mut self, pr: PointRef) -> R<P> {
        let g = self.geo(pr.geo)?;
        point_of(&g, pr.geo, pr.pos)
    }

    fn line(&mut self, id: GeoId) -> R<(P, P)> {
        match self.geo(id)? {
            G::Line(a, b) => Ok((a, b)),
            _ => Err(Bad),
        }
    }

    fn circ(&mut self, id: GeoId) -> R<(P, D)> {
        match self.geo(id)? {
            G::Circle(c, r) | G::Arc(c, r, _, _) => Ok((c, r)),
            _ => Err(Bad),
        }
    }
}

fn point_of(g: &G, id: GeoId, pos: PointPos) -> R<P> {
    match (g, pos) {
        (_, PointPos::Edge) => Err(Bad),
        (G::Point(p), _) => Ok(*p),
        (G::Line(a, _), PointPos::Start) => Ok(*a),
        (G::Line(_, b), PointPos::End) => Ok(*b),
        (G::Line(..), PointPos::Mid) => {
            // FreeCAD lines have no mid point; the axes' "mid" is the origin for convenience.
            if id == H_AXIS || id == V_AXIS {
                Ok(P::c(Vec2::ZERO))
            } else {
                Err(Bad)
            }
        }
        (G::Circle(c, _), PointPos::Mid) | (G::Arc(c, ..), PointPos::Mid) => Ok(*c),
        (G::Circle(..), _) => Err(Bad),
        (G::Arc(c, r, s, _), PointPos::Start) => Ok(arc_pt(*c, *r, *s)),
        (G::Arc(c, r, _, e), PointPos::End) => Ok(arc_pt(*c, *r, *e)),
    }
}

fn arc_pt(c: P, r: D, a: D) -> P {
    P { x: c.x + r * a.cos(), y: c.y + r * a.sin() }
}

/// Unit tangent of a curve at an endpoint.
fn end_tangent(g: &G, pos: PointPos) -> R<P> {
    match (g, pos) {
        (G::Line(a, b), PointPos::Start | PointPos::End) => Ok(b.sub(*a).unit()),
        (G::Arc(_, _, s, _), PointPos::Start) => Ok(P { x: -s.sin(), y: s.cos() }),
        (G::Arc(_, _, _, e), PointPos::End) => Ok(P { x: -e.sin(), y: e.cos() }),
        _ => Err(Bad),
    }
}

/// Unit tangent of a curve at (or near) a point lying on it.
fn tangent_at(g: &G, at: P) -> R<P> {
    match g {
        G::Line(a, b) => Ok(b.sub(*a).unit()),
        G::Circle(c, _) | G::Arc(c, ..) => Ok(at.sub(*c).perp().unit()),
        G::Point(_) => Err(Bad),
    }
}

/// Signed distance of p from the line (a,b) (positive on the left).
fn line_sdist(a: P, b: P, p: P) -> D {
    let d = b.sub(a);
    d.cross(p.sub(a)) / d.len()
}

fn wrap_angle(u: P, d: P) -> D {
    // signed angle from u to d
    u.cross(d).atan2(u.dot(d))
}

fn sym_line(ev: &mut Ev, a: PointRef, b: PointRef, l: GeoId) -> R<Vec<D>> {
    let p = ev.point(a)?;
    let q = ev.point(b)?;
    let (la, lb) = ev.line(l)?;
    let d = lb.sub(la);
    let len = d.len();
    let mid = p.add(q).scale(D::c(0.5));
    Ok(vec![d.cross(mid.sub(la)) / len, d.dot(q.sub(p)) / len])
}

/// Evaluate the residual rows of one sketcher constraint.
pub(crate) fn eval_constraint(ev: &mut Ev, c: &Constraint, st: &Setup) -> R<Vec<D>> {
    use ConstraintKind::*;
    let rows = match &c.kind {
        Coincident(a, b) => {
            let p = ev.point(*a)?;
            let q = ev.point(*b)?;
            vec![p.x - q.x, p.y - q.y]
        }
        Horizontal(g) => {
            let (a, b) = ev.line(*g)?;
            vec![b.y - a.y]
        }
        Vertical(g) => {
            let (a, b) = ev.line(*g)?;
            vec![b.x - a.x]
        }
        HorizontalPoints(a, b) => {
            let p = ev.point(*a)?;
            let q = ev.point(*b)?;
            vec![q.y - p.y]
        }
        VerticalPoints(a, b) => {
            let p = ev.point(*a)?;
            let q = ev.point(*b)?;
            vec![q.x - p.x]
        }
        Parallel(g1, g2) => {
            let (a1, b1) = ev.line(*g1)?;
            let (a2, b2) = ev.line(*g2)?;
            let (d1, d2) = (b1.sub(a1), b2.sub(a2));
            vec![d1.cross(d2) / (d1.len() * d2.len())]
        }
        Perpendicular(g1, g2) => {
            let x1 = ev.geo(*g1)?;
            let x2 = ev.geo(*g2)?;
            match (x1, x2) {
                (G::Line(a1, b1), G::Line(a2, b2)) => {
                    let (d1, d2) = (b1.sub(a1), b2.sub(a2));
                    vec![d1.dot(d2) / (d1.len() * d2.len())]
                }
                (G::Line(a, b), G::Circle(c, _) | G::Arc(c, ..))
                | (G::Circle(c, _) | G::Arc(c, ..), G::Line(a, b)) => vec![line_sdist(a, b, c)],
                (G::Circle(c1, r1) | G::Arc(c1, r1, ..), G::Circle(c2, r2) | G::Arc(c2, r2, ..)) => {
                    vec![c2.sub(c1).len() - (r1.sq() + r2.sq()).sqrt()]
                }
                _ => return Err(Bad),
            }
        }
        Tangent(g1, g2) => {
            let x1 = ev.geo(*g1)?;
            let x2 = ev.geo(*g2)?;
            match (x1, x2) {
                (G::Line(a1, b1), G::Line(a2, b2)) => {
                    let (d1, d2) = (b1.sub(a1), b2.sub(a2));
                    vec![d1.cross(d2) / (d1.len() * d2.len()), line_sdist(a1, b1, a2)]
                }
                (G::Line(a, b), G::Circle(c, r) | G::Arc(c, r, ..))
                | (G::Circle(c, r) | G::Arc(c, r, ..), G::Line(a, b)) => {
                    let sd = line_sdist(a, b, c);
                    let s = pick(&st.s1, || sgn(sd.v));
                    vec![sd * s - r]
                }
                (G::Circle(c1, r1) | G::Arc(c1, r1, ..), G::Circle(c2, r2) | G::Arc(c2, r2, ..)) => {
                    let dist = c2.sub(c1).len();
                    // FreeCAD: internal tangency if the centre distance is smaller than the
                    // larger radius at set-up time.
                    let internal = pick(&st.s1, || if dist.v < r1.v.abs().max(r2.v.abs()) { 1.0 } else { 0.0 });
                    if internal > 0.5 {
                        let s = pick(&st.s2, || sgn(r1.v - r2.v));
                        vec![dist - (r1 - r2) * s]
                    } else {
                        vec![dist - (r1 + r2)]
                    }
                }
                _ => return Err(Bad),
            }
        }
        TangentAt(p1, p2) => {
            let (p1, p2) = if p1.pos == PointPos::Edge { (*p2, *p1) } else { (*p1, *p2) };
            if p1.pos == PointPos::Edge || p1.pos == PointPos::Mid {
                return Err(Bad);
            }
            let g1 = ev.geo(p1.geo)?;
            let g2 = ev.geo(p2.geo)?;
            let a = point_of(&g1, p1.geo, p1.pos)?;
            let t1 = end_tangent(&g1, p1.pos)?;
            match p2.pos {
                PointPos::Start | PointPos::End => {
                    let b = point_of(&g2, p2.geo, p2.pos)?;
                    let t2 = end_tangent(&g2, p2.pos)?;
                    vec![a.x - b.x, a.y - b.y, t1.cross(t2)]
                }
                PointPos::Edge => {
                    let on = match g2 {
                        G::Line(la, lb) => line_sdist(la, lb, a),
                        G::Circle(c, r) | G::Arc(c, r, ..) => a.sub(c).len() - r,
                        G::Point(_) => return Err(Bad),
                    };
                    let t2 = tangent_at(&g2, a)?;
                    vec![on, t1.cross(t2)]
                }
                PointPos::Mid => return Err(Bad),
            }
        }
        Equal(g1, g2) => {
            let x1 = ev.geo(*g1)?;
            let x2 = ev.geo(*g2)?;
            match (x1, x2) {
                (G::Line(a1, b1), G::Line(a2, b2)) => vec![b1.sub(a1).len() - b2.sub(a2).len()],
                (G::Circle(_, r1) | G::Arc(_, r1, ..), G::Circle(_, r2) | G::Arc(_, r2, ..)) => vec![r1 - r2],
                _ => return Err(Bad),
            }
        }
        PointOnObject(p, g) => {
            let pt = ev.point(*p)?;
            match ev.geo(*g)? {
                G::Line(a, b) => vec![line_sdist(a, b, pt)],
                G::Circle(c, r) | G::Arc(c, r, ..) => vec![pt.sub(c).len() - r],
                G::Point(_) => return Err(Bad),
            }
        }
        Symmetric(a, b, m) => {
            if m.pos == PointPos::Edge {
                sym_line(ev, *a, *b, m.geo)?
            } else {
                let p = ev.point(*a)?;
                let q = ev.point(*b)?;
                let mm = ev.point(*m)?;
                vec![(p.x + q.x) * 0.5 - mm.x, (p.y + q.y) * 0.5 - mm.y]
            }
        }
        SymmetricLine(a, b, l) => sym_line(ev, *a, *b, *l)?,
        Distance(a, b, d) => {
            let p = ev.point(*a)?;
            let q = ev.point(*b)?;
            vec![q.sub(p).len() - *d]
        }
        DistanceLine(g, d) => {
            let (a, b) = ev.line(*g)?;
            vec![b.sub(a).len() - *d]
        }
        PointLineDistance(p, g, d) => {
            let pt = ev.point(*p)?;
            let sd = match ev.geo(*g)? {
                G::Line(a, b) => line_sdist(a, b, pt),
                G::Circle(c, r) | G::Arc(c, r, ..) => pt.sub(c).len() - r,
                G::Point(_) => return Err(Bad),
            };
            let s = pick(&st.s1, || sgn(sd.v));
            vec![sd * s - *d]
        }
        DistanceX(a, b, d) => {
            let p = ev.point(*a)?;
            let q = ev.point(*b)?;
            vec![q.x - p.x - *d]
        }
        DistanceY(a, b, d) => {
            let p = ev.point(*a)?;
            let q = ev.point(*b)?;
            vec![q.y - p.y - *d]
        }
        LockX(p, v) => vec![ev.point(*p)?.x - *v],
        LockY(p, v) => vec![ev.point(*p)?.y - *v],
        Radius(g, v) => vec![ev.circ(*g)?.1 - *v],
        Diameter(g, v) => vec![ev.circ(*g)?.1 * 2.0 - *v],
        Angle(g, a) => match ev.geo(*g)? {
            G::Line(p, q) => {
                let u = P::c(Vec2::polar(*a));
                vec![wrap_angle(u, q.sub(p))]
            }
            G::Arc(_, _, s, e) => vec![e - s - *a],
            _ => return Err(Bad),
        },
        AngleBetween(g1, g2, a) => {
            let (a1, b1) = ev.line(*g1)?;
            let (a2, b2) = ev.line(*g2)?;
            let (d1, d2) = (b1.sub(a1), b2.sub(a2));
            let (ca, sa) = (a.cos(), a.sin());
            let u = P { x: d1.x * ca - d1.y * sa, y: d1.x * sa + d1.y * ca };
            vec![wrap_angle(u, d2)]
        }
        Block(g) => {
            if crate::is_external(*g) || *g >= ev.sk.geometry.len() {
                return Err(Bad);
            }
            let o = ev.lay.offsets[*g];
            let n = ev.lay.counts[*g];
            if st.block.len() != n {
                return Err(Bad);
            }
            (0..n).map(|i| ev.var(o + i) - st.block[i]).collect()
        }
    };
    if rows.iter().any(|r| !r.v.is_finite()) {
        return Err(Bad);
    }
    Ok(rows)
}

/// One sketcher constraint inside a [`System`].
pub(crate) struct SysCon {
    pub ci: usize,
    pub setup: Setup,
    /// First global row.
    pub start: usize,
    pub nrows: usize,
    /// Global parameter indices this constraint touches.
    pub params: Vec<usize>,
}

/// A decoupled sub-problem (connected component of the constraint graph), like
/// FreeCAD's GCS subsystems.
#[derive(Clone, Debug, Default)]
pub(crate) struct Block {
    /// Indices into `System::cons`.
    pub cons: Vec<usize>,
    /// Global row ids, in local row order.
    pub rows: Vec<usize>,
    /// Global parameter ids, in local column order.
    pub params: Vec<usize>,
}

impl Block {
    pub fn col_map(&self, n: usize) -> Vec<usize> {
        let mut col = vec![usize::MAX; n];
        for (k, &p) in self.params.iter().enumerate() {
            col[p] = k;
        }
        col
    }
}

/// The assembled equation system for a set of constraints.
pub(crate) struct System<'a> {
    pub sk: &'a Sketch,
    pub lay: &'a Layout,
    pub cons: Vec<SysCon>,
    /// row -> constraint index
    pub row_con: Vec<usize>,
    pub malformed: Vec<usize>,
}

impl<'a> System<'a> {
    /// Build the system from all constraints accepted by `include`, choosing
    /// configuration-dependent options from `x0`. Every constraint (driving or not) is
    /// checked for malformation.
    pub fn new(sk: &'a Sketch, lay: &'a Layout, x0: &DVector<f64>, include: impl Fn(&Constraint) -> bool) -> System<'a> {
        let mut cons = Vec::new();
        let mut row_con = Vec::new();
        let mut malformed = Vec::new();
        for (ci, c) in sk.constraints.iter().enumerate() {
            let mut st = Setup::default();
            if let ConstraintKind::Block(g) = c.kind {
                if g < sk.geometry.len() {
                    let o = lay.offsets[g];
                    st.block = (0..lay.counts[g]).map(|i| x0[o + i]).collect();
                }
            }
            let mut ev = Ev::new(sk, lay, x0.as_slice());
            match eval_constraint(&mut ev, c, &st) {
                Ok(rows) => {
                    if include(c) {
                        let start = row_con.len();
                        for _ in 0..rows.len() {
                            row_con.push(ci);
                        }
                        cons.push(SysCon { ci, setup: st, start, nrows: rows.len(), params: ev.slots.clone() });
                    }
                }
                Err(Bad) => malformed.push(ci),
            }
        }
        System { sk, lay, cons, row_con, malformed }
    }

    pub fn rows(&self) -> usize {
        self.row_con.len()
    }

    /// Split into decoupled blocks. `extra` lists additional parameter groups that must
    /// end up in the same block (e.g. the parameters of a dragged geometry).
    pub fn blocks(&self, extra: &[Vec<usize>]) -> Vec<Block> {
        let n = self.lay.n;
        let mut parent: Vec<usize> = (0..n).collect();
        fn find(p: &mut [usize], mut i: usize) -> usize {
            while p[i] != i {
                p[i] = p[p[i]];
                i = p[i];
            }
            i
        }
        let mut touched = vec![false; n];
        let groups = self.cons.iter().map(|c| &c.params).chain(extra.iter());
        for g in groups {
            for &q in g.iter() {
                touched[q] = true;
            }
            for w in g.windows(2) {
                let (a, b) = (find(&mut parent, w[0]), find(&mut parent, w[1]));
                if a != b {
                    parent[a.max(b)] = a.min(b);
                }
            }
        }
        let mut root_block: Vec<usize> = vec![usize::MAX; n];
        let mut blocks: Vec<Block> = Vec::new();
        for q in 0..n {
            if !touched[q] {
                continue;
            }
            let r = find(&mut parent, q);
            if root_block[r] == usize::MAX {
                root_block[r] = blocks.len();
                blocks.push(Block::default());
            }
            blocks[root_block[r]].params.push(q);
        }
        for (k, c) in self.cons.iter().enumerate() {
            // constraints touching no free parameter (e.g. only external geometry) go
            // into their own block
            let b = match c.params.first() {
                Some(&q) => root_block[find(&mut parent, q)],
                None => {
                    blocks.push(Block::default());
                    blocks.len() - 1
                }
            };
            blocks[b].cons.push(k);
            blocks[b].rows.extend(c.start..c.start + c.nrows);
        }
        blocks
    }

    /// Residuals and Jacobian of one block (local rows x local params).
    pub fn eval_block(&self, b: &Block, col: &[usize], x: &DVector<f64>) -> (DVector<f64>, DMatrix<f64>) {
        let m = b.rows.len();
        let mut f = DVector::zeros(m);
        let mut j = DMatrix::zeros(m, b.params.len());
        let mut r0 = 0;
        for &k in &b.cons {
            let c = &self.cons[k];
            let mut ev = Ev::new(self.sk, self.lay, x.as_slice());
            let rows = eval_constraint(&mut ev, &self.sk.constraints[c.ci], &c.setup).unwrap_or_else(|_| vec![D::c(f64::NAN); c.nrows]);
            for (i, r) in rows.iter().enumerate() {
                f[r0 + i] = r.v;
                for (s, &gi) in ev.slots.iter().enumerate() {
                    j[(r0 + i, col[gi])] += r.g[s];
                }
            }
            r0 += c.nrows;
        }
        (f, j)
    }

    /// All residuals (global row order).
    #[allow(dead_code)]
    pub fn residual(&self, x: &DVector<f64>) -> DVector<f64> {
        let mut f = DVector::zeros(self.rows());
        for c in &self.cons {
            let mut ev = Ev::new(self.sk, self.lay, x.as_slice());
            let rows = eval_constraint(&mut ev, &self.sk.constraints[c.ci], &c.setup).unwrap_or_else(|_| vec![D::c(f64::NAN); c.nrows]);
            for (i, r) in rows.iter().enumerate() {
                f[c.start + i] = r.v;
            }
        }
        f
    }

    /// Residual row values of one constraint at x.
    pub fn constraint_rows(&self, ci: usize, x: &DVector<f64>) -> Option<Vec<f64>> {
        let c = self.cons.iter().find(|c| c.ci == ci)?;
        let mut ev = Ev::new(self.sk, self.lay, x.as_slice());
        eval_constraint(&mut ev, &self.sk.constraints[ci], &c.setup).ok().map(|r| r.iter().map(|d| d.v).collect())
    }
}

/// Parameters of a geometry (empty for external geometry).
pub(crate) fn geo_params(lay: &Layout, g: GeoId) -> Vec<usize> {
    if g < lay.offsets.len() {
        (lay.offsets[g]..lay.offsets[g] + lay.counts[g]).collect()
    } else {
        vec![]
    }
}

/// Drag residual rows (target equations) for interactive point moves, with the
/// Jacobian over the block's local columns.
pub(crate) fn eval_drag(sk: &Sketch, lay: &Layout, col: &[usize], ncols: usize, x: &DVector<f64>, p: PointRef, target: Vec2) -> R<(DVector<f64>, DMatrix<f64>)> {
    let mut ev = Ev::new(sk, lay, x.as_slice());
    let g = ev.geo(p.geo)?;
    let t = P::c(target);
    let rows = match (p.pos, g) {
        (PointPos::Edge, G::Line(a, b)) => {
            let m = a.add(b).scale(D::c(0.5));
            vec![m.x - t.x, m.y - t.y]
        }
        (PointPos::Edge, G::Circle(c, r) | G::Arc(c, r, ..)) => vec![t.sub(c).len() - r],
        (PointPos::Edge, G::Point(q)) => vec![q.x - t.x, q.y - t.y],
        (pos, g) => {
            let q = point_of(&g, p.geo, pos)?;
            vec![q.x - t.x, q.y - t.y]
        }
    };
    let mut f = DVector::zeros(rows.len());
    let mut j = DMatrix::zeros(rows.len(), ncols);
    for (k, r) in rows.iter().enumerate() {
        f[k] = r.v;
        for (s, &gi) in ev.slots.iter().enumerate() {
            if col[gi] != usize::MAX {
                j[(k, col[gi])] += r.g[s];
            }
        }
    }
    Ok((f, j))
}
