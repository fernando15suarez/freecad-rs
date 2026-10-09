//! Closed profile extraction (wires for Pad/Pocket), loop nesting and tessellation.

use crate::{Curve, GeoId, Sketch, Vec2};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;
use std::fmt;

/// Endpoint coincidence tolerance (FreeCAD's Precision::Confusion is 1e-7; sketches
/// are solved to ~1e-9, we allow a bit more slack).
pub const LOOP_TOLERANCE: f64 = 1e-6;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LoopEdge {
    pub geo: GeoId,
    /// Traversed end -> start.
    pub reversed: bool,
}

/// Ordered, head-to-tail connected, closed sequence of edges.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProfileLoop {
    pub edges: Vec<LoopEdge>,
}

/// An outer wire with its holes (one planar face).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProfileFace {
    pub outer: ProfileLoop,
    pub holes: Vec<ProfileLoop>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProfileError {
    /// No profile geometry at all.
    Empty,
    /// An edge endpoint is not connected to any other edge.
    OpenWire { geo: GeoId },
    /// More than two edges meet at a vertex.
    BranchingWire { geo: GeoId },
    /// Two edges cross or overlap away from shared endpoints.
    Intersecting { a: GeoId, b: GeoId },
    /// Zero-length line or zero-radius circle/arc.
    DegenerateEdge { geo: GeoId },
}

impl fmt::Display for ProfileError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ProfileError::Empty => write!(f, "sketch has no closed profile geometry"),
            ProfileError::OpenWire { geo } => write!(f, "open wire: geometry {geo} has an unconnected endpoint"),
            ProfileError::BranchingWire { geo } => {
                write!(f, "more than two edges meet at an endpoint of geometry {geo}")
            }
            ProfileError::Intersecting { a, b } => write!(f, "geometries {a} and {b} intersect"),
            ProfileError::DegenerateEdge { geo } => write!(f, "geometry {geo} is degenerate"),
        }
    }
}

impl std::error::Error for ProfileError {}

/// Polyline approximation of a curve: `segments` segments for circles/arcs
/// (circles return a closed polyline, first point repeated), 1 for lines, the point itself
/// for points.
pub fn tessellate_curve(c: &Curve, segments: usize) -> Vec<Vec2> {
    let segments = segments.max(1);
    match c {
        Curve::Point { p } => vec![*p],
        Curve::Line { a, b } => vec![*a, *b],
        Curve::Circle { .. } | Curve::Arc { .. } => (0..=segments).map(|i| c.eval(i as f64 / segments as f64)).collect(),
    }
}

/// Like [`tessellate_curve`] but optionally reversed (for walking loops).
pub fn tessellate_edge(c: &Curve, reversed: bool, segments: usize) -> Vec<Vec2> {
    let mut v = tessellate_curve(c, segments);
    if reversed {
        v.reverse();
    }
    v
}

fn segs_for(c: &Curve, per_turn: usize) -> usize {
    match c {
        Curve::Line { .. } | Curve::Point { .. } => 1,
        _ => ((c.sweep() / (2.0 * PI)) * per_turn as f64).ceil().max(2.0) as usize,
    }
}

pub(crate) fn loop_polygon(sk: &Sketch, l: &ProfileLoop, per_turn: usize) -> Vec<Vec2> {
    let mut pts: Vec<Vec2> = Vec::new();
    for e in &l.edges {
        let c = &sk.geometry[e.geo].curve;
        let seg = tessellate_edge(c, e.reversed, segs_for(c, per_turn));
        if pts.is_empty() {
            pts.extend(seg);
        } else {
            pts.extend(seg.into_iter().skip(1));
        }
    }
    if let (Some(f), Some(l)) = (pts.first().copied(), pts.last().copied()) {
        if f.dist(l) > 0.0 {
            if f.dist(l) < LOOP_TOLERANCE * 10.0 {
                *pts.last_mut().unwrap() = f;
            } else {
                pts.push(f);
            }
        }
    }
    pts
}

pub(crate) fn loop_signed_area(sk: &Sketch, l: &ProfileLoop) -> f64 {
    let mut a2 = 0.0; // twice the area
    for e in &l.edges {
        let c = &sk.geometry[e.geo].curve;
        let s = if e.reversed { -1.0 } else { 1.0 };
        a2 += s
            * match c {
                Curve::Line { a, b } => a.cross(*b),
                Curve::Circle { c, r } => 2.0 * PI * r * r + 0.0 * c.x,
                Curve::Arc { c, r, start, .. } => {
                    let t0 = *start;
                    let t1 = start + c_sweep(sk, e.geo);
                    r * c.x * (t1.sin() - t0.sin()) - r * c.y * (t1.cos() - t0.cos()) + r * r * (t1 - t0)
                }
                Curve::Point { .. } => 0.0,
            };
    }
    0.5 * a2
}

fn c_sweep(sk: &Sketch, g: GeoId) -> f64 {
    sk.geometry[g].curve.sweep()
}

/// Even-odd point in polygon test against a closed polyline.
pub fn loop_contains_point(poly: &[Vec2], p: Vec2) -> bool {
    let mut inside = false;
    let n = poly.len();
    if n < 3 {
        return false;
    }
    let mut j = n - 1;
    for i in 0..n {
        let (a, b) = (poly[i], poly[j]);
        if (a.y > p.y) != (b.y > p.y) {
            let x = (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x;
            if p.x < x {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}

/// Bullseye nesting for closed polygons given with a sample point on each boundary.
/// Returns (outer, holes) pairs.
pub fn nest_loop_polygons(polys: &[Vec<Vec2>], samples: &[Vec2], areas: &[f64]) -> Vec<(usize, Vec<usize>)> {
    let n = polys.len();
    let mut parents: Vec<Vec<usize>> = vec![vec![]; n];
    for i in 0..n {
        for j in 0..n {
            if i != j && areas[j] > areas[i] && loop_contains_point(&polys[j], samples[i]) {
                parents[i].push(j);
            }
        }
    }
    let depth: Vec<usize> = parents.iter().map(|p| p.len()).collect();
    let direct: Vec<Option<usize>> = parents
        .iter()
        .map(|ps| ps.iter().copied().min_by(|&a, &b| areas[a].partial_cmp(&areas[b]).unwrap()))
        .collect();
    let mut out = Vec::new();
    for o in 0..n {
        if depth[o].is_multiple_of(2) {
            let holes: Vec<usize> = (0..n).filter(|&h| depth[h] == depth[o] + 1 && direct[h] == Some(o)).collect();
            out.push((o, holes));
        }
    }
    out
}

pub(crate) fn nest_loops(sk: &Sketch, loops: &[ProfileLoop]) -> Vec<(usize, Vec<usize>)> {
    let polys: Vec<Vec<Vec2>> = loops.iter().map(|l| loop_polygon(sk, l, 256)).collect();
    let samples: Vec<Vec2> = loops
        .iter()
        .map(|l| sk.geometry[l.edges[0].geo].curve.eval(0.5))
        .collect();
    let areas: Vec<f64> = loops.iter().map(|l| loop_signed_area(sk, l).abs()).collect();
    nest_loop_polygons(&polys, &samples, &areas)
}

struct Edge {
    geo: GeoId,
    curve: Curve,
    ends: Option<(Vec2, Vec2)>,
}

pub(crate) fn profile_loops(sk: &Sketch) -> Result<Vec<ProfileLoop>, ProfileError> {
    let tol = LOOP_TOLERANCE;
    let mut edges: Vec<Edge> = Vec::new();
    for (i, g) in sk.geometry.iter().enumerate() {
        if g.construction {
            continue;
        }
        match &g.curve {
            Curve::Point { .. } => {}
            Curve::Line { a, b } => {
                if a.dist(*b) < tol {
                    return Err(ProfileError::DegenerateEdge { geo: i });
                }
                edges.push(Edge { geo: i, curve: g.curve.clone(), ends: Some((*a, *b)) });
            }
            Curve::Circle { r, .. } => {
                if r.abs() < tol {
                    return Err(ProfileError::DegenerateEdge { geo: i });
                }
                edges.push(Edge { geo: i, curve: g.curve.clone(), ends: None });
            }
            Curve::Arc { r, .. } => {
                if r.abs() < tol {
                    return Err(ProfileError::DegenerateEdge { geo: i });
                }
                let c = &g.curve;
                edges.push(Edge { geo: i, curve: c.clone(), ends: Some((c.start_point().unwrap(), c.end_point().unwrap())) });
            }
        }
    }
    if edges.is_empty() {
        return Err(ProfileError::Empty);
    }

    // vertices
    let mut verts: Vec<Vec2> = Vec::new();
    let vid = |p: Vec2, verts: &mut Vec<Vec2>| -> usize {
        if let Some(k) = verts.iter().position(|v| v.dist(p) <= tol) {
            k
        } else {
            verts.push(p);
            verts.len() - 1
        }
    };
    // per edge: (start vertex, end vertex)
    let mut ev: Vec<Option<(usize, usize)>> = Vec::new();
    for e in &edges {
        ev.push(e.ends.map(|(a, b)| (vid(a, &mut verts), vid(b, &mut verts))));
    }
    // adjacency: vertex -> [(edge, is_start)]
    let mut adj: Vec<Vec<(usize, bool)>> = vec![vec![]; verts.len()];
    for (k, v) in ev.iter().enumerate() {
        if let Some((s, e)) = v {
            adj[*s].push((k, true));
            adj[*e].push((k, false));
        }
    }
    for a in &adj {
        if a.len() == 1 {
            return Err(ProfileError::OpenWire { geo: edges[a[0].0].geo });
        }
        if a.len() > 2 {
            return Err(ProfileError::BranchingWire { geo: edges[a[0].0].geo });
        }
    }

    check_intersections(&edges, &verts, tol)?;

    let mut loops = Vec::new();
    let mut used = vec![false; edges.len()];
    for k in 0..edges.len() {
        if used[k] {
            continue;
        }
        let Some((s0, _)) = ev[k] else {
            used[k] = true;
            loops.push(ProfileLoop { edges: vec![LoopEdge { geo: edges[k].geo, reversed: false }] });
            continue;
        };
        let mut l = Vec::new();
        let (mut cur, mut rev) = (k, false);
        loop {
            used[cur] = true;
            l.push(LoopEdge { geo: edges[cur].geo, reversed: rev });
            let (s, e) = ev[cur].unwrap();
            let (arrive, arrive_is_start) = if rev { (s, true) } else { (e, false) };
            if arrive == s0 {
                break;
            }
            // the other adjacency entry at this vertex
            let other = adj[arrive].iter().copied().find(|&(ek, is_s)| !(ek == cur && is_s == arrive_is_start)).unwrap();
            cur = other.0;
            rev = !other.1;
            if used[cur] {
                break;
            }
        }
        loops.push(ProfileLoop { edges: l });
    }
    Ok(loops)
}

fn on_arc(c: &Curve, p: Vec2, tol: f64) -> bool {
    match c {
        Curve::Arc { c: ctr, r, start, .. } => {
            let a = (p.y - ctr.y).atan2(p.x - ctr.x);
            let rel = (a - start).rem_euclid(2.0 * PI);
            let sw = c.sweep();
            let atol = tol / r.abs().max(1e-12);
            rel <= sw + atol || rel >= 2.0 * PI - atol
        }
        _ => true,
    }
}

/// Intersection points (sampled; overlapping curves yield their overlap endpoints plus a
/// marker point) between two edges.
fn intersections(a: &Curve, b: &Curve, tol: f64) -> Vec<Vec2> {
    match (a, b) {
        (Curve::Line { a: p, b: q }, Curve::Line { a: r, b: s }) => {
            let d1 = *q - *p;
            let d2 = *s - *r;
            let den = d1.cross(d2);
            let l1 = d1.length();
            let l2 = d2.length();
            if den.abs() <= 1e-12 * l1 * l2 {
                // parallel: overlapping if collinear and ranges overlap
                if (d1.cross(*r - *p) / l1).abs() > tol {
                    return vec![];
                }
                let u = d1 * (1.0 / l1);
                let t0 = u.dot(*r - *p);
                let t1 = u.dot(*s - *p);
                let (lo, hi) = (t0.min(t1).max(0.0), t0.max(t1).min(l1));
                if hi - lo > tol {
                    return vec![*p + u * ((lo + hi) * 0.5)];
                }
                if hi - lo > -tol {
                    return vec![*p + u * lo];
                }
                return vec![];
            }
            let t = (*r - *p).cross(d2) / den;
            let w = (*r - *p).cross(d1) / den;
            let et = tol / l1;
            let ew = tol / l2;
            if t >= -et && t <= 1.0 + et && w >= -ew && w <= 1.0 + ew {
                vec![*p + d1 * t]
            } else {
                vec![]
            }
        }
        (Curve::Line { a: p, b: q }, Curve::Circle { c, r } | Curve::Arc { c, r, .. }) => {
            let d = *q - *p;
            let f = *p - *c;
            let aa = d.dot(d);
            let bb = 2.0 * f.dot(d);
            let cc = f.dot(f) - r * r;
            let disc = bb * bb - 4.0 * aa * cc;
            let l = aa.sqrt();
            // tangency tolerance
            let dist_line = (d.cross(*c - *p) / l).abs();
            let mut ts = vec![];
            if (dist_line - r.abs()).abs() <= tol {
                ts.push(-bb / (2.0 * aa));
            } else if disc >= 0.0 {
                let sq = disc.sqrt();
                ts.push((-bb - sq) / (2.0 * aa));
                ts.push((-bb + sq) / (2.0 * aa));
            }
            let et = tol / l;
            ts.into_iter()
                .filter(|t| *t >= -et && *t <= 1.0 + et)
                .map(|t| *p + d * t)
                .filter(|pt| on_arc(b, *pt, tol))
                .collect()
        }
        (Curve::Circle { .. } | Curve::Arc { .. }, Curve::Line { .. }) => intersections(b, a, tol),
        (Curve::Circle { c: c1, r: r1 } | Curve::Arc { c: c1, r: r1, .. }, Curve::Circle { c: c2, r: r2 } | Curve::Arc { c: c2, r: r2, .. }) => {
            let (r1, r2) = (r1.abs(), r2.abs());
            let d = c1.dist(*c2);
            if d <= tol && (r1 - r2).abs() <= tol {
                // same circle: overlap if the arcs share any stretch
                let mut pts = vec![];
                for (x, y) in [(a, b), (b, a)] {
                    for t in [0.0, 0.25, 0.5, 0.75, 1.0] {
                        let p = x.eval(t);
                        if on_arc(y, p, tol) {
                            pts.push(p);
                        }
                    }
                }
                return pts;
            }
            if d > r1 + r2 + tol || d < (r1 - r2).abs() - tol || d <= tol {
                return vec![];
            }
            let aa = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
            let h = (r1 * r1 - aa * aa).max(0.0).sqrt();
            let u = (*c2 - *c1) * (1.0 / d);
            let m = *c1 + u * aa;
            let tangent = (d - (r1 + r2)).abs() <= tol || (d - (r1 - r2).abs()).abs() <= tol;
            let pts = if tangent || h <= tol { vec![m] } else { vec![m + u.perp() * h, m - u.perp() * h] };
            pts.into_iter().filter(|p| on_arc(a, *p, tol) && on_arc(b, *p, tol)).collect()
        }
        _ => vec![],
    }
}

fn check_intersections(edges: &[Edge], _verts: &[Vec2], tol: f64) -> Result<(), ProfileError> {
    let ends = |e: &Edge| -> Vec<Vec2> {
        match e.ends {
            Some((a, b)) => vec![a, b],
            None => vec![],
        }
    };
    for i in 0..edges.len() {
        for j in (i + 1)..edges.len() {
            let (ei, ej) = (&edges[i], &edges[j]);
            let ci = ends(ei);
            let cj = ends(ej);
            for p in intersections(&ei.curve, &ej.curve, tol) {
                let shared = ci.iter().any(|v| v.dist(p) <= tol * 10.0) && cj.iter().any(|v| v.dist(p) <= tol * 10.0);
                if !shared {
                    return Err(ProfileError::Intersecting { a: ei.geo, b: ej.geo });
                }
            }
        }
    }
    Ok(())
}
