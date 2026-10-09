//! Geometry creation tools (FreeCAD DrawSketchHandler*): snapping and auto-constraints.

use super::{geom, pick, Proj, SketchEditor};
use eframe::egui::Pos2;
use fc_part::params::OnUsers;
use fc_part::Document;
use fc_sketch::{ConstraintKind as K, Curve, Geometry, GeoId, PointRef, Sketch, Vec2, H_AXIS, V_AXIS};
use std::f64::consts::PI;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tool {
    Select,
    Point,
    Line,
    Polyline,
    Rectangle,
    CenterRectangle,
    Circle,
    Arc,
    Arc3,
    Slot,
}

impl Tool {
    /// (tool, toolbar label, shortcut shown in the tooltip)
    pub const ALL: [(Tool, &'static str, &'static str); 10] = [
        (Tool::Select, "Select", "Esc"),
        (Tool::Point, "Point", "G, Y"),
        (Tool::Line, "Line", "G, L"),
        (Tool::Polyline, "Polyline", "G, M"),
        (Tool::Rectangle, "Rectangle", "G, R"),
        (Tool::CenterRectangle, "Ctr. rect", "G, V"),
        (Tool::Circle, "Circle", "G, C"),
        (Tool::Arc, "Arc", "G, A"),
        (Tool::Arc3, "Arc 3pt", "G, 3"),
        (Tool::Slot, "Slot", "G, S"),
    ];

    pub fn parse(s: &str) -> Option<Tool> {
        Some(match s.to_ascii_lowercase().as_str() {
            "select" | "none" => Tool::Select,
            "point" => Tool::Point,
            "line" => Tool::Line,
            "polyline" | "lineset" => Tool::Polyline,
            "rectangle" | "rect" => Tool::Rectangle,
            "crect" | "centerrect" | "centeredrectangle" => Tool::CenterRectangle,
            "circle" => Tool::Circle,
            "arc" => Tool::Arc,
            "arc3" | "3arc" => Tool::Arc3,
            "slot" => Tool::Slot,
            _ => return None,
        })
    }

    pub fn hint(&self, step: usize, chaining: bool) -> &'static str {
        match (self, step) {
            (Tool::Select, _) => "",
            (Tool::Point, _) => "Point: click to place a point",
            (Tool::Line, 0) => "Line: click the start point",
            (Tool::Line, _) => "Line: click the end point (Esc cancels)",
            (Tool::Polyline, 0) if !chaining => "Polyline: click the first point",
            (Tool::Polyline, _) => "Polyline: click the next point; click the first point to close; right-click or Esc ends",
            (Tool::Rectangle, 0) => "Rectangle: click the first corner",
            (Tool::Rectangle, _) => "Rectangle: click the opposite corner",
            (Tool::CenterRectangle, 0) => "Centered rectangle: click the center",
            (Tool::CenterRectangle, _) => "Centered rectangle: click a corner",
            (Tool::Circle, 0) => "Circle: click the center",
            (Tool::Circle, _) => "Circle: click a point on the rim",
            (Tool::Arc, 0) => "Arc: click the center",
            (Tool::Arc, 1) => "Arc: click the start point",
            (Tool::Arc, _) => "Arc: move around the center and click the end point",
            (Tool::Arc3, 0) => "3-point arc: click the start point",
            (Tool::Arc3, 1) => "3-point arc: click the end point",
            (Tool::Arc3, _) => "3-point arc: click a point on the arc",
            (Tool::Slot, 0) => "Slot: click the first center",
            (Tool::Slot, 1) => "Slot: click the second center",
            (Tool::Slot, _) => "Slot: click to set the radius",
        }
    }
}

/// What a cursor position snapped to (and therefore which auto-constraint it gets).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Snap {
    None,
    Point(PointRef),
    Curve(GeoId),
    Grid,
}

#[derive(Clone, Copy, Debug)]
pub struct SnapPt {
    pub p: Vec2,
    pub snap: Snap,
}

pub struct ToolState {
    pub tool: Tool,
    pub pts: Vec<SnapPt>,
    /// Current (snapped) cursor.
    pub cur: Option<SnapPt>,
    /// Arc tool: accumulated sweep while moving around the center.
    pub arc_sweep: f64,
    arc_prev: f64,
    /// Polyline: previous segment.
    pub poly_prev: Option<GeoId>,
    pub poly_first: Option<PointRef>,
}

impl ToolState {
    pub fn new(tool: Tool) -> Self {
        ToolState { tool, pts: vec![], cur: None, arc_sweep: 0.0, arc_prev: 0.0, poly_prev: None, poly_first: None }
    }
    pub fn reset(&mut self) {
        *self = ToolState { cur: self.cur, ..ToolState::new(self.tool) };
    }
}

/// Auto Horizontal/Vertical for a nearly axis-aligned segment (FreeCAD: 2 degrees).
pub fn hv_auto(a: Vec2, b: Vec2) -> Option<bool> {
    let d = b - a;
    if d.length() < 1e-9 {
        return None;
    }
    let ang = d.y.atan2(d.x).abs();
    let tol = 2f64.to_radians();
    if ang < tol || PI - ang < tol {
        Some(true)
    } else if (ang - PI / 2.0).abs() < tol {
        Some(false)
    } else {
        None
    }
}

/// Grid spacing (sketch units) giving at least ~18 px between lines (1-2-5 steps).
pub fn grid_spacing(ppu: f64) -> f64 {
    let target = 18.0 / ppu;
    let base = 10f64.powf(target.log10().floor());
    for m in [1.0, 2.0, 5.0, 10.0] {
        if base * m >= target {
            return base * m;
        }
    }
    base * 10.0
}

fn snap_constraint(s: &SnapPt, target: PointRef) -> Option<K> {
    match s.snap {
        Snap::Point(p) => Some(K::Coincident(p, target)),
        Snap::Curve(g) => Some(K::PointOnObject(target, g)),
        _ => None,
    }
}

impl SketchEditor {
    pub(crate) fn snap_at(&self, sk: &Sketch, pj: &Proj, pos: Pos2) -> SnapPt {
        let raw = pj.w(pos).unwrap_or(Vec2::ZERO);
        if self.auto_constraints {
            if let Some(pr) = pick::nearest_point(sk, pj, pos, pick::POINT_PX) {
                return SnapPt { p: sk.point(pr), snap: Snap::Point(pr) };
            }
            if let Some((g, p)) = pick::nearest_edge(sk, pj, pos, pick::EDGE_PX, true) {
                return SnapPt { p, snap: Snap::Curve(g) };
            }
        }
        if self.grid {
            let s = grid_spacing(pj.ppu());
            return SnapPt { p: Vec2::new((raw.x / s).round() * s, (raw.y / s).round() * s), snap: Snap::Grid };
        }
        SnapPt { p: raw, snap: Snap::None }
    }

    pub(crate) fn tool_move(&mut self, doc: &Document, pj: &Proj, pos: Pos2) {
        let sp = self.snap_at(self.sk(doc), pj, pos);
        self.tool.cur = Some(sp);
        if self.tool.tool == Tool::Arc && self.tool.pts.len() == 2 {
            let c = self.tool.pts[0].p;
            let a = (sp.p - c).y.atan2((sp.p - c).x);
            let mut d = a - self.tool.arc_prev;
            while d > PI {
                d -= 2.0 * PI;
            }
            while d < -PI {
                d += 2.0 * PI;
            }
            self.tool.arc_sweep += d;
            self.tool.arc_prev = a;
        }
    }

    pub(crate) fn tool_right_click(&mut self, doc: &mut Document, _pj: &Proj, _pos: Pos2) {
        if self.tool.tool == Tool::Select {
            self.sel.clear();
        } else if !self.tool.pts.is_empty() || self.tool.poly_prev.is_some() {
            self.tool.reset();
        } else {
            self.set_tool(Tool::Select);
        }
        let _ = doc;
    }

    pub(crate) fn tool_press(&mut self, doc: &mut Document, pj: &Proj, pos: Pos2) {
        self.tool_move(doc, pj, pos);
        let sp = self.tool.cur.unwrap();
        let n = self.tool.pts.len();
        let tiny = 1e-6;
        match self.tool.tool {
            Tool::Select => {}
            Tool::Point => {
                self.create(doc, vec![Geometry::point(sp.p)], vec![], |b| snap_constraint(&sp, PointRef::start(b)).into_iter().collect());
            }
            Tool::Line => {
                if n == 0 {
                    self.tool.pts.push(sp);
                    return;
                }
                let s0 = self.tool.pts[0];
                if s0.p.dist(sp.p) < tiny {
                    return;
                }
                self.tool.pts.clear();
                self.create(doc, vec![Geometry::line(s0.p, sp.p)], vec![], |b| {
                    let mut v: Vec<K> = vec![];
                    v.extend(snap_constraint(&s0, PointRef::start(b)));
                    v.extend(snap_constraint(&sp, PointRef::end(b)));
                    match hv_auto(s0.p, sp.p) {
                        Some(true) => v.push(K::Horizontal(b)),
                        Some(false) => v.push(K::Vertical(b)),
                        None => {}
                    }
                    v
                });
            }
            Tool::Polyline => {
                let start = match self.tool.poly_prev {
                    Some(g) => self.sk(doc).try_point(PointRef::end(g)).map(|p| SnapPt { p, snap: Snap::Point(PointRef::end(g)) }),
                    None => self.tool.pts.first().copied(),
                };
                let Some(s0) = start else {
                    self.tool.pts.push(sp);
                    return;
                };
                if s0.p.dist(sp.p) < tiny {
                    return;
                }
                let first = self.tool.poly_first;
                let closes = matches!(sp.snap, Snap::Point(p) if Some(p) == first);
                let b = self.create(doc, vec![Geometry::line(s0.p, sp.p)], vec![], |b| {
                    let mut v: Vec<K> = vec![];
                    v.extend(snap_constraint(&s0, PointRef::start(b)));
                    v.extend(snap_constraint(&sp, PointRef::end(b)));
                    match hv_auto(s0.p, sp.p) {
                        Some(true) => v.push(K::Horizontal(b)),
                        Some(false) => v.push(K::Vertical(b)),
                        None => {}
                    }
                    v
                });
                if self.tool.poly_first.is_none() {
                    self.tool.poly_first = Some(PointRef::start(b));
                }
                self.tool.pts.clear();
                if closes {
                    self.tool.reset();
                } else {
                    self.tool.poly_prev = Some(b);
                }
            }
            Tool::Rectangle | Tool::CenterRectangle => {
                if n == 0 {
                    self.tool.pts.push(sp);
                    return;
                }
                let s0 = self.tool.pts[0];
                let centered = self.tool.tool == Tool::CenterRectangle;
                let (c0, c2) = if centered { (s0.p * 2.0 - sp.p, sp.p) } else { (s0.p, sp.p) };
                if (c2.x - c0.x).abs() < tiny || (c2.y - c0.y).abs() < tiny {
                    return;
                }
                self.tool.pts.clear();
                let c = [c0, Vec2::new(c2.x, c0.y), c2, Vec2::new(c0.x, c2.y)];
                let mut geos: Vec<Geometry> = (0..4).map(|i| Geometry::line(c[i], c[(i + 1) % 4])).collect();
                if centered {
                    geos.push(Geometry::construction(Curve::Point { p: s0.p }));
                }
                let structural = move |b: GeoId| -> Vec<K> {
                    let mut v = vec![];
                    for i in 0..4 {
                        v.push(K::Coincident(PointRef::end(b + i), PointRef::start(b + (i + 1) % 4)));
                    }
                    v.push(K::Horizontal(b));
                    v.push(K::Horizontal(b + 2));
                    v.push(K::Vertical(b + 1));
                    v.push(K::Vertical(b + 3));
                    if centered {
                        v.push(K::Symmetric(PointRef::start(b), PointRef::start(b + 2), PointRef::start(b + 4)));
                    }
                    v
                };
                let st = structural(self.sk(doc).geometry.len());
                self.create(doc, geos, st, |b| {
                    let mut v: Vec<K> = vec![];
                    if centered {
                        v.extend(snap_constraint(&s0, PointRef::start(b + 4)));
                    } else {
                        v.extend(snap_constraint(&s0, PointRef::start(b)));
                    }
                    v.extend(snap_constraint(&sp, PointRef::start(b + 2)));
                    v
                });
            }
            Tool::Circle => {
                if n == 0 {
                    self.tool.pts.push(sp);
                    return;
                }
                let s0 = self.tool.pts[0];
                let r = s0.p.dist(sp.p);
                if r < tiny {
                    return;
                }
                self.tool.pts.clear();
                self.create(doc, vec![Geometry::circle(s0.p, r)], vec![], |b| {
                    let mut v: Vec<K> = vec![];
                    v.extend(snap_constraint(&s0, PointRef::mid(b)));
                    if let Snap::Point(p) = sp.snap {
                        v.push(K::PointOnObject(p, b));
                    }
                    v
                });
            }
            Tool::Arc => {
                if n == 0 {
                    self.tool.pts.push(sp);
                    return;
                }
                let c = self.tool.pts[0].p;
                if n == 1 {
                    if c.dist(sp.p) < tiny {
                        return;
                    }
                    self.tool.pts.push(sp);
                    self.tool.arc_prev = (sp.p - c).y.atan2((sp.p - c).x);
                    self.tool.arc_sweep = 0.0;
                    return;
                }
                let (s0, s1) = (self.tool.pts[0], self.tool.pts[1]);
                let r = c.dist(s1.p);
                let sa = (s1.p - c).y.atan2((s1.p - c).x);
                let sweep = self.tool.arc_sweep;
                if sweep.abs() < 1e-3 {
                    return;
                }
                let sweep = sweep.clamp(-2.0 * PI + 1e-3, 2.0 * PI - 1e-3);
                let (st, en, first_is_start) = if sweep > 0.0 { (sa, sa + sweep, true) } else { (sa + sweep, sa, false) };
                self.tool.pts.clear();
                self.create(doc, vec![Geometry::arc(c, r, st, en)], vec![], |b| {
                    let mut v: Vec<K> = vec![];
                    v.extend(snap_constraint(&s0, PointRef::mid(b)));
                    let (p1, p2) = if first_is_start { (PointRef::start(b), PointRef::end(b)) } else { (PointRef::end(b), PointRef::start(b)) };
                    v.extend(snap_constraint(&s1, p1));
                    v.extend(snap_constraint(&sp, p2));
                    v
                });
            }
            Tool::Arc3 => {
                if n < 2 {
                    if n == 1 && self.tool.pts[0].p.dist(sp.p) < tiny {
                        return;
                    }
                    self.tool.pts.push(sp);
                    return;
                }
                let (s0, s1) = (self.tool.pts[0], self.tool.pts[1]);
                let Some((arc, first_is_start)) = arc3(s0.p, s1.p, sp.p) else { return };
                self.tool.pts.clear();
                self.create(doc, vec![Geometry::new(arc)], vec![], |b| {
                    let mut v: Vec<K> = vec![];
                    let (p1, p2) = if first_is_start { (PointRef::start(b), PointRef::end(b)) } else { (PointRef::end(b), PointRef::start(b)) };
                    v.extend(snap_constraint(&s0, p1));
                    v.extend(snap_constraint(&s1, p2));
                    v
                });
            }
            Tool::Slot => {
                if n < 2 {
                    if n == 1 && self.tool.pts[0].p.dist(sp.p) < tiny {
                        return;
                    }
                    self.tool.pts.push(sp);
                    return;
                }
                let (s0, s1) = (self.tool.pts[0], self.tool.pts[1]);
                let r = slot_radius(s0.p, s1.p, sp.p);
                if r < tiny {
                    return;
                }
                self.tool.pts.clear();
                let geos = slot_geometry(s0.p, s1.p, r);
                let st = |b: GeoId| {
                    vec![
                        K::TangentAt(PointRef::start(b), PointRef::start(b + 2)),
                        K::TangentAt(PointRef::end(b), PointRef::start(b + 3)),
                        K::TangentAt(PointRef::end(b + 1), PointRef::end(b + 2)),
                        K::TangentAt(PointRef::start(b + 1), PointRef::end(b + 3)),
                        K::Equal(b, b + 1),
                    ]
                };
                let structural = st(self.sk(doc).geometry.len());
                self.create(doc, geos, structural, |b| {
                    let mut v: Vec<K> = vec![];
                    v.extend(snap_constraint(&s0, PointRef::mid(b)));
                    v.extend(snap_constraint(&s1, PointRef::mid(b + 1)));
                    match hv_auto(s0.p, s1.p) {
                        Some(true) => v.push(K::Horizontal(b + 2)),
                        Some(false) => v.push(K::Vertical(b + 2)),
                        None => {}
                    }
                    v
                });
            }
        }
    }

    /// Add geometry + structural constraints + auto-constraints in one undo step. Auto
    /// constraints that would make the sketch redundant/conflicting are dropped (FreeCAD's
    /// "avoid redundant auto-constraints"). Returns the first new GeoId.
    fn create(&mut self, doc: &mut Document, geos: Vec<Geometry>, structural: Vec<K>, autos: impl FnOnce(GeoId) -> Vec<K>) -> GeoId {
        self.push_undo(doc);
        let constr = self.construction;
        let Some(sk) = doc.sketch_mut(self.feature) else { return 0 };
        let base = sk.geometry.len();
        for mut g in geos {
            // the centered rectangle's center point is always construction
            if !(matches!(g.curve, Curve::Point { .. }) && g.construction) {
                g.construction = constr;
            }
            sk.add_geometry(g);
        }
        for k in structural {
            sk.add_constraint(k);
        }
        let first_auto = sk.constraints.len();
        // Coincident with a point of the new geometry itself is meaningless; skip self-references.
        for k in autos(base) {
            if let K::Coincident(a, b) = &k {
                if a == b {
                    continue;
                }
            }
            sk.add_constraint(k);
        }
        // drop offending auto-constraints
        for _ in 0..8 {
            let Some(sk) = doc.sketch_mut(self.feature) else { break };
            let rep = sk.diagnose();
            let bad: Vec<usize> = rep.conflicting.iter().chain(&rep.redundant).copied().filter(|&c| c >= first_auto).collect();
            let Some(&worst) = bad.iter().max() else { break };
            let _ = doc.delete_constraints(self.feature, &[worst], OnUsers::Freeze);
        }
        self.resolve(doc);
        base
    }
}

/// Arc through start, end and a point on it. Returns (arc, true if `a` is the arc's start).
pub fn arc3(a: Vec2, b: Vec2, on: Vec2) -> Option<(Curve, bool)> {
    let (c, r) = geom::circle_through(a, on, b)?;
    let ang = |p: Vec2| (p - c).y.atan2((p - c).x);
    let (aa, ab, ao) = (ang(a), ang(b), ang(on));
    // CCW from a to b passes through `on`?
    let ccw = geom::norm_angle(ao - aa) < geom::norm_angle(ab - aa);
    Some(if ccw { (Curve::Arc { c, r, start: aa, end: ab }, true) } else { (Curve::Arc { c, r, start: ab, end: aa }, false) })
}

/// FreeCAD's slot radius from the cursor: distance to the center segment, or to the nearer center.
pub fn slot_radius(c1: Vec2, c2: Vec2, p: Vec2) -> f64 {
    let len = c1.dist(c2);
    if len < 1e-12 {
        return 0.0;
    }
    let (l1, l2) = (p.dist(c1), p.dist(c2));
    if l1 * l1 + len * len > l2 * l2 && l2 * l2 + len * len > l1 * l1 {
        ((c2 - c1).cross(p - c1)).abs() / len
    } else {
        l1.min(l2)
    }
}

pub fn slot_geometry(c1: Vec2, c2: Vec2, r: f64) -> Vec<Geometry> {
    let ang = (c2 - c1).y.atan2((c2 - c1).x);
    let a1 = Curve::Arc { c: c1, r, start: PI / 2.0 + ang, end: 1.5 * PI + ang };
    let a2 = Curve::Arc { c: c2, r, start: 1.5 * PI + ang, end: PI / 2.0 + ang };
    let (p11, p12) = (a1.start_point().unwrap(), a1.end_point().unwrap());
    let (p21, p22) = (a2.start_point().unwrap(), a2.end_point().unwrap());
    vec![Geometry::new(a1), Geometry::new(a2), Geometry::line(p11, p22), Geometry::line(p12, p21)]
}

/// Name of the auto-constraint a snap produces (for the cursor hint).
pub fn snap_glyph(s: &Snap) -> Option<super::render::Glyph> {
    use super::render::Glyph;
    match s {
        Snap::Point(_) => Some(Glyph::Coincident),
        Snap::Curve(g) if *g == H_AXIS || *g == V_AXIS => Some(Glyph::PointOn),
        Snap::Curve(_) => Some(Glyph::PointOn),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn auto_hv_tolerance() {
        assert_eq!(hv_auto(Vec2::ZERO, Vec2::new(10.0, 0.3)), Some(true));
        assert_eq!(hv_auto(Vec2::ZERO, Vec2::new(-0.2, -10.0)), Some(false));
        assert_eq!(hv_auto(Vec2::ZERO, Vec2::new(10.0, 1.0)), None);
    }

    #[test]
    fn three_point_arc_orientation() {
        // through the top: CCW from (1,0) to (-1,0)
        let (a, first_is_start) = arc3(Vec2::new(1.0, 0.0), Vec2::new(-1.0, 0.0), Vec2::new(0.0, 1.0)).unwrap();
        assert!(first_is_start);
        assert!((a.sweep() - PI).abs() < 1e-9);
        // through the bottom: the arc runs from the second point
        let (_, first_is_start) = arc3(Vec2::new(1.0, 0.0), Vec2::new(-1.0, 0.0), Vec2::new(0.0, -1.0)).unwrap();
        assert!(!first_is_start);
    }

    #[test]
    fn slot_shape() {
        let g = slot_geometry(Vec2::ZERO, Vec2::new(10.0, 0.0), 2.0);
        assert_eq!(g.len(), 4);
        assert!((g[2].curve.length() - 10.0).abs() < 1e-9);
        assert!((slot_radius(Vec2::ZERO, Vec2::new(10.0, 0.0), Vec2::new(5.0, 3.0)) - 3.0).abs() < 1e-9);
        assert!(grid_spacing(10.0) >= 1.8 - 1e-12);
    }
}
