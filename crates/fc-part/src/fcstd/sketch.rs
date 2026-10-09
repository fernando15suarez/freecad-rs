//! Sketcher::SketchObject import: geometry, constraints, and re-expressing a sketch in a
//! different (rotated/translated) in-plane frame.

use super::xml::{attr_f64, attr_i64, elements, FcObj};
use super::{NoteKind, Notes};
use fc_sketch::{
    Constraint, ConstraintKind as K, Curve, GeoId, Geometry, PointPos, PointRef, Sketch, Vec2, H_AXIS, ROOT, V_AXIS,
};
use std::f64::consts::PI;

/// FreeCAD's "undefined" GeoId.
const GEO_UNDEF: i64 = -2000;

/// A sketch as read from the file, before frame adjustments.
pub(crate) struct ImportedSketch {
    pub sketch: Sketch,
    /// FreeCAD geometry index -> our GeoId.
    pub geo_map: Vec<Option<usize>>,
    /// FreeCAD constraint index -> our constraint indices (usually one; two for
    /// endpoint perpendicularity). Empty = dropped.
    pub con_map: Vec<Vec<usize>>,
    /// FreeCAD geometry indices of `Edge1, Edge2, ...` (non-construction curves).
    pub edges: Vec<usize>,
    /// FreeCAD geometry indices of `Axis0, Axis1, ...` (construction lines).
    pub axes: Vec<usize>,
}

impl ImportedSketch {
    pub fn our_con(&self, fc: usize) -> Option<usize> {
        self.con_map.get(fc).and_then(|v| v.first().copied())
    }
}

fn gp_ax2_xdir(n: [f64; 3]) -> [f64; 3] {
    // OCCT gp_Ax2(P, V) default X direction.
    let (a, b, c) = (n[0], n[1], n[2]);
    let (aa, ba, ca) = (a.abs(), b.abs(), c.abs());
    let v = if ba <= aa && ba <= ca {
        if aa > ca { [-c, 0.0, a] } else { [c, 0.0, -a] }
    } else if aa <= ba && aa <= ca {
        if ba > ca { [0.0, -c, b] } else { [0.0, c, -b] }
    } else if aa > ba {
        [-b, a, 0.0]
    } else {
        [b, -a, 0.0]
    };
    crate::placement::normalize(v)
}

fn rotate_about(v: [f64; 3], k: [f64; 3], ang: f64) -> [f64; 3] {
    use crate::placement::*;
    let (s, c) = ang.sin_cos();
    add(add(scale(v, c), scale(cross(k, v), s)), scale(k, dot(k, v) * (1.0 - c)))
}

/// Local frame (X, Y, N) of a FreeCAD conic from its stored normal and AngleXU.
fn conic_frame(n: roxmltree::Node) -> ([f64; 3], [f64; 3], [f64; 3]) {
    let g = |a: &str| attr_f64(n, a).unwrap_or(0.0);
    let mut nn = [g("NormalX"), g("NormalY"), g("NormalZ")];
    if crate::placement::norm(nn) < 1e-12 {
        nn = [0.0, 0.0, 1.0];
    }
    let nn = crate::placement::normalize(nn);
    let x = rotate_about(gp_ax2_xdir(nn), nn, g("AngleXU"));
    let y = crate::placement::cross(nn, x);
    (x, y, nn)
}

fn norm_angle(a: f64) -> f64 {
    let mut a = a % (2.0 * PI);
    if a <= -PI {
        a += 2.0 * PI;
    } else if a > PI {
        a -= 2.0 * PI;
    }
    a
}

/// One geometry: (curve, was clockwise) or an error if unsupported.
fn read_curve(g: roxmltree::Node) -> Result<(Curve, bool), String> {
    let ty = g.attribute("type").unwrap_or("");
    let find = |tag: &str| elements(g).find(|c| c.tag_name().name() == tag);
    let f = |n: roxmltree::Node, a: &str| attr_f64(n, a).unwrap_or(0.0);
    match ty {
        "Part::GeomLineSegment" => {
            let n = find("LineSegment").ok_or("LineSegment element missing")?;
            Ok((Curve::Line { a: Vec2::new(f(n, "StartX"), f(n, "StartY")), b: Vec2::new(f(n, "EndX"), f(n, "EndY")) }, false))
        }
        "Part::GeomPoint" => {
            let n = find("GeomPoint").ok_or("GeomPoint element missing")?;
            Ok((Curve::Point { p: Vec2::new(f(n, "X"), f(n, "Y")) }, false))
        }
        "Part::GeomCircle" => {
            let n = find("Circle").ok_or("Circle element missing")?;
            Ok((Curve::Circle { c: Vec2::new(f(n, "CenterX"), f(n, "CenterY")), r: f(n, "Radius") }, false))
        }
        "Part::GeomArcOfCircle" => {
            let n = find("ArcOfCircle").ok_or("ArcOfCircle element missing")?;
            let c = Vec2::new(f(n, "CenterX"), f(n, "CenterY"));
            let r = f(n, "Radius");
            let (x, y, nn) = conic_frame(n);
            let at = |t: f64| {
                let d = crate::placement::add(crate::placement::scale(x, t.cos()), crate::placement::scale(y, t.sin()));
                d[1].atan2(d[0])
            };
            let (t0, t1) = (f(n, "StartAngle"), f(n, "EndAngle"));
            let mut sweep = t1 - t0;
            while sweep <= 0.0 {
                sweep += 2.0 * PI;
            }
            if nn[2] >= 0.0 {
                let s = norm_angle(at(t0));
                Ok((Curve::Arc { c, r, start: s, end: s + sweep }, false))
            } else {
                // Clockwise in the sketch plane: the CCW arc runs from FreeCAD's end parameter
                // to its start parameter (FreeCAD's sketcher numbers the endpoints the same way).
                let s = norm_angle(at(t1));
                Ok((Curve::Arc { c, r, start: s, end: s + sweep }, true))
            }
        }
        other => Err(format!("unsupported geometry type {other}")),
    }
}

fn is_construction(g: roxmltree::Node) -> bool {
    // FreeCAD >= 0.20: SketchGeometryExtension geometryModeFlags, bit 1 = construction;
    // internal-alignment geometry is construction too. Older files: <Construction value>.
    let mut ext_flag = None;
    if let Some(exts) = elements(g).find(|c| c.tag_name().name() == "GeoExtensions") {
        for e in elements(exts) {
            if e.attribute("type") == Some("Sketcher::SketchGeometryExtension") {
                if let Some(flags) = e.attribute("geometryModeFlags") {
                    let bits: Vec<char> = flags.chars().rev().collect();
                    ext_flag = Some(bits.get(1) == Some(&'1'));
                }
                if attr_i64(e, "internalGeometryType").unwrap_or(0) != 0 {
                    return true;
                }
            }
        }
    }
    if let Some(c) = elements(g).find(|c| c.tag_name().name() == "Construction") {
        let v = c.attribute("value").unwrap_or("0");
        if v == "1" || v == "true" {
            return true;
        }
    }
    ext_flag.unwrap_or(false)
}

struct Elems {
    ids: [i64; 3],
    pos: [i64; 3],
}

fn read_elems(c: roxmltree::Node) -> Elems {
    let mut e = Elems { ids: [GEO_UNDEF; 3], pos: [0; 3] };
    if let (Some(ids), Some(ps)) = (c.attribute("ElementIds"), c.attribute("ElementPositions")) {
        let ids: Vec<i64> = ids.split_whitespace().filter_map(|s| s.parse().ok()).collect();
        let ps: Vec<i64> = ps.split_whitespace().filter_map(|s| s.parse().ok()).collect();
        let n = 3.min(ids.len()).min(ps.len());
        e.ids[..n].copy_from_slice(&ids[..n]);
        e.pos[..n].copy_from_slice(&ps[..n]);
        return e;
    }
    for (i, (a, b)) in [("First", "FirstPos"), ("Second", "SecondPos"), ("Third", "ThirdPos")].iter().enumerate() {
        e.ids[i] = attr_i64(c, a).unwrap_or(GEO_UNDEF);
        e.pos[i] = attr_i64(c, b).unwrap_or(0);
    }
    e
}

/// Import the sketch's Geometry and Constraints. Notes go to `notes` under `name`.
pub(crate) fn import_sketch(obj: &FcObj, notes: &mut Notes) -> ImportedSketch {
    let name = obj.name.clone();
    let mut sk = Sketch::new();
    let mut geo_map = Vec::new();
    let mut edges = Vec::new();
    let mut axes = Vec::new();
    let mut is_line = Vec::new();
    if let Some(list) = obj.prop("Geometry").and_then(|p| elements(p).find(|c| c.tag_name().name() == "GeometryList")) {
        for (i, g) in elements(list).filter(|n| n.tag_name().name() == "Geometry").enumerate() {
            let constr = is_construction(g);
            let ty = g.attribute("type").unwrap_or("");
            let is_point = ty == "Part::GeomPoint";
            if !constr && !is_point {
                edges.push(i);
            }
            if constr && ty == "Part::GeomLineSegment" {
                axes.push(i);
            }
            match read_curve(g) {
                Ok((curve, _clockwise)) => {
                    is_line.push(matches!(curve, Curve::Line { .. }));
                    geo_map.push(Some(sk.geometry.len()));
                    sk.geometry.push(Geometry { curve, construction: constr });
                }
                Err(e) => {
                    notes.push(&name, NoteKind::Skipped, format!("geometry {i}: {e}; constraints on it are dropped"));
                    geo_map.push(None);
                    is_line.push(false);
                }
            }
        }
    }
    if obj.link_sub_list("ExternalGeometry").len() + obj.link_sub_list("ExternalGeo").len() > 0 {
        notes.push(&name, NoteKind::Skipped, "external geometry is not supported; constraints on it are dropped".into());
    }

    let mut con_map = Vec::new();
    if let Some(list) = obj.prop("Constraints").and_then(|p| elements(p).find(|c| c.tag_name().name() == "ConstraintList")) {
        for (ci, c) in elements(list).filter(|n| n.tag_name().name() == "Constrain").enumerate() {
            let ty = attr_i64(c, "Type").unwrap_or(0);
            let value = attr_f64(c, "Value").unwrap_or(0.0);
            let driving = c.attribute("IsDriving").map(|v| v != "0" && v != "false").unwrap_or(true);
            let active = c.attribute("IsActive").map(|v| v != "0" && v != "false").unwrap_or(true);
            let cname = c.attribute("Name").unwrap_or("").to_string();
            let el = read_elems(c);
            let mut tr = Translate { geo_map: &geo_map, ok: true, why: String::new() };
            if !active {
                notes.push(&name, NoteKind::Skipped, format!("constraint {ci} is deactivated (dropped)"));
                con_map.push(vec![]);
                continue;
            }
            let kinds = tr.kinds(ty, &el, value, &is_line);
            match kinds {
                Some(ks) if tr.ok => {
                    let mut idx = Vec::new();
                    for (j, k) in ks.into_iter().enumerate() {
                        let mut con = if driving || !k.is_dimensional() { Constraint::new(k) } else { Constraint::reference(k) };
                        if j == 0 && !cname.is_empty() {
                            con.name = Some(cname.clone());
                        }
                        idx.push(sk.constraints.len());
                        sk.constraints.push(con);
                    }
                    if idx.len() > 1 {
                        notes.push(&name, NoteKind::Approximated, format!("constraint {ci} split into {} constraints {:?}", idx.len(), idx));
                    }
                    con_map.push(idx);
                }
                _ => {
                    let why = if tr.why.is_empty() { format!("constraint type {ty} is not supported") } else { tr.why };
                    notes.push(&name, NoteKind::Skipped, format!("constraint {ci}: {why}"));
                    con_map.push(vec![]);
                }
            }
        }
    }
    ImportedSketch { sketch: sk, geo_map, con_map, edges, axes }
}

struct Translate<'a> {
    geo_map: &'a [Option<usize>],
    ok: bool,
    why: String,
}

impl Translate<'_> {
    fn fail(&mut self, why: impl Into<String>) {
        if self.ok {
            self.why = why.into();
        }
        self.ok = false;
    }
    fn geo(&mut self, g: i64) -> GeoId {
        match g {
            -1 => H_AXIS,
            -2 => V_AXIS,
            g if g >= 0 => match self.geo_map.get(g as usize).copied().flatten() {
                Some(o) => o,
                None => {
                    self.fail(format!("references unsupported geometry {g}"));
                    0
                }
            },
            GEO_UNDEF => {
                self.fail("missing geometry reference");
                0
            }
            _ => {
                self.fail("references external geometry");
                0
            }
        }
    }
    /// FreeCAD's PointPos. Positions of a clockwise arc already refer to the CCW-emulated
    /// endpoints (the sketcher uses `getStartPoint(emulateCCWXY)`), so no swap is needed.
    fn pos(&self, _g: i64, p: i64) -> PointPos {
        match p {
            1 => PointPos::Start,
            2 => PointPos::End,
            3 => PointPos::Mid,
            _ => PointPos::Edge,
        }
    }
    fn pt(&mut self, g: i64, p: i64) -> PointRef {
        if (g == -1 || g == -2) && p == 1 {
            return PointRef::root();
        }
        let geo = self.geo(g);
        PointRef::new(geo, self.pos(g, p))
    }
    fn kinds(&mut self, ty: i64, e: &Elems, v: f64, is_line: &[bool]) -> Option<Vec<K>> {
        let [f, s, t] = e.ids;
        let [fp, sp, tp] = e.pos;
        let has_s = s != GEO_UNDEF;
        let has_t = t != GEO_UNDEF;
        let line = |g: i64| g >= 0 && is_line.get(g as usize).copied().unwrap_or(false);
        Some(match ty {
            1 => vec![K::Coincident(self.pt(f, fp), self.pt(s, sp))],
            2 | 3 => {
                let horiz = ty == 2;
                if has_s {
                    let (a, b) = (self.pt(f, fp), self.pt(s, sp));
                    vec![if horiz { K::HorizontalPoints(a, b) } else { K::VerticalPoints(a, b) }]
                } else {
                    let g = self.geo(f);
                    vec![if horiz { K::Horizontal(g) } else { K::Vertical(g) }]
                }
            }
            4 => vec![K::Parallel(self.geo(f), self.geo(s))],
            5 => {
                if has_t {
                    self.fail("tangency via a point is not supported");
                    return None;
                }
                match (fp != 0, sp != 0) {
                    (false, false) => vec![K::Tangent(self.geo(f), self.geo(s))],
                    (true, true) => vec![K::TangentAt(self.pt(f, fp), self.pt(s, sp))],
                    (true, false) => vec![K::TangentAt(self.pt(f, fp), PointRef::edge(self.geo(s)))],
                    (false, true) => vec![K::TangentAt(self.pt(s, sp), PointRef::edge(self.geo(f)))],
                }
            }
            6 => {
                if !has_s {
                    if fp == 0 && line(f) {
                        vec![K::DistanceLine(self.geo(f), v)]
                    } else {
                        self.fail("Distance on a single non-line element (arc length) is not supported");
                        return None;
                    }
                } else {
                    match (fp != 0, sp != 0) {
                        (true, true) => vec![K::Distance(self.pt(f, fp), self.pt(s, sp), v)],
                        (true, false) => vec![K::PointLineDistance(self.pt(f, fp), self.geo(s), v)],
                        (false, true) => vec![K::PointLineDistance(self.pt(s, sp), self.geo(f), v)],
                        (false, false) => {
                            self.fail("edge-to-edge Distance is not supported");
                            return None;
                        }
                    }
                }
            }
            7 | 8 => {
                let x = ty == 7;
                if !has_s {
                    if fp == 0 {
                        let g = self.geo(f);
                        let (a, b) = (PointRef::start(g), PointRef::end(g));
                        // A swapped arc has no meaning here; FreeCAD only allows lines.
                        vec![if x { K::DistanceX(a, b, v) } else { K::DistanceY(a, b, v) }]
                    } else {
                        let p = self.pt(f, fp);
                        vec![if x { K::LockX(p, v) } else { K::LockY(p, v) }]
                    }
                } else {
                    let (a, b) = (self.pt(f, fp), self.pt(s, sp));
                    vec![if x { K::DistanceX(a, b, v) } else { K::DistanceY(a, b, v) }]
                }
            }
            9 => {
                if has_t {
                    self.fail("angle via a point is not supported");
                    return None;
                }
                if !has_s {
                    vec![K::Angle(self.geo(f), v)]
                } else if sp != 0 || fp != 0 {
                    // Directions run away from the chosen endpoints.
                    let flips = (fp == 2) as u8 + (sp == 2) as u8;
                    let v2 = if flips == 1 { v + PI } else { v };
                    vec![K::AngleBetween(self.geo(f), self.geo(s), v2)]
                } else {
                    vec![K::AngleBetween(self.geo(f), self.geo(s), v)]
                }
            }
            10 => {
                if has_t {
                    self.fail("perpendicularity via a point is not supported");
                    return None;
                }
                let (gf, gs) = (self.geo(f), self.geo(s));
                match (fp != 0, sp != 0) {
                    (false, false) => vec![K::Perpendicular(gf, gs)],
                    (true, true) => vec![K::Coincident(self.pt(f, fp), self.pt(s, sp)), K::Perpendicular(gf, gs)],
                    (true, false) => vec![K::PointOnObject(self.pt(f, fp), gs), K::Perpendicular(gf, gs)],
                    (false, true) => vec![K::PointOnObject(self.pt(s, sp), gf), K::Perpendicular(gf, gs)],
                }
            }
            11 => vec![K::Radius(self.geo(f), v)],
            12 => vec![K::Equal(self.geo(f), self.geo(s))],
            13 => vec![K::PointOnObject(self.pt(f, fp), self.geo(s))],
            14 => {
                let (a, b) = (self.pt(f, fp), self.pt(s, sp));
                if tp == 0 {
                    vec![K::SymmetricLine(a, b, self.geo(t))]
                } else {
                    vec![K::Symmetric(a, b, self.pt(t, tp))]
                }
            }
            17 => vec![K::Block(self.geo(f))],
            18 => vec![K::Diameter(self.geo(f), v)],
            15 => {
                self.fail("internal alignment constraint (ellipse/B-spline geometry) is not supported");
                return None;
            }
            16 => {
                self.fail("Snell's law constraint is not supported");
                return None;
            }
            0 => {
                self.fail("constraint of type None");
                return None;
            }
            other => {
                self.fail(format!("constraint type {other} is not supported"));
                return None;
            }
        })
    }
}

/// How a FreeCAD constraint's value maps to ours after a frame change:
/// `ours = scale * freecad + add` (in expression units: mm, or degrees for angles).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ValueMap {
    pub scale: f64,
    pub add: f64,
}

impl ValueMap {
    pub const ID: ValueMap = ValueMap { scale: 1.0, add: 0.0 };
    pub fn is_identity(&self) -> bool {
        self.scale == 1.0 && self.add.abs() < 1e-12
    }
}

/// Re-express a sketch drawn in FreeCAD's frame in our frame, where FreeCAD's local point
/// p maps to `R(theta) p + t`. Returns per-constraint value maps (our indices).
pub(crate) fn transform_sketch(sk: &mut Sketch, theta: f64, t: Vec2, sketch_name: &str, notes: &mut Notes) -> Vec<ValueMap> {
    let (s, c) = theta.sin_cos();
    let rot = |p: Vec2| Vec2::new(c * p.x - s * p.y, s * p.x + c * p.y);
    let map = |p: Vec2| rot(p) + t;
    for g in &mut sk.geometry {
        g.curve = match g.curve.clone() {
            Curve::Point { p } => Curve::Point { p: map(p) },
            Curve::Line { a, b } => Curve::Line { a: map(a), b: map(b) },
            Curve::Circle { c, r } => Curve::Circle { c: map(c), r },
            Curve::Arc { c, r, start, end } => Curve::Arc { c: map(c), r, start: start + theta, end: end + theta },
        };
    }
    let q = theta / (PI / 2.0);
    let quarter = ((q.round() - q).abs() < 1e-9).then(|| (q.round() as i64).rem_euclid(4));
    let shifted = t.length() > 1e-9;
    let mut out = vec![ValueMap::ID; sk.constraints.len()];
    let mut approx = Vec::new();
    // FreeCAD axis -> (our axis, reversed) for a quarter turn k.
    let axis = |g: GeoId, k: i64| -> (GeoId, bool) {
        match (g == H_AXIS, k) {
            (true, 0) => (H_AXIS, false),
            (true, 1) => (V_AXIS, false),
            (true, 2) => (H_AXIS, true),
            (true, _) => (V_AXIS, true),
            (false, 0) => (V_AXIS, false),
            (false, 1) => (H_AXIS, true),
            (false, 2) => (V_AXIS, true),
            (false, _) => (H_AXIS, false),
        }
    };
    let is_axis = |g: GeoId| g == H_AXIS || g == V_AXIS;
    let is_fixed_origin = |p: &PointRef| p.geo == ROOT || (is_axis(p.geo) && p.pos != PointPos::End && p.pos != PointPos::Edge);
    for (ci, con) in sk.constraints.iter_mut().enumerate() {
        let geos = con.kind.geometries();
        let uses_axis = geos.iter().any(|&g| is_axis(g));
        let uses_root = geos.contains(&ROOT);
        let Some(k) = quarter else {
            match &mut con.kind {
                K::Angle(g, a) if !matches!(sk.geometry.get(*g).map(|g| &g.curve), Some(Curve::Arc { .. })) => {
                    *a += theta;
                    out[ci].add = theta.to_degrees();
                }
                K::Horizontal(_) | K::Vertical(_) | K::HorizontalPoints(..) | K::VerticalPoints(..) | K::DistanceX(..)
                | K::DistanceY(..) | K::LockX(..) | K::LockY(..) => approx.push(ci),
                _ if uses_axis || (uses_root && shifted) => approx.push(ci),
                _ => {}
            }
            continue;
        };
        let odd = k % 2 == 1;
        // Components: DistanceX (dx) / DistanceY (dy) after rotation.
        // Returns (is_x_after, sign).
        let comp = |was_x: bool| -> (bool, f64) {
            match (was_x, k) {
                (true, 0) => (true, 1.0),
                (true, 1) => (false, 1.0),
                (true, 2) => (true, -1.0),
                (true, _) => (false, -1.0),
                (false, 0) => (false, 1.0),
                (false, 1) => (true, -1.0),
                (false, 2) => (false, -1.0),
                (false, _) => (true, 1.0),
            }
        };
        let kind = con.kind.clone();
        con.kind = match kind {
            K::Horizontal(g) if odd => K::Vertical(g),
            K::Vertical(g) if odd => K::Horizontal(g),
            K::HorizontalPoints(a, b) if odd => K::VerticalPoints(a, b),
            K::VerticalPoints(a, b) if odd => K::HorizontalPoints(a, b),
            K::DistanceX(a, b, d) | K::DistanceY(a, b, d) => {
                let was_x = matches!(kind, K::DistanceX(..));
                let (x, sign) = comp(was_x);
                let tc = if x { t.x } else { t.y };
                let mut add = 0.0;
                if is_fixed_origin(&a) {
                    add += tc;
                }
                if is_fixed_origin(&b) {
                    add -= tc;
                }
                if (is_axis(a.geo) && a.pos == PointPos::End) || (is_axis(b.geo) && b.pos == PointPos::End) {
                    approx.push(ci);
                }
                out[ci] = ValueMap { scale: sign, add };
                let nd = sign * d + add;
                if x { K::DistanceX(a, b, nd) } else { K::DistanceY(a, b, nd) }
            }
            K::LockX(p, d) | K::LockY(p, d) => {
                let (x, sign) = comp(matches!(kind, K::LockX(..)));
                let add = if x { t.x } else { t.y };
                out[ci] = ValueMap { scale: sign, add };
                if x { K::LockX(p, sign * d + add) } else { K::LockY(p, sign * d + add) }
            }
            K::Angle(g, a) if !matches!(sk.geometry.get(g).map(|g| &g.curve), Some(Curve::Arc { .. })) => {
                out[ci].add = theta.to_degrees();
                K::Angle(g, a + theta)
            }
            other => other,
        };
        // Axis references.
        if uses_axis {
            if shifted {
                approx.push(ci);
            }
            let mut flips = 0;
            let mut fix = |g: &mut GeoId| {
                if is_axis(*g) {
                    let (ng, rev) = axis(*g, k);
                    *g = ng;
                    flips += rev as i32;
                }
            };
            match &mut con.kind {
                K::Parallel(a, b) | K::Perpendicular(a, b) | K::Tangent(a, b) | K::Equal(a, b) => {
                    fix(a);
                    fix(b)
                }
                K::AngleBetween(a, b, v) => {
                    fix(a);
                    fix(b);
                    if flips % 2 == 1 {
                        *v += PI;
                    }
                }
                K::PointOnObject(p, g) | K::PointLineDistance(p, g, _) => {
                    if is_axis(p.geo) {
                        approx.push(ci);
                    }
                    fix(g)
                }
                K::SymmetricLine(_, _, g) | K::Horizontal(g) | K::Vertical(g) | K::Angle(g, _) | K::DistanceLine(g, _) => fix(g),
                _ => {
                    // Point references on axes (start = origin is ROOT already).
                    if geos.iter().any(|&g| is_axis(g)) {
                        approx.push(ci);
                    }
                }
            }
        } else if uses_root && shifted && !matches!(con.kind, K::DistanceX(..) | K::DistanceY(..)) {
            approx.push(ci);
        }
    }
    approx.sort_unstable();
    approx.dedup();
    if !approx.is_empty() {
        notes.push(
            sketch_name,
            NoteKind::Approximated,
            format!(
                "sketch frame differs from FreeCAD's by {:.3} deg / ({:.4}, {:.4}) mm; constraints {:?} depend on the frame and could not be re-expressed exactly",
                theta.to_degrees(),
                t.x,
                t.y,
                approx
            ),
        );
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A fully constrained sketch using frame-dependent constraints.
    fn frame_sketch() -> Sketch {
        let mut s = Sketch::new();
        let l0 = s.add_geometry(Geometry::line(Vec2::new(1.0, 2.0), Vec2::new(6.0, 2.0)));
        let l1 = s.add_geometry(Geometry::line(Vec2::new(6.0, 2.0), Vec2::new(6.0, 5.0)));
        let l2 = s.add_geometry(Geometry::line(Vec2::new(-2.0, 0.0), Vec2::new(-2.0 + 3f64.sqrt(), 1.0)));
        let p = s.add_geometry(Geometry::point(Vec2::new(4.0, 0.0)));
        s.add_constraint(K::Horizontal(l0));
        s.add_constraint(K::Vertical(l1));
        s.add_constraint(K::Coincident(PointRef::end(l0), PointRef::start(l1)));
        s.add_constraint(K::DistanceX(PointRef::root(), PointRef::start(l0), 1.0));
        s.add_constraint(K::DistanceY(PointRef::root(), PointRef::start(l0), 2.0));
        s.add_constraint(K::DistanceX(PointRef::start(l0), PointRef::end(l0), 5.0));
        s.add_constraint(K::DistanceY(PointRef::start(l1), PointRef::end(l1), 3.0));
        s.add_constraint(K::LockX(PointRef::start(l2), -2.0));
        s.add_constraint(K::PointOnObject(PointRef::start(l2), H_AXIS));
        s.add_constraint(K::Angle(l2, PI / 6.0));
        s.add_constraint(K::DistanceLine(l2, 2.0));
        s.add_constraint(K::PointOnObject(PointRef::start(p), H_AXIS));
        s.add_constraint(K::AngleBetween(H_AXIS, l2, PI / 6.0));
        s.remove_constraint(12); // keep it fully constrained without redundancy
        s.add_constraint(K::LockX(PointRef::start(p), 4.0));
        s
    }

    #[test]
    fn quarter_turns_keep_the_sketch_solved() {
        let base = frame_sketch();
        let r = base.diagnose();
        assert_eq!((r.dof, r.redundant.len(), r.conflicting.len()), (0, 0, 0), "{r:?}");
        for k in 0..4 {
            for t in [Vec2::ZERO, Vec2::new(2.0, -3.0)] {
                let mut s = base.clone();
                let mut notes = Notes::default();
                transform_sketch(&mut s, k as f64 * PI / 2.0, t, "S", &mut notes);
                let before = s.clone();
                let rep = s.solve();
                assert!(rep.is_ok(), "k={k} t={t:?}: {rep:?}");
                assert_eq!(rep.dof, 0);
                for (a, b) in s.geometry.iter().zip(&before.geometry) {
                    for pos in [PointPos::Start, PointPos::End] {
                        if let (Some(p), Some(q)) = (a.curve.point(pos), b.curve.point(pos)) {
                            // Constraints on the H axis cannot follow a translation exactly.
                            if t.length() == 0.0 {
                                assert!(p.dist(q) < 1e-9, "k={k}: {:?} moved to {:?}", q, p);
                            }
                        }
                    }
                }
                if t.length() > 0.0 {
                    assert!(notes.0.iter().any(|n| n.kind == NoteKind::Approximated));
                }
            }
        }
    }

    #[test]
    fn quarter_turn_maps_constraint_kinds() {
        let mut s = frame_sketch();
        let mut notes = Notes::default();
        let maps = transform_sketch(&mut s, PI / 2.0, Vec2::new(0.5, 0.0), "S", &mut notes);
        assert!(matches!(s.constraints[0].kind, K::Vertical(0)));
        assert!(matches!(s.constraints[1].kind, K::Horizontal(1)));
        // DistanceX(root, p, 1) -> DistanceY(root, p, 1) (x -> y), plus the shift's y part (0).
        assert!(matches!(s.constraints[3].kind, K::DistanceY(_, _, v) if (v - 1.0).abs() < 1e-12));
        // DistanceY(root, p, 2) -> DistanceX(root, p, -2 + 0.5).
        assert!(matches!(s.constraints[4].kind, K::DistanceX(_, _, v) if (v + 1.5).abs() < 1e-12));
        assert_eq!(maps[4], ValueMap { scale: -1.0, add: 0.5 });
        // H axis -> V axis.
        assert!(matches!(s.constraints[8].kind, K::PointOnObject(_, g) if g == V_AXIS));
    }
}
