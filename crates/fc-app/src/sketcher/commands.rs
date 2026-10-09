//! Constraint commands acting on the selection (FreeCAD CommandConstraints.cpp), delete,
//! construction and driving/reference toggles.

use super::{geom, DimEdit, PendingDelete, Sel, SketchEditor};
use eframe::egui::Pos2;
use fc_part::params::OnUsers;
use fc_part::Document;
use fc_sketch::{ConstraintKind as K, Curve, GeoId, PointRef, Sketch, Vec2, ROOT};
use std::f64::consts::PI;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cmd {
    Coincident,
    PointOnObject,
    Horizontal,
    Vertical,
    Parallel,
    Perpendicular,
    Tangent,
    Equal,
    Symmetric,
    Block,
    Lock,
    DistanceX,
    DistanceY,
    Distance,
    Radius,
    Diameter,
    Angle,
}

impl Cmd {
    /// (command, toolbar label, shortcut, tooltip)
    pub const ALL: [(Cmd, &'static str, &'static str, &'static str); 17] = [
        (Cmd::Coincident, "Coincident", "C", "Two or more vertices; a vertex and a curve; two circles/arcs (concentric)"),
        (Cmd::PointOnObject, "Point on object", "O", "Vertices and a curve (or axis)"),
        (Cmd::Horizontal, "Horizontal", "H", "Lines, or two vertices"),
        (Cmd::Vertical, "Vertical", "V", "Lines, or two vertices"),
        (Cmd::Parallel, "Parallel", "P", "Two or more lines"),
        (Cmd::Perpendicular, "Perpendicular", "N", "Two curves"),
        (Cmd::Tangent, "Tangent", "T", "Two curves; two endpoints; an endpoint and a curve"),
        (Cmd::Equal, "Equal", "E", "Two or more lines, or circles/arcs"),
        (Cmd::Symmetric, "Symmetric", "S", "Two vertices and a line or a vertex (selected last)"),
        (Cmd::Block, "Block", "K, B", "Fix the selected curves where they are"),
        (Cmd::Lock, "Lock", "K, L", "Fix the selected vertices (horizontal + vertical distance from the origin)"),
        (Cmd::DistanceX, "H. dist", "L", "Two vertices, a line, or one vertex (from the origin)"),
        (Cmd::DistanceY, "V. dist", "I", "Two vertices, a line, or one vertex (from the origin)"),
        (Cmd::Distance, "Distance", "D", "Two vertices, a line, or a vertex and a curve"),
        (Cmd::Radius, "Radius", "R", "Circles/arcs"),
        (Cmd::Diameter, "Diameter", "K, O", "Circles/arcs"),
        (Cmd::Angle, "Angle", "A", "Two lines, one line (to the horizontal), or an arc (sweep)"),
    ];

    pub fn parse(s: &str) -> Option<Cmd> {
        Some(match s.to_ascii_lowercase().replace(['_', '-'], "").as_str() {
            "coincident" => Cmd::Coincident,
            "pointonobject" | "pointon" => Cmd::PointOnObject,
            "horizontal" => Cmd::Horizontal,
            "vertical" => Cmd::Vertical,
            "parallel" => Cmd::Parallel,
            "perpendicular" => Cmd::Perpendicular,
            "tangent" => Cmd::Tangent,
            "equal" => Cmd::Equal,
            "symmetric" => Cmd::Symmetric,
            "block" => Cmd::Block,
            "lock" => Cmd::Lock,
            "distancex" | "hdistance" | "horizontaldistance" => Cmd::DistanceX,
            "distancey" | "vdistance" | "verticaldistance" => Cmd::DistanceY,
            "distance" => Cmd::Distance,
            "radius" => Cmd::Radius,
            "diameter" => Cmd::Diameter,
            "angle" => Cmd::Angle,
            _ => return None,
        })
    }

    pub fn dimensional(&self) -> bool {
        matches!(self, Cmd::DistanceX | Cmd::DistanceY | Cmd::Distance | Cmd::Radius | Cmd::Diameter | Cmd::Angle)
    }
}

/// Selection split by kind, in selection order.
pub struct Parts {
    pub pts: Vec<PointRef>,
    pub edges: Vec<GeoId>,
    pub cons: Vec<usize>,
}

impl SketchEditor {
    pub(crate) fn parts(&self) -> Parts {
        let mut p = Parts { pts: vec![], edges: vec![], cons: vec![] };
        for s in &self.sel {
            match s {
                Sel::Point(x) => p.pts.push(*x),
                Sel::Edge(g) => p.edges.push(*g),
                Sel::Constraint(c) => p.cons.push(*c),
            }
        }
        p
    }

    /// Run a constraint command from the UI: on error, report it like FreeCAD's
    /// "Wrong selection" message.
    pub(crate) fn run_cmd_ui(&mut self, doc: &mut Document, cmd: Cmd) -> bool {
        match self.run_cmd(doc, cmd) {
            Ok(()) => true,
            Err(e) => {
                self.notify(format!("Wrong selection: {e}"), true);
                false
            }
        }
    }

    pub(crate) fn run_cmd(&mut self, doc: &mut Document, cmd: Cmd) -> Result<(), String> {
        let sk = self.sk(doc);
        let new = build(sk, cmd, &self.parts())?;
        if new.is_empty() {
            return Err("nothing to constrain".into());
        }
        self.push_undo(doc);
        let was_invalid = self.invalid();
        let s = doc.sketch_mut(self.feature).ok_or("no sketch")?;
        let first = s.constraints.len();
        for k in new {
            s.add_constraint(k);
        }
        let last = s.constraints.len() - 1;
        self.sel.clear();
        self.resolve(doc);
        if cmd.dimensional() {
            // FreeCAD: a dimension that would over-constrain the sketch becomes a reference.
            // The diagnosis may blame an older constraint, so test the new ones (last first).
            let mut made_ref = 0;
            for c in (first..=last).rev() {
                if was_invalid || !self.invalid() {
                    break;
                }
                if let Some(s) = doc.sketch_mut(self.feature) {
                    s.constraints[c].driving = false;
                }
                self.resolve(doc);
                made_ref += 1;
            }
            if made_ref > 0 {
                self.notify("The dimension would over-constrain the sketch; it was added as a reference (non-driving) dimension.", false);
            }
            // the popup edits the last driving one (or the last), the others follow it
            let sk = self.sk(doc);
            let main = (first..=last).rev().find(|c| sk.constraints[*c].driving).unwrap_or(last);
            let also: Vec<usize> = (first..=last).filter(|c| *c != main && sk.constraints[*c].driving).collect();
            self.sel = vec![Sel::Constraint(main)];
            self.open_dim_edit(doc, main, true, None);
            if let Some(de) = self.dim_edit.as_mut() {
                de.also = also;
            }
        } else {
            self.notify(format!("Added {} constraint{}", geom::kind_name(&self.sk(doc).constraints[last].kind), if last > first { "s" } else { "" }), false);
        }
        Ok(())
    }

    /// Open the datum popup for constraint `ci`.
    pub(crate) fn open_dim_edit(&mut self, doc: &Document, ci: usize, created: bool, at: Option<Pos2>) {
        let sk = self.sk(doc);
        let Some(c) = sk.constraints.get(ci) else { return };
        let key = self.ckey(doc, ci);
        let value = match doc.expressions.get(&key) {
            Some(f) => format!("={f}"),
            None => match geom::display_value(&c.kind) {
                Some(v) => format!("{} {}", geom::fmt_num(v), geom::unit_of(&c.kind)),
                None => return,
            },
        };
        let mut pos = at.or(self.cursor_px).unwrap_or(self.rect.center()) + eframe::egui::vec2(16.0, 16.0);
        pos.x = pos.x.min(self.rect.right() - 290.0).max(self.rect.left() + 4.0);
        pos.y = pos.y.min(self.rect.bottom() - 220.0).max(self.rect.top() + 4.0);
        self.dim_edit = Some(DimEdit { ci, also: vec![], value, name: c.name.clone().unwrap_or_default(), reference: !c.driving, created, pos, error: None, focus: true });
    }

    /// Apply a value text (number with units, or `=formula`) and a name to a dimension.
    pub(crate) fn apply_dimension(&mut self, doc: &mut Document, ci: usize, text: &str, name: Option<&str>) -> Result<(), String> {
        let sk = self.sk(doc);
        let Some(c) = sk.constraints.get(ci).cloned() else { return Err("no such constraint".into()) };
        if !c.kind.is_dimensional() {
            return Err("not a dimension".into());
        }
        if let Some(n) = name {
            let n = n.trim();
            let old = c.name.clone().unwrap_or_default();
            if old != n {
                if n.is_empty() {
                    doc.unname_constraint(self.feature, ci, OnUsers::Refuse).map_err(|u| format!("the name is used by: {}", u.join(", ")))?;
                } else {
                    // formulas using the old name follow the rename (like FreeCAD's ObjectIdentifier renames)
                    let users = if old.is_empty() { vec![] } else { doc.users_of(&self.ckey(doc, ci)) };
                    doc.name_constraint(self.feature, ci, n).map_err(|e| e.to_string())?;
                    for u in users {
                        if let Some(p) = u.strip_prefix("param:") {
                            if let Some(v) = doc.parameters.get(p).cloned() {
                                doc.parameters.insert(p.to_string(), rename_ident(&v, &old, n));
                            }
                        } else if let Some(v) = doc.expressions.get(&u).cloned() {
                            doc.expressions.insert(u, rename_ident(&v, &old, n));
                        }
                    }
                }
            }
        }
        let key = self.ckey(doc, ci);
        let t = text.trim();
        if let Some(f) = t.strip_prefix('=') {
            if !c.driving {
                return Err("a reference dimension cannot be driven by a formula".into());
            }
            doc.set_expression(&key, f.trim()).map_err(|e| e.to_string())?;
        } else if !t.is_empty() {
            let e = fc_part::expr::parse(t).map_err(|e| e.to_string())?;
            let mut v = e
                .eval(&mut |r: &str| {
                    let k = doc.canonical_name(r).ok_or_else(|| fc_part::expr::ExprError(format!("unknown name '{r}'")))?;
                    doc.read_value(&k).ok_or_else(|| fc_part::expr::ExprError(format!("'{r}' has no value")))
                })
                .map_err(|e| e.to_string())?;
            if !v.is_finite() {
                return Err("invalid value".into());
            }
            match c.kind {
                K::DistanceX(..) | K::DistanceY(..) | K::LockX(..) | K::LockY(..) => {
                    if c.kind.value().unwrap_or(0.0) < 0.0 {
                        v = -v.abs();
                    }
                }
                K::Distance(..) | K::DistanceLine(..) | K::Radius(..) | K::Diameter(..) if v <= 0.0 => {
                    return Err("the value must be positive".into());
                }
                _ => {}
            }
            if doc.expressions.contains_key(&key) {
                let _ = doc.set_expression(&key, "");
            }
            doc.write_value(&key, v);
        }
        self.resolve(doc);
        Ok(())
    }

    /// Flip driving/reference on the selected dimensions (FreeCAD "Toggle driving/reference").
    pub(crate) fn toggle_driving(&mut self, doc: &mut Document) {
        let cons: Vec<usize> = self.parts().cons;
        let dims: Vec<usize> = cons.into_iter().filter(|c| self.sk(doc).constraints.get(*c).is_some_and(|c| c.kind.is_dimensional())).collect();
        if dims.is_empty() {
            self.notify("Wrong selection: select dimensional constraints to toggle driving/reference", true);
            return;
        }
        self.push_undo(doc);
        for c in dims {
            let key = self.ckey(doc, c);
            let Some(s) = doc.sketch_mut(self.feature) else { return };
            let now_driving = !s.constraints[c].driving;
            s.constraints[c].driving = now_driving;
            if !now_driving && doc.expressions.contains_key(&key) {
                let _ = doc.set_expression(&key, "");
                self.notify(format!("Constraint{} is now a reference; its formula was removed", c + 1), false);
            }
        }
        self.resolve(doc);
    }

    /// Toggle construction on the selected curves, or the creation mode when none are selected.
    pub(crate) fn toggle_construction(&mut self, doc: &mut Document) {
        let edges: Vec<GeoId> = self.parts().edges.into_iter().filter(|g| !fc_sketch::is_external(*g)).collect();
        if edges.is_empty() {
            self.construction = !self.construction;
            self.notify(if self.construction { "Construction mode: new geometry is construction geometry" } else { "Normal geometry mode" }, false);
            return;
        }
        self.push_undo(doc);
        if let Some(s) = doc.sketch_mut(self.feature) {
            for g in edges {
                s.geometry[g].construction = !s.geometry[g].construction;
            }
        }
        self.resolve(doc);
    }

    /// Delete the selection (FreeCAD onDelete): constraints, curves, and for vertices the
    /// constraints on that vertex (or the point geometry itself).
    pub(crate) fn delete_selection(&mut self, doc: &mut Document, policy: OnUsers) {
        let p = self.parts();
        let sk = self.sk(doc);
        let mut geos: Vec<GeoId> = p.edges.iter().copied().filter(|g| !fc_sketch::is_external(*g)).collect();
        let mut cons = p.cons.clone();
        for pr in &p.pts {
            if pr.geo == ROOT {
                continue;
            }
            if matches!(sk.geometry.get(pr.geo).map(|g| &g.curve), Some(Curve::Point { .. })) {
                geos.push(pr.geo);
            } else {
                let pc = geom::canon(sk, *pr);
                for (ci, c) in sk.constraints.iter().enumerate() {
                    if geom::point_refs(&c.kind).iter().any(|q| geom::canon(sk, *q) == pc) {
                        cons.push(ci);
                    }
                }
            }
        }
        if geos.is_empty() && cons.is_empty() {
            return;
        }
        self.delete_items(doc, cons, geos, policy);
    }

    pub(crate) fn delete_items(&mut self, doc: &mut Document, mut cons: Vec<usize>, mut geos: Vec<GeoId>, policy: OnUsers) {
        let sk = self.sk(doc);
        geos.sort_unstable();
        geos.dedup();
        for (ci, c) in sk.constraints.iter().enumerate() {
            if c.kind.geometries().iter().any(|g| geos.contains(g)) {
                cons.push(ci);
            }
        }
        cons.sort_unstable();
        cons.dedup();
        let snap = (self.sk(doc).clone(), doc.expressions.clone(), doc.parameters.clone());
        match doc.delete_constraints(self.feature, &cons, policy) {
            Err(users) => {
                // `Sketch.Constraints[i]` -> FreeCAD-style `Constraint<i+1>` for this sketch
                let prefix = format!("{}.Constraints[", self.sketch_name(doc));
                let users = users
                    .into_iter()
                    .map(|u| match u.strip_prefix(&prefix).and_then(|t| t.strip_suffix(']')).and_then(|t| t.parse::<usize>().ok()) {
                        Some(i) => format!("Constraint{} ({})", i + 1, self.sk(doc).constraints.get(i).map(|c| geom::kind_name(&c.kind)).unwrap_or("?")),
                        None => u,
                    })
                    .collect();
                self.pending_delete = Some(PendingDelete { constraints: cons, geometry: geos, users });
                return;
            }
            Ok(()) => {}
        }
        // undo entry for the state before the delete
        self.undo.push(super::Snapshot { sketch: snap.0, expressions: snap.1, parameters: snap.2 });
        self.redo.clear();
        for g in geos.iter().rev() {
            let _ = doc.delete_geometry(self.feature, *g, OnUsers::Freeze);
        }
        self.pending_delete = None;
        self.sel.clear();
        self.hover = None;
        self.label_off.clear();
        self.resolve(doc);
        self.notify(format!("Deleted {} constraint(s) and {} element(s)", cons.len(), geos.len()), false);
    }
}

/// Replace whole-word identifier `old` (also after a `.`, as in `Sketch.old`) by `new`.
fn rename_ident(src: &str, old: &str, new: &str) -> String {
    let word = |c: char| c.is_ascii_alphanumeric() || c == '_';
    let mut out = String::new();
    let mut i = 0;
    let b = src.as_bytes();
    while i < src.len() {
        if src[i..].starts_with(old) {
            let before_ok = i == 0 || !word(b[i - 1] as char);
            let after = i + old.len();
            let after_ok = after >= src.len() || !word(b[after] as char);
            if before_ok && after_ok {
                out.push_str(new);
                i = after;
                continue;
            }
        }
        let ch = src[i..].chars().next().unwrap();
        out.push(ch);
        i += ch.len_utf8();
    }
    out
}

fn measure_angle(d1: Vec2, d2: Vec2) -> f64 {
    geom::norm_angle(d2.y.atan2(d2.x) - d1.y.atan2(d1.x))
}

/// Build the constraints a command adds for a selection (FreeCAD's selection rules).
fn build(sk: &Sketch, cmd: Cmd, p: &Parts) -> Result<Vec<K>, String> {
    let pts: Vec<PointRef> = p.pts.iter().map(|x| geom::canon(sk, *x)).collect();
    let edges = &p.edges;
    let lines: Vec<GeoId> = edges.iter().copied().filter(|g| geom::is_line(sk, *g)).collect();
    let rounds: Vec<GeoId> = edges.iter().copied().filter(|g| geom::is_round(sk, *g)).collect();
    let internal = |g: &GeoId| !fc_sketch::is_external(*g);
    let pt = |r: PointRef| sk.point(r);
    let mut v = vec![];
    match cmd {
        Cmd::Coincident => {
            if pts.len() >= 2 && edges.is_empty() {
                for q in &pts[1..] {
                    v.push(K::Coincident(pts[0], *q));
                }
            } else if pts.len() == 1 && !edges.is_empty() {
                for e in edges {
                    v.push(K::PointOnObject(pts[0], *e));
                }
            } else if pts.is_empty() && rounds.len() >= 2 && rounds.len() == edges.len() {
                for e in &rounds[1..] {
                    v.push(K::Coincident(PointRef::mid(rounds[0]), PointRef::mid(*e)));
                }
            } else {
                return Err("select two or more vertices, a vertex and a curve, or two circles/arcs".into());
            }
        }
        Cmd::PointOnObject => {
            if pts.is_empty() || edges.is_empty() {
                return Err("select vertices and a curve".into());
            }
            for q in &pts {
                for e in edges {
                    if q.geo != *e {
                        v.push(K::PointOnObject(*q, *e));
                    }
                }
            }
        }
        Cmd::Horizontal | Cmd::Vertical => {
            let h = cmd == Cmd::Horizontal;
            let own: Vec<GeoId> = lines.iter().copied().filter(internal).collect();
            if !own.is_empty() && pts.is_empty() && own.len() == edges.len() {
                for g in own {
                    v.push(if h { K::Horizontal(g) } else { K::Vertical(g) });
                }
            } else if pts.len() == 2 && edges.is_empty() {
                v.push(if h { K::HorizontalPoints(pts[0], pts[1]) } else { K::VerticalPoints(pts[0], pts[1]) });
            } else {
                return Err("select one or more lines, or two vertices".into());
            }
        }
        Cmd::Parallel => {
            if lines.len() < 2 || lines.len() != edges.len() || !pts.is_empty() {
                return Err("select two or more lines".into());
            }
            for g in &lines[1..] {
                v.push(K::Parallel(lines[0], *g));
            }
        }
        Cmd::Perpendicular => {
            if edges.len() != 2 || !pts.is_empty() {
                return Err("select two curves".into());
            }
            v.push(K::Perpendicular(edges[0], edges[1]));
        }
        Cmd::Tangent => {
            if edges.len() == 2 && pts.is_empty() {
                v.push(K::Tangent(edges[0], edges[1]));
            } else if pts.len() == 2 && edges.is_empty() && pts[0].geo != pts[1].geo {
                v.push(K::TangentAt(pts[0], pts[1]));
            } else if pts.len() == 1 && edges.len() == 1 {
                v.push(K::TangentAt(pts[0], PointRef::edge(edges[0])));
            } else {
                return Err("select two curves, two endpoints, or an endpoint and a curve".into());
            }
        }
        Cmd::Equal => {
            let same = (lines.len() == edges.len() && lines.len() >= 2) || (rounds.len() == edges.len() && rounds.len() >= 2);
            if !same || !pts.is_empty() {
                return Err("select two or more lines, or two or more circles/arcs".into());
            }
            for g in &edges[1..] {
                v.push(K::Equal(edges[0], *g));
            }
        }
        Cmd::Symmetric => {
            if pts.len() == 2 && edges.len() == 1 && geom::is_line(sk, edges[0]) {
                v.push(K::SymmetricLine(pts[0], pts[1], edges[0]));
            } else if pts.len() == 3 && edges.is_empty() {
                v.push(K::Symmetric(pts[0], pts[1], pts[2]));
            } else if pts.len() == 1 && edges.len() == 1 && geom::is_line(sk, edges[0]) && internal(&edges[0]) {
                v.push(K::Symmetric(PointRef::start(edges[0]), PointRef::end(edges[0]), pts[0]));
            } else {
                return Err("select two vertices and a symmetry line or vertex (last), or a line and a vertex".into());
            }
        }
        Cmd::Block => {
            let own: Vec<GeoId> = edges.iter().copied().filter(internal).collect();
            if own.is_empty() {
                return Err("select curves to block".into());
            }
            for g in own {
                v.push(K::Block(g));
            }
        }
        Cmd::Lock => {
            let own: Vec<PointRef> = pts.iter().copied().filter(|q| q.geo != ROOT).collect();
            if own.is_empty() {
                return Err("select vertices to lock".into());
            }
            for q in own {
                let p = pt(q);
                v.push(K::LockX(q, p.x));
                v.push(K::LockY(q, p.y));
            }
        }
        Cmd::DistanceX | Cmd::DistanceY => {
            let x = cmd == Cmd::DistanceX;
            let comp = |a: Vec2| if x { a.x } else { a.y };
            let (a, b) = if pts.len() == 2 && edges.is_empty() {
                (pts[0], pts[1])
            } else if pts.is_empty() && lines.len() == 1 && edges.len() == 1 && internal(&lines[0]) {
                (PointRef::start(lines[0]), PointRef::end(lines[0]))
            } else if pts.len() == 1 && edges.is_empty() && pts[0].geo != ROOT {
                let q = pts[0];
                v.push(if x { K::LockX(q, pt(q).x) } else { K::LockY(q, pt(q).y) });
                return Ok(v);
            } else {
                return Err("select two vertices, a line, or one vertex".into());
            };
            // order so the value is positive, like FreeCAD
            let (a, b) = if comp(pt(b)) - comp(pt(a)) < 0.0 { (b, a) } else { (a, b) };
            let d = comp(pt(b)) - comp(pt(a));
            v.push(if x { K::DistanceX(a, b, d) } else { K::DistanceY(a, b, d) });
        }
        Cmd::Distance => {
            if pts.len() == 2 && edges.is_empty() {
                v.push(K::Distance(pts[0], pts[1], pt(pts[0]).dist(pt(pts[1]))));
            } else if pts.is_empty() && edges.len() == 1 && lines.len() == 1 && internal(&lines[0]) {
                v.push(K::DistanceLine(lines[0], sk.geometry[lines[0]].curve.length()));
            } else if pts.len() == 1 && edges.len() == 1 && pts[0].geo != edges[0] {
                let p0 = pt(pts[0]);
                let d = if let Some((a, b)) = geom::line_of(sk, edges[0]) {
                    ((b - a).cross(p0 - a) / (b - a).length()).abs()
                } else if let Some((c, r)) = geom::circle_of(sk, edges[0]) {
                    (p0.dist(c) - r).abs()
                } else {
                    return Err("select a vertex and a line or circle".into());
                };
                v.push(K::PointLineDistance(pts[0], edges[0], d));
            } else {
                return Err("select two vertices, a line, or a vertex and a curve".into());
            }
        }
        Cmd::Radius | Cmd::Diameter => {
            if rounds.is_empty() || rounds.len() != edges.len() || !pts.is_empty() {
                return Err("select circles or arcs".into());
            }
            for g in rounds {
                let r = geom::circle_of(sk, g).unwrap().1;
                v.push(if cmd == Cmd::Radius { K::Radius(g, r) } else { K::Diameter(g, 2.0 * r) });
            }
        }
        Cmd::Angle => {
            if lines.len() == 2 && edges.len() == 2 && pts.is_empty() {
                let (a1, b1) = geom::line_of(sk, lines[0]).unwrap();
                let (a2, b2) = geom::line_of(sk, lines[1]).unwrap();
                let ang = measure_angle(b1 - a1, b2 - a2);
                if ang <= PI {
                    v.push(K::AngleBetween(lines[0], lines[1], ang));
                } else {
                    v.push(K::AngleBetween(lines[1], lines[0], 2.0 * PI - ang));
                }
            } else if edges.len() == 1 && pts.is_empty() && lines.len() == 1 && internal(&lines[0]) {
                let (a, b) = geom::line_of(sk, lines[0]).unwrap();
                v.push(K::Angle(lines[0], (b - a).y.atan2((b - a).x)));
            } else if edges.len() == 1 && pts.is_empty() && matches!(sk.geometry.get(edges[0]).map(|g| &g.curve), Some(Curve::Arc { .. })) {
                v.push(K::Angle(edges[0], sk.geometry[edges[0]].curve.sweep()));
            } else {
                return Err("select two lines, one line, or an arc".into());
            }
        }
    }
    Ok(v)
}

#[cfg(test)]
mod tests {
    use super::*;
    use fc_sketch::{Geometry, Sketch};

    #[test]
    fn rename_identifier_whole_words() {
        assert_eq!(rename_ident("width*0.7 + Sketch.width - widths", "width", "w"), "w*0.7 + Sketch.w - widths");
    }

    #[test]
    fn selection_rules() {
        let mut sk = Sketch::new();
        sk.add_geometry(Geometry::line(Vec2::new(0.0, 0.0), Vec2::new(10.0, 1.0)));
        sk.add_geometry(Geometry::circle(Vec2::new(20.0, 0.0), 3.0));
        let parts = |pts: Vec<PointRef>, edges: Vec<GeoId>| Parts { pts, edges, cons: vec![] };
        // DistanceX of a reversed pair is ordered to be positive
        let v = build(&sk, Cmd::DistanceX, &parts(vec![PointRef::end(0), PointRef::start(0)], vec![])).unwrap();
        assert!(matches!(v[0], K::DistanceX(a, _, d) if a == PointRef::start(0) && (d - 10.0).abs() < 1e-12));
        // a single vertex gives a lock from the origin
        assert!(matches!(build(&sk, Cmd::DistanceY, &parts(vec![PointRef::end(0)], vec![])).unwrap()[0], K::LockY(..)));
        assert!(matches!(build(&sk, Cmd::Radius, &parts(vec![], vec![1])).unwrap()[0], K::Radius(1, r) if (r - 3.0).abs() < 1e-12));
        assert!(build(&sk, Cmd::Radius, &parts(vec![], vec![0])).is_err());
        assert!(matches!(build(&sk, Cmd::Coincident, &parts(vec![PointRef::end(0)], vec![1])).unwrap()[0], K::PointOnObject(..)));
        assert!(matches!(build(&sk, Cmd::Distance, &parts(vec![PointRef::mid(1)], vec![0])).unwrap()[0], K::PointLineDistance(..)));
    }
}
