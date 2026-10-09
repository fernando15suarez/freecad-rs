//! Drawing the sketch in edit mode (FreeCAD EditModeCoinManager colors and conventions).

use super::tools::{self, Snap, Tool};
use super::{geom, pick, Drag, Proj, Sel, SketchEditor};
use eframe::egui::{self, Align2, Color32 as C, FontId, Painter, Pos2, Rect, Shape, Stroke, Vec2 as V};
use fc_part::Document;
use fc_sketch::{ConstraintKind as K, Curve, GeoId, PointPos, PointRef, Sketch, Vec2, H_AXIS, ROOT, V_AXIS};
use std::f64::consts::PI;

// FreeCAD DrawingParameters defaults
pub const COL_CURVE: C = C::WHITE;
pub const COL_FULL: C = C::from_rgb(0, 255, 0);
pub const COL_FULL_ELEM: C = C::from_rgb(128, 208, 160);
pub const COL_CONSTRUCTION: C = C::from_rgb(0, 0, 220);
pub const COL_CONSTRUCTION_FULL: C = C::from_rgb(143, 169, 253);
pub const COL_INVALID: C = C::from_rgb(255, 109, 0);
pub const COL_DIM: C = C::from_rgb(255, 38, 0);
pub const COL_REF: C = C::from_rgb(0, 38, 255);
pub const COL_EXPR: C = C::from_rgb(255, 127, 38);
pub const COL_CURSOR_TEXT: C = C::from_rgb(0, 0, 255);
pub const COL_AXIS_X: C = C::from_rgb(204, 51, 51);
pub const COL_AXIS_Y: C = C::from_rgb(51, 204, 51);
pub const COL_ROOT: C = C::from_rgb(204, 51, 153);
// hover / selection like the 3D view (gpu::HOVER_COLOR / SEL_COLOR)
pub const COL_HOVER: C = C::from_rgb(237, 224, 26);
pub const COL_SEL: C = C::from_rgb(28, 178, 28);
// solver diagnosis
pub const COL_CONFLICT: C = C::from_rgb(235, 0, 0);
pub const COL_REDUNDANT: C = C::from_rgb(255, 150, 0);

const LINE_W: f32 = 2.0;
const DIM_FONT: f32 = 13.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Glyph {
    H,
    V,
    Parallel,
    Perpendicular,
    Tangent,
    Equal,
    PointOn,
    Symmetric,
    Block,
    Coincident,
}

/// Geometry a dimension label is laid out against; the label offset stored in
/// `SketchEditor::label_off` is the parameter `param_from_cursor` returns.
pub enum DimFrame {
    /// Dimension line offset along the unit normal `n`; param = offset from `p1·n`.
    Linear { p1: Vec2, p2: Vec2, n: Vec2 },
    /// Radius/diameter; param = label direction angle.
    Radial { c: Vec2, r: f64, default: f64 },
    /// Angle arc at `x` from `base` sweeping `sweep`; param = arc radius.
    Angular { x: Vec2, base: f64, sweep: f64, default_r: Option<f64> },
    /// Drawn directly between two points.
    Direct { a: Vec2, b: Vec2 },
}

impl DimFrame {
    pub fn param_from_cursor(&self, w: Vec2) -> f64 {
        match self {
            DimFrame::Linear { p1, n, .. } => w.dot(*n) - p1.dot(*n),
            DimFrame::Radial { c, .. } => (w - *c).y.atan2((w - *c).x),
            DimFrame::Angular { x, .. } => w.dist(*x).max(1e-6),
            DimFrame::Direct { .. } => 0.0,
        }
    }
}

fn sv(v: V) -> V {
    let l = v.length();
    if l < 1e-6 { V::X } else { v / l }
}

pub fn arrow(p: &Painter, tip: Pos2, dir: V, col: C) {
    let d = sv(dir);
    let n = V::new(-d.y, d.x);
    p.add(Shape::convex_polygon(vec![tip, tip - d * 10.0 + n * 3.5, tip - d * 10.0 - n * 3.5], col, Stroke::NONE));
}

/// Text centered at `c`, optional background; returns its rect.
fn label(p: &Painter, c: Pos2, text: &str, col: C, bg: Option<C>) -> Rect {
    let g = p.layout_no_wrap(text.to_string(), FontId::proportional(DIM_FONT), col);
    let r = Rect::from_center_size(c, g.size());
    if let Some(b) = bg {
        p.rect_filled(r.expand(2.0), 3.0, b);
    }
    p.galley(r.min, g, col);
    r
}

/// Constraint badge: rounded square filled with `fill`, white symbol.
pub fn badge(p: &Painter, c: Pos2, g: Glyph, fill: C, ring: Option<C>) -> Rect {
    let r = Rect::from_center_size(c, egui::vec2(17.0, 17.0));
    if let Some(rc) = ring {
        // conflicting / redundant: glowing halo + white border
        p.rect_filled(r.expand(6.0), 8.0, C::from_rgba_unmultiplied(rc.r(), rc.g(), rc.b(), 90));
        p.rect_filled(r, 3.5, fill);
        p.rect_stroke(r, 3.5, Stroke::new(1.5, C::WHITE), egui::StrokeKind::Outside);
    } else {
        p.rect_filled(r, 3.5, fill);
    }
    let w = Stroke::new(1.6, C::WHITE);
    let o = |x: f32, y: f32| c + egui::vec2(x, y);
    match g {
        Glyph::H | Glyph::V => {
            p.text(c + egui::vec2(0.0, 0.5), Align2::CENTER_CENTER, if g == Glyph::H { "H" } else { "V" }, FontId::proportional(12.0), C::WHITE);
        }
        Glyph::Parallel => {
            p.line_segment([o(-4.0, 5.0), o(0.0, -5.0)], w);
            p.line_segment([o(0.5, 5.0), o(4.5, -5.0)], w);
        }
        Glyph::Perpendicular => {
            p.line_segment([o(0.0, -5.5), o(0.0, 4.5)], w);
            p.line_segment([o(-5.5, 4.5), o(5.5, 4.5)], w);
        }
        Glyph::Tangent => {
            p.circle_stroke(o(0.0, -1.5), 3.8, w);
            p.line_segment([o(-6.0, 2.3), o(6.0, 2.3)], w);
        }
        Glyph::Equal => {
            p.line_segment([o(-5.0, -2.2), o(5.0, -2.2)], w);
            p.line_segment([o(-5.0, 2.2), o(5.0, 2.2)], w);
        }
        Glyph::PointOn => {
            p.line_segment([o(-6.0, 4.0), o(6.0, -4.0)], w);
            p.circle_filled(o(0.0, 0.0), 2.6, C::WHITE);
        }
        Glyph::Symmetric => {
            p.line_segment([o(0.0, -6.0), o(0.0, 6.0)], Stroke::new(1.2, C::WHITE));
            p.add(Shape::convex_polygon(vec![o(-6.5, -3.5), o(-2.0, 0.0), o(-6.5, 3.5)], C::WHITE, Stroke::NONE));
            p.add(Shape::convex_polygon(vec![o(6.5, -3.5), o(6.5, 3.5), o(2.0, 0.0)], C::WHITE, Stroke::NONE));
        }
        Glyph::Block => {
            p.rect_filled(Rect::from_min_max(o(-4.5, -0.5), o(4.5, 5.5)), 1.0, C::WHITE);
            p.add(Shape::line(vec![o(-2.8, -0.5), o(-2.8, -3.5), o(-1.5, -5.2), o(1.5, -5.2), o(2.8, -3.5), o(2.8, -0.5)], w));
        }
        Glyph::Coincident => {
            p.circle_stroke(c, 4.5, w);
            p.circle_filled(c, 2.0, C::WHITE);
        }
    }
    r
}

fn polyline(p: &Painter, pts: Vec<Pos2>, col: C, w: f32, dashed: bool) {
    if pts.len() < 2 {
        return;
    }
    if dashed {
        p.extend(Shape::dashed_line(&pts, Stroke::new(w, col), 7.0, 4.0));
    } else {
        p.add(Shape::line(pts, Stroke::new(w, col)));
    }
}

impl SketchEditor {
    fn constraint_color(&self, doc: &Document, ci: usize) -> (C, Option<C>) {
        let sk = self.sk(doc);
        let c = &sk.constraints[ci];
        let ring = self.report.as_ref().and_then(|r| {
            if r.conflicting.contains(&ci) || r.malformed.contains(&ci) {
                Some(COL_CONFLICT)
            } else if r.redundant.contains(&ci) || r.partially_redundant.contains(&ci) {
                Some(COL_REDUNDANT)
            } else {
                None
            }
        });
        let col = if self.hover == Some(Sel::Constraint(ci)) {
            COL_HOVER
        } else if self.is_sel(Sel::Constraint(ci)) {
            COL_SEL
        } else if let Some(r) = ring {
            r
        } else if !c.driving {
            COL_REF
        } else if doc.expressions.contains_key(&self.ckey(doc, ci)) {
            COL_EXPR
        } else {
            COL_DIM
        };
        (col, ring)
    }

    pub(crate) fn dim_text(&self, doc: &Document, ci: usize) -> String {
        let sk = self.sk(doc);
        let c = &sk.constraints[ci];
        let Some(v) = geom::display_value(&c.kind) else { return String::new() };
        let num = geom::fmt_num(v);
        let val = match c.kind {
            K::Radius(..) => format!("R {num} mm"),
            K::Diameter(..) => format!("Ø {num} mm"),
            K::Angle(..) | K::AngleBetween(..) => format!("{num}°"),
            _ => format!("{num} mm"),
        };
        let mut s = match &c.name {
            Some(n) => format!("{n} = {val}"),
            None => val,
        };
        if doc.expressions.contains_key(&self.ckey(doc, ci)) {
            s = format!("ƒx {s}");
        }
        if !c.driving {
            s = format!("({s})");
        }
        s
    }

    /// Layout frame of a dimensional constraint.
    pub(crate) fn dim_frame(&self, sk: &Sketch, ci: usize) -> Option<DimFrame> {
        let c = sk.constraints.get(ci)?;
        let pt = |r: PointRef| sk.try_point(r);
        Some(match &c.kind {
            K::DistanceX(a, b, _) => DimFrame::Linear { p1: pt(*a)?, p2: pt(*b)?, n: Vec2::new(0.0, 1.0) },
            K::DistanceY(a, b, _) => DimFrame::Linear { p1: pt(*a)?, p2: pt(*b)?, n: Vec2::new(1.0, 0.0) },
            K::LockX(a, _) => DimFrame::Linear { p1: Vec2::ZERO, p2: pt(*a)?, n: Vec2::new(0.0, 1.0) },
            K::LockY(a, _) => DimFrame::Linear { p1: Vec2::ZERO, p2: pt(*a)?, n: Vec2::new(1.0, 0.0) },
            K::Distance(a, b, _) => {
                let (p1, p2) = (pt(*a)?, pt(*b)?);
                let d = (p2 - p1).normalized();
                let n = if d.length() < 0.5 { Vec2::new(0.0, 1.0) } else { d.perp() };
                DimFrame::Linear { p1, p2, n }
            }
            K::DistanceLine(g, _) => {
                let (p1, p2) = geom::line_of(sk, *g)?;
                let d = (p2 - p1).normalized();
                let n = if d.length() < 0.5 { Vec2::new(0.0, 1.0) } else { d.perp() };
                DimFrame::Linear { p1, p2, n }
            }
            K::PointLineDistance(a, g, _) => {
                let p = pt(*a)?;
                let f = if let Some((la, lb)) = geom::line_of(sk, *g) {
                    let d = (lb - la).normalized();
                    la + d * (p - la).dot(d)
                } else {
                    let (cc, r) = geom::circle_of(sk, *g)?;
                    let d = (p - cc).normalized();
                    cc + if d.length() < 0.5 { Vec2::new(r, 0.0) } else { d * r }
                };
                DimFrame::Direct { a: p, b: f }
            }
            K::Radius(g, _) | K::Diameter(g, _) => {
                let (cc, r) = geom::circle_of(sk, *g)?;
                let default = match &sk.geometry[*g].curve {
                    Curve::Arc { start, .. } => start + sk.geometry[*g].curve.sweep() * 0.5,
                    _ => PI / 4.0,
                };
                DimFrame::Radial { c: cc, r, default }
            }
            K::Angle(g, v) => match &sk.geometry.get(*g)?.curve {
                Curve::Line { a, .. } => DimFrame::Angular { x: *a, base: 0.0, sweep: *v, default_r: None },
                Curve::Arc { c: cc, r, start, .. } => DimFrame::Angular { x: *cc, base: *start, sweep: *v, default_r: Some(*r) },
                _ => return None,
            },
            K::AngleBetween(g1, g2, v) => {
                let (a1, b1) = geom::line_of(sk, *g1)?;
                let (a2, b2) = geom::line_of(sk, *g2)?;
                let (d1, d2) = ((b1 - a1).normalized(), (b2 - a2).normalized());
                let x = geom::line_intersection(a1, d1, a2, d2).unwrap_or(b1.lerp(a2, 0.5));
                DimFrame::Angular { x, base: d1.y.atan2(d1.x), sweep: *v, default_r: None }
            }
            _ => return None,
        })
    }

    /// Where icons of constraints on curve `g` go: (anchor, along, outward normal).
    fn icon_anchor(&self, sk: &Sketch, pj: &Proj, g: GeoId) -> Option<(Pos2, V, V)> {
        let up = |along: V| {
            let n = V::new(along.y, -along.x);
            if n.y > 0.0 { -n } else { n }
        };
        match g {
            H_AXIS => {
                let o = pj.s(Vec2::ZERO);
                let u = sv(pj.s(Vec2::new(1.0, 0.0)) - o);
                Some((o + u * 60.0, u, up(u)))
            }
            V_AXIS => {
                let o = pj.s(Vec2::ZERO);
                let u = sv(pj.s(Vec2::new(0.0, 1.0)) - o);
                Some((o + u * 60.0, u, V::new(-1.0, 0.0)))
            }
            _ => match &sk.geometry.get(g)?.curve {
                Curve::Point { p } => Some((pj.s(*p), V::X, V::new(0.6, -0.8))),
                Curve::Line { a, b } => {
                    let (sa, sb) = (pj.s(*a), pj.s(*b));
                    let u = sv(sb - sa);
                    Some((sa + (sb - sa) * 0.5, u, up(u)))
                }
                c @ (Curve::Circle { .. } | Curve::Arc { .. }) => {
                    let (cc, t) = match c {
                        Curve::Arc { c: cc, start, .. } => (*cc, start + c.sweep() * 0.5),
                        Curve::Circle { c: cc, .. } => (*cc, PI * 0.75),
                        _ => unreachable!(),
                    };
                    let (cc_, r) = match c {
                        Curve::Arc { r, .. } | Curve::Circle { r, .. } => (cc, *r),
                        _ => unreachable!(),
                    };
                    let p = cc_ + Vec2::polar(t) * r;
                    let sp = pj.s(p);
                    let n = sv(sp - pj.s(cc_));
                    Some((sp, V::new(-n.y, n.x), n))
                }
            },
        }
    }

    pub(crate) fn render(&mut self, doc: &Document, pj: &Proj, p: &Painter) {
        self.hits.clear();
        let sk = self.sk(doc);
        let rect = self.rect;
        let invalid = self.invalid();
        let full = self.fully_constrained(doc);

        // grid
        if self.grid {
            let s = tools::grid_spacing(pj.ppu());
            let corners: Vec<Vec2> = [rect.left_top(), rect.right_top(), rect.left_bottom(), rect.right_bottom()].iter().filter_map(|c| pj.w(*c)).collect();
            if corners.len() == 4 {
                let (mut x0, mut x1, mut y0, mut y1) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
                for c in &corners {
                    x0 = x0.min(c.x);
                    x1 = x1.max(c.x);
                    y0 = y0.min(c.y);
                    y1 = y1.max(c.y);
                }
                let st = Stroke::new(1.0, C::from_rgba_unmultiplied(120, 120, 140, 70));
                let (i0, i1) = ((x0 / s).floor() as i64, (x1 / s).ceil() as i64);
                if i1 - i0 < 400 {
                    for i in i0..=i1 {
                        p.line_segment([pj.s(Vec2::new(i as f64 * s, y0)), pj.s(Vec2::new(i as f64 * s, y1))], st);
                    }
                }
                let (j0, j1) = ((y0 / s).floor() as i64, (y1 / s).ceil() as i64);
                if j1 - j0 < 400 {
                    for j in j0..=j1 {
                        p.line_segment([pj.s(Vec2::new(x0, j as f64 * s)), pj.s(Vec2::new(x1, j as f64 * s))], st);
                    }
                }
            }
        }

        // axes (FreeCAD: external geometry -1 / -2)
        let o = pj.s(Vec2::ZERO);
        for (g, dir, col) in [(H_AXIS, Vec2::new(1.0, 0.0), COL_AXIS_X), (V_AXIS, Vec2::new(0.0, 1.0), COL_AXIS_Y)] {
            let u = sv(pj.s(dir) - o);
            let (c, w) = if self.hover == Some(Sel::Edge(g)) {
                (COL_HOVER, 3.0)
            } else if self.is_sel(Sel::Edge(g)) {
                (COL_SEL, 3.0)
            } else {
                (col, 1.5)
            };
            p.line_segment([o - u * 6000.0, o + u * 6000.0], Stroke::new(w, c));
        }

        // curves
        for (gi, g) in sk.geometry.iter().enumerate() {
            if matches!(g.curve, Curve::Point { .. }) {
                continue;
            }
            let pts = pick::screen_polyline(pj, &g.curve);
            let fully = self.fully.get(gi).copied().unwrap_or(false);
            let hl = if self.hover == Some(Sel::Edge(gi)) {
                Some(COL_HOVER)
            } else if self.is_sel(Sel::Edge(gi)) {
                Some(COL_SEL)
            } else {
                None
            };
            let base = if invalid {
                COL_INVALID
            } else if g.construction {
                if fully { COL_CONSTRUCTION_FULL } else { COL_CONSTRUCTION }
            } else if full {
                COL_FULL
            } else if fully {
                COL_FULL_ELEM
            } else {
                COL_CURVE
            };
            let col = hl.unwrap_or(base);
            let w = if hl.is_some() { LINE_W + 1.5 } else { LINE_W };
            if !g.construction {
                polyline(p, pts.clone(), C::from_black_alpha(60), w + 2.0, false);
            }
            polyline(p, pts, col, w, g.construction);
        }

        // constraints
        if self.show_constraints {
            self.render_constraints(doc, pj, p);
        }

        // vertices
        let mut rings: Vec<(PointRef, C)> = vec![];
        if self.show_constraints {
            for (ci, c) in sk.constraints.iter().enumerate() {
                if let K::Coincident(a, _) = c.kind {
                    let (col, ring) = self.constraint_color(doc, ci);
                    if ring.is_some() || col == COL_SEL || col == COL_HOVER {
                        rings.push((geom::canon(sk, a), ring.unwrap_or(col)));
                    }
                }
            }
        }
        for pr in geom::all_points(sk) {
            let Some(q) = sk.try_point(pr) else { continue };
            let s = pj.s(q);
            if !rect.expand(10.0).contains(s) {
                continue;
            }
            if let Some((_, rc)) = rings.iter().find(|(r, _)| *r == geom::canon(sk, pr)) {
                p.circle_stroke(s, 6.5, Stroke::new(2.0, *rc));
            }
            let hl = if self.hover == Some(Sel::Point(pr)) {
                Some(COL_HOVER)
            } else if self.is_sel(Sel::Point(pr)) {
                Some(COL_SEL)
            } else {
                None
            };
            let base = if pr.geo == ROOT {
                COL_ROOT
            } else {
                let g = &sk.geometry[pr.geo];
                let fully = self.fully.get(pr.geo).copied().unwrap_or(false);
                if invalid {
                    COL_INVALID
                } else if full {
                    COL_FULL
                } else if fully {
                    if g.construction { COL_CONSTRUCTION_FULL } else { COL_FULL_ELEM }
                } else {
                    C::WHITE
                }
            };
            let r = if hl.is_some() { 5.0 } else { 3.5 };
            p.circle(s, r, hl.unwrap_or(base), Stroke::new(1.0, C::from_black_alpha(140)));
        }

        // tool preview, snap hint, cursor text
        self.render_tool(doc, pj, p);

        if let Drag::Box { start, cur } = self.drag {
            let r = Rect::from_two_pos(start, cur);
            p.rect_filled(r, 0.0, C::from_rgba_unmultiplied(80, 140, 255, 40));
            p.rect_stroke(r, 0.0, Stroke::new(1.0, C::from_rgb(40, 90, 220)), egui::StrokeKind::Middle);
        }
    }

    fn render_constraints(&mut self, doc: &Document, pj: &Proj, p: &Painter) {
        let sk = self.sk(doc);
        let ppu = pj.ppu();
        let (bmn, bmx) = geom::bbox(sk);
        let center = bmn.lerp(bmx, 0.5);
        let mut placed: Vec<Rect> = vec![];
        let mut levels: Vec<(Vec2, f64, f64, f64, f64)> = vec![];
        let mut hits: Vec<(usize, Rect)> = vec![];
        let pt_anchor = |pr: PointRef, normal: V| -> Option<(Pos2, V, V)> { sk.try_point(pr).map(|q| (pj.s(q), V::X, normal)) };
        let diag = V::new(0.7, -0.7);
        // dimensions first so icons avoid their labels
        for (pass, (ci, c)) in [0, 1].into_iter().flat_map(|pass| sk.constraints.iter().enumerate().map(move |x| (pass, x))) {
            if (pass == 0) != c.kind.is_dimensional() {
                continue;
            }
            let (col, ring) = self.constraint_color(doc, ci);
            let mut items: Vec<(Option<(Pos2, V, V)>, Glyph)> = vec![];
            let geo = |g: GeoId| self.icon_anchor(sk, pj, g);
            match &c.kind {
                K::Coincident(..) => {}
                K::Horizontal(g) => items.push((geo(*g), Glyph::H)),
                K::Vertical(g) => items.push((geo(*g), Glyph::V)),
                K::Parallel(a, b) => items.extend([(geo(*a), Glyph::Parallel), (geo(*b), Glyph::Parallel)]),
                K::Perpendicular(a, b) => items.extend([(geo(*a), Glyph::Perpendicular), (geo(*b), Glyph::Perpendicular)]),
                K::Tangent(a, b) => items.extend([(geo(*a), Glyph::Tangent), (geo(*b), Glyph::Tangent)]),
                K::Equal(a, b) => items.extend([(geo(*a), Glyph::Equal), (geo(*b), Glyph::Equal)]),
                K::Block(g) => items.push((geo(*g), Glyph::Block)),
                K::TangentAt(a, _) => items.push((pt_anchor(*a, diag), Glyph::Tangent)),
                K::PointOnObject(a, _) => items.push((pt_anchor(*a, diag), Glyph::PointOn)),
                K::HorizontalPoints(a, b) | K::VerticalPoints(a, b) => {
                    let glyph = if matches!(c.kind, K::HorizontalPoints(..)) { Glyph::H } else { Glyph::V };
                    let at = sk.try_point(*a).zip(sk.try_point(*b)).map(|(qa, qb)| (pj.s(qa.lerp(qb, 0.5)), V::X, V::new(0.0, -1.0)));
                    items.push((at, glyph));
                }
                K::Symmetric(a, b, m) => {
                    let at = if m.pos == PointPos::Edge { None } else { sk.try_point(*m) };
                    let q = at.or_else(|| Some(sk.try_point(*a)?.lerp(sk.try_point(*b)?, 0.5)));
                    items.push((q.map(|q| (pj.s(q), V::X, diag)), Glyph::Symmetric));
                }
                K::SymmetricLine(a, b, _) => {
                    let at = sk.try_point(*a).zip(sk.try_point(*b)).map(|(qa, qb)| (pj.s(qa.lerp(qb, 0.5)), V::X, diag));
                    items.push((at, Glyph::Symmetric));
                }
                _ => {
                    // dimensions
                    let text = self.dim_text(doc, ci);
                    let Some(frame) = self.dim_frame(sk, ci) else { continue };
                    let bg = ring.map(|r| C::from_rgba_unmultiplied(r.r(), r.g(), r.b(), 70));
                    let off = self.label_off.get(&ci).copied();
                    let r = draw_dim(p, pj, &frame, off, ppu, center, &text, col, bg, &mut levels, &placed);
                    placed.push(r.expand(2.0));
                    hits.push((ci, r));
                }
            }
            for (at, glyph) in items {
                let Some((anchor, along, normal)) = at else { continue };
                let pos = place(&mut placed, anchor, along, normal);
                hits.push((ci, badge(p, pos, glyph, col, ring)));
            }
        }
        self.hits = hits;
    }

    fn render_tool(&self, doc: &Document, pj: &Proj, p: &Painter) {
        let t = &self.tool;
        if t.tool == Tool::Select {
            return;
        }
        let Some(cur) = t.cur else { return };
        let Some(cs) = self.cursor_px else { return };
        let col = if self.construction { COL_CONSTRUCTION } else { C::WHITE };
        let dashed = self.construction;
        let line = |a: Vec2, b: Vec2| polyline(p, vec![pj.s(a), pj.s(b)], col, 1.5, dashed);
        let curve = |c: &Curve| polyline(p, pick::screen_polyline(pj, c), col, 1.5, dashed);
        let mut info = format!("({}, {})", geom::fmt_num(cur.p.x), geom::fmt_num(cur.p.y));
        let mut hv: Option<bool> = None;
        let n = t.pts.len();
        let sk = self.sk(doc);
        match t.tool {
            Tool::Line | Tool::Polyline => {
                let start = match t.poly_prev {
                    Some(g) => sk.try_point(PointRef::end(g)),
                    None => t.pts.first().map(|s| s.p),
                };
                if let Some(a) = start {
                    line(a, cur.p);
                    let d = cur.p - a;
                    info = format!("{} mm, {}°", geom::fmt_num(d.length()), geom::fmt_num(d.y.atan2(d.x).to_degrees()));
                    hv = tools::hv_auto(a, cur.p);
                }
            }
            Tool::Rectangle | Tool::CenterRectangle if n == 1 => {
                let s0 = t.pts[0].p;
                let (c0, c2) = if t.tool == Tool::CenterRectangle { (s0 * 2.0 - cur.p, cur.p) } else { (s0, cur.p) };
                let c = [c0, Vec2::new(c2.x, c0.y), c2, Vec2::new(c0.x, c2.y)];
                for i in 0..4 {
                    line(c[i], c[(i + 1) % 4]);
                }
                info = format!("{} x {} mm", geom::fmt_num((c2.x - c0.x).abs()), geom::fmt_num((c2.y - c0.y).abs()));
            }
            Tool::Circle if n == 1 => {
                let r = t.pts[0].p.dist(cur.p);
                curve(&Curve::Circle { c: t.pts[0].p, r });
                polyline(p, vec![pj.s(t.pts[0].p), pj.s(cur.p)], col, 1.0, true);
                info = format!("R {} mm", geom::fmt_num(r));
            }
            Tool::Arc if n >= 1 => {
                let c = t.pts[0].p;
                if n == 1 {
                    polyline(p, vec![pj.s(c), pj.s(cur.p)], col, 1.0, true);
                    info = format!("R {} mm", geom::fmt_num(c.dist(cur.p)));
                } else {
                    let s1 = t.pts[1].p;
                    let r = c.dist(s1);
                    let sa = (s1 - c).y.atan2((s1 - c).x);
                    let sw = t.arc_sweep;
                    if sw.abs() > 1e-4 {
                        let (st, en) = if sw > 0.0 { (sa, sa + sw) } else { (sa + sw, sa) };
                        curve(&Curve::Arc { c, r, start: st, end: en });
                    }
                    polyline(p, vec![pj.s(c), pj.s(c + (cur.p - c).normalized() * r)], col, 1.0, true);
                    info = format!("{}°", geom::fmt_num(sw.to_degrees()));
                }
            }
            Tool::Arc3 if n >= 1 => {
                if n == 1 {
                    polyline(p, vec![pj.s(t.pts[0].p), pj.s(cur.p)], col, 1.0, true);
                } else if let Some((arc, _)) = tools::arc3(t.pts[0].p, t.pts[1].p, cur.p) {
                    curve(&arc);
                    if let Curve::Arc { r, .. } = arc {
                        info = format!("R {} mm", geom::fmt_num(r));
                    }
                }
            }
            Tool::Slot if n >= 1 => {
                if n == 1 {
                    line(t.pts[0].p, cur.p);
                    hv = tools::hv_auto(t.pts[0].p, cur.p);
                } else {
                    let r = tools::slot_radius(t.pts[0].p, t.pts[1].p, cur.p);
                    if r > 1e-9 {
                        for g in tools::slot_geometry(t.pts[0].p, t.pts[1].p, r) {
                            curve(&g.curve);
                        }
                    }
                    info = format!("R {} mm", geom::fmt_num(r));
                }
            }
            _ => {}
        }
        // auto-constraint hints next to the cursor (FreeCAD shows the constraint icons)
        let mut x = 14.0;
        if self.auto_constraints {
            if let Some(g) = tools::snap_glyph(&cur.snap) {
                badge(p, cs + egui::vec2(x + 8.0, -16.0), g, COL_DIM, None);
                x += 20.0;
            }
            if let Some(h) = hv {
                badge(p, cs + egui::vec2(x + 8.0, -16.0), if h { Glyph::H } else { Glyph::V }, COL_DIM, None);
            }
        }
        if matches!(cur.snap, Snap::Point(_)) {
            p.circle_stroke(pj.s(cur.p), 7.0, Stroke::new(1.5, COL_HOVER));
        } else if matches!(cur.snap, Snap::Curve(_)) {
            p.circle_filled(pj.s(cur.p), 3.0, COL_HOVER);
        }
        p.text(cs + egui::vec2(16.0, 14.0), Align2::LEFT_TOP, info, FontId::proportional(12.5), COL_CURSOR_TEXT);
    }
}

/// Icon position near `anchor`, avoiding icons already placed (FreeCAD stacks icons).
fn place(placed: &mut Vec<Rect>, anchor: Pos2, along: V, normal: V) -> Pos2 {
    let base = anchor + normal * 15.0;
    for k in 0..12 {
        let step = if k % 2 == 0 { (k / 2) as f32 } else { -((k / 2 + 1) as f32) };
        let c = base + along * step * 20.0;
        let r = Rect::from_center_size(c, egui::vec2(18.0, 18.0));
        if !placed.iter().any(|q| q.intersects(r)) {
            placed.push(r);
            return c;
        }
    }
    base
}

/// Draws a dimension; returns the label rect (for picking).
#[allow(clippy::too_many_arguments)]
fn draw_dim(p: &Painter, pj: &Proj, f: &DimFrame, off: Option<f64>, ppu: f64, center: Vec2, text: &str, col: C, bg: Option<C>, levels: &mut Vec<(Vec2, f64, f64, f64, f64)>, labels: &[Rect]) -> Rect {
    let st = Stroke::new(1.3, col);
    let gap = 26.0 / ppu;
    match f {
        DimFrame::Linear { p1, p2, n } => {
            let off_user = off;
            let (n1, n2) = (p1.dot(*n), p2.dot(*n));
            let t = n.perp();
            let (lo, hi) = (p1.dot(t).min(p2.dot(t)), p1.dot(t).max(p2.dot(t)));
            let g = p.layout_no_wrap(text.to_string(), FontId::proportional(DIM_FONT), col);
            let nsd = sv(pj.s(*n) - pj.s(Vec2::ZERO));
            let ext = ((nsd.x.abs() * g.size().x + nsd.y.abs() * g.size().y) as f64 + 8.0) / ppu;
            let span = |c: f64, out: f64| if out > 0.0 { (c, c + ext) } else { (c - ext, c) };
            let off = off.unwrap_or_else(|| {
                let mid = p1.lerp(*p2, 0.5);
                let out = mid.dot(*n) >= center.dot(*n) - 1e-9;
                let o = if out { 1.0 } else { -1.0 };
                let mut c = if out { n1.max(n2) + gap } else { n1.min(n2) - gap };
                // stack parallel dimensions (line + label) instead of overlapping them
                for _ in 0..20 {
                    let (ca, cb) = span(c, o);
                    let clash = levels.iter().any(|(m, la, lb, l0, l1)| {
                        let sgn = m.dot(*n);
                        let (la, lb) = if sgn > 0.0 { (*la, *lb) } else { (-*lb, -*la) };
                        sgn.abs() > 0.99 && ca < lb + gap * 0.2 && cb > la - gap * 0.2 && *l0 <= hi + gap && *l1 >= lo - gap
                    });
                    if !clash {
                        break;
                    }
                    c += o * gap;
                }
                c - n1
            });
            let mut c = n1 + off;
            let outward = if c >= (n1 + n2) * 0.5 { 1.0 } else { -1.0 };
            // label position for level `c`
            let layout = |c: f64| {
                let q1 = *p1 + *n * (c - n1);
                let q2 = *p2 + *n * (c - n2);
                let (sq1, sq2) = (pj.s(q1), pj.s(q2));
                let mid = sq1 + (sq2 - sq1) * 0.5;
                let mut ns = sv(pj.s(q1.lerp(q2, 0.5) + *n) - mid);
                // text on the outer side of the dimension line
                if c < (n1 + n2) * 0.5 {
                    ns = -ns;
                }
                let half = (ns.x.abs() * g.size().x + ns.y.abs() * g.size().y) * 0.5 + 3.0;
                Rect::from_center_size(mid + ns * half, g.size())
            };
            if off_user.is_none() {
                for _ in 0..8 {
                    let r = layout(c);
                    if !labels.iter().any(|l| l.intersects(r)) {
                        break;
                    }
                    c += outward * gap;
                }
            }
            let (la, lb) = span(c, outward);
            levels.push((*n, la, lb, lo, hi));
            let q1 = *p1 + *n * (c - n1);
            let q2 = *p2 + *n * (c - n2);
            let (s1, s2, sq1, sq2) = (pj.s(*p1), pj.s(*p2), pj.s(q1), pj.s(q2));
            for (a, b) in [(s1, sq1), (s2, sq2)] {
                let d = b - a;
                if d.length() > 4.0 {
                    let u = sv(d);
                    p.line_segment([a + u * 3.0, b + u * 5.0], Stroke::new(1.0, col));
                }
            }
            let along = sq2 - sq1;
            if along.length() > 1.0 {
                let u = sv(along);
                if along.length() > 24.0 {
                    p.line_segment([sq1, sq2], st);
                    arrow(p, sq1, -u, col);
                    arrow(p, sq2, u, col);
                } else {
                    p.line_segment([sq1 - u * 16.0, sq2 + u * 16.0], st);
                    arrow(p, sq1, u, col);
                    arrow(p, sq2, -u, col);
                }
            }
            let r = layout(c);
            label(p, r.center(), text, col, bg)
        }
        DimFrame::Direct { a, b } => {
            let (sa, sb) = (pj.s(*a), pj.s(*b));
            let d = sb - sa;
            if d.length() > 1.0 {
                p.line_segment([sa, sb], st);
                arrow(p, sa, -sv(d), col);
                arrow(p, sb, sv(d), col);
            }
            let n = V::new(-sv(d).y, sv(d).x);
            let n = if n.y > 0.0 { -n } else { n };
            let g = p.layout_no_wrap(text.to_string(), FontId::proportional(DIM_FONT), col);
            let half = (n.x.abs() * g.size().x + n.y.abs() * g.size().y) * 0.5 + 3.0;
            label(p, sa + d * 0.5 + n * half, text, col, bg)
        }
        DimFrame::Radial { c, r, default } => {
            let th = off.unwrap_or(*default);
            let u = Vec2::polar(th);
            let is_diam = text.contains('Ø');
            let tip = *c + u * *r;
            let (sc, st_) = (pj.s(*c), pj.s(tip));
            let us = sv(st_ - sc);
            let start = if is_diam { pj.s(*c - u * *r) } else { sc };
            p.line_segment([start, st_ + us * 18.0], st);
            arrow(p, st_, us, col);
            if is_diam {
                arrow(p, start, -us, col);
            }
            let g = p.layout_no_wrap(text.to_string(), FontId::proportional(DIM_FONT), col);
            let half = (us.x.abs() * g.size().x + us.y.abs() * g.size().y) * 0.5 + 2.0;
            label(p, st_ + us * (20.0 + half), text, col, bg)
        }
        DimFrame::Angular { x, base, sweep, default_r } => {
            let rr = off.unwrap_or(default_r.map(|r| r + 18.0 / ppu).unwrap_or(42.0 / ppu));
            let nseg = 32;
            let pts: Vec<Pos2> = (0..=nseg).map(|i| pj.s(*x + Vec2::polar(base + sweep * i as f64 / nseg as f64) * rr)).collect();
            // reference rays from the vertex
            let sx = pj.s(*x);
            for a in [*base, base + sweep] {
                let e = pj.s(*x + Vec2::polar(a) * rr * 1.15);
                p.extend(Shape::dashed_line(&[sx, e], Stroke::new(1.0, col), 4.0, 3.0));
            }
            if pts.len() >= 2 && (pts[0] - pts[nseg]).length() + (pts[0] - pts[nseg / 2]).length() > 2.0 {
                p.add(Shape::line(pts.clone(), st));
                arrow(p, pts[0], pts[0] - pts[1], col);
                arrow(p, pts[nseg], pts[nseg] - pts[nseg - 1], col);
            }
            let m = pj.s(*x + Vec2::polar(base + sweep * 0.5) * rr);
            let u = sv(m - sx);
            let g = p.layout_no_wrap(text.to_string(), FontId::proportional(DIM_FONT), col);
            let half = (u.x.abs() * g.size().x + u.y.abs() * g.size().y) * 0.5 + 3.0;
            label(p, m + u * half, text, col, bg)
        }
    }
}
