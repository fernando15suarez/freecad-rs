//! The assembly solver.
//!
//! Unknowns: for every non-grounded instance that takes part in a joint, an incremental
//! rotation vector `w` (left-multiplied, about the instance origin) and a translation
//! `dt` — 6 per instance. The system is always re-linearised at `w = dt = 0`, where the
//! first-order map `R(w) = (I + [w]x) R0` has the exact value and the exact derivative;
//! accepted steps are applied with the exact exponential map. Rotations are scaled by a
//! characteristic length `L` (columns hold `L·w`) so that "minimal movement" weighs a
//! rotation like the displacement it causes at the size of the parts.
//!
//! Residuals come from [`crate::joint`] with exact gradients (forward AD, [`crate::dual`]).
//! Each connected island of free instances is solved on its own with Levenberg–Marquardt
//! (minimum-norm Gauss–Newton steps `δ = -Jᵀ(JJᵀ + λI)⁻¹ r`, or the equivalent normal
//! equations when there are fewer unknowns than equations), so underconstrained parts
//! move as little as possible from where they are — the same "nearest solution" behaviour
//! FreeCAD's Ondsel solver and Onshape show.
//!
//! Diagnosis (like fc-sketch's): at the end, a pivoted Cholesky factorisation of `JJᵀ`
//! picks a maximal independent set of equations. `rank` gives the remaining degrees of
//! freedom (`6·n − rank`); every equation outside the set is a linear combination of
//! others: if the solve converged it is *redundant*, otherwise the joints in its
//! dependency group are reported as *conflicting*.

use crate::dual::{self, dv_c, D, DV};
use crate::frame::{Frame, M3};
use crate::geom::{infer_frame, ElementGeom, ImplicitPoint, RefKind};
use crate::joint::{distance_mode, is_zero_angle, DistanceMode, Joint, JointKind, JointRef, Sense};
use crate::model::{Assembly, InstanceId, InstanceOf, JointId, PartId, PartResolver};
use fc_part::placement::{dot, norm, sub};
use fc_part::V3;
use nalgebra::{DMatrix, DVector};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet, VecDeque};
use std::f64::consts::PI;

#[derive(Clone, Debug)]
pub struct SolveOptions {
    pub max_iterations: usize,
    /// Convergence: max |residual| (mm for distances, radians for angles).
    pub tolerance: f64,
    /// Run the rank analysis (DOF, redundant/conflicting joints).
    pub diagnose: bool,
}

impl Default for SolveOptions {
    fn default() -> Self {
        SolveOptions { max_iterations: 200, tolerance: 1e-10, diagnose: true }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SolveStatus {
    /// All joints satisfied, no redundant equations.
    Solved,
    /// All joints satisfied, but some equations are implied by others (FreeCAD warns).
    Redundant,
    /// Not solvable: some joints contradict each other (dependent and unsatisfied).
    Conflicting,
    /// Not solvable (e.g. impossible geometry) or did not converge.
    Failed,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SolveReport {
    pub status: SolveStatus,
    /// Remaining degrees of freedom of the non-grounded instances (6·n − rank).
    pub dof: usize,
    pub free_instances: usize,
    pub equations: usize,
    pub rank: usize,
    pub iterations: usize,
    /// Max |residual| at the end.
    pub residual: f64,
    pub redundant_joints: Vec<JointId>,
    pub conflicting_joints: Vec<JointId>,
    /// Joints still violated at the end.
    pub unsatisfied_joints: Vec<JointId>,
    /// Joints whose references could not be resolved (skipped), with the reason.
    pub unresolved_joints: Vec<(JointId, String)>,
    /// Joints with an active limit.
    pub limited_joints: Vec<JointId>,
    /// No instance is grounded (everything floats; FreeCAD refuses to solve this).
    pub no_ground: bool,
    pub moved: Vec<InstanceId>,
}

impl SolveReport {
    pub fn ok(&self) -> bool {
        matches!(self.status, SolveStatus::Solved | SolveStatus::Redundant)
    }
}

#[derive(Debug, thiserror::Error, PartialEq)]
pub enum SolveError {
    #[error("no such instance {0}")]
    NoInstance(InstanceId),
    #[error("no such joint {0}")]
    NoJoint(JointId),
    #[error("{0}")]
    Unresolved(String),
}

/// What a drag aims at.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DragTarget {
    /// Move the instance so that its placement becomes this frame (as far as joints allow).
    Frame(Frame),
    /// Move the instance so that its local point `local` lands on `world` (screen-ray
    /// drag: the grabbed point follows the cursor's point on the drag plane).
    Point { local: V3, world: V3 },
}

// ---------------------------------------------------------------------------------------
// Reference resolution

/// A resolved joint side: instance index and the JCS in instance-local coordinates.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Side {
    pub inst: usize,
    pub local: Frame,
    pub kind: RefKind,
}

/// Follows a sub-element path into sub-assemblies: for an instance of an assembly,
/// `element` is `"<instance name>.<element>"` (recursively, like FreeCAD's
/// `SubAsm.Part.Face3`). Returns the leaf part, the leaf's frame in the outer instance's
/// coordinates and the leaf element name.
pub(crate) fn descend<'e>(res: &dyn PartResolver, of: InstanceOf, element: &'e str) -> Result<(PartId, Frame, &'e str), String> {
    let mut of = of;
    let mut el = element;
    let mut acc = Frame::IDENTITY;
    for _ in 0..64 {
        match of {
            InstanceOf::Part(p) => return Ok((p, acc, el)),
            InstanceOf::Assembly(a) => {
                let sub = res.sub_assembly(a).ok_or_else(|| format!("sub-assembly {a} not available"))?;
                let (name, rest) = el.split_once('.').ok_or_else(|| format!("{el}: references into a sub-assembly need an instance path (\"Part.Face1\")"))?;
                let inst = sub.instance_by_name(name).ok_or_else(|| format!("no instance {name} in {}", sub.name))?;
                acc = acc.mul(&inst.placement);
                of = inst.of;
                el = rest;
            }
        }
    }
    Err("sub-assemblies nest too deeply (cycle?)".to_string())
}

pub(crate) fn resolve_ref(asm: &Assembly, res: &dyn PartResolver, r: &JointRef, ignore_vertex: bool) -> Result<Side, String> {
    let idx = |id: InstanceId| asm.instances.iter().position(|i| i.id == id).ok_or_else(|| format!("no instance {id}"));
    match r {
        JointRef::Frame { instance, local } => Ok(Side { inst: idx(*instance)?, local: *local, kind: RefKind::Frame }),
        JointRef::Origin { instance } => Ok(Side { inst: idx(*instance)?, local: Frame::IDENTITY, kind: RefKind::Frame }),
        JointRef::AssemblyConnector(c) => {
            let c = asm.connectors.iter().find(|x| x.id == *c).ok_or_else(|| format!("no assembly connector {c}"))?;
            Ok(Side { inst: idx(c.instance)?, local: c.local, kind: RefKind::Frame })
        }
        JointRef::PartConnector { instance, connector } => {
            let i = idx(*instance)?;
            let p = match asm.instances[i].of {
                InstanceOf::Part(p) => p,
                InstanceOf::Assembly(_) => return Err("mate connectors of parts inside a sub-assembly: use an assembly connector or a geometry path".to_string()),
            };
            let f = res.connector_frame(p, *connector).ok_or_else(|| format!("mate connector {connector} not found"))?;
            Ok(Side { inst: i, local: f, kind: RefKind::Frame })
        }
        JointRef::Geometry { instance, element, implicit } => {
            let i = idx(*instance)?;
            let (p, acc, element) = descend(res, asm.instances[i].of, element)?;
            // The second pick may carry the same path prefix.
            let second = |n: &str| -> Result<ElementGeom, String> {
                let leaf = match descend(res, asm.instances[i].of, n) {
                    Ok((q, f, e)) if q == p && f.is_same(&acc, 1e-12) => e,
                    _ => n.rsplit('.').next().unwrap_or(n),
                };
                res.element(p, leaf).ok_or_else(|| format!("element {n} not found"))
            };
            if let ImplicitPoint::Infer(mode) = implicit {
                let f = res.infer_frame(p, element, *mode).ok_or_else(|| format!("cannot infer a frame on {element}"))?;
                let kind = res.element(p, element).map(|g| g.kind()).unwrap_or(RefKind::Frame);
                return Ok(Side { inst: i, local: acc.mul(&f), kind });
            }
            let g = res.element(p, element).ok_or_else(|| format!("element {element} not found"))?;
            let pg = match implicit {
                ImplicitPoint::Center | ImplicitPoint::Infer(_) => None,
                ImplicitPoint::Vertex(n) | ImplicitPoint::EdgeCenter(n) => Some(second(n)?),
            };
            Ok(Side { inst: i, local: acc.mul(&infer_frame(&g, pg.as_ref(), ignore_vertex)), kind: g.kind() })
        }
    }
}

// ---------------------------------------------------------------------------------------
// Equations

#[derive(Clone, Copy, Debug)]
enum Eq {
    /// rIJ = 0 (3)
    AtPoint,
    /// rIJ·xI = rIJ·yI = 0 (2)
    InLine,
    /// rIJ·yI = rIJ·zI = 0 (2) — pin slot along xI
    InLineX,
    /// rIJ·zI = offset (1)
    InPlane(f64),
    /// zI ∥ zJ as the rotation vector between the axes (2)
    ZAlign,
    /// R_I = R_J as the rotation vector of R_Iᵀ R_J (3)
    Orient,
    /// zI·zJ = c (1)
    DirCos(f64),
    /// |rIJ| = d (1)
    DistPP(f64),
    /// distance of J's origin from I's Z axis = d (1)
    DistXY(f64),
    /// rIJ·xI + r·θz = C (1)
    RackPin { r: f64, c: f64, th0: f64 },
    /// 2π·rIJ·zI − p·θz = C (1)
    Screw { p: f64, c: f64, th0: f64 },
    /// orbitJI + ratio·orbitIJ = C (1)
    Gear { ratio: f64, c: f64, o1: f64, o2: f64 },
    /// rIJ·zI = bound (active limit)
    LenBound(f64),
    /// θz = bound (active limit, radians)
    AngBound { bound: f64, th0: f64 },
}


/// World JCS of a side, as duals over the local 12 slots.
struct WM {
    p: DV,
    x: DV,
    y: DV,
    z: DV,
}

fn wrap_near(a: f64, reference: f64) -> f64 {
    a + ((reference - a) / (2.0 * PI)).round() * 2.0 * PI
}

fn theta_z(i: &WM, j: &WM) -> D {
    dual::dot(&i.y, &j.x).atan2(dual::dot(&i.x, &j.x))
}
fn orbit(i: &WM, j: &WM) -> D {
    let r = dual::vsub(&j.p, &i.p);
    dual::dot(&r, &i.y).atan2(dual::dot(&r, &i.x))
}

/// θ/sin θ factor for the log map, from s = sin θ (≥ 0) and c = cos θ.
fn log_factor(s: D, c: D) -> D {
    if s.v > 1e-7 {
        let th = s.atan2(c);
        // th / s
        let inv = 1.0 / s.v;
        let mut g = [0.0; dual::NL];
        for k in 0..dual::NL {
            g[k] = th.g[k] * inv - th.v * s.g[k] * inv * inv;
        }
        D { v: th.v * inv, g }
    } else if c.v > 0.0 {
        D::c(1.0)
    } else {
        D::c(PI / 1e-7)
    }
}

fn eval_eq(eq: &Eq, i: &WM, j: &WM, out: &mut Vec<D>) {
    let r = dual::vsub(&j.p, &i.p);
    match *eq {
        Eq::AtPoint => out.extend_from_slice(&r),
        Eq::InLine => {
            out.push(dual::dot(&r, &i.x));
            out.push(dual::dot(&r, &i.y));
        }
        Eq::InLineX => {
            out.push(dual::dot(&r, &i.y));
            out.push(dual::dot(&r, &i.z));
        }
        Eq::InPlane(off) => out.push(dual::dot(&r, &i.z).add_c(-off)),
        Eq::ZAlign => {
            let s1 = dual::dot(&i.x, &j.z);
            let s2 = dual::dot(&i.y, &j.z);
            let c = dual::dot(&i.z, &j.z);
            let f = log_factor((s1 * s1 + s2 * s2).sqrt(), c);
            out.push(s1 * f);
            out.push(s2 * f);
        }
        Eq::Orient => {
            // R = R_Iᵀ R_J, R[a][b] = aI · bJ
            let ai = [&i.x, &i.y, &i.z];
            let bj = [&j.x, &j.y, &j.z];
            let m = |a: usize, b: usize| dual::dot(ai[a], bj[b]);
            let w = [(m(2, 1) - m(1, 2)).scale(0.5), (m(0, 2) - m(2, 0)).scale(0.5), (m(1, 0) - m(0, 1)).scale(0.5)];
            let c = (m(0, 0) + m(1, 1) + m(2, 2)).add_c(-1.0).scale(0.5);
            let f = log_factor(dual::norm(&w), c);
            for k in 0..3 {
                out.push(w[k] * f);
            }
        }
        Eq::DirCos(c) => out.push(dual::dot(&i.z, &j.z).add_c(-c)),
        Eq::DistPP(d) => out.push(dual::norm(&r).add_c(-d)),
        Eq::DistXY(d) => {
            let a = dual::dot(&r, &i.x);
            let b = dual::dot(&r, &i.y);
            out.push((a * a + b * b).sqrt().add_c(-d))
        }
        Eq::RackPin { r: pr, c, th0 } => {
            let th = theta_z(i, j);
            let th = th.add_c(wrap_near(th.v, th0) - th.v);
            out.push((dual::dot(&r, &i.x) + th.scale(pr)).add_c(-c))
        }
        Eq::Screw { p, c, th0 } => {
            let th = theta_z(i, j);
            let th = th.add_c(wrap_near(th.v, th0) - th.v);
            out.push((dual::dot(&r, &i.z).scale(2.0 * PI) - th.scale(p)).add_c(-c))
        }
        Eq::Gear { ratio, c, o1, o2 } => {
            let a = orbit(j, i);
            let a = a.add_c(wrap_near(a.v, o1) - a.v);
            let b = orbit(i, j);
            let b = b.add_c(wrap_near(b.v, o2) - b.v);
            out.push((a + b.scale(ratio)).add_c(-c))
        }
        Eq::LenBound(b) => out.push(dual::dot(&r, &i.z).add_c(-b)),
        Eq::AngBound { bound, th0 } => {
            let th = theta_z(i, j);
            let th = th.add_c(wrap_near(th.v, th0) - th.v);
            out.push(th.add_c(-bound))
        }
    }
}

/// A joint prepared for solving: resolved sides (sense and offsets folded into `b`) and
/// its equations.
#[derive(Clone, Debug)]
struct Plan {
    id: JointId,
    a: Side,
    b: Side,
    eqs: Vec<Eq>,
    /// Limits not yet active: (is_length, bound, is_min).
    limits: Vec<(bool, f64, bool)>,
}

struct Ctx<'a> {
    /// Current instance placements (indexed like `asm.instances`).
    pl: &'a [Frame],
}

impl Ctx<'_> {
    fn world(&self, s: &Side) -> Frame {
        self.pl[s.inst].mul(&s.local)
    }
}

fn marker_wm(pl: &Frame, local: &Frame, slot: Option<usize>, inv_l: f64) -> WM {
    let r0: M3 = pl.matrix();
    let lm = local.matrix();
    let rot = |v: V3| crate::frame::mat_vec(&r0, v);
    let prel = rot(local.pos);
    let p0 = fc_part::placement::add(prel, pl.pos);
    let axes = [rot(lm[0]), rot(lm[1]), rot(lm[2])];
    match slot {
        None => WM { p: dv_c(p0), x: dv_c(axes[0]), y: dv_c(axes[1]), z: dv_c(axes[2]) },
        Some(s) => {
            let w: DV = [D::var(0.0, s).scale(inv_l), D::var(0.0, s + 1).scale(inv_l), D::var(0.0, s + 2).scale(inv_l)];
            let dt: DV = [D::var(0.0, s + 3), D::var(0.0, s + 4), D::var(0.0, s + 5)];
            let rotd = |a: V3| {
                let a = dv_c(a);
                dual::vadd(&a, &dual::cross(&w, &a))
            };
            let pr = dv_c(prel);
            let p = dual::vadd(&dual::vadd(&dv_c(p0), &dual::cross(&w, &pr)), &dt);
            WM { p, x: rotd(axes[0]), y: rotd(axes[1]), z: rotd(axes[2]) }
        }
    }
}

fn axis_only_flip(i: &Frame, j: &Frame) -> bool {
    dot(i.z_axis(), j.z_axis()) < 0.0
}

/// Half-turn about X/Y/Z or identity: which one brings `j` closest to `i` (Ondsel's
/// four roots of the direction-cosine form of a fixed orientation).
fn nearest_half_turn(i: &Frame, j: &Frame) -> Option<V3> {
    let mi = i.matrix();
    let mj = j.matrix();
    let m = |a: usize, b: usize| dot(mi[a], mj[b]);
    let cands = [
        (m(0, 0) + m(1, 1) + m(2, 2), None),
        (m(0, 0) - m(1, 1) - m(2, 2), Some([1.0, 0.0, 0.0])),
        (-m(0, 0) + m(1, 1) - m(2, 2), Some([0.0, 1.0, 0.0])),
        (-m(0, 0) - m(1, 1) + m(2, 2), Some([0.0, 0.0, 1.0])),
    ];
    let mut best = cands[0];
    for c in cands.iter().skip(1) {
        if c.0 > best.0 + 1e-12 {
            best = *c;
        }
    }
    best.1
}

fn build_plan(asm: &Assembly, res: &dyn PartResolver, ji: usize, pl: &[Frame]) -> Result<Plan, String> {
    let j: &Joint = &asm.joints[ji];
    let ignore_vertex = matches!(j.kind, JointKind::Distance { .. });
    let mut a = resolve_ref(asm, res, &j.a, ignore_vertex)?;
    let mut b = resolve_ref(asm, res, &j.b, ignore_vertex)?;
    a.local = a.local.mul(&j.offset_a);
    b.local = b.local.mul(&j.offset_b);
    if a.inst == b.inst {
        return Err("both sides are on the same instance".into());
    }
    let ctx = Ctx { pl };
    let mut eqs = vec![];
    // Orientation class: 0 none, 1 axis only, 2 full
    let mut orient = 0;
    match j.kind {
        JointKind::Fastened => {
            eqs.extend([Eq::AtPoint, Eq::Orient]);
            orient = 2;
        }
        JointKind::Revolute => {
            eqs.extend([Eq::AtPoint, Eq::ZAlign]);
            orient = 1;
        }
        JointKind::Cylindrical => {
            eqs.extend([Eq::InLine, Eq::ZAlign]);
            orient = 1;
        }
        JointKind::Slider => {
            eqs.extend([Eq::InLine, Eq::Orient]);
            orient = 2;
        }
        JointKind::PinSlot => {
            eqs.extend([Eq::InLineX, Eq::ZAlign]);
            orient = 1;
        }
        JointKind::Planar => {
            eqs.extend([Eq::InPlane(0.0), Eq::ZAlign]);
            orient = 1;
        }
        JointKind::Ball => eqs.push(Eq::AtPoint),
        JointKind::Parallel => {
            eqs.push(Eq::ZAlign);
            orient = 1;
        }
        JointKind::Perpendicular => eqs.push(Eq::DirCos(0.0)),
        JointKind::Angle { angle } => {
            if is_zero_angle(angle) {
                eqs.push(Eq::ZAlign);
                orient = 1;
            } else {
                eqs.push(Eq::DirCos(angle.to_radians().cos()));
            }
        }
        JointKind::Distance { distance, mode } => {
            let (mode, swap, extra) = distance_mode(mode, a.kind, b.kind);
            if swap {
                std::mem::swap(&mut a, &mut b);
            }
            let d = distance + extra;
            match mode {
                DistanceMode::PointPoint | DistanceMode::Auto => {
                    if d.abs() < 1e-7 {
                        eqs.push(Eq::AtPoint)
                    } else {
                        eqs.push(Eq::DistPP(d))
                    }
                }
                DistanceMode::PointPlane => eqs.push(Eq::InPlane(d)),
                DistanceMode::PlanePlane => {
                    eqs.extend([Eq::InPlane(d), Eq::ZAlign]);
                    orient = 1;
                }
                DistanceMode::PointLine => {
                    if d.abs() < 1e-7 {
                        eqs.push(Eq::InLine)
                    } else {
                        eqs.push(Eq::DistXY(d))
                    }
                }
                DistanceMode::LinePlane => eqs.extend([Eq::InPlane(d), Eq::DirCos(0.0)]),
                DistanceMode::LineLine => {
                    if d.abs() < 1e-7 {
                        eqs.extend([Eq::InLine, Eq::ZAlign])
                    } else {
                        eqs.extend([Eq::DistXY(d), Eq::ZAlign])
                    }
                    orient = 1;
                }
            }
        }
        _ => {}
    }
    // Orientation sense: fold the chosen half turn into J's local frame.
    let (wi, wj) = (ctx.world(&a), ctx.world(&b));
    match (orient, j.sense) {
        (0, _) | (_, Sense::Aligned) => {}
        (_, Sense::Flipped) => b.local = b.local.flipped(),
        (1, Sense::Auto) => {
            if axis_only_flip(&wi, &wj) {
                b.local = b.local.flipped();
            }
        }
        (_, Sense::Auto) => {
            if let Some(ax) = nearest_half_turn(&wi, &wj) {
                b.local = b.local.mul(&Frame::rotation(ax, PI));
            }
        }
    }
    // Coupling joints: constants from the current placements (Ondsel's aConstant).
    let wi = ctx.world(&a);
    let wj = ctx.world(&b);
    let mi = WM { p: dv_c(wi.pos), x: dv_c(wi.x_axis()), y: dv_c(wi.y_axis()), z: dv_c(wi.z_axis()) };
    let mj = WM { p: dv_c(wj.pos), x: dv_c(wj.x_axis()), y: dv_c(wj.y_axis()), z: dv_c(wj.z_axis()) };
    let th0 = theta_z(&mi, &mj).v;
    let rz = dot(sub(wj.pos, wi.pos), wi.z_axis());
    let rx = dot(sub(wj.pos, wi.pos), wi.x_axis());
    match j.kind {
        JointKind::RackPinion { pitch_radius } => {
            eqs.push(Eq::RackPin { r: pitch_radius, c: rx + pitch_radius * th0, th0 })
        }
        JointKind::Screw { pitch } => eqs.push(Eq::Screw { p: pitch, c: 2.0 * PI * rz - pitch * th0, th0 }),
        JointKind::Gears { radius1, radius2 } | JointKind::Belt { radius1, radius2 } => {
            let r2 = if matches!(j.kind, JointKind::Belt { .. }) { -radius2 } else { radius2 };
            let ratio = if r2.abs() > 1e-12 { radius1 / r2 } else { 0.0 };
            let o1 = orbit(&mj, &mi).v;
            let o2 = orbit(&mi, &mj).v;
            eqs.push(Eq::Gear { ratio, c: o1 + ratio * o2, o1, o2 });
        }
        _ => {}
    }
    let mut limits = vec![];
    let l = &j.limits;
    if let Some(v) = l.length_min {
        limits.push((true, v, true));
    }
    if let Some(v) = l.length_max {
        limits.push((true, v, false));
    }
    if let Some(v) = l.angle_min {
        limits.push((false, v.to_radians(), true));
    }
    if let Some(v) = l.angle_max {
        limits.push((false, v.to_radians(), false));
    }
    let _ = th0;
    Ok(Plan { id: j.id, a, b, eqs, limits })
}

// ---------------------------------------------------------------------------------------
// The numeric system of one island

struct Island {
    /// Instance indices of the free instances (column blocks).
    insts: Vec<usize>,
    /// Plan indices.
    plans: Vec<usize>,
}

struct Eval {
    r: DVector<f64>,
    j: DMatrix<f64>,
    /// row -> plan index
    row_plan: Vec<usize>,
}

fn eval_island(isl: &Island, plans: &[Plan], col_of: &HashMap<usize, usize>, pl: &[Frame], inv_l: f64, want_j: bool) -> Eval {
    let mut rows: Vec<D> = vec![];
    let mut row_plan = vec![];
    let mut blocks: Vec<(Option<usize>, Option<usize>, usize, usize)> = vec![];
    for &pi in &isl.plans {
        let p = &plans[pi];
        let ca = col_of.get(&p.a.inst).copied();
        let cb = col_of.get(&p.b.inst).copied();
        let wi = marker_wm(&pl[p.a.inst], &p.a.local, ca.map(|_| 0), inv_l);
        let wj = marker_wm(&pl[p.b.inst], &p.b.local, cb.map(|_| 6), inv_l);
        let start = rows.len();
        for e in &p.eqs {
            eval_eq(e, &wi, &wj, &mut rows);
        }
        for _ in start..rows.len() {
            row_plan.push(pi);
        }
        blocks.push((ca, cb, start, rows.len()));
    }
    let m = rows.len();
    let n = isl.insts.len() * 6;
    let r = DVector::from_iterator(m, rows.iter().map(|d| d.v));
    let mut j = DMatrix::zeros(if want_j { m } else { 0 }, n);
    if want_j {
        for (ca, cb, s, e) in blocks {
            for row in s..e {
                let g = &rows[row].g;
                if let Some(c) = ca {
                    for k in 0..6 {
                        j[(row, c * 6 + k)] += g[k];
                    }
                }
                if let Some(c) = cb {
                    for k in 0..6 {
                        j[(row, c * 6 + k)] += g[6 + k];
                    }
                }
            }
        }
    }
    Eval { r, j, row_plan }
}

fn max_abs(v: &DVector<f64>) -> f64 {
    v.iter().fold(0.0f64, |m, x| m.max(x.abs()))
}

/// Levenberg–Marquardt minimum-norm step.
fn lm_step(j: &DMatrix<f64>, r: &DVector<f64>, lambda: f64) -> Option<DVector<f64>> {
    let (m, n) = j.shape();
    if m <= n {
        let mut a = j * j.transpose();
        let sc = (0..m).fold(0.0f64, |s, i| s.max(a[(i, i)])).max(1e-300);
        for i in 0..m {
            a[(i, i)] += lambda * sc + 1e-14 * sc;
        }
        let ch = a.cholesky()?;
        let y = ch.solve(r);
        Some(-(j.transpose() * y))
    } else {
        let mut a = j.transpose() * j;
        let sc = (0..n).fold(0.0f64, |s, i| s.max(a[(i, i)])).max(1e-300);
        for i in 0..n {
            a[(i, i)] += lambda * sc + 1e-14 * sc;
        }
        let ch = a.cholesky()?;
        Some(-ch.solve(&(j.transpose() * r)))
    }
}

fn apply_step(pl: &mut [Frame], isl: &Island, d: &DVector<f64>, inv_l: f64) {
    for (k, &ii) in isl.insts.iter().enumerate() {
        let w = [d[6 * k] * inv_l, d[6 * k + 1] * inv_l, d[6 * k + 2] * inv_l];
        let t = [d[6 * k + 3], d[6 * k + 4], d[6 * k + 5]];
        pl[ii] = pl[ii].perturbed(w, t);
    }
}

/// LM on one island; returns (converged, iterations).
fn solve_island(isl: &Island, plans: &[Plan], col_of: &HashMap<usize, usize>, pl: &mut [Frame], inv_l: f64, opts: &SolveOptions) -> (bool, usize) {
    let mut ev = eval_island(isl, plans, col_of, pl, inv_l, true);
    let mut cost = ev.r.norm_squared();
    let mut lambda = 1e-12;
    let mut it = 0;
    while it < opts.max_iterations {
        if max_abs(&ev.r) <= opts.tolerance {
            return (true, it);
        }
        it += 1;
        let Some(d) = lm_step(&ev.j, &ev.r, lambda) else {
            lambda = (lambda * 10.0).max(1e-10);
            if lambda > 1e6 {
                break;
            }
            continue;
        };
        let mut trial = pl.to_vec();
        apply_step(&mut trial, isl, &d, inv_l);
        let ev2 = eval_island(isl, plans, col_of, &trial, inv_l, false);
        let c2 = ev2.r.norm_squared();
        if c2 < cost || c2 == 0.0 {
            pl.copy_from_slice(&trial);
            ev = eval_island(isl, plans, col_of, pl, inv_l, true);
            let small_gain = cost - c2 < 1e-30 * cost.max(1.0);
            cost = c2;
            lambda = (lambda / 10.0).max(1e-15);
            if small_gain && d.norm() < 1e-14 {
                break;
            }
        } else {
            lambda *= 10.0;
            if lambda > 1e8 {
                break;
            }
        }
    }
    (max_abs(&ev.r) <= opts.tolerance, it)
}

/// Rank analysis of J's rows by pivoted Cholesky of JJᵀ: (rank, dependent rows with
/// their dependency groups).
fn row_dependencies(j: &DMatrix<f64>) -> (usize, Vec<(usize, Vec<usize>)>) {
    let m = j.nrows();
    if m == 0 {
        return (0, vec![]);
    }
    let g = j * j.transpose();
    let mut a = g.clone();
    let mut perm: Vec<usize> = (0..m).collect();
    let maxdiag = (0..m).fold(0.0f64, |s, i| s.max(a[(i, i)]));
    let tol = 1e-11 * maxdiag.max(1e-300);
    let mut rank = 0;
    // in-place pivoted Cholesky (lower), a[(i,k)] for i>=k holds L
    for k in 0..m {
        let (mut best, mut bi) = (-1.0, k);
        for i in k..m {
            if a[(i, i)] > best {
                best = a[(i, i)];
                bi = i;
            }
        }
        if best <= tol {
            break;
        }
        if bi != k {
            a.swap_rows(k, bi);
            a.swap_columns(k, bi);
            perm.swap(k, bi);
        }
        let d = a[(k, k)].sqrt();
        a[(k, k)] = d;
        for i in (k + 1)..m {
            a[(i, k)] /= d;
        }
        // Symmetric update of the trailing block (rows/columns are swapped later).
        for c in (k + 1)..m {
            let lck = a[(c, k)];
            if lck == 0.0 {
                continue;
            }
            for i in (k + 1)..m {
                let v = a[(i, k)] * lck;
                a[(i, c)] -= v;
            }
        }
        rank += 1;
    }
    let mut deps = vec![];
    if rank < m {
        // Coefficients of dependent row q on the independent rows: (J_I J_Iᵀ) x = J_I J_qᵀ.
        let l = a.view((0, 0), (rank, rank)).lower_triangle();
        for q in rank..m {
            let rhs = DVector::from_iterator(rank, (0..rank).map(|k| g[(perm[k], perm[q])]));
            let y = l.solve_lower_triangular(&rhs).unwrap_or_else(|| DVector::zeros(rank));
            let x = l.transpose().solve_upper_triangular(&y).unwrap_or_else(|| DVector::zeros(rank));
            let xmax = x.iter().fold(0.0f64, |s, v| s.max(v.abs()));
            let mut grp: Vec<usize> = (0..rank).filter(|&k| x[k].abs() > 1e-8 * xmax.max(1.0)).map(|k| perm[k]).collect();
            grp.push(perm[q]);
            deps.push((perm[q], grp));
        }
    }
    (rank, deps)
}

// ---------------------------------------------------------------------------------------
// Public entry points

pub(crate) struct Prepared {
    plans: Vec<Plan>,
    unresolved: Vec<(JointId, String)>,
    inv_l: f64,
}

fn prepare(asm: &Assembly, res: &dyn PartResolver, pl: &[Frame]) -> Prepared {
    let mut plans = vec![];
    let mut unresolved = vec![];
    for (ji, j) in asm.joints.iter().enumerate() {
        if j.suppressed {
            continue;
        }
        match build_plan(asm, res, ji, pl) {
            Ok(p) => plans.push(p),
            Err(e) => unresolved.push((j.id, e)),
        }
    }
    // Characteristic length: lever arms of the joint frames about their instance origins.
    let mut l: f64 = 1.0;
    for p in &plans {
        l = l.max(norm(p.a.local.pos)).max(norm(p.b.local.pos));
    }
    Prepared { plans, unresolved, inv_l: 1.0 / l }
}

fn islands(asm: &Assembly, plans: &[Plan], only: Option<usize>) -> Vec<Island> {
    let n = asm.instances.len();
    let free = |i: usize| !asm.instances[i].grounded;
    let mut adj: Vec<Vec<usize>> = vec![vec![]; n];
    for (pi, p) in plans.iter().enumerate() {
        let _ = pi;
        if free(p.a.inst) && free(p.b.inst) {
            adj[p.a.inst].push(p.b.inst);
            adj[p.b.inst].push(p.a.inst);
        }
    }
    let mut comp = vec![usize::MAX; n];
    let mut out: Vec<Island> = vec![];
    let starts: Vec<usize> = match only {
        Some(i) => vec![i],
        None => (0..n).collect(),
    };
    for s in starts {
        if !free(s) || comp[s] != usize::MAX {
            continue;
        }
        let id = out.len();
        let mut q = VecDeque::from([s]);
        comp[s] = id;
        let mut insts = vec![];
        while let Some(u) = q.pop_front() {
            insts.push(u);
            for &v in &adj[u] {
                if comp[v] == usize::MAX {
                    comp[v] = id;
                    q.push_back(v);
                }
            }
        }
        insts.sort_unstable();
        out.push(Island { insts, plans: vec![] });
    }
    for (pi, p) in plans.iter().enumerate() {
        let c = if free(p.a.inst) { comp[p.a.inst] } else if free(p.b.inst) { comp[p.b.inst] } else { usize::MAX };
        if c != usize::MAX && c < out.len() {
            out[c].plans.push(pi);
        }
    }
    out
}

fn col_map(isl: &Island) -> HashMap<usize, usize> {
    isl.insts.iter().enumerate().map(|(k, &i)| (i, k)).collect()
}

/// Solve without touching the assembly: the report and the new placements (one per
/// instance, in `asm.instances` order).
pub fn compute_solve(asm: &Assembly, res: &dyn PartResolver, opts: &SolveOptions) -> (SolveReport, Vec<Frame>) {
    let start: Vec<Frame> = asm.instances.iter().map(|i| i.placement).collect();
    let mut pl = start.clone();
    let prep = prepare(asm, res, &pl);
    let mut plans = prep.plans;
    let inv_l = prep.inv_l;
    let isls = islands(asm, &plans, None);
    let nfree = asm.instances.iter().filter(|i| !i.grounded).count();
    let mut rep = SolveReport {
        status: SolveStatus::Solved,
        dof: 0,
        free_instances: nfree,
        equations: 0,
        rank: 0,
        iterations: 0,
        residual: 0.0,
        redundant_joints: vec![],
        conflicting_joints: vec![],
        unsatisfied_joints: vec![],
        unresolved_joints: prep.unresolved,
        limited_joints: vec![],
        no_ground: !asm.instances.iter().any(|i| i.grounded),
        moved: vec![],
    };
    let mut all_ok = true;
    let mut redundant: HashSet<JointId> = HashSet::new();
    let mut conflicting: HashSet<JointId> = HashSet::new();
    let mut unsat: HashSet<JointId> = HashSet::new();
    let mut limited: HashSet<JointId> = HashSet::new();
    let mut total_rank = 0;
    for mut isl in isls {
        let cols = col_map(&isl);
        let (mut ok, mut its) = solve_island(&isl, &plans, &cols, &mut pl, inv_l, opts);
        // Active-set limits: turn violated bounds into equalities and solve again.
        for _round in 0..8 {
            let mut changed = false;
            for &pi in &isl.plans.clone() {
                let p = &plans[pi];
                if p.limits.is_empty() {
                    continue;
                }
                let wi = Ctx { pl: &pl }.world(&p.a);
                let wj = Ctx { pl: &pl }.world(&p.b);
                let mi = WM { p: dv_c(wi.pos), x: dv_c(wi.x_axis()), y: dv_c(wi.y_axis()), z: dv_c(wi.z_axis()) };
                let mj = WM { p: dv_c(wj.pos), x: dv_c(wj.x_axis()), y: dv_c(wj.y_axis()), z: dv_c(wj.z_axis()) };
                let len = dot(sub(wj.pos, wi.pos), wi.z_axis());
                // Angles are compared on the branch nearest to the bound.
                let raw = theta_z(&mi, &mj).v;
                let mut keep = vec![];
                let mut add = vec![];
                for &(is_len, bound, is_min) in &p.limits {
                    let v = if is_len { len } else { wrap_near(raw, bound) };
                    let violated = if is_min { v < bound - 1e-9 } else { v > bound + 1e-9 };
                    if violated {
                        add.push(if is_len { Eq::LenBound(bound) } else { Eq::AngBound { bound, th0: bound } });
                    } else {
                        keep.push((is_len, bound, is_min));
                    }
                }
                if !add.is_empty() {
                    let pm = &mut plans[pi];
                    pm.eqs.extend(add);
                    pm.limits = keep;
                    limited.insert(pm.id);
                    changed = true;
                }
            }
            if !changed {
                break;
            }
            isl.plans.sort_unstable();
            let (o, i2) = solve_island(&isl, &plans, &cols, &mut pl, inv_l, opts);
            ok = o;
            its += i2;
        }
        rep.iterations += its;
        let ev = eval_island(&isl, &plans, &cols, &pl, inv_l, opts.diagnose);
        rep.equations += ev.r.len();
        rep.residual = rep.residual.max(max_abs(&ev.r));
        for (row, &pi) in ev.row_plan.iter().enumerate() {
            if ev.r[row].abs() > opts.tolerance.max(1e-9) * 10.0 {
                unsat.insert(plans[pi].id);
            }
        }
        if opts.diagnose {
            let (rank, deps) = row_dependencies(&ev.j);
            total_rank += rank;
            for (row, grp) in deps {
                let bad = grp.iter().any(|&r| ev.r[r].abs() > opts.tolerance.max(1e-9) * 10.0);
                if ok || !bad {
                    redundant.insert(plans[ev.row_plan[row]].id);
                } else {
                    for r in grp {
                        conflicting.insert(plans[ev.row_plan[r]].id);
                    }
                }
            }
        }
        all_ok &= ok;
    }
    rep.rank = total_rank;
    rep.dof = if opts.diagnose { 6 * nfree - total_rank.min(6 * nfree) } else { 0 };
    let sorted = |s: HashSet<JointId>| {
        let mut v: Vec<JointId> = s.into_iter().collect();
        v.sort_unstable();
        v
    };
    rep.redundant_joints = sorted(redundant);
    rep.conflicting_joints = sorted(conflicting);
    rep.unsatisfied_joints = sorted(unsat);
    rep.limited_joints = sorted(limited);
    rep.status = if !all_ok {
        if rep.conflicting_joints.is_empty() {
            SolveStatus::Failed
        } else {
            SolveStatus::Conflicting
        }
    } else if !rep.redundant_joints.is_empty() {
        SolveStatus::Redundant
    } else {
        SolveStatus::Solved
    };
    if !all_ok {
        // Like FreeCAD: a failed solve leaves the parts where they were.
        pl = start.clone();
    }
    rep.moved = asm
        .instances
        .iter()
        .zip(pl.iter().zip(start.iter()))
        .filter(|(_, (a, b))| !a.is_same(b, 1e-12))
        .map(|(i, _)| i.id)
        .collect();
    (rep, pl)
}

/// Drag `instance` toward `target` while keeping every joint satisfied; returns the
/// report and the new placements (in `asm.instances` order).
pub fn compute_drag(asm: &Assembly, res: &dyn PartResolver, instance: InstanceId, target: DragTarget, opts: &SolveOptions) -> Result<(SolveReport, Vec<Frame>), SolveError> {
    let di = asm.instances.iter().position(|i| i.id == instance).ok_or(SolveError::NoInstance(instance))?;
    let start: Vec<Frame> = asm.instances.iter().map(|i| i.placement).collect();
    if asm.instances[di].grounded {
        let (rep, _) = compute_solve(asm, res, &SolveOptions { diagnose: false, ..opts.clone() });
        return Ok((SolveReport { moved: vec![], ..rep }, start));
    }
    let mut pl = start.clone();
    let prep = prepare(asm, res, &pl);
    let plans = prep.plans;
    let inv_l = prep.inv_l;
    let isls = islands(asm, &plans, Some(di));
    let isl = &isls[0];
    let cols = col_map(isl);
    let dc = cols[&di];
    let n = isl.insts.len() * 6;
    let lscale = 1.0 / inv_l;
    // Goal residuals on the dragged instance (slots 0..6 = its column block).
    let goal = |pl: &[Frame]| -> (DVector<f64>, DMatrix<f64>) {
        let wm = marker_wm(&pl[di], &Frame::IDENTITY, Some(0), inv_l);
        let mut rows = vec![];
        match target {
            DragTarget::Frame(t) => {
                let tm = WM { p: dv_c(t.pos), x: dv_c(t.x_axis()), y: dv_c(t.y_axis()), z: dv_c(t.z_axis()) };
                eval_eq(&Eq::AtPoint, &tm, &wm, &mut rows);
                let mut rr = vec![];
                eval_eq(&Eq::Orient, &tm, &wm, &mut rr);
                rows.extend(rr.into_iter().map(|d| d.scale(lscale)));
            }
            DragTarget::Point { local, world } => {
                let lm = marker_wm(&pl[di], &Frame::translation(local), Some(0), inv_l);
                for k in 0..3 {
                    rows.push(lm.p[k].add_c(-world[k]));
                }
                let _ = wm;
            }
        }
        let r = DVector::from_iterator(rows.len(), rows.iter().map(|d| d.v));
        let mut jm = DMatrix::zeros(rows.len(), n);
        for (i, d) in rows.iter().enumerate() {
            for k in 0..6 {
                jm[(i, dc * 6 + k)] = d.g[k];
            }
        }
        (r, jm)
    };
    if isl.plans.is_empty() {
        // Free part: just move it.
        pl[di] = match target {
            DragTarget::Frame(t) => t,
            DragTarget::Point { local, world } => {
                let cur = pl[di].transform_point(local);
                Frame { pos: fc_part::placement::add(pl[di].pos, sub(world, cur)), rot: pl[di].rot }
            }
        };
    } else {
        // Constrained least squares by KKT steps:
        // min |Jt δ + rt|² + μ|δ|²  s.t.  Jc δ = -rc
        let mut mu = 1e-8;
        let mut prev_goal = f64::INFINITY;
        for _ in 0..opts.max_iterations.min(100) {
            let ev = eval_island(isl, &plans, &cols, &pl, inv_l, true);
            let (rt, jt) = goal(&pl);
            let m = ev.r.len();
            // Schur complement on the constraints: H = JtᵀJt + μI only couples the dragged
            // instance's 6 columns, so H⁻¹ is a 6×6 block plus a scaled identity.
            let b0 = dc * 6;
            let jtb = jt.columns(b0, 6).into_owned();
            let hb = jtb.transpose() * &jtb;
            let sc = (0..6).fold(1e-12f64, |s, i| s.max(hb[(i, i)]));
            let mu_abs = mu * sc + 1e-12;
            let mut hd = hb.clone();
            for i in 0..6 {
                hd[(i, i)] += mu_abs;
            }
            let Some(hd_inv) = hd.try_inverse() else { break };
            let hinv = |x: &DMatrix<f64>| -> DMatrix<f64> {
                let mut y = x / mu_abs;
                let blk = &hd_inv * x.rows(b0, 6);
                y.rows_mut(b0, 6).copy_from(&blk);
                y
            };
            let g = -(jt.transpose() * &rt);
            let hg = hinv(&DMatrix::from_column_slice(n, 1, g.as_slice()));
            let a = hinv(&ev.j.transpose());
            let mut sm = &ev.j * &a;
            let sdiag = (0..m).fold(1e-300f64, |s, i| s.max(sm[(i, i)]));
            for i in 0..m {
                sm[(i, i)] += 1e-10 * sdiag;
            }
            let Some(ch) = sm.cholesky() else { break };
            let rhs = &ev.j * &hg + DMatrix::from_column_slice(m, 1, ev.r.as_slice());
            let nu = ch.solve(&rhs);
            let sol = hg - &a * nu;
            let mut d = DVector::from_column_slice(sol.as_slice());
            // Keep steps in the linear range.
            let dn = d.norm();
            let cap = lscale * 0.25;
            if dn > cap {
                d *= cap / dn;
            }
            apply_step(&mut pl, isl, &d, inv_l);
            let gn = rt.norm();
            if d.norm() < 1e-10 * lscale && max_abs(&ev.r) < 1e-9 {
                break;
            }
            if gn > prev_goal {
                mu = (mu * 4.0).min(1.0);
            }
            prev_goal = gn;
        }
    }
    // Polish: exact joint solve from the dragged pose (nearest solution).
    let mut tmp = asm.clone();
    for (inst, f) in tmp.instances.iter_mut().zip(pl.iter()) {
        inst.placement = *f;
    }
    let (mut rep, pl2) = compute_solve(&tmp, res, opts);
    if !rep.ok() {
        // Could not reach a consistent pose: stay where we were.
        rep.moved.clear();
        return Ok((rep, start));
    }
    rep.moved = asm
        .instances
        .iter()
        .zip(pl2.iter().zip(start.iter()))
        .filter(|(_, (a, b))| !a.is_same(b, 1e-12))
        .map(|(i, _)| i.id)
        .collect();
    Ok((rep, pl2))
}

/// World frames (I, J) of a joint at the current placements, after offsets (not sense).
pub fn joint_world_frames(asm: &Assembly, res: &dyn PartResolver, joint: JointId) -> Result<(Frame, Frame), SolveError> {
    let j = asm.joint(joint).ok_or(SolveError::NoJoint(joint))?;
    let iv = matches!(j.kind, JointKind::Distance { .. });
    let a = resolve_ref(asm, res, &j.a, iv).map_err(SolveError::Unresolved)?;
    let b = resolve_ref(asm, res, &j.b, iv).map_err(SolveError::Unresolved)?;
    let fa = asm.instances[a.inst].placement.mul(&a.local).mul(&j.offset_a);
    let fb = asm.instances[b.inst].placement.mul(&b.local).mul(&j.offset_b);
    Ok((fa, fb))
}

/// Instances connected to a grounded instance through joints (optionally ignoring one joint).
pub fn connected_to_ground(asm: &Assembly, ignore: Option<JointId>) -> HashSet<InstanceId> {
    let mut adj: HashMap<InstanceId, Vec<InstanceId>> = HashMap::new();
    let conn_inst = |r: &JointRef| match r {
        JointRef::AssemblyConnector(c) => asm.connectors.iter().find(|x| x.id == *c).map(|x| x.instance),
        other => other.instance(),
    };
    for j in &asm.joints {
        if j.suppressed || Some(j.id) == ignore {
            continue;
        }
        if let (Some(a), Some(b)) = (conn_inst(&j.a), conn_inst(&j.b)) {
            adj.entry(a).or_default().push(b);
            adj.entry(b).or_default().push(a);
        }
    }
    let mut seen: HashSet<InstanceId> = asm.instances.iter().filter(|i| i.grounded).map(|i| i.id).collect();
    let mut q: VecDeque<InstanceId> = seen.iter().copied().collect();
    while let Some(u) = q.pop_front() {
        for &v in adj.get(&u).map(|v| v.as_slice()).unwrap_or(&[]) {
            if seen.insert(v) {
                q.push_back(v);
            }
        }
    }
    seen
}

/// FreeCAD's "match JCS" pre-solve (`Joint.preSolve`): if one side of the joint is not
/// connected to ground by the other joints, move it — with everything hanging off it —
/// so that its JCS lands on the other one (flipped when the Z axes point apart).
/// Returns whether something moved.
pub fn presolve(asm: &mut Assembly, res: &dyn PartResolver, joint: JointId) -> Result<bool, SolveError> {
    let (fa, fb) = joint_world_frames(asm, res, joint)?;
    let j = asm.joint(joint).ok_or(SolveError::NoJoint(joint))?.clone();
    let conn_inst = |r: &JointRef| match r {
        JointRef::AssemblyConnector(c) => asm.connectors.iter().find(|x| x.id == *c).map(|x| x.instance),
        other => other.instance(),
    };
    let (Some(ia), Some(ib)) = (conn_inst(&j.a), conn_inst(&j.b)) else { return Ok(false) };
    let grounded = connected_to_ground(asm, Some(joint));
    let (moving, mframe, fframe) = if !grounded.contains(&ia) {
        (ia, fa, fb)
    } else if !grounded.contains(&ib) {
        (ib, fb, fa)
    } else {
        return Ok(false);
    };
    // Everything connected to the moving part without this joint.
    let mut group: HashSet<InstanceId> = HashSet::from([moving]);
    let mut q = VecDeque::from([moving]);
    while let Some(u) = q.pop_front() {
        for o in &asm.joints {
            if o.id == joint || o.suppressed {
                continue;
            }
            let (Some(a), Some(b)) = (conn_inst(&o.a), conn_inst(&o.b)) else { continue };
            for (x, y) in [(a, b), (b, a)] {
                if x == u && !grounded.contains(&y) && group.insert(y) {
                    q.push_back(y);
                }
            }
        }
    }
    let same_dir = dot(mframe.z_axis(), fframe.z_axis()) > 0.0;
    let same_dir = match j.sense {
        Sense::Auto => same_dir,
        Sense::Aligned => true,
        Sense::Flipped => false,
    };
    let m = if same_dir { mframe } else { mframe.flipped() };
    let t = fframe.mul(&m.inverse());
    for inst in asm.instances.iter_mut() {
        if group.contains(&inst.id) && !inst.grounded {
            inst.placement = t.mul(&inst.placement);
        }
    }
    Ok(true)
}

impl Assembly {
    /// Solve the joints, moving non-grounded instances as little as possible.
    pub fn solve(&mut self, res: &dyn PartResolver, opts: &SolveOptions) -> SolveReport {
        let (rep, pl) = compute_solve(self, res, opts);
        for (i, f) in self.instances.iter_mut().zip(pl) {
            i.placement = f;
        }
        rep
    }
    /// Drag an instance toward a target while honouring the joints.
    pub fn drag(&mut self, res: &dyn PartResolver, instance: InstanceId, target: DragTarget, opts: &SolveOptions) -> Result<SolveReport, SolveError> {
        let (rep, pl) = compute_drag(self, res, instance, target, opts)?;
        for (i, f) in self.instances.iter_mut().zip(pl) {
            i.placement = f;
        }
        Ok(rep)
    }
    /// FreeCAD's pre-solve for a newly added joint, see [`presolve`].
    pub fn presolve_joint(&mut self, res: &dyn PartResolver, joint: JointId) -> Result<bool, SolveError> {
        presolve(self, res, joint)
    }
    pub fn joint_frames(&self, res: &dyn PartResolver, joint: JointId) -> Result<(Frame, Frame), SolveError> {
        joint_world_frames(self, res, joint)
    }
}
