//! Port of OndselSolver's static position solve, the one FreeCAD 1.1's Assembly
//! workbench runs on every solve.
//!
//! Call chain in FreeCAD (`src/Mod/Assembly/App/AssemblyObject.cpp`, OndselSolver in
//! `src/3rdParty/OndselSolver/OndselSolver`):
//! `AssemblyObject::solve` → `ASMTAssembly::runPreDrag` → `System::runPreDrag` →
//! `SystemSolver::runPreDrag` → `runPosIC` → `PosICNewtonRaphson::run`.
//!
//! What that solve is (and what this module reproduces):
//!
//! * **Coordinates** (`PartFrame::qX`, `qE`): per part, the position of its frame and its
//!   four Euler parameters `E = (e0, e1, e2, e3)` (vector part first, `e3` scalar — the
//!   order of our quaternions), with the rotation matrix of `EulerParameters::calcABC`
//!   `A(E) = (e3² − e·e) I + 2 e eᵀ + 2 e3 [e]×` (not normalised while iterating).
//! * **Euler constraint** (`EulerConstraint`): `|E|² − 1 = 0` per part.
//! * **Minimum movement** (`AnyPosICNewtonRaphson::fillY` / `fillPyPx`, weights from
//!   `Part::fillqsuWeights`): the solve is the stationary point of
//!   `½ Σ (q − q_old)ᵀ W (q − q_old)` subject to all constraints, found by Newton on the
//!   full Lagrangian system `[−W + Σλ ∂²G ; Jᵀ ; J ; 0]` (the constraints'
//!   `fillPosICJacob` add the `λ`-weighted second derivatives). Weights:
//!   `wX = 1e6 · m/m_max + 1e3`, `wE[0..3] = 1e6 · J_i/J_max + 1e3`, `wE[3] = 1e3`.
//!   FreeCAD gives every part mass 1 and inertias (1, 1, 1) (`makeMbdPart`).
//! * **Ground**: grounded parts are pinned by a fixed joint to the assembly's marker
//!   (`fixGroundedPart`); here they are simply constants.
//! * **Redundant constraints** (`PosICNewtonRaphson::run`, `handleSingularMatrix`,
//!   `GESpMatFullPvPosIC`): equations found dependent during elimination are dropped
//!   (their multipliers stay 0). We do the same per iteration in the Schur-complement
//!   factorisation (zero pivots are skipped).
//! * **Convergence**: Ondsel iterates to the numerical limit (`isConvergedToNumericalLimit`,
//!   at most 100 iterations, no line search).
//!
//! Differences in the numerics (not in the result):
//! * our joint equations ([`crate::solver`]) differ from Ondsel's in form but define the
//!   same constraint manifold, so the stationary point is the same;
//! * the second derivatives of the joint equations are central differences of their
//!   exact (dual-number) gradients, where Ondsel has closed forms;
//! * the Newton matrix is factored by an envelope LDLᵀ in body order (breadth first from
//!   the ground), where Ondsel uses sparse Gaussian elimination with Markowitz pivoting
//!   (`GESpMatParPvMarkoFast`); zero pivots of equations play the role of Ondsel's
//!   redundant-constraint removal.
//!
//! **Weights in FreeCAD 1.1.** FreeCAD 1.1.3 ships OndselSolver 1.0.1, whose results
//! match the weights above with `J_i/J_max = 1/3` exactly (all 8 parity cases to 1e-15,
//! checked by running the OndselSolver sources with that ratio against FreeCAD 1.1.3's
//! output), whereas the current OndselSolver sources give `J_i/J_max = 1`. The ratio is
//! [`OndselOptions::inertia_ratio`].

use crate::dual::{self, dv_c, D, DV, SIDE_B};
use crate::frame::Frame;
use crate::solver::{eval_eq, Island, Plan, WM};
use nalgebra::SVector;
use std::collections::{HashMap, VecDeque};

/// `maxw` / `minw` of `Part::fillqsuWeights`.
pub const MAX_W: f64 = 1.0e6;
pub const MIN_W: f64 = 1.0e3;

/// Options of the Ondsel solve.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OndselOptions {
    /// `aJi / aJiMax` in `Part::fillqsuWeights` (all parts have the same inertia in
    /// FreeCAD): 1/3 reproduces FreeCAD 1.1.3, 1 the current OndselSolver sources.
    pub inertia_ratio: f64,
    /// Newton iteration limit (`NewtonRaphson::iterMax`).
    pub max_iterations: usize,
}

impl OndselOptions {
    /// What FreeCAD 1.1 (OndselSolver 1.0.1) computes.
    pub const FREECAD_1_1: OndselOptions = OndselOptions { inertia_ratio: 1.0 / 3.0, max_iterations: 100 };
    /// The weights of the current OndselSolver sources (`Part::fillqsuWeights`).
    pub const ONDSEL_MASTER: OndselOptions = OndselOptions { inertia_ratio: 1.0, max_iterations: 100 };
}

impl Default for OndselOptions {
    fn default() -> Self {
        OndselOptions::FREECAD_1_1
    }
}

/// Position and Euler parameters of one body (`qX`, `qE`).
type Q = [f64; 7];
type V7 = SVector<f64, 7>;

fn q_of(f: &Frame) -> Q {
    [f.pos[0], f.pos[1], f.pos[2], f.rot[0], f.rot[1], f.rot[2], f.rot[3]]
}

fn frame_of(q: &Q) -> Frame {
    let n = (q[3] * q[3] + q[4] * q[4] + q[5] * q[5] + q[6] * q[6]).sqrt();
    Frame { pos: [q[0], q[1], q[2]], rot: [q[3] / n, q[4] / n, q[5] / n, q[6] / n] }
}

/// `A(E) v` with Ondsel's (unnormalised) rotation matrix of the Euler parameters.
fn rot_apply(e: &[D; 4], v: [f64; 3]) -> DV {
    let ee = e[0] * e[0] + e[1] * e[1] + e[2] * e[2];
    let s = e[3] * e[3] - ee;
    let ev = (e[0].scale(v[0]) + e[1].scale(v[1])) + e[2].scale(v[2]);
    let c = [e[1].scale(v[2]) - e[2].scale(v[1]), e[2].scale(v[0]) - e[0].scale(v[2]), e[0].scale(v[1]) - e[1].scale(v[0])];
    let mut out = [D::c(0.0); 3];
    for k in 0..3 {
        out[k] = s.scale(v[k]) + (e[k] * ev).scale(2.0) + (e[3] * c[k]).scale(2.0);
    }
    out
}

/// One side of a joint: a body of the island (its coordinates, slot offset) or a
/// constant placement.
#[derive(Clone, Copy)]
enum SideQ<'a> {
    Var(&'a Q, usize),
    Const(&'a Frame),
}

fn side_wm(s: SideQ, local: &Frame) -> WM {
    match s {
        SideQ::Const(f) => {
            let w = f.mul(local);
            WM { p: dv_c(w.pos), x: dv_c(w.x_axis()), y: dv_c(w.y_axis()), z: dv_c(w.z_axis()) }
        }
        SideQ::Var(q, off) => {
            let x = [D::var(q[0], off), D::var(q[1], off + 1), D::var(q[2], off + 2)];
            let e = [D::var(q[3], off + 3), D::var(q[4], off + 4), D::var(q[5], off + 5), D::var(q[6], off + 6)];
            let p = dual::vadd(&x, &rot_apply(&e, local.pos));
            WM { p, x: rot_apply(&e, local.x_axis()), y: rot_apply(&e, local.y_axis()), z: rot_apply(&e, local.z_axis()) }
        }
    }
}

/// A constraint row: value and gradient blocks on (at most) two bodies of the island.
struct Row {
    val: f64,
    a: Option<(usize, [f64; 7])>,
    b: Option<(usize, [f64; 7])>,
}

impl Row {
    fn blocks(&self) -> impl Iterator<Item = &(usize, [f64; 7])> {
        self.a.iter().chain(self.b.iter())
    }
}

/// The island in Ondsel's coordinates.
struct Sys<'a> {
    plans: &'a [Plan],
    plan_ids: &'a [usize],
    /// instance index → body index of the island
    body_of: HashMap<usize, usize>,
    /// fixed placements (all instances; island bodies are overridden by `q`)
    pl: &'a [Frame],
}

impl Sys<'_> {
    fn side<'q>(&'q self, inst: usize, q: &'q [Q], off: usize) -> SideQ<'q> {
        match self.body_of.get(&inst) {
            Some(&k) => SideQ::Var(&q[k], off),
            None => SideQ::Const(&self.pl[inst]),
        }
    }
    /// Rows of one plan as duals over the 14 local slots.
    fn plan_rows(&self, p: &Plan, qa: SideQ, qb: SideQ, out: &mut Vec<D>) {
        let wi = side_wm(qa, &p.a.local);
        let wj = side_wm(qb, &p.b.local);
        for e in &p.eqs {
            eval_eq(e, &wi, &wj, out);
        }
    }
    /// All rows: one Euler row per body, then the joint rows in plan order.
    fn rows(&self, q: &[Q]) -> Vec<Row> {
        let mut rows = Vec::with_capacity(q.len() * 2);
        for (k, b) in q.iter().enumerate() {
            let val = b[3] * b[3] + b[4] * b[4] + b[5] * b[5] + b[6] * b[6] - 1.0;
            rows.push(Row { val, a: Some((k, [0.0, 0.0, 0.0, 2.0 * b[3], 2.0 * b[4], 2.0 * b[5], 2.0 * b[6]])), b: None });
        }
        let mut buf = vec![];
        for &pi in self.plan_ids {
            let p = &self.plans[pi];
            buf.clear();
            self.plan_rows(p, self.side(p.a.inst, q, 0), self.side(p.b.inst, q, SIDE_B), &mut buf);
            let ka = self.body_of.get(&p.a.inst).copied();
            let kb = self.body_of.get(&p.b.inst).copied();
            for d in &buf {
                let blk = |k: Option<usize>, off: usize| k.map(|k| (k, std::array::from_fn(|i| d.g[off + i])));
                rows.push(Row { val: d.v, a: blk(ka, 0), b: blk(kb, SIDE_B) });
            }
        }
        rows
    }
    fn plan_row_count(&self, q: &[Q]) -> Vec<usize> {
        let mut out = vec![];
        let mut buf = vec![];
        for &pi in self.plan_ids {
            let p = &self.plans[pi];
            buf.clear();
            self.plan_rows(p, self.side(p.a.inst, q, 0), self.side(p.b.inst, q, SIDE_B), &mut buf);
            out.push(buf.len());
        }
        out
    }
    /// Gradient of `Σ λ_r G_r` of one plan over the 14 local slots.
    fn plan_grad(&self, p: &Plan, q: &[Q], lam: &[f64], buf: &mut Vec<D>) -> [f64; 14] {
        buf.clear();
        self.plan_rows(p, self.side(p.a.inst, q, 0), self.side(p.b.inst, q, SIDE_B), buf);
        let mut g = [0.0; 14];
        for (d, l) in buf.iter().zip(lam) {
            for i in 0..14 {
                g[i] += l * d.g[i];
            }
        }
        g
    }
}

/// Envelope (profile) LDLᵀ factorisation of the symmetric Newton (KKT) matrix, without
/// pivoting. Zero pivots mark dependent (redundant) equations, which are dropped like
/// Ondsel's redundant-constraint removal (their multiplier step is 0).
struct Envelope {
    first: Vec<usize>,
    /// row i holds columns first[i]..=i (L below the diagonal, D on it after factoring)
    rows: Vec<Vec<f64>>,
}

impl Envelope {
    fn new(first: Vec<usize>) -> Envelope {
        Envelope { rows: first.iter().enumerate().map(|(i, &f)| vec![0.0; i - f + 1]).collect(), first }
    }
    fn add(&mut self, i: usize, j: usize, v: f64) {
        let (i, j) = if j > i { (j, i) } else { (i, j) };
        let f = self.first[i];
        debug_assert!(j >= f, "entry ({i}, {j}) outside the envelope");
        self.rows[i][j - f] += v;
    }
    /// In-place LDLᵀ; `scale[i]` is the size a pivot is compared with. Returns the
    /// dependent rows.
    fn factor(&mut self, scale: &[f64], rel_tol: f64) -> Vec<bool> {
        let m = self.rows.len();
        let mut dep = vec![false; m];
        let mut d = vec![0.0; m];
        let mut tmp = vec![0.0; m];
        for i in 0..m {
            let fi = self.first[i];
            // tmp[j] = L[i][j] D[j]
            for j in fi..i {
                if dep[j] {
                    tmp[j] = 0.0;
                    self.rows[i][j - fi] = 0.0;
                    continue;
                }
                let fj = self.first[j];
                let mut t = self.rows[i][j - fi];
                for k in fi.max(fj)..j {
                    t -= tmp[k] * self.rows[j][k - fj];
                }
                tmp[j] = t;
                self.rows[i][j - fi] = t / d[j];
            }
            let mut di = self.rows[i][i - fi];
            for j in fi..i {
                di -= tmp[j] * self.rows[i][j - fi];
            }
            if di.is_nan() || di.abs() <= rel_tol * scale[i] {
                dep[i] = true;
                for j in fi..i {
                    self.rows[i][j - fi] = 0.0;
                }
                d[i] = 1.0;
            } else {
                d[i] = di;
            }
            self.rows[i][i - fi] = d[i];
        }
        dep
    }
    fn solve(&self, dep: &[bool], b: &mut [f64]) {
        let m = b.len();
        for i in 0..m {
            let fi = self.first[i];
            if dep[i] {
                b[i] = 0.0;
                continue;
            }
            let mut s = b[i];
            for k in fi..i {
                s -= self.rows[i][k - fi] * b[k];
            }
            b[i] = s;
        }
        for i in 0..m {
            let fi = self.first[i];
            b[i] = if dep[i] { 0.0 } else { b[i] / self.rows[i][i - fi] };
        }
        for i in (0..m).rev() {
            let fi = self.first[i];
            let x = b[i];
            if x == 0.0 {
                continue;
            }
            for k in fi..i {
                b[k] -= self.rows[i][k - fi] * x;
            }
        }
    }
}

/// Result of [`solve_island`].
pub(crate) struct Outcome {
    pub converged: bool,
    pub iterations: usize,
}

/// Ondsel's position solve (`PosICNewtonRaphson`) on one island: moves the island's
/// instances in `pl` to the stationary point of the weighted distance to `q_old`
/// subject to the joints. `wmul` scales each instance's weights (the drag step holds the
/// dragged part with a large factor).
pub(crate) fn solve_island(isl: &Island, plans: &[Plan], pl: &mut [Frame], q_old: &[Frame], wmul: &[f64], o: &OndselOptions, tol: f64) -> Outcome {
    let nb = isl.insts.len();
    let body_of: HashMap<usize, usize> = isl.insts.iter().enumerate().map(|(k, &i)| (i, k)).collect();
    let fixed: Vec<Frame> = pl.to_vec();
    let sys = Sys { plans, plan_ids: &isl.plans, body_of, pl: &fixed };
    // State and reference (sign of E continuous with the reference).
    let q0: Vec<Q> = isl.insts.iter().map(|&i| q_of(&q_old[i])).collect();
    let mut q: Vec<Q> = isl
        .insts
        .iter()
        .zip(&q0)
        .map(|(&i, r)| {
            let mut b = q_of(&pl[i]);
            if b[3] * r[3] + b[4] * r[4] + b[5] * r[5] + b[6] * r[6] < 0.0 {
                for v in &mut b[3..] {
                    *v = -*v;
                }
            }
            b
        })
        .collect();
    // Part::fillqsuWeights with m = 1, J = (1, 1, 1) for every part.
    let wx = MAX_W + MIN_W;
    let we = MAX_W * o.inertia_ratio + MIN_W;
    let w: Vec<[f64; 7]> = isl.insts.iter().map(|&i| [wx, wx, wx, we, we, we, MIN_W].map(|v| v * wmul[i])).collect();

    // Row bookkeeping: Euler rows then plan rows; λ per row.
    let counts = sys.plan_row_count(&q);
    let m = nb + counts.iter().sum::<usize>();
    let mut plan_start = Vec::with_capacity(counts.len());
    let mut s0 = nb;
    for c in &counts {
        plan_start.push(s0);
        s0 += c;
    }
    let mut lam = vec![0.0; m];

    // Envelope ordering: bodies breadth-first from the ground, rows by their last body.
    let mut adj: Vec<Vec<usize>> = vec![vec![]; nb];
    let mut grounded_bodies = vec![];
    for &pi in &isl.plans {
        let p = &plans[pi];
        match (sys.body_of.get(&p.a.inst), sys.body_of.get(&p.b.inst)) {
            (Some(&a), Some(&b)) => {
                adj[a].push(b);
                adj[b].push(a);
            }
            (Some(&a), None) | (None, Some(&a)) => grounded_bodies.push(a),
            _ => {}
        }
    }
    let mut bpos = vec![usize::MAX; nb];
    let mut next = 0;
    let mut queue: VecDeque<usize> = VecDeque::new();
    for s in grounded_bodies.into_iter().chain(0..nb) {
        if bpos[s] != usize::MAX {
            continue;
        }
        bpos[s] = next;
        next += 1;
        queue.push_back(s);
        while let Some(u) = queue.pop_front() {
            for &v in &adj[u] {
                if bpos[v] == usize::MAX {
                    bpos[v] = next;
                    next += 1;
                    queue.push_back(v);
                }
            }
        }
    }
    // Unknowns in elimination order: per body (breadth first), its 7 coordinates, then
    // the equations whose last body it is (its Euler equation, joints to earlier bodies
    // or to the ground).
    let rows0 = sys.rows(&q);
    let row_bodies: Vec<Vec<usize>> = rows0.iter().map(|r| r.blocks().map(|b| b.0).collect()).collect();
    let mut by_pos: Vec<usize> = (0..nb).collect();
    by_pos.sort_by_key(|&k| bpos[k]);
    let mut rows_of_last: Vec<Vec<usize>> = vec![vec![]; nb];
    for (r, bs) in row_bodies.iter().enumerate() {
        let last = *bs.iter().max_by_key(|&&k| bpos[k]).expect("row without a body");
        rows_of_last[last].push(r);
    }
    let mut var_at = vec![0; nb];
    let mut row_at = vec![0; m];
    let mut n = 0;
    for &k in &by_pos {
        var_at[k] = n;
        n += 7;
        for &r in &rows_of_last[k] {
            row_at[r] = n;
            n += 1;
        }
    }
    // Envelope: a body's coordinates reach back to the bodies it shares a joint with
    // (Hessian cross terms), an equation to its first body.
    let mut first: Vec<usize> = (0..n).collect();
    for k in 0..nb {
        for c in 0..7 {
            first[var_at[k] + c] = var_at[k];
        }
    }
    for &pi in &isl.plans {
        let p = &plans[pi];
        if let (Some(&a), Some(&b)) = (sys.body_of.get(&p.a.inst), sys.body_of.get(&p.b.inst)) {
            let (lo, hi) = if var_at[a] < var_at[b] { (a, b) } else { (b, a) };
            for c in 0..7 {
                first[var_at[hi] + c] = first[var_at[hi] + c].min(var_at[lo]);
            }
        }
    }
    for (r, bs) in row_bodies.iter().enumerate() {
        first[row_at[r]] = bs.iter().map(|&k| var_at[k]).min().unwrap();
    }
    // Pivot scales: weights for coordinates, J W⁻¹ Jᵀ for equations.
    let mut scale = vec![0.0; n];
    for k in 0..nb {
        for c in 0..7 {
            scale[var_at[k] + c] = w[k][c];
        }
    }

    let mut it = 0;
    let mut converged = false;
    let mut small_steps = 0;
    let mut last_dq = f64::INFINITY;
    let mut buf = vec![];
    while it < o.max_iterations {
        it += 1;
        let rows = sys.rows(&q);
        debug_assert_eq!(rows.len(), m);
        // Stationarity residual r1 = W (q − q_old) − Jᵀλ, per body.
        let mut r1: Vec<V7> = (0..nb).map(|k| V7::from_fn(|i, _| w[k][i] * (q[k][i] - q0[k][i]))).collect();
        for (r, row) in rows.iter().enumerate() {
            if lam[r] != 0.0 {
                for (k, g) in row.blocks() {
                    for i in 0..7 {
                        r1[*k][i] -= lam[r] * g[i];
                    }
                }
            }
        }
        // Newton matrix [W − Σλ∂²G, Jᵀ; J, 0] (Ondsel's `pypx` up to sign).
        let mut kkt = Envelope::new(first.clone());
        for k in 0..nb {
            for i in 0..7 {
                let mut v = w[k][i];
                if i >= 3 {
                    v -= 2.0 * lam[k];
                }
                kkt.add(var_at[k] + i, var_at[k] + i, v);
            }
        }
        for (r, row) in rows.iter().enumerate() {
            let mut sc = 0.0;
            for (k, g) in row.blocks() {
                for i in 0..7 {
                    if g[i] != 0.0 {
                        kkt.add(row_at[r], var_at[*k] + i, g[i]);
                        sc += g[i] * g[i] / w[*k][i];
                    }
                }
            }
            scale[row_at[r]] = sc;
        }
        // λ-weighted second derivatives of the joint equations (central differences of
        // the exact gradients).
        for (pk, &pi) in isl.plans.iter().enumerate() {
            let p = &plans[pi];
            let lp = &lam[plan_start[pk]..plan_start[pk] + counts[pk]];
            if lp.iter().all(|&l| l == 0.0) {
                continue;
            }
            let sides = [sys.body_of.get(&p.a.inst).copied(), sys.body_of.get(&p.b.inst).copied()];
            let mut h = [[0.0f64; 14]; 14];
            for (sd, kb) in sides.iter().enumerate() {
                let Some(k) = *kb else { continue };
                for c in 0..7 {
                    let step = 1e-6 * q[k][c].abs().max(1.0);
                    let mut qp = q.clone();
                    qp[k][c] += step;
                    let gp = sys.plan_grad(p, &qp, lp, &mut buf);
                    qp[k][c] -= 2.0 * step;
                    let gm = sys.plan_grad(p, &qp, lp, &mut buf);
                    for i in 0..14 {
                        h[i][sd * 7 + c] = (gp[i] - gm[i]) / (2.0 * step);
                    }
                }
            }
            for si in 0..2 {
                let Some(ki) = sides[si] else { continue };
                for sj in 0..2 {
                    let Some(kj) = sides[sj] else { continue };
                    for i in 0..7 {
                        for j in 0..7 {
                            let (gi, gj) = (var_at[ki] + i, var_at[kj] + j);
                            if gi >= gj {
                                let v = 0.5 * (h[si * 7 + i][sj * 7 + j] + h[sj * 7 + j][si * 7 + i]);
                                kkt.add(gi, gj, -v);
                            }
                        }
                    }
                }
            }
        }
        // Right-hand side [−r1; −G]; the solution is [dq; −dλ].
        let mut x = vec![0.0; n];
        for k in 0..nb {
            for i in 0..7 {
                x[var_at[k] + i] = -r1[k][i];
            }
        }
        for (r, row) in rows.iter().enumerate() {
            x[row_at[r]] = -row.val;
        }
        let dep = kkt.factor(&scale, 1e-10);
        kkt.solve(&dep, &mut x);
        let mut dq_max = 0.0f64;
        let mut size = 1.0f64;
        for k in 0..nb {
            for i in 0..7 {
                let dq = x[var_at[k] + i];
                q[k][i] += dq;
                if i < 3 {
                    dq_max = dq_max.max(dq.abs());
                    size = size.max(q[k][i].abs());
                } else {
                    dq_max = dq_max.max(dq.abs() * 100.0);
                }
            }
        }
        for r in 0..m {
            lam[r] -= x[row_at[r]];
        }
        if !dq_max.is_finite() {
            break;
        }
        // Numerical limit: steps at round-off level, or no longer shrinking once small.
        if dq_max <= 1e-14 * size {
            converged = true;
            break;
        }
        if dq_max < 1e-11 * size && dq_max > 0.5 * last_dq {
            small_steps += 1;
            if small_steps >= 3 {
                converged = true;
                break;
            }
        }
        last_dq = dq_max;
    }
    let resid = sys.rows(&q).iter().fold(0.0f64, |a, r| a.max(r.val.abs()));
    let ok = converged && resid <= tol && q.iter().all(|b| b.iter().all(|v| v.is_finite()));
    if ok {
        for (k, &i) in isl.insts.iter().enumerate() {
            pl[i] = frame_of(&q[k]);
        }
    }
    Outcome { converged: ok, iterations: it }
}
