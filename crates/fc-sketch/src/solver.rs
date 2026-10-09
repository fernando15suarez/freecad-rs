//! DogLeg solver, drag solver and the solve/diagnose pipeline.

use crate::diagnose::{choose_skipped, qr_diagnose};
use crate::equations::{eval_drag, geo_params, Block, Layout, System};
use crate::{PointPos, PointRef, SolveReport, SolveStatus, Sketch, Vec2};
use nalgebra::{DMatrix, DVector};
use std::collections::{BTreeSet, HashSet};

/// Residual (max-abs) the solver aims for.
const TOL_TARGET: f64 = 1e-12;
/// Residual (max-abs) below which a solve counts as converged.
const TOL_ACCEPT: f64 = 1e-9;
/// FreeCAD's `convergenceRedundant` (applied to err^2).
const CONVERGENCE_REDUNDANT: f64 = 1e-10;
const MAX_ITER: usize = 500;

pub(crate) struct SolveOut {
    pub x: DVector<f64>,
    pub converged: bool,
    pub iters: usize,
}

fn select(f: &DVector<f64>, j: &DMatrix<f64>, active: &[bool]) -> (DVector<f64>, DMatrix<f64>) {
    let idx: Vec<usize> = (0..f.len()).filter(|&i| active[i]).collect();
    let fa = DVector::from_iterator(idx.len(), idx.iter().map(|&i| f[i]));
    let ja = j.select_rows(idx.iter());
    (fa, ja)
}

/// Factorisation of a Jacobian for minimum-norm solves and null-space projection:
/// Cholesky of J J^T when J has (well-conditioned) full row rank, SVD otherwise.
pub(crate) enum RowSolver {
    Empty(usize),
    Chol(DMatrix<f64>, nalgebra::Cholesky<f64, nalgebra::Dyn>),
    Svd(nalgebra::SVD<f64, nalgebra::Dyn, nalgebra::Dyn>, f64),
}

impl RowSolver {
    pub fn new(j: &DMatrix<f64>) -> RowSolver {
        let n = j.ncols();
        if j.nrows() == 0 || n == 0 {
            return RowSolver::Empty(n);
        }
        if j.nrows() <= n {
            let jjt = j * j.transpose();
            let dmax = jjt.diagonal().max();
            let dmin = jjt.diagonal().min();
            if dmax > 0.0 && dmax.is_finite() && dmin > 1e-8 * dmax {
                if let Some(ch) = jjt.cholesky() {
                    let l = ch.l();
                    let lmin = l.diagonal().iter().fold(f64::INFINITY, |a, &v| a.min(v.abs()));
                    let lmax = l.diagonal().iter().fold(0.0f64, |a, &v| a.max(v.abs()));
                    // rank deficiency / ill conditioning -> SVD
                    if lmin > 1e-6 * lmax {
                        return RowSolver::Chol(j.clone(), ch);
                    }
                }
            }
        }
        let svd = j.clone().svd(true, true);
        let smax = svd.singular_values.max();
        RowSolver::Svd(svd, smax)
    }

    /// Minimum-norm least-squares solution of J h = b.
    pub fn min_norm(&self, b: &DVector<f64>) -> DVector<f64> {
        match self {
            RowSolver::Empty(n) => DVector::zeros(*n),
            RowSolver::Chol(j, ch) => j.transpose() * ch.solve(b),
            RowSolver::Svd(svd, smax) => {
                let n = svd.v_t.as_ref().map(|v| v.ncols()).unwrap_or(0);
                if !(*smax > 0.0) || !smax.is_finite() {
                    return DVector::zeros(n);
                }
                svd.solve(b, smax * 1e-12).unwrap_or_else(|_| DVector::zeros(n))
            }
        }
    }

    /// Project the columns of V onto the null space of J.
    pub fn project_null(&self, v: &DMatrix<f64>, rel_tol: f64) -> DMatrix<f64> {
        match self {
            RowSolver::Empty(_) => v.clone(),
            RowSolver::Chol(j, ch) => v - j.transpose() * ch.solve(&(j * v)),
            RowSolver::Svd(svd, smax) => {
                let vt = svd.v_t.as_ref().unwrap();
                let sv = &svd.singular_values;
                let mut out = v.clone();
                for i in 0..sv.len() {
                    if sv[i] > rel_tol * smax {
                        let row = vt.row(i);
                        let coef = row * v; // 1 x k
                        out -= row.transpose() * coef;
                    }
                }
                out
            }
        }
    }
}

/// Minimum-norm least-squares solution of J h = b.
pub(crate) fn min_norm_solve(j: &DMatrix<f64>, b: &DVector<f64>) -> DVector<f64> {
    let h = RowSolver::new(j).min_norm(b);
    if h.iter().all(|v| v.is_finite()) {
        h
    } else {
        DVector::zeros(j.ncols())
    }
}

/// Orthonormal basis (columns) of the null space of J.
pub(crate) fn null_space(j: &DMatrix<f64>, rel_tol: f64) -> DMatrix<f64> {
    let (m, n) = j.shape();
    if n == 0 {
        return DMatrix::zeros(0, 0);
    }
    if m == 0 {
        return DMatrix::identity(n, n);
    }
    let jj = if m < n {
        let mut p = DMatrix::zeros(n, n);
        p.view_mut((0, 0), (m, n)).copy_from(j);
        p
    } else {
        j.clone()
    };
    let svd = jj.svd(false, true);
    let vt = svd.v_t.unwrap();
    let sv = &svd.singular_values;
    let smax = sv.max();
    let cols: Vec<usize> = (0..sv.len()).filter(|&i| !(sv[i] > rel_tol * smax)).collect();
    let mut nb = DMatrix::zeros(n, cols.len());
    for (k, &i) in cols.iter().enumerate() {
        for p in 0..n {
            nb[(p, k)] = vt[(i, p)];
        }
    }
    nb
}

fn inf_norm(v: &DVector<f64>) -> f64 {
    v.iter().fold(0.0f64, |a, &b| a.max(b.abs()))
}

fn gather(x: &DVector<f64>, params: &[usize]) -> DVector<f64> {
    DVector::from_iterator(params.len(), params.iter().map(|&p| x[p]))
}

fn scatter(x: &mut DVector<f64>, params: &[usize], xl: &DVector<f64>) {
    for (k, &p) in params.iter().enumerate() {
        x[p] = xl[k];
    }
}

/// Powell's DogLeg trust-region method (FreeCAD's default algorithm) with
/// minimum-norm Gauss-Newton steps, on one block. Returns the full parameter vector
/// (only the block's parameters change). `active` masks the block's local rows.
pub(crate) fn dogleg(sys: &System, b: &Block, col: &[usize], x0: &DVector<f64>, active: &[bool]) -> SolveOut {
    let mut xf = x0.clone();
    let eval = |xf: &DVector<f64>| {
        let (f, j) = sys.eval_block(b, col, xf);
        select(&f, &j, active)
    };
    let mut x = gather(x0, &b.params);
    let (mut f, mut j) = eval(&xf);
    if f.is_empty() {
        return SolveOut { x: xf, converged: true, iters: 0 };
    }
    let mut fx = 0.5 * f.norm_squared();
    let mut delta = -1.0;
    let mut iters = 0;
    while iters < MAX_ITER {
        if inf_norm(&f) <= TOL_TARGET {
            break;
        }
        iters += 1;
        let g = j.transpose() * &f;
        let h_gn = -min_norm_solve(&j, &f);
        let jg = &j * &g;
        let gn2 = g.norm_squared();
        let alpha = if jg.norm_squared() > 0.0 { gn2 / jg.norm_squared() } else { 0.0 };
        let h_sd = &g * (-alpha);
        if delta < 0.0 {
            delta = h_gn.norm().max(1e-6);
        }
        let h = if h_gn.norm() <= delta {
            h_gn.clone()
        } else if h_sd.norm() >= delta {
            &h_sd * (delta / h_sd.norm())
        } else {
            // dogleg: h_sd + beta (h_gn - h_sd) with |h| = delta
            let a = h_sd.clone();
            let bv = &h_gn - &h_sd;
            let bb = bv.norm_squared();
            let ab = a.dot(&bv);
            let c = a.norm_squared() - delta * delta;
            let beta = if bb > 0.0 { (-ab + (ab * ab - bb * c).max(0.0).sqrt()) / bb } else { 0.0 };
            a + bv * beta
        };
        let hn = h.norm();
        if !hn.is_finite() || hn <= 1e-15 * (x.norm() + 1e-15) {
            break;
        }
        let x_new = &x + &h;
        let mut xf_new = xf.clone();
        scatter(&mut xf_new, &b.params, &x_new);
        let (f_new, j_new) = eval(&xf_new);
        let fx_new = 0.5 * f_new.norm_squared();
        let lin = &f + &j * &h;
        let dl = fx - 0.5 * lin.norm_squared();
        let rho = if dl > 0.0 { (fx - fx_new) / dl } else { -1.0 };
        if fx_new < fx && fx_new.is_finite() {
            x = x_new;
            xf = xf_new;
            f = f_new;
            j = j_new;
            fx = fx_new;
        }
        if rho > 0.75 {
            delta = delta.max(3.0 * hn);
        } else if rho < 0.25 {
            delta = hn * 0.5;
            if delta < 1e-15 * (x.norm() + 1e-15) {
                break;
            }
        }
    }
    let converged = inf_norm(&f) <= TOL_ACCEPT;
    SolveOut { x: xf, converged, iters }
}

/// Move a point toward `target` while keeping the constraints: Gauss-Newton on the
/// constraints with the drag equations minimised (damped) in the constraints' null
/// space. FreeCAD likewise solves the temporary drag constraint in a lower-priority
/// subsystem.
fn drag(sys: &System, b: &Block, col: &[usize], x0: &DVector<f64>, p: PointRef, target: Vec2) -> (DVector<f64>, usize) {
    let nl = b.params.len();
    let mut xf = x0.clone();
    let mut iters = 0;
    let all = vec![true; b.rows.len()];
    let drag_eval = |xf: &DVector<f64>| eval_drag(sys.sk, sys.lay, col, nl, xf, p, target);
    for _ in 0..100 {
        iters += 1;
        let (f, j) = {
            let (f, j) = sys.eval_block(b, col, &xf);
            select(&f, &j, &all)
        };
        let Ok((g, gj)) = drag_eval(&xf) else { break };
        let rs = RowSolver::new(&j);
        let h0 = -rs.min_norm(&f);
        // K = P G^T (drag gradients projected on the constraints' null space);
        // h = h0 + K pinv(G K) (-(g + G h0))  ==  h0 + N pinv(G N) (...)
        let k = rs.project_null(&gj.transpose(), 1e-10);
        let mm = &gj * &k;
        let rhs = -(&g + &gj * &h0);
        let w = min_norm_solve(&mm, &rhs);
        let h = &h0 + &k * w;
        if !h.iter().all(|v| v.is_finite()) {
            break;
        }
        let base = inf_norm(&f).max(1e-9);
        let x = gather(&xf, &b.params);
        let mut step = h.clone();
        let mut ok = false;
        for _ in 0..30 {
            let mut xn = xf.clone();
            scatter(&mut xn, &b.params, &(&x + &step));
            let (fnew, _) = sys.eval_block(b, col, &xn);
            let gnew = drag_eval(&xn).map(|(gv, _)| gv.norm()).unwrap_or(f64::INFINITY);
            let primary_ok = inf_norm(&fnew) <= 10.0 * base + 1e-3 * step.norm() && fnew.iter().all(|v| v.is_finite());
            // the drag objective must not get worse (damped Gauss-Newton)
            let secondary_ok = gnew <= g.norm() * (1.0 + 1e-12) || inf_norm(&f) > TOL_ACCEPT;
            if primary_ok && secondary_ok {
                ok = true;
                break;
            }
            step *= 0.5;
        }
        if !ok {
            break;
        }
        scatter(&mut xf, &b.params, &(&x + &step));
        if step.norm() <= 1e-12 * (1.0 + x.norm()) {
            break;
        }
    }
    (xf, iters)
}

pub(crate) struct RunOut {
    pub report: SolveReport,
    pub solution: Option<DVector<f64>>,
    pub measured: Vec<(usize, f64)>,
    pub layout: Layout,
}

pub(crate) fn run(sk: &Sketch, drag_to: Option<(PointRef, Vec2)>) -> RunOut {
    let lay = Layout::new(sk);
    let x0 = lay.pack(sk);
    let (report, solution, measured) = {
        let sys = System::new(sk, &lay, &x0, |c| c.driving);
        run_system(sk, &lay, &sys, &x0, drag_to)
    };
    RunOut { report, solution, measured, layout: lay }
}

struct BlockRun {
    start: DVector<f64>,
    converged: bool,
    groups: Vec<Vec<usize>>, // global rows
}

fn run_system(
    sk: &Sketch,
    lay: &Layout,
    sys: &System,
    x0: &DVector<f64>,
    drag_to: Option<(PointRef, Vec2)>,
) -> (SolveReport, Option<DVector<f64>>, Vec<(usize, f64)>) {
    let mut report = SolveReport {
        status: SolveStatus::Failed,
        dof: lay.n,
        conflicting: vec![],
        redundant: vec![],
        partially_redundant: vec![],
        malformed: sys.malformed.clone(),
        iterations: 0,
    };
    let extra: Vec<Vec<usize>> = match drag_to {
        Some((p, _)) => {
            let valid = p.geo < sk.geometry.len() && (sk.try_point(p).is_some() || p.pos == PointPos::Edge);
            if !valid {
                return (report, None, vec![]);
            }
            vec![geo_params(lay, p.geo)]
        }
        None => vec![],
    };
    let blocks = sys.blocks(&extra);
    let drag_block = extra.first().and_then(|ps| blocks.iter().position(|b| b.params.contains(&ps[0])));

    if !sys.malformed.is_empty() {
        // FreeCAD refuses to solve sketches with malformed constraints.
        let rank: usize = blocks
            .iter()
            .map(|b| qr_diagnose(&sys.eval_block(b, &b.col_map(lay.n), x0).1).rank)
            .sum();
        report.dof = lay.n - rank;
        return (report, None, vec![]);
    }

    let mut x = x0.clone();
    let mut runs: Vec<BlockRun> = Vec::with_capacity(blocks.len());
    let mut rank = 0;
    for (bi, b) in blocks.iter().enumerate() {
        let col = b.col_map(lay.n);
        let all = vec![true; b.rows.len()];
        let mut start = x0.clone();
        if drag_block == Some(bi) {
            let (p, t) = drag_to.unwrap();
            let (xd, it) = drag(sys, b, &col, x0, p, t);
            report.iterations += it;
            start = xd;
        }
        let mut out = dogleg(sys, b, &col, &start, &all);
        if !out.converged && drag_block == Some(bi) {
            // the drag went somewhere unsolvable; solve from the original state instead
            let o2 = dogleg(sys, b, &col, x0, &all);
            if o2.converged {
                out = o2;
                start = x0.clone();
            }
        }
        report.iterations += out.iters;
        let (_, j) = sys.eval_block(b, &col, &out.x);
        let qd = qr_diagnose(&j);
        rank += qd.rank;
        scatter(&mut x, &b.params, &gather(&out.x, &b.params));
        let groups = qd.groups.into_iter().map(|g| g.into_iter().map(|r| b.rows[r]).collect()).collect();
        runs.push(BlockRun { start, converged: out.converged, groups });
    }
    report.dof = lay.n - rank;

    let all_groups: Vec<Vec<usize>> = runs.iter().flat_map(|r| r.groups.iter().cloned()).collect();
    let mut redundant_rows: HashSet<usize> = HashSet::new();
    if !all_groups.is_empty() {
        let skipped = choose_skipped(&all_groups, &sys.row_con);
        for (bi, b) in blocks.iter().enumerate() {
            if runs[bi].groups.is_empty() {
                continue;
            }
            let col = b.col_map(lay.n);
            let active: Vec<bool> = b.rows.iter().map(|r| !skipped.contains(r)).collect();
            let out2 = dogleg(sys, b, &col, &runs[bi].start, &active);
            report.iterations += out2.iters;
            if !out2.converged {
                continue;
            }
            let (f2, _) = sys.eval_block(b, &col, &out2.x);
            let mut block_red = false;
            for (li, r) in b.rows.iter().enumerate() {
                if skipped.contains(r) && f2[li] * f2[li] < CONVERGENCE_REDUNDANT {
                    redundant_rows.insert(*r);
                    block_red = true;
                }
            }
            let block_conflict = runs[bi].groups.iter().any(|g| !g.iter().any(|r| redundant_rows.contains(r)));
            if !runs[bi].converged && block_red && !block_conflict {
                // dependencies were all consistent: take the reduced solution
                scatter(&mut x, &b.params, &gather(&out2.x, &b.params));
                runs[bi].converged = true;
            }
        }
        let mut conflicting: BTreeSet<usize> = BTreeSet::new();
        for g in &all_groups {
            if !g.iter().any(|r| redundant_rows.contains(r)) {
                conflicting.extend(g.iter().map(|&r| sys.row_con[r]));
            }
        }
        let mut red_tags: BTreeSet<usize> = redundant_rows.iter().map(|&r| sys.row_con[r]).collect();
        let mut partial: BTreeSet<usize> = red_tags.clone();
        for r in 0..sys.rows() {
            if !redundant_rows.contains(&r) {
                red_tags.remove(&sys.row_con[r]);
            }
        }
        for t in &red_tags {
            partial.remove(t);
        }
        report.conflicting = conflicting.into_iter().collect();
        report.redundant = red_tags.into_iter().collect();
        report.partially_redundant = partial.into_iter().collect();
    }

    if !report.conflicting.is_empty() {
        report.status = SolveStatus::Conflicting;
        return (report, None, vec![]);
    }
    if !runs.iter().all(|r| r.converged) {
        report.status = SolveStatus::Failed;
        return (report, None, vec![]);
    }
    report.status = if report.redundant.is_empty() && report.partially_redundant.is_empty() {
        SolveStatus::Solved
    } else {
        SolveStatus::Redundant
    };

    // measure reference (non-driving) dimensional constraints
    let refsys = System::new(sk, lay, x0, |c| !c.driving && c.kind.is_dimensional());
    let mut measured = Vec::new();
    for c in &refsys.cons {
        if let (Some(rows), Some(v)) = (refsys.constraint_rows(c.ci, &x), sk.constraints[c.ci].kind.value()) {
            if let Some(r0) = rows.first() {
                measured.push((c.ci, v + r0));
            }
        }
    }
    (report, Some(x), measured)
}

pub(crate) fn free_parameters(sk: &Sketch) -> Vec<Vec<bool>> {
    let lay = Layout::new(sk);
    let x0 = lay.pack(sk);
    let sys = System::new(sk, &lay, &x0, |c| c.driving);
    // parameters not touched by any constraint are free
    let mut free = vec![true; lay.n];
    for b in sys.blocks(&[]) {
        let (_, j) = sys.eval_block(&b, &b.col_map(lay.n), &x0);
        let nb = null_space(&j, 1e-9);
        for (k, &p) in b.params.iter().enumerate() {
            let s: f64 = (0..nb.ncols()).map(|c| nb[(k, c)] * nb[(k, c)]).sum();
            free[p] = s > 1e-10;
        }
    }
    (0..sk.geometry.len())
        .map(|g| (0..lay.counts[g]).map(|k| free[lay.offsets[g] + k]).collect())
        .collect()
}

pub(crate) fn fully_constrained(sk: &Sketch) -> Vec<bool> {
    free_parameters(sk).into_iter().map(|v| !v.iter().any(|&f| f)).collect()
}
