//! Rank-revealing diagnosis, after FreeCAD's `GCS::System::diagnose` (dense QR path).

use nalgebra::DMatrix;
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

/// Relative pivot threshold for the rank decision.
/// (FreeCAD uses 1e-13 with Eigen; we solve to ~1e-12 residuals and use a slightly
/// looser threshold so that geometric dependencies at the solution are recognised.)
pub(crate) const PIVOT_THRESHOLD: f64 = 1e-10;

pub(crate) struct QrDiag {
    pub rank: usize,
    /// Conflict groups, as solver-row indices.
    pub groups: Vec<Vec<usize>>,
}

/// Full-pivot Householder QR of J^T (params x rows), mimicking Eigen's
/// `FullPivHouseholderQR` pivot order, then FreeCAD's grouping of dependent rows.
pub(crate) fn qr_diagnose(j: &DMatrix<f64>) -> QrDiag {
    let m = j.nrows(); // solver constraints = columns of J^T
    let n = j.ncols(); // params = rows of J^T
    if m == 0 {
        return QrDiag { rank: 0, groups: vec![] };
    }
    let mut a = j.transpose(); // n x m
    let size = n.min(m);
    let mut colperm: Vec<usize> = (0..m).collect();
    let mut maxpivot = 0.0f64;
    let mut nonzero = size;
    {
        let data = a.as_mut_slice(); // column-major, n rows
        // per-column (max |a| over rows >= k, first row attaining it)
        let colmax_of = |data: &[f64], jj: usize, k: usize| -> (f64, usize) {
            let mut best = -1.0;
            let mut row = k;
            for (t, val) in data[jj * n + k..jj * n + n].iter().enumerate() {
                if val.abs() > best {
                    best = val.abs();
                    row = k + t;
                }
            }
            (best, row)
        };
        let mut colmax: Vec<(f64, usize)> = (0..m).map(|jj| colmax_of(data, jj, 0)).collect();
        let mut v: Vec<f64> = vec![0.0; n];
        let mut vnz: Vec<usize> = Vec::with_capacity(n);
        let mut dirty = vec![false; m];
        for k in 0..size {
            // biggest |a| in the bottom-right corner, column-major order, first max wins
            // (same pivot choice as Eigen's FullPivHouseholderQR)
            let mut best = -1.0;
            let (mut bi, mut bj) = (k, k);
            for (jj, cm) in colmax.iter().enumerate().skip(k) {
                if cm.0 > best {
                    best = cm.0;
                    bi = cm.1;
                    bj = jj;
                }
            }
            if k == 0 {
                maxpivot = best;
            }
            if best <= f64::EPSILON * maxpivot * (size as f64) || best == 0.0 {
                nonzero = k;
                break;
            }
            if bj != k {
                for ii in 0..n {
                    data.swap(k * n + ii, bj * n + ii);
                }
                colperm.swap(k, bj);
                colmax.swap(k, bj);
            }
            if bi != k {
                for jj in k..m {
                    data.swap(jj * n + k, jj * n + bi);
                }
            }
            // Householder reflection on column k, rows k..n (sparse-aware)
            let ck = &data[k * n + k..k * n + n];
            let norm = ck.iter().map(|e| e * e).sum::<f64>().sqrt();
            let alpha = ck[0];
            let beta = if alpha >= 0.0 { -norm } else { norm };
            let vk = &mut v[..n - k];
            vk.copy_from_slice(ck);
            vk[0] = alpha - beta;
            vnz.clear();
            vnz.extend((0..n - k).filter(|&t| vk[t] != 0.0));
            let vn2: f64 = vnz.iter().map(|&t| vk[t] * vk[t]).sum();
            if vn2 > 0.0 {
                for jj in (k + 1)..m {
                    let colj = &mut data[jj * n + k..jj * n + n];
                    let sdot: f64 = vnz.iter().map(|&t| colj[t] * vk[t]).sum();
                    if sdot != 0.0 {
                        let fct = 2.0 * sdot / vn2;
                        for &t in vnz.iter() {
                            colj[t] -= fct * vk[t];
                        }
                        dirty[jj] = true;
                    }
                }
            }
            data[k * n + k] = beta;
            for ii in (k + 1)..n {
                data[k * n + ii] = 0.0;
            }
            // refresh column maxima for rows >= k+1
            for jj in (k + 1)..m {
                let (val, row) = colmax[jj];
                let need = dirty[jj] || row == k || row == bi || (bi > k && data[jj * n + bi].abs() >= val);
                if need {
                    colmax[jj] = if k + 1 < n { colmax_of(data, jj, k + 1) } else { (0.0, k + 1) };
                    dirty[jj] = false;
                }
            }
        }
    }
    let thr = PIVOT_THRESHOLD * maxpivot.abs();
    let rank = (0..nonzero).filter(|&k| a[(k, k)].abs() > thr).count();
    if rank >= m {
        return QrDiag { rank, groups: vec![] };
    }
    // R = upper triangle, rows 0..size
    let mut r = DMatrix::zeros(size, m);
    for i in 0..size {
        for jj in i..m {
            r[(i, jj)] = a[(i, jj)];
        }
    }
    // eliminateNonZerosOverPivotInUpperTriangularMatrix
    for i in 1..rank {
        for row in 0..i {
            if r[(row, i)].abs() > 1e-10 {
                let coef = r[(row, i)] / r[(i, i)];
                for c in (i + 1)..m {
                    let t = r[(i, c)];
                    r[(row, c)] -= coef * t;
                }
            }
            r[(row, i)] = 0.0;
        }
    }
    let mut groups = Vec::with_capacity(m - rank);
    for jj in rank..m {
        let mut g = Vec::new();
        for row in 0..rank {
            if r[(row, jj)].abs() > 1e-10 {
                g.push(colperm[row]);
            }
        }
        g.push(colperm[jj]);
        groups.push(g);
    }
    QrDiag { rank, groups }
}

/// FreeCAD's popularity contest: choose solver rows to drop so that every conflict
/// group loses at least one member. Rows of the same sketcher constraint that are in
/// unsatisfied groups are dropped together.
pub(crate) fn choose_skipped(groups: &[Vec<usize>], row_con: &[usize]) -> HashSet<usize> {
    let mut mult: HashMap<usize, usize> = HashMap::new();
    for &c in row_con {
        *mult.entry(c).or_default() += 1;
    }
    let mut skipped = HashSet::new();
    let mut satisfied: BTreeSet<usize> = BTreeSet::new();
    loop {
        let mut map: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
        for (gi, g) in groups.iter().enumerate() {
            if satisfied.contains(&gi) {
                continue;
            }
            for &row in g {
                map.entry(row).or_default().insert(gi);
            }
        }
        if map.is_empty() {
            break;
        }
        // max_element with FreeCAD's ordering
        let mut best: Option<(usize, usize, usize, usize)> = None; // (row, popularity, mult, tag)
        for (&row, set) in &map {
            let tag = row_con[row];
            let cand = (row, set.len(), mult[&tag], tag);
            best = match best {
                None => Some(cand),
                Some(b) => {
                    let less = cand.1 > b.1 || (cand.1 == b.1 && cand.2 < b.2) || (cand.1 == b.1 && cand.2 == b.2 && cand.3 > b.3);
                    if less {
                        Some(cand)
                    } else {
                        Some(b)
                    }
                }
            };
        }
        let tag = best.unwrap().3;
        for (&row, set) in &map {
            if row_con[row] == tag {
                skipped.insert(row);
                satisfied.extend(set.iter().copied());
            }
        }
    }
    skipped
}
