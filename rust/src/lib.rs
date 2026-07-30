use nalgebra::{DMatrix, DVector};
use numpy::{PyReadonlyArray1, PyReadonlyArray2};
use pyo3::prelude::*;

/// Convert a numpy 2D float64 array view into an nalgebra DMatrix.
fn to_dmatrix(arr: &PyReadonlyArray2<f64>) -> DMatrix<f64> {
    let a = arr.as_array();
    let nr = a.nrows();
    let nc = a.ncols();
    DMatrix::from_fn(nr, nc, |i, j| a[[i, j]])
}

/// Convert a numpy 1D float64 array view into an nalgebra DVector.
fn to_dvector(arr: &PyReadonlyArray1<f64>) -> DVector<f64> {
    let a = arr.as_array();
    let n = a.len();
    DVector::from_fn(n, |i, _| a[i])
}

/// Indices `i` where the Euclidean distance between `states[i]` and `query`
/// is <= radius.
#[pyfunction]
fn euclidean_neighbors(
    states: PyReadonlyArray2<f64>,
    query: PyReadonlyArray1<f64>,
    radius: f64,
) -> PyResult<Vec<usize>> {
    let s = states.as_array();
    let q = query.as_array();
    let n = s.nrows();
    let d = s.ncols();
    let r2 = radius * radius;
    let mut out = Vec::new();
    for i in 0..n {
        let mut acc = 0.0f64;
        for j in 0..d {
            let diff = s[[i, j]] - q[j];
            acc += diff * diff;
        }
        if acc <= r2 {
            out.push(i);
        }
    }
    Ok(out)
}

/// Solve `G d = diff` robustly. Falls back to a regularized solve and then a
/// pseudo-inverse if `G` is singular/ill-conditioned.
fn robust_solve(g: &DMatrix<f64>, diff: &DVector<f64>) -> Option<DVector<f64>> {
    let n = g.nrows();
    // Primary: LU decomposition.
    if let Some(sol) = g.clone().lu().solve(diff) {
        if sol.iter().all(|v| v.is_finite()) {
            return Some(sol);
        }
    }
    // Regularized solve.
    let reg = 1e-9;
    let mut gr = g.clone();
    for i in 0..n {
        gr[(i, i)] += reg;
    }
    if let Some(sol) = gr.clone().lu().solve(diff) {
        if sol.iter().all(|v| v.is_finite()) {
            return Some(sol);
        }
    }
    // Pseudo-inverse fallback.
    if let Ok(pinv) = g.clone().pseudo_inverse(1e-12) {
        let sol = pinv * diff;
        if sol.iter().all(|v| v.is_finite()) {
            return Some(sol);
        }
    }
    None
}

/// Cost at a fixed arrival time `t` for the linear connection problem.
fn cost_at(
    a: &DMatrix<f64>,
    q: &DMatrix<f64>,
    c: &DVector<f64>,
    x0: &DVector<f64>,
    x1: &DVector<f64>,
    t: f64,
) -> f64 {
    let n = a.nrows();

    // Drift generator: expm([[A, I],[0, 0]] * t) -> top-left = Phi, top-right = Ad.
    let mut drift = DMatrix::<f64>::zeros(2 * n, 2 * n);
    drift.view_mut((0, 0), (n, n)).copy_from(a);
    for i in 0..n {
        drift[(i, n + i)] = 1.0;
    }
    let z = (drift * t).exp();
    let phi = z.view((0, 0), (n, n)).into_owned();
    let ad = z.view((0, n), (n, n)).into_owned();
    let xbar = &phi * x0 + &ad * c;

    // Van Loan generator for the Gramian:
    // M = expm([[-A, Q],[0, A^T]] * t); G = M22^T @ M12.
    let mut gram_gen = DMatrix::<f64>::zeros(2 * n, 2 * n);
    gram_gen.view_mut((0, 0), (n, n)).copy_from(&(-a));
    gram_gen.view_mut((0, n), (n, n)).copy_from(q);
    gram_gen.view_mut((n, n), (n, n)).copy_from(&a.transpose());
    let m = (gram_gen * t).exp();
    let m12 = m.view((0, n), (n, n)).into_owned();
    let m22 = m.view((n, n), (n, n)).into_owned();
    let g = m22.transpose() * m12;

    let diff = x1 - xbar;
    match robust_solve(&g, &diff) {
        Some(d) => {
            let quad = diff.dot(&d);
            let cost = t + quad;
            if cost.is_finite() {
                cost
            } else {
                f64::INFINITY
            }
        }
        None => f64::INFINITY,
    }
}

/// Optimal-time linear connection cost, or None if not connectable.
#[pyfunction]
fn linear_cost(
    a: PyReadonlyArray2<f64>,
    q: PyReadonlyArray2<f64>,
    c: PyReadonlyArray1<f64>,
    x0: PyReadonlyArray1<f64>,
    x1: PyReadonlyArray1<f64>,
    tau_max: f64,
) -> PyResult<Option<f64>> {
    let a = to_dmatrix(&a);
    let q = to_dmatrix(&q);
    let c = to_dvector(&c);
    let x0 = to_dvector(&x0);
    let x1 = to_dvector(&x1);

    let eps = 1e-4;
    if tau_max <= eps {
        return Ok(None);
    }

    let objective = |t: f64| -> f64 {
        if t <= eps {
            return f64::INFINITY;
        }
        let cost = cost_at(&a, &q, &c, &x0, &x1, t);
        if !cost.is_finite() || cost <= 0.0 {
            f64::INFINITY
        } else {
            cost
        }
    };

    // Golden-section search over [eps, tau_max].
    let inv_phi = (5.0f64.sqrt() - 1.0) / 2.0; // 1/phi ~= 0.618
    let mut lo = eps;
    let mut hi = tau_max;
    let mut x1p = hi - inv_phi * (hi - lo);
    let mut x2p = lo + inv_phi * (hi - lo);
    let mut f1 = objective(x1p);
    let mut f2 = objective(x2p);
    for _ in 0..100 {
        if f1 <= f2 {
            hi = x2p;
            x2p = x1p;
            f2 = f1;
            x1p = hi - inv_phi * (hi - lo);
            f1 = objective(x1p);
        } else {
            lo = x1p;
            x1p = x2p;
            f1 = f2;
            x2p = lo + inv_phi * (hi - lo);
            f2 = objective(x2p);
        }
    }

    // Best over the final bracket plus the sampled endpoints.
    let candidates = [
        (x1p, f1),
        (x2p, f2),
        ((lo + hi) / 2.0, objective((lo + hi) / 2.0)),
    ];
    let mut best = f64::INFINITY;
    for &(_, f) in candidates.iter() {
        if f < best {
            best = f;
        }
    }

    if best.is_finite() {
        Ok(Some(best))
    } else {
        Ok(None)
    }
}

#[pymodule]
fn krrtstar_core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(euclidean_neighbors, m)?)?;
    m.add_function(wrap_pyfunction!(linear_cost, m)?)?;
    Ok(())
}
