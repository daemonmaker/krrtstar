//! Native hot-path primitives for the krrtstar planner.
//!
//! Provides the optimal-time connection for linear systems
//!     x_dot = A x + B u + c,   cost = integral_0^tau (1 + u^T R u) dt
//! including full trajectory reconstruction, plus a Euclidean neighbour
//! pre-filter.
//!
//! The key performance property exploited here: the matrix integrals
//! `Phi(t) = e^{A t}`, `Ad(t) = int_0^t e^{A s} ds` and the weighted
//! controllability Gramian `G(t)` depend only on `(A, Q, t)` -- never on the
//! endpoint states. `LinearConnector` therefore precomputes them once on a
//! fixed time grid and reuses them for every cost query, which turns each
//! query into a handful of matrix-vector products.

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
    if let Some(sol) = g.clone().lu().solve(diff) {
        if sol.iter().all(|v| v.is_finite()) {
            return Some(sol);
        }
    }
    let reg = 1e-9;
    let mut gr = g.clone();
    for i in 0..n {
        gr[(i, i)] += reg;
    }
    if let Some(sol) = gr.lu().solve(diff) {
        if sol.iter().all(|v| v.is_finite()) {
            return Some(sol);
        }
    }
    if let Ok(pinv) = g.clone().pseudo_inverse(1e-12) {
        let sol = pinv * diff;
        if sol.iter().all(|v| v.is_finite()) {
            return Some(sol);
        }
    }
    None
}

/// `Phi(t)` and `Ad(t) = int_0^t e^{A s} ds` via one augmented exponential.
fn phi_and_ad(a: &DMatrix<f64>, t: f64) -> (DMatrix<f64>, DMatrix<f64>) {
    let n = a.nrows();
    let mut drift = DMatrix::<f64>::zeros(2 * n, 2 * n);
    drift.view_mut((0, 0), (n, n)).copy_from(a);
    for i in 0..n {
        drift[(i, n + i)] = 1.0;
    }
    let z = (drift * t).exp();
    (
        z.view((0, 0), (n, n)).into_owned(),
        z.view((0, n), (n, n)).into_owned(),
    )
}

/// Weighted controllability Gramian `G(t)` via Van Loan's method:
/// `M = expm([[-A, Q],[0, A^T]] t)`, then `G = M22^T M12`.
fn gramian(a: &DMatrix<f64>, q: &DMatrix<f64>, t: f64) -> DMatrix<f64> {
    let n = a.nrows();
    let mut gen = DMatrix::<f64>::zeros(2 * n, 2 * n);
    gen.view_mut((0, 0), (n, n)).copy_from(&(-a));
    gen.view_mut((0, n), (n, n)).copy_from(q);
    gen.view_mut((n, n), (n, n)).copy_from(&a.transpose());
    let m = (gen * t).exp();
    let m12 = m.view((0, n), (n, n)).into_owned();
    let m22 = m.view((n, n), (n, n)).into_owned();
    m22.transpose() * m12
}

/// Cost at a fixed arrival time `t`, together with the costate vector `d`.
///
/// `d = G(t)^{-1} (x1 - xbar(t))` is needed both for the cost and to
/// reconstruct the optimal control, so it is returned to avoid recomputation.
fn cost_and_costate(
    a: &DMatrix<f64>,
    q: &DMatrix<f64>,
    c: &DVector<f64>,
    x0: &DVector<f64>,
    x1: &DVector<f64>,
    t: f64,
) -> (f64, Option<DVector<f64>>) {
    let (phi, ad) = phi_and_ad(a, t);
    let xbar = &phi * x0 + &ad * c;
    let g = gramian(a, q, t);
    let diff = x1 - xbar;
    match robust_solve(&g, &diff) {
        Some(d) => {
            let cost = t + diff.dot(&d);
            if cost.is_finite() {
                (cost, Some(d))
            } else {
                (f64::INFINITY, None)
            }
        }
        None => (f64::INFINITY, None),
    }
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
    cost_and_costate(a, q, c, x0, x1, t).0
}

/// Find the optimal arrival time by a coarse global scan followed by a
/// golden-section refinement inside the best bracket.
///
/// The coarse scan guards against the cost curve's local minima (a pure
/// golden-section search assumes unimodality, which does not always hold).
fn optimal_tau(
    a: &DMatrix<f64>,
    q: &DMatrix<f64>,
    c: &DVector<f64>,
    x0: &DVector<f64>,
    x1: &DVector<f64>,
    tau_max: f64,
) -> Option<(f64, f64)> {
    let eps = 1e-4;
    if tau_max <= eps {
        return None;
    }

    let objective = |t: f64| -> f64 {
        if t <= eps {
            return f64::INFINITY;
        }
        let cost = cost_at(a, q, c, x0, x1, t);
        if !cost.is_finite() || cost <= 0.0 {
            f64::INFINITY
        } else {
            cost
        }
    };

    let grid_n = 48usize;
    let mut best_t = f64::NAN;
    let mut best_f = f64::INFINITY;
    let mut grid = Vec::with_capacity(grid_n);
    for k in 0..grid_n {
        let t = eps + (tau_max - eps) * (k as f64) / ((grid_n - 1) as f64);
        let f = objective(t);
        grid.push((t, f));
        if f < best_f {
            best_f = f;
            best_t = t;
        }
    }
    if !best_f.is_finite() {
        return None;
    }

    let k_best = grid.iter().position(|&(t, _)| t == best_t).unwrap_or(0);
    let mut lo = grid[k_best.saturating_sub(1)].0;
    let mut hi = grid[(k_best + 1).min(grid_n - 1)].0;
    if hi <= lo {
        return Some((best_t, best_f));
    }

    refine_golden(&objective, lo, hi, &mut best_t, &mut best_f);
    let _ = (&mut lo, &mut hi);
    if best_f.is_finite() {
        Some((best_t, best_f))
    } else {
        None
    }
}

/// Golden-section refinement of `objective` on `[lo, hi]`, updating the best
/// point found so far.
fn refine_golden<F: Fn(f64) -> f64>(
    objective: &F,
    mut lo: f64,
    mut hi: f64,
    best_t: &mut f64,
    best_f: &mut f64,
) {
    let inv_phi = (5.0f64.sqrt() - 1.0) / 2.0; // 1/phi ~= 0.618
    let mut p1 = hi - inv_phi * (hi - lo);
    let mut p2 = lo + inv_phi * (hi - lo);
    let mut f1 = objective(p1);
    let mut f2 = objective(p2);
    for _ in 0..80 {
        if f1 <= f2 {
            hi = p2;
            p2 = p1;
            f2 = f1;
            p1 = hi - inv_phi * (hi - lo);
            f1 = objective(p1);
        } else {
            lo = p1;
            p1 = p2;
            f1 = f2;
            p2 = lo + inv_phi * (hi - lo);
            f2 = objective(p2);
        }
        if (hi - lo).abs() < 1e-7 {
            break;
        }
    }
    for &(t, f) in [(p1, f1), (p2, f2)].iter() {
        if f < *best_f {
            *best_f = f;
            *best_t = t;
        }
    }
}

/// Reconstruct the sampled optimal state/control trajectory for a known
/// arrival time.
///
/// The optimal control is `u(t) = R^{-1} B^T y(t)` with costate
/// `y(t) = e^{A^T (tau - t)} d`. States and costates are propagated together
/// through the composite system
///     d/dt [x; y] = [[A, Q],[0, -A^T]] [x; y] + [c; 0],
/// embedded in an affine `(2n+1)` generator. Because the sample times are
/// uniform, a single matrix exponential of the per-step generator is computed
/// once and applied repeatedly, instead of one exponential per sample.
#[allow(clippy::too_many_arguments)]
fn reconstruct(
    a: &DMatrix<f64>,
    b: &DMatrix<f64>,
    rinv: &DMatrix<f64>,
    q: &DMatrix<f64>,
    c: &DVector<f64>,
    x0: &DVector<f64>,
    x1: &DVector<f64>,
    tau: f64,
    n_samples: usize,
) -> Option<(Vec<f64>, Vec<f64>, Vec<f64>)> {
    let n = a.nrows();
    let u_dim = b.ncols();
    let samples = n_samples.max(2);

    let d = cost_and_costate(a, q, c, x0, x1, tau).1?;

    let dim = 2 * n + 1;
    let mut comp = DMatrix::<f64>::zeros(dim, dim);
    comp.view_mut((0, 0), (n, n)).copy_from(a);
    comp.view_mut((0, n), (n, n)).copy_from(q);
    comp.view_mut((n, n), (n, n)).copy_from(&(-a.transpose()));
    for i in 0..n {
        comp[(i, 2 * n)] = c[i];
    }

    let y0 = (a.transpose() * tau).exp() * &d;

    let mut z = DVector::<f64>::zeros(dim);
    for i in 0..n {
        z[i] = x0[i];
        z[n + i] = y0[i];
    }
    z[2 * n] = 1.0;

    let dt = tau / ((samples - 1) as f64);
    let step = (comp * dt).exp();
    let rinv_bt = rinv * b.transpose();

    let mut times = Vec::with_capacity(samples);
    let mut states = Vec::with_capacity(samples * n);
    let mut controls = Vec::with_capacity(samples * u_dim);

    for k in 0..samples {
        times.push(dt * (k as f64));
        for i in 0..n {
            states.push(z[i]);
        }
        let y = DVector::from_fn(n, |i, _| z[n + i]);
        let u = &rinv_bt * y;
        for j in 0..u_dim {
            controls.push(u[j]);
        }
        if k + 1 < samples {
            z = &step * &z;
        }
    }

    // Pin endpoints exactly (guards against small numerical drift).
    for i in 0..n {
        states[i] = x0[i];
        states[(samples - 1) * n + i] = x1[i];
    }
    if let Some(last) = times.last_mut() {
        *last = tau;
    }

    Some((times, states, controls))
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
    Ok(optimal_tau(&a, &q, &c, &x0, &x1, tau_max).map(|(_, cost)| cost))
}

/// Full optimal connection: arrival time, cost, and the sampled optimal
/// state/control trajectory.
///
/// Returns `(tau, cost, times, states_flat, controls_flat)` where the flat
/// vectors are row-major with shapes `(n_samples, x_dim)` and
/// `(n_samples, u_dim)`. Returns `None` if the states cannot be connected.
#[pyfunction]
#[allow(clippy::too_many_arguments)]
fn linear_connect(
    a: PyReadonlyArray2<f64>,
    b: PyReadonlyArray2<f64>,
    rinv: PyReadonlyArray2<f64>,
    c: PyReadonlyArray1<f64>,
    x0: PyReadonlyArray1<f64>,
    x1: PyReadonlyArray1<f64>,
    tau_max: f64,
    n_samples: usize,
) -> PyResult<Option<(f64, f64, Vec<f64>, Vec<f64>, Vec<f64>)>> {
    let a = to_dmatrix(&a);
    let b = to_dmatrix(&b);
    let rinv = to_dmatrix(&rinv);
    let c = to_dvector(&c);
    let x0 = to_dvector(&x0);
    let x1 = to_dvector(&x1);
    let q = &b * &rinv * b.transpose();

    let (tau, cost) = match optimal_tau(&a, &q, &c, &x0, &x1, tau_max) {
        Some(v) => v,
        None => return Ok(None),
    };
    match reconstruct(&a, &b, &rinv, &q, &c, &x0, &x1, tau, n_samples) {
        Some((times, states, controls)) => Ok(Some((tau, cost, times, states, controls))),
        None => Ok(None),
    }
}

/// Precomputed per-time-step matrices shared by all cost queries.
struct GridPoint {
    t: f64,
    phi: DMatrix<f64>,
    ad_c: DVector<f64>,
    ginv: DMatrix<f64>,
}

/// Regularized inverse; falls back to a pseudo-inverse.
///
/// A small relative regularization makes unreachable directions produce very
/// large costs (so they are filtered out) rather than the artificially small
/// cost a bare pseudo-inverse would report.
fn reg_inverse(g: &DMatrix<f64>) -> Option<DMatrix<f64>> {
    let n = g.nrows();
    let scale = g.trace().abs().max(1.0);
    let mut gr = g.clone();
    for i in 0..n {
        gr[(i, i)] += 1e-12 * scale;
    }
    if let Some(inv) = gr.try_inverse() {
        if inv.iter().all(|v| v.is_finite()) {
            return Some(inv);
        }
    }
    g.clone().pseudo_inverse(1e-12).ok()
}

/// A reusable connector for one linear system.
///
/// Holds the system matrices plus a precomputed time grid, so batched cost
/// queries avoid recomputing any matrix exponentials.
#[pyclass]
struct LinearConnector {
    a: DMatrix<f64>,
    b: DMatrix<f64>,
    rinv: DMatrix<f64>,
    q: DMatrix<f64>,
    c: DVector<f64>,
    tau_max: f64,
    n_samples: usize,
    grid: Vec<GridPoint>,
}

#[pymethods]
impl LinearConnector {
    #[new]
    #[pyo3(signature = (a, b, rinv, c, tau_max, n_samples, grid_n = 64))]
    fn new(
        a: PyReadonlyArray2<f64>,
        b: PyReadonlyArray2<f64>,
        rinv: PyReadonlyArray2<f64>,
        c: PyReadonlyArray1<f64>,
        tau_max: f64,
        n_samples: usize,
        grid_n: usize,
    ) -> PyResult<Self> {
        let a = to_dmatrix(&a);
        let b = to_dmatrix(&b);
        let rinv = to_dmatrix(&rinv);
        let c = to_dvector(&c);
        let q = &b * &rinv * b.transpose();

        let eps = 1e-4;
        let grid_n = grid_n.max(2);
        let mut grid = Vec::with_capacity(grid_n);
        for k in 0..grid_n {
            let t = eps + (tau_max - eps) * (k as f64) / ((grid_n - 1) as f64);
            let (phi, ad) = phi_and_ad(&a, t);
            let g = gramian(&a, &q, t);
            if let Some(ginv) = reg_inverse(&g) {
                grid.push(GridPoint {
                    t,
                    phi,
                    ad_c: ad * &c,
                    ginv,
                });
            }
        }

        Ok(Self {
            a,
            b,
            rinv,
            q,
            c,
            tau_max,
            n_samples,
            grid,
        })
    }

    /// Grid-approximated optimal cost from each row of `states` to `x1`.
    ///
    /// Returns one cost per row (`f64::INFINITY` when not connectable). This is
    /// the planner's ranking/filtering query: it reuses the precomputed grid,
    /// so no matrix exponentials are taken here.
    fn cost_batch_to(
        &self,
        states: PyReadonlyArray2<f64>,
        x1: PyReadonlyArray1<f64>,
    ) -> PyResult<Vec<f64>> {
        let s = states.as_array();
        let x1 = to_dvector(&x1);
        let n = self.a.nrows();
        let rows = s.nrows();
        let mut out = Vec::with_capacity(rows);

        for i in 0..rows {
            let x0 = DVector::from_fn(n, |j, _| s[[i, j]]);
            let mut best = f64::INFINITY;
            for gp in &self.grid {
                let xbar = &gp.phi * &x0 + &gp.ad_c;
                let diff = &x1 - xbar;
                let d = &gp.ginv * &diff;
                let cost = gp.t + diff.dot(&d);
                if cost.is_finite() && cost > 0.0 && cost < best {
                    best = cost;
                }
            }
            out.push(best);
        }
        Ok(out)
    }

    /// Grid-approximated optimal cost from `x0` to each row of `states`.
    ///
    /// The drift `xbar(t)` depends only on `x0`, so it is computed once per
    /// grid point and reused across all targets.
    fn cost_batch_from(
        &self,
        x0: PyReadonlyArray1<f64>,
        states: PyReadonlyArray2<f64>,
    ) -> PyResult<Vec<f64>> {
        let x0 = to_dvector(&x0);
        let s = states.as_array();
        let n = self.a.nrows();
        let rows = s.nrows();
        let mut out = vec![f64::INFINITY; rows];

        for gp in &self.grid {
            let xbar = &gp.phi * &x0 + &gp.ad_c;
            for i in 0..rows {
                let diff = DVector::from_fn(n, |j, _| s[[i, j]] - xbar[j]);
                let d = &gp.ginv * &diff;
                let cost = gp.t + diff.dot(&d);
                if cost.is_finite() && cost > 0.0 && cost < out[i] {
                    out[i] = cost;
                }
            }
        }
        Ok(out)
    }

    /// Refined optimal cost for a single pair (grid scan + golden section).
    fn cost(&self, x0: PyReadonlyArray1<f64>, x1: PyReadonlyArray1<f64>) -> PyResult<Option<f64>> {
        let x0 = to_dvector(&x0);
        let x1 = to_dvector(&x1);
        Ok(optimal_tau(&self.a, &self.q, &self.c, &x0, &x1, self.tau_max).map(|(_, cost)| cost))
    }

    /// Full connection with trajectory reconstruction.
    fn connect(
        &self,
        x0: PyReadonlyArray1<f64>,
        x1: PyReadonlyArray1<f64>,
    ) -> PyResult<Option<(f64, f64, Vec<f64>, Vec<f64>, Vec<f64>)>> {
        let x0 = to_dvector(&x0);
        let x1 = to_dvector(&x1);
        let (tau, cost) =
            match optimal_tau(&self.a, &self.q, &self.c, &x0, &x1, self.tau_max) {
                Some(v) => v,
                None => return Ok(None),
            };
        match reconstruct(
            &self.a, &self.b, &self.rinv, &self.q, &self.c, &x0, &x1, tau, self.n_samples,
        ) {
            Some((times, states, controls)) => Ok(Some((tau, cost, times, states, controls))),
            None => Ok(None),
        }
    }
}

#[pymodule]
fn krrtstar_core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(euclidean_neighbors, m)?)?;
    m.add_function(wrap_pyfunction!(linear_cost, m)?)?;
    m.add_function(wrap_pyfunction!(linear_connect, m)?)?;
    m.add_class::<LinearConnector>()?;
    Ok(())
}
