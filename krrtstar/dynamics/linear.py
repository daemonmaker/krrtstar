"""Analytic linear dynamics with a general optimal-time connection.

This replaces the legacy per-model, Maple-derived closed-form solutions with a
single solver valid for *any* controllable linear system

    x_dot = A x + B u + c,     cost = integral_0^tau (1 + u^T R u) dt

read from configuration. The optimal fixed-final-state connection follows the
formulation of Webb & van den Berg, "Kinodynamic RRT*" (2013):

  * weighted controllability Gramian
        G(t) = integral_0^t e^{A s} B R^{-1} B^T e^{A^T s} ds
  * open-loop drift
        xbar(t) = e^{A t} x0 + integral_0^t e^{A s} ds . c
  * cost(t) = t + (x1 - xbar(t))^T G(t)^{-1} (x1 - xbar(t))
  * optimal arrival time tau* = argmin_t cost(t)

We compute the required matrix integrals with the matrix exponential (no
symbolic derivation, no MATLAB/Maple), which is why this generalizes.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
from scipy.linalg import expm
from scipy.optimize import minimize_scalar

from .. import accel
from .base import ControlBounds, Trajectory


class AnalyticLinearDynamics:
    """Optimal-control connection for a configurable linear system."""

    def __init__(
        self,
        A: np.ndarray,
        B: np.ndarray,
        c: Optional[np.ndarray] = None,
        R: Optional[np.ndarray] = None,
        u_bounds: Optional[np.ndarray] = None,
        tau_max: float = 20.0,
        n_traj_samples: int = 32,
        use_accel: bool = True,
    ) -> None:
        A = np.asarray(A, dtype=float)
        B = np.asarray(B, dtype=float)
        self.x_dim = A.shape[0]
        self.u_dim = B.shape[1]
        if A.shape != (self.x_dim, self.x_dim):
            raise ValueError("A must be square (x_dim x x_dim)")
        if B.shape[0] != self.x_dim:
            raise ValueError("B must have x_dim rows")

        self.A = A
        self.B = B
        self.c = np.zeros(self.x_dim) if c is None else np.asarray(c, dtype=float)
        self.R = np.eye(self.u_dim) if R is None else np.asarray(R, dtype=float)
        self.Rinv = np.linalg.inv(self.R)
        self.Q = B @ self.Rinv @ B.T  # x_dim x x_dim
        self._bounds = ControlBounds(
            None if u_bounds is None else np.asarray(u_bounds, dtype=float)
        )
        self.tau_max = float(tau_max)
        self.n_traj_samples = int(n_traj_samples)
        # Allows forcing the pure-Python reference path (used for parity tests).
        self.use_accel = bool(use_accel)

        # Precompute the augmented generator used for the drift integral
        # expm([[A, I],[0,0]] t) -> top-left = e^{A t}, top-right = int_0^t e^{A s} ds
        n = self.x_dim
        self._drift_gen = np.zeros((2 * n, 2 * n))
        self._drift_gen[:n, :n] = A
        self._drift_gen[:n, n:] = np.eye(n)

        # Van Loan generator for the Gramian:
        # expm([[-A, Q],[0, A^T]] t) = [[M11, M12],[0, M22]];
        # e^{A^T t} = M22, and G(t) = M22^T @ M12
        self._gram_gen = np.zeros((2 * n, 2 * n))
        self._gram_gen[:n, :n] = -A
        self._gram_gen[:n, n:] = self.Q
        self._gram_gen[n:, n:] = A.T

        # Reusable native connector (precomputed time-grid exponentials).
        self._connector = (
            accel.make_connector(
                self.A, self.B, self.Rinv, self.c, self.tau_max, self.n_traj_samples
            )
            if self.use_accel
            else None
        )

    # ------------------------------------------------------------------ #
    # Core matrix integrals
    # ------------------------------------------------------------------ #
    def _phi_and_drift(self, t: float):
        """Return ``(e^{A t}, xbar_offset)`` where ``xbar = phi@x0 + offset``."""
        n = self.x_dim
        Z = expm(self._drift_gen * t)
        phi = Z[:n, :n]
        integ = Z[:n, n:]  # int_0^t e^{A s} ds
        return phi, integ @ self.c

    def gramian(self, t: float) -> np.ndarray:
        """Weighted controllability Gramian ``G(t)`` via Van Loan's method."""
        n = self.x_dim
        M = expm(self._gram_gen * t)
        M12 = M[:n, n:]
        M22 = M[n:, n:]
        return M22.T @ M12

    # ------------------------------------------------------------------ #
    # Cost / connect
    # ------------------------------------------------------------------ #
    def _cost_at(self, t: float, x0: np.ndarray, x1: np.ndarray):
        """Return ``(cost, d, xbar)`` for a fixed arrival time ``t``."""
        phi, offset = self._phi_and_drift(t)
        xbar = phi @ x0 + offset
        G = self.gramian(t)
        diff = x1 - xbar
        # Solve G d = diff robustly; if uncontrollable in time t, fall back to
        # a least-squares solution and flag large residuals as infeasible.
        try:
            d = np.linalg.solve(G, diff)
        except np.linalg.LinAlgError:
            d, *_ = np.linalg.lstsq(G, diff, rcond=None)
        cost = t + float(diff @ d)
        return cost, d, xbar

    def _optimal_tau(self, x0: np.ndarray, x1: np.ndarray):
        eps = 1e-4

        def objective(t: float) -> float:
            if t <= eps:
                return np.inf
            cost, _, _ = self._cost_at(t, x0, x1)
            # Numerical guards: cost must be finite and positive.
            if not np.isfinite(cost) or cost <= 0:
                return np.inf
            return cost

        # Coarse scan to seed a robust bracket (the cost curve can have local
        # minima; a global-ish scan avoids the bounded solver getting stuck).
        grid = np.linspace(eps, self.tau_max, 48)
        vals = np.array([objective(float(t)) for t in grid])
        if not np.any(np.isfinite(vals)):
            return None
        k = int(np.argmin(vals))
        lo = grid[max(k - 1, 0)]
        hi = grid[min(k + 1, len(grid) - 1)]
        res = minimize_scalar(
            objective, bounds=(lo, hi), method="bounded",
            options={"xatol": 1e-5},
        )
        if res.success and np.isfinite(res.fun) and res.fun <= vals[k]:
            return float(res.x), float(res.fun)
        return float(grid[k]), float(vals[k])

    def cost(self, x0: np.ndarray, x1: np.ndarray) -> Optional[float]:
        x0 = np.asarray(x0, dtype=float).reshape(-1)
        x1 = np.asarray(x1, dtype=float).reshape(-1)
        # Prefer the native (Rust) solver for the hot-path cost query.
        if self._connector is not None:
            rust = self._connector.cost(x0, x1)
            if rust is not None and np.isfinite(rust):
                return float(rust)
        elif self.use_accel:
            rust = accel.linear_cost(self.A, self.Q, self.c, x0, x1, self.tau_max)
            if rust is not None and np.isfinite(rust):
                return float(rust)
        result = self._optimal_tau(x0, x1)
        if result is None:
            return None
        return result[1]

    def connect(self, x0: np.ndarray, x1: np.ndarray) -> Optional[Trajectory]:
        x0 = np.asarray(x0, dtype=float).reshape(-1)
        x1 = np.asarray(x1, dtype=float).reshape(-1)

        # Full connection (arrival time, cost, and sampled trajectory) in Rust.
        if self.use_accel:
            if self._connector is not None:
                native = accel.unpack_connect(
                    self._connector.connect(x0, x1), self.x_dim, self.u_dim
                )
            else:
                native = accel.linear_connect(
                    self.A, self.B, self.Rinv, self.c, x0, x1,
                    self.tau_max, self.n_traj_samples,
                )
            if native is not None:
                tau, cost, times, states, controls = native
                if np.isfinite(cost) and np.all(np.isfinite(states)):
                    return Trajectory(
                        tau=tau,
                        cost=cost,
                        times=times,
                        states=states,
                        controls=controls,
                        feasible=self._bounds.satisfied(controls),
                    )

        result = self._optimal_tau(x0, x1)
        if result is None:
            return None
        tau, cost = result
        return self._reconstruct(tau, cost, x0, x1)

    # ------------------------------------------------------------------ #
    # Trajectory reconstruction
    # ------------------------------------------------------------------ #
    def _reconstruct(self, tau: float, cost: float, x0: np.ndarray, x1: np.ndarray) -> Trajectory:
        """Reconstruct sampled states/controls for the optimal connection.

        The optimal control is ``u(t) = R^{-1} B^T e^{A^T (tau - t)} d`` with
        ``d = G(tau)^{-1} (x1 - xbar(tau))``. States are obtained by propagating
        the composite (state, costate) system, computed via one augmented
        matrix exponential per sample time.
        """
        n = self.x_dim
        _, d, _ = self._cost_at(tau, x0, x1)

        # Composite generator [[A, Q],[0, -A^T]] with affine drift [c; 0].
        comp = np.zeros((2 * n + 1, 2 * n + 1))
        comp[:n, :n] = self.A
        comp[:n, n : 2 * n] = self.Q
        comp[n : 2 * n, n : 2 * n] = -self.A.T
        comp[:n, 2 * n] = self.c  # affine term for state row

        y0 = expm(self.A.T * tau) @ d  # costate at t=0
        z0 = np.concatenate([x0, y0, [1.0]])

        times = np.linspace(0.0, tau, self.n_traj_samples)
        states = np.zeros((self.n_traj_samples, n))
        controls = np.zeros((self.n_traj_samples, self.u_dim))
        for i, t in enumerate(times):
            zt = expm(comp * t) @ z0
            xt = zt[:n]
            yt = zt[n : 2 * n]
            states[i] = xt
            controls[i] = self.Rinv @ self.B.T @ yt
        # Pin endpoints exactly (guards against small numerical drift).
        states[0] = x0
        states[-1] = x1

        feasible = self._bounds.satisfied(controls)
        return Trajectory(
            tau=tau,
            cost=cost,
            times=times,
            states=states,
            controls=controls,
            feasible=feasible,
        )

    # ------------------------------------------------------------------ #
    # Batched cost queries (planner ranking / filtering)
    # ------------------------------------------------------------------ #
    def cost_batch_to(self, states: np.ndarray, x1: np.ndarray) -> np.ndarray:
        """Costs from each row of ``states`` to ``x1``.

        Uses the native connector's precomputed time grid when available, which
        avoids recomputing matrix exponentials per query. Unreachable pairs come
        back as ``inf``.
        """
        states = np.ascontiguousarray(states, dtype=float)
        x1 = np.asarray(x1, dtype=float).reshape(-1)
        if self._connector is not None and states.shape[0] > 0:
            return np.asarray(self._connector.cost_batch_to(states, x1), dtype=float)
        return self._cost_loop(states, x1, reverse=False)

    def cost_batch_from(self, x0: np.ndarray, states: np.ndarray) -> np.ndarray:
        """Costs from ``x0`` to each row of ``states``."""
        states = np.ascontiguousarray(states, dtype=float)
        x0 = np.asarray(x0, dtype=float).reshape(-1)
        if self._connector is not None and states.shape[0] > 0:
            return np.asarray(self._connector.cost_batch_from(x0, states), dtype=float)
        return self._cost_loop(states, x0, reverse=True)

    def _cost_loop(self, states: np.ndarray, other: np.ndarray, reverse: bool) -> np.ndarray:
        out = np.full(states.shape[0], np.inf)
        for i, s in enumerate(states):
            c = self.cost(other, s) if reverse else self.cost(s, other)
            if c is not None and np.isfinite(c):
                out[i] = c
        return out

    def u_bounds(self) -> Optional[np.ndarray]:
        return self._bounds.bounds
