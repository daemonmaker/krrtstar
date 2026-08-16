"""Nonlinear systems connected through a local linearization.

The legacy C++ planner had no nonlinear solver. For its ``NONHOLONOMIC`` car it
instead rebuilt the linear system about each freshly drawn sample -- see the
``x_rand[3]*sth`` block of ``rrtstar()`` in ``src/dynamicrrt.cpp`` -- and then
reused the closed-form linear optimal-control connection. This module ports that
strategy in a general form: a :class:`NonlinearSystem` supplies ``f`` and its
Jacobians, and :class:`LinearizedDynamics` turns those into a
:class:`~krrtstar.dynamics.base.Dynamics` backend by delegating to
:class:`~krrtstar.dynamics.linear.AnalyticLinearDynamics`.
"""

from __future__ import annotations

from collections import OrderedDict
from typing import Optional, Protocol, Tuple, runtime_checkable

import numpy as np

from .base import Trajectory
from .linear import AnalyticLinearDynamics


@runtime_checkable
class NonlinearSystem(Protocol):
    """A continuous-time system ``x_dot = f(x, u)`` with known Jacobians."""

    x_dim: int
    u_dim: int

    def f(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """State derivative at ``(x, u)``, shape ``(x_dim,)``."""
        ...

    def jacobians(self, x: np.ndarray, u: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """``(A, B) = (df/dx, df/du)`` evaluated at ``(x, u)``."""
        ...


def numerical_jacobians(
    system: NonlinearSystem,
    x: np.ndarray,
    u: np.ndarray,
    eps: float = 1e-6,
) -> Tuple[np.ndarray, np.ndarray]:
    """Central-difference ``(A, B)`` for ``system`` at ``(x, u)``.

    Only intended for verifying analytic Jacobians: the production path always
    uses :meth:`NonlinearSystem.jacobians`.
    """
    x = np.asarray(x, dtype=float).reshape(-1)
    u = np.asarray(u, dtype=float).reshape(-1)
    A = np.zeros((system.x_dim, system.x_dim))
    B = np.zeros((system.x_dim, system.u_dim))
    for j in range(system.x_dim):
        step = np.zeros_like(x)
        step[j] = eps
        A[:, j] = (system.f(x + step, u) - system.f(x - step, u)) / (2.0 * eps)
    for j in range(system.u_dim):
        step = np.zeros_like(u)
        step[j] = eps
        B[:, j] = (system.f(x, u + step) - system.f(x, u - step)) / (2.0 * eps)
    return A, B


class NonholonomicCar:
    """The original's ``NONHOLONOMIC`` car.

    State ``x = [px, py, theta, v, kappa]`` (position, heading, speed,
    curvature) and control ``u = [a, kappa_dot]``::

        px_dot    = v * cos(theta)
        py_dot    = v * sin(theta)
        theta_dot = v * kappa
        v_dot     = u[0]
        kappa_dot = u[1]

    which reproduces the original's ``B(3,0) = 1``, ``B(4,1) = 1`` and its
    state-dependent ``A``. Note that the heading is uncontrollable at ``v = 0``
    (``theta_dot`` and its Jacobian row both vanish), which is why the original
    kept the sampled speed away from zero.
    """

    x_dim = 5
    u_dim = 2

    #: Control penalty from the original (``control_penalty = 1``,
    #: ``control_penalty1 = 50``): steering is 50x more expensive than throttle.
    default_R = np.diag([1.0, 50.0])

    def f(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=float).reshape(-1)
        u = np.asarray(u, dtype=float).reshape(-1)
        theta, v, kappa = x[2], x[3], x[4]
        return np.array(
            [v * np.cos(theta), v * np.sin(theta), v * kappa, u[0], u[1]],
            dtype=float,
        )

    def jacobians(self, x: np.ndarray, u: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        x = np.asarray(x, dtype=float).reshape(-1)
        theta, v, kappa = x[2], x[3], x[4]
        cth, sth = np.cos(theta), np.sin(theta)
        A = np.zeros((5, 5))
        A[0, 2] = -v * sth
        A[0, 3] = cth
        A[1, 2] = v * cth
        A[1, 3] = sth
        A[2, 3] = kappa
        A[2, 4] = v
        B = np.zeros((5, 2))
        B[3, 0] = 1.0
        B[4, 1] = 1.0
        return A, B


class LinearizedDynamics:
    """Connect states of a nonlinear system through a local linearization.

    Every connection from ``x0`` linearizes the system about ``x0`` with
    ``u = 0`` (this is what the original did -- it rebuilt ``A`` from the sample
    state) and solves the resulting affine linear problem exactly::

        A, B = system.jacobians(x0, 0)
        c     = f(x0, 0) - A @ x0        # so A x + c matches f to first order
        x_dot = A x + B u + c

    **This is an approximation.** The returned trajectory satisfies the
    *linearized* dynamics, not the true ones: it starts at ``x0`` and ends
    exactly at ``x1`` for the linear model, but replaying its controls through
    the real system only approximately reaches ``x1``. The deviation grows with
    ``tau`` and with the distance from the linearization point, so costs are
    likewise only approximate. Use :meth:`simulate` to measure the deviation for
    a given connection, and ``max_endpoint_error`` to reject connections whose
    true-dynamics endpoint drifts too far -- a cheap way to keep the planner
    honest. (The original policed the same problem with hard gates instead: it
    refused connections whose heading differed by more than ``pi/4`` or whose
    endpoints were more than 20 units apart.)

    Batched cost queries (``cost_batch_to`` / ``cost_batch_from``) are
    deliberately **not** provided. Those exist so a single linear system can
    amortize its time-grid matrix exponentials across many candidate pairs, but
    here every candidate ``x0`` needs its own linearization, so there is nothing
    to share. The planner detects their absence and falls back to per-pair
    :meth:`cost` calls (see ``_batch_costs`` in :mod:`krrtstar.planner`).
    """

    def __init__(
        self,
        system: NonlinearSystem,
        R: Optional[np.ndarray] = None,
        u_bounds: Optional[np.ndarray] = None,
        tau_max: float = 20.0,
        n_traj_samples: int = 32,
        use_accel: bool = True,
        max_endpoint_error: Optional[float] = None,
        cache_size: int = 512,
    ) -> None:
        self.system = system
        self.x_dim = int(system.x_dim)
        self.u_dim = int(system.u_dim)
        if R is None:
            R = getattr(system, "default_R", None)
        self.R = None if R is None else np.asarray(R, dtype=float)
        self._u_bounds = None if u_bounds is None else np.asarray(u_bounds, dtype=float)
        self.tau_max = float(tau_max)
        self.n_traj_samples = int(n_traj_samples)
        self.use_accel = bool(use_accel)
        self.max_endpoint_error = (
            None if max_endpoint_error is None else float(max_endpoint_error)
        )
        self.cache_size = max(int(cache_size), 1)
        # LRU of linearizations keyed by the exact bytes of the linearization
        # point. Planner queries reuse tree-node states verbatim, so identical
        # keys recur often; distinct states always get their own linearization.
        self._cache: "OrderedDict[bytes, AnalyticLinearDynamics]" = OrderedDict()

    # ------------------------------------------------------------------ #
    # Linearization
    # ------------------------------------------------------------------ #
    def linearize(self, x0: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return ``(A, B, c)`` for the first-order model about ``x0``."""
        x0 = np.asarray(x0, dtype=float).reshape(-1)
        u0 = np.zeros(self.u_dim)
        A, B = self.system.jacobians(x0, u0)
        A = np.asarray(A, dtype=float)
        B = np.asarray(B, dtype=float)
        c = np.asarray(self.system.f(x0, u0), dtype=float) - A @ x0
        return A, B, c

    def linear_model(self, x0: np.ndarray) -> AnalyticLinearDynamics:
        """The :class:`AnalyticLinearDynamics` linearized about ``x0``."""
        x0 = np.ascontiguousarray(x0, dtype=float).reshape(-1)
        key = x0.tobytes()
        cached = self._cache.get(key)
        if cached is not None:
            self._cache.move_to_end(key)
            return cached
        A, B, c = self.linearize(x0)
        model = AnalyticLinearDynamics(
            A=A,
            B=B,
            c=c,
            R=self.R,
            u_bounds=self._u_bounds,
            tau_max=self.tau_max,
            n_traj_samples=self.n_traj_samples,
            use_accel=self.use_accel,
        )
        self._cache[key] = model
        if len(self._cache) > self.cache_size:
            self._cache.popitem(last=False)
        return model

    # ------------------------------------------------------------------ #
    # Dynamics interface
    # ------------------------------------------------------------------ #
    def cost(self, x0: np.ndarray, x1: np.ndarray) -> Optional[float]:
        """Cost of the linearized connection (an approximation, see the class docs)."""
        return self.linear_model(x0).cost(x0, x1)

    def connect(self, x0: np.ndarray, x1: np.ndarray) -> Optional[Trajectory]:
        traj = self.linear_model(x0).connect(x0, x1)
        if traj is None:
            return None
        if self.max_endpoint_error is not None:
            if self.endpoint_error(traj) > self.max_endpoint_error:
                return None
        return traj

    def u_bounds(self) -> Optional[np.ndarray]:
        return self._u_bounds

    # ------------------------------------------------------------------ #
    # True-dynamics simulation (approximation diagnostics)
    # ------------------------------------------------------------------ #
    def simulate(
        self,
        x0: np.ndarray,
        controls: np.ndarray,
        times: np.ndarray,
        substeps: int = 4,
    ) -> np.ndarray:
        """Forward-integrate the TRUE nonlinear dynamics with RK4.

        ``controls`` are the samples at ``times`` (as returned by
        :meth:`connect`) and are interpolated linearly in between. Returns the
        ``(len(times), x_dim)`` states of the real system, whose last row can be
        compared against the requested target to quantify the linearization
        error.
        """
        x0 = np.asarray(x0, dtype=float).reshape(-1)
        times = np.asarray(times, dtype=float).reshape(-1)
        controls = np.asarray(controls, dtype=float).reshape(len(times), self.u_dim)
        states = np.empty((len(times), self.x_dim))
        if len(times) == 0:
            return states
        x = x0.copy()
        states[0] = x
        for i in range(len(times) - 1):
            h = (times[i + 1] - times[i]) / substeps
            t = times[i]
            for _ in range(substeps):
                x = self._rk4_step(x, t, h, times, controls)
                t += h
            states[i + 1] = x
        return states

    def _control_at(self, t: float, times: np.ndarray, controls: np.ndarray) -> np.ndarray:
        return np.array(
            [np.interp(t, times, controls[:, j]) for j in range(self.u_dim)],
            dtype=float,
        )

    def _rk4_step(
        self, x: np.ndarray, t: float, h: float, times: np.ndarray, controls: np.ndarray
    ) -> np.ndarray:
        f = self.system.f
        u0 = self._control_at(t, times, controls)
        uh = self._control_at(t + 0.5 * h, times, controls)
        u1 = self._control_at(t + h, times, controls)
        k1 = np.asarray(f(x, u0), dtype=float)
        k2 = np.asarray(f(x + 0.5 * h * k1, uh), dtype=float)
        k3 = np.asarray(f(x + 0.5 * h * k2, uh), dtype=float)
        k4 = np.asarray(f(x + h * k3, u1), dtype=float)
        return x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def endpoint_error(self, traj: Trajectory) -> float:
        """Distance between the true-dynamics endpoint and the requested one."""
        simulated = self.simulate(traj.states[0], traj.controls, traj.times)
        return float(np.linalg.norm(simulated[-1] - traj.states[-1]))


#: Nonlinear systems available to configuration files by name.
SYSTEMS = {"nonholonomic": NonholonomicCar}


def build_system(name: str) -> NonlinearSystem:
    try:
        return SYSTEMS[name]()
    except KeyError:
        raise ValueError(
            f"Unknown nonlinear system {name!r}; available: {sorted(SYSTEMS)}"
        ) from None
