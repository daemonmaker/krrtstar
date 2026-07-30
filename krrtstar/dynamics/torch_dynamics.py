"""Best-effort learned-dynamics backend built on PyTorch.

This backend plugs a learned forward model into the same :class:`Dynamics`
interface used by the analytic linear system. Instead of a closed-form
optimal-control connection, it *steers* toward the target by optimizing an
open-loop control sequence through the learned model (autograd gradient descent
or random shooting). Connections are therefore best-effort: :meth:`connect` may
return ``None`` when it clearly fails to reach the target.

The learned model is any callable mapping ``(x, u) -> x_next`` where ``x`` has
shape ``(B, x_dim)`` and ``u`` has shape ``(B, u_dim)``. The output may be the
next state directly, or a ``(x_next, cost)`` tuple (only ``x_next`` is used).

``torch`` is imported lazily so that merely importing this module never fails;
the heavy dependency is only required when a :class:`TorchDynamics` is actually
constructed.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from .base import ControlBounds, Trajectory

try:  # pragma: no cover - exercised only when torch is installed
    import torch as _torch  # noqa: F401

    _TORCH_AVAILABLE = True
except Exception:  # pragma: no cover - the common case in the light test env
    _TORCH_AVAILABLE = False


class TorchDynamics:
    """Learned-dynamics backend with best-effort steering connections."""

    def __init__(
        self,
        model_path: Optional[str] = None,
        x_dim: Optional[int] = None,
        u_dim: Optional[int] = None,
        u_bounds: Optional[np.ndarray] = None,
        connect_method: str = "gradient",
        model=None,
        horizon: int = 20,
        dt: float = 0.05,
        iters: int = 100,
        lr: float = 0.1,
        goal_tolerance: float = 0.25,
    ) -> None:
        try:
            import torch
        except Exception as exc:  # pragma: no cover - depends on env
            raise ImportError(
                "TorchDynamics requires PyTorch, which is not installed. "
                "Install it with `pip install torch` (or add it to the "
                "project's optional dependencies) to use the learned backend."
            ) from exc

        self._torch = torch

        if x_dim is None or u_dim is None:
            raise ValueError("TorchDynamics requires explicit x_dim and u_dim")
        self.x_dim = int(x_dim)
        self.u_dim = int(u_dim)

        if model is not None and model_path is not None:
            raise ValueError("provide either model or model_path, not both")
        if model is not None:
            self.model = model
        elif model_path is not None:
            self.model = torch.jit.load(model_path)
        else:
            raise ValueError("TorchDynamics requires either model or model_path")

        # Put the model in eval mode when possible; the model itself is treated
        # as a fixed forward map, we only optimize the control inputs.
        if hasattr(self.model, "eval"):
            try:
                self.model.eval()
            except Exception:  # pragma: no cover - defensive
                pass

        self.connect_method = str(connect_method)
        self.horizon = int(horizon)
        self.dt = float(dt)
        self.iters = int(iters)
        self.lr = float(lr)
        self.goal_tolerance = float(goal_tolerance)

        bounds = None if u_bounds is None else np.asarray(u_bounds, dtype=float)
        self._bounds = ControlBounds(bounds)

    # ------------------------------------------------------------------ #
    # Model / tensor helpers
    # ------------------------------------------------------------------ #
    def _step(self, x, u):
        """Apply the learned model for one step, returning ``x_next``.

        ``x`` and ``u`` are ``(B, x_dim)`` / ``(B, u_dim)`` tensors.
        """
        out = self.model(x, u)
        if isinstance(out, (tuple, list)):
            out = out[0]
        return out

    def _bounds_tensors(self):
        """Return ``(lo, hi)`` tensors of shape ``(u_dim,)`` or ``None``."""
        if self._bounds.bounds is None:
            return None
        torch = self._torch
        lo = torch.as_tensor(self._bounds.bounds[:, 0], dtype=torch.float32)
        hi = torch.as_tensor(self._bounds.bounds[:, 1], dtype=torch.float32)
        return lo, hi

    def _rollout(self, x0, U):
        """Roll the model out from ``x0`` under controls ``U``.

        Args:
            x0: ``(x_dim,)`` tensor start state.
            U: ``(horizon, u_dim)`` control tensor.

        Returns:
            ``states`` tensor of shape ``(horizon + 1, x_dim)`` including x0.
        """
        x = x0.reshape(1, self.x_dim)
        states = [x]
        for k in range(self.horizon):
            u = U[k].reshape(1, self.u_dim)
            x = self._step(x, u)
            states.append(x)
        return self._torch.cat(states, dim=0)

    # ------------------------------------------------------------------ #
    # Connection strategies
    # ------------------------------------------------------------------ #
    def _connect_gradient(self, x0_t, x1_t):
        torch = self._torch
        U = torch.zeros(self.horizon, self.u_dim, requires_grad=True)
        opt = torch.optim.Adam([U], lr=self.lr)
        bounds = self._bounds_tensors()

        for _ in range(self.iters):
            opt.zero_grad()
            states = self._rollout(x0_t, U)
            final = states[-1]
            loss = torch.sum((final - x1_t) ** 2)
            loss.backward()
            opt.step()
            if bounds is not None:
                with torch.no_grad():
                    lo, hi = bounds
                    U.copy_(torch.min(torch.max(U, lo), hi))
        return U.detach()

    def _connect_shooting(self, x0_t, x1_t, num_samples: int = 256):
        torch = self._torch
        bounds = self._bounds_tensors()
        best_U = None
        best_dist = None
        for _ in range(num_samples):
            if bounds is not None:
                lo, hi = bounds
                U = lo + (hi - lo) * torch.rand(self.horizon, self.u_dim)
            else:
                U = torch.randn(self.horizon, self.u_dim)
            with torch.no_grad():
                states = self._rollout(x0_t, U)
                dist = torch.norm(states[-1] - x1_t).item()
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best_U = U
        return best_U

    # ------------------------------------------------------------------ #
    # Public API (Dynamics protocol)
    # ------------------------------------------------------------------ #
    def connect(self, x0: np.ndarray, x1: np.ndarray) -> Optional[Trajectory]:
        torch = self._torch
        x0 = np.asarray(x0, dtype=float).reshape(-1)
        x1 = np.asarray(x1, dtype=float).reshape(-1)
        x0_t = torch.as_tensor(x0, dtype=torch.float32)
        x1_t = torch.as_tensor(x1, dtype=torch.float32)

        if self.connect_method == "shooting":
            U = self._connect_shooting(x0_t, x1_t)
        else:
            U = self._connect_gradient(x0_t, x1_t)

        # Final rollout for the trajectory we return.
        with torch.no_grad():
            states_t = self._rollout(x0_t, U)
        states = states_t.detach().cpu().numpy().astype(float)
        controls_seq = U.detach().cpu().numpy().astype(float)

        # controls has horizon + 1 rows: repeat/pad the last control.
        controls = np.zeros((self.horizon + 1, self.u_dim), dtype=float)
        controls[: self.horizon] = controls_seq
        controls[self.horizon] = controls_seq[-1]

        times = np.linspace(0.0, self.horizon * self.dt, self.horizon + 1)
        tau = float(self.horizon * self.dt)

        final = states[-1]
        final_dist = float(np.linalg.norm(final - x1))

        # Clearly failed: bail out entirely.
        if final_dist > 5.0 * self.goal_tolerance:
            return None

        # Control effort with R = I, integrated over the (constant per-step) grid.
        effort = float(np.sum(controls_seq * controls_seq) * self.dt)
        cost = tau + effort

        within_bounds = self._bounds.satisfied(controls)
        feasible = bool(final_dist <= self.goal_tolerance and within_bounds)

        return Trajectory(
            tau=tau,
            cost=cost,
            times=times,
            states=states,
            controls=controls,
            feasible=feasible,
        )

    def cost(self, x0: np.ndarray, x1: np.ndarray) -> Optional[float]:
        traj = self.connect(x0, x1)
        if traj is None:
            return None
        return traj.cost

    def u_bounds(self) -> Optional[np.ndarray]:
        return self._bounds.bounds
