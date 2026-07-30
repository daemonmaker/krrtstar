"""Best-effort learned-dynamics backend tests.

The whole module is skipped when PyTorch is not installed, keeping the default
``poetry run pytest`` run fast and green in the light environment.
"""

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from krrtstar.dynamics.base import Trajectory
from krrtstar.dynamics.torch_dynamics import TorchDynamics


class DoubleIntegrator(torch.nn.Module):
    """Deterministic linear double integrator: state ``[pos, vel]``, ``u`` = accel.

    ``x_next = x + (A @ x + B @ u) * dt`` with ``A = [[0, 1], [0, 0]]`` and
    ``B = [[0], [1]]``.
    """

    def __init__(self, dt: float = 0.05) -> None:
        super().__init__()
        self.dt = dt
        A = torch.tensor([[0.0, 1.0], [0.0, 0.0]])
        B = torch.tensor([[0.0], [1.0]])
        self.register_buffer("A", A)
        self.register_buffer("B", B)

    def forward(self, x, u):
        dx = x @ self.A.t() + u @ self.B.t()
        return x + dx * self.dt


def test_torch_dynamics_gradient_connect():
    dt = 0.05
    model = DoubleIntegrator(dt=dt)

    dyn = TorchDynamics(
        model=model,
        x_dim=2,
        u_dim=1,
        u_bounds=np.array([[-5.0, 5.0]]),
        connect_method="gradient",
        horizon=30,
        dt=dt,
        iters=200,
        lr=0.2,
        goal_tolerance=0.2,
    )

    x0 = np.array([0.0, 0.0])
    x1 = np.array([1.0, 0.0])
    traj = dyn.connect(x0, x1)

    assert isinstance(traj, Trajectory)
    assert traj.states.shape == (31, 2)
    assert traj.controls.shape == (31, 1)
    np.testing.assert_allclose(traj.states[0], x0, atol=1e-5)

    final = traj.states[-1]
    assert np.linalg.norm(final - x1) < 0.3

    assert traj.cost > 0
    cost = dyn.cost(x0, x1)
    assert cost is not None and cost > 0


def test_u_bounds_roundtrip():
    model = DoubleIntegrator()
    dyn = TorchDynamics(
        model=model,
        x_dim=2,
        u_dim=1,
        u_bounds=np.array([[-5.0, 5.0]]),
    )
    bounds = dyn.u_bounds()
    assert bounds is not None
    np.testing.assert_allclose(bounds, np.array([[-5.0, 5.0]]))
