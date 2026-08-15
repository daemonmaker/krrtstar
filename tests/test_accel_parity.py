import numpy as np
import pytest

from krrtstar import accel
from krrtstar.dynamics.linear import AnalyticLinearDynamics


def test_euclidean_neighbors_matches_numpy():
    rng = np.random.default_rng(0)
    states = rng.standard_normal((100, 3))
    query = np.zeros(3)
    radius = 1.2
    idx = accel.euclidean_neighbors(states, query, radius)
    ref = np.nonzero(np.linalg.norm(states - query, axis=1) <= radius)[0]
    assert sorted(int(i) for i in idx) == sorted(int(i) for i in ref)


@pytest.mark.skipif(not accel.HAVE_RUST, reason="Rust core not built")
def test_rust_linear_cost_matches_python():
    A = np.array([[0.0, 1.0], [0.0, 0.0]])
    B = np.array([[0.0], [1.0]])
    R = np.array([[1.0]])
    dyn = AnalyticLinearDynamics(A, B, R=R)
    rng = np.random.default_rng(1)
    for _ in range(10):
        x0 = rng.uniform(-3, 3, size=2)
        x1 = rng.uniform(-3, 3, size=2)
        rust = accel.linear_cost(dyn.A, dyn.Q, dyn.c, x0, x1, dyn.tau_max)
        # Python reference via the same optimal-time objective (bypass accel).
        py = dyn._optimal_tau(x0, x1)
        assert rust is not None and py is not None
        assert rust == pytest.approx(py[1], rel=1e-2, abs=1e-2)
