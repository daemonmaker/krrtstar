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


def double_integrator_2d():
    A = np.array(
        [[0.0, 0.0, 1.0, 0.0],
         [0.0, 0.0, 0.0, 1.0],
         [0.0, 0.0, 0.0, 0.0],
         [0.0, 0.0, 0.0, 0.0]]
    )
    B = np.array([[0.0, 0.0], [0.0, 0.0], [1.0, 0.0], [0.0, 1.0]])
    return A, B


@pytest.mark.skipif(not accel.HAVE_RUST, reason="Rust core not built")
def test_rust_connect_matches_python_reconstruction():
    """Native trajectory reconstruction must match the Python reference."""
    A, B = double_integrator_2d()
    R = np.eye(2)
    rust = AnalyticLinearDynamics(A, B, R=R, n_traj_samples=64, use_accel=True)
    py = AnalyticLinearDynamics(A, B, R=R, n_traj_samples=64, use_accel=False)
    rng = np.random.default_rng(0)
    for _ in range(8):
        x0 = rng.uniform(-3, 3, size=4)
        x1 = rng.uniform(-3, 3, size=4)
        tr = rust.connect(x0, x1)
        tp = py.connect(x0, x1)
        assert tr is not None and tp is not None
        assert tr.tau == pytest.approx(tp.tau, rel=1e-3, abs=1e-3)
        assert tr.cost == pytest.approx(tp.cost, rel=1e-4, abs=1e-4)
        assert tr.times.shape == tp.times.shape
        assert np.allclose(tr.states, tp.states, atol=1e-3)
        assert np.allclose(tr.controls, tp.controls, atol=1e-3)


@pytest.mark.skipif(not accel.HAVE_RUST, reason="Rust core not built")
def test_native_connect_trajectory_is_physically_consistent():
    """The native trajectory must satisfy the ODE, endpoints, and cost identity."""
    A, B = double_integrator_2d()
    R = np.eye(2)
    dyn = AnalyticLinearDynamics(A, B, R=R, n_traj_samples=400, use_accel=True)
    x0 = np.array([-1.0, 0.5, 0.2, -0.3])
    x1 = np.array([2.0, -1.0, 0.0, 0.0])
    traj = dyn.connect(x0, x1)
    assert traj is not None
    # endpoints
    assert np.allclose(traj.states[0], x0, atol=1e-9)
    assert np.allclose(traj.states[-1], x1, atol=1e-9)
    # cost identity: cost == tau + control effort
    effort = np.trapezoid(
        np.einsum("ti,ij,tj->t", traj.controls, R, traj.controls), traj.times
    )
    assert traj.cost == pytest.approx(traj.tau + effort, rel=1e-3, abs=1e-3)
    # satisfies xdot = A x + B u
    xdot_fd = np.gradient(traj.states, traj.times, axis=0)
    xdot_dyn = (A @ traj.states.T + B @ traj.controls.T).T
    assert np.max(np.abs(xdot_fd - xdot_dyn)[3:-3]) < 1e-3


def test_batched_costs_agree_with_pairwise():
    """Batched grid costs must bound and closely track the refined costs."""
    A, B = double_integrator_2d()
    dyn = AnalyticLinearDynamics(A, B, R=np.eye(2))
    rng = np.random.default_rng(3)
    states = rng.uniform(-3, 3, size=(12, 4))
    target = np.array([1.0, -1.0, 0.0, 0.0])

    batch_to = dyn.cost_batch_to(states, target)
    pair_to = np.array([dyn.cost(s, target) for s in states], dtype=float)
    # The grid scan cannot beat the refined optimum, and should be close to it.
    assert np.all(batch_to >= pair_to - 1e-6)
    assert np.allclose(batch_to, pair_to, rtol=0.05, atol=0.05)

    batch_from = dyn.cost_batch_from(target, states)
    pair_from = np.array([dyn.cost(target, s) for s in states], dtype=float)
    assert np.all(batch_from >= pair_from - 1e-6)
    assert np.allclose(batch_from, pair_from, rtol=0.05, atol=0.05)


def test_batched_costs_match_python_fallback():
    """The pure-Python batch fallback must agree with the native batch."""
    A, B = double_integrator_2d()
    fast = AnalyticLinearDynamics(A, B, R=np.eye(2), use_accel=True)
    slow = AnalyticLinearDynamics(A, B, R=np.eye(2), use_accel=False)
    rng = np.random.default_rng(4)
    states = rng.uniform(-2, 2, size=(6, 4))
    target = np.array([0.5, 0.5, 0.0, 0.0])
    a = fast.cost_batch_to(states, target)
    b = slow.cost_batch_to(states, target)
    assert np.allclose(a, b, rtol=0.05, atol=0.05)


def test_planner_works_with_dynamics_lacking_batch_methods():
    """Backends without batched cost (e.g. learned models) still plan."""
    from krrtstar.planner import KRRTStar

    base = AnalyticLinearDynamics(np.zeros((2, 2)), np.eye(2))

    class NoBatch:
        """Minimal Dynamics implementation exposing only the protocol methods."""

        x_dim = 2
        u_dim = 2

        def cost(self, x0, x1):
            return base.cost(x0, x1)

        def connect(self, x0, x1):
            return base.connect(x0, x1)

        def u_bounds(self):
            return None

    dyn = NoBatch()
    assert not hasattr(dyn, "cost_batch_to")
    planner = KRRTStar(
        dynamics=dyn,
        state_bounds=np.array([[-4, -4], [4, 4]]),
        x_init=np.array([-3.0, -3.0]),
        x_goal=np.array([3.0, 3.0]),
        connection_radius=8.0,
        euclidean_gate=8.0,
        seed=2,
    )
    result = planner.grow(60)
    assert len(result.tree.nodes) > 1


@pytest.mark.skipif(not accel.HAVE_RUST, reason="Rust core not built")
def test_native_connect_respects_control_bounds_flag():
    A, B = double_integrator_2d()
    dyn = AnalyticLinearDynamics(
        A, B, u_bounds=np.array([[-1e-3, 1e-3], [-1e-3, 1e-3]]), use_accel=True
    )
    traj = dyn.connect(np.zeros(4), np.array([5.0, 5.0, 0.0, 0.0]))
    assert traj is not None
    assert traj.feasible is False

