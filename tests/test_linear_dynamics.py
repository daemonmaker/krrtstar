import numpy as np
import pytest
from scipy.integrate import quad
from scipy.linalg import expm

from krrtstar.dynamics.linear import AnalyticLinearDynamics


def double_integrator_1d():
    A = np.array([[0.0, 1.0], [0.0, 0.0]])
    B = np.array([[0.0], [1.0]])
    return A, B


def test_gramian_matches_quadrature():
    A, B = double_integrator_1d()
    R = np.array([[1.0]])
    dyn = AnalyticLinearDynamics(A, B, R=R)
    Q = B @ np.linalg.inv(R) @ B.T
    t = 1.7

    def entry(i, j):
        f = lambda s: (expm(A * s) @ Q @ expm(A.T * s))[i, j]
        return quad(f, 0.0, t)[0]

    G_num = np.array([[entry(i, j) for j in range(2)] for i in range(2)])
    assert np.allclose(G_num, dyn.gramian(t), atol=1e-8)


def test_connect_hits_endpoints():
    A, B = double_integrator_1d()
    dyn = AnalyticLinearDynamics(A, B)
    x0 = np.array([-1.0, 0.5])
    x1 = np.array([2.0, -1.0])
    traj = dyn.connect(x0, x1)
    assert traj is not None
    assert np.allclose(traj.states[0], x0, atol=1e-6)
    assert np.allclose(traj.states[-1], x1, atol=1e-6)
    assert traj.tau > 0


def test_cost_equals_time_plus_control_effort():
    A, B = double_integrator_1d()
    R = np.array([[2.0]])
    dyn = AnalyticLinearDynamics(A, B, R=R, n_traj_samples=300)
    x0 = np.array([-1.0, 0.5])
    x1 = np.array([2.0, -1.0])
    traj = dyn.connect(x0, x1)
    integrand = np.einsum("ti,ij,tj->t", traj.controls, R, traj.controls)
    effort = np.trapezoid(integrand, traj.times)
    assert traj.cost == pytest.approx(traj.tau + effort, rel=1e-3, abs=1e-3)


def test_trajectory_satisfies_dynamics():
    A, B = double_integrator_1d()
    dyn = AnalyticLinearDynamics(A, B, n_traj_samples=300)
    traj = dyn.connect(np.array([0.0, 0.0]), np.array([3.0, 0.0]))
    xdot_fd = np.gradient(traj.states, traj.times, axis=0)
    xdot_dyn = (A @ traj.states.T + B @ traj.controls.T).T
    # ignore endpoints where finite differences are one-sided
    assert np.max(np.abs(xdot_fd - xdot_dyn)[3:-3]) < 1e-3


def test_symmetric_cost_for_symmetric_system():
    # single integrator: cost is symmetric and connect always succeeds
    A = np.zeros((2, 2))
    B = np.eye(2)
    dyn = AnalyticLinearDynamics(A, B)
    x0 = np.array([0.0, 0.0])
    x1 = np.array([1.0, 2.0])
    c01 = dyn.cost(x0, x1)
    c10 = dyn.cost(x1, x0)
    assert c01 == pytest.approx(c10, rel=1e-3)


def test_control_bounds_flag_infeasible():
    A, B = double_integrator_1d()
    dyn = AnalyticLinearDynamics(A, B, u_bounds=np.array([[-0.001, 0.001]]))
    traj = dyn.connect(np.array([0.0, 0.0]), np.array([5.0, 0.0]))
    assert traj is not None
    assert traj.feasible is False
