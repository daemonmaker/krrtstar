import numpy as np

from krrtstar.dynamics.linear import AnalyticLinearDynamics
from krrtstar.geometry import Box, CollisionChecker, Environment, PoseMapping, Robot, Sphere
from krrtstar.planner import KRRTStar


def single_integrator_2d():
    # Cheap dynamics (A = 0, B = I) so tests run fast.
    return AnalyticLinearDynamics(np.zeros((2, 2)), np.eye(2))


def test_finds_path_in_free_space():
    dyn = single_integrator_2d()
    planner = KRRTStar(
        dynamics=dyn,
        state_bounds=np.array([[-5, -5], [5, 5]]),
        x_init=np.array([-4.0, -4.0]),
        x_goal=np.array([4.0, 4.0]),
        connection_radius=6.0,
        goal_bias=0.2,
        goal_tolerance=1.0,
        euclidean_gate=6.0,
        seed=1,
    )
    result = planner.grow(200)
    assert result.found
    assert result.cost is not None and result.cost > 0
    # path is a valid chain back to the root
    path = result.tree.path_to(result.goal_index)
    assert path[0] == 0


def test_progress_callback_invoked():
    dyn = single_integrator_2d()
    planner = KRRTStar(
        dynamics=dyn,
        state_bounds=np.array([[-5, -5], [5, 5]]),
        x_init=np.array([0.0, 0.0]),
        x_goal=np.array([3.0, 3.0]),
        connection_radius=6.0,
        euclidean_gate=6.0,
        seed=2,
    )
    calls = []
    planner.grow(60, progress_cb=lambda it, tree: calls.append(it), progress_every=20)
    assert len(calls) >= 2  # periodic + final


def test_headless_growth_without_collision():
    dyn = single_integrator_2d()
    planner = KRRTStar(
        dynamics=dyn,
        state_bounds=np.array([[-5, -5], [5, 5]]),
        x_init=np.array([0.0, 0.0]),
        x_goal=np.array([1.0, 1.0]),
        connection_radius=6.0,
        seed=3,
    )
    result = planner.grow(30)
    assert len(result.tree.nodes) >= 1


def test_rewire_reduces_costs_monotonic_tree():
    dyn = single_integrator_2d()
    planner = KRRTStar(
        dynamics=dyn,
        state_bounds=np.array([[-5, -5], [5, 5]]),
        x_init=np.array([0.0, 0.0]),
        x_goal=np.array([4.0, 0.0]),
        connection_radius=8.0,
        euclidean_gate=8.0,
        rewire=True,
        seed=5,
    )
    result = planner.grow(120)
    # every node's cost should be >= its parent's cost (valid cost-to-come tree)
    for i, node in enumerate(result.tree.nodes):
        if node.parent != -1:
            assert node.cost_from_start >= result.tree.nodes[node.parent].cost_from_start - 1e-6
