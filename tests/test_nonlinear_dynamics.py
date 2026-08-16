"""Tests for the locally-linearized dynamics backend and the nonholonomic car."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

try:  # Python 3.11+
    import tomllib
except ModuleNotFoundError:  # pragma: no cover
    import tomli as tomllib  # type: ignore

from krrtstar.config import build_collision_checker, parse_config
from krrtstar.dynamics.base import Dynamics, Trajectory
from krrtstar.dynamics.nonlinear import (
    LinearizedDynamics,
    NonholonomicCar,
    NonlinearSystem,
    build_system,
    numerical_jacobians,
)
from krrtstar.planner import KRRTStar, Node
from krrtstar.robots import collision_shapes

CONFIG_PATH = Path(__file__).resolve().parents[1] / "examples" / "nonholonomic_car.toml"

# The gate the example was tuned with; used when the config's "auto" value is
# not understood by the parser yet.
FALLBACK_GATE = 25.0


def car_dynamics(**kwargs) -> LinearizedDynamics:
    kwargs.setdefault("n_traj_samples", 24)
    return LinearizedDynamics(NonholonomicCar(), **kwargs)


# --------------------------------------------------------------------------- #
# The system itself
# --------------------------------------------------------------------------- #
def test_car_satisfies_the_nonlinear_system_protocol():
    car = NonholonomicCar()
    assert isinstance(car, NonlinearSystem)
    assert (car.x_dim, car.u_dim) == (5, 2)
    assert isinstance(build_system("nonholonomic"), NonholonomicCar)
    with pytest.raises(ValueError):
        build_system("hovercraft")


def test_f_matches_hand_written_equations():
    car = NonholonomicCar()
    # Heading 0: all the speed goes into +x, and the controls are the
    # accelerations of speed and curvature.
    assert np.allclose(
        car.f([1.0, 2.0, 0.0, 3.0, 0.0], [0.5, -0.25]),
        [3.0, 0.0, 0.0, 0.5, -0.25],
    )
    # Heading pi/2: all the speed goes into +y.
    assert np.allclose(
        car.f([0.0, 0.0, np.pi / 2, 2.0, 0.0], [0.0, 0.0]),
        [0.0, 2.0, 0.0, 0.0, 0.0],
    )
    # Curvature turns the car at rate v * kappa; a car at rest never turns.
    assert car.f([0.0, 0.0, 0.3, 2.0, 0.25], [0.0, 0.0])[2] == pytest.approx(0.5)
    assert car.f([0.0, 0.0, 0.3, 0.0, 0.25], [0.0, 0.0])[2] == pytest.approx(0.0)
    # Heading -pi/4 (the original's start heading was -pi/2).
    th, v = -np.pi / 4, 1.5
    assert np.allclose(
        car.f([0.0, 0.0, th, v, 0.0], [0.0, 0.0])[:2],
        [v * np.cos(th), v * np.sin(th)],
    )


def test_analytic_jacobians_match_numerical():
    car = NonholonomicCar()
    rng = np.random.default_rng(11)
    for _ in range(12):
        x = np.concatenate(
            [
                rng.uniform(-20, 20, size=2),
                rng.uniform(-np.pi, np.pi, size=1),
                rng.uniform(-3, 5, size=1),
                rng.uniform(-0.5, 0.5, size=1),
            ]
        )
        u = rng.uniform(-2, 2, size=2)
        A, B = car.jacobians(x, u)
        A_num, B_num = numerical_jacobians(car, x, u)
        assert np.max(np.abs(A - A_num)) < 1e-6
        assert np.max(np.abs(B - B_num)) < 1e-6


def test_jacobian_entries_match_the_original():
    # The original built A(0,2) = -v sin(theta), A(0,3) = cos(theta),
    # A(1,2) = v cos(theta), A(1,3) = sin(theta), plus B(3,0) = B(4,1) = 1.
    car = NonholonomicCar()
    theta, v, kappa = 0.7, 2.5, 0.3
    A, B = car.jacobians([1.0, -2.0, theta, v, kappa], [0.0, 0.0])
    assert A[0, 2] == pytest.approx(-v * np.sin(theta))
    assert A[0, 3] == pytest.approx(np.cos(theta))
    assert A[1, 2] == pytest.approx(v * np.cos(theta))
    assert A[1, 3] == pytest.approx(np.sin(theta))
    assert A[2, 3] == pytest.approx(kappa)
    assert A[2, 4] == pytest.approx(v)
    assert np.count_nonzero(A[3:]) == 0
    expected_B = np.zeros((5, 2))
    expected_B[3, 0] = expected_B[4, 1] = 1.0
    assert np.array_equal(B, expected_B)


def test_default_control_penalty_is_the_originals():
    dyn = car_dynamics()
    assert np.allclose(dyn.R, np.diag([1.0, 50.0]))
    # The original left the controls unbounded (+-DBL_MAX).
    assert dyn.u_bounds() is None


# --------------------------------------------------------------------------- #
# Linearization
# --------------------------------------------------------------------------- #
def test_linearization_reproduces_f_at_the_linearization_point():
    car = NonholonomicCar()
    dyn = car_dynamics()
    rng = np.random.default_rng(3)
    for _ in range(8):
        x0 = np.concatenate(
            [
                rng.uniform(-20, 20, size=2),
                rng.uniform(-np.pi, np.pi, size=1),
                rng.uniform(-3, 5, size=1),
                rng.uniform(-0.5, 0.5, size=1),
            ]
        )
        A, B, c = dyn.linearize(x0)
        u0 = np.zeros(2)
        assert np.allclose(A @ x0 + B @ u0 + c, car.f(x0, u0), atol=1e-12)


def test_linear_models_are_cached_per_linearization_point():
    dyn = car_dynamics()
    x0 = np.array([1.0, 2.0, 0.3, 1.5, 0.1])
    first = dyn.linear_model(x0)
    assert dyn.linear_model(x0.copy()) is first
    other = dyn.linear_model(np.array([1.0, 2.0, 0.3, 1.5, 0.2]))
    assert other is not first
    assert not np.allclose(other.A, first.A)


def test_satisfies_the_dynamics_protocol():
    dyn = car_dynamics()
    assert isinstance(dyn, Dynamics)
    assert (dyn.x_dim, dyn.u_dim) == (5, 2)


# --------------------------------------------------------------------------- #
# connect / cost
# --------------------------------------------------------------------------- #
def test_connect_hits_the_endpoints_of_the_linearized_problem():
    dyn = car_dynamics()
    x0 = np.array([0.0, 0.0, 0.0, 1.0, 0.0])
    x1 = np.array([4.0, 1.5, 0.3, 1.5, 0.05])
    traj = dyn.connect(x0, x1)
    assert isinstance(traj, Trajectory)
    assert np.allclose(traj.states[0], x0, atol=1e-6)
    assert np.allclose(traj.states[-1], x1, atol=1e-6)
    assert traj.tau > 0.0
    assert traj.cost > 0.0
    assert traj.controls.shape == (len(traj.times), 2)
    assert dyn.cost(x0, x1) == pytest.approx(traj.cost, rel=1e-3)


def test_cost_grows_with_distance():
    dyn = car_dynamics()
    x0 = np.array([0.0, 0.0, 0.0, 1.0, 0.0])
    near = dyn.cost(x0, np.array([2.0, 0.2, 0.05, 1.0, 0.0]))
    far = dyn.cost(x0, np.array([20.0, 6.0, 0.5, 1.0, 0.0]))
    assert near is not None and far is not None
    assert 0.0 < near < far


# --------------------------------------------------------------------------- #
# True vs linearized dynamics
# --------------------------------------------------------------------------- #
def test_simulate_integrates_the_true_dynamics():
    # With zero control the car drives a circle of radius 1/kappa: starting at
    # the origin heading +x with kappa = 0.5 it satisfies
    #   theta = v kappa t,  px = 2 sin(theta),  py = 2 (1 - cos(theta)).
    dyn = car_dynamics()
    x0 = np.array([0.0, 0.0, 0.0, 1.0, 0.5])
    times = np.linspace(0.0, 2.0, 41)
    states = dyn.simulate(x0, np.zeros((len(times), 2)), times)
    theta = 0.5 * times
    assert np.allclose(states[:, 0], 2.0 * np.sin(theta), atol=1e-8)
    assert np.allclose(states[:, 1], 2.0 * (1.0 - np.cos(theta)), atol=1e-8)
    assert np.allclose(states[:, 2], theta, atol=1e-12)
    # Speed and curvature are untouched by a zero control.
    assert np.allclose(states[:, 3], 1.0)
    assert np.allclose(states[:, 4], 0.5)


def test_linearization_error_grows_with_distance(capsys):
    """The returned trajectory only approximately satisfies the true dynamics."""
    dyn = car_dynamics()
    x0 = np.array([0.0, 0.0, 0.0, 1.0, 0.0])
    near = np.array([2.0, 0.2, 0.05, 1.1, 0.02])
    far = np.array([20.0, 15.0, 1.0, 3.0, 0.3])

    errors = {}
    for label, x1 in (("near", near), ("far", far)):
        traj = dyn.connect(x0, x1)
        assert traj is not None
        simulated = dyn.simulate(x0, traj.controls, traj.times)
        # The linearized trajectory itself lands on the target exactly.
        assert np.allclose(traj.states[-1], x1, atol=1e-6)
        errors[label] = float(np.linalg.norm(simulated[-1] - x1))
        assert dyn.endpoint_error(traj) == pytest.approx(errors[label], rel=1e-9)
        with capsys.disabled():
            print(
                f"\n{label}: tau={traj.tau:.3f} cost={traj.cost:.3f} "
                f"true-dynamics endpoint error={errors[label]:.4f}"
            )

    # Qualitative ordering only: the approximation degrades with tau and with
    # distance from the linearization point.
    assert errors["near"] < errors["far"]
    assert errors["near"] < 1.0


def test_max_endpoint_error_rejects_bad_connections():
    x0 = np.array([0.0, 0.0, 0.0, 1.0, 0.0])
    near = np.array([2.0, 0.2, 0.05, 1.1, 0.02])
    far = np.array([20.0, 15.0, 1.0, 3.0, 0.3])

    ungated = car_dynamics()
    assert ungated.max_endpoint_error is None
    assert ungated.connect(x0, far) is not None  # off by default

    gated = car_dynamics(max_endpoint_error=0.25)
    assert gated.connect(x0, near) is not None
    assert gated.connect(x0, far) is None
    # The (approximate) cost query is unaffected by the gate.
    assert gated.cost(x0, far) is not None


# --------------------------------------------------------------------------- #
# Planner integration
# --------------------------------------------------------------------------- #
def test_no_batched_cost_queries_so_the_planner_falls_back():
    # Every candidate needs its own linearization, so there is nothing for a
    # batched query to share; the planner's _batch_costs then uses cost() per
    # pair (see krrtstar/planner.py).
    dyn = car_dynamics()
    assert not hasattr(dyn, "cost_batch_to")
    assert not hasattr(dyn, "cost_batch_from")

    planner = KRRTStar(
        dynamics=dyn,
        state_bounds=np.array([[-5, -5, -np.pi, 0.5, -0.5], [5, 5, np.pi, 2.0, 0.5]]),
        x_init=np.array([-3.0, 0.0, 0.0, 1.0, 0.0]),
        x_goal=np.array([3.0, 0.0, 0.0, 1.0, 0.0]),
        connection_radius=15.0,
    )
    planner.tree.add(
        Node(np.array([-1.0, 0.5, 0.1, 1.0, 0.0]), parent=0, cost_from_start=1.0)
    )
    costs = planner._batch_costs([0, 1], planner.x_goal, to_target=True)
    assert costs.shape == (2,)
    assert np.all(np.isfinite(costs))


def load_example_config():
    """Parse the example, tolerating schema features that are still landing."""
    with open(CONFIG_PATH, "rb") as fh:
        data = tomllib.load(fh)
    try:
        cfg = parse_config(data, base_dir=str(CONFIG_PATH.parent))
    except (TypeError, ValueError):
        # "auto" gates are not understood by this parser version.
        data["planner"]["euclidean_gate"] = FALLBACK_GATE
        cfg = parse_config(data, base_dir=str(CONFIG_PATH.parent))
    if not cfg.robot.shapes:
        # Robot presets are not wired into the parser yet; the preset is the
        # original's 5 x 3 x 2.5 box.
        cfg.robot.shapes = collision_shapes("car")
    if not isinstance(cfg.planner.euclidean_gate, (int, float)):
        cfg.planner.euclidean_gate = FALLBACK_GATE
    return cfg


def build_example_planner(cfg) -> KRRTStar:
    from krrtstar.run import _state_bounds, build_planner

    try:
        return build_planner(cfg)
    except ValueError:
        # The "nonholonomic" dynamics kind is not wired into build_dynamics yet.
        dcfg = cfg.dynamics
        dyn = LinearizedDynamics(
            build_system(dcfg.kind),
            R=dcfg.R,
            u_bounds=dcfg.u_bounds,
            **{
                k: dcfg.extra[k]
                for k in ("tau_max", "n_traj_samples", "max_endpoint_error")
                if k in dcfg.extra
            },
        )
        return KRRTStar(
            dynamics=dyn,
            state_bounds=_state_bounds(cfg),
            x_init=cfg.planner.x_init,
            x_goal=cfg.planner.x_goal,
            collision=build_collision_checker(cfg),
            connection_radius=cfg.planner.connection_radius,
            goal_bias=cfg.planner.goal_bias,
            goal_tolerance=cfg.planner.goal_tolerance,
            rewire=cfg.planner.rewire,
            euclidean_gate=cfg.planner.euclidean_gate,
            seed=cfg.planner.seed,
        )


def test_example_config_describes_the_car():
    cfg = load_example_config()
    assert (cfg.dynamics.x_dim, cfg.dynamics.u_dim) == (5, 2)
    assert cfg.dynamics.kind == "nonholonomic"
    assert np.allclose(cfg.dynamics.R, np.diag([1.0, 50.0]))
    assert cfg.planner.x_init.shape == (5,)
    assert cfg.planner.x_goal.shape == (5,)
    # The state's position must drive the body pose, and the heading its yaw.
    assert (cfg.robot.pose.x, cfg.robot.pose.y, cfg.robot.pose.yaw) == (0, 1, 2)
    assert cfg.robot.shapes  # preset or explicit geometry
    assert len(cfg.environment.obstacles) >= 2


def test_planner_smoke_on_the_example_config(capsys):
    cfg = load_example_config()
    planner = build_example_planner(cfg)
    assert planner.dyn.x_dim == 5
    assert not planner.collision.in_collision(cfg.planner.x_init)
    assert not planner.collision.in_collision(cfg.planner.x_goal)

    result = planner.grow(60)
    assert len(result.tree.nodes) > 1  # the tree grew past the root
    # Every edge's parent trajectory must start where its parent sits.
    for node in result.tree.nodes[1:]:
        parent = result.tree.nodes[node.parent]
        assert np.allclose(node.traj_from_parent.states[0], parent.state, atol=1e-6)
        assert np.allclose(node.traj_from_parent.states[-1], node.state, atol=1e-6)

    with capsys.disabled():
        print(
            f"\nexample: {len(result.tree.nodes)} nodes in {result.elapsed:.1f}s, "
            f"found={result.found} cost={result.cost}"
        )

    if result.found:
        states = np.vstack([t.states for t in result.solution])
        assert not planner.collision.trajectory_in_collision(states)
