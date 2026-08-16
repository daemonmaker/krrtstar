"""The dynamical systems ported from the original implementation.

Covers the config surface (robot presets, pose axes, the nonlinear system kind)
and checks the ported constants against ``includes/dynamicrrt.h`` /
``includes/robots.hpp``.
"""

import os

import numpy as np
import pytest

from krrtstar.config import build_collision_checker, build_dynamics, load_config, parse_config
from krrtstar.dynamics.linear import AnalyticLinearDynamics
from krrtstar.geometry import Box, Cylinder
from krrtstar.robots import QUAD_LENGTH, ROTOR_RADIUS, collision_shapes, default_pose

EXAMPLES = os.path.join(os.path.dirname(__file__), "..", "examples")

ALL_EXAMPLES = [
    "single_integrator_2d",
    "double_integrator_1d",
    "double_integrator_2d",
    "double_integrator_3d",
    "quadrotor_window",
    "quadrotor_two_walls",
    "nonholonomic_car",
    "rich_geometry_3d",
]


def path_for(name):
    return os.path.join(EXAMPLES, f"{name}.toml")


# --------------------------------------------------------------------------- #
# Every shipped example must be constructible
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("name", ALL_EXAMPLES)
def test_example_config_builds(name):
    cfg = load_config(path_for(name))
    dyn = build_dynamics(cfg.dynamics)
    assert dyn.x_dim == cfg.dynamics.x_dim
    assert dyn.u_dim == cfg.dynamics.u_dim
    checker = build_collision_checker(cfg)
    assert checker.backend in {"rust", "python"}
    # the start state must be collision free, or the example is broken
    assert not checker.in_collision(cfg.planner.x_init)
    assert not checker.in_collision(cfg.planner.x_goal)


@pytest.mark.parametrize("name", ALL_EXAMPLES)
def test_example_grows_a_tree(name):
    from krrtstar.run import build_planner

    cfg = load_config(path_for(name))
    cfg.planner.target_nodes = 40
    planner = build_planner(cfg)
    result = planner.grow(40)
    assert len(result.tree.nodes) >= 1
    assert result.elapsed is not None


# --------------------------------------------------------------------------- #
# 1D double integrator
# --------------------------------------------------------------------------- #
def test_double_integrator_1d_matches_original():
    cfg = load_config(path_for("double_integrator_1d"))
    assert (cfg.dynamics.x_dim, cfg.dynamics.u_dim) == (2, 1)
    assert np.allclose(cfg.dynamics.A, [[0, 1], [0, 0]])
    assert np.allclose(cfg.dynamics.B, [[0], [1]])
    # the original's u_bounds for DOUBLE_INTEGRATOR_1D were +/- 10
    assert np.allclose(cfg.dynamics.u_bounds, [[-10, 10]])


def test_double_integrator_1d_pins_y_to_zero():
    """Index 1 is velocity, so the robot must not be placed at y = velocity."""
    cfg = load_config(path_for("double_integrator_1d"))
    pose = cfg.robot.pose
    assert pose.x == 0
    assert pose.y is None
    moving = np.array([3.0, 7.5])  # position 3, velocity 7.5
    assert np.allclose(pose.position(moving), [3.0, 0.0, 0.0])


def test_double_integrator_1d_finds_a_solution():
    from krrtstar.run import run_experiment

    cfg = load_config(path_for("double_integrator_1d"))
    result = run_experiment(cfg)
    assert result.found
    assert result.cost > 0


# --------------------------------------------------------------------------- #
# Quadrotor
# --------------------------------------------------------------------------- #
def test_quadrotor_matches_original_constants():
    cfg = load_config(path_for("quadrotor_two_walls"))
    A, B, R = cfg.dynamics.A, cfg.dynamics.B, cfg.dynamics.R
    assert (cfg.dynamics.x_dim, cfg.dynamics.u_dim) == (10, 3)

    # positions integrate velocities; attitudes integrate their rates
    assert np.allclose(A[0:3, 3:6], np.eye(3))
    assert np.allclose(A[6:8, 8:10], np.eye(2))
    # gravity couples attitude into horizontal acceleration
    assert A[3, 7] == pytest.approx(9.81)
    assert A[4, 6] == pytest.approx(-9.81)
    # B: thrust / mass, and torque * length / inertia
    assert B[5, 0] == pytest.approx(1.0 / 0.5)
    assert B[8, 1] == pytest.approx(QUAD_LENGTH / 0.1)
    assert B[9, 2] == pytest.approx(QUAD_LENGTH / 0.1)
    # control penalty diag(0.25, 0.5, 0.5) * control_penalty(0.1)
    assert np.allclose(np.diag(R), [0.025, 0.05, 0.05])
    # asymmetric thrust bounds about hover, symmetric torques
    assert np.allclose(cfg.dynamics.u_bounds[0], [-4.545, 9.935])
    assert np.allclose(cfg.dynamics.u_bounds[1], [-3.62, 3.62])


def test_quadrotor_robot_is_the_rotor_spanning_disc():
    cfg = load_config(path_for("quadrotor_two_walls"))
    assert len(cfg.robot.shapes) == 1
    disc = cfg.robot.shapes[0]
    assert isinstance(disc, Cylinder)
    assert disc.radius == pytest.approx(QUAD_LENGTH + ROTOR_RADIUS)


def test_quadrotor_pose_uses_roll_and_pitch_indices():
    cfg = load_config(path_for("quadrotor_two_walls"))
    pose = cfg.robot.pose
    assert (pose.x, pose.y, pose.z) == (0, 1, 2)
    assert (pose.roll, pose.pitch) == (6, 7)
    assert pose.has_orientation
    level = np.zeros(10)
    assert np.allclose(pose.rotation(level), np.eye(3))
    rolled = np.zeros(10)
    rolled[6] = 0.3
    assert not np.allclose(pose.rotation(rolled), np.eye(3))


def test_quadrotor_two_walls_world_matches_original():
    """Two thin walls at x = +/-1, each split into four boxes by a window."""
    cfg = load_config(path_for("quadrotor_two_walls"))
    boxes = [o for o in cfg.environment.obstacles if isinstance(o, Box)]
    assert len(boxes) == 8  # 4 segments per wall
    xs = sorted({round(float(np.asarray(b.center)[0]), 6) for b in boxes})
    assert xs == [-1.0, 1.0]
    assert all(np.asarray(b.extents, float)[0] == pytest.approx(0.1) for b in boxes)
    # The window openings are free space.
    checker = build_collision_checker(cfg)
    assert not checker.in_collision(np.array([1.0, 1.0, 1.0, 0, 0, 0, 0, 0, 0, 0]))
    assert not checker.in_collision(np.array([-1.0, -1.0, 4.0, 0, 0, 0, 0, 0, 0, 0]))
    # ...and the solid parts of the walls are not.
    assert checker.in_collision(np.array([1.0, -3.0, 1.0, 0, 0, 0, 0, 0, 0, 0]))
    assert checker.in_collision(np.array([-1.0, 3.0, 4.0, 0, 0, 0, 0, 0, 0, 0]))


# --------------------------------------------------------------------------- #
# Nonholonomic car
# --------------------------------------------------------------------------- #
def test_nonholonomic_config_builds_linearized_dynamics():
    from krrtstar.dynamics.nonlinear import LinearizedDynamics, NonholonomicCar

    cfg = load_config(path_for("nonholonomic_car"))
    assert cfg.dynamics.kind == "nonholonomic"
    dyn = build_dynamics(cfg.dynamics)
    assert isinstance(dyn, LinearizedDynamics)
    assert isinstance(dyn.system, NonholonomicCar)
    assert (dyn.x_dim, dyn.u_dim) == (5, 2)


def test_nonholonomic_control_penalty_matches_original():
    cfg = load_config(path_for("nonholonomic_car"))
    # control_penalty = 1 (acceleration), control_penalty1 = 50 (curvature rate)
    assert np.allclose(np.diag(cfg.dynamics.R), [1.0, 50.0])


def test_car_robot_body_matches_original():
    cfg = load_config(path_for("nonholonomic_car"))
    body = cfg.robot.shapes[0]
    assert isinstance(body, Box)
    assert np.allclose(body.extents, [5.0, 3.0, 2.5])
    assert np.allclose(body.center, [2.5, 1.5, 0.0])
    assert cfg.robot.pose.yaw == 2


# --------------------------------------------------------------------------- #
# Robot presets in config
# --------------------------------------------------------------------------- #
def test_preset_supplies_geometry_and_default_pose():
    cfg = parse_config(
        {
            "dynamics": {"x_dim": 5, "u_dim": 2},
            "environment": {"bounds": [[-5, -5, -5], [5, 5, 5]]},
            "robot": {"preset": "car"},
        }
    )
    assert len(cfg.robot.shapes) == 1
    assert cfg.robot.pose == default_pose("car")


def test_explicit_pose_overrides_preset_default():
    cfg = parse_config(
        {
            "dynamics": {"x_dim": 5, "u_dim": 2},
            "environment": {"bounds": [[-5, -5, -5], [5, 5, 5]]},
            "robot": {"preset": "car", "pose_from_state": {"x": 0, "y": 1}},
        }
    )
    assert cfg.robot.pose.yaw is None


def test_preset_and_extra_geometry_combine():
    cfg = parse_config(
        {
            "dynamics": {"x_dim": 2, "u_dim": 2},
            "environment": {"bounds": [[-5, -5, -5], [5, 5, 5]]},
            "robot": {
                "preset": "puck",
                "geometry": [{"type": "sphere", "center": [0, 0, 1], "radius": 0.2}],
            },
        }
    )
    assert len(cfg.robot.shapes) == 2


def test_unknown_preset_is_rejected():
    with pytest.raises(ValueError, match="Unknown robot preset"):
        parse_config(
            {
                "dynamics": {"x_dim": 2, "u_dim": 2},
                "environment": {"bounds": [[-1, -1, -1], [1, 1, 1]]},
                "robot": {"preset": "hovercraft"},
            }
        )


def test_pose_axis_accepts_none_spellings():
    for spelling in ("none", "off", False):
        cfg = parse_config(
            {
                "dynamics": {"x_dim": 2, "u_dim": 1},
                "environment": {"bounds": [[-1, -1, -1], [1, 1, 1]]},
                "robot": {"pose_from_state": {"x": 0, "y": spelling}},
            }
        )
        assert cfg.robot.pose.y is None
    with pytest.raises(ValueError, match="Invalid pose index"):
        parse_config(
            {
                "dynamics": {"x_dim": 2, "u_dim": 1},
                "environment": {"bounds": [[-1, -1, -1], [1, 1, 1]]},
                "robot": {"pose_from_state": {"y": "sideways"}},
            }
        )


# --------------------------------------------------------------------------- #
# Bounded connect attempts
# --------------------------------------------------------------------------- #
def test_max_connect_attempts_bounds_expensive_calls():
    """Systems that reject most connections must not do unbounded work.

    Without a cap, a tree of n nodes can trigger O(n) connect() calls per
    iteration when control bounds reject nearly every candidate.
    """
    from krrtstar.planner import KRRTStar

    base = AnalyticLinearDynamics(np.zeros((2, 2)), np.eye(2))

    class Counting:
        """Mostly-infeasible dynamics: only every 4th connection is usable.

        This mimics a system whose control bounds reject most connections, so the
        tree still grows but the planner keeps scanning candidates.
        """

        x_dim, u_dim = 2, 2

        def __init__(self):
            self.connects = 0

        def cost(self, x0, x1):
            return base.cost(x0, x1)

        def cost_batch_to(self, states, x1):
            return base.cost_batch_to(states, x1)

        def cost_batch_from(self, x0, states):
            return base.cost_batch_from(x0, states)

        def connect(self, x0, x1):
            self.connects += 1
            if self.connects % 4:
                return None
            return base.connect(x0, x1)

        def u_bounds(self):
            return None

    def run(cap):
        dyn = Counting()
        planner = KRRTStar(
            dynamics=dyn,
            state_bounds=np.array([[-5, -5], [5, 5]]),
            x_init=np.array([0.0, 0.0]),
            x_goal=np.array([4.0, 4.0]),
            connection_radius=50.0,
            seed=1,
            max_connect_attempts=cap,
        )
        planner.grow(60)
        return dyn.connects

    capped = run(3)
    uncapped = run(None)
    # Two connect phases per iteration (parent choice, rewire), each capped.
    assert capped <= 2 * 3 * 60
    assert capped < uncapped, (capped, uncapped)


def test_max_connect_attempts_parses_from_config():
    cfg = parse_config(
        {
            "dynamics": {"x_dim": 2, "u_dim": 2},
            "environment": {"bounds": [[-1, -1, -1], [1, 1, 1]]},
            "planner": {"max_connect_attempts": 12},
        }
    )
    assert cfg.planner.max_connect_attempts == 12
