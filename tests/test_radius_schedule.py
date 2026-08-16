import math

import numpy as np
import pytest

from krrtstar.dynamics.linear import AnalyticLinearDynamics
from krrtstar.planner import KRRTStar
from krrtstar.radius import (
    ConstantRadius,
    GeometricRadius,
    RRTStarRadius,
    build_schedule,
    derive_gamma,
)


# --------------------------------------------------------------------------- #
# Schedules
# --------------------------------------------------------------------------- #
def test_constant_schedule_is_flat():
    sched = ConstantRadius(7.5)
    assert [sched.radius(n) for n in (1, 10, 1000)] == [7.5, 7.5, 7.5]


def test_rrtstar_schedule_shrinks_monotonically():
    sched = RRTStarRadius(gamma=20.0, dim=4)
    values = [sched.radius(n) for n in (10, 50, 200, 1000, 5000)]
    assert all(a > b for a, b in zip(values, values[1:])), values


def test_rrtstar_schedule_matches_formula():
    sched = RRTStarRadius(gamma=12.0, dim=3)
    n = 250
    expected = 12.0 * (math.log(n) / n) ** (1.0 / 3.0)
    assert sched.radius(n) == pytest.approx(expected)


def test_rrtstar_schedule_handles_tiny_trees():
    # log(1) == 0 would zero the radius and stall the search on the first step.
    sched = RRTStarRadius(gamma=10.0, dim=2)
    assert sched.radius(0) > 0.0
    assert sched.radius(1) == sched.radius(2)


def test_rrtstar_schedule_respects_bounds():
    sched = RRTStarRadius(gamma=1000.0, dim=2, r_min=2.0, r_max=5.0)
    assert sched.radius(2) == 5.0        # clamped above
    assert sched.radius(10 ** 9) == 2.0  # clamped below


def test_derive_gamma_pins_reference_point():
    gamma = derive_gamma(reference_radius=12.0, dim=4, n_reference=300)
    sched = RRTStarRadius(gamma=gamma, dim=4)
    assert sched.radius(300) == pytest.approx(12.0)
    # more generous earlier on, tighter later
    assert sched.radius(50) > 12.0
    assert sched.radius(3000) < 12.0


def test_geometric_schedule_matches_legacy_multiplier():
    shrink = GeometricRadius(r0=10.0, multiplier=0.99)
    assert shrink.radius(0) == pytest.approx(10.0)
    assert shrink.radius(100) == pytest.approx(10.0 * 0.99 ** 100)
    grow = GeometricRadius(r0=10.0, multiplier=1.01, r_max=12.0)
    assert grow.radius(1000) == 12.0  # clamped


# --------------------------------------------------------------------------- #
# build_schedule / config surface
# --------------------------------------------------------------------------- #
def test_build_schedule_defaults_to_constant():
    sched = build_schedule("constant", connection_radius=9.0, dim=4, target_nodes=100)
    assert isinstance(sched, ConstantRadius)
    assert sched.radius(500) == 9.0


def test_build_schedule_rrtstar_shrinks_below_the_reference_radius():
    """The default gamma must let the radius drop under connection_radius.

    Pinning gamma at target_nodes would keep r >= connection_radius for the
    whole run, which measurably only costs time.
    """
    sched = build_schedule("rrtstar", connection_radius=12.0, dim=4, target_nodes=600)
    assert isinstance(sched, RRTStarRadius)
    assert sched.radius(600) < 12.0
    assert sched.radius(10) > 12.0
    # crossing point is early, around a quarter of the target
    assert sched.radius(150) == pytest.approx(12.0, rel=0.05)


def test_build_schedule_rrtstar_defaults_to_a_floor():
    sched = build_schedule("rrtstar", connection_radius=12.0, dim=4, target_nodes=600)
    assert sched.r_min == pytest.approx(6.0)
    assert sched.radius(10 ** 9) == pytest.approx(6.0)


def test_build_schedule_honours_explicit_gamma():
    sched = build_schedule(
        "rrtstar", connection_radius=9.0, dim=4, target_nodes=400, gamma=42.0
    )
    assert sched.gamma == pytest.approx(42.0)


def test_build_schedule_rejects_unknown_kind():
    with pytest.raises(ValueError, match="Unknown radius schedule"):
        build_schedule("magic", connection_radius=1.0, dim=2, target_nodes=10)


def test_config_parses_radius_schedule():
    from krrtstar.config import parse_config

    cfg = parse_config(
        {
            "dynamics": {"x_dim": 2, "u_dim": 2},
            "environment": {"bounds": [[-1, -1, -1], [1, 1, 1]]},
            "planner": {
                "connection_radius": 8.0,
                "radius_schedule": "rrtstar",
                "radius_gamma": 15.0,
                "radius_min": 1.0,
                "radius_max": 20.0,
                "radius_multiplier": 0.995,
            },
        }
    )
    assert cfg.planner.radius_schedule == "rrtstar"
    assert cfg.planner.radius_gamma == pytest.approx(15.0)
    assert cfg.planner.radius_min == pytest.approx(1.0)
    assert cfg.planner.radius_max == pytest.approx(20.0)
    assert cfg.planner.radius_multiplier == pytest.approx(0.995)


def test_config_defaults_preserve_constant_behaviour():
    from krrtstar.config import parse_config

    cfg = parse_config(
        {
            "dynamics": {"x_dim": 2, "u_dim": 2},
            "environment": {"bounds": [[-1, -1, -1], [1, 1, 1]]},
            "planner": {"connection_radius": 8.0},
        }
    )
    assert cfg.planner.radius_schedule == "constant"


# --------------------------------------------------------------------------- #
# Planner integration
# --------------------------------------------------------------------------- #
def _planner(schedule=None, seed=1, radius=8.0):
    dyn = AnalyticLinearDynamics(np.zeros((2, 2)), np.eye(2))
    return KRRTStar(
        dynamics=dyn,
        state_bounds=np.array([[-5, -5], [5, 5]]),
        x_init=np.array([-4.0, -4.0]),
        x_goal=np.array([4.0, 4.0]),
        connection_radius=radius,
        goal_bias=0.15,
        goal_tolerance=1.0,
        euclidean_gate=8.0,
        seed=seed,
        radius_schedule=schedule,
    )


def test_planner_defaults_to_constant_schedule():
    planner = _planner()
    assert isinstance(planner.schedule, ConstantRadius)
    assert planner.radius == pytest.approx(8.0)


def test_planner_radius_tracks_schedule_during_growth():
    observed = []

    class Recording:
        def radius(self, n_nodes):
            observed.append(n_nodes)
            return 8.0

    planner = _planner(schedule=Recording())
    planner.grow(25)
    # the schedule is consulted every iteration, with a growing tree size
    assert len(observed) >= 25
    assert observed[0] <= observed[-1]


def test_planner_finds_path_with_shrinking_radius():
    schedule = build_schedule("rrtstar", connection_radius=8.0, dim=2, target_nodes=200)
    planner = _planner(schedule=schedule)
    result = planner.grow(200)
    assert result.found
    assert result.cost is not None and result.cost > 0


def test_shrinking_radius_keeps_a_valid_cost_tree():
    schedule = build_schedule("rrtstar", connection_radius=8.0, dim=2, target_nodes=150)
    result = _planner(schedule=schedule, seed=4).grow(150)
    for node in result.tree.nodes:
        if node.parent != -1:
            parent = result.tree.nodes[node.parent]
            assert node.cost_from_start >= parent.cost_from_start - 1e-6


def test_radius_floor_prevents_collapse():
    """With a hard floor the planner must still connect nodes."""
    schedule = RRTStarRadius(gamma=1e-6, dim=2, r_min=6.0)
    result = _planner(schedule=schedule, seed=7).grow(60)
    assert len(result.tree.nodes) > 1  # would be 1 if nothing could connect


# --------------------------------------------------------------------------- #
# Euclidean gate calibration
# --------------------------------------------------------------------------- #
def _di2d():
    A = np.array(
        [[0.0, 0.0, 1.0, 0.0],
         [0.0, 0.0, 0.0, 1.0],
         [0.0, 0.0, 0.0, 0.0],
         [0.0, 0.0, 0.0, 0.0]]
    )
    B = np.array([[0.0, 0.0], [0.0, 0.0], [1.0, 0.0], [0.0, 1.0]])
    return AnalyticLinearDynamics(A, B, R=np.eye(2))


def test_calibrated_gate_covers_every_in_radius_pair():
    """The calibrated gate must not discard candidates inside the cost radius.

    A hand-picked gate can silently drop most of the near set, which costs
    solution quality; this is the regression guard for that.
    """
    from krrtstar.radius import calibrate_euclidean_gate

    dyn = _di2d()
    bounds = np.array([[-5, -5, -3, -3], [5, 5, 3, 3]], dtype=float)
    radius = 12.0
    gate = calibrate_euclidean_gate(dyn, bounds, radius, samples=200, seed=1)

    rng = np.random.default_rng(7)
    starts = rng.uniform(bounds[0], bounds[1], size=(300, 4))
    worst = 0.0
    for _ in range(20):
        target = rng.uniform(bounds[0], bounds[1])
        costs = dyn.cost_batch_to(starts, target)
        within = np.isfinite(costs) & (costs <= radius)
        if within.any():
            worst = max(worst, float(np.linalg.norm(starts[within] - target, axis=1).max()))
    assert gate >= worst, f"gate {gate} would discard in-radius candidates up to {worst}"


def test_calibrated_gate_exceeds_a_naive_small_gate():
    from krrtstar.radius import calibrate_euclidean_gate

    dyn = _di2d()
    bounds = np.array([[-5, -5, -3, -3], [5, 5, 3, 3]], dtype=float)
    gate = calibrate_euclidean_gate(dyn, bounds, 12.0, samples=150, seed=0)
    # The example configs previously shipped a gate of 6.0, which is far too
    # small for a cost radius of 12 in this system.
    assert gate > 6.0


def test_gate_config_accepts_auto_and_none():
    from krrtstar.config import parse_config

    def gate_of(value):
        data = {
            "dynamics": {"x_dim": 2, "u_dim": 2},
            "environment": {"bounds": [[-1, -1, -1], [1, 1, 1]]},
            "planner": {"euclidean_gate": value},
        }
        return parse_config(data).planner.euclidean_gate

    assert gate_of("auto") == "auto"
    assert gate_of("none") is None
    assert gate_of(7.5) == pytest.approx(7.5)
    with pytest.raises(ValueError, match="Invalid euclidean_gate"):
        gate_of("sometimes")


def test_auto_gate_is_resolved_to_a_number_by_the_driver():
    from krrtstar.config import load_config
    from krrtstar.run import build_planner
    import os

    cfg = load_config(
        os.path.join(os.path.dirname(__file__), "..", "examples", "single_integrator_2d.toml")
    )
    cfg.planner.euclidean_gate = "auto"
    cfg.planner.target_nodes = 20
    planner = build_planner(cfg)
    assert isinstance(planner.euclidean_gate, float)
    assert planner.euclidean_gate > 0.0
