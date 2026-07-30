import os

import numpy as np

from krrtstar.config import build_collision_checker, build_dynamics, load_config
from krrtstar.dynamics.linear import AnalyticLinearDynamics
from krrtstar.experiment import load_experiment, save_experiment
from krrtstar.planner import KRRTStar

EXAMPLE = os.path.join(os.path.dirname(__file__), "..", "examples", "double_integrator_2d.toml")


def test_load_example_config_and_build():
    cfg = load_config(EXAMPLE)
    assert cfg.dynamics.kind == "linear"
    assert cfg.dynamics.x_dim == 4
    dyn = build_dynamics(cfg.dynamics)
    assert isinstance(dyn, AnalyticLinearDynamics)
    checker = build_collision_checker(cfg)
    # a point on the central wall should collide
    assert checker.in_collision(np.array([0.0, 0.0, 0.0, 0.0]))


def test_experiment_round_trip(tmp_path):
    dyn = AnalyticLinearDynamics(np.zeros((2, 2)), np.eye(2))
    planner = KRRTStar(
        dynamics=dyn,
        state_bounds=np.array([[-5, -5], [5, 5]]),
        x_init=np.array([-3.0, -3.0]),
        x_goal=np.array([3.0, 3.0]),
        connection_radius=8.0,
        euclidean_gate=8.0,
        seed=11,
    )
    result = planner.grow(80)
    bundle = str(tmp_path / "exp")
    save_experiment(bundle, result, config_path=EXAMPLE)

    assert os.path.exists(os.path.join(bundle, "tree.npz"))
    assert os.path.exists(os.path.join(bundle, "meta.json"))
    assert os.path.exists(os.path.join(bundle, "config.toml"))

    tree, meta = load_experiment(bundle)
    assert len(tree.nodes) == len(result.tree.nodes)
    assert meta["x_dim"] == 2
    assert meta["found"] == result.found
    # states preserved
    assert np.allclose(tree.states, result.tree.states)
