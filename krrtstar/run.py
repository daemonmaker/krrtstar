"""High-level experiment driver.

Ties a parsed configuration to the planner, supporting tree growth with or
without live 3D visualization during growth.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from .config import (
    ExperimentConfig,
    build_collision_checker,
    build_dynamics,
    load_config,
)
from .planner import KRRTStar, PlanResult


def build_planner(cfg: ExperimentConfig) -> KRRTStar:
    dynamics = build_dynamics(cfg.dynamics)
    collision = build_collision_checker(cfg)
    planner_cfg = cfg.planner
    if planner_cfg.x_init is None or planner_cfg.x_goal is None:
        raise ValueError("planner config requires x_init and x_goal")
    return KRRTStar(
        dynamics=dynamics,
        state_bounds=_state_bounds(cfg),
        x_init=planner_cfg.x_init,
        x_goal=planner_cfg.x_goal,
        collision=collision,
        connection_radius=planner_cfg.connection_radius,
        goal_bias=planner_cfg.goal_bias,
        goal_tolerance=planner_cfg.goal_tolerance,
        rewire=planner_cfg.rewire,
        euclidean_gate=planner_cfg.euclidean_gate,
        seed=planner_cfg.seed,
        max_connect_attempts=planner_cfg.max_connect_attempts,
        sample_regions=planner_cfg.sample_regions,
    )


def _state_bounds(cfg: ExperimentConfig) -> np.ndarray:
    """Derive planner state bounds.

    Uses explicit ``[planner] state_bounds`` when present, otherwise falls back
    to a box around init/goal padded by the environment extent so sampling
    covers the workspace.
    """
    raw = cfg.raw.get("planner", {})
    if "state_bounds" in raw:
        return np.asarray(raw["state_bounds"], float)
    x_dim = cfg.dynamics.x_dim
    lo = np.full(x_dim, -1.0)
    hi = np.full(x_dim, 1.0)
    env = cfg.environment.bounds
    dim = min(env.shape[1], x_dim)
    lo[:dim] = env[0, :dim]
    hi[:dim] = env[1, :dim]
    return np.stack([lo, hi])


def run_experiment(cfg: ExperimentConfig, live_callback=None) -> PlanResult:
    """Build and grow a planner from a config, optionally visualizing live.

    When live visualization is enabled the viewer is finalized after growth so
    the solution is drawn and the window stays open for inspection.
    """
    planner = build_planner(cfg)
    progress_cb = None
    if cfg.visualization.live:
        if live_callback is not None:
            progress_cb = live_callback
        else:
            from .viz import make_live_callback

            progress_cb = make_live_callback(
                cfg, keep_open=cfg.visualization.keep_open
            )
    result = planner.grow(cfg.planner.target_nodes, progress_cb=progress_cb)

    # Viewers expose finish() to draw the solution and hold the window open.
    finish = getattr(progress_cb, "finish", None)
    if callable(finish):
        finish(result)
    return result


def run_from_file(path: str, live_callback=None) -> PlanResult:
    return run_experiment(load_config(path), live_callback=live_callback)
