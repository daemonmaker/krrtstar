"""Configuration schema and loader.

An experiment is fully described by a TOML file with ``[dynamics]``,
``[robot]``, ``[environment]``, ``[planner]`` and ``[visualization]`` sections.
This module parses that data into validated dataclasses and constructs the
runtime objects (dynamics backend, robot, environment, collision checker).
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np

try:  # Python 3.11+
    import tomllib
except ModuleNotFoundError:  # pragma: no cover
    import tomli as tomllib  # type: ignore

from .dynamics.linear import AnalyticLinearDynamics
from .geometry import Box, CollisionChecker, Environment, PoseMapping, Robot, Sphere


@dataclass
class DynamicsConfig:
    kind: str
    x_dim: int
    u_dim: int
    A: Optional[np.ndarray] = None
    B: Optional[np.ndarray] = None
    c: Optional[np.ndarray] = None
    R: Optional[np.ndarray] = None
    u_bounds: Optional[np.ndarray] = None
    model_path: Optional[str] = None
    connect_method: str = "gradient"
    extra: Dict[str, Any] = field(default_factory=dict)


@dataclass
class PlannerConfig:
    target_nodes: int = 1000
    connection_radius: float = 10.0
    goal_bias: float = 0.1
    goal_tolerance: float = 0.5
    rewire: bool = True
    # Euclidean pre-filter for candidate neighbours: a number, the string
    # "auto" to calibrate it from the dynamics, or None for an exact near set.
    euclidean_gate: Optional[Any] = None
    seed: int = 0
    x_init: Optional[np.ndarray] = None
    x_goal: Optional[np.ndarray] = None
    # Near-set radius schedule: "constant" (default), "rrtstar" or "geometric".
    radius_schedule: str = "constant"
    radius_gamma: Optional[float] = None
    radius_multiplier: float = 1.0
    radius_min: Optional[float] = None
    radius_max: Optional[float] = None


@dataclass
class VisualizationConfig:
    live: bool = False
    show_tree: bool = True
    show_nodes: bool = False
    show_robot: bool = True
    show_obstacles: bool = True
    # Hold the window open after planning finishes so the result can be
    # inspected; disable for unattended runs.
    keep_open: bool = True


@dataclass
class ExperimentConfig:
    dynamics: DynamicsConfig
    planner: PlannerConfig
    visualization: VisualizationConfig
    robot: Robot
    environment: Environment
    raw: Dict[str, Any] = field(default_factory=dict)
    base_dir: str = "."


def _arr(value, dtype=float):
    return None if value is None else np.asarray(value, dtype=dtype)


def _parse_dynamics(data: Dict[str, Any]) -> DynamicsConfig:
    kind = data.get("kind", "linear")
    return DynamicsConfig(
        kind=kind,
        x_dim=int(data["x_dim"]),
        u_dim=int(data["u_dim"]),
        A=_arr(data.get("A")),
        B=_arr(data.get("B")),
        c=_arr(data.get("c")),
        R=_arr(data.get("R")),
        u_bounds=_arr(data.get("u_bounds")),
        model_path=data.get("model_path"),
        connect_method=data.get("connect_method", "gradient"),
        extra={k: v for k, v in data.items() if k not in {
            "kind", "x_dim", "u_dim", "A", "B", "c", "R", "u_bounds",
            "model_path", "connect_method",
        }},
    )


def _parse_shape(data: Dict[str, Any]):
    t = data["type"]
    if t == "sphere":
        center = _arr(data.get("center", [0, 0, 0]))
        return Sphere(center=center, radius=float(data["radius"]))
    if t == "box":
        return Box(center=_arr(data["center"]), extents=_arr(data["extents"]))
    raise ValueError(f"Unsupported shape type: {t}")


def _parse_robot(data: Dict[str, Any]) -> Robot:
    shapes = [_parse_shape(s) for s in data.get("geometry", [])]
    pose_data = data.get("pose_from_state", {})
    pose = PoseMapping(
        x=int(pose_data.get("x", 0)),
        y=int(pose_data.get("y", 1)),
        z=None if pose_data.get("z") is None else int(pose_data["z"]),
    )
    return Robot(shapes=shapes, pose=pose)


def _parse_environment(data: Dict[str, Any]) -> Environment:
    bounds = _arr(data["bounds"])
    obstacles = [_parse_shape(s) for s in data.get("obstacles", [])]
    return Environment(bounds=bounds, obstacles=obstacles)


def _parse_gate(value):
    """Parse ``euclidean_gate``: a number, ``"auto"``, ``"none"``, or unset."""
    if value is None:
        return None
    if isinstance(value, str):
        text = value.strip().lower()
        if text == "auto":
            return "auto"
        if text in {"none", "off", "exact"}:
            return None
        raise ValueError(
            f"Invalid euclidean_gate {value!r}; expected a number, 'auto' or 'none'"
        )
    return float(value)


def _parse_planner(data: Dict[str, Any]) -> PlannerConfig:
    return PlannerConfig(
        target_nodes=int(data.get("target_nodes", 1000)),
        connection_radius=float(data.get("connection_radius", data.get("start_radius", 10.0))),
        goal_bias=float(data.get("goal_bias", 0.1)),
        goal_tolerance=float(data.get("goal_tolerance", 0.5)),
        rewire=bool(data.get("rewire", True)),
        euclidean_gate=_parse_gate(data.get("euclidean_gate")),
        seed=int(data.get("seed", 0)),
        x_init=_arr(data.get("x_init")),
        x_goal=_arr(data.get("x_goal")),
        radius_schedule=str(data.get("radius_schedule", "constant")),
        radius_gamma=(None if data.get("radius_gamma") is None else float(data["radius_gamma"])),
        radius_multiplier=float(data.get("radius_multiplier", 1.0)),
        radius_min=(None if data.get("radius_min") is None else float(data["radius_min"])),
        radius_max=(None if data.get("radius_max") is None else float(data["radius_max"])),
    )


def _parse_visualization(data: Dict[str, Any]) -> VisualizationConfig:
    return VisualizationConfig(
        live=bool(data.get("live", False)),
        show_tree=bool(data.get("show_tree", True)),
        show_nodes=bool(data.get("show_nodes", False)),
        show_robot=bool(data.get("show_robot", True)),
        show_obstacles=bool(data.get("show_obstacles", True)),
        keep_open=bool(data.get("keep_open", True)),
    )


def parse_config(data: Dict[str, Any], base_dir: str = ".") -> ExperimentConfig:
    return ExperimentConfig(
        dynamics=_parse_dynamics(data.get("dynamics", {})),
        planner=_parse_planner(data.get("planner", {})),
        visualization=_parse_visualization(data.get("visualization", {})),
        robot=_parse_robot(data.get("robot", {})),
        environment=_parse_environment(data.get("environment", {"bounds": [[-10, -10, -10], [10, 10, 10]]})),
        raw=data,
        base_dir=base_dir,
    )


def load_config(path: str) -> ExperimentConfig:
    with open(path, "rb") as fh:
        data = tomllib.load(fh)
    return parse_config(data, base_dir=os.path.dirname(os.path.abspath(path)))


def build_dynamics(cfg: DynamicsConfig):
    """Instantiate the dynamics backend selected by the config."""
    if cfg.kind == "linear":
        if cfg.A is None or cfg.B is None:
            raise ValueError("linear dynamics require A and B matrices")
        return AnalyticLinearDynamics(
            A=cfg.A, B=cfg.B, c=cfg.c, R=cfg.R, u_bounds=cfg.u_bounds,
            **{k: cfg.extra[k] for k in ("tau_max", "n_traj_samples") if k in cfg.extra},
        )
    if cfg.kind == "torch":
        from .dynamics.torch_dynamics import TorchDynamics

        return TorchDynamics(
            model_path=cfg.model_path,
            x_dim=cfg.x_dim,
            u_dim=cfg.u_dim,
            u_bounds=cfg.u_bounds,
            connect_method=cfg.connect_method,
        )
    raise ValueError(f"Unknown dynamics kind: {cfg.kind}")


def build_collision_checker(cfg: ExperimentConfig) -> CollisionChecker:
    return CollisionChecker(cfg.robot, cfg.environment)
