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
from .geometry import (
    Box,
    Capsule,
    CollisionChecker,
    Cylinder,
    Environment,
    Mesh,
    PoseMapping,
    Robot,
    Sphere,
    load_mesh,
    rotation_from_euler,
    rotation_from_quat,
)


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
    euclidean_gate: Optional[float] = None
    seed: int = 0
    # Caps expensive connect() calls per iteration; None means unbounded.
    max_connect_attempts: Optional[int] = None
    # (weight, bounds) regions to bias sampling toward, e.g. narrow passages.
    sample_regions: List[Any] = field(default_factory=list)
    x_init: Optional[np.ndarray] = None
    x_goal: Optional[np.ndarray] = None


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


def _parse_rotation(data: Dict[str, Any]):
    """Read an optional orientation from a shape entry.

    Accepts ``rotation`` as Euler angles ``[roll, pitch, yaw]``, a quaternion
    ``[w, x, y, z]``, or a full 3x3 matrix; ``euler`` and ``quaternion`` are
    accepted as explicit aliases.
    """
    if "euler" in data:
        return rotation_from_euler(*[float(v) for v in data["euler"]])
    if "quaternion" in data:
        return rotation_from_quat([float(v) for v in data["quaternion"]])
    if "rotation" in data:
        return _arr(data["rotation"])  # geometry normalizes 3 / 4 / 3x3
    return None


def _parse_shape(data: Dict[str, Any], base_dir: str = "."):
    t = data["type"]
    rotation = _parse_rotation(data)
    if t == "sphere":
        return Sphere(center=_arr(data.get("center", [0, 0, 0])), radius=float(data["radius"]))
    if t == "box":
        return Box(center=_arr(data["center"]), extents=_arr(data["extents"]), rotation=rotation)
    if t == "capsule":
        return Capsule(
            center=_arr(data.get("center", [0, 0, 0])),
            radius=float(data["radius"]),
            height=float(data["height"]),
            rotation=rotation,
        )
    if t == "cylinder":
        return Cylinder(
            center=_arr(data.get("center", [0, 0, 0])),
            radius=float(data["radius"]),
            height=float(data["height"]),
            rotation=rotation,
        )
    if t == "mesh":
        path = data["path"]
        if not os.path.isabs(path):
            path = os.path.join(base_dir, path)
        vertices, faces = load_mesh(path)
        return Mesh(
            vertices=vertices,
            faces=faces,
            center=_arr(data.get("center", [0, 0, 0])),
            rotation=rotation,
            scale=float(data.get("scale", 1.0)),
        )
    raise ValueError(f"Unsupported shape type: {t}")


def _parse_pose_mapping(data: Dict[str, Any]) -> PoseMapping:
    """Read a state->pose index mapping.

    Each axis takes a state index. Write ``false`` or ``"none"`` to pin an axis
    to zero, which a system with fewer position dimensions than the world needs
    (e.g. a 1-D double integrator, whose index 1 is velocity, not ``y``).
    """

    def axis(key, default=None):
        if key not in data:
            return default
        value = data[key]
        if value is None or value is False:
            return None
        if isinstance(value, str):
            if value.strip().lower() in {"none", "off", "unused"}:
                return None
            raise ValueError(f"Invalid pose index for {key!r}: {value!r}")
        return int(value)

    quat = data.get("quat")
    return PoseMapping(
        x=axis("x", 0),
        y=axis("y", 1),
        z=axis("z"),
        roll=axis("roll"),
        pitch=axis("pitch"),
        yaw=axis("yaw"),
        quat=None if quat is None else [int(v) for v in quat],
    )


def _parse_robot(data: Dict[str, Any], base_dir: str = ".") -> Robot:
    """Build a robot from an optional preset plus any explicit geometry.

    ``preset`` names one of the models ported from the original implementation
    (see :mod:`krrtstar.robots`); its collision geometry is used, and its default
    state->pose mapping applies unless ``pose_from_state`` overrides it.
    """
    from .robots import collision_shapes, default_pose

    shapes = []
    preset = data.get("preset")
    if preset is not None:
        shapes.extend(collision_shapes(str(preset)))
    shapes.extend(_parse_shape(s, base_dir) for s in data.get("geometry", []))

    if "pose_from_state" in data:
        pose = _parse_pose_mapping(data["pose_from_state"])
    elif preset is not None:
        pose = default_pose(str(preset))
    else:
        pose = _parse_pose_mapping({})
    return Robot(shapes=shapes, pose=pose)


def _parse_environment(data: Dict[str, Any], base_dir: str = ".") -> Environment:
    bounds = _arr(data["bounds"])
    obstacles = [_parse_shape(s, base_dir) for s in data.get("obstacles", [])]
    return Environment(bounds=bounds, obstacles=obstacles)


def _parse_planner(data: Dict[str, Any]) -> PlannerConfig:
    return PlannerConfig(
        target_nodes=int(data.get("target_nodes", 1000)),
        connection_radius=float(data.get("connection_radius", data.get("start_radius", 10.0))),
        goal_bias=float(data.get("goal_bias", 0.1)),
        goal_tolerance=float(data.get("goal_tolerance", 0.5)),
        rewire=bool(data.get("rewire", True)),
        euclidean_gate=(None if data.get("euclidean_gate") is None else float(data["euclidean_gate"])),
        seed=int(data.get("seed", 0)),
        max_connect_attempts=(
            None if data.get("max_connect_attempts") is None
            else int(data["max_connect_attempts"])
        ),
        sample_regions=[
            (float(r["weight"]), _arr(r["bounds"])) for r in data.get("sample_regions", [])
        ],
        x_init=_arr(data.get("x_init")),
        x_goal=_arr(data.get("x_goal")),
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
        robot=_parse_robot(data.get("robot", {}), base_dir),
        environment=_parse_environment(
            data.get("environment", {"bounds": [[-10, -10, -10], [10, 10, 10]]}), base_dir
        ),
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
    if cfg.kind in {"nonholonomic", "linearized"}:
        from .dynamics.nonlinear import LinearizedDynamics, build_system

        name = cfg.extra.get("system", "nonholonomic" if cfg.kind == "nonholonomic" else None)
        if name is None:
            raise ValueError("linearized dynamics require a 'system' name")
        extra = {
            k: cfg.extra[k]
            for k in ("tau_max", "n_traj_samples", "max_endpoint_error")
            if k in cfg.extra
        }
        return LinearizedDynamics(
            build_system(str(name)), R=cfg.R, u_bounds=cfg.u_bounds, **extra
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
