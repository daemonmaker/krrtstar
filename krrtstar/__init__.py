"""krrtstar: hybrid Python + Rust kinodynamic RRT* planner.

Dynamics, robot geometry, obstacle environments, and experiments are all defined
by configuration data rather than compiled per model.
"""

from __future__ import annotations

__version__ = "0.1.0"

from .dynamics.base import Dynamics, Trajectory
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
from .planner import KRRTStar, Node, PlanResult, Tree

__all__ = [
    "__version__",
    "Dynamics",
    "Trajectory",
    "AnalyticLinearDynamics",
    "Box",
    "Capsule",
    "Cylinder",
    "Mesh",
    "Sphere",
    "Robot",
    "Environment",
    "PoseMapping",
    "CollisionChecker",
    "load_mesh",
    "rotation_from_euler",
    "rotation_from_quat",
    "KRRTStar",
    "Tree",
    "Node",
    "PlanResult",
]
