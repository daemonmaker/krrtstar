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
    CollisionChecker,
    Environment,
    PoseMapping,
    Robot,
    Sphere,
)
from .planner import KRRTStar, Node, PlanResult, Tree

__all__ = [
    "__version__",
    "Dynamics",
    "Trajectory",
    "AnalyticLinearDynamics",
    "Box",
    "Sphere",
    "Robot",
    "Environment",
    "PoseMapping",
    "CollisionChecker",
    "KRRTStar",
    "Tree",
    "Node",
    "PlanResult",
]
