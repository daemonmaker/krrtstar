"""3D geometry primitives, robot model, and collision checking.

Robot and obstacle geometry are described entirely by configuration data. The
robot is a set of primitives defined in the robot frame; its world placement is
derived from the planner state through a configurable index mapping (which state
components are x/y/z). This replaces Callisto's compiled robot/world model.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np


@dataclass
class Sphere:
    center: np.ndarray  # (3,) in the owning frame
    radius: float

    def translated(self, offset: np.ndarray) -> "Sphere":
        return Sphere(np.asarray(self.center, float) + offset, self.radius)

    def aabb(self):
        c = np.asarray(self.center, float)
        r = self.radius
        return c - r, c + r


@dataclass
class Box:
    center: np.ndarray  # (3,)
    extents: np.ndarray  # (3,) full side lengths

    def translated(self, offset: np.ndarray) -> "Box":
        return Box(np.asarray(self.center, float) + offset, np.asarray(self.extents, float))

    def aabb(self):
        c = np.asarray(self.center, float)
        h = np.asarray(self.extents, float) / 2.0
        return c - h, c + h


Shape = object  # Sphere | Box


def _closest_point_on_box(point: np.ndarray, box: Box) -> np.ndarray:
    lo, hi = box.aabb()
    return np.minimum(np.maximum(point, lo), hi)


def _sphere_box_collide(sphere: Sphere, box: Box) -> bool:
    closest = _closest_point_on_box(np.asarray(sphere.center, float), box)
    return float(np.linalg.norm(np.asarray(sphere.center, float) - closest)) <= sphere.radius


def _sphere_sphere_collide(a: Sphere, b: Sphere) -> bool:
    d = float(np.linalg.norm(np.asarray(a.center, float) - np.asarray(b.center, float)))
    return d <= a.radius + b.radius


def _box_box_collide(a: Box, b: Box) -> bool:
    alo, ahi = a.aabb()
    blo, bhi = b.aabb()
    return bool(np.all(alo <= bhi) and np.all(blo <= ahi))


def shapes_collide(a: Shape, b: Shape) -> bool:
    """Narrow-phase collision test between two supported primitives."""
    if isinstance(a, Sphere) and isinstance(b, Sphere):
        return _sphere_sphere_collide(a, b)
    if isinstance(a, Sphere) and isinstance(b, Box):
        return _sphere_box_collide(a, b)
    if isinstance(a, Box) and isinstance(b, Sphere):
        return _sphere_box_collide(b, a)
    if isinstance(a, Box) and isinstance(b, Box):
        return _box_box_collide(a, b)
    raise TypeError(f"Unsupported shape pair: {type(a)}, {type(b)}")


@dataclass
class PoseMapping:
    """Maps a planner state to a world-frame translation of the robot."""

    x: int = 0
    y: int = 1
    z: Optional[int] = None

    def position(self, state: np.ndarray) -> np.ndarray:
        state = np.asarray(state, float).reshape(-1)
        px = state[self.x]
        py = state[self.y]
        pz = state[self.z] if self.z is not None else 0.0
        return np.array([px, py, pz], dtype=float)


@dataclass
class Robot:
    """A robot as a collection of primitives plus a state->pose mapping."""

    shapes: Sequence[Shape] = field(default_factory=list)
    pose: PoseMapping = field(default_factory=PoseMapping)

    def placed_shapes(self, state: np.ndarray) -> List[Shape]:
        offset = self.pose.position(state)
        return [s.translated(offset) for s in self.shapes]


@dataclass
class Environment:
    """World bounds and static obstacle primitives."""

    bounds: np.ndarray  # (2, 3): [[xmin,ymin,zmin],[xmax,ymax,zmax]]
    obstacles: Sequence[Shape] = field(default_factory=list)


class CollisionChecker:
    """Broad-phase AABB reject + narrow-phase primitive tests."""

    def __init__(self, robot: Robot, environment: Environment) -> None:
        self.robot = robot
        self.environment = environment
        self._obs_aabbs = [o.aabb() for o in environment.obstacles]

    def in_collision(self, state: np.ndarray) -> bool:
        placed = self.robot.placed_shapes(state)
        for shape in placed:
            slo, shi = shape.aabb()
            for obs, (olo, ohi) in zip(self.environment.obstacles, self._obs_aabbs):
                # Broad phase: skip clearly separated pairs.
                if np.any(shi < olo) or np.any(slo > ohi):
                    continue
                if shapes_collide(shape, obs):
                    return True
        return False

    def trajectory_in_collision(self, states: np.ndarray) -> bool:
        """Sampled collision check along a trajectory's states."""
        for state in np.asarray(states, float):
            if self.in_collision(state):
                return True
        return False
