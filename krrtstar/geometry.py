"""3D geometry primitives, robot model, and collision checking.

Robot and obstacle geometry are described entirely by configuration data. The
robot is a set of shapes defined in the robot frame; its world placement is
derived from the planner state through a configurable index mapping (which state
components give position and, optionally, orientation). This replaces Callisto's
compiled robot/world model.

Supported shapes: :class:`Sphere`, :class:`Box` (axis-aligned or oriented),
:class:`Capsule`, :class:`Cylinder`, and triangle :class:`Mesh`.

Every shape exposes a *core* convex set plus a *margin* (its "round" radius):

* sphere  -> core is a point,   margin = radius
* capsule -> core is a segment, margin = radius
* box     -> core is the (possibly oriented) box, margin = 0
* cylinder-> core is the cylinder,                margin = 0
* mesh    -> a soup of triangles (non-convex; handled per triangle), margin = 0

Two shapes intersect when the distance between their cores is at most the sum of
their margins. Cores are described by a support function, which is all the
GJK-based narrow phase in :mod:`krrtstar.collision` (and parry3d in the Rust
core) needs.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

# --------------------------------------------------------------------------- #
# Rotations
# --------------------------------------------------------------------------- #
IDENTITY = np.eye(3)


def rotation_from_euler(roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> np.ndarray:
    """Intrinsic Z-Y-X (yaw-pitch-roll) rotation matrix."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def rotation_from_quat(q: Sequence[float]) -> np.ndarray:
    """Rotation matrix from a quaternion ``(w, x, y, z)`` (normalized)."""
    w, x, y, z = (float(v) for v in q)
    n = np.sqrt(w * w + x * x + y * y + z * z)
    if n == 0.0:
        return IDENTITY.copy()
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _as_rotation(rotation) -> np.ndarray:
    if rotation is None:
        return IDENTITY.copy()
    rotation = np.asarray(rotation, dtype=float)
    if rotation.shape == (3, 3):
        return rotation
    if rotation.shape == (4,):
        return rotation_from_quat(rotation)
    if rotation.shape == (3,):  # roll, pitch, yaw
        return rotation_from_euler(*rotation)
    raise ValueError(f"Unsupported rotation specification with shape {rotation.shape}")


def _vec3(value, default=0.0) -> np.ndarray:
    if value is None:
        return np.full(3, default, dtype=float)
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size != 3:
        raise ValueError(f"Expected a 3-vector, got {arr.size} values")
    return arr


# --------------------------------------------------------------------------- #
# Shapes
# --------------------------------------------------------------------------- #
@dataclass
class Sphere:
    """A ball. Core set is its center point; the radius is the margin."""

    center: np.ndarray
    radius: float

    def __post_init__(self):
        self.center = _vec3(self.center)
        self.radius = float(self.radius)

    @property
    def margin(self) -> float:
        return self.radius

    def support(self, direction: np.ndarray) -> np.ndarray:
        return self.center

    def core_points(self) -> np.ndarray:
        return self.center.reshape(1, 3)

    def transformed(self, rotation, translation) -> "Sphere":
        rot = _as_rotation(rotation)
        return Sphere(rot @ self.center + _vec3(translation), self.radius)

    def translated(self, offset) -> "Sphere":
        return Sphere(self.center + _vec3(offset), self.radius)

    def aabb(self) -> Tuple[np.ndarray, np.ndarray]:
        return self.center - self.radius, self.center + self.radius


@dataclass
class Box:
    """A box, axis-aligned by default or oriented when ``rotation`` is given."""

    center: np.ndarray
    extents: np.ndarray  # full side lengths
    rotation: Optional[np.ndarray] = None

    def __post_init__(self):
        self.center = _vec3(self.center)
        self.extents = _vec3(self.extents)
        self.rotation = _as_rotation(self.rotation)

    @property
    def margin(self) -> float:
        return 0.0

    @property
    def half_extents(self) -> np.ndarray:
        return self.extents / 2.0

    def support(self, direction: np.ndarray) -> np.ndarray:
        # Farthest corner along `direction`: pick the sign of each local axis.
        local = self.rotation.T @ np.asarray(direction, float)
        signs = np.where(local >= 0.0, 1.0, -1.0)
        return self.center + self.rotation @ (signs * self.half_extents)

    def core_points(self) -> np.ndarray:
        h = self.half_extents
        corners = np.array(
            [[sx * h[0], sy * h[1], sz * h[2]]
             for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)],
            dtype=float,
        )
        return corners @ self.rotation.T + self.center

    def transformed(self, rotation, translation) -> "Box":
        rot = _as_rotation(rotation)
        return Box(rot @ self.center + _vec3(translation), self.extents, rot @ self.rotation)

    def translated(self, offset) -> "Box":
        return Box(self.center + _vec3(offset), self.extents, self.rotation)

    def aabb(self) -> Tuple[np.ndarray, np.ndarray]:
        pts = self.core_points()
        return pts.min(axis=0), pts.max(axis=0)

    def is_axis_aligned(self, tol: float = 1e-12) -> bool:
        return bool(np.allclose(self.rotation, IDENTITY, atol=tol))


@dataclass
class Capsule:
    """A capsule: a segment along the local Z axis, inflated by ``radius``."""

    center: np.ndarray
    radius: float
    height: float  # length of the inner segment (excludes the end caps)
    rotation: Optional[np.ndarray] = None

    def __post_init__(self):
        self.center = _vec3(self.center)
        self.radius = float(self.radius)
        self.height = float(self.height)
        self.rotation = _as_rotation(self.rotation)

    @property
    def margin(self) -> float:
        return self.radius

    def segment(self) -> Tuple[np.ndarray, np.ndarray]:
        axis = self.rotation @ np.array([0.0, 0.0, self.height / 2.0])
        return self.center - axis, self.center + axis

    def support(self, direction: np.ndarray) -> np.ndarray:
        a, b = self.segment()
        direction = np.asarray(direction, float)
        return a if float(direction @ a) >= float(direction @ b) else b

    def core_points(self) -> np.ndarray:
        a, b = self.segment()
        return np.stack([a, b])

    def transformed(self, rotation, translation) -> "Capsule":
        rot = _as_rotation(rotation)
        return Capsule(
            rot @ self.center + _vec3(translation), self.radius, self.height,
            rot @ self.rotation,
        )

    def translated(self, offset) -> "Capsule":
        return Capsule(self.center + _vec3(offset), self.radius, self.height, self.rotation)

    def aabb(self) -> Tuple[np.ndarray, np.ndarray]:
        pts = self.core_points()
        return pts.min(axis=0) - self.radius, pts.max(axis=0) + self.radius


@dataclass
class Cylinder:
    """A right circular cylinder whose axis is the local Z axis."""

    center: np.ndarray
    radius: float
    height: float  # full height
    rotation: Optional[np.ndarray] = None

    def __post_init__(self):
        self.center = _vec3(self.center)
        self.radius = float(self.radius)
        self.height = float(self.height)
        self.rotation = _as_rotation(self.rotation)

    @property
    def margin(self) -> float:
        return 0.0

    def support(self, direction: np.ndarray) -> np.ndarray:
        # Exact cylinder support: extreme on the rim in the local XY plane and
        # on the flat cap along local Z.
        local = self.rotation.T @ np.asarray(direction, float)
        radial = local[:2]
        norm = float(np.linalg.norm(radial))
        xy = (radial / norm) * self.radius if norm > 1e-15 else np.zeros(2)
        z = (self.height / 2.0) if local[2] >= 0.0 else -(self.height / 2.0)
        return self.center + self.rotation @ np.array([xy[0], xy[1], z])

    def core_points(self) -> np.ndarray:
        # Bounding sample: the two cap centers, used only for coarse bounds.
        axis = self.rotation @ np.array([0.0, 0.0, self.height / 2.0])
        return np.stack([self.center - axis, self.center + axis])

    def transformed(self, rotation, translation) -> "Cylinder":
        rot = _as_rotation(rotation)
        return Cylinder(
            rot @ self.center + _vec3(translation), self.radius, self.height,
            rot @ self.rotation,
        )

    def translated(self, offset) -> "Cylinder":
        return Cylinder(self.center + _vec3(offset), self.radius, self.height, self.rotation)

    def aabb(self) -> Tuple[np.ndarray, np.ndarray]:
        # Tight AABB of a cylinder: the axis extent plus the disc extent per axis.
        axis = self.rotation @ np.array([0.0, 0.0, self.height / 2.0])
        # Radial extent along world axis i is r * |sin(angle)| = r*sqrt(1 - u_i^2)
        u = self.rotation @ np.array([0.0, 0.0, 1.0])
        radial = self.radius * np.sqrt(np.maximum(0.0, 1.0 - u ** 2))
        extent = np.abs(axis) + radial
        return self.center - extent, self.center + extent


@dataclass
class Mesh:
    """A triangle mesh (possibly non-convex).

    ``vertices`` is ``(n, 3)`` and ``faces`` is ``(m, 3)`` of vertex indices.
    Non-convex meshes are handled triangle-by-triangle by the narrow phase.

    Semantics: a mesh is treated as a **surface**, not a filled solid. A shape
    that fits entirely inside a closed mesh without touching any triangle is
    therefore *not* in collision. Keep this in mind for large hollow meshes,
    where a trajectory could pass through the interior: model such obstacles
    with primitives (boxes/cylinders), or ensure the robot is large enough
    relative to the mesh that it must touch a face.
    """

    vertices: np.ndarray
    faces: np.ndarray
    center: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rotation: Optional[np.ndarray] = None
    scale: float = 1.0

    def __post_init__(self):
        self.vertices = np.asarray(self.vertices, dtype=float).reshape(-1, 3)
        self.faces = np.asarray(self.faces, dtype=np.int64).reshape(-1, 3)
        self.center = _vec3(self.center)
        self.rotation = _as_rotation(self.rotation)
        self.scale = float(self.scale)

    @property
    def margin(self) -> float:
        return 0.0

    def world_vertices(self) -> np.ndarray:
        return (self.vertices * self.scale) @ self.rotation.T + self.center

    def triangles(self) -> np.ndarray:
        """World-frame triangles as an ``(m, 3, 3)`` array."""
        return self.world_vertices()[self.faces]

    def support(self, direction: np.ndarray) -> np.ndarray:
        # Only meaningful for convex meshes; the narrow phase uses triangles().
        verts = self.world_vertices()
        return verts[int(np.argmax(verts @ np.asarray(direction, float)))]

    def core_points(self) -> np.ndarray:
        return self.world_vertices()

    def transformed(self, rotation, translation) -> "Mesh":
        rot = _as_rotation(rotation)
        return Mesh(
            self.vertices, self.faces, rot @ self.center + _vec3(translation),
            rot @ self.rotation, self.scale,
        )

    def translated(self, offset) -> "Mesh":
        return Mesh(self.vertices, self.faces, self.center + _vec3(offset),
                    self.rotation, self.scale)

    def aabb(self) -> Tuple[np.ndarray, np.ndarray]:
        verts = self.world_vertices()
        return verts.min(axis=0), verts.max(axis=0)


Shape = object  # Sphere | Box | Capsule | Cylinder | Mesh

PRIMITIVES = (Sphere, Box, Capsule, Cylinder, Mesh)


# --------------------------------------------------------------------------- #
# Mesh loading
# --------------------------------------------------------------------------- #
def load_obj(path: str) -> Tuple[np.ndarray, np.ndarray]:
    """Minimal OBJ reader returning ``(vertices, triangular_faces)``.

    Polygonal faces are fan-triangulated. Vertex texture/normal indices
    (``v/vt/vn``) are ignored.
    """
    vertices: List[List[float]] = []
    faces: List[List[int]] = []
    with open(path, "r") as fh:
        for line in fh:
            if line.startswith("v "):
                parts = line.split()
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif line.startswith("f "):
                idx = []
                for token in line.split()[1:]:
                    raw = token.split("/")[0]
                    i = int(raw)
                    # OBJ indices are 1-based; negatives count from the end.
                    idx.append(i - 1 if i > 0 else len(vertices) + i)
                for k in range(1, len(idx) - 1):
                    faces.append([idx[0], idx[k], idx[k + 1]])
    if not vertices or not faces:
        raise ValueError(f"No triangles found in OBJ file: {path}")
    return np.asarray(vertices, dtype=float), np.asarray(faces, dtype=np.int64)


def load_mesh(path: str) -> Tuple[np.ndarray, np.ndarray]:
    """Load a triangle mesh, returning ``(vertices, faces)``.

    ``.obj`` is handled natively. Other formats are read through ``trimesh`` or
    ``pyvista`` when either is installed.
    """
    ext = os.path.splitext(path)[1].lower()
    if ext == ".obj":
        return load_obj(path)

    try:
        import trimesh  # noqa: WPS433 (optional dependency)

        mesh = trimesh.load(path, force="mesh")
        return (
            np.asarray(mesh.vertices, dtype=float),
            np.asarray(mesh.faces, dtype=np.int64),
        )
    except ImportError:
        pass

    try:
        import pyvista  # noqa: WPS433 (optional dependency)

        surface = pyvista.read(path).triangulate().extract_surface()
        faces = surface.faces.reshape(-1, 4)[:, 1:]
        return (
            np.asarray(surface.points, dtype=float),
            np.asarray(faces, dtype=np.int64),
        )
    except ImportError as exc:
        raise ImportError(
            f"Cannot load '{ext}' meshes; install trimesh or pyvista, "
            "or supply an .obj file"
        ) from exc


# --------------------------------------------------------------------------- #
# Pose mapping
# --------------------------------------------------------------------------- #
@dataclass
class PoseMapping:
    """Maps a planner state to a world-frame rigid transform of the robot.

    Position comes from the ``x``/``y``/``z`` state indices. Orientation is
    optional: either Euler angles from the ``roll``/``pitch``/``yaw`` indices or
    a quaternion from four ``quat`` indices (``w, x, y, z``).
    """

    x: int = 0
    y: int = 1
    z: Optional[int] = None
    roll: Optional[int] = None
    pitch: Optional[int] = None
    yaw: Optional[int] = None
    quat: Optional[Sequence[int]] = None

    def position(self, state: np.ndarray) -> np.ndarray:
        state = np.asarray(state, float).reshape(-1)
        return np.array(
            [
                state[self.x],
                state[self.y],
                state[self.z] if self.z is not None else 0.0,
            ],
            dtype=float,
        )

    def rotation(self, state: np.ndarray) -> np.ndarray:
        state = np.asarray(state, float).reshape(-1)
        if self.quat is not None:
            return rotation_from_quat([state[i] for i in self.quat])
        if self.roll is None and self.pitch is None and self.yaw is None:
            return IDENTITY.copy()
        return rotation_from_euler(
            roll=state[self.roll] if self.roll is not None else 0.0,
            pitch=state[self.pitch] if self.pitch is not None else 0.0,
            yaw=state[self.yaw] if self.yaw is not None else 0.0,
        )

    @property
    def has_orientation(self) -> bool:
        return self.quat is not None or any(
            i is not None for i in (self.roll, self.pitch, self.yaw)
        )

    def transform(self, state: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        return self.rotation(state), self.position(state)


# --------------------------------------------------------------------------- #
# Robot / environment
# --------------------------------------------------------------------------- #
@dataclass
class Robot:
    """A robot as a collection of shapes plus a state->pose mapping."""

    shapes: Sequence[Shape] = field(default_factory=list)
    pose: PoseMapping = field(default_factory=PoseMapping)

    def placed_shapes(self, state: np.ndarray) -> List[Shape]:
        rotation, translation = self.pose.transform(state)
        return [s.transformed(rotation, translation) for s in self.shapes]


@dataclass
class Environment:
    """World bounds and static obstacle shapes."""

    bounds: np.ndarray  # (2, 3): [[xmin,ymin,zmin],[xmax,ymax,zmax]]
    obstacles: Sequence[Shape] = field(default_factory=list)


def shapes_collide(a: Shape, b: Shape) -> bool:
    """Narrow-phase collision test between two supported shapes."""
    from .collision import shapes_intersect

    return shapes_intersect(a, b)


class CollisionChecker:
    """Broad-phase AABB reject plus narrow-phase shape tests.

    Uses the native (Rust/parry3d) collision scene when the compiled core is
    available, and the pure-Python GJK narrow phase otherwise.
    """

    def __init__(self, robot: Robot, environment: Environment, use_accel: bool = True) -> None:
        self.robot = robot
        self.environment = environment
        self._obs_aabbs = [o.aabb() for o in environment.obstacles]
        self.use_accel = bool(use_accel)
        self._scene = None
        if self.use_accel:
            from . import accel

            self._scene = accel.make_collision_scene(robot, environment)

    @property
    def backend(self) -> str:
        return "rust" if self._scene is not None else "python"

    def in_collision(self, state: np.ndarray) -> bool:
        if self._scene is not None:
            rotation, translation = self.robot.pose.transform(state)
            return bool(
                self._scene.in_collision_at(
                    np.ascontiguousarray(rotation, float).reshape(-1),
                    np.ascontiguousarray(translation, float),
                )
            )
        return self._in_collision_python(state)

    def _in_collision_python(self, state: np.ndarray) -> bool:
        from .collision import shapes_intersect

        for shape in self.robot.placed_shapes(state):
            slo, shi = shape.aabb()
            for obs, (olo, ohi) in zip(self.environment.obstacles, self._obs_aabbs):
                # Broad phase: skip clearly separated pairs.
                if np.any(shi < olo) or np.any(slo > ohi):
                    continue
                if shapes_intersect(shape, obs):
                    return True
        return False

    def trajectory_in_collision(self, states: np.ndarray) -> bool:
        """Sampled collision check along a trajectory's states."""
        states = np.asarray(states, float)
        if states.ndim == 1:
            states = states.reshape(1, -1)
        if self._scene is not None:
            # Pose mapping stays in Python; the native scene only sees geometry.
            n = states.shape[0]
            rotations = np.empty((n, 9))
            translations = np.empty((n, 3))
            for i, state in enumerate(states):
                rotation, translation = self.robot.pose.transform(state)
                rotations[i] = np.asarray(rotation, float).reshape(-1)
                translations[i] = translation
            return bool(
                self._scene.trajectory_in_collision(
                    np.ascontiguousarray(rotations), np.ascontiguousarray(translations)
                )
            )
        for state in states:
            if self._in_collision_python(state):
                return True
        return False
