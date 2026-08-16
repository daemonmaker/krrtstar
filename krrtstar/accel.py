"""Optional Rust acceleration shim.

If the compiled ``krrtstar_core`` extension (built from ``rust/`` via maturin)
is importable, its fast primitives are exposed here. Otherwise the pure-Python
fallbacks are used, so the package works with or without the native build.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

try:
    import krrtstar_core as _core  # type: ignore

    HAVE_RUST = True
except Exception:  # pragma: no cover - depends on build
    _core = None
    HAVE_RUST = False


def backend() -> str:
    return "rust" if HAVE_RUST else "python"


def euclidean_neighbors(states: np.ndarray, query: np.ndarray, radius: float) -> np.ndarray:
    """Indices of ``states`` within ``radius`` (Euclidean) of ``query``."""
    states = np.ascontiguousarray(states, dtype=float)
    query = np.ascontiguousarray(query, dtype=float).reshape(-1)
    if HAVE_RUST and states.shape[0] > 0:
        return np.asarray(_core.euclidean_neighbors(states, query, float(radius)), dtype=int)
    if states.shape[0] == 0:
        return np.zeros(0, dtype=int)
    dists = np.linalg.norm(states - query, axis=1)
    return np.nonzero(dists <= radius)[0]


def linear_cost(
    A: np.ndarray,
    Q: np.ndarray,
    c: np.ndarray,
    x0: np.ndarray,
    x1: np.ndarray,
    tau_max: float,
) -> Optional[float]:
    """Optimal linear connection cost via the native core, if available."""
    if not HAVE_RUST:
        return None
    return _core.linear_cost(
        np.ascontiguousarray(A, float),
        np.ascontiguousarray(Q, float),
        np.ascontiguousarray(c, float),
        np.ascontiguousarray(x0, float).reshape(-1),
        np.ascontiguousarray(x1, float).reshape(-1),
        float(tau_max),
    )


def _add_shape(scene, side: str, shape) -> None:
    """Register one shape with the native scene (``side`` is robot/obstacle)."""
    from .geometry import Box, Capsule, Cylinder, Mesh, Sphere

    center = np.ascontiguousarray(shape.center, float)
    if isinstance(shape, Sphere):
        scene.add_sphere(side, center, float(shape.radius))
        return
    rot = np.ascontiguousarray(getattr(shape, "rotation", np.eye(3)), float).reshape(-1)
    if isinstance(shape, Box):
        scene.add_box(side, center, rot, np.ascontiguousarray(shape.extents, float))
    elif isinstance(shape, Capsule):
        scene.add_capsule(side, center, rot, float(shape.radius), float(shape.height))
    elif isinstance(shape, Cylinder):
        scene.add_cylinder(side, center, rot, float(shape.radius), float(shape.height))
    elif isinstance(shape, Mesh):
        scene.add_mesh(
            side,
            center,
            rot,
            np.ascontiguousarray(shape.vertices, float),
            np.ascontiguousarray(shape.faces, np.uint32),
            float(shape.scale),
        )
    else:
        raise TypeError(f"Unsupported shape for the native scene: {type(shape)!r}")


def make_collision_scene(robot, environment):
    """Build a native collision scene, or return ``None`` if unavailable.

    Robot shapes are registered in the robot frame; obstacles in world
    coordinates. Queries supply the robot's world transform, so the state ->
    pose mapping stays in Python (see :class:`krrtstar.geometry.PoseMapping`).
    """
    if not HAVE_RUST or not hasattr(_core, "CollisionScene"):
        return None
    try:
        scene = _core.CollisionScene()
        for shape in robot.shapes:
            _add_shape(scene, "robot", shape)
        for shape in environment.obstacles:
            _add_shape(scene, "obstacle", shape)
        scene.finalize()
        return scene
    except Exception:  # pragma: no cover - unsupported shape or older core
        return None


def make_connector(
    A: np.ndarray,
    B: np.ndarray,
    Rinv: np.ndarray,
    c: np.ndarray,
    tau_max: float,
    n_samples: int,
    grid_n: int = 64,
):
    """Create a reusable native connector for one linear system.

    The connector precomputes the time-grid matrix exponentials once, so batched
    cost queries need no further exponentials. Returns ``None`` when the native
    core is unavailable.
    """
    if not HAVE_RUST:
        return None
    try:
        return _core.LinearConnector(
            np.ascontiguousarray(A, float),
            np.ascontiguousarray(B, float),
            np.ascontiguousarray(Rinv, float),
            np.ascontiguousarray(c, float).reshape(-1),
            float(tau_max),
            int(n_samples),
            int(grid_n),
        )
    except Exception:  # pragma: no cover - older core builds
        return None


def unpack_connect(result, x_dim: int, u_dim: int):
    """Reshape a native connect result into arrays, or ``None``."""
    if result is None:
        return None
    tau, cost, times, states_flat, controls_flat = result
    times = np.asarray(times, dtype=float)
    states = np.asarray(states_flat, dtype=float).reshape(len(times), x_dim)
    controls = np.asarray(controls_flat, dtype=float).reshape(len(times), u_dim)
    return float(tau), float(cost), times, states, controls


def linear_connect(
    A: np.ndarray,
    B: np.ndarray,
    Rinv: np.ndarray,
    c: np.ndarray,
    x0: np.ndarray,
    x1: np.ndarray,
    tau_max: float,
    n_samples: int,
):
    """Full optimal connection (time, cost, sampled trajectory) via Rust.

    Returns ``(tau, cost, times, states, controls)`` with ``states`` shaped
    ``(n_samples, x_dim)`` and ``controls`` shaped ``(n_samples, u_dim)``, or
    ``None`` when the native core is unavailable or the states are not
    connectable.
    """
    if not HAVE_RUST:
        return None
    result = _core.linear_connect(
        np.ascontiguousarray(A, float),
        np.ascontiguousarray(B, float),
        np.ascontiguousarray(Rinv, float),
        np.ascontiguousarray(c, float).reshape(-1),
        np.ascontiguousarray(x0, float).reshape(-1),
        np.ascontiguousarray(x1, float).reshape(-1),
        float(tau_max),
        int(n_samples),
    )
    return unpack_connect(
        result, int(np.asarray(A).shape[0]), int(np.asarray(B).shape[1])
    )
