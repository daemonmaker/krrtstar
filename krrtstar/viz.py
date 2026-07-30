"""3D visualization of planner trees, solutions, robot and obstacles.

Rendering is done with `PyVista <https://pyvista.org>`_, which is an optional
dependency (the ``viz`` Poetry group). Importing this module never fails when
PyVista is missing; the dependency is imported lazily inside the functions that
actually render, so the rest of the package stays usable in headless / minimal
installs.
"""

from __future__ import annotations

from typing import Any, Callable, List, Optional

import numpy as np

from .geometry import Box, PoseMapping, Sphere

_MISSING_MSG = (
    "pyvista is required for visualization; install with `poetry install --with viz`"
)


def _require_pyvista():
    """Import and return the ``pyvista`` module.

    Raises:
        ImportError: with an actionable message when PyVista is not installed.
    """
    try:
        import pyvista  # noqa: WPS433 (intentional lazy import)
    except ImportError as exc:  # pragma: no cover - exercised only without pyvista
        raise ImportError(_MISSING_MSG) from exc
    return pyvista


# --------------------------------------------------------------------------- #
# Geometry -> mesh helpers
# --------------------------------------------------------------------------- #
def _shape_to_mesh(pyvista, shape: Any):
    """Convert a :class:`Box` or :class:`Sphere` into a PyVista mesh."""
    if isinstance(shape, Box):
        center = np.asarray(shape.center, float).reshape(-1)
        ex, ey, ez = (float(v) for v in np.asarray(shape.extents, float).reshape(-1))
        return pyvista.Cube(
            center=tuple(center), x_length=ex, y_length=ey, z_length=ez
        )
    if isinstance(shape, Sphere):
        center = np.asarray(shape.center, float).reshape(-1)
        return pyvista.Sphere(radius=float(shape.radius), center=tuple(center))
    raise TypeError(f"Unsupported shape for rendering: {type(shape)!r}")


def _positions_from_states(pose: PoseMapping, states: np.ndarray) -> np.ndarray:
    """Map an ``(m, x_dim)`` state array to an ``(m, 3)`` position array."""
    states = np.atleast_2d(np.asarray(states, float))
    return np.array([pose.position(s) for s in states], dtype=float)


def _polyline(pyvista, points: np.ndarray):
    """Build a single connected polyline mesh through ``points`` (``(m, 3)``)."""
    points = np.asarray(points, float).reshape(-1, 3)
    if points.shape[0] < 2:
        return None
    return pyvista.lines_from_points(points)


# --------------------------------------------------------------------------- #
# Static rendering
# --------------------------------------------------------------------------- #
def _add_obstacles(pyvista, plotter, cfg) -> None:
    for obstacle in cfg.environment.obstacles:
        try:
            mesh = _shape_to_mesh(pyvista, obstacle)
        except TypeError:
            continue
        plotter.add_mesh(mesh, color="firebrick", opacity=0.35, name=None)


def _add_tree(pyvista, plotter, cfg, tree) -> List[Any]:
    """Draw tree edges and return the list of added actors."""
    pose = cfg.robot.pose
    nodes = tree.nodes
    actors: List[Any] = []
    for i, node in enumerate(nodes):
        if node.parent is None or node.parent < 0:
            continue
        parent = nodes[node.parent]
        traj = node.traj_from_parent
        if traj is not None and getattr(traj, "states", None) is not None:
            pts = _positions_from_states(pose, traj.states)
        else:
            pts = np.stack(
                [pose.position(parent.state), pose.position(node.state)]
            )
        line = _polyline(pyvista, pts)
        if line is not None:
            actor = plotter.add_mesh(
                line,
                color="lightsteelblue",
                line_width=1.0,
                name=f"krrtstar_tree_edge_{i}",
            )
            actors.append(actor)
    return actors


def _add_solution(pyvista, plotter, cfg, solution: List[Any]) -> None:
    pose = cfg.robot.pose
    chunks = []
    for traj in solution:
        states = getattr(traj, "states", None)
        if states is None:
            continue
        chunks.append(_positions_from_states(pose, states))
    if not chunks:
        return
    pts = np.concatenate(chunks, axis=0)
    line = _polyline(pyvista, pts)
    if line is not None:
        plotter.add_mesh(line, color="limegreen", line_width=5.0)


def _add_robot(pyvista, plotter, cfg, state: np.ndarray, color: str) -> None:
    for shape in cfg.robot.placed_shapes(state):
        try:
            mesh = _shape_to_mesh(pyvista, shape)
        except TypeError:
            continue
        plotter.add_mesh(mesh, color=color, opacity=0.9)


def render_result(cfg, result, save_path=None, offscreen=False, show=None):
    """Render a planning result in 3D and return the PyVista plotter.

    Args:
        cfg: an :class:`~krrtstar.config.ExperimentConfig`.
        result: a :class:`~krrtstar.planner.PlanResult`.
        save_path: if given, a screenshot is written to this path.
        offscreen: build the plotter without opening an interactive window.
        show: whether to open the interactive window. Defaults to ``True`` only
            when not rendering offscreen and not saving to a file.

    Returns:
        The configured :class:`pyvista.Plotter`.
    """
    pyvista = _require_pyvista()

    plotter = pyvista.Plotter(off_screen=offscreen)

    if cfg.visualization.show_obstacles:
        _add_obstacles(pyvista, plotter, cfg)

    _add_tree(pyvista, plotter, cfg, result.tree)

    solution = getattr(result, "solution", None)
    if solution is not None:
        _add_solution(pyvista, plotter, cfg, solution)

    if cfg.visualization.show_robot:
        x_init = cfg.planner.x_init
        x_goal = cfg.planner.x_goal
        if x_init is not None:
            _add_robot(pyvista, plotter, cfg, np.asarray(x_init, float), "royalblue")
        if x_goal is not None:
            _add_robot(pyvista, plotter, cfg, np.asarray(x_goal, float), "gold")

    plotter.add_axes()
    try:
        plotter.view_isometric()
    except Exception:  # pragma: no cover - camera setup is best-effort
        pass

    if save_path is not None:
        plotter.screenshot(save_path)

    if show is None:
        show = not offscreen and save_path is None
    if show:
        plotter.show()

    return plotter


# --------------------------------------------------------------------------- #
# Live rendering
# --------------------------------------------------------------------------- #
def make_live_callback(cfg, offscreen: bool = False) -> Callable[[int, Any], None]:
    """Return a ``callback(iteration, tree)`` that live-renders tree growth.

    The callback keeps a single persistent :class:`pyvista.Plotter` open across
    invocations. Obstacles are drawn once; on every call the tree edges are
    cleared and redrawn so the user watches the tree expand.

    Rendering is best-effort: any failure (including missing display, missing
    PyVista, or a closed window) degrades to a no-op so the planner never
    crashes because of visualization.
    """
    state: dict = {"plotter": None, "failed": False, "tree_actors": []}

    def _init_plotter():
        pyvista = _require_pyvista()
        plotter = pyvista.Plotter(off_screen=offscreen)
        if cfg.visualization.show_obstacles:
            _add_obstacles(pyvista, plotter, cfg)
        plotter.add_axes()
        try:
            plotter.show(interactive_update=True, auto_close=False)
        except TypeError:  # pragma: no cover - older/newer signature differences
            plotter.show(auto_close=False)
        state["pyvista"] = pyvista
        return plotter

    def callback(iteration: int, tree) -> None:
        if state["failed"]:
            return
        try:
            if state["plotter"] is None:
                state["plotter"] = _init_plotter()
            pyvista = state["pyvista"]
            plotter = state["plotter"]

            for actor in state["tree_actors"]:
                try:
                    plotter.remove_actor(actor, reset_camera=False)
                except Exception:  # pragma: no cover - actor may already be gone
                    pass
            state["tree_actors"] = _add_tree(pyvista, plotter, cfg, tree)

            plotter.update()
        except Exception as exc:  # pragma: no cover - environment dependent
            state["failed"] = True
            print(f"[krrtstar.viz] live rendering disabled: {exc}")

    return callback
