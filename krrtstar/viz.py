"""3D visualization of planner trees, solutions, robot and obstacles.

Rendering is done with `PyVista <https://pyvista.org>`_, which is an optional
dependency (the ``viz`` Poetry group). Importing this module never fails when
PyVista is missing; the dependency is imported lazily inside the functions that
actually render, so the rest of the package stays usable in headless / minimal
installs.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from typing import Any, Callable, List, Optional

import numpy as np

from .geometry import Box, Capsule, Cylinder, Mesh, PoseMapping, Sphere

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
_LOCAL_Z = np.array([0.0, 0.0, 1.0])


def _rotation_of(shape: Any) -> np.ndarray:
    """Rotation matrix of a shape, defaulting to the identity."""
    rotation = getattr(shape, "rotation", None)
    if rotation is None:
        return np.eye(3)
    return np.asarray(rotation, float).reshape(3, 3)


def _rigid_matrix(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """Compose a 4x4 homogeneous transform from a rotation and a translation."""
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def _axis_direction(shape: Any) -> np.ndarray:
    """World-space direction of a shape's local Z axis (its symmetry axis)."""
    return _rotation_of(shape) @ _LOCAL_Z


def _box_mesh(pyvista, shape: Box):
    """Cube built at the origin, then placed by the box's rotation + center."""
    ex, ey, ez = (float(v) for v in np.asarray(shape.extents, float).reshape(-1))
    try:
        mesh = pyvista.Cube(
            x_length=ex, y_length=ey, z_length=ez, point_dtype="float64"
        )
    except TypeError:  # pragma: no cover - older PyVista without point_dtype
        mesh = pyvista.Cube(x_length=ex, y_length=ey, z_length=ez)
    center = np.asarray(shape.center, float).reshape(-1)
    mesh.transform(_rigid_matrix(_rotation_of(shape), center), inplace=True)
    return mesh


def _capsule_mesh(pyvista, shape: Capsule):
    """Capsule via ``pyvista.Capsule``, or a cylinder plus two end-cap spheres."""
    center = np.asarray(shape.center, float).reshape(-1)
    radius = float(shape.radius)
    direction = _axis_direction(shape)
    capsule = getattr(pyvista, "Capsule", None)
    if capsule is not None:
        try:
            return capsule(
                center=tuple(center),
                direction=tuple(direction),
                radius=radius,
                cylinder_length=float(shape.height),
            )
        except TypeError:  # pragma: no cover - signature differs across versions
            pass
    start, end = shape.segment()
    body = pyvista.Cylinder(
        center=tuple(center),
        direction=tuple(direction),
        radius=radius,
        height=float(shape.height),
    )
    caps = [
        pyvista.Sphere(radius=radius, center=tuple(np.asarray(p, float)))
        for p in (start, end)
    ]
    return body.merge(caps)


def _cylinder_mesh(pyvista, shape: Cylinder):
    center = np.asarray(shape.center, float).reshape(-1)
    return pyvista.Cylinder(
        center=tuple(center),
        direction=tuple(_axis_direction(shape)),
        radius=float(shape.radius),
        height=float(shape.height),
    )


def _triangle_mesh(pyvista, shape: Mesh):
    """Triangle soup as ``PolyData``, using the mesh's world-frame vertices."""
    vertices = np.asarray(shape.world_vertices(), float).reshape(-1, 3)
    faces = np.asarray(shape.faces, np.int64).reshape(-1, 3)
    # VTK cell array: each triangle is prefixed by its vertex count.
    cells = np.hstack(
        [np.full((faces.shape[0], 1), 3, dtype=np.int64), faces]
    ).ravel()
    return pyvista.PolyData(vertices, cells)


def _shape_to_mesh(pyvista, shape: Any):
    """Convert a :mod:`krrtstar.geometry` shape into a PyVista mesh.

    Handles :class:`Sphere`, :class:`Box` (oriented or not), :class:`Capsule`,
    :class:`Cylinder` and triangle :class:`Mesh`.

    Raises:
        TypeError: for shape types that cannot be rendered. Callers skip those.
    """
    if isinstance(shape, Box):
        return _box_mesh(pyvista, shape)
    if isinstance(shape, Sphere):
        center = np.asarray(shape.center, float).reshape(-1)
        return pyvista.Sphere(radius=float(shape.radius), center=tuple(center))
    if isinstance(shape, Capsule):
        return _capsule_mesh(pyvista, shape)
    if isinstance(shape, Cylinder):
        return _cylinder_mesh(pyvista, shape)
    if isinstance(shape, Mesh):
        return _triangle_mesh(pyvista, shape)
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

    offscreen = _resolve_offscreen(pyvista, offscreen)
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
    if show and not offscreen:
        # Raise the window, then block so it stays up for inspection.
        _bring_to_front(plotter, WINDOW_TITLE)
        plotter.show(title=WINDOW_TITLE, auto_close=False)

    return plotter


# --------------------------------------------------------------------------- #
# Window placement
# --------------------------------------------------------------------------- #
WINDOW_TITLE = "krrtstar"


def _bring_to_front(plotter, title: Optional[str] = None) -> None:
    """Best-effort raise of the render window above others, with focus.

    VTK exposes no portable "raise window" API, so this tries several
    mechanisms in order and ignores every failure:

    1. Make sure the window is mapped and drawn (sufficient on some platforms).
    2. Qt-backed plotters (``pyvistaqt``) expose a real application window.
    3. Windows: ``SetForegroundWindow`` on the native handle via ctypes.
    4. macOS: ask System Events to front the current process.
    5. X11: ask the window manager through ``wmctrl`` / ``xdotool``.
    """
    ren_win = getattr(plotter, "ren_win", None)
    if ren_win is None:
        return

    try:
        if title:
            ren_win.SetWindowName(title)
        ren_win.ShowWindowOn()
        ren_win.Render()
        ren_win.Frame()
    except Exception:  # pragma: no cover - platform dependent
        pass

    app_window = getattr(plotter, "app_window", None)
    if app_window is not None:  # pragma: no cover - requires pyvistaqt
        for method in ("show", "raise_", "activateWindow"):
            try:
                getattr(app_window, method)()
            except Exception:
                pass
        return

    if sys.platform.startswith("win"):  # pragma: no cover - platform specific
        try:
            import ctypes

            hwnd = int(ren_win.GetGenericWindowId())
            ctypes.windll.user32.BringWindowToTop(hwnd)
            ctypes.windll.user32.SetForegroundWindow(hwnd)
        except Exception:
            pass
        return

    if sys.platform == "darwin":  # pragma: no cover - platform specific
        script = (
            "tell application \"System Events\" to set frontmost of the first "
            f"process whose unix id is {os.getpid()} to true"
        )
        _run_quiet(["osascript", "-e", script])
        return

    # X11 (also covers XWayland).
    if title:  # pragma: no cover - needs a window manager
        if _run_quiet(["wmctrl", "-a", title]):
            return
        _run_quiet(["xdotool", "search", "--name", title, "windowactivate"])


def _run_quiet(cmd) -> bool:
    """Run ``cmd`` if available; return True on a zero exit status."""
    exe = shutil.which(cmd[0])
    if exe is None:
        return False
    try:
        completed = subprocess.run(  # noqa: S603 - fixed, non-shell argv
            [exe, *cmd[1:]], capture_output=True, timeout=3, check=False
        )
    except Exception:  # pragma: no cover - platform dependent
        return False
    return completed.returncode == 0


def has_display() -> bool:
    """Whether an interactive display is likely available.

    Used to avoid blocking on a window that can never appear (headless servers,
    CI, SSH without X forwarding), which would otherwise hang the process.
    """
    if sys.platform.startswith("win") or sys.platform == "darwin":
        return True
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def _resolve_offscreen(pyvista, requested: bool) -> bool:
    """Decide whether rendering must be offscreen.

    Honours an explicit request, PyVista's global ``OFF_SCREEN`` switch (set by
    the ``PYVISTA_OFF_SCREEN`` environment variable), and the absence of a
    display. Keeping this in sync with the plotter matters: constructing an
    on-screen plotter where no window can appear makes an interactive ``show()``
    block forever.
    """
    if requested:
        return True
    if bool(getattr(pyvista, "OFF_SCREEN", False)):
        return True
    return not has_display()


# --------------------------------------------------------------------------- #
# Live rendering
# --------------------------------------------------------------------------- #
class LiveViewer:
    """Live 3D view of tree growth that stays open for inspection.

    Instances are callable, so a viewer can be handed straight to
    :meth:`krrtstar.planner.KRRTStar.grow` as its ``progress_cb``. A single
    :class:`pyvista.Plotter` is kept across calls: obstacles are drawn once and
    the tree edges are redrawn on each update.

    Call :meth:`finish` once planning completes to draw the solution and (unless
    ``keep_open`` is False or rendering is offscreen) block so the window stays
    up until the user closes it.

    Rendering is best-effort: any failure (missing display, missing PyVista, a
    closed window) disables the viewer rather than breaking the planner.
    """

    def __init__(self, cfg, offscreen: bool = False, keep_open: bool = True,
                 title: str = WINDOW_TITLE) -> None:
        self.cfg = cfg
        self.offscreen = bool(offscreen)
        self.keep_open = bool(keep_open)
        self.title = title
        self.failed = False
        self.finished = False
        self._pyvista = None
        self._plotter = None
        self._tree_actors: List[Any] = []
        # Resolved once the plotter is built (see _resolve_offscreen).
        self._offscreen_effective = bool(offscreen)

    # -------------------------------------------------------------- #
    @property
    def plotter(self):
        return self._plotter

    def _init_plotter(self):
        pyvista = _require_pyvista()
        self._offscreen_effective = _resolve_offscreen(pyvista, self.offscreen)
        plotter = pyvista.Plotter(
            off_screen=self._offscreen_effective, title=self.title
        )
        if self.cfg.visualization.show_obstacles:
            _add_obstacles(pyvista, plotter, self.cfg)
        plotter.add_axes()
        try:
            plotter.view_isometric()
        except Exception:  # pragma: no cover - camera setup is best-effort
            pass
        try:
            plotter.show(
                title=self.title, interactive_update=True, auto_close=False
            )
        except TypeError:  # pragma: no cover - signature differences
            plotter.show(auto_close=False)
        self._pyvista = pyvista
        if not self._offscreen_effective:
            _bring_to_front(plotter, self.title)
        return plotter

    def __call__(self, iteration: int, tree) -> None:
        """Redraw the growing tree (planner progress callback)."""
        if self.failed or self.finished:
            return
        try:
            if self._plotter is None:
                self._plotter = self._init_plotter()
            plotter = self._plotter
            for actor in self._tree_actors:
                try:
                    plotter.remove_actor(actor, reset_camera=False)
                except Exception:  # pragma: no cover - actor may already be gone
                    pass
            self._tree_actors = _add_tree(self._pyvista, plotter, self.cfg, tree)
            plotter.update()
        except Exception as exc:  # pragma: no cover - environment dependent
            self.failed = True
            print(f"[krrtstar.viz] live rendering disabled: {exc}")

    # -------------------------------------------------------------- #
    def finish(self, result=None) -> None:
        """Draw the final solution and hold the window open for inspection."""
        if self.failed or self._plotter is None or self.finished:
            return
        self.finished = True
        plotter = self._plotter
        try:
            if result is not None:
                solution = getattr(result, "solution", None)
                if solution:
                    _add_solution(self._pyvista, plotter, self.cfg, solution)
                if self.cfg.visualization.show_robot:
                    x_init = self.cfg.planner.x_init
                    x_goal = self.cfg.planner.x_goal
                    if x_init is not None:
                        _add_robot(
                            self._pyvista, plotter, self.cfg,
                            np.asarray(x_init, float), "royalblue",
                        )
                    if x_goal is not None:
                        _add_robot(
                            self._pyvista, plotter, self.cfg,
                            np.asarray(x_goal, float), "gold",
                        )
            # The initial camera only framed the obstacles; refit so the whole
            # tree and solution are visible for inspection.
            try:
                plotter.reset_camera()
            except Exception:  # pragma: no cover - camera setup is best-effort
                pass
            plotter.render()
        except Exception as exc:  # pragma: no cover - environment dependent
            print(f"[krrtstar.viz] could not draw final solution: {exc}")

        # Offscreen rendering has no window to keep open, and blocking would
        # hang automated/headless runs.
        if self._offscreen_effective or not self.keep_open:
            return

        # VTK's interactor quits on 'q'/'e'; some window managers do not deliver
        # a close event to it, so mention the key explicitly.
        print("[krrtstar.viz] window open for inspection - press 'q' (or close it) to exit")
        try:  # pragma: no cover - requires an interactive display
            _bring_to_front(plotter, self.title)
            plotter.show(title=self.title, auto_close=False)
        except Exception:  # pragma: no cover - fall back to the raw event loop
            try:
                plotter.iren.start()
            except Exception as exc:
                print(f"[krrtstar.viz] could not hold the window open: {exc}")

    def close(self) -> None:
        """Close the window, if one is open."""
        if self._plotter is None:
            return
        try:
            self._plotter.close()
        except Exception:  # pragma: no cover - already closed
            pass
        self._plotter = None


def make_live_callback(
    cfg, offscreen: bool = False, keep_open: bool = True
) -> "LiveViewer":
    """Return a :class:`LiveViewer` for live rendering of tree growth."""
    return LiveViewer(cfg, offscreen=offscreen, keep_open=keep_open)
