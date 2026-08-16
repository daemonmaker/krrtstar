"""Rendering of the robot models in :mod:`krrtstar.robots`.

Each preset carries two geometries: the coarse body the planner collides with and
a detailed assembly meant only for display. :func:`render_robot_model` draws the
detailed assembly and, by default, overlays the collision body as a translucent
shell so the gap between "what the robot looks like" and "what the planner sees"
is visible at a glance.

Like :mod:`krrtstar.viz`, PyVista is imported lazily, so importing this module
works in installs without the ``viz`` group.

Render every preset to a directory from the command line::

    python -m krrtstar.robot_viz --out /tmp/robot_models
"""

from __future__ import annotations

import os
from typing import Any, List, Optional, Sequence, Tuple

import numpy as np

from .robots import DEFAULT_POSES, MODEL_PRESETS, collision_shapes, model_shapes
from .viz import (
    WINDOW_TITLE,
    _bring_to_front,
    _require_pyvista,
    _resolve_offscreen,
    _shape_to_mesh,
)

#: Camera offset from the model center. Raised well above the horizon so the
#: flat discs (the puck, the quadrotor's collision body) read as discs rather
#: than as edge-on slivers.
ISO_VIEW_VECTOR = (1.0, -1.0, 0.85)

COLLISION_COLOR = "royalblue"
COLLISION_OPACITY = 0.18


def _placed(shape: Any, name: str, state: Optional[Sequence[float]]) -> Any:
    """Place ``shape`` in the world using the preset's state -> pose mapping."""
    if state is None:
        return shape
    rotation, translation = DEFAULT_POSES[name].transform(np.asarray(state, float))
    return shape.transformed(rotation, translation)


def _add_display_model(pyvista, plotter, name: str, state) -> int:
    """Draw the detailed assembly, one actor per coloured part."""
    drawn = 0
    for part in model_shapes(name):
        try:
            mesh = _shape_to_mesh(pyvista, _placed(part.shape, name, state))
        except TypeError:  # pragma: no cover - every preset part is renderable
            continue
        plotter.add_mesh(
            mesh, color=part.color, opacity=part.opacity, smooth_shading=True
        )
        drawn += 1
    return drawn


def _add_collision_overlay(pyvista, plotter, name: str, state) -> int:
    """Draw the coarse collision body as a translucent, outlined shell."""
    drawn = 0
    for shape in collision_shapes(name):
        try:
            mesh = _shape_to_mesh(pyvista, _placed(shape, name, state))
        except TypeError:  # pragma: no cover - every preset body is renderable
            continue
        plotter.add_mesh(mesh, color=COLLISION_COLOR, opacity=COLLISION_OPACITY)
        # Silhouette edges (box edges, cylinder rims) rather than the full
        # tessellation, which would bury the model underneath.
        try:
            edges = mesh.extract_feature_edges(feature_angle=30.0)
        except Exception:  # pragma: no cover - depends on the mesh type
            edges = None
        if edges is not None and edges.n_points:
            plotter.add_mesh(edges, color=COLLISION_COLOR, style="wireframe",
                             line_width=2.0, opacity=0.75)
        drawn += 1
    return drawn


def _frame_camera(plotter) -> None:
    """Point the camera down at the model from an isometric-ish angle and fit."""
    for step in (
        lambda: plotter.view_vector(ISO_VIEW_VECTOR, viewup=(0.0, 0.0, 1.0)),
        plotter.reset_camera,
    ):
        try:
            step()
        except Exception:  # pragma: no cover - camera setup is best-effort
            pass


def render_robot_model(
    name: str,
    save_path: Optional[str] = None,
    offscreen: bool = True,
    show: Optional[bool] = None,
    show_collision_body: bool = True,
    state: Optional[Sequence[float]] = None,
    window_size: Tuple[int, int] = (900, 900),
):
    """Render one robot preset and return the PyVista plotter.

    Args:
        name: a preset name (``"puck"``, ``"quadrotor"``, ``"car"``).
        save_path: if given, a screenshot is written to this path.
        offscreen: build the plotter without opening an interactive window.
        show: whether to open the interactive window. Defaults to ``True`` only
            when not rendering offscreen and not saving to a file.
        show_collision_body: also draw the coarse collision body as a
            translucent overlay, showing what the planner actually collides
            with.
        state: a planner state for this system; the model is placed through
            :data:`krrtstar.robots.DEFAULT_POSES`, so orientation components
            (quadrotor roll/pitch, car yaw) tilt or turn the model.
        window_size: render window size in pixels.

    Returns:
        The configured :class:`pyvista.Plotter`.

    Raises:
        ValueError: for an unknown preset name.
    """
    pyvista = _require_pyvista()

    offscreen = _resolve_offscreen(pyvista, offscreen)
    plotter = pyvista.Plotter(off_screen=offscreen, window_size=tuple(window_size))

    _add_display_model(pyvista, plotter, name, state)
    if show_collision_body:
        _add_collision_overlay(pyvista, plotter, name, state)

    title = name if state is None else f"{name} (posed)"
    plotter.add_text(title, font_size=16, color="black")
    if show_collision_body:
        plotter.add_text(
            "translucent blue: collision body",
            position="lower_left",
            font_size=9,
            color="royalblue",
        )
    plotter.add_axes()
    # Each model frames itself: the quadrotor spans ~0.4 m, the car ~5 m.
    _frame_camera(plotter)

    if save_path is not None:
        directory = os.path.dirname(os.path.abspath(save_path))
        os.makedirs(directory, exist_ok=True)
        plotter.screenshot(save_path)

    if show is None:
        show = not offscreen and save_path is None
    if show and not offscreen:  # pragma: no cover - needs a display
        _bring_to_front(plotter, WINDOW_TITLE)
        plotter.show(title=WINDOW_TITLE, auto_close=False)

    return plotter


def render_all_robot_models(directory: str, offscreen: bool = True) -> List[str]:
    """Render every preset to ``<directory>/robot_<name>.png``.

    Returns:
        The paths written, in preset registration order.
    """
    os.makedirs(directory, exist_ok=True)
    paths: List[str] = []
    for name in MODEL_PRESETS:
        path = os.path.join(directory, f"robot_{name}.png")
        plotter = render_robot_model(
            name, save_path=path, offscreen=offscreen, show=False
        )
        try:
            plotter.close()
        except Exception:  # pragma: no cover - cleanup only
            pass
        paths.append(path)
    return paths


def _main(argv: Optional[Sequence[str]] = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", default="robot_models",
                        help="directory to write the PNGs to")
    parser.add_argument("--name", choices=sorted(MODEL_PRESETS),
                        help="render only this preset")
    parser.add_argument("--no-collision-body", action="store_true",
                        help="omit the translucent collision-body overlay")
    args = parser.parse_args(argv)

    if args.name is None:
        for path in render_all_robot_models(args.out):
            print(path)
        return 0

    path = os.path.join(args.out, f"robot_{args.name}.png")
    plotter = render_robot_model(
        args.name,
        save_path=path,
        show=False,
        show_collision_body=not args.no_collision_body,
    )
    plotter.close()
    print(path)
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    raise SystemExit(_main())
