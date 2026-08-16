"""Rendering of the full shape vocabulary (spheres, boxes, capsules, ...)."""

import os

import numpy as np
import pytest

pyvista = pytest.importorskip("pyvista")

from krrtstar.config import load_config
from krrtstar.geometry import (
    Box,
    Capsule,
    Cylinder,
    Mesh,
    Sphere,
    rotation_from_euler,
)
from krrtstar.run import build_planner
from krrtstar.viz import _shape_to_mesh, render_result

EXAMPLE = os.path.join(
    os.path.dirname(__file__), "..", "examples", "double_integrator_2d.toml"
)

# Failures that indicate the headless VM lacks a working OpenGL / OSMesa stack.
_HEADLESS_HINTS = (
    "opengl",
    "osmesa",
    "glx",
    "mesa",
    "framebuffer",
    "display",
    "x11",
    "xdisplay",
    "render window",
    "cannot connect",
    "libgl",
    "egl",
)

ROT_Z90 = rotation_from_euler(yaw=np.pi / 2)
ROT_Y90 = rotation_from_euler(pitch=np.pi / 2)


def _is_headless_failure(exc: BaseException) -> bool:
    text = str(exc).lower()
    return any(hint in text for hint in _HEADLESS_HINTS)


def _square_mesh() -> Mesh:
    """A unit square in the z=0 plane, as two triangles."""
    vertices = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]]
    )
    faces = np.array([[0, 1, 2], [0, 2, 3]])
    return Mesh(vertices=vertices, faces=faces, center=[1.0, 2.0, 3.0])


def _extents(mesh) -> np.ndarray:
    """Side lengths of a mesh's axis-aligned bounding box."""
    x0, x1, y0, y1, z0, z1 = (float(v) for v in mesh.bounds)
    return np.array([x1 - x0, y1 - y0, z1 - z0])


def _bounds_center(mesh) -> np.ndarray:
    x0, x1, y0, y1, z0, z1 = (float(v) for v in mesh.bounds)
    return np.array([(x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0])


SHAPES = {
    "sphere": Sphere(center=[1.0, 2.0, 3.0], radius=0.75),
    "box_axis_aligned": Box(center=[0.0, 0.0, 0.0], extents=[2.0, 1.0, 3.0]),
    "box_oriented": Box(center=[1.0, 2.0, 3.0], extents=[2.0, 1.0, 3.0], rotation=ROT_Z90),
    "capsule": Capsule(center=[0.0, 1.0, 0.0], radius=0.4, height=2.0, rotation=ROT_Y90),
    "cylinder": Cylinder(center=[0.0, 0.0, 1.0], radius=0.5, height=2.0),
    "mesh": _square_mesh(),
}


@pytest.mark.parametrize("name", sorted(SHAPES))
def test_shape_to_mesh_is_renderable(name):
    mesh = _shape_to_mesh(pyvista, SHAPES[name])
    assert mesh is not None
    assert mesh.n_points > 0, f"{name} produced an empty mesh"
    assert np.all(np.isfinite(mesh.points))


def test_unsupported_shape_raises_type_error():
    with pytest.raises(TypeError):
        _shape_to_mesh(pyvista, object())


def test_oriented_box_is_rotated_and_positioned():
    """A 90 deg yaw must swing the long axis onto Y without moving the center."""
    box = Box(center=[5.0, 0.0, 0.0], extents=[4.0, 1.0, 1.0], rotation=ROT_Z90)
    mesh = _shape_to_mesh(pyvista, box)

    assert np.allclose(_bounds_center(mesh), [5.0, 0.0, 0.0], atol=1e-6)
    assert np.allclose(_extents(mesh), [1.0, 4.0, 1.0], atol=1e-6)


def test_axis_aligned_box_bounds():
    box = Box(center=[-1.0, 2.0, 0.5], extents=[4.0, 1.0, 2.0])
    mesh = _shape_to_mesh(pyvista, box)

    assert np.allclose(_bounds_center(mesh), [-1.0, 2.0, 0.5], atol=1e-6)
    assert np.allclose(_extents(mesh), [4.0, 1.0, 2.0], atol=1e-6)


def test_capsule_bounds_follow_its_axis():
    """height is the inner segment, so the Z span is height + 2 * radius."""
    capsule = Capsule(center=[0.0, 0.0, 0.0], radius=0.5, height=4.0)
    mesh = _shape_to_mesh(pyvista, capsule)

    ex, ey, ez = _extents(mesh)
    assert np.allclose(_bounds_center(mesh), [0.0, 0.0, 0.0], atol=1e-3)
    # Tolerant on the exact tessellation, strict about which axis dominates.
    assert ez > max(ex, ey) + 1.0
    assert ez == pytest.approx(5.0, abs=0.1)
    assert ex == pytest.approx(1.0, abs=0.1)
    assert ey == pytest.approx(1.0, abs=0.1)


def test_capsule_falls_back_to_cylinder_plus_caps(monkeypatch):
    """Older PyVista versions have no Capsule; the union must still fit."""
    monkeypatch.delattr(pyvista, "Capsule", raising=False)
    capsule = Capsule(center=[0.0, 0.0, 0.0], radius=0.5, height=4.0)
    mesh = _shape_to_mesh(pyvista, capsule)

    ex, ey, ez = _extents(mesh)
    assert mesh.n_points > 0
    assert ez > max(ex, ey) + 1.0
    assert ez == pytest.approx(5.0, abs=0.1)


def test_cylinder_rotated_onto_x_axis():
    cylinder = Cylinder(
        center=[2.0, 0.0, 0.0], radius=0.5, height=3.0, rotation=ROT_Y90
    )
    mesh = _shape_to_mesh(pyvista, cylinder)

    ex, ey, ez = _extents(mesh)
    assert np.allclose(_bounds_center(mesh), [2.0, 0.0, 0.0], atol=1e-3)
    assert ex == pytest.approx(3.0, abs=1e-3)
    assert ey == pytest.approx(1.0, abs=1e-2)
    assert ez == pytest.approx(1.0, abs=1e-2)


def test_mesh_uses_world_vertices():
    shape = _square_mesh()
    mesh = _shape_to_mesh(pyvista, shape)

    assert mesh.n_cells == 2
    assert mesh.n_points == 4
    assert np.allclose(
        np.sort(mesh.points, axis=0),
        np.sort(shape.world_vertices(), axis=0),
        atol=1e-6,
    )


def test_render_scene_with_all_shape_types(tmp_path, monkeypatch):
    """End-to-end offscreen render of a scene made of the new shape types."""
    monkeypatch.setenv("PYVISTA_OFF_SCREEN", "true")

    cfg = load_config(EXAMPLE)
    cfg.planner.target_nodes = 30
    cfg.environment.obstacles = [
        Sphere(center=[2.0, 2.0, 0.0], radius=1.0),
        Box(center=[-3.0, 1.0, 0.0], extents=[3.0, 1.0, 1.0], rotation=ROT_Z90),
        Capsule(center=[0.0, -3.0, 0.0], radius=0.5, height=2.0, rotation=ROT_Y90),
        Cylinder(center=[3.0, -2.0, 0.0], radius=0.8, height=2.0),
        Mesh(
            vertices=np.array(
                [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]]
            ),
            faces=np.array([[0, 1, 2], [0, 2, 3]]),
            center=[-2.0, -3.0, 0.0],
            scale=2.0,
        ),
    ]

    planner = build_planner(cfg)
    result = planner.grow(cfg.planner.target_nodes)

    out_path = str(tmp_path / "out.png")
    plotter = None
    try:
        plotter = render_result(
            cfg, result, save_path=out_path, offscreen=True, show=False
        )
    except Exception as exc:  # noqa: BLE001 - we want to classify the failure
        if _is_headless_failure(exc):
            pytest.skip(f"headless rendering unavailable: {exc}")
        raise
    finally:
        if plotter is not None:
            try:
                plotter.close()
            except Exception:  # pragma: no cover - cleanup only
                pass

    assert os.path.exists(out_path), "screenshot PNG was not created"
    assert os.path.getsize(out_path) > 0, "screenshot PNG is empty"
