"""The ported robot models: their dimensions, and rendering them.

The dimension checks pin each preset to the numbers in the legacy C++
implementation (``includes/robots.hpp``); the rendering checks exercise
:mod:`krrtstar.robot_viz` offscreen.
"""

import os

import numpy as np
import pytest

pyvista = pytest.importorskip("pyvista")

from krrtstar.geometry import Box, Cylinder
from krrtstar.robot_viz import render_all_robot_models, render_robot_model
from krrtstar.robots import (
    BEAM_HEIGHT,
    BEAM_WIDTH,
    CENTER_SIDE,
    MOTOR_RADIUS,
    QUAD_LENGTH,
    ROTOR_RADIUS,
    collision_shapes,
    model_shapes,
)

PRESETS = ("puck", "quadrotor", "car")

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


def _is_headless_failure(exc: BaseException) -> bool:
    text = str(exc).lower()
    return any(hint in text for hint in _HEADLESS_HINTS)


def _render(name, **kwargs):
    """Render offscreen, skipping the test when the VM cannot render at all."""
    try:
        return render_robot_model(name, offscreen=True, show=False, **kwargs)
    except Exception as exc:  # noqa: BLE001 - we want to classify the failure
        if _is_headless_failure(exc):
            pytest.skip(f"headless rendering unavailable: {exc}")
        raise


# --------------------------------------------------------------------------- #
# Geometry: the ported dimensions
# --------------------------------------------------------------------------- #
def test_puck_is_a_disc_of_radius_1_25():
    shapes = collision_shapes("puck")
    assert len(shapes) == 1
    disc = shapes[0]
    assert isinstance(disc, Cylinder)
    assert disc.radius == pytest.approx(1.25)
    assert disc.height == pytest.approx(1.0)
    assert np.allclose(disc.center, [0.0, 0.0, 0.0])

    # The display model is the same disc, just coloured.
    model = model_shapes("puck")
    assert len(model) == 1
    part = model[0]
    assert isinstance(part.shape, Cylinder)
    assert (part.shape.radius, part.shape.height) == (
        pytest.approx(1.25), pytest.approx(1.0)
    )
    assert isinstance(part.color, str) and part.color
    assert part.opacity == pytest.approx(1.0)


def test_quadrotor_collision_body_spans_the_rotor_tips():
    shapes = collision_shapes("quadrotor")
    assert len(shapes) == 1
    disc = shapes[0]
    assert isinstance(disc, Cylinder)
    assert disc.radius == pytest.approx(QUAD_LENGTH + ROTOR_RADIUS)
    assert disc.height == pytest.approx(QUAD_LENGTH / 2.0)


def test_quadrotor_model_has_crossing_beams():
    boxes = [p.shape for p in model_shapes("quadrotor") if isinstance(p.shape, Box)]
    extents = [np.asarray(b.extents, float) for b in boxes]

    along_x = np.array([2 * QUAD_LENGTH, BEAM_WIDTH, BEAM_HEIGHT])
    along_y = np.array([BEAM_WIDTH, 2 * QUAD_LENGTH, BEAM_HEIGHT])
    for wanted in (along_x, along_y):
        assert any(np.allclose(e, wanted) for e in extents), (
            f"no beam with extents {wanted}"
        )


def test_quadrotor_model_has_four_rotors_and_four_motors():
    cylinders = [p.shape for p in model_shapes("quadrotor") if isinstance(p.shape, Cylinder)]

    rotors = [c for c in cylinders if c.radius == pytest.approx(ROTOR_RADIUS)]
    motors = [c for c in cylinders if c.radius == pytest.approx(MOTOR_RADIUS)]
    assert len(rotors) == 4
    assert len(motors) == 4

    # One rotor per arm tip, at +/-length on each of X and Y.
    tips = {(round(c.center[0], 6), round(c.center[1], 6)) for c in rotors}
    assert tips == {
        (round(QUAD_LENGTH, 6), 0.0),
        (round(-QUAD_LENGTH, 6), 0.0),
        (0.0, round(QUAD_LENGTH, 6)),
        (0.0, round(-QUAD_LENGTH, 6)),
    }
    # Rotors sit above the beams they are mounted on.
    assert all(c.center[2] > BEAM_HEIGHT / 2.0 for c in rotors)


def test_quadrotor_hub_is_rotated_45_degrees_about_z():
    hub = None
    for part in model_shapes("quadrotor"):
        shape = part.shape
        if isinstance(shape, Box) and np.allclose(
            shape.extents, [CENTER_SIDE, CENTER_SIDE, BEAM_HEIGHT]
        ):
            hub = shape
            break
    assert hub is not None, "no central hub box in the quadrotor model"

    root_half = np.sqrt(0.5)  # cos(45 deg) == sin(45 deg)
    x_axis = np.asarray(hub.rotation, float) @ np.array([1.0, 0.0, 0.0])
    assert np.allclose(x_axis, [root_half, root_half, 0.0], atol=1e-9)


def test_car_is_a_5x3x2_5_box_offset_to_the_state_corner():
    shapes = collision_shapes("car")
    assert len(shapes) == 1
    body = shapes[0]
    assert isinstance(body, Box)
    assert np.allclose(body.extents, [5.0, 3.0, 2.5])
    assert np.allclose(body.center, [2.5, 1.5, 0.0])


@pytest.mark.parametrize("lookup", [collision_shapes, model_shapes])
def test_unknown_preset_raises_value_error(lookup):
    with pytest.raises(ValueError):
        lookup("hovercraft")


# --------------------------------------------------------------------------- #
# Rendering
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("name", PRESETS)
def test_render_robot_model_writes_png(name, tmp_path, monkeypatch):
    monkeypatch.setenv("PYVISTA_OFF_SCREEN", "true")

    out_path = str(tmp_path / f"{name}.png")
    plotter = _render(name, save_path=out_path)
    try:
        assert os.path.exists(out_path), f"{name}: screenshot PNG was not created"
        assert os.path.getsize(out_path) > 0, f"{name}: screenshot PNG is empty"
    finally:
        plotter.close()


def test_render_all_robot_models_writes_every_preset(tmp_path, monkeypatch):
    monkeypatch.setenv("PYVISTA_OFF_SCREEN", "true")

    directory = str(tmp_path / "models")
    try:
        paths = render_all_robot_models(directory)
    except Exception as exc:  # noqa: BLE001 - we want to classify the failure
        if _is_headless_failure(exc):
            pytest.skip(f"headless rendering unavailable: {exc}")
        raise

    assert [os.path.basename(p) for p in paths] == [
        f"robot_{name}.png" for name in PRESETS
    ]
    assert all(os.path.getsize(p) > 0 for p in paths)


def test_render_without_collision_body_still_draws_the_model(tmp_path, monkeypatch):
    monkeypatch.setenv("PYVISTA_OFF_SCREEN", "true")

    out_path = str(tmp_path / "quad_model_only.png")
    plotter = _render("quadrotor", save_path=out_path, show_collision_body=False)
    try:
        assert os.path.getsize(out_path) > 0
    finally:
        plotter.close()


def _screenshot(name, path, state):
    plotter = _render(name, save_path=path, state=state)
    try:
        return np.asarray(plotter.screenshot(return_img=True))
    finally:
        plotter.close()


def test_posed_render_differs_from_unposed(tmp_path, monkeypatch):
    """A rolled/pitched quadrotor must not render like a level one."""
    monkeypatch.setenv("PYVISTA_OFF_SCREEN", "true")

    level_path = str(tmp_path / "level.png")
    tilted_path = str(tmp_path / "tilted.png")
    level = _screenshot("quadrotor", level_path, [0, 0, 0, 0, 0, 0, 0.0, 0.0, 0, 0])
    tilted = _screenshot("quadrotor", tilted_path, [0, 0, 0, 0, 0, 0, 0.35, 0.25, 0, 0])

    assert level.shape == tilted.shape
    changed = float(np.mean(np.any(level != tilted, axis=-1)))
    assert changed > 0.01, f"only {changed:.4%} of pixels changed; pose not applied"

    with open(level_path, "rb") as fh:
        level_bytes = fh.read()
    with open(tilted_path, "rb") as fh:
        tilted_bytes = fh.read()
    assert level_bytes != tilted_bytes


def test_car_yaw_turns_the_body(tmp_path, monkeypatch):
    monkeypatch.setenv("PYVISTA_OFF_SCREEN", "true")

    straight = _screenshot("car", str(tmp_path / "straight.png"), [0, 0, 0.0, 1, 0])
    turned = _screenshot("car", str(tmp_path / "turned.png"), [0, 0, 0.9, 1, 0])

    changed = float(np.mean(np.any(straight != turned, axis=-1)))
    assert changed > 0.01, f"only {changed:.4%} of pixels changed; yaw not applied"
