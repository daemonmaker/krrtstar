import os

import pytest

pytest.importorskip("pyvista")

from krrtstar.config import load_config
from krrtstar.run import build_planner
from krrtstar.viz import render_result

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


def _is_headless_failure(exc: BaseException) -> bool:
    text = str(exc).lower()
    return any(hint in text for hint in _HEADLESS_HINTS)


def test_render_result_writes_png(tmp_path, monkeypatch):
    monkeypatch.setenv("PYVISTA_OFF_SCREEN", "true")

    cfg = load_config(EXAMPLE)
    # Keep the tree tiny so the test is fast.
    cfg.planner.target_nodes = 40

    planner = build_planner(cfg)
    result = planner.grow(cfg.planner.target_nodes)

    out_path = str(tmp_path / "out.png")
    try:
        plotter = render_result(
            cfg, result, save_path=out_path, offscreen=True, show=False
        )
    except Exception as exc:  # noqa: BLE001 - we want to classify the failure
        if _is_headless_failure(exc):
            pytest.skip(f"headless rendering unavailable: {exc}")
        raise
    finally:
        # Best-effort cleanup of the render window if one was created.
        plotter_obj = locals().get("plotter")
        if plotter_obj is not None:
            try:
                plotter_obj.close()
            except Exception:  # pragma: no cover - cleanup only
                pass

    assert os.path.exists(out_path), "screenshot PNG was not created"
    assert os.path.getsize(out_path) > 0, "screenshot PNG is empty"
