import json
import os

import numpy as np
import pytest

from krrtstar.cli import format_duration
from krrtstar.dynamics.linear import AnalyticLinearDynamics
from krrtstar.experiment import save_experiment
from krrtstar.planner import KRRTStar

EXAMPLE = os.path.join(os.path.dirname(__file__), "..", "examples", "double_integrator_2d.toml")


def free_space_planner(seed=1, **kw):
    dyn = AnalyticLinearDynamics(np.zeros((2, 2)), np.eye(2))
    return KRRTStar(
        dynamics=dyn,
        state_bounds=np.array([[-4, -4], [4, 4]]),
        x_init=np.array([-3.0, -3.0]),
        x_goal=np.array([3.0, 3.0]),
        connection_radius=8.0,
        euclidean_gate=8.0,
        seed=seed,
        **kw,
    )


# --------------------------------------------------------------------------- #
# Timing
# --------------------------------------------------------------------------- #
def test_result_reports_elapsed_and_rate():
    result = free_space_planner().grow(40)
    assert result.elapsed is not None
    assert result.elapsed > 0.0
    assert result.iterations == 40
    rate = result.nodes_per_second
    assert rate is not None and rate > 0.0
    assert rate == pytest.approx(len(result.tree.nodes) / result.elapsed, rel=1e-6)


def test_elapsed_excludes_nothing_obvious_and_grows_with_work():
    small = free_space_planner(seed=2).grow(20)
    large = free_space_planner(seed=2).grow(120)
    assert large.elapsed > small.elapsed


def test_elapsed_is_persisted_in_experiment_meta(tmp_path):
    result = free_space_planner(seed=3).grow(30)
    bundle = str(tmp_path / "exp")
    save_experiment(bundle, result, config_path=EXAMPLE)
    with open(os.path.join(bundle, "meta.json")) as fh:
        meta = json.load(fh)
    assert meta["elapsed_seconds"] == pytest.approx(result.elapsed)
    assert meta["iterations"] == 30


@pytest.mark.parametrize(
    "seconds,expected",
    [
        (None, "n/a"),
        (0.0421, "42.1 ms"),
        (1.4712, "1.47 s"),
        (59.9, "59.90 s"),
        (83.4, "1m 23.4s"),
    ],
)
def test_format_duration(seconds, expected):
    assert format_duration(seconds) == expected


# --------------------------------------------------------------------------- #
# Live viewer lifecycle
# --------------------------------------------------------------------------- #
def test_live_viewer_is_a_usable_progress_callback():
    """The viewer must be callable and degrade gracefully without PyVista."""
    from krrtstar.viz import LiveViewer, make_live_callback

    cfg = _example_cfg()
    viewer = make_live_callback(cfg, offscreen=True, keep_open=False)
    assert isinstance(viewer, LiveViewer)
    assert callable(viewer)
    assert hasattr(viewer, "finish")


def test_run_experiment_finalizes_the_viewer():
    """run_experiment must call finish() so the window shows the solution."""
    from krrtstar.run import run_experiment

    cfg = _example_cfg()
    cfg.visualization.live = True
    cfg.planner.target_nodes = 25

    class RecordingViewer:
        def __init__(self):
            self.calls = 0
            self.finished_with = "not-called"

        def __call__(self, iteration, tree):
            self.calls += 1

        def finish(self, result=None):
            self.finished_with = result

    viewer = RecordingViewer()
    result = run_experiment(cfg, live_callback=viewer)
    assert viewer.calls >= 1
    assert viewer.finished_with is result


def test_finish_is_noop_without_a_plotter():
    """finish() before any render must not raise (no window was opened)."""
    from krrtstar.viz import LiveViewer

    viewer = LiveViewer(_example_cfg(), offscreen=True, keep_open=False)
    viewer.finish(None)  # should be a quiet no-op
    viewer.close()


def test_keep_open_defaults_true_and_is_configurable():
    from krrtstar.config import parse_config

    cfg = parse_config({"dynamics": {"x_dim": 2, "u_dim": 2},
                        "environment": {"bounds": [[-1, -1, -1], [1, 1, 1]]}})
    assert cfg.visualization.keep_open is True

    cfg2 = parse_config({"dynamics": {"x_dim": 2, "u_dim": 2},
                         "environment": {"bounds": [[-1, -1, -1], [1, 1, 1]]},
                         "visualization": {"keep_open": False}})
    assert cfg2.visualization.keep_open is False


def test_offscreen_finish_does_not_block():
    """Offscreen finish must return promptly (guards against a hung CI run)."""
    pytest.importorskip("pyvista")
    import time

    from krrtstar.run import build_planner
    from krrtstar.viz import LiveViewer

    cfg = _example_cfg()
    cfg.planner.target_nodes = 20
    viewer = LiveViewer(cfg, offscreen=True, keep_open=True)
    planner = build_planner(cfg)
    try:
        result = planner.grow(20, progress_cb=viewer, progress_every=10)
        start = time.perf_counter()
        viewer.finish(result)
        assert time.perf_counter() - start < 30.0
    except Exception as exc:  # pragma: no cover - headless rendering may fail
        if _is_headless_failure(exc):
            pytest.skip(f"headless rendering unavailable: {exc}")
        raise
    finally:
        viewer.close()


# --------------------------------------------------------------------------- #
# Offscreen resolution (guards against blocking on a window that cannot appear)
# --------------------------------------------------------------------------- #
def test_has_display_follows_environment(monkeypatch):
    from krrtstar import viz

    monkeypatch.setattr(viz.sys, "platform", "linux")
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    assert viz.has_display() is False

    monkeypatch.setenv("DISPLAY", ":0")
    assert viz.has_display() is True


def test_resolve_offscreen_prevents_blocking_without_a_display(monkeypatch):
    """A headless run must resolve to offscreen so finish() cannot block."""
    from krrtstar import viz

    class FakePyvista:
        OFF_SCREEN = False

    # Explicit request wins.
    assert viz._resolve_offscreen(FakePyvista, True) is True

    # PyVista's global switch (PYVISTA_OFF_SCREEN) wins.
    class GlobalOffscreen:
        OFF_SCREEN = True

    assert viz._resolve_offscreen(GlobalOffscreen, False) is True

    # No display -> offscreen.
    monkeypatch.setattr(viz, "has_display", lambda: False)
    assert viz._resolve_offscreen(FakePyvista, False) is True

    # Display available and nothing forcing offscreen -> interactive.
    monkeypatch.setattr(viz, "has_display", lambda: True)
    assert viz._resolve_offscreen(FakePyvista, False) is False


def test_finish_does_not_block_when_resolved_offscreen(monkeypatch):
    """finish() must respect the resolved offscreen flag, not the request."""
    from krrtstar.viz import LiveViewer

    viewer = LiveViewer(_example_cfg(), offscreen=False, keep_open=True)

    calls = {"show": 0, "render": 0, "reset": 0}

    class FakePlotter:
        def render(self):
            calls["render"] += 1

        def reset_camera(self):
            calls["reset"] += 1

        def show(self, *a, **kw):  # must never be called for offscreen
            calls["show"] += 1

    viewer._plotter = FakePlotter()
    viewer._pyvista = object()
    viewer._offscreen_effective = True  # as resolved in a headless run

    viewer.finish(None)
    assert calls["show"] == 0
    assert calls["render"] == 1


def _example_cfg():
    from krrtstar.config import load_config

    return load_config(EXAMPLE)


def _is_headless_failure(exc: Exception) -> bool:
    hints = ("opengl", "osmesa", "glx", "egl", "display", "framebuffer", "x11")
    text = str(exc).lower()
    return any(h in text for h in hints)
