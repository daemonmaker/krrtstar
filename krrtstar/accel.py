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
