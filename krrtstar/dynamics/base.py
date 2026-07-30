"""Dynamics interface and shared trajectory type.

Every dynamics backend (classic linear physics or a learned PyTorch model)
implements :class:`Dynamics`. The planner only ever talks to this interface,
which is what makes the system runtime-configurable instead of compiled per
model as in the legacy C++ code.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Protocol, runtime_checkable

import numpy as np


@dataclass
class Trajectory:
    """A time-parameterized connection between two states.

    Attributes:
        tau: total duration of the trajectory.
        cost: optimal-control cost of the connection (``tau + control effort``).
        times: 1-D array of sample times in ``[0, tau]``.
        states: ``(len(times), x_dim)`` array of sampled states.
        controls: ``(len(times), u_dim)`` array of sampled controls.
        feasible: whether the connection satisfies control/limit constraints.
    """

    tau: float
    cost: float
    times: np.ndarray
    states: np.ndarray
    controls: np.ndarray
    feasible: bool = True

    @property
    def x0(self) -> np.ndarray:
        return self.states[0]

    @property
    def x1(self) -> np.ndarray:
        return self.states[-1]


@runtime_checkable
class Dynamics(Protocol):
    """Runtime-pluggable system dynamics.

    The two required numbers are the state and control dimensions. Everything
    else is expressed through :meth:`connect` / :meth:`cost`, so the planner is
    fully decoupled from the concrete model.
    """

    x_dim: int
    u_dim: int

    def cost(self, x0: np.ndarray, x1: np.ndarray) -> Optional[float]:
        """Return the optimal connection cost from ``x0`` to ``x1``.

        Returns ``None`` when the states cannot be connected.
        """
        ...

    def connect(self, x0: np.ndarray, x1: np.ndarray) -> Optional[Trajectory]:
        """Return the optimal trajectory from ``x0`` to ``x1`` or ``None``."""
        ...

    def u_bounds(self) -> Optional[np.ndarray]:
        """Return a ``(u_dim, 2)`` array of ``[min, max]`` control bounds."""
        ...


@dataclass
class ControlBounds:
    """Helper wrapping optional per-dimension control bounds."""

    bounds: Optional[np.ndarray] = None

    def satisfied(self, controls: np.ndarray) -> bool:
        if self.bounds is None:
            return True
        lo = self.bounds[:, 0]
        hi = self.bounds[:, 1]
        return bool(np.all(controls >= lo - 1e-9) and np.all(controls <= hi + 1e-9))
