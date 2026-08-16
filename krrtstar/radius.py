"""Connection-radius schedules for kRRT*.

The near-set radius controls the cost threshold within which candidate parents
and rewiring targets are considered. It trades solution quality against work per
iteration: a large radius considers more candidates (better rewiring, higher
cost), a small one is cheap but may miss improvements.

RRT*'s asymptotic-optimality argument requires the radius to shrink with the
number of samples like

    r(n) = gamma * (log(n) / n)^(1/d)

with ``d`` the state-space dimension and ``gamma`` large enough (it depends on
the free-space volume). A radius that is held constant keeps a bounded set of
candidates and forfeits that guarantee; one that shrinks too quickly stops
finding connections at all. This module provides the schedules and a way to pin
``gamma`` to something meaningful rather than guessing it.

Note that for kinodynamic planning the radius lives in *cost* space (cost is
``tau + control effort``), not in Euclidean distance, so ``gamma`` is not a
length. Deriving it from a reference radius (see :func:`build_schedule`) is
therefore much easier to reason about than setting it directly.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Protocol, runtime_checkable

import numpy as np


@runtime_checkable
class RadiusSchedule(Protocol):
    """Maps the current tree size to a connection radius."""

    def radius(self, n_nodes: int) -> float:
        ...


@dataclass
class ConstantRadius:
    """A fixed radius (the historical behaviour)."""

    value: float

    def radius(self, n_nodes: int) -> float:
        return float(self.value)

    def __repr__(self) -> str:  # pragma: no cover - cosmetic
        return f"ConstantRadius({self.value:g})"


@dataclass
class RRTStarRadius:
    """The shrinking schedule ``r(n) = gamma * (log n / n)^(1/dim)``.

    Args:
        gamma: scale factor; see :func:`derive_gamma` to pin it to a reference.
        dim: state-space dimension used for the exponent.
        r_min: floor, so the radius cannot collapse and stall the search.
        r_max: ceiling, to bound the work done in the earliest iterations.
    """

    gamma: float
    dim: int
    r_min: float = 0.0
    r_max: float = math.inf

    def radius(self, n_nodes: int) -> float:
        n = max(int(n_nodes), 2)  # log(1) = 0 would give a zero radius
        value = self.gamma * (math.log(n) / n) ** (1.0 / max(self.dim, 1))
        return float(min(max(value, self.r_min), self.r_max))

    def __repr__(self) -> str:  # pragma: no cover - cosmetic
        return (
            f"RRTStarRadius(gamma={self.gamma:g}, dim={self.dim}, "
            f"r_min={self.r_min:g}, r_max={self.r_max:g})"
        )


@dataclass
class GeometricRadius:
    """Legacy-style geometric schedule ``r(n) = r0 * multiplier**n``.

    Mirrors the old C++ ``RADIUS_MULTIPLIER`` knob: values slightly below 1
    shrink the radius as the tree grows, slightly above 1 grow it.
    """

    r0: float
    multiplier: float = 1.0
    r_min: float = 0.0
    r_max: float = math.inf

    def radius(self, n_nodes: int) -> float:
        value = self.r0 * (self.multiplier ** max(int(n_nodes), 0))
        return float(min(max(value, self.r_min), self.r_max))

    def __repr__(self) -> str:  # pragma: no cover - cosmetic
        return f"GeometricRadius(r0={self.r0:g}, multiplier={self.multiplier:g})"


def derive_gamma(reference_radius: float, dim: int, n_reference: int) -> float:
    """Pick ``gamma`` so that ``r(n_reference) == reference_radius``.

    This makes the shrinking schedule directly comparable to a constant radius:
    the two agree once the tree reaches ``n_reference`` nodes, and the schedule
    is simply more generous before that point.
    """
    n = max(int(n_reference), 2)
    factor = (math.log(n) / n) ** (1.0 / max(int(dim), 1))
    if factor <= 0.0:
        return float(reference_radius)
    return float(reference_radius) / factor


def build_schedule(
    kind: str,
    connection_radius: float,
    dim: int,
    target_nodes: int,
    gamma: Optional[float] = None,
    multiplier: float = 1.0,
    r_min: Optional[float] = None,
    r_max: Optional[float] = None,
) -> RadiusSchedule:
    """Construct a schedule from configuration values.

    ``kind`` is ``"constant"`` (default), ``"rrtstar"`` or ``"geometric"``.

    For ``"rrtstar"``, ``gamma`` defaults to :func:`derive_gamma` pinned at a
    *quarter* of ``target_nodes``, so the radius reaches ``connection_radius``
    early and then shrinks below it. Pinning at ``target_nodes`` instead would
    mean the radius never drops under ``connection_radius``, which measurably
    only wastes work: on the bundled 2D example that produced solutions
    identical to a constant radius while running ~26% slower, because quality
    had already saturated at that radius. Shrinking below it matched the same
    cost roughly 31% faster.

    ``r_min`` defaults to half of ``connection_radius`` so the radius cannot
    collapse to the point where nothing connects.
    """
    kind = (kind or "constant").lower()
    hi = math.inf if r_max is None else float(r_max)

    if kind == "constant":
        return ConstantRadius(float(connection_radius))
    if kind == "rrtstar":
        if gamma is None:
            gamma = derive_gamma(
                connection_radius, dim, max(8, int(target_nodes) // 4)
            )
        lo = float(connection_radius) / 2.0 if r_min is None else float(r_min)
        return RRTStarRadius(gamma=float(gamma), dim=int(dim), r_min=lo, r_max=hi)
    lo = 0.0 if r_min is None else float(r_min)
    if kind == "geometric":
        return GeometricRadius(
            r0=float(connection_radius), multiplier=float(multiplier), r_min=lo, r_max=hi
        )
    raise ValueError(
        f"Unknown radius schedule: {kind!r} (expected 'constant', 'rrtstar' or 'geometric')"
    )


def calibrate_euclidean_gate(
    dynamics,
    state_bounds,
    radius: float,
    samples: int = 400,
    safety: float = 1.25,
    seed: int = 0,
) -> float:
    """Estimate a Euclidean pre-filter radius that keeps the near set intact.

    The planner can pre-filter candidate neighbours by Euclidean distance before
    paying for cost queries. That is only sound if every state within the *cost*
    radius is also within the Euclidean gate -- otherwise valid near neighbours
    are silently discarded, which costs solution quality and breaks the
    rewiring argument that kRRT* relies on.

    Euclidean distance is not a bound on the optimal-control cost (cost mixes
    time with control effort, and velocity components move a state without
    moving its position), so the gate cannot be derived in closed form for an
    arbitrary system. Instead this samples random state pairs, measures the
    largest Euclidean distance among pairs whose cost is within ``radius``, and
    inflates it by ``safety``.

    The result is an estimate, not a guarantee: raise ``samples``/``safety``, or
    disable gating entirely (``euclidean_gate = None``) for an exact near set.
    """
    bounds = np.asarray(state_bounds, dtype=float)
    lo, hi = bounds[0], bounds[1]
    rng = np.random.default_rng(seed)
    starts = rng.uniform(lo, hi, size=(samples, lo.size))
    targets = rng.uniform(lo, hi, size=(samples, lo.size))

    worst = 0.0
    batch = getattr(dynamics, "cost_batch_to", None)
    for target in targets:
        if batch is not None:
            costs = np.asarray(batch(starts, target), dtype=float)
        else:  # pragma: no cover - backends without batched costs
            costs = np.array(
                [dynamics.cost(s, target) or np.inf for s in starts], dtype=float
            )
        within = np.isfinite(costs) & (costs <= radius)
        if within.any():
            dists = np.linalg.norm(starts[within] - target, axis=1)
            worst = max(worst, float(dists.max()))
    if worst <= 0.0:
        # Nothing was connectable within the radius; fall back to no gating.
        return float("inf")
    return worst * float(safety)
