"""Kinodynamic RRT* planner.

A clean, dynamics-agnostic implementation of kRRT* (Webb & van den Berg, 2013)
built on the :class:`~krrtstar.dynamics.base.Dynamics` interface. The expensive
inner operations (nearest search, connect) are delegated to the dynamics
backend and, when available, an optional Rust acceleration module for the
Euclidean nearest-neighbor pre-filter.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional

import numpy as np

from .dynamics.base import Dynamics, Trajectory
from .geometry import CollisionChecker

ProgressCallback = Callable[[int, "Tree"], None]


@dataclass
class Node:
    state: np.ndarray
    parent: int
    cost_from_start: float
    traj_from_parent: Optional[Trajectory] = None


@dataclass
class Tree:
    """Planner tree with cached state matrix for fast pre-filtering."""

    x_dim: int
    nodes: List[Node] = field(default_factory=list)

    def add(self, node: Node) -> int:
        self.nodes.append(node)
        return len(self.nodes) - 1

    @property
    def states(self) -> np.ndarray:
        if not self.nodes:
            return np.zeros((0, self.x_dim))
        return np.array([n.state for n in self.nodes], dtype=float)

    def path_to(self, idx: int) -> List[int]:
        path = []
        while idx != -1:
            path.append(idx)
            idx = self.nodes[idx].parent
        return list(reversed(path))


@dataclass
class PlanResult:
    tree: Tree
    goal_index: Optional[int]
    solution: Optional[List[Trajectory]]
    cost: Optional[float]

    @property
    def found(self) -> bool:
        return self.goal_index is not None


class UniformSampler:
    def __init__(self, bounds: np.ndarray, rng: np.random.Generator):
        self.lo = np.asarray(bounds[0], float)
        self.hi = np.asarray(bounds[1], float)
        self.rng = rng

    def sample(self) -> np.ndarray:
        return self.rng.uniform(self.lo, self.hi)


class KRRTStar:
    def __init__(
        self,
        dynamics: Dynamics,
        state_bounds: np.ndarray,
        x_init: np.ndarray,
        x_goal: np.ndarray,
        collision: Optional[CollisionChecker] = None,
        connection_radius: float = 10.0,
        goal_bias: float = 0.1,
        goal_tolerance: float = 0.5,
        rewire: bool = True,
        euclidean_gate: Optional[float] = None,
        seed: int = 0,
    ) -> None:
        self.dyn = dynamics
        self.state_bounds = np.asarray(state_bounds, float)
        self.x_init = np.asarray(x_init, float).reshape(-1)
        self.x_goal = np.asarray(x_goal, float).reshape(-1)
        self.collision = collision
        self.radius = float(connection_radius)
        self.goal_bias = float(goal_bias)
        self.goal_tolerance = float(goal_tolerance)
        self.rewire = bool(rewire)
        self.euclidean_gate = euclidean_gate
        self.rng = np.random.default_rng(seed)

        self.tree = Tree(x_dim=dynamics.x_dim)
        self.tree.add(Node(self.x_init, parent=-1, cost_from_start=0.0))
        self.goal_index: Optional[int] = None
        self._sampler = UniformSampler(self.state_bounds, self.rng)

    # ------------------------------------------------------------------ #
    def _sample(self) -> np.ndarray:
        if self.rng.random() < self.goal_bias:
            return self.x_goal.copy()
        return self._sampler.sample()

    def _candidate_indices(self, x: np.ndarray) -> np.ndarray:
        """Euclidean pre-filter of tree nodes before costly connect calls."""
        states = self.tree.states
        if states.shape[0] == 0:
            return np.zeros(0, dtype=int)
        if self.euclidean_gate is None:
            return np.arange(states.shape[0])
        dists = np.linalg.norm(states - x, axis=1)
        return np.nonzero(dists <= self.euclidean_gate)[0]

    def _collision_free(self, traj: Optional[Trajectory]) -> bool:
        if traj is None or not traj.feasible:
            return False
        if self.collision is None:
            return True
        return not self.collision.trajectory_in_collision(traj.states)

    def _choose_parent(self, x_new: np.ndarray):
        """Return ``(parent_idx, trajectory, cost_from_start)`` or ``None``."""
        best = None
        for i in self._candidate_indices(x_new):
            node = self.tree.nodes[i]
            traj = self.dyn.connect(node.state, x_new)
            if traj is None or traj.cost > self.radius:
                continue
            total = node.cost_from_start + traj.cost
            if best is not None and total >= best[2]:
                continue
            if not self._collision_free(traj):
                continue
            best = (i, traj, total)
        return best

    def _rewire(self, new_idx: int) -> None:
        new_node = self.tree.nodes[new_idx]
        x_new = new_node.state
        for i in self._candidate_indices(x_new):
            if i == new_idx:
                continue
            node = self.tree.nodes[i]
            traj = self.dyn.connect(x_new, node.state)
            if traj is None or traj.cost > self.radius:
                continue
            new_cost = new_node.cost_from_start + traj.cost
            if new_cost + 1e-9 >= node.cost_from_start:
                continue
            if not self._collision_free(traj):
                continue
            delta = node.cost_from_start - new_cost
            node.parent = new_idx
            node.traj_from_parent = traj
            node.cost_from_start = new_cost
            self._propagate_cost(i, delta)

    def _propagate_cost(self, root_idx: int, delta: float) -> None:
        children = [j for j, n in enumerate(self.tree.nodes) if n.parent == root_idx]
        for j in children:
            self.tree.nodes[j].cost_from_start -= delta
            self._propagate_cost(j, delta)

    def _try_goal(self, from_idx: int) -> None:
        node = self.tree.nodes[from_idx]
        if float(np.linalg.norm(node.state - self.x_goal)) > self.goal_tolerance:
            return
        cost = node.cost_from_start
        if self.goal_index is None or cost < self.tree.nodes[self.goal_index].cost_from_start:
            self.goal_index = from_idx

    # ------------------------------------------------------------------ #
    def grow(
        self,
        n: int,
        progress_cb: Optional[ProgressCallback] = None,
        progress_every: int = 50,
    ) -> PlanResult:
        for it in range(n):
            x_new = self._sample()
            parent = self._choose_parent(x_new)
            if parent is not None:
                parent_idx, traj, total = parent
                new_idx = self.tree.add(
                    Node(x_new, parent=parent_idx, cost_from_start=total, traj_from_parent=traj)
                )
                if self.rewire:
                    self._rewire(new_idx)
                self._try_goal(new_idx)
            if progress_cb is not None and (it % progress_every == 0):
                progress_cb(it, self.tree)
        if progress_cb is not None:
            progress_cb(n, self.tree)
        return self.result()

    def result(self) -> PlanResult:
        if self.goal_index is None:
            return PlanResult(self.tree, None, None, None)
        path = self.tree.path_to(self.goal_index)
        trajs = [self.tree.nodes[i].traj_from_parent for i in path if self.tree.nodes[i].traj_from_parent]
        cost = self.tree.nodes[self.goal_index].cost_from_start
        return PlanResult(self.tree, self.goal_index, trajs, cost)
