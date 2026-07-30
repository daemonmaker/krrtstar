"""Save and load experiments.

An experiment bundle is a self-describing directory:

    experiment/
      config.toml    # resolved experiment configuration
      tree.npz       # serialized tree (states, parents, costs, controls, taus)
      meta.json      # references, seed, versions, solution metadata

This lets a tree, its robot/environment references, and hyper-parameters be
persisted and reloaded deterministically.
"""

from __future__ import annotations

import json
import os
import shutil
from dataclasses import asdict
from typing import Optional

import numpy as np

from . import __version__
from .planner import Node, PlanResult, Tree


def _tree_arrays(tree: Tree):
    n = len(tree.nodes)
    x_dim = tree.x_dim
    states = np.zeros((n, x_dim))
    parents = np.zeros(n, dtype=np.int64)
    costs = np.zeros(n)
    taus = np.full(n, np.nan)
    for i, node in enumerate(tree.nodes):
        states[i] = node.state
        parents[i] = node.parent
        costs[i] = node.cost_from_start
        if node.traj_from_parent is not None:
            taus[i] = node.traj_from_parent.tau
    return states, parents, costs, taus


def save_experiment(
    path: str,
    result: PlanResult,
    config_path: Optional[str] = None,
    meta_extra: Optional[dict] = None,
) -> str:
    """Write an experiment bundle to ``path`` (a directory)."""
    os.makedirs(path, exist_ok=True)
    states, parents, costs, taus = _tree_arrays(result.tree)
    np.savez_compressed(
        os.path.join(path, "tree.npz"),
        states=states,
        parents=parents,
        costs=costs,
        taus=taus,
        x_dim=np.array([result.tree.x_dim]),
    )

    if config_path and os.path.exists(config_path):
        shutil.copy(config_path, os.path.join(path, "config.toml"))

    meta = {
        "version": __version__,
        "n_nodes": len(result.tree.nodes),
        "x_dim": result.tree.x_dim,
        "goal_index": result.goal_index,
        "solution_cost": result.cost,
        "found": result.found,
        "config": os.path.basename(config_path) if config_path else None,
    }
    if meta_extra:
        meta.update(meta_extra)
    with open(os.path.join(path, "meta.json"), "w") as fh:
        json.dump(meta, fh, indent=2)
    return path


def load_experiment(path: str):
    """Load a bundle, returning ``(tree, meta)``.

    The reconstructed tree carries states, parent links, and costs. Full
    per-edge trajectories are not re-solved on load (they can be regenerated
    from the dynamics); ``taus`` are preserved for reference.
    """
    with np.load(os.path.join(path, "tree.npz")) as data:
        states = data["states"]
        parents = data["parents"]
        costs = data["costs"]
        x_dim = int(data["x_dim"][0])
    tree = Tree(x_dim=x_dim)
    for i in range(states.shape[0]):
        tree.nodes.append(
            Node(state=states[i], parent=int(parents[i]), cost_from_start=float(costs[i]))
        )
    meta_path = os.path.join(path, "meta.json")
    meta = {}
    if os.path.exists(meta_path):
        with open(meta_path) as fh:
            meta = json.load(fh)
    return tree, meta
