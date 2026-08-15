"""Command-line entrypoint.

Usage:
    python -m krrtstar.cli CONFIG.toml [--out DIR] [--live] [--render OUT.png]
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Optional

from .accel import backend
from .config import load_config
from .experiment import save_experiment
from .run import run_experiment


def format_duration(seconds: Optional[float]) -> str:
    """Human-readable duration, e.g. ``842.1 ms``, ``1.47 s``, ``2m 03.4s``."""
    if seconds is None:
        return "n/a"
    if seconds < 1.0:
        return f"{seconds * 1000.0:.1f} ms"
    if seconds < 60.0:
        return f"{seconds:.2f} s"
    minutes, rest = divmod(seconds, 60.0)
    return f"{int(minutes)}m {rest:04.1f}s"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Run a kRRT* experiment from a config file.")
    parser.add_argument("config", help="path to a TOML experiment config")
    parser.add_argument("--out", help="directory to save the experiment bundle")
    parser.add_argument("--live", action="store_true", help="visualize tree growth live")
    parser.add_argument("--render", help="render the final result to an image path")
    parser.add_argument("--nodes", type=int, help="override planner.target_nodes")
    parser.add_argument(
        "--no-keep-open",
        action="store_true",
        help="close the live window when planning finishes instead of holding it open",
    )
    args = parser.parse_args(argv)

    cfg = load_config(args.config)
    if args.live:
        cfg.visualization.live = True
    if args.no_keep_open:
        cfg.visualization.keep_open = False
    if args.nodes is not None:
        cfg.planner.target_nodes = args.nodes

    print(f"[krrtstar] backend={backend()} dynamics={cfg.dynamics.kind} "
          f"x_dim={cfg.dynamics.x_dim} target_nodes={cfg.planner.target_nodes}")

    wall_start = time.perf_counter()
    result = run_experiment(cfg)

    cost = "n/a" if result.cost is None else f"{result.cost:.4f}"
    print(f"[krrtstar] nodes={len(result.tree.nodes)} found={result.found} cost={cost}")

    rate = result.nodes_per_second
    planning = f"[krrtstar] planning time: {format_duration(result.elapsed)}"
    if rate is not None:
        planning += f" ({rate:.1f} nodes/s)"
    print(planning)

    if args.out:
        save_experiment(args.out, result, config_path=args.config)
        print(f"[krrtstar] saved experiment to {args.out}")

    if args.render:
        try:
            from .viz import render_result

            render_result(cfg, result, save_path=args.render, offscreen=True)
            print(f"[krrtstar] rendered to {args.render}")
        except Exception as exc:  # pragma: no cover - viz is optional
            print(f"[krrtstar] render skipped: {exc}", file=sys.stderr)

    # Total wall time. With live visualization this includes however long the
    # window stayed open for inspection.
    print(f"[krrtstar] total time: {format_duration(time.perf_counter() - wall_start)}")

    return 0 if result.found else 1


if __name__ == "__main__":
    raise SystemExit(main())
