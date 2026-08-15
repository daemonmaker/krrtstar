"""Command-line entrypoint.

Usage:
    python -m krrtstar.cli CONFIG.toml [--out DIR] [--live] [--render OUT.png]
"""

from __future__ import annotations

import argparse
import sys

from .config import load_config
from .experiment import save_experiment
from .run import run_experiment
from .accel import backend


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Run a kRRT* experiment from a config file.")
    parser.add_argument("config", help="path to a TOML experiment config")
    parser.add_argument("--out", help="directory to save the experiment bundle")
    parser.add_argument("--live", action="store_true", help="visualize tree growth live")
    parser.add_argument("--render", help="render the final result to an image path")
    parser.add_argument("--nodes", type=int, help="override planner.target_nodes")
    args = parser.parse_args(argv)

    cfg = load_config(args.config)
    if args.live:
        cfg.visualization.live = True
    if args.nodes is not None:
        cfg.planner.target_nodes = args.nodes

    print(f"[krrtstar] backend={backend()} dynamics={cfg.dynamics.kind} "
          f"x_dim={cfg.dynamics.x_dim} target_nodes={cfg.planner.target_nodes}")
    result = run_experiment(cfg)
    print(f"[krrtstar] nodes={len(result.tree.nodes)} found={result.found} cost={result.cost}")

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

    return 0 if result.found else 1


if __name__ == "__main__":
    raise SystemExit(main())
