# krrtstar

Hybrid **Python + Rust** kinodynamic RRT\* (kRRT\*) motion planner with
**config-driven dynamics**. System dynamics, robot geometry, obstacle
environments, and experiments are defined by configuration/data files rather
than compiled per model (as in the legacy C++ implementation).

See [`docs/rewrite-plan.md`](docs/rewrite-plan.md) for the design and
[`docs/conversation.md`](docs/conversation.md) for background.

## Features

- **Config-driven linear dynamics** — define `A`, `B`, `c`, control penalty `R`,
  and control bounds in a TOML file. A single general optimal-time connection
  solver (controllability Gramian + arrival-time optimization) replaces the
  legacy per-model closed forms — no MATLAB/Maple.
- **3D geometry & collision** — robot and obstacles as spheres/boxes; broad +
  narrow phase collision checking; trajectory collision checking.
- **kRRT\* planner** — nearest/near search, best-parent selection, and rewiring
  over the pluggable `Dynamics` interface.
- **Save / load experiments** — persist the tree, config, and metadata as a
  self-describing bundle.
- **Visualization** — 3D rendering of the tree, trajectories, robot, and
  obstacles, with optional live updates during tree growth (headless supported).
- **Rust hot path (optional)** — a `krrtstar_core` extension accelerates the
  Euclidean neighbor pre-filter and linear connection cost; the package falls
  back to pure Python when it is not built.
- **Learned dynamics (best-effort)** — an optional PyTorch backend steers via
  gradient/shooting and is treated as best-effort.

## Install

```bash
poetry install                 # core (numpy, scipy) + dev (pytest)
poetry install --with viz      # + pyvista for 3D visualization
poetry install --with torch    # + PyTorch for learned dynamics
```

Build the optional Rust accelerator into the Poetry venv:

```bash
poetry run maturin develop --manifest-path rust/Cargo.toml --release
```

## Quick start

```bash
poetry run python -m krrtstar.cli examples/double_integrator_2d.toml --out /tmp/exp
```

Or from Python:

```python
from krrtstar.run import run_from_file
result = run_from_file("examples/double_integrator_2d.toml")
print(result.found, result.cost)
```

## Testing

```bash
poetry run pytest
```

## Layout

```
krrtstar/        Python package (config, dynamics, geometry, planner, IO, viz)
rust/            Optional Rust acceleration crate (krrtstar_core)
examples/        Example experiment configs
tests/           pytest suite
docs/            Design docs (conversation.md, rewrite-plan.md)
```
