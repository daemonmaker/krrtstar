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
- **3D geometry & collision** — spheres, oriented boxes, capsules, cylinders, and
  triangle meshes loaded from `.obj` (or any format `trimesh`/`pyvista` can
  read), for both the robot and the obstacles. Shapes can be rotated, and the
  robot's orientation can be driven from the state (Euler indices or a
  quaternion). Collision runs natively via `parry3d` when the Rust core is
  built, with a pure-Python GJK narrow phase as the fallback; both are
  cross-validated to agree exactly.
- **kRRT\* planner** — nearest/near search, best-parent selection, and rewiring
  over the pluggable `Dynamics` interface.
- **Save / load experiments** — persist the tree, config, and metadata as a
  self-describing bundle.
- **Visualization** — 3D rendering of the tree, trajectories, robot, and
  obstacles, with optional live updates during tree growth (headless supported).
  The window opens in the foreground and stays open after planning finishes so
  the result can be inspected; press `q` (or close the window) to exit. Set
  `[visualization] keep_open = false` (or pass `--no-keep-open`) for unattended
  runs.
- **Timing** — planning duration and throughput are reported and stored in the
  saved experiment metadata.
- **Rust hot path (optional)** — a `krrtstar_core` extension runs the whole
  connection in native code: the Euclidean neighbor pre-filter, batched
  connection costs (sharing a precomputed time grid), and full trajectory
  reconstruction, plus `parry3d` collision checking (~15x faster than the Python
  fallback). The package falls back to pure Python when it is not built.
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

Watch the tree grow in 3D and inspect the result when it finishes:

```bash
poetry run python -m krrtstar.cli examples/double_integrator_2d.toml --live
```

which reports, for example:

```
[krrtstar] backend=rust dynamics=linear x_dim=4 target_nodes=300
[krrtstar] nodes=236 found=True cost=15.6876
[krrtstar] planning time: 1.42 s (166.2 nodes/s)
[krrtstar] total time: 1.51 s
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

> Mesh obstacles are treated as **surfaces**, not filled solids: a shape that
> fits entirely inside a closed mesh without touching a triangle is not in
> collision. Prefer primitives for large hollow volumes.

```
krrtstar/        Python package (config, dynamics, geometry, planner, IO, viz)
rust/            Optional Rust acceleration crate (krrtstar_core)
examples/        Example experiment configs
tests/           pytest suite
docs/            Design docs (conversation.md, rewrite-plan.md)
```
