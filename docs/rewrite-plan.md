# Rewrite Plan: Hybrid Python + Rust kRRT\*

This plan describes how to convert the current Windows/C++/Callisto/MATLAB
research code into a portable, hybrid **Python + Rust** motion planner in which
system dynamics, robot geometry, obstacle environments, and experiments are all
defined by **configuration and data files** rather than compile-time macros.

Background and rationale are in [`conversation.md`](./conversation.md).

---

## 1. Goals and non-goals

### Goals (derived from requirements)

1. Let a user define the **matrices of a linear dynamical model** (`A`, `B`, `c`,
   cost `R`, control bounds) via config.
2. Let a user define **3D objects** for the robot model and an environment of
   obstacles via config/asset files.
3. Perform **collision checking** between 3D objects.
4. **Visualize trees and trajectories in 3D.**
5. **Render the robot model** in 3D.
6. **Render a 3D environment of obstacles.**
7. **Save and load an experiment**: the tree, references to the robot model, the
   environment definition, and hyper-parameters.
8. Support tree generation **with and without visualization** during tree growth.
9. Use **Rust for the hot path** (tree, k-d tree, connect, collision inner loop).
10. Support **learned (PyTorch) dynamics as best-effort** alongside classic
    physics.

### Non-goals (initial scope)

- Reproducing Callisto or MATLAB/Maple exactly. We replace them with portable
  equivalents (a Python viz stack; runtime LQR/CARE solves).
- Formal asymptotic-optimality guarantees for learned dynamics (best-effort only).
- GPU-accelerated batch planning (possible later; not required now).

---

## 2. Target architecture

```
+-------------------------------------------------------------------+
| Python package  (orchestration / config / ML / viz / IO)          |
|                                                                   |
|  config loader (TOML/YAML)  -> Problem spec (validated)           |
|  dynamics backends:                                               |
|     - AnalyticLinearDynamics  (A, B, c, R from config)            |
|     - TorchDynamics           (TorchScript model, best-effort)    |
|  geometry: robot + obstacle 3D models (meshes / primitives)       |
|  experiment IO: save/load tree + refs + hyper-params              |
|  visualization: 3D trees, trajectories, robot, obstacles          |
|  planner driver: with/without live visualization                  |
+-------------------------------┬-----------------------------------+
                                |  PyO3 / maturin FFI
+-------------------------------▼-----------------------------------+
| Rust core crate  (hot path)                                       |
|  - kRRT* main loop (sample, nearest, connect, rewire)             |
|  - k-d tree nearest-neighbor over runtime dimension               |
|  - linear-dynamics connect (Gramian / LQR BVP solver)             |
|  - collision checking (broad + narrow phase)                      |
|  - dynamics dispatch: native linear | callback to Python (Torch)  |
|  - progress callbacks for live visualization                      |
+-------------------------------------------------------------------+
```

### Language / tooling choices

- **Rust core** built with **maturin** + **PyO3**, exposed as a Python extension
  module (e.g. `krrtstar_core`).
- Linear algebra: **`nalgebra`** (dynamic-size matrices) in Rust; **NumPy** in
  Python.
- Learned dynamics: **PyTorch**, exported to **TorchScript**; native inference
  via the **`tch`** crate when the hot path needs it, otherwise a Python
  callback.
- Python packaging & env: **Poetry** virtualenv per the repo, **pytest** for
  tests.
- 3D geometry + collision: primitives + triangle meshes; narrow-phase via a
  crate such as **`parry3d`** (Rust) exposed to Python.
- 3D visualization: **`pyvista`**/VTK (or `trimesh` + `meshcat`) in Python.

---

## 3. Data model and configuration schema

All experiment inputs are data. A single experiment config references dynamics,
robot, environment, planner, and visualization sections.

### 3.1 Dynamics

```toml
[dynamics]
kind = "linear"          # "linear" | "torch"

# --- kind = "linear" ---
x_dim = 4
u_dim = 2
A = [[0,0,1,0],[0,0,0,1],[0,0,0,0],[0,0,0,0]]
B = [[0,0],[0,0],[1,0],[0,1]]
c = [0,0,0,0]
R = [[1,0],[0,1]]                 # quadratic control penalty
u_bounds = [[-2,2],[-2,2]]

# --- kind = "torch" ---
# model_path = "models/pendulum_f_r.pt"   # TorchScript
# x_dim = 2
# u_dim = 1
# connect_method = "gradient"             # best-effort steering
```

### 3.2 Robot model and obstacle environment (3D)

```toml
[robot]
# geometry is a list of primitives and/or mesh assets in the robot frame
[[robot.geometry]]
type = "sphere"; radius = 0.25
# [[robot.geometry]] type = "mesh"; path = "assets/quad.obj"
# position mapping: which state indices map to x/y/z (and orientation)
pose_from_state = { x = 0, y = 1, z = 2 }

[environment]
bounds = [[-10,-10,-10],[10,10,10]]
[[environment.obstacles]]
type = "box"; center = [0,0,0]; extents = [1,4,1]
# [[environment.obstacles]] type = "mesh"; path = "assets/wall.obj"
```

### 3.3 Planner hyper-parameters

```toml
[planner]
target_nodes = 10000
start_radius = 10.0
radius_multiplier = 1.01
goal_bias = 0.1            # epsilon: probability of sampling the goal
rewire = true
seed = 0
x_init = [ -8, -8, 0, 0 ]
x_goal = [  8,  8, 0, 0 ]
```

### 3.4 Visualization

```toml
[visualization]
live = false              # requirement: with/without viz during growth
show_tree = true
show_nodes = false
show_robot = true
show_obstacles = true
```

### 3.5 Experiment save/load format

An experiment bundle is a directory (or archive) containing:

```
experiment/
  config.toml           # full resolved config (dynamics/robot/env/planner/viz)
  tree.bin              # serialized tree (nodes, parents, states, controls, costs)
  meta.json             # references: robot asset path, env definition, hashes, seed, versions
  assets/               # copied or referenced meshes
```

- Tree serialization: a compact binary (e.g. `bincode`/`serde` in Rust, exposed
  to Python) plus a JSON sidecar of metadata so an experiment is reproducible and
  self-describing.
- `meta.json` records the config hash, RNG seed, code/version info, and asset
  references so a saved experiment reloads deterministically.

---

## 4. Core interfaces

### 4.1 Dynamics interface (Python-facing)

```python
class Dynamics(Protocol):
    x_dim: int
    u_dim: int
    def propagate(self, x, u, dt): ...                # forward simulation
    def cost(self, x0, x1): ...                        # (cost, tau, dcost_dtau)
    def connect(self, x0, x1): ...                     # Trajectory | None (states+controls over [0,tau])
    def reachable_bounds(self, x0, radius): ...        # BOUNDS for k-d pruning
    def u_bounds(self): ...
```

- `AnalyticLinearDynamics` implements the optimal-time linear BVP (Section 5).
- `TorchDynamics` implements best-effort steering via gradient/shooting and may
  return `None` when it cannot reach the target within control bounds.

### 4.2 Rust core surface (PyO3)

```
build_planner(config)             -> PlannerHandle
grow(handle, n, progress_cb=None) -> stats          # progress_cb enables live viz
get_tree(handle)                  -> Tree           # states, parents, controls, costs
save_experiment(handle, path)
load_experiment(path)             -> PlannerHandle
collision_check(scene, pose)      -> bool
```

- The Rust core owns the tree, k-d tree, and collision scene. For `linear`
  dynamics it evaluates `connect`/`cost` natively; for `torch` dynamics it calls
  back into Python (or `tch`).
- `progress_cb` is invoked periodically during growth so Python can render
  incremental tree/trajectory updates when `visualization.live = true`, and is
  simply omitted when growing headless.

---

## 5. The hard part: general `connect` for linear dynamics

Replace the Maple-derived, per-model closed-form polynomials with **one general
solver** valid for any linear system read from config.

For `xdot = Ax + Bu + c` with quadratic control cost `integral u^T R u dt`:

1. Compute the weighted controllability Gramian
   `G(tau) = integral_0^tau exp(A s) B R^-1 B^T exp(A^T s) ds`.
2. The cost of connecting `x0 -> x1` in fixed time `tau` is a closed function of
   `G(tau)` and the drift `xbar(tau) = exp(A tau) x0 + integral_0^tau exp(A s) c ds`.
3. Minimize cost over `tau > 0` via a 1-D root find of `d(cost)/d(tau) = 0`
   (reusing a robust root finder; `rpoly` remains a reference for polynomial
   cases).
4. Reconstruct the optimal control and state trajectory for collision checking
   and visualization.

Implementation notes:
- Use matrix-exponential integration (`expm` of the augmented system) to get
  `exp(A tau)` and the Gramian together.
- `reachable_bounds` uses the Gramian to produce an ellipsoidal/box bound for
  k-d tree pruning, generalizing the current `calc_*_reachable_set`.
- This eliminates the MATLAB (`care_solutions.m`) and Maple (`dynamicrrt.mw`)
  dependencies: LQR/CARE-type solves happen at runtime.

For **Torch** dynamics: gradient-descent on the control (as sketched by the
prototype's `find_controls`) or a short iLQR/MPC rollout through the model;
enforce `u_bounds`; return `None` on failure. Best-effort by design.

---

## 6. Collision, geometry, and visualization

- **Geometry**: represent robot and obstacles as primitives (sphere, box,
  capsule, cylinder) and/or triangle meshes loaded from asset files. The robot
  pose is derived from the state via a config-defined mapping.
- **Collision checking**: broad phase (AABB/BVH) + narrow phase via a 3D
  collision crate (e.g. `parry3d`). Trajectory checking samples the connect
  trajectory and tests robot-vs-obstacle at each sample (mirroring current
  threshold/collision behavior, but portable). Runs in the Rust hot path.
- **Visualization** (Python): render obstacles, robot mesh, the tree edges, and
  candidate/solution trajectories in 3D with `pyvista`/VTK. Live mode consumes
  `progress_cb` updates during growth; headless mode skips all rendering. This
  replaces Callisto entirely.

---

## 7. Component work breakdown

| # | Component | Work | Invasiveness / risk |
|---|-----------|------|---------------------|
| 1 | Config schema + loader | TOML/YAML -> validated `Problem`; matrices, geometry, planner, viz | Low |
| 2 | Dynamics interface | Define `Dynamics`; `AnalyticLinearDynamics` first | Low–Med |
| 3 | General linear `connect` | Gramian/LQR BVP solver + 1-D time optimization | **High (core math risk)** |
| 4 | Rust core: tree + k-d tree | Port `rrtstar()` + `KD_Tree`, runtime dimension via `nalgebra` | High |
| 5 | Collision + geometry | Primitives/meshes, broad+narrow phase (`parry3d`) | Med |
| 6 | PyO3 bindings + maturin | Expose build/grow/get_tree/save/load/collision | Med |
| 7 | Experiment save/load | Binary tree + JSON meta + asset refs | Low–Med |
| 8 | Visualization | `pyvista` 3D tree/trajectory/robot/obstacles; live callback | Med |
| 9 | Torch backend | TorchScript load; gradient/shooting connect; best-effort | Med |
| 10 | Tests | pytest parity vs known closed-form integrators; round-trip IO | Low |
| 11 | Packaging | Poetry venv, maturin build, CI | Low |

---

## 8. Incremental milestones

The de-risking order front-loads the math and the abstraction in Python, then
moves the hot path into Rust.

- **M0 — Scaffold.** Poetry project, package skeleton, maturin/PyO3 hello-world
  extension, pytest wired up, config loader for the `[dynamics]` linear section.
- **M1 — Analytic dynamics in pure Python.** Implement `AnalyticLinearDynamics`
  with the Gramian-based `connect`/`cost`. Validate numerically against the
  existing closed-form single- and double-integrator results (parity tests).
- **M2 — Planner in pure Python.** Port kRRT\* + k-d tree to Python/NumPy using
  the `Dynamics` interface. Reproduce a 2D double-integrator example fully from a
  config file. Add save/load of an experiment.
- **M3 — 3D geometry, collision, visualization.** Config-defined robot + obstacle
  meshes, collision checking, and `pyvista` rendering of tree/trajectories/robot.
  Support live vs headless growth.
- **M4 — Rust hot path.** Reimplement the tree, k-d tree, linear `connect`, and
  collision inner loop in Rust behind PyO3. Keep the Python planner as a
  reference oracle for differential testing. Add the `progress_cb` for live viz.
- **M5 — Torch backend (best-effort).** `TorchDynamics` with gradient/shooting
  steering; TorchScript export path; integrate as a dynamics option. Document its
  best-effort nature and failure handling.
- **M6 — Polish.** Experiment bundle format finalized, docs, examples, CI
  building the Rust wheel and running pytest.

Each milestone is independently shippable and testable, satisfying the
requirement set incrementally.

---

## 9. Requirement -> milestone traceability

| Requirement | Delivered in |
|-------------|--------------|
| Define linear model matrices via config | M0–M1 |
| Define 3D robot + obstacle objects | M3 |
| Collision checking between 3D objects | M3 (Rust in M4) |
| Visualize trees and trajectories in 3D | M3 |
| Render robot model | M3 |
| Render 3D obstacle environment | M3 |
| Save/load experiment (tree, refs, hyper-params) | M2 (format finalized M6) |
| Tree growth with/without visualization | M2 (headless) + M3/M4 (live callback) |
| Rust for the hot path | M4 |
| Learned models best-effort | M5 |

---

## 10. Risks and mitigations

- **General `connect` correctness/perf (highest risk).** Mitigate with a
  pure-Python reference implementation (M1) and parity tests against the current
  closed-form integrators before optimizing in Rust.
- **FFI / PyTorch callback overhead.** Prefer native linear `connect` in Rust;
  for Torch, batch queries or run TorchScript via `tch` inside Rust to avoid
  per-edge Python round-trips.
- **Dynamic-dimension linear algebra in Rust.** Use `nalgebra` dynamic matrices;
  accept a modest overhead vs the old fixed-size Eigen types in exchange for
  runtime configurability.
- **Learned-dynamics reachability.** Planner must handle `connect -> None`
  gracefully; treat Torch mode as best-effort with no optimality guarantee.
- **Dropping Callisto/MATLAB/Maple.** Replace viz with `pyvista`, collision with
  `parry3d`, and symbolic/CARE solves with runtime numerical routines.

---

## 11. Repository layout (target)

```
.
├─ pyproject.toml            # Poetry: Python package + maturin build backend
├─ rust/                     # Rust core crate (krrtstar_core)
│  ├─ Cargo.toml
│  └─ src/                   # tree, kd-tree, connect, collision, PyO3 bindings
├─ python/krrtstar/          # Python package
│  ├─ config.py              # config schema + loader
│  ├─ dynamics/              # Dynamics interface, analytic, torch
│  ├─ geometry.py            # robot/obstacle 3D models
│  ├─ experiment.py          # save/load bundles
│  ├─ viz.py                 # 3D visualization (pyvista)
│  └─ planner.py             # driver (with/without live viz)
├─ examples/                 # example experiment configs + assets
├─ tests/                    # pytest suite (parity, round-trip IO)
└─ docs/                     # conversation.md, rewrite-plan.md
```

The legacy `src/`, `includes/`, `krrtstar/`, `callisto2.20/`, `matlab/`, and
`maple/` trees are retained as reference during the port and removed once parity
is established.

---

## 12. Implementation status

An initial working implementation of the full stack has landed (see the
`krrtstar/` Python package, `rust/` crate, `examples/`, and `tests/`):

| Requirement | Status |
|-------------|--------|
| Define linear model matrices via config | Done — `[dynamics]` TOML, `AnalyticLinearDynamics` |
| General optimal-time connect (no MATLAB/Maple) | Done — controllability Gramian + arrival-time optimization; verified against numerical quadrature and control-effort cost |
| Define 3D robot + obstacle objects | Done — `geometry.py` (sphere/box) + config |
| Collision checking between 3D objects | Done — broad + narrow phase |
| Visualize trees/trajectories/robot/obstacles in 3D | Done — `viz.py` (PyVista), offscreen render verified |
| Save/load experiment (tree, refs, hyper-params) | Done — `experiment.py` bundle (`tree.npz` + `config.toml` + `meta.json`) |
| Tree growth with/without visualization | Done — headless by default; live `progress_cb` |
| Rust for the hot path | Done — `krrtstar_core`: neighbor pre-filter, batched connection costs, and full trajectory reconstruction; wired into the planner with a pure-Python fallback |
| Learned models best-effort | Done — `TorchDynamics` (gradient/shooting), best-effort |

Tests: 26 passing + 1 skipped (Torch, when PyTorch is not installed).

### Hot-path performance

The whole connection now runs natively. Two things mattered most:

1. **Trajectory reconstruction in Rust.** The costate/state pair is propagated
   through the composite affine system. Because sample times are uniform, one
   matrix exponential of the per-step generator is computed and applied
   repeatedly, instead of one exponential per sample (~7.5x faster `connect`).
2. **Batched cost queries with a shared time grid.** `Phi(t)`, `Ad(t)` and the
   Gramian `G(t)` depend only on `(A, Q, t)` — never on the endpoint states — so
   `LinearConnector` precomputes them once per system and reuses them for every
   query. Profiling showed cost queries (24k calls) dominated at ~89% of
   runtime; batching removed nearly all of it.

End-to-end on `examples/double_integrator_2d.toml` (identical solution cost):

| Stage | Wall time |
|-------|-----------|
| Pure Python | ~24s |
| Rust cost + Python reconstruction | ~15s |
| Rust reconstruction | ~10s |
| Rust batched cost + reconstruction | **~1.5s** |

### Connection-radius schedule

The near-set radius is now configurable via `[planner] radius_schedule`:
`constant` (default), `rrtstar` (`r(n) = gamma * (log n / n)^(1/d)`, which is
what RRT*'s asymptotic-optimality argument requires), or `geometric` (the legacy
`RADIUS_MULTIPLIER` behaviour). `gamma` is pinned by default so the radius
reaches `connection_radius` at a quarter of the target node count and then
shrinks below it, with a floor at half the configured radius.

Measured on the 2D double integrator (10 seeds, exact near set): quality
saturates around radius 12 (radius 3 finds nothing; 12 and 18 give identical
cost, 18 just costs more time). At 600 nodes the shrinking schedule matches the
best constant radius' cost about 20% faster; shrinking harder trades ~1.5% cost
for ~59% less time. Pinning `gamma` at the *target* node count is a trap -- the
radius then never drops below `connection_radius`, producing identical solutions
~26% slower.

Tuning this surfaced a correctness bug in the Euclidean pre-filter. The gate
rejects candidates by Euclidean distance before the cost query, but Euclidean
distance does not bound the optimal-control cost, so the hand-picked
`euclidean_gate = 6.0` shipped in the examples was discarding 2986 candidates
that were inside the cost radius of 12 while keeping only 2408 -- over half the
near set silently dropped, worth ~14% of solution quality, and enough to make the
radius schedule look like it did nothing at all. The gate is now optional, can be
calibrated from the dynamics (`euclidean_gate = "auto"`), and is documented as an
approximation.

Follow-on work (not yet done): richer geometry (meshes, oriented boxes,
capsules) and moving collision checking into Rust (both in progress separately),
and a rigorous rather than sampled bound for the Euclidean gate.
