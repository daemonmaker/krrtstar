# Conversation Log: Toward a Hybrid Python + Rust Rewrite

This document preserves the design conversation that motivated the rewrite plan in
[`rewrite-plan.md`](./rewrite-plan.md). It is lightly edited for readability but
faithful to the original exchange.

---

## 1. "What's in this repo?"

**Summary of findings:**

The repository is **`daemonmaker/krrtstar`**, a research codebase implementing
**kinodynamic RRT\*** (kRRT\*), a sampling-based motion planner for systems with
dynamics (velocity, acceleration, control inputs) rather than purely geometric
paths. The latest commit was working on a Keras-based Python port.

### What it does

Plans trajectories for robots with dynamics in cluttered environments. Extends
RRT\* (optimal rapidly-exploring random trees) to kinodynamic systems using:

- k-d trees for fast nearest-neighbor search,
- closed-form reachable-set calculations for connecting states,
- **Callisto** for 3D visualization and collision checking,
- **LQR / CARE-based cost matrices** (computed in MATLAB) for optimal control
  penalties.

Supported dynamics (selectable via compile-time flags): single integrator (2D),
double integrator (1D and 2D), quadrotor, nonholonomic car.

### Repository layout

| Directory | Purpose |
|-----------|---------|
| `src/` | Main C++ implementation (`dynamicrrt.cpp` ~5k lines, `rpoly.cpp` for root finding) |
| `includes/` | Headers: dynamics, worlds, robots, k-d tree, simulation, visualization |
| `krrtstar/` | Visual Studio 2010 solution/project (`.sln`, `.vcxproj`) |
| `callisto2.20/` | Bundled [Callisto](http://www.nieuwenhuisen.nl/callisto/) 3D library (headers, `.lib`, `.dll`) |
| `matlab/` | Scripts for CARE/LQR solutions and kRRT\* penalty matrix computation |
| `maple/` | Maple worksheet (`dynamicrrt.mw`) for symbolic work |
| `python/` | In-progress Python/Keras reimplementation (`krrtstar.py`) |

### Core C++ architecture

- `includes/dynamicrrt.h` defines planner configuration, dynamics types, and
  data structures (tree nodes, k-d tree, worlds).
- `src/dynamicrrt.cpp` implements `rrtstar()` (the main kRRT\* loop), an
  interactive experiment runner with logging, collision checking, path
  simulation, and visualization hooks.
- Dependencies: **Eigen**, **Boost**, **Callisto**, **MATLAB** headers (`mat.h`).

### Python port (in progress)

`python/krrtstar.py` is a partial rewrite using **Keras/TensorFlow** for learned
forward dynamics and reward models. It defines `ForwardDynamicsAndRewardDNN`,
basic `rrt()` / `drrt()` loops, and a commented-out `krrtstar()` stub. It imports
`dynamics` and `dynamics.pendulum`, which are **not present in this repo**, so the
Python side is incomplete and depends on external modules.

### Platform notes

- Windows-centric; built for Visual Studio 2010 with Callisto DLLs.
- No modern build system (`pyproject.toml`, `requirements.txt`) — a legacy C++
  research project with an experimental Python branch.

---

## 2. "What would it take to convert this into a hybrid Python + Rust or C/C++ program that permits system dynamics to be specified via configuration files using classic physics methods (as now) or PyTorch models?"

**Short version:** This is effectively a **re-architecture / rewrite**, not a
port. The existing C++ bakes each dynamics model into the program at *compile
time* through preprocessor macros, fixed-size Eigen types, and hand-derived
symbolic math. Making dynamics *runtime-configurable* fundamentally breaks those
assumptions. The existing Python side already sketches the abstraction we want
(`Dynamics` + `DistanceMeasure`, with both an analytic `Pendulum` and a learned
`ForwardDynamicsAndRewardDNN`).

### Why the current code resists this — three hard couplings

1. **Dimensions are compile-time constants.** `X_DIM`, `U_DIM`, `Z_DIM` are
   `#define`s chosen by a single `#define DYNAMICS ...` switch. Every core type
   is a *fixed-size* Eigen matrix templated on those macros
   (`typedef Eigen::Matrix<double,X_DIM,X_DIM> natural_dynamics_t;`). Runtime
   configurability requires dynamically-sized matrices.

2. **The dynamics model is scattered across the preprocessor.** The linear
   system `xdot = Ax + Bu + c` and cost weights are hand-populated per model in
   `setupParameters()` with big `#if (DYNAMICS == ...)` blocks. World/robot/state-space
   selection is likewise macro-driven.

3. **The performance-critical "connect" is symbolically derived per model.**
   kRRT\* connects two states by solving an optimal-time two-point boundary value
   problem. Here that's done in closed form via polynomial root-finding, with
   coefficients *derived in Maple* (`maple/dynamicrrt.mw`) and pasted in as literal
   expressions, dispatched by `#if (DYNAMICS == ...)`. Cost/gain matrices
   (LQR/CARE) are computed offline in MATLAB and read from `parameters.txt`. So
   "classic physics" today means "a human did the symbolic optimal-control
   derivation per model" — this does not generalize for free.

Also heavy coupling to **Callisto** (Windows-only viz + collision) and **MATLAB**
(`mat.h`), both of which we'd want to drop.

### Target hybrid architecture

```
Python (orchestration / UX / ML)
  - load config -> build Problem spec
  - dynamics backends: AnalyticDynamics (classic physics) | TorchDynamics (PyTorch)
  - sampling, goals, collision-world spec, visualization
  - pytest test suite, experiment logging
        |  FFI boundary (PyO3 / pybind11 / cffi)
Rust or C/C++ core (hot loop)
  - kRRT* tree, k-d tree nearest-neighbor, rewire
  - generic over runtime dimensions (nalgebra / Eigen dynamic)
  - calls a Dynamics trait for: propagate, cost, connect (BVP),
    reachable-set bounds, collision
```

Recommendation: **Rust for the core** (via **PyO3/maturin**) for memory-safe
graph/tree code, `nalgebra` for dynamic-size linear algebra, and clean Python
packaging. C++ with pybind11 is the lower-friction option to salvage existing
Eigen-based routines like `rpoly`.

### The dynamics abstraction + config schema

One interface implemented by both a classic-physics backend and a PyTorch
backend:

```python
class Dynamics(Protocol):
    x_dim: int; u_dim: int
    def propagate(self, x, u, dt) -> x_next
    def cost(self, x0, x1) -> (cost, tau, dcost_dt)
    def connect(self, x0, x1) -> Trajectory | None
    def reachable_bounds(self, x0, radius) -> BOUNDS
    def u_bounds(self) -> array
```

Config selects and parameterizes the backend (linear matrices `A/B/c/R` for the
classic path; a TorchScript model path for the learned path). This replaces both
the `#define DYNAMICS` switch and the `setupParameters()` `#if` blocks with data.

### The genuinely hard part: `connect` for arbitrary dynamics

- **Linear analytic backend** -> replace the Maple-derived polynomials with a
  *general* fixed-final-state LQR / indirect optimal-control solver. For
  `xdot = Ax + Bu` with quadratic control cost, the optimal-time connection
  reduces to computing the weighted controllability Gramian
  `G(tau) = integral_0^tau exp(At) B R^-1 B^T exp(A^T t) dt` and minimizing cost
  over `tau` via a 1-D root find. Works for *any* linear system read from config —
  no per-model symbolic derivation, no MATLAB, no Maple. This is the key insight
  that makes classic physics config-driven.
- **PyTorch backend** -> no BVP solve; use gradient/shooting (the Python prototype
  gestures at this via `find_controls` iterating on `u_grads`), or a short
  MPC/iLQR rollout using autograd through the learned model.

### Key risks / decisions

- **FFI callback overhead**: calling Python (esp. PyTorch) per edge from a native
  hot loop can dominate runtime. Mitigate by batching connection queries or
  exporting models to TorchScript/ONNX and running inference inside the native
  core (`tch` crate for Rust, LibTorch for C++).
- **Learned-dynamics connect quality**: shooting/gradient BVP solves are slower
  and can fail; the planner must tolerate `connect -> None`.
- **Optimality guarantees**: kRRT\* asymptotic optimality assumes a proper
  steering function; learned models won't satisfy this rigorously — treat the
  PyTorch mode as best-effort.
- **Language choice**: Rust (safer graph code, clean packaging) vs C++ (reuse
  `rpoly`/Eigen).

---

## 3. Refined requirements (user notes)

The system needs to be able to:

- allow a user to define the matrices for a linear dynamical model,
- define 3D objects to represent the robot model and an environment of obstacles,
- perform collision checking between 3D objects,
- visualize trees and trajectories in 3D,
- render the robot model,
- render a 3D environment of obstacles,
- save and load an experiment, i.e. a tree, references to the robot model,
  environment definition, and any hyper-parameters,
- support tree generation with and without visualization during tree growth,
- use Rust for the hot path,
- treat learned models as best-effort.

These requirements are the basis for [`rewrite-plan.md`](./rewrite-plan.md).
