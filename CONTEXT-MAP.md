# Workspace Context Map

`open-kinematics` is a virtual uv workspace. Each directory under `packages/`
is independently versioned, tested, built, and released.
The root `uv.lock` is the workspace's only tracked dependency lockfile.

## Product responsibilities

- `suspension_contracts`: solver-independent, versioned Geometry Contract V1.
- `suspension_kernel`: the generic C++ multibody kernel, its CMake build, and the
  product-independent part of the ctypes binding (library path resolution,
  exported-symbol probing, the ABI gate, build metadata, error types). Carries no
  element semantics.
- `suspension_kinematics`: daily suspension geometry design and optimization.
- `suspension_multibody`: high-fidelity quasi-static K&C, load analysis, the
  axle dynamics semantics layer over the native kernel, and optional Adams
  validation. Its Python surface is layered by dependency direction: the low
  layer (`modeling/`, with `modeling/primitives/` holding the spatial algebra and
  the joint/body declarations), the authoring layer (`templates/`,
  `connections/`, `rigs/`, `subsystems/`, `preparation/`, `cases/`), the
  compilation layer (`compilation/`, `simulation/`, `studies/`), and the result
  and reporting layer (`schema/`, `results/`, `outputs/`, `report/`, `io/`).
  `modeling/` must not import any of the layers above it, and
  `tests/architecture/test_import_boundaries.py` enforces that in a fresh
  process per entry point. The `elements/` (A1) and `analysis/` (A2) packages
  stay where they are; both have live production callers and a native
  capability that does not exist yet, and the reason, blocking condition and
  release condition for each are stated in
  `packages/suspension_multibody/README.md` and checked against the boundary
  gate by `packages/suspension_multibody/scripts/check_composable_release.py`.

## Native axle dynamics kernel

The kernel sources live in
`packages/suspension_kernel/cpp/axle_dynamics/axle_kernel.cpp` and are built by
CMake + Ninja into `packages/suspension_kernel/src/suspension_kernel/native/`.
`suspension_multibody` keeps a copy in its own `native/` directory, which is the
path its ctypes boundary loads and the path its wheel packages; that copy is
produced by `packages/suspension_multibody/scripts/build_axle_native.py`, a
wrapper that delegates to the kernel build. The kernel is reached only through
`suspension_multibody.axle_dynamics`. Importing the package without the library
succeeds; running a transient case raises `NativeKernelUnavailableError`. Design,
results, and known limitations live in
`packages/suspension_multibody/docs/axle_dynamics_*.md`; the kernel's own layout
and build are described in `packages/suspension_kernel/README.md`.

## Dependency direction

```text
suspension_kinematics --> suspension_contracts <-- suspension_multibody
suspension_multibody --> suspension_kernel
```

The two solver products must not import each other. The contracts package must
not import either solver product or Adams-related code. Adams discovery and
execution remain inside `suspension_multibody`; importing any product must not
start Adams.

## Geometry handoff

`suspension_kinematics.adapters.export_geometry_contract` exports Geometry
Contract V1. `suspension_multibody.adapters.front_axle_model_from_contract`
consumes that contract and accepts multibody-specific mass data separately.
The contract therefore transfers geometry only, not force elements, tire data,
or solver settings.

## Local artifacts

Generated analysis results, animations, and Adams evidence belong below
`artifacts/` at the workspace root or package level. They are intentionally
ignored and are not package source, release input, or a dependency of runtime
code.

## Composable multibody architecture

The composable architecture is implemented, and the packages above describe the
state it produced. The migration record -- what each subtask changed, and what it
verified -- is in
[EPIC.md](.codex-tasks/multibody-composable-architecture/EPIC.md) and
[SUBTASKS.csv](.codex-tasks/multibody-composable-architecture/SUBTASKS.csv) with
its per-task evidence under `tasks/`; the design rationale is in
[DESIGN.md](.codex-tasks/multibody-composable-architecture/DESIGN.md). Those files
record the plan and its history, so they are not the place to read what the code
does now.
The domain glossary is in
[suspension_multibody/CONTEXT.md](packages/suspension_multibody/CONTEXT.md), and
the executable extension examples are in
[suspension_multibody/docs/composable_extension_examples.md](packages/suspension_multibody/docs/composable_extension_examples.md).
