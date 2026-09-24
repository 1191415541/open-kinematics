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
  validation. Its Python surface is the authoring layer (`preparation/`), the
  contract, result and runner layers (`schema/`, `results/`, `kernel/`,
  `simulation/`, `cases/`) and the reporting layer (`report/`); the `elements/`
  (A1) and `analysis/` (A2) packages stay where they are, with the reason and
  removal condition recorded per item in the deletion record of
  `.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-08-delete/`.

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

## Composable Multibody Architecture Plan

The target design is recorded in
[EPIC.md](.codex-tasks/multibody-composable-architecture/EPIC.md) and
[DESIGN.md](.codex-tasks/multibody-composable-architecture/DESIGN.md).
This is a planning deliverable, not a statement that the migration is implemented.
Implementation dependencies and acceptance criteria live in that directory's
`SUBTASKS.csv` and `TASKS.md`; implementation has not started.
The domain glossary is in
[suspension_multibody/CONTEXT.md](packages/suspension_multibody/CONTEXT.md).
