# Workspace Context Map

`open-kinematics` is a virtual uv workspace. Each directory under `packages/`
is independently versioned, tested, built, and released.
The root `uv.lock` is the workspace's only tracked dependency lockfile.

## Product responsibilities

- `suspension_contracts`: solver-independent, versioned Geometry Contract V1.
- `suspension_kinematics`: daily suspension geometry design and optimization.
- `suspension_multibody`: high-fidelity quasi-static K&C, load analysis, the
  native transient axle dynamics kernel, and optional Adams validation.

## Native axle dynamics kernel

`cpp/axle_dynamics/axle_kernel.cpp` builds a shared library into
`packages/suspension_multibody/src/suspension_multibody/native/` and is reached
only through `suspension_multibody.axle_dynamics`. Build it with
`packages/suspension_multibody/scripts/build_axle_native.py`. Importing the
package without the library succeeds; running a transient case raises
`NativeKernelUnavailableError`. Design, results, and known limitations live in
`packages/suspension_multibody/docs/axle_dynamics_*.md`.

## Dependency direction

```text
suspension_kinematics --> suspension_contracts <-- suspension_multibody
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
