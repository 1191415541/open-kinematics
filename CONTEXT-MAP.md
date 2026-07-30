# Workspace Context Map

`open-kinematics` is a virtual uv workspace. Each directory under `packages/`
is independently versioned, tested, built, and released.
The root `uv.lock` is the workspace's only tracked dependency lockfile.

## Product responsibilities

- `suspension_contracts`: solver-independent, versioned Geometry Contract V1.
- `suspension_kinematics`: daily suspension geometry design and optimization.
- `suspension_multibody`: high-fidelity quasi-static K&C, load analysis, and
  optional Adams validation.

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
