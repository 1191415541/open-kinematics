# Independent Suspension Products

The repository is a virtual workspace containing independently versioned
Suspension Kinematics and Suspension Multibody products. They exchange only
through the dependency-light, versioned `suspension_contracts` package so the
fast design solver remains independent from the higher-fidelity verification,
load, dynamic, flexible-body, and Adams analysis solver.

## Considered Options

- A single package with shared solver internals would reduce file boundaries but
  would make independent MBD validation less credible and couple optional Adams
  and future flexible-body dependencies to daily geometry work.
- Two products without a common contract would preserve independence but make
  geometry exchange ad hoc and unstable.

## Consequences

Each product owns its algorithms, package metadata, tests, CLI, and release
cadence. `suspension_contracts` owns only stable interchange data and cannot
depend on either solver. Breaking product renames are permitted for this
migration; no compatibility aliases are retained.
