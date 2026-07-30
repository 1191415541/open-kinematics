# open-kinematics

`open-kinematics` is a uv workspace for independently developed suspension
engineering products.

- `packages/suspension_kinematics`: daily suspension geometry design and
  optimization.
- `packages/suspension_contracts`: versioned, solver-independent geometry
  exchange contracts.
- `packages/suspension_multibody`: quasi-static suspension K&C, load, and Adams
  validation analysis.

Each product has its own package metadata, tests, CLI, and release version.

## Workspace commands

```bash
uv sync --all-packages --all-extras --all-groups
just check
just test
```

`suspension_kinematics` and `suspension_multibody` both depend on
`suspension_contracts`; they do not depend on one another. See
[CONTEXT-MAP.md](CONTEXT-MAP.md) for the ownership and artifact boundaries.
