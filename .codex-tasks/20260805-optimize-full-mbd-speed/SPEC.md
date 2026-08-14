# Task Specification

## Goals

- Establish a reproducible wall-clock baseline for the full-vehicle multibody solver and the installed Adams vehicle cases.
- Improve full-vehicle multibody time-domain throughput without changing the physical topology, force laws, solver tolerances, or acceptance metrics.
- Keep long-horizon full-vehicle runs on the constraint manifold from the validated initial state, with deterministic failure diagnostics and bounded step recovery.
- Make the optimized solver faster than the Adams reference under the same scenario, duration, output cadence, and machine.
- Preserve the current constraint-quality gate: no increase in maximum position/velocity residuals and no state clipping.

## Non-Goals

- Do not simplify the full suspension, steering, tire, or chassis rigid-body topology.
- Do not replace PAC2002/Fiala tire behavior or Adams nonlinear force curves with lower-fidelity substitutes.
- Do not loosen residual tolerances or hide solver failures by suppressing diagnostics.
- Do not change the four handling and four ride acceptance scenarios in this task.

## Constraints

- Project root: `C:/杂件/open-kinematics`.
- Runtime: Python 3.12, NumPy/SciPy, `uv` package manager.
- Solver units remain kg-mm-N-s for the Adams-compatible vehicle path.
- Benchmark comparisons use identical case inputs, end time, output step, initial conditions, and acceptance tolerances.
- Optimizations must remain deterministic and preserve the existing 93-test gate.

## Environment

- **Project root**: `C:/杂件/open-kinematics`
- **Language/runtime**: Python 3.12
- **Package manager**: uv
- **Test framework**: pytest
- **Build command**: `uv run --package suspension-multibody`
- **Existing test count**: 93 relevant dynamics/Adams/vehicle tests

## Risk Assessment

- [x] External dependency: installed Adams 2024.1 is available locally; no new dependency is planned.
- [x] Breaking changes: shared constraint and force evaluation paths can affect all modes; full regression is mandatory.
- [x] Large file generation: benchmark logs are stored under the ignored task directory.
- [x] Long-running tests: full Adams comparisons are expensive; short deterministic probes are used during iteration.

## Deliverables

- Reproducible full-vehicle and Adams performance benchmark harness with raw timing evidence.
- Targeted optimizations for the dominant solver hotspots, with focused regression tests.
- Updated performance report including runtime, speedup, residuals, event counts, and acceptance status.

## Done-When

- [ ] A fixed benchmark matrix is reproducible on the current machine and records Adams and full-MBD wall time.
- [ ] Full-MBD wall time is lower than Adams wall time for the agreed benchmark matrix.
- [ ] Maximum position/velocity residuals are no worse than the pre-optimization baseline.
- [ ] The fixed 5 s step-steer case completes without tire-unload runaway or position-manifold failure.
- [ ] No independent velocity/acceleration clipping is present in the optimized path.
- [ ] Relevant pytest, Ruff, ty, and `git diff --check` gates pass.

## Final Validation Command

```bash
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/dynamics packages/suspension_multibody/tests/adams packages/suspension_multibody/tests/vehicle -q
uv run --package suspension-multibody ruff check packages/suspension_multibody/src packages/suspension_multibody/tests
uv run --package suspension-multibody ty check packages/suspension_multibody/src
git diff --check
```

## Demo Flow

1. Run the benchmark harness against the fixed Adams handling and ride reference bundles.
2. Compare wall time and speedup together with residual and event metrics.
3. Run the relevant regression gates.
