# Progress Log

## Session Start

- **Date**: 2026-08-05
- **Task name**: `20260805-optimize-full-mbd-speed`
- **Task dir**: `.codex-tasks/20260805-optimize-full-mbd-speed/`
- **Spec**: See `SPEC.md`
- **Plan**: See `TODO.csv` (9 milestones)
- **Environment**: Python 3.12 / NumPy-SciPy / pytest / uv

## Context Recovery Block

- **Current milestone**: #9 - 执行 Adams 对比和全量门禁
- **Current status**: PAUSED after FAILED (retry 4)
- **Last completed**: #8 - 实现鲁棒静态平衡与积分恢复
- **Key context**: C-mode initialization now applies the existing bushing preload fit. The 0.2 s probe completes with maximum position residual `4.46e-7 mm`, maximum physical global linear acceleration `55.43 mm/s^2`, and zero velocity-recovery events.
- **Known issues**: The 2 ms/one-corrector 5 s candidate failed at `t=0.62375 s` after `119.83 s`; it is both unstable and already slower than the `69.063 s` Adams reference. A 5 ms/one-corrector run with `rho_inf=0.2` failed earlier at `t=0.110625 s`.
- **Next action**: Resume only after deciding whether to change the projection/dynamics algorithm; no final speed claim is made.

## Decisions

- Performance comparisons will keep the current physics, solver tolerances, and residual gates fixed. A speedup obtained by relaxing `projection_failure_tolerance` or disabling diagnostics is invalid.
- Optimization order is topology/Jacobian reuse, force/tire allocation reduction, and only then internal-step scheduling; this keeps numerical changes auditable.

## Milestone 1: 建立同工况速度基线

- **Status**: DONE
- **Started**: 10:20
- **Completed**: 10:52
- **What was done**:
  - Added raw/benchmark_baseline.py to measure model build/solve wall time and residuals.
  - Recorded a real Adams/Car step_steer run: 501 samples, 5.0 s, 69.063 s wall time.
  - Recorded a full-MBD 2 ms probe: 3 samples, about 6.367 s solve time, 0.029675 mm maximum position residual, 5.68e-12 velocity residual.
- **Key decisions**:
  - Decision: Do not derive a speedup from unequal simulation durations.
  - Reasoning: Adams/Car's driver run is 5 s while the expensive full-MBD probe is intentionally 2 ms; direct division would be misleading.
- **Problems encountered**:
  - Problem: Direct Adams Solver short-run command cannot initialize the Smartdriver without the complete Adams/Car execution path.
  - Resolution: Keep the full Adams/Car timing as a reference record and mark the short probe as non-comparable until a same-duration harness is available.
- **Validation**: uv run --package suspension-multibody python .codex-tasks/20260805-optimize-full-mbd-speed/raw/benchmark_baseline.py -> exit 0
- **Files changed**:
  - .codex-tasks/20260805-optimize-full-mbd-speed/raw/benchmark_baseline.py - reproducible timing and residual report.
- **Next step**: Milestone 2 - 定位求解热点

## Milestone 2: 定位求解热点

- **Status**: DONE
- **Started**: 10:53
- **Completed**: 11:14
- **What was done**:
  - Added raw/profile_full_vehicle.py and generated a cProfile/pstats artifact from the real Adams step-steer input.
  - Confirmed the post-cache profile is 1.875 s for the 2 ms probe; the largest remaining groups are dynamic force evaluation, constraint Jacobian/SVD, and implicit trial steps.
- **Validation**: uv run --package suspension-multibody python raw/profile_full_vehicle.py -> exit 0; position residual 0.0296749 mm and velocity residual 5.68e-12.
- **Next step**: Milestone 3 - optimize constraint system assembly.

## Milestone 3: 优化约束系统组装

- **Status**: IN_PROGRESS
- **Started**: 11:15
- **What was done**:
  - Cached per-run body mass properties and scaled block mass matrices in ConstrainedDynamicIntegrator.
  - Avoided repeated mass validation/eigenvalue checks and repeated block-diagonal assembly inside every KKT evaluation.
- **Validation**: targeted core/dynamics/vehicle tests 31 passed; Ruff passed. A second timing run is required before marking the milestone complete.

## Milestone 3: 优化约束系统组装

- **Status**: DONE
- **Started**: 11:15
- **Completed**: 11:34
- **What was done**:
  - Cached per-run body mass properties, scaled block mass matrices, and acceleration scaling.
  - Reused local-Jacobian row counts instead of recomputing constraint residuals.
  - Added a first-call rank decision: the measured full-vehicle 79-row constraint matrix is full rank, so later KKT solves skip the per-call SVD; rank-deficient systems retain the existing orthonormal independent-row path.
- **Validation**: targeted core/dynamics/vehicle tests 31 passed; Ruff passed; short probe solve wall time 0.796 s, max position residual 0.0296749 mm and max velocity residual 6.37e-12.
- **Next step**: Milestone 4 - optimize force-element and tire evaluation.

## Milestone 4: 优化力元和轮胎评估

- **Status**: DONE
- **Started**: 11:35
- **Completed**: 11:58
- **What was done**:
  - Added direct dynamic evaluators for bushings, linear springs, and bump stops to avoid unused static tangent construction.
  - Cached tire model instances in ContactTireElement.
  - Added a scalar 3D cross-product kernel for high-frequency wrench and contact operations.
  - Avoided public velocity-copy calls in the internal integrator, contact, and actuator paths.
- **Validation**: targeted core/dynamics/vehicle tests 34 passed; Ruff passed; three strict probe repeats had median solve wall time 0.507 s, min 0.504 s, max position residual 0.0296749 mm and max velocity residual 7.22e-12.
- **Next step**: Milestone 5 - optimize stable internal stepping.

## Milestone 5: 优化内部积分步进

- **Status**: DONE
- **Started**: 11:59
- **Completed**: 12:17
- **What was done**:
  - Added raw/sweep_solver_settings.py and recorded a 20 ms real Adams step-steer sweep.
  - Verified that increasing the internal step above 0.25 ms is not safe for this stiff steering case: 0.5 ms and larger settings either fail the position-manifold gate or produce non-physical acceleration/load growth.
  - Verified that reducing corrector iterations improves short-probe wall time but changes long-window channels non-monotonically; strict default remains unchanged.
  - Added explicit DynamicSolverSettings.reuse_constraint_linearization, default false; the opt-in path reuses within-step constraint linearization while retaining per-candidate force evaluation.
- **Validation**: strict sweep output is raw/internal_step_sweep.json; 0.25 ms settings remain convergent with max position residual 0.0676518 mm over 20 ms. The opt-in reuse sweep remains convergent at 0.25 ms with max position residual 0.0677583 mm; larger steps still fail or diverge.
- **Next step**: Milestone 6 - add performance regression gates.

## Milestone 6: 加入性能回归门禁

- **Status**: DONE
- **Started**: 12:18
- **Completed**: 13:10
- **What was done**:
  - Extended raw/benchmark_baseline.py with repeat count, sample wall times, median/minimum timing, and residual maxima across repeats.
  - Added tests/performance/test_full_vehicle_runtime.py for a repository-local three-sample full-vehicle runtime/residual gate.
  - Replaced the dense KKT solve with a block-mass Schur complement path and added trusted trial-state construction for the constrained integrator.
  - Parameterized the benchmark script for end time, output/internal step, corrector count, and optional constraint-linearization reuse.
- **Validation**: performance gate 3 passed; full multibody test suite 208 passed; Ruff and ty passed; five strict short-probe repeats had median wall time 0.421 s, maximum position residual 0.0296749 mm, and maximum velocity residual 7.79e-12.
- **Known limitation**: the measured Adams record is a 5 s run while the strict MBD probe is 2 ms; the report keeps timing non-comparable until the 5 s MBD run is completed.

## Milestone 7 (superseded): 执行 Adams 对比验收

- **Status**: PAUSED
- **Started**: 13:11
- **What was done**:
  - Added benchmark arguments so the exact Adams duration/output step can be selected without editing the harness.
  - Confirmed internal steps above 0.25 ms fail the strict position-manifold or physical-result gates for the real step-steer topology.
  - Started the same-duration run with `end_time=5 s` and `output_step=0.01 s`; the structured report records the first failure at `t=0.116375 s`, after 23.262 s wall time on the latest repeat.
- **Validation**: superseded while static-balance diagnostics are refreshed; no Adams speedup claim is made.

## Scope Change: 稳定性优先

- **Date**: 2026-08-05
- **Reason**: The requested Adams-speed target cannot be measured while the full-MBD run loses the physical contact state and exits the constraint manifold.
- **Constraint**: Preserve the existing topology, force laws, tolerances, and strict `0.25 ms` internal-step default; add only auditable equilibrium initialization, diagnostics, and bounded recovery.

## Milestone 7: 定位长时域失稳根因

- **Status**: DONE
- **Started**: 2026-08-05
- **Completed**: 2026-08-14
- **What was done**:
  - Added candidate-step failure context to `ConstrainedDynamicIntegrator.last_failure`, including first/last discarded trial, per-constraint row ranges, residual maxima, events, accelerations, and multipliers.
  - Updated `raw/diagnose_long_horizon_instability.py` to report initial generalized forces in global/local coordinates and to run the exact integrator instance used by the diagnostic setup.
  - The current worktree completed the 0.2 s step-steer probe; the report preserves the old 5 s failure record as historical evidence rather than claiming a current reproduction.
- **Validation**: `uv run --package suspension-multibody python .codex-tasks/20260805-optimize-full-mbd-speed/raw/diagnose_long_horizon_instability.py --end-time 0.2 --step-size 0.01` -> exit 0; targeted constrained tests 5 passed.
- **Next step**: Milestone 8 - implement robust static balance and bounded integration recovery.

## Milestone 8: 实现鲁棒静态平衡与积分恢复

- **Status**: DONE
- **Started**: 2026-08-14
- **Completed**: 2026-08-14
- **What was done**:
  - Wired `_fit_c_mode_static_preloads` into C-mode static-equilibrium initialization before actuator and dynamic-element construction.
  - Corrected the `+Y` spindle convention: positive forward rolling magnitude initializes a negative right-hand wheel angular velocity, and tire slip uses the same sign.
  - Kept the strict position/velocity tolerances and integration acceptance logic unchanged; no velocity-recovery event occurred in the 0.2 s probe.
- **Validation**: physics+dynamics 34 passed; Adams correlation+performance 6 passed; diagnostic 0.2 s probe completed with `4.46e-7 mm` maximum position residual and `9.31e-12` maximum velocity residual.

### Validation failure 1

- **Command**: `uv run --package suspension-multibody pytest packages/suspension_multibody/tests/physics packages/suspension_multibody/tests/dynamics -q`
- **Result**: 33 passed, 1 failed.
- **Failure**: `test_free_rolling_wheel_has_zero_longitudinal_slip` produced `slip_ratio=-2` because contact kinematics and initial wheel spin used the opposite sign from the right-hand `+Y` spindle convention.
- **Resolution**: Use negative spindle angular velocity for positive forward rolling magnitude and re-run the same gate.

## Milestone 9: 执行 Adams 对比和全量门禁

- **Status**: PAUSED after FAILED (retry 4)
- **Started**: 2026-08-14
- **Validation contract**: Same-duration 5 s benchmark plus dynamics/adams/vehicle/performance pytest, Ruff, ty, and `git diff --check`.

### Validation failure 1

- **Command**: `uv run --package suspension-multibody python .codex-tasks/20260805-optimize-full-mbd-speed/raw/benchmark_baseline.py --end-time 0.2 --step-size 0.01 --internal-step-size 0.005 --max-corrector-iterations 1 --repeat 1`
- **Result**: exited 2 after 25.03 s.
- **Failure**: Position-manifold maintenance failed at `t=0.1640625 s` after reduction to the `7.8125e-05 s` minimum-step region.
- **Resolution**: Reject this candidate and validate a `2 ms` internal step before retrying the final gate.

### Validation failure 2

- **Commands**: `--internal-step-size 0.005 --max-corrector-iterations 2` and `--internal-step-size 0.005 --max-corrector-iterations 3` with `--end-time 0.2`.
- **Result**: the two-corrector run failed at `t=0.1421875 s`; the three-corrector run completed in `39.38 s` but reached `0.03688 mm` maximum position residual.
- **Resolution**: Reject the 5 ms settings and test `2.5 ms`.

### Validation failure 3

- **Command**: `uv run --package suspension-multibody python .codex-tasks/20260805-optimize-full-mbd-speed/raw/benchmark_baseline.py --end-time 5 --step-size 0.01 --internal-step-size 0.002 --max-corrector-iterations 1 --repeat 1`.
- **Result**: exited 2 after `119.83 s`.
- **Failure**: Position-manifold maintenance failed at `t=0.62375 s`, after the trial step reached `6.25e-05 s`; elapsed time already exceeded the `69.063 s` Adams baseline.
- **Resolution**: Keep the milestone failed while inspecting the failure state; no speedup claim is valid.

### Validation failure 4

- **Command**: `uv run --package suspension-multibody python .codex-tasks/20260805-optimize-full-mbd-speed/raw/benchmark_baseline.py --end-time 0.2 --step-size 0.01 --internal-step-size 0.005 --max-corrector-iterations 1 --generalized-alpha-rho-inf 0.2 --repeat 1`.
- **Result**: exited 2 after `36.23 s`.
- **Failure**: Position-manifold maintenance failed at `t=0.110625 s`.
- **Resolution**: Pause the milestone; retain the diagnostic artifacts and do not claim completion.
