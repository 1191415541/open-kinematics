# p2-01 (c): gates and numeric gates, starting values

All commands run from the repository root, 2026-09-29. Exit codes are the real ones.

## Three architecture gates (seconds each)

- `uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check`
  - exit: 0
  - output: `OK: no unregistered Python boundary violation`
- `uv run --no-sync python packages/suspension_kernel/scripts/check_module_layering.py --strict --final`
  - exit: 0
  - output: `OK: layering matches the recorded baseline`
- `uv run --no-sync python packages/suspension_multibody/scripts/check_composable_release.py --skip-isolation`
  - exit: 1
  - output: `FAIL: 1 of 3 release checks failed`
- `uv run --no-sync python packages/suspension_multibody/scripts/check_composable_release.py --skip-isolation`
  - exit: 0 (after `scripts/build_axle_native.py`)
  - output: `OK: 3 release checks passed`
  - note: the first run exited **1** because the kernel rebuild (`build_suspension_kernel.py`) had left the axle mirror stale: `NativeKernelUnavailableError: the native kernel mirror ... is older and different ...; run just build-axle-native`. `scripts/build_axle_native.py` (exit 0) refreshed it. **Every kernel rebuild in this Epic must be followed by `build_axle_native.py`** — recorded here because p2-02 rebuilds the kernel.

## Numeric gates

## Numeric gates

- `uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check`
  - exit: 0
  - output: `artifacts hashed : 26`, `combined sha256 : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`, `OK: dynamic output matches the frozen baseline byte-for-byte`
- `uv run --no-sync python packages/suspension_multibody/scripts/case_parity_check.py` (no arguments)
  - exit: 0
  - output: `OK: 8 families accepted`
  - **`--check` does not exist**: `case_parity_check.py --check` exits 2 with
    `error: unrecognized arguments: --check`. The accepted flags are `--family`,
    `--allow-partial`, `--record`. Any acceptance command in this Epic that writes
    `case_parity_check.py --check` is wrong and must be run without it.
- `uv run --no-sync python packages/suspension_multibody/scripts/kc_perf_gate.py --check`
  - exit: 0
  - output: `OK: benchmarks are within the recorded budget`

## `kc_parity_check.py` scope

Not run as a gate in this row. When it is used for K/C equivalence, it must be run
as `kc_native_probe.py && kc_native_c_probe.py && kc_parity_check.py --check
--actual-dir artifacts/kc-native-probe`. Without `--actual-dir` it compares the
frozen snapshot against itself and always passes, which is not evidence.

## Kernel rebuild requires the axle mirror to be refreshed

The three architecture gates were run **after** `build_suspension_kernel.py`
(exit 0) had rebuilt the kernel. `check_composable_release.py --skip-isolation`
then exited **1** with:

```
NativeKernelUnavailableError: the native kernel mirror at
packages/suspension_multibody/src/suspension_multibody/native/suspension_kernel.dll
is older and different from
packages/suspension_kernel/src/suspension_kernel/native/suspension_kernel.dll;
run `just build-axle-native` ...
```

`scripts/build_axle_native.py` (exit 0) refreshed the mirror and the gate then
exited 0. **Every kernel rebuild in this Epic must be followed by
`build_axle_native.py`** — recorded here because p2-02 and possibly p5-04 rebuild
the kernel.
