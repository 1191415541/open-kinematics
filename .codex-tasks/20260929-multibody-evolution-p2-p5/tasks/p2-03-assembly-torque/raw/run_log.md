# p2-03: every command run, with its real exit code

Repository root `E:\杂件\open-kinematics`, 2026-10-01. Nothing below is a plan; each line is
a command that ran and the exit code it returned.

## The four acceptance commands

### 1. modelling and subsystem tests

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/modeling packages/suspension_multibody/tests/subsystems -q -p no:cacheprovider
........................................................................ [ 32%]
........................................................................ [ 64%]
........................................................................ [ 96%]
........                                                                 [100%]
224 passed in 5.15s
exit = 0
```

The two new files alone:

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/subsystems/test_rotational_torque_element.py -q -p no:cacheprovider
12 passed in 1.25s
exit = 0

$ uv run --no-sync pytest packages/suspension_multibody/tests/modeling -q -p no:cacheprovider
60 passed in 1.54s
exit = 0
```

### 2. architecture

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/architecture -q -p no:cacheprovider
........................................................................ [ 48%]
........................................................................ [ 97%]
...                                                                      [100%]
147 passed in 590.05s (0:09:50)
exit = 0
```

### 3. kernel module layering

```
$ uv run --no-sync python packages/suspension_kernel/scripts/check_module_layering.py --strict --final
source edge evidence       : 229 records (tu_include 213, symbol_reference 175, both 159; kind mentions 388)
target modules missing     : 0
legacy modules present     : 0
legacy modules unregistered: 0 []
mutual (reverse) edges     : 0
self-including headers     : 0
cross-aggregate includes   : 0
module cycles (SCC size>1) : 0

OK: layering matches the recorded baseline
exit = 0
```

### 4. dynamic hash sentinel

```
$ uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
artifacts hashed : 26
combined sha256  : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9
acceptance exit  : 1
failed cases     : ['combined_load', 'in_phase_road', 'large_amplitude_high_frequency', 'opposite_phase_road', 'road_pulse', 'road_sine', 'road_step_finite_rise', 'single_wheel_road', 'tire_liftoff_and_recontact']

OK: dynamic output matches the frozen baseline byte-for-byte
exit = 0
```

The required sha256, exactly. See `raw/legacy_elements_parity.md` for what the per-case
labels and the `acceptance exit : 1` mean (pre-existing baseline state, identical to what
p2-01 and p2-02 recorded).

```
$ git status --short -- packages/suspension_multibody/tests/data/
$ echo "exit=$?"
exit = 0        (empty output: no frozen baseline changed)
```

## The other gates the SPEC names

### `just check-fast` — its parts, run individually

`just check-fast` as one command exits **1**, and the reason is `lint`. The parts:

| step | command | result |
|---|---|---|
| lint | `uv run --all-packages ruff check .` | **exit 1 — 17 errors, all in `.codex-tasks/` raw probe scripts of other rows; see below** |
| type-check | `uv run --no-sync ty check .` | exit 0 — `All checks passed!` |
| gate-architecture 1 | `legacy_surface_gate.py --check` | exit 0 — `OK: no unregistered Python boundary violation` |
| gate-architecture 2 | `check_module_layering.py --strict --final` | exit 0 (above) |
| gate-architecture 3 | `check_composable_release.py --skip-isolation` | exit 0 — `OK: 3 release checks passed` |
| test-fast | the fast set | exit 0 — `1130 passed, 1 xfailed in 38.34s` |
| test-other (kernel) | `uv run --package suspension-kernel pytest packages/suspension_kernel/tests` | exit 0 — `41 passed in 14.27s` |
| test-other (contracts) | `uv run --package suspension-contracts pytest packages/suspension_contracts/tests` | exit 0 — `32 passed in 0.11s` |

**The 17 lint errors are not this row's, and are the reason `just check-fast` as a whole is
red.** Every one of them is in an untracked probe script another row left in
`.codex-tasks/`:

```
$ uv run --no-sync ruff check . 2>&1 | grep -E "^\s+-->" | sed 's/.*--> //' | cut -d: -f1 | sort -u
.codex-tasks\20260929-multibody-evolution-p2-p5\tasks\p2-01-freeze\raw\rear_steer_probe.py
.codex-tasks\20260929-multibody-evolution-p2-p5\tasks\p3-01-freeze\probe_axle_rollcenter.py
.codex-tasks\20260929-multibody-evolution-p2-p5\tasks\p3-01-freeze\probe_configs.py
.codex-tasks\20260929-multibody-evolution-p2-p5\tasks\p3-01-freeze\probe_explicit5.py
.codex-tasks\20260929-multibody-evolution-p2-p5\tasks\p3-01-freeze\probe_three.py
.codex-tasks\20260929-multibody-evolution-p2-p5\tasks\p3-01-freeze\probe_trailingarm_vehicle.py
```

Two measurements establish that this row did not cause it:

```
$ uv run --no-sync ruff check --exclude .codex-tasks .
All checks passed!
exit = 0

$ git ls-files -z -- '*.py' | xargs -0 uv run --no-sync ruff check
All checks passed!
exit = 0
```

Both scopes include every file this row touched and both are clean; the only failures are in
files git does not track, which were written at 01:05 and 02:36 while this row's first edit
was after 05:40 (`ls -l --time-style` on `rear_steer_probe.py` and `probe_three.py`). They
belong to p2-01 and p3-01 and are outside this row's write scope; fixing another row's raw
probe would be an unrequested change to a file this row does not own.

This row's own files, checked alone:

```
$ uv run --no-sync ruff check \
    packages/suspension_multibody/src/suspension_multibody/compilation/element_blocks.py \
    packages/suspension_multibody/src/suspension_multibody/compilation/__init__.py \
    packages/suspension_multibody/src/suspension_multibody/subsystems/element_build.py \
    packages/suspension_multibody/src/suspension_multibody/modeling/primitives/elements.py \
    packages/suspension_multibody/src/suspension_multibody/modeling/primitives/__init__.py \
    packages/suspension_multibody/tests/modeling/test_rotational_torque_element.py \
    packages/suspension_multibody/tests/subsystems/test_rotational_torque_element.py
All checks passed!
exit = 0
```

### The numeric gates (this row adds an element into the kernel input, so all three)

```
$ uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
combined sha256  : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9
OK: dynamic output matches the frozen baseline byte-for-byte
exit = 0

$ uv run --no-sync python packages/suspension_multibody/scripts/case_parity_check.py
  handling           PASS      4 open-loop shapes match an independent expansion (0.0e+00); closed-loop refused
  ride_four_post     PASS      expansion matches an independently sampled excitation (0.0e+00)
  ride_random_road   PASS      expansion matches an independently expanded profile (0.0e+00)
  comparison         N/A       a per-target gate, not a solve: the kernel never reads a reference
OK: 8 families accepted
exit = 0

$ uv run --no-sync python packages/suspension_multibody/scripts/kc_perf_gate.py --check
  k-100: 0.8065 s vs baseline 1.0230 s (x0.788)
  c-66: 1.1446 s vs baseline 1.4904 s (x0.768)
OK: benchmarks are within the recorded budget
exit = 0
```

Run without `--check` for `case_parity_check.py`, per the correction p2-01 registered
(`tasks/p2-01-freeze/raw/gates_baseline.md:33-36`: the flag does not exist and exits 2).
`kc_parity_check.py` is not run: without `--actual-dir` it compares the frozen snapshot
against itself, which would be no evidence.

### The remaining architecture gate script

```
$ uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
mode      : migration
findings  : 0

OK: no unregistered Python boundary violation
exit = 0

$ uv run --no-sync python packages/suspension_multibody/scripts/check_composable_release.py --skip-isolation
[PASS] migration list: 0 retained legacy imports (0 production, 0 test) and 4 retired packages gone
[PASS] documentation roots: 16 documented modules present, 4 retired absent
[PASS] documentation examples: 3 example(s) executed: E-1, E-2, E-3

OK: 3 release checks passed
exit = 0
```

The new `compilation/element_blocks.py` does not appear in the migration registry and does
not need to: the registry lists *legacy* imports, and the file imports nothing retired.

## Scope self-checks

```
$ git diff --numstat -- \
    .../preparation/vehicle_dynamic.py .../cases/vehicle_dynamic.py .../cases/vehicle_kc.py \
    .../subsystems/brake.py .../subsystems/drive.py .../templates/roles.py .../templates/builtin.py \
    .../subsystems/types.py .../subsystems/assembler.py
(no output)
exit = 0
```

Nine forbidden files, zero changes. `packages/suspension_kernel/**` likewise has no change
from this row: `git status --short` there lists only p2-02's own modifications, all present
before this row started.

```
$ grep -n "mark.skip\|mark.xfail\|pytest.skip\|xfail" <the two new test files>
(no output; grep exit 1)
```

No new skip and no new xfail.

## The `git diff` of this row, in full

```
$ git diff --numstat -- packages/suspension_multibody/src/ packages/suspension_multibody/tests/
20	0	packages/suspension_multibody/src/suspension_multibody/compilation/__init__.py
4	0	packages/suspension_multibody/src/suspension_multibody/modeling/primitives/__init__.py
140	0	packages/suspension_multibody/src/suspension_multibody/modeling/primitives/elements.py
21	0	packages/suspension_multibody/src/suspension_multibody/subsystems/element_build.py

$ git status --short -- packages/suspension_multibody/ | grep '^??'
?? packages/suspension_multibody/src/suspension_multibody/compilation/element_blocks.py
?? packages/suspension_multibody/tests/modeling/test_rotational_torque_element.py
?? packages/suspension_multibody/tests/subsystems/test_rotational_torque_element.py
```

Four modified files (185 added lines, **0 deleted**), three new files. The other modified
entries in the wider `git status` (`api.py`, `__init__.py`, `vehicle/roll_centers.py`,
`kernel/native.py`, the `tests/axle_dynamics/*` files, `tests/architecture/test_element_block_layout.py`,
plus the untracked `docs/`, `vehicle/screw_kinematics.py`, `tests/api/`, `tests/vehicle/`)
belong to the other agents working in this tree and were not touched here.

`git diff --check` for the four modified files: clean (no whitespace errors reported).
