# p2-03 (d): the existing element types' products are unchanged

All commands run from the repository root, 2026-10-01. Exit codes and outputs are real.

## 1. The byte-for-byte gate

```
$ uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
  static_equilibrium: PASSED
  road_step_finite_rise: FAILED
  road_pulse: FAILED
  road_sine: FAILED
  single_wheel_road: FAILED
  in_phase_road: FAILED
  opposite_phase_road: FAILED
  braking: PASSED
  driving: PASSED
  lateral_or_steering: PASSED
  combined_load: FAILED
  tire_liftoff_and_recontact: FAILED
  large_amplitude_high_frequency: FAILED
artifacts hashed : 26
combined sha256  : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9
acceptance exit  : 1
failed cases     : ['combined_load', 'in_phase_road', 'large_amplitude_high_frequency', 'opposite_phase_road', 'road_pulse', 'road_sine', 'road_step_finite_rise', 'single_wheel_road', 'tire_liftoff_and_recontact']

OK: dynamic output matches the frozen baseline byte-for-byte
exit=0
```

**`combined sha256 = fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9` — the
required value, character for character**, identical to the baseline p2-01 recorded
(`tasks/p2-01-freeze/raw/gates_baseline.md:27`). The gate exits **0**.

### About the per-case `FAILED` lines, and the `acceptance exit : 1`

Both are the **pre-existing state of the gate, not a regression from this row**. The gate's own
verdict — `OK: dynamic output matches the frozen baseline byte-for-byte` — is the judgement:
the 26 artifacts it hashed are byte-identical to the recorded ones, which is why the
`combined sha256` matches, and the gate exits 0.

The `acceptance exit : 1` and the nine case names were already there before this row, and
p2-02 registered the same pair independently
(`tasks/p2-02-kernel-torque/raw/kernel_torque_evidence.md:119-135`: same sha256, same
`acceptance exit : 1`, `failed cases : [... 9 个 ...]`, and its note that the column is the
acceptance suite's pre-existing state rather than something that row introduced). p2-01's own
record (`tasks/p2-01-freeze/raw/gates_baseline.md:26-27`) captured the sha256 and the OK line
but did not print the per-case block. Recorded verbatim here so the reading is not mistaken
for a clean 13/13, and so the two lineages can be compared: the numbers are identical.

## 2. No frozen baseline was re-recorded

```
$ git status --short -- packages/suspension_multibody/tests/data/
$ echo "exit=$?"
exit=0
```

Empty output: **no file under `tests/data/` changed**. `dynamic_hash_baseline.json` and
`kc_baseline/**` are untouched. If the sha256 above had differed, this row would have been
reverted rather than the baseline re-recorded (`EPIC.md:228`, D5).

## 3. Existing element products are unchanged by construction

The parity claim for the six existing families (`spring`, `damper`, `bump_stop`,
`anti_roll_bar`, `tire`, `bushing`; declared at `subsystems/types.py:451`) rests on the diff,
not on a reading:

- The dispatch chain in `subsystems/element_build.py` is **append-only**. The full diff of
  that file is three hunks, all additions (`raw/element_declaration.md` §2 quotes it whole).
  The six existing branches are context lines, and `raise ValueError(...)` moved from `:72`
  to `:76` — its text and behaviour are unchanged, and a test pins it
  (`test_an_unknown_kind_is_still_refused`, `tests/subsystems/test_rotational_torque_element.py:322`).
- `modeling/primitives/elements.py` gained a new class and one module-level constant. No
  existing class body changed (measured: `git diff --numstat` = `140 0` for that file, and
  the only *modified* region is the import list at `:27-33`, where one name was added).
- `element_rows` (`subsystems/element_build.py:212`) is untouched, so the **recorded
  element order** — which the contract document mirrors — is unchanged for every existing
  assembly.
- No caller of the new code exists in the product yet: `build_element` is reached only
  through `subsystems/runtime.py:306`, and nothing in the product emits
  `kind="rotational_torque"` (p2-04's `brake.py`/`drive.py` are the intended producers). So
  every existing assembly's element list is exactly what it was.

## 4. The p2-01 path snapshot

Snapshot: `tasks/p2-01-freeze/raw/torque_path_snapshot.json` (comparison basis named by
`SPEC.md:66`). It records the *torque path* — `_build_wheel_torque_signals`'s input/output
shapes and sample counts, and the `wheel_torque` / `brake_torque` contract table headers and
row counts.

**This row produces no entry in either.** Measured: the two tables are emitted by
`cases/vehicle_dynamic.py:579/582` from `prepared.wheel_torque` / `prepared.brake_torque`,
which come from `preparation/vehicle_dynamic.py` — a file this row does not touch (and is
forbidden to: its torque segment belongs to p2-05). `cases/vehicle_dynamic.py` is likewise
untouched.

```
$ git diff --numstat -- packages/suspension_multibody/src/suspension_multibody/preparation/vehicle_dynamic.py \
                        packages/suspension_multibody/src/suspension_multibody/cases/vehicle_dynamic.py \
                        packages/suspension_multibody/src/suspension_multibody/cases/vehicle_kc.py
(no output — all three unchanged)
```

So the torque-path comparison is **trivially equal — the path was not touched — and that is
the registered conclusion**, not an omission. The two facts that make it a real statement
rather than a restatement of "I did not edit those files":

1. the `dynamic_hash_sentinel` sha256 above covers the dynamic outputs those tables feed, and
   it is unchanged;
2. the new element is not reachable from the torque path at all: nothing in
   `preparation/` or `cases/` constructs a `rotational_torque` row (§3, last bullet).

## 5. Every other acceptance gate, for the record

See `raw/run_log.md` for the full table and verbatim output. Summary: `ruff` 0, `ty` 0,
`tests/modeling` + `tests/subsystems` 224 passed / exit 0, `tests/architecture` 147 passed /
exit 0, `check_module_layering.py --strict --final` OK / exit 0, `legacy_surface_gate.py
--check` OK / exit 0, `check_composable_release.py --skip-isolation` 3 checks passed /
exit 0, `case_parity_check.py` 8 families accepted / exit 0, `kc_perf_gate.py --check`
within budget / exit 0. No new skip or xfail: the two new test files contain no
`pytest.mark.skip`, `pytest.mark.xfail` or `pytest.skip` (measured).
