# p2-01 (5): starting state — skip/xfail counts, existing failures, no-writes proof

Read-only row. This is the reference every later row's "no new skip/xfail" claim
is measured against.

## Skip / xfail starting counts

| suite | command | result |
|---|---|---|
| fast set | `pytest packages/suspension_multibody/tests --ignore=.../adams --ignore=.../architecture --ignore=.../cases -q` | **1078 passed, 1 xfailed** (1 xfail, 0 skipped) |
| `tests/adams` | `pytest packages/suspension_multibody/tests/adams --collect-only -q` | 207 collected (the environment skips are runtime, not collection) |

The single xfail in the fast set is pre-existing; this row adds none. The
contract's hard gate for this Epic is **1 skipped / 1 xfailed** on the full suite
(the 1 skipped lives in `tests/adams`' environment-gated cases and is registered).

## Numeric-gate starting state

| gate | result |
|---|---|
| `dynamic_hash_sentinel.py --check` | exit 0; 26 artifacts; combined sha256 `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9` |
| `case_parity_check.py` (no args) | exit 0; `OK: 8 families accepted` |
| `kc_perf_gate.py --check` | exit 0; within budget |

This is the same combined hash the stage-one Epic closed on, so the two Epics
agree on their zero-regression reference.

## Existing failures

**None.** The fast set is green (1078 passed), the three architecture gates are
green, and all three numeric gates are green. There is therefore no pre-existing
failure to list; any failure appearing later in this Epic is introduced by it.

## No-writes proof for this row

```
$ git status --short
?? packages/suspension_multibody/docs/multibody_architecture_evolution.md
```

The only untracked file is the upstream route map itself (not this row's product).
No `packages/**` file was modified: this row's write range is `raw/` plus the
session scratch directory. Confirmed by `git status --short` showing no ` M ` or
` M` under `packages/`.

## Baseline files are untouched

`git status --short -- packages/suspension_multibody/tests/data/` prints nothing,
so `tests/data/kc_baseline/**`, `dynamic_hash_baseline.json`,
`vehicle_dynamics_baseline/sha256.json` and `kc_perf_baseline_native.json` are all
as they were. Nothing in this row wrote to them.

## What later rows inherit from this row

- p2-02: the ABI triple is **16 / 31 / 1** (`raw/anchors_F1_F5.md`), and every
  kernel rebuild must be followed by `scripts/build_axle_native.py` or the release
  gate fails on a stale mirror (`raw/gates_baseline.md`).
- p2-05: the torque-path reference is `raw/torque_path_snapshot.json`.
- p2-06: both refusal layers and the fact that the authoring one is a silent
  overwrite (`raw/rear_steer_refusal.md`).
- All rows: the numeric-gate calls (`case_parity_check.py` takes **no** `--check`).
