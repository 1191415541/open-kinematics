#!/usr/bin/env python
"""
K/C parity gate: the frozen reference states vs a candidate implementation.

The snapshot under ``tests/data/kc_baseline`` was produced by the Python
quasi-static solver and is **frozen**.  It is the oracle the native kernel is
judged against, and the order matters: the Python output was captured *before*
the Python implementation was retired, so the reference outlives the thing it
was captured from.  ``--record`` is gone with that solver -- regenerating the
snapshot would mean re-deriving the oracle from the implementation under test.

* K  -- a 3x3 wheel-travel x rack grid (9 states);
* C  -- the six standard load paths x 11 levels at the neutral K point
        (66 states), single-side mode.

``--check --actual-dir DIR`` compares a candidate implementation's JSON files
against the snapshot using the tolerances the strict Adams gates already use, so
the same numbers govern both comparisons.  ``scripts/kc_native_probe.py`` and
``scripts/kc_native_c_probe.py`` write those candidates.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BASELINE = PACKAGE_ROOT / "tests" / "data" / "kc_baseline"


def _tolerance(field: str, reference: float) -> float:
    magnitude = abs(reference)
    if field.endswith("_mm"):
        return 0.1 + 0.002 * magnitude
    if field.endswith("_deg"):
        return 0.02 + 0.005 * magnitude
    return 1e-6 + 1e-4 * magnitude


def _compare_scalar(
    label: str, reference: float, actual: float, tolerance: float
) -> str | None:
    error = abs(actual - reference)
    if error > tolerance:
        return (
            f"{label}: reference={reference!r} actual={actual!r} "
            f"error={error!r} tolerance={tolerance!r}"
        )
    return None


def _compare_states(
    name: str,
    expected: list[dict[str, object]],
    actual: list[dict[str, object]],
) -> list[str]:
    failures: list[str] = []
    want = {str(state["case_id"]): state for state in expected}
    got = {str(state["case_id"]): state for state in actual}
    for case_id in sorted(set(want) - set(got)):
        failures.append(f"{name}: missing case {case_id}")
    for case_id in sorted(set(got) - set(want)):
        failures.append(f"{name}: unexpected case {case_id}")
    for case_id in sorted(set(want) & set(got)):
        for field, reference in want[case_id].items():
            if field in ("case_id", "path", "side_mode", "metrics"):
                continue
            if isinstance(reference, list):
                values = got[case_id].get(field)
                if not isinstance(values, list) or len(values) != len(reference):
                    failures.append(f"{name}/{case_id}: {field} shape mismatch")
                    continue
                for index, (ref, act) in enumerate(zip(reference, values)):
                    problem = _compare_scalar(
                        f"{name}/{case_id}/{field}[{index}]",
                        float(ref),
                        float(act),
                        _tolerance(field, float(ref)),
                    )
                    if problem:
                        failures.append(problem)
            elif isinstance(reference, float):
                actual_value = got[case_id].get(field)
                if not isinstance(actual_value, (int, float)):
                    failures.append(f"{name}/{case_id}: {field} missing")
                    continue
                problem = _compare_scalar(
                    f"{name}/{case_id}/{field}",
                    reference,
                    float(actual_value),
                    _tolerance(field, reference),
                )
                if problem:
                    failures.append(problem)
    return failures


def main() -> int:
    """Judge a candidate against the frozen snapshot."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE)
    parser.add_argument(
        "--actual-dir",
        type=Path,
        help="directory holding the candidate k_states.json/c_states.json",
    )
    args = parser.parse_args()

    actual_dir = args.actual_dir or args.baseline
    if not (args.baseline / "k_states.json").is_file():
        raise SystemExit(f"no snapshot at {args.baseline}")
    failures: list[str] = []
    for name in ("k_states.json", "c_states.json"):
        expected = json.loads((args.baseline / name).read_text(encoding="utf-8"))
        candidate_path = actual_dir / name
        if not candidate_path.is_file():
            failures.append(f"missing candidate file {candidate_path}")
            continue
        actual = json.loads(candidate_path.read_text(encoding="utf-8"))
        failures.extend(_compare_states(name, expected, actual))
    if failures:
        print(f"FAIL: {len(failures)} parity violations")
        for failure in failures[:20]:
            print(f"  {failure}")
        if len(failures) > 20:
            print(f"  ... {len(failures) - 20} more")
        return 1
    print("OK: candidate matches the frozen K/C snapshot within tolerance")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
