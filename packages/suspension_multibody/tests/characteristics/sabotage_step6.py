"""
Sabotage harness for step 6: every claim must be falsifiable.

A green test suite proves nothing about whether it *tests* anything.  Each case below
introduces one realistic defect into the oracle's source, re-runs the step-6 tests,
and requires a failure.  A defect that survives means the corresponding claim is
unverified, and is reported as MISSED rather than quietly tolerated.

The module is restored from an in-memory copy after every case, and the restore is
verified byte-for-byte, so a failure mid-run cannot leave the tree modified.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(r"E:\杂件\open-kinematics")
MODULE = ROOT / "packages/suspension_multibody/tests/characteristics/oracle_equivalence.py"
TESTS = "packages/suspension_multibody/tests/characteristics/test_route_equivalence.py"

# Each sabotage: (name, exact text to replace, replacement).
SABOTAGES: list[tuple[str, str, str]] = [
    (
        "transpose route solves A instead of A^T",
        "z = np.linalg.solve(matrix.T, measurement.T)",
        "z = np.linalg.solve(matrix, measurement.T)",
    ),
    (
        "unit-basis route transposes the tangent",
        "columns.append(measurement @ np.linalg.solve(matrix, load_map @ unit))",
        "columns.append(measurement @ np.linalg.solve(matrix.T, load_map @ unit))",
    ),
    (
        "unit-basis route drops the measurement map",
        "columns.append(measurement @ np.linalg.solve(matrix, load_map @ unit))",
        "columns.append(np.linalg.solve(matrix, load_map @ unit))",
    ),
    (
        # Two mutations were tried here and REMOVED after measurement showed both are
        # semantically equivalent rather than surviving defects.  Recorded so a later
        # reader does not "restore" them as coverage:
        #
        #   1. `bumped[mode, :] += step` -> `bumped[mode, 0] += step`.  The code reads
        #      only `quotient[:, 0]`, and bumping the whole row changes column 0 by
        #      exactly what bumping column 0 alone changes -- bit-identical across four
        #      probes including a badly scaled one.
        #   2. dividing by the nominal `applied` step instead of the `realised` delta.
        #      With the step scaled to the loads it is representable, so the two agree
        #      to the last bit or two: worst gap 3.8e-11 across loads spanning
        #      1e-8..1e12.  The `realised` divisor is honest defensive accounting, not
        #      a load-bearing fix; the A1 cases below are what guard the scaling.
        #
        # Asserting either would record a false negative.
        "perturbation route scales the quotient wrongly",
        "result[:, mode] = quotient[:, 0]",
        "result[:, mode] = quotient[:, 0] * step",
    ),
    (
        "perturbation route omits the direct term",
        "result[:, mode] = quotient[:, 0]",
        "result[:, mode] = quotient[:, 0] - direct[:, mode]",
    ),
    (
        "operator drops D entirely",
        "return measurement @ np.linalg.solve(matrix, load_map) + direct",
        "return measurement @ np.linalg.solve(matrix, load_map)",
    ),
    (
        "D row-count check removed (the silent broadcast defect)",
        """    if measurement.shape[0] != direct.shape[0]:
        raise CharacteristicsError(
            f"measurement_map has {measurement.shape[0]} rows but direct_term has "
            f"{direct.shape[0]}; both map into measurement space and must agree"
        )""",
        "    pass",
    ),
    (
        "empty-measurement refusal removed (vacuous agreement)",
        """    if measurement.shape[0] == 0:
        raise CharacteristicsError(
            "the measurement set is empty, so every route trivially agrees and the "
            "comparison proves nothing"
        )""",
        "    pass",
    ),
    (
        "mode-count check removed",
        """    if columns.shape[1] < load_map.shape[1]:
        raise CharacteristicsError(
            f"recovering the full response operator needs at least "
            f"{load_map.shape[1]} load columns (one per mode), got {columns.shape[1]}"
        )""",
        "    pass",
    ),
    (
        "non-finite input check removed",
        """    if not np.all(np.isfinite(array)):
        raise CharacteristicsError(f"{name} contains a non-finite entry")""",
        "    pass",
    ),
    (
        "zero-coupling mode not refused by unit-basis route",
        """        if not np.any(load_map[:, mode]):
            # Same unidentifiability as the perturbation route, by the same argument:
            # a unit load in an uncoupled direction produces no signal, and dividing
            # the untouched residual by the unit load yields a number unrelated to the
            # requested mode.
            raise CharacteristicsError(
                f"load mode {mode} has no coupling into the system, so its response "
                "is not identifiable from that load direction"
            )""",
        "        pass",
    ),
    (
        "step validation removed",
        """    if not np.isfinite(step) or step <= 0.0:
        raise CharacteristicsError("the perturbation step must be positive and finite")""",
        "    pass",
    ),
    (
        "zero-coupling mode not refused by perturbation route",
        """        if not np.any(load_map[:, mode]):
            # A mode whose `B_w` column is entirely zero carries no information about
            # that load direction: the difference quotient does not vanish, it returns
            # whatever the *untouched* part of the response happens to be, divided by
            # `h`.  So the failure mode is a large garbage number rather than a
            # plausible zero -- and it looks convergent as `h` shrinks.  Refused here
            # so the caller learns the mode is unidentifiable instead of reading a
            # number that has nothing to do with it.
            raise CharacteristicsError(
                f"load mode {mode} has no coupling into the system, so its response "
                "is not identifiable from that load direction"
            )""",
        "        pass",
    ),
    (
        "transpose route drops the direct term",
        "c_transpose = z.T @ load_map + direct",
        "c_transpose = z.T @ load_map",
    ),
    (
        "perturbation route ignores the load map's mode coupling",
        "quotient = (\n            _direct_answer(matrix, load_map, direct, bumped, measurement) - base\n        ) / realised",
        "quotient = (\n            _direct_answer(matrix, load_map, direct, columns, measurement) - base\n        ) / realised",
    ),
    (
        # The A1 defect itself, restored as a sabotage so it can never come back
        # unnoticed: an absolute step is absorbed once the loads dwarf it.
        "perturbation step is absolute rather than scaled (the A1 defect)",
        "        return np.full_like(load_row, max(step, scale * 1e-6))",
        "        return np.full_like(load_row, step)",
    ),
    (
        "perturbation step scaled by the mode row instead of the whole problem",
        "    scale = float(np.max(np.abs(all_loads))) if all_loads.size else 0.0",
        "    scale = float(np.max(np.abs(load_row))) if load_row.size else 0.0",
    ),
    (
        "unit-basis route drops the direct term",
        "return np.hstack(columns) + direct",
        "return np.hstack(columns)",
    ),
    (
        "loads row-count check removed",
        """    if columns.shape[0] != load_map.shape[1]:
        raise CharacteristicsError(
            f"loads has {columns.shape[0]} rows but the load map expects "
            f"{load_map.shape[1]}"
        )""",
        "    pass",
    ),
]


def run_tests() -> tuple[bool, str]:
    """Return (passed, tail-of-output) for the step-6 test file."""
    proc = subprocess.run(
        [
            "uv", "run", "--no-sync", "pytest", TESTS,
            "-q", "-p", "no:cacheprovider", "--no-header", "-x",
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    tail = (proc.stdout or "")[-400:]
    return proc.returncode == 0, tail


def main() -> int:
    original = MODULE.read_bytes()
    digest = __import__("hashlib").sha256(original).hexdigest()

    base_ok, base_tail = run_tests()
    if not base_ok:
        print("BASELINE FAILS -- fix before sabotaging:")
        print(base_tail)
        return 1
    print("baseline: PASS\n")

    caught = 0
    missed: list[str] = []
    errors: list[str] = []
    try:
        for index, (name, old, new) in enumerate(SABOTAGES, start=1):
            text = original.decode("utf-8")
            if old not in text:
                errors.append(f"{index:2d}. {name}: PATTERN NOT FOUND (harness stale)")
                print(f"{index:2d}. {name}: PATTERN NOT FOUND -- harness is stale")
                continue
            if text.count(old) != 1:
                errors.append(f"{index:2d}. {name}: pattern appears {text.count(old)}x")
                print(f"{index:2d}. {name}: AMBIGUOUS PATTERN ({text.count(old)}x)")
                continue
            MODULE.write_text(text.replace(old, new), encoding="utf-8")
            try:
                passed, _ = run_tests()
            finally:
                MODULE.write_bytes(original)
            if passed:
                missed.append(name)
                print(f"{index:2d}. {name}: MISSED (tests still pass)")
            else:
                caught += 1
                print(f"{index:2d}. {name}: caught")
    finally:
        MODULE.write_bytes(original)

    restored = __import__("hashlib").sha256(MODULE.read_bytes()).hexdigest()
    print()
    print(f"caught {caught}/{len(SABOTAGES) - len(errors)}")
    if missed:
        print("MISSED:")
        for name in missed:
            print(f"  - {name}")
    if errors:
        print("HARNESS ERRORS:")
        for item in errors:
            print(f"  - {item}")
    print(f"module restored: {'clean' if restored == digest else 'DIRTY -- INVESTIGATE'}")

    final_ok, _ = run_tests()
    print(f"final baseline: {'PASS' if final_ok else 'FAIL'}")
    return 0 if (not missed and not errors and restored == digest and final_ok) else 1


if __name__ == "__main__":
    sys.exit(main())
