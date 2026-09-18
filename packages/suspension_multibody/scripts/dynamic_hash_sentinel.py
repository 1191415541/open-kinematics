#!/usr/bin/env python
"""
Dynamic-path output hash sentinel (gate B, byte-level tier).

Runs the frozen axle-dynamics acceptance matrix and hashes every produced
result artifact:

* ``arrays.npz`` bytes per case and variant (``native_result`` /
  ``native_refined_result``);
* a canonical view of each ``manifest.json`` with machine/timing dependent keys
  (``performance``, ``native_build``) removed.

A combined digest over the sorted per-artifact entries is compared against the
recorded baseline.  A non-zero acceptance exit code is *expected*: three road
cases are documented to fail the ``time_convergence`` gate on
``fixture.force_z`` alone (``docs/axle_dynamics_results.md``), and the project
records that honestly rather than loosening the cases.  The sentinel therefore
records the exit code and the failing case list, and fails only on drift.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[1]
ACCEPTANCE = PACKAGE_ROOT / "scripts" / "run_axle_dynamics_acceptance.py"
DEFAULT_OUTPUT = REPO_ROOT / "artifacts" / "axle-dynamics-acceptance"
DEFAULT_BASELINE = PACKAGE_ROOT / "tests" / "data" / "dynamic_hash_baseline.json"
VOLATILE_MANIFEST_KEYS = ("performance", "native_build")


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _canonical_manifest(manifest: dict[str, object]) -> bytes:
    trimmed = {
        key: value
        for key, value in manifest.items()
        if key not in VOLATILE_MANIFEST_KEYS
    }
    return json.dumps(
        trimmed, sort_keys=True, separators=(",", ":"), default=str
    ).encode("utf-8")


def run_acceptance(output: Path) -> int:
    """Run the frozen acceptance matrix and return its exit code."""
    command = [sys.executable, str(ACCEPTANCE), "--output", str(output)]
    return int(subprocess.run(command, cwd=str(REPO_ROOT)).returncode)


def _entries(output: Path) -> list[dict[str, object]]:
    entries: list[dict[str, object]] = []
    for manifest_path in sorted(output.glob("*/native/*/manifest.json")):
        arrays_path = manifest_path.parent / "arrays.npz"
        if not arrays_path.is_file():
            continue
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        entries.append(
            {
                "artifact": str(
                    manifest_path.parent.relative_to(output)
                ).replace("\\", "/"),
                "arrays_npz_sha256": _sha256(arrays_path.read_bytes()),
                "manifest_sha256": _sha256(_canonical_manifest(manifest)),
                "status": manifest.get("status"),
                "completed_samples": manifest.get("completed_samples"),
            }
        )
    return entries


def measure(output: Path, exit_code: int | None) -> dict[str, object]:
    """Hash every produced artifact and summarise the run."""
    report_path = output / "acceptance_report.json"
    if not report_path.is_file():
        raise SystemExit(f"no acceptance report under {output}")
    report = json.loads(report_path.read_text(encoding="utf-8"))
    entries = _entries(output)
    if not entries:
        raise SystemExit(f"no native result artifacts under {output}")
    cases = report.get("cases", [])
    failed = sorted(
        str(case.get("case"))
        for case in cases
        if str(case.get("status", "")).upper() != "PASSED"
    )
    return {
        "artifact_count": len(entries),
        "combined_sha256": _sha256(
            json.dumps(entries, sort_keys=True, separators=(",", ":")).encode(
                "utf-8"
            )
        ),
        "acceptance_exit_code": exit_code,
        "acceptance_statuses": {
            str(case.get("case")): case.get("status") for case in cases
        },
        "failed_cases": failed,
        "entries": entries,
    }


def main() -> int:
    """Record or check the dynamic output hash baseline."""
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--record", action="store_true")
    mode.add_argument("--check", action="store_true")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE)
    parser.add_argument(
        "--skip-run", action="store_true", help="reuse an existing output dir"
    )
    args = parser.parse_args()

    exit_code = None
    if not args.skip_run:
        # A failed run must not be scored against stale artifacts: the mirror
        # freshness guard, for example, aborts in under two seconds and leaves
        # the previous output in place, which would otherwise look like a pass.
        report_path = args.output / "acceptance_report.json"
        before = report_path.stat().st_mtime if report_path.is_file() else 0.0
        exit_code = run_acceptance(args.output)
        after = report_path.stat().st_mtime if report_path.is_file() else 0.0
        if after <= before:
            raise SystemExit(
                "acceptance did not refresh its output (exit code "
                f"{exit_code}); refusing to hash stale artifacts"
            )
    measured = measure(args.output, exit_code)

    print(f"artifacts hashed : {measured['artifact_count']}")
    print(f"combined sha256  : {measured['combined_sha256']}")
    print(f"acceptance exit  : {measured['acceptance_exit_code']}")
    print(f"failed cases     : {measured['failed_cases']}")

    if args.record:
        args.baseline.parent.mkdir(parents=True, exist_ok=True)
        args.baseline.write_text(
            json.dumps(measured, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(f"recorded baseline -> {args.baseline}")
        return 0

    if not args.baseline.is_file():
        raise SystemExit(f"no baseline at {args.baseline}; run --record first")
    expected = json.loads(args.baseline.read_text(encoding="utf-8"))
    if expected["combined_sha256"] == measured["combined_sha256"]:
        print("\nOK: dynamic output matches the frozen baseline byte-for-byte")
        return 0

    print("\nFAIL: dynamic output drifted")
    want = {e["artifact"]: e for e in expected["entries"]}
    got = {e["artifact"]: e for e in measured["entries"]}  # ty: ignore[not-iterable]
    for artifact in sorted(set(want) | set(got)):
        if artifact not in got:
            print(f"  missing : {artifact}")
        elif artifact not in want:
            print(f"  added   : {artifact}")
        elif want[artifact] != got[artifact]:
            print(f"  drifted : {artifact}")
    if expected["failed_cases"] != measured["failed_cases"]:
        print(
            "  failing cases changed: "
            f"{expected['failed_cases']} -> {measured['failed_cases']}"
        )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
