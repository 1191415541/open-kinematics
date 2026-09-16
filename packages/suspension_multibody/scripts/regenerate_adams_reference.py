"""
Regenerate the Adams/Car full-vehicle reference case on the installed release.

The PAC2002 correlation gate compares native results against an Adams reference
case.  That reference is a large ``.res`` file which is not version controlled,
so this script exists to make it reproducible: anyone with Adams/Car installed
can regenerate the case the gate expects, and the generated
``adams_execution.json`` records the release that actually ran the solve.

Verified behaviour: the ``step_steer`` maneuver produces identical Adams histories
under 2024.1 and 2025.1.1, and the native comparison against either is
bit-identical.  Regenerating on a newer release therefore does not invalidate
existing evidence, but the recorded producer id will name the new release.

Usage::

    uv run --package suspension-multibody python \
        packages/suspension_multibody/scripts/regenerate_adams_reference.py

Use ``--output-root`` to write somewhere other than the default case directory,
and ``--case`` to pick a different built-in maneuver.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from suspension_multibody.adams import discover_profile
from suspension_multibody.adams.vehicle_handling import (
    HANDLING_CASES,
    run_adams_car_handling_case,
)

DEFAULT_OUTPUT_ROOT = Path("artifacts/adams-full-source-2025_1_1/step_steer")


def regenerate(output_root: Path, case: str, *, require_available: bool) -> dict[str, Any]:
    """Run one Adams/Car handling case and report the recorded evidence."""
    profile = discover_profile()
    summary: dict[str, Any] = {
        "profile": profile.name,
        "home": profile.home,
        "version": profile.version,
        "executable": profile.executable,
        "database": profile.database_path,
        "available": profile.available,
        "license_probe": profile.license_probe,
        "message": profile.message,
        "case": case,
        "output_root": str(output_root),
    }
    if not profile.available:
        if require_available:
            raise RuntimeError(f"Adams/Car is unavailable: {profile.message}")
        summary["status"] = "skipped"
        return summary

    history = run_adams_car_handling_case(profile, case, output_root)
    execution_path = output_root / "adams_execution.json"
    execution = (
        json.loads(execution_path.read_text(encoding="utf-8"))
        if execution_path.is_file()
        else {}
    )
    summary.update(
        {
            "status": "generated",
            "sample_count": len(history.time),
            "start_s": float(history.time[0]),
            "end_s": float(history.time[-1]),
            "channel_count": len(history.channels),
            "producer": execution.get("producer"),
            "analysis_mode": execution.get("analysis_mode"),
            "returncode": execution.get("returncode"),
            "raw_result_path": execution.get("raw_result_path"),
        }
    )
    return summary


def main() -> None:
    """Parse arguments, regenerate the reference case, and print the summary."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--case", default="step_steer", choices=sorted(HANDLING_CASES))
    parser.add_argument(
        "--allow-missing-adams",
        action="store_true",
        help="Report a skip instead of failing when Adams/Car is not installed.",
    )
    args = parser.parse_args()

    summary = regenerate(
        args.output_root,
        args.case,
        require_available=not args.allow_missing_adams,
    )
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
