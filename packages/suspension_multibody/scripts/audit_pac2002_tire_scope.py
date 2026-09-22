"""
Audit the native PAC2002 scope against every tire in the installed Adams library.

The fail-closed registry in ``kernel/capabilities.py`` is only trustworthy if it has been
measured against real tire files.  This script parses every stock ``.tir``, asks
``pac2002_unsupported_native_reasons`` whether the native kernel would accept it,
and reports the accept/reject split with the exact reason for each rejection.

It is the evidence behind the coverage declaration published in the comparison
manifest: it says which slices of the shipped Adams tire set the native model can
actually reproduce, and names the feature each remaining tire asks for.

Usage::

    uv run --package suspension-multibody python \
        packages/suspension_multibody/scripts/audit_pac2002_tire_scope.py

Add ``--json <path>`` to write the full per-tire table.
"""

from __future__ import annotations

import argparse
import json
import pathlib
from collections import Counter

from suspension_multibody.adams import discover_profile
from suspension_multibody.adams.full_vehicle_model import _parse_tire
from suspension_multibody.kernel.capabilities import (
    PAC2002_SUPPORTED_NATIVE_USE_MODES,
    pac2002_native_use_mode,
    pac2002_unsupported_native_reasons,
)

# Tire formats that are not PAC2002 and therefore never a native PAC2002 target.
_OTHER_FORMATS = ("FIALA", "FTIRE", "CDTIRE", "PAC89", "PAC94", "PACTIME", "SWIFT")


def library_root() -> pathlib.Path:
    """Return the Adams tire library directory of the installed release."""
    profile = discover_profile()
    if not profile.available or profile.database_path is None:
        raise RuntimeError(f"Adams/Car is unavailable: {profile.message}")
    return pathlib.Path(profile.database_path) / "tires.tbl"


def tire_format(coefficients: dict[str, float]) -> str:
    """Classify a parsed tire by the format switch the importer recorded."""
    if coefficients.get("PAC2002_UNSUPPORTED_PAC_MC"):
        return "PAC-MC"
    if coefficients.get("PROPERTY_FILE_FORMAT_PAC2002"):
        return "PAC2002"
    return "OTHER"


def collect(root: pathlib.Path) -> list[dict[str, object]]:
    """Parse every ``.tir`` under ``root`` and classify its native scope."""
    rows: list[dict[str, object]] = []
    for path in sorted(root.glob("*.tir")):
        text = path.read_text(encoding="ascii", errors="replace")
        upper = text.upper()
        coefficients = _parse_tire(path)
        kind = tire_format(coefficients)
        if kind == "OTHER" and any(name in upper for name in _OTHER_FORMATS):
            kind = next(name for name in _OTHER_FORMATS if name in upper)
        reasons = (
            pac2002_unsupported_native_reasons(coefficients)
            if kind in {"PAC2002", "PAC-MC"}
            else ("not_a_pac2002_tire",)
        )
        rows.append(
            {
                "tire": path.name,
                "format": kind,
                "use_mode": (
                    pac2002_native_use_mode(coefficients)
                    if kind in {"PAC2002", "PAC-MC"}
                    else None
                ),
                "accepted": not reasons,
                "reasons": list(reasons),
            }
        )
    return rows


def summarize(rows: list[dict[str, object]]) -> dict[str, object]:
    """Reduce the per-tire table to the counts the coverage claim rests on."""
    pac2002 = [row for row in rows if row["format"] in {"PAC2002", "PAC-MC"}]
    accepted = [row for row in pac2002 if row["accepted"]]
    reason_counts: Counter[str] = Counter()
    for row in pac2002:
        if row["accepted"]:
            continue
        for reason in row["reasons"]:  # type: ignore[union-attr]
            head = reason.split(" (")[0]
            reason_counts[head] += 1
    return {
        "tire_files": len(rows),
        "pac2002_tires": len(pac2002),
        "pac_mc_tires": sum(1 for row in rows if row["format"] == "PAC-MC"),
        "accepted": len(accepted),
        "accepted_tires": sorted(row["tire"] for row in accepted),
        "rejected": len(pac2002) - len(accepted),
        "rejection_reasons": dict(sorted(reason_counts.items())),
        "supported_use_modes": sorted(PAC2002_SUPPORTED_NATIVE_USE_MODES),
        "use_modes_seen": dict(
            sorted(
                Counter(
                    row["use_mode"] for row in pac2002 if row["use_mode"] is not None
                ).items()
            )
        ),
    }


def main() -> None:
    """Parse arguments, run the audit, and print the summary."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=pathlib.Path, default=None)
    parser.add_argument("--json", type=pathlib.Path, default=None)
    parser.add_argument("--all", action="store_true", help="print every tire row")
    parser.add_argument("--json-only", action="store_true")
    args = parser.parse_args()

    root = args.root or library_root()
    rows = collect(root)
    summary = summarize(rows)
    if not args.json_only:
        print(json.dumps(summary, indent=2, sort_keys=True))
        if args.all:
            print("\n--- per tire ---")
            for row in rows:
                mark = "accept" if row["accepted"] else "reject"
                detail = "; ".join(row["reasons"])  # type: ignore[arg-type]
                print(f"  {mark}  {row['tire']:52s} {row['format']:8s} {detail}")
    if args.json is not None:
        args.json.write_text(
            json.dumps({"summary": summary, "tires": rows}, indent=2, sort_keys=True),
            encoding="utf-8",
        )


if __name__ == "__main__":
    main()
