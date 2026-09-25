#!/usr/bin/env python
"""
Baseline gate for the composable-multibody architecture epic (subtask 01).

``--check`` validates the frozen baseline evidence produced by
``.codex-tasks/multibody-composable-architecture/tasks/01-baseline/collect_baseline.py``
and its companion records, and refuses to pass on weak evidence:

* the native kernel must be present, and the two ``suspension_kernel.dll``
  copies (canonical kernel package and the ``suspension_multibody`` mirror) must
  hold identical bytes -- the freshness guard in
  ``suspension_multibody/kernel/native.py:84`` refuses to load a stale mirror,
  so a mismatched pair is a broken environment, not a passing baseline;
* the recorded source fingerprint must match the tree on disk, and none of the
  fingerprinted sources may be deleted: a baseline may not silently outlive the
  sources it describes;
* every required command must carry its exact command text, an integer exit
  code, a ``pass`` result, non-empty ``observes`` prose, and -- for the test
  suites -- a quantitative summary line.  A stub entry that merely claims
  ``"pass"`` without the command text and observation it stands for is rejected,
  because that is exactly the weak evidence this gate exists to catch;
* **any** non-zero exit code fails the gate.  A recorded failure is a finding
  for the subtask that owns it, never something this gate waves through: the
  epic requires a clean baseline, and ``dynamic_hash_sentinel``'s internal
  acceptance exit code is recorded in its ``observes`` prose rather than
  smuggled in as an allowed non-zero command exit;
* the frozen numerical baselines (K/C snapshot, dynamic output hash, native
  performance budget) must be present on disk, unchanged against their recorded
  digests, and self-consistent;
* every A1-A10 quantity must resolve to a named tolerance authority, matched on
  identifier boundaries -- a stray substring is not a reference, and the
  ``authorities`` list must actually contain the id;
* both goal-relevant gaps must be present by id, explicitly open or closed, and
  any open gap must name its owning subtask; every known limit must carry a
  classification, and malformed entries fail rather than being skipped.

The gate deliberately does **not** re-run the expensive command set: it checks
that the recorded evidence is complete, internally consistent, and still true of
the current tree.  Re-running belongs to the subtask that owns each command and
to subtask 13's independent acceptance.

Usage (from the repository root):

    uv run --no-sync python packages/suspension_multibody/scripts/check_composable_baseline.py --check
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path
from typing import Any

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
TASK_DIRECTORY = (
    REPOSITORY_ROOT
    / ".codex-tasks/multibody-composable-architecture/tasks/01-baseline"
)

EVIDENCE_PATH = TASK_DIRECTORY / "BASELINE.json"
COMMANDS_PATH = TASK_DIRECTORY / "COMMANDS.json"
TOLERANCES_PATH = TASK_DIRECTORY / "TOLERANCES.json"
FINDINGS_PATH = TASK_DIRECTORY / "FINDINGS.json"

LIBRARY_NAME = "suspension_kernel.dll"
KERNEL_NATIVE = REPOSITORY_ROOT / "packages/suspension_kernel/src/suspension_kernel/native"
MULTIBODY_NATIVE = (
    REPOSITORY_ROOT / "packages/suspension_multibody/src/suspension_multibody/native"
)
MULTIBODY_DATA = PACKAGE_ROOT / "tests/data"

#: Commands the epic's command set requires.  A command absent from the
#: recorded evidence is a gap, not a silent omission.
REQUIRED_COMMANDS = (
    "build_axle_native",
    "kernel_layering",
    "kernel_tests",
    "contracts_tests",
    "multibody_tests",
    "dynamic_hash",
    "kc_parity",
    "case_parity",
    "kc_perf",
    "legacy_surface_gate",
    "ruff",
    "ty",
    "build_contracts",
    "build_kernel",
    "build_multibody",
    "git_diff_check",
)

#: Test suites whose bare "passed" is not comparable evidence: they must record
#: the pytest summary line so a later run can be diffed against it.
REQUIRED_QUANTITATIVE = {
    "kernel_tests": "summary_line",
    "contracts_tests": "summary_line",
    "multibody_tests": "summary_line",
}

#: Every command entry must state what it ran and what it observed.  Without
#: these, an entry can claim success for work it never describes.
REQUIRED_PROSE_FIELDS = ("command", "observes")

#: Resolutions accepted for an A1-A10 quantity.  ``defined`` means an existing
#: gate already carries the rule; the others name the subtask that must land it
#: before that quantity can be asserted.
ACCEPTED_TOLERANCE_STATUSES = (
    "defined",
    "defined_by_existing_solver",
    "to_be_established_by_04_06",
    "to_be_established_by_08_09",
)

REQUIRED_FROZEN_BASELINES = (
    "kc_snapshot_manifest",
    "kc_snapshot_k_states_sha256",
    "kc_snapshot_c_states_sha256",
    "dynamic_hash_baseline",
    "kc_perf_baseline_native",
)

#: Frozen artefacts whose recorded digest must still match the file on disk.
FROZEN_DIGESTS = (
    ("kc_snapshot_k_states_sha256", "kc_baseline/k_states.json"),
    ("kc_snapshot_c_states_sha256", "kc_baseline/c_states.json"),
    ("dynamic_hash_baseline_sha256", "dynamic_hash_baseline.json"),
    ("kc_perf_baseline_native_sha256", "kc_perf_baseline_native.json"),
)

#: Gaps the epic's baseline freeze must account for, with their owning subtask.
EXPECTED_GAPS = {"GAP-1": "07", "GAP-2": "10"}


def _repository_root() -> Path:
    """
    Return the workspace root as git reports it.

    Resolving from ``__file__`` is unreliable when the checkout lives under a
    path whose encoding differs between the shell and Python on Windows, so the
    root is asked for rather than inferred.
    """
    import os
    import subprocess

    completed = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        capture_output=True,
        text=True,
        encoding="utf-8",
        check=True,
    )
    return Path(os.path.normpath(completed.stdout.strip()))


class Report:
    """Accumulate findings so every problem is reported in one run."""

    def __init__(self) -> None:
        self.failures: list[str] = []
        self.notes: list[str] = []

    def fail(self, message: str) -> None:
        """Record a reason the baseline cannot be accepted."""
        self.failures.append(message)

    def note(self, message: str) -> None:
        """Record a fact that supports the verdict."""
        self.notes.append(message)

    def emit(self) -> int:
        """Print the notes and any failures, and return the process exit code."""
        for message in self.notes:
            print(f"  {message}")
        if self.failures:
            print()
            print("FAIL: baseline evidence is incomplete or inconsistent")
            for message in self.failures:
                print(f"  - {message}")
            return 1
        print()
        print("OK: baseline evidence is complete, consistent, and current")
        return 0


def sha256(path: Path) -> str:
    """Return the SHA-256 digest of ``path``, read in bounded chunks."""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_report(path: Path, report: Report) -> Any | None:
    """Read a JSON evidence file, recording a failure if it is absent or invalid."""
    if not path.is_file():
        report.fail(f"missing or unreadable evidence file: {path}")
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError) as error:
        report.fail(f"{path.name} is not valid JSON: {error}")
        return None


def _names_authority(authority: str, authority_ids: set[str]) -> bool:
    """
    Return whether an authority string names one of the declared ids.

    Matching is on identifier boundaries rather than raw substrings: a
    one-character id must not be satisfied by appearing inside an unrelated
    word, and an empty id must never match anything.
    """
    # Hyphens stay inside a token: "kc_parity_tolerance-not-a-rule" is one
    # identifier and must not be read as a mention of "kc_parity_tolerance".
    tokens = set(re.findall(r"[A-Za-z0-9_-]+", authority))
    return bool(tokens & {identifier for identifier in authority_ids if identifier})


def check_native(evidence: dict[str, Any], report: Report) -> None:
    """Require the kernel to exist, with both copies byte-identical."""
    kernel_library = KERNEL_NATIVE / LIBRARY_NAME
    mirror_library = MULTIBODY_NATIVE / LIBRARY_NAME

    if not kernel_library.is_file():
        report.fail(f"missing canonical native kernel: {kernel_library}")
        return
    if not mirror_library.is_file():
        report.fail(f"missing mirrored native kernel: {mirror_library}")
        return

    kernel_digest = sha256(kernel_library)
    mirror_digest = sha256(mirror_library)
    if kernel_digest != mirror_digest:
        report.fail(
            "the native kernel mirror does not match the canonical library "
            f"({mirror_digest[:16]} vs {kernel_digest[:16]}); the freshness guard "
            "at suspension_multibody/kernel/native.py:84 will refuse to load it. "
            "Repair with: uv run python "
            "packages/suspension_multibody/scripts/build_axle_native.py"
        )
        return

    recorded = evidence.get("native", {})
    if not isinstance(recorded, dict):
        report.fail("BASELINE.json native section is not an object")
        return
    if not recorded.get("mirrors_identical"):
        report.fail("BASELINE.json records the native mirrors as not identical")
    for key, digest in (
        ("canonical_library", kernel_digest),
        ("mirror_library", mirror_digest),
    ):
        recorded_digest = recorded.get(key, {}).get("sha256")
        if recorded_digest != digest:
            report.fail(
                f"{key}: recorded sha256 {recorded_digest!r} does not match the "
                f"library on disk {digest!r}"
            )

    build = recorded.get("build_metadata")
    if not isinstance(build, dict) or "abi_version" not in build:
        report.fail("BASELINE.json native.build_metadata is missing or lacks abi_version")
    else:
        report.note(
            "native kernel present; both copies identical "
            f"({kernel_digest[:16]}); abi {build.get('abi_version')}/"
            f"{build.get('core_abi_version')}/{build.get('vehicle_abi_version')}"
        )


def check_source_fingerprint(evidence: dict[str, Any], report: Report) -> None:
    """Refuse a baseline that has outlived the sources it describes."""
    recorded = evidence.get("sources", {}).get("kernel_cpp")
    if not isinstance(recorded, dict) or not isinstance(recorded.get("files"), dict):
        report.fail("BASELINE.json sources.kernel_cpp is missing its per-file fingerprint")
        return

    files: dict[str, str] = recorded["files"]
    if not files:
        report.fail("BASELINE.json sources.kernel_cpp fingerprints no files")
        return

    missing = [name for name in files if not (REPOSITORY_ROOT / name).is_file()]
    if missing:
        report.fail(
            f"sources.kernel_cpp lists {len(missing)} file(s) that no longer exist, "
            f"first: {missing[0]}"
        )
        return

    drifted = [
        name for name, digest in files.items() if sha256(REPOSITORY_ROOT / name) != digest
    ]
    if drifted:
        report.fail(
            f"{len(drifted)} kernel source file(s) changed since the baseline was "
            f"collected, first: {drifted[0]}; re-collect with collect_baseline.py"
        )
        return

    report.note(
        f"kernel source fingerprint matches the tree ({recorded.get('count')} files, "
        f"{recorded.get('combined_sha256', '')[:16]})"
    )


def check_commands(commands_document: Any, report: Report) -> None:
    """Every required command needs its text, prose, a real result, and a clean exit."""
    if not isinstance(commands_document, dict):
        return
    entries = commands_document.get("commands")
    if not isinstance(entries, list):
        report.fail("COMMANDS.json has no commands list")
        return

    by_id: dict[str, dict[str, Any]] = {}
    for entry in entries:
        if not isinstance(entry, dict) or not isinstance(entry.get("id"), str):
            report.fail(f"COMMANDS.json entry without a string id: {entry!r}")
            continue
        if entry["id"] in by_id:
            report.fail(f"COMMANDS.json declares {entry['id']!r} twice")
            continue
        by_id[entry["id"]] = entry

    missing = [name for name in REQUIRED_COMMANDS if name not in by_id]
    if missing:
        report.fail(f"commands missing from the evidence: {', '.join(missing)}")

    for name in REQUIRED_COMMANDS:
        entry = by_id.get(name)
        if entry is None:
            continue

        for field in REQUIRED_PROSE_FIELDS:
            value = entry.get(field)
            if not isinstance(value, str) or not value.strip():
                report.fail(
                    f"{name}: {field} is empty; an entry must state the command it "
                    "ran and what it observed"
                )

        exit_code = entry.get("exit_code")
        result = entry.get("result")
        if result != "pass":
            report.fail(f"{name}: result {result!r} is not an executed pass")
        if not isinstance(exit_code, int) or isinstance(exit_code, bool):
            report.fail(f"{name}: exit_code {exit_code!r} is not an integer")
            continue
        if exit_code != 0:
            report.fail(
                f"{name}: recorded exit code {exit_code} is a failure; the epic "
                "requires a clean baseline, so a non-zero exit here must be fixed "
                "rather than recorded"
            )

        required_field = REQUIRED_QUANTITATIVE.get(name)
        if required_field and not entry.get(required_field):
            report.fail(
                f"{name}: no {required_field}; a bare 'pass' is not comparable evidence"
            )

    report.note(
        f"command evidence complete ({len(by_id)} entries, "
        f"{len(REQUIRED_COMMANDS)} required, all with prose and exit code)"
    )


def check_tolerances(tolerances: Any, report: Report) -> None:
    """Every A1-A10 quantity must resolve to a declared authority."""
    if not isinstance(tolerances, dict):
        return
    authorities = tolerances.get("authorities")
    units = tolerances.get("planned_units")
    if not isinstance(authorities, list) or not authorities:
        report.fail("TOLERANCES.json declares no tolerance authorities")
        return
    if not isinstance(units, list) or not units:
        report.fail("TOLERANCES.json declares no planned quantities")
        return

    authority_ids: set[str] = set()
    for item in authorities:
        if not isinstance(item, dict):
            report.fail(f"invalid tolerance authority entry: {item!r}")
            continue
        identifier = item.get("id")
        if not isinstance(identifier, str) or not identifier:
            report.fail(f"tolerance authority without a usable id: {item!r}")
            continue
        if not str(item.get("rule", "")).strip():
            report.fail(f"tolerance authority {identifier!r} declares no rule")
        authority_ids.add(identifier)

    unresolved: list[str] = []
    for unit in units:
        if not isinstance(unit, dict):
            report.fail(f"invalid planned unit entry: {unit!r}")
            continue
        quantity = unit.get("quantity")
        status = unit.get("status")
        if not unit.get("unit"):
            report.fail(f"quantity {quantity!r} has no unit")
        if status not in ACCEPTED_TOLERANCE_STATUSES:
            unresolved.append(f"{quantity!r}={status!r}")
        authority = unit.get("authority", "")
        if not isinstance(authority, str):
            report.fail(f"quantity {quantity!r} has a non-string authority")
            continue
        if status == "defined" and not _names_authority(authority, authority_ids):
            report.fail(
                f"quantity {quantity!r} claims a defined tolerance but names no "
                f"declared authority: {authority!r}"
            )

    if unresolved:
        report.fail(
            "quantities without an accepted tolerance resolution: " + ", ".join(unresolved)
        )
        return

    pending = [
        unit.get("quantity")
        for unit in units
        if isinstance(unit, dict)
        and str(unit.get("status", "")).startswith("to_be_established")
    ]
    report.note(
        f"tolerance authorities cover all {len(units)} quantities "
        f"({len(pending)} deferred to named owning subtasks)"
    )


def check_findings(findings: Any, report: Report) -> None:
    """Require the two goal-relevant gaps to be owned and the limits classified."""
    if not isinstance(findings, dict):
        return
    gaps = findings.get("gaps")
    limits = findings.get("non_goal_limits")
    if not isinstance(gaps, list) or not gaps:
        report.fail("FINDINGS.json records no gap verdicts")
        return
    if not isinstance(limits, list) or not limits:
        report.fail("FINDINGS.json records no known-limit classification")
        return

    seen_gaps: dict[str, dict[str, Any]] = {}
    for gap in gaps:
        if not isinstance(gap, dict):
            report.fail(f"invalid gap entry: {gap!r}")
            continue
        identifier = gap.get("id")
        if not isinstance(identifier, str) or not identifier:
            report.fail(f"gap without a usable id: {gap!r}")
            continue
        seen_gaps[identifier] = gap
        # ``still_present`` decides whether the gap is open or closed.  A gap
        # that omits it is unevaluated, which is not a closed gap.
        still_present = gap.get("still_present")
        if not isinstance(still_present, bool):
            report.fail(f"gap {identifier!r} does not state still_present as a boolean")
            continue
        if still_present and not gap.get("owner"):
            report.fail(f"gap {identifier!r} is open but names no owning subtask")
        if not gap.get("evidence"):
            report.fail(f"gap {identifier!r} has no evidence")

    for identifier, owner in EXPECTED_GAPS.items():
        gap = seen_gaps.get(identifier)
        if gap is None:
            report.fail(f"expected gap {identifier!r} is missing from FINDINGS.json")
            continue
        if gap.get("still_present") is True and gap.get("owner") != owner:
            report.fail(
                f"gap {identifier!r} is open but its owner is "
                f"{gap.get('owner')!r}, expected {owner!r}"
            )

    unclassified: list[str] = []
    for limit in limits:
        if not isinstance(limit, dict):
            report.fail(f"invalid known-limit entry: {limit!r}")
            continue
        identifier = limit.get("id")
        if not isinstance(identifier, str) or not identifier:
            report.fail(f"known limit without a usable id: {limit!r}")
            continue
        if not limit.get("classification"):
            unclassified.append(identifier)
    if unclassified:
        report.fail(f"known limits without a classification: {', '.join(unclassified)}")
        return

    report.note(
        f"gap verdicts recorded ({len(seen_gaps)} gaps, both expected ids present); "
        f"{len(limits)} known limits classified"
    )


def check_frozen_baselines(evidence: dict[str, Any], report: Report) -> None:
    """Frozen references must be present and unchanged against their digests."""
    recorded = evidence.get("frozen_baselines")
    if not isinstance(recorded, dict):
        report.fail("BASELINE.json has no frozen_baselines section")
        return

    missing = [name for name in REQUIRED_FROZEN_BASELINES if name not in recorded]
    if missing:
        report.fail(f"frozen baselines missing from the evidence: {', '.join(missing)}")
        return

    manifest = recorded["kc_snapshot_manifest"]
    if not isinstance(manifest, dict):
        report.fail("frozen K/C snapshot manifest is not an object")
        return
    if manifest.get("contract") != "kc-parity-v1":
        report.fail(f"unexpected K/C snapshot contract: {manifest.get('contract')!r}")
    if manifest.get("k_state_count") != 9 or manifest.get("c_state_count") != 66:
        report.fail(
            "K/C snapshot state counts changed: "
            f"K={manifest.get('k_state_count')} C={manifest.get('c_state_count')}"
        )

    for key, relative in FROZEN_DIGESTS:
        snapshot = MULTIBODY_DATA / relative
        if not snapshot.is_file():
            report.fail(f"missing frozen baseline file: {snapshot}")
            continue
        recorded_digest = recorded.get(key)
        if recorded_digest is None:
            report.fail(
                f"BASELINE.json does not record a digest for tests/data/{relative}"
            )
            continue
        if sha256(snapshot) != recorded_digest:
            report.fail(f"tests/data/{relative} changed since the baseline was collected")

    dynamic = recorded["dynamic_hash_baseline"]
    if not isinstance(dynamic, dict):
        report.fail("frozen dynamic hash baseline is not an object")
    else:
        combined = dynamic.get("combined_sha256")
        failed = dynamic.get("failed_cases")
        if not combined or not isinstance(failed, list):
            report.fail("dynamic hash baseline is missing its digest or failed-case list")
        else:
            report.note(
                f"dynamic hash baseline frozen at {combined[:16]} with "
                f"{len(failed)} known failing cases"
            )

    budget = recorded["kc_perf_baseline_native"]
    if not isinstance(budget, dict):
        report.fail("frozen performance baseline is not an object")
    else:
        if budget.get("implementation") != "native":
            report.fail("performance baseline is not the native implementation's")
        for workload in ("k-100", "c-66"):
            if workload not in budget.get("min_seconds", {}):
                report.fail(f"performance baseline lacks the {workload} workload")


def check_environment(evidence: dict[str, Any], report: Report) -> None:
    """Note the recorded revision and the dirty state captured with the baseline."""
    environment = evidence.get("environment", {})
    if not isinstance(environment, dict):
        report.fail("BASELINE.json environment section is not an object")
        return
    for field in ("git_head", "platform", "python"):
        if not environment.get(field):
            report.fail(f"BASELINE.json environment is missing {field}")
    head = environment.get("git_head_short", "unknown")
    # Only tracked modifications matter here: untracked entries are the
    # evidence files and the new gate script themselves.  A modified or staged
    # tracked file would mean the baseline describes a tree that no longer
    # exists, which the reader needs to see.
    tracked_dirty = [
        line
        for line in str(environment.get("git_status_porcelain", "")).splitlines()
        if line.strip() and not line.startswith("??")
    ]
    report.note(
        f"baseline collected at {head}"
        + (
            f" (working tree had {len(tracked_dirty)} tracked modification(s))"
            if tracked_dirty
            else ""
        )
    )


def main() -> int:
    """Validate the recorded baseline evidence and report every problem found."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="validate the recorded baseline evidence (the only supported mode)",
    )
    args = parser.parse_args()
    if not args.check:
        parser.error("--check is required; this gate validates recorded evidence")

    report = Report()
    evidence = load_report(EVIDENCE_PATH, report)
    commands = load_report(COMMANDS_PATH, report)
    tolerances = load_report(TOLERANCES_PATH, report)
    findings = load_report(FINDINGS_PATH, report)

    if isinstance(evidence, dict):
        check_native(evidence, report)
        check_source_fingerprint(evidence, report)
        check_frozen_baselines(evidence, report)
        check_environment(evidence, report)
    check_commands(commands, report)
    check_tolerances(tolerances, report)
    check_findings(findings, report)

    return report.emit()


if __name__ == "__main__":
    sys.exit(main())
