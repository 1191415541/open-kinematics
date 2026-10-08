"""Verify the preserved pre-delete gate without executing retired sources."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    repo = root.parents[1]
    raw = root / "raw"

    def digest(path: Path) -> str:
        return hashlib.sha256(path.read_bytes()).hexdigest()

    inventory = json.loads((raw / "inventory.json").read_text(encoding="utf-8"))
    mapping = json.loads((raw / "migration-consumers.json").read_text(encoding="utf-8"))
    report = json.loads((raw / "cutover-validation.json").read_text(encoding="utf-8"))
    assert inventory["snapshot_complete"]
    assert report["passed"] is True
    assert report["inventory_sha256"] == mapping["inventory_sha256"] == digest(raw / "inventory.json")
    assert report["mapping_sha256"] == digest(raw / "migration-consumers.json")
    assert report["consumer_count"] == len(mapping["consumers"]) == len([
        row for row in inventory["consumers"] if row["category"] != "test"])
    assert len(report["checks"]) == 3
    stages = ("numeric", "structural", "pytest")
    for check, stage in zip(report["checks"], stages):
        assert check["returncode"] == 0
        command = check["command"]
        if stage == "pytest":
            assert command == ["uv", "run", "--no-sync", "pytest",
                "packages/suspension_multibody/tests/simulation/test_single_model_pipeline.py",
                "packages/suspension_multibody/tests/physics/test_wheel_spin_boundary.py",
                "packages/suspension_multibody/tests/architecture/test_unified_model_cutover.py",
                "packages/suspension_multibody/tests/simulation/test_document_route_reaches_every_registered_family.py",
                "-q", "-p", "no:cacheprovider"]
        else:
            assert len(command) == 3
            assert Path(command[1]).resolve() == root / "planning/checks.py"
            assert command[2] == stage
        logfile = raw / check["log"]
        assert digest(logfile) == check["log_sha256"]
        output = logfile.read_text(encoding="utf-8")
        assert (f"PASS: {stage};" in output) if stage != "pytest" else "61 passed" in output
        print(f"archived {stage}: exit 0, log SHA256 verified")
    for name, expected in inventory["frozen_fingerprints"].items():
        assert digest(repo / name) == expected, name
    print(f"PASS: pre-delete evidence for {report['consumer_count']} consumers and 9 unchanged frozen files")
    print("Archive verification only; original pre-delete execution remains in cutover-validation.json.")


if __name__ == "__main__":
    main()
