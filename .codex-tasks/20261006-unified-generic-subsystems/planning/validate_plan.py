"""Validate the unified generic-subsystems Taskmaster plan without product imports."""
from __future__ import annotations

import csv
import hashlib
import json
import re
import shlex
from pathlib import Path


TASK_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = TASK_ROOT.parents[1]


def main() -> None:
    with (TASK_ROOT / "SUBTASKS.csv").open(encoding="utf-8", newline="") as handle:
        children = list(csv.DictReader(handle))
    assert len(children) == 10
    by_id = {row["id"]: row for row in children}
    assert len(by_id) == len(children)
    with (TASK_ROOT / "planning/artifacts.csv").open(encoding="utf-8", newline="") as handle:
        planned = {row["path"]: row for row in csv.DictReader(handle)}
    assert all(row["owner"] in by_id for row in planned.values())

    visiting: set[str] = set()
    visited: set[str] = set()
    owned_targets: dict[str, set[str]] = {identity: set() for identity in by_id}

    def visit(identity: str) -> None:
        assert identity not in visiting, f"dependency cycle: {identity}"
        if identity in visited:
            return
        visiting.add(identity)
        for dependency in filter(None, by_id[identity]["depends_on"].split(";")):
            assert dependency in by_id, dependency
            visit(dependency)
        visiting.remove(identity)
        visited.add(identity)

    def dependency_ids(identity: str) -> set[str]:
        result = {identity}
        for dependency in filter(None, by_id[identity]["depends_on"].split(";")):
            result.update(dependency_ids(dependency))
        return result

    def command_targets(command: str) -> tuple[list[str], list[str]]:
        words = shlex.split(command)
        assert words[:3] == ["uv", "run", "--no-sync"], command
        if words[3] == "python":
            return [words[4]], words
        assert words[3] == "pytest", command
        paths = []
        skip_value = False
        for word in words[4:]:
            if skip_value:
                skip_value = False
                continue
            if word == "-p":
                skip_value = True
            elif not word.startswith("-"):
                paths.append(word.split("::", 1)[0])
        roots = {path.split("/")[1] for path in paths if path.startswith("packages/")}
        assert not ("suspension_multibody" in roots and len(roots) > 1), command
        return paths, words

    def validate_command(command: str, child_id: str, must_exist: bool) -> None:
        targets, _ = command_targets(command)
        for target in targets:
            owned_targets[child_id].add(target)
            if (REPO_ROOT / target).exists():
                continue
            assert not must_exist and target in planned, f"unowned or missing command target: {target}"
            assert planned[target]["owner"] in dependency_ids(child_id), (child_id, target)

    for child in children:
        visit(child["id"])
        assert child["task_type"] == "single-full"
        assert child["status"] in {"TODO", "IN_PROGRESS", "DONE", "FAILED"}
        if child["status"] != "TODO":
            for dependency in filter(None, child["depends_on"].split(";")):
                assert by_id[dependency]["status"] == "DONE", (child["id"], dependency)
        directory = TASK_ROOT / child["task_dir"]
        for filename in ("SPEC.md", "TODO.csv", "PROGRESS.md"):
            assert (directory / filename).is_file(), directory / filename
        spec = (directory / "SPEC.md").read_text(encoding="utf-8")
        match = re.search(r"依赖：([^。\n]+)", spec)
        assert match is not None
        assert match[1] == (child["depends_on"] or "无"), (child["id"], "SPEC dependency mismatch")
        validate_command(child["validation_command"], child["id"], must_exist=True)
        with (directory / "TODO.csv").open(encoding="utf-8", newline="") as handle:
            steps = list(csv.DictReader(handle))
        assert 3 <= len(steps) <= 4, (child["id"], len(steps))
        assert [step["id"] for step in steps] == [str(i) for i in range(1, len(steps) + 1)]
        assert all(step["status"] in {"TODO", "IN_PROGRESS", "DONE", "FAILED"} for step in steps)
        for index, step in enumerate(steps):
            if step["status"] != "TODO":
                assert all(previous["status"] == "DONE" for previous in steps[:index]), (child["id"], step["id"], "prior steps incomplete")
            validate_command(step["validation_command"], child["id"], must_exist=step["status"] == "DONE")
            if step["status"] == "DONE":
                assert step["completed_at"] and step["notes"], (child["id"], step["id"])
                evidence_path = TASK_ROOT / "raw/validation" / f"child-{child['id']}-step-{step['id']}.json"
                assert evidence_path.is_file(), evidence_path
                evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
                assert evidence["returncode"] == 0 and evidence["command"] == shlex.split(step["validation_command"])
                assert evidence["started_at"] and evidence["finished_at"]
                logfile = TASK_ROOT / evidence["log"]
                assert logfile.is_file() and hashlib.sha256(logfile.read_bytes()).hexdigest() == evidence["log_sha256"]
        if child["status"] == "DONE":
            assert all(step["status"] == "DONE" for step in steps)
            assert child["completed_at"] and child["notes"]
        if child["status"] == "TODO":
            assert all(step["status"] == "TODO" for step in steps)

        if child["id"] == "1" and any(step["status"] != "TODO" for step in steps[1:]):
            inventory_path = TASK_ROOT / "raw/inventory.json"
            assert inventory_path.is_file(), "inventory required before product changes"
            inventory = json.loads(inventory_path.read_text(encoding="utf-8"))
            assert inventory["snapshot_complete"] is True
            assert inventory["captured_at"] and inventory["production_fingerprint"]

    for target, artifact in planned.items():
        assert target in owned_targets[artifact["owner"]], (target, "missing from owner validation")

    for required in ("EPIC.md", "SUBTASKS.csv", "PROGRESS.md", "planning/review.md", "planning/README.md"):
        assert (TASK_ROOT / required).is_file(), required
    print("Plan valid: 10 children; DAG/SPEC/step order; command paths/ownership; DONE logs; separate pytest roots")


if __name__ == "__main__":
    main()
