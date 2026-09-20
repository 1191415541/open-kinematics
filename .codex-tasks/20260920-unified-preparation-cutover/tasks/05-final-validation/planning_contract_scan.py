"""Validate the durable planning and evidence contracts for this Epic.

Record model enforced here:

* gate commands are executed one by one and recorded immediately with the real
  exit code and an existing evidence file;
* a task's ``validation_command`` may be a ``&&`` chain, but each atomic command
  in that chain (quotes aware) needs its own successful record with the same
  task label;
* record-audit commands (``planning_contract_scan.py``) are never required as
  execution evidence, while functional/static commands always are;
* ``--final-preclose`` is the pre-close check run before the final status is
  written, ``--final`` is the post-hoc full state and evidence verification.
"""

from __future__ import annotations

import argparse
import csv
import os
import re
from pathlib import Path

EPIC_ROOT = Path(__file__).parents[2]
REPO_ROOT = EPIC_ROOT.parents[1]
MATRIX = EPIC_ROOT / "PREPARATION_MATRIX.md"
EPIC = EPIC_ROOT / "EPIC.md"
SUBTASKS = EPIC_ROOT / "SUBTASKS.csv"
PROGRESS = EPIC_ROOT / "PROGRESS.md"
TASKS = EPIC_ROOT / "tasks"
EXPECTED_KEYS = (
    ("axle", "axle_dynamic"),
    ("axle", "kc_quasi_static"),
    ("vehicle", "vehicle_kc"),
    ("vehicle", "handling"),
    ("vehicle", "ride_four_post"),
    ("vehicle", "ride_random_road"),
    ("vehicle", "vehicle_dynamic"),
)
RECORD_FIELDS = ("时间", "子任务/门禁", "命令", "退出码", "摘要", "证据文件")
PHASE_RECORD_LABELS = ("04-pre-delete", "04-post-delete")
PHASE_SCAN_MARKERS = {
    "04-pre-delete": "legacy_reference_scan.py --pre-delete",
    "04-post-delete": "legacy_reference_scan.py --post-delete",
}
# Only this script audits records; it never has to be its own execution evidence.
RECORD_AUDIT_MARKER = "planning_contract_scan.py"
PHASE_LABEL_PREFIX = "子任务 "


def _rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _task_directory(task_id: int) -> Path:
    matches = [
        path for path in TASKS.iterdir() if path.name.startswith(f"{task_id:02d}-")
    ]
    assert len(matches) == 1, (task_id, matches)
    return matches[0]


def _task_rows(task_id: int) -> list[dict[str, str]]:
    return _rows(_task_directory(task_id) / "TODO.csv")


def _assert_matrix(*, non_vehicle_only: bool = False) -> None:
    text = MATRIX.read_text(encoding="utf-8")
    required = EXPECTED_KEYS[:6] if non_vehicle_only else EXPECTED_KEYS
    rows = [
        line
        for line in text.splitlines()
        if line.startswith("| `(") and "suspension_multibody.preparation." in line
    ]
    assert len(rows) == 7, rows
    for assembly, family in required:
        key = f'`("{assembly}", "{family}")`'
        row = next((line for line in rows if key in line), None)
        assert row is not None, key
        assert "suspension_multibody.preparation." in row
        assert row.count("|") >= 9, row
    assert "context keys produced" in text
    assert "compiler consumer" in text
    assert "document owner" in text
    assert "Legacy `prepared` 适配" in text


def _assert_planning_files() -> None:
    subtasks = _rows(SUBTASKS)
    assert len(subtasks) == 5
    assert [row["id"] for row in subtasks] == [str(value) for value in range(1, 6)]
    assert [row["status"] for row in subtasks] == ["TODO"] * 5
    for task_id in range(1, 6):
        rows = _task_rows(task_id)
        assert rows, task_id
        assert all(row["status"] == "TODO" for row in rows), task_id
        assert all(
            row["acceptance_criteria"] and row["validation_command"] for row in rows
        ), task_id


def _record_blocks() -> list[str]:
    text = PROGRESS.read_text(encoding="utf-8")
    matches = [
        match
        for match in re.finditer(r"^### 验证记录：.*$", text, flags=re.MULTILINE)
        if match.group(0) != "### 验证记录：<子任务或门禁>"
    ]
    blocks: list[str] = []
    for index, match in enumerate(matches):
        end = matches[index + 1].start() if index + 1 < len(matches) else len(text)
        blocks.append(text[match.start() : end])
    return blocks


def _record_value(block: str, field: str) -> str | None:
    match = re.search(
        rf"^- \*\*{re.escape(field)}\*\*：(.*)$",
        block,
        flags=re.MULTILINE,
    )
    return match.group(1).strip() if match else None


def _normalise_command(value: str) -> str:
    return " ".join(value.replace("`", "").split())


def _split_atomic_commands(command: str) -> list[str]:
    """Split a ``&&`` chain into atomic commands, ignoring quoted ``&&``."""
    parts: list[str] = []
    current: list[str] = []
    quote: str | None = None
    index = 0
    while index < len(command):
        char = command[index]
        if quote is not None:
            current.append(char)
            if char == "\\" and quote == '"' and index + 1 < len(command):
                index += 1
                current.append(command[index])
            elif char == quote:
                quote = None
        elif char in "\"'":
            quote = char
            current.append(char)
        elif char == "&" and command[index : index + 2] == "&&":
            parts.append("".join(current))
            current = []
            index += 1
        else:
            current.append(char)
        index += 1
    parts.append("".join(current))
    return [part.strip() for part in parts if part.strip()]


def _is_record_audit(command: str) -> bool:
    return RECORD_AUDIT_MARKER in _normalise_command(command)


def _task_blocks(task_id: int) -> list[str]:
    """Collect records of a task, including its ``子任务 NN-<phase>`` blocks."""
    prefix = f"子任务 {task_id:02d}"
    return [
        block
        for block in _record_blocks()
        if (label := _record_value(block, "子任务/门禁")) is not None
        and (
            label == prefix
            or label.startswith(prefix + " /")
            or label.startswith(prefix + ":")
            or label.startswith(prefix + "-")
        )
    ]


def _evidence_path(value: str) -> Path:
    raw = value.strip().strip("`").strip()
    expanded = os.path.expanduser(os.path.expandvars(raw))
    path = Path(expanded)
    return path if path.is_absolute() else REPO_ROOT / expanded


def _assert_evidence(values: dict[str, str]) -> None:
    evidence = values["证据文件"]
    assert evidence != "无", evidence
    evidence_path = _evidence_path(evidence)
    assert evidence_path.is_file(), evidence_path


def _validate_record_block(block: str, *, require_evidence: bool) -> dict[str, str]:
    values: dict[str, str] = {}
    for field in RECORD_FIELDS:
        value = _record_value(block, field)
        assert value, field
        values[field] = value
    assert re.fullmatch(r"-?\d+", values["退出码"]), values["退出码"]
    assert _normalise_command(values["命令"]) != "同上", values["命令"]
    if require_evidence:
        _assert_evidence(values)
    return values


def _assert_progress_template() -> None:
    text = PROGRESS.read_text(encoding="utf-8")
    assert "### 验证记录：<子任务或门禁>" in text
    for field in RECORD_FIELDS:
        assert f"**{field}**" in text, field
    assert "实际退出码" in text
    assert "原始输出" in text


def _assert_progress_records(*, require_task: int | None = None) -> None:
    blocks = _record_blocks()
    assert blocks, "no verification record blocks"
    for block in blocks:
        _validate_record_block(block, require_evidence=False)
    if require_task is not None:
        _assert_task_records(require_task)


def _phase_blocks(label: str) -> list[str]:
    expected = f"{PHASE_LABEL_PREFIX}{label}"
    return [
        block
        for block in _record_blocks()
        if _record_value(block, "子任务/门禁") == expected
    ]


def _assert_phase_records(label: str) -> None:
    blocks = _phase_blocks(label)
    assert blocks, f"missing progress record for {label}"
    values = [
        _validate_record_block(block, require_evidence=False) for block in blocks
    ]
    marker = PHASE_SCAN_MARKERS[label]
    other_markers = [
        other for key, other in PHASE_SCAN_MARKERS.items() if key != label
    ]
    for value in values:
        command = _normalise_command(value["命令"])
        assert all(other not in command for other in other_markers), (label, command)
    successful = [value for value in values if value["退出码"] == "0"]
    assert successful, (label, [value["退出码"] for value in values])
    marked = [
        value
        for value in successful
        if marker in _normalise_command(value["命令"])
    ]
    assert marked, (label, marker)
    for value in marked:
        _assert_evidence(value)


def _assert_task_records(
    task_id: int, *, rows: list[dict[str, str]] | None = None
) -> None:
    blocks = _task_blocks(task_id)
    assert blocks, task_id
    values = [
        _validate_record_block(block, require_evidence=False) for block in blocks
    ]
    successful = [value for value in values if value["退出码"] == "0"]
    for value in successful:
        _assert_evidence(value)
    actual_commands = [_normalise_command(value["命令"]) for value in successful]
    for row in _task_rows(task_id) if rows is None else rows:
        for atomic in _split_atomic_commands(row["validation_command"]):
            if _is_record_audit(atomic):
                continue
            expected = _normalise_command(atomic)
            assert expected in actual_commands, (task_id, expected)


def _assert_task_done(task_id: int) -> None:
    rows = _task_rows(task_id)
    assert rows and all(row["status"] == "DONE" for row in rows), task_id
    assert all(
        row["completed_at"]
        and row["notes"]
        and row["acceptance_criteria"]
        and row["validation_command"]
        for row in rows
    ), task_id


def _assert_preclose_state() -> None:
    """Pre-close check: 01-04 DONE and 05 steps 1-4 DONE with real evidence.

    The last 05 step is still open and therefore needs no self-attestation yet.
    """
    _assert_matrix()
    subtasks = _rows(SUBTASKS)
    assert [row["status"] for row in subtasks[:4]] == ["DONE"] * 4
    assert subtasks[4]["status"] in {"TODO", "IN_PROGRESS"}
    for task_id in range(1, 5):
        _assert_task_done(task_id)
        _assert_task_records(task_id)
    rows = _task_rows(5)
    assert len(rows) == 5, 5
    done_rows = rows[:4]
    assert all(row["status"] == "DONE" for row in done_rows), 5
    assert all(row["status"] in {"TODO", "IN_PROGRESS"} for row in rows[4:]), 5
    assert all(
        row["completed_at"]
        and row["notes"]
        and row["acceptance_criteria"]
        and row["validation_command"]
        for row in done_rows
    ), 5
    _assert_task_records(5, rows=done_rows)
    for label in PHASE_RECORD_LABELS:
        _assert_phase_records(label)


def _assert_final_state() -> None:
    """Post-hoc full state and evidence verification, run after statuses close."""
    _assert_matrix()
    subtasks = _rows(SUBTASKS)
    assert all(row["status"] == "DONE" for row in subtasks)
    assert all(
        row["completed_at"] and row["notes"] and row["validation_command"]
        for row in subtasks
    )
    assert "- **状态**：DONE" in EPIC.read_text(encoding="utf-8")
    for task_id in range(1, 6):
        _assert_task_done(task_id)
        _assert_task_records(task_id)
    for label in PHASE_RECORD_LABELS:
        _assert_phase_records(label)


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--matrix", action="store_true")
    group.add_argument("--non-vehicle-matrix", action="store_true")
    group.add_argument("--planning", action="store_true")
    group.add_argument("--progress-template", action="store_true")
    group.add_argument("--progress-records")
    group.add_argument("--final-preclose", action="store_true")
    group.add_argument("--final", action="store_true")
    args = parser.parse_args()

    if args.matrix:
        _assert_matrix()
    elif args.non_vehicle_matrix:
        _assert_matrix(non_vehicle_only=True)
    elif args.planning:
        _assert_matrix()
        _assert_planning_files()
    elif args.progress_template:
        _assert_progress_template()
    elif args.progress_records:
        label = args.progress_records
        if label.startswith(PHASE_LABEL_PREFIX):
            label = label[len(PHASE_LABEL_PREFIX) :]
        if label == "planning":
            _assert_progress_records()
        elif label in PHASE_RECORD_LABELS:
            _assert_phase_records(label)
        else:
            _assert_progress_records(require_task=int(args.progress_records))
    elif args.final_preclose:
        _assert_preclose_state()
    elif args.final:
        _assert_final_state()
    print("planning_contract_scan=pass")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
