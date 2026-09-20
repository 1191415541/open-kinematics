"""Validate the durable planning and evidence contracts for this Epic."""

from __future__ import annotations

import argparse
import csv
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
        assert all(row["acceptance_criteria"] and row["validation_command"] for row in rows), task_id


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


def _task_blocks(task_id: int) -> list[str]:
    prefix = f"子任务 {task_id:02d}"
    return [
        block
        for block in _record_blocks()
        if (
            (label := _record_value(block, "子任务/门禁")) == prefix
            or (label is not None and label.startswith(prefix + " /"))
            or (label is not None and label.startswith(prefix + ":"))
        )
    ]


def _validate_record_block(block: str, *, require_evidence: bool) -> dict[str, str]:
    values: dict[str, str] = {}
    for field in RECORD_FIELDS:
        value = _record_value(block, field)
        assert value, field
        values[field] = value
    assert re.fullmatch(r"-?\d+", values["退出码"]), values["退出码"]
    assert _normalise_command(values["命令"]) != "同上", values["命令"]
    if require_evidence:
        evidence = values["证据文件"]
        assert evidence != "无", evidence
        evidence_path = REPO_ROOT / evidence.strip("`").strip()
        assert evidence_path.is_file(), evidence_path
    return values


def _assert_progress_template() -> None:
    text = PROGRESS.read_text(encoding="utf-8")
    assert "### 验证记录：<子任务或门禁>" in text
    for field in RECORD_FIELDS:
        assert f"**{field}**" in text, field
    assert "实际退出码" in text
    assert "原始输出" in text


def _assert_progress_records(
    *, require_task: int | None = None, require_evidence: bool = False
) -> None:
    blocks = _record_blocks()
    assert blocks, "no verification record blocks"
    for block in blocks:
        _validate_record_block(block, require_evidence=require_evidence)
    if require_task is not None:
        task_blocks = _task_blocks(require_task)
        assert task_blocks, require_task
        for block in task_blocks:
            _validate_record_block(block, require_evidence=require_evidence)


def _assert_task_records(task_id: int) -> None:
    blocks = _task_blocks(task_id)
    assert blocks, task_id
    values = [
        _validate_record_block(block, require_evidence=True)
        for block in blocks
    ]
    actual_commands = [_normalise_command(value["命令"]) for value in values]
    for row in _task_rows(task_id):
        expected = _normalise_command(row["validation_command"])
        assert any(expected in command for command in actual_commands), (task_id, expected)


def _assert_preclose_state() -> None:
    _assert_matrix()
    subtasks = _rows(SUBTASKS)
    assert [row["status"] for row in subtasks[:4]] == ["DONE"] * 4
    assert subtasks[4]["status"] in {"TODO", "IN_PROGRESS"}
    for task_id in range(1, 5):
        rows = _task_rows(task_id)
        assert rows and all(row["status"] == "DONE" for row in rows), task_id
        _assert_task_records(task_id)


def _assert_final_state() -> None:
    _assert_matrix()
    subtasks = _rows(SUBTASKS)
    assert all(row["status"] == "DONE" for row in subtasks)
    assert all(
        row["completed_at"] and row["notes"] and row["validation_command"]
        for row in subtasks
    )
    assert "- **状态**：DONE" in EPIC.read_text(encoding="utf-8")
    for task_id in range(1, 6):
        rows = _task_rows(task_id)
        assert rows and all(row["status"] == "DONE" for row in rows), task_id
        assert all(
            row["completed_at"]
            and row["notes"]
            and row["acceptance_criteria"]
            and row["validation_command"]
            for row in rows
        ), task_id
        _assert_task_records(task_id)


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
        if args.progress_records == "planning":
            _assert_progress_records()
        else:
            _assert_progress_records(require_task=int(args.progress_records), require_evidence=True)
    elif args.final_preclose:
        _assert_preclose_state()
    elif args.final:
        _assert_final_state()
    print("planning_contract_scan=pass")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
