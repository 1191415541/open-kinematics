"""Regression tests for the planning/evidence gate semantics.

Standard library only (unittest). The tests synthesize a planning fixture tree in
the scratch/temp directory and patch ``planning_contract_scan`` globals onto it;
no real execution records are written into the Epic PROGRESS.

Covered contracts:

* a task ``validation_command`` ``&&`` chain is split into atomic commands,
  ignoring ``&&`` inside quotes;
* every functional/static atomic command needs its own successful, evidenced
  record of the same task;
* missing record, failed record, missing evidence and whole-chain-only records
  are rejected;
* record-audit commands (``planning_contract_scan.py``) are never required as
  execution evidence;
* phase labels ``子任务 04-pre-delete`` / ``子任务 04-post-delete`` are
  recognised by ``_task_blocks`` and never mixed between phases;
* ``--final-preclose`` needs no self-attestation of the closing step while
  ``--final`` still rejects missing functional evidence.
"""

from __future__ import annotations

import contextlib
import csv
import importlib.util
import io
import os
import sys
import tempfile
import unittest
from collections.abc import Sequence
from pathlib import Path
from typing import Any, cast
from unittest import mock

HERE = Path(__file__).resolve().parent
SCRIPT = HERE / "planning_contract_scan.py"

_SPEC = importlib.util.spec_from_file_location("planning_contract_scan", SCRIPT)
assert _SPEC is not None and _SPEC.loader is not None
scan = cast(Any, importlib.util.module_from_spec(_SPEC))
_SPEC.loader.exec_module(scan)

TODO_HEADER = (
    "id",
    "task",
    "status",
    "acceptance_criteria",
    "validation_command",
    "completed_at",
    "retry_count",
    "notes",
)


def _run_main(argv: list[str]) -> int:
    with contextlib.redirect_stdout(io.StringIO()):
        with mock.patch.object(sys, "argv", argv):
            return scan.main()
SUBTASK_HEADER = (
    "id",
    "task",
    "status",
    "task_type",
    "depends_on",
    "task_dir",
    "acceptance_criteria",
    "validation_command",
    "completed_at",
    "retry_count",
    "notes",
)
SCAN = "python .codex-tasks/epic/tasks/05-final-validation/planning_contract_scan.py"
PRE_DELETE = (
    "uv run --package suspension-multibody python "
    ".codex-tasks/epic/tasks/04-delete-legacy/legacy_reference_scan.py --pre-delete"
)
POST_DELETE = (
    "uv run --package suspension-multibody python "
    ".codex-tasks/epic/tasks/04-delete-legacy/legacy_reference_scan.py --post-delete"
)
MATRIX_ROWS = (
    ('axle', 'axle_dynamic'),
    ('axle', 'kc_quasi_static'),
    ('vehicle', 'vehicle_kc'),
    ('vehicle', 'handling'),
    ('vehicle', 'ride_four_post'),
    ('vehicle', 'ride_random_road'),
    ('vehicle', 'vehicle_dynamic'),
)


def _write_csv(path: Path, header: tuple[str, ...], rows: list[dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(header))
        writer.writeheader()
        writer.writerows(rows)


def _todo_row(index: int, command: str, status: str = "TODO") -> dict[str, str]:
    done = status == "DONE"
    return {
        "id": str(index),
        "task": f"fixture row {index}",
        "status": status,
        "acceptance_criteria": "fixture acceptance",
        "validation_command": command,
        "completed_at": "2026-09-20" if done else "",
        "retry_count": "0",
        "notes": "fixture note" if done else "",
    }


def _record_block(label: str, command: str, exit_code: int, evidence: str) -> str:
    return (
        f"### 验证记录：{label}\n"
        "- **时间**：2026-09-20\n"
        f"- **子任务/门禁**：{label}\n"
        f"- **命令**：{command}\n"
        f"- **退出码**：{exit_code}\n"
        "- **摘要**：fixture\n"
        f"- **证据文件**：{evidence}\n"
    )


def _matrix_text() -> str:
    lines = [
        "# Preparation fixture matrix",
        "",
        "context keys produced / compiler consumer / document owner / Legacy `prepared` 适配",
        "",
        "| registry key | preparation module | preparation type | domain inputs | context keys produced | compiler consumer | document owner | registry registration | primary tests |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for assembly, family in MATRIX_ROWS:
        lines.append(
            f'| `("{assembly}", "{family}")` | '
            f"suspension_multibody.preparation.{family} | T | D | C | CC | O | R | P |"
        )
    return "\n".join(lines) + "\n"


def build_fixture(
    root: Path,
    *,
    task_commands: dict[int, list[str]],
    task_statuses: dict[int, list[str]] | None = None,
    subtask_statuses: list[str] | None = None,
    epic_status: str = "TODO",
    records: Sequence[tuple[str, str, int, str | None]] = (),
) -> None:
    """Write a minimal planning tree satisfying the gate file layout."""
    task_statuses = task_statuses or {
        task_id: ["TODO"] * len(commands) for task_id, commands in task_commands.items()
    }
    subtask_statuses = subtask_statuses or ["TODO"] * 5
    (root / "EPIC.md").write_text(
        "# fixture epic\n\n- **状态**：%s\n" % epic_status, encoding="utf-8"
    )
    (root / "PREPARATION_MATRIX.md").write_text(_matrix_text(), encoding="utf-8")

    subtask_rows = []
    for task_id in range(1, 6):
        status = subtask_statuses[task_id - 1]
        done = status == "DONE"
        subtask_rows.append(
            {
                "id": str(task_id),
                "task": f"fixture subtask {task_id}",
                "status": status,
                "task_type": "single-full",
                "depends_on": "" if task_id == 1 else str(task_id - 1),
                "task_dir": f"tasks/{task_id:02d}-fixture",
                "acceptance_criteria": "fixture acceptance",
                "validation_command": "fixture command",
                "completed_at": "2026-09-20" if done else "",
                "retry_count": "0",
                "notes": "fixture note" if done else "",
            }
        )
    _write_csv(root / "SUBTASKS.csv", SUBTASK_HEADER, subtask_rows)

    for task_id, commands in task_commands.items():
        rows = [
            _todo_row(index, command, task_statuses[task_id][index - 1])
            for index, command in enumerate(commands, start=1)
        ]
        _write_csv(root / "tasks" / f"{task_id:02d}-fixture" / "TODO.csv", TODO_HEADER, rows)

    evidence_dir = root / "evidence"
    evidence_dir.mkdir(parents=True, exist_ok=True)
    blocks = ["### 验证记录：<子任务或门禁>\n"]
    for index, (label, command, exit_code, evidence_name) in enumerate(records, start=1):
        if evidence_name is None:
            evidence = "无"
        else:
            evidence_file = evidence_dir / evidence_name
            evidence_file.write_text("fixture evidence\n", encoding="utf-8")
            evidence = str(evidence_file)
        blocks.append(_record_block(label, command, exit_code, evidence))
    (root / "PROGRESS.md").write_text("\n".join(blocks), encoding="utf-8")


class FixtureTestCase(unittest.TestCase):
    def setUp(self) -> None:
        scratch = os.environ.get("PI_SCRATCH_DIR")
        base = Path(scratch) if scratch and Path(scratch).is_dir() else None
        self._tmp = tempfile.TemporaryDirectory(dir=base)
        self.root = Path(self._tmp.name)
        self._originals = {
            name: getattr(scan, name)
            for name in (
                "EPIC_ROOT",
                "REPO_ROOT",
                "MATRIX",
                "EPIC",
                "SUBTASKS",
                "PROGRESS",
                "TASKS",
            )
        }
        scan.EPIC_ROOT = self.root
        scan.REPO_ROOT = self.root
        scan.MATRIX = self.root / "PREPARATION_MATRIX.md"
        scan.EPIC = self.root / "EPIC.md"
        scan.SUBTASKS = self.root / "SUBTASKS.csv"
        scan.PROGRESS = self.root / "PROGRESS.md"
        scan.TASKS = self.root / "tasks"

    def tearDown(self) -> None:
        for name, value in self._originals.items():
            setattr(scan, name, value)
        self._tmp.cleanup()

    def build(
        self,
        *,
        task_commands: dict[int, list[str]],
        task_statuses: dict[int, list[str]] | None = None,
        subtask_statuses: list[str] | None = None,
        epic_status: str = "TODO",
        records: Sequence[tuple[str, str, int, str | None]] = (),
    ) -> None:
        build_fixture(
            self.root,
            task_commands=task_commands,
            task_statuses=task_statuses,
            subtask_statuses=subtask_statuses,
            epic_status=epic_status,
            records=records,
        )


class AtomicSplitTests(unittest.TestCase):
    def test_quoted_and_is_not_split(self) -> None:
        command = (
            "python -c \"print('a && b')\" && git diff --check && uv run pytest -q"
        )
        self.assertEqual(
            scan._split_atomic_commands(command),
            [
                "python -c \"print('a && b')\"",
                "git diff --check",
                "uv run pytest -q",
            ],
        )

    def test_single_command_stays_single(self) -> None:
        self.assertEqual(scan._split_atomic_commands("git diff --check"), ["git diff --check"])

    def test_record_audit_detection(self) -> None:
        self.assertTrue(scan._is_record_audit(f"{SCAN} --progress-records 04-pre-delete"))
        self.assertFalse(scan._is_record_audit("git diff --check"))


class TaskRecordTests(FixtureTestCase):
    def test_valid_per_command_records_pass(self) -> None:
        self.build(
            task_commands={1: ["uv run pytest -q && git diff --check"]},
            records=[
                ("子任务 01", "uv run pytest -q", 0, "one.log"),
                ("子任务 01", "git diff --check", 0, "two.log"),
            ],
        )
        scan._assert_task_records(1)

    def test_missing_record_rejected(self) -> None:
        self.build(
            task_commands={1: ["uv run pytest -q && git diff --check"]},
            records=[("子任务 01", "uv run pytest -q", 0, "one.log")],
        )
        with self.assertRaises(AssertionError):
            scan._assert_task_records(1)

    def test_failed_exit_code_rejected(self) -> None:
        self.build(
            task_commands={1: ["uv run pytest -q && git diff --check"]},
            records=[
                ("子任务 01", "uv run pytest -q", 0, "one.log"),
                ("子任务 01", "git diff --check", 1, "two.log"),
            ],
        )
        with self.assertRaises(AssertionError):
            scan._assert_task_records(1)

    def test_missing_evidence_rejected(self) -> None:
        self.build(
            task_commands={1: ["git diff --check"]},
            records=[("子任务 01", "git diff --check", 0, None)],
        )
        with self.assertRaises(AssertionError):
            scan._assert_task_records(1)

    def test_whole_chain_record_rejected(self) -> None:
        self.build(
            task_commands={1: ["uv run pytest -q && git diff --check"]},
            records=[
                ("子任务 01", "uv run pytest -q && git diff --check", 0, "chain.log"),
            ],
        )
        with self.assertRaises(AssertionError):
            scan._assert_task_records(1)

    def test_quoted_chain_records_pass(self) -> None:
        command = "python -c \"assert '&&'\" && git diff --check"
        self.build(
            task_commands={1: [command]},
            records=[
                ("子任务 01", "python -c \"assert '&&'\"", 0, "quoted.log"),
                ("子任务 01", "git diff --check", 0, "diff.log"),
            ],
        )
        scan._assert_task_records(1)

    def test_record_audit_command_needs_no_evidence(self) -> None:
        self.build(
            task_commands={1: [f"git diff --check && {SCAN} --progress-records 01"]},
            records=[("子任务 01", "git diff --check", 0, "diff.log")],
        )
        scan._assert_task_records(1)

    def test_relative_evidence_path_supported(self) -> None:
        self.build(task_commands={1: ["git diff --check"]})
        evidence = self.root / "raw" / "diff.log"
        evidence.parent.mkdir(parents=True, exist_ok=True)
        evidence.write_text("relative evidence\n", encoding="utf-8")
        progress = (self.root / "PROGRESS.md").read_text(encoding="utf-8")
        progress += _record_block("子任务 01", "git diff --check", 0, "raw/diff.log")
        (self.root / "PROGRESS.md").write_text(progress, encoding="utf-8")
        scan._assert_task_records(1)


class PhaseLabelTests(FixtureTestCase):
    def _build_phases(self, extra_records: Sequence[tuple[str, str, int, str | None]] = ()) -> None:
        self.build(
            task_commands={4: [PRE_DELETE, POST_DELETE]},
            records=[
                ("子任务 04-pre-delete", PRE_DELETE, 0, "pre.log"),
                ("子任务 04-post-delete", POST_DELETE, 0, "post.log"),
                *extra_records,
            ],
        )

    def test_phase_labels_are_recognised(self) -> None:
        self._build_phases()
        labels = {
            scan._record_value(block, "子任务/门禁") for block in scan._task_blocks(4)
        }
        self.assertEqual(labels, {"子任务 04-pre-delete", "子任务 04-post-delete"})
        self.assertEqual(len(scan._phase_blocks("04-pre-delete")), 1)
        self.assertEqual(len(scan._phase_blocks("04-post-delete")), 1)
        scan._assert_task_records(4)
        scan._assert_phase_records("04-pre-delete")
        scan._assert_phase_records("04-post-delete")

    def test_pre_post_are_not_mixed(self) -> None:
        self._build_phases(
            extra_records=[("子任务 04-pre-delete", POST_DELETE, 0, "mixed.log")]
        )
        with self.assertRaises(AssertionError):
            scan._assert_phase_records("04-pre-delete")

    def test_phase_requires_real_evidence(self) -> None:
        self.build(
            task_commands={4: [PRE_DELETE]},
            records=[("子任务 04-pre-delete", PRE_DELETE, 0, None)],
        )
        with self.assertRaises(AssertionError):
            scan._assert_phase_records("04-pre-delete")

    def test_phase_requires_successful_scan_record(self) -> None:
        self.build(
            task_commands={4: [PRE_DELETE]},
            records=[("子任务 04-pre-delete", PRE_DELETE, 1, "pre.log")],
        )
        with self.assertRaises(AssertionError):
            scan._assert_phase_records("04-pre-delete")


FINAL_STEP = f"{SCAN} --final-preclose && git diff --check"


def _closing_fixture(
    root: Path, *, final: bool, include_diff_record: bool = True
) -> None:
    task_commands = {
        1: ["cmd one"],
        2: ["cmd two"],
        3: ["cmd three"],
        4: [PRE_DELETE, POST_DELETE],
        5: ["cmd five 1", "cmd five 2", "cmd five 3", "cmd five 4", FINAL_STEP],
    }
    if final:
        task_statuses = {task_id: ["DONE"] * len(cmds) for task_id, cmds in task_commands.items()}
        subtask_statuses = ["DONE"] * 5
        epic_status = "DONE"
    else:
        task_statuses = {
            task_id: ["DONE"] * len(cmds) for task_id, cmds in task_commands.items()
        }
        task_statuses[5] = ["DONE"] * 4 + ["TODO"]
        subtask_statuses = ["DONE", "DONE", "DONE", "DONE", "TODO"]
        epic_status = "TODO"
    records: list[tuple[str, str, int, str | None]] = [
        ("子任务 01", "cmd one", 0, "t1.log"),
        ("子任务 02", "cmd two", 0, "t2.log"),
        ("子任务 03", "cmd three", 0, "t3.log"),
        ("子任务 04-pre-delete", PRE_DELETE, 0, "pre.log"),
        ("子任务 04-post-delete", POST_DELETE, 0, "post.log"),
        ("子任务 05", "cmd five 1", 0, "t5a.log"),
        ("子任务 05", "cmd five 2", 0, "t5b.log"),
        ("子任务 05", "cmd five 3", 0, "t5c.log"),
        ("子任务 05", "cmd five 4", 0, "t5d.log"),
    ]
    if include_diff_record:
        records.append(("子任务 05", "git diff --check", 0, "t5e.log"))
    build_fixture(
        root,
        task_commands=task_commands,
        task_statuses=task_statuses,
        subtask_statuses=subtask_statuses,
        epic_status=epic_status,
        records=records,
    )


class PrecloseAndFinalTests(FixtureTestCase):
    def test_preclose_needs_no_self_attestation(self) -> None:
        _closing_fixture(self.root, final=False, include_diff_record=False)
        scan._assert_preclose_state()

    def test_preclose_cli_routes(self) -> None:
        _closing_fixture(self.root, final=False, include_diff_record=False)
        self.assertEqual(_run_main(["planning_contract_scan.py", "--final-preclose"]), 0)

    def test_final_accepts_full_state_with_evidence(self) -> None:
        _closing_fixture(self.root, final=True, include_diff_record=True)
        scan._assert_final_state()

    def test_final_rejects_missing_functional_evidence(self) -> None:
        _closing_fixture(self.root, final=True, include_diff_record=False)
        with self.assertRaises(AssertionError):
            scan._assert_final_state()

    def test_final_cli_routes(self) -> None:
        _closing_fixture(self.root, final=True, include_diff_record=True)
        self.assertEqual(_run_main(["planning_contract_scan.py", "--final"]), 0)

    def test_final_rejects_open_subtask(self) -> None:
        _closing_fixture(self.root, final=True, include_diff_record=True)
        subtasks = (self.root / "SUBTASKS.csv").read_text(encoding="utf-8")
        (self.root / "SUBTASKS.csv").write_text(
            subtasks.replace("fixture subtask 4,DONE", "fixture subtask 4,TODO", 1),
            encoding="utf-8",
        )
        with self.assertRaises(AssertionError):
            scan._assert_final_state()


class PhaseCliTests(FixtureTestCase):
    def test_progress_records_accepts_prefixed_phase_label(self) -> None:
        self.build(
            task_commands={4: [PRE_DELETE]},
            records=[("子任务 04-pre-delete", PRE_DELETE, 0, "pre.log")],
        )
        argv = ["planning_contract_scan.py", "--progress-records", "子任务 04-pre-delete"]
        self.assertEqual(_run_main(argv), 0)

    def test_progress_records_rejects_unknown_label(self) -> None:
        self.build(task_commands={1: ["cmd one"]})
        argv = ["planning_contract_scan.py", "--progress-records", "04-pre-delete"]
        with self.assertRaises(AssertionError):
            _run_main(argv)


if __name__ == "__main__":
    unittest.main()
