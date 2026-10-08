"""Run only the declared Taskmaster checks and retain each command's exit code."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shlex
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


TASK_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = TASK_ROOT.parents[1]
UV = ["uv", "run", "--no-sync"]

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
STRUCTURAL = [
    [*UV, "ruff", "check", "."],
    [*UV, "ty", "check", "."],
    [*UV, "python", "packages/suspension_multibody/tests/architecture/legacy_surface_gate.py", "--check"],
    [*UV, "python", "packages/suspension_kernel/scripts/check_module_layering.py", "--strict", "--final"],
    [*UV, "python", "packages/suspension_multibody/scripts/check_composable_release.py", "--skip-isolation"],
]
NUMERIC = [
    [*UV, "python", "packages/suspension_multibody/scripts/dynamic_hash_sentinel.py", "--check"],
    [*UV, "python", "packages/suspension_multibody/scripts/case_parity_check.py"],
    [*UV, "python", "packages/suspension_multibody/scripts/kc_perf_gate.py"],
]


def child_commands(identity: str) -> list[list[str]]:
    with (TASK_ROOT / "SUBTASKS.csv").open(encoding="utf-8", newline="") as handle:
        child = next(row for row in csv.DictReader(handle) if row["id"] == identity)
    with (TASK_ROOT / child["task_dir"] / "TODO.csv").open(encoding="utf-8", newline="") as handle:
        return [shlex.split(row["validation_command"]) for row in csv.DictReader(handle)]


def run_step(identity: str, step: int, command: list[str]) -> None:
    directory = TASK_ROOT / "raw" / "validation"
    directory.mkdir(parents=True, exist_ok=True)
    started = datetime.now(timezone.utc).isoformat()
    result = subprocess.run(command, cwd=REPO_ROOT, capture_output=True, text=True, encoding="utf-8", errors="replace", check=False)
    logfile = directory / f"child-{identity}-step-{step}.log"
    logfile.write_text(result.stdout + result.stderr, encoding="utf-8")
    evidence = {
        "command": command,
        "returncode": result.returncode,
        "started_at": started,
        "finished_at": datetime.now(timezone.utc).isoformat(),
        "log": str(logfile.relative_to(TASK_ROOT)),
        "log_sha256": hashlib.sha256(logfile.read_bytes()).hexdigest(),
    }
    (directory / f"child-{identity}-step-{step}.json").write_text(json.dumps(evidence, ensure_ascii=False, indent=2), encoding="utf-8")
    print(result.stdout, end="")
    print(result.stderr, end="")
    if result.returncode:
        raise SystemExit(result.returncode)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("stage", choices=["step", "structural", "numeric", "closeout", *[f"child-{i}" for i in range(1, 11)]])
    parser.add_argument("child", nargs="?", type=int)
    parser.add_argument("step", nargs="?", type=int)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if args.stage == "step":
        if args.child not in range(1, 11) or args.step is None:
            parser.error("step requires a child ID and a step ID")
        commands = child_commands(str(args.child))
        if args.step not in range(1, len(commands) + 1):
            parser.error("invalid step ID")
        commands = [commands[args.step - 1]]
    elif args.stage.startswith("child-"):
        commands = child_commands(args.stage.removeprefix("child-"))
    elif args.stage == "numeric":
        commands = NUMERIC
    elif args.stage == "structural":
        commands = STRUCTURAL
    else:
        commands = [*STRUCTURAL, ["git", "diff", "--check"]]
    if args.dry_run:
        print(json.dumps(commands, ensure_ascii=False, indent=2))
        return

    if args.stage == "step" or args.stage.startswith("child-"):
        identity = str(args.child) if args.stage == "step" else args.stage.removeprefix("child-")
        for index, command in enumerate(commands, 1):
            step = args.step if args.stage == "step" else index
            print(shlex.join(command), flush=True)
            run_step(identity, step, command)
        print(f"PASS: {args.stage}; {len(commands)} leaf command(s) with evidence")
        return

    evidence: list[dict[str, object]] = []
    for index, command in enumerate(commands, 1):
        print(shlex.join(command), flush=True)
        result = subprocess.run(command, cwd=REPO_ROOT, capture_output=True, text=True, encoding="utf-8", errors="replace", check=False)
        logfile = TASK_ROOT / "raw" / f"{args.stage}-{index}.log"
        logfile.write_text(result.stdout + result.stderr, encoding="utf-8")
        print(result.stdout, end="")
        print(result.stderr, end="")
        evidence.append({"command": command, "returncode": result.returncode, "log": str(logfile.relative_to(TASK_ROOT))})
        (TASK_ROOT / "raw" / f"{args.stage}.json").write_text(json.dumps(evidence, ensure_ascii=False, indent=2), encoding="utf-8")
        if result.returncode:
            raise SystemExit(result.returncode)
    print(f"PASS: {args.stage}; {len(commands)} declared checks")


if __name__ == "__main__":
    main()
