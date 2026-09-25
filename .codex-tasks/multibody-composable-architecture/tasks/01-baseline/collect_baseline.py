#!/usr/bin/env python
"""Collect the frozen baseline evidence for subtask 01.

Reads the workspace (hashes, native build metadata, existing baselines) and
writes ``BASELINE.json`` next to this task's records.  It does **not** modify
source, tests, or baselines: it only observes.

Run from the repository root:

    uv run --no-sync python <this script>

The produced file is the input of
``packages/suspension_multibody/scripts/check_composable_baseline.py --check``.
"""

from __future__ import annotations

import hashlib
import json
import os
import platform
import subprocess
import sys
from pathlib import Path


def _repository_root() -> Path:
    """Locate the workspace root without relying on this file's path."""
    completed = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        capture_output=True,
        text=True,
        encoding="utf-8",
        check=True,
    )
    return Path(os.path.normpath(completed.stdout.strip()))


ROOT = _repository_root()
TASK_DIR = Path(__file__).resolve().parent
KERNEL_NATIVE = ROOT / "packages/suspension_kernel/src/suspension_kernel/native"
MULTIBODY_NATIVE = ROOT / "packages/suspension_multibody/src/suspension_multibody/native"

LIBRARY_NAME = "suspension_kernel.dll"
MULTIBODY_DATA = ROOT / "packages/suspension_multibody/tests/data"
KERNEL_CPP = ROOT / "packages/suspension_kernel/cpp"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git(*args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=ROOT, capture_output=True, text=True, check=True
    ).stdout.strip()


def tree_fingerprint(directory: Path, suffixes: tuple[str, ...]) -> dict[str, object]:
    """Hash every file under ``directory`` with one of ``suffixes``."""
    files = sorted(
        path
        for path in directory.rglob("*")
        if path.is_file() and path.suffix in suffixes
    )
    per_file = {
        str(path.relative_to(ROOT)).replace("\\", "/"): sha256(path) for path in files
    }
    combined = hashlib.sha256()
    for name, digest in sorted(per_file.items()):
        combined.update(f"{name}:{digest}\n".encode())
    return {"count": len(files), "combined_sha256": combined.hexdigest(), "files": per_file}


def load_json(path: Path) -> object:
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> int:
    kernel_library = KERNEL_NATIVE / LIBRARY_NAME
    mirror_library = MULTIBODY_NATIVE / LIBRARY_NAME

    evidence: dict[str, object] = {}

    evidence["environment"] = {
        "platform": platform.platform(),
        "python": platform.python_version(),
        "processor": platform.processor(),
        "git_head": git("rev-parse", "HEAD"),
        "git_head_short": git("rev-parse", "--short", "HEAD"),
        "git_head_date": git("log", "-1", "--format=%cI"),
        "git_status_porcelain": git("status", "--porcelain"),
    }

    evidence["native"] = {
        "canonical_library": {
            "path": str(kernel_library.relative_to(ROOT)).replace("\\", "/"),
            "sha256": sha256(kernel_library),
            "size": kernel_library.stat().st_size,
        },
        "mirror_library": {
            "path": str(mirror_library.relative_to(ROOT)).replace("\\", "/"),
            "sha256": sha256(mirror_library),
            "size": mirror_library.stat().st_size,
        },
        "mirrors_identical": sha256(kernel_library) == sha256(mirror_library),
        "mirror_freshness_rule": (
            "packages/suspension_multibody/src/suspension_multibody/kernel/native.py:84 "
            "require_fresh_mirror refuses to load a mirror that is older than and "
            "byte-different from the canonical kernel library; "
            "the repair is `uv run python "
            "packages/suspension_multibody/scripts/build_axle_native.py`"
        ),
        "build_metadata": load_json(KERNEL_NATIVE / "native_build.json"),
    }

    evidence["sources"] = {
        "kernel_cpp": tree_fingerprint(KERNEL_CPP, (".cpp", ".hpp", ".h", ".txt")),
    }

    evidence["frozen_baselines"] = {
        "kc_snapshot_manifest": load_json(MULTIBODY_DATA / "kc_baseline/manifest.json"),
        "kc_snapshot_k_states_sha256": sha256(MULTIBODY_DATA / "kc_baseline/k_states.json"),
        "kc_snapshot_c_states_sha256": sha256(MULTIBODY_DATA / "kc_baseline/c_states.json"),
        # Digests of the frozen files themselves, so the gate can detect a
        # baseline that was edited after collection rather than trusting the
        # parsed contents alone.
        "dynamic_hash_baseline_sha256": sha256(MULTIBODY_DATA / "dynamic_hash_baseline.json"),
        "kc_perf_baseline_native_sha256": sha256(
            MULTIBODY_DATA / "kc_perf_baseline_native.json"
        ),
        "dynamic_hash_baseline": load_json(MULTIBODY_DATA / "dynamic_hash_baseline.json"),
        "kc_perf_baseline_native": load_json(MULTIBODY_DATA / "kc_perf_baseline_native.json"),
    }

    destination = TASK_DIR / "BASELINE.json"
    destination.write_text(
        json.dumps(evidence, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"wrote {destination}")
    print(f"native mirrors identical: {evidence['native']['mirrors_identical']}")
    print(f"kernel cpp fingerprint   : {evidence['sources']['kernel_cpp']['combined_sha256'][:16]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
