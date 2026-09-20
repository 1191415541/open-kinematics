"""Validate that the legacy vehicle_dynamics module is no longer referenced."""

from __future__ import annotations

import argparse
from pathlib import Path

TARGET = Path("packages/suspension_multibody/src/suspension_multibody/vehicle_dynamics.py")
ROOTS = (
    Path("packages"),
    Path("docs"),
    Path(".github"),
    Path("scripts"),
    Path("README.md"),
    Path("pyproject.toml"),
    Path("justfile"),
)
EXCLUDED_DIR_NAMES = {".codex-tasks", ".git", ".pytest_cache", ".ruff_cache", ".mypy_cache", ".venv", ".tox", ".nox", "__pycache__", "build", "dist", "htmlcov", "node_modules", "site-packages", ".egg-info"}
TEXT_SUFFIXES = {
    ".bat",
    ".cfg",
    ".conf",
    ".css",
    ".html",
    ".ini",
    ".json",
    ".js",
    ".lock",
    ".md",
    ".py",
    ".ps1",
    ".rst",
    ".sh",
    ".sql",
    ".toml",
    ".ts",
    ".tsx",
    ".txt",
    ".xml",
    ".yaml",
    ".yml",
}
TEXT_NAMES = {"Dockerfile", "Makefile", "README", "justfile"}
NEEDLES = (
    "suspension_multibody.vehicle_dynamics",
    "suspension_multibody/vehicle_dynamics.py",
    "suspension_multibody/vehicle_dynamics",
    "suspension_multibody\\vehicle_dynamics.py",
    "suspension_multibody\\vehicle_dynamics",
    "from .vehicle_dynamics import",
    "from ..vehicle_dynamics import",
    "from . import vehicle_dynamics",
    "from .. import vehicle_dynamics",
    "from suspension_multibody import vehicle_dynamics",
    "from vehicle_dynamics import",
    "import suspension_multibody.vehicle_dynamics",
    "import vehicle_dynamics",
    "vehicle_dynamics.py",
)


def _files() -> list[Path]:
    paths: list[Path] = []
    for root in ROOTS:
        if root.is_file():
            paths.append(root)
            continue
        if not root.is_dir():
            continue
        for path in root.rglob("*"):
            if path.is_file() and not any(
                part in EXCLUDED_DIR_NAMES for part in path.parts
            ):
                paths.append(path)
    return paths


def _is_text(path: Path) -> bool:
    return path.name in TEXT_NAMES or path.suffix.lower() in TEXT_SUFFIXES


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--pre-delete", action="store_true")
    group.add_argument("--post-delete", action="store_true")
    args = parser.parse_args()

    if args.pre_delete and not TARGET.exists():
        raise SystemExit(f"pre-delete target is missing: {TARGET}")
    if args.post_delete and TARGET.exists():
        raise SystemExit(f"post-delete target still exists: {TARGET}")

    bad = []
    for path in _files():
        if path == TARGET or not _is_text(path):
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        if any(needle in text for needle in NEEDLES):
            bad.append(str(path))
    if bad:
        raise SystemExit("legacy vehicle_dynamics references:\n" + "\n".join(bad))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
