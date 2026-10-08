"""Exercise native trial evaluation and candidate projection without Python state."""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

from suspension_kernel.binding.build import discover_compiler

ROOT = Path(__file__).resolve().parents[3]
KERNEL = ROOT / "packages/suspension_kernel"


@pytest.fixture(scope="module")
def facts(tmp_path_factory):
    executable = tmp_path_factory.mktemp("law_trial") / "probe.exe"
    archives = sorted((KERNEL / "build/Release").glob("libmb_*.a"))
    assert archives, "build the kernel before testing native state"
    built = subprocess.run([
        str(discover_compiler()), "-std=c++17", "-O2", "-flto=auto", "-fno-fat-lto-objects",
        "-fopenmp", "-I", "packages/suspension_kernel/cpp/include",
        "packages/suspension_kernel/tests/fixtures/law_trial_probe.cpp", "-o", str(executable),
        "-Wl,--start-group", *(path.relative_to(ROOT).as_posix() for path in archives), "-Wl,--end-group",
    ], cwd=ROOT, capture_output=True, text=True, check=False)
    assert built.returncode == 0, built.stderr
    completed = subprocess.run([str(executable)], capture_output=True, text=True, check=False)
    assert completed.returncode == 0, completed.stderr
    return {row.split()[0]: list(map(int, row.split()[1:])) for row in completed.stdout.splitlines()}


def test_force_trial_is_pure_and_repeatable(facts):
    assert facts["lifecycle"][0] == 1


def test_rejected_projection_cannot_change_accepted_state(facts):
    assert facts["lifecycle"][1:4] == [1, 1, 1]


def test_law_instances_are_isolated_and_commit_is_explicit(facts):
    assert facts["lifecycle"][4] == 1
    assert facts["commit"] == [1]
