"""Native relative-angle row: arbitrary reference, multi-turn phase and derivatives."""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

from suspension_kernel.binding.build import discover_compiler

ROOT = Path(__file__).resolve().parents[3]
KERNEL = ROOT / "packages/suspension_kernel"


@pytest.fixture(scope="module")
def probe(tmp_path_factory):
    executable = tmp_path_factory.mktemp("joint_coordinate") / "probe.exe"
    archives = sorted((KERNEL / "build/Release").glob("libmb_*.a"))
    assert archives, "build the native kernel first"
    built = subprocess.run([
        str(discover_compiler()), "-std=c++17", "-O2", "-flto=auto",
        "-fno-fat-lto-objects", "-fopenmp", "-I", "packages/suspension_kernel/cpp/include",
        "packages/suspension_kernel/tests/fixtures/joint_coordinate_probe.cpp",
        "-o", str(executable), "-Wl,--start-group",
        *(path.relative_to(ROOT).as_posix() for path in archives), "-Wl,--end-group",
    ], cwd=ROOT, capture_output=True, text=True, check=False)
    assert built.returncode == 0, built.stderr
    completed = subprocess.run([str(executable)], capture_output=True, text=True, check=False)
    assert completed.returncode == 0, completed.stderr
    return {row.split()[0]: [float(value) for value in row.split()[1:]] for row in completed.stdout.splitlines()}


def test_scalar_and_directional_rows_match_finite_difference(probe):
    residual, jacobian, directional = probe["errors"]
    assert residual < 1e-12
    assert jacobian <= 1e-6
    assert directional <= 1e-6


def test_relative_lock_reaction_is_an_axial_couple(probe):
    magnitude, transverse = probe["reaction"]
    assert abs(magnitude) == pytest.approx(7)
    assert transverse < 1e-12


def test_prescribed_rate_keeps_unwrapped_signal(probe):
    assert probe["rate"] == [2.5]
