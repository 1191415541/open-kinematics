"""Exercise the shipped native reader, scalar evaluator and directional pass."""

from __future__ import annotations

import copy
import json
import math
import subprocess
from pathlib import Path

import numpy as np
import pytest
from suspension_contracts.multibody import ContractError, validate_model
from suspension_multibody.authoring.function_program import (
    compile_curve,
    compile_function,
    compile_surface,
)

from suspension_kernel.binding.build import discover_compiler

ROOT = Path(__file__).resolve().parents[3]
KERNEL = ROOT / "packages/suspension_kernel"


@pytest.fixture(scope="module")
def probe(tmp_path_factory):
    executable = tmp_path_factory.mktemp("function_program") / "probe.exe"
    archives = sorted((KERNEL / "build/Release").glob("libmb_*.a"))
    assert archives, "build the kernel before running native program tests"
    command = [
        str(discover_compiler()),
        "-std=c++17",
        "-O2",
        "-flto=auto",
        "-fno-fat-lto-objects",
        "-fopenmp",
        "-I",
        "packages/suspension_kernel/cpp/include",
        "packages/suspension_kernel/tests/fixtures/function_program_probe.cpp",
        "-o",
        str(executable),
        "-Wl,--start-group",
        *(path.relative_to(ROOT).as_posix() for path in archives),
        "-Wl,--end-group",
    ]
    built = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
    assert built.returncode == 0, built.stderr

    def run(program, *, time=0.35, error=False, element=None):
        model = document(program)
        if element:
            model["elements"] = [element]
        if not error:
            validate_model(model)
        completed = subprocess.run(
            [str(executable), str(time)],
            input=json.dumps(model),
            capture_output=True,
            text=True,
        )
        assert completed.returncode == 0, completed.stderr
        if error:
            assert completed.stdout.startswith("error "), completed.stdout
            return completed.stdout
        rows = completed.stdout.splitlines()
        assert rows[0].startswith("value "), completed.stdout
        if element:
            df = [list(map(float, row.split()[1:])) for row in rows if row.startswith("force_dual ")]
            fd_force = [list(map(float, row.split()[1:])) for row in rows if row.startswith("force_fd ")]
            np.testing.assert_allclose(df, fd_force, atol=1e-6, rtol=1e-6)
            balance = next(row for row in rows if row.startswith("balance "))
            np.testing.assert_allclose(list(map(float, balance.split()[1:])), 0, atol=1e-12)
            power = next(row for row in rows if row.startswith("power "))
            actual, ledger = map(float, power.split()[1:])
            assert actual == ledger
        else:
            assert len(rows) == 2, completed.stdout
        scalar, dual, derivative, fd, smooth = map(float, rows[0].split()[1:])
        assert scalar == dual == float(rows[1].split()[1])
        assert abs(derivative - fd) / max(1, abs(derivative), abs(fd)) <= 1e-6
        return scalar, derivative, smooth

    return run


@pytest.mark.parametrize("kind,unit", [("force", "N"), ("torque", "Nm")])
def test_offset_wrench_rotated_reference_derivative_and_balance(probe, kind, unit):
    binding = measurement("relative_velocity")
    program = compile_function("gain*tanh(v/speed)", output_unit=unit, bindings={
        "v": binding, "gain": {"unit": unit, "value": 7}, "speed": {"unit": "m/s", "value": 1}})
    markers = {key: copy.deepcopy(binding[key]) for key in ("action", "reaction", "reference")}
    element = {"name": "function", "type": kind, "parameters": {**markers, "axis": [1, 0, 0], "program_id": 0}}
    assert abs(probe(program, element=element)[1]) > 1e-6


def document(program):
    return {
        "contract": "multibody-model",
        "contract_version": 1,
        "kind": "model",
        "name": "function-probe",
        "units": {"length": "m", "mass": "kg", "time": "s", "angle": "rad"},
        "bodies": [
            {
                "name": "a",
                "mass": 1,
                "inertia": np.eye(3).tolist(),
                "position": [0.8, -0.3, 0.2],
                "quaternion": [1, 0, 0, 0],
                "velocity": [0.3, 0.5, -0.4],
                "omega": [0.2, -0.1, 0.4],
            },
            {
                "name": "b",
                "mass": 1,
                "inertia": np.eye(3).tolist(),
                "position": [-0.2, 0.4, -0.1],
                "quaternion": [math.cos(0.3), 0, math.sin(0.3), 0],
                "velocity": [-0.1, 0.7, 0.2],
                "omega": [-0.3, 0.2, 0.1],
            },
        ],
        "function_programs": [program],
    }


def measurement(kind="relative_position"):
    return {
        "source": "measurement",
        "measurement": kind,
        "unit": "rad/s"
        if kind == "relative_angular_velocity"
        else "m/s"
        if kind == "relative_velocity"
        else "m",
        "action": {"body": "a", "point": [0.2, -0.1, 0.3]},
        "reaction": {"body": "b", "point": [-0.1, 0.2, 0.15]},
        "reference": {"body": "b", "quaternion": [math.cos(0.2), 0, 0, math.sin(0.2)]},
        "axis": [1, 0, 0],
    }


@pytest.mark.parametrize(
    "expression,expected",
    [
        ("+x", 0.7),
        ("-x", -0.7),
        ("x+y", 1.0),
        ("x-y", 0.4),
        ("x*y", 0.21),
        ("x/y", 0.7 / 0.3),
        ("x**3", 0.7**3),
        ("sin(x)", math.sin(0.7)),
        ("cos(x)", math.cos(0.7)),
        ("tanh(x)", math.tanh(0.7)),
        ("exp(x)", math.exp(0.7)),
        ("sqrt(x)", math.sqrt(0.7)),
        ("2", 2),
    ],
)
def test_each_operation_scalar(probe, expression, expected):
    program = compile_function(
        expression,
        output_unit="1",
        bindings={"x": {"unit": "1", "value": 0.7}, "y": {"unit": "1", "value": 0.3}},
    )
    assert probe(program)[0] == pytest.approx(expected, rel=1e-13)


@pytest.mark.parametrize(
    "expression",
    [
        "p/L",
        "-(p/L)",
        "+(p/L)",
        "p/L+2",
        "p/L-2",
        "p/L*3",
        "3/(p/L)",
        "(p/L)**3",
        "sin(p/L)",
        "cos(p/L)",
        "tanh(p/L)",
        "exp(p/L)",
        "sqrt(p/L)",
    ],
)
def test_state_operation_dual_against_difference(probe, expression):
    program = compile_function(
        expression,
        output_unit="1",
        bindings={"p": measurement(), "L": {"unit": "m", "value": 2}},
    )
    assert abs(probe(program)[1]) > 1e-6


@pytest.mark.parametrize(
    "kind",
    ["position", "relative_position", "relative_velocity", "relative_angular_velocity"],
)
def test_rotating_reference_and_attachment_measurement(probe, kind):
    binding = measurement(kind)
    program = compile_function(
        "p", output_unit=binding["unit"], bindings={"p": binding}
    )
    value, derivative, _ = probe(program)
    angle = 0.6
    ry = np.array(
        [
            [math.cos(angle), 0, math.sin(angle)],
            [0, 1, 0],
            [-math.sin(angle), 0, math.cos(angle)],
        ]
    )
    axis = ry @ [math.cos(0.4), math.sin(0.4), 0]
    a = np.array([1, -0.4, 0.5])
    b = np.array([-0.2, 0.4, -0.1]) + ry @ [-0.1, 0.2, 0.15]
    if kind == "position":
        expected = a @ axis
    elif kind == "relative_position":
        expected = (a - b) @ axis
    elif kind == "relative_velocity":
        va = [0.3, 0.5, -0.4] + np.cross([0.2, -0.1, 0.4], [0.2, -0.1, 0.3])
        vb = [-0.1, 0.7, 0.2] + np.cross([-0.3, 0.2, 0.1], ry @ [-0.1, 0.2, 0.15])
        expected = (va - vb - np.cross([-0.3, 0.2, 0.1], a - b)) @ axis
    else:
        expected = (np.array([0.2, -0.1, 0.4]) - [-0.3, 0.2, 0.1]) @ axis
    assert value == pytest.approx(expected, rel=1e-12)
    assert abs(derivative) > 1e-6


def test_internal_time_and_signal_interpolation(probe):
    program = compile_function(
        "time/t0+signal",
        output_unit="1",
        bindings={
            "t0": {"unit": "s", "value": 2},
            "signal": {"unit": "1", "source": "channel", "samples": [[0, 1], [1, 3]]},
        },
    )
    for time in (0.125, 0.35, 0.825):
        assert probe(program, time=time)[0] == pytest.approx(time / 2 + 1 + 2 * time)


def test_step_curve_surface_derivatives(probe):
    binding = {
        "p": measurement(),
        "zero": {"unit": "m", "value": 0},
        "end": {"unit": "m", "value": 2},
    }
    program = compile_function(
        "step(p,zero,0,end,3)", bindings=binding, output_unit="1"
    )
    assert abs(probe(program)[1]) > 1e-6
    for interpolation in ("piecewise_linear", "akima"):
        table = compile_curve(
            [[-2, 2], [0, 1], [1, 3], [2, 4], [3, 8]],
            independent_unit="m",
            dependent_unit="N",
            interpolation=interpolation,
        )
        program = compile_function(
            "curve(p)", bindings={"p": measurement()}, tables={"curve": table}
        )
        assert abs(probe(program)[1]) > 1e-6
    surface = compile_surface(
        [-2, 2],
        [-2, 2],
        [[0, 4], [8, 20]],
        independent_units=("m", "m"),
        dependent_unit="N",
    )
    program = compile_function(
        "surface(p,p)", bindings={"p": measurement()}, tables={"surface": surface}
    )
    assert abs(probe(program)[1]) > 1e-6


def test_table_engineering_units_become_si(probe):
    table = compile_curve(
        [[0, 0], [1000, 2]], independent_unit="mm", dependent_unit="kN"
    )
    program = compile_function(
        "curve(x)",
        bindings={"x": {"unit": "mm", "value": 250}},
        tables={"curve": table},
    )
    assert probe(program)[0] == pytest.approx(500)


@pytest.mark.parametrize(
    "mutation",
    [
        "version",
        "arity",
        "fractional",
        "cycle",
        "binding",
        "table",
        "policy",
        "surface_shape",
        "measurement",
    ],
)
def test_reader_rejects_malformed_program(probe, mutation):
    table = compile_surface(
        [0, 2],
        [0, 2],
        [[1, 2], [3, 4]],
        independent_units=("m", "m"),
        dependent_unit="N",
    )
    program = compile_function(
        "surface(p,p)", bindings={"p": measurement()}, tables={"surface": table}
    )
    broken = copy.deepcopy(program)
    if mutation == "version":
        broken["program_version"] = 2
    elif mutation == "arity":
        broken["nodes"][-1]["args"] = [0]
    elif mutation == "fractional":
        broken["nodes"][-1]["args"][0] = 0.5
    elif mutation == "cycle":
        broken["nodes"][-1]["args"][0] = len(broken["nodes"]) - 1
    elif mutation == "binding":
        broken["nodes"][0]["binding"] = 99
    elif mutation == "table":
        broken["nodes"][-1]["table"] = 99
    elif mutation == "policy":
        broken["tables"][0]["extrapolation"] = "guess"
    elif mutation == "surface_shape":
        broken["tables"][0]["values"] = [[1]]
    else:
        broken["bindings"][1]["measurement"] = "guess"
    with pytest.raises(ContractError):
        validate_model(document(broken))
    probe(broken, error=True)
