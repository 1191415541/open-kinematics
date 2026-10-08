"""Restricted expression compiler tests."""

import pytest

from suspension_multibody.authoring.function_program import (
    FunctionProgramError,
    compile_curve,
    compile_function,
    compile_surface,
)


def test_brake_formula_is_versioned_and_has_stable_bindings() -> None:
    program = compile_function(
        "-pressure * area * mu * radius * tanh(spin / speed)",
        bindings={
            "pressure": {"unit": "Pa", "source": "channel"},
            "area": {"unit": "m^2", "source": "property"},
            "mu": {"unit": "1", "source": "property"},
            "radius": {"unit": "m", "source": "property"},
            "spin": {
                "unit": "rad/s",
                "source": "measurement",
                "measurement": "relative_angular_velocity",
                "action": "rotor",
                "reaction": "caliper",
                "reference": "rotor",
            },
            "speed": {"unit": "rad/s", "source": "property"},
        },
        output_unit="Nm",
    )
    assert program["program_version"] == 1
    assert program["outputs"]["unit"] == "Nm"
    assert [row["name"] for row in program["bindings"]] == [
        "time",
        "area",
        "mu",
        "pressure",
        "radius",
        "speed",
        "spin",
    ]
    assert all("unit" in node for node in program["nodes"])
    assert program["nodes"][program["outputs"]["value"]]["op"] == "mul"


def test_torque_unit_is_distinct_from_force_and_unit_error_is_located() -> None:
    program = compile_function(
        "torque_gain * demand",
        bindings={"torque_gain": {"unit": "Nm"}, "demand": {"unit": "1"}},
        output_unit="Nm",
    )
    assert program["outputs"]["unit"] == "Nm"
    with pytest.raises(FunctionProgramError, match="expected N"):
        compile_function(
            "torque_gain * demand",
            bindings={"torque_gain": {"unit": "Nm"}, "demand": {"unit": "1"}},
            output_unit="N",
        )


@pytest.mark.parametrize("expression", ["obj.value", "f(x)", "x[0]", "lambda x: x"])
def test_unsafe_python_constructs_are_rejected(expression: str) -> None:
    with pytest.raises(FunctionProgramError):
        compile_function(expression, bindings={"x": {"unit": "1"}}, output_unit="1")


def test_unknown_binding_and_invalid_dag_reference_are_rejected() -> None:
    with pytest.raises(FunctionProgramError, match="unknown binding"):
        compile_function("missing + 1", output_unit="1")
    with pytest.raises(FunctionProgramError, match="incompatible units"):
        compile_function(
            "x + y", bindings={"x": {"unit": "m"}, "y": {"unit": "s"}}, output_unit="m"
        )


def test_curve_interpolation_and_extrapolation_are_explicit() -> None:
    curve = compile_curve(
        [(0, 0), (1, 2)],
        independent_unit="m",
        dependent_unit="N",
        extrapolation="error",
    )
    assert curve["dimension"] == 1 and curve["extrapolation"] == "error"
    with pytest.raises(FunctionProgramError, match="increasing"):
        compile_curve([(0, 0), (0, 1)], independent_unit="m", dependent_unit="N")
    with pytest.raises(FunctionProgramError, match="unsupported"):
        compile_curve(
            [(0, 0), (1, 1)],
            independent_unit="m",
            dependent_unit="N",
            extrapolation="wrap",
        )


def test_variables_form_a_dag_and_cycles_are_rejected() -> None:
    program = compile_function(
        "b + b",
        bindings={"gain": {"unit": "N", "value": 5}},
        variables={"a": "gain * 2", "b": "a * 3"},
    )
    assert program["nodes"][-1]["args"][0] == program["nodes"][-1]["args"][1]
    with pytest.raises(FunctionProgramError, match="cyclic variable dependency"):
        compile_function("a", variables={"a": "b", "b": "a"}, output_unit="1")


def test_surface_and_step_have_typed_operands() -> None:
    table = compile_surface(
        [0, 1],
        [0, 2],
        [[0, 0], [2, 4]],
        independent_units=("1", "1"),
        dependent_unit="Nm",
        extrapolation="error",
    )
    program = compile_function(
        "map(x, y)",
        bindings={"x": {"unit": "1"}, "y": {"unit": "1"}},
        tables={"map": table},
        output_unit="Nm",
    )
    assert program["nodes"][-1]["op"] == "surface"
    assert program["tables"][0]["extrapolation"] == "error"
    step = compile_function(
        "step(time, t0, f0, t1, f1)",
        bindings={
            "t0": {"unit": "s", "value": 0},
            "t1": {"unit": "s", "value": 1},
            "f0": {"unit": "N", "value": 0},
            "f1": {"unit": "N", "value": 1},
        },
    )
    assert step["nodes"][-1]["op"] == "step"


def test_input_scale_and_measurement_references_are_explicit() -> None:
    program = compile_function(
        "x",
        bindings={"x": {"unit": "mm", "source": "channel", "channel": "travel"}},
        output_unit="m",
    )
    assert program["bindings"][1]["scale"] == 0.001
    with pytest.raises(FunctionProgramError, match="needs action"):
        compile_function(
            "x",
            bindings={
                "x": {
                    "unit": "m",
                    "source": "measurement",
                    "measurement": "relative_position",
                }
            },
            output_unit="m",
        )
