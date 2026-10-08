"""Compile restricted force-law expressions into a versioned data program."""

from __future__ import annotations

import ast
import math
from dataclasses import dataclass, field
from typing import Any, Mapping

from .errors import AuthoringError

__all__ = [
    "FunctionProgramError",
    "compile_function",
    "compile_curve",
    "compile_surface",
    "evaluate_constant_function",
]


class FunctionProgramError(AuthoringError):
    """A function program is unsafe, cyclic or dimensionally invalid."""


_OPS = {ast.Add: "add", ast.Sub: "sub", ast.Mult: "mul", ast.Div: "div", ast.Pow: "pow"}
_FUNCS = {"sin": "sin", "cos": "cos", "tanh": "tanh", "exp": "exp", "sqrt": "sqrt"}
_UNITS: dict[str, tuple[int, int, int]] = {
    "1": (0, 0, 0),
    "s": (0, 1, 0),
    "m": (1, 0, 0),
    "rad": (0, 0, 0),
    "N": (0, 0, 1),
    "Nm": (1, 0, 1),
    "N*m": (1, 0, 1),
    "Pa": (-2, 0, 1),
    "m^2": (2, 0, 0),
    "rad/s": (0, -1, 0),
    "m/s": (1, -1, 0),
    "N/m": (-1, 0, 1),
    "N*s/m": (-1, 1, 1),
}
_ALIASES = {
    "mm": ("m", 0.001),
    "mm/s": ("m/s", 0.001),
    "deg": ("rad", math.pi / 180),
    "deg/s": ("rad/s", math.pi / 180),
    "N*mm": ("Nm", 0.001),
    "kN": ("N", 1000),
    "MPa": ("Pa", 1e6),
}


def _unit(value: str, where: str) -> tuple[int, int, int]:
    try:
        return _UNITS[_ALIASES.get(value, (value, 1))[0]]
    except KeyError as exc:
        raise FunctionProgramError(f"{where}: unsupported unit {value!r}") from exc


def _add(a, b, where):
    if a != b:
        raise FunctionProgramError(
            f"{where}: operands have incompatible units {a} and {b}"
        )
    return a


@dataclass
class _Compiler:
    bindings: Mapping[str, Mapping[str, Any]]
    nodes: list[dict[str, Any]]
    units: list[tuple[int, int, int]]
    variables: Mapping[str, str] = field(default_factory=dict)
    tables: Mapping[str, Mapping[str, Any]] = field(default_factory=dict)
    visiting: list[str] = field(default_factory=list)
    variable_nodes: dict[str, tuple[int, tuple[int, int, int]]] = field(
        default_factory=dict
    )

    def emit(self, node: dict[str, Any], unit: tuple[int, int, int]) -> int:
        index = len(self.nodes)
        node["unit"] = _unit_name(unit)
        self.nodes.append(node)
        self.units.append(unit)
        return index

    def visit(self, expr: ast.AST) -> tuple[int, tuple[int, int, int]]:
        if (
            isinstance(expr, ast.Constant)
            and isinstance(expr.value, (int, float))
            and not isinstance(expr.value, bool)
        ):
            value = float(expr.value)
            if not math.isfinite(value):
                raise FunctionProgramError("constant must be finite")
            unit = (0, 0, 0)
            return self.emit({"op": "constant", "value": value}, unit), unit
        if isinstance(expr, ast.Name):
            if expr.id in self.variables:
                if expr.id in self.visiting:
                    raise FunctionProgramError(
                        f"cyclic variable dependency: {' -> '.join(self.visiting + [expr.id])}"
                    )
                if expr.id not in self.variable_nodes:
                    self.visiting.append(expr.id)
                    self.variable_nodes[expr.id] = self.visit(
                        ast.parse(self.variables[expr.id], mode="eval").body
                    )
                    self.visiting.pop()
                return self.variable_nodes[expr.id]
            if expr.id == "time":
                return self.emit(
                    {"op": "binding", "binding": 0}, (0, 1, 0)
                ), (0, 1, 0)
            if expr.id not in self.bindings:
                raise FunctionProgramError(f"unknown binding {expr.id!r}")
            binding = self.bindings[expr.id]
            unit = _unit(str(binding.get("unit", "1")), f"binding {expr.id!r}")
            return self.emit(
                {
                    "op": "binding",
                    "binding": sorted(self.bindings).index(expr.id) + 1,
                },
                unit,
            ), unit
        if isinstance(expr, ast.UnaryOp) and isinstance(expr.op, (ast.USub, ast.UAdd)):
            child, unit = self.visit(expr.operand)
            return self.emit(
                {
                    "op": "neg" if isinstance(expr.op, ast.USub) else "identity",
                    "args": [child],
                },
                unit,
            ), unit
        if isinstance(expr, ast.BinOp) and type(expr.op) in _OPS:
            left, lu = self.visit(expr.left)
            right, ru = self.visit(expr.right)
            op = _OPS[type(expr.op)]
            if op in {"add", "sub"}:
                unit = _add(lu, ru, "add/sub")
            elif op == "mul":
                unit = tuple(a + b for a, b in zip(lu, ru))
            elif op == "div":
                unit = tuple(a - b for a, b in zip(lu, ru))
            else:
                if ru != (0, 0, 0):
                    raise FunctionProgramError("power exponent must be dimensionless")
                if not isinstance(expr.right, ast.Constant) or not isinstance(
                    expr.right.value, (int, float)
                ):
                    raise FunctionProgramError(
                        "power exponent must be a numeric constant"
                    )
                exponent = float(expr.right.value)
                if exponent != int(exponent):
                    raise FunctionProgramError("non-integer powers are not supported")
                unit = tuple(int(a * exponent) for a in lu)
            return self.emit({"op": op, "args": [left, right]}, unit), unit
        if (
            isinstance(expr, ast.Call)
            and isinstance(expr.func, ast.Name)
            and expr.func.id in self.tables
        ):
            table = self.tables[expr.func.id]
            dimension = int(table["dimension"])
            if len(expr.args) != dimension or expr.keywords:
                raise FunctionProgramError(
                    f"table {expr.func.id}: expected {dimension} positional arguments"
                )
            arguments = [self.visit(arg) for arg in expr.args]
            axis_units = (
                [table["independent_unit"]]
                if dimension == 1
                else table["independent_units"]
            )
            for (_, unit), axis_unit in zip(arguments, axis_units):
                _add(unit, _unit(axis_unit, expr.func.id), expr.func.id)
            output_unit = _unit(table["dependent_unit"], expr.func.id)
            return self.emit(
                {
                    "op": "curve" if dimension == 1 else "surface",
                    "args": [node for node, _ in arguments],
                    "table": sorted(self.tables).index(expr.func.id),
                },
                output_unit,
            ), output_unit
        if (
            isinstance(expr, ast.Call)
            and isinstance(expr.func, ast.Name)
            and expr.func.id == "step"
        ):
            if len(expr.args) != 5 or expr.keywords:
                raise FunctionProgramError("step requires x, x0, y0, x1, y1")
            values = [self.visit(arg) for arg in expr.args]
            _add(values[0][1], values[1][1], "step x0")
            _add(values[0][1], values[3][1], "step x1")
            output_unit = _add(values[2][1], values[4][1], "step y")
            return self.emit(
                {"op": "step", "args": [node for node, _ in values]}, output_unit
            ), output_unit
        if (
            isinstance(expr, ast.Call)
            and isinstance(expr.func, ast.Name)
            and expr.func.id in _FUNCS
        ):
            if len(expr.args) != 1 or expr.keywords:
                raise FunctionProgramError(
                    f"{expr.func.id}: exactly one positional argument is required"
                )
            child, unit = self.visit(expr.args[0])
            if unit != (0, 0, 0):
                raise FunctionProgramError(
                    f"{expr.func.id}: argument must be dimensionless"
                )
            return self.emit({"op": _FUNCS[expr.func.id], "args": [child]}, unit), unit
        if isinstance(
            expr,
            (ast.Attribute, ast.Subscript, ast.Lambda, ast.Dict, ast.List, ast.Tuple),
        ):
            raise FunctionProgramError(
                "attribute, collection and subscript expressions are forbidden"
            )
        raise FunctionProgramError(f"unsupported expression node {type(expr).__name__}")


def _unit_name(unit: tuple[int, int, int]) -> str:
    if unit == (0, 0, 0):
        return "1"
    if unit == (0, 1, 0):
        return "s"
    if unit == (1, 0, 0):
        return "m"
    if unit == (0, 0, 1):
        return "N"
    if unit == (1, 0, 1):
        return "Nm"
    if unit == (0, -1, 0):
        return "1/s"
    return f"L^{unit[0]}T^{unit[1]}F^{unit[2]}"


def compile_function(
    expression: str,
    *,
    bindings: Mapping[str, Mapping[str, Any]] | None = None,
    output_unit: str = "N",
    variables: Mapping[str, str] | None = None,
    tables: Mapping[str, Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    """Return a deterministic program; formulas never cross into native code."""
    try:
        tree = ast.parse(expression, mode="eval")
    except SyntaxError as exc:
        raise FunctionProgramError(f"invalid formula: {exc.msg}") from exc
    bindings = bindings or {}
    for name, binding in bindings.items():
        if name == "time" or name in (variables or {}) or name in (tables or {}):
            raise FunctionProgramError(f"binding {name!r}: duplicate or reserved name")
        source = binding.get("source", "constant" if "value" in binding else None)
        if source == "measurement":
            if binding.get("measurement") not in {
                "position",
                "relative_position",
                "relative_velocity",
                "relative_angular_velocity",
            }:
                raise FunctionProgramError(f"binding {name!r}: unknown measurement")
            for reference in ("action", "reaction", "reference"):
                if not binding.get(reference):
                    raise FunctionProgramError(
                        f"binding {name!r}: measurement needs {reference}"
                    )
            measured_unit = "rad/s" if binding["measurement"] == "relative_angular_velocity" else "m/s" if binding["measurement"] == "relative_velocity" else "m"
            if _unit(str(binding.get("unit", "1")), name) != _unit(measured_unit, name):
                raise FunctionProgramError(f"binding {name!r}: measurement requires {measured_unit}")
        _unit(str(binding.get("unit", "1")), name)
    compiler = _Compiler(bindings, [], [], variables or {}, tables or {})
    for name in compiler.variables:
        compiler.visit(ast.Name(id=name))
    root, unit = compiler.visit(tree.body)
    expected = _unit(output_unit, "output")
    if unit != expected:
        raise FunctionProgramError(
            f"output has unit {_unit_name(unit)}, expected {output_unit}"
        )
    return {
        "program_version": 1,
        "nodes": compiler.nodes,
        "bindings": [{"name": "time", "unit": "s", "source": "time"}]
        + [
            {
                "name": name,
                **dict(value),
                "scale": 1 if value.get("source") == "measurement" else _ALIASES.get(str(value.get("unit", "1")), ("", 1))[1],
            }
            for name, value in sorted(bindings.items())
        ],
        "tables": [dict(table) for _, table in sorted((tables or {}).items())],
        "outputs": {"value": root, "unit": _ALIASES.get(output_unit, (output_unit, 1))[0]},
    }


def evaluate_constant_function(expression: str, values: Mapping[str, float]) -> float:
    """Resolve a parameter recipe before submission, in its declared value units."""
    program = compile_function(expression, bindings={
        name: {"value": value, "unit": "1", "source": "constant"}
        for name, value in values.items()
    }, output_unit="1")
    result: list[float] = []
    for node in program["nodes"]:
        op = node["op"]
        args = [result[index] for index in node.get("args", ())]
        if op == "constant":
            value = float(node["value"])
        elif op == "binding":
            binding = program["bindings"][node["binding"]]
            if binding["source"] != "constant":
                raise FunctionProgramError("parameter recipes may only read constants")
            value = float(binding["value"])
        elif op == "add":
            value = args[0] + args[1]
        elif op == "sub":
            value = args[0] - args[1]
        elif op == "mul":
            value = args[0] * args[1]
        elif op == "div":
            value = args[0] / args[1]
        elif op == "pow":
            value = args[0] ** args[1]
        elif op == "neg":
            value = -args[0]
        elif op == "identity":
            value = args[0]
        elif op in {"sin", "cos", "tanh", "exp", "sqrt"}:
            value = getattr(math, op)(args[0])
        else:
            raise FunctionProgramError(f"parameter recipe cannot use {op!r}")
        if not math.isfinite(value):
            raise FunctionProgramError("parameter recipe must resolve to a finite value")
        result.append(value)
    return result[program["outputs"]["value"]]


def compile_curve(
    points: Any,
    *,
    independent_unit: str,
    dependent_unit: str,
    interpolation: str = "piecewise_linear",
    extrapolation: str = "clamp",
) -> dict[str, Any]:
    """Validate a monotone 1-D table with an explicit interpolation policy."""
    if interpolation not in {"piecewise_linear", "akima"} or extrapolation not in {
        "clamp",
        "linear",
        "error",
    }:
        raise FunctionProgramError(
            "unsupported curve interpolation or extrapolation policy"
        )
    rows = tuple((float(x), float(y)) for x, y in points)
    if len(rows) < 2 or any(
        not math.isfinite(x) or not math.isfinite(y) for row in rows for x, y in (row,)
    ):
        raise FunctionProgramError("curve needs at least two finite points")
    if any(b[0] <= a[0] for a, b in zip(rows, rows[1:])):
        raise FunctionProgramError("curve abscissas must be strictly increasing")
    _unit(independent_unit, "curve independent")
    _unit(dependent_unit, "curve dependent")
    xscale = _ALIASES.get(independent_unit, (independent_unit, 1))[1]
    yscale = _ALIASES.get(dependent_unit, (dependent_unit, 1))[1]
    return {
        "dimension": 1,
        "points": [[x*xscale, y*yscale] for x, y in rows],
        "independent_unit": independent_unit,
        "dependent_unit": dependent_unit,
        "interpolation": interpolation,
        "extrapolation": extrapolation,
    }


def compile_surface(
    x_axis: Any,
    y_axis: Any,
    values: Any,
    *,
    independent_units: tuple[str, str],
    dependent_unit: str,
    extrapolation: str = "clamp",
) -> dict[str, Any]:
    """Validate a rectangular bilinear table with explicit extrapolation."""
    axes = [list(map(float, x_axis)), list(map(float, y_axis))]
    rows = [list(map(float, row)) for row in values]
    if extrapolation not in {"clamp", "linear", "error"}:
        raise FunctionProgramError("unsupported surface extrapolation")
    if any(
        len(axis) < 2 or any(b <= a for a, b in zip(axis, axis[1:])) for axis in axes
    ):
        raise FunctionProgramError("surface axes must be strictly increasing")
    if len(rows) != len(axes[0]) or any(len(row) != len(axes[1]) for row in rows):
        raise FunctionProgramError("surface values must match the two axes")
    if any(not math.isfinite(value) for row in axes + rows for value in row):
        raise FunctionProgramError("surface values must be finite")
    for unit in independent_units:
        _unit(unit, "surface axis")
    _unit(dependent_unit, "surface output")
    if len(independent_units) != 2:
        raise FunctionProgramError("surface requires exactly two axis units")
    for axis, unit in zip(axes, independent_units):
        scale = _ALIASES.get(unit, (unit, 1))[1]
        axis[:] = [value*scale for value in axis]
    scale = _ALIASES.get(dependent_unit, (dependent_unit, 1))[1]
    rows = [[value*scale for value in row] for row in rows]
    return {
        "dimension": 2,
        "x_axis": axes[0],
        "y_axis": axes[1],
        "values": rows,
        "independent_units": list(independent_units),
        "dependent_unit": dependent_unit,
        "interpolation": "bilinear",
        "extrapolation": extrapolation,
    }
