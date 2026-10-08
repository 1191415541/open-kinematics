"""The same interpreter accepts zero-body and detailed force subsystems."""

import ast
import inspect

import numpy as np
import pytest

from suspension_multibody.authoring import generic
from suspension_multibody.presets import generic_template

from ._torque import torque_graph


@pytest.mark.parametrize("role", ["brake", "drive"])
def test_a_detailed_provider_adds_only_its_declared_body(role):
    simple = torque_graph(role)
    payload = generic_template(role).to_payload()
    payload["bodies"] = [{"name": "housing", "fixed": True, "mass": 3, "inertia": np.eye(3).tolist()}]
    detailed = torque_graph(role, payload=payload)
    assert set(detailed.bodies) - set(simple.bodies) == {"unit.housing"}
    assert detailed.bodies["unit.housing"].mass == 3
    assert detailed.elements == simple.elements
    assert torque_graph(role).bodies.keys() == simple.bodies.keys()


def _business_decisions(source):
    tree = ast.parse(source)
    tests = [node.test for node in ast.walk(tree) if isinstance(node, (ast.If, ast.IfExp))]
    tests += [condition for node in ast.walk(tree) if isinstance(node, ast.comprehension) for condition in node.ifs]
    return [ast.unparse(test) for test in tests if any(
        isinstance(node, ast.Constant) and node.value in {"brake", "drive", "suspension", "vehicle", "simplified"}
        for node in ast.walk(test))]


def test_the_interpreter_has_no_business_provider_branch():
    assert _business_decisions(inspect.getsource(generic)) == []


@pytest.mark.parametrize("source", ["def f(role):\n if role == 'brake': return 1",
    "def f(rows):\n return [r for r in rows if r.role == 'drive']",
    "def f(name):\n return 1 if name == 'simplified' else 0"])
def test_the_branch_detector_rejects_real_provider_branches(source):
    assert _business_decisions(source)
