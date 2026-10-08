"""Production accepts ordinary documents and rejects retired execution objects."""

import ast
import inspect

import pytest

from suspension_multibody.api import validate
from suspension_multibody.authoring import generic
from tests.authoring.test_generic_multibody import _case
from tests.benchmark_fixture import benchmark_model


def test_old_automotive_input_requires_explicit_offline_migration():
    with pytest.raises((TypeError, ValueError)):
        validate(benchmark_model(), _case())


def test_production_assembler_reads_no_automotive_membership_policy():
    tree = ast.parse(inspect.getsource(generic))
    calls = {node.func.id for node in ast.walk(tree) if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)}
    assert not {"check_root", "check_assembly_roles", "compose_axle", "compose_vehicle"}.intersection(calls)
