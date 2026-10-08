"""The production tree contains one resolved model-to-native route."""

import ast
from pathlib import Path

import pytest

from suspension_multibody.authoring import assemble_generic, migrate_v1_axle
from suspension_multibody.authoring.migration import MigrationError
from tests.authoring.test_offline_model_migration import explicit_source

SOURCE = Path(__file__).parents[2] / "src" / "suspension_multibody"


def test_old_runtime_entry_modules_are_retired():
    for relative in ("preparation", "subsystems", "authoring/solver.py", "rigs/compose.py"):
        path = SOURCE / relative
        assert not path.is_file()
        assert not list(path.rglob("*.py"))


def test_native_run_modules_do_not_call_old_conversion():
    offenders = []
    for path in SOURCE.rglob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and getattr(node.func, "id", getattr(node.func, "attr", "")) == "axle_dynamics_model":
                offenders.append(path.relative_to(SOURCE).as_posix())
    assert offenders == []


def test_offline_input_adapter_preserves_declared_si_inertia():
    graph = assemble_generic(migrate_v1_axle(explicit_source()))
    body = graph.bodies["model.sub.json.slider"]
    assert body.mass == 2
    assert body.inertia[0, 0] == 1e-6


def test_an_axle_declaring_no_inertia_remains_kinematic_data():
    source = explicit_source()
    source["bodies"][0]["mass"] = 0
    source["bodies"][0]["inertia"] = [[0, 0, 0]]*3
    with pytest.raises(MigrationError, match="no declared mass"):
        migrate_v1_axle(source)
