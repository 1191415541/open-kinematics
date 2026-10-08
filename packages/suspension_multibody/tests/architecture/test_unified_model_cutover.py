"""The explicit resolved route never selects an automotive builder or decoder."""

from __future__ import annotations

import ast
from pathlib import Path

import numpy as np
import pytest

from suspension_multibody.api import simulate, validate
from suspension_multibody.authoring.loader import DocumentLoader
from suspension_multibody.compilation.resolved import compile_resolved, plan_from_case
from suspension_multibody.modeling.resolved import ResolvedModel
from suspension_multibody.results.envelope import ResultEnvelope

from ..authoring.test_generic_multibody import _assembly, _case


def test_memory_file_and_resolved_ir_share_payload_and_result(tmp_path):
    from suspension_multibody.authoring.migration import save_migrated_assembly

    assembly, case = _assembly(), _case()
    path = save_migrated_assembly(assembly, tmp_path)
    memory, file = validate(assembly, case), validate(path, case)
    resolved = DocumentLoader().load(assembly, case).resolve()
    explicit = compile_resolved(resolved, plan_from_case(case))
    assert memory.model_payload == file.model_payload == explicit.model_payload
    assert memory.case_payload == file.case_payload == explicit.case_payload
    one, two = simulate(assembly, case), simulate(path, case)
    assert isinstance(one.result, ResultEnvelope)
    assert isinstance(two.result, ResultEnvelope)
    assert isinstance(one.compiled.request.model, ResolvedModel)
    assert one.result.body_ids == two.result.body_ids
    for name, values in one.result.named_blocks.items():
        if name != "diagnostics":
            np.testing.assert_array_equal(values, two.result.named_blocks[name])


def test_generic_submission_does_not_call_automotive_preparation(monkeypatch):
    import importlib.abc
    import sys

    class BlockRetired(importlib.abc.MetaPathFinder):
        def find_spec(self, fullname, path=None, target=None):
            if fullname.startswith(("suspension_multibody.preparation", "suspension_multibody.subsystems")):
                raise AssertionError("automotive preparation reached by resolved submission")

    monkeypatch.setattr(sys, "meta_path", [BlockRetired(), *sys.meta_path])
    run = simulate(_assembly(), _case())
    assert run.status == "success"
    assert isinstance(run.result, ResultEnvelope)


def test_offline_migration_has_no_execution_import_or_call():
    path = Path(__file__).parents[2] / "src/suspension_multibody/authoring/migration.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    forbidden = {"run_request", "run_compiled", "run_contract", "submit", "solve", "NativeContractBackend"}
    imports = {name.name.rsplit(".", 1)[-1] for node in ast.walk(tree)
        if isinstance(node, (ast.Import, ast.ImportFrom)) for name in node.names}
    calls = {node.func.id if isinstance(node.func, ast.Name) else node.func.attr
        for node in ast.walk(tree) if isinstance(node, ast.Call)
        and isinstance(node.func, (ast.Name, ast.Attribute))}
    assert not (imports | calls) & forbidden


@pytest.fixture
def migration_gate():
    import importlib.util

    path = Path(__file__).parents[2] / "scripts/check_unified_model_migration.py"
    spec = importlib.util.spec_from_file_location("migration_gate", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("source", [
    "import run_contract as execute", "from backend import submit as execute", "backend.submit()", "solver.solve()",
])
def test_execution_boundary_recognizes_aliases_and_method_calls(migration_gate, source):
    assert migration_gate.execution_names(ast.parse(source)) & {"run_contract", "submit", "solve"}


@pytest.mark.parametrize("damage", ["duplicate", "source", "references", "target"])
def test_consumer_mapping_refuses_forged_evidence(migration_gate, tmp_path, monkeypatch, damage):
    source = tmp_path / "consumer.py"
    source.write_text("from model import simulate\nsimulate(model, case)\n", encoding="utf-8")
    target = tmp_path / "target.py"
    target.write_text("def simulate(): pass\n", encoding="utf-8")
    evidence = tmp_path / "test_probe.py"
    evidence.write_text("def test_probe(): pass\n", encoding="utf-8")
    original = {"file": source.name, "line": 2, "kind": "Call", "symbols": ["run_request"], "category": "production"}
    row = {**original, "disposition": "migrated", "source_sha256": migration_gate.digest(source.read_bytes()),
        "replacement_references": migration_gate.current_references(source),
        "target": [{"file": target.name, "symbol": "simulate", "source_sha256": migration_gate.digest(target.read_bytes())}],
        "evidence": [evidence.name]}
    inventory = {"consumers": [original], "registrations": {"preparations": []}}
    mapping = {"consumers": [row], "protocols": []}
    monkeypatch.setattr(migration_gate, "ROOT", tmp_path)
    monkeypatch.setattr(migration_gate, "consumers", lambda: [])
    migration_gate.validate_mapping(inventory, mapping)
    if damage == "duplicate":
        mapping["consumers"].append(row)
    elif damage == "source":
        row["source_sha256"] = "forged"
    elif damage == "references":
        row["replacement_references"] = []
    else:
        row["target"][0]["symbol"] = "missing"
    with pytest.raises(ValueError):
        migration_gate.validate_mapping(inventory, mapping)
