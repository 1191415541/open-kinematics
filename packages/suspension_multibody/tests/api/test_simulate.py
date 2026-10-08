"""The public document entry compiles and submits exactly once."""

import inspect
import json

import numpy as np
import pytest

from suspension_multibody import api
from suspension_multibody.authoring import AssemblyDocument
from suspension_multibody.authoring.errors import AuthoringError
from suspension_multibody.authoring.loader import DocumentLoader
from suspension_multibody.authoring.migration import (
    migrate_v1_kc_case,
    save_migrated_assembly,
)
from suspension_multibody.compilation.resolved import compile_resolved, plan_from_case
from tests.authoring.test_generic_multibody import _assembly, _case
from tests.benchmark_fixture import benchmark_model


def test_simulate_is_a_public_name():
    import suspension_multibody as package
    assert package.simulate is api.simulate
    assert "simulate" in package.__all__
    assert package._PUBLIC_NAMES["simulate"] == (".api", "simulate")


def test_simulate_has_a_signature_and_a_documented_contract():
    assert list(inspect.signature(api.simulate).parameters) == ["assembly_document", "case_document"]
    assert "resolved assembly and case" in inspect.getdoc(api.simulate)


def test_simulate_only_compiles_and_submits():
    source = inspect.getsource(api.simulate)
    assert "run_compiled(validate(assembly_document, case_document))" in source
    assert "compile_resolved" in inspect.getsource(api.validate)
    assert "DocumentLoader" in inspect.getsource(api.validate)


@pytest.mark.parametrize("mode", ["K", "C"])
def test_a_document_pair_runs_end_to_end(tmp_path, mode):
    model = benchmark_model()
    if mode == "C":
        from tests.cases.kc_quasi_static.kc_fixtures import _compliant_model
        model = _compliant_model()
    arguments = {"wheel_values_mm": (0., 20.), "rack_values_mm": (0.,)} if mode == "K" else {"paths": ("fz",), "levels": 3, "maximum": 1.}
    assembly, case = migrate_v1_kc_case(model, mode=mode, **arguments)
    path = save_migrated_assembly(assembly, tmp_path)
    run = api.simulate(path, case)
    assert run.status == "success"
    assert len(run.raw.cases) == (2 if mode == "K" else 3)
    assert run.raw.states.shape[0] == sum(int(row["sample_count"]) for row in run.raw.cases)
    assert np.isfinite(run.raw.states).all()


def test_the_document_pair_reaches_the_same_submission():
    assembly, case = _assembly(), _case()
    actual = api.validate(assembly, case)
    expected = compile_resolved(DocumentLoader().load(assembly, case).resolve(), plan_from_case(case))
    assert actual.model_payload == expected.model_payload
    assert actual.case_payload == expected.case_payload
    assert actual.metadata["compiler"] == "ResolvedModelCompiler"


def test_the_assembly_document_override_reaches_the_model():
    original = _assembly()
    payload = original.to_payload()
    payload["subsystems"][0]["overrides"] = {"hardpoints": {"mount": [0, 0, .3]}}
    covered = AssemblyDocument.from_payload(payload, subsystems={row.ref: row.subsystem for row in original.entries})
    before, after = [api.validate(item, _case()) for item in (original, covered)]
    assert before.model_document["elements"][0]["parameters"]["point_b"] != after.model_document["elements"][0]["parameters"]["point_b"]
    assert original.entries[0].subsystem.payload["hardpoints"]["mount"][2] != .3


@pytest.mark.parametrize("spelling", ["path", "document", "memory"])
def test_the_three_assembly_spellings_are_one_run(tmp_path, spelling):
    assembly = _assembly()
    path = save_migrated_assembly(assembly, tmp_path)
    sources = {"path": path, "document": AssemblyDocument.load(path), "memory": assembly}
    actual, expected = [api.simulate(source, _case()) for source in (sources[spelling], assembly)]
    assert actual.status == expected.status == "success"
    np.testing.assert_array_equal(actual.raw.states, expected.raw.states)
    assert actual.compiled.model_payload == expected.compiled.model_payload


@pytest.mark.parametrize("damage", ["both", "none", "family"])
def test_invalid_case_reading_is_refused(damage):
    assembly, declared = migrate_v1_kc_case(benchmark_model(), mode="K")
    case = api.validate(assembly, declared).case_document
    if damage == "both":
        case["c"] = {"paths": ["fz"], "levels": 3, "maximum": 1}
    elif damage == "none":
        case.pop("k")
    else:
        case["family"] = "unknown"
    with pytest.raises(ValueError):
        api.validate(assembly, case)


def test_a_contract_model_document_is_not_an_assembly_document():
    with pytest.raises(AuthoringError, match="AssemblyDocument"):
        api.simulate({"contract": "multibody-model"}, _case())


def test_a_non_document_case_is_refused():
    with pytest.raises(AuthoringError, match="case input"):
        api.simulate(_assembly(), ["not-a-document"])


def test_case_path_and_memory_use_identical_bytes(tmp_path):
    case = tmp_path / "case.json"
    case.write_text(json.dumps(_case()), encoding="utf-8")
    first, second = [api.validate(_assembly(), item) for item in (case, _case())]
    assert first.case_payload == second.case_payload


def test_the_scattered_entries_are_retired_from_public_surface():
    import suspension_multibody as package
    for name in ("run_case", "run_dynamic_case", "run_vehicle_dynamics", "kc_case_run"):
        assert name not in package.__all__
        with pytest.raises(AttributeError):
            getattr(package, name)
