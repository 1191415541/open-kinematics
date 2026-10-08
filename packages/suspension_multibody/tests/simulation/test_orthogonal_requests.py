"""Study and protocol are explicit run facts over one immutable physical graph."""

from copy import deepcopy

import pytest
from suspension_contracts import ContractError

from suspension_multibody import api
from suspension_multibody.authoring.loader import DocumentLoader
from suspension_multibody.compilation.resolved import compile_resolved, plan_from_case
from suspension_multibody.modeling.resolved import ResolvedModel, ResolvedSolvePlan
from suspension_multibody.simulation import SimulationRequest

from ._documents import PROTOCOLS, documents


def test_a_named_rig_and_a_named_protocol_need_not_spell_the_same():
    request = SimulationRequest(assembly="generic", rig="custom_bench", family="axle_dynamic", study="dynamic")
    assert request.rig == "custom_bench"
    assert request.family == "axle_dynamic"
    assert request.study == "dynamic"


def test_naming_only_a_rig_does_not_infer_a_protocol():
    with pytest.raises(ValueError, match="explicit protocol"):
        SimulationRequest(assembly="generic", rig="kc_quasi_static")


def test_an_unknown_study_is_refused_before_submission():
    _, case = documents()
    plan = plan_from_case(case).to_document()
    plan["study"] = "implicit_study"
    with pytest.raises(ContractError):
        ResolvedSolvePlan(plan)


def test_a_plan_needs_an_explicit_study():
    _, case = documents()
    plan = plan_from_case(case).to_document()
    del plan["study"]
    with pytest.raises(ContractError):
        ResolvedSolvePlan(plan)


@pytest.mark.parametrize("protocol", PROTOCOLS)
def test_every_protocol_compiles_the_same_loaded_model(protocol):
    source, case = documents(protocol)
    loaded = DocumentLoader().load(source, case)
    model = loaded.resolve()
    compiled = compile_resolved(model, plan_from_case(case))
    assert compiled.request.model is model
    assert compiled.metadata["compiler"] == "ResolvedModelCompiler"
    assert compiled.metadata["model_fingerprint"] == model.fingerprint
    assert compiled.case_document["family"] == protocol
    assert compiled.model_document["units"]["length"] == "m"


def test_one_model_has_the_same_fingerprint_under_two_studies():
    source, quasi_case = documents("kc_quasi_static")
    _, dynamic_case = documents("axle_dynamic")
    model = DocumentLoader().load(source, dynamic_case).resolve()
    quasi = compile_resolved(model, plan_from_case(quasi_case))
    dynamic = compile_resolved(model, plan_from_case(dynamic_case))
    assert quasi.request.model is dynamic.request.model is model
    assert quasi.metadata["study"] == "quasi_static"
    assert dynamic.metadata["study"] == "dynamic"
    assert quasi.metadata["model_fingerprint"] == dynamic.metadata["model_fingerprint"]
    for key in ("bodies", "frames", "joints", "elements", "tires", "coordinates"):
        assert quasi.model_document.get(key, []) == dynamic.model_document.get(key, [])


def test_a_request_that_contradicts_its_protocol_is_refused():
    source, case = documents()
    model = DocumentLoader().load(source, case).resolve()
    request = SimulationRequest(assembly="generic", family="vehicle_kc", model=model)
    with pytest.raises(ValueError, match="disagrees"):
        compile_resolved(model, plan_from_case(case), request=request)


def test_an_unknown_protocol_is_refused_before_submission():
    source, case = documents()
    model = DocumentLoader().load(source, case).resolve()
    plan = plan_from_case(case).to_document()
    plan["protocol"] = "unregistered_protocol"
    with pytest.raises((ValueError, ContractError), match="protocol|family"):
        compile_resolved(model, ResolvedSolvePlan(plan))


def test_memory_and_saved_documents_resolve_to_one_graph(tmp_path):
    from suspension_multibody.authoring.migration import save_migrated_assembly

    source, case = documents()
    saved = save_migrated_assembly(source, tmp_path)
    memory = api.validate(source, case)
    file = api.validate(saved, case)
    assert isinstance(memory.request.model, ResolvedModel)
    assert isinstance(file.request.model, ResolvedModel)
    assert memory.request.model.fingerprint == file.request.model.fingerprint
    assert memory.model_payload == file.model_payload
    assert memory.case_payload == file.case_payload


def test_compilation_requires_the_resolved_graph():
    source, case = documents()
    with pytest.raises(TypeError, match="ResolvedModel"):
        compile_resolved(source, plan_from_case(case))


def test_the_compiled_metadata_describes_the_actual_plan():
    source, case = documents("kc_quasi_static")
    before = deepcopy(case)
    result = api.validate(source, case)
    assert case == before
    assert result.metadata["solve_plan"]["protocol"] == "kc_quasi_static"
    assert result.metadata["study"] == "quasi_static"
    assert result.metadata["solve_plan"]["excitation"]["c"] == case["c"]


def test_driven_coordinates_follow_the_explicit_boundary():
    source, case = documents()
    model = DocumentLoader().load(source, case).resolve()
    original = model.to_document()
    free_plan = plan_from_case(case).to_document()
    locked_plan = deepcopy(free_plan)
    locked_plan["boundaries"] = [{"name": "hold", "coordinate": "wheel.spin", "mode": "locked", "units": "rad", "value": 0}]
    free = compile_resolved(model, ResolvedSolvePlan(free_plan))
    locked = compile_resolved(model, ResolvedSolvePlan(locked_plan))
    assert not any(row["type"] == "driven_rotation" for row in free.model_document["joints"])
    assert sum(row["type"] == "driven_rotation" for row in locked.model_document["joints"]) == 1
    assert model.to_document() == original
    assert locked.request.model is free.request.model is model
