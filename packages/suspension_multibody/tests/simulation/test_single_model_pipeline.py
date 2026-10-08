"""All declarations reach one immutable graph emitter and motion compiler."""

from dataclasses import replace

import numpy as np
import pytest
from suspension_contracts import ContractError, unpack_container

from suspension_multibody.api import validate
from suspension_multibody.authoring.loader import DocumentLoader
from suspension_multibody.compilation.resolved import compile_resolved, plan_from_case
from suspension_multibody.modeling.resolved import ResolvedModel, ResolvedSolvePlan
from suspension_multibody.simulation import (
    SimulationRequest,
    run_compiled,
)

from ..authoring.test_generic_multibody import _assembly, _case
from ..authoring.test_unified_document_loader import EXAMPLES
from ..physics.test_wheel_spin_boundary import plan, wheel_assembly


def test_python_document_and_resolved_request_use_same_compiler():
    declaration, case = _assembly(), _case()
    model = DocumentLoader().load(declaration, case).resolve()
    resolved = plan_from_case(case)
    request = SimulationRequest(assembly="an-arbitrary-machine", family=case["family"], model=model, case=resolved)
    compiled = compile_resolved(model, resolved, request=request)
    assert compiled.request.model is model
    authored = validate(declaration, case)
    assert compiled.model_payload == authored.model_payload
    assert compiled.case_payload == authored.case_payload
    assert compiled.metadata["compiler"] == authored.metadata["compiler"] == "ResolvedModelCompiler"
    assert compiled.metadata["model_fingerprint"] == authored.metadata["model_fingerprint"] == model.fingerprint
    assert run_compiled(compiled).status == "success"


def test_file_and_memory_pin_tir_payload_before_compiling():
    loader = DocumentLoader(resource_root=EXAMPLES)
    loaded = loader.load("tire.assembly.json", "dynamic.case.json")
    first, second = loaded.resolve(), loader.load(loaded.assembly, loaded.case).resolve()
    assert first.resource_payload and first.resource_payload == second.resource_payload
    run = plan_from_case(loaded.case.to_payload())
    one, two = compile_resolved(first, run), compile_resolved(second, run)
    assert one.model_payload == two.model_payload
    _, blob = unpack_container(one.model_payload)
    assert blob == first.resource_payload
    assert run_compiled(one).status == "success"
    changed = ResolvedModel(first.to_document(), bytes([first.resource_payload[0] ^ 1]) + first.resource_payload[1:])
    assert changed.fingerprint != first.fingerprint


@pytest.mark.parametrize("family", ["axle_dynamic", "vehicle_dynamic"])
def test_protocol_never_selects_or_rebuilds_model_entities(family):
    source = _assembly()
    case = {**_case(), "family": family}
    model = DocumentLoader().load(source, case).resolve()
    compiled = compile_resolved(model, plan_from_case(case))
    assert compiled.model_document["bodies"] == model.to_document()["bodies"]
    assert compiled.model_document["joints"] == model.to_document()["joints"]
    assert run_compiled(compiled).status == "success"


def test_study_change_keeps_physical_model_fingerprint():
    model = DocumentLoader().load(_assembly(), _case()).resolve()
    dynamic = compile_resolved(model, plan_from_case(_case()))
    case = {**_case(), "family": "kc_quasi_static", "c": {"loads": [{"fz": 0}], "load_marker": "mechanism.input"}}
    static = compile_resolved(model, plan_from_case(case))
    assert dynamic.metadata["model_fingerprint"] == static.metadata["model_fingerprint"]
    for key in ("bodies", "joints", "elements", "tires"):
        assert dynamic.model_document.get(key) == static.model_document.get(key)
    assert static.metadata["study"] == "quasi_static"


def test_explicit_activation_changes_the_run_without_mutating_wheel_model():
    model = DocumentLoader().load(wheel_assembly(), _case()).resolve()
    given = model.to_document()
    run = plan("locked").to_document()
    run["element_activation"] = [{"entity": "wheel.tire", "active": False}]
    disabled = compile_resolved(model, ResolvedSolvePlan(run))
    active = compile_resolved(model, plan("locked"))
    assert disabled.model_document["tires"] == []
    assert active.model_document["tires"] == given["tires"]
    assert disabled.metadata["model_fingerprint"] == active.metadata["model_fingerprint"]
    assert model.to_document() == given
    assert run_compiled(disabled).status == "success"


@pytest.mark.parametrize("activation, message", [
    ([{"entity": "missing", "active": False}], "unknown force element"),
    ([{"entity": "wheel.tire", "active": False}]*2, "duplicate activation"),
])
def test_activation_rejects_unknown_or_repeated_element_ids(activation, message):
    model = DocumentLoader().load(wheel_assembly(), _case()).resolve()
    run = plan().to_document()
    run["element_activation"] = activation
    with pytest.raises(ValueError, match=message):
        compile_resolved(model, ResolvedSolvePlan(run))


def test_compiled_motion_reaches_native_without_payload_patching():
    samples = np.linspace(0, .06, 121).tolist()
    speed = 300
    raw = plan("prescribed_angle", samples=samples, values=[speed * t for t in samples], rates=[speed] * len(samples)).to_document()
    raw["initial_state"] = {"wheel.wheel": {"omega": [0, speed, 0]}}
    raw["solver"] = {"initialization_mode": "provided_consistent_state", "internal_step_s": .0001}
    model = DocumentLoader().load(wheel_assembly(), raw).resolve()
    compiled = compile_resolved(model, ResolvedSolvePlan(raw))
    assert sum(row["type"] == "revolute" for row in compiled.model_document["joints"]) == 1
    assert sum(row["type"] == "driven_rotation" for row in compiled.model_document["joints"]) == 1
    result = run_compiled(compiled)
    index = result.raw.body_names.index("wheel.wheel")
    np.testing.assert_allclose(result.raw.states[:, index, 11], speed, atol=1e-7)


def test_ir_compiler_refuses_stale_kernel_motion_capability(monkeypatch):
    from suspension_multibody.kernel import capabilities

    model = DocumentLoader().load(wheel_assembly(), plan().to_document()).resolve()
    monkeypatch.setattr(capabilities, "kernel_capability_document", lambda: {})
    with pytest.raises(ValueError, match="motion version"):
        compile_resolved(model, plan())


def test_ir_compiler_rejects_missing_binary_resources():
    bundle = DocumentLoader(resource_root=EXAMPLES).load("tire.assembly.json", "dynamic.case.json")
    with pytest.raises(ValueError, match="pinned payload"):
        compile_resolved(ResolvedModel(bundle.resolve().to_document()), plan_from_case(bundle.case.to_payload()))


def test_plan_cannot_smuggle_model_entities_through_excitation():
    raw = plan("free").to_document()
    raw["excitation"] = {"bodies": []}
    with pytest.raises(ContractError, match="cannot redefine"):
        ResolvedSolvePlan(raw)


def test_ir_request_rejects_disagreeing_protocol():
    model = DocumentLoader().load(_assembly(), _case()).resolve()
    request = SimulationRequest(assembly="generic", family="axle_dynamic", model=model, case=plan_from_case(_case()))
    with pytest.raises(ValueError, match="disagrees"):
        compile_resolved(model, request.case, request=replace(request, family="vehicle_dynamic"))


@pytest.mark.parametrize("family", ["kc_quasi_static", "vehicle_kc", "axle_dynamic", "vehicle_dynamic", "handling", "ride_four_post", "ride_random_road"])
def test_every_native_protocol_consumes_the_same_ir(family):
    model = DocumentLoader().load(wheel_assembly(), _case()).resolve()
    graph = model.to_document()
    graph["elements"].append({"name": "servo", "type": "steering_actuator", "target": "turntable", "parameters": {
        "type": "rotation", "body": "wheel.wheel", "reaction_body": "support.carrier",
        "axis_local": [0, 1, 0], "stiffness": 1000, "damping": 10}})
    model = ResolvedModel(graph, model.resource_payload)
    case = {**_case(), "family": family, "solver": {"initialization_mode": "provided_consistent_state"}}
    if family in {"kc_quasi_static", "vehicle_kc"}:
        case["c"] = {"loads": [{"fz": 0}], "load_marker": "wheel.hub"}
    elif family == "handling":
        case["handling"] = {"steering": [{"actuator": "turntable", "shape": "constant", "amplitude_rad": 0}]}
    elif family == "ride_four_post":
        case["four_post"] = {"corners": [{"tire": "wheel.tire", "amplitude_m": 0, "frequency_hz": 1}]}
    elif family == "ride_random_road":
        case["ride_random_road"] = {"speed_mps": 1, "wheels": [{"tire": "wheel.tire", "components": [{"amplitude_m": 0, "wavelength_m": 1}]}]}
    compiled = compile_resolved(model, plan_from_case(case))
    assert compiled.metadata["model_fingerprint"] == model.fingerprint
    assert compiled.model_document["elements"] == graph["elements"]
    assert run_compiled(compiled).status == "success"


def test_unknown_protocol_capability_is_rejected_at_compile():
    model = DocumentLoader().load(_assembly(), _case()).resolve()
    raw = {**_case(), "family": "comparison"}
    with pytest.raises(ValueError, match="case protocol"):
        compile_resolved(model, plan_from_case(raw))


@pytest.mark.parametrize("family", ["kc_quasi_static", "vehicle_kc", "axle_dynamic", "vehicle_dynamic", "ride_four_post", "ride_random_road"])
def test_spin_boundary_is_overlaid_after_each_protocol_expands(family):
    raw = plan().to_document()
    raw["protocol"] = family
    raw["study"] = "quasi_static" if family in {"kc_quasi_static", "vehicle_kc"} else "dynamic"
    raw["solver"] = {"initialization_mode": "provided_consistent_state"}
    if family in {"kc_quasi_static", "vehicle_kc"}:
        raw["excitation"] = {"c": {"loads": [{"fz": 0}], "load_marker": "wheel.hub"}}
    elif family == "ride_four_post":
        raw["excitation"] = {"four_post": {"corners": [{"tire": "wheel.tire", "amplitude_m": 0}]}}
    elif family == "ride_random_road":
        raw["excitation"] = {"ride_random_road": {"wheels": [{"tire": "wheel.tire", "components": [{"amplitude_m": 0, "wavelength_m": 1}]}]}}
    model = DocumentLoader().load(wheel_assembly(), raw).resolve()
    result = run_compiled(compile_resolved(model, ResolvedSolvePlan(raw)))
    assert result.status == "success"
    index = result.raw.body_names.index("wheel.wheel")
    np.testing.assert_allclose(result.raw.states[:, index, 3:7], [[1, 0, 0, 0]] * 3, atol=1e-10)


def test_irregular_samples_preserve_native_times_and_model():
    raw = plan("free", samples=[0, .0007, .002]).to_document()
    raw["solver"] = {"initialization_mode": "provided_consistent_state"}
    model = DocumentLoader().load(wheel_assembly(), raw).resolve()
    result = run_compiled(compile_resolved(model, ResolvedSolvePlan(raw)))
    np.testing.assert_array_equal(result.raw.times_s, raw["samples"])
    assert result.compiled.metadata["model_fingerprint"] == model.fingerprint
