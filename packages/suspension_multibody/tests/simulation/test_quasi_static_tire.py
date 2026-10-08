"""The same Wheel/Tire data is evaluated by static and dynamic studies."""
import numpy as np
import pytest

from suspension_multibody.api import simulate, validate
from suspension_multibody.authoring import (
    AssemblyDocument,
    SubsystemDocument,
    assemble_generic,
)
from suspension_multibody.authoring.properties import ElementPropertyDocument
from suspension_multibody.compilation.resolved import compile_resolved
from suspension_multibody.modeling.resolved import ResolvedSolvePlan

from ..physics.test_wheel_spin_boundary import wheel_assembly


def _source(stiffness=200_000):
    source = wheel_assembly()
    entries = {row.ref: row.subsystem for row in source.entries}
    wheel = entries["wheel"]
    properties = dict(wheel.properties)
    law = {"document": "element_properties", "schema_version": 1, "name": "linear-tire",
        "element_type": "tire", "model": "native_brush", "units": {"length": "m", "force": "N"},
        "parameters": {"unloaded_radius": .334, "maximum_compression": .1,
            "vertical_stiffness": stiffness, "vertical_damping": 0}}
    properties["tire"] = ElementPropertyDocument.from_payload(law)
    entries["wheel"] = SubsystemDocument.from_payload(wheel.to_payload(), template=wheel.template, properties=properties)
    payload = source.to_payload()
    payload["gravity"] = [0, 0, 0]
    return AssemblyDocument.from_payload(payload, subsystems=entries)


def _plan(study="quasi_static", *, active=True):
    return {"schema_version": 1, "name": "tire-probe", "study": study,
        "protocol": "kc_quasi_static" if study == "quasi_static" else "vehicle_dynamic",
        "samples": [0, .001], "solver": {"initialization_mode": "provided_consistent_state"},
        "boundaries": [{"name": "hold", "coordinate": "wheel.spin", "mode": "locked", "units": "rad", "value": 0}],
        "element_activation": [{"entity": "wheel.tire", "active": active}],
        "excitation": {"drive_mode": "pad", "k": {"pad_height_mm": [20]}} if study == "quasi_static" and active else
            {"c": {"loads": [{"fz": 0}], "load_marker": "wheel.hub"}} if study == "quasi_static" else {},
        "inputs": [], "outputs": []}


def _solve(stiffness=200_000, *, active=True):
    run = simulate(_source(stiffness), _plan(active=active))
    return run.compiled.model_document, run.result


def test_the_model_document_declares_the_tires_the_assembly_carries():
    source = _source()
    graph = assemble_generic(source).resolved_model()
    declared = validate(source, _plan()).model_document["tires"]
    assert len(declared) == 1
    assert declared[0]["name"] == "wheel.tire"
    assert declared[0]["body"] == "wheel.wheel"
    assert declared[0]["model"] == "native_brush"
    assert declared[0]["parameters"]["vertical_stiffness"] == 200_000
    assert declared[0]["parameters"]["unloaded_radius"] == pytest.approx(.334)
    assert declared[0]["parameters"]["longitudinal_friction_coefficient"] > 0
    inactive = compile_resolved(graph, ResolvedSolvePlan(_plan(active=False)))
    assert inactive.model_document["tires"] == []
    assert len(graph.to_document()["tires"]) == 1


def test_a_tire_declared_by_the_model_enters_the_residual():
    _, result = _solve()
    assert result.tire_ids == ("wheel.tire",)
    state = result.tire_state("wheel.tire")
    compression, force = state[:, 2], state[:, 4]
    assert np.all(compression > 0)
    assert np.all(force > 0)
    np.testing.assert_allclose(force, 200_000*compression, rtol=1e-9)


def test_changing_the_vertical_property_changes_the_response():
    _, soft = _solve()
    _, stiff = _solve(400_000)
    soft_state, stiff_state = soft.tire_state("wheel.tire"), stiff.tire_state("wheel.tire")
    np.testing.assert_allclose(soft_state[:, 2], stiff_state[:, 2])
    assert soft_state[0, 4] > 0
    assert stiff_state[0, 4] == pytest.approx(2*soft_state[0, 4], rel=1e-9)


def test_a_model_with_no_tire_produces_no_tire_row():
    _, result = _solve(active=False)
    assert result.tire_ids == ()
    assert "tire_output" not in result.named_blocks


def test_the_quasi_static_tire_is_the_same_entry_the_dynamic_reading_emits():
    graph = assemble_generic(_source()).resolved_model()
    quasi = compile_resolved(graph, ResolvedSolvePlan(_plan()))
    dynamic = compile_resolved(graph, ResolvedSolvePlan(_plan("dynamic")))
    assert quasi.request.model is dynamic.request.model is graph
    assert quasi.model_document["tires"] == dynamic.model_document["tires"]


def test_the_result_channel_reports_the_solved_compression():
    _, result = _solve()
    state = result.tire_state("wheel.tire")
    np.testing.assert_array_equal(state, result.raw.block("tire_output")[:, 0])
    assert np.all(state[:, 2] > 0)
    assert result.case_residuals()[1] < 1e-6


def test_the_neutral_coefficients_are_the_kernel_s_requirement():
    document, _ = _solve()
    coefficients = ("longitudinal_friction_coefficient", "lateral_friction_coefficient",
        "longitudinal_brush_stiffness", "lateral_brush_stiffness",
        "longitudinal_relaxation_length", "lateral_relaxation_length", "detached_relaxation_s")
    assert all(document["tires"][0]["parameters"][key] > 0 for key in coefficients)


def test_a_declaring_model_declares_a_physical_compression_limit():
    document, _ = _solve()
    parameters = document["tires"][0]["parameters"]
    assert 0 < parameters["maximum_compression"] < parameters["unloaded_radius"]


def test_declaring_a_tire_leaves_the_frozen_kinematics_untouched():
    _, inactive = _solve(active=False)
    _, active = _solve()
    for entity in active.body_ids:
        np.testing.assert_allclose(inactive.body_state(entity), active.body_state(entity), atol=1e-6)
