"""Every protocol validates entities on the same ordinary document route."""

from copy import deepcopy

import numpy as np
import pytest

from suspension_multibody import api
from suspension_multibody.authoring import AssemblyDocument, CaseDocument
from suspension_multibody.compilation.resolved import plan_from_case
from suspension_multibody.kernel.capabilities import kernel_capability_document
from suspension_multibody.results import ResultEnvelope

from ..authoring.test_generic_multibody import _assembly, _case

PROTOCOLS = ("kc_quasi_static", "axle_dynamic", "vehicle_kc", "vehicle_dynamic",
             "handling", "ride_four_post", "ride_random_road")


def _grid_case(protocol="kc_quasi_static"):
    return {**_case(), "family": protocol,
            "c": {"loads": [{"fz": 0}], "load_marker": "mechanism.input"}}


def test_all_protocols_are_declared_by_the_loaded_kernel():
    assert set(PROTOCOLS) <= set(kernel_capability_document()["case_families"])


@pytest.mark.parametrize("protocol", ["axle_dynamic", "vehicle_dynamic"])
def test_dynamic_protocols_compile_and_solve_the_identical_assembly(protocol):
    source = _assembly()
    run = api.simulate(source, {**_case(), "family": protocol})
    assert run.status == "success"
    assert run.compiled.request.model.name == source.name
    assert isinstance(run.result, ResultEnvelope)
    assert run.result.body_ids == ("mechanism.support", "mechanism.carriage")


@pytest.mark.parametrize("protocol", ["kc_quasi_static", "vehicle_kc"])
def test_grid_protocols_solve_the_same_generic_mechanism(protocol):
    run = api.simulate(_assembly(), _grid_case(protocol))
    assert run.status == "success"
    assert isinstance(run.result, ResultEnvelope)
    assert len(run.result.cases) == 1
    assert np.isfinite(run.result.body_state("mechanism.carriage")).all()


def test_missing_protocol_entities_are_refused_before_submission():
    for protocol, section, extra, message in (
        ("handling", "handling", {"steering": [{"actuator": "missing", "shape": "step", "amplitude": 0}]}, "steering actuators"),
        ("ride_four_post", "four_post", {"corners": [{"tire": "missing", "amplitude": 0, "frequency_hz": 1}]}, "tire IDs"),
        ("ride_random_road", "ride_random_road", {"wheels": [{"tire": "missing", "x_offset": 0, "y_offset": 0}], "speed_mps": 1, "road": {"kind": "sine", "amplitude": 0, "wavelength": 1}}, "tire IDs"),
    ):
        with pytest.raises(ValueError, match=message):
            api.validate(_assembly(), {**_case(), "family": protocol, section: extra})


def test_an_unknown_protocol_is_refused_by_name():
    with pytest.raises(ValueError, match="no_such_family"):
        api.validate(_assembly(), {**_case(), "family": "no_such_family"})


@pytest.mark.parametrize("assembly", [object(), {"kind": "model"}])
def test_both_model_halves_must_be_ordinary_authored_documents(assembly):
    with pytest.raises((TypeError, ValueError)):
        api.validate(assembly, _case())


def test_case_samples_and_inputs_remain_declared_data():
    run = {"schema_version": 1, "name": "sampled", "study": "dynamic",
           "protocol": "axle_dynamic", "samples": [0, .001, .002], "solver": {},
           "boundaries": [], "outputs": [], "inputs": [{"name": "load", "role": "body_wrench",
           "body": "mechanism.carriage", "values": [[0] * 6] * 3}]}
    compiled = api.validate(_assembly(), run)
    assert compiled.metadata["solve_plan"]["samples"] == [0, .001, .002]
    assert compiled.case_document["blobs"][0]["shape"] == [3, 6]
    assert compiled.case_document["blobs"][0]["body"] == "mechanism.carriage"
    assert plan_from_case(_case()).to_document()["samples"] == [0, .001, .002]


def test_a_missing_required_study_section_is_refused():
    with pytest.raises(ValueError, match="study"):
        CaseDocument({"schema_version": 1, "name": "bad", "samples": [0, .001]})


def test_a_dynamic_input_off_the_case_grid_is_refused():
    given = {"schema_version": 1, "name": "sampled", "study": "dynamic",
             "protocol": "axle_dynamic", "samples": [0, .001, .002], "solver": {},
             "boundaries": [], "outputs": [], "inputs": [{"name": "load", "role": "body_wrench",
             "body": "mechanism.carriage", "values": [[0] * 6] * 2}]}
    with pytest.raises(ValueError, match="shape"):
        api.validate(_assembly(), given)


def test_mode_independent_declarations_keep_the_same_model():
    source = _assembly()
    one = api.validate(source, _case())
    payload = deepcopy(source.to_payload())
    payload["mode"] = "C"
    two = api.validate(AssemblyDocument.from_payload(payload,
        subsystems={entry.ref: entry.subsystem for entry in source.entries}), _case())
    assert one.model_payload == two.model_payload
    assert one.case_payload == two.case_payload
