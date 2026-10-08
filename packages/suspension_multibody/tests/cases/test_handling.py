"""Handling shapes and independently expanded tables use the same compiler."""

import numpy as np
import pytest

from suspension_multibody.api import simulate, validate
from suspension_multibody.authoring.loader import CaseDocument
from suspension_multibody.cases import (
    SteeringShape,
    handling_case_document,
    handling_steering_signals,
)
from suspension_multibody.schema.solver import AxleSolverSettings

from ._family_documents import (
    assert_document_parity,
    family_case,
    fixture_documents,
    sampled_case,
)


@pytest.fixture(scope="module")
def prepared():
    return fixture_documents(gravity=True)


def _case(base, shape):
    actuator = next(row["actuator"] for row in base.to_payload()["inputs"] if row["role"] == "steering_target")
    shapes = (SteeringShape(actuator, shape, amplitude=.008, start_s=.0005,
        rise_s=.0005, frequency_hz=4., phase_rad=0.),)
    native = handling_case_document(name="handling-"+shape, shapes=shapes,
        times_s=tuple(base.to_payload()["samples"]), settings=AxleSolverSettings())
    return family_case(base, native), shapes


@pytest.mark.parametrize("shape", ["constant", "ramp", "step", "sine"])
def test_the_family_matches_an_independently_expanded_manoeuvre(prepared, shape):
    assembly, base = prepared
    case, shapes = _case(base, shape)
    target, rate = handling_steering_signals(shapes, tuple(case.to_payload()["samples"]))
    produced = simulate(assembly, case).raw
    reference = simulate(assembly, sampled_case(case, target, rate, "steering_target", "actuator")).raw
    assert produced.status == reference.status == "success"
    difference = np.abs(produced.block("body_state")-reference.block("body_state"))
    assert float(difference.max()) < 1e-12


def test_the_family_compiles_identical_documents_from_file_and_memory(prepared, tmp_path):
    assembly, base = prepared
    case, _ = _case(base, "ramp")
    compiled = assert_document_parity(assembly, case, tmp_path)
    assert compiled.case_document["handling"] == case.to_payload()["excitation"]["handling"]


def test_a_declared_protocol_still_validates_study_identity(prepared):
    assembly, base = prepared
    case, _ = _case(base, "ramp")
    wrong = case.to_payload()
    wrong["protocol"] = "vehicle_kc"
    with pytest.raises(ValueError, match="grid protocol"):
        validate(assembly, CaseDocument(wrong))


def test_a_closed_loop_manoeuvre_is_refused_by_name(prepared):
    assembly, base = prepared
    case, _ = _case(base, "ramp")
    wrong = case.to_payload()
    wrong["excitation"]["handling"]["steering"][0]["shape"] = "iso_lane_change"
    with pytest.raises(Exception, match="not open-loop|iso_lane_change|constant.*ramp"):
        simulate(assembly, wrong)
