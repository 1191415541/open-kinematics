"""Random-road profiles agree with independently sampled per-tire tables."""

import numpy as np
import pytest

from suspension_multibody.api import simulate, validate
from suspension_multibody.authoring.loader import CaseDocument
from suspension_multibody.cases import (
    RandomRoadWheel,
    RoadComponent,
    ride_random_road_case_document,
    ride_random_road_signals,
)
from suspension_multibody.schema.solver import AxleSolverSettings

from ._family_documents import (
    assert_document_parity,
    family_case,
    fixture_documents,
    sampled_case,
)

_PROFILE = (RoadComponent(.004, 25.), RoadComponent(.002, 8., phase_rad=.7),
    RoadComponent(.0008, 2.5, phase_rad=2.1))
_SPEED_MPS = 20.


@pytest.fixture(scope="module")
def prepared():
    assembly, base = fixture_documents()
    names = [row["tire"] for row in base.to_payload()["inputs"] if row["role"] == "wheel_torque"]
    wheels = tuple(RandomRoadWheel(name, tuple(RoadComponent(row.amplitude_m, row.wavelength_m,
        phase_rad=row.phase_rad-(0. if index < 2 else 2.*np.pi*2.4/row.wavelength_m))
        for row in _PROFILE)) for index, name in enumerate(names))
    native = ride_random_road_case_document(name="random-road", wheels=wheels, speed_mps=_SPEED_MPS,
        times_s=tuple(base.to_payload()["samples"]), settings=AxleSolverSettings())
    return assembly, family_case(base, native), wheels


def test_the_family_matches_an_independently_expanded_road(prepared):
    assembly, case, wheels = prepared
    height, velocity = ride_random_road_signals(wheels, _SPEED_MPS, tuple(case.to_payload()["samples"]))
    produced = simulate(assembly, case).raw
    reference = simulate(assembly, sampled_case(case, height, velocity, "road_height", "tire")).raw
    assert produced.status == reference.status == "success"
    assert float(np.abs(produced.block("body_state")-reference.block("body_state")).max()) < 1e-12


def test_the_family_compiles_identical_documents_from_file_and_memory(prepared, tmp_path):
    assembly, case, _ = prepared
    compiled = assert_document_parity(assembly, case, tmp_path)
    assert compiled.case_document["ride_random_road"]["speed_mps"] == _SPEED_MPS
    assert compiled.case_document["ride_random_road"] == case.to_payload()["excitation"]["ride_random_road"]


def test_a_declared_protocol_still_validates_study_identity(prepared):
    assembly, case, _ = prepared
    wrong = case.to_payload()
    wrong["protocol"] = "vehicle_kc"
    with pytest.raises(ValueError, match="grid protocol"):
        validate(assembly, CaseDocument(wrong))


def test_a_component_without_a_wavelength_is_refused(prepared):
    assembly, case, _ = prepared
    wrong = case.to_payload()
    wrong["excitation"]["ride_random_road"]["wheels"][0]["components"][0]["wavelength_m"] = 0.
    with pytest.raises(Exception, match="wavelength"):
        simulate(assembly, wrong)


def test_an_unknown_wheel_is_refused(prepared):
    assembly, case, _ = prepared
    wrong = case.to_payload()
    wrong["excitation"]["ride_random_road"]["wheels"][0]["tire"] = "middle_left"
    with pytest.raises(ValueError, match="tire"):
        validate(assembly, wrong)
