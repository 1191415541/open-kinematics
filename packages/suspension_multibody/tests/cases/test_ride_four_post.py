"""Four-post shapes agree with independently sampled road input tables."""

import numpy as np
import pytest

from suspension_multibody.api import simulate, validate
from suspension_multibody.authoring.loader import CaseDocument
from suspension_multibody.cases import (
    FourPostCorner,
    ride_four_post_case_document,
    ride_four_post_corner_signals,
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
    assembly, base = fixture_documents()
    names = [row["tire"] for row in base.to_payload()["inputs"] if row["role"] == "wheel_torque"]
    corners = tuple(FourPostCorner(name, amplitude_m=.002 if index < 2 else .0015,
        frequency_hz=8. if index < 2 else 6., phase_rad=(0., 0., .3, -.3)[index])
        for index, name in enumerate(names))
    native = ride_four_post_case_document(name="four-post", corners=corners,
        times_s=tuple(base.to_payload()["samples"]), settings=AxleSolverSettings())
    return assembly, family_case(base, native), corners


def test_the_family_matches_an_independently_sampled_excitation(prepared):
    assembly, case, corners = prepared
    height, velocity = ride_four_post_corner_signals(corners, tuple(case.to_payload()["samples"]))
    produced = simulate(assembly, case).raw
    reference = simulate(assembly, sampled_case(case, height, velocity, "road_height", "tire")).raw
    assert produced.status == reference.status == "success"
    assert float(np.abs(produced.block("body_state")-reference.block("body_state")).max()) < 1e-12


def test_the_family_compiles_identical_documents_from_file_and_memory(prepared, tmp_path):
    assembly, case, _ = prepared
    compiled = assert_document_parity(assembly, case, tmp_path)
    assert compiled.case_document["four_post"] == case.to_payload()["excitation"]["four_post"]


def test_a_declared_protocol_still_validates_study_identity(prepared):
    assembly, case, _ = prepared
    wrong = case.to_payload()
    wrong["protocol"] = "vehicle_kc"
    with pytest.raises(ValueError, match="grid protocol"):
        validate(assembly, CaseDocument(wrong))


def test_a_corner_without_a_known_tire_is_rejected(prepared):
    assembly, case, _ = prepared
    wrong = case.to_payload()
    wrong["excitation"]["four_post"]["corners"][0]["tire"] = "not_a_wheel"
    with pytest.raises(ValueError, match="tire"):
        validate(assembly, wrong)
