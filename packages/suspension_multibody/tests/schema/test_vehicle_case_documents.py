"""Analysis documents retain excitation, sample grids and solver declarations."""

import pytest

from suspension_multibody.authoring.errors import AuthoringError
from suspension_multibody.authoring.loader import CaseDocument
from suspension_multibody.authoring.migration import (
    migrate_v1_case,
    migrate_v1_vehicle_case,
)
from suspension_multibody.cases.ride_four_post import FourPostCorner
from suspension_multibody.cases.ride_four_post import (
    case_document as four_post_document,
)
from suspension_multibody.cases.ride_random_road import RandomRoadWheel, RoadComponent
from suspension_multibody.cases.ride_random_road import (
    case_document as random_road_document,
)
from suspension_multibody.compilation.resolved import plan_from_case
from suspension_multibody.schema.solver import AxleSolverSettings
from tests.vehicle.vehicle_fixtures import _case, _vehicle

TIMES = (0., .001, .002)
SETTINGS = AxleSolverSettings(initialization_mode="provided_consistent_state")
IDS = {"tire:front_left": "wheel_front_left.front_left"}


def _four():
    return four_post_document(name="four", corners=(FourPostCorner("front_left", .01, .02, 1.5, .5),),
        times_s=TIMES, settings=SETTINGS)


def _road():
    return random_road_document(name="road", wheels=(RandomRoadWheel("front_left",
        (RoadComponent(.01, 5.), RoadComponent(.002, 1., 1.))),), speed_mps=20.,
        times_s=TIMES, settings=SETTINGS)


def test_a_four_post_case_reads_its_own_corners(tmp_path):
    migrated = migrate_v1_case(_four(), entity_ids=IDS)
    path = migrated.save(tmp_path / "case.json")
    import json

    again = CaseDocument(json.loads(path.read_text(encoding="utf-8")))
    assert again.to_payload() == migrated.to_payload()
    payload = again.to_payload()
    assert payload["samples"] == list(TIMES)
    assert payload["excitation"]["four_post"]["corners"] == [{"tire": IDS["tire:front_left"],
        "offset_m": .01, "amplitude_m": .02, "frequency_hz": 1.5, "phase_rad": .5}]


@pytest.mark.parametrize("section,factory", [("four_post", _four), ("ride_random_road", _road)])
def test_a_family_document_without_its_section_is_refused(section, factory):
    payload = factory()
    payload.pop(section)
    with pytest.raises(AuthoringError, match=section):
        CaseDocument(payload)


def test_a_four_post_document_without_corners_is_refused():
    payload = _four()
    payload["four_post"]["corners"] = []
    with pytest.raises(AuthoringError):
        CaseDocument(payload)


def test_a_random_road_case_reads_its_own_profiles():
    payload = migrate_v1_case(_road(), entity_ids=IDS).to_payload()
    road = payload["excitation"]["ride_random_road"]
    assert road["speed_mps"] == 20.
    assert road["wheels"] == [{"tire": IDS["tire:front_left"], "components": [
        {"amplitude_m": .01, "wavelength_m": 5., "phase_rad": 0.},
        {"amplitude_m": .002, "wavelength_m": 1., "phase_rad": 1.}]}]
    assert payload["samples"] == list(TIMES)


def test_a_vehicle_case_without_explicit_entity_mapping_is_refused():
    with pytest.raises(ValueError, match="unmapped entity"):
        migrate_v1_case(_four(), entity_ids={})


def test_the_vehicle_case_carries_the_window_the_document_states():
    model = _vehicle()
    source = _case(model)
    source = source.model_copy(update={"solver": source.solver.model_copy(update={"end_time": .002})})
    _, case = migrate_v1_vehicle_case(source)
    assert case.to_payload()["samples"] == list(TIMES)
    assert case.to_payload()["name"] == source.name


@pytest.mark.parametrize("mode", ["provided_consistent_state", "static_equilibrium"])
def test_a_family_document_preserves_its_explicit_initialization(mode):
    payload = _four()
    payload["solver"]["initialization_mode"] = mode
    assert migrate_v1_case(payload, entity_ids=IDS).to_payload()["solver"]["initialization_mode"] == mode


def test_a_non_positive_step_is_refused():
    payload = _four()
    payload["time"]["step_s"] = 0.
    with pytest.raises(ValueError, match="positive"):
        plan_from_case(payload)


def test_each_family_states_its_inputs_in_its_own_section():
    payload = _four()
    payload["family"] = "ride_random_road"
    with pytest.raises(AuthoringError, match="ride_random_road"):
        CaseDocument(payload)
