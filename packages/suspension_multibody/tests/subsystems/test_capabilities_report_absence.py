"""Case axes refer to declared rig coordinates, with absent branches omitted."""

import pytest

from suspension_multibody.api import validate
from suspension_multibody.authoring.migration import migrate_v1_kc_case
from tests.benchmark_fixture import benchmark_model
from tests.composable.fixtures import trailing_arm_model


def test_declared_steering_offers_the_rack_axis():
    run = validate(*migrate_v1_kc_case(benchmark_model(), mode="K"))
    assert set(run.case_document["k"]["axis_map"]) == {"wheel", "rack"}


def test_absent_steering_omits_the_rack_axis_but_keeps_wheel_travel():
    run = validate(*migrate_v1_kc_case(trailing_arm_model(), mode="K"))
    assert set(run.case_document["k"]["axis_map"]) == {"wheel"}
    assert len(run.case_document["k"]["axis_map"]["wheel"]) == 2


def test_requiring_an_absent_coordinate_names_it():
    source, case = migrate_v1_kc_case(trailing_arm_model(), mode="K")
    payload = case.to_payload()
    payload["excitation"]["k"]["axis_map"]["rack"] = "rig.rack_drive"
    with pytest.raises(ValueError, match="rig.rack_drive"):
        validate(source, payload)
