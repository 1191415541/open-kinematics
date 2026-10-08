"""The offline converter declares exactly the coordinates available to a rig."""

from copy import deepcopy

import numpy as np

from suspension_multibody.api import simulate, validate
from suspension_multibody.authoring.migration import migrate_v1_kc_case
from tests.benchmark_fixture import benchmark_model
from tests.composable.fixtures import trailing_arm_model


def _compiled(*, steering, wheel=(-10., 0., 10.), rack=(-5., 0., 5.)):
    source = benchmark_model() if steering else trailing_arm_model()
    return validate(*migrate_v1_kc_case(source, mode="K", wheel_values_mm=wheel,
        rack_values_mm=rack if steering else ()))


def _driven(compiled):
    return {row["name"] for row in compiled.model_document["joints"] if row["type"] == "driven_translation"}


def _named(section):
    mapping = section["axis_map"]
    return {*mapping.get("wheel", []), *([mapping["rack"]] if "rack" in mapping else [])}


def test_an_assembly_with_steering_offers_the_rack_coordinate():
    assert "kc_rig.sub.json.rack_drive" in _driven(_compiled(steering=True))


def test_an_assembly_without_steering_offers_only_wheel_travel():
    assert _driven(_compiled(steering=False)) == {"kc_rig.sub.json.wheel_drive_L", "kc_rig.sub.json.wheel_drive_R"}


def test_the_grid_keeps_the_rack_axis_when_the_assembly_has_steering():
    compiled = _compiled(steering=True)
    assert "rack" in compiled.case_document["k"]["axis_map"]
    assert len(compiled.case_document["k"]["wheel_values_mm"])*len(compiled.case_document["k"]["rack_values_mm"]) == 9


def test_the_grid_drops_the_rack_axis_when_it_cannot_be_driven():
    compiled = _compiled(steering=False)
    section = compiled.case_document["k"]
    assert "rack" not in section["axis_map"]
    assert section["rack_values_mm"] == []
    assert len(section["wheel_values_mm"]) == 3
    assert not any("rack" in name for name in _named(section))


def test_a_neutral_rack_is_an_explicit_single_value():
    section = _compiled(steering=True, rack=()).case_document["k"]
    assert section["rack_values_mm"] == [0]
    assert "rack" in section["axis_map"]


def test_the_rig_and_the_grid_shrink_together():
    for steering in (True, False):
        compiled = _compiled(steering=steering)
        assert _named(compiled.case_document["k"]) == _driven(compiled)


def test_an_explicit_axis_map_never_names_an_undrivable_rack():
    compiled = _compiled(steering=False)
    names = _named(compiled.case_document["k"])
    assert names == _driven(compiled)
    assert not any("rack" in name for name in names)


def test_a_no_steering_axle_runs_through_the_public_entry():
    source, case = migrate_v1_kc_case(trailing_arm_model(), mode="K", wheel_values_mm=(-10., 0., 10.))
    before = deepcopy(case.to_payload())
    result = simulate(source, case)
    assert len(result.raw.cases) == 3
    assert case.to_payload() == before
    assert not any("rack" in body for body in result.result.body_ids)
    centers = [result.result.frame_pose("wheel.sub.json.wheel_center_"+side)[:, 2, 3] for side in ("L", "R")]
    np.testing.assert_allclose(centers[0], centers[1], atol=1e-10)
    np.testing.assert_allclose(centers[0][::2], centers[0][2]+np.array([-.01, 0, .01]), atol=1e-8)
    for index in range(3):
        assert result.result.case_residuals(index)[0] < 1e-8
    with_steering = _compiled(steering=True)
    assert "rack" in with_steering.case_document["k"]["axis_map"]
