"""An absent steering subsystem produces no rack body, coordinate or channel."""

import numpy as np
import pytest

from suspension_multibody.api import simulate, validate
from suspension_multibody.authoring.migration import migrate_v1_kc_case
from tests.benchmark_fixture import benchmark_model


def _documents(steering=True, remove_center=False):
    model = benchmark_model()
    if not steering:
        # Absence is authored topology, so the migration uses an explicit trailing-arm template.
        from tests.composable.fixtures import trailing_arm_model
        model = trailing_arm_model()
    if remove_center:
        model = model.model_copy(update={"hardpoints": {key: value for key, value in model.hardpoints.items() if key != "rack_center"}})
    return migrate_v1_kc_case(model, mode="K", wheel_values_mm=(-10., 0., 10.))


def test_the_model_declares_no_rack_coordinate_without_one():
    compiled = validate(*_documents(False, True))
    joints = compiled.model_document["joints"]
    driven = [row["target"] for row in joints if row["type"] == "driven_translation"]
    assert driven == ["kc_rig.sub.json.wheel_drive_L", "kc_rig.sub.json.wheel_drive_R"]
    assert not any("rack" in row["name"] for row in compiled.model_document["bodies"])


def test_the_model_still_declares_the_rack_when_it_has_one():
    compiled = validate(*_documents())
    assert "kc_rig.sub.json.rack_drive" in [row.get("target") for row in compiled.model_document["joints"]]


def test_a_no_steering_assembly_runs_through_the_public_entry():
    result = simulate(*_documents(False, True)).result
    assert result.status == "success"
    assert len(result.cases) == 3
    for index in range(3):
        assert max(result.case_residuals(index)) < 1e-6


def test_the_rack_channel_disappears_rather_than_reporting_zero():
    run = simulate(*_documents(False))
    axes = run.compiled.case_document["k"]["axis_map"]
    assert set(axes) == {"wheel"}
    assert not any("rack" in name for name in run.result.body_ids + run.result.constraint_ids)
    wheel = run.result.frame_pose("wheel.sub.json.wheel_center_L")
    heights = [wheel[run.result.case_samples(index).stop - 1, 2, 3] for index in range(3)]
    np.testing.assert_allclose(np.diff(heights), [.01, .01], atol=1e-7)


def test_the_steering_run_still_reports_the_rack_channel():
    run = simulate(*_documents())
    assert run.compiled.case_document["k"]["axis_map"]["rack"] == "kc_rig.sub.json.rack_drive"
    assert any("rack" in name for name in run.result.body_ids)


def test_a_steering_rig_cannot_bind_to_an_assembly_without_its_port():
    assembly, case = _documents(False)
    declared = case.to_payload()
    declared["excitation"]["k"]["axis_map"]["rack"] = "missing.rack"
    with pytest.raises(ValueError, match="missing.rack"):
        validate(assembly, declared)
