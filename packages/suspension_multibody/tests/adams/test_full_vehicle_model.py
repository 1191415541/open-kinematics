"""Real Adams source-input importer tests."""

from pathlib import Path

import pytest

from suspension_multibody.adams import (
    build_adams_vehicle_model,
    load_adams_full_vehicle_input,
)

_CASE = Path("artifacts/adams/correlation-reference-real-si/handling-pac2002-v1/step_steer")


@pytest.mark.skipif(not _CASE.is_dir(), reason="real Adams reference artifacts are unavailable")
def test_importer_uses_adams_source_files_and_builds_full_model() -> None:
    data = load_adams_full_vehicle_input(_CASE)
    model = build_adams_vehicle_model(data)

    assert data.initial_forward_speed_mps == pytest.approx(16.667)
    assert data.pac2002_coefficients["PCY1"] == pytest.approx(1.3507)
    assert model.name.startswith("Demo_Vehicle_Variants")
    assert model.chassis.mass == pytest.approx(1399.735175708)
    assert model.wheels[0].tire.unloaded_radius == pytest.approx(344.0)
    assert model.wheels[0].tire.pac2002_coefficients["PDY1"] == pytest.approx(1.0489)
    assert data.spring_curve[0] == pytest.approx((-100.0, -12_500.0))
    assert data.damper_curve[0] == pytest.approx((-1270.0, -1495.5))
    assert data.bumpstop_curve[-1] == pytest.approx((54.0, 31_050.0))
    assert model.front_axle.bodies[1].inertia[0][0] > 1_000.0
    assert model.rear_axle.rack_fixed_to_chassis
    assert model.wheels[0].tire.cornering_stiffness == pytest.approx(21.92 * 4_850.0)
    assert data.unsupported_user_functions
