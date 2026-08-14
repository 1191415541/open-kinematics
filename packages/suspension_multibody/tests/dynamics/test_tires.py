"""Combined-slip tire model tests."""

import math

import pytest

from suspension_multibody.dynamics import (
    FialaTireModel,
    Pac2002TireModel,
    TireKinematics,
)


@pytest.mark.parametrize(
    "model",
    [
        FialaTireModel(80_000.0, 120_000.0, 1.0, pneumatic_trail=40.0),
        Pac2002TireModel(80_000.0, 120_000.0, 1.0, pneumatic_trail=40.0),
    ],
)
def test_combined_slip_respects_friction_ellipse(model: object) -> None:
    force = model.evaluate(
        TireKinematics(normal_load=4_000.0, slip_angle=0.35, slip_ratio=0.35)
    )

    utilization = math.hypot(force.fx / 4_000.0, force.fy / 4_000.0)
    assert utilization <= 1.0 + 1e-12
    assert force.mz == pytest.approx(-40.0 * force.fy)


def test_fiala_and_pac2002_return_zero_at_zero_load() -> None:
    state = TireKinematics(normal_load=0.0, slip_angle=0.4, slip_ratio=0.5)

    assert FialaTireModel(80_000.0, 120_000.0, 1.0).evaluate(state).fz == 0.0
    assert Pac2002TireModel(80_000.0, 120_000.0, 1.0).evaluate(state).fz == 0.0


def test_pac2002_zero_slip_has_no_tangential_force() -> None:
    model = Pac2002TireModel(
        cornering_stiffness=80_000.0,
        longitudinal_stiffness=120_000.0,
        friction_coefficient=1.0,
        coefficients={
            "FNOMIN": 4_850.0,
            "PCX1": 1.65,
            "PDX1": 1.0,
            "PKX1": 20.0,
            "PHX1": 0.01,
            "PVX1": 0.02,
            "PCY1": 1.3,
            "PDY1": 1.0,
            "PKY1": -16.0,
            "PHY1": 0.01,
            "PVY1": 0.02,
        },
    )
    forces = model.evaluate(TireKinematics(normal_load=5_000.0))
    assert abs(forces.fx) < 1e-12
    assert abs(forces.fy) < 1e-12
