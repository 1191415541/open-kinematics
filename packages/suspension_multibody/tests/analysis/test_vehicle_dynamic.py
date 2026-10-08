"""Vehicle time-domain result tests."""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.results import ResultEnvelope
from suspension_multibody.schema import (
    DynamicCaseSpec,
    DynamicSolverSettings,
    MassSpec,
    PrescribedMotion,
    TimeSignal,
    Vec3,
    VehicleBodyModel,
)
from suspension_multibody.schema.model import AxleDeclaration
from tests.simulation._replay_documents import replay


def _model() -> AxleDeclaration:
    return AxleDeclaration(
        hardpoints={
            "UPPER_INBOARD_FRONT": Vec3(x=0, y=-300, z=300),
            "UPPER_INBOARD_REAR": Vec3(x=300, y=-300, z=300),
            "UPPER_OUTBOARD": Vec3(x=150, y=-700, z=250),
            "LOWER_INBOARD_FRONT": Vec3(x=0, y=-320, z=0),
            "LOWER_INBOARD_REAR": Vec3(x=320, y=-320, z=0),
            "LOWER_OUTBOARD": Vec3(x=150, y=-720, z=50),
            "TIE_ROD_INBOARD": Vec3(x=100, y=-250, z=100),
            "TIE_ROD_OUTBOARD": Vec3(x=180, y=-700, z=100),
            "WHEEL_CENTER": Vec3(x=160, y=-760, z=150),
            "RACK_CENTER": Vec3(x=100, y=0, z=100),
        },
        mass=MassSpec(sprung_mass=1000.0),
    )


def _vehicle() -> VehicleBodyModel:
    return VehicleBodyModel(
        degrees_of_freedom=15,
        mass=1500.0,
        inertia=(
            (600_000.0, 0.0, 0.0),
            (0.0, 1_800_000.0, 0.0),
            (0.0, 0.0, 2_000_000.0),
        ),
        wheelbase=2800.0,
        front_track=1600.0,
        rear_track=1600.0,
    )


def test_vehicle_kc_dynamic_replays_body_roll() -> None:
    case = DynamicCaseSpec(
        mode="vehicle_kc_dynamic",
        solver=DynamicSolverSettings(end_time=0.02, step_size=0.01),
        vehicle=_vehicle(),
        prescribed_motions=(
            PrescribedMotion(
                target="body_roll",
                displacement=TimeSignal(times=(0.0, 0.02), values=(0.0, 0.1)),
            ),
        ),
    )
    result = replay(case)
    assert isinstance(result, ResultEnvelope)
    assert result.raw.status == "success"
    assert result.body_state("body.body")[-1, 3] == pytest.approx(np.cos(.05), abs=1e-7)


def test_legacy_vehicle_integrator_is_rejected() -> None:
    case = DynamicCaseSpec(
        mode="vehicle_dynamic",
        solver=DynamicSolverSettings(end_time=0.01, step_size=0.01),
        vehicle=_vehicle(),
    )

    with pytest.raises(ValueError, match="vehicle_kc_dynamic"):
        replay(case)
