"""Vehicle-KC time-domain Adams gate tests."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from suspension_multibody.adams import AdamsProfile
from suspension_multibody.adams.time_domain import history_from_dynamic_bundle
from suspension_multibody.adams.vehicle_kc_time_domain import (
    _adams_piecewise_linear,
    _supported_roll_signal,
    validate_vehicle_kc_time_domain,
)
from suspension_multibody.api import run_dynamic_case
from suspension_multibody.schema import (
    DynamicCaseSpec,
    DynamicSolverSettings,
    FrontAxleModel,
    MassSpec,
    PrescribedMotion,
    TimeSignal,
    Vec3,
    VehicleBodyModel,
)


def _profile(tmp_path: Path) -> AdamsProfile:
    return AdamsProfile(
        name="fixture",
        home=str(tmp_path),
        executable="adams.bat",
        version="2024.1",
        license_file=None,
        template_id="fixture",
        subsystem_id="fixture",
        database_path=str(tmp_path),
        report_dictionary=None,
        export_fields=(),
        available=True,
        license_probe="passed",
        message="fixture",
    )


def _model() -> FrontAxleModel:
    return FrontAxleModel(
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


def _case() -> DynamicCaseSpec:
    return DynamicCaseSpec(
        mode="vehicle_kc_dynamic",
        solver=DynamicSolverSettings(end_time=0.02, step_size=0.01),
        vehicle=VehicleBodyModel(
            mass=1500.0,
            inertia=(
                (600_000.0, 0.0, 0.0),
                (0.0, 1_800_000.0, 0.0),
                (0.0, 0.0, 2_000_000.0),
            ),
            wheelbase=2800.0,
            front_track=1600.0,
            rear_track=1600.0,
        ),
        prescribed_motions=(
            PrescribedMotion(
                target="body_roll",
                displacement=TimeSignal(times=(0.0, 0.02), values=(0.0, 0.1)),
            ),
        ),
    )


def test_vehicle_kc_gate_uses_external_history_without_reference(tmp_path: Path) -> None:
    model = _model()
    case = _case()
    assert case.vehicle is not None
    reference = history_from_dynamic_bundle(
        run_dynamic_case(model, case),
        body=case.vehicle.name,
        channels=("body_roll",),
        units={"body_roll": "rad"},
    )

    def runner(_profile: AdamsProfile, request_path: Path, output_dir: Path) -> None:
        request = json.loads(request_path.read_text(encoding="utf-8"))
        assert request["analysis"] == "vehicle_kc_time_domain"
        assert "reference" not in request
        (output_dir / "adams_time_history.json").write_text(
            json.dumps(reference.as_dict()), encoding="utf-8"
        )

    result = validate_vehicle_kc_time_domain(
        _profile(tmp_path), model, case, runner=runner, output_dir=tmp_path / "gate"
    )

    assert result.ok


def test_vehicle_kc_roll_function_is_piecewise_linear() -> None:
    function = _adams_piecewise_linear(
        TimeSignal(times=(0.0, 1.0, 2.0), values=(0.0, 0.1, 0.0))
    )

    assert "TIME-1" in function
    assert "TIME-2" in function


def test_vehicle_kc_native_runner_rejects_unsupported_nonzero_pitch() -> None:
    case = _case().model_copy(
        update={
            "prescribed_motions": (
                PrescribedMotion(
                    target="body_pitch",
                    displacement=TimeSignal(constant=0.01),
                ),
            )
        }
    )

    with pytest.raises(ValueError, match="body_roll only"):
        _supported_roll_signal(case)
