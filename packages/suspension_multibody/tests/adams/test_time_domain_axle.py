"""Axle time-domain Adams gate tests."""

from __future__ import annotations

import json
from pathlib import Path

from suspension_multibody.adams import AdamsProfile
from suspension_multibody.adams.time_domain import history_from_dynamic_bundle
from suspension_multibody.adams.time_domain_gate import validate_axle_time_domain
from suspension_multibody.api import run_dynamic_case
from suspension_multibody.schema import (
    DynamicCaseSpec,
    DynamicSolverSettings,
    FrontAxleModel,
    MassSpec,
    PrescribedMotion,
    TimeSignal,
    Vec3,
    WrenchInput,
    WrenchSignal,
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
        mode="axle_dynamic",
        solver=DynamicSolverSettings(end_time=0.02, step_size=0.01),
        prescribed_motions=(
            PrescribedMotion(
                target="wheel_travel_left",
                displacement=TimeSignal(times=(0.0, 0.02), values=(0.0, 5.0)),
            ),
        ),
        wrench_inputs=(
            WrenchInput(
                target="right",
                wrench=WrenchSignal(fz=TimeSignal(constant=25.0)),
            ),
        ),
    )


def test_axle_gate_serializes_inputs_without_reference_values(tmp_path: Path) -> None:
    model = _model()
    case = _case()
    reference = history_from_dynamic_bundle(
        run_dynamic_case(model, case),
        body="axle",
        channels=(
            "left_wheel_center_x",
            "left_wheel_center_y",
            "left_wheel_center_z",
            "left_camber_deg",
            "left_toe_deg",
            "right_wheel_center_x",
            "right_wheel_center_y",
            "right_wheel_center_z",
            "right_camber_deg",
            "right_toe_deg",
        ),
    )

    def runner(_profile: AdamsProfile, request_path: Path, output_dir: Path) -> None:
        request = json.loads(request_path.read_text(encoding="utf-8"))
        assert request["analysis"] == "axle_time_domain"
        assert "reference" not in request
        assert request["case"]["wrench_inputs"][0]["wrench"]["fz"]["constant"] == 25.0
        (output_dir / "adams_time_history.json").write_text(
            json.dumps(reference.as_dict()), encoding="utf-8"
        )

    result = validate_axle_time_domain(
        _profile(tmp_path), model, case, runner=runner, output_dir=tmp_path / "gate"
    )

    assert result.ok
    report = json.loads(Path(result.output_path).read_text(encoding="utf-8"))
    assert report["runner_invoked"]
    assert report["comparison"]["passed"]


def test_axle_gate_rejects_runner_without_result(tmp_path: Path) -> None:
    result = validate_axle_time_domain(
        _profile(tmp_path),
        _model(),
        _case(),
        runner=lambda _profile, _request, _output: None,
        output_dir=tmp_path / "failed-gate",
    )

    assert not result.ok
    assert "did not produce" in result.report["error"]
