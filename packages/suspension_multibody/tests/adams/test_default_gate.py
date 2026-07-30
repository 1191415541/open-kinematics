"""Offline tests for the built-in Adams runner/reference gate."""

from pathlib import Path

import pytest

from suspension_multibody.adams import AdamsProfile
from suspension_multibody.adams.reference import build_default_reference
from suspension_multibody.adams.runner import _maximum_minimum, _pair, _result_component


def test_default_reference_solves_adams_hardpoints_independently(
    tmp_path: Path,
) -> None:
    database = tmp_path / "shared_car_database.cdb"
    subsystem_dir = database / "subsystems.tbl"
    subsystem_dir.mkdir(parents=True)
    (subsystem_dir / "TR_Front_Suspension.sub").write_text(
        """[HARDPOINT]
 'lca_front' 'left/right' 67 -400 180
 'lca_outer' 'left/right' 267 -750 130
 'lca_rear' 'left/right' 467 -450 185
 'tierod_inner' 'left/right' 467 -400 330
 'tierod_outer' 'left/right' 417 -750 330
 'uca_front' 'left/right' 367 -450 555
 'uca_outer' 'left/right' 307 -675 555
 'uca_rear' 'left/right' 517 -490 560
 'wheel_center' 'left/right' 267 -760 330
$ next section
""",
        encoding="utf-8",
    )
    profile = AdamsProfile(
        name="fixture",
        home=str(tmp_path),
        executable=None,
        version="2024.1",
        license_file=None,
        template_id="_double_wishbone.tpl",
        subsystem_id="TR_Front_Suspension.sub",
        database_path=str(database),
        report_dictionary=None,
        export_fields=(),
        available=True,
        license_probe="passed",
        message="fixture",
    )

    reference = build_default_reference(profile)

    assert reference["K_geometry"]["left_toe_change_deg"] == pytest.approx(
        0.723426, abs=1e-5
    )
    assert reference["K_geometry"]["left_camber_change_deg"] == pytest.approx(
        0.387693, abs=1e-5
    )
    assert reference["static_load"]["left_wheel_force_n"] == pytest.approx(2943.0)


def test_default_runner_parses_reports_and_static_result(tmp_path: Path) -> None:
    report = """
Maximum Left Toe Angle = 0.3499 (deg)
Minimum Left Toe Angle = -0.3766 (deg)
Lateral compliance steer = -0.0030 -0.0030 deg/kN
"""
    result = tmp_path / "result.res"
    result.write_text(
        """<?xml version="1.0"?>
<Results xmlns="http://www.mscsoftware.com/:xrf10"><Analysis><StepMap>
<Entity name="left_tire_forces"><Component name="normal" id="2" /></Entity>
</StepMap><Data name="quasiStatic_001"><Step>1 2785.636</Step></Data>
</Analysis></Results>""",
        encoding="utf-8",
    )

    assert _maximum_minimum(report, "Left Toe Angle") == (0.3499, -0.3766)
    assert _pair(report, "Lateral compliance steer") == (-0.003, -0.003)
    assert _result_component(result, "left_tire_forces", "normal") == pytest.approx(
        2785.636
    )
