"""Strict pure-K manifest, parser, comparison and source-patching tests."""

from pathlib import Path

import pytest

from suspension_multibody.adams import AdamsProfile
from suspension_multibody.adams.strict_k import (
    _parse_kinematic_result,
    _replace_exact,
    build_equivalence_manifest,
    compare_k_states,
)


def test_exact_adams_patch_rejects_missing_or_duplicate_fields() -> None:
    assert _replace_exact("a=0", "a=0", "a=1", "flag") == "a=1"
    with pytest.raises(ValueError, match="layout"):
        _replace_exact("a=0 a=0", "a=0", "a=1", "flag")


def test_strict_k_manifest_freezes_inputs_and_equal_snapshot_hashes(
    tmp_path: Path,
) -> None:
    database = tmp_path / "shared_car_database.cdb"
    subsystems = database / "subsystems.tbl"
    subsystems.mkdir(parents=True)
    (subsystems / "TR_Front_Suspension.sub").write_text(
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
$ end
""",
        encoding="utf-8",
    )
    (subsystems / "TR_Steering.sub").write_text(
        """[HARDPOINT]
 'tierod_inner' 'left/right' 467 -400 330
$ end
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

    manifest = build_equivalence_manifest(profile)

    assert manifest["contract"] == "strict-adams-k-v1"
    assert (
        manifest["adams_snapshot_sha256"]
        == manifest["suspension_multibody_snapshot_sha256"]
    )
    physical = manifest["physical_input"]
    assert physical["drive"]["wheel_travel_mm"] == [-10.0, 0.0, 10.0]
    assert physical["drive"]["rack_displacement_mm"] == [-5.0, 0.0, 5.0]
    assert physical["hardpoints_mm"]["rack_center"] == [-467.0, 0.0, 330.0]
    assert physical["boundaries"]["analysis_mode"] == "kinematic"


def test_parse_pure_k_result_maps_coordinates_and_vehicle_signs(tmp_path: Path) -> None:
    result = tmp_path / "k.res"
    result.write_text(
        """<?xml version="1.0"?>
<Results xmlns="urn:test"><Analysis><StepMap>
<Entity name="gel_spindle_XFORM"><Component name="X" id="1"/><Component name="Y" id="2"/><Component name="Z" id="3"/></Entity>
<Entity name="ger_spindle_XFORM"><Component name="X" id="4"/><Component name="Y" id="5"/><Component name="Z" id="6"/></Entity>
<Entity name="toe_angle"><Component name="left" id="7"/><Component name="right" id="8"/></Entity>
<Entity name="camber_angle"><Component name="left" id="9"/><Component name="right" id="10"/></Entity>
<Entity name="steering_rack_input"><Component name="rack_input" id="11"/></Entity>
<Entity name="wheel_travel"><Component name="vertical_left" id="12"/><Component name="vertical_right" id="13"/></Entity>
</StepMap><Data name="kinematic_001"><Step>267 -760 320 267 760 320 0.01 0.01 0.02 0.02 5 10 10</Step></Data></Analysis></Results>
""",
        encoding="utf-8",
    )

    parsed = _parse_kinematic_result(result)

    assert parsed["left_wheel_center_x_mm"] == -267
    assert parsed["right_wheel_center_y_mm"] == 760
    assert parsed["left_toe_deg"] == pytest.approx(-0.5729578)
    assert parsed["left_camber_deg"] == pytest.approx(1.1459156)
    assert parsed["adams_rack_input_mm"] == 5


def test_compare_strict_k_requires_nine_complete_absolute_states() -> None:
    states = []
    for wheel in (-10.0, 0.0, 10.0):
        for rack in (-5.0, 0.0, 5.0):
            states.append(
                {
                    "case_id": f"k-w{wheel:+g}-r{rack:+g}",
                    **{
                        f"{side}_wheel_center_{axis}_mm": 1.0
                        for side in ("left", "right")
                        for axis in ("x", "y", "z")
                    },
                    **{
                        f"{side}_{angle}_deg": 0.1
                        for side in ("left", "right")
                        for angle in ("toe", "camber")
                    },
                }
            )

    report = compare_k_states(states, [dict(state) for state in states])

    assert report["passed"]
    assert report["case_count"] == 9
    assert report["field_count"] == 90
    with pytest.raises(ValueError, match="nine case IDs"):
        compare_k_states(states, states[:-1])
