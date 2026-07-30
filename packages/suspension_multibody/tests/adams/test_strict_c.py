"""Strict native-Adams C-model generation, parsing and comparison tests."""

from pathlib import Path

import pytest

from suspension_multibody.adams.strict_c import (
    C_FIELD_KINDS,
    LOAD_PATHS,
    _parse_raw_adams_result,
    _raw_command_text,
    compare_c_states,
    write_raw_adams_models,
)
from suspension_multibody.analysis import LoadPath
from suspension_multibody.schema import FrontAxleModel, MassSpec, Vec3


def _model() -> FrontAxleModel:
    points = {
        "uca_front": Vec3(x=0, y=-400, z=500),
        "uca_rear": Vec3(x=100, y=-400, z=500),
        "uca_outer": Vec3(x=50, y=-700, z=500),
        "lca_front": Vec3(x=0, y=-400, z=100),
        "lca_rear": Vec3(x=100, y=-400, z=100),
        "lca_outer": Vec3(x=50, y=-700, z=100),
        "tierod_inner": Vec3(x=50, y=-350, z=300),
        "tierod_outer": Vec3(x=50, y=-700, z=300),
        "wheel_center": Vec3(x=50, y=-750, z=300),
        "rack_center": Vec3(x=50, y=0, z=300),
    }
    return FrontAxleModel(
        name="strict_c_fixture", hardpoints=points, mass=MassSpec(sprung_mass=1)
    )


def test_raw_c_writer_has_only_common_elements_and_neutral_rack_drive(
    tmp_path: Path,
) -> None:
    generated = write_raw_adams_models(_model(), tmp_path)

    assert len(generated) == 6
    text = generated[0].path.read_text(encoding="ascii")
    assert text.count("BUSHING/") == 16
    assert text.count("K = 5000, 5000, 5000") == 16
    assert text.count("KT = 5000000, 5000000, 5000000") == 16
    assert "BUSHING/1, I = 100, J = 101" in text
    assert "BUSHING/2, I = 101, J = 100" in text
    assert "MOTION/1" in text
    assert "FUNCTION = 0" in text
    assert "ACCGRAV/KGRAV = 0" in text
    assert "IF(TIME-1:-100*TIME,-100,200*TIME-300)" in text
    assert "strict_c_l_wheel_longitudinal" in text
    assert "strict_c_r_wheel_longitudinal" in text
    assert _raw_command_text("strict_c_fx", LOAD_PATHS[0]).startswith(
        "\nfile/model=strict_c_fx\nsimulate/static\n"
    )
    assert "simulate/statics, end=1, steps=10" in _raw_command_text(
        "strict_c_fx", LOAD_PATHS[0]
    )
    assert "simulate/statics, end=2, steps=10" in _raw_command_text(
        "strict_c_fx", LOAD_PATHS[0]
    )


def test_raw_c_parser_uses_zero_load_step_for_response(tmp_path: Path) -> None:
    result = tmp_path / "fy.res"
    result.write_text(
        """<?xml version="1.0"?>
<Results xmlns="urn:test"><Analysis><StepMap>
<Entity name="time"><Component name="TIME" id="1"/></Entity>
<Entity name="strict_c_l_wheel_response"><Component name="x" id="2"/><Component name="y" id="3"/><Component name="z" id="4"/><Component name="lateral_x" id="5"/><Component name="lateral_y" id="6"/><Component name="lateral_z" id="7"/></Entity>
<Entity name="strict_c_r_wheel_response"><Component name="x" id="8"/><Component name="y" id="9"/><Component name="z" id="10"/><Component name="lateral_x" id="11"/><Component name="lateral_y" id="12"/><Component name="lateral_z" id="13"/></Entity>
<Entity name="strict_c_l_wheel_longitudinal"><Component name="x" id="14"/><Component name="y" id="15"/><Component name="z" id="16"/></Entity>
<Entity name="strict_c_r_wheel_longitudinal"><Component name="x" id="17"/><Component name="y" id="18"/><Component name="z" id="19"/></Entity>
</StepMap><Data name="quasiStatic_001">
<Step>0 0 0 0 0 1 0 0 0 0 0 1 0 1 0 0 1 0 0</Step>
<Step>.5 0 0 0 0 1 0 0 0 0 0 1 0 1 0 0 1 0 0</Step>
<Step>1 -1 -2 -3 .1 1 0 0 0 0 0 1 0 1 0 0 1 0 0</Step>
<Step>1.5 0 0 0 0 1 0 0 0 0 0 1 0 1 0 0 1 0 0</Step>
<Step>2 1 2 3 -.0998334166 .9950041653 0 0 0 0 0 1 0 .9950041653 .0998334166 0 1 0 0</Step>
</Data></Analysis></Results>
""",
        encoding="utf-8",
    )

    states = _parse_raw_adams_result(result, LoadPath("fy", "fy", 10, levels=3))

    assert [state["case_id"] for state in states] == ["c-fy-00", "c-fy-01", "c-fy-02"]
    assert states[1]["left_wheel_center_dx_mm"] == 0
    assert states[2]["left_wheel_center_dx_mm"] == 1
    assert states[2]["left_load_fy_n"] == 10
    assert states[2]["left_toe_delta_deg"] == pytest.approx(-5.729577951)
    assert states[2]["left_camber_delta_deg"] == 0


def test_strict_c_comparison_requires_complete_common_grid() -> None:
    states = [
        {
            "case_id": f"c-{path.name}-{index:02d}",
            **{field: 0.0 for field in C_FIELD_KINDS},
        }
        for path in LOAD_PATHS
        for index in range(path.levels)
    ]

    report = compare_c_states(states, [dict(state) for state in states])

    assert report["passed"]
    assert report["case_count"] == 66
    assert report["field_count"] == 66 * len(C_FIELD_KINDS)
    with pytest.raises(ValueError, match="complete common state grid"):
        compare_c_states(states, states[:-1])
