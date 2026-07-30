"""Generated isolated Adams/Car source tests."""

from pathlib import Path

from suspension_mbd.adams import AdamsProfile
from suspension_mbd.adams.equivalent_model import write_equivalent_sources
from suspension_mbd.adams.reference import _read_hardpoints

_POINTS = {
    "lca_front": (1.0, -2.0, 3.0),
    "lca_outer": (4.0, -5.0, 6.0),
    "lca_rear": (7.0, -8.0, 9.0),
    "tierod_inner": (10.0, -11.0, 12.0),
    "tierod_outer": (13.0, -14.0, 15.0),
    "uca_front": (16.0, -17.0, 18.0),
    "uca_outer": (19.0, -20.0, 21.0),
    "uca_rear": (22.0, -23.0, 24.0),
    "wheel_center": (25.0, -26.0, 27.0),
}


def _profile(tmp_path: Path) -> AdamsProfile:
    database = tmp_path / "shared_car_database.cdb"
    subsystems = database / "subsystems.tbl"
    assemblies = database / "assemblies.tbl"
    subsystems.mkdir(parents=True)
    assemblies.mkdir()
    hardpoints = "\n".join(
        f" '{name}' 'left/right' {x + 100:g} {y:g} {z:g}"
        for name, (x, y, z) in _POINTS.items()
    )
    (subsystems / "TR_Front_Suspension.sub").write_text(
        """[HARDPOINT]
{hardpoints}
[PARAMETER]
 'kinematic_flag' 'single' 'integer' 0
 'camber_angle' 'left/right' 'real' -0.5
(VARIANTS)
 'kinematic_flag' 'single' 'integer' 1
""".format(hardpoints=hardpoints),
        encoding="utf-8",
    )
    (subsystems / "TR_Steering.sub").write_text(
        """[PARAMETER]
 'kinematic_flag' 'single' 'integer' 0
""",
        encoding="utf-8",
    )
    (assemblies / "mdi_front_vehicle.asy").write_text(
        """[ASSEMBLY_HEADER]
 ASSEMBLY_CLASS = 'suspension'
[SUBSYSTEM]
 USAGE = 'mdids://acar_shared/subsystems.tbl/MDI_FRONT_SUSPENSION.sub'
[SUBSYSTEM]
 USAGE = 'mdids://acar_shared/subsystems.tbl/MDI_FRONT_STEERING.sub'
[PARAMETER]
 'compliance_matrix_flag' 'single' 'integer' 1
 'compliance_objects_flag' 'single' 'integer' 1
""",
        encoding="utf-8",
    )
    return AdamsProfile(
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


def _manifest() -> dict[str, object]:
    return {
        "physical_input": {
            "adams_template_hardpoints_mm": {
                name: list(values) for name, values in _POINTS.items()
            }
        }
    }


def test_generator_rewrites_manifest_hardpoints_in_an_isolated_suspension_assembly(
    tmp_path: Path,
) -> None:
    profile = _profile(tmp_path)
    generated = write_equivalent_sources(profile, _manifest(), tmp_path / "runtime", mode="K")

    assert _read_hardpoints(generated.suspension) == _POINTS
    assert "101" in (
        Path(profile.database_path or "")
        / "subsystems.tbl"
        / "TR_Front_Suspension.sub"
    ).read_text(encoding="utf-8")
    assembly = generated.assembly.read_text(encoding="utf-8")
    assert "ASSEMBLY_CLASS = 'suspension'" in assembly
    assert generated.suspension.as_posix() in assembly
    assert generated.steering.as_posix() in assembly
    assert "Demo_Vehicle" not in assembly
    assert "'kinematic_flag' 'single' 'integer' 1" in generated.suspension.read_text(
        encoding="utf-8"
    )
    assert generated.hashes["assembly_sha256"]


def test_generator_keeps_compliance_mode_in_the_generated_subsystems(tmp_path: Path) -> None:
    profile = _profile(tmp_path)
    generated = write_equivalent_sources(profile, _manifest(), tmp_path / "runtime", mode="C")

    assert "'kinematic_flag' 'single' 'integer' 0" in generated.suspension.read_text(
        encoding="utf-8"
    )
    assert "'kinematic_flag' 'single' 'integer' 0" in generated.steering.read_text(
        encoding="utf-8"
    )
