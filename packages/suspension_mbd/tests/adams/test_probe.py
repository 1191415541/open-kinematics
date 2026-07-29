"""Adams installation probe tests."""

import subprocess
from pathlib import Path

import suspension_mbd.adams.probe as probe_module
from suspension_mbd.adams import discover_profile


def test_local_adams_profile_discovers_expected_template() -> None:
    profile = discover_profile()
    if not profile.available:
        assert "not found" in profile.message or "failed" in profile.message
        return
    assert profile.version == "2024.1"
    assert profile.template_id == "_double_wishbone.tpl"
    assert profile.subsystem_id == "TR_Front_Suspension.sub"
    assert "lcam" in profile.export_fields
    assert "ltoe" in profile.export_fields


def test_unknown_profile_is_explicitly_unavailable() -> None:
    profile = discover_profile("missing-profile")
    assert not profile.available
    assert profile.license_probe == "unknown-profile"


def test_license_probe_is_part_of_availability(monkeypatch, tmp_path: Path) -> None:
    database = tmp_path / "shared_car_database.cdb"
    (database / "templates.tbl").mkdir(parents=True)
    (database / "subsystems.tbl").mkdir()
    (database / "templates.tbl" / "_double_wishbone.tpl").write_text("fixture")
    (database / "subsystems.tbl" / "TR_Front_Suspension.sub").write_text("fixture")
    (tmp_path / "bin").mkdir()
    executable = tmp_path / "bin" / "adams2024_1.bat"
    executable.write_text("fixture")
    monkeypatch.setattr(
        probe_module,
        "_run_version_probe",
        lambda path: ("2024.1", "failed", "license unavailable"),
    )
    profile = discover_profile(home=tmp_path)
    assert not profile.available
    assert profile.license_probe == "failed"


def test_version_probe_accepts_dotted_version(monkeypatch, tmp_path: Path) -> None:
    executable = tmp_path / "adams.bat"
    executable.write_text("fixture")
    monkeypatch.setattr(
        probe_module.subprocess,
        "run",
        lambda *args, **kwargs: subprocess.CompletedProcess(
            args=args[0], returncode=0, stdout="Version = 2024.1_x64", stderr=""
        ),
    )
    version, license_probe, _detail = probe_module._run_version_probe(executable)
    assert version == "2024.1"
    assert license_probe == "passed"
