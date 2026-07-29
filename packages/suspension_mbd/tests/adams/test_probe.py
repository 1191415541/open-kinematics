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
        lambda path: ("2024.1", "version fixture"),
    )
    monkeypatch.setattr(
        probe_module,
        "_run_license_probe",
        lambda path: ("failed", "license unavailable"),
    )
    profile = discover_profile(home=tmp_path)
    assert not profile.available
    assert profile.license_probe == "failed"


def test_version_probe_does_not_claim_license_success(monkeypatch, tmp_path: Path) -> None:
    executable = tmp_path / "adams.bat"
    executable.write_text("fixture")
    monkeypatch.setattr(
        probe_module.subprocess,
        "run",
        lambda *args, **kwargs: subprocess.CompletedProcess(
            args=args[0], returncode=0, stdout="Version = 2024.1_x64", stderr=""
        ),
    )
    version, _detail = probe_module._run_version_probe(executable)
    assert version == "2024.1"


def test_discovers_installation_from_path(monkeypatch, tmp_path: Path) -> None:
    home = _installation_fixture(tmp_path)
    executable = home / "bin" / "adams2024_1.bat"
    monkeypatch.setattr(probe_module.shutil, "which", lambda name: str(executable))
    monkeypatch.setattr(probe_module, "_registry_homes", lambda: ())
    monkeypatch.setattr(probe_module, "_run_version_probe", lambda path: ("2024.1", "ok"))
    monkeypatch.setattr(probe_module, "_run_license_probe", lambda path: ("passed", "ok"))

    profile = discover_profile()

    assert profile.available
    assert profile.home == str(home)


def test_discovers_installation_from_registry(monkeypatch, tmp_path: Path) -> None:
    home = _installation_fixture(tmp_path)
    monkeypatch.setattr(probe_module.shutil, "which", lambda name: None)
    monkeypatch.setattr(probe_module, "_registry_homes", lambda: (home,))
    monkeypatch.setattr(probe_module, "_run_version_probe", lambda path: ("2024.1", "ok"))
    monkeypatch.setattr(probe_module, "_run_license_probe", lambda path: ("passed", "ok"))

    profile = discover_profile()

    assert profile.available
    assert profile.home == str(home)


def test_license_probe_requires_product_start_marker(monkeypatch, tmp_path: Path) -> None:
    executable = tmp_path / "adams2024_1.bat"
    executable.write_text("fixture")
    seen: list[str] = []

    def run(args, **kwargs):
        seen.extend(str(value) for value in args)
        return subprocess.CompletedProcess(args=args, returncode=0, stdout="Version = 2024.1_x64", stderr="")

    monkeypatch.setattr(probe_module.subprocess, "run", run)

    status, _detail = probe_module._run_license_probe(executable)

    assert status != "passed"
    command = " ".join(seen)
    assert "acar" in command
    assert "ru-acar" in command
    assert " b " in command


def _installation_fixture(root: Path) -> Path:
    database = root / "acar" / "shared_car_database.cdb"
    (database / "templates.tbl").mkdir(parents=True)
    (database / "subsystems.tbl").mkdir()
    (database / "templates.tbl" / "_double_wishbone.tpl").write_text("fixture")
    (database / "subsystems.tbl" / "TR_Front_Suspension.sub").write_text("fixture")
    (root / "acar" / "acar_report_dictionary.csv").write_text("channel,lcam\nchannel,ltoe\n")
    (root / "bin").mkdir()
    (root / "bin" / "adams2024_1.bat").write_text("fixture")
    return root
