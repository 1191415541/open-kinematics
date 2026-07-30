"""Offline Adams adapter tests."""

import json
from pathlib import Path

from suspension_multibody.adams import AdamsBatchAdapter, AdamsProfile, discover_profile


def _reference() -> dict[str, dict[str, float]]:
    return {
        "K_geometry": {
            "wheel_center_z": 100.0,
            "camber_deg": -1.0,
            "steering_displacement": 5.0,
        },
        "C_compliance": {"wheel_rate": 0.01, "toe_rate": 0.02},
        "static_load": {"spring_force": 1000.0, "aligning_moment": 20.0},
    }


def _offline_profile(tmp_path: Path) -> AdamsProfile:
    database = tmp_path / "shared_car_database.cdb"
    (database / "templates.tbl").mkdir(parents=True)
    (database / "subsystems.tbl").mkdir()
    (database / "templates.tbl" / "_double_wishbone.tpl").write_text("fixture")
    (database / "subsystems.tbl" / "TR_Front_Suspension.sub").write_text("fixture")
    executable = tmp_path / "adams2024_1.bat"
    executable.write_text("fixture")
    dictionary = tmp_path / "acar_report_dictionary.csv"
    dictionary.write_text("channel,time\n")
    fields = (
        "time",
        "lcam",
        "ltoe",
        "rcam",
        "rtoe",
        "lf_wc_rise",
        "rf_wc_rise",
        "lspring_force",
        "rspring_force",
        "lfcam",
        "rfcam",
        "steering_displacement",
    )
    return AdamsProfile(
        name="fixture",
        home=str(tmp_path),
        executable=str(executable),
        version="2024.1",
        license_file=None,
        template_id="_double_wishbone.tpl",
        subsystem_id="TR_Front_Suspension.sub",
        database_path=str(database),
        report_dictionary=str(dictionary),
        export_fields=fields,
        available=True,
        license_probe="passed",
        message="fixture",
    )


def _runner(payload: dict[str, object]):
    def run(request_path: Path, output_dir: Path) -> None:
        del request_path
        (output_dir / "adams_results.json").write_text(
            json.dumps(payload), encoding="utf-8"
        )

    return run


def test_smoke_exports_only_mapping_metadata(tmp_path: Path) -> None:
    profile = discover_profile()
    result = AdamsBatchAdapter(profile).smoke(tmp_path)
    if not profile.available:
        assert not result.ok
        return
    assert result.ok
    payload = json.loads((tmp_path / "adams_smoke.json").read_text(encoding="utf-8"))
    assert payload["template_id"] == "_double_wishbone.tpl"
    assert payload["execution"] == "version_probe_and_report_dictionary"
    assert not list(tmp_path.glob("*.tpl"))
    assert not list(tmp_path.glob("*.cdb"))


def test_full_validation_requires_numeric_baseline(tmp_path: Path) -> None:
    profile = discover_profile()
    result = AdamsBatchAdapter(profile).full(tmp_path)
    if not profile.available:
        assert not result.ok
        return
    assert not result.ok
    payload = json.loads(
        (tmp_path / "adams_full_validation.json").read_text(encoding="utf-8")
    )
    assert payload["execution"] == "full_profile_and_equivalence_contract"
    assert payload["checks"]["version"]
    assert payload["checks"]["license_probe"]
    assert payload["checks"]["required_report_fields"]
    assert payload["checks"]["installed_assets"]
    assert not payload["checks"]["comparison"]
    assert (
        payload["comparison_error"] == "numeric reference is required for the full gate"
    )


def test_full_numeric_comparison_passes(tmp_path: Path) -> None:
    reference = _reference()
    result = AdamsBatchAdapter(
        _offline_profile(tmp_path), runner=_runner({"groups": reference})
    ).full(tmp_path / "pass", reference=reference)
    assert result.ok
    assert result.report is not None
    assert all(result.report["checks"].values())  # type: ignore[union-attr]
    assert result.report["comparisons"]["K_geometry"]["passed"]  # type: ignore[index]


def test_full_numeric_comparison_reads_csv_results(tmp_path: Path) -> None:
    reference = _reference()

    def csv_runner(request_path: Path, output_dir: Path) -> None:
        del request_path
        rows = ["group,field,value"]
        rows.extend(
            f"{group},{field},{value}"
            for group, values in reference.items()
            for field, value in values.items()
        )
        (output_dir / "adams_results.csv").write_text("\n".join(rows), encoding="utf-8")

    result = AdamsBatchAdapter(_offline_profile(tmp_path), runner=csv_runner).full(
        tmp_path / "csv", reference=reference
    )
    assert result.ok


def test_full_numeric_comparison_accepts_flat_adams_fields(tmp_path: Path) -> None:
    reference = _reference()
    flat = {
        "camber_deg": reference["K_geometry"]["camber_deg"],
        "wheel_center_z": reference["K_geometry"]["wheel_center_z"],
        "steering_displacement": reference["K_geometry"]["steering_displacement"],
        "wheel_rate": reference["C_compliance"]["wheel_rate"],
        "toe_rate": reference["C_compliance"]["toe_rate"],
        "spring_force": reference["static_load"]["spring_force"],
        "aligning_moment": reference["static_load"]["aligning_moment"],
    }
    result = AdamsBatchAdapter(_offline_profile(tmp_path), runner=_runner(flat)).full(
        tmp_path / "flat", reference=reference
    )
    assert result.ok


def test_full_numeric_comparison_converts_explicit_units(tmp_path: Path) -> None:
    reference = _reference()
    actual = {
        "groups": {
            "K_geometry": {
                "wheel_center_z": {"value": 0.1, "unit": "m"},
                "camber_deg": {"value": -0.0174532925199433, "unit": "rad"},
                "steering_displacement": {"value": 0.005, "unit": "m"},
            },
            "C_compliance": {
                "wheel_rate": {"value": 0.01, "unit": "mm/N"},
                "toe_rate": {"value": 0.02, "unit": "mm/N"},
            },
            "static_load": {
                "spring_force": {"value": 1.0, "unit": "kN"},
                "aligning_moment": {"value": 0.02, "unit": "N*m"},
            },
        }
    }
    result = AdamsBatchAdapter(_offline_profile(tmp_path), runner=_runner(actual)).full(
        tmp_path / "units", reference=reference
    )
    assert result.ok


def test_full_numeric_comparison_rejects_unknown_units(tmp_path: Path) -> None:
    reference = _reference()
    actual = {"groups": {group: dict(values) for group, values in reference.items()}}
    actual["groups"]["K_geometry"]["wheel_center_z"] = {"value": 100.0, "unit": "inch"}  # type: ignore[index]
    result = AdamsBatchAdapter(_offline_profile(tmp_path), runner=_runner(actual)).full(
        tmp_path / "bad-unit", reference=reference
    )
    assert not result.ok
    assert "unsupported unit" in str(result.report["comparison_error"])  # type: ignore[index]


def test_full_numeric_comparison_rejects_out_of_tolerance(tmp_path: Path) -> None:
    reference = _reference()
    actual = {"groups": {group: dict(values) for group, values in reference.items()}}
    actual["groups"]["K_geometry"]["camber_deg"] = -0.8  # type: ignore[index]
    result = AdamsBatchAdapter(_offline_profile(tmp_path), runner=_runner(actual)).full(
        tmp_path / "fail", reference=reference
    )
    assert not result.ok
    assert not result.report["checks"]["K_geometry"]  # type: ignore[index]


def test_full_numeric_comparison_rejects_missing_group(tmp_path: Path) -> None:
    reference = _reference()
    actual = {
        "groups": {
            "K_geometry": reference["K_geometry"],
            "static_load": reference["static_load"],
        }
    }
    result = AdamsBatchAdapter(_offline_profile(tmp_path), runner=_runner(actual)).full(
        tmp_path / "missing", reference=reference
    )
    assert not result.ok
    assert not result.report["checks"]["C_compliance"]  # type: ignore[index]


def test_full_numeric_comparison_rejects_reduced_baseline(tmp_path: Path) -> None:
    reference = {
        "K_geometry": {"camber_deg": 0.0},
        "C_compliance": _reference()["C_compliance"],
        "static_load": _reference()["static_load"],
    }
    result = AdamsBatchAdapter(
        _offline_profile(tmp_path), runner=_runner({"groups": reference})
    ).full(tmp_path / "reduced", reference=reference)
    assert not result.ok
    assert "fewer than" in str(result.report["comparison_error"])  # type: ignore[index]


def test_full_numeric_comparison_rejects_unknown_actual_field(tmp_path: Path) -> None:
    reference = _reference()
    actual = {"groups": {group: dict(values) for group, values in reference.items()}}
    actual["groups"]["static_load"]["unexpected"] = 1.0  # type: ignore[index]
    result = AdamsBatchAdapter(_offline_profile(tmp_path), runner=_runner(actual)).full(
        tmp_path / "unknown", reference=reference
    )
    assert not result.ok
    assert "unknown fields" in str(result.report["comparison_error"])  # type: ignore[index]


def test_full_validation_rejects_unavailable_profile(tmp_path: Path) -> None:
    profile = _offline_profile(tmp_path)
    unavailable = AdamsProfile(
        **{**profile.__dict__, "available": False, "license_probe": "failed"}
    )
    called = False

    def runner(request_path: Path, output_dir: Path) -> None:
        nonlocal called
        called = True
        del request_path, output_dir

    result = AdamsBatchAdapter(unavailable, runner=runner).full(
        tmp_path / "unavailable", reference=_reference()
    )
    assert not result.ok
    assert not called
    assert not result.report["checks"]["profile_available"]  # type: ignore[index]


def test_full_numeric_comparison_reports_runner_failure(tmp_path: Path) -> None:
    def failing_runner(request_path: Path, output_dir: Path) -> None:
        del request_path, output_dir
        raise RuntimeError("runner fixture failed")

    result = AdamsBatchAdapter(_offline_profile(tmp_path), runner=failing_runner).full(
        tmp_path / "runner-fail", reference=_reference()
    )
    assert not result.ok
    assert "runner failed" in str(result.report["comparison_error"])  # type: ignore[index]
