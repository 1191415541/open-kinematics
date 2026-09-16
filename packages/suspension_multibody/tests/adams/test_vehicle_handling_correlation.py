"""Handling numerical-correlation gate tests."""

from __future__ import annotations

from pathlib import Path

import pytest

from suspension_multibody.adams import AdamsProfile
from suspension_multibody.adams.time_domain import TimeHistory
from suspension_multibody.adams.vehicle_correlation import (
    validate_handling_correlation,
    validate_handling_correlation_matrix,
)
from suspension_multibody.adams.vehicle_handling import (
    _replace_rt_tire_property,
)
from suspension_multibody.adams.vehicle_reference import write_vehicle_reference_bundle
from suspension_multibody.analysis.vehicle_correlation_model import (
    VehicleCorrelationRun,
)


def test_tire_override_rewrites_only_the_rt_variant() -> None:
    """
    The tire override must not touch the subsystem's default tire.

    A tire subsystem declares its default tire in ``[WHEEL_ASSEMBLY]`` and the
    high-performance tire in the ``{rt}`` variant.  The handling assembly selects
    the variant, so rewriting the default entry as well would silently change an
    unrelated configuration.
    """
    payload = (
        "[WHEEL_ASSEMBLY]\n"
        " PROPERTY_FILE        = 'mdids://acar_shared/tires.tbl/TR_front_pac89.tir'\n"
        " CONTACT_TYPE         = 'handling'\n"
        "(VARIANTS)\n"
        "{rt}\n"
        " PROPERTY_FILE  = 'mdids://acar_shared/tires.tbl/pac2002_235_60R16.tir'\n"
        " HIGH_PERFORMANCE  =  'yes'\n"
    )
    replacement = Path("/tmp/variant/pac2002_um21.tir")

    patched = _replace_rt_tire_property(payload, replacement, "TR_Front_Tires.sub")

    assert "TR_front_pac89.tir" in patched, "default tire entry was modified"
    assert patched.count("TR_front_pac89.tir") == 1
    assert replacement.as_posix() in patched
    assert "pac2002_235_60R16.tir" not in patched


def test_tire_override_requires_a_variant_block() -> None:
    """A subsystem without the ``{rt}`` block must fail loudly."""
    with pytest.raises(ValueError, match="no .rt. variant block"):
        _replace_rt_tire_property(
            "[WHEEL_ASSEMBLY]\n PROPERTY_FILE = 'a.tir'\n",
            Path("/tmp/b.tir"),
            "TR_Front_Tires.sub",
        )


def _profile(tmp_path: Path) -> AdamsProfile:
    return AdamsProfile("fixture", str(tmp_path), "adams.bat", "2024.1", None, "t", "s", str(tmp_path), None, (), True, "passed", "fixture")


def _bundle(root: Path, case: str) -> None:
    path = root / case
    raw = path / "adams_raw"
    raw.mkdir(parents=True)
    for suffix in (".adm", ".cmd", ".msg", ".res"):
        (raw / f"{case}{suffix}").write_text("fixture", encoding="ascii")
    duration = {"steady_state_circle": 17.0, "step_steer": 5.0, "sine_steer": 6.0, "double_lane_change": 12.0}[case]
    time = tuple(index * 0.01 for index in range(int(duration * 100) + 1))
    history = TimeHistory(
        time=time,
        channels={"steering": tuple(0.0 for _ in time), "lateral_acceleration": tuple(1.0 for _ in time), "yaw_rate": tuple(0.1 for _ in time), "roll_angle": tuple(0.01 for _ in time)},
        units={"steering": "rad", "lateral_acceleration": "m/s^2", "yaw_rate": "rad/s", "roll_angle": "rad"},
    )
    write_vehicle_reference_bundle(case=case, category="handling_stability", history=history, output_dir=path, profile=_profile(root), input_manifest={})


def test_handling_gate_compares_all_cases(tmp_path: Path) -> None:
    reference = tmp_path / "reference"
    for case in ("steady_state_circle", "step_steer", "sine_steer", "double_lane_change"):
        _bundle(reference, case)

    def simulator(case: str) -> VehicleCorrelationRun:
        payload = __import__("json").loads(
            (reference / case / "adams_reference_bundle.json").read_text()
        )
        bundle = TimeHistory.from_mapping(payload["history"])
        return VehicleCorrelationRun(
            case,
            15,
            bundle,
            payload["input_manifest_hash"],
            ("test",),
        )

    result = validate_handling_correlation(reference, output_dir=tmp_path / "out", simulator=simulator)

    assert result.ok
    assert all(item["status"] == "PASS" for item in result.report["cases"].values())


def test_handling_matrix_keeps_native_brush_blocked_without_reference(
    tmp_path: Path,
) -> None:
    reference = tmp_path / "reference"
    for case in ("steady_state_circle", "step_steer", "sine_steer", "double_lane_change"):
        _bundle(reference, case)

    result = validate_handling_correlation_matrix(
        {"pac2002": reference},
        output_dir=tmp_path / "matrix",
    )

    assert not result.ok
    assert result.report["variants"]["pac2002"]["tire_model"] == "pac2002"
    brush = result.report["variants"]["native_brush"]
    assert brush["status"] == "BLOCKED"
    assert brush["passed"] is False
