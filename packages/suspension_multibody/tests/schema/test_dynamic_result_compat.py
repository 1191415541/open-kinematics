"""
Historical ``DynamicResultBundle`` loading and time-domain adapter compatibility.

The bundle keeps its version inside ``manifest`` while the other v1 input
documents carry ``schema_version`` at the document root.  These tests pin the
public loader for schema-legal samples, the strict rejection paths that must not
weaken, and the Adams time-domain adapter that still consumes the legacy bundle.
No real historical artifact ships with the package, so only schema-legal sample
round trips are claimed here.
"""

from __future__ import annotations

import json
from collections.abc import Callable
from pathlib import Path
from typing import Any

import pytest
import yaml

from suspension_multibody.adams.time_domain import history_from_dynamic_bundle
from suspension_multibody.results import TimeSeriesResult, TimeSeriesSample
from suspension_multibody.schema import (
    CaseSpec,
    DynamicCaseSpec,
    DynamicManifest,
    DynamicResultBundle,
    DynamicSolverSettings,
    DynamicTimeSample,
    FrontAxleModel,
    MassSpec,
    Provenance,
    Vec3,
    VehicleDynamicCase,
    VehicleModel,
    load_case,
    load_dynamic_case,
    load_dynamic_result,
    load_model,
    load_vehicle_dynamic_case,
    load_vehicle_model,
)

_INPUT_LOADERS: dict[str, Callable[[Path], object]] = {
    "model": load_model,
    "case": load_case,
    "dynamic_case": load_dynamic_case,
    "vehicle_model": load_vehicle_model,
    "vehicle_dynamic_case": load_vehicle_dynamic_case,
}


def _bundle() -> DynamicResultBundle:
    return DynamicResultBundle(
        manifest=DynamicManifest(
            run_id="history-run",
            mode="axle_dynamic",
            sample_count=2,
            provenance=Provenance(package_version="0.1.0"),
        ),
        samples=(
            DynamicTimeSample(time=0.0, body="axle", metrics={"heave_mm": 0.0}),
            DynamicTimeSample(time=0.1, body="axle", metrics={"heave_mm": 1.0}),
        ),
    )


def _bundle_payload() -> dict[str, Any]:
    """Return a schema-legal historical payload with out-of-order samples."""
    return {
        "manifest": {
            "schema_version": 1,
            "format_version": "1.0",
            "run_id": "history-run",
            "mode": "axle_dynamic",
            "sample_count": 3,
            "provenance": {"package_version": "0.1.0"},
        },
        "samples": [
            {"time": 0.1, "body": "axle", "metrics": {"heave_mm": 1.0}},
            {"time": 0.0, "body": "axle", "metrics": {"heave_mm": 0.0}},
            {"time": 0.1, "body": "wheel", "metrics": {"heave_mm": 5.0}},
        ],
        "diagnostics": [],
    }


def _write(path: Path, payload: dict[str, Any], *, as_yaml: bool = False) -> Path:
    text = (
        yaml.safe_dump(payload, sort_keys=False)
        if as_yaml
        else json.dumps(payload, indent=2)
    )
    path.write_text(text, encoding="utf-8")
    return path


def _input_documents(vehicle_model: VehicleModel) -> dict[str, dict[str, Any]]:
    return {
        "model": FrontAxleModel(
            hardpoints={"LOWER_FRONT_LEFT": Vec3(x=100.0, y=-700.0, z=200.0)},
            mass=MassSpec(sprung_mass=1200.0),
        ).model_dump(mode="json"),
        "case": CaseSpec(mode="K").model_dump(mode="json"),
        "dynamic_case": DynamicCaseSpec(
            mode="axle_dynamic",
            solver=DynamicSolverSettings(end_time=0.02, step_size=0.01),
        ).model_dump(mode="json"),
        "vehicle_model": vehicle_model.model_dump(mode="json"),
        "vehicle_dynamic_case": VehicleDynamicCase(
            solver=DynamicSolverSettings(end_time=0.02, step_size=0.01),
            vehicle=vehicle_model,
        ).model_dump(mode="json"),
    }


def _payload_with_unknown(location: str) -> dict[str, Any]:
    payload = _bundle_payload()
    if location == "bundle":
        payload["unexpected"] = 1
    elif location == "manifest":
        payload["manifest"]["unexpected"] = 1
    else:
        payload["samples"][0]["unexpected"] = 1
    return payload


def test_load_dynamic_result_reads_production_written_bundle(tmp_path: Path) -> None:
    path = tmp_path / "bundle.json"
    bundle = _bundle()
    path.write_text(bundle.model_dump_json(indent=2), encoding="utf-8")

    assert load_dynamic_result(path) == bundle


@pytest.mark.parametrize("as_yaml", [False, True])
def test_load_dynamic_result_reads_schema_legal_history_samples(
    tmp_path: Path, as_yaml: bool
) -> None:
    suffix = ".yaml" if as_yaml else ".json"
    path = _write(tmp_path / f"bundle{suffix}", _bundle_payload(), as_yaml=as_yaml)

    bundle = load_dynamic_result(path)

    assert bundle.manifest.schema_version == 1
    assert bundle.manifest.run_id == "history-run"
    assert [sample.body for sample in bundle.samples] == ["axle", "axle", "wheel"]


@pytest.mark.parametrize("version", [None, 2])
def test_load_dynamic_result_rejects_missing_or_non_one_manifest_version(
    tmp_path: Path, version: int | None
) -> None:
    payload = _bundle_payload()
    manifest = dict(payload["manifest"])
    if version is None:
        manifest.pop("schema_version")
    else:
        manifest["schema_version"] = version
    payload["manifest"] = manifest

    with pytest.raises(ValueError, match="unsupported schema_version"):
        load_dynamic_result(_write(tmp_path / "bundle.json", payload))


@pytest.mark.parametrize("location", ["bundle", "manifest", "sample"])
def test_load_dynamic_result_rejects_unknown_fields(
    tmp_path: Path, location: str
) -> None:
    path = _write(tmp_path / "bundle.json", _payload_with_unknown(location))

    with pytest.raises(ValueError, match="extra_forbidden"):
        load_dynamic_result(path)


def test_load_dynamic_result_still_forbids_root_schema_version(tmp_path: Path) -> None:
    payload = _bundle_payload()
    payload["schema_version"] = 1

    with pytest.raises(ValueError, match="extra_forbidden"):
        load_dynamic_result(_write(tmp_path / "bundle.json", payload))


@pytest.mark.parametrize("kind", sorted(_INPUT_LOADERS))
def test_input_loaders_keep_requiring_top_level_schema_version(
    tmp_path: Path, full_vehicle_model: VehicleModel, kind: str
) -> None:
    document = _input_documents(full_vehicle_model)[kind]
    loader = _INPUT_LOADERS[kind]
    without_version = {
        key: value for key, value in document.items() if key != "schema_version"
    }

    with pytest.raises(ValueError, match="unsupported schema_version"):
        loader(_write(tmp_path / f"{kind}-missing.json", without_version))
    with pytest.raises(ValueError, match="unsupported schema_version 2"):
        loader(_write(tmp_path / f"{kind}-v2.json", {**document, "schema_version": 2}))


def test_history_adapter_filters_body_and_sorts_samples(tmp_path: Path) -> None:
    bundle = load_dynamic_result(_write(tmp_path / "bundle.json", _bundle_payload()))

    history = history_from_dynamic_bundle(bundle, body="axle", channels=("heave_mm",))

    assert history.time == (0.0, 0.1)
    assert history.channels["heave_mm"] == (0.0, 1.0)
    assert history.as_dict() == {
        "time": [0.0, 0.1],
        "channels": {"heave_mm": [0.0, 1.0]},
    }


def test_history_adapter_rejects_unknown_body(tmp_path: Path) -> None:
    bundle = load_dynamic_result(_write(tmp_path / "bundle.json", _bundle_payload()))

    with pytest.raises(ValueError, match="no samples for body"):
        history_from_dynamic_bundle(bundle, body="chassis", channels=("heave_mm",))


def test_history_adapter_rejects_missing_metric(tmp_path: Path) -> None:
    bundle = load_dynamic_result(_write(tmp_path / "bundle.json", _bundle_payload()))

    with pytest.raises(ValueError, match="is unavailable for body"):
        history_from_dynamic_bundle(bundle, body="axle", channels=("camber_deg",))


def test_history_adapter_keeps_consuming_unified_time_series_result() -> None:
    result = TimeSeriesResult.from_samples(
        (
            TimeSeriesSample(time=0.0, body="axle", metrics={"heave_mm": 0.0}),
            TimeSeriesSample(time=0.1, body="axle", metrics={"heave_mm": 1.0}),
        ),
        mode="axle_dynamic",
    )

    history = history_from_dynamic_bundle(result, body="axle", channels=("heave_mm",))

    assert history.time == (0.0, 0.1)
    assert history.channels["heave_mm"] == (0.0, 1.0)
