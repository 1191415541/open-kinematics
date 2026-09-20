"""
Full-vehicle dynamic authoring and contract-boundary tests.

The bit-exact reference is the frozen ctypes snapshot checked by
``scripts/case_parity_check.py``; these tests stay independent of the retired
flat Python boundary and pin the case-document behavior that snapshot cannot
explain by itself.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest

from suspension_multibody.preparation.vehicle_dynamic import prepare_vehicle_run
from suspension_multibody.schema import RoadSurfaceSpec, TimeSignal, Vec3
from suspension_multibody.simulation import SimulationRequest, run_request

_FIXTURE = Path(__file__).resolve().parents[1] / "vehicle" / "test_native_vehicle.py"


def _fixture():
    spec = importlib.util.spec_from_file_location("native_vehicle_fixture", _FIXTURE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def fixture():
    return _fixture()


def _run(model, case):
    """Run one case through the unified runner on the default preparation."""
    return run_request(
        SimulationRequest(
            assembly="vehicle",
            family="vehicle_dynamic",
            model=model,
            case=case,
            context={"prepared": prepare_vehicle_run(model, case)},
        )
    ).raw


@pytest.mark.parametrize("kind", ["native_brush", "pac2002", "fiala"])
def test_contract_path_runs_every_tire_model(fixture, kind: str) -> None:
    base = fixture._positioned_vehicle(fixture._vehicle())
    model = base.model_copy(
        update={
            "wheels": tuple(
                wheel.model_copy(
                    update={"tire": wheel.tire.model_copy(update={"kind": kind})}
                )
                for wheel in base.wheels
            )
        }
    )
    case = fixture._case(model)
    produced = _run(model, case)
    assert produced.status == "success"
    assert [entry["name"] for entry in produced.cases] == [case.name]
    assert produced.block("body_state").shape[1] == len(produced.document["manifest"]["bodies"])


def test_nondefault_road_and_initial_state_are_case_inputs(fixture) -> None:
    model = fixture._positioned_vehicle(fixture._vehicle())
    default = _run(model, fixture._case(model))
    case = fixture._case(model).model_copy(
        update={
            "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
            "initial_states": fixture._uniform_velocity_initial_states(model),
        }
    )
    changed = _run(model, case)
    assert changed.status == "success"
    assert not np.array_equal(
        changed.block("body_state"), default.block("body_state")
    )
    assert changed.block("tire_output")[-1, 0, 4] > 0.0


def test_a_measured_vertical_table_reaches_the_contract(fixture) -> None:
    table = ((0.0, 0.0), (0.025, 5000.0), (0.05, 10000.0), (0.1, 20000.0))
    base = fixture._positioned_vehicle(fixture._pac2002_model(combined=True))
    model = base.model_copy(
        update={
            "wheels": tuple(
                wheel.model_copy(
                    update={
                        "tire": wheel.tire.model_copy(
                            update={
                                "pac2002_tables": {
                                    "deflection_load_curve": table,
                                }
                            }
                        )
                    }
                )
                for wheel in base.wheels
            )
        }
    )
    produced = _run(model, fixture._case(model))
    assert produced.status == "success"
    assert produced.block("tire_output").shape == (2, 4, 41)


def test_the_model_document_declares_the_vehicle_path(fixture) -> None:
    from suspension_multibody.cases.vehicle_dynamic import model_document
    from suspension_multibody.preparation.vehicle_dynamic import prepare_vehicle_run

    model = fixture._positioned_vehicle(fixture._vehicle())
    prepared = prepare_vehicle_run(model, fixture._case(model))
    document, model_blob = model_document(model, prepared)
    assert document["capabilities"] == ["vehicle"]
    assert document["units"]["length"] == "m"
    assert len(document["tires"]) == 4
    kinds = {element["type"] for element in document["elements"]}
    assert "steering_actuator" in kinds
    assert "spring_damper" not in kinds
    assert [entry["name"] for entry in document["blobs"]] == [
        f"tire-parameters-{index}" for index in range(4)
    ]
    for entry in document["blobs"]:
        assert entry["shape"] == [226]
        assert entry["offset"] + entry["length"] <= len(model_blob)
    assert len(model_blob) == 4 * 226 * 8


def test_the_case_document_carries_the_road_and_the_steering(fixture) -> None:
    from suspension_multibody.cases.vehicle_dynamic import case_document
    from suspension_multibody.preparation.vehicle_dynamic import prepare_vehicle_run

    model = fixture._positioned_vehicle(fixture._vehicle())
    case = fixture._case(model, brake=0.3, steering=TimeSignal(constant=0.02))
    prepared = prepare_vehicle_run(model, case)
    document, blob = case_document(model, case, prepared)
    assert document["family"] == "vehicle_dynamic"
    assert document["inputs"]["road"]["kind"] == "plane"
    roles = {entry["role"] for entry in document["blobs"]}
    assert {"steering_target", "steering_rate", "brake_torque"} <= roles
    for entry in document["blobs"]:
        assert entry["offset"] + entry["length"] <= len(blob)


@pytest.mark.parametrize("document_location", ["values", "context"])
@pytest.mark.parametrize("case_family", ["vehicle_dynamic", "handling"])
def test_documents_take_priority_over_prepared_context(
    document_location: str, case_family: str, monkeypatch
) -> None:
    from suspension_contracts import CONTRACT_VERSION

    from suspension_multibody.cases import vehicle_dynamic
    from suspension_multibody.simulation import compile_request, prepare_request

    model_document = {
        "contract": "multibody-model",
        "contract_version": CONTRACT_VERSION,
        "kind": "model",
    }
    case_document = {
        "contract": "multibody-case",
        "contract_version": CONTRACT_VERSION,
        "kind": "case",
        "family": case_family,
    }
    context = {"vehicle_dynamic_prepared": object()}
    if document_location == "context":
        context.update(model_document=model_document, case_document=case_document)
        model, case = object(), object()
    else:
        model, case = model_document, case_document

    def refuse_authoring(*args, **kwargs):
        raise AssertionError("document bypass must not reauthor model or case")

    monkeypatch.setattr(vehicle_dynamic, "model_document", refuse_authoring)
    monkeypatch.setattr(vehicle_dynamic, "case_document", refuse_authoring)
    request = SimulationRequest(
        assembly="vehicle", family="vehicle_dynamic", model=model, case=case,
        context=context,
    )
    prepared = prepare_request(request)
    if case_family != "vehicle_dynamic":
        with pytest.raises(ValueError, match="family"):
            compile_request(prepared.request)
    else:
        compiled = compile_request(prepared.request)
        assert compiled.model_document == model_document
        assert compiled.case_document == case_document
