"""
The handling family: the manoeuvre it expands is the manoeuvre it was asked for.

Same shape of check as the four-post family, for the same reason: the family's
whole job is expanding a declaration, so it is expanded twice -- once in the
kernel, once here -- and both runs are compared.

The family's own run goes through the preparation registry from the domain
objects -- the vehicle model and the manoeuvre -- while the reference run is the
same excitation handed to the vehicle family as an explicit table, which is a
document request and is therefore compiled without any preparation.
"""

from __future__ import annotations

import importlib.util
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from suspension_contracts import pack_container

from suspension_multibody.axle_dynamics.schema import AxleSolverSettings
from suspension_multibody.cases import (
    SteeringShape,
    handling_case_document,
    handling_steering_signals,
    vehicle_dynamic_case_document,
    vehicle_dynamic_model_document,
)
from suspension_multibody.preparation.handling import (
    HandlingManoeuvre,
    HandlingPrepared,
)
from suspension_multibody.preparation.vehicle_dynamic import prepare_vehicle_run
from suspension_multibody.schema import Vec3
from suspension_multibody.simulation import (
    CompilerRegistry,
    DocumentPairCompiler,
    SimulationRequest,
    compile_request,
    default_preparation_registry,
    prepare_request,
    run_request,
)

_FIXTURE = Path(__file__).resolve().parents[1] / "vehicle" / "test_native_vehicle.py"


def _fixture():
    spec = importlib.util.spec_from_file_location("handling_vehicle_fixture", _FIXTURE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def prepared():
    fixture = _fixture()
    model = fixture._positioned_vehicle(fixture._vehicle())
    # The shared fixture runs with gravity switched off, which leaves the
    # vehicle with no load path at all: a prescribed steering angle then has a
    # singular static problem because nothing anchors the body in space.  A
    # manoeuvre happens on the ground, so this fixture puts it there.
    base = fixture._case(model)
    case = base.model_copy(
        update={
            "solver": base.solver.model_copy(
                update={
                    # Gravity is expressed in the model's own length unit, so
                    # a vehicle model in millimetres wants mm/s^2 here.
                    "gravity": Vec3(x=0.0, y=0.0, z=-9806.65),
                    "end_time": 0.004,
                    "step_size": 0.001,
                    "internal_step_size": 0.001,
                    "min_internal_step_size": 0.001,
                }
            )
        }
    )
    state = prepare_vehicle_run(model, case)
    return model, case, state


def _shapes(base, shape: str) -> tuple[SteeringShape, ...]:
    actuator = base.steering.names[0]
    return (
        SteeringShape(
            actuator,
            shape,  # type: ignore[arg-type]
            amplitude=0.008,
            start_s=5e-4,
            rise_s=5e-4,
            frequency_hz=4.0,
            phase_rad=0.0,
        ),
    )


def _document_registry() -> CompilerRegistry:
    """Return a registry that carries authored vehicle documents through."""
    registry = CompilerRegistry()
    registry.register(DocumentPairCompiler("vehicle", "vehicle_dynamic"))
    return registry


@pytest.mark.parametrize("shape", ["constant", "ramp", "step", "sine"])
def test_the_family_matches_an_independently_expanded_manoeuvre(
    prepared, shape: str
) -> None:
    model, case, base = prepared
    times = tuple(float(value) for value in base.times)
    actuator = base.steering.names[0]
    # Every shape starts from the settled straight-running state, which is the
    # only state the static trim can reach from the assembling pose.
    shapes = _shapes(base, shape)
    produced = run_request(
        SimulationRequest(
            assembly="vehicle",
            family="handling",
            model=model,
            case=HandlingManoeuvre(
                vehicle_case=case, shapes=shapes, name=f"handling-{shape}"
            ),
        )
    ).raw

    target, rate = handling_steering_signals(shapes, times)
    steering = replace(
        base.steering,
        target=np.asarray(target[actuator], dtype=float),
        target_rate=np.asarray(rate[actuator], dtype=float),
    )
    # The explicit reference has to run on the solver block the family document
    # declares, so it is prepared with that solver rather than the fixture
    # case's own settings.  It carries its own documents, which is what makes it
    # a document request: the vehicle family's compiler frames them as they are.
    explicit = replace(base, steering=steering, solver=AxleSolverSettings())
    model_doc, model_blob = vehicle_dynamic_model_document(model, explicit)
    case_doc, case_blob = vehicle_dynamic_case_document(model, case, explicit)
    reference = run_request(
        SimulationRequest(
            assembly="vehicle",
            family="vehicle_dynamic",
            model=model_doc,
            case=case_doc,
            context={
                "model_payload": pack_container(model_doc, model_blob),
                "case_payload": pack_container(case_doc, case_blob),
            },
        ),
        registry=_document_registry(),
    ).raw

    assert produced.status == "success"
    difference = np.abs(produced.block("body_state") - reference.block("body_state"))
    assert float(difference.max()) < 1e-12, f"max difference {difference.max():.3e}"


def test_the_family_prepares_its_documents_through_the_default_registry(
    prepared,
) -> None:
    model, case, base = prepared
    shapes = _shapes(base, "ramp")
    request = SimulationRequest(
        assembly="vehicle",
        family="handling",
        model=model,
        case=HandlingManoeuvre(vehicle_case=case, shapes=shapes, name="handling-prepared"),
    )

    registry = default_preparation_registry()
    assert ("vehicle", "handling") in registry.keys()
    result = prepare_request(request)

    assert isinstance(result.value, HandlingPrepared)
    assert result.context["prepared_simulation"].value is result.value
    assert result.context["case_document"]["family"] == "handling"
    assert result.context["model_document"]["name"] == "handling-prepared"
    # The compiler consumes the prepared context: the documents it submits are
    # the prepared ones, and it does not author them again.
    compiled = compile_request(result.request)
    assert compiled.model_document == result.value.model_document
    assert compiled.case_document == result.value.case_document
    assert compiled.model_payload == result.value.model_payload


def test_a_document_request_bypasses_preparation_and_still_validates_identity(
    prepared, monkeypatch
) -> None:
    from suspension_multibody.preparation import handling as handling_preparation

    model, case, base = prepared
    calls: list[SimulationRequest] = []
    monkeypatch.setattr(
        handling_preparation, "prepare_request", lambda request: calls.append(request)
    )
    times = tuple(float(value) for value in base.times)
    document = handling_case_document(
        name="handling-bypass",
        shapes=_shapes(base, "ramp"),
        times_s=times,
        settings=AxleSolverSettings(),
    )
    model_doc, model_blob = vehicle_dynamic_model_document(model, base)
    model_payload = pack_container(model_doc, model_blob)
    request = SimulationRequest(
        assembly="vehicle",
        family="handling",
        model=model_doc,
        case=document,
        context={"model_payload": model_payload},
    )

    bypassed = prepare_request(request)

    # The request already carries its documents, so no family preparation runs
    # and the assembly is not built a second time.
    assert calls == []
    assert "prepared_simulation" not in bypassed.context
    compiled = compile_request(bypassed.request)
    assert compiled.case_document == document
    assert compiled.model_payload == model_payload
    # Bypassing preparation is not skipping the compiler: the documents still
    # have to satisfy the family's contract identity.
    wrong_family = dict(document)
    wrong_family["family"] = "vehicle_kc"
    with pytest.raises(ValueError, match="case family"):
        compile_request(
            SimulationRequest(
                assembly="vehicle",
                family="handling",
                model=model_doc,
                case=wrong_family,
                context={"model_payload": model_payload},
            )
        )


def test_a_closed_loop_manoeuvre_is_refused_by_name(prepared) -> None:
    model, case, base = prepared
    times = tuple(float(value) for value in base.times)
    actuator = base.steering.names[0]
    document = handling_case_document(
        name="handling-lane-change",
        shapes=(SteeringShape(actuator, "ramp", amplitude=0.01, rise_s=1e-3),),
        times_s=times,
        settings=AxleSolverSettings(),
    )
    # A closed-loop manoeuvre is a driver model, not a shape; the case layer has
    # to say so rather than quietly approximating one.
    document["handling"]["steering"][0]["shape"] = "iso_lane_change"
    model_doc, model_blob = vehicle_dynamic_model_document(model, base)
    with pytest.raises(Exception, match="not open-loop"):
        run_request(
            SimulationRequest(
                assembly="vehicle",
                family="handling",
                model=model_doc,
                case=document,
                context={"model_payload": pack_container(model_doc, model_blob)},
            )
        )
