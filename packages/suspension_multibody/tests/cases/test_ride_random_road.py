"""
The random-road family: the road it drives down is the road it was asked for.

Same shape of check as the other expansion families: the components are expanded
twice -- once in the kernel, once here -- and both runs are compared.

The family's own run goes through the preparation registry from the domain
objects -- the vehicle model, the profiles and the speed -- while the reference
run is a document request that is compiled without any preparation.
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
    RandomRoadWheel,
    RoadComponent,
    ride_random_road_case_document,
    ride_random_road_signals,
    vehicle_dynamic_case_document,
    vehicle_dynamic_model_document,
)
from suspension_multibody.preparation.ride_random_road import (
    RandomRoadRide,
    RideRandomRoadPrepared,
)
from suspension_multibody.preparation.vehicle_dynamic import prepare_vehicle_run
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

#: A rough road: long wavelengths carry the ride, short ones the harshness.
_PROFILE = (
    RoadComponent(amplitude_m=0.004, wavelength_m=25.0),
    RoadComponent(amplitude_m=0.002, wavelength_m=8.0, phase_rad=0.7),
    RoadComponent(amplitude_m=0.0008, wavelength_m=2.5, phase_rad=2.1),
)

_SPEED_MPS = 20.0


def _fixture():
    spec = importlib.util.spec_from_file_location("random_road_vehicle_fixture", _FIXTURE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def prepared():
    fixture = _fixture()
    model = fixture._positioned_vehicle(fixture._vehicle())
    return model, fixture._case(model), prepare_vehicle_run(model, fixture._case(model))


def _wheels(names: tuple[str, ...]) -> tuple[RandomRoadWheel, ...]:
    # Front wheels lead; the rear pair follows the same profile after the
    # wheelbase, which is a phase on every component.
    wheels = []
    for name in names:
        delay = 0.0 if name.startswith("front") else 2.4 / _SPEED_MPS
        components = tuple(
            RoadComponent(
                amplitude_m=component.amplitude_m,
                wavelength_m=component.wavelength_m,
                phase_rad=component.phase_rad
                - 2.0 * np.pi * delay * _SPEED_MPS / component.wavelength_m,
            )
            for component in _PROFILE
        )
        wheels.append(RandomRoadWheel(name, components))
    return tuple(wheels)


def _document_registry() -> CompilerRegistry:
    """Return a registry that carries authored vehicle documents through."""
    registry = CompilerRegistry()
    registry.register(DocumentPairCompiler("vehicle", "vehicle_dynamic"))
    return registry


def test_the_family_matches_an_independently_expanded_road(prepared) -> None:
    model, case, base = prepared
    times = tuple(float(value) for value in base.times)
    wheels = _wheels(tuple(model.wheels[index].name for index in range(len(model.wheels))))
    produced = run_request(
        SimulationRequest(
            assembly="vehicle",
            family="ride_random_road",
            model=model,
            case=RandomRoadRide(
                vehicle_case=case,
                wheels=wheels,
                speed_mps=_SPEED_MPS,
                name="random-road",
            ),
        )
    ).raw

    height, velocity = ride_random_road_signals(wheels, _SPEED_MPS, times)
    # The explicit reference has to run on the solver block the family document
    # declares, so it is prepared with that solver rather than the fixture
    # case's own settings.  It carries its own documents, which is what makes it
    # a document request: the vehicle family's compiler frames them as they are.
    explicit = replace(
        base,
        road_height=height,
        road_velocity=velocity,
        solver=AxleSolverSettings(),
    )
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

    difference = np.abs(produced.block("body_state") - reference.block("body_state"))
    assert float(difference.max()) < 1e-12, f"max difference {difference.max():.3e}"


def test_the_family_prepares_its_documents_through_the_default_registry(
    prepared,
) -> None:
    model, case, base = prepared
    wheels = _wheels(("front_left",))
    request = SimulationRequest(
        assembly="vehicle",
        family="ride_random_road",
        model=model,
        case=RandomRoadRide(
            vehicle_case=case,
            wheels=wheels,
            speed_mps=_SPEED_MPS,
            name="random-road-prepared",
        ),
    )

    registry = default_preparation_registry()
    assert ("vehicle", "ride_random_road") in registry.keys()
    result = prepare_request(request)

    assert isinstance(result.value, RideRandomRoadPrepared)
    assert result.context["prepared_simulation"].value is result.value
    assert result.context["case_document"]["family"] == "ride_random_road"
    assert result.context["model_document"]["name"] == "random-road-prepared"
    assert result.context["case_document"]["ride_random_road"]["speed_mps"] == _SPEED_MPS
    compiled = compile_request(result.request)
    assert compiled.model_document == result.value.model_document
    assert compiled.case_document == result.value.case_document
    assert compiled.model_payload == result.value.model_payload


def test_a_document_request_bypasses_preparation_and_still_validates_identity(
    prepared, monkeypatch
) -> None:
    from suspension_multibody.preparation import (
        ride_random_road as random_road_preparation,
    )

    model, case, base = prepared
    calls: list[SimulationRequest] = []
    monkeypatch.setattr(
        random_road_preparation,
        "prepare_request",
        lambda request: calls.append(request),
    )
    times = tuple(float(value) for value in base.times)
    wheels = _wheels(("front_left",))
    document = ride_random_road_case_document(
        name="random-road",
        wheels=wheels,
        speed_mps=_SPEED_MPS,
        times_s=times,
        settings=AxleSolverSettings(),
    )
    model_doc, model_blob = vehicle_dynamic_model_document(model, base)
    model_payload = pack_container(model_doc, model_blob)
    request = SimulationRequest(
        assembly="vehicle",
        family="ride_random_road",
        model=model_doc,
        case=document,
        context={"model_payload": model_payload},
    )

    bypassed = prepare_request(request)

    assert calls == []
    assert "prepared_simulation" not in bypassed.context
    compiled = compile_request(bypassed.request)
    assert compiled.case_document == document
    assert compiled.model_payload == model_payload
    # Bypassing preparation is not skipping the compiler: the documents still
    # have to satisfy the family's contract identity.
    wrong_family = dict(document)
    wrong_family["family"] = "handling"
    with pytest.raises(ValueError, match="case family"):
        compile_request(
            SimulationRequest(
                assembly="vehicle",
                family="ride_random_road",
                model=model_doc,
                case=wrong_family,
                context={"model_payload": model_payload},
            )
        )


def test_a_component_without_a_wavelength_is_refused(prepared) -> None:
    model, case, base = prepared
    times = tuple(float(value) for value in base.times)
    wheels = (RandomRoadWheel("front_left", (RoadComponent(0.001, 0.0),)),)
    document = ride_random_road_case_document(
        name="random-road", wheels=wheels, speed_mps=_SPEED_MPS, times_s=times,
        settings=AxleSolverSettings(),
    )
    model_doc, model_blob = vehicle_dynamic_model_document(model, base)
    with pytest.raises(Exception, match="positive wavelength|wavelength"):
        run_request(
            SimulationRequest(
                assembly="vehicle",
                family="ride_random_road",
                model=model_doc,
                case=document,
                context={"model_payload": pack_container(model_doc, model_blob)},
            )
        )


def test_an_unknown_wheel_is_refused(prepared) -> None:
    model, case, base = prepared
    times = tuple(float(value) for value in base.times)
    wheels = (RandomRoadWheel("middle_left", _PROFILE),)
    document = ride_random_road_case_document(
        name="random-road", wheels=wheels, speed_mps=_SPEED_MPS, times_s=times,
        settings=AxleSolverSettings(),
    )
    model_doc, model_blob = vehicle_dynamic_model_document(model, base)
    with pytest.raises(Exception, match="unknown tire"):
        run_request(
            SimulationRequest(
                assembly="vehicle",
                family="ride_random_road",
                model=model_doc,
                case=document,
                context={"model_payload": pack_container(model_doc, model_blob)},
            )
        )
