"""
The random-road family: the road it drives down is the road it was asked for.

Same shape of check as the other expansion families: the components are expanded
twice -- once in the kernel, once here -- and both runs are compared.
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
    vehicle_dynamic_model_document,
)
from suspension_multibody.simulation import SimulationRequest, run_request
from suspension_multibody.vehicle_dynamics import prepare_vehicle_run

_FIXTURE = Path(__file__).resolve().parents[1] / "vehicle" / "test_native_vehicle.py"

#: A rough road: long wavelengths carry the ride, short ones the harshness.
_PROFILE = (
    RoadComponent(amplitude_m=0.004, wavelength_m=25.0),
    RoadComponent(amplitude_m=0.002, wavelength_m=8.0, phase_rad=0.7),
    RoadComponent(amplitude_m=0.0008, wavelength_m=2.5, phase_rad=2.1),
)


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
        delay = 0.0 if name.startswith("front") else 2.4 / 20.0
        components = tuple(
            RoadComponent(
                amplitude_m=component.amplitude_m,
                wavelength_m=component.wavelength_m,
                phase_rad=component.phase_rad
                - 2.0 * np.pi * delay * 20.0 / component.wavelength_m,
            )
            for component in _PROFILE
        )
        wheels.append(RandomRoadWheel(name, components))
    return tuple(wheels)


def test_the_family_matches_an_independently_expanded_road(prepared) -> None:
    model, case, base = prepared
    times = tuple(float(value) for value in base.times)
    wheels = _wheels(tuple(model.wheels[index].name for index in range(len(model.wheels))))
    wheels = tuple(
        RandomRoadWheel(wheel.tire, wheel.components) for wheel in wheels
    )
    document = ride_random_road_case_document(
        name="random-road", wheels=wheels, speed_mps=20.0, times_s=times,
        settings=AxleSolverSettings(),
    )
    model_doc, model_blob = vehicle_dynamic_model_document(model, base)
    model_payload = pack_container(model_doc, model_blob)
    produced = run_request(
        SimulationRequest(
            assembly="vehicle",
            family="ride_random_road",
            model=model_doc,
            case=document,
            context={"model_payload": model_payload},
        )
    ).raw

    height, velocity = ride_random_road_signals(wheels, 20.0, times)
    # The explicit reference has to run on the solver block the family document
    # declares, so it is prepared with that solver rather than the fixture
    # case's own settings.
    explicit = replace(
        base,
        road_height=height,
        road_velocity=velocity,
        solver=AxleSolverSettings(),
    )
    reference = run_request(
        SimulationRequest(
            assembly="vehicle",
            family="vehicle_dynamic",
            model=model,
            case=case,
            context={"prepared": explicit},
        )
    ).raw

    difference = np.abs(produced.block("body_state") - reference.block("body_state"))
    assert float(difference.max()) < 1e-12, f"max difference {difference.max():.3e}"


def test_a_component_without_a_wavelength_is_refused(prepared) -> None:
    model, case, base = prepared
    times = tuple(float(value) for value in base.times)
    wheels = (RandomRoadWheel("front_left", (RoadComponent(0.001, 0.0),)),)
    document = ride_random_road_case_document(
        name="random-road", wheels=wheels, speed_mps=20.0, times_s=times,
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
        name="random-road", wheels=wheels, speed_mps=20.0, times_s=times,
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
