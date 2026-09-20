"""
The handling family: the manoeuvre it expands is the manoeuvre it was asked for.

Same shape of check as the four-post family, for the same reason: the family's
whole job is expanding a declaration, so it is expanded twice -- once in the
kernel, once here -- and both runs are compared.
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
    vehicle_dynamic_model_document,
)
from suspension_multibody.schema import Vec3
from suspension_multibody.simulation import SimulationRequest, run_request
from suspension_multibody.vehicle_dynamics import prepare_vehicle_run

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


@pytest.mark.parametrize("shape", ["constant", "ramp", "step", "sine"])
def test_the_family_matches_an_independently_expanded_manoeuvre(
    prepared, shape: str
) -> None:
    model, case, base = prepared
    times = tuple(float(value) for value in base.times)
    actuator = base.steering.names[0]
    # Every shape starts from the settled straight-running state, which is the
    # only state the static trim can reach from the assembling pose.
    shapes = (
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
    document = handling_case_document(
        name=f"handling-{shape}", shapes=shapes, times_s=times,
        settings=AxleSolverSettings(),
    )
    model_doc, model_blob = vehicle_dynamic_model_document(model, base)
    model_payload = pack_container(model_doc, model_blob)
    produced = run_request(
        SimulationRequest(
            assembly="vehicle",
            family="handling",
            model=model_doc,
            case=document,
            context={"model_payload": model_payload},
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
    # case's own settings.
    explicit = replace(base, steering=steering, solver=AxleSolverSettings())
    reference = run_request(
        SimulationRequest(
            assembly="vehicle",
            family="vehicle_dynamic",
            model=model,
            case=case,
            context={"prepared": explicit},
        )
    ).raw

    assert produced.status == "success"
    difference = np.abs(produced.block("body_state") - reference.block("body_state"))
    assert float(difference.max()) < 1e-12, f"max difference {difference.max():.3e}"


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
