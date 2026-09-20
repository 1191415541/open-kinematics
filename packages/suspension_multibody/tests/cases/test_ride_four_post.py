"""
The four-post family: the excitation it expands is the excitation it was asked for.

The only honest way to check a family whose job is to expand a declaration is to
expand it twice, independently, and run both.  So the same corner shapes are
sampled here with numpy and handed to the *vehicle* family as an explicit table,
and the two runs have to agree.
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
    FourPostCorner,
    ride_four_post_case_document,
    ride_four_post_corner_signals,
    vehicle_dynamic_model_document,
)
from suspension_multibody.simulation import SimulationRequest, run_request
from suspension_multibody.vehicle_dynamics import prepare_vehicle_run

_FIXTURE = Path(__file__).resolve().parents[1] / "vehicle" / "test_native_vehicle.py"

# The pads start at zero so the assembling pose is a solution of the static
# problem, and the shapes are small enough that no corner changes contact state
# inside the short window this fixture runs for.
_CORNERS = (
    FourPostCorner("front_left", amplitude_m=0.002, frequency_hz=8.0),
    FourPostCorner("front_right", amplitude_m=0.002, frequency_hz=8.0),
    FourPostCorner("rear_left", amplitude_m=0.0015, frequency_hz=6.0, phase_rad=0.3),
    FourPostCorner("rear_right", amplitude_m=0.0015, frequency_hz=6.0, phase_rad=-0.3),
)


def _fixture():
    spec = importlib.util.spec_from_file_location("four_post_vehicle_fixture", _FIXTURE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def prepared():
    fixture = _fixture()
    model = fixture._positioned_vehicle(fixture._vehicle())
    case = fixture._case(model)
    return model, case, prepare_vehicle_run(model, case)


def test_the_family_matches_an_independently_sampled_excitation(prepared) -> None:
    model, case, base = prepared
    times = tuple(float(value) for value in base.times)
    settings = AxleSolverSettings()
    document = ride_four_post_case_document(
        name="four-post", corners=_CORNERS, times_s=times, settings=settings
    )
    model_doc, model_blob = vehicle_dynamic_model_document(model, base)
    produced = run_request(
        SimulationRequest(
            assembly="vehicle",
            family="ride_four_post",
            model=model_doc,
            case=document,
            context={"model_payload": pack_container(model_doc, model_blob)},
        )
    ).raw

    # The same excitation, computed here rather than by the kernel, handed to
    # the vehicle family as explicit per-tire tables.
    height, velocity = ride_four_post_corner_signals(_CORNERS, times)
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

    assert produced.status == "success"
    assert reference.status == "success"
    difference = np.abs(produced.block("body_state") - reference.block("body_state"))
    # The two sides evaluate the same sine with different libraries, so the
    # agreement is to rounding rather than bit-for-bit.
    assert float(difference.max()) < 1e-12, f"max difference {difference.max():.3e}"


def test_a_corner_without_a_shape_is_rejected(prepared) -> None:
    model, case, base = prepared
    times = tuple(float(value) for value in base.times)
    document = ride_four_post_case_document(
        name="four-post",
        corners=_CORNERS,
        times_s=times,
        settings=AxleSolverSettings(),
    )
    document["four_post"]["corners"][0]["tire"] = "not_a_wheel"
    model_doc, model_blob = vehicle_dynamic_model_document(model, base)
    with pytest.raises(Exception, match="unknown tire"):
        run_request(
            SimulationRequest(
                assembly="vehicle",
                family="ride_four_post",
                model=model_doc,
                case=document,
                context={"model_payload": pack_container(model_doc, model_blob)},
            )
        )
