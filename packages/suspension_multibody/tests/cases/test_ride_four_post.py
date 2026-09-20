"""
The four-post family: the excitation it expands is the excitation it was asked for.

The only honest way to check a family whose job is to expand a declaration is to
expand it twice, independently, and run both.  So the same corner shapes are
sampled here with numpy and handed to the *vehicle* family as an explicit table,
and the two runs have to agree.

The family's own run goes through the preparation registry from the domain
objects -- the vehicle model and the rig's corners -- while the reference run is
a document request that is compiled without any preparation.
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
    vehicle_dynamic_case_document,
    vehicle_dynamic_model_document,
)
from suspension_multibody.preparation.ride_four_post import (
    FourPostRide,
    RideFourPostPrepared,
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


def _document_registry() -> CompilerRegistry:
    """Return a registry that carries authored vehicle documents through."""
    registry = CompilerRegistry()
    registry.register(DocumentPairCompiler("vehicle", "vehicle_dynamic"))
    return registry


def test_the_family_matches_an_independently_sampled_excitation(prepared) -> None:
    model, case, base = prepared
    times = tuple(float(value) for value in base.times)
    settings = AxleSolverSettings()
    produced = run_request(
        SimulationRequest(
            assembly="vehicle",
            family="ride_four_post",
            model=model,
            case=FourPostRide(vehicle_case=case, corners=_CORNERS, name="four-post"),
        )
    ).raw

    # The same excitation, computed here rather than by the kernel, handed to
    # the vehicle family as explicit per-tire tables.
    height, velocity = ride_four_post_corner_signals(_CORNERS, times)
    # The explicit reference has to run on the solver block the family document
    # declares, so it is prepared with that solver rather than the fixture
    # case's own settings.  It carries its own documents, which is what makes it
    # a document request: the vehicle family's compiler frames them as they are.
    explicit = replace(
        base,
        road_height=height,
        road_velocity=velocity,
        solver=settings,
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

    assert produced.status == "success"
    assert reference.status == "success"
    difference = np.abs(produced.block("body_state") - reference.block("body_state"))
    # The two sides evaluate the same sine with different libraries, so the
    # agreement is to rounding rather than bit-for-bit.
    assert float(difference.max()) < 1e-12, f"max difference {difference.max():.3e}"


def test_the_family_prepares_its_documents_through_the_default_registry(
    prepared,
) -> None:
    model, case, base = prepared
    request = SimulationRequest(
        assembly="vehicle",
        family="ride_four_post",
        model=model,
        case=FourPostRide(vehicle_case=case, corners=_CORNERS, name="four-post-prepared"),
    )

    registry = default_preparation_registry()
    assert ("vehicle", "ride_four_post") in registry.keys()
    result = prepare_request(request)

    assert isinstance(result.value, RideFourPostPrepared)
    assert result.context["prepared_simulation"].value is result.value
    assert result.context["case_document"]["family"] == "ride_four_post"
    assert result.context["model_document"]["name"] == "four-post-prepared"
    compiled = compile_request(result.request)
    assert compiled.model_document == result.value.model_document
    assert compiled.case_document == result.value.case_document
    assert compiled.model_payload == result.value.model_payload


def test_a_document_request_bypasses_preparation_and_still_validates_identity(
    prepared, monkeypatch
) -> None:
    from suspension_multibody.preparation import ride_four_post as four_post_preparation

    model, case, base = prepared
    calls: list[SimulationRequest] = []
    monkeypatch.setattr(
        four_post_preparation, "prepare_request", lambda request: calls.append(request)
    )
    times = tuple(float(value) for value in base.times)
    document = ride_four_post_case_document(
        name="four-post",
        corners=_CORNERS,
        times_s=times,
        settings=AxleSolverSettings(),
    )
    model_doc, model_blob = vehicle_dynamic_model_document(model, base)
    model_payload = pack_container(model_doc, model_blob)
    request = SimulationRequest(
        assembly="vehicle",
        family="ride_four_post",
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
                family="ride_four_post",
                model=model_doc,
                case=wrong_family,
                context={"model_payload": model_payload},
            )
        )


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
