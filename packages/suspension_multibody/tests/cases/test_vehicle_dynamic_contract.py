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

from suspension_multibody.schema import RoadSurfaceSpec, TimeSignal, Vec3
from tests.vehicle._unified_entry import compile_vehicle, solve_vehicle

_FIXTURE = Path(__file__).resolve().parents[1] / "vehicle" / "vehicle_fixtures.py"


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
    return solve_vehicle(model, case).run.raw


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
    """
    A raised road and a stated initial state are case inputs, not model ones.

    The road is raised by a *physical* bump: 50 mm is a bump, while the 1 m this
    test used to raise it by is three wheel radii of penetration, and a state
    given with that much penetration *and* a 10 m/s slip is a corner the tire's
    relaxation cannot start the integration from.  Measured on this fixture: 1 m
    with a zero velocity starts, 1 m with 1 m/s starts, 1 m with 10 m/s does not,
    50 mm with 10 m/s does.  The assertions below are unchanged -- the road and
    the state still have to reach the solver and still have to carry the wheel.
    """
    model = fixture._positioned_vehicle(fixture._vehicle())
    default = _run(model, fixture._case(model))
    case = fixture._case(model).model_copy(
        update={
            "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=0.05)),
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
    from suspension_contracts import unpack_container

    model = fixture._positioned_vehicle(fixture._pac2002_model(combined=True))
    compiled = compile_vehicle(model, fixture._case(model))
    document = compiled.model_document
    _, model_blob = unpack_container(compiled.model_payload)
    assert document["capabilities"] == ["vehicle"]
    assert document["units"]["length"] == "m"
    assert len(document["tires"]) == 4
    kinds = {element["type"] for element in document["elements"]}
    assert "steering_actuator" in kinds
    assert "spring_damper" not in kinds
    assert len({entry["name"] for entry in document["blobs"]}) == 4
    for entry in document["blobs"]:
        assert entry["shape"] == [226]
        assert entry["offset"] + entry["length"] <= len(model_blob)
    assert len(model_blob) == 4 * 226 * 8


def test_an_axial_corner_is_emitted_as_the_three_element_types(fixture) -> None:
    """
    A corner's spring, damper and stop leave as three named elements.

    The contract document is what the kernel reads, so the names in it are the
    interface.  This asserts them by name rather than by count: a rename that
    kept the count would still be a document the kernel cannot read.
    """
    from suspension_multibody.schema import StaticDamper

    model = fixture._positioned_vehicle(
        fixture._vehicle(
            front_dampers=(
                StaticDamper(
                    name="gas_damper",
                    body_a="chassis",
                    body_b="lower_arm",
                    point_a=fixture.Vec3(x=0, y=-500, z=100),
                    point_b=fixture.Vec3(x=0, y=-700, z=100),
                    viscous_damping=12.0,
                ),
            )
        )
    )
    document = compile_vehicle(model, fixture._case(model)).model_document
    kinds = {element["type"] for element in document["elements"]}

    # This fixture's axle declares dampers and no springs, so `damper` is the
    # name that must appear -- it proves the record left as its own element type
    # rather than being folded into a spring's parameters.  The retired fused
    # name must not appear.
    assert "damper" in kinds, sorted(kinds)
    assert "spring" not in kinds, sorted(kinds)
    assert "spring_damper" not in kinds


def test_the_case_document_carries_the_road_and_the_steering(fixture) -> None:
    from suspension_contracts import unpack_container

    model = fixture._positioned_vehicle(fixture._vehicle())
    case = fixture._case(model, brake=0.3, steering=TimeSignal(constant=0.02))
    compiled = compile_vehicle(model, case)
    document = compiled.case_document
    _, blob = unpack_container(compiled.case_payload)
    assert document["family"] == "vehicle_dynamic"
    assert document["inputs"]["road"]["kind"] == "plane"
    roles = {entry["role"] for entry in document["blobs"]}
    assert {"steering_target", "steering_rate", "brake_torque"} <= roles
    for entry in document["blobs"]:
        assert entry["offset"] + entry["length"] <= len(blob)


@pytest.mark.parametrize("case_family", ["vehicle_dynamic", "handling"])
def test_documents_define_identity_without_a_prepared_context(case_family, tmp_path):
    from suspension_multibody.api import validate

    from ._family_documents import assert_document_parity, fixture_documents
    from .test_handling import _case

    assembly, case = fixture_documents()
    if case_family == "handling":
        case, _ = _case(case, "ramp")
    compiled = assert_document_parity(assembly, case, tmp_path)
    assert compiled.case_document["family"] == case_family
    with pytest.raises(TypeError):
        validate(assembly, case, context={"vehicle_dynamic_prepared": object()})
