"""
The emitted K/C documents must be valid members of the contract.

This is the Python half of the contract boundary.  The kernel reads exactly
these documents, so what they assert is what the reader may rely on: the unit
block says millimetres, every point is body-local, a driven coordinate names the
signal that drives it, and the two families describe two genuinely different
models.
"""

from __future__ import annotations

import pytest
from suspension_contracts import (
    contract_hash,
    pack_container,
    unpack_container,
    validate_case,
    validate_model,
)

from suspension_multibody.analysis.benchmarks import benchmark_model
from suspension_multibody.model import build_front_axle
from suspension_multibody.native_kc import NativeKcError, case_document, model_document
from suspension_multibody.native_kc.load_paths import LoadPath

from .test_native_kc_parity import _compliant_model


def test_ideal_model_document_is_a_valid_contract_member() -> None:
    assembly = build_front_axle(benchmark_model(), "K")
    document = model_document(assembly, name="benchmark-k", drive_wheels=True)
    validate_model(document)
    assert document["units"]["length"] == "mm"
    assert len(document["bodies"]) == len(assembly.bodies)
    # The K model carries the rigid kinematic set with its paired ball joints
    # folded into their equivalent revolutes, plus the driven coordinates as
    # prescribed rows.
    from suspension_multibody.native_kc.convert import collapse_spherical_pairs

    expected = {joint.name for joint in collapse_spherical_pairs(assembly.ideal_constraints)}
    carried = {
        joint["name"]
        for joint in document["joints"]
        if not joint["type"].startswith("driven_")
    }
    assert carried == expected
    driven = [joint for joint in document["joints"] if joint["type"].startswith("driven_")]
    assert [joint["target"] for joint in driven] == [
        "wheel_drive_L",
        "wheel_drive_R",
        "rack_drive",
    ]
    assert {marker["name"] for marker in document["markers"]} == {
        "wheel_center_L",
        "wheel_center_R",
    }


def test_compliant_model_document_carries_its_bushings() -> None:
    assembly = build_front_axle(_compliant_model(), "C")
    document = model_document(assembly, name="benchmark-c", drive_wheels=False)
    validate_model(document)
    bushing_elements = [e for e in document["elements"] if e["type"] == "bushing"]
    assert len(bushing_elements) == len(assembly.bushings)
    assert len(bushing_elements[0]["parameters"]["stiffness"]) == 6
    # The compliant set keeps the arm mounts free and hands them to the
    # bushings, which is the whole physical difference from the K model.
    assert len([j for j in document["joints"] if not j["type"].startswith("driven_")]) == len(
        assembly.constraints
    )
    assert [j["target"] for j in document["joints"] if j["type"].startswith("driven_")] == [
        "rack_neutral"
    ]


def test_joint_points_are_body_local() -> None:
    assembly = build_front_axle(benchmark_model(), "K")
    document = model_document(assembly, name="benchmark-k", drive_wheels=True)
    # The frame conversion happens once, in the emitter: a constraint whose
    # world point differs from its body-local point must arrive local.
    constraint = assembly.ideal_constraints[0]
    body = assembly.bodies[constraint.body_a]
    entry = next(j for j in document["joints"] if j["name"] == constraint.name)
    expected = body.pose.inverse().transform_point(constraint.point_a)
    assert entry["point_a"] == pytest.approx([float(v) for v in expected])


def test_k_case_document_is_a_valid_contract_member() -> None:
    assembly = build_front_axle(benchmark_model(), "K")
    document = case_document(
        assembly,
        family="kc_quasi_static",
        name="k-grid",
        wheel_values_mm=(-10.0, 0.0, 10.0),
        rack_values_mm=(-5.0, 0.0, 5.0),
        drive="wheel_center",
        left_right_mode="symmetric",
        drive_wheels=True,
    )
    validate_case(document)
    assert document["k"]["rack_values_mm"] == [-5.0, 0.0, 5.0]
    assert document["k"]["axis_map"]["rack"] == "rack_drive"
    assert document["k"]["axis_map"]["wheel"] == ["wheel_drive_L", "wheel_drive_R"]


def test_c_case_document_is_a_valid_contract_member() -> None:
    assembly = build_front_axle(_compliant_model(), "C")
    document = case_document(
        assembly,
        family="kc_quasi_static",
        name="c-paths",
        paths=tuple(path.name for path in LoadPath.standard()),
        levels=11,
        maximum=1.0,
        side_mode="single",
        drive_wheels=False,
    )
    validate_case(document)
    assert document["c"]["paths"] == ["fx", "fy", "fz", "mx", "my", "mz"]
    assert document["c"]["load_marker"] == "wheel_center_L"


def test_the_case_document_carries_its_time_grid_and_solver() -> None:
    from suspension_multibody.native_kc.workflow import DEFAULT_SETTINGS, DEFAULT_TIMES

    assembly = build_front_axle(benchmark_model(), "K")
    document = case_document(
        assembly,
        family="kc_quasi_static",
        name="k-grid",
        wheel_values_mm=(0.0,),
        rack_values_mm=(0.0,),
        times_s=DEFAULT_TIMES,
        settings=DEFAULT_SETTINGS,
        drive_wheels=True,
    )
    validate_case(document)
    assert document["time"] == {
        "start_s": 0.0,
        "end_s": pytest.approx(DEFAULT_TIMES[-1]),
        "step_s": pytest.approx(DEFAULT_TIMES[1]),
    }
    assert document["solver"]["integrator"] == "ggl_generalized_alpha"
    assert document["solver"]["internal_step_s"] == pytest.approx(2.5e-4)


def test_an_unknown_family_is_rejected() -> None:
    assembly = build_front_axle(benchmark_model(), "K")
    with pytest.raises(NativeKcError):
        case_document(assembly, family="rally", name="nope")


def test_documents_round_trip_through_the_container_format() -> None:
    """The wire format has to carry a real axle document, not just fixtures."""
    assembly = build_front_axle(benchmark_model(), "K")
    model = model_document(assembly, name="benchmark-k", drive_wheels=True)
    case = case_document(
        assembly,
        family="kc_quasi_static",
        name="k-grid",
        wheel_values_mm=(-10.0, 0.0, 10.0),
        rack_values_mm=(-5.0, 0.0, 5.0),
        drive_wheels=True,
    )
    blob = b"\x00\x01\x02\x03"
    payload = pack_container({"model": model, "case": case}, blob)
    document, restored_blob = unpack_container(payload)
    assert document["model"] == model
    assert document["case"] == case
    assert restored_blob == blob
    # Canonical hashing is stable regardless of the dict insertion order.
    reordered = {key: document[key] for key in reversed(list(document))}
    assert contract_hash(reordered) == contract_hash(document)
