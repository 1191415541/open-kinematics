"""The unified SI compiler emits valid K/C model and analysis contracts."""

import numpy as np
import pytest
from suspension_contracts import (
    contract_hash,
    pack_container,
    unpack_container,
    validate_case,
    validate_model,
)

from suspension_multibody.api import validate
from suspension_multibody.authoring import assemble_generic
from suspension_multibody.authoring.migration import migrate_v1_kc_case
from suspension_multibody.cases.kc_quasi_static.settings import (
    DEFAULT_SETTINGS,
    DEFAULT_TIMES,
)
from tests.benchmark_fixture import benchmark_model

from .kc_fixtures import _compliant_model


def _compiled(mode="K", **kwargs):
    model = benchmark_model() if mode == "K" else _compliant_model()
    return validate(*migrate_v1_kc_case(model, mode=mode, **kwargs))


def test_ideal_model_document_is_a_valid_contract_member():
    assembly, case = migrate_v1_kc_case(benchmark_model(), mode="K", wheel_values_mm=(0.,))
    graph = assemble_generic(assembly)
    document = validate(assembly, case).model_document
    validate_model(document)
    assert document["units"]["length"] == "m"
    assert len(document["bodies"]) == len(graph.bodies)
    expected = {row["name"] for row in graph.joints}
    assert expected <= {row["name"] for row in document["joints"]}
    driven = [row for row in document["joints"] if row["type"].startswith("driven_") and ".spin_" not in row["name"]]
    assert len([row for row in document["joints"] if ".spin_" in row["name"]]) == 2
    assert [row["target"].rsplit(".", 1)[-1] for row in driven] == ["wheel_drive_L", "wheel_drive_R", "rack_drive"]
    assert {"wheel_center_L", "wheel_center_R"} <= {row["name"].rsplit(".", 1)[-1] for row in document["markers"]}


def test_compliant_model_document_carries_its_bushings():
    assembly, case = migrate_v1_kc_case(_compliant_model(), mode="C", paths=("fz",))
    graph = assemble_generic(assembly)
    document = validate(assembly, case).model_document
    validate_model(document)
    bushings = [row for row in document["elements"] if row["type"] == "bushing"]
    assert len(bushings) == len([row for row in graph.elements if row["type"] == "bushing"])
    assert len(bushings[0]["parameters"]["stiffness"]) == 6
    assert {row["name"] for row in graph.joints} <= {row["name"] for row in document["joints"]}
    assert [row["target"].rsplit(".", 1)[-1] for row in document["joints"] if row["type"].startswith("driven_") and ".spin_" not in row["name"]] == ["rack_neutral"]


def test_joint_points_are_body_local():
    assembly, case = migrate_v1_kc_case(benchmark_model(), mode="K", wheel_values_mm=(0.,))
    graph = assemble_generic(assembly)
    document = validate(assembly, case).model_document
    first = graph.joints[0]
    emitted = next(row for row in document["joints"] if row["name"] == first["name"])
    body = graph.bodies[first["body_a"]]
    world = body.pose.transform_point(np.asarray(first["point_a"]))
    np.testing.assert_allclose(body.pose.inverse().transform_point(world), emitted["point_a"], atol=1e-15)


def test_k_case_document_is_a_valid_contract_member():
    document = _compiled(wheel_values_mm=(-10., 0., 10.), rack_values_mm=(-5., 0., 5.)).case_document
    validate_case(document)
    assert document["k"]["rack_values_mm"] == [-5., 0., 5.]
    assert document["k"]["axis_map"]["rack"] == "kc_rig.sub.json.rack_drive"
    assert document["k"]["axis_map"]["wheel"] == ["kc_rig.sub.json.wheel_drive_L", "kc_rig.sub.json.wheel_drive_R"]


def test_c_case_document_is_a_valid_contract_member():
    document = _compiled("C", paths=("fx", "fy", "fz", "mx", "my", "mz")).case_document
    validate_case(document)
    assert document["c"]["paths"] == ["fx", "fy", "fz", "mx", "my", "mz"]
    assert document["c"]["load_marker"].endswith(".wheel_center_L")


def test_the_case_document_carries_its_time_grid_and_solver():
    document = _compiled(wheel_values_mm=(0.,), times_s=DEFAULT_TIMES, settings=DEFAULT_SETTINGS).case_document
    validate_case(document)
    assert document["time"] == {"start_s": 0., "end_s": pytest.approx(DEFAULT_TIMES[-1]), "step_s": pytest.approx(DEFAULT_TIMES[1])}
    assert document["solver"]["integrator"] == "ggl_generalized_alpha"
    assert document["solver"]["internal_step_s"] == pytest.approx(2.5e-4)


def test_an_unknown_family_is_rejected():
    assembly, case = migrate_v1_kc_case(benchmark_model(), mode="K")
    data = case.to_payload()
    data["protocol"] = "rally"
    with pytest.raises(Exception, match="rally"):
        validate(assembly, data)


def test_documents_round_trip_through_the_container_format():
    compiled = _compiled(wheel_values_mm=(-10., 0., 10.), rack_values_mm=(-5., 0., 5.))
    model, case = compiled.model_document, compiled.case_document
    blob = b"\x00\x01\x02\x03"
    document, restored_blob = unpack_container(pack_container({"model": model, "case": case}, blob))
    assert document["model"] == model
    assert document["case"] == case
    assert restored_blob == blob
    reordered = {key: document[key] for key in reversed(list(document))}
    assert contract_hash(reordered) == contract_hash(document)
