"""File and memory authors produce a complete, deterministic SI graph."""

import numpy as np
import pytest
from suspension_contracts import ContractError

from suspension_multibody.authoring import AssemblyDocument, assemble_generic
from suspension_multibody.authoring.errors import AuthoringError
from suspension_multibody.authoring.migration import save_migrated_assembly
from suspension_multibody.modeling.resolved import ResolvedModel
from suspension_multibody.schema import Vec3
from tests.benchmark_fixture import benchmark_model

from ._generic import axle_source, reordered, resolved


@pytest.mark.parametrize("mode", ["K", "C"])
def test_file_and_memory_preserve_body_and_frame_order(tmp_path, mode):
    source = axle_source(mode)
    loaded = AssemblyDocument.load(save_migrated_assembly(source, tmp_path))
    before, after = [resolved(row).to_document() for row in (source, loaded)]
    for section in ("bodies", "frames", "joints", "elements", "tires"):
        assert before[section] == after[section]
    assert resolved(source).fingerprint == resolved(loaded).fingerprint


@pytest.mark.parametrize("mode,expected", [("K", 16), ("C", 12)])
def test_the_active_constraint_count_is_preserved(mode, expected):
    assert len(resolved(axle_source(mode)).to_document()["joints"]) == expected


def test_fingerprint_tracks_geometry_and_is_stable():
    model = benchmark_model()
    first = resolved(axle_source(model=model))
    assert resolved(axle_source(model=model)).fingerprint == first.fingerprint
    points = {**model.hardpoints, "wheel_center": Vec3(x=5, y=-700, z=300)}
    changed = model.model_copy(update={"hardpoints": points})
    assert resolved(axle_source(model=changed)).fingerprint != first.fingerprint


def test_fingerprint_ignores_collection_order():
    source = axle_source()
    assert resolved(reordered(source)).fingerprint == resolved(source).fingerprint


def test_a_duplicate_instance_is_refused():
    source = axle_source()
    payload = source.to_payload()
    payload["subsystems"].append(payload["subsystems"][0])
    with pytest.raises(AuthoringError, match="repeats a subsystem reference"):
        resolved(AssemblyDocument.from_payload(payload, subsystems={row.ref: row.subsystem for row in source.entries}))


def test_an_empty_assembly_is_refused():
    with pytest.raises(AuthoringError):
        AssemblyDocument.from_payload({"document": "assembly", "schema_version": 1, "name": "empty",
            "assembly_kind": "generic_multibody", "subsystems": []}, subsystems={})


def test_a_dangling_frame_is_refused():
    graph = resolved(axle_source()).to_document()
    graph["frames"][0]["body"] = "ghost"
    with pytest.raises(ContractError, match="ghost"):
        ResolvedModel(graph)


def test_every_joint_endpoint_is_declared_and_coincident():
    built = assemble_generic(axle_source())
    graph = built.resolved_model().to_document()
    for row in graph["joints"]:
        a, b = [built.bodies[row["body_"+end]].pose.transform_point(np.asarray(row["point_"+end])) for end in ("a", "b")]
        np.testing.assert_allclose(a, b, atol=1e-9, rtol=0)
        if "axis_a" in row:
            assert abs(np.linalg.norm(row["axis_a"])-1) < 1e-9
