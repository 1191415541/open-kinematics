"""The frozen historical entities and physical endpoints survive offline migration."""

import json
from pathlib import Path

import numpy as np
import pytest

from suspension_multibody.authoring import assemble_generic, migrate_v1_axle
from tests.benchmark_fixture import benchmark_model

SNAPSHOT = Path(__file__).parents[4] / ".codex-tasks/20260922-suspension-template-architecture/tasks/20260922-04-subsystems/raw/assembly_snapshot.json"
_SNAPSHOTS = json.loads(SNAPSHOT.read_text(encoding="utf-8"))
_TYPES = {"BallJoint": "spherical", "RevoluteJoint": "revolute", "PrismaticJoint": "prismatic", "WeldJoint": "fixed"}


def local(name):
    value = name.rsplit(".", 1)[-1]
    return "ground" if value == "chassis" else value


def combination(expected):
    model = benchmark_model().model_copy(update={"rack_fixed_to_chassis": expected["rack_fixed_to_chassis"]})
    source = migrate_v1_axle(model, mode=expected["mode"])
    return source, assemble_generic(source)


@pytest.mark.parametrize("key", list(_SNAPSHOTS))
def test_body_identity_fixedness_and_rotation_match_the_snapshot(key):
    expected = _SNAPSHOTS[key]
    source, built = combination(expected)
    assert {local(name) for name in built.bodies} == set(expected["bodies"])
    for name, body in built.bodies.items():
        assert body.fixed == (local(name) == "ground")
        np.testing.assert_array_equal(body.pose.rotation, np.eye(3))
    # The native order is now the explicit subsystem declaration order.
    assert list(built.bodies) == [entry.ref + "." + row["name"] for entry in source.entries
                                  for row in entry.subsystem.template.payload["bodies"]]


@pytest.mark.parametrize("key", list(_SNAPSHOTS))
def test_active_constraints_and_force_ids_match_the_frozen_columns(key):
    expected = _SNAPSHOTS[key]
    _, built = combination(expected)
    assert {local(row["name"]): row["type"] for row in built.joints} == {
        row["name"]: _TYPES[row["type"]] for row in expected["constraints"]}
    assert [local(row["name"]) for row in built.elements] == expected["elements"]
    assert [local(row["name"]) for row in built.elements if row["type"] == "bushing"] == expected["bushings"]


@pytest.mark.parametrize("key", list(_SNAPSHOTS))
def test_every_frozen_connection_keeps_its_physical_world_point(key):
    expected = _SNAPSHOTS[key]
    _, built = combination(expected)
    joints = {local(row["name"]): row for row in built.joints}
    elements = {local(row["name"]): row for row in built.elements}
    for connection in expected["connections"]:
        name = connection["name"]
        if connection["kind"] == "bushing":
            name = name.replace("uca_mount_", "uca_bushing_").replace("lca_mount_", "lca_bushing_")
        row = joints.get(name, elements.get(name))
        if expected["mode"] == "K" and name.endswith("inner_rear"):
            assert row is None
            continue
        if name.startswith("wheel_center_"):
            assert row is None
            continue
        assert row is not None, name
        for end in ("a", "b"):
            assert local(row["body_" + end]) == connection["body_" + end]
            point = row.get("parameters", row)["point_" + end]
            world = built.bodies[row["body_" + end]].pose.transform_point(np.asarray(point))
            frozen = expected["points"][connection["body_" + end] + "::" + connection["point_" + end]]
            np.testing.assert_array_equal(world, np.asarray(frozen) * .001)
        if "axis_a" in row:
            assert abs(np.linalg.norm(row["axis_a"]) - 1) < 1e-9


@pytest.mark.parametrize("key", list(_SNAPSHOTS))
def test_frozen_locator_geometry_is_present_in_the_declarations(key):
    expected = _SNAPSHOTS[key]
    source, built = combination(expected)
    by_body = {}
    for entry in source.entries:
        for row in entry.subsystem.template.payload["hardpoints"]:
            owner = entry.ref + "." + row.get("owner", "")
            if owner not in built.bodies:
                continue
            world = built.bodies[owner].pose.transform_point(np.asarray(entry.subsystem.payload["hardpoints"][row["name"]]) * .001)
            by_body.setdefault(local(owner), []).append(world)
    for row in (*built.joints, *built.elements):
        for end in ("a", "b"):
            owner = row["body_"+end]
            point = row.get("parameters", row)["point_"+end]
            by_body.setdefault(local(owner), []).append(built.bodies[owner].pose.transform_point(np.asarray(point)))
    for name, value in expected["points"].items():
        body = name.split("::")[0]
        assert any(np.array_equal(point, np.asarray(value) * .001) for point in by_body[body]), name
    all_points = [point for points in by_body.values() for point in points]
    for name, value in expected["hardpoints"].items():
        assert any(np.array_equal(point, np.asarray(value) * .001) for point in all_points), name


def test_c_placeholders_remain_eight_zero_stiffness_mounts():
    _, built = combination(_SNAPSHOTS["C_rack_fixed_false"])
    assert len(built.elements) == 8
    for row in built.elements:
        assert row["type"] == "bushing"
        np.testing.assert_array_equal(row["parameters"]["stiffness"], np.zeros((6, 6)))
        assert local(row["body_a"]) == "ground"
        assert local(row["body_b"]).startswith(("upper_arm_", "lower_arm_"))
