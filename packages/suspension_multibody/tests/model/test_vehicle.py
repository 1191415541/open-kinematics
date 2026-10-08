"""Vehicle declarations become ordinary subsystem graphs with physical welds."""

import numpy as np
import pytest

from suspension_multibody.authoring import assemble_generic, migrate_v1_vehicle
from suspension_multibody.schema import Vec3
from suspension_multibody.templates import RACK_HOUSING_MASS, WHEEL_HUB_MASS
from tests.vehicle.vehicle_fixtures import _vehicle


def test_vehicle_has_two_declared_axles_four_wheels_and_one_body():
    graph = assemble_generic(migrate_v1_vehicle(_vehicle()))
    assert len(graph.bodies) == 29
    assert "body.chassis" in graph.bodies
    assert sum(name.endswith(".chassis") for name in graph.bodies) == 1
    assert len(graph.tires) == 4
    assert len([row for row in graph.joints if "wheel_spin" in row["name"]]) == 4
    assert {row["body"] for row in graph.tires} == {"wheel_" + wheel.name + "." + wheel.body for wheel in _vehicle().wheels}


def test_vehicle_preserves_chassis_and_all_declared_mass():
    source = _vehicle()
    graph = assemble_generic(migrate_v1_vehicle(source))
    expected = source.chassis.mass + sum(row.mass for row in (*source.front_axle.bodies, *source.rear_axle.bodies, *source.wheels)) + 4 * WHEEL_HUB_MASS + 2 * RACK_HOUSING_MASS
    assert graph.bodies["body.chassis"].mass == source.chassis.mass
    assert sum(row.mass for row in graph.bodies.values()) == pytest.approx(expected, abs=1e-9)
    assert all(np.asarray(row["parameters"]["center_local"]).shape == (3,) for row in graph.tires)


def test_explicit_fixed_mount_keeps_wheel_inertia_and_native_joint_visible():
    source = _vehicle()
    mount = source.wheels[0].model_copy(update={"mount_joint_kind": "fixed", "center_local": Vec3(x=35, y=-8, z=12),
        "inertia": ((4, .2, .1), (.2, 5, .3), (.1, .3, 6))})
    graph = assemble_generic(migrate_v1_vehicle(source.model_copy(update={"wheels": (mount, *source.wheels[1:])})))
    body = graph.bodies["wheel_front_left." + mount.body]
    np.testing.assert_allclose(body.inertia, np.asarray(mount.inertia)*1e-6, atol=1e-12)
    joint = next(row for row in graph.joints if row["name"] == "wheel_front_left.mount")
    assert joint["type"] == "fixed"
    assert body.mass == mount.mass
    a = graph.bodies[joint["body_a"]].pose.transform_point(np.asarray(joint["point_a"]))
    b = body.pose.transform_point(np.asarray(joint["point_b"]))
    np.testing.assert_allclose(a, b, atol=1e-9)
