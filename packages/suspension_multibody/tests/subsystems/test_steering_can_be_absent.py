"""An omitted steering declaration leaves no hidden body or inferred guide."""

import numpy as np
import pytest

from suspension_multibody.authoring import assemble_generic
from suspension_multibody.presets import generic_template
from tests.authoring.test_unified_subsystem_templates import (
    assembly,
    carrier_subsystem,
    subsystem,
)


def steering_pair(mode="K"):
    points = {name: [0, 0, .334] for name in generic_template("suspension").hardpoint_names}
    points.update(upper_rear=[1, 0, .334], lower_rear=[1, 0, .334])
    common = {"support": carrier_subsystem(), "suspension": subsystem(generic_template("suspension"), points)}
    steering = subsystem(generic_template("steering"), {"center": [0, 0, .334], "housing_center": [0, 0, .334]})
    return tuple(assemble_generic(assembly(rows, mode=mode, pairings={"suspension": {"rack": "steering.rack" if "steering" in rows else "support.rack"}}))
                 for rows in ({**common, "steering": steering}, common))


@pytest.mark.parametrize("mode", ["K", "C"])
def test_the_steering_subset_preserves_all_other_physics(mode):
    full, reduced = steering_pair(mode)
    assert set(full.bodies) - set(reduced.bodies) == {"steering.rack", "steering.housing"}
    assert not set(reduced.bodies) - set(full.bodies)
    unchanged = tuple(row for row in full.joints if not row["name"].startswith("steering.") and "tie_inner" not in row["name"])
    assert unchanged == tuple(row for row in reduced.joints if "tie_inner" not in row["name"])
    for original, changed in zip((row for row in full.joints if "tie_inner" in row["name"]),
                                 (row for row in reduced.joints if "tie_inner" in row["name"])):
        assert original["body_a"] == "steering.rack" and changed["body_a"] == "support.carrier"
        assert original["body_b"] == changed["body_b"]
        for end in ("a", "b"):
            np.testing.assert_array_equal(full.bodies[original["body_"+end]].pose.transform_point(np.asarray(original["point_"+end])),
                reduced.bodies[changed["body_"+end]].pose.transform_point(np.asarray(changed["point_"+end])))
    assert full.elements == reduced.elements and full.tires == reduced.tires
    for key in reduced.bodies:
        a, b = full.bodies[key], reduced.bodies[key]
        assert a.mass == b.mass and a.fixed == b.fixed
        np.testing.assert_array_equal(a.pose.quaternion, b.pose.quaternion)
        np.testing.assert_array_equal(a.pose.translation, b.pose.translation)
        np.testing.assert_array_equal(a.inertia, b.inertia)
    assert len(reduced.elements) == (4 if mode == "K" else 12)
    assert not any("rack" in name for name in reduced.bodies)
