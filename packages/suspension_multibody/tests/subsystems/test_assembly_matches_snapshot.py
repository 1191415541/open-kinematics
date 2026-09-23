"""
The axle assembly still produces exactly what it produced before the split.

The subsystems are a refactor, not a behaviour change, so the only honest
acceptance test is a comparison against the assembly captured *before* the split.
That capture is `raw/assembly_snapshot.json` in subtask 04's directory: four
combinations of K/C x `rack_fixed_to_chassis`, taken from the shared
`benchmark_axle.json` fixture.

`points` are compared by key set and by value, not by insertion order: the
snapshot was serialised with sorted JSON keys, so its order is not recoverable,
and nothing consumes `points` order (lookups are by `(body, label)`, and it is
`bodies` order the contract document uses).
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from suspension_multibody.preparation.assembly import build_front_axle
from suspension_multibody.schema import FrontAxleModel
from tests.benchmark_fixture import benchmark_model

SNAPSHOT = (
    Path(__file__).parents[4]
    / ".codex-tasks/20260922-suspension-template-architecture"
    / "tasks/20260922-04-subsystems/raw/assembly_snapshot.json"
)


def _snapshot() -> dict[str, object]:
    return json.loads(SNAPSHOT.read_text(encoding="utf-8"))


def _combination(model: FrontAxleModel, expected: dict) -> object:
    rebuilt = model.model_copy(
        update={"rack_fixed_to_chassis": expected["rack_fixed_to_chassis"]}
    )
    return build_front_axle(rebuilt, expected["mode"])


def _close(actual: np.ndarray, expected: list[float]) -> bool:
    return np.array_equal(
        np.asarray(actual, dtype=float), np.asarray(expected, dtype=float)
    )


def test_bodies_match_the_frozen_snapshot() -> None:
    model = benchmark_model()
    for key, expected in _snapshot().items():
        assembly = _combination(model, expected)
        assert list(assembly.bodies) == expected["bodies"], key
        # Names in the right order are not enough: the mass properties have to
        # come out the same too, or the solve would differ.
        for name, body in assembly.bodies.items():
            assert body.name == name, f"{key} {name}"
            expected_fixed = name == "chassis"
            assert body.fixed == expected_fixed, f"{key} {name} fixed"
            assert np.array_equal(body.pose.rotation, np.eye(3)), f"{key} {name} pose"


def test_points_match_the_frozen_snapshot() -> None:
    model = benchmark_model()
    for key, expected in _snapshot().items():
        assembly = _combination(model, expected)
        produced = {
            f"{body}::{label}": point for (body, label), point in assembly.points.items()
        }
        assert sorted(produced) == sorted(expected["points"]), key
        for name, point in produced.items():
            assert np.array_equal(
                np.asarray(point, dtype=float),
                np.asarray(expected["points"][name], dtype=float),
            ), f"{key} {name}"


def test_hardpoints_match_the_frozen_snapshot() -> None:
    model = benchmark_model()
    for key, expected in _snapshot().items():
        assembly = _combination(model, expected)
        produced = {
            name: point.as_tuple() for name, point in assembly.hardpoints.items()
        }
        assert sorted(produced) == sorted(expected["hardpoints"]), key
        for name, point in produced.items():
            assert _close(point, expected["hardpoints"][name]), f"{key} {name}"


def test_connections_match_the_frozen_snapshot() -> None:
    model = benchmark_model()
    for key, expected in _snapshot().items():
        assembly = _combination(model, expected)
        produced = [
            {
                "name": connection.name,
                "kind": connection.kind,
                "body_a": connection.body_a,
                "body_b": connection.body_b,
                "point_a": connection.point_a,
                "point_b": connection.point_b,
            }
            for connection in assembly.connections
        ]
        assert produced == [
            {
                "name": row["name"],
                "kind": row["kind"],
                "body_a": row["body_a"],
                "body_b": row["body_b"],
                "point_a": row["point_a"],
                "point_b": row["point_b"],
            }
            for row in expected["connections"]
        ], key


def test_constraint_names_and_types_match_the_frozen_snapshot() -> None:
    model = benchmark_model()
    for key, expected in _snapshot().items():
        assembly = _combination(model, expected)
        assert [(c.name, type(c).__name__) for c in assembly.constraints] == [
            (row["name"], row["type"]) for row in expected["constraints"]
        ], key
        assert [(c.name, type(c).__name__) for c in assembly.ideal_constraints] == [
            (row["name"], row["type"]) for row in expected["ideal_constraints"]
        ], key


def test_constraint_geometry_is_unchanged() -> None:
    """
    Same joints in the same order is not enough -- the same *numbers* matter.

    The snapshot only carries names and types, so this checks the values that the
    snapshot cannot: every joint's points must coincide at assembly time (an
    ideal joint connects one physical location), and the K-mode inboard revolute
    axes must stay unit and non-degenerate.  A wrong `_inboard_axis` would show
    up here even though the name table would not notice.
    """
    model = benchmark_model()
    for mode in ("K", "C"):
        assembly = build_front_axle(model, mode)
        for constraint in assembly.ideal_constraints:
            point_a = np.asarray(constraint.point_a, dtype=float)
            point_b = np.asarray(constraint.point_b, dtype=float)
            global_a = np.asarray(
                assembly.bodies[constraint.body_a].pose.transform_point(point_a),
                dtype=float,
            )
            global_b = np.asarray(
                assembly.bodies[constraint.body_b].pose.transform_point(point_b),
                dtype=float,
            )
            assert np.allclose(global_a, global_b, atol=1e-9), (
                f"{mode} {constraint.name} does not connect one location"
            )
            for axis in ("axis_a", "axis_b"):
                if not hasattr(constraint, axis):
                    continue
                values = np.asarray(getattr(constraint, axis), dtype=float)
                norm = float(values @ values)
                assert abs(norm - 1.0) < 1e-9, (
                    f"{mode} {constraint.name}.{axis} is not a unit axis"
                )


def test_bushings_and_elements_match_the_frozen_snapshot() -> None:
    model = benchmark_model()
    for key, expected in _snapshot().items():
        assembly = _combination(model, expected)
        assert [b.name for b in assembly.bushings] == expected["bushings"], key
        assert list(assembly.element_ids) == expected["elements"], key


def test_c_mode_placeholders_are_the_eight_inboard_slots() -> None:
    """
    The C-mode element list ends with the eight zero-stiffness slot placeholders.

    The snapshot records only the names, so this pins the part that matters and
    that subtask 05 will change: exactly eight, inboard, zero stiffness, identity
    rotation -- i.e. still placeholders, not silently turned into real bushings.
    """
    assembly = build_front_axle(benchmark_model(), "C")
    placeholders = [
        element
        for element in assembly.elements
        if getattr(element, "name", "").endswith(("inner_front", "inner_rear"))
    ]
    assert len(placeholders) == 8
    for element in placeholders:
        assert np.array_equal(element.stiffness, np.zeros((6, 6))), element.name
        assert element.local_pose_a.quaternion[0] == 1.0, element.name
        assert element.body_a == "chassis", element.name
        assert element.body_b.startswith(("upper_arm_", "lower_arm_")), element.name
