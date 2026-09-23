"""
Requirement 15: a single-axle assembly may have no steering at all.

The point of this test is not that the assembly *builds* without steering -- it is
that it builds the *exact subset*: the rack, the tie rods and their joints are
gone, and everything else is untouched, bit for bit.  A "degenerate rack body that
carries no constraint" would pass a weaker test and would be wrong.
"""

from __future__ import annotations

import numpy as np

from suspension_multibody.preparation.assembly import build_front_axle
from suspension_multibody.schema import FrontAxleModel, RigidBodySpec
from suspension_multibody.subsystems import DEFAULT_AXLE_SUBSYSTEMS, AssemblyRequest
from tests.benchmark_fixture import benchmark_model

#: Everything the steering subsystem owns, and nothing else.  The benchmark
#: fixture has `rack_fixed_to_chassis` false, so its guide row is `rack_guide`;
#: `rack_fixed_to_chassis` is the other branch of the same constraint.
ABSENT_BODIES = {"rack", "tie_rod_L", "tie_rod_R"}
ABSENT_CONSTRAINTS = {
    "rack_guide",
    "rack_tie_joint_L",
    "rack_tie_joint_R",
    "tie_upright_joint_L",
    "tie_upright_joint_R",
}
ABSENT_CONNECTIONS = {
    "rack_tie_joint_L",
    "rack_tie_joint_R",
    "tie_upright_joint_L",
    "tie_upright_joint_R",
}
ABSENT_POINTS = {
    "rack::center",
    "rack::tie_L",
    "rack::tie_R",
    "chassis::rack_center",
    "tie_rod_L::inner",
    "tie_rod_L::outer",
    "tie_rod_R::inner",
    "tie_rod_R::outer",
    "upright_L::tie_outer",
    "upright_R::tie_outer",
}
#: `RACK_CENTER` is the assembly's own synthesised copy; `rack_center` and its
#: per-side mirrors come from the model, and they are gone too because this test
#: removes the hardpoint -- steering was its only consumer.
ABSENT_HARDPOINTS = {
    "RACK_CENTER",
    "rack_center",
    "rack_center__L",
    "rack_center__R",
}


def _without_steering() -> FrontAxleModel:
    # `rack_center` is only required because steering needs it; dropping the
    # subsystem has to drop that requirement too, not leave a hardpoint nobody
    # resolves.
    model = benchmark_model()
    hardpoints = {
        name: point
        for name, point in model.hardpoints.items()
        if name != "rack_center"
    }
    return model.model_copy(update={"hardpoints": hardpoints})


def _request(mode: str) -> AssemblyRequest:
    return AssemblyRequest(mode=mode, subsystems=DEFAULT_AXLE_SUBSYSTEMS - {"steering"})


def _both(mode: str):
    return build_front_axle(benchmark_model(), mode), build_front_axle(
        _without_steering(), mode, _request(mode)
    )


def test_the_exact_steering_content_disappears() -> None:
    full, reduced = _both("K")

    assert set(full.bodies) - set(reduced.bodies) == ABSENT_BODIES
    assert set(reduced.bodies) - set(full.bodies) == set()
    assert {c.name for c in full.constraints} - {
        c.name for c in reduced.constraints
    } == ABSENT_CONSTRAINTS
    assert {c.name for c in reduced.constraints} - {
        c.name for c in full.constraints
    } == set()
    assert {c.name for c in full.ideal_constraints} - {
        c.name for c in reduced.ideal_constraints
    } == ABSENT_CONSTRAINTS
    assert {c.name for c in full.connections} - {
        c.name for c in reduced.connections
    } == ABSENT_CONNECTIONS
    reduced_points = {f"{body}::{label}" for body, label in reduced.points}
    full_points = {f"{body}::{label}" for body, label in full.points}
    assert full_points - reduced_points == ABSENT_POINTS
    assert reduced_points - full_points == set()
    assert set(full.hardpoints) - set(reduced.hardpoints) == ABSENT_HARDPOINTS
    assert set(reduced.hardpoints) - set(full.hardpoints) == set()
    # Steering owns no elements, so the element lists must be identical.
    assert list(reduced.element_ids) == list(full.element_ids)
    assert [b.name for b in reduced.bushings] == [b.name for b in full.bushings]


def test_every_remaining_point_is_bit_identical() -> None:
    full, reduced = _both("K")
    for key, point in reduced.points.items():
        assert np.array_equal(np.asarray(point), np.asarray(full.points[key])), key


def test_the_remaining_rows_keep_their_order_and_identity() -> None:
    """
    A subset must be the same rows, not merely the same names.

    Slice the full constraint list down to the names the reduced assembly kept
    and require the result to match the reduced list exactly: same order, same
    types, same connection rows.
    """
    full, reduced = _both("K")
    kept = {c.name for c in reduced.constraints}
    assert [
        (c.name, type(c).__name__) for c in full.constraints if c.name in kept
    ] == [(c.name, type(c).__name__) for c in reduced.constraints]
    kept_connections = {c.name for c in reduced.connections}
    assert [c for c in full.connections if c.name in kept_connections] == list(
        reduced.connections
    )


def test_no_degenerate_rack_body_is_left_behind() -> None:
    _, reduced = _both("K")
    assert "rack" not in reduced.bodies


def test_a_declared_steering_body_spec_is_tolerated_not_rejected() -> None:
    """
    Dropping the subsystem must not make the *model* invalid.

    A model that still declares a rack or tie rod body spec is describing bodies
    this assembly chose not to carry; that is not an error, and it must not blow
    up while the mass table is applied.
    """
    model = _without_steering()
    with_specs = model.model_copy(
        update={
            "bodies": (
                *model.bodies,
                RigidBodySpec(name="rack", mass=1.0),
                RigidBodySpec(name="tie_rod_L", mass=1.0),
                RigidBodySpec(name="tie_rod_R", mass=1.0),
            )
        }
    )
    reduced = build_front_axle(with_specs, "K", _request("K"))
    assert "rack" not in reduced.bodies
    assert set(reduced.bodies) == set(_both("K")[1].bodies)


def test_c_mode_also_works_without_steering() -> None:
    full, reduced = _both("C")
    assert "rack" not in reduced.bodies
    # The eight inboard placeholders are the suspension's, not steering's, so they
    # stay, in the same order: 2 arms x 2 inboard points x 2 sides.
    assert list(reduced.element_ids) == list(full.element_ids)
    assert len(reduced.elements) == 8


def test_an_axle_cannot_claim_brake_or_drive() -> None:
    """
    Requirement 17 / D8: a single-axle assembly has no brake and no drive.

    The assembly cannot build them, so accepting the role and then reporting it in
    `capabilities` would be a lie a rig would plan against.  It refuses instead.
    """
    for role in ("brake", "drive"):
        request = AssemblyRequest(
            mode="K", subsystems=DEFAULT_AXLE_SUBSYSTEMS | {role}
        )
        try:
            build_front_axle(benchmark_model(), "K", request)
        except ValueError as error:
            assert role in str(error)
        else:  # pragma: no cover - the call must raise
            raise AssertionError(f"an axle assembly must refuse the {role} role")
