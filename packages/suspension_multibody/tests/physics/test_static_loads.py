"""
Static equilibrium over however many contact points the assembly has.

Two things are being asserted, and they are different questions:

* **does a solution exist** -- three equilibrium equations in ``N`` unknowns have
  one exactly when the loads are compatible with the contact geometry, which is
  what the residual of the least-squares solution measures.  A vehicle on four
  wheels, a truck on six and a single corner on one are all judged the same way;
* **is it unique** -- ``rank(A) == N``.  Four wheels are three independent
  equations against four unknowns, so the answer is the minimum-norm member of a
  family and says so; one contact point directly under the centre of mass is
  ``rank(A) = 1 = N`` and is therefore *not* marked as a family.

The single-contact-point pair of cases is the one that matters most: the same
geometry, the same rank-one matrix, and two different answers, because the rank
of the matrix does not change when the loads do.  What separates them is whether
the loads can be balanced at all, and the module reports the measured residual,
the tolerance and the contact-point count when they cannot -- never the rank,
which would say nothing.

The four-wheel case is the production path (``vehicle/service.py``), and its
minimum-norm answer is pinned here as it was before: this file adds the N-point
cases beside that behaviour rather than replacing it.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.schema import (
    FrontAxleModel,
    MassSpec,
    RigidBodySpec,
    SteeringSystemSpec,
    TireModelSpec,
    Vec3,
    VehicleModel,
    WheelSpec,
)
from suspension_multibody.subsystems.assembler import AxleEntry, compose_entries_runtime
from suspension_multibody.subsystems.types import (
    DEFAULT_VEHICLE_SUBSYSTEMS,
    AssemblyRequest,
)
from suspension_multibody.subsystems.vehicle_parts import _body_from_spec
from suspension_multibody.vehicle.static_loads import (
    IncompatibleStaticLoadsError,
    compute_static_wheel_loads,
    compute_static_wheel_loads_for_assembly,
)

#: The role set a vehicle carries: brake and drive included, chassis owned.
_VEHICLE_REQUEST = AssemblyRequest(mode="K", subsystems=DEFAULT_VEHICLE_SUBSYSTEMS)

#: The four corners a two-axle ``VehicleModel`` declares.
_CORNERS = ("front_left", "front_right", "rear_left", "rear_right")


def _axle(name: str, x: float, *, side_y: float = -750.0) -> FrontAxleModel:
    """
    Return one double-wishbone axle at ``x``, on the ``side_y`` side.

    ``side_y = 0`` puts every part of the axle on the vehicle centreline, which is
    the geometry a single-contact-point case needs: a corner bench whose mass is
    over its own wheel.
    """
    return FrontAxleModel(
        name=name,
        hardpoints={
            "UPPER_INBOARD_FRONT": Vec3(x=x, y=side_y + 250.0, z=500.0),
            "UPPER_INBOARD_REAR": Vec3(x=x + 150.0, y=side_y + 250.0, z=500.0),
            "UPPER_OUTBOARD": Vec3(x=x, y=side_y, z=350.0),
            "LOWER_INBOARD_FRONT": Vec3(x=x, y=side_y + 250.0, z=100.0),
            "LOWER_INBOARD_REAR": Vec3(x=x + 150.0, y=side_y + 250.0, z=100.0),
            "LOWER_OUTBOARD": Vec3(x=x, y=side_y, z=100.0),
            "TIE_ROD_INBOARD": Vec3(x=x, y=side_y + 300.0, z=250.0),
            "TIE_ROD_OUTBOARD": Vec3(x=x, y=side_y, z=250.0),
            "WHEEL_CENTER": Vec3(x=x, y=side_y, z=300.0),
            "RACK_CENTER": Vec3(x=x, y=0.0, z=250.0),
        },
        mass=MassSpec(sprung_mass=600.0),
        bodies=tuple(
            RigidBodySpec(name=body, mass=10.0)
            for body in (
                "rack", "upper_arm_L", "upper_arm_R", "lower_arm_L", "lower_arm_R",
                "upright_L", "upright_R", "tie_rod_L", "tie_rod_R",
            )
        ),
    )


def _wheel(name: str) -> WheelSpec:
    return WheelSpec(
        name=name,
        body=f"wheel_{name}",
        center_local=Vec3(),
        mass=20.0,
        axial_inertia=2.0,
        tire=TireModelSpec(kind="fiala", vertical_stiffness=20.0),
    )


def _entry(placement: str, axle: FrontAxleModel, wheels: tuple[WheelSpec, ...], sides=None):
    return AxleEntry(
        placement=placement,
        prefix=f"{placement}_",
        axle=axle,
        replace_bodies={},
        wheels=wheels,
        sides=sides,
    )


def _assemble(entries, *, chassis_mass: float = 1200.0):
    """Assemble the entries under one chassis of the given mass."""
    return compose_entries_runtime(
        tuple(entries),
        chassis_name="chassis",
        mode="K",
        request=_VEHICLE_REQUEST,
        chassis_body=_body_from_spec(RigidBodySpec(name="chassis", mass=chassis_mass)),
    )


def _three_axle_assembly():
    """
    Return a three-axle vehicle: six wheel ends, and therefore six contact points.

    An entry list is how a vehicle with three axles is stated -- the
    ``VehicleModel`` shape names exactly two -- so this is also the case that shows
    the contact points come from the assembly rather than from a corner-name tuple.
    """
    return _assemble(
        _entry(placement, _axle(placement, x), (_wheel(f"{placement}_left"), _wheel(f"{placement}_right")))
        for placement, x in (("front", 1400.0), ("middle", 0.0), ("rear", -1400.0))
    )


def _corner_assembly(*, side_y: float = 0.0):
    """
    Return a one-wheel bench: one suspension, one wheel end and one contact point.

    ``side_y = 0`` states the axle on the vehicle centreline, so the bench's mass
    sits over its own contact point -- the geometry the compatible case needs.
    """
    return _assemble(
        [
            _entry(
                "corner",
                _axle("corner", 0.0, side_y=side_y),
                (_wheel("single_left"),),
                sides=("L",),
            )
        ]
    )


def _equilibrium_rows(result):
    """
    Rebuild the three balance rows from the result's own reported numbers.

    Returned as ``(force, pitch, roll)`` residuals of ``A x - b``.  Rebuilding them
    here rather than reading a residual off the result is what makes the tolerance
    assertion independent of the solve: the numbers come from the loads, the
    contact points and the centre of mass the result publishes, and nothing else.
    """
    names = tuple(result.support_points)
    matrix = np.array(
        [
            np.ones(len(names)),
            [result.support_points[name][0] - result.center_of_mass[0] for name in names],
            [result.support_points[name][1] - result.center_of_mass[1] for name in names],
        ],
        dtype=float,
    )
    loads = np.array([result.wheel_loads[name] for name in names], dtype=float)
    # The static state: gravity only, which is what these cases solve for.
    rhs = np.array(
        [
            result.total_mass * 9810.0,
            0.0,
            0.0,
        ],
        dtype=float,
    )
    return np.abs(matrix @ loads - rhs)


def test_three_axle_six_contact_points_balance() -> None:
    """
    Six contact points solve, and the six loads carry the weight.

    The contact points are the six wheel ends the assembly declares, one per
    corner of three axles -- not four corners of a car with two extra names.
    Three independent equations cannot pin six unknowns, so the answer is the
    minimum-norm member of the family and is marked as such.
    """
    assembly = _three_axle_assembly()

    result = compute_static_wheel_loads_for_assembly(assembly)

    assert len(result.support_points) == 6
    assert set(result.support_points) == {
        "front_left", "front_right",
        "middle_left", "middle_right",
        "rear_left", "rear_right",
    }
    assert set(result.wheel_loads) == set(result.support_points)
    assert result.rank == 3
    # Three independent equations against six unknowns: a family, not a point.
    assert result.unique is False
    assert result.residual <= result.residual_tolerance
    # Force and moment balance, rebuilt from the published numbers.
    for residual in _equilibrium_rows(result):
        assert residual <= result.residual_tolerance
    assert np.isclose(
        sum(result.wheel_loads.values()),
        result.total_mass * 9810.0,
        rtol=0.0,
        atol=result.residual_tolerance,
    )
    # A minimum-norm member of a symmetric six-point layout splits the load evenly;
    # any other balanced solution would satisfy the balance checks and fail here.
    quarter = result.total_mass * 9810.0 / 6.0
    for name, value in result.wheel_loads.items():
        assert np.isclose(value, quarter, rtol=1e-9, atol=1e-6), name


def test_three_axle_six_contact_points_transfer_load_longitudinally() -> None:
    """A longitudinal acceleration moves load from the first axle to the last."""
    assembly = _three_axle_assembly()
    static = compute_static_wheel_loads_for_assembly(assembly)
    accelerated = compute_static_wheel_loads_for_assembly(
        assembly, acceleration=np.array([1_000.0, 0.0, 0.0])
    )

    assert accelerated.rank == 3
    assert accelerated.residual <= accelerated.residual_tolerance
    assert accelerated.wheel_loads["front_left"] < static.wheel_loads["front_left"]
    assert accelerated.wheel_loads["rear_left"] > static.wheel_loads["rear_left"]


def test_single_contact_point_under_the_centre_of_mass_solves_and_is_unique() -> None:
    """
    One contact point, directly below the centre of mass, with no horizontal
    acceleration: the equations are compatible, so there is a solution, and
    ``rank(A) = 1 = N`` says it is the only one.

    This is the case the point-count and rank thresholds of the old test would
    have refused outright -- ``rank = 1 < 3`` -- and it is a perfectly ordinary
    reading for a corner bench.
    """
    assembly = _corner_assembly()
    result = compute_static_wheel_loads_for_assembly(assembly)

    assert len(result.support_points) == 1
    # The precondition the case is about: the contact point is beneath the centre
    # of mass, so the moment arms are zero and the loads are compatible.
    contact = result.support_points["single_left"]
    assert abs(result.center_of_mass[0] - contact[0]) <= 1e-9
    assert abs(result.center_of_mass[1] - contact[1]) <= 1e-9

    assert result.rank == 1
    assert result.rank == len(result.support_points)
    assert result.unique is True
    assert result.residual <= result.residual_tolerance
    assert result.residual == 0.0
    # The one reaction carries the whole weight.
    assert np.isclose(
        result.wheel_loads["single_left"],
        result.total_mass * 9810.0,
        rtol=0.0,
        atol=result.residual_tolerance,
    )
    for residual in _equilibrium_rows(result):
        assert residual <= result.residual_tolerance


def _single_point_incompatible_cases() -> list[tuple[str, dict]]:
    """
    Return the single-contact-point states whose loads cannot be balanced there.

    The first keeps the geometry of the compatible case and changes only the
    loads: a lateral acceleration at a contact point directly below the centre of
    mass leaves a moment nothing can carry.  The second moves the contact point
    away from the centre of mass *and* accelerates, which is the same statement
    made more visible.  Both have the same rank-one balance matrix.
    """
    return [
        (
            "lateral acceleration under the centre of mass",
            {"acceleration": np.array([0.0, 1_000.0, 0.0]), "side_y": 0.0},
        ),
        (
            "contact point away from the centre of mass under longitudinal and "
            "lateral acceleration",
            {"acceleration": np.array([-3_000.0, 2_000.0, 0.0]), "side_y": -700.0},
        ),
    ]


@pytest.mark.parametrize(
    ("label", "case"),
    _single_point_incompatible_cases(),
    ids=[label for label, _ in _single_point_incompatible_cases()],
)
def test_single_contact_point_reports_incompatible_loads_by_name(
    label: str, case: dict
) -> None:
    """
    One contact point whose loads cannot be balanced there is refused, by name,
    with the measured residual, the tolerance and the contact-point count.

    The rank of this matrix is one whether or not the loads are compatible, so it
    could not have made this judgement; the residual is what does.  The message
    therefore names the residual and the tolerance, and never the rank.
    """
    assembly = _corner_assembly(side_y=case["side_y"])
    with pytest.raises(IncompatibleStaticLoadsError) as raised:
        compute_static_wheel_loads_for_assembly(assembly, acceleration=case["acceleration"])

    error = raised.value
    text = str(error)
    assert error.contact_points == 1
    assert error.residual > error.tolerance
    # The message carries the three numbers that make the refusal readable, and it
    # is a plain ValueError subclass so the production caller is unaffected.
    assert repr(error.residual) in text
    assert repr(error.tolerance) in text
    assert "1 contact point(s)" in text
    assert "rank" not in text
    assert isinstance(error, ValueError)


def test_the_rank_of_a_single_contact_point_matrix_does_not_change_with_the_loads() -> None:
    """
    The rank cannot separate a solvable single-contact-point state from an
    unsolvable one, because it is the same matrix in both.

    Asserted rather than argued: the same geometry solves with no acceleration and
    is refused with a lateral one, and the balance matrix is rank one either way.
    """
    assembly = _corner_assembly()
    solved = compute_static_wheel_loads_for_assembly(assembly)
    with pytest.raises(IncompatibleStaticLoadsError):
        compute_static_wheel_loads_for_assembly(
            assembly, acceleration=np.array([0.0, 1_000.0, 0.0])
        )

    # Rebuild both matrices from the same published geometry: they are equal.
    contact = solved.support_points["single_left"]
    matrix = np.array(
        [
            [1.0],
            [contact[0] - solved.center_of_mass[0]],
            [contact[1] - solved.center_of_mass[1]],
        ],
        dtype=float,
    )
    assert int(np.linalg.matrix_rank(matrix)) == solved.rank == 1


def test_four_wheel_vehicle_keeps_the_minimum_norm_split(full_vehicle_model) -> None:
    """
    The production four-wheel case is unchanged: four contact points, three
    independent equations, the minimum-norm split and the ``unique = False`` mark.

    ``vehicle/service.py`` reads this result for the dynamic service, so the shape
    and the value of the four-wheel answer are what must not move -- the N-point
    cases above are additions beside it.
    """
    result = compute_static_wheel_loads(full_vehicle_model)

    assert set(result.support_points) == set(_CORNERS)
    assert set(result.wheel_loads) == set(_CORNERS)
    assert len(result.support_points) == 4
    assert result.rank == 3
    assert result.rank < len(result.support_points)
    assert result.unique is False
    assert result.residual <= result.residual_tolerance
    for residual in _equilibrium_rows(result):
        assert residual <= result.residual_tolerance
    quarter = result.summary.total / 4.0
    for name, value in result.wheel_loads.items():
        assert np.isclose(value, quarter, rtol=1e-9, atol=1e-8), name


def test_the_contact_points_are_the_assembly_wheel_table() -> None:
    """
    The contact points come from the assembled vehicle, not from a corner tuple.

    Every wheel end the assembly declares contributes exactly one point, named
    after that wheel end, at the road plane under its wheel centre -- which is why
    a six-wheel vehicle gets six and a corner bench gets one from the same code.
    """
    assembly = compose_entries_runtime(
        [
            _entry("front", _axle("front", 1_400.0), (_wheel("front_left"), _wheel("front_right"))),
            _entry("rear", _axle("rear", -1_400.0), (_wheel("rear_left"), _wheel("rear_right"))),
        ],
        chassis_name="chassis",
        mode="K",
        request=_VEHICLE_REQUEST,
    )
    result = compute_static_wheel_loads_for_assembly(assembly)

    assert set(result.support_points) == set(assembly.wheel_centers)
    for name, (body, local_center) in assembly.wheel_centers.items():
        expected = assembly.state.point_world(body, local_center)
        point = result.support_points[name]
        assert point[0] == expected[0]
        assert point[1] == expected[1]
        assert point[2] == 0.0


def test_a_two_axle_model_still_reports_through_the_model_door() -> None:
    """
    The ``VehicleModel`` door keeps working: it composes the vehicle and reads the
    four contact points off the assembly, exactly as the service does.
    """
    model = VehicleModel(
        chassis=RigidBodySpec(name="chassis", mass=1200.0),
        front_axle=_axle("front", 1_400.0),
        rear_axle=_axle("rear", -1_400.0),
        wheels=tuple(_wheel(name) for name in _CORNERS),
        steering=SteeringSystemSpec(ratio=16.0),
    )

    result = compute_static_wheel_loads(model)

    assert set(result.wheel_loads) == set(_CORNERS)
    assert result.rank == 3
    assert result.unique is False
    assert result.residual <= result.residual_tolerance
