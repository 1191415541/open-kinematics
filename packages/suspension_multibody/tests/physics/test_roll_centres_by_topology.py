"""
Roll centres for topologies that are not the double wishbone.

The point of the construction is that it reads the assembly's own constraint set, so
one algorithm covers the double wishbone, a five-link, a MacPherson strut and a twist
beam.  These models are the smallest declarations each topology assembles from; the
assertions are the properties the geometry must have whatever the linkage -- finite
values, mirror symmetry, and a sign that follows from where the arms point -- rather
than a number copied from one fixture.

The double wishbone's own comparison against the arm-line construction lives in
``tests/physics/test_vehicle_physics.py``.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.schema import (
    FrontAxleModel,
    IdealJointSpec,
    MassSpec,
    RigidBodySpec,
    SteeringSystemSpec,
    TireModelSpec,
    Vec3,
    VehicleModel,
    WheelSpec,
)
from suspension_multibody.vehicle.roll_centers import compute_vehicle_roll_centers

_X = Vec3(x=1.0, y=0.0, z=0.0)
_Y = Vec3(x=0.0, y=1.0, z=0.0)
_Z = Vec3(x=0.0, y=0.0, z=1.0)

#: The point where the twist beam's two trailing arms are joined: on the vehicle
#: centreline, ahead of the arm pivots.  Both arms state it, so the revolute joint
#: between them has coincident points and the pair is genuinely assembled.
_BEAM = Vec3(x=1180.0, y=0.0, z=250.0)

_WHEELS = ("front_left", "front_right", "rear_left", "rear_right")


def _spherical(name: str, a: str, point_a: Vec3, b: str, point_b: Vec3) -> IdealJointSpec:
    return IdealJointSpec(
        name=name, kind="spherical", body_a=a, body_b=b, point_a=point_a, point_b=point_b
    )


def _revolute(
    name: str, a: str, point_a: Vec3, b: str, point_b: Vec3, axis: Vec3
) -> IdealJointSpec:
    return IdealJointSpec(
        name=name,
        kind="revolute",
        body_a=a,
        body_b=b,
        point_a=point_a,
        point_b=point_b,
        axis_a=axis,
        axis_b=axis,
    )


def _prismatic(
    name: str, a: str, point_a: Vec3, b: str, point_b: Vec3, axis: Vec3
) -> IdealJointSpec:
    return IdealJointSpec(
        name=name,
        kind="prismatic",
        body_a=a,
        body_b=b,
        point_a=point_a,
        point_b=point_b,
        axis_a=axis,
        axis_b=axis,
    )


def _axle(name: str, joints: tuple[IdealJointSpec, ...], bodies: tuple[str, ...]) -> FrontAxleModel:
    return FrontAxleModel(
        name=name,
        topology="explicit",
        hardpoints={
            "WHEEL_CENTER": Vec3(x=1400.0, y=-720.0, z=300.0),
            "WHEEL_CENTER__R": Vec3(x=1400.0, y=720.0, z=300.0),
        },
        mass=MassSpec(sprung_mass=600.0),
        bodies=tuple(RigidBodySpec(name=body, mass=10.0) for body in bodies),
        joints=joints,
    )


def _vehicle(axle: FrontAxleModel) -> VehicleModel:
    return VehicleModel(
        chassis=RigidBodySpec(name="chassis", mass=1200.0),
        front_axle=axle,
        rear_axle=axle,
        wheels=tuple(
            WheelSpec(
                name=name,
                body=f"wheel_{name}",
                center_local=Vec3(),
                mass=20.0,
                axial_inertia=2.0,
                tire=TireModelSpec(kind="fiala", vertical_stiffness=20.0),
            )
            for name in _WHEELS
        ),
        steering=SteeringSystemSpec(ratio=16.0),
    )


def _five_link() -> FrontAxleModel:
    """Five two-force rods per side, each pinned to the chassis and to the upright."""
    joints: list[IdealJointSpec] = []
    bodies = ["upright_L", "upright_R"]
    for side, sign in (("L", -1.0), ("R", 1.0)):
        for link in range(1, 6):
            body = f"link{link}_{side}"
            bodies.append(body)
            chassis_point = Vec3(x=1400.0 + link * 20.0, y=sign * 500.0, z=300.0 + link * 10.0)
            upright_point = Vec3(x=1400.0 + link * 20.0, y=sign * 690.0, z=290.0 + link * 10.0)
            joints.append(
                _spherical(f"{body}_chassis", "chassis", chassis_point, body, chassis_point)
            )
            joints.append(
                _spherical(f"{body}_upright", body, upright_point, f"upright_{side}", upright_point)
            )
    return _axle("five_link", tuple(joints), tuple(bodies))


def _macpherson() -> FrontAxleModel:
    """Build a lower arm plus a strut that slides in its tower and pivots on the upright."""
    joints: list[IdealJointSpec] = []
    bodies = ["upright_L", "upright_R"]
    for side, sign in (("L", -1.0), ("R", 1.0)):
        lower = f"lower_{side}"
        strut = f"strut_{side}"
        bodies += [lower, strut]
        pivot = Vec3(x=1400.0, y=sign * 500.0, z=150.0)
        joints.append(_revolute(f"{lower}_pivot", "chassis", pivot, lower, pivot, _X))
        outer = Vec3(x=1400.0, y=sign * 700.0, z=150.0)
        joints.append(_spherical(f"{lower}_outer", lower, outer, f"upright_{side}", outer))
        top = Vec3(x=1400.0, y=sign * 560.0, z=800.0)
        joints.append(_prismatic(f"{strut}_slide", "chassis", top, strut, top, _Z))
        bottom = Vec3(x=1400.0, y=sign * 560.0, z=300.0)
        joints.append(_spherical(f"{strut}_lower", strut, bottom, f"upright_{side}", bottom))
    return _axle("macpherson", tuple(joints), tuple(bodies))


def _twist_beam() -> FrontAxleModel:
    """Two trailing arms pivoting on the chassis, joined by the beam between them."""
    joints: list[IdealJointSpec] = []
    for side, sign in (("L", -1.0), ("R", 1.0)):
        pivot = Vec3(x=1100.0, y=sign * 480.0, z=250.0)
        joints.append(
            _revolute(f"arm_{side}", "chassis", pivot, f"upright_{side}", pivot, _Y)
        )
    joints.append(
        # Both arms carry the beam's centre point: a revolute joint's two points must
        # coincide, so the shared point sits on the vehicle centreline.  Stating the
        # same point on each arm is what makes the two arms a joined pair.
        _revolute("beam", "upright_L", _BEAM, "upright_R", _BEAM, _Z)
    )
    return _axle("twist_beam", tuple(joints), ("upright_L", "upright_R"))


_TOPOLOGIES = {
    "five-link": _five_link,
    "macpherson": _macpherson,
    "twist-beam": _twist_beam,
}


@pytest.mark.parametrize("topology", sorted(_TOPOLOGIES))
def test_every_topology_gives_a_finite_symmetric_roll_centre(topology: str) -> None:
    """
    Each topology yields a finite roll centre on the vehicle centreline.

    Finite says the construction read a mechanism rather than dividing by a zero the
    solve produced; a zero lateral position says the two sides were treated alike,
    which a construction that favoured one side could not produce on a mirror-image
    axle.
    """
    centers = compute_vehicle_roll_centers(_vehicle(_TOPOLOGIES[topology]()))

    assert set(centers) == {"front", "rear"}
    for result in centers.values():
        assert np.all(np.isfinite(result.center)), topology
        assert np.isclose(result.center[0], 0.0, atol=1e-6), topology
        assert result.lateral_force < 0.0, topology
        assert np.isfinite(result.roll_moment), topology


@pytest.mark.parametrize("topology", sorted(_TOPOLOGIES))
def test_every_topology_mirrors_the_two_sides(topology: str) -> None:
    """
    The two sides' patches and force-line slopes are mirror images.

    Each side's read comes from a separate solve of the same mirrored mechanism, so
    an asymmetry here would be the construction's, not the geometry's.
    """
    centers = compute_vehicle_roll_centers(_vehicle(_TOPOLOGIES[topology]()))
    for result in centers.values():
        assert np.isclose(
            result.left_contact_patch[1], -result.right_contact_patch[1], atol=1e-9
        ), topology
        assert np.isclose(
            result.left_contact_patch_slope, -result.right_contact_patch_slope, atol=1e-9
        ), topology


@pytest.mark.parametrize("topology", sorted(_TOPOLOGIES))
def test_the_height_is_the_ratio_the_generalized_loads_define(topology: str) -> None:
    """
    The reported height is exactly ``-Q_phi / Q_uy`` of the matrix.

    This is the arithmetic the documentation states, asserted rather than assumed: a
    construction that reported a geometric intersection instead could still return a
    finite symmetric point and would fail here.
    """
    for result in compute_vehicle_roll_centers(_vehicle(_TOPOLOGIES[topology]())).values():
        assert result.lateral_force != 0.0, topology
        assert np.isclose(
            result.center[1], -result.roll_moment / result.lateral_force, rtol=1e-12
        ), topology


def test_the_height_matches_the_force_line_primitives_it_reports() -> None:
    """
    The reported height is the one the reported primitives define.

    The result carries the two contact patches and the two force-line slopes it was
    built from, so the height can be rebuilt here -- outside the module, from those
    four numbers alone -- and must come out the same.  A construction that reported a
    geometric intersection would have nothing to rebuild from and would fail.
    """
    for result in compute_vehicle_roll_centers(_vehicle(_two_way_wishbone())).values():
        patches = np.array([result.left_contact_patch, result.right_contact_patch])
        slopes = np.array([result.left_contact_patch_slope, result.right_contact_patch_slope])
        lateral = float(np.sum(-np.ones(2)))
        moment = float(np.sum(patches[:, 1] * slopes))
        assert np.isclose(result.lateral_force, lateral, rtol=1e-12), result.axle
        assert np.isclose(result.roll_moment, moment, rtol=1e-12), result.axle
        assert np.isclose(result.center[1], -moment / lateral, rtol=1e-12), result.axle


def test_a_lateral_translation_carries_the_patch_the_other_way() -> None:
    """
    Each side's patch moves by ``-u_y`` per unit lateral translation of the sprung mass.

    This is the matrix's first column, and it is a kinematic fact independent of the
    solve: the patch is a material point of the wheel end, so moving the sprung mass
    sideways by ``u`` moves the wheel end, and with it the patch, by ``-u`` relative to
    it.  The module's lateral-force column must be the sum of that column, so its value
    is ``-2`` whenever both patches carry a unit lateral force -- asserted here so a
    change to the column is caught.
    """
    for result in compute_vehicle_roll_centers(_vehicle(_two_way_wishbone())).values():
        assert np.isclose(result.lateral_force, -2.0, rtol=1e-12), result.axle


def _two_way_wishbone() -> FrontAxleModel:
    """State the double wishbone the explicit way, so the fixture is self-contained."""
    joints: list[IdealJointSpec] = []
    bodies = ["upright_L", "upright_R"]
    for side, sign in (("L", -1.0), ("R", 1.0)):
        upper, lower = f"upper_{side}", f"lower_{side}"
        bodies += [upper, lower]
        for arm, inner_z, outer_z in ((upper, 500.0, 350.0), (lower, 100.0, 100.0)):
            inner = Vec3(x=1400.0, y=sign * 500.0, z=inner_z)
            outer = Vec3(x=1400.0, y=sign * 750.0, z=outer_z)
            joints.append(_revolute(f"{arm}_pivot", "chassis", inner, arm, inner, _X))
            joints.append(_spherical(f"{arm}_outer", arm, outer, f"upright_{side}", outer))
    return _axle("double_wishbone", tuple(joints), tuple(bodies))
