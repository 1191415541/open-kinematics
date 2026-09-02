"""Full-vehicle topology assembly tests."""

import numpy as np

from suspension_multibody.core import PrismaticJoint
from suspension_multibody.model import build_vehicle
from suspension_multibody.model.front_axle import _build_explicit_axle
from suspension_multibody.schema import (
    FrontAxleModel,
    MassSpec,
    Pose,
    Quaternion,
    RigidBodySpec,
    SteeringSystemSpec,
    Vec3,
    VehicleModel,
    WheelSpec,
)


def _axle(name: str, x: float) -> FrontAxleModel:
    return FrontAxleModel(
        name=name,
        hardpoints={
            "UPPER_INBOARD_FRONT": Vec3(x=x, y=-500.0, z=500.0),
            "UPPER_INBOARD_REAR": Vec3(x=x + 150.0, y=-500.0, z=500.0),
            "UPPER_OUTBOARD": Vec3(x=x, y=-750.0, z=350.0),
            "LOWER_INBOARD_FRONT": Vec3(x=x, y=-500.0, z=100.0),
            "LOWER_INBOARD_REAR": Vec3(x=x + 150.0, y=-500.0, z=100.0),
            "LOWER_OUTBOARD": Vec3(x=x, y=-750.0, z=100.0),
            "TIE_ROD_INBOARD": Vec3(x=x, y=-450.0, z=250.0),
            "TIE_ROD_OUTBOARD": Vec3(x=x, y=-750.0, z=250.0),
            "WHEEL_CENTER": Vec3(x=x, y=-750.0, z=300.0),
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


def _vehicle() -> VehicleModel:
    return VehicleModel(
        chassis=RigidBodySpec(name="chassis", mass=1200.0),
        front_axle=_axle("front", 1_400.0),
        rear_axle=_axle("rear", -1_400.0),
        wheels=tuple(
            WheelSpec(
                name=name,
                body=f"wheel_{name}",
                center_local=Vec3(),
                mass=20.0,
                axial_inertia=2.0,
            )
            for name in ("front_left", "front_right", "rear_left", "rear_right")
        ),
        steering=SteeringSystemSpec(ratio=16.0),
    )


def test_build_vehicle_contains_two_axles_and_four_spinning_wheels() -> None:
    assembly = build_vehicle(_vehicle())

    assert set(assembly.axle_assemblies) == {"front", "rear"}
    assert set(assembly.wheel_ids) == {
        "front_left",
        "front_right",
        "rear_left",
        "rear_right",
    }
    assert len([item for item in assembly.constraints if "wheel_spin" in item.name]) == 4
    assert "front_upright_L" in assembly.component_ids
    assert "rear_upright_R" in assembly.component_ids


def test_build_vehicle_preserves_chassis_and_wheel_mass() -> None:
    assembly = build_vehicle(_vehicle())

    assert assembly.bodies["chassis"].mass == 1200.0
    assert assembly.total_mass == 1460.0
    assert all(assembly.wheel_center_local(name).shape == (3,) for name in assembly.wheel_ids)


def test_build_vehicle_condenses_fixed_wheel_into_mount() -> None:
    model = _vehicle()
    fixed_wheel = model.wheels[0].model_copy(
        update={"mount_joint_kind": "fixed", "mass": 20.0}
    )
    model = model.model_copy(
        update={"wheels": (fixed_wheel, *model.wheels[1:])}
    )

    assembly = build_vehicle(model)

    assert assembly.wheel_body_names["front_left"] == "front_upright_L"
    assert "wheel_front_left" not in assembly.bodies
    assert assembly.bodies["front_upright_L"].mass == 30.0
    assert assembly.total_mass == 1460.0
    assert not any(
        item.name.endswith("wheel_mount_front_left")
        for item in assembly.constraints
    )


def test_fixed_wheel_condensation_preserves_world_mass_properties() -> None:
    model = _vehicle()
    fixed_wheel = model.wheels[0].model_copy(
        update={
            "mount_joint_kind": "fixed",
            "center_local": Vec3(x=35.0, y=-8.0, z=12.0),
            "inertia": ((4.0, 0.2, 0.1), (0.2, 5.0, 0.3), (0.1, 0.3, 6.0)),
        }
    )
    wheel_model = model.model_copy(
        update={"wheels": (fixed_wheel, *model.wheels[1:])}
    )
    fixed_model = wheel_model.model_copy(
        update={"wheels": (fixed_wheel, *model.wheels[1:])}
    )
    original = build_vehicle(
        wheel_model.model_copy(
            update={
                "wheels": (
                    fixed_wheel.model_copy(update={"mount_joint_kind": "revolute"}),
                    *model.wheels[1:],
                )
            }
        )
    )
    condensed = build_vehicle(fixed_model)

    def mass_properties(assembly):
        first_moment = np.zeros(3)
        inertia_at_origin = np.zeros((3, 3))
        for body in assembly.bodies.values():
            center = body.pose.transform_point(body.center_of_mass)
            rotation = body.pose.rotation
            first_moment += body.mass * center
            offset = center
            inertia_at_origin += (
                rotation @ body.inertia @ rotation.T
                + body.mass
                * ((offset @ offset) * np.eye(3) - np.outer(offset, offset))
            )
        return sum(body.mass for body in assembly.bodies.values()), first_moment, inertia_at_origin

    expected = mass_properties(original)
    actual = mass_properties(condensed)
    assert actual[0] == expected[0]
    np.testing.assert_allclose(actual[1], expected[1], atol=1e-10)
    np.testing.assert_allclose(actual[2], expected[2], atol=1e-8)


def test_explicit_free_rack_has_axis_guide() -> None:
    """显式悬架中的自由齿条必须由沿齿条轴的理想棱柱副导向."""
    axle = FrontAxleModel(
        name="explicit_rack",
        hardpoints={"RACK_CENTER": Vec3()},
        mass=MassSpec(sprung_mass=1.0),
        bodies=(
            RigidBodySpec(
                name="rack",
                mass=1.0,
                pose=Pose(rotation=Quaternion(w=2**-0.5, x=2**-0.5)),
            ),
        ),
        topology="explicit",
        rack_fixed_to_chassis=False,
    )

    assembly = _build_explicit_axle(axle, "K")

    guides = [item for item in assembly.constraints if item.name == "rack_guide"]
    assert len(guides) == 1
    assert isinstance(guides[0], PrismaticJoint)
    np.testing.assert_allclose(guides[0].axis_a, (0.0, 1.0, 0.0))
    np.testing.assert_allclose(guides[0].axis_b, (0.0, 0.0, -1.0), atol=1e-12)
