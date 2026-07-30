"""Schema force elements are converted into executable side-paired elements."""

import numpy as np

from suspension_multibody.elements import (
    BushingElement,
    LinearSpringElement,
    VerticalTireElement,
)
from suspension_multibody.model import build_front_axle
from suspension_multibody.schema import (
    Bushing6x6,
    FrontAxleModel,
    LinearSpring,
    MassSpec,
    Pose,
    Vec3,
    VerticalTire,
)


def _model() -> FrontAxleModel:
    hardpoints = {
        "uca_front": [-100, -500, 400],
        "uca_rear": [100, -500, 400],
        "uca_outer": [0, -700, 450],
        "lca_front": [-120, -500, 150],
        "lca_rear": [120, -500, 150],
        "lca_outer": [0, -700, 150],
        "tierod_inner": [100, -400, 250],
        "tierod_outer": [50, -700, 250],
        "wheel_center": [0, -700, 300],
        "rack_center": [0, 0, 250],
    }
    matrix = tuple(
        tuple(float(10 if row == column else 0) for column in range(6))
        for row in range(6)
    )
    return FrontAxleModel(
        hardpoints=hardpoints,
        mass=MassSpec(sprung_mass=1000),
        springs=(
            LinearSpring(
                name="coilover",
                body_a="chassis",
                body_b="lower_arm",
                point_a=Vec3(y=-500, z=200),
                point_b=Vec3(y=-700, z=150),
                stiffness=100,
                free_length=200,
            ),
        ),
        tires=(
            VerticalTire(
                stiffness=1000,
                unloaded_radius=300,
                contact_point=Vec3(y=-700),
            ),
        ),
        bushings=(
            Bushing6x6(
                name="mount",
                body_a="chassis",
                body_b="lower_arm",
                pose_a=Pose(translation=Vec3(y=-500, z=150)),
                pose_b=Pose(translation=Vec3(y=-500, z=150)),
                stiffness=matrix,
            ),
        ),
    )


def test_force_elements_are_side_paired_and_c_bushings_are_active() -> None:
    model = _model()
    k_assembly = build_front_axle(model, "K")
    c_assembly = build_front_axle(model, "C")
    assert (
        sum(isinstance(item, LinearSpringElement) for item in k_assembly.elements) == 2
    )
    assert (
        sum(isinstance(item, VerticalTireElement) for item in k_assembly.elements) == 2
    )
    assert not any(isinstance(item, BushingElement) for item in k_assembly.elements)
    assert any(
        isinstance(item, BushingElement) and np.linalg.norm(item.stiffness) > 0
        for item in c_assembly.elements
    )
    assert len(c_assembly.ideal_constraints) > len(c_assembly.constraints)
