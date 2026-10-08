"""Schema force elements are converted into executable side-paired elements."""

import numpy as np

from suspension_multibody.authoring import assemble_generic, migrate_v1_axle
from suspension_multibody.schema import (
    Bushing6x6,
    LinearSpring,
    MassSpec,
    Pose,
    Vec3,
    VerticalTire,
)
from suspension_multibody.schema.model import AxleDeclaration


def _model() -> AxleDeclaration:
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
    return AxleDeclaration(
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
    k_assembly, c_assembly = [assemble_generic(migrate_v1_axle(model, mode=mode)) for mode in ("K", "C")]
    assert sum(item["type"] == "spring" for item in k_assembly.elements) == 2
    assert len(k_assembly.tires) == 2
    assert not any(item["type"] == "bushing" for item in k_assembly.elements)
    assert any(item["type"] == "bushing" and np.linalg.norm(item["parameters"]["stiffness"]) > 0
               for item in c_assembly.elements)
    assert len(k_assembly.joints) > len(c_assembly.joints)
