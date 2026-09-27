"""
The fixtures the acceptance run builds from.

Kept beside the acceptance script rather than imported from the test suite: the
acceptance run is meant to be *independent* of how the subtasks were tested, and a
fixture that lives in `tests/` would tie it to that suite's refactors.

Three models:

* :func:`probe_model` -- the plain axle the flow checks compose, with one spring and
  one tire so that `elements` and `tires` are non-empty and their names are
  comparable;
* :func:`trailing_arm` -- a genuinely different suspension layout, used to check that
  choosing a template changes the model's entities.  It declares **no upper arm**, so
  a model built from it cannot contain one;
* :func:`explicit_model` -- an explicit-topology axle in either rack branch, used to
  check that the composition layer handles topologies the templates do not describe.
"""

from __future__ import annotations

from suspension_multibody.schema import (
    FrontAxleModel,
    IdealJointSpec,
    LinearSpring,
    MassSpec,
    Pose,
    Quaternion,
    RigidBodySpec,
    Vec3,
)
from suspension_multibody.templates.model import (
    ConnectionDefinition,
    PartDefinition,
    PropertySlot,
    Template,
)

__all__ = ["explicit_model", "probe_model", "trailing_arm"]

_HARDPOINTS = {
    "uca_front": Vec3(x=-100, y=-500, z=400),
    "uca_rear": Vec3(x=100, y=-500, z=400),
    "uca_outer": Vec3(x=0, y=-700, z=450),
    "lca_front": Vec3(x=-120, y=-500, z=150),
    "lca_rear": Vec3(x=120, y=-500, z=150),
    "lca_outer": Vec3(x=0, y=-700, z=150),
    "tierod_inner": Vec3(x=100, y=-400, z=250),
    "tierod_outer": Vec3(x=50, y=-700, z=250),
    "wheel_center": Vec3(x=0, y=-700, z=300),
    "rack_center": Vec3(x=0, y=0, z=250),
}


def probe_model() -> FrontAxleModel:
    """
    The plain axle the flow checks compose.

    **No tire.**  A vertical tire in a K case makes the run fail its static-equilibrium
    initialisation: K *prescribes* the wheel travel, so the wheel centre is driven to a
    chosen height, while the tire develops a force from the compression that follows --
    and the two cannot both be satisfied.  Measured: with the tire declared, the default
    template fails at every non-zero amplitude; without it, it converges at both 20 mm
    and 40 mm.

    That is a property of the K reading and not of this fixture -- the snapshot, parity
    and template checks all read K without tires for the same reason.  A fixture that
    could not solve the *default* template would make the template comparison a
    comparison between two failures.
    """
    return FrontAxleModel(
        hardpoints=dict(_HARDPOINTS),
        mass=MassSpec(sprung_mass=1000),
        springs=(
            LinearSpring(
                name="spring_L",
                body_a="chassis",
                point_a=Vec3(x=0.0, y=-500.0, z=400.0),
                body_b="lower_arm_L",
                point_b=Vec3(x=0.0, y=-650.0, z=200.0),
                stiffness=100.0,
                # Exactly one of free_length / reference_length: the element refuses
                # both at once, because they are two ways of stating one reference.
                free_length=200.0,
            ),
        ),
    )


def trailing_arm() -> Template:
    """
    A single-arm layout: one arm and one upright per side, and no upper arm.

    The suspension role requires seven mount names, so the upper ones are declared --
    they locate geometry without owning a body.  That is the structural difference
    this template exists to exercise: the *entities* change, not merely their names.
    """
    connections: list[ConnectionDefinition] = []
    for side in ("L", "R"):
        connections.extend(
            [
                ConnectionDefinition(
                    f"lca_mount_{side}_inner_front",
                    "lower_front",
                    joint="revolute",
                    bushing=f"lca_bushing_{side}_inner_front",
                    owner=f"lower_arm_{side}",
                    label="inner_front",
                    axis_reference_role="lower_rear",
                    far_owner="chassis",
                    far_label=f"lca_{side}_inner_front",
                ),
                ConnectionDefinition(
                    f"lca_mount_{side}_inner_rear",
                    "lower_rear",
                    owner=f"lower_arm_{side}",
                    label="inner_rear",
                    far_owner="chassis",
                    far_label=f"lca_{side}_inner_rear",
                ),
                ConnectionDefinition(
                    f"lower_arm_{side}_outer_joint",
                    "lower_outer",
                    joint="spherical",
                    owner=f"lower_arm_{side}",
                    label="outer",
                    far_owner=f"upright_{side}",
                    far_label=f"lower_arm_{side}_outer",
                ),
                # The upper mounts: declared because the role demands the names, inert
                # because this layout has no upper arm to constrain.
                ConnectionDefinition(
                    f"upper_locator_{side}_front",
                    "upper_front",
                    owner=f"lower_arm_{side}",
                    label="inner_front_upper",
                ),
                ConnectionDefinition(
                    f"upper_locator_{side}_rear",
                    "upper_rear",
                    owner=f"lower_arm_{side}",
                    label="inner_rear_upper",
                ),
                ConnectionDefinition(
                    f"upper_locator_{side}_outer",
                    "upper_outer",
                    owner=f"lower_arm_{side}",
                    label="outer_upper",
                ),
                ConnectionDefinition(
                    f"wheel_center_{side}",
                    "wheel_center",
                    owner=f"upright_{side}",
                    label="wheel_center",
                ),
            ]
        )
    return Template(
        name="trailing_arm",
        role="suspension",
        parts=(
            PartDefinition("chassis", fixed=True),
            PartDefinition("rack"),
            PartDefinition("lower_arm_L"),
            PartDefinition("upright_L"),
            # No `upper_arm_L`: this layout has none, and building one anyway is the
            # defect this template is here to catch.
            PartDefinition("tie_rod_L"),
            PartDefinition("lower_arm_R"),
            PartDefinition("upright_R"),
            PartDefinition("tie_rod_R"),
        ),
        connections=tuple(connections),
        property_slots=(
            PropertySlot("spring", "N/m"),
            PropertySlot("damper", "N*s/m"),
            PropertySlot(
                "bushing",
                "N/m",
                default=0.0,
                connections=(
                    "lca_mount_L_inner_front",
                    "lca_mount_R_inner_front",
                ),
            ),
        ),
    )


def explicit_model(*, rack_fixed: bool) -> FrontAxleModel:
    """
    An explicit-topology axle: it states its own parts and joints.

    Two branches, because the rack's attachment is the difference the acceptance
    criterion names: a free rack is guided by a prismatic joint, a rack fixed to the
    chassis is welded to it.
    """
    bodies = [
        RigidBodySpec(
            name="rack",
            mass=10.0,
            inertia=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            pose=Pose(rotation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)),
        ),
        RigidBodySpec(
            name="upright_L",
            mass=20.0,
            inertia=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            pose=Pose(rotation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)),
        ),
    ]
    joints = [
        IdealJointSpec(
            name="rack_to_upright_L",
            kind="spherical",
            body_a="rack",
            body_b="upright_L",
            point_a=Vec3(x=50.0, y=-400.0, z=250.0),
            point_b=Vec3(x=50.0, y=-400.0, z=250.0),
        ),
    ]
    return FrontAxleModel(
        hardpoints={
            "wheel_center": Vec3(x=0.0, y=-700.0, z=300.0),
            "rack_center": Vec3(x=0.0, y=0.0, z=250.0),
        },
        mass=MassSpec(sprung_mass=1000),
        topology="explicit",
        rack_fixed_to_chassis=rack_fixed,
        bodies=tuple(bodies),
        joints=tuple(joints),
    )
