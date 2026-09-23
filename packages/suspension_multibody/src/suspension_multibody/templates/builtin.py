"""
The built-in double-wishbone template.

This is the existing `symmetric_proxy` topology written down as data.  It is the
reference the rest of the architecture is checked against, so the K/C mapping is
transcribed from what `build_front_axle` actually does rather than from what the
requirement's prose says it should do.

The one difference worth stating plainly, because a reader who trusts the prose
will get it wrong: in K mode the upper and lower arms each carry **one** revolute
joint, at the inboard *front* point, with the axis running to the inboard rear
point.  The inboard rear point carries no constraint of its own -- it exists only
to define that axis.  Putting a joint there as well would produce 14 constraints
where the assembly produces 13.

In C mode both inboard points become ball joints backed by bushings, and the
points that have no bushing column -- the arm outer points, the tie rod ends and
the rack guide -- stay joints.  That is why C mode has 9 constraints rather than
0: the compliant arm mounts replace four of the K joints, they do not remove
every joint in the model.
"""

from __future__ import annotations

from .model import (
    ConnectionDefinition,
    OutputDeclaration,
    PartDefinition,
    PropertySlot,
    Template,
)
from .registry import register

__all__ = ["DOUBLE_WISHBONE", "register_builtins"]

#: The template name the assembly layer will refer to.
DOUBLE_WISHBONE_NAME = "double_wishbone"

#: The bodies the template declares.  Both sides are in one template because the
#: anti-roll bar spans them and cannot belong to a one-sided subsystem.
_PARTS: tuple[PartDefinition, ...] = (
    PartDefinition("chassis", fixed=True),
    PartDefinition("rack"),
    PartDefinition("upper_arm_L"),
    PartDefinition("lower_arm_L"),
    PartDefinition("upright_L"),
    PartDefinition("tie_rod_L"),
    PartDefinition("upper_arm_R"),
    PartDefinition("lower_arm_R"),
    PartDefinition("upright_R"),
    PartDefinition("tie_rod_R"),
)

#: The connection points, with both columns written down.
#:
#: Sixteen connections: eight per side.  The inboard arm points carry a joint
#: column *and* a bushing column, which is the K/C choice the user described; the
#: outer points and the tie rod ends carry only a joint column, and so are joints
#: in both modes.
_CONNECTIONS: tuple[ConnectionDefinition, ...] = (
    # Left side.
    ConnectionDefinition("uca_mount_L_inner_front", "upper_front", "revolute", "uca_bushing_L_inner_front"),
    ConnectionDefinition("uca_mount_L_inner_rear", "upper_rear", None, "uca_bushing_L_inner_rear"),
    ConnectionDefinition("lca_mount_L_inner_front", "lower_front", "revolute", "lca_bushing_L_inner_front"),
    ConnectionDefinition("lca_mount_L_inner_rear", "lower_rear", None, "lca_bushing_L_inner_rear"),
    ConnectionDefinition("upper_arm_L_outer_joint", "upper_outer", "spherical"),
    ConnectionDefinition("lower_arm_L_outer_joint", "lower_outer", "spherical"),
    ConnectionDefinition("rack_tie_joint_L", "tie_inner", "spherical"),
    ConnectionDefinition("tie_upright_joint_L", "tie_outer", "spherical"),
    # Right side: the same structure, mirrored.
    ConnectionDefinition("uca_mount_R_inner_front", "upper_front", "revolute", "uca_bushing_R_inner_front"),
    ConnectionDefinition("uca_mount_R_inner_rear", "upper_rear", None, "uca_bushing_R_inner_rear"),
    ConnectionDefinition("lca_mount_R_inner_front", "lower_front", "revolute", "lca_bushing_R_inner_front"),
    ConnectionDefinition("lca_mount_R_inner_rear", "lower_rear", None, "lca_bushing_R_inner_rear"),
    ConnectionDefinition("upper_arm_R_outer_joint", "upper_outer", "spherical"),
    ConnectionDefinition("lower_arm_R_outer_joint", "lower_outer", "spherical"),
    ConnectionDefinition("rack_tie_joint_R", "tie_inner", "spherical"),
    ConnectionDefinition("tie_upright_joint_R", "tie_outer", "spherical"),
    # The wheel centre is an attachment point with no constraint of its own: it
    # locates the wheel and is what the drive coordinates measure against, so the
    # wheel role requires it as a mount while neither column applies.
    ConnectionDefinition("wheel_center_L", "wheel_center"),
    ConnectionDefinition("wheel_center_R", "wheel_center"),
)

#: The rack guide is a separate connection: it attaches the rack to the chassis
#: and is a joint in both modes.  Its point is the rack centre.
_RACK_GUIDE = ConnectionDefinition("rack_guide", "rack_center", "prismatic")

#: Property slots the template asks a properties file to fill.  The defaults are
#: the simplified template's own numbers: a template may carry its values
#: directly, which is what the current assembly does.
_PROPERTY_SLOTS: tuple[PropertySlot, ...] = (
    PropertySlot("spring", "N/m"),
    PropertySlot("damper", "N*s/m"),
    PropertySlot("bushing", "N/m"),
    PropertySlot("wheel_mass", "kg"),
    PropertySlot("unloaded_radius", "m"),
    PropertySlot("spin_axis", "-"),
    PropertySlot("wheel_center", "-"),
    PropertySlot("upper_front", "-"),
    PropertySlot("upper_rear", "-"),
    PropertySlot("upper_outer", "-"),
    PropertySlot("lower_front", "-"),
    PropertySlot("lower_rear", "-"),
    PropertySlot("lower_outer", "-"),
    PropertySlot("chassis_reference", "-"),
    PropertySlot("chassis_mass", "kg"),
    PropertySlot("chassis_inertia", "kg*m^2"),
    PropertySlot("rack_center", "-"),
    PropertySlot("tie_inner", "-"),
    PropertySlot("tie_outer", "-"),
    PropertySlot("rack_axis", "-"),
    PropertySlot("rack_fixed_to_chassis", "-"),
    PropertySlot("wheel_inertia", "kg*m^2"),
    PropertySlot("tire", "-"),
)

#: What the template contributes to the result.
_OUTPUTS: tuple[OutputDeclaration, ...] = (
    OutputDeclaration("wheel_travel", "mm", "kernel"),
    OutputDeclaration("camber", "deg", "derived"),
    OutputDeclaration("toe", "deg", "derived"),
    OutputDeclaration("track_change", "mm", "derived"),
    OutputDeclaration("wheel_center_pose", "mm", "kernel"),
    OutputDeclaration("tire_force", "N", "kernel"),
)

#: The suspension role's mounts are declared by the suspension half of this
#: template; the steering, wheel and chassis halves contribute theirs.  A single
#: template therefore satisfies four roles' mounts at once, which is why the
#: contract check unions the connections rather than partitioning them.
DOUBLE_WISHBONE = Template(
    name=DOUBLE_WISHBONE_NAME,
    role="suspension",
    parts=_PARTS,
    connections=_CONNECTIONS + (_RACK_GUIDE,),
    elastic_slots=("spring", "damper", "bushing"),
    property_slots=_PROPERTY_SLOTS,
    outputs=_OUTPUTS,
    suspension_kind="double_wishbone",
    description=(
        "The built-in double-wishbone template, transcribed from the existing "
        "symmetric_proxy assembly: arms rotate on a single inboard revolute in K "
        "mode, both inboard points become ball joints backed by bushings in C "
        "mode, and the outer points and tie rod ends stay joints in both."
    ),
)


def register_builtins() -> Template:
    """
    Register the built-in templates, idempotently.

    Called at import so the registry always holds the built-ins; safe to call
    again, because the second call replaces the first with the same object.
    """
    return register(DOUBLE_WISHBONE, replace=True)
