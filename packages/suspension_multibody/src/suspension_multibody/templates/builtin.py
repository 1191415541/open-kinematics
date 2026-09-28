"""
The built-in double-wishbone template.

This is the existing `symmetric_proxy` topology written down as data.  It is the
reference the rest of the architecture is checked against, so the K/C mapping is
transcribed from what the composition entry actually does rather than from what the
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

__all__ = [
    "BUILTINS",
    "CHASSIS",
    "DEFAULT_MOUNT_STIFFNESS",
    "DOUBLE_WISHBONE",
    "DOUBLE_WISHBONE_NAME",
    "STEERING_FIXED",
    "STEERING_GUIDED",
    "WHEEL",
    "register_builtins",
]

#: The built-in template's own bushing stiffness: zero.
#:
#: Zero is the *recorded* state of this template, not a placeholder waiting to be
#: filled.  The C-mode compliance in the published baseline comes from the model's
#: own `model.bushings`, and the four inboard slots this template declares are
#: zero-stiffness slots on top of that.  Changing this number changes the C solve,
#: and the frozen C snapshot cannot be regenerated, so a template that wants real
#: stiffness here supplies it through a properties file instead (subtask 06).
DEFAULT_MOUNT_STIFFNESS = 0.0

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
#:
#: Two facts are stated per connection that a single `joint` field cannot carry,
#: and both are transcribed from what the assembly produces rather than from what
#: the prose suggests:
#:
#: * the inboard **rear** arm point carries nothing in K mode -- the arm pivots on
#:   one revolute at the front point whose axis runs to the rear one -- so its
#:   bushing column is active in C only, and its joint column exists for C's ideal
#:   set alone;
#: * the inboard **front** point is a revolute in K and a ball joint in C, which
#:   is why the joint type is declared per mode.
#:
#: `owner` / `label` name where each point sits, and `fixed_owner` /
#: `fixed_label` name the far end of a two-ended mount.  Stating them is what lets
#: the assembly resolve a declared connection against a real hardpoint instead of
#: inferring the attachment from the connection's own name.
def _mount(
    name: str,
    role: str,
    *,
    owner: str,
    label: str,
    joint: str | None,
    bushing: str | None,
    joint_modes: tuple[str, ...] = ("K", "C"),
    bushing_modes: tuple[str, ...] = ("K", "C"),
    joint_kind_by_mode: tuple[tuple[str, str], ...] = (),
    axis_reference_role: str = "",
) -> ConnectionDefinition:
    """
    Build one two-ended inboard mount: the arm's point plus the chassis's.

    The chassis-side label is the one the assembly records, derived from the arm
    the mount belongs to and the side it is on.  Deriving it here rather than
    letting a builder guess keeps the two ends of one mount in agreement.
    """
    side = owner.rsplit("_", 1)[-1]
    stem = "uca" if owner.startswith("upper") else "lca"
    return ConnectionDefinition(
        name=name,
        role=role,
        joint=joint,
        bushing=bushing,
        owner=owner,
        label=label,
        joint_modes=joint_modes,
        bushing_modes=bushing_modes,
        joint_kind_by_mode=joint_kind_by_mode,
        far_owner="chassis",
        far_label=f"{stem}_{side}_{label}",
        axis_reference_role=axis_reference_role,
    )


_CONNECTIONS: tuple[ConnectionDefinition, ...] = (
    # Left side.  The inboard mounts are two-ended; the outer joints and tie rod
    # ends are one-ended, because the subsystem that owns them already places the
    # far end on the other arm or on the rack.
    _mount(
        "uca_mount_L_inner_front",
        "upper_front",
        owner="upper_arm_L",
        label="inner_front",
        joint="revolute",
        bushing="uca_bushing_L_inner_front",
        joint_kind_by_mode=(("C", "spherical"),),
        axis_reference_role="upper_rear",
    ),
    _mount(
        "uca_mount_L_inner_rear",
        "upper_rear",
        owner="upper_arm_L",
        label="inner_rear",
        joint="spherical",
        bushing="uca_bushing_L_inner_rear",
        joint_modes=("C",),
        bushing_modes=("C",),
    ),
    _mount(
        "lca_mount_L_inner_front",
        "lower_front",
        owner="lower_arm_L",
        label="inner_front",
        joint="revolute",
        bushing="lca_bushing_L_inner_front",
        joint_kind_by_mode=(("C", "spherical"),),
        axis_reference_role="lower_rear",
    ),
    _mount(
        "lca_mount_L_inner_rear",
        "lower_rear",
        owner="lower_arm_L",
        label="inner_rear",
        joint="spherical",
        bushing="lca_bushing_L_inner_rear",
        joint_modes=("C",),
        bushing_modes=("C",),
    ),
    ConnectionDefinition(
        "upper_arm_L_outer_joint",
        "upper_outer",
        "spherical",
        owner="upper_arm_L",
        label="outer",
        far_owner="upright_L",
        far_label="upper_arm_L_outer",
    ),
    ConnectionDefinition(
        "lower_arm_L_outer_joint",
        "lower_outer",
        "spherical",
        owner="lower_arm_L",
        label="outer",
        far_owner="upright_L",
        far_label="lower_arm_L_outer",
    ),
    ConnectionDefinition(
        "rack_tie_joint_L",
        "tie_inner",
        "spherical",
        owner="tie_rod_L",
        label="inner",
        far_owner="rack",
        far_label="tie_L",
    ),
    ConnectionDefinition(
        "tie_upright_joint_L",
        "tie_outer",
        "spherical",
        owner="tie_rod_L",
        label="outer",
        far_owner="upright_L",
        far_label="tie_outer",
    ),
    # Right side: the same structure, mirrored.
    _mount(
        "uca_mount_R_inner_front",
        "upper_front",
        owner="upper_arm_R",
        label="inner_front",
        joint="revolute",
        bushing="uca_bushing_R_inner_front",
        joint_kind_by_mode=(("C", "spherical"),),
        axis_reference_role="upper_rear",
    ),
    _mount(
        "uca_mount_R_inner_rear",
        "upper_rear",
        owner="upper_arm_R",
        label="inner_rear",
        joint="spherical",
        bushing="uca_bushing_R_inner_rear",
        joint_modes=("C",),
        bushing_modes=("C",),
    ),
    _mount(
        "lca_mount_R_inner_front",
        "lower_front",
        owner="lower_arm_R",
        label="inner_front",
        joint="revolute",
        bushing="lca_bushing_R_inner_front",
        joint_kind_by_mode=(("C", "spherical"),),
        axis_reference_role="lower_rear",
    ),
    _mount(
        "lca_mount_R_inner_rear",
        "lower_rear",
        owner="lower_arm_R",
        label="inner_rear",
        joint="spherical",
        bushing="lca_bushing_R_inner_rear",
        joint_modes=("C",),
        bushing_modes=("C",),
    ),
    ConnectionDefinition(
        "upper_arm_R_outer_joint",
        "upper_outer",
        "spherical",
        owner="upper_arm_R",
        label="outer",
        far_owner="upright_R",
        far_label="upper_arm_R_outer",
    ),
    ConnectionDefinition(
        "lower_arm_R_outer_joint",
        "lower_outer",
        "spherical",
        owner="lower_arm_R",
        label="outer",
        far_owner="upright_R",
        far_label="lower_arm_R_outer",
    ),
    ConnectionDefinition(
        "rack_tie_joint_R",
        "tie_inner",
        "spherical",
        owner="tie_rod_R",
        label="inner",
        far_owner="rack",
        far_label="tie_R",
    ),
    ConnectionDefinition(
        "tie_upright_joint_R",
        "tie_outer",
        "spherical",
        owner="tie_rod_R",
        label="outer",
        far_owner="upright_R",
        far_label="tie_outer",
    ),
    # The wheel centre is an attachment point with no constraint of its own: it
    # locates the wheel and is what the drive coordinates measure against, so the
    # wheel role requires it as a mount while neither column applies.
    ConnectionDefinition(
        "wheel_center_L", "wheel_center", owner="upright_L", label="wheel_center"
    ),
    ConnectionDefinition(
        "wheel_center_R", "wheel_center", owner="upright_R", label="wheel_center"
    ),
)

#: The rack guide is a separate connection: it attaches the rack to the chassis
#: and is a joint in both modes.  Its point is the rack centre.
_RACK_GUIDE = ConnectionDefinition(
    "rack_guide",
    "rack_center",
    "prismatic",
    owner="rack",
    label="center",
    far_owner="chassis",
    far_label="rack_center",
)

#: Property slots the template asks a properties file to fill.  The defaults are
#: the simplified template's own numbers: a template may carry its values
#: directly, which is what the current assembly does.
_PROPERTY_SLOTS: tuple[PropertySlot, ...] = (
    PropertySlot("spring", "N/m"),
    PropertySlot("damper", "N*s/m"),
    #: The inboard mount bushing the C-mode column activates.  Its default is
    #: zero, not an oversight: the frozen C snapshot in `tests/data/kc_baseline`
    #: was solved with the compliant mounts at *zero* stiffness, and that snapshot
    #: is an oracle the implementation is judged against -- it cannot be
    #: regenerated (the Python quasi-static solver that produced it was retired).
    #: A template whose bushing column is meant to carry real stiffness supplies
    #: one through a properties file; `DEFAULT_MOUNT_STIFFNESS` is simply the
    #: built-in simplified template's own number.
    PropertySlot(
        "bushing",
        "N/m",
        default=DEFAULT_MOUNT_STIFFNESS,
        connections=tuple(
            connection.name
            for connection in _CONNECTIONS
            if connection.bushing is not None
        ),
    ),
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


# --- the single-role templates ------------------------------------------------
#
# The composition used to answer "what does the steering subsystem contain" from
# the function that emitted it: `steering.py` held the rack, the tie rods, the two
# ball joints and the guide as literals, and a *file* could describe none of them.
# These four templates are those same declarations written down as data, one per
# role, so those modules can read their declaration instead of carrying it.
#
# The steering role needs *two* templates rather than one, and that is the file
# format's own limit showing through rather than a preference: whether the rack is
# guided along the model's rack axis or bolted to the chassis is decided by
# `model.rack_fixed_to_chassis`, which is a *model* field, and a template has no
# spelling for a conditional.  Two templates state the two cases and the module
# picks the one the model asks for -- the same way a per-mode column states its
# two cases by declaring both.

_STEERING_PARTS: tuple[PartDefinition, ...] = (
    PartDefinition("rack"),
    PartDefinition("tie_rod_L"),
    PartDefinition("tie_rod_R"),
)

#: The tie rod ends, both sides: a ball joint at the rack and one at the upright.
#: The names are the ones the recorded contract carries, so a template-driven
#: steering emits the same constraints the literal build did.
_STEERING_TIES: tuple[ConnectionDefinition, ...] = (
    ConnectionDefinition(
        "rack_tie_joint_L",
        "tie_inner",
        "spherical",
        owner="tie_rod_L",
        label="inner",
        far_owner="rack",
        far_label="tie_L",
    ),
    ConnectionDefinition(
        "tie_upright_joint_L",
        "tie_outer",
        "spherical",
        owner="tie_rod_L",
        label="outer",
        far_owner="upright_L",
        far_label="tie_outer",
        # The tie rod is this joint's first body, while the rack-side joint records
        # the rack first.  Both are what the assembly has always emitted, and
        # stating them is what makes the template describe *that* assembly rather
        # than a plausible one.
        first_body="far",
    ),
    ConnectionDefinition(
        "rack_tie_joint_R",
        "tie_inner",
        "spherical",
        owner="tie_rod_R",
        label="inner",
        far_owner="rack",
        far_label="tie_R",
    ),
    ConnectionDefinition(
        "tie_upright_joint_R",
        "tie_outer",
        "spherical",
        owner="tie_rod_R",
        label="outer",
        far_owner="upright_R",
        far_label="tie_outer",
        first_body="far",
    ),
)

_STEERING_SLOTS: tuple[PropertySlot, ...] = (
    PropertySlot("rack_axis", "-", default=0.0),
    PropertySlot("rack_fixed_to_chassis", "-", default=0.0),
)

_STEERING_OUTPUTS: tuple[OutputDeclaration, ...] = (
    OutputDeclaration("rack_displacement", "mm", "kernel"),
)

STEERING_GUIDED = Template(
    name="steering_guided",
    role="steering",
    parts=_STEERING_PARTS,
    connections=_STEERING_TIES
    + (
        ConnectionDefinition(
            "rack_guide",
            "rack_center",
            "prismatic",
            owner="rack",
            label="center",
            far_owner="chassis",
            far_label="rack_center",
        ),
    ),
    property_slots=_STEERING_SLOTS,
    outputs=_STEERING_OUTPUTS,
    description=(
        "A rack guided along the model's rack axis: the rack is a body with one "
        "ideal degree of freedom, and the axis it slides along is the model's "
        "rather than the template's."
    ),
)

STEERING_FIXED = Template(
    name="steering_fixed",
    role="steering",
    parts=_STEERING_PARTS,
    connections=_STEERING_TIES
    + (
        ConnectionDefinition(
            "rack_fixed_to_chassis",
            "rack_center",
            "fixed",
            owner="rack",
            label="center",
            far_owner="chassis",
            far_label="rack_center",
        ),
    ),
    property_slots=_STEERING_SLOTS,
    outputs=_STEERING_OUTPUTS,
    description=(
        "A rack bolted to the chassis: the vehicle declares one steering system, "
        "so the axle it does not steer carries a rack that cannot move."
    ),
)

CHASSIS = Template(
    name="chassis_fixed",
    role="chassis",
    parts=(PartDefinition("chassis", fixed=True),),
    connections=(
        ConnectionDefinition(
            "chassis_reference",
            "chassis_reference",
            owner="chassis",
            label="reference",
        ),
    ),
    property_slots=(
        PropertySlot("chassis_mass", "kg", default=0.0),
        PropertySlot("chassis_inertia", "kg*m^2", default=0.0),
    ),
    outputs=(OutputDeclaration("chassis_pose", "mm", "kernel"),),
    description=(
        "The ground-side body an axle reacts against: one fixed body.  The axle "
        "side consumes no mass here, which is the recorded state rather than an "
        "omission."
    ),
)

WHEEL = Template(
    name="wheel_on_upright",
    role="wheel",
    # No parts: on a single axle the wheel is supplied by the rig rather than by
    # the assembly (decision D9), so the wheel role declares where a wheel would
    # attach instead of declaring a body.
    parts=(),
    connections=(
        ConnectionDefinition(
            "wheel_center_L",
            "wheel_center",
            owner="upright_L",
            label="wheel_center",
        ),
        # The spin axis carries no owner: which body spins and about what is the
        # *model's* statement (`VehicleModel.wheels[].spin_axis`), and the axle
        # side hangs its tire on the wheel centre instead.
        ConnectionDefinition("spin_axis_L", "spin_axis"),
        ConnectionDefinition(
            "wheel_center_R",
            "wheel_center",
            owner="upright_R",
            label="wheel_center",
        ),
        ConnectionDefinition("spin_axis_R", "spin_axis"),
    ),
    property_slots=(
        PropertySlot("unloaded_radius", "m", default=0.0),
        PropertySlot("wheel_mass", "kg", default=0.0),
        PropertySlot("wheel_inertia", "kg*m^2", default=0.0),
        PropertySlot("tire", "-", default=0.0),
    ),
    outputs=(
        OutputDeclaration("wheel_center_pose", "mm", "kernel"),
        OutputDeclaration("tire_force", "N", "kernel"),
    ),
    description=(
        "The wheel end: the point a wheel attaches at and the axis it spins "
        "about, with the tire's law supplied by the model."
    ),
)

#: Every template the package registers at import, in registration order.
BUILTINS: tuple[Template, ...] = (
    DOUBLE_WISHBONE,
    STEERING_GUIDED,
    STEERING_FIXED,
    CHASSIS,
    WHEEL,
)


def register_builtins() -> tuple[Template, ...]:
    """
    Register the built-in templates, idempotently.

    Called at import so the registry always holds the built-ins; safe to call
    again, because a second call replaces each one with the same object.
    """
    return tuple(register(template, replace=True) for template in BUILTINS)
