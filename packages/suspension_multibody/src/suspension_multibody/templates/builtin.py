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
    "BRAKE",
    "BUILTINS",
    "CHASSIS",
    "DEFAULT_MOUNT_STIFFNESS",
    "DOUBLE_WISHBONE",
    "DOUBLE_WISHBONE_NAME",
    "DRIVE",
    "STEERING",
    "STEERING_GUIDED",
    "RACK_HOUSING_MASS",
    "VEHICLE_BODY",
    "WHEEL",
    "WHEEL_HUB_MASS",
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

#: The wheel hub's mass in kg, and where the number comes from.
#:
#: 方式 A makes the hub a body of its own -- one revolute joint along `spin_axis`
#: gives the wheel its spin -- and no model declares a spec for it, because the
#: template is what puts it there.  The template therefore states its mass, and the
#: value is the recorded source model's own spindle part rather than a number chosen
#: here: `TR_Front_Suspension.ges_spindle` is at `MASS = 1.102840393` in
#: `artifacts/adams/correlation-reference-real/handling/double_lane_change/adams_raw/
#: handling_double_lane_change_dynamic.adm`, and the hub plays that part in this
#: topology.  It must be positive: a free body with no mass has no dynamic reading at
#: all, so a hub the model says nothing about would otherwise stop every dynamic run.
WHEEL_HUB_MASS = 1.102840393

#: The rack housing's mass in kg, from the same source (`TR_Steering.ges_rack_housing`,
#: `MASS = 4`).  The housing is the steering's own body: the rack slides in it, and it
#: mounts to the chassis (or to ground when the assembly has none), so it weighs
#: something in every reading that integrates the model.
RACK_HOUSING_MASS = 4.0

#: The template name the assembly layer will refer to.
DOUBLE_WISHBONE_NAME = "double_wishbone"

#: The bodies the template declares.  Both sides are in one template because the
#: anti-roll bar spans them and cannot belong to a one-sided subsystem.
#:
#: Only the two parts the template *invents* carry a mass of their own: the arms,
#: the upright and the tie rod are the parts a model describes, and weighing them
#: here would answer a question the model is entitled to answer instead.  The hub is
#: the exception, and it has to be (see :data:`WHEEL_HUB_MASS`).
_PARTS: tuple[PartDefinition, ...] = (
    PartDefinition("upper_arm_L"),
    PartDefinition("lower_arm_L"),
    PartDefinition("upright_L"),
    PartDefinition("tie_rod_L"),
    PartDefinition("wheel_hub_L", mass=WHEEL_HUB_MASS),
    PartDefinition("upper_arm_R"),
    PartDefinition("lower_arm_R"),
    PartDefinition("upright_R"),
    PartDefinition("tie_rod_R"),
    PartDefinition("wheel_hub_R", mass=WHEEL_HUB_MASS),
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
    # 方式 A's wheel spin joint: the suspension's own hub turns on the upright along
    # the wheel's axis, which is the *wheel's* to state (`WheelSpec.spin_axis`, whose
    # default is the lateral axis this assembly builds) rather than a hardpoint of
    # this template -- a template cannot name an axis it does not place.
    ConnectionDefinition(
        "wheel_spin_joint_L",
        "wheel_center",
        "revolute",
        owner="wheel_hub_L",
        label="center",
        far_owner="upright_L",
        far_label="spindle",
    ),
    ConnectionDefinition(
        "wheel_spin_joint_R",
        "wheel_center",
        "revolute",
        owner="wheel_hub_R",
        label="center",
        far_owner="upright_R",
        far_label="spindle",
    ),
    ConnectionDefinition(
        "wheel_center_L", "wheel_center", owner="wheel_hub_L", label="wheel_center"
    ),
    ConnectionDefinition(
        "wheel_center_R", "wheel_center", owner="wheel_hub_R", label="wheel_center"
    ),
    # The rack's centre: the *steering* template's mount, declared here too because
    # the axle's model carries it as one of its own reference points (RACK_CENTER)
    # and a file subsystem may only place hardpoints its template declares.  It is
    # the same pattern the brake and drive mounts follow -- a role declares the mount
    # another role's body carries -- and it is a declaration with no column, so it
    # constrains nothing and belongs to nobody here.
    ConnectionDefinition("rack_center_L", "rack_center"),
)

#: Property slots the template asks a properties file to fill.  The defaults are
#: the simplified template's own numbers: a template may carry its values
#: directly, which is what the current assembly does.
_PROPERTY_SLOTS: tuple[PropertySlot, ...] = (
    PropertySlot("spring", "N/m"),
    PropertySlot("damper", "N*s/m"),
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
    PropertySlot("wheel_center", "-"),
    PropertySlot("spin_axis", "-"),
    PropertySlot("upper_front", "-"),
    PropertySlot("upper_rear", "-"),
    PropertySlot("upper_outer", "-"),
    PropertySlot("lower_front", "-"),
    PropertySlot("lower_rear", "-"),
    PropertySlot("lower_outer", "-"),
    PropertySlot("tie_inner", "-"),
    PropertySlot("tie_outer", "-"),
)

#: What the template contributes to the result.
_OUTPUTS: tuple[OutputDeclaration, ...] = (
    OutputDeclaration("wheel_travel", "mm", "kernel"),
    OutputDeclaration("camber", "deg", "derived"),
    OutputDeclaration("toe", "deg", "derived"),
    OutputDeclaration("track_change", "mm", "derived"),
    OutputDeclaration("wheel_center_pose", "mm", "kernel"),
)

DOUBLE_WISHBONE = Template(
    name=DOUBLE_WISHBONE_NAME,
    role="suspension",
    parts=_PARTS,
    connections=_CONNECTIONS,
    elastic_slots=("spring", "damper", "bushing"),
    property_slots=_PROPERTY_SLOTS,
    outputs=_OUTPUTS,
    suspension_kind="double_wishbone",
    description=(
        "The built-in double-wishbone template: arms rotate on a single inboard revolute "
        "in K mode, both inboard points become ball joints backed by bushings in C "
        "mode, outer points and tie rod ends stay joints in both modes, and wheel hubs "
        "are connected to uprights via revolute spin joints."
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
    #: The housing is the steering's own support, not a part a model authors: it is
    #: what the rack slides in and what mounts to the chassis, so its mass is the
    #: template's to state (see :data:`RACK_HOUSING_MASS`).
    PartDefinition("rack_housing", mass=RACK_HOUSING_MASS),
)

_STEERING_CONNECTIONS: tuple[ConnectionDefinition, ...] = (
    ConnectionDefinition(
        "rack_guide",
        "rack_center",
        "prismatic",
        owner="rack",
        label="center",
        far_owner="rack_housing",
        far_label="rack_center",
    ),
    ConnectionDefinition(
        "housing_mount",
        "rack_center",
        "fixed",
        owner="rack_housing",
        label="mount",
        far_owner="chassis",
        far_label="rack_center",
    ),
    ConnectionDefinition(
        "rack_tie_L",
        "tie_inner",
        owner="rack",
        label="tie_L",
    ),
    ConnectionDefinition(
        "rack_tie_R",
        "tie_inner",
        owner="rack",
        label="tie_R",
    ),
)

_STEERING_SLOTS: tuple[PropertySlot, ...] = (
    PropertySlot("rack_axis", "-", default=0.0),
    PropertySlot("rack_center", "-", default=0.0),
    PropertySlot("tie_inner", "-", default=0.0),
)

_STEERING_OUTPUTS: tuple[OutputDeclaration, ...] = (
    OutputDeclaration("rack_displacement", "mm", "kernel"),
)

STEERING = Template(
    name="steering",
    role="steering",
    parts=_STEERING_PARTS,
    connections=_STEERING_CONNECTIONS,
    property_slots=_STEERING_SLOTS,
    outputs=_STEERING_OUTPUTS,
    description=(
        "A steering mechanism: rack guided along rack housing via prismatic joint. "
        "Tie rods belong to the suspension subsystem."
    ),
)

STEERING_GUIDED = STEERING

VEHICLE_BODY = Template(
    name="vehicle_body",
    role="chassis",
    parts=(PartDefinition("chassis", fixed=False),),
    connections=(
        ConnectionDefinition(
            "chassis_reference",
            "chassis_reference",
            owner="chassis",
            label="reference",
        ),
    ),
    property_slots=(
        PropertySlot("chassis_reference", "-", default=0.0),
        PropertySlot("chassis_mass", "kg", default=0.0),
        PropertySlot("chassis_inertia", "kg*m^2", default=0.0),
    ),
    outputs=(OutputDeclaration("chassis_pose", "mm", "kernel"),),
    description=(
        "The vehicle body: one rigid body carrying vehicle mass and inertia. "
        "Used only in full vehicle assemblies."
    ),
)

#: The axle's chassis role: the *fixed* support a single axle reacts against.
#:
#: An axle carries no vehicle body -- its inboard mounts hang on a fixed
#: reference, and that is what makes a K/C reading a static problem with an
#: answer.  It is deliberately not `VEHICLE_BODY`: that body is free and carries
#: the vehicle's mass, and an axle built from it has nothing to react against.
#: Measured on the file-authored axle, the static trim then stops at
#: ``force_residual = 1.0`` with the whole weight of the floating body as the
#: worst row, and every case of the run fails with status 6.
AXLE_CHASSIS = Template(
    name="axle_chassis",
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
        "The ground-side body a single axle reacts against: one fixed support. "
        "The axle side consumes no mass here, which is the recorded state rather "
        "than an omission.  A vehicle's own body is VEHICLE_BODY."
    ),
)

CHASSIS = AXLE_CHASSIS

WHEEL = Template(
    name="wheel_on_hub",
    role="wheel",
    parts=(),
    connections=(
        ConnectionDefinition(
            "wheel_center_L",
            "wheel_center",
            owner="wheel_hub_L",
            label="wheel_center",
        ),
        ConnectionDefinition("spin_axis_L", "spin_axis"),
        ConnectionDefinition(
            "wheel_center_R",
            "wheel_center",
            owner="wheel_hub_R",
            label="wheel_center",
        ),
        ConnectionDefinition("spin_axis_R", "spin_axis"),
    ),
    property_slots=(
        PropertySlot("unloaded_radius", "m", default=0.0),
        PropertySlot("wheel_mass", "kg", default=0.0),
        PropertySlot("wheel_inertia", "kg*m^2", default=0.0),
        PropertySlot("tire", "-", default=0.0),
        PropertySlot("wheel_center", "-", default=0.0),
        PropertySlot("spin_axis", "-", default=0.0),
    ),
    outputs=(
        OutputDeclaration("wheel_center_pose", "mm", "kernel"),
        OutputDeclaration("tire_force", "N", "kernel"),
    ),
    description=(
        "The wheel end attached to the wheel hub, with the tire's law supplied by the model."
    ),
)

_BRAKE_MOUNTS: tuple[ConnectionDefinition, ...] = tuple(
    ConnectionDefinition(f"{mount}_{side}", mount)
    for mount in ("wheel_center", "spin_axis")
    for side in ("L", "R")
)

BRAKE = Template(
    name="brake_4wdisk_simplified",
    role="brake",
    parts=(),
    connections=_BRAKE_MOUNTS,
    property_slots=(
        PropertySlot("brake_mu", "-", default=0.4),
        PropertySlot("piston_area", "mm^2", default=2500.0),
        PropertySlot("effective_piston_radius", "mm", default=145.0),
        PropertySlot("front_brake_bias", "-", default=0.6),
        PropertySlot("max_brake_value", "-", default=0.1),
    ),
    outputs=(OutputDeclaration("brake_torque", "N*mm", "kernel"),),
    suspension_kind="brake_4wdisk",
    description="The simplified torque-only brake template under the brake role.",
)

_DRIVE_MOUNTS: tuple[ConnectionDefinition, ...] = tuple(
    ConnectionDefinition(f"{mount}_{side}", mount)
    for mount in ("wheel_center", "spin_axis")
    for side in ("L", "R")
)

DRIVE = Template(
    name="powertrain_simplified",
    role="drive",
    parts=(),
    connections=_DRIVE_MOUNTS,
    property_slots=(
        PropertySlot("driven_wheels", "-", default=0.0),
        PropertySlot("drive_split", "-", default=0.0),
        PropertySlot("maximum_drive_torque", "N*mm", default=0.0),
    ),
    outputs=(OutputDeclaration("drive_torque", "N*mm", "kernel"),),
    suspension_kind="driveline",
    description="The simplified torque-only powertrain template under the drive role.",
)

#: Every template the package registers at import, in registration order.
BUILTINS: tuple[Template, ...] = (
    DOUBLE_WISHBONE,
    STEERING,
    VEHICLE_BODY,
    WHEEL,
    BRAKE,
    DRIVE,
)


def register_builtins() -> tuple[Template, ...]:
    """
    Register the built-in templates, idempotently.

    Called at import so the registry always holds the built-ins; safe to call
    again, because a second call replaces each one with the same object.
    """
    return tuple(register(template, replace=True) for template in BUILTINS)
