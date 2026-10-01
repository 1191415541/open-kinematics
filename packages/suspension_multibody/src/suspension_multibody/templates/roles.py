"""
The seven subsystem roles, and what each one promises.

A *role* is the interface a subsystem is assembled through; a *template* is one
implementation of it.  Keeping them apart is what lets a simplified brake
template (no bodies, just a torque) be replaced later by a detailed one (calipers
and rotors) without touching the assembly layer or the rigs.

The division matters because the two facts "what a subsystem must provide" and
"what a subsystem happens to contain" were previously the same fact, written out
in the assembly function.  A role says only the first.

A role deliberately does **not** express availability.  "The steering subsystem
may be absent from a single-axle assembly" and "brake and drive exist only on the
full vehicle" are decisions about an *assembly*, not about a role, so they live
with the assembly declarations and not here.
"""

from __future__ import annotations

from dataclasses import dataclass

__all__ = [
    "ROLES",
    "RoleSpec",
    "RoleSpecError",
    "get_role",
    "role_names",
]


class RoleSpecError(ValueError):
    """A role name is unknown, or a role declaration is inconsistent."""


@dataclass(frozen=True)
class RoleSpec:
    """
    What one subsystem role promises the assembly.

    `required_mounts` is the set of hardpoint roles the template must name: the
    geometry the assembly attaches things to.  `required_slots` is the set of
    property slots that must be filled before instantiation.  `outputs` names the
    minimum-unit outputs the role contributes.  `has_torque_channel` says whether
    the role feeds a torque to the wheels -- true for brake and drive only, which
    is how a rig knows whether to expect one.

    None of these fields describes *how* the template is built.  There is no
    `body_count`, no `has_bodies`, no `is_simple`: those are implementation
    choices, and putting them here would make the simplified templates
    un-replaceable.
    """

    #: The role's name, matching the `role` field of the templates that use it.
    name: str
    #: Hardpoint roles the template must declare.
    required_mounts: tuple[str, ...] = ()
    #: Property slots the template must declare and the properties file must fill.
    required_slots: tuple[str, ...] = ()
    #: Minimum-unit outputs the role contributes.
    outputs: tuple[str, ...] = ()
    #: Whether the role feeds a wheel torque.
    has_torque_channel: bool = False
    #: Free-text note about the role's place in the architecture.
    note: str = ""


#: The seven roles, keyed by name.  See `EPIC.md` G2 for the availability matrix
#: (which roles each assembly carries).
ROLES: dict[str, RoleSpec] = {
    "suspension": RoleSpec(
        name="suspension",
        required_mounts=(
            "upper_front",
            "upper_rear",
            "upper_outer",
            "lower_front",
            "lower_rear",
            "lower_outer",
            "wheel_center",
            "tie_inner",
            "tie_outer",
        ),
        required_slots=("spring", "damper", "bushing"),
        outputs=("wheel_travel", "camber", "toe", "track_change"),
        note=(
            "The left/right suspension pair including arms, uprights, tie rods "
            "and wheel hubs with wheel spin joints."
        ),
    ),
    "steering": RoleSpec(
        name="steering",
        required_mounts=("rack_center", "tie_inner"),
        required_slots=("rack_axis",),
        outputs=("rack_displacement",),
        note=(
            "The steering mechanism: rack and rack housing. Tie rods belong to "
            "the suspension subsystem."
        ),
    ),
    "wheel": RoleSpec(
        name="wheel",
        required_mounts=("wheel_center", "spin_axis"),
        required_slots=("unloaded_radius", "wheel_mass", "wheel_inertia", "tire"),
        outputs=("wheel_center_pose", "tire_force"),
        note=(
            "Carries both the wheel body and the tire.  On a single-axle "
            "assembly the wheel is supplied by the rig rather than by the "
            "assembly (user decision D9), but it is declared through this same "
            "role either way."
        ),
    ),
    "chassis": RoleSpec(
        name="chassis",
        required_mounts=("chassis_reference",),
        required_slots=("chassis_mass", "chassis_inertia"),
        outputs=("chassis_pose",),
        note=(
            "The vehicle body. Only the full-vehicle assembly carries a chassis."
        ),
    ),
    "brake": RoleSpec(
        name="brake",
        required_mounts=("wheel_center", "spin_axis"),
        required_slots=(
            "piston_area",
            "effective_radius",
            "friction_coeff",
            "rotor_inertia",
        ),
        outputs=("brake_torque",),
        has_torque_channel=True,
        note=(
            "Only the full-vehicle assembly carries a brake (user decision D8).  "
            "The simplified template owns no bodies and emits one torque element "
            "per wheel; a detailed template may add calipers and rotors under "
            "this same role.  The slot set is the standardized one of the "
            "roadmap's 2.1 section: the source document's own demand scaling is "
            "a module constant, and the front/rear split is no longer a slot "
            "because one element per wheel states its own share."
        ),
    ),
    "drive": RoleSpec(
        name="drive",
        required_mounts=("wheel_center", "spin_axis"),
        required_slots=("gear_ratio", "efficiency", "max_torque"),
        outputs=("drive_torque",),
        has_torque_channel=True,
        note=(
            "Only the full-vehicle assembly carries a drive (user decision D8).  "
            "The simplified template emits one torque element per driven wheel; a "
            "detailed template may add powertrain bodies and a differential under "
            "this same role.  `driven_wheels`/`drive_split` are no longer slots: "
            "with one element per wheel a wheel is driven because it has an "
            "element, and `DrivelineSpec` still owns the split's validation."
        ),
    ),
    "anti_roll_bar": RoleSpec(
        name="anti_roll_bar",
        required_mounts=(
            "chassis_mount_L",
            "chassis_mount_R",
            "droplink_mount_L",
            "droplink_mount_R",
        ),
        required_slots=("torsional_stiffness",),
        outputs=("anti_roll_torque",),
        note=(
            "The anti-roll bar is a subsystem of its own rather than a pair of "
            "elements on the suspension: it spans both sides of one axle, so "
            "neither side's template can own it.  The simplified template carries "
            "a torsion bar and one droplink per side and states its four mounts.  "
            "`has_torque_channel` stays false: the bar feeds no wheel torque."
        ),
    ),
}


def _check_roles() -> None:
    """Reject a role table that cannot be trusted, where it is written."""
    expected = {
        "suspension",
        "steering",
        "wheel",
        "chassis",
        "brake",
        "drive",
        "anti_roll_bar",
    }
    if set(ROLES) != expected:
        missing = sorted(expected - set(ROLES))
        extra = sorted(set(ROLES) - expected)
        raise RoleSpecError(f"role table mismatch; missing={missing} extra={extra}")
    for name, spec in ROLES.items():
        if spec.name != name:
            raise RoleSpecError(f"role {name!r} carries the wrong name {spec.name!r}")
        implementation_fields = {"body_count", "has_bodies", "is_simple", "bodies"}
        leaked = implementation_fields & set(vars(spec))
        if leaked:
            raise RoleSpecError(
                f"role {name!r} leaks implementation detail {sorted(leaked)}; "
                "whether a template has bodies is the template's business"
            )
    torque_roles = {name for name, spec in ROLES.items() if spec.has_torque_channel}
    if torque_roles != {"brake", "drive"}:
        raise RoleSpecError(
            f"only brake and drive carry a torque channel; found {sorted(torque_roles)}"
        )


_check_roles()


def get_role(name: str) -> RoleSpec:
    """Return a role by name, naming the unknown one if it is not registered."""
    try:
        return ROLES[name]
    except KeyError as exc:
        known = ", ".join(sorted(ROLES))
        raise RoleSpecError(
            f"unknown subsystem role {name!r}; known roles are {known}"
        ) from exc


def role_names() -> tuple[str, ...]:
    """Return the seven role names in a stable order."""
    return tuple(sorted(ROLES))
