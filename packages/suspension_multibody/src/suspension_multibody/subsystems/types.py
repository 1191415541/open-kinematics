"""
What a subsystem hands back to the assembly.

A subsystem does not construct force elements: the assembly layer owns that, and
`legacy_surface_gate` only tolerates the existing registered importers.  So a
subsystem returns *declarations* -- bodies, points, connections, constraints,
element rows -- and the assembly turns them into runtime objects.

Each piece is additive: a subsystem contributes its own slice and the assembly
concatenates the slices in the order it always has, so splitting the build apart
cannot reorder anything.

`Connection` is defined here rather than in `front_axle`, and `front_axle`
re-exports it under the same name.  The subsystems have to build connections, and
a subsystem that imported `front_axle` would close a cycle: `front_axle` imports
this package.  The class is a plain frozen record, so this only moves where it is
defined, not what it does.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Literal, Mapping

import numpy as np

from ..modeling.primitives.joints import Constraint, RigidBody
from ..modeling.primitives.spatial import SE3
from ..schema import Vec3
from .geometry import local_point, local_pose, lookup_hardpoint, mirror_point

if TYPE_CHECKING:
    from ..schema import FrontAxleModel, RigidBodySpec

__all__ = [
    "DEFAULT_AXLE_SUBSYSTEMS",
    "DEFAULT_VEHICLE_SUBSYSTEMS",
    "ELEMENT_KINDS",
    "MODES",
    "SIDES",
    "SUBSYSTEM_ROLES",
    "WHEELS",
    "AssemblyRequest",
    "Connection",
    "ResolvedElement",
    "SubsystemContext",
    "SubsystemOutput",
    "merge_outputs",
]

#: The two sides a symmetric axle generates.
Side = Literal["L", "R"]
SIDES: tuple[Side, Side] = ("L", "R")

#: K uses the joint column, C the bushing column.
Mode = Literal["K", "C"]
MODES: tuple[Mode, Mode] = ("K", "C")

#: The six subsystem roles a single-axle assembly can carry.  `wheel` rather
#: than `tire`: the wheel subsystem covers both the wheel body and the tire.
SUBSYSTEM_ROLES: frozenset[str] = frozenset(
    {"suspension", "steering", "wheel", "chassis", "brake", "drive"}
)

#: The subsystem set a single-axle suspension assembly carries by default.  It
#: has no brake and no drive (requirement 17 / D8); `wheel` is present because
#: the rig supplies the wheels (D9), not because the axle builds a wheel body.
DEFAULT_AXLE_SUBSYSTEMS: frozenset[str] = frozenset(
    {"chassis", "suspension", "steering", "wheel"}
)

#: The subsystem set the full-vehicle assembly carries: all six.  Brake and drive
#: are the vehicle's own (requirement 17 / D8) and are what the simplified brake
#: and drive templates implement.
DEFAULT_VEHICLE_SUBSYSTEMS: frozenset[str] = frozenset(
    {"chassis", "suspension", "steering", "wheel", "brake", "drive"}
)

#: The four corner names a wheel torque is emitted for, in the document's own
#: order.  This mirrors `_WHEEL_NAMES` in `preparation/vehicle_dynamic.py` (the
#: order the kernel ABI takes its per-wheel torque buffers in) and the
#: `WheelSpec.name` literal; a torque map keyed in another order would still
#: compare equal as a dict, so the order is written down here rather than left to
#: whichever loop happens to build it.
WHEELS: tuple[str, str, str, str] = (
    "front_left",
    "front_right",
    "rear_left",
    "rear_right",
)

#: Element declarations a subsystem may return.  `front_axle` maps each kind to
#: its constructor; a new kind is a new branch there, not a new import here.
ELEMENT_KINDS = (
    "spring",
    "damper",
    "bump_stop",
    "anti_roll_bar",
    "tire",
    "bushing",
)


@dataclass(frozen=True)
class Connection:
    """Stable physical connection identifier."""

    name: str
    kind: Literal["ideal", "bushing"]
    body_a: str
    body_b: str
    point_a: str
    point_b: str


@dataclass(frozen=True)
class AssemblyRequest:
    """
    What a caller asks an axle assembly to contain.

    Absence is declared *here*, at assembly time, rather than by adding a field
    to `FrontAxleModel`: `api.py` hashes `model.model_dump(mode="json")` into
    `Provenance.model_hash`, so a dump-visible flag would change every existing
    model hash and every recorded result byte.  Requirement 15 only needs "this
    assembly has no steering subsystem", and that is a property of the assembly,
    not of the geometry.
    """

    mode: Mode = "K"
    #: Which of the six roles this assembly carries.  The default is the
    #: single-axle set, which is what every existing caller gets.
    subsystems: frozenset[str] = DEFAULT_AXLE_SUBSYSTEMS
    #: The suspension template to build, either an instantiation or a name.
    #:
    #: A *name* is the user-facing form: an expert registers a template and a
    #: caller selects it by name, which is what "choose a template" means to
    #: somebody who did not write it.  An already-instantiated template is also
    #: accepted, because a caller that resolved its own properties should not have
    #: to hand back a name and have them re-resolved.
    suspension_template: object | None = None
    #: The steering and chassis templates to build, in the same two forms
    #: `suspension_template` accepts: a registered name, or an already-resolved
    #: instantiation.
    #:
    #: A role that names none gets that role's own default -- the registered
    #: built-in its subsystem already selects.  They are separate fields rather
    #: than one mapping so a caller cannot put a suspension template under
    #: `steering` without anything noticing, which a mapping would accept
    #: silently.
    #:
    #: The wheel role is absent on purpose: its wheel centre is a per-side mount
    #: and the role owns no body on an axle, so the file format has no body to
    #: hang one on -- see `subsystems/wheel.py`.
    steering_template: object | None = None
    chassis_template: object | None = None

    def carries(self, role: str) -> bool:
        """Return whether this assembly carries a subsystem role."""
        return role in self.subsystems

    @property
    def instantiated_suspension(self) -> object:
        """
        Return the suspension instantiation, resolving a name or the default.

        Three inputs reach here and all three end in one instance, so the rest of
        the package never has to ask which form a caller used.
        """
        from ..templates import DOUBLE_WISHBONE, instantiate

        if self.suspension_template is not None:
            return _as_instance(self.suspension_template, mode=self.mode)
        return instantiate(
            DOUBLE_WISHBONE,
            mode=self.mode,
            properties=self._model_properties(),
        )

    def role_instance(self, role: str) -> object | None:
        """
        Return the template instance a caller supplied for one role, or `None`.

        `None` means "that role's own default", which is the registered built-in
        the role's subsystem already selects.  It is deliberately *not* resolved
        here: for steering the default depends on the model
        (`rack_fixed_to_chassis`), and a request that guessed would be choosing a
        topology the model did not ask for.
        """
        selection = getattr(self, _role_template_field(role), None)
        if selection is None:
            return None
        return _as_instance(
            selection,
            mode=self.mode,
            role=role,
            properties={} if role != "suspension" else self._model_properties(),
        )

    def _model_properties(self) -> dict[str, float]:
        """Return the property values the default instantiation needs."""
        # The spring and damper slots are model-owned today; a zero keeps the
        # instantiation constructible without claiming a stiffness nobody set.
        return {"spring": 0.0, "damper": 0.0}

    def mount_stiffness(self) -> np.ndarray:
        """
        Return the 6x6 stiffness the assembly's compliant mounts carry.

        Asked of the *instance* rather than read from a slot by name: which slot
        feeds a bushing column is the template's declaration, and a file is free to
        call that slot anything.  Reading ``properties["bushing"]`` here would make
        the built-in template's slot name load-bearing and turn a differently named
        mount into a silent zero -- a compliant assembly that is a mechanism.

        A zero-stiffness matrix means the slot is declared but carries no
        compliance, which is the built-in template's state.  A file that states the
        whole table gets it as written: a mount's rotational diagonals are what a
        single number cannot express.
        """
        instance = self.instantiated_suspension
        active: tuple[str, ...] = tuple(getattr(instance, "bushings", ()))
        if not active:
            return np.zeros((6, 6))
        return instance.bushing_stiffness_matrix(active[0])


#: The field each role's template selection travels in.  Named once because the
#: accessor and the resolver have to agree on the spelling, and a role added to one
#: without the other would resolve to `None` -- "the default" -- silently.
_ROLE_TEMPLATE_FIELD: dict[str, str] = {
    "suspension": "suspension_template",
    "steering": "steering_template",
    "chassis": "chassis_template",
}


def _role_template_field(role: str) -> str:
    """Return the field one role's template selection travels in."""
    try:
        return _ROLE_TEMPLATE_FIELD[role]
    except KeyError as exc:
        raise ValueError(
            f"role {role!r} has no template selection; the roles that do are "
            f"{sorted(_ROLE_TEMPLATE_FIELD)}"
        ) from exc


def _as_instance(
    selection: object,
    *,
    mode: str,
    role: str = "suspension",
    properties: Mapping[str, float] | None = None,
) -> object:
    """
    Resolve whatever a caller put in a role's template field.

    A string is a registered name and is looked up; anything else must already be
    an instantiation, because accepting a bare ``Template`` here would silently
    skip the property resolution that decides which columns carry stiffness.
    ``properties`` is what a name-selected template is instantiated with: the
    suspension's model-owned spring and damper, and nothing for a role whose slots
    carry their own defaults.
    """
    from ..templates import instantiate
    from ..templates.instantiate import SubsystemInstance
    from ..templates.registry import get as get_template

    supplied = {"spring": 0.0, "damper": 0.0} if properties is None else dict(properties)
    if isinstance(selection, str):
        return instantiate(get_template(selection), mode=mode, properties=supplied)
    if isinstance(selection, SubsystemInstance):
        if selection.mode != mode:
            raise ValueError(
                f"the {role} template was instantiated for mode "
                f"{selection.mode!r} but this assembly is mode {mode!r}; the mode "
                "belongs to the instance, so resolve it for the mode you mean"
            )
        return selection
    raise TypeError(
        f"{role}_template must be a registered template name or a "
        f"SubsystemInstance, got {type(selection).__name__}"
    )


@dataclass
class SubsystemContext:
    """
    The shared state a subsystem reads and writes while contributing.

    Mutable on purpose: the axle is built in one ordered pass, and a later
    subsystem needs the bodies an earlier one emitted (steering's tie rod joints
    reach the uprights the suspension subsystem created).  The assembly owns the
    pass order; a subsystem only ever appends to `bodies`/`points`/`hardpoints`.
    """

    model: FrontAxleModel
    request: AssemblyRequest
    #: Bodies emitted so far, in emission order.
    bodies: dict[str, RigidBody] = field(default_factory=dict)
    #: Body-local points, keyed `(body, label)`, in emission order.
    points: dict[tuple[str, str], np.ndarray] = field(default_factory=dict)
    #: Assembly hardpoints: the model's own plus generated copies.
    hardpoints: dict[str, Vec3] = field(default_factory=dict)
    #: Per-side hardpoints as schema values, for alias lookup.
    side_schema: dict[str, dict[str, Vec3]] = field(default_factory=dict)

    @property
    def mode(self) -> Mode:
        """Return the K/C mode this assembly is being built in."""
        return self.request.mode

    @property
    def mount_bushing_stiffness(self) -> np.ndarray:
        """
        Return the 6x6 stiffness the template's mount bushing slot resolves to.

        A subsystem asks for the *number* and never decides it: the template (or,
        from subtask 06 on, a properties file) owns it.  The built-in template
        declares zero, which is what keeps the frozen C snapshot valid; a template
        with real compliance is a different template, substituted by name.
        """
        return self.request.mount_stiffness()

    @property
    def body_specs(self) -> dict[str, RigidBodySpec]:
        """Return schema body specs by name, for mass properties."""
        return {spec.name: spec for spec in self.model.bodies}

    def lookup(self, side: Side, role: str) -> Vec3:
        """Resolve a hardpoint role on one side through its alias list."""
        return lookup_hardpoint(self.side_schema[side], role)

    def point(self, side: Side, role: str) -> np.ndarray:
        """Resolve a hardpoint role on one side as a global array."""
        return self.lookup(side, role).as_array()

    def mirror(self, side: Side, role: str) -> np.ndarray:
        """Resolve a role from the model's own hardpoints, mirrored to `side`."""
        return mirror_point(lookup_hardpoint(self.model.hardpoints, role), side)

    def attachment_point(self, value: object, side: Side) -> np.ndarray:
        """Return a schema attachment point in the requested side's coordinates."""
        # Springs, dampers and stops carry a bare `Vec3`; a bushing carries a
        # `Pose`, whose translation is the attachment point.
        point = value if isinstance(value, Vec3) else value.translation  # type: ignore[attr-defined]
        return mirror_point(point, side)

    def local(self, body: str, point_global: np.ndarray) -> np.ndarray:
        """Convert a global hardpoint into `body`-local coordinates."""
        return local_point(self.bodies, body, point_global)

    def local_attachment(self, value: object, side: Side, body: str) -> SE3:
        """Convert a schema attachment pose from vehicle to body coordinates."""
        return local_pose(value, side, body, self.bodies)  # type: ignore[arg-type]

    def placeholder_pose(
        self, global_point: np.ndarray, local_point_: np.ndarray
    ) -> tuple[SE3, SE3]:
        """
        Return the two identity-rotation poses a C-mode slot placeholder needs.

        The original build writes the chassis point and the arm point with an
        explicit identity quaternion rather than a converted frame, so the
        placeholder is reproduced here exactly rather than via `local_attachment`.
        """
        quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        return (
            SE3(translation=global_point, quaternion=quaternion),
            SE3(translation=local_point_, quaternion=quaternion),
        )


@dataclass
class SubsystemOutput:
    """
    One subsystem's contribution to an axle assembly.

    `bodies` and `points` are ordered mappings because their order is part of the
    assembly's contract: the contract document lists bodies in this order, and
    the recorded point order is part of what the assembly produces.  A subsystem
    appends; only the assembly decides the sequence.
    """

    #: Bodies this subsystem introduces, in the order it wants them listed.
    bodies: dict[str, RigidBody] = field(default_factory=dict)
    #: Body-local points, keyed `(body, label)`.
    points: dict[tuple[str, str], np.ndarray] = field(default_factory=dict)
    #: Extra hardpoints the subsystem wants visible on the assembly.
    hardpoints: dict[str, object] = field(default_factory=dict)
    #: Attachment descriptions.
    connections: list[Connection] = field(default_factory=list)
    #: Constraints that are active in this mode.
    constraints: list[Constraint] = field(default_factory=list)
    #: Constraints that are ideal in this mode (a superset of `constraints` in C).
    ideal_constraints: list[Constraint] = field(default_factory=list)
    #: C-mode slot placeholders, which the original build appends last.
    bushings: list[ResolvedElement] = field(default_factory=list)
    #: Elastic element rows the assembly builds, in emission order.
    elements: list[ResolvedElement] = field(default_factory=list)


@dataclass(frozen=True)
class ResolvedElement:
    """
    One elastic element a subsystem wants built, with its geometry resolved.

    The subsystem decides what belongs where and computes body-local points;
    `front_axle` owns the constructor call, because it is the module the legacy
    surface gate has registered as an `elements` importer.
    """

    #: One of `ELEMENT_KINDS`.
    kind: str
    #: Stable element name.
    name: str
    #: The schema spec carrying the parameters.
    spec: object
    body_a: str | None = None
    point_a: np.ndarray | None = None
    body_b: str | None = None
    point_b: np.ndarray | None = None
    local_pose_a: object | None = None
    local_pose_b: object | None = None


def merge_outputs(*outputs: SubsystemOutput) -> SubsystemOutput:
    """
    Concatenate subsystem contributions, preserving each one's internal order.

    Bodies are merged by key so a later subsystem can refine an earlier body (the
    assembly gives the chassis its mass specs at the end), while lists are
    appended in argument order.
    """
    merged = SubsystemOutput()
    for output in outputs:
        merged.bodies.update(output.bodies)
        merged.points.update(output.points)
        merged.hardpoints.update(output.hardpoints)
        merged.connections.extend(output.connections)
        merged.constraints.extend(output.constraints)
        merged.ideal_constraints.extend(output.ideal_constraints)
        merged.bushings.extend(output.bushings)
        merged.elements.extend(output.elements)
    return merged
