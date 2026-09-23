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
from typing import TYPE_CHECKING, Literal

import numpy as np

from ..preparation.assembly.types import Constraint, RigidBody
from ..preparation.geometry import SE3
from ..schema import Vec3
from .geometry import local_point, local_pose, lookup_hardpoint, mirror_point

if TYPE_CHECKING:
    from ..schema import FrontAxleModel, RigidBodySpec

__all__ = [
    "DEFAULT_AXLE_SUBSYSTEMS",
    "ELEMENT_KINDS",
    "MODES",
    "SIDES",
    "SUBSYSTEM_ROLES",
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

    def carries(self, role: str) -> bool:
        """Return whether this assembly carries a subsystem role."""
        return role in self.subsystems


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
