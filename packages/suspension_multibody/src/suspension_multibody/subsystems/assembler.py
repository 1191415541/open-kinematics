"""
The vehicle assembler: an entry list in, one vehicle runtime out.

Until now a vehicle was two axles *hard-coded*: the vehicle module knew that a
vehicle has a front axle and a rear axle, named them itself and gave each one a
literal prefix.  Everything tricky about merging two axles -- what to do with the
two bodies both of them call ``chassis``, how to keep two flat namespaces apart --
was therefore written down twice: once as a rule, once as the assumption that
there are exactly two.

This module keeps the mechanisms and drops the assumption.  A caller hands in a
list of :class:`AxleEntry`; each entry says where its axle sits, which prefix its
entities carry, which of its bodies are taken over by the vehicle's own bodies,
and which wheels it owns.  Nothing here knows how many entries there are, what a
placement is called or which one is the "front" one -- the same code assembles
two axles or five, and a placement named ``middle`` is no different from any
other.

``compose_vehicle_runtime`` in ``subsystems/vehicle_assembly.py`` is the
adapter that turns a :class:`~suspension_multibody.schema.VehicleModel` into such
an entry list, so the historical model keeps working while the assembler stops
being about it.

Two details are load-bearing and therefore stated rather than left implicit:

**The override is a declaration, not a name rule.**  Both axles declare a chassis
body, and the vehicle has exactly one.  Which axle bodies the vehicle's own
chassis body takes over is ``AxleEntry.replace_bodies``; the older code decided
it by comparing strings against ``"chassis"``/``"ground"``, which meant a document
could not name its chassis anything else.

**The vehicle owns the wheel ends.**  An axle composed inside a vehicle is
composed *without* the wheel role (``_AXLE_ROLES_IN_A_VEHICLE``): the wheel
subsystem's contribution **is** the wheel end, and the vehicle's own wheel ends
are the ones its entries name.  Saying that as a role rather than deleting the
axle's tires afterwards is what removed the filter that used to stand here --
same physics, but decided by what an assembly *carries* instead of by a type
test on the vertical tire element at the assembly stage.  The removed element
class is named in `subsystems/element_build.py`, which still builds it; this
module names it nowhere, so a grep for it here can only mean it came back.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING, Any, Literal

import numpy as np

from ..connections.policy import check_root
from ..modeling.primitives import (
    Constraint,
    RevoluteJoint,
    RigidBody,
    RigidBodyState,
    WeldJoint,
)
from ..schema import FrontAxleModel, UnitSystem, WheelSpec
from .capabilities import capabilities_for
from .runtime import SubsystemRuntime
from .si_assembly import si_assembly_for_axle
from .types import (
    DEFAULT_AXLE_SUBSYSTEMS,
    DEFAULT_VEHICLE_SUBSYSTEMS,
    AssemblyRequest,
    Connection,
)
from .vehicle_parts import (
    _add_wheel,
    _condense_welded_bodies,
    _drop_isolated_bodies,
    _rename_connections,
    _rename_dataclasses,
)

if TYPE_CHECKING:  # pragma: no cover - annotation only, keeps the import acyclic
    from .vehicle_assembly import VehicleRuntime

__all__ = [
    "ArticulationSpec",
    "AxleEntry",
    "VehicleFacts",
    "compose_entries_runtime",
    "runtime_for_study",
    "vehicle_facts",
]


#: The axle's own role vocabulary, minus the role the vehicle owns.
#:
#: The wheel role's entire contribution is the wheel end -- the wheel body and
#: the tire -- and for a vehicle those are the vehicle's, named by its entries.
#: Composing an axle with the role would have the axle describe a wheel end that
#: the vehicle then has to un-describe, which is what the deleted type filter
#: was doing.
_AXLE_ROLES_IN_A_VEHICLE: frozenset[str] = DEFAULT_AXLE_SUBSYSTEMS - {"wheel"}

@dataclass(frozen=True)
class AxleEntry:
    """
    One axle's place in a vehicle assembly, as the caller declares it.

    ``placement`` names the place the axle occupies and keys the per-axle runtimes
    on the result; the assembler never interprets it.  ``prefix`` is the qualifier
    every body, point, constraint, ideal constraint, element and connection of
    that axle gets, because the vehicle lists one flat namespace and two axles
    both declare a body called ``upright_L``.

    ``replace_bodies`` says which of *that axle's* bodies do not survive as its
    own: each key is an axle body name and each value the vehicle body that takes
    its place -- both axles hand their ``chassis``/``ground`` over to the
    vehicle's single chassis body.  A body that is declared but not overridden
    keeps its own identity under the prefix.

    ``wheels`` are the wheel ends the entry owns, in the order they are attached.

    ``sides`` are the sides of *this* axle when it does not carry the vehicle's
    own pair: a three-wheeled vehicle is two suspensions and one corner, and the
    corner is the entry that says so.  `None` means the entry carries whatever the
    vehicle declares, which is what every entry did before this field existed.
    """

    placement: str
    prefix: str
    axle: FrontAxleModel
    replace_bodies: Mapping[str, str]
    wheels: tuple[WheelSpec, ...]
    sides: tuple[str, ...] | None = None


@dataclass(frozen=True)
class ArticulationSpec:
    """
    One joint between two bodies of the assembled vehicle, stated by name.

    A vehicle built from entries may carry more than one body-level unit -- a
    tractor and a trailer, say -- and the joint between them is neither an axle's
    nor the vehicle's: it belongs to the *pair*.  It is stated here, by the names
    those bodies carry in the finished assembly, so the joining is a declaration
    rather than something derived from a name or a placement.

    Only joint kinds that already exist are built.  A new kind of joint is a
    kernel question, and inventing one here would put a constraint the solver has
    never seen on a model that then fails somewhere further down.

    ``point_a_local``/``point_b_local`` are local to the named bodies, which is
    the same convention every other entity in the assembly uses.  A ``revolute``
    also needs an axis per body, because a relative rotation is not readable at
    this level: the assembler sees points, not poses.
    """

    kind: Literal["weld", "revolute"]
    body_a: str
    body_b: str
    point_a_local: Any = (0.0, 0.0, 0.0)
    point_b_local: Any = (0.0, 0.0, 0.0)
    axis_a: Any = (0.0, 0.0, 1.0)
    axis_b: Any = (0.0, 0.0, 1.0)
    name: str = ""

    def __post_init__(self) -> None:
        if self.kind not in ("weld", "revolute"):
            raise ValueError(
                f"articulation {self.name or self.body_a!r} has an unknown kind "
                f"{self.kind!r}; the kinds are weld and revolute, because those "
                "are the joints the assembly already knows how to build"
            )
        if not self.body_a or not self.body_b:
            raise ValueError("an articulation names the two bodies it joins")
        if self.body_a == self.body_b:
            raise ValueError(
                f"articulation {self.name or self.body_a!r} joins {self.body_a!r} "
                "to itself; a joint between a body and itself says nothing"
            )
@dataclass(frozen=True)
class VehicleFacts:
    """
    What the *preparation* layer needs to know about the model it is preparing.

    A reader of the assembled vehicle -- the dynamics preparation above all -- used
    to answer three questions by walking ``VehicleModel.front_axle`` and
    ``VehicleModel.rear_axle`` itself: which axles exist, whether any of them
    declares physical bushings (which decides K or C), and whether the steering
    rack is bolted to the body (a topology the native solver does not support on a
    trailing axle).  Those are facts about the *assembly*, not about a reader, and
    stating them here is what lets a preparation stop reaching for a two-axle
    attribute.

    ``axles`` is the placement names, in assembly order; ``placements_with_bushings``
    is the subset whose model declares compliance; ``rack_fixed_to_chassis`` maps a
    placement to whether its rack is bolted.  A preparation that needs a different
    fact asks for it here rather than re-deriving it, so "which axles are there" has
    one answer.

    ``axle_units`` is the unit system each placed axle declares, keyed by placement,
    and ``static_rotation_axes`` is every static-only axis an axle body declares,
    already named the way the assembly names that body.  The last one is a *fact
    about the assembly*, not about the model: an entry's prefix and the bodies the
    vehicle takes over both decide what that body ends up being called, and a reader
    that re-derived the name would be re-implementing the merge it is reading.
    """

    axles: tuple[str, ...]
    placements_with_bushings: frozenset[str] = frozenset()
    rack_fixed_to_chassis: Mapping[str, bool] = field(default_factory=dict)
    axle_units: Mapping[str, UnitSystem] = field(default_factory=dict)
    static_rotation_axes: tuple[tuple[str, tuple[float, float, float]], ...] = ()

    @property
    def has_physical_bushings(self) -> bool:
        """Return whether any axle of this assembly declares compliance."""
        return bool(self.placements_with_bushings)


def _placed_body_name(entry: AxleEntry, body: str) -> str:
    """Return the name one axle body ends up carrying in the vehicle."""
    return entry.replace_bodies.get(body, f"{entry.prefix}{body}")


def vehicle_facts(entries: Sequence[AxleEntry]) -> VehicleFacts:
    """Return the facts a preparation reads, derived from the entry list."""
    return VehicleFacts(
        axles=tuple(entry.placement for entry in entries),
        placements_with_bushings=frozenset(
            entry.placement for entry in entries if entry.axle.bushings
        ),
        rack_fixed_to_chassis={
            entry.placement: bool(entry.axle.rack_fixed_to_chassis)
            for entry in entries
        },
        axle_units={entry.placement: entry.axle.units for entry in entries},
        static_rotation_axes=tuple(
            (_placed_body_name(entry, body.name), tuple(body.static_rotation_axis_local.as_tuple()))
            for entry in entries
            for body in entry.axle.bodies
            if body.static_rotation_axis_local is not None
        ),
    )


def _articulation_rows(
    spec: ArticulationSpec, bodies: Mapping[str, RigidBody]
) -> tuple[Constraint, ...]:
    """
    Build the constraint one articulation states, checking both ends exist.

    The names are the *finished* assembly's, so an articulation naming a body no
    entry produced is refused here rather than reaching the solver as a joint on a
    body that is not there.
    """
    missing = sorted(name for name in (spec.body_a, spec.body_b) if name not in bodies)
    if missing:
        raise ValueError(
            f"articulation {spec.name or spec.body_a!r} names body(ies) the "
            f"assembly does not carry: {missing}; the bodies are "
            f"{sorted(bodies)}"
        )
    point_a = np.asarray(spec.point_a_local, dtype=float)
    point_b = np.asarray(spec.point_b_local, dtype=float)
    if spec.kind == "weld":
        row: Constraint = WeldJoint(
            spec.body_a,
            point_a,
            spec.body_b,
            point_b,
            name=spec.name or f"articulation_{spec.body_b}",
        )
    else:
        row = RevoluteJoint(
            spec.body_a,
            point_a,
            np.asarray(spec.axis_a, dtype=float),
            spec.body_b,
            point_b,
            np.asarray(spec.axis_b, dtype=float),
            name=spec.name or f"articulation_{spec.body_b}",
        )
    return (row,)


def compose_entries_runtime(
    entries: Sequence[AxleEntry],
    *,
    chassis_name: str,
    mode: Literal["K", "C"] = "K",
    request: AssemblyRequest | None = None,
    chassis_body: RigidBody | None = None,
    articulations: Sequence[ArticulationSpec] = (),
    unit_bodies: Mapping[str, RigidBody] | None = None,
) -> VehicleRuntime:
    """
    Compose the entries, and the wheels each of them owns, into one vehicle.

    Every entry is built by the composition layer (``si_assembly_for_axle``), so
    an entry is a *user* of a composed axle rather than a second assembly path
    beside it: a change to how an axle is composed reaches the vehicle without
    either being edited.

    ``chassis_name`` is the name of the single body the entries' overridden
    bodies are merged into.  ``chassis_body`` is that body itself; when it is not
    given, the assembler takes it from the first entry that declares a body for
    it, which is the form a document uses when its chassis subsystem is one of the
    entries.  ``request`` names the subsystems the vehicle carries; it defaults to
    the full set, so a caller that says nothing gets exactly the vehicle it always
    got.

    ``articulations`` are the joints between the assembled bodies that belong to
    neither an entry nor the vehicle -- a tractor and a trailer share one -- and
    they are stated explicitly, by the names the finished assembly uses.  No
    articulation means no extra row, so every existing caller gets what it had.

    ``unit_bodies`` are the whole assemblies an entry list says nothing about,
    keyed by the name they carry.  A vehicle document places *axles*, so the
    body a trailer's axles hang from is not in any entry; stating it here is what
    lets an articulation name it, and a body already declared by an entry keeps
    the entry's declaration rather than being replaced.
    """
    # Imported here rather than at module scope: ``vehicle_assembly`` imports this
    # module, so a module-level import back would be a cycle.
    from .vehicle_assembly import VehicleRuntime

    if mode not in ("K", "C"):
        raise ValueError(f"mode must be K or C, got {mode!r}")
    if request is not None and request.mode != mode:
        raise ValueError(
            f"mode {mode!r} disagrees with request.mode {request.mode!r}; "
            "pass one or the other, or make them agree"
        )
    resolved = request or AssemblyRequest(
        mode=mode, subsystems=DEFAULT_VEHICLE_SUBSYSTEMS
    )

    # The global rules are applied to the *root* category here, so a vehicle is
    # checked by the same statement that checks an axle rather than by an
    # assertion local to this function.  The role set is the caller's when it
    # named one, which is what makes the check able to fail: a vehicle that does
    # not steer or does not brake is refused here rather than assembled.
    check_root("vehicle", resolved.subsystems)
    bodies: dict[str, RigidBody] = {}
    for name, body in (unit_bodies or {}).items():
        bodies[name] = body if body.name == name else replace(body, name=name)
    if chassis_body is not None:
        bodies[chassis_name] = (
            chassis_body
            if chassis_body.name == chassis_name
            else replace(chassis_body, name=chassis_name)
        )
    points: dict[tuple[str, str], np.ndarray] = {}
    constraints: list[Constraint] = []
    ideal_constraints: list[Constraint] = []
    elements: list[object] = []
    connections: list[Connection] = []
    axle_assemblies: dict[str, SubsystemRuntime] = {}
    wheel_specs: dict[str, WheelSpec] = {}
    wheel_centers: dict[str, tuple[str, np.ndarray]] = {}
    wheel_body_names: dict[str, str] = {}
    wheel_rotations_local: dict[str, np.ndarray] = {}

    for entry in entries:
        composed = si_assembly_for_axle(
            entry.axle,
            # The axle carries whichever of its own roles the vehicle declares:
            # brake and drive are the vehicle's, and an axle inside a vehicle is
            # not an independent single-axle simulation.  The **wheel** role is
            # the vehicle's too, and that is the one that matters here: the wheel
            # role's whole contribution is the wheel end, and a vehicle's wheel
            # ends are the ones its entries name.  So the intersection is taken
            # with `_AXLE_ROLES_IN_A_VEHICLE`, which is the axle's vocabulary
            # minus the role the vehicle owns.
            request=AssemblyRequest(
                mode=resolved.mode,
                subsystems=resolved.subsystems & _AXLE_ROLES_IN_A_VEHICLE,
                # The entry's own sides when it declares them, the vehicle's
                # otherwise: a corner is a property of the axle that has one.
                sides=entry.sides or resolved.sides,
                # The role templates the caller supplied travel down to each axle.
                # A file's steering or chassis declaration is *one* description of
                # a vehicle, and both axles are built from it; dropping them here
                # would make a document's own steering subsystem decorative, which
                # is the state this forwarding exists to end.
                steering_template=resolved.steering_template,
                chassis_template=resolved.chassis_template,
            ),
        )
        axle: SubsystemRuntime = composed.assembly.physical
        axle_assemblies[entry.placement] = axle
        # The prefix is carried by the *name*, not by a nesting path, because the
        # vehicle's document lists one flat body sequence.  A body the entry hands
        # over is named by the vehicle body that takes its place instead; that body
        # is the caller's when they supplied it, and otherwise it is the first
        # entry that declares one, which is what lets an entry list describe a
        # vehicle whose chassis subsystem is one of its own entries.
        overridden = entry.replace_bodies
        body_map = {
            old: overridden.get(old, f"{entry.prefix}{old}") for old in axle.bodies
        }
        for old_name, body in axle.bodies.items():
            new_name = body_map[old_name]
            if old_name in overridden:
                if new_name in bodies:
                    continue
            elif new_name in bodies:
                raise ValueError(f"duplicate vehicle body {new_name!r}")
            bodies[new_name] = replace(body, name=new_name)
        points.update(
            {
                (body_map[body], label): np.asarray(point, dtype=float).copy()
                for (body, label), point in axle.points.items()
            }
        )
        constraints.extend(
            _rename_dataclasses(axle.constraints, body_map, entry.prefix)
        )
        ideal_constraints.extend(
            _rename_dataclasses(axle.ideal_constraints, body_map, entry.prefix)
        )
        # The axle contributes its own elements as they are.  It used to have its
        # vertical tires deleted here, because the vehicle's wheel ends carry
        # tires too; the axle is now composed *without* the wheel role
        # (``_AXLE_ROLES_IN_A_VEHICLE``), so there is no second statement about a
        # wheel to delete and the assembly stage no longer decides the wheel's
        # lifecycle by a type test.
        elements.extend(_rename_dataclasses(axle.elements, body_map, entry.prefix))
        connections.extend(
            _rename_connections(axle.connections, body_map, entry.prefix)
        )

        for wheel in entry.wheels:
            wheel_specs[wheel.name] = wheel
            side = "L" if wheel.name.endswith("left") else "R"
            hub = body_map.get(f"wheel_hub_{side}")
            upright = body_map[f"upright_{side}"]
            carrier = hub if hub and (hub, "wheel_center") in points else upright
            center = points[(carrier, "wheel_center")]
            if wheel.mount_joint_kind == "fixed":
                default_mount = f"upright_{side}"
            else:
                default_mount = (
                    f"wheel_hub_{side}" if hub in bodies else f"upright_{side}"
                )
            mount_body = wheel.mount_body or default_mount
            actual_mount_body = body_map.get(mount_body, mount_body)
            if actual_mount_body not in bodies:
                raise ValueError(
                    f"wheel {wheel.name!r} mount body {mount_body!r} is undefined"
                )
            runtime_body = (
                actual_mount_body
                if wheel.mount_joint_kind == "fixed"
                else wheel.body
            )
            _add_wheel(
                wheel,
                carrier,
                center,
                actual_mount_body,
                runtime_body,
                bodies,
                points,
                constraints,
                connections,
                wheel_centers,
                wheel_body_names,
                wheel_rotations_local,
                entry.prefix,
            )

    # The joints between assembled bodies belong to neither an entry nor the
    # vehicle, so they are stated by name and built here, once every body the
    # entries contributed is known: an articulation naming a body nobody
    # produced is refused instead of reaching the solver.
    for articulation in articulations:
        rows = _articulation_rows(articulation, bodies)
        constraints.extend(rows)
        ideal_constraints.extend(rows)

    assembly = VehicleRuntime(
        mode=mode,
        bodies=bodies,
        state=RigidBodyState(bodies),
        points=points,
        constraints=tuple(constraints),
        ideal_constraints=tuple(ideal_constraints),
        elements=tuple(elements),
        connections=tuple(connections),
        wheel_specs=wheel_specs,
        wheel_centers=wheel_centers,
        wheel_body_names=wheel_body_names,
        wheel_rotations_local=wheel_rotations_local,
        axle_assemblies=axle_assemblies,
        # The full vehicle carries all six roles, brake and drive included
        capabilities=capabilities_for(
            subsystems=resolved.subsystems,
            body_names=frozenset(bodies),
        ),
        # Named here so the two post-processing rules below stop asking whether a
        # body happens to be *called* chassis: the caller said which body the
        # vehicle's own is, and that answer travels with the runtime.
        chassis_name=chassis_name,
    )
    # The two post-processing steps are the historical module's, and they are
    # called with a *vehicle runtime*: both read the fields by name and return a
    # new value with ``dataclasses.replace``, which preserves the class of the
    # object it is given, so a runtime carrying the same fields comes back as a
    # runtime.  The ignores state that deliberately -- a second copy of the weld
    # condensation or the isolated-body rule would drift from this one with no
    # result showing it.
    condensed = _condense_welded_bodies(assembly)  # type: ignore[arg-type]
    return _drop_isolated_bodies(condensed)  # type: ignore[arg-type,return-value]


def runtime_for_study(assembly: VehicleRuntime) -> SubsystemRuntime:
    """
    Return the assembled vehicle in the face a *study* reads.

    A study reads one assembly two ways, and the value it reads is the one the
    composition layer builds -- a :class:`SubsystemRuntime`.  A vehicle assembled
    from entries is a different object with different tables (wheel ends, the
    per-axle runtimes), so it needs an explicit conversion to be readable, and
    the conversion belongs here rather than in each caller: it is a mechanical
    re-projection of the entities this module just built, and a caller that
    wrote its own would be a second answer to what the assembly contains.

    Nothing is dropped and nothing is invented: bodies, points, both constraint
    columns, elements and connections are carried over as they are, and the two
    fields a vehicle runtime does not have -- the hardpoint table and the C
    column's own bushings -- are stated empty rather than filled with a guess.
    A construct the reading cannot express raises there, where the document is
    being written, instead of disappearing here.
    """
    return SubsystemRuntime(
        mode=assembly.mode,
        bodies=assembly.bodies,
        points=assembly.points,
        hardpoints={},
        connections=assembly.connections,
        constraints=assembly.constraints,
        ideal_constraints=assembly.ideal_constraints,
        bushings=(),
        elements=assembly.elements,
        capabilities=assembly.capabilities,
        state=assembly.state,
    )
