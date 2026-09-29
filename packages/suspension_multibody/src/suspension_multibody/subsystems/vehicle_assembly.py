"""
The full vehicle, built by the composition layer.

The vehicle is two axles under one chassis with four wheel ends attached.  It used
to be assembled by the retired vehicle builder calling the historical
axle build twice and merging the results by hand; this module reaches the same
model by composing each axle through ``subsystems/si_assembly.py`` and joining
the two under a local chassis.

Two decisions are worth stating, because both are load-bearing:

**The wheel ends and the weld handling are the same code.**  They are subtle --
a fixed wheel is condensed into its mount body by composite mass properties, and
a weld may be fused or sent as a constraint depending on an environment switch --
and a second implementation of either would drift from the first without any
result showing it.  So this module *calls* the existing helpers rather than
re-deriving them.  That dependency is transitional: it is on the historical
module's internals, and it disappears when both live in one place.

**The field names match the historical assembly exactly.**  The post-processing
helpers are written against a value with those fields and use ``dataclasses.replace``
to return a new one, so a runtime carrying the same names can be handed to them
unchanged.  That is what makes "the new path and the old one agree" checkable
field by field instead of by eye.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Literal

import numpy as np

from ..connections.policy import check_root
from ..modeling.primitives import (
    Constraint,
    RigidBody,
    RigidBodyState,
    VerticalTireElement,
)
from ..schema import VehicleModel, WheelSpec
from .capabilities import AssemblyCapabilities, capabilities_for
from .runtime import SubsystemRuntime
from .si_assembly import si_assembly_for_axle
from .types import (
    DEFAULT_AXLE_SUBSYSTEMS,
    DEFAULT_VEHICLE_SUBSYSTEMS,
    AssemblyRequest,
    Connection,
)

__all__ = [
    "VehicleRuntime",
    "compose_vehicle_runtime",
]


@dataclass(frozen=True)
class VehicleRuntime:
    """
    The runtime face of one assembled vehicle.

    The fields are the historical assembly's, in the same order and with the same
    meaning, so that the post-processing helpers (weld condensation, isolated-body
    dropping) and every existing reader work on this value unchanged.  ``state``
    is built from ``bodies``; the four wheel tables are keyed by wheel name and
    answer the three different questions the vehicle layer asks about a wheel:
    where its centre is, which body carries it, and how its reference frame sits
    in that body.
    """

    mode: Literal["K", "C"]
    bodies: dict[str, RigidBody]
    state: RigidBodyState
    points: dict[tuple[str, str], np.ndarray]
    constraints: tuple[Constraint, ...]
    ideal_constraints: tuple[Constraint, ...]
    elements: tuple[object, ...]
    connections: tuple[Connection, ...]
    wheel_specs: dict[str, WheelSpec]
    wheel_centers: dict[str, tuple[str, np.ndarray]]
    wheel_body_names: dict[str, str]
    wheel_rotations_local: dict[str, np.ndarray]
    #: The per-axle runtime this vehicle was merged from, keyed ``front``/``rear``.
    #: A SubsystemRuntime exposes the same ``points`` mapping the historical axle
    #: assembly did, which is what the steering branch of the vehicle layer reads.
    axle_assemblies: dict[str, SubsystemRuntime]
    body_aliases: dict[str, str] = field(default_factory=dict)
    capabilities: AssemblyCapabilities | None = None

    @property
    def component_ids(self) -> tuple[str, ...]:
        """Return deterministic body identifiers."""
        return tuple(self.bodies)

    @property
    def wheel_ids(self) -> tuple[str, ...]:
        """Return deterministic corner identifiers."""
        return tuple(self.wheel_specs)

    @property
    def element_ids(self) -> tuple[str, ...]:
        """Return deterministic force-element identifiers."""
        return tuple(
            getattr(element, "name", f"element_{index}")
            for index, element in enumerate(self.elements)
        )

    @property
    def total_mass(self) -> float:
        """Return the sum of all movable and fixed body masses."""
        return float(sum(body.mass for body in self.bodies.values()))

    def wheel_center_local(self, wheel: str) -> np.ndarray:
        """Return the wheel-center point on its upright body."""
        try:
            return self.wheel_centers[wheel][1].copy()
        except KeyError as exc:
            raise KeyError(f"unknown wheel {wheel!r}") from exc


def compose_vehicle_runtime(
    model: VehicleModel,
    mode: Literal["K", "C"] = "K",
    request: AssemblyRequest | None = None,
) -> VehicleRuntime:
    """
    Compose suspension, wheel ends and chassis into one vehicle runtime.

    Each axle is built by the composition layer, so a vehicle is a *user* of the
    composed axle rather than a second assembly path beside it: a change to how an
    axle is composed reaches the vehicle automatically, which is the property the
    two hand-written paths could never have.

    ``request`` names the subsystems the vehicle carries.  It defaults to the full
    set, so a caller that says nothing gets exactly the vehicle it always got, and
    a caller that resolved its roles from an assembly file gets that assembly
    instead of a role list this function decided on its own.
    """
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

    # The mechanisms live in `vehicle_parts`, in this same layer: welding, name
    # mapping and wheel placement are the composition's own business, and reaching
    # into the retired author-layer package for them would have kept that package
    # alive after everything that used to be in it was replaced.
    from .vehicle_parts import (
        _add_wheel,
        _body_from_spec,
        _condense_welded_bodies,
        _drop_isolated_bodies,
        _rename_connections,
        _rename_dataclasses,
    )

    chassis = _body_from_spec(model.chassis)
    bodies: dict[str, RigidBody] = {chassis.name: chassis}
    points: dict[tuple[str, str], np.ndarray] = {}
    constraints: list[Constraint] = []
    ideal_constraints: list[Constraint] = []
    elements: list[object] = []
    connections: list[Connection] = []
    axle_assemblies: dict[str, SubsystemRuntime] = {}
    wheel_specs = {wheel.name: wheel for wheel in model.wheels}
    wheel_centers: dict[str, tuple[str, np.ndarray]] = {}
    wheel_body_names: dict[str, str] = {}
    wheel_rotations_local: dict[str, np.ndarray] = {}

    for axle_name, axle_model, prefix in (
        ("front", model.front_axle, "front_"),
        ("rear", model.rear_axle, "rear_"),
    ):
        composed = si_assembly_for_axle(
            axle_model,
            # The axle carries whichever of its own roles the vehicle declares:
            # brake and drive are the vehicle's, and an axle inside a vehicle is
            # not an independent single-axle simulation, so the intersection with
            # the axle's own role vocabulary is what it may carry.
            request=AssemblyRequest(
                mode=resolved.mode,
                subsystems=resolved.subsystems & DEFAULT_AXLE_SUBSYSTEMS,
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
        axle_assemblies[axle_name] = axle
        # The prefix is carried by the *name*, not by a nesting path, because the
        # vehicle's document lists one flat body sequence.  Two axles both declare
        # a body called ``chassis``; the vehicle's own chassis is the one that
        # survives, and every other axle body is qualified by its end.
        body_map = {
            old: model.chassis.name if old in ("chassis", "ground") else f"{prefix}{old}"
            for old in axle.bodies
        }
        for old_name, body in axle.bodies.items():
            if old_name in ("chassis", "ground"):
                continue
            new_name = body_map[old_name]
            if new_name in bodies:
                raise ValueError(f"duplicate vehicle body {new_name!r}")
            bodies[new_name] = replace(body, name=new_name)
        points.update(
            {
                (body_map[body], label): np.asarray(point, dtype=float).copy()
                for (body, label), point in axle.points.items()
            }
        )
        constraints.extend(_rename_dataclasses(axle.constraints, body_map, prefix))
        ideal_constraints.extend(
            _rename_dataclasses(axle.ideal_constraints, body_map, prefix)
        )
        # The axle's own vertical tires are dropped: the vehicle owns its wheels
        # and the wheel ends carry their tires, so keeping both would count the
        # same tire twice.
        elements.extend(
            element
            for element in _rename_dataclasses(axle.elements, body_map, prefix)
            if not isinstance(element, VerticalTireElement)
        )
        connections.extend(_rename_connections(axle.connections, body_map, prefix))

        for wheel in model.wheels:
            if not wheel.name.startswith(f"{axle_name}_"):
                continue
            side = "L" if wheel.name.endswith("left") else "R"
            hub = body_map.get(f"wheel_hub_{side}")
            upright = body_map[f"upright_{side}"]
            carrier = hub if hub and (hub, "wheel_center") in points else upright
            center = points[(carrier, "wheel_center")]
            if wheel.mount_joint_kind == "fixed":
                default_mount = f"upright_{side}"
            else:
                default_mount = f"wheel_hub_{side}" if hub in bodies else f"upright_{side}"
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
                prefix,
            )

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
