"""
The runtime values a set of subsystem contributions amounts to.

``subsystems/composition.py`` joins contributions into a :class:`ModelFragment`,
which is *identity*: body names, point roles, joint and port declarations.  A
contract emitter needs more than identity -- it needs the built objects, in the
order the document lists them -- and that face is what this module provides.

The distinction is the same one ``modeling/assembly.py`` draws between an
``Assembly`` and the build behind it, and it is deliberate here:

* the **fragment** answers "what is in this model, and where did it come from";
* the **runtime** answers "what object does each entity become, in which order".

Nothing here solves, submits native or decodes a result.  It builds the values a
compiler reads, from the contributions the subsystem layer already produced.

The seven faces a family emitter reads are ``bodies``, ``points``,
``constraints``, ``ideal_constraints``, ``bushings``, ``elements`` and
``hardpoints``; ``state`` and ``capabilities`` travel with them because every
caller that reads four of the seven reads those two as well.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
from dataclasses import dataclass
from typing import Any, Literal

import numpy as np

from ..modeling.primitives.joints import Constraint, RigidBody, RigidBodyState
from .capabilities import AssemblyCapabilities, capabilities_for
from .types import Connection, ResolvedElement, SubsystemOutput

__all__ = [
    "ElementBuilder",
    "RuntimeDifference",
    "RuntimeOrder",
    "SubsystemRuntime",
    "diff_against_reference",
    "runtime_from_outputs",
    "wheel_centre_local",
]

#: How a :class:`~.types.ResolvedElement` becomes the runtime object a document
#: carries.  Injected rather than imported so that this module does not depend on
#: the assembly package that imports it: the caller that owns element
#: construction passes it in.
ElementBuilder = Callable[[ResolvedElement], object]


@dataclass(frozen=True)
class RuntimeOrder:
    """
    The entity sequence the contract document records, per face.

    Each entry is the list of names in document order.  A name the runtime does
    not carry is skipped rather than refused, and a name the runtime carries but
    the order omits keeps its relative position at the end -- so an order that is
    merely incomplete is usable, while an order that is wrong is still visible in
    the comparison against the reference.
    """

    bodies: tuple[str, ...] = ()
    constraints: tuple[str, ...] = ()
    ideal_constraints: tuple[str, ...] = ()
    connections: tuple[str, ...] = ()

    def apply(self, names: Iterable[str], recorded: tuple[str, ...]) -> list[str]:
        """Return ``names`` reordered by ``recorded``, keeping the rest in place."""
        present = list(names)
        known = set(present)
        ordered = [name for name in recorded if name in known]
        seen = set(ordered)
        ordered.extend(name for name in present if name not in seen)
        return ordered


@dataclass(frozen=True)
class SubsystemRuntime:
    """
    The runtime values one assembly's contributions amount to.

    Immutable, and ordered exactly as the contract document lists the entities:
    a body list, a point mapping and a joint sequence are part of what an
    assembly *produces*, so the order is carried here rather than recomputed by
    each reader.

    ``state`` is built once, from the final body table, because the callers that
    read it (``api``, the reports) read it per state and rebuilding it per call
    would make a frozen value quietly quadratic.
    """

    #: Which physical connection set this runtime carries.
    mode: Literal["K", "C"]
    #: Bodies by name, in emission order.  SI: metres, kilograms.
    bodies: dict[str, RigidBody]
    #: Body-local points, keyed ``(body, label)``.
    points: dict[tuple[str, str], np.ndarray]
    #: Assembly hardpoints: the model's own plus the ones a subsystem generated.
    hardpoints: dict[str, Any]
    #: Connection rows, for the assembly's own accounting.
    connections: tuple[Connection, ...]
    #: The constraints ``mode`` selects.
    constraints: tuple[Constraint, ...]
    #: The ideal-joint column, which K activates and C keeps beside it.
    ideal_constraints: tuple[Constraint, ...]
    #: The C column's bushings, in emission order.
    bushings: tuple[object, ...]
    #: Every built force element, in the order the document lists them.
    elements: tuple[object, ...]
    #: What this assembly carries, for a rig to bind against.
    capabilities: AssemblyCapabilities | None = None
    #: The ground the tires are measured against, when the assembly's model declared one.
    #:
    #: Carried here because ``contract.model_document`` only ever receives the runtime:
    #: the source model is consumed while the assembly is built, so a value the document
    #: needs has to travel with the runtime or it is lost on the way.  ``None`` means the
    #: reading supplies its own ground (the pad reading states a height per scan point).
    road: object | None = None
    #: The body state built from ``bodies``.
    state: RigidBodyState | None = None

    def __post_init__(self) -> None:
        if self.state is None:
            object.__setattr__(self, "state", RigidBodyState(self.bodies))

    @property
    def component_ids(self) -> tuple[str, ...]:
        """Return the body names, in emission order."""
        return tuple(self.bodies)

    @property
    def element_ids(self) -> tuple[str, ...]:
        """Return stable force-element identifiers for result tables."""
        return tuple(
            getattr(element, "name", f"element_{index}")
            for index, element in enumerate(self.elements)
        )

    def point(self, body: str, label: str) -> np.ndarray:
        """
        Return a body-local point by stable label.

        A copy, like the assembly reader it replaces: a caller that mutates the
        array it was handed would otherwise edit the model for every later
        reader, and the resulting drift is invisible in a result.
        """
        return self.points[(body, label)].copy()


#: The labels a wheel centre is recorded under, in the order a reader tries them.
_WHEEL_CENTRE_LABELS: tuple[str, ...] = ("wheel_center", "spindle")


def wheel_centre_local(assembly: Any, body: str) -> np.ndarray:
    """
    Return `body`'s wheel centre, in `body`'s own frame, or refuse by name.

    Which label states it is the topology's: a suspension template that hangs the
    wheel straight on the upright records it as `wheel_center`, while one that gives
    the wheel a hub of its own (`方式 A`) records the same point on the upright as
    `spindle` -- the point its spin joint turns about -- and states the wheel centre
    on the hub.  A reader that carries the *upright's* pose beside the centre has to
    ask for the point in the upright's frame, and this is the one place that knows
    which label that is.
    """
    for label in _WHEEL_CENTRE_LABELS:
        if (body, label) in getattr(assembly, "points", ()):
            return np.asarray(assembly.point(body, label), dtype=float)
    raise ValueError(f"the assembly records no wheel centre on {body!r}")


def runtime_from_outputs(
    outputs: Iterable[SubsystemOutput],
    *,
    mode: Literal["K", "C"],
    roles: frozenset[str] | None = None,
    capabilities: AssemblyCapabilities | None = None,
    element_builder: ElementBuilder | None = None,
    order: "RuntimeOrder | None" = None,
    road: object | None = None,
) -> SubsystemRuntime:
    """
    Build the runtime face from the contributions of one assembly.

    ``outputs`` are the subsystem contributions in the order the document lists
    them.  Bodies merge by key so a later contribution can refine an earlier one
    (the assembly gives the chassis its mass specs last); the sequences append in
    argument order, because that order *is* the recorded contract.

    ``element_builder`` turns a declared :class:`ResolvedElement` into the runtime
    object.  It is a parameter rather than an import because the module that owns
    element construction imports this package: passing it in keeps the dependency
    one-way.

    ``order`` states the sequence the *document* records, when it differs from the
    order the contributions happen to be listed in.  The axle records its
    constraints interleaved by side while contributing them by role, so the
    sequence is a fact about the document rather than about the contributions, and
    it is stated where it is known instead of being inferred from names.
    """
    if mode not in ("K", "C"):
        raise ValueError(f"mode must be K or C, got {mode!r}")

    bodies: dict[str, RigidBody] = {}
    points: dict[tuple[str, str], np.ndarray] = {}
    hardpoints: dict[str, Any] = {}
    connections: list[Connection] = []
    constraints: list[Constraint] = []
    ideal_constraints: list[Constraint] = []
    elements: list[object] = []

    build = element_builder if element_builder is not None else _default_element_builder()

    for output in outputs:
        bodies.update(output.bodies)
        points.update(output.points)
        hardpoints.update(output.hardpoints)
        connections.extend(output.connections)
        constraints.extend(output.constraints)
        ideal_constraints.extend(output.ideal_constraints)
        for row in output.elements:
            elements.append(build(row))
        # The compliance column is built into the element list as well, because
        # the document lists one element sequence and a bushing is a member of
        # it.  Keeping them apart would break the invariant every reader relies
        # on -- that the compliance column is a *subset* of the elements -- and
        # the placeholders come last, exactly as the recorded order has them.
        for row in output.bushings:
            elements.append(build(row))

    # A C-mode assembly reports its explicit bushings separately, because a
    # caller asks "which compliance elements does this carry" without walking the
    # whole element list.  K carries none by construction.
    resolved_bushings = (
        tuple(element for element in elements if _is_bushing(element))
        if mode == "C"
        else ()
    )

    if capabilities is None and roles is not None:
        capabilities = capabilities_for(
            subsystems=frozenset(roles), body_names=frozenset(bodies)
        )

    if order is not None:
        bodies = _reorder(bodies, order.bodies)
        constraints = _reorder(constraints, order.constraints)
        ideal_constraints = _reorder(ideal_constraints, order.ideal_constraints)
        connections = _reorder(connections, order.connections)

    return SubsystemRuntime(
        mode=mode,
        bodies=bodies,
        points=points,
        hardpoints=hardpoints,
        connections=tuple(connections),
        constraints=tuple(constraints),
        ideal_constraints=tuple(ideal_constraints),
        bushings=resolved_bushings,
        elements=tuple(elements),
        capabilities=capabilities,
        road=road,
    )


def _reorder(items: Any, recorded: tuple[str, ...]) -> Any:
    """
    Reorder a mapping or a sequence by a recorded name order.

    Mappings keep their type, because a runtime's faces are read both by index
    and by name and a caller that received a dict must keep receiving one.
    """
    if not recorded:
        return items
    if isinstance(items, dict):
        known = set(items)
        return {
            **{name: items[name] for name in recorded if name in known},
            **{name: value for name, value in items.items() if name not in set(recorded)},
        }
    ordered: list[Any] = []
    by_name: dict[str, list[Any]] = {}
    for item in items:
        by_name.setdefault(getattr(item, "name", ""), []).append(item)
    seen: set[int] = set()
    for name in recorded:
        for item in by_name.get(name, ()):
            ordered.append(item)
            seen.add(id(item))
    ordered.extend(item for item in items if id(item) not in seen)
    return ordered


def _default_element_builder() -> ElementBuilder:
    """
    Return the element builder a composition uses when none is passed in.

    The constructor lives in this package (``element_build``), so the caller that
    used to have to inject it no longer does; the parameter stays because a study
    may want to build elements a different way, and because a caller that owns no
    element semantics should not have to reach for one.
    """
    from .element_build import build_element

    return build_element


def _is_bushing(element: object) -> bool:
    """Return whether a built element is a six-axis bushing."""
    return type(element).__name__ == "BushingElement"


@dataclass(frozen=True)
class RuntimeDifference:
    """
    How one runtime differs from the assembly it is meant to replace.

    Reported rather than raised: during a migration the two are allowed to
    differ, and the value of the comparison is that the difference is *listed*
    instead of being discovered later as a changed number.
    """

    #: Names present in the reference but absent from the runtime, by face.
    missing: dict[str, tuple[str, ...]]
    #: Names present in the runtime but absent from the reference, by face.
    extra: dict[str, tuple[str, ...]]
    #: Faces whose order or sequence differs.
    reordered: tuple[str, ...]

    @property
    def equal(self) -> bool:
        """Return whether the two agree on every face compared."""
        return not (
            self.reordered
            or any(self.missing.values())
            or any(self.extra.values())
        )

    def report(self) -> str:
        """Return a one-block human-readable summary."""
        lines = [f"runtime vs reference: {'identical' if self.equal else 'differs'}"]
        for face in sorted(set(self.missing) | set(self.extra)):
            missing = list(self.missing.get(face, ()))
            extra = list(self.extra.get(face, ()))
            if not missing and not extra:
                continue
            lines.append(f"  {face}:")
            if missing:
                lines.append(f"    only in reference ({len(missing)}): {missing}")
            if extra:
                lines.append(f"    only in runtime   ({len(extra)}): {extra}")
        if self.reordered:
            lines.append(f"  order differs: {list(self.reordered)}")
        return "\n".join(lines)


def diff_against_reference(
    runtime: SubsystemRuntime, reference: Any
) -> RuntimeDifference:
    """
    Compare a runtime against the assembly it is meant to replace.

    ``reference`` is any object exposing the same seven faces, which is what the
    historical assembly does.  Names are compared per face and order is compared
    on the sequences whose order the document records; the point face is compared
    as a key set because a document resolves points by name.
    """
    faces: tuple[tuple[str, Callable[[Any], list[str]]], ...] = (
        ("bodies", lambda source: list(getattr(source, "bodies", {}))),
        ("points", lambda source: sorted(f"{b}.{p}" for b, p in getattr(source, "points", {}))),
        (
            "constraints",
            lambda source: [getattr(c, "name", "") for c in getattr(source, "constraints", ())],
        ),
        (
            "ideal_constraints",
            lambda source: [
                getattr(c, "name", "") for c in getattr(source, "ideal_constraints", ())
            ],
        ),
        (
            "bushings",
            lambda source: [getattr(b, "name", "") for b in getattr(source, "bushings", ())],
        ),
        (
            "elements",
            lambda source: [
                f"{type(e).__name__}:{getattr(e, 'name', '')}"
                for e in getattr(source, "elements", ())
            ],
        ),
        ("hardpoints", lambda source: sorted(getattr(source, "hardpoints", {}))),
    )

    missing: dict[str, tuple[str, ...]] = {}
    extra: dict[str, tuple[str, ...]] = {}
    reordered: list[str] = []

    for face, reader in faces:
        reference_names = reader(reference)
        runtime_names = reader(runtime)
        if sorted(reference_names) != sorted(runtime_names):
            missing[face] = tuple(sorted(set(reference_names) - set(runtime_names)))
            extra[face] = tuple(sorted(set(runtime_names) - set(reference_names)))
        if reference_names != runtime_names:
            reordered.append(face)

    return RuntimeDifference(
        missing=missing, extra=extra, reordered=tuple(reordered)
    )
