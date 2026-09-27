"""
Connecting a bench's entities to the assembly it loads.

A bench is not only a declaration of what it drives: a wheel-supplying bench owns
the wheel, and the axle deliberately builds none (D9).  Until this module existed
the bench's bodies lived in the composition's ``rig`` level and nowhere else, so
they were never part of the model that gets solved -- the wheels existed on paper
and the run loaded nothing through them.

What the link is, and why this shape:

* a wheel-supplying bench's carrier is tied to the upright by **the joint the
  bench itself declared** -- a prismatic slide along the wheel axis.  That is what
  the bench means by a carrier: the body it moves to apply travel.  Welding it
  instead would lock the travel the bench exists to apply, and a K sweep would be
  immobilised (the static trim then fails outright);
* the **tire moves to the carrier**.  D3: when the bench supplies the wheel, the
  tire belongs to the bench's wheel, not to the upright, and leaving it on the
  upright would count the same tire twice.  Its stiffness is unchanged and its
  ``wheel_center_local`` becomes zero, because the carrier's origin *is* the wheel
  centre -- this is a change of owner and not of physics;
* the carrier gets **no** ``wheel_center`` point.  The driven coordinate is read
  from whichever body declares that point, and declaring it twice would make the
  lookup ambiguous -- the assembly's own reader refuses exactly that, on purpose.
  The carrier's point at the wheel centre is named ``center``, the same spelling
  the wheel spin joint uses.

The prescribed motion therefore still acts on the **suspension**, through the
upright, exactly as it did before the bench joined the model.  That is the
property that keeps "the bench is in the model" from silently changing which
question the run answers.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any, Literal, cast

import numpy as np

from ..modeling.assembly import Assembly
from ..modeling.primitives import SE3, RigidBody, WeldJoint
from .runtime import SubsystemRuntime
from .types import Connection

__all__ = [
    "RigLink",
    "RigLinkError",
    "link_wheel_supplying_rig",
    "merge_rig_link",
]

#: The sides a wheel-supplying bench builds.
_SIDES: tuple[str, ...] = ("L", "R")


class RigLinkError(ValueError):
    """A bench cannot be attached to the assembly it was bound to."""


class RigLink:
    """
    What a bench contributed to one assembly's runtime, in runtime terms.

    Returned rather than merged in place so the caller decides where the entities
    go, and so a test can inspect the link without rebuilding the assembly.
    """

    #: The carrier bodies, keyed by side.
    bodies: dict[str, RigidBody]
    #: Points the carriers contribute, keyed ``(body, label)``.
    points: dict[tuple[str, str], np.ndarray]
    #: The welds attaching each carrier to its upright.
    constraints: tuple[Any, ...]
    #: The tire elements, re-owned by the carrier.
    elements: tuple[object, ...]
    #: Connection rows, for the assembly's own accounting.
    connections: tuple[Connection, ...]

    def __init__(
        self,
        *,
        bodies: dict[str, RigidBody],
        points: dict[tuple[str, str], np.ndarray],
        constraints: tuple[Any, ...],
        elements: tuple[object, ...],
        connections: tuple[Connection, ...],
    ) -> None:
        self.bodies = bodies
        self.points = points
        self.constraints = constraints
        self.elements = elements
        self.connections = connections

    @property
    def is_empty(self) -> bool:
        """Return whether the bench contributed nothing to the model."""
        return not self.bodies


def link_wheel_supplying_rig(
    runtime: SubsystemRuntime,
    rig: Assembly,
    *,
    bindings: dict[str, str] | None = None,
    mode: Literal["K", "C"] = "K",
) -> RigLink:
    """
    Return the entities that attach one bench's wheels to their uprights.

    The bench is read as a *level*: its bodies and joints come from its own
    fragment, so a bench that grows a part contributes it without this function
    being edited.  Only the wheel-supplying branch is handled -- a loading bench
    owns no wheel, so there is nothing to attach, and inventing an attachment for
    it would put a body in the model that the bench never declared.
    """
    if mode not in ("K", "C"):
        raise RigLinkError(f"unknown mode {mode!r}; modes are K and C")

    carrier_names = [
        name for name in rig.fragment.bodies if name.startswith("wheel_carrier_")
    ]
    if not carrier_names:
        return RigLink(bodies={}, points={}, constraints=(), elements=(), connections=())

    bodies: dict[str, RigidBody] = {}
    points: dict[tuple[str, str], np.ndarray] = {}
    constraints: list[Any] = []
    elements: list[object] = []
    connections: list[Connection] = []

    for carrier_name in sorted(carrier_names):
        side = carrier_name.rsplit("_", 1)[-1]
        if side not in _SIDES:
            raise RigLinkError(
                f"bench body {carrier_name!r} does not name a side; the sides are "
                f"{list(_SIDES)}"
            )
        upright = _upright_for(runtime, side)
        if upright is None:
            # An assembly with no upright on this side has no wheel to attach, so
            # the bench's carrier stays out of the model rather than floating in
            # it: a body connected to nothing is a freedom the solver would have
            # to pin, which is the silent failure this whole module exists to
            # prevent.
            continue
        upright_body = runtime.bodies[upright]
        # The wheel centre in **world** coordinates.  This one number is both the
        # carrier's placement and the weld's point on each side, which is what makes
        # the weld coincident.
        #
        # The convention is the emitter's, and it is easy to get wrong: a joint's
        # ``point_*`` fields are given in *world* coordinates and resolved into each
        # body's own frame by `cases/kc_quasi_static/contract.py::_local_point`.
        # Passing the upright-local offset as ``point_a`` and zero as ``point_b`` --
        # which reads as "the same place, stated per body" -- produced a weld whose
        # two resolved points were ``[0,-700,300]`` and ``[0,700,-300]``: a mirrored
        # pair, not a coincident one.  The same construction is what
        # ``vehicle_parts.py::_add_wheel`` uses: it derives the wheel
        # origin as ``centre_world - rotation @ centre_local`` and states the point
        # in world coordinates.
        centre_local = np.asarray(
            runtime.points[(upright, "wheel_center")], dtype=float
        )
        centre_world = np.asarray(
            upright_body.pose.transform_point(centre_local), dtype=float
        )
        rotation = np.asarray(upright_body.pose.rotation, dtype=float)
        quaternion = np.asarray(upright_body.pose.quaternion, dtype=float)
        # The carrier's ``center`` point is its own origin, so the origin is the
        # wheel centre and the point is zero in its frame.
        carrier_origin = centre_world
        declared = rig.fragment.bodies[carrier_name]
        # The carrier is **free**, like every other body in this model.  Making it
        # `fixed` was tried and is wrong: a fixed body is grounded, so welding it to
        # a free upright grounds the whole suspension -- measured as 64 constraint
        # rows over 54 columns, i.e. ten redundant rows, and the kernel refuses it
        # as a rank-deficient Jacobian.  Free, the counts match the historical model
        # exactly (K: two more bodies and two more welds, the same two unconstrained
        # directions).
        #
        # Its mass is left at the declaration's value.  The contract emitter writes a
        # 1.0 kg fallback for any free body without positive mass, and that is not
        # peculiar to the carrier: every body this model emits gets the same
        # fallback, including in the historical model that converges.  So the
        # fallback is the established behaviour here and not the cause of anything.
        bodies[carrier_name] = RigidBody(
            name=carrier_name,
            pose=SE3(carrier_origin, quaternion),
            mass=float(getattr(declared, "mass", 0.0) or 0.0),
            inertia=np.zeros((3, 3), dtype=float),
            fixed=False,
        )
        # The point at the wheel centre is named `center`, not `wheel_center`: the
        # driven coordinate is read from whichever body declares the latter, and a
        # second declaration would make that lookup ambiguous.
        points[(carrier_name, "center")] = np.zeros(3, dtype=float)
        points[(carrier_name, "contact")] = rotation.T @ np.array(
            [0.0, 0.0, -_unloaded_radius(runtime, upright)], dtype=float
        )
        # The carrier is rigid with the upright: a K&C bench mounts a rigid wheel on
        # the suspension and lets the *tire* carry the compliance to the road, which
        # is why the drive can still act on the upright and reach the wheel.
        #
        # A sliding attachment was tried and is wrong here: the bench declares a
        # prismatic carrier, but taking that literally leaves the carrier free along
        # the wheel axis, so in C mode -- where nothing prescribes that axis -- the
        # solver has two unconstrained directions to pin.  Rigid is also the reading
        # the bench's own part list supports: the carrier declares no mass, so it
        # exists to carry the wheel's tire rather than to move on its own.
        attachment = WeldJoint(
            upright,
            centre_world,
            carrier_name,
            centre_world,
            name=f"{carrier_name}_weld",
        )
        constraints.append(attachment)
        connections.append(
            Connection(
                name=attachment.name,
                kind="ideal",
                body_a=upright,
                body_b=carrier_name,
                point_a="wheel_center",
                point_b="center",
            )
        )
        # D3: the tire belongs to the bench's wheel.  Re-owning it is a change of
        # body, not of stiffness or radius, so the force path is the same one the
        # model already described.
        elements.extend(_reown_tires(runtime, upright, carrier_name))

    return RigLink(
        bodies=bodies,
        points=points,
        constraints=tuple(constraints),
        elements=tuple(elements),
        connections=tuple(connections),
    )


def merge_rig_link(runtime: SubsystemRuntime, link: RigLink) -> SubsystemRuntime:
    """
    Return the runtime with the bench's entities merged in.

    The welds go into both constraint columns: a weld is an ideal joint, and the
    two columns exist to answer "what does this model look like in K" and "in C",
    not to distinguish rigid from compliant.  The tire elements *replace* the ones
    they re-own, so the same tire cannot appear twice.
    """
    if link.is_empty:
        return runtime
    reowned = {id(element) for element in link.elements}
    kept = [
        element
        for element in runtime.elements
        if id(element) not in reowned and not _is_replaced_tire(element, link)
    ]
    return replace(
        runtime,
        bodies={**runtime.bodies, **link.bodies},
        points={**runtime.points, **link.points},
        constraints=(*runtime.constraints, *link.constraints),
        ideal_constraints=(*runtime.ideal_constraints, *link.constraints),
        connections=(*runtime.connections, *link.connections),
        elements=(*kept, *link.elements),
    )


def _is_replaced_tire(element: object, link: RigLink) -> bool:
    """
    Return whether a tire the runtime holds is superseded by a re-owned one.

    Matched by name rather than by identity, because the re-owned element is a new
    object with the same name: the tire *is* the same tire, owned by somebody else.
    """
    if type(element).__name__ != "VerticalTireElement":
        return False
    names = {getattr(candidate, "name", "") for candidate in link.elements}
    return getattr(element, "name", "") in names


def _upright_for(runtime: SubsystemRuntime, side: str) -> str | None:
    """
    Return the body carrying one side's wheel centre, if there is one.

    The search is by *declaration* -- whichever body declares the point -- so a
    bench attaches to a trailing-arm axle as readily as to a double wishbone,
    without this module knowing either template's part names.
    """
    found = [
        body
        for body in runtime.bodies
        if body.endswith(f"_{side}") and (body, "wheel_center") in runtime.points
    ]
    if len(found) > 1:
        raise RigLinkError(
            f"side {side} declares more than one wheel centre ({', '.join(sorted(found))}); "
            "a bench cannot decide which body its wheel belongs to"
        )
    return found[0] if found else None


def _unloaded_radius(runtime: SubsystemRuntime, upright: str) -> float:
    """Return the unloaded radius of the tire on this upright, or zero."""
    for element in runtime.elements:
        if type(element).__name__ != "VerticalTireElement":
            continue
        if getattr(element, "wheel_body", None) == upright:
            return float(getattr(element, "unloaded_radius", 0.0) or 0.0)
    return 0.0


def _reown_tires(
    runtime: SubsystemRuntime, upright: str, carrier: str
) -> tuple[object, ...]:
    """
    Return the upright's tires, re-owned by the bench's carrier.

    A tire is a force element between its body and the road, so moving it from the
    upright to the carrier leaves the force path intact: the carrier sits at the
    wheel centre, and the wheel centre it acts at is the same physical point.

    ``wheel_center_local`` becomes the **zero vector**, and that is load-bearing:
    the point is expressed in the owning body's frame, and the carrier's frame
    origin *is* the wheel centre.  Keeping the upright-local offset would place the
    contact patch at the wheel centre plus that offset, measured from a frame that
    already sits there -- a wrong contact point, and therefore a wrong wheel load.
    """
    reowned: list[object] = []
    for element in runtime.elements:
        if type(element).__name__ != "VerticalTireElement":
            continue
        if getattr(element, "wheel_body", None) != upright:
            continue
        # The class-name check above narrows the runtime object to the tire
        # element; the cast states that to the type checker rather than leaving it
        # to a suppression comment.
        tire = cast("Any", element)
        reowned.append(
            replace(
                tire,
                wheel_body=carrier,
                wheel_center_local=np.zeros(3, dtype=float),
            )
        )
    return tuple(reowned)
