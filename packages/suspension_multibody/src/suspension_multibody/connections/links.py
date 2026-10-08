"""
Explicit interface pairings, turned into physical rows.

``matcher`` decides *which* offered port satisfies *which* requirement.  That is
a decision, and a decision on its own joins nothing.  This module is the other
half: a contribution that knows *how* two subsystems are joined states a
:class:`LinkSpec`, and the resolved bindings are turned into the rows a fragment
carries -- a ``Connection`` beside the joint or bushing it stands for, with the
type, both bodies, both points and both local coordinates written down.

Two properties are deliberate:

* **No recipe, no rows.**  A contribution that states no ``LinkSpec`` produces
  nothing here, so an assembly that never mentioned a link keeps exactly the
  entities it had.  That is what makes the change additive rather than a
  re-recording of every existing model.
* **The recipe sits on the contribution side.**  Geometry is not folded into
  :class:`~suspension_multibody.modeling.ports.PortRequirement`: a requirement
  says what a neighbour must offer, a ``LinkSpec`` says what to build once that
  neighbour has been found.  Matching stays the matcher's job, so this module
  adds no second way to decide who connects to whom.

Only joint and element types that already exist are used.  A new kind of joint is
a kernel question, not a pairing one.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from typing import Any, Literal

import numpy as np

from ..modeling.identity import EntityId
from ..modeling.instance import Connection, ModelFragment
from ..modeling.ports import GeometryPort, PortSpec
from ..modeling.primitives.elements import BushingElement
from ..modeling.primitives.joints import RevoluteJoint, WeldJoint
from ..modeling.primitives.spatial import SE3
from .matcher import AmbiguousBindingError, Binding, BindingError

__all__ = [
    "LinkError",
    "LinkKind",
    "LinkRow",
    "LinkRows",
    "LinkSpec",
    "build_links",
    "explicit_bindings_from_pairings",
]

#: The joint a link stands for.  ``weld`` and ``revolute`` are ideal constraints;
#: ``bushing`` is a compliant element row.
LinkKind = Literal["weld", "revolute", "bushing"]

#: The kinds whose entity is an ideal constraint rather than a force element.
_IDEAL_KINDS: tuple[str, ...] = ("weld", "revolute")

#: A frame with no rotation, for the attachment frames of a bushing row.
_NO_ROTATION = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)


class LinkError(ValueError):
    """A link recipe is not usable as stated."""


@dataclass(frozen=True)
class LinkSpec:
    """
    How one requirement is joined once its counterpart port is known.

    ``role`` names the requirement this link fills, so the port comes from the
    match rather than from a second guess here.  ``body_a`` and ``point_a_local``
    are the needing side's own end: which body carries the link, and where on it,
    in that body's local coordinates.

    The other end defaults to the matched port -- its owner body, its local frame
    origin and its own local name, because those are the claims it was matched
    on.  ``body_b`` / ``point_b_local`` / ``point_b_label`` override that when the
    attachment belongs somewhere else than the port's declared frame.

    A ``revolute`` link also needs an axis per body.  There is no relative
    rotation to read at this level -- the link layer sees points, not poses -- so
    the recipe states the axis in each body's local frame.

    A ``bushing`` link carries the recipe's stiffness; leaving it unset gives the
    zero matrix, the same reading the template's mount slot already has: a mount
    that is declared and carries no compliance.
    """

    role: str
    kind: LinkKind
    body_a: str
    point_a_local: Any = (0.0, 0.0, 0.0)
    point_a_label: str = "link_a"
    body_b: str = ""
    point_b_local: Any = None
    point_b_label: str = ""
    axis_a: Any = (0.0, 1.0, 0.0)
    axis_b: Any = (0.0, 1.0, 0.0)
    stiffness: Any = None
    name: str = ""

    def __post_init__(self) -> None:
        if not self.role:
            raise LinkError("a link recipe needs the role of the requirement it fills")
        if self.kind not in ("weld", "revolute", "bushing"):
            raise LinkError(
                f"link {self.role!r} has an unknown kind {self.kind!r}; the kinds "
                "are weld, revolute and bushing"
            )
        if not self.body_a:
            raise LinkError(f"link {self.role!r} needs the body it is attached to")


@dataclass(frozen=True)
class LinkRow:
    """One generated link, itemised: type, both bodies, both points, coordinates."""

    role: str
    kind: str
    row_name: str
    row_type: str
    port_id: str
    body_a: str
    body_b: str
    point_a_label: str
    point_b_label: str
    point_a: tuple[float, float, float]
    point_b: tuple[float, float, float]


@dataclass(frozen=True)
class LinkRows:
    """The rows one set of recipes produces, plus the itemised form for audits."""

    fragment: ModelFragment
    rows: tuple[LinkRow, ...] = field(default=())


def build_links(
    bindings: Iterable[Binding],
    ports: Mapping[str, PortSpec],
    specs: Iterable[LinkSpec],
    *,
    instance: tuple[str, ...] = (),
) -> LinkRows:
    """
    Turn resolved bindings into joint or bushing rows, one per recipe.

    ``bindings`` is what ``match_requirements`` returned, ``ports`` the same
    offered-port mapping it matched against, and ``specs`` the recipes the
    contributions stated.  No recipe means no rows: the fragment comes back empty
    and merging it changes nothing.

    Every failure names the requirement role, because that is the name the author
    wrote and can repair: a role the match never bound, a role bound to more than
    one port, a port that is not offered, or a port carrying no frame.
    """
    recipes = tuple(specs)
    if not recipes:
        return LinkRows(fragment=ModelFragment(instance=instance))

    held = {binding.requirement.role: binding for binding in bindings}
    joints: dict[str, Any] = {}
    ideal: dict[str, Any] = {}
    bushings: dict[str, Any] = {}
    connections: dict[str, Any] = {}
    rows: list[LinkRow] = []

    for spec in recipes:
        binding = held.get(spec.role)
        if binding is None or not binding.port_ids:
            raise BindingError(
                f"link {spec.role!r} names a requirement this assembly did not "
                f"bind; the bound requirements are {sorted(held) or '(none)'}"
            )
        if len(binding.port_ids) > 1:
            raise AmbiguousBindingError(
                f"requirement {spec.role!r} was bound to {len(binding.port_ids)} "
                f"ports {list(binding.port_ids)}; a link joins one port, so the "
                "pairing has to say which one is meant",
                tuple(binding.port_ids),
            )
        port_id = binding.port_ids[0]
        port = ports.get(port_id)
        if port is None:
            raise BindingError(
                f"link {spec.role!r} was bound to port {port_id!r}, which is not "
                f"offered; the offered ports are {sorted(ports) or '(none)'}"
            )
        if not isinstance(port, GeometryPort):
            # Named through the class rather than through `port.kind`: the
            # narrowed type is the one that lacks a frame, and reading a channel
            # attribute off it would assume the very thing being reported.
            raise BindingError(
                f"link {spec.role!r} was bound to port {port_id!r}, a "
                f"{type(port).__name__} with no attachment frame; a link needs a "
                "geometric port"
            )

        body_b = spec.body_b or port.owner.local
        point_a = _point(spec.point_a_local)
        point_b = (
            _point(spec.point_b_local)
            if spec.point_b_local is not None
            else _point(port.pose.translation)
        )
        point_b_label = spec.point_b_label or port.id.local
        row_name = spec.name or f"{spec.role}_{body_b}"

        row: Any
        if spec.kind == "weld":
            row = WeldJoint(spec.body_a, point_a, body_b, point_b, name=row_name)
            joints[row_name] = {"kind": type(row).__name__, "constraint": row}
            ideal[row_name] = {"kind": type(row).__name__, "constraint": row}
        elif spec.kind == "revolute":
            row = RevoluteJoint(
                spec.body_a,
                point_a,
                _point(spec.axis_a),
                body_b,
                point_b,
                _point(spec.axis_b),
                name=row_name,
            )
            joints[row_name] = {"kind": type(row).__name__, "constraint": row}
            ideal[row_name] = {"kind": type(row).__name__, "constraint": row}
        else:
            row = BushingElement(
                name=row_name,
                body_a=spec.body_a,
                body_b=body_b,
                local_pose_a=SE3(translation=point_a, quaternion=_NO_ROTATION),
                local_pose_b=SE3(translation=point_b, quaternion=_NO_ROTATION),
                stiffness=_stiffness(spec.stiffness),
            )
            bushings[row_name] = {"kind": type(row).__name__, "row": row}

        connections[row_name] = _connection(
            row_name,
            spec.kind,
            spec.body_a,
            body_b,
            spec.point_a_label,
            point_b_label,
        )
        rows.append(
            LinkRow(
                role=spec.role,
                kind=spec.kind,
                row_name=row_name,
                row_type=type(row).__name__,
                port_id=port_id,
                body_a=spec.body_a,
                body_b=body_b,
                point_a_label=spec.point_a_label,
                point_b_label=point_b_label,
                point_a=(float(point_a[0]), float(point_a[1]), float(point_a[2])),
                point_b=(float(point_b[0]), float(point_b[1]), float(point_b[2])),
            )
        )

    return LinkRows(
        fragment=ModelFragment(
            instance=instance,
            joints=joints,
            ideal_constraints=ideal,
            bushings=bushings,
            connections=connections,
        ),
        rows=tuple(rows),
    )


def explicit_bindings_from_pairings(
    pairings: Mapping[str, str],
    ports: Mapping[str, PortSpec],
    *,
    instance: tuple[str, ...],
) -> dict[str, str]:
    """
    Render a document's pairings into the mapping ``match_requirements`` takes.

    A document pairs a requirement role with a *port name*; the matcher works in
    port ids, which carry the mounting path.  Reconciling the two spellings is
    done once, here, so a pairing written in a file and one written in code
    cannot come to mean different things.

    A name this assembly does not offer is refused, naming the requirement role,
    the port name and the ports that are offered: the repair is in the file, and
    the message has to say which two names to bring together.
    """
    rendered: dict[str, str] = {}
    offered = sorted({port.id.local for port in ports.values()})
    for role, name in pairings.items():
        port_id = str(EntityId(instance, str(name)))
        if port_id not in ports:
            raise BindingError(
                f"the pairing for requirement {role!r} names port {name!r}, which "
                f"this assembly does not offer; the offered ports are "
                f"{offered or '(none)'}"
            )
        rendered[role] = port_id
    return rendered


def _connection(
    name: str, kind: str, body_a: str, body_b: str, point_a: str, point_b: str
):
    """Build the stable connection row that stands beside the joint or bushing."""
    return Connection(
        name,
        "ideal" if kind in _IDEAL_KINDS else "bushing",
        body_a,
        body_b,
        point_a,
        point_b,
    )


def _point(value: Any) -> np.ndarray:
    """Return a local point or axis as three finite floats."""
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise LinkError(
            f"a link point or axis must be three finite values, got {value!r}"
        )
    return vector


def _stiffness(value: Any) -> np.ndarray:
    """Return a bushing stiffness, defaulting to the zero matrix."""
    if value is None:
        return np.zeros((6, 6), dtype=float)
    return np.asarray(value, dtype=float)
