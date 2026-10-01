"""
Two steps between a port pairing and the kernel's element block.

A force element reaches the kernel one of two ways: as an entry of a *contract
document*, which the family emitters author, or as an ``ElementBlock`` on the
generic ``mb_core_run`` surface.  The rotational actuator has no document route
yet -- the document reader names its types from a fixed list, and that list is the
kernel's -- so this module is the second route, and it is the only place in the
product that produces an element block.

It holds the two steps, in the order a caller performs them:

``pair_torque_bodies``
    Which two bodies the couple acts between.  Taken from a resolved
    :class:`~suspension_multibody.connections.matcher.MatchReport` and the ports
    it matched against -- never from a body name.  The rule is the one
    :mod:`~suspension_multibody.connections.links` already applies to a matched
    port (``body_b = spec.body_b or port.owner.local``): the neighbour's body is
    whatever the matched port says it is.
``rotational_torque_block``
    That element as the numbers an ``ElementBlock`` carries: the family's kind,
    the two body indices, and the parameter slots the kernel's own layout table
    declares for this family.

The block itself is *declarative* rather than a ctypes struct.  ``ElementBlock``
belongs to the kernel's ABI, and the convention in this repository is that the
structures of the generic core surface are mirrored by whoever calls it rather
than re-declared by the product -- ``tests/architecture/test_core_abi.py`` states
that on purpose.  So a row is a tuple of plain numbers, sized to the ABI's block,
and filling the struct is a copy.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field

import numpy as np

from ..modeling.ports import GeometryPort, PortSpec
from ..modeling.primitives.elements import (
    RotationalTorqueElement,
    RotationalTorqueParameters,
)

__all__ = [
    "ELEMENT_BLOCK_INT_SIZE",
    "ELEMENT_BLOCK_SIZE",
    "ELEMENT_CURVE_SLOTS",
    "ELEMENT_ROTATIONAL_TORQUE",
    "ROTATIONAL_TORQUE_AXIS_A_INDEX",
    "ROTATIONAL_TORQUE_DAMPING_INDEX",
    "ROTATIONAL_TORQUE_DEMAND_SOURCE_INT",
    "ROTATIONAL_TORQUE_DEMAND_TIRE_INT",
    "ROTATIONAL_TORQUE_MAX_TORQUE_INDEX",
    "ROTATIONAL_TORQUE_REFERENCE_QUATERNION_INDEX",
    "ROTATIONAL_TORQUE_STIFFNESS_INDEX",
    "ElementBlockRow",
    "TorquePairing",
    "pair_torque_bodies",
    "rotational_torque_block",
    "torque_element_row",
]

#: The family's ``ElementKind`` value.  Part of the ABI: an element block's
#: ``kind`` field selects the reader branch, so this number is the whole family.
ELEMENT_ROTATIONAL_TORQUE = 7
#: The parameter slots this family's row of ``kElementLayouts`` declares.  They are
#: indices into the block's parameter array, not offsets into a struct: the array
#: is uniform and every family reads its own run out of it.
ROTATIONAL_TORQUE_STIFFNESS_INDEX = 128
ROTATIONAL_TORQUE_DAMPING_INDEX = 129
#: Three consecutive doubles: the axis in ``body_a``'s local frame.
ROTATIONAL_TORQUE_AXIS_A_INDEX = 130
#: Four consecutive doubles: ``body_a``'s reference orientation.
ROTATIONAL_TORQUE_REFERENCE_QUATERNION_INDEX = 133
ROTATIONAL_TORQUE_MAX_TORQUE_INDEX = 137

#: ``kElementBlockSize``: how many doubles one block's parameter array holds.
ELEMENT_BLOCK_SIZE = 216
#: ``kElementIntBlockSize``: how many ints one block's integer array holds.
ELEMENT_BLOCK_INT_SIZE = 16
#: ``kElementCurveSlots``: how many curve references each element is allotted.
#: This family declares none, but the array is per-element and a caller sizing it
#: needs the number.
ELEMENT_CURVE_SLOTS = 8


#: The two integer slots the demand channel occupies, matching the kernel's own
#: ``ELEMENT_INT_TORQUE_DEMAND_SOURCE`` / ``ELEMENT_INT_TORQUE_TIRE``.  These are
#: indices into the block's integer array rather than parameter slots: what the
#: element follows is a choice, not a magnitude.
ROTATIONAL_TORQUE_DEMAND_SOURCE_INT = 6
ROTATIONAL_TORQUE_DEMAND_TIRE_INT = 7

#: The three demand sources, as the kernel's ``TorqueDemandSource`` spells them.
TORQUE_DEMAND_UNIT = 0
TORQUE_DEMAND_WHEEL = 1
TORQUE_DEMAND_BRAKE = 2


class ElementBlockError(ValueError):
    """An element cannot be stated as a kernel block."""


@dataclass(frozen=True)
class TorquePairing:
    """
    The two bodies one torque element acts between, and where they came from.

    ``driven_body`` receives the couple and ``reaction_body`` receives its equal
    and opposite: that is the native element's own convention, stated in terms of
    the bodies rather than in terms of ``a``/``b`` so a reader of this value does
    not have to know the kernel's field names.

    ``port_id`` and ``role`` are kept so the decision can be audited afterwards:
    which requirement was met, by which offered port, is what makes this a pairing
    rather than an arbitrary pair of names.
    """

    name: str
    role: str
    port_id: str
    driven_body: str
    reaction_body: str


@dataclass(frozen=True)
class ElementBlockRow:
    """
    One element as the numbers an ``ElementBlock`` carries.

    ``parameters`` and ``ints`` are the full ABI arrays, zero-filled outside the
    family's own run, so a caller fills a block by assigning them wholesale.  That
    is deliberate: a row that carried only its family's slots would leave the
    caller to decide what the rest of a block means, and the padding *is* part of
    the ABI.
    """

    kind: int
    body_a: int
    body_b: int
    parameters: tuple[float, ...] = field(default=())
    ints: tuple[int, ...] = field(default=())


def pair_torque_bodies(
    *,
    name: str,
    role: str,
    own_body: str,
    report: object,
    ports: Mapping[str, PortSpec],
) -> TorquePairing:
    """
    Resolve the two bodies of one torque element from a binding, by port.

    ``report`` is what
    :func:`~suspension_multibody.connections.matcher.match_requirements` returned
    and ``ports`` the offered-port mapping it matched against, so this function
    consumes the matcher's decision instead of making a second one.  ``role`` names
    the requirement the element fills -- the same role the needing contribution
    declared and the same name the caller would have written for a link recipe.
    ``own_body`` is the needing side's own body: a contribution states it, exactly
    as a ``LinkSpec`` states ``body_a``, because a requirement says what a
    neighbour must offer and never which of its own bodies is in play.

    The neighbour end is the matched port's **owner body**, read from
    ``port.owner.local``.  A port that carries no owner body -- a channel port, or
    a name this assembly does not offer -- is refused by name rather than replaced
    with a guess: a substitution here would produce a model that runs and answers
    about the wrong pair.
    """
    from ..connections.matcher import BindingError

    if not name:
        raise ElementBlockError("a torque pairing needs the element's name")
    if not own_body:
        raise ElementBlockError(
            f"torque element {name!r} needs the requiring side's own body; a "
            "requirement does not name one, so the contribution has to state it"
        )

    binding_for = getattr(report, "binding_for", None)
    binding = binding_for(role) if callable(binding_for) else None
    if binding is None or not binding.port_ids:
        raise BindingError(
            f"torque element {name!r} names requirement {role!r}, which this "
            "assembly did not bind; the bound requirements are "
            f"{sorted(b.requirement.role for b in getattr(report, 'bindings', ())) or '(none)'}"
        )
    if len(binding.port_ids) > 1:
        raise BindingError(
            f"torque element {name!r} names requirement {role!r}, which was bound "
            f"to {len(binding.port_ids)} ports {list(binding.port_ids)}; a couple "
            "acts between two bodies, so the pairing has to say which one is meant"
        )
    port_id = binding.port_ids[0]
    port = ports.get(port_id)
    if port is None:
        raise BindingError(
            f"torque element {name!r} was bound to port {port_id!r}, which is not "
            f"offered; the offered ports are {sorted(ports) or '(none)'}"
        )
    if not isinstance(port, GeometryPort):
        # Named through the class rather than through `port.kind`: the narrowed
        # type is the one whose owner is a body, and reading a frame attribute off
        # it would assume the very thing being reported.
        raise BindingError(
            f"torque element {name!r} was bound to port {port_id!r}, a "
            f"{type(port).__name__}; a couple's reaction needs a port whose owner is "
            "a body"
        )
    return TorquePairing(
        name=name,
        role=role,
        port_id=port_id,
        # The couple is applied to the requiring side and reacts on the neighbour:
        # the element drives the body that asked for the connection.
        driven_body=own_body,
        reaction_body=port.owner.local,
    )


def torque_element_row(
    pairing: TorquePairing, parameters: RotationalTorqueParameters
) -> object:
    """
    Return the element declaration the construction layer builds.

    The row's two bodies are the pairing's, so the whole path from "which port met
    which requirement" to the built element has no step that reads a body name.
    """
    from ..subsystems.types import ResolvedElement

    return ResolvedElement(
        kind="rotational_torque",
        name=pairing.name,
        spec=parameters,
        body_a=pairing.reaction_body,
        body_b=pairing.driven_body,
    )


def rotational_torque_block(
    element: RotationalTorqueElement, *, body_index: Mapping[str, int]
) -> ElementBlockRow:
    """
    Encode one built torque element as the numbers its element block carries.

    ``body_index`` maps a body name to its index in the model's body list, which
    is the order the caller hands the kernel.  A name the mapping does not carry is
    refused: an element silently pointed at body 0 would apply its couple to
    whatever body happened to be first.
    """
    for end, body in (("reaction", element.body_a), ("driven", element.body_b)):
        if body not in body_index:
            raise ElementBlockError(
                f"torque element {element.name!r} names {end} body {body!r}, which "
                f"the model does not carry; its bodies are "
                f"{sorted(body_index) or '(none)'}"
            )
    parameters = element.parameters
    values = [0.0] * ELEMENT_BLOCK_SIZE
    values[ROTATIONAL_TORQUE_STIFFNESS_INDEX] = float(parameters.stiffness)
    values[ROTATIONAL_TORQUE_DAMPING_INDEX] = float(parameters.damping)
    for offset, value in enumerate(np.asarray(parameters.axis_a, dtype=float)):
        values[ROTATIONAL_TORQUE_AXIS_A_INDEX + offset] = float(value)
    for offset, value in enumerate(
        np.asarray(parameters.reference_quaternion, dtype=float)
    ):
        values[ROTATIONAL_TORQUE_REFERENCE_QUATERNION_INDEX + offset] = float(value)
    values[ROTATIONAL_TORQUE_MAX_TORQUE_INDEX] = float(parameters.max_torque)
    # The demand channel's two slots (p2-08).  `parameters.demand_source` names
    # the driver signal the law follows and `parameters.demand_tire` which of the
    # case's tire columns it reads; the pair describes the element, so it travels
    # with the element the way its stiffness does.  They were zero and zero
    # before this field existed, which is exactly the unit demand with no tire --
    # so a caller that states neither gets the pre-p2-08 couple.
    ints = [0] * ELEMENT_BLOCK_INT_SIZE
    ints[ROTATIONAL_TORQUE_DEMAND_SOURCE_INT] = int(parameters.demand_source)
    ints[ROTATIONAL_TORQUE_DEMAND_TIRE_INT] = int(parameters.demand_tire)
    return ElementBlockRow(
        kind=ELEMENT_ROTATIONAL_TORQUE,
        # `body_a` is the reaction end and `body_b` the driven one, matching the
        # native element's fields: swapping them reverses every couple it applies.
        body_a=body_index[element.body_a],
        body_b=body_index[element.body_b],
        parameters=tuple(values),
        ints=tuple(ints),
    )
