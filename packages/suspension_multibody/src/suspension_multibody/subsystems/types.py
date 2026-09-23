"""
What a subsystem hands back to the assembly.

A subsystem does not construct force elements: the assembly layer owns that, and
`legacy_surface_gate` only tolerates the existing registered importers.  So a
subsystem returns *declarations* -- bodies, points, connections, constraints --
and the assembly turns them into runtime objects.

Each piece is additive: a subsystem contributes its own slice and the assembly
concatenates the slices in the order it always has, so splitting the build apart
cannot reorder anything.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np

from ..preparation.assembly.types import Constraint, RigidBody

if TYPE_CHECKING:
    # `Connection` lives in `front_axle`, which is also the module that will
    # import this package.  Importing it at runtime would close that cycle; the
    # annotation is all this module needs from it.
    from ..preparation.assembly.front_axle import Connection

__all__ = ["SubsystemOutput", "merge_outputs"]


@dataclass
class SubsystemOutput:
    """
    One subsystem's contribution to an axle assembly.

    `bodies` and `points` are ordered mappings because their order is part of the
    assembly's contract: the contract document lists bodies in this order.  A
    subsystem appends; only the assembly decides the sequence.
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
    #: Compliant elements the subsystem declares (the assembly builds them).
    bushings: list[object] = field(default_factory=list)


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
    return merged
