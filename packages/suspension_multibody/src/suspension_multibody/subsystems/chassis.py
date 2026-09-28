"""
The chassis subsystem: the ground-side body an axle reacts against.

On a single axle the chassis is a fixed reference body with no mass.  That is not
an oversight to be "fixed" later: `FrontAxleModel.mass` (`MassSpec`) is never read
by the axle assembly, upstream or here, and the whole K/C baseline is recorded
against an inertialess fixed chassis.  Consuming `MassSpec` here would change
every axle result, so the fact is preserved and recorded rather than repaired.
"""

from __future__ import annotations

import numpy as np

from ..modeling.primitives.joints import RigidBody
from ..modeling.primitives.spatial import SE3
from ..templates.builtin import CHASSIS
from ..templates.instantiate import SubsystemInstance, instantiate
from .types import SubsystemContext, SubsystemOutput

__all__ = ["build", "role"]

#: The role this subsystem implements.
role = "chassis"


def build(context: SubsystemContext) -> SubsystemOutput:
    """
    Contribute the axle's fixed chassis body, as the chassis template declares it.

    The *declaration* is the template's: which bodies the role has, and which of
    them are fixed.  The pose and the inertia are the axle's own, because the axle
    side consumes no `MassSpec` -- `_with_body_specs` fills the mass table later,
    so that table keeps its single authority.

    Reading the declaration rather than repeating it is what makes a chassis
    template replaceable: a template that declared a second body would produce a
    second body here, which is the property that was missing while the name was a
    literal in this function.  A *file* subsystem's chassis template reaches here
    through the request, so an authored chassis is the one that is built.
    """
    requested = context.request.role_instance("chassis")
    instance = (
        requested
        if isinstance(requested, SubsystemInstance)
        else instantiate(CHASSIS, mode=context.mode)
    )
    return SubsystemOutput(
        bodies={
            part.name: RigidBody(
                part.name,
                pose=SE3.identity(),
                inertia=np.eye(3),
                center_of_mass=np.zeros(3),
                fixed=part.fixed,
            )
            for part in instance.template.parts
        }
    )
