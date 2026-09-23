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

from ..preparation.assembly.types import RigidBody
from ..preparation.geometry import SE3
from .types import SubsystemContext, SubsystemOutput

__all__ = ["build", "role"]

#: The role this subsystem implements.
role = "chassis"


def build(context: SubsystemContext) -> SubsystemOutput:
    """
    Contribute the axle's fixed chassis body.

    The body carries mass specs from `model.bodies` later, in the assembly's
    `_with_body_specs` pass, so that the mass table keeps its single authority.
    """
    del context  # 轴侧 chassis 不读 MassSpec，也不读任何硬点
    return SubsystemOutput(
        bodies={
            "chassis": RigidBody(
                "chassis",
                pose=SE3.identity(),
                inertia=np.eye(3),
                center_of_mass=np.zeros(3),
                fixed=True,
            )
        }
    )
