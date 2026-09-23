"""
The wheel subsystem: the wheel body and the tire.

Availability is deliberately asymmetric (requirement 19 / D9):

* on a full vehicle the wheel subsystem builds `wheel.body` and its spin joint,
  which is what `preparation/assembly/vehicle.py` does today;
* on a single axle it builds **no body at all**.  Adams does the same: the axle
  assembly `acar_gs_front.asy` carries only `suspension`/`steering` plus the
  suspension testrig, and the wheels come from the rig's own parameters
  (`testrig_tire_property_file='RIGID_WHEEL'`, `testrig_wheel_radius`,
  `tire_stiffness`).  The axle keeps its `VerticalTireElement` mounted on the
  upright, exactly as now; turning that into an independent wheel body would move
  both the K/C baseline and the axle dynamics baseline.

So the axle-side contribution here is the *tire* declaration.  The rig-side wheel
belongs to subtask 10.
"""

from __future__ import annotations

from .types import SIDES, ResolvedElement, Side, SubsystemContext, SubsystemOutput

__all__ = ["build", "role", "tires"]

#: The role this subsystem implements.
role = "wheel"


def build(context: SubsystemContext) -> SubsystemOutput:
    """
    Contribute the axle-side wheel content.

    Nothing on a single axle: no body (D9), and the tires are element
    declarations rather than bodies, so they come from `tires` instead.
    """
    del context
    return SubsystemOutput()


def tires(context: SubsystemContext, side: Side) -> list[ResolvedElement]:
    """
    Declare one vertical tire at `upright_{side}`'s wheel centre.

    The tire hangs off the upright, as it always has; the element constructor
    stays in `front_axle` because that module is the registered `elements`
    importer.  This function only decides what exists and where.  Names repeat
    per spec, exactly as the loop it replaces did.
    """
    local_center = context.local(
        f"upright_{side}", context.mirror(side, "wheel_center")
    )
    return [
        ResolvedElement(
            kind="tire",
            name=f"tire_{side}",
            spec=spec,
            body_a=f"upright_{side}",
            point_a=local_center,
        )
        for spec in context.model.tires
    ]


def sides() -> tuple[Side, Side]:
    """Return the sides the tire pass runs over, in assembly order."""
    return SIDES
