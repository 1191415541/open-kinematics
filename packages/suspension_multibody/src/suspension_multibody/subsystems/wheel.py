"""
The wheel subsystem: the wheel body and the tire.

Availability is deliberately asymmetric (requirement 19 / D9):

* on a full vehicle the wheel subsystem builds `wheel.body` and its spin joint,
  which is what `subsystems/vehicle_parts.py` does today;
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

from ..templates.builtin import WHEEL
from ..templates.instantiate import SubsystemInstance, instantiate
from ..templates.model import ConnectionDefinition, TemplateError
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


def _instance(context: SubsystemContext) -> SubsystemInstance:
    """
    Return the wheel template to read: the registered built-in.

    The built-in is the only wheel template that can be read, and the reason is
    the file format's rather than a choice made here: the wheel centre is a
    *per-side* mount, the conversion mirrors a per-side mount by its owner's side
    token, and a template may only own its own bodies -- while on an axle the wheel
    role builds no body at all (decision D9), because the wheel comes from the rig
    and the wheel body belongs to the model.  A file therefore has no body to hang
    a wheel centre on, and a document's wheel subsystem is not a wheel topology.

    So the attachment stays the built-in's, which names the suspension's upright --
    the body that does carry the wheel centre.
    """
    return instantiate(WHEEL, mode=context.mode)


def _wheel_mount(context: SubsystemContext, side: Side) -> ConnectionDefinition:
    """
    Return the wheel-centre mount the wheel template declares for one side.

    Found by *role* and side rather than by name, because the name is the
    template's and the role is the interface: a template that calls its wheel
    centre something else is still a wheel template, and looking the name up
    would make the spelling load-bearing again -- which is the state this
    declaration exists to end.
    """
    instance = _instance(context)
    matches = [
        connection
        for connection in instance.template.connections
        if connection.role == "wheel_center" and connection.owner.endswith(f"_{side}")
    ]
    if len(matches) != 1:
        raise TemplateError(
            f"wheel template {instance.template.name!r} declares {len(matches)} "
            f"wheel-centre mounts for side {side!r}; the role needs exactly one, "
            "and a tire with nowhere to hang is not a wheel"
        )
    return matches[0]


def tires(context: SubsystemContext, side: Side) -> list[ResolvedElement]:
    """
    Declare one vertical tire per model tire, where the template says it hangs.

    The template states the *attachment* -- which body carries the wheel centre
    and which hardpoint role locates it -- while the law is the model's own
    `model.tires`, exactly as before.  The element constructor stays in
    `front_axle` because that module is the registered `elements` importer; this
    function only decides what exists and where.
    """
    mount = _wheel_mount(context, side)
    local_center = context.local(mount.owner, context.mirror(side, mount.role))
    return [
        ResolvedElement(
            kind="tire",
            name=f"tire_{side}",
            spec=spec,
            body_a=mount.owner,
            point_a=local_center,
        )
        for spec in context.model.tires
    ]


def sides() -> tuple[Side, Side]:
    """Return the sides the tire pass runs over, in assembly order."""
    return SIDES
