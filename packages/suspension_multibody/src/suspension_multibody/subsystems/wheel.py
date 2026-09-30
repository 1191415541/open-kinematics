"""
The wheel subsystem: the wheel end -- the wheel body and the tire.

This module is the **single producer** of the wheel end.  What the wheel end is
comes from one declaration -- the wheel template -- and both topologies read that
same declaration:

* the wheel template's parts are the wheel bodies, placed by the template's own
  wheel-centre mount, exactly the way every other subsystem's parts are placed;
* the template's wheel-centre mount also says which body the tire hangs on.

The built-in template declares **no parts**, and that is not an omission: on a
single axle the wheel comes from the bench and the wheel body belongs to the
model (decision D9, and Adams' own `acar_gs_front.asy` assembly behaves the same
way -- the wheels come from the rig's parameters).  The axle therefore builds no
wheel body of its own, and `tires` declares the wheel end's other half: where the
tire acts.  A wheel template that *does* declare a wheel body is honoured -- the
body is built here and the single-axle composition then condenses it into the
body that carries the wheel centre (see ``si_assembly._wheel_end_is_supplied``),
so a file may describe wheels without moving the frozen K/C baseline.

Which template is read is decided in one place, :func:`template_instance`: the
wheel template a *file* put on the request when it carries one, and the
registered built-in otherwise.  Reading the attribute rather than going through
the role table is deliberate and is stated here so it is not mistaken for
sloppiness: ``AssemblyRequest`` carries no wheel field yet (the role table in
``types._ROLE_TEMPLATE_FIELD`` lists steering and chassis), so the lookup answers
``None`` today and starts carrying a file's wheel template the moment that field
exists -- without this module being edited again.  Registering the carrier field
is the remaining half of the file-read chain and is registered as subtask 04b.
"""

from __future__ import annotations

import numpy as np

from ..modeling.primitives.joints import RigidBody
from ..templates.builtin import WHEEL
from ..templates.instantiate import SubsystemInstance, instantiate
from ..templates.model import ConnectionDefinition, TemplateError
from .geometry import body_from_part
from .types import SIDES, ResolvedElement, Side, SubsystemContext, SubsystemOutput

__all__ = [
    "build",
    "role",
    "sides",
    "template_instance",
    "tires",
    "wheel_center_body",
]

#: The role this subsystem implements.
role = "wheel"


def template_instance(context: SubsystemContext) -> SubsystemInstance:
    """Return the wheel template this assembly reads."""
    requested = _requested(context)
    if isinstance(requested, SubsystemInstance):
        return requested
    return instantiate(WHEEL, mode=context.mode)


def _requested(context: SubsystemContext) -> object | None:
    """Return the wheel template the request carries, or ``None``."""
    return getattr(context.request, "wheel_template", None)


def build(context: SubsystemContext) -> SubsystemOutput:
    """
    Contribute the wheel end's bodies, as the wheel template declares them.

    The set is the template's own part list, read rather than repeated: a
    template that declares a wheel body produces one, and the built-in -- which
    declares none -- produces none, which is the state the frozen K/C and axle
    dynamics baselines were recorded against (D9).

    A part the *rest of the assembly* already declared is skipped rather than
    re-declared.  That is not a name rule: the built-in template's wheel centre
    hangs on ``wheel_hub_L``, a part the suspension template invents, so a
    template that named it would otherwise have two subsystems claim one body --
    refused by the composition as a duplicate, and rightly.

    What a part weighs is the template's statement, the same way the hub's mass
    is (see :func:`~.geometry.body_from_part`): a template that declares the
    wheel body's mass is saying "this body exists and weighs this", and nothing
    else in the flow can say it for a body the model does not describe.
    """
    instance = template_instance(context)
    bodies: dict[str, RigidBody] = {}
    for part in instance.template.parts:
        if part.name in context.bodies:
            continue
        bodies[part.name] = body_from_part(
            part.name,
            part,
            center_of_mass=_part_center(context, instance, part.name),
        )
    return SubsystemOutput(bodies=bodies)


def _part_center(
    context: SubsystemContext, instance: SubsystemInstance, name: str
) -> np.ndarray | None:
    """Return where the template attaches one of its own parts, or ``None``."""
    side = _side_suffix(name)
    if side is None:
        return None
    return context.part_placement(name, instance.template.connections, side)


def _side_suffix(name: str) -> Side | None:
    """Return the side a part name ends in, or ``None`` for a side-less name."""
    for side in SIDES:
        if name.endswith(f"_{side}"):
            return side
    return None


def _wheel_mount(instance: SubsystemInstance, side: Side) -> ConnectionDefinition:
    """
    Return the wheel-centre mount the wheel template declares for one side.

    Found by *role* and side rather than by name, because the name is the
    template's and the role is the interface: a template that calls its wheel
    centre something else is still a wheel template, and looking the name up
    would make the spelling load-bearing again -- which is the state this
    declaration exists to end.
    """
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


def wheel_center_body(context: SubsystemContext, side: Side) -> str:
    """
    Return the body this assembly's wheel centre belongs to on one side.

    It is the template's declaration, with one fallback: a model that has no body
    for the template's `wheel_center` hardpoint at all -- an axle whose
    wheel-carrying body is the upright -- keeps its wheel on the upright.  The
    fallback is a statement about the *assembly*, not about a template name: it
    only fires when the declared owner is not a body this assembly carries.
    """
    owner = _wheel_mount(template_instance(context), side).owner
    if owner not in context.bodies and f"upright_{side}" in context.bodies:
        return f"upright_{side}"
    return owner


def tires(context: SubsystemContext, side: Side) -> list[ResolvedElement]:
    """
    Declare one vertical tire per model tire, where the template says it hangs.

    The template states the *attachment* -- which body carries the wheel centre
    and which hardpoint role locates it -- while the law is the model's own
    `model.tires`, exactly as before.  The element constructor stays in
    `element_build` because that module is the registered `elements` importer;
    this function only decides what exists and where.
    """
    owner = wheel_center_body(context, side)
    mount = _wheel_mount(template_instance(context), side)
    local_center = context.local(owner, context.mirror(side, mount.role))
    return [
        ResolvedElement(
            kind="tire",
            name=f"tire_{side}",
            spec=spec,
            body_a=owner,
            point_a=local_center,
        )
        for spec in context.model.tires
    ]

def sides(context: SubsystemContext) -> tuple[Side, ...]:
    """
    Return the sides the tire pass runs over, in assembly order.

    The assembly's own declaration, not the symmetric pair: a one-sided corner has
    one tire, and asking a two-sided constant for it would declare a tire on a
    side that has no wheel to carry it.
    """
    return tuple(context.request.sides)
