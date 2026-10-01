"""
The anti-roll bar subsystem: a torsion bar with a droplink per side.

Why the bar is a subsystem of its own
-------------------------------------
The bar spans **both sides of one axle**.  A per-side template cannot own it, and
that is not a stylistic preference: the suspension template is instantiated once
per side, so an anti-roll bar living there would be declared twice and would have
to reach across the axle to the other side's template -- exactly the cross-side
guessing the assembly layer forbids (``EPIC.md:233``).  Declaring it here, once,
is what lets the bar name its own four mounts and its own parts.

Which physics this module uses, and why
---------------------------------------
Two anti-roll-bar physics exist in this repository and they are **not**
interchangeable (``tasks/p4-01-freeze/raw/arb_two_physics.md``):

* ``modeling/primitives/elements.py::AntiRollBarElement`` -- an *elastic* member
  whose couple is ``torsional_stiffness * (right rise - left rise)``: the bar
  resists the difference in vertical travel of its two ends, and stores
  ``0.5 * k * difference^2``.
* the kernel's torsional bar (``cpp/src/element/anti_roll.cpp``) -- a couple from
  the bar's own *twist angle and rate*, with damping, delivered as a pure moment
  pair.  The preparation layer refuses to treat the two as equivalent
  (``preparation/vehicle_dynamic.py``, "is not equivalent"), and that refusal is
  left standing.

**This subsystem uses the elastic link law.**  A torsion bar is an elastic
member; the alternative in this repository, ``RotationalTorqueElement``, is a
*driven actuator* whose amplitude is ``min(stiffness * demand, max_torque)`` --
it carries no stored energy and takes its magnitude from a driver demand.  Using
it for the bar would state the physics wrongly, so it is not used here.

What each part is
-----------------
``torsion_bar_L`` / ``torsion_bar_R`` are the two halves of the bar, each turning
in a chassis mount; ``droplink_L`` / ``droplink_R`` connect the bar to the wheel
end.  The bar's own element is emitted once, between the two halves -- see
:func:`bar_element`.

The module reads the four mounts off the **template's own port declarations**
rather than naming any body, which is the same rule ``brake`` and ``drive``
follow for their reaction bodies.
"""

from __future__ import annotations

from ..modeling.primitives import RigidBodyState
from ..schema import AntiRollBar
from ..templates import SubsystemInstance
from ..templates.model import Template
from .assembly import assemble_from_template
from .geometry import resolve_body
from .types import ResolvedElement, SubsystemContext, SubsystemOutput

#: The role this module implements.
role = "anti_roll_bar"

#: The built-in template this module builds, re-exported so a caller substitutes
#: the template by name the way ``brake.SIMPLIFIED_BRAKE`` does.
SIMPLIFIED_ANTI_ROLL_BAR: Template | None = None

#: The four port names, in the order the template declares them.
PORTS: tuple[str, ...] = (
    "chassis_mount_L",
    "chassis_mount_R",
    "droplink_mount_L",
    "droplink_mount_R",
)

#: The element kind the bar's own couple is carried by.  The elastic link law.
BAR_ELEMENT_KIND = "anti_roll_bar"


def _template() -> Template:
    """Return the registered anti-roll bar template."""
    from ..templates.builtin import ANTI_ROLL_BAR

    return ANTI_ROLL_BAR


def build(instance: SubsystemInstance, context: SubsystemContext) -> SubsystemOutput:
    """
    Turn an anti-roll bar instance into the bodies it declares.

    The bodies are the template's own: a torsion-bar half per side and a droplink
    per side.  Nothing is named here, so a detailed template with a bar across the
    axle and real bushes replaces this one without touching this function.
    """
    return assemble_from_template(instance, context)


def port_owners() -> dict[str, str]:
    """
    Return each of the four ports mapped to the part it is attached to.

    Read off the template's declarations rather than restated, so a template that
    moves a mount onto a different part moves the answer with it.
    """
    return {port.name: port.owner for port in _template().ports}


def bar_bodies() -> tuple[str, str]:
    """
    Return the two bar halves' body names, in left-then-right order.

    The names come from the port declarations: the part each `chassis_mount`
    hangs on is the bar half that mount carries.
    """
    owners = port_owners()
    return owners["chassis_mount_L"], owners["chassis_mount_R"]


def droplink_bodies() -> tuple[str, str]:
    """Return the two droplinks' body names, in left-then-right order."""
    owners = port_owners()
    return owners["droplink_mount_L"], owners["droplink_mount_R"]


def bar_element(spec: AntiRollBar, context: SubsystemContext) -> ResolvedElement:
    """
    Return the bar's own elastic element row, spanning the two bar halves.

    The two ends are the bar halves the *template* declares, so the row follows
    the template rather than a body-name convention.  The link points are the
    model's own, resolved into each body's frame the same way the suspension's
    other rows are.
    """
    left, right = bar_bodies()
    # The bar halves carry the side in their own names (`torsion_bar_L`), so the
    # schema's own body name resolves against this assembly's body table the same
    # way every other element's does.
    left_body = resolve_body(left.rsplit("_", 1)[0], "L", context.bodies)
    right_body = resolve_body(right.rsplit("_", 1)[0], "R", context.bodies)
    return ResolvedElement(
        kind=BAR_ELEMENT_KIND,
        name=spec.name,
        spec=spec,
        body_a=left_body,
        point_a=context.local(left_body, spec.left_link_point.as_array()),
        body_b=right_body,
        point_b=context.local(right_body, spec.right_link_point.as_array()),
    )


def configure(spec: AntiRollBar, context: SubsystemContext) -> float:
    """
    Return the bar's torsional stiffness, as the element reads it.

    One function so the number has one home, which is what a detailed template
    will replace.  The schema's bar carries no reference difference of its own --
    the link law's zero is the bar's own undeformed state -- so only the stiffness
    is read here.
    """
    del context
    return float(spec.torsional_stiffness)


def evaluate(
    spec: AntiRollBar, left: str, right: str, stiffness: float, state: RigidBodyState
):
    """
    Evaluate the bar's couple for a caller that has the two ends already resolved.

    Present so a test can exercise the law without composing an axle; the
    production path builds the row with :func:`bar_element` and lets the modelling
    layer evaluate it.
    """
    from ..modeling.primitives import AntiRollBarElement
    from ..modeling.primitives.joints import RigidBodyState as _State  # noqa: F401

    element = AntiRollBarElement(
        name=spec.name,
        left_body=left,
        left_point=spec.left_link_point.as_array(),
        right_body=right,
        right_point=spec.right_link_point.as_array(),
        stiffness=stiffness,
    )
    return element.evaluate(state)


__all__ = [
    "BAR_ELEMENT_KIND",
    "PORTS",
    "bar_bodies",
    "bar_element",
    "build",
    "configure",
    "droplink_bodies",
    "evaluate",
    "port_owners",
    "role",
]
