"""
The drive subsystem: a torque-element powertrain under the `drive` role.

Like the simplified brake, this template owns **no bodies**.  It declares the
motor-to-wheel transfer and emits one **torque element** per driven wheel,
between the wheel it turns and the body that reacts it; it does not build an
engine, a gearbox, a differential or a driveshaft.  Adams Car's inactive-powertrain
path (`pvs_driveline=0`) is the same state of affairs, and its `_powertrain.tpl`
is a sibling template under the same role rather than a different mechanism.

The slot set is the standardized one of the roadmap's 2.1 section:
``gear_ratio`` / ``efficiency`` / ``max_torque``.  The element's wheel torque is

    T = max_torque * gear_ratio * efficiency * share * drive_input

which is the driving branch of `preparation/vehicle_dynamic.py`'s
`_build_wheel_torque_signals` (its `:1585-1588`) with the transfer written out:
that builder applies `maximum_drive_torque` to the wheel directly, so the two
agree exactly while the transfer is the identity.  ``gear_ratio`` and
``efficiency`` therefore default to ``1.0`` **on purpose** and not because the
roadmap's names were filled in with a placeholder: an identity transfer is what
makes this element reproduce the recorded wheel torque value for value, and a
wheel-hub motor with a reduction states its own ratio, at which point
``max_torque`` reads as the motor's torque rather than the wheel's.  The
multiplication is written in the source's own association order so the floats are
bit-identical, not merely close.

Two things this module deliberately does **not** do:

* it does not apply a unit scale -- the caller's `scale` belongs to the
  preparation layer, and a subsystem that silently scaled would double-count it;
* it does not decide availability.  "Only the full vehicle has a drive" is an
  assembly-level declaration (requirement 17 / D8), not a property of the role.

``driven_wheels`` and ``drive_split`` no longer appear as slots.  They were
declared "for the role contract's sake" -- placeholders for facts that live on
`:class:`~suspension_multibody.schema.DrivelineSpec` -- and with one element per
wheel the two facts are stated by the elements themselves: a wheel is driven
because it has an element, and its share is the element's own ``share``.  Keeping
a second, weaker representation of the same facts in the template is what this
removal ends.  ``DrivelineSpec`` still validates them, and this module's
`drive_amplitude` refuses a non-zero torque with no driven wheel exactly as the
live builder does.

**Which body reacts the couple is not decided here.**  The driven wheel is the
requiring side's own body and is passed in; the reaction end is read off a matched
port's owner by
:func:`~suspension_multibody.compilation.element_blocks.pair_torque_bodies`, so a
subframe and a vehicle body both meet the requirement without either name
appearing here.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import cast

import numpy as np

from ..modeling.ports import PortSpec
from ..modeling.primitives.elements import RotationalTorqueParameters
from ..schema import DrivelineSpec
from ..templates import (
    SubsystemInstance,
    Template,
    register,
)
from ..templates.builtin import DRIVE
from .assembly import assemble_from_template
from .types import WHEELS, ResolvedElement, SubsystemContext, SubsystemOutput

__all__ = [
    "DRIVE_REACTION_ROLE",
    "SIMPLIFIED_DRIVE",
    "SIMPLIFIED_DRIVE_NAME",
    "build",
    "drive_amplitude",
    "register_simplified",
    "role",
    "torque_parameters",
    "wheel_shares",
    "wheel_torque_element",
]

#: The role this subsystem implements.
role = "drive"

#: The name the simplified template registers under.  A detailed powertrain
#: template is a sibling name in the same registry, not a different code path.
SIMPLIFIED_DRIVE_NAME = "powertrain_simplified"

#: The requirement role the couple's reaction end comes from.
#:
#: A *role*, never a body name: the reaction body is the matched port's owner, so
#: a subframe or a vehicle body meets it without this module knowing either name.
#: Declaring the matching need on the template belongs to the composition path;
#: ``template_document_from`` does not write ``needs`` today, so a need stated on
#: the built-in template would be dropped by the export/read round trip --
#: recorded in ``raw/reaction_paths.md`` rather than papered over.
DRIVE_REACTION_ROLE = "drive_reaction"

#: The simplified driveline's own slots.  These are the role's required slots with
#: the transfer's identity values as defaults, so the template carries its own
#: numbers rather than referring to a properties file.  ``max_torque`` is zero
#: because a driveline that drives nothing is the state every existing model is
#: in; a caller that wants drive states the number.

SIMPLIFIED_DRIVE = DRIVE


def register_simplified() -> Template:
    """
    Register the simplified drive template, replacing any previous one.

    Substitution is registration: a detailed powertrain template registered under
    the same role reaches the same assembly path with no code change.
    """
    return register(SIMPLIFIED_DRIVE, replace=True)


def build(instance: SubsystemInstance, context: SubsystemContext) -> SubsystemOutput:
    """
    Contribute the bodies this drive instance declares.

    The simplified template declares none, so this returns none -- through the
    shared role path, so a template that declares an engine or a differential
    body reaches the assembly without this function changing.
    """
    return assemble_from_template(instance, context)


def wheel_shares(driveline: DrivelineSpec) -> dict[str, float]:
    """
    Return each wheel's share of the driveline's torque, in wheel order.

    The share comes from `DrivelineSpec.drive_split`, read in the wheel order the
    kernel's buffers use, and every assertion the live builder makes about it is
    made here too -- naming the offending field rather than leaving a silent zero:

    * a non-zero ``maximum_drive_torque`` with no ``driven_wheels`` is refused
      rather than treated as "drives nothing";
    * each driven wheel needs a positive share, because a zero share would emit a
      silent zero for a wheel the model claims to drive;
    * a wheel outside the four corners cannot appear, which `DrivelineSpec`'s own
      validator already enforces.

    The shares are returned for all four wheels -- an undriven wheel's share is
    simply not read, because it has no element -- so a caller does not have to ask
    which wheels the mapping is keyed for.
    """
    shares = dict(zip(WHEELS, driveline.drive_split))
    driven_wheels = driveline.driven_wheels
    if driveline.maximum_drive_torque > 0.0 and not driven_wheels:
        raise ValueError("drive torque requires driveline.driven_wheels")
    for name in sorted(set(driven_wheels)):
        if shares[name] <= 0.0:
            raise ValueError(
                f"each driven wheel requires a positive drive_split; {name!r} "
                f"has {shares[name]!r}"
            )
    return shares


def drive_amplitude(
    driveline: DrivelineSpec,
    *,
    slots: Mapping[str, float],
    wheel: str,
    drive: float,
) -> float:
    """
    Return one wheel's drive torque for a stated driveline input.

    ``slots`` is the template's resolved property map -- ``gear_ratio``,
    ``efficiency`` and ``max_torque`` -- and it is what makes the element
    template-driven rather than a second copy of the driveline's fields.  The
    arithmetic is the live builder's, with the transfer written out::

        T = max_torque * gear_ratio * efficiency * share * drive_input

    ``drive`` is the normalized drive input and must be within ``[-1, 1]``;
    reverse drive is a negative torque and is not absolute-valued.  An undriven
    wheel gets zero -- not a share of the torque -- because "the model does not
    drive this wheel" is a state the caller must be able to read off the number.
    """
    drive_value = float(drive)
    if drive_value < -1.0 or drive_value > 1.0:
        raise ValueError(
            f"drive_input must be normalized to [-1, 1]; got {drive_value!r}"
        )
    if wheel not in WHEELS:
        raise ValueError(
            f"unknown wheel {wheel!r}; a drive element is named for one of the "
            f"four corner names {list(WHEELS)}"
        )
    if wheel not in set(driveline.driven_wheels):
        return 0.0
    max_torque = float(slots["max_torque"])
    gear_ratio = float(slots["gear_ratio"])
    efficiency = float(slots["efficiency"])
    share = wheel_shares(driveline)[wheel]
    return max_torque * gear_ratio * efficiency * share * drive_value


def torque_parameters(
    driveline: DrivelineSpec,
    *,
    slots: Mapping[str, float],
    wheel: str,
    drive: float,
    axis_a: object | None = None,
    demand_source: int = 0,
    demand_tire: int = -1,
    gain_scale: float = 1.0,
    reaction_role: str | None = None,
) -> RotationalTorqueParameters:
    """
    Return the couple one driven wheel's element carries.

    ``stiffness`` is the magnitude at this input and ``max_torque`` the magnitude
    at full input, so the element law's cap cannot bite below the input the
    caller asked for.

    **The magnitude, not the signed torque, and that is a recorded limitation of
    the landed kernel family rather than a choice made here.**  ``RotationalTorque``'s
    law (``cpp/src/element/anti_roll.cpp:135-139``) is a *resistance* law: the
    couple opposes the real-time relative rate, with the demand channel still
    hardcoded to ``1.0`` at ``:132``.  A negative gain is refused outright by
    ``RotationalTorqueParameters`` (``modeling/primitives/elements.py:667-670``)
    and a signed couple would need a discriminating demand, so an element built
    on this family today applies the demanded *magnitude* against the pair's
    relative motion: a drive element therefore decelerates its wheel instead of
    accelerating it, and reverse drive is not distinguishable from forward drive
    at the element.  ``drive_amplitude`` keeps the live builder's signed value so
    the parity with ``_build_wheel_torque_signals`` is still exact; carrying that
    sign into the couple is a kernel-side change belonging to the family's demand
    channel (p2-02) and is registered in ``raw/reaction_paths.md`` rather than
    worked around here.

    ``axis_a`` is the wheel's spin axis **in the reaction body's frame**, because
    that is the frame the element law reads.  ``None`` means the body-frame y
    axis, which is the lateral axis for the built-in topology; a model whose spin
    axis is not its body's y axis passes the axis explicitly -- the role requires
    a ``spin_axis`` mount for exactly this.
    """
    amplitude = drive_amplitude(driveline, slots=slots, wheel=wheel, drive=drive)
    cap = drive_amplitude(driveline, slots=slots, wheel=wheel, drive=1.0)
    axis = (
        np.array([0.0, 1.0, 0.0])
        if axis_a is None
        else np.asarray(axis_a, dtype=float)
    )
    return RotationalTorqueParameters(
        stiffness=abs(amplitude) * gain_scale,
        max_torque=cap * gain_scale,
        axis_a=axis,
        demand_source=demand_source,
        demand_tire=demand_tire,
    )


def wheel_torque_element(
    driveline: DrivelineSpec,
    *,
    slots: Mapping[str, float],
    wheel: str,
    own_body: str,
    report: object,
    ports: Mapping[str, PortSpec],
    drive: float,
    axis_a: object | None = None,
    demand_source: int = 0,
    demand_tire: int = -1,
    gain_scale: float = 1.0,
    reaction_role: str | None = None,
) -> ResolvedElement:
    """
    Return the drive torque element one driven wheel carries, as a declared row.

    ``own_body`` is the body the couple turns -- the wheel, which the needing side
    states exactly as a ``LinkSpec`` states ``body_a``.  The reaction end is the
    matched port's owner, read by the pairing step, so a subframe and a vehicle
    body both meet the requirement without either name appearing here.

    The compilation step is imported inside the call rather than at module scope
    for the same reason `brake.py` does: the ``compilation`` package reaches
    ``rigs`` and ``studies`` through its own import, and a subsystem that pulled
    that chain in at import time would make composing a bare axle pay for it.
    """
    from ..compilation.element_blocks import pair_torque_bodies, torque_element_row

    pairing = pair_torque_bodies(
        name=f"drive_{wheel}",
        role=DRIVE_REACTION_ROLE if reaction_role is None else reaction_role,
        own_body=own_body,
        report=report,
        ports=ports,
    )
    element = torque_element_row(
        pairing,
        torque_parameters(
            driveline, slots=slots, wheel=wheel, drive=drive, axis_a=axis_a,
            demand_source=demand_source, demand_tire=demand_tire,
            gain_scale=gain_scale,
        ),
    )
    return cast(ResolvedElement, element)
