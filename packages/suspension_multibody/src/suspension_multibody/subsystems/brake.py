"""
The brake subsystem: a simplified, torque-only brake under the `brake` role.

The simplified template owns **no bodies at all**.  It does not build a caliper,
a rotor or a brake disc; what it contributes is a *distribution* and a per-wheel
torque.  Adams Car's own simplified brake is the same thing: the 0-body
`_brake_system_4Wdisk.tpl` sits under `MAJOR_ROLE='brake_system'` next to the
4-body `_brake_system_4Wdisk_calipers.tpl`, and the assembly does not know which
one it got.

Three things are deliberately true of this module, because they are what makes
the simplified brake replaceable rather than a dead end (requirement 20 / D11):

* the torque is a **non-negative magnitude**.  Which way it acts is decided by
  the kernel from the sign of the wheel's axial speed (the Adams source says the
  same thing with `STEP(<wheel speed>,-10,1,10,-1)`; the Python side must not
  flip the sign, and does not);
* no unit scale is applied here.  The kernel's `brake_torque` channel and the
  callers that feed it own unit conversion;
* the parameters are the role's slots, not this template's.  A detailed template
  may declare the same slots and add caliper and rotor parts; it is registered
  under that role and assembled through the same path (see `assembly.py`).

The amplitude shape is transcribed from the frozen Adams document
`artifacts/adams-fiala-handling/step_steer/adams_raw/handling_step_steer_dynamic.adm`
(`SFORCE/31-34`)::

    T = 2 * piston_area * bias_share * demand * max_brake_value
        * brake_mu * effective_piston_radius

with `bias_share` the front axle's `front_brake_bias` and the rear axle's
`1 - front_brake_bias`.  Two facts about that document are recorded rather than
smoothed over:

* the constant `0.1` (this module's `max_brake_value`) comes from the source
  document's demand scaling; its own provenance in the `.sub` files it was
  generated from could not be verified here -- it is an *unverified* constant,
  not a derived one;
* the document carries two effective piston radii (145.0 front, 130.0 rear),
  while the role's parameter subset (decision D10) allows one.  This module takes
  the parameter the role defines and does not invent a second one; the rear-axle
  deviation that follows is a recorded deviation, not a hidden correction.

The authoritative arithmetic check lives in `raw/brake_torque_evidence.md` in
subtask 04's directory.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping

from ..schema import TimeSignal
from ..templates import (
    ConnectionDefinition,
    OutputDeclaration,
    PropertySlot,
    SubsystemInstance,
    Template,
    register,
)
from .assembly import assemble_from_template
from .types import WHEELS, SubsystemContext, SubsystemOutput

__all__ = [
    "SIMPLIFIED_BRAKE",
    "SIMPLIFIED_BRAKE_NAME",
    "bias_share",
    "build",
    "register_simplified",
    "role",
    "wheel_torque_amplitudes",
]

#: The role this subsystem implements.
role = "brake"

#: The name the simplified template registers under.  A detailed template is a
#: different name in the same registry, not a different code path.
SIMPLIFIED_BRAKE_NAME = "brake_4wdisk_simplified"

#: The two hardpoint roles the brake role requires, declared as attachment points
#: with no constraint of their own: the point locates the wheel the torque acts
#: on.  Neither column applies, which is what `ConnectionDefinition` expresses by
#: leaving `joint` and `bushing` unset.
_BRAKE_MOUNTS: tuple[ConnectionDefinition, ...] = tuple(
    ConnectionDefinition(f"{mount}_{side}", mount)
    for mount in ("wheel_center", "spin_axis")
    for side in ("L", "R")
)

#: The simplified brake's own numbers.  These are the role's required slots with
#: the Adams simple-brake values as defaults, so the template carries its own
#: parameters rather than referring to a properties file.  `front_brake_bias`
#: coincides with `DrivelineSpec.front_brake_bias`; the other four are the
#: torque subset of decision D10 (rotor geometry is the detailed template's).
SIMPLIFIED_BRAKE = Template(
    name=SIMPLIFIED_BRAKE_NAME,
    role="brake",
    #: Zero parts is the point, not an omission: see the module docstring.
    parts=(),
    connections=_BRAKE_MOUNTS,
    property_slots=(
        PropertySlot("brake_mu", "-", default=0.4),
        PropertySlot("piston_area", "mm^2", default=2500.0),
        PropertySlot("effective_piston_radius", "mm", default=145.0),
        PropertySlot("front_brake_bias", "-", default=0.6),
        PropertySlot("max_brake_value", "-", default=0.1),
    ),
    outputs=(OutputDeclaration("brake_torque", "N*mm", "kernel"),),
    suspension_kind="brake_4wdisk",
    description=(
        "The simplified 4-wheel-disc brake: no bodies, only the per-wheel brake "
        "torque amplitude.  Direction is the kernel's, taken from the wheel's "
        "axial speed sign.  A caliper/rotor template is a sibling under the same "
        "role."
    ),
)


def register_simplified() -> Template:
    """
    Register the simplified brake template, replacing any previous one.

    Registration is the whole substitution mechanism: swapping this template for
    a detailed one is a `register` call under the same role, and nothing in the
    assembly layer changes.
    """
    return register(SIMPLIFIED_BRAKE, replace=True)


def build(instance: SubsystemInstance, context: SubsystemContext) -> SubsystemOutput:
    """
    Contribute the bodies this brake instance declares.

    The simplified template declares none, so this returns none -- but it returns
    them *through the shared role path*, so a detailed template that declares
    calipers gets them without this function changing.
    """
    return assemble_from_template(instance, context)


def bias_share(parameters: Mapping[str, float], wheel: str) -> float:
    """
    Return the braking torque share one wheel receives.

    The front axle takes `front_brake_bias`, the rear axle the remainder.  Both
    are shares of one demand, so they sum to one; a name that is neither front nor
    rear is refused rather than silently given zero, because a silent zero would
    look like a working brake that does nothing.
    """
    front_bias = float(parameters["front_brake_bias"])
    if wheel.startswith("front_"):
        return front_bias
    if wheel.startswith("rear_"):
        return 1.0 - front_bias
    raise ValueError(
        f"unknown wheel {wheel!r}; brake bias is defined for the four corner "
        f"names {list(WHEELS)}"
    )


def wheel_torque_amplitudes(
    parameters: Mapping[str, float],
    *,
    times: Iterable[float],
    demand: TimeSignal,
) -> dict[str, tuple[float, ...]]:
    """
    Return the non-negative brake torque amplitude at each wheel and time.

    The arithmetic is the Adams `SFORCE/31-34` shape, evaluated in the source's
    own multiplication order so the recorded numbers reproduce exactly::

        T = 2 * piston_area * bias_share * demand * max_brake_value
            * brake_mu * effective_piston_radius

    The demand is the normalized brake input and must be within `[0, 1]`; the
    range check names `brake_input` so a bad case file says which signal is wrong.

    No sign is applied and no unit scale is multiplied in.  A caller that wants
    the applied torque hands these magnitudes to the kernel's `brake_torque`
    channel, which flips them against the wheel's axial speed.
    """
    moments = tuple(float(time) for time in times)
    piston_area = float(parameters["piston_area"])
    max_brake_value = float(parameters["max_brake_value"])
    brake_mu = float(parameters["brake_mu"])
    radius = float(parameters["effective_piston_radius"])

    amplitudes: dict[str, tuple[float, ...]] = {}
    for wheel in WHEELS:
        share = bias_share(parameters, wheel)
        values: list[float] = []
        for time in moments:
            input_value = float(demand.value_at(time))
            if input_value < 0.0 or input_value > 1.0:
                raise ValueError(
                    f"brake_input must be normalized to [0, 1]; got {input_value!r} "
                    f"at t={time!r}"
                )
            values.append(
                2.0
                * piston_area
                * share
                * input_value
                * max_brake_value
                * brake_mu
                * radius
            )
        amplitudes[wheel] = tuple(values)
    return amplitudes
