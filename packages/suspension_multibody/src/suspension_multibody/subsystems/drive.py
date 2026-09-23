"""
The drive subsystem: a simplified, torque-only powertrain under the `drive` role.

Like the simplified brake, this template owns **no bodies**.  It declares a
distribution and emits a per-wheel drive torque; it does not build an engine, a
gearbox, a differential or a driveshaft.  Adams Car's inactive-powertrain path
(`pvs_driveline=0`) is the same state of affairs, and its `_powertrain.tpl` is a
sibling template under the same role rather than a different mechanism.

The numbers here are the ones `DrivelineSpec` already owns, and the arithmetic is
the driving branch of `preparation/vehicle_dynamic.py`'s
`_build_wheel_torque_signals` (its `:1485-1495`), value for value::

    T[name] = maximum_drive_torque * shares[name] * drive_input   (driven wheel)
    T[name] = 0.0                                                (otherwise)

with `shares` the `drive_split` tuple read in wheel order.  The multiplication is
written in the source's own association order so the floats are bit-identical,
not merely close: this module is the subsystem spelling of a formula the
preparation layer already computes, and the two are locked together by a test
that compares the two dictionaries value by value.

Two things this module deliberately does **not** do:

* it does not apply a unit scale -- the caller's `scale` belongs to the
  preparation layer, and a subsystem that silently scaled would double-count it;
* it does not decide availability.  "Only the full vehicle has a drive" is an
  assembly-level declaration (requirement 17 / D8), not a property of the role.

`driven_wheels` and `drive_split` are declared as slots for the role contract's
sake.  They are not numbers: the wheel *names* and the four-way split live on
`DrivelineSpec`, whose validation owns them (the split must sum to one, a
non-zero torque needs a non-empty driven set).  The template therefore declares
the slots with numeric placeholders and says so here rather than inventing a
second, weaker representation of the same facts.
"""

from __future__ import annotations

from collections.abc import Iterable

from ..schema import DrivelineSpec, TimeSignal
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
    "SIMPLIFIED_DRIVE",
    "SIMPLIFIED_DRIVE_NAME",
    "build",
    "register_simplified",
    "role",
    "wheel_torque_amplitudes",
]

#: The role this subsystem implements.
role = "drive"

#: The name the simplified template registers under.  A detailed powertrain
#: template is a sibling name in the same registry, not a different code path.
SIMPLIFIED_DRIVE_NAME = "powertrain_simplified"

#: The two hardpoint roles the drive role requires.  Neither column applies: the
#: point locates the wheel the torque acts on.  `spin_axis` is what the kernel
#: needs to know which way the torque acts, so it is declared here even though the
#: simplified template carries no geometry of its own.
_DRIVE_MOUNTS: tuple[ConnectionDefinition, ...] = tuple(
    ConnectionDefinition(f"{mount}_{side}", mount)
    for mount in ("wheel_center", "spin_axis")
    for side in ("L", "R")
)

#: The simplified driveline's slots.  The three numbers are placeholders: the
#: real `driven_wheels`/`drive_split`/`maximum_drive_torque` values live on
#: `DrivelineSpec` (see the module docstring), and `wheel_torque_amplitudes`
#: reads them from there.  Declaring the slots is what satisfies the role
#: contract; filling them with a wrong-but-plausible number would be worse than
#: the explicit zero.
SIMPLIFIED_DRIVE = Template(
    name=SIMPLIFIED_DRIVE_NAME,
    role="drive",
    #: Zero parts is the point, not an omission: see the module docstring.
    parts=(),
    connections=_DRIVE_MOUNTS,
    property_slots=(
        #: Wheel names, not a number.  `DrivelineSpec.driven_wheels` owns them.
        PropertySlot("driven_wheels", "-", default=0.0),
        #: The four-way split, not a number.  `DrivelineSpec.drive_split` owns it.
        PropertySlot("drive_split", "-", default=0.0),
        PropertySlot("maximum_drive_torque", "N*mm", default=0.0),
    ),
    outputs=(OutputDeclaration("drive_torque", "N*mm", "kernel"),),
    suspension_kind="driveline",
    description=(
        "The simplified powertrain: no bodies, only the per-wheel drive torque.  "
        "Availability is the full-vehicle assembly's call.  A powertrain template "
        "with bodies is a sibling under the same role."
    ),
)


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


def wheel_torque_amplitudes(
    driveline: DrivelineSpec,
    *,
    times: Iterable[float],
    drive: TimeSignal,
) -> dict[str, tuple[float, ...]]:
    """
    Return the drive torque at each wheel and time, matching the live build.

    `_build_wheel_torque_signals` computes this when no direct per-wheel override
    is set, and the two must agree exactly: this is a subsystem spelling of an
    existing formula, not a second model of it.  The checks it applies are the
    same ones, and they name the offending field:

    * a non-zero `maximum_drive_torque` with no `driven_wheels` is refused rather
      than treated as "drives nothing";
    * each driven wheel needs a positive share, because a zero share would emit a
      silent zero for a wheel the model claims to drive;
    * `drive_input` must be within `[-1, 1]`.

    No scale is applied.  The preparation layer multiplies its own scale in; doing
    it here would apply it twice.
    """
    shares = dict(zip(WHEELS, driveline.drive_split))
    driven_wheels = driveline.driven_wheels
    driven = set(driven_wheels)
    if driveline.maximum_drive_torque > 0.0 and not driven_wheels:
        raise ValueError("drive torque requires driveline.driven_wheels")
    for name in driven:
        if shares[name] <= 0.0:
            raise ValueError(
                f"each driven wheel requires a positive drive_split; {name!r} "
                f"has {shares[name]!r}"
            )

    amplitudes: dict[str, tuple[float, ...]] = {}
    for name in WHEELS:
        values: list[float] = []
        for time in times:
            drive_value = float(drive.value_at(float(time)))
            if drive_value < -1.0 or drive_value > 1.0:
                raise ValueError(
                    "drive_input must be normalized to [-1, 1]; got "
                    f"{drive_value!r} at t={float(time)!r}"
                )
            values.append(
                driveline.maximum_drive_torque * shares[name] * drive_value
                if name in driven
                else 0.0
            )
        amplitudes[name] = tuple(values)
    return amplitudes
