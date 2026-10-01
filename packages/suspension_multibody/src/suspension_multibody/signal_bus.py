"""
The signal bus: named measurement points out of a run, and actuator inputs into one.

What this is for
----------------
A controller has to read state and write commands *inside* a run.  Today the two
halves of that exist but do not meet: a run produces named blocks (``body_state``,
``tire_output``, ``steering_output`` ...) that a caller reads by index, and the
case document carries inputs that a caller has to know by name.  Neither side has
a vocabulary, so "read the wheel speed and command the damper" is a paragraph of
plumbing rather than a call.

The bus is that vocabulary.  It is deliberately **not** a second computation path:

* a **measurement channel** names a quantity the run already produced, plus the
  block and column it lives in.  Reading goes through the run's own result object,
  so a value read here is the *same* value the result document carries -- not a
  re-derivation that can disagree with it;
* an **actuator channel** names an input the case document already accepts, plus
  where in that document it goes.  Writing produces the *document edit* a caller
  would otherwise have made by hand, which is what keeps the bus on the input side
  of the boundary rather than inventing a second way to influence a solve.

How it relates to the static ``outputs`` declarations
-----------------------------------------------------
`outputs/declarations.py` says what a run *declares* it produces, before anything
computes it, and `outputs/builtin.py` holds the concrete declaration sets.  The
bus does not replace them or compete with them: **the declaration set is the
source of truth for what a run produces, and the bus is a typed reader over it.**
Every measurement channel therefore states the declaration it corresponds to, and
`measurement_channels_without_declaration()` reports any channel whose block has
no matching declaration -- "declared but not wired" and "wired but not declared"
both have to be visible rather than assumed away.

The frozen Adams channel table (`results/channels.py::ChannelRegistry`, loaded
from `adams/axle_channels.yaml`) is a third thing and stays untouched: it is a
*contract* with an external tool about which channels it exports, not a runtime
bus for this package.

Layering
--------
This module reads a result object and edits a case document.  Both are values that
already exist at the API layer, so it sits beside `api.py` and imports neither the
kernel nor a solver: it must not become a fourth place that knows how to submit or
how to compute a force.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

import numpy as np

__all__ = [
    "ACTUATOR_CHANNELS",
    "MEASUREMENT_CHANNELS",
    "ActuatorChannel",
    "BusError",
    "MeasurementChannel",
    "SignalBus",
    "measurement_channels_without_declaration",
    "open_bus",
]


class BusError(ValueError):
    """A channel name is unknown, or a reading cannot be taken."""


@dataclass(frozen=True)
class MeasurementChannel:
    """
    One quantity a run produces, named once.

    ``block`` and ``column`` say *where* the number lives; ``unit`` says what it
    is.  Reading never recomputes: it indexes the block the run already carries,
    which is why a bus reading and the result document's own field are equal by
    construction rather than by agreement.

    A quantity with more than one component (an angular velocity, a linear
    acceleration) states the columns it occupies and is read as an array; a
    caller that wants one component says which, rather than the bus guessing.
    """

    name: str
    block: str
    unit: str
    #: Which entity the block is indexed by: a body, a tire, or none.
    entity: str = ""
    #: The columns of that entity's row this quantity occupies, as a slice.
    #: Named here rather than sliced with literals at the call site, so the layout
    #: has one home (the kernel's own `kStatePerBody` order).
    columns: slice = field(default_factory=lambda: slice(0, 0))
    #: The declaration this channel corresponds to, from `outputs/`.
    declaration: str = ""
    note: str = ""

    def read(self, raw: Any, *, entity: str = "", sample: int = -1) -> np.ndarray:
        """
        Read this channel out of one run's raw result.

        Returns the channel's own columns, so a multi-component quantity keeps its
        components.  ``sample`` defaults to the last sample, which is where a
        controller reading a finished run is asking about; a caller driving a loop
        passes the index it is at.
        """
        block = _block_of(raw, self.block)
        row = _row_of(block, self, raw, entity=entity, sample=sample)
        return np.asarray(row, dtype=float)[self.columns]

    def read_component(
        self, raw: Any, component: int, *, entity: str = "", sample: int = -1
    ) -> float:
        """
        Read one component of this channel, naming the one that is out of range.

        Kept separate from :meth:`read` so a scalar read is an explicit choice
        rather than the bus silently picking the first component.
        """
        values = self.read(raw, entity=entity, sample=sample)
        if not 0 <= component < values.size:
            raise BusError(
                f"channel {self.name!r} carries {values.size} component(s) and "
                f"component {component} was asked for"
            )
        return float(values[component])


@dataclass(frozen=True)
class ActuatorChannel:
    """
    One input a case document accepts, named once.

    ``path`` is where the value goes inside the case document, as a tuple of keys.
    Writing returns a new document with that path set -- the same edit a caller
    would make by hand -- so the bus never becomes a second way to influence a
    solve, only a named way to say what to write.
    """

    name: str
    unit: str
    #: Where the value lands in the case document.
    path: tuple[str, ...]
    note: str = ""

    def write(self, case_document: Mapping[str, Any], value: float) -> dict[str, Any]:
        """
        Return ``case_document`` with this channel's path set to ``value``.

        The document is not mutated: a run's inputs are its own, and a bus that
        rewrote the caller's mapping would make "the run I already submitted" and
        "the run I described" the same object.  Missing intermediate mappings are
        created, because a case document omitting an optional input is legal.
        """
        if not np.isfinite(value):
            raise BusError(
                f"channel {self.name!r} was given {value!r}; an actuator command must "
                "be a finite number"
            )
        written = _copy_path(case_document, self.path)
        target = written
        for key in self.path[:-1]:
            target = target[key]
        target[self.path[-1]] = float(value)
        return written


def _copy_path(document: Mapping[str, Any], path: Sequence[str]) -> dict[str, Any]:
    """Return a copy of ``document`` with the mappings along ``path`` copied."""
    copied: dict[str, Any] = dict(document)
    cursor = copied
    for key in path[:-1]:
        existing = cursor.get(key)
        if existing is None:
            cursor[key] = {}
        elif not isinstance(existing, Mapping):
            raise BusError(
                f"{'.'.join(path)!r} passes through {key!r}, which holds "
                f"{type(existing).__name__} rather than a mapping"
            )
        else:
            cursor[key] = dict(existing)
        cursor = cursor[key]
    return copied


#: The tire block's column holding the vertical contact force, in newtons.  The
#: same column `results/kc_state.py::_TIRE_LOAD_COLUMN` names; restated here so
#: this module does not import the result layer just for one index.
_TIRE_LOAD_COLUMN = 4

#: The measurement channels this bus exposes.
#:
#: Wheel speed comes from the wheel body's own spin rate in `body_state`; body
#: acceleration is the chassis body's acceleration half of the same block.  Both
#: are *reads of the result document*, which is what makes them checkable against
#: it (see `raw/point_parity.md`).
MEASUREMENT_CHANNELS: tuple[MeasurementChannel, ...] = (
    MeasurementChannel(
        name="wheel_speed",
        block="body_state",
        unit="rad/s",
        entity="wheel",
        columns=slice(10, 13),
        declaration="wheel_spin_rate",
        note=(
            "the wheel body's angular velocity, world frame -- columns 10..12 of "
            "the body-state row.  Read out of the solved body state rather than "
            "re-derived from a travel signal, so it is the same number the result "
            "document carries"
        ),
    ),
    MeasurementChannel(
        name="body_acceleration",
        block="body_state",
        unit="mm/s^2",
        entity="body",
        columns=slice(13, 16),
        declaration="body_acceleration",
        note=(
            "the chassis body's linear acceleration, world frame -- columns "
            "13..15 of the body-state row"
        ),
    ),
    MeasurementChannel(
        name="tire_vertical_load",
        block="tire_output",
        unit="N",
        entity="tire",
        columns=slice(_TIRE_LOAD_COLUMN, _TIRE_LOAD_COLUMN + 1),
        declaration="tire_force",
        note=(
            "the vertical force the contact law produced, at the sample asked "
            "for.  Column 4 of the tire row is the same column "
            "`results/kc_state.py` reads its own load from"
        ),
    ),
)



#: The actuator channels this bus exposes.
#:
#: Variable damping lands on a damper element's compression coefficient inside the
#: *model* document; the motor torque lands on a body wrench moment inside the
#: *case* document.  `write` edits whichever document the path names, and both
#: paths are stated so a caller can see exactly what it is changing.
ACTUATOR_CHANNELS: tuple[ActuatorChannel, ...] = (
    ActuatorChannel(
        name="variable_damping_L",
        unit="N*s/mm",
        path=("elements", "damper_L", "parameters", "compression_damping"),
        note=(
            "the left damper's compression coefficient.  It lives in the model "
            "document, so a write is a model edit: the same model re-submitted "
            "with a different coefficient is a different run"
        ),
    ),
    ActuatorChannel(
        name="motor_torque_FL",
        unit="N*mm",
        path=("body_wrench", "front_wheel_hub_L", "moment"),
        note=(
            "the drive moment applied to the front left wheel hub, in the case "
            "document's own body-wrench table"
        ),
    ),
)


def _block_of(raw: Any, name: str) -> np.ndarray:
    """Return one block of a run's raw result, naming the block if it is absent."""
    blocks = getattr(raw, "named_blocks", None) or getattr(raw, "blocks", None)
    if blocks is None or name not in blocks:
        known = ", ".join(sorted(blocks)) if blocks else "<none>"
        raise BusError(
            f"the run carries no block {name!r}; it produced {known}"
        )
    return np.asarray(blocks[name])


def _row_of(
    block: np.ndarray,
    channel: MeasurementChannel,
    raw: Any,
    *,
    entity: str,
    sample: int,
) -> np.ndarray:
    """
    Return the row one channel names, for one entity at one sample.

    ``entity`` may be given explicitly; when it is not, the channel's own
    ``entity`` field decides how to index -- a tire channel needs a tire name, a
    body channel a body name -- and the caller is told when it has not supplied
    one, rather than the read silently landing on the first entity.
    """
    if block.ndim != 3:
        raise BusError(
            f"channel {channel.name!r} expects a per-entity block and got shape "
            f"{block.shape}"
        )
    index = _entity_index(raw, channel, entity)
    return block[sample, index, :]


def _entity_index(raw: Any, channel: MeasurementChannel, entity: str) -> int:
    """Return which entity row one channel reads."""
    if channel.entity == "body":
        names = tuple(getattr(raw, "body_names", ()) or ())
        field_name = "body"
    elif channel.entity == "tire":
        names = tuple(getattr(raw, "tire_names", ()) or ())
        field_name = "tire"
    elif channel.entity == "wheel":
        names = tuple(getattr(raw, "body_names", ()) or ())
        field_name = "wheel"
    else:
        return 0
    if not entity:
        raise BusError(
            f"channel {channel.name!r} reads one {field_name} and none was named; "
            f"its block is indexed by {field_name}, so the read would have landed "
            f"on {names[0]!r} had it been guessed"
        )
    for index, name in enumerate(names):
        if name == entity or str(name).endswith(entity):
            return index
    raise BusError(
        f"channel {channel.name!r} names {entity!r} and the run carries no such "
        f"{field_name}; it has {', '.join(str(n) for n in names)}"
    )


@dataclass(frozen=True)
class SignalBus:
    """
    A reader over one run's results and a writer of one case document's inputs.

    The two halves are separate on purpose: a bus handed a result object can read
    but not write, and one handed a document can write but not read.  A controller
    holds both because a controller is what does both.
    """

    raw: Any = None
    case_document: Mapping[str, Any] | None = None
    measurements: tuple[MeasurementChannel, ...] = MEASUREMENT_CHANNELS
    actuators: tuple[ActuatorChannel, ...] = ACTUATOR_CHANNELS

    def measurement(self, name: str) -> MeasurementChannel:
        """Return a measurement channel by name, naming an unknown one."""
        for channel in self.measurements:
            if channel.name == name:
                return channel
        raise BusError(
            f"unknown measurement channel {name!r}; the bus offers "
            f"{', '.join(c.name for c in self.measurements)}"
        )

    def actuator(self, name: str) -> ActuatorChannel:
        """Return an actuator channel by name, naming an unknown one."""
        for channel in self.actuators:
            if channel.name == name:
                return channel
        raise BusError(
            f"unknown actuator channel {name!r}; the bus offers "
            f"{', '.join(c.name for c in self.actuators)}"
        )

    def read(self, name: str, *, entity: str = "", sample: int = -1) -> float:
        """Read one measurement channel out of this bus's run."""
        if self.raw is None:
            raise BusError(
                "this bus carries no run to read; `open_bus` binds one when a run "
                "has been produced"
            )
        return self.measurement(name).read(self.raw, entity=entity, sample=sample)

    def write(
        self, name: str, value: float, *, document: Mapping[str, Any] | None = None
    ) -> dict[str, Any]:
        """
        Return the given document with one actuator channel set to ``value``.

        ``document`` defaults to the one the bus was opened with, and the original
        is never mutated.
        """
        target = self.case_document if document is None else document
        if target is None:
            raise BusError(
                "this bus carries no document to write; pass one to `write`"
            )
        return self.actuator(name).write(target, value)


def open_bus(
    raw: Any = None, case_document: Mapping[str, Any] | None = None
) -> SignalBus:
    """
    Return a bus over a run's results and/or a case document.

    Either half may be omitted: a controller that has only submitted its run reads
    with the first, and one that is still authoring inputs writes with the second.
    """
    return SignalBus(raw=raw, case_document=case_document)


def measurement_channels_without_declaration(
    declarations: Any = None,
) -> tuple[MeasurementChannel, ...]:
    """
    Return the measurement channels no output declaration accounts for.

    The declaration set is the source of truth for what a run produces, so a
    channel with no declaration behind it is a claim nobody has made; and a
    declaration with no channel is something a caller cannot reach through the
    bus.  Both directions matter, and reporting the first is what keeps the bus
    from quietly inventing a vocabulary of its own.
    """
    if declarations is None:
        from .outputs.builtin import ASSEMBLY_OUTPUTS, RIG_OUTPUTS

        names = {entry.name for entry in ASSEMBLY_OUTPUTS.outputs}
        names |= {entry.name for entry in RIG_OUTPUTS.outputs}
    else:
        names = {str(getattr(item, "name", item)) for item in declarations}
    return tuple(
        channel for channel in MEASUREMENT_CHANNELS if channel.declaration not in names
    )
