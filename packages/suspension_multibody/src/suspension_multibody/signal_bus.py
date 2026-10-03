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

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any, Literal

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
    One input a document accepts, named once, with where the solver reads it.

    A channel states the *document* it lands in and the edit that document
    needs, because the two channels a caller wants are not the same kind of
    thing: a damping coefficient is a number inside one element of the model
    document's ``elements`` **array**, while a wheel moment is a column of a
    case document's payload **blob**, reached through the descriptor that names
    it.  Writing a nested mapping into either document -- which is what an
    earlier version did -- produced a document the kernel never read: the array
    is not a mapping, and the blob's bytes are what the reader dereferences.
    """

    name: str
    unit: str
    #: Which document this channel writes: ``"model"`` or ``"case"``.
    document: Literal["model", "case"] = "case"
    #: The element name, for a channel that edits one entry of ``elements``.
    element: str = ""
    #: The element parameters this channel sets, all to the same value.
    #:
    #: A damper is not symmetric: the solver picks its coefficient from the sign
    #: of the relative rate (negative is compression, positive is rebound), so a
    #: channel that set only one of the two would move the run in one direction
    #: and leave the other exactly as authored -- measured: setting only
    #: `compression_damping` on a rig whose relative motion is an extension moved
    #: the solved state by 0.0.
    parameters: tuple[str, ...] = ()
    #: The blob role, for a channel that edits a payload table.
    role: str = ""
    #: The entity the role's table is keyed by (a body or a tire name).
    entity: str = ""
    #: The columns of one table row this channel sets, as a slice of that row.
    #:
    #: A ``body_wrench`` row is ``[fx, fy, fz, mx, my, mz]``, so a moment channel
    #: sets columns 3..6 and leaves the force columns exactly as the case stated
    #: them.  Writing the whole row would silently zero a force the caller never
    #: asked to change.
    component: slice = slice(0, 0)
    note: str = ""

    def write(
        self,
        document: Mapping[str, Any],
        blob: bytes,
        value: float,
        *,
        sample: int = -1,
    ) -> tuple[Mapping[str, Any], bytes]:
        """
        Return the document and blob this channel's value belongs in.

        The document is not mutated: a run's inputs are its own, and a bus that
        rewrote the caller's mapping would make "the run I already submitted" and
        "the run I described" the same object.
        """
        if not np.isfinite(value):
            raise BusError(
                f"channel {self.name!r} was given {value!r}; an actuator command must "
                "be a finite number"
            )
        if self.element:
            return _write_element_parameters(self, document, value), blob
        if self.role:
            return document, _write_blob_sample(self, document, blob, value, sample)
        raise BusError(f"channel {self.name!r} states neither an element nor a role")


def _write_element_parameters(
    channel: ActuatorChannel, document: Mapping[str, Any], value: float
) -> dict[str, Any]:
    """
    Set one parameter of one named element in a model document's array.

    ``elements`` is a list, and each entry carries its own ``parameters``
    mapping.  The entry is found by name -- never by position, which would
    silently retarget if the assembly ever reordered its elements -- and a
    document with no such element is refused by name rather than accepted and
    ignored, because a write that lands nowhere is worse than a failed one.
    """
    written = dict(document)
    elements = written.get("elements")
    if not isinstance(elements, list):
        raise BusError(
            f"channel {channel.name!r} edits element {channel.element!r}, but the "
            f"document's 'elements' holds {type(elements).__name__} rather than a list"
        )
    names: list[str] = []
    patched: list[Any] = []
    found = False
    for entry in elements:
        if not isinstance(entry, Mapping):
            patched.append(entry)
            continue
        name = str(entry.get("name", ""))
        names.append(name)
        if name != channel.element:
            patched.append(entry)
            continue
        parameters = dict(entry.get("parameters") or {})
        missing = [name for name in channel.parameters if name not in parameters]
        if missing:
            raise BusError(
                f"channel {channel.name!r} sets {missing}, which element "
                f"{channel.element!r} does not carry; it has "
                f"{sorted(parameters) if parameters else 'no parameters'}"
            )
        for name in channel.parameters:
            parameters[name] = float(value)
        updated = dict(entry)
        updated["parameters"] = parameters
        patched.append(updated)
        found = True
    if not found:
        raise BusError(
            f"channel {channel.name!r} names element {channel.element!r} and the "
            f"document carries no such element; it has {names or 'no elements'}"
        )
    written["elements"] = patched
    return written


def _write_blob_sample(
    channel: ActuatorChannel,
    document: Mapping[str, Any],
    blob: bytes,
    value: float,
    sample: int,
) -> bytes:
    """
    Write one sample of a payload table, at the descriptor's own offset.

    The kernel reads a case table by *dereferencing* the descriptor's
    ``offset``/``length`` inside the blob, so a value that is to reach the solver
    has to be written into those bytes.  Setting a JSON key instead changes the
    document and nothing else -- measured: the solved trajectory did not move at
    all.  ``sample=-1`` writes the last sample, which is the one a caller
    inspecting a settled run means by "the command".
    """
    entries = document.get("blobs")
    if not isinstance(entries, list):
        raise BusError(
            f"channel {channel.name!r} edits role {channel.role!r}, but the document "
            f"carries no 'blobs' list (it holds {type(entries).__name__})"
        )
    matches = [
        entry
        for entry in entries
        if isinstance(entry, Mapping)
        and entry.get("role") == channel.role
        and entry.get("body", entry.get("tire")) == channel.entity
    ]
    if not matches:
        described = sorted(
            str(entry.get("role"))
            for entry in entries
            if isinstance(entry, Mapping) and entry.get("role")
        )
        raise BusError(
            f"channel {channel.name!r} names role {channel.role!r} for "
            f"{channel.entity!r}, and the document describes none; it has {described}"
        )
    descriptor = matches[0]
    if descriptor.get("dtype") != "float64":
        raise BusError(
            f"channel {channel.name!r} writes float64 and role {channel.role!r} "
            f"declares {descriptor.get('dtype')!r}"
        )
    offset = int(descriptor.get("offset", 0))
    length = int(descriptor.get("length", 0))
    count = length // 8
    if count <= 0 or offset < 0 or offset + length > len(blob):
        raise BusError(
            f"channel {channel.name!r}: role {channel.role!r} declares offset={offset} "
            f"length={length} in a {len(blob)}-byte blob"
        )
    index = count - 1 if sample < 0 else sample
    if not 0 <= index < count:
        raise BusError(
            f"channel {channel.name!r} was given sample {sample}, and role "
            f"{channel.role!r} carries {count} of them"
        )
    row = int(descriptor.get("shape", [count, 1])[1]) if isinstance(
        descriptor.get("shape"), list
    ) and len(descriptor.get("shape") or []) == 2 else 1
    component = channel.component
    if component == slice(0, 0):
        # A channel that names no component addresses a one-value table, which is
        # the whole row; anything wider would need the channel to say which
        # column it means rather than have this guess.
        component = slice(0, row)
    if component.stop > row or component.start < 0:
        raise BusError(
            f"channel {channel.name!r} sets columns {component.start}..{component.stop} "
            f"of a {row}-wide row"
        )
    rows = count // row if row else 0
    if rows <= 0:
        raise BusError(
            f"channel {channel.name!r}: role {channel.role!r} declares a "
            f"{row}-wide row in {count} values"
        )
    # The value reaches every sample of the slot unless one is named: a command
    # that is to hold over the run is the ordinary case, and a partial write
    # would leave a table whose earlier rows disagree with its later ones.
    patched = bytearray(blob)
    positions = range(rows) if sample < 0 else (sample,)
    for position in positions:
        if not 0 <= position < rows:
            raise BusError(
                f"channel {channel.name!r} was given sample {sample}, and role "
                f"{channel.role!r} carries {rows} of them"
            )
        for column in range(component.start, component.stop):
            start = offset + (position * row + column) * 8
            patched[start : start + 8] = np.float64(value).tobytes()
    return bytes(patched)


#: The tire block's column holding the vertical contact force, in newtons.  The
#: same column `results/kc_state.py::_TIRE_LOAD_COLUMN` names; restated here so
#: this module does not import the result layer just for one index.
_TIRE_LOAD_COLUMN = 4

#: The tire block's column holding the longitudinal slip velocity, in m/s.  The
#: same column `results/` reads and the one the kernel's ABS law measures, so a
#: controller reading the bus and the law acting in the solver see one quantity.
_TIRE_SLIP_COLUMN = 7

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

    MeasurementChannel(
        name="longitudinal_slip",
        block="tire_output",
        unit="m/s",
        entity="tire",
        columns=slice(_TIRE_SLIP_COLUMN, _TIRE_SLIP_COLUMN + 1),
        declaration="tire_force",
        note=(
            "the tire's longitudinal slip velocity, at the sample asked for -- "
            "column 7 of the tire row, the same column the ABS law's own "
            "measurement is derived from and the one `results/kc_state.py` "
            "reads.  A closed-loop controller needs a slip measurement, and "
            "taking it from the result document rather than re-deriving it is "
            "what keeps the bus reading the same number the solver produced "
            "(subtask p5-04)"
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
        # The coefficient lives in the *model* document, inside one element of
        # its ``elements`` array -- not at a nested key of the document root.
        document="model",
        element="damper_L",
        # The solver picks compression or rebound from the sign of the relative
        # rate, so "set the damper" means both directions: a channel that set one
        # would silently leave the other at its authored value.
        parameters=("compression_damping", "rebound_damping"),
        note=(
            "the left damper's compression and rebound coefficients, set on that "
            "element's own ``parameters``.  It lives in the model document, so a "
            "write is a model edit: the same model re-submitted with different "
            "coefficients is a different run"
        ),
    ),
    ActuatorChannel(
        name="motor_torque_FL",
        unit="N*mm",
        # The moment is a per-sample table in the case document's payload blob,
        # found through the descriptor that names the body it acts on.
        document="case",
        role="body_wrench",
        entity="front_wheel_hub_L",
        # A wrench row is [fx, fy, fz, mx, my, mz]: this channel is the moment.
        component=slice(3, 6),
        note=(
            "the drive moment applied to the front left wheel hub.  The kernel "
            "reads this table by dereferencing its descriptor inside the case "
            "blob, so the write lands in those bytes: a value set on the JSON "
            "root would never reach the solver"
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
    model_document: Mapping[str, Any] | None = None
    case_blob: bytes | None = None
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
        self,
        name: str,
        value: float,
        *,
        document: Mapping[str, Any] | None = None,
        blob: bytes | None = None,
        sample: int = -1,
    ) -> tuple[dict[str, Any], bytes]:
        """
        Return the document and blob that carry this channel's value.

        Which document the channel edits is the channel's own declaration: a
        damping coefficient is a model edit, a wheel moment is a case-table edit.
        ``document`` defaults to the matching one the bus was opened with, so a
        caller that opened the bus over both halves does not have to say which.
        The original is never mutated.
        """
        channel = self.actuator(name)
        target = self._document_for(channel, document)
        if target is None:
            raise BusError(
                f"this bus carries no {channel.document} document to write; pass "
                "one to `write`, or open the bus with the document this channel edits"
            )
        current_blob = self.case_blob if blob is None else blob
        if channel.role and current_blob is None:
            raise BusError(
                f"channel {name!r} writes a table in the case payload, and no blob "
                "was supplied; the bytes the descriptor points at are what the "
                "kernel reads, so a write without them cannot land"
            )
        return channel.write(
            target, current_blob or b"", value, sample=sample
        )

    def _document_for(
        self, channel: ActuatorChannel, override: Mapping[str, Any] | None
    ) -> Mapping[str, Any] | None:
        """Return the document one channel edits, honouring an explicit override."""
        if override is not None:
            return override
        if channel.document == "model":
            return self.model_document
        return self.case_document


def open_bus(
    raw: Any = None,
    case_document: Mapping[str, Any] | None = None,
    model_document: Mapping[str, Any] | None = None,
    case_blob: bytes | None = None,
) -> SignalBus:
    """
    Return a bus over a run's results and/or the documents it was submitted with.

    Any half may be omitted: a controller that has only submitted its run reads
    with the first, and one that is still authoring inputs writes with the rest.
    The blob is a parameter because a case table reaches the solver through its
    bytes, and a writer that cannot see them cannot move the solve.
    """
    return SignalBus(
        raw=raw,
        case_document=case_document,
        model_document=model_document,
        case_blob=case_blob,
    )


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
