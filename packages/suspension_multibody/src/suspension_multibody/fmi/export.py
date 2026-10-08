"""
Export one solved model/case pair as an FMI 2.0 Co-Simulation FMU.

Scope, frozen by the epic's D4 ruling: the FMU carries the **model** and its
**input/output variables**, and nothing else.  There is no Python-side
evaluation in the artifact -- the exported binary calls the native kernel, and
Python's whole job here is to author the two contract documents, pack them into
their containers, describe the variables and frame the archive.

What is inside the archive
--------------------------

``modelDescription.xml``
    The FMI 2.0 model description: the co-simulation entry point, the unit
    definitions, the variable list, and the model structure's output
    dependencies.
``binaries/<platform>/<identifier>.dll``
    The C wrapper that implements the FMI 2.0 Co-Simulation symbol set.  It
    loads the kernel and calls ``suspension_kernel_run``.
``resources/model.bin``, ``resources/case.bin``
    The two contract containers, byte for byte what the run was compiled
    from.  They are *resources* rather than regenerated numbers because the
    FMU must solve the documents the exporter was handed, not a second
    reading of them.
``resources/bindings.txt``
    Where each variable's number lives inside those two containers: for an
    input, the byte offset of its slot in the case blob; for an output, the
    result block and column it is read from; plus the case's own time grid, so
    the instance can tell which sample its clock is at.

Why a binding is part of the declaration
----------------------------------------

A variable list that names quantities but not the bytes they come from is a
claim, not an interface -- and the failure mode is silent: the wrapper reads a
plausible number from the wrong offset and the FMU looks like it works.  So
every :class:`FmiVariable` carries an :class:`FmiBinding`, the XML and the
binary's own descriptor file are generated from the same objects, and a
declaration that cannot be bound is a refusal rather than an omission.

Where the variable list comes from
----------------------------------

* **Inputs** are the case document's own per-sample tables: the ``blobs``
  descriptors carry a ``role`` (and, for the per-tire roles, a ``tire``), and
  the descriptor's ``offset``/``length`` *are* the binding.  A descriptor of the
  run being exported is a value an outside simulator must *supply*, so every one
  of them is ``causality="input"``.  The role's unit is read from what the
  kernel does with it: a normalized driver demand is dimensionless, a road
  height is a length, a wheel torque is a moment.
* **Outputs** are the bus's measurement channels
  (:data:`signal_bus.MEASUREMENT_CHANNELS`), which state the result block and
  the column of each quantity, expanded over the entities the model document
  declares -- one entity per body for a body-indexed channel, one per tire for a
  tire-indexed channel -- plus the accepted time grid itself.  Those are
  quantities the run *produces*, so every one is ``causality="output"``.

The two halves are complementary rather than overlapping: an input is a table
the case document carries and an output is a column the result carries.  A name
can therefore not appear with two directions by accident, and the tests assert
exactly that.

What is deliberately *not* declared
-----------------------------------

``ASSEMBLY_OUTPUTS`` names more than the result container carries: the upright
poses, the wheel centres and the steering samples are derived by the reporting
layer from the raw body state, and the diagnostic counters live in a ledger
whose rows are not sample-major.  None of them is a column of the result blob,
so the wrapper shipped in the archive could not serve them; declaring them would
be advertising outputs that read nothing.  They are left out, and the tests
assert that every declared output is bound to a block the run actually emits --
which is the property that makes the list worth trusting.

Reproducibility
---------------

``guid`` is a digest of the model hash and the case's own canonical bytes, so
exporting the same pair twice produces a byte-identical ``modelDescription.xml``
and a byte-identical FMU.  A random or timestamped GUID would make the artifact
unreproducible, which is the one property a co-simulation export cannot afford
to lose.
"""

from __future__ import annotations

import json
import platform
import shutil
import zipfile
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal
from xml.etree import ElementTree

from suspension_contracts import contract_hash, unpack_container

from ..io import canonical_hash
from ..signal_bus import MEASUREMENT_CHANNELS

__all__ = [
    "FMI_MODEL_IDENTIFIER",
    "FMI_VERSION",
    "FmiBinding",
    "FmiExportError",
    "FmiVariable",
    "FmuExport",
    "bindings_resource",
    "description_json",
    "export_fmu",
    "fmi_binary_name",
    "fmi_platform",
    "model_description_document",
    "read_description",
    "variable_declarations",
]

#: The FMI version this module emits.  D4's ruling names 2.0 Co-Simulation; the
#: generated ``modelDescription.xml`` declares the same string.
FMI_VERSION = "2.0"

#: The co-simulation model identifier, which is also the binary's stem.
FMI_MODEL_IDENTIFIER = "suspension_multibody_axle"

#: The archive members the exporter writes.  Named once so the test asserts the
#: contract rather than a repeated literal.
_MODEL_DESCRIPTION = "modelDescription.xml"
_RESOURCES_MODEL = "resources/model.bin"
_RESOURCES_CASE = "resources/case.bin"
_RESOURCES_BINDINGS = "resources/bindings.txt"

#: The name the wrapper uses for the time grid variable.  It is not a column of
#: any block: the accepted sample instants are the instance's own clock, which
#: is why its binding carries an empty ``block``.
TIME_VARIABLE = "time_s"

#: ``float64`` is the only dtype the kernel's containers carry, so a binding's
#: byte arithmetic needs exactly one item size.
_ITEM_SIZE = 8

#: The unit each per-tire blob role is measured in, and what it means.
#:
#: The unit is a property of the *role*, exactly as the kernel states it: a
#: normalized driver demand is a fraction and nothing scales it, a road height
#: rides in the document's length unit, and a wheel torque in its moment unit.
#: The axle family's document states ``length: m``, so a road height is metres;
#: naming the unit here is how the FMU keeps saying what the kernel reads.
_ROLE_UNITS: dict[str, tuple[str, str]] = {
    "road_height": ("m", "road height at the tire contact, per sample"),
    "road_velocity": ("m/s", "road surface velocity at the tire, per sample"),
    "wheel_torque": ("N*m", "wheel torque excitation, per sample"),
    "brake_torque": ("N*m", "brake torque excitation, per sample"),
    "throttle_demand": ("1", "normalized throttle demand in [-1, 1], per sample"),
    "brake_pressure": ("1", "normalized brake demand in [0, 1], per sample"),
    "steering_target": ("rad", "steering target, per sample"),
    "steering_rate": ("rad/s", "steering rate, per sample"),
    "sample_times": ("s", "explicit sample instants for a non-uniform grid"),
}

#: Roles that are named per driven coordinate rather than per tire.
_COORDINATE_ROLES: dict[str, tuple[str, str]] = {
    "driven_offset": ("m", "prescribed offset of one driven coordinate, per sample"),
    "driven_offset_rate": ("m/s", "prescribed rate of one driven coordinate, per sample"),
}

#: The roles that are named per body.
_BODY_ROLES: dict[str, tuple[str, str]] = {
    "body_wrench": ("N", "applied body wrench, per sample and component"),
}

#: Which block an entity-indexed channel is indexed against, keyed by the bus's
#: own ``entity`` spelling.  A channel naming bodies reads the body list of the
#: model document; one naming tires reads its tire list.  The translation is
#: here rather than in the bus because the *document* is what this export has.
_ENTITY_SOURCE: dict[str, str] = {
    "body": "bodies",
    "wheel": "bodies",
    "tire": "tires",
}

#: The unit a measurement channel is reported in, keyed by channel name.  The
#: bus states the unit in ``MeasurementChannel.unit``; these are the same
#: strings, kept here because the FMI variable also needs the *words*.
_CHANNEL_DESCRIPTIONS: dict[str, str] = {
    "wheel_speed": (
        "the body's angular velocity, world frame -- the same columns of the "
        "body-state block the signal bus exposes as `wheel_speed`"
    ),
    "body_acceleration": (
        "the body's linear acceleration, world frame -- the same columns the "
        "signal bus exposes as `body_acceleration`"
    ),
    "tire_vertical_load": (
        "the vertical force the contact law produced, at the sample asked for"
    ),
    "longitudinal_slip": (
        "the tire's longitudinal slip velocity, at the sample asked for"
    ),
}


class FmiExportError(ValueError):
    """An FMU cannot be exported from what was handed in."""


@dataclass(frozen=True)
class FmiBinding:
    """
    Where one variable's number actually lives in the artifact's payload.

    An **input** is a slot in the case container's blob: ``offset`` is the byte
    offset of its first sample and ``count`` how many samples it holds, so
    ``count`` is non-zero exactly for an input.  FMI has no vector-valued real,
    so one scalar writes the same number into every sample of the slot: the
    value an outside simulator sets is a constant profile over the horizon the
    case document states.

    An **output** is a column of the result container's blob: ``block`` names
    the result block, ``row`` which entity row inside a sample and ``column``
    which column inside that row.  Which sample is read follows the instance's
    own time against the case's time grid, so a stepping caller sees the run
    evolve.  A binding with an empty ``block`` is the time grid itself, which
    the instance answers from its own clock.
    """

    offset: int = 0
    count: int = 0
    block: str = ""
    row: int = 0
    column: int = 0

    @property
    def direction(self) -> str:
        """Return ``"input"`` for a filled table slot and ``"output"`` for a column."""
        return "input" if self.count else "output"


@dataclass(frozen=True)
class FmiVariable:
    """
    One variable the FMU exposes, with its FMI direction.

    ``causality`` is the FMI 2.0 spelling: an ``input`` is a value the
    simulator supplies and the model reads, an ``output`` is a value the model
    produces and the simulator reads, and a ``parameter`` is a value fixed for
    the run.  ``variability`` says how the value behaves over time; a
    per-sample table is ``continuous`` because the solver may ask for it at any
    instant, and a run-fixed value is ``fixed``.
    """

    name: str
    reference: int
    causality: Literal["input", "output", "parameter"]
    variability: str
    unit: str
    description: str
    binding: FmiBinding


@dataclass(frozen=True)
class FmuExport:
    """The artifact one export produced, and what it says about itself."""

    path: Path
    model_identifier: str
    variables: tuple[FmiVariable, ...]
    fmi_version: str
    guid: str

    @property
    def inputs(self) -> tuple[FmiVariable, ...]:
        """Return the variables an outside simulator supplies."""
        return tuple(item for item in self.variables if item.causality == "input")

    @property
    def outputs(self) -> tuple[FmiVariable, ...]:
        """Return the variables the model produces."""
        return tuple(item for item in self.variables if item.causality == "output")


# --- the variable list ------------------------------------------------------


def _role_unit(role: str) -> str | None:
    """Return the unit one blob role is measured in, or `None` if it is unknown."""
    if role in _ROLE_UNITS:
        return _ROLE_UNITS[role][0]
    if role in _COORDINATE_ROLES:
        return _COORDINATE_ROLES[role][0]
    if role in _BODY_ROLES:
        return _BODY_ROLES[role][0]
    return None


def _role_description(role: str, descriptor: Mapping[str, Any]) -> str:
    """Return the words one input variable carries, naming what qualifies it."""
    if role in _ROLE_UNITS:
        text = _ROLE_UNITS[role][1]
    elif role in _COORDINATE_ROLES:
        text = _COORDINATE_ROLES[role][1]
    elif role in _BODY_ROLES:
        text = _BODY_ROLES[role][1]
    else:
        text = f"case table with role {role!r}"
    qualifiers = [
        f"{key}={descriptor[key]!r}"
        for key in ("tire", "coordinate", "body")
        if key in descriptor
    ]
    shape = descriptor.get("shape")
    if isinstance(shape, list):
        qualifiers.append("shape=" + "x".join(str(int(v)) for v in shape))
    return text + (" (" + ", ".join(qualifiers) + ")" if qualifiers else "")


def _input_variable_name(role: str, descriptor: Mapping[str, Any]) -> str:
    """Return the FMI name of one case table, qualified by what it is named for."""
    for key in ("tire", "coordinate", "body"):
        if key in descriptor:
            return f"{role}[{descriptor[key]}]"
    return str(role)


def _input_declarations(case_document: Mapping[str, Any]) -> list[FmiVariable]:
    """
    Return the input variables one case document's own tables declare.

    The document is the source of truth: whatever ``blobs`` descriptors the
    exported case carries are exactly the tables the kernel will read, so the
    export cannot advertise an input the run does not have or omit one it does.
    Each is an ``input`` because the outside simulator supplies it -- the case
    document states *which* table it is, not what the numbers are -- and the
    descriptor's own ``offset`` and ``length`` are its binding, so the wrapper
    writes back to the exact bytes the kernel reads.
    """
    variables: list[FmiVariable] = []
    for descriptor in case_document.get("blobs", ()) or ():
        if not isinstance(descriptor, Mapping):
            continue
        role = str(descriptor.get("role", ""))
        if not role:
            continue
        name = _input_variable_name(role, descriptor)
        unit = _role_unit(role)
        if unit is None:
            # An unrecognised role is not silently given a unit: an unknown
            # input cannot be described, and describing it in the wrong unit
            # would be worse than refusing.
            raise FmiExportError(
                f"case table {name!r} carries role {role!r}, which this export "
                "has no unit for; add the role to the unit table rather than "
                "exporting a variable whose unit is a guess"
            )
        offset = int(descriptor.get("offset", 0))
        length = int(descriptor.get("length", 0))
        if length <= 0 or length % _ITEM_SIZE:
            raise FmiExportError(
                f"case table {name!r} states a {length}-byte range, which is not "
                f"a whole number of {_ITEM_SIZE}-byte samples; the binding would "
                "not address the bytes the kernel reads"
            )
        variables.append(
            FmiVariable(
                name=name,
                reference=-1,
                causality="input",
                variability="continuous",
                unit=unit,
                description=_role_description(role, descriptor),
                binding=FmiBinding(offset=offset, count=length // _ITEM_SIZE),
            )
        )
    return variables


def _entities_of(model_document: Mapping[str, Any], kind: str) -> tuple[str, ...]:
    """Return the entity names one channel's block is indexed by, in run order."""
    key = _ENTITY_SOURCE.get(kind)
    if key is None:
        return ()
    entries = model_document.get(key, ()) or ()
    names: list[str] = []
    for entry in entries:
        if isinstance(entry, Mapping) and "name" in entry:
            names.append(str(entry["name"]))
    return tuple(names)


def _output_declarations(
    model_document: Mapping[str, Any],
) -> list[FmiVariable]:
    """
    Return the output variables the run's result container can actually serve.

    The bus's measurement channels state the block and the columns each
    quantity occupies; the model document states the entities those blocks are
    indexed by.  Crossing the two gives one variable per (channel, entity,
    component), and every one of them is a column of a block the kernel writes
    -- which is the property that makes the declaration checkable rather than
    aspirational.  The accepted time grid is added first because it is the
    coordinate every other output is reported against.
    """
    variables: list[FmiVariable] = [
        FmiVariable(
            name=TIME_VARIABLE,
            reference=-1,
            causality="output",
            variability="continuous",
            unit="s",
            description="accepted sample time grid, as the run reports it",
            binding=FmiBinding(),
        )
    ]
    for channel in MEASUREMENT_CHANNELS:
        entities = _entities_of(model_document, channel.entity)
        if channel.entity and not entities:
            raise FmiExportError(
                f"channel {channel.name!r} reads one {channel.entity} and the "
                "model document declares none, so the export cannot name the "
                "variable it would advertise"
            )
        row_count = len(entities) if entities else 1
        width = channel.columns.stop - channel.columns.start
        base_note = _CHANNEL_DESCRIPTIONS.get(channel.name, channel.note)
        for row, entity in enumerate(entities or ("",)):
            label = f"{channel.name}[{entity}]" if entity else channel.name
            for component in range(width):
                suffix = "" if width == 1 else f"[{component}]"
                variables.append(
                    FmiVariable(
                        name=label + suffix,
                        reference=-1,
                        causality="output",
                        variability="continuous",
                        unit=channel.unit,
                        description=(
                            base_note
                            if width == 1
                            else f"{base_note}, component {component}"
                        ),
                        binding=FmiBinding(
                            block=channel.block,
                            row=row,
                            column=channel.columns.start + component,
                        ),
                    )
                )
    if row_count == 0:  # pragma: no cover - defensive
        raise FmiExportError("the model document declares no entity to read from")
    return variables


def variable_declarations(
    case_document: Mapping[str, Any],
    model_document: Mapping[str, Any],
) -> tuple[FmiVariable, ...]:
    """
    Return every variable the FMU exposes, with references assigned.

    Inputs come first so the FMI value references are stable under a change to
    the output declaration set -- an existing consumer's input references do
    not move when a channel is added, which is the property that makes the
    number worth having.
    """
    ordered: list[FmiVariable] = [
        *_input_declarations(case_document),
        *_output_declarations(model_document),
    ]
    names = [item.name for item in ordered]
    duplicates = sorted({name for name in names if names.count(name) > 1})
    if duplicates:
        raise FmiExportError(
            f"the same name was declared twice: {', '.join(duplicates)}; an FMI "
            "variable name is unique, so the two declarations have to be told "
            "apart before they are exported"
        )
    return tuple(
        FmiVariable(
            name=item.name,
            reference=index,
            causality=item.causality,
            variability=item.variability,
            unit=item.unit,
            description=item.description,
            binding=item.binding,
        )
        for index, item in enumerate(ordered)
    )


def _case_grid(case_document: Mapping[str, Any]) -> tuple[float, float, int]:
    """
    Return the case's own ``(start, step, sample count)`` time grid.

    The grid is what turns the instance's clock into a sample index, so an
    output read at a step addresses the sample the caller's time names.  It is
    read from the case document rather than recomputed: a second derivation of
    the same grid is how a reader ends up one sample away from the writer.
    """
    time = case_document.get("time")
    if not isinstance(time, Mapping):
        raise FmiExportError(
            "the case document states no 'time' entry, so the export cannot "
            "say which sample an output is read at"
        )
    if "samples" in time:
        raise FmiExportError(
            "the case document carries an explicit sample-time table, whose "
            "instants are not a uniform grid; the FMU's binding addresses "
            "samples by index, so a non-uniform case has to be exported as a "
            "uniform one or the binding would be a guess"
        )
    start = float(time.get("start_s", 0.0))
    step = float(time.get("step_s", 0.0))
    end = time.get("end_s")
    if step <= 0.0 or not isinstance(end, (int, float)):
        raise FmiExportError(
            f"the case document's time entry is not a usable grid: {dict(time)!r}"
        )
    samples = int(round((float(end) - start) / step)) + 1
    if samples <= 0:
        raise FmiExportError(
            f"the case document's time entry yields {samples} samples: {dict(time)!r}"
        )
    return start, step, samples


def bindings_resource(
    case_document: Mapping[str, Any],
    variables: Sequence[FmiVariable],
) -> str:
    """
    Return the text the wrapper reads to find every variable's bytes.

    One line per fact, so the format needs no parser: a ``grid`` line states the
    case's start time, step and sample count, an ``input`` line the scalar slot
    a value is written to, and an ``output`` line the block and column a value
    is read from.  It is generated from the same bindings the XML declares, so
    the two cannot disagree -- and if they could, this is the half the kernel
    would actually be driven by.
    """
    start, step, samples = _case_grid(case_document)
    lines = [
        "# suspension_multibody FMU variable bindings",
        f"grid {start!r} {step!r} {samples}",
    ]
    for variable in variables:
        binding = variable.binding
        if variable.causality == "input":
            lines.append(
                f"input {variable.reference} {binding.offset} {binding.count}"
            )
        else:
            lines.append(
                f"output {variable.reference} {binding.block or '-'} "
                f"{binding.row} {binding.column}"
            )
    return "\n".join(lines) + "\n"


# --- the model description --------------------------------------------------


def fmi_platform() -> str:
    """
    Return the FMI platform directory name for the running interpreter.

    FMI 2.0 fixes these spellings, so they are written out rather than derived
    from ``platform.machine()`` -- the archive layout is part of the standard
    and a near-miss directory name is a binary no importer finds.
    """
    system = platform.system()
    machine = platform.machine().lower()
    if system == "Windows":
        return "win64" if platform.architecture()[0] == "64bit" else "win32"
    if system == "Darwin":
        return "darwin64" if not machine.startswith("arm") else "darwin64"
    return "linux64" if platform.architecture()[0] == "64bit" else "linux32"


def fmi_binary_name() -> str:
    """Return the binary's file name, including the platform's own suffix."""
    system = platform.system()
    if system == "Windows":
        return f"{FMI_MODEL_IDENTIFIER}.dll"
    if system == "Darwin":
        return f"lib{FMI_MODEL_IDENTIFIER}.dylib"
    return f"lib{FMI_MODEL_IDENTIFIER}.so"


def _unit_definitions(variables: Iterable[FmiVariable]) -> list[str]:
    """
    Return the unit names the declared variables need, sorted and deduplicated.

    A unit that means a physical quantity gets its SI base factors; a
    dimensionless fraction is ``1`` with no factors at all.  Declaring the
    factors is what lets an importing tool convert, and it is cheap to state
    once rather than per variable.
    """
    units = {variable.unit for variable in variables if variable.unit}
    return sorted(units)


#: The SI base exponents of the units this export can declare, as the
#: ``(kg, m, s, A, K, mol, cd, rad)`` tuple FMI's ``<BaseUnit>`` spells as
#: *positive-integer attributes*.  A unit whose exponent set needs a fractional
#: or negative power is not expressible this way and is therefore not declared
#: with base units at all: ``<BaseUnit>`` cannot hold it, and writing it as a
#: factor would be writing an attribute the schema does not define (measured:
#: FMI 2.0 puts ``factor``/``offset`` on ``<DisplayUnitDefinition>``, not on
#: ``<BaseUnit>``).
_UNIT_FACTORS: dict[str, tuple[int, ...]] = {
    "1": (0, 0, 0, 0, 0, 0, 0, 0),
    "-": (0, 0, 0, 0, 0, 0, 0, 0),
    "s": (0, 0, 1, 0, 0, 0, 0, 0),
    "m": (0, 1, 0, 0, 0, 0, 0, 0),
    "m/s": (0, 1, -1, 0, 0, 0, 0, 0),
    "m/s^2": (0, 1, -2, 0, 0, 0, 0, 0),
    "mm": (0, 1, 0, 0, 0, 0, 0, 0),
    "mm/s^2": (0, 1, -2, 0, 0, 0, 0, 0),
    "N": (1, 1, -2, 0, 0, 0, 0, 0),
    "N*m": (1, 2, -2, 0, 0, 0, 0, 0),
    "rad": (0, 0, 0, 0, 0, 0, 0, 1),
    "rad/s": (0, 0, -1, 0, 0, 0, 0, 1),
    "count": (0, 0, 0, 0, 0, 0, 1, 0),
    "bool": (0, 0, 0, 0, 0, 0, 1, 0),
    "deg": (0, 0, 0, 0, 0, 0, 0, 1),
}

#: The ``<BaseUnit>`` attribute names, in the exponent tuple's own order.
_BASE_UNIT_ATTRIBUTES = ("kg", "m", "s", "A", "K", "mol", "cd", "rad")


def _base_unit_attributes(unit: str) -> dict[str, str]:
    """
    Return the ``<BaseUnit>`` attributes one declared unit is spelled with.

    Only the exponents FMI 2.0 can express are emitted: a zero exponent is the
    absent attribute (the schema's own default), and a negative one is *not*
    representable at all -- ``<BaseUnit>`` takes non-negative integers.  A unit
    needing a negative power therefore declares no base unit, which leaves it a
    named unit the importing tool passes through untouched, and that is the
    honest statement for ``m/s`` rather than an attribute the schema rejects.
    """
    entry = _UNIT_FACTORS.get(unit)
    if entry is None:
        raise FmiExportError(
            f"no SI base exponents are recorded for unit {unit!r}; a declared "
            "unit has to state them or an importing tool cannot interpret it"
        )
    attributes: dict[str, str] = {}
    for name, exponent in zip(_BASE_UNIT_ATTRIBUTES, entry):
        if exponent > 0:
            attributes[name] = str(exponent)
    return attributes


def model_description_document(
    *,
    model_identifier: str,
    guid: str,
    variables: tuple[FmiVariable, ...],
    name: str = "suspension_multibody_axle",
) -> str:
    """
    Return the FMI 2.0 ``modelDescription.xml`` text for one export.

    Only the standard library is used: the document is a small, fixed shape and
    ``xml.etree`` writes it without a schema library.  The element order is the
    schema's, which is the one thing an XML writer cannot get away with
    reordering -- FMI 2.0 is a sequence, not a set.
    """
    root = ElementTree.Element(
        "fmiModelDescription",
        {
            "fmiVersion": FMI_VERSION,
            "modelName": name,
            "guid": "{" + guid + "}",
            "description": (
                "Suspension multibody axle model exported for co-simulation; "
                "the binary calls the native kernel on the bundled contract "
                "documents and exposes the case's own input tables and the "
                "run's declared measurement surface"
            ),
            "generationTool": "suspension_multibody.fmi",
            "variableNamingConvention": "structured",
            "numberOfEventIndicators": "0",
        },
    )
    ElementTree.SubElement(
        root,
        "CoSimulation",
        {
            "modelIdentifier": model_identifier,
            "canHandleVariableCommunicationStepSize": "true",
            "canInterpolateInputs": "true",
            "maxOutputDerivativeOrder": "0",
            "canRunAsynchronously": "false",
            "canBeInstantiatedOnlyOncePerProcess": "false",
            "canNotUseMemoryManagementFunctions": "true",
            "providesDirectionalDerivative": "false",
        },
    )
    # `<UnitDefinitions>` is one element holding every `<Unit>`; FMI 2.0's
    # sequence has it once.  A unit whose base exponents are not all
    # expressible (see `_base_unit_attributes`) is written without a
    # `<BaseUnit>` child, which is a named unit and schema-valid.
    units = _unit_definitions(variables)
    if units:
        definitions = ElementTree.SubElement(root, "UnitDefinitions")
        for unit in units:
            entry = ElementTree.SubElement(definitions, "Unit", {"name": unit})
            attributes = _base_unit_attributes(unit)
            if attributes:
                ElementTree.SubElement(entry, "BaseUnit", attributes)
    variables_element = ElementTree.SubElement(root, "ModelVariables")
    for variable in variables:
        attributes = {
            "name": variable.name,
            "valueReference": str(variable.reference),
            "causality": variable.causality,
            "variability": variable.variability,
            "description": variable.description,
        }
        if variable.unit:
            attributes["unit"] = variable.unit
        scalar = ElementTree.SubElement(
            variables_element, "ScalarVariable", attributes
        )
        # Every variable is a Real: the case tables are float64 and so are the
        # result blocks.  Declaring Integer/Boolean/String types would be
        # advertising a typed interface the kernel does not have.
        ElementTree.SubElement(scalar, "Real", {"relativeQuantity": "false"})
    structure = ElementTree.SubElement(root, "ModelStructure")
    outputs = ElementTree.SubElement(structure, "Outputs")
    # `index` is the variable's *1-based position* in `<ModelVariables>`, not its
    # `valueReference`: the two are different numbers and using the reference
    # here would point at a different variable entirely (measured against the
    # FMI 2.0 schema, where `fmi2ValueReference` and the structure index are
    # separate quantities).
    for position, variable in enumerate(variables, start=1):
        if variable.causality != "output":
            continue
        ElementTree.SubElement(
            outputs, "Unknown", {"index": str(position), "dependencies": ""}
        )
    return ElementTree.tostring(root, encoding="unicode", xml_declaration=False)


# --- the archive ------------------------------------------------------------


def _guid(model_document: Mapping[str, Any], case_payload: bytes) -> str:
    """
    Return the export's GUID, derived from the pair it was exported from.

    The model contributes its package-canonical hash and the case contributes
    the SHA-256 of its container bytes, so two exports of one pair agree and
    two exports of different pairs do not.  Nothing here is random or
    time-dependent: a co-simulation artifact a build system cannot reproduce is
    one every downstream cache has to invalidate.
    """
    case_document, _blob = unpack_container(case_payload)
    material = {
        "model_hash": canonical_hash(model_document),
        "case_contract_hash": contract_hash(case_document),
        "case_payload_sha256": canonical_hash(
            {"case_bytes": case_payload.hex()}
        ),
    }
    return canonical_hash(material)


def _binary_path(package_root: Path) -> Path:
    """Return where the built wrapper binary lives inside the package."""
    return package_root / "fmi" / fmi_binary_name()


def _build_script() -> Path:
    """Return the script that builds the wrapper binary."""
    return Path(__file__).resolve().parents[3] / "scripts" / "build_fmu_binary.py"


def _require_binary(package_root: Path) -> Path:
    """Return the wrapper binary, refusing by name when it has not been built."""
    binary = _binary_path(package_root)
    if not binary.is_file():
        raise FmiExportError(
            f"the FMU wrapper binary is missing at {binary}; run "
            f"`uv run --no-sync python {_build_script()}` to build it, which "
            "needs an x86_64 C compiler on PATH (see the script's compiler "
            "notes)"
        )
    return binary


def export_fmu(
    destination: Path,
    *,
    assembly_document: Any,
    case_document: Any,
) -> FmuExport:
    """
    Export one run as an FMI 2.0 Co-Simulation FMU and return what it declares.

    The ordinary assembly and case documents use the same validation and
    compilation entry as ``simulate``. The compiled containers are archived
    unchanged and the FMI binary evaluates them with the native kernel.

    ``destination`` may be a ``.fmu`` path or a directory, in which case the
    file is named after the model identifier.
    """
    from ..api import validate

    compiled = validate(assembly_document, case_document)
    model_doc, case_doc = compiled.model_document, compiled.case_document
    name = str(case_doc.get("name") or compiled.request.name or "run")
    variables = variable_declarations(case_doc, model_doc)
    guid = _guid(compiled.model_document, compiled.case_payload)
    package_root = Path(__file__).resolve().parents[1]
    binary = _require_binary(package_root)

    target = Path(destination)
    if target.suffix.lower() != ".fmu":
        target = target / f"{FMI_MODEL_IDENTIFIER}.fmu"
    target.parent.mkdir(parents=True, exist_ok=True)

    description = model_description_document(
        model_identifier=FMI_MODEL_IDENTIFIER,
        guid=guid,
        variables=variables,
        name=name,
    )
    # The ZIP timestamps are pinned so re-exporting one pair produces the same
    # bytes.  `zipfile` otherwise stamps the wall clock into every member.
    with zipfile.ZipFile(target, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        _write_member(archive, _MODEL_DESCRIPTION, description.encode("utf-8"))
        _write_member(
            archive,
            f"binaries/{fmi_platform()}/{binary.name}",
            binary.read_bytes(),
        )
        _write_member(archive, _RESOURCES_MODEL, bytes(compiled.model_payload))
        _write_member(archive, _RESOURCES_CASE, bytes(compiled.case_payload))
        _write_member(
            archive,
            _RESOURCES_BINDINGS,
            bindings_resource(case_doc, variables).encode("utf-8"),
        )

    return FmuExport(
        path=target,
        model_identifier=FMI_MODEL_IDENTIFIER,
        variables=variables,
        fmi_version=FMI_VERSION,
        guid=guid,
    )


def _write_member(archive: zipfile.ZipFile, name: str, payload: bytes) -> None:
    """Write one archive member with a fixed timestamp, for reproducibility."""
    info = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
    info.compress_type = zipfile.ZIP_DEFLATED
    info.external_attr = 0o644 << 16
    archive.writestr(info, payload)


def read_description(path: Path) -> ElementTree.Element:
    """
    Return the parsed ``modelDescription.xml`` of an exported FMU.

    Convenience for callers and tests that want the document without reopening
    the archive by hand; it reads the archive and parses the member.
    """
    with zipfile.ZipFile(path) as archive:
        text = archive.read(_MODEL_DESCRIPTION).decode("utf-8")
    return ElementTree.fromstring(text)


def description_json(path: Path) -> str:
    """Return the FMU's variable list as a readable JSON string."""
    root = read_description(path)
    variables = []
    for scalar in root.iter("ScalarVariable"):
        variables.append(
            {
                "name": scalar.get("name"),
                "reference": int(scalar.get("valueReference", "-1")),
                "causality": scalar.get("causality"),
                "variability": scalar.get("variability"),
                "unit": scalar.get("unit"),
                "description": scalar.get("description"),
            }
        )
    return json.dumps({"variables": variables}, indent=2)


def _copy_binary(source: Path, destination: Path) -> None:
    """Copy the wrapper binary into a staging directory (used by the builder)."""
    shutil.copy2(source, destination)
