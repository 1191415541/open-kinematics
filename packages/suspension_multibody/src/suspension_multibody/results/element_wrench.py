"""
The frozen element-wrench fact channel (05 step 5), decoded.

The channel is a *fact* surface: native records, once per sample, the wrench each
element law actually applied to one body -- the world force, the world moment
about the receiving body's origin, the type of law that applied it and the world
point it acts at.  Nothing here computes a constitutive law, and nothing here
calls native: the module only reads a block a run has already produced.

The channel is **off** by default.  Native emits the ``element_wrench`` block
only while ``SUSPENSION_KERNEL_ELEMENT_WRENCH_OUTPUT`` asks for it -- unset,
empty, or a leading ``0`` is off, which is what :func:`element_wrench_enabled`
mirrors -- and while it is off nothing is allocated, described or written, so
the block is absent and the result document keeps ``contract_version`` 1.  With
the switch on the version is 2 and every sample carries ``(records, 13)`` rows.

The columns are frozen by ``mb_config/element_wrench.hpp``:
:data:`FORCE_COLUMNS` is the world force in N, :data:`MOMENT_COLUMNS` the world
moment in N*m about the receiving body's origin, :data:`TYPE_CODE_COLUMN` the
element type code, :data:`POINT_COLUMNS` the world coordinates of the point the
force acts at, and :data:`BODY_A_COLUMN`, :data:`BODY_B_COLUMN` and
:data:`BODY_COLUMN` the element's two bodies and the body the record belongs to.
The record carries those indices, not names: naming a body is the caller's job,
which is why :func:`decode_element_wrench` only *checks* ``body_names``.

Row order is part of the channel, because a record has no index column of its
own.  Within a sample the types are laid out in code order (1..7), and within a
type the rows of element ``x`` start at ``x * rows_per_element(code)`` and run
one row per end: spring, bushing, anti-roll bar and steering actuator reserve
two rows each (end 0 and end 1 -- for a bushing, end 0 is its ``body_b`` end and
end 1 its ``body_a`` end), a tire's drive/brake pair four (drive, its reaction,
brake, its reaction), and a tire's contact wrench or an external source one.
A row nothing was applied to stays NaN rather than being filled with zeros, so a
row whose force, moment and type code are all NaN is *not* a record: the element
applied nothing to that body in that sample.  A genuinely applied zero stays
``0.0`` and *is* a record.  An element that carries one body rather than two --
a tire's contact wrench, an external source -- writes ``-1`` in the body column
it does not have, which is native's "no such body" and not an out-of-range name.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping, Protocol, Sequence

import numpy as np

from .raw import RawContractResult

ELEMENT_WRENCH_WIDTH = 13
"""Columns per record: force, moment, type code, point, body a, body b, body."""

ELEMENT_WRENCH_BLOCK = "element_wrench"
"""The result block the channel writes; absent while the channel is off."""

ELEMENT_WRENCH_SWITCH = "SUSPENSION_KERNEL_ELEMENT_WRENCH_OUTPUT"
"""The environment variable that turns the channel on."""

FORCE_COLUMNS: tuple[int, int, int] = (0, 1, 2)
"""World force columns (N)."""

MOMENT_COLUMNS: tuple[int, int, int] = (3, 4, 5)
"""World moment columns (N*m) about the receiving body's origin."""

TYPE_CODE_COLUMN = 6
"""The column holding the element type code."""

POINT_COLUMNS: tuple[int, int, int] = (7, 8, 9)
"""World coordinates of the point the force acts at (m)."""

BODY_A_COLUMN = 10
"""The column holding the element's body a index."""

BODY_B_COLUMN = 11
"""The column holding the element's body b index."""

BODY_COLUMN = 12
"""The column holding the body the record belongs to."""

_ELEMENT_WRENCH_TYPES: tuple[tuple[int, str, int], ...] = (
    (1, "spring", 2),
    (2, "bushing", 2),
    (3, "anti_roll", 2),
    (4, "steering", 2),
    (5, "drive_brake", 4),
    (6, "tire", 1),
    (7, "external", 1),
)

ELEMENT_WRENCH_TYPE_NAMES: Mapping[int, str] = MappingProxyType(
    {code: name for code, name, _ in _ELEMENT_WRENCH_TYPES}
)
"""The channel's frozen type codes, mapped to the law that applied the wrench."""

_ROWS_PER_ELEMENT: Mapping[int, int] = MappingProxyType(
    {code: rows for code, _, rows in _ELEMENT_WRENCH_TYPES}
)

# The columns a row has to carry for the element to have applied anything: the
# force, the moment and the type code, which the layout keeps contiguous.
_APPLIED_COLUMNS = slice(FORCE_COLUMNS[0], TYPE_CODE_COLUMN + 1)


class ElementWrenchBlocks(Protocol):
    """A result object that exposes native block arrays by name."""

    @property
    def named_blocks(self) -> Mapping[str, np.ndarray]:
        """Return the run's blocks, keyed by block name."""
        ...


ElementWrenchSource = RawContractResult | Mapping[str, np.ndarray] | ElementWrenchBlocks
"""What :func:`element_wrench_block` accepts: the neutral result, a block
mapping, or any object exposing ``named_blocks`` (or, at run time, ``blocks``)."""


def element_wrench_enabled() -> bool:
    """
    Whether the switch asks native for the element-wrench channel.

    This mirrors the native predicate exactly: the variable unset, empty, or
    starting with ``0`` is off.  Native reads the variable on every call, so
    setting it in-process takes effect on the next run; this function is only a
    reading of the same switch, never a substitute for the block itself.
    """
    value = os.environ.get(ELEMENT_WRENCH_SWITCH)
    return value is not None and value != "" and not value.startswith("0")


def rows_per_element(type_code: int) -> int:
    """
    Return the rows one element of ``type_code`` reserves in a sample.

    The count is the row stride of that type's group, so the rows of element
    ``x`` are ``x * rows_per_element(type_code)`` onward, one per end.  A code
    the channel does not define is refused rather than guessed at.
    """
    rows = _ROWS_PER_ELEMENT.get(type_code)
    if rows is None:
        raise ValueError(f"{type_code!r} is not an element wrench type code")
    return rows


@dataclass(frozen=True)
class ElementWrenchRecord:
    """
    One element's applied wrench on one body in one sample.

    ``force`` is the world force in N, ``moment`` the world moment in N*m about
    the origin of body ``body``, and ``point`` the world point the force acts at.
    ``body_a`` and ``body_b`` are the element's own two bodies and ``body`` is
    the one this record belongs to, all as indices into the model's body order;
    a ``-1`` in ``body_a`` or ``body_b`` means the element has no such body,
    which is what a tire's contact wrench and an external source write.  A
    record is only produced for a row the element actually applied something to,
    so a row left NaN never becomes one; a pure-moment record keeps the point
    its row was opened with, which may be NaN.
    """

    sample: int
    type_code: int
    type_name: str
    body: int
    body_a: int
    body_b: int
    force: tuple[float, float, float]
    moment: tuple[float, float, float]
    point: tuple[float, float, float]


def _blocks_of(result: ElementWrenchSource) -> Mapping[str, np.ndarray]:
    """Return the block mapping of a result, however the caller spells it."""
    if isinstance(result, Mapping):
        return result
    for attribute in ("named_blocks", "blocks"):
        blocks = getattr(result, attribute, None)
        if isinstance(blocks, Mapping):
            return blocks
    raise TypeError(
        "an element wrench source must be a mapping of block arrays, or an "
        "object exposing named_blocks or blocks"
    )


def element_wrench_block(result: ElementWrenchSource) -> np.ndarray | None:
    """
    Return the element-wrench block, or ``None`` when the channel was off.

    An absent block is the normal state of the default path, not an error, so
    this never raises for a missing channel: a caller that wants the block
    distinguishes "the switch was off" from "the run failed" by asking
    :func:`element_wrench_enabled`.
    """
    return _blocks_of(result).get(ELEMENT_WRENCH_BLOCK)


def decode_element_wrench(
    result: ElementWrenchSource,
    *,
    body_names: Sequence[str] | None = None,
) -> tuple[ElementWrenchRecord, ...]:
    """
    Decode the element-wrench block into records, sample by sample.

    A missing block is an empty tuple: the channel was off.  A block that is not
    ``(samples, records, ELEMENT_WRENCH_WIDTH)`` is refused, as is a row whose
    type code the channel does not define -- a code that is silently treated as
    "unknown" would be a fact this module invented.

    Rows nothing was applied to are skipped, so the result only carries facts.
    ``body_names``, when given, is checked against the record's body indices and
    nothing more: the records stay index-based, because naming a body is the
    caller's model knowledge and not something the channel carries.
    """
    block = element_wrench_block(result)
    if block is None:
        return ()
    values = np.asarray(block, dtype=np.float64)
    if values.ndim != 3 or values.shape[2] != ELEMENT_WRENCH_WIDTH:
        raise ValueError(
            f"the {ELEMENT_WRENCH_BLOCK!r} block must be "
            f"(samples, records, {ELEMENT_WRENCH_WIDTH}); got shape {values.shape}"
        )
    records: list[ElementWrenchRecord] = []
    for sample in range(values.shape[0]):
        for row in range(values.shape[1]):
            entry = values[sample, row]
            if np.isnan(entry[_APPLIED_COLUMNS]).any():
                # The element applied nothing to this body in this sample.
                continue
            records.append(_record(entry, sample, row, body_names))
    return tuple(records)


def _record(
    entry: np.ndarray,
    sample: int,
    row: int,
    body_names: Sequence[str] | None,
) -> ElementWrenchRecord:
    """Build one record from a row the element applied a wrench in."""
    code_value = float(entry[TYPE_CODE_COLUMN])
    code = int(code_value)
    name = ELEMENT_WRENCH_TYPE_NAMES.get(code)
    if name is None or code_value != code:
        raise ValueError(
            f"element wrench row {row} of sample {sample} carries the unknown "
            f"type code {code_value!r}"
        )
    body_a = _index(float(entry[BODY_A_COLUMN]), "body_a", sample, row)
    body_b = _index(float(entry[BODY_B_COLUMN]), "body_b", sample, row)
    body = _index(float(entry[BODY_COLUMN]), "body", sample, row)
    if body_names is not None:
        for column, index in (("body_a", body_a), ("body_b", body_b), ("body", body)):
            # A negative index is native's "no such body" -- an external source
            # or a tire carries one body, not two -- and is not an out-of-range
            # name.  The record's own body is always a real one.
            if index < 0 and column != "body":
                continue
            if not 0 <= index < len(body_names):
                raise ValueError(
                    f"element wrench row {row} of sample {sample} names {column} "
                    f"{index}, outside the {len(body_names)} body names given"
                )
    return ElementWrenchRecord(
        sample=sample,
        type_code=code,
        type_name=name,
        body=body,
        body_a=body_a,
        body_b=body_b,
        force=_vector(entry, FORCE_COLUMNS),
        moment=_vector(entry, MOMENT_COLUMNS),
        point=_vector(entry, POINT_COLUMNS),
    )


def _vector(
    entry: np.ndarray, columns: tuple[int, int, int]
) -> tuple[float, float, float]:
    """Return the three columns of a vector field as plain floats."""
    return (
        float(entry[columns[0]]),
        float(entry[columns[1]]),
        float(entry[columns[2]]),
    )


def _index(value: float, column: str, sample: int, row: int) -> int:
    """Return a body index column, refusing anything that is not an index."""
    index = int(value) if np.isfinite(value) else None
    if index is None or index != value:
        raise ValueError(
            f"element wrench row {row} of sample {sample} carries {column} "
            f"{value!r}, which is not a body index"
        )
    return index


__all__ = [
    "BODY_A_COLUMN",
    "BODY_B_COLUMN",
    "BODY_COLUMN",
    "ELEMENT_WRENCH_BLOCK",
    "ELEMENT_WRENCH_SWITCH",
    "ELEMENT_WRENCH_TYPE_NAMES",
    "ELEMENT_WRENCH_WIDTH",
    "FORCE_COLUMNS",
    "MOMENT_COLUMNS",
    "POINT_COLUMNS",
    "TYPE_CODE_COLUMN",
    "ElementWrenchBlocks",
    "ElementWrenchRecord",
    "ElementWrenchSource",
    "decode_element_wrench",
    "element_wrench_block",
    "element_wrench_enabled",
    "rows_per_element",
]
