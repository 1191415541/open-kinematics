"""Shared numeric-entry parsing helpers for GUI workbenches."""

from __future__ import annotations

from dataclasses import dataclass

PARTIAL_FLOAT_TEXT = frozenset({"", "+", "-", ".", "+.", "-."})


@dataclass(frozen=True)
class ParsedFloatEntry:
    """Result of parsing a live GUI float entry."""

    value: float
    is_valid: bool
    is_complete: bool


@dataclass(frozen=True)
class ParsedIntEntry:
    """Result of parsing a live GUI integer entry."""

    value: int
    is_valid: bool
    is_complete: bool


def parse_float_entry(text: str, previous: float) -> ParsedFloatEntry:
    """Parse a live numeric entry without rejecting partial edits."""
    stripped = text.strip()
    if stripped in PARTIAL_FLOAT_TEXT:
        return ParsedFloatEntry(previous, is_valid=True, is_complete=False)
    try:
        return ParsedFloatEntry(float(stripped), is_valid=True, is_complete=True)
    except ValueError:
        return ParsedFloatEntry(previous, is_valid=False, is_complete=False)


def parse_int_entry(text: str, previous: int) -> ParsedIntEntry:
    """
    Parse a live integer entry while supporting partial edits.

    Accepts textual integers like ``"12"`` and float-like integers such as ``"12.0"``.
    """
    parsed = parse_float_entry(text, float(previous))
    if not parsed.is_valid:
        return ParsedIntEntry(previous, is_valid=False, is_complete=False)
    if not parsed.is_complete:
        return ParsedIntEntry(previous, is_valid=True, is_complete=False)

    candidate = parsed.value
    rounded = round(candidate)
    if abs(candidate - rounded) > 1e-9:
        return ParsedIntEntry(previous, is_valid=False, is_complete=False)
    return ParsedIntEntry(int(rounded), is_valid=True, is_complete=True)
