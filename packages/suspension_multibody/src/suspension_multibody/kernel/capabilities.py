"""
The kernel's capability declaration, read lazily.

The native library declares what it can actually run -- the supported PAC2002
USE_MODEs, the coefficient families and feature flags it refuses, and the
reasons it refuses them.  That declaration is the single source for every
fail-closed scope check on the authoring side, so it is read from the library
rather than transcribed into Python, where a copy would drift in the direction
that matters (the old hand-copied list claimed the kernel never applies the
validity-range clamps while the kernel was calling ``pac2002_clamp_load`` on
every tire step).

Reading stays lazy on purpose: ``suspension_multibody`` imports the authoring
schemas on the default path, and those schemas import this module.  Nothing
here loads the library at import time -- the first access to one of the
kernel-sourced constants, or the first scope check, opens it.
"""

from __future__ import annotations

import functools
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class Pac2002FeatureFamily:
    """
    A documented PAC2002 feature the native kernel cannot run exactly.

    ``coefficients`` lists the tire-property keywords that only carry a non-zero
    value when the feature is actually requested.
    """

    name: str
    reason: str
    coefficients: frozenset[str]


@functools.lru_cache(maxsize=1)
def _kernel_scope() -> tuple[
    frozenset[int], frozenset[str], frozenset[str], tuple[Pac2002FeatureFamily, ...]
]:
    """
    Return the kernel's capability declaration.

    Read from the shared library rather than carried here, and read lazily: the
    authoring schemas import this module, and there is no reason for them to need
    a built kernel until a tire actually reaches the scope check.
    """
    import ctypes

    from .native import load_library

    reader = load_library().suspension_kernel_capabilities
    reader.argtypes = [
        ctypes.c_char_p,
        ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_size_t),
    ]
    reader.restype = ctypes.c_int32
    needed = ctypes.c_size_t(0)
    status = int(reader(None, 0, ctypes.byref(needed)))
    if status != 11:
        raise RuntimeError(
            f"kernel capability probe returned {status}; expected the size request"
        )
    buffer = ctypes.create_string_buffer(int(needed.value))
    status = int(reader(buffer, int(needed.value), ctypes.byref(needed)))
    if status != 0:
        raise RuntimeError(f"kernel capability read returned {status}")
    document = json.loads(buffer.value.decode("utf-8"))
    families = tuple(
        Pac2002FeatureFamily(
            name=str(entry["name"]),
            reason=str(entry["reason"]),
            coefficients=frozenset(str(value) for value in entry["coefficients"]),
        )
        for entry in document["pac2002_refused_families"]
    )
    return (
        frozenset(int(value) for value in document["pac2002_supported_use_modes"]),
        frozenset(str(value) for value in document["pac2002_refused_parameters"]),
        frozenset(str(value) for value in document["pac2002_refused_feature_flags"]),
        families,
    )


#: The constants the kernel owns.  They are resolved on first access rather than
#: at import, so importing the authoring schemas does not load the library.
_KERNEL_SOURCED = (
    "PAC2002_SUPPORTED_NATIVE_USE_MODES",
    "PAC2002_UNSUPPORTED_NATIVE_PARAMETERS",
    "PAC2002_UNSUPPORTED_NATIVE_FEATURE_FLAGS",
    "PAC2002_UNSUPPORTED_FEATURE_FAMILIES",
    "PAC2002_MUST_BE_ZERO_COEFFICIENTS",
)


def __getattr__(name: str) -> Any:
    """Resolve the kernel-sourced constants on first access."""
    if name not in _KERNEL_SOURCED:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    modes, parameters, flags, families = _kernel_scope()
    if name == "PAC2002_SUPPORTED_NATIVE_USE_MODES":
        value: Any = modes
    elif name == "PAC2002_UNSUPPORTED_NATIVE_PARAMETERS":
        value = parameters
    elif name == "PAC2002_UNSUPPORTED_NATIVE_FEATURE_FLAGS":
        value = flags
    elif name == "PAC2002_UNSUPPORTED_FEATURE_FAMILIES":
        value = families
    else:
        value = parameters | frozenset(
            coefficient for family in families for coefficient in family.coefficients
        )
    globals()[name] = value
    return value


def pac2002_native_use_mode(coefficients: Mapping[str, float]) -> int:
    """Return the Adams USE_MODE magnitude used for native scope checks."""
    raw = abs(float(coefficients.get("USE_MODE", 14.0)))
    if not math.isfinite(raw):
        raise ValueError("PAC2002 USE_MODE must be finite")
    return int(round(raw))


def _is_absent_or_zero(value: float) -> bool:
    if not math.isfinite(value):
        return False
    return abs(value) <= 1.0e-12


def pac2002_unsupported_native_reasons(
    coefficients: Mapping[str, float],
) -> tuple[str, ...]:
    """
    List native PAC2002 blockers that must not be silently approximated.

    This is the *capability*-derived half of the Adams evidence: it can only
    refuse what the kernel itself says it cannot run.  A bare module-level name
    would not do: the kernel-sourced constants resolve through ``__getattr__``,
    which a global lookup inside a function does not consult.
    """
    supported_modes, refused_parameters, refused_flags, refused_families = _kernel_scope()
    reasons: list[str] = []

    use_mode = pac2002_native_use_mode(coefficients)
    if use_mode not in supported_modes:
        reasons.append(f"unsupported USE_MODE {use_mode}")

    for name in sorted(refused_parameters):
        value = float(coefficients.get(name, 0.0))
        if not math.isfinite(value):
            reasons.append(f"non-finite unsupported parameter {name}")
        elif abs(value) > 1.0e-12:
            reasons.append(f"unsupported parameter {name}")

    for name in sorted(refused_flags):
        value = float(coefficients.get(name, 0.0))
        if not math.isfinite(value):
            reasons.append(f"non-finite unsupported feature flag {name}")
        elif abs(value) > 1.0e-12:
            reasons.append(name.replace("PAC2002_UNSUPPORTED_", "unsupported ").lower())

    for family in refused_families:
        requested = [
            name
            for name in sorted(family.coefficients)
            if not _is_absent_or_zero(float(coefficients.get(name, 0.0)))
        ]
        if requested:
            reasons.append(f"{family.reason} ({', '.join(requested)})")

    return tuple(reasons)
