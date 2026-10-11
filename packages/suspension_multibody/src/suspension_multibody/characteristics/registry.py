"""
The explicit definition catalogue: which kinds and versions exist.

A registry rather than dispatch by naming convention.  The failure it prevents is
specific: a caller asks for ``roll_force_equivalent_v2``, the code does not know
that name, and it falls back to whatever definition it *does* have -- silently
answering a different question under the requested name.  Here an unknown kind or
version is refused, and the available set is named in the error.

The catalogue is deliberately small and flat.  It lists what this build can
actually compute; a definition that is designed but not implemented belongs in the
capability document as an unimplemented kind, not in here as a stub that returns
something plausible.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from .centers import (
    PITCH_CENTER_FORCE_VERSION,
    ROLL_CENTER_FORCE_VERSION,
    ROLL_CENTER_LEGACY_VERSION,
    force_equivalent_pitch_center,
    force_equivalent_roll_center,
    legacy_roll_center,
)
from .values import DOMAINS, CharacteristicsError

#: The measurement definitions this build implements, by kind.
MEASUREMENT_KINDS = ("toe", "camber", "frame_position")


@dataclass(frozen=True)
class Definition:
    """
    One named, versioned definition and the domain it is meaningful in.

    ``operator`` is ``None`` when the definition is realised by the caller's own
    declared operator rather than by a function here -- a measurement derivative
    is computed from whatever measurement the request named, so there is no single
    function to point at.  That is distinct from a missing implementation, which
    would not appear in the catalogue at all.

    ``requires`` names the data the operator needs.  It is carried here rather
    than discovered by catching a ``TypeError`` at call time: a request that needs
    the contact-motion basis should be refused because the boundary did not
    declare one, not because a call happened to fail.
    """

    kind: str
    version: int
    domain: str
    operator: Callable[..., Any] | None
    requires: tuple[str, ...]

    def __post_init__(self) -> None:
        if self.domain not in DOMAINS:
            raise CharacteristicsError(f"definition {self.kind!r} names unknown domain {self.domain!r}")
        if self.version < 1:
            raise CharacteristicsError(f"definition {self.kind!r} needs a positive version")


def _catalogue() -> tuple[Definition, ...]:
    return (
        Definition(
            kind="measurement_derivative",
            version=1,
            domain="kinematic",
            operator=None,  # supplied by the caller's declared measurement operator
            requires=("measurements", "perturbations"),
        ),
        Definition(
            kind=ROLL_CENTER_FORCE_VERSION,
            version=2,
            domain="equilibrium",
            operator=force_equivalent_roll_center,
            requires=("contact_points", "contact_motions", "load_weights"),
        ),
        Definition(
            kind=PITCH_CENTER_FORCE_VERSION,
            version=1,
            domain="equilibrium",
            operator=force_equivalent_pitch_center,
            requires=("contact_points", "contact_motions", "load_weights"),
        ),
        Definition(
            kind=ROLL_CENTER_LEGACY_VERSION,
            version=1,
            domain="equilibrium",
            operator=legacy_roll_center,
            requires=("contact_patches", "contact_slopes"),
        ),
    )


def definitions() -> tuple[Definition, ...]:
    """Return every implemented definition."""
    return _catalogue()


def definition(kind: str, version: int) -> Definition:
    """
    Return one exact definition, or refuse and name what does exist.

    Both the kind and the version must match.  Matching only the kind would let a
    future v3 request be answered with the v2 formula, which is exactly the
    silent substitution the version field exists to prevent.
    """
    matches = [item for item in _catalogue() if item.kind == kind]
    if not matches:
        available = ", ".join(sorted({item.kind for item in _catalogue()}))
        raise CharacteristicsError(
            f"unknown characteristics definition kind {kind!r}; implemented kinds: {available}"
        )
    for item in matches:
        if item.version == version:
            return item
    available = ", ".join(str(item.version) for item in matches)
    raise CharacteristicsError(
        f"definition {kind!r} has no version {version}; implemented versions: {available}"
    )


def require_domain(kind: str, version: int, domain: str) -> Definition:
    """
    Return a definition only if it is meaningful in the requested domain.

    The domain is checked separately from the existence of the definition because
    the two failures mean different things: an unknown kind is a typo or an
    unimplemented feature, while a known kind in the wrong domain is a request
    that does not make sense (a frozen-state stiffness has no quasi-static
    meaning).
    """
    found = definition(kind, version)
    if found.domain != domain:
        raise CharacteristicsError(
            f"definition {kind!r} v{version} is a {found.domain!r} definition and cannot "
            f"answer a {domain!r} request"
        )
    return found


def missing_requirements(found: Definition, supplied: Any) -> tuple[str, ...]:
    """
    Return the requirement names the caller did not supply.

    An empty tuple means the request carries what the definition needs.  This
    turns "the boundary did not declare contact motions" into a named gap instead
    of a failure deep inside the operator.
    """
    return tuple(name for name in found.requires if name not in supplied)


__all__ = [
    "MEASUREMENT_KINDS",
    "Definition",
    "definition",
    "definitions",
    "missing_requirements",
    "require_domain",
]
