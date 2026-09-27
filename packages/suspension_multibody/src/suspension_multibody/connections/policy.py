"""
The global assembly rules: what a root assembly kind may and must contain.

These rules are the ones the user fixed and refused to have relaxed (``D3``). They
were previously spread across the assembly builders -- each one knowing that an
axle has no brake and a vehicle must have steering -- so "is this legal?" had as
many answers as there were places that built an assembly, and a template or a
recipe could quietly step outside them.

Here they are stated once, as data, and applied to the *root* assembly's category.
That last word carries the weight: a vehicle contains two axles, and an axle
inside a vehicle is not an independent single-axle simulation. Applying the
single-axle rule to it would demand that the vehicle's own wheels be removed, so
the rule has to be looked up by the root's kind and checked once.

Nothing here may be overridden by a template, a recipe, or a user registration.
That is not a convention: :func:`check_root` is the only sanctioned entry point
and it takes no override argument, so a caller that wants a different rule has to
change this file and say so in review.
"""

from __future__ import annotations

from dataclasses import dataclass

__all__ = [
    "AXLE_RULE",
    "ROOT_KINDS",
    "RootRule",
    "RuleViolation",
    "VEHICLE_RULE",
    "check_root",
    "rule_for",
]

#: The two root categories the global rules distinguish.
ROOT_KINDS: tuple[str, ...] = ("axle", "vehicle")

#: The six subsystem roles, in the vocabulary the templates use.
ROLES: tuple[str, ...] = ("suspension", "steering", "wheel", "chassis", "brake", "drive")


class RuleViolation(ValueError):
    """
    An assembly breaks a global rule.

    A distinct type rather than a bare ``ValueError`` because the rule is not a
    capability shortfall: an axle carrying a brake is not "missing something",
    it is an assembly that must not exist. Callers that report the difference to
    a user need to be able to tell those apart.
    """


@dataclass(frozen=True)
class RootRule:
    """
    What one root category forbids and requires.

    ``forbidden`` and ``required`` are roles, not template names, so a new
    template for a forbidden role is refused by construction rather than by
    someone remembering to add a check.
    """

    kind: str
    #: Roles this kind must not carry, at its own level.
    forbidden: frozenset[str]
    #: Roles this kind must carry.
    required: frozenset[str]
    #: Whether the rig supplies the wheels, or the assembly owns them.
    wheels_from_rig: bool
    #: Why the rule is what it is, for the error message.
    reason: str


#: A single axle: suspension plus a fixed chassis support, optional steering, and
#: **no** brake or drive.  The wheels come from the rig, because a bare axle on a
#: bench is loaded through them.
#:
#: ``chassis`` is required and always has been in effect: the axle's suspension
#: hangs from a fixed support, and an assembly without one has nothing to react
#: against.  It was missing from ``required`` while the assembly enforced it
#: inline, which meant the rule and the behaviour disagreed; the assembly's check
#: has moved here, so the list now says what the rule's own name says.
AXLE_RULE = RootRule(
    kind="axle",
    forbidden=frozenset({"brake", "drive"}),
    required=frozenset({"suspension", "chassis"}),
    wheels_from_rig=True,
    reason=(
        "a single-axle bench assembly carries no brake or drive: the bench loads "
        "the axle through its wheels, and the wheel itself belongs to the rig"
    ),
)

#: A full vehicle: it owns its wheels, and it must have steering, brake and drive.
VEHICLE_RULE = RootRule(
    kind="vehicle",
    forbidden=frozenset(),
    required=frozenset({"suspension", "steering", "brake", "drive"}),
    wheels_from_rig=False,
    reason=(
        "a full vehicle owns its wheels and must be able to steer, brake and "
        "drive; a vehicle missing one of those is not a vehicle model"
    ),
)

_RULES: dict[str, RootRule] = {AXLE_RULE.kind: AXLE_RULE, VEHICLE_RULE.kind: VEHICLE_RULE}


def rule_for(kind: str) -> RootRule:
    """Return the rule for a root category, naming the unknown one if it is not."""
    try:
        return _RULES[kind]
    except KeyError as exc:
        known = ", ".join(sorted(_RULES))
        raise RuleViolation(
            f"unknown root assembly kind {kind!r}; the known kinds are {known}"
        ) from exc


def check_root(kind: str, roles: frozenset[str] | set[str]) -> None:
    """
    Raise unless an assembly of category ``kind`` may carry exactly ``roles``.

    ``roles`` is what the root level itself carries -- a nested axle's roles are
    not passed here, and deliberately so: the rule is about the root, and asking
    a nested axle to satisfy the single-axle rule would make every vehicle fail.

    Both directions are checked. A forbidden role present is a violation, and a
    required role absent is one too, because "the vehicle has no brake" and "the
    axle has a brake" are the same class of mistake: an assembly that is not the
    thing it claims to be.
    """
    rule = rule_for(kind)
    present = set(roles)

    unknown = sorted(present - set(ROLES))
    if unknown:
        raise RuleViolation(
            f"assembly of kind {kind!r} declares unknown role(s) {unknown}; the "
            f"known roles are {', '.join(ROLES)}"
        )

    forbidden_present = sorted(present & rule.forbidden)
    if forbidden_present:
        raise RuleViolation(
            f"assembly of kind {kind!r} must not carry {forbidden_present}: "
            f"{rule.reason}"
        )

    required_missing = sorted(rule.required - present)
    if required_missing:
        raise RuleViolation(
            f"assembly of kind {kind!r} is missing required role(s) "
            f"{required_missing}: {rule.reason}"
        )
