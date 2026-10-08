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

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field

__all__ = [
    "ASSEMBLY_RULES",
    "AXLE_RULE",
    "ROOT_KINDS",
    "AssemblyRule",
    "RuleViolation",
    "RootRule",
    "RuleViolation",
    "VEHICLE_RULE",
    "check_assembly_roles",
    "check_assembly_shape",
    "check_forbidden_roles",
    "check_root",
    "rule_for",
    "rule_for_assembly",
]

#: The two root categories the global rules distinguish.
ROOT_KINDS: tuple[str, ...] = ("axle", "vehicle")

#: The six subsystem roles, in the vocabulary the templates use.
ROLES: tuple[str, ...] = (
    "suspension",
    "steering",
    "wheel",
    "chassis",
    "brake",
    "drive",
    "anti_roll_bar",
)


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
    required=frozenset({"suspension"}),
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


#: What a file-driven assembly must carry, per root category.
#:
#: The same rules the linear reads above state, written as *counts and placements*
#: rather than as a set of roles, because a file declares which subsystem sits where
#: and "two suspensions" is satisfied by two front axles -- which is not a vehicle.
#: Kept here rather than in the authoring layer so there is one home for "what an
#: assembly of this kind is": the layers that consume it differ in how they read an
#: assembly, not in what an assembly is allowed to be.
@dataclass(frozen=True)
class AssemblyRule:
    """The exact shape one assembly category must have."""

    kind: str
    #: How many subsystems each functional role carries, exactly.
    #:
    #: Only the roles whose number is a fact about the *kind* belong here: a vehicle
    #: has exactly one body, and how many suspensions it has is a fact about the
    #: vehicle rather than an axle count a table may fix in advance.
    role_counts: Mapping[str, int]
    #: Roles that must appear at least this many times, with no ceiling.
    #:
    #: This is what lets a file declare three axles.  A mapping rather than a set,
    #: so a kind can state "at least two" when that is the least a meaningful
    #: assembly of that kind has.
    min_counts: Mapping[str, int] = field(default_factory=dict)
    #: ``(functional_role, placement_role)`` that must be present.
    required_placements: frozenset[tuple[str, str]] = frozenset()
    #: Roles this category may carry, and at most how many.  An axle's steering is
    #: optional but never doubled, which is a *ceiling* rather than a count.
    at_most: Mapping[str, int] = field(default_factory=dict)
    #: Roles this category must not carry at all.
    forbidden_roles: frozenset[str] = frozenset()
    #: Whether every one of the four wheel ends must be accounted for.
    wheels_complete: bool = False


#: The rules, keyed by the category name a file states in ``assembly_kind``.
ASSEMBLY_RULES: dict[str, AssemblyRule] = {
    "generic_multibody": AssemblyRule(kind="generic_multibody", role_counts={}),
    "suspension_axle": AssemblyRule(
        kind="suspension_axle",
        role_counts={"suspension": 1},
        at_most={"steering": 1, "chassis": 1, "wheel": 1},
        forbidden_roles=frozenset({"brake", "drive"}),
    ),
    "full_vehicle": AssemblyRule(
        kind="full_vehicle",
        # A file declares how many axles it has, and "two suspensions" written here
        # would refuse a three-axle truck before the document was even read.  What
        # stays fixed is the *shape*: one body, one steering, one brake, one drive,
        # and at least one suspension -- see `min_counts` and the uniqueness check.
        role_counts={"chassis": 1, "steering": 1, "brake": 1, "drive": 1},
        min_counts={"suspension": 1},
        at_most={},
        required_placements=frozenset(),
        forbidden_roles=frozenset(),
        wheels_complete=True,
    ),
}

#: The four wheel ends a full vehicle must account for.
WHEEL_ENDS: tuple[str, ...] = ("front_left", "front_right", "rear_left", "rear_right")


def rule_for_assembly(kind: str) -> AssemblyRule:
    """Return the rule for a file's ``assembly_kind``, naming unknown ones."""
    try:
        return ASSEMBLY_RULES[kind]
    except KeyError as exc:
        known = ", ".join(sorted(ASSEMBLY_RULES))
        raise RuleViolation(
            f"unknown assembly kind {kind!r}; the known kinds are {known}"
        ) from exc


def check_forbidden_roles(kind: str, roles: Iterable[str]) -> None:
    """
    Refuse a role the category must not carry, before anything else is judged.

    Asked first because "an axle has no brake" is true whatever else the file says:
    reporting a missing chassis instead would name the wrong repair, and the role
    that is actually forbidden would stay in the file.
    """
    rule = rule_for_assembly(kind)
    forbidden = sorted(set(str(role) for role in roles) & rule.forbidden_roles)
    if forbidden:
        raise RuleViolation(
            f"an assembly of kind {kind!r} forbids {forbidden}: the bench loads the "
            "axle through wheels it supplies, and a brake or drive belongs to a vehicle"
        )


def check_assembly_shape(
    kind: str, assignments: Iterable[tuple[str, str]]
) -> None:
    """
    Raise unless a file-driven assembly has the shape its category fixes.

    Asked after every reference has been checked against the file it names: a count
    taken over assignments that do not match their files would report a number
    rather than the mistake behind it.
    """
    rule = rule_for_assembly(kind)
    pairs = [(str(role), str(placement)) for role, placement in assignments]
    roles = [role for role, _placement in pairs]
    if kind == "generic_multibody":
        return

    for role, placement in sorted(rule.required_placements):
        if (role, placement) not in pairs:
            raise RuleViolation(
                f"an assembly of kind {kind!r} requires one {placement} {role}; found "
                f"{sorted(pairs)}"
            )

    for role, floor in rule.min_counts.items():
        found = roles.count(role)
        if found < floor:
            raise RuleViolation(
                f"an assembly of kind {kind!r} requires at least {_word(floor)} "
                f"{role} subsystem(s), found {found}"
            )

    # One subsystem per placement: two suspensions at the same placement is a file
    # that names the same axle twice, and the assembly would carry it twice without
    # any reader being able to tell which one a wheel came from.  This is the check
    # that replaces "there are exactly two suspensions, at front and rear".
    repeated: dict[tuple[str, str], int] = {}
    for pair in pairs:
        repeated[pair] = repeated.get(pair, 0) + 1
    twice = sorted(f"{role} at {placement}" for pair, count in repeated.items()
                   if count > 1 and pair[0] == "suspension" for role, placement in [pair])
    if twice:
        raise RuleViolation(
            f"an assembly of kind {kind!r} places {twice} more than once; one axle "
            "is one placement, so a file that names the same one twice does not say "
            "which suspension a wheel came from"
        )

    # The rule's own order, not an alphabetical one: the first role the rule
    # names is the first thing a missing assembly should be told about.
    for role, expected in rule.role_counts.items():
        found = roles.count(role)
        if found != expected:
            raise RuleViolation(
                f"an assembly of kind {kind!r} requires exactly {_word(expected)} "
                f"{role} subsystem(s), found {found}"
            )

    for role, ceiling in rule.at_most.items():
        found = roles.count(role)
        if found > ceiling:
            raise RuleViolation(
                f"an assembly of kind {kind!r} carries at most {_word(ceiling)} {role} "
                f"subsystem(s), found {found}"
            )

    if rule.wheels_complete:
        # The wheel ends come from the file, not from a fixed four-corner list: a
        # three-axle truck and a single-wheel bench are both vehicles, and the
        # The wheel ends come from the file, not from a fixed four-corner list: a
        # three-axle truck and a single-wheel bench are both vehicles, and the question
        # a reader can answer is "is every wheel I declared carried by a suspension I
        # declared?" -- which is what this checks.  A missing axle is then a wheel whose
        # placement no suspension accounts for.
        #
        # Both sides answer about the *same* set of ends: a wheel declared at ``any``
        # stands for every end the assembly declares, and so does a suspension declared
        # at ``any``.  Reading one side as "every declared end" and the other as the
        # four corners of a four-wheel car is what would make a three-axle file look
        # like it had unclaimed wheels.
        stated = {
            placement
            for role, placement in pairs
            if role in {"suspension", "wheel"} and placement != "any"
        }
        ends: set[str] = set()
        for placement in stated:
            ends.update(_covered_corners(placement))
        declared = {
            placement
            for role, placement in pairs
            if role == "wheel"
        }
        if not declared:
            raise RuleViolation(
                f"an assembly of kind {kind!r} declares no wheel subsystem; the "
                "wheels are what the assembly's load paths end at"
            )
        wide = any(placement == "any" for role, placement in pairs if role == "wheel")
        carried: set[str] = set()
        for role, placement in pairs:
            if role != "suspension":
                continue
            carried.update(ends if placement == "any" else _covered_corners(placement))
        unclaimed = sorted(
            end for end in (ends if wide else declared) if end not in carried
        )
        if unclaimed:
            raise RuleViolation(
                f"an assembly of kind {kind!r} declares wheel end(s) {unclaimed} that "
                f"no suspension accounts for; the suspensions are at "
                f"{sorted(carried) or '(none)'}"
            )


#: How a small count reads in a message.  Spelling it out is what makes the sentence
#: an instruction -- "requires exactly one chassis" -- rather than a table row.
_COUNT_WORDS: dict[int, str] = {1: "one", 2: "two", 3: "three", 4: "four"}


def _word(count: int) -> str:
    """Return a small count as a word, and anything larger as digits."""
    return _COUNT_WORDS.get(count, str(count))


def check_assembly_roles(
    kind: str, assignments: Iterable[tuple[str, str]]
) -> None:
    """Refuse an assembly that breaks either the forbidden-role check or the shape."""
    pairs = [(str(role), str(placement)) for role, placement in assignments]
    check_forbidden_roles(kind, [role for role, _placement in pairs])
    check_assembly_shape(kind, pairs)


def _covered_corners(placement: str) -> set[str]:
    """
    Return the wheel ends one placement accounts for.

    Called by the shape check for every declared suspension, so it answers only for
    placements that name an axle or an end -- the ``any`` spelling is resolved by the
    caller, which is the only place that knows every end the assembly declares.
    """
    return _ends_of(placement)


def _ends_of(placement: str) -> set[str]:
    """
    Return the wheel ends a placement name covers.

    A placement that already names an end (``front_left``, ``middle_right``) covers
    exactly that one; any other placement covers its own left and right.  The test is
    the ``_left``/``_right`` suffix rather than membership in :data:`WHEEL_ENDS`,
    because that constant lists the four ends of a *four-corner* vehicle, and a middle
    axle's ends are real without appearing in it.
    """
    if placement.endswith(("_left", "_right")):
        return {placement}
    return {f"{placement}_left", f"{placement}_right"}
