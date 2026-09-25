"""
Port matching: which offered port satisfies which requirement, and why.

The rule order is fixed and stated once, because the interesting failure is the
*ambiguous* case:

1. **An explicit mapping wins.**  If the caller said "this requirement meets that
   port", that is the answer, and nothing is inferred.  A user who knows their
   model should not have to argue with a heuristic.
2. **Otherwise filter on role, capabilities and labels.**  A requirement is
   satisfied by a port whose role matches, which offers every capability the
   requirement names, and whose labels agree exactly.
3. **Zero candidates** is a failure if the requirement is required, and a
   recorded disappearance if it is optional.
4. **More than one candidate is an ambiguity, not a choice.**  The list is
   reported so the caller can add an explicit mapping.  Picking the closest name,
   or the nearest one geometrically, is what produced rigs that bind to whatever
   happened to be there -- and a run that looks steered but is not.

Every resolution is recorded, including the ones that failed, so a binding can be
audited afterwards rather than re-derived.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field

from ..modeling.ports import PortRequirement, PortSpec

__all__ = [
    "AmbiguousBindingError",
    "Binding",
    "BindingError",
    "UnmetRequirementError",
    "match_requirements",
]


class BindingError(ValueError):
    """A requirement could not be bound and the caller must decide."""


class AmbiguousBindingError(BindingError):
    """
    Several ports satisfy one requirement.

    Carries the candidate ids, because the fix is to add an explicit mapping and
    the caller needs to know what to name.
    """

    def __init__(self, message: str, candidates: tuple[str, ...]) -> None:
        super().__init__(message)
        self.candidates = candidates


class UnmetRequirementError(BindingError):
    """A required port had no candidate at all."""


@dataclass(frozen=True)
class Binding:
    """
    One resolved requirement: which port satisfied it, and how it was chosen.

    ``explicit`` records whether the caller named it.  Keeping that flag means a
    later reader can tell a deliberate binding from an inferred one, which is the
    difference between "the model says so" and "the matcher guessed and got it
    right this time".
    """

    requirement: PortRequirement
    #: The port ids that satisfied it, in the order they were selected.
    port_ids: tuple[str, ...]
    explicit: bool = False

    def resolved(self) -> bool:
        """Return whether this requirement found its ports."""
        return bool(self.port_ids)


@dataclass(frozen=True)
class MatchReport:
    """The outcome of matching one instance's requirements against its neighbours."""

    bindings: tuple[Binding, ...] = ()
    #: Requirements that found nothing and were declared optional.
    disappeared: tuple[str, ...] = ()
    #: Outputs that went away with an optional branch.
    dropped_outputs: tuple[str, ...] = ()
    #: Human-readable trace of every decision, for the audit.
    trace: tuple[str, ...] = field(default=())

    def binding_for(self, role: str) -> Binding | None:
        """Return the binding for a requirement role, if it was resolved."""
        for binding in self.bindings:
            if binding.requirement.role == role:
                return binding
        return None


def match_requirements(
    requirements: Iterable[PortRequirement],
    candidates: Mapping[str, PortSpec],
    *,
    explicit: Mapping[str, str] | None = None,
) -> MatchReport:
    """
    Bind each requirement to a port from ``candidates``.

    ``candidates`` is keyed by port id (the rendered :class:`EntityId`), and
    ``explicit`` maps a requirement's role to the port id the caller chose.
    A caller-supplied port id that does not exist is an error rather than a
    silent fallback to inference: the mapping was wrong, and quietly matching
    something else would hide it.
    """
    chosen_explicitly = dict(explicit or {})
    bindings: list[Binding] = []
    disappeared: list[str] = []
    dropped: list[str] = []
    trace: list[str] = []

    for requirement in requirements:
        named = chosen_explicitly.get(requirement.role)
        if named is not None:
            port = candidates.get(named)
            if port is None:
                raise BindingError(
                    f"requirement {requirement.role!r} was mapped to port {named!r}, "
                    f"which does not exist; the candidates are "
                    f"{sorted(candidates) or '(none)'}"
                )
            trace.append(f"{requirement.role}: explicit -> {named}")
            bindings.append(
                Binding(requirement=requirement, port_ids=(named,), explicit=True)
            )
            continue

        matches = sorted(
            port_id
            for port_id, port in candidates.items()
            if requirement.accepts(port)
        )
        if not matches:
            if requirement.required:
                raise UnmetRequirementError(
                    f"required port {requirement.role!r} has no candidate; the "
                    f"offered ports are {sorted(candidates) or '(none)'}"
                )
            disappeared.append(requirement.role)
            dropped.extend(requirement.bound_outputs)
            trace.append(
                f"{requirement.role}: optional, no candidate -> disappears with "
                f"{list(requirement.bound_outputs)}"
            )
            continue

        if len(matches) > 1 and requirement.count == 1:
            raise AmbiguousBindingError(
                f"port {requirement.role!r} matches {len(matches)} candidates "
                f"{matches}; add an explicit mapping to say which one is meant",
                tuple(matches),
            )

        wanted = matches[: requirement.count]
        trace.append(f"{requirement.role}: inferred -> {wanted}")
        bindings.append(Binding(requirement=requirement, port_ids=tuple(wanted)))

    unused = sorted(set(chosen_explicitly) - {r.role for r in requirements})
    if unused:
        raise BindingError(
            f"explicit mapping names requirement(s) {unused}, which this instance "
            "does not declare"
        )

    return MatchReport(
        bindings=tuple(bindings),
        disappeared=tuple(disappeared),
        dropped_outputs=tuple(dict.fromkeys(dropped)),
        trace=tuple(trace),
    )
