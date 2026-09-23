"""
What a run declares it produces, before anything computes it.

An output has an owner.  An assembly declares the outputs its subsystems produce;
a rig declares the ones its own driving and instrumentation add.  Neither knows
the other's list, and a run is only well defined once the two have been merged.

"Minimum-unit" is the load-bearing phrase: these are the *smallest* things a run
produces -- a wheel centre, a tire force, a rack displacement -- not the numbers
a report wants.  Everything a report wants (camber, toe, KC gradients, stability
indices) is a derived output computed from these, which is what makes a new
report metric a declaration rather than a new code path.

Declarations are data, so they serialise.  A rig author and an assembly author
have to be able to compare two lists on paper before a run exists.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any, Literal

__all__ = [
    "DOMAINS",
    "OUTPUT_NAME_PATTERN",
    "DeclarationSet",
    "DeclarationError",
    "OutputDeclaration",
    "merge_declarations",
]

#: Which side of a run declared an output.  `assembly` and `rig` are the two
#: owners that get merged; `derived` marks a computed output so a merged set can
#: never be mistaken for a produced one.  `merged` is what merging produces, and
#: it is the only domain whose members may carry differing domains of their own --
#: that is what a merge *is*.
Domain = Literal["assembly", "rig", "derived", "merged"]

DOMAINS: tuple[str, ...] = ("assembly", "rig", "derived", "merged")

#: Output names are the contract between two independently written sides, so
#: they stay flat, lowercase and dotted-namespace free.
OUTPUT_NAME_PATTERN = r"^[a-z][a-z0-9_]*$"


class DeclarationError(ValueError):
    """An output declaration is malformed, or two of them disagree."""


@dataclass(frozen=True)
class OutputDeclaration:
    """
    One minimum-unit output, described well enough to be checked before a run.

    The fields are the five a report needs in order to refuse a wrong input
    rather than silently produce a wrong number: what it is called, what it is
    measured in, what physical kind it is, where it comes from, and what shape
    its value has.  `description` is for the human reading the merged list.

    `shape` is the value's shape with `None` standing for "a sample axis whose
    length the run decides".  An empty tuple is a scalar.
    """

    name: str
    unit: str
    #: Physical dimension, e.g. "length", "force", "angle", "time", "count".
    dimension: str
    #: Which side declares it: see `Domain`.
    domain: str = "assembly"
    #: Value shape; `None` entries are axis lengths only the run knows.
    shape: tuple[int | None, ...] = ()
    description: str = ""

    def __post_init__(self) -> None:
        if not self.name:
            raise DeclarationError("an output declaration must carry a name")
        if self.domain not in DOMAINS:
            raise DeclarationError(
                f"output {self.name!r} has unknown domain {self.domain!r}; "
                f"domains are {list(DOMAINS)}"
            )
        if not self.unit:
            raise DeclarationError(
                f"output {self.name!r} must declare a unit "
                "(use '-' for a dimensionless value)"
            )
        if not self.dimension:
            raise DeclarationError(
                f"output {self.name!r} must declare a physical dimension"
            )
        for size in self.shape:
            if size is not None and size < 0:
                raise DeclarationError(
                    f"output {self.name!r} has a negative axis length {size}"
                )

    def to_json(self) -> dict[str, Any]:
        """Return a plain mapping, ready for JSON."""
        return {
            "name": self.name,
            "unit": self.unit,
            "dimension": self.dimension,
            "domain": self.domain,
            "shape": [None if size is None else int(size) for size in self.shape],
            "description": self.description,
        }
    @classmethod
    def from_json(cls, payload: dict[str, Any]) -> OutputDeclaration:
        """Rebuild a declaration from `to_json` output."""
        return cls(
            name=str(payload["name"]),
            unit=str(payload["unit"]),
            dimension=str(payload["dimension"]),
            domain=str(payload.get("domain", "assembly")),
            shape=tuple(
                None if size is None else int(size) for size in payload.get("shape", ())
            ),
            description=str(payload.get("description", "")),
        )


@dataclass(frozen=True)
class DeclarationSet:
    """
    One side's complete declaration list, in the order it wants them listed.

    Order is preserved because a merged list is read by people and compared in
    tests; a set would make the merge output unstable for no benefit.
    """

    domain: str
    outputs: tuple[OutputDeclaration, ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        if self.domain not in DOMAINS:
            raise DeclarationError(
                f"declaration set has unknown domain {self.domain!r}; "
                f"domains are {list(DOMAINS)}"
            )
        mismatched = [
            item.name
            for item in self.outputs
            if self.domain != "merged" and item.domain != self.domain
        ]
        if mismatched:
            raise DeclarationError(
                f"declaration set for domain {self.domain!r} carries output(s) "
                f"{mismatched} declared under another domain"
            )
        names = [item.name for item in self.outputs]
        duplicates = sorted({name for name in names if names.count(name) > 1})
        if duplicates:
            raise DeclarationError(
                f"declaration set for domain {self.domain!r} declares "
                f"{duplicates} more than once"
            )

    def names(self) -> tuple[str, ...]:
        """Return the declared names in declaration order."""
        return tuple(item.name for item in self.outputs)

    def get(self, name: str) -> OutputDeclaration:
        """Return one declaration by name, naming the unknown one if absent."""
        for item in self.outputs:
            if item.name == name:
                return item
        known = ", ".join(self.names()) or "(none)"
        raise DeclarationError(
            f"domain {self.domain!r} does not declare output {name!r}; "
            f"it declares {known}"
        )

    def to_json(self) -> dict[str, Any]:
        """Return a plain mapping, ready for JSON."""
        return {
            "domain": self.domain,
            "outputs": [item.to_json() for item in self.outputs],
        }

    def dumps(self) -> str:
        """Serialise the set to a JSON string."""
        return json.dumps(self.to_json(), sort_keys=True, indent=2)

    @classmethod
    def from_json(cls, payload: dict[str, Any]) -> DeclarationSet:
        """Rebuild a set from `to_json` output."""
        return cls(
            domain=str(payload["domain"]),
            outputs=tuple(
                OutputDeclaration.from_json(item)
                for item in payload.get("outputs", ())
            ),
        )

    @classmethod
    def loads(cls, text: str) -> DeclarationSet:
        """Rebuild a set from `dumps` output."""
        return cls.from_json(json.loads(text))


def merge_declarations(*sets: DeclarationSet) -> DeclarationSet:
    """
    Merge declaration sets into one, refusing a genuine disagreement.

    The same name from two sides is only a conflict when the two sides describe
    *different things*: a different unit, a different dimension, or a different
    shape.  A name two sides describe identically is one output with two
    producers, which is normal -- an assembly and a rig can both measure a wheel
    centre.  Refusing that would make the merge unusable; accepting a unit
    mismatch would put a millimetre and an inch in the same list.

    A derived declaration may never merge with a produced one, because a merged
    list is what a run is checked to have produced.
    """
    merged: dict[str, OutputDeclaration] = {}
    order: list[str] = []
    for declaration_set in sets:
        for item in declaration_set.outputs:
            existing = merged.get(item.name)
            if existing is None:
                merged[item.name] = item
                order.append(item.name)
                continue
            for field_name in ("unit", "dimension", "shape"):
                if getattr(existing, field_name) != getattr(item, field_name):
                    raise DeclarationError(
                        f"output {item.name!r} conflicts: "
                        f"{existing.domain} declares {field_name} "
                        f"{getattr(existing, field_name)!r}, "
                        f"{item.domain} declares {getattr(item, field_name)!r}"
                    )
            if (existing.domain == "derived") != (item.domain == "derived"):
                raise DeclarationError(
                    f"output {item.name!r} conflicts: {existing.domain} and "
                    f"{item.domain} cannot share a name, because one is computed "
                    "and the other is produced"
                )
    # The merged set says so in its own domain: its members keep the domain that
    # declared them (that is the evidence of who owns what), while the set as a
    # whole is the merge.  Anything else would either lose that evidence or
    # pretend one side had declared the other's outputs.
    if not sets:
        raise DeclarationError("merge_declarations needs at least one set")
    domain = "derived" if merged and all(
        item.domain == "derived" for item in merged.values()
    ) else "merged"
    return DeclarationSet(
        domain=domain,
        outputs=tuple(merged[name] for name in order),
    )
