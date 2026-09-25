"""
The simulation request: independent inputs, one compiled submission.

A request used to be a `(assembly, family)` pair, and a `rig` that had to be the
family spelled again.  Those are three different questions and the request now
says so:

* **assembly** -- which device under test (`axle`, `vehicle`);
* **rig** -- which test bench drives and measures it.  This is the dimension a
  caller actually chooses; the *family* follows from the bench, because a bench
  is a bench and the reading it takes is part of what it is;
* **family** -- which contract compiler and preparation route the run through.
  An internal compatibility key, and deliberately *not* required to equal the
  bench's name: the benches that ship kept the family names so existing callers
  did not have to move, and a newly authored bench names its family explicitly
  (`RigSpec.family`) instead of being forced to imitate one;
* **study** -- how the model is read (quasi-static or dynamic).  The bench also
  declares this, so the two must agree when both are given;
* **case** and **outputs** -- what is run and what is asked for.  They travel as
  values on the request rather than inside a family-owned mega-schema, which is
  what makes "the same assembly, two studies" expressible at all.

None of the five is derived from another except the family, which a bench
answers for itself.  Naming a bench alone is therefore enough, and naming a
family alone still works because the shipped benches are named after theirs.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

__all__ = ["CompiledSimulation", "SimulationRequest"]

#: The context keys a request may carry authored contract documents under.
#: Named here rather than in the preparation because the *request* is what a
#: caller fills in, and a key invented at the consumer would be discovered only
#: by reading that consumer's source.
DOCUMENT_CONTEXT_KEYS: tuple[str, ...] = (
    "model_document",
    "case_document",
    "model_document_pair",
    "model_payload",
    "case_payload",
)


@dataclass(frozen=True)
class SimulationRequest:
    """
    One user-level simulation request before contract compilation.

    ``model`` and ``case`` deliberately remain family-owned objects.  The
    request standardises orchestration metadata without flattening each case's
    physical vocabulary into one mega-schema.
    """

    assembly: str
    #: The test bench this run is on.  This is the dimension the routing is
    #: built on: a caller names the bench it wants and the family follows.
    rig: str = ""
    #: The case family: the contract compiler/preparation key.  May be left
    #: empty when `rig` is named, and is then filled in from it -- that is how
    #: "the bench picks the reading" is expressed rather than promised.  It is
    #: *not* required to equal `rig` for a bench that declares its own family.
    family: str = ""
    #: How the model is read.  Optional: the bench declares one, and a bench
    #: that declares none fits either.  Given here it must agree with the
    #: bench's, because the study changes the solve and a mismatch means the
    #: caller and the bench disagree about what is being run.
    study: str = ""
    model: Any = None
    case: Any = None
    #: What the run is asked to produce, when a caller wants to be explicit.
    #: Empty means "whatever the assembly and the bench declare", which is the
    #: merged declaration set the outputs layer already computes.
    outputs: tuple[str, ...] = ()
    name: str | None = None
    request_kind: str | None = None
    context: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        assembly = str(self.assembly).strip().lower()
        rig = str(self.rig).strip().lower()
        family = str(self.family).strip().lower()
        study = str(self.study).strip().lower()

        if not assembly:
            raise ValueError("simulation request assembly must not be empty")
        if not family and not rig:
            raise ValueError(
                "simulation request needs a family or a rig; the rig is the "
                "routing dimension and the family follows from it"
            )

        # The rig is the routing dimension and the family follows from it, so
        # naming either one is enough.  A family that is not a registered bench
        # is left alone rather than refused: the compiler and preparation
        # registries accept any key a caller registers, and a test or a
        # downstream product may register its own.  Rejecting it here would move
        # their error to the request and make the registries unextendable.
        if not family:
            family = rig
        elif not rig:
            rig = family
        # A rig and a family that *disagree* are no longer a caller error: a
        # bench may declare a family other than its own name, and that is the
        # whole point of separating the two axes.  What is checked is the
        # narrower, real question -- does this bench route there -- and it is
        # asked where the bench registry is reachable, in `resolve_route`
        # below, so the request itself stays free of a registry import.

        request_kind = (
            family if self.request_kind is None else str(self.request_kind).strip().lower()
        )
        if not request_kind:
            raise ValueError("simulation request kind must not be empty")

        object.__setattr__(self, "assembly", assembly)
        object.__setattr__(self, "rig", rig)
        object.__setattr__(self, "family", family)
        object.__setattr__(self, "study", study)
        object.__setattr__(self, "request_kind", request_kind)
        object.__setattr__(self, "outputs", tuple(self.outputs))
        object.__setattr__(self, "context", dict(self.context))

    @property
    def kind(self) -> str:
        """Return the normalized request kind used for compiler metadata."""
        assert self.request_kind is not None
        return self.request_kind

    def with_route(self, *, rig: str, family: str, study: str = "") -> SimulationRequest:
        """Return this request re-routed to `rig`/`family`, keeping everything else."""
        from dataclasses import replace

        return replace(self, rig=rig, family=family, study=study, request_kind=None)

    def resolved_study(self, example_default: str) -> str:
        """
        Return the study this request asks for, resolving the bench's declaration.

        The bench answers when the request does not, and a request that does must
        agree with it: two answers would make the run's reading a coin toss.
        """
        from ..rigs.rig import get_rig

        declared = ""
        if self.rig:
            try:
                spec = get_rig(self.rig)
            except Exception:  # noqa: BLE001 - an unregistered rig has no say
                declared = ""
            else:
                declared = str(spec.study or "")
        if self.study and declared and self.study != declared:
            raise ValueError(
                f"simulation request asks for study {self.study!r} but rig "
                f"{self.rig!r} declares {declared!r}; the bench settles how it "
                "reads the model, so name the bench that takes the reading you want"
            )
        return self.study or declared or example_default


@dataclass(frozen=True)
class CompiledSimulation:
    """A fully materialised native contract submission."""

    request: SimulationRequest
    model_document: dict[str, Any]
    case_document: dict[str, Any]
    model_payload: bytes
    case_payload: bytes
    layout: dict[str, Any] = field(default_factory=dict)
    metadata: dict[str, Any] = field(default_factory=dict)

    @property
    def assembly(self) -> str:
        return self.request.assembly

    @property
    def rig(self) -> str:
        return self.request.rig

    @property
    def family(self) -> str:
        return self.request.family

    @property
    def request_kind(self) -> str:
        return self.request.kind

    @property
    def name(self) -> str | None:
        return self.request.name

    def documents(self) -> tuple[dict[str, Any], dict[str, Any]]:
        """Return model and case documents in native submission order."""
        return self.model_document, self.case_document

    def payloads(self) -> tuple[bytes, bytes]:
        """Return packed model and case payloads in native submission order."""
        return self.model_payload, self.case_payload
