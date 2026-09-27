"""
One view of a model, whatever shape it arrived in.

A contract compiler needs the same facts every time -- bodies with their mass
properties, body-local points, joints and the built force elements -- and two shapes
carry them:

* :class:`~suspension_multibody.subsystems.runtime.SubsystemRuntime` -- what the
  composition layer produces: bodies, both column sets (K's ``ideal_constraints``
  and C's ``constraints``), the built element objects and the points, all keyed the
  way a document needs them;
* :class:`~suspension_multibody.modeling.assembly.SimulationAssembly` -- the SI
  composition, which carries bodies, points, joints, tires and ports by entity
  id and is what the global rules were checked against.

Rather than teach every emitter about both, this module normalises either into
one :class:`ModelView`.  The *shape* is the only thing that varies; the facts do
not, and a third spelling added later is another constructor here rather than
another branch in an emitter.

The view keeps the physical assembly it was read from.  That is load-bearing:
the family document emitters take an assembly (they need masses, the K/C
columns and the built elements to write a contract), so a view built from a
composition -- which is identity and ports -- has to carry the build along
rather than making the emitter obtain it a second way.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any, Literal

from ..modeling.assembly import SimulationAssembly

__all__ = [
    "MM",
    "ModelView",
    "ViewError",
    "view_of",
]

#: Millimetres per metre.  The K/C contract document is in mm and the SI model
#: in metres, so the readers below convert in one place instead of at every use
#: -- which is the mistake that turns a 10 mm travel into ten metres.
MM = 1000.0


class ViewError(ValueError):
    """A model cannot be read as a compile view."""


@dataclass(frozen=True)
class ModelView:
    """
    The facts a contract compiler needs, normalised.

    ``physical`` is the assembly a family's document emitter is handed, and
    ``source`` is the object the view was built from.  They differ only when the
    source was a composition: then ``source`` is the composition (which owns the
    fingerprint and the provenance) and ``physical`` is the build inside it
    (which owns the entities).  Asking each level for what it owns is what keeps
    a document and a fingerprint from disagreeing.
    """

    #: The name the model was built under.
    name: str
    #: The active physical connection set.
    mode: Literal["K", "C"]
    #: Bodies by name, in emission order.
    bodies: Mapping[str, Any]
    #: Body-local points, keyed ``(body, label)``, in the source's own units.
    points: Mapping[tuple[str, str], Any]
    #: The joint set ``mode`` selects.
    joints: tuple[Any, ...]
    #: The other column, kept because a study may ask for either.
    inactive_joints: tuple[Any, ...] = ()
    #: The C column's bushings, in emission order.
    bushings: tuple[Any, ...] = ()
    #: Spring, damper, bump-stop and anti-roll-bar elements, in emission order.
    elements: tuple[Any, ...] = ()
    #: Vertical tire declarations, in emission order.
    tires: tuple[Any, ...] = ()
    #: The structural fingerprint of the composition, when there is one.
    fingerprint: str = ""
    #: Provenance per body, when the source records it (A5's tracing).
    provenance: Mapping[str, Any] = field(default_factory=dict)
    #: The assembly a family emitter authors its document from.
    physical: Any = None
    #: The object the view was built from.
    source: Any = None

    def joint_types(self) -> tuple[str, ...]:
        """Return the class name of each joint, in order."""
        return tuple(type(joint).__name__ for joint in self.joints)


def view_of(source: Any) -> ModelView:
    """
    Return the compile view of one assembly.

    Two shapes reach here and there used to be three readers: a composition (which
    carries identity, ports and a fingerprint) and the runtime face itself.  The
    runtime reader serves both -- a composition is read *through* its runtime -- so
    the second reader for a hand-built assembly is gone along with that assembly.
    """
    from ..subsystems.runtime import SubsystemRuntime

    if isinstance(source, SimulationAssembly):
        return _from_simulation_assembly(source)
    if isinstance(source, SubsystemRuntime):
        return _view_from_runtime(source)
    raise ViewError(
        "a compile view is built from a SimulationAssembly or a SubsystemRuntime, "
        f"got {type(source).__name__}"
    )


def _identity_of(assembly: Any) -> Mapping[str, Any]:
    """Return the body provenance a composed assembly records, if any."""
    from ..modeling.assembly import Assembly

    if not isinstance(assembly, Assembly):
        return {}
    found: dict[str, Any] = {}
    for child in assembly.walk():
        provenance = child.provenance
        if provenance is None:
            continue
        for name in child.fragment.bodies:
            found[name] = provenance
    return found


def _from_simulation_assembly(source: SimulationAssembly) -> ModelView:
    """
    Read the composition, taking the entities from the runtime it carries.

    A composition is *identity and ports*; the objects a document needs come from
    the runtime face the composition was built with.  That face used to be a
    hand-built assembly, which meant a composition could not produce a document
    without one -- the dependency this reader removes.  Now the runtime *is* the
    only shape there is, so this reads it and reports the composition's own
    fingerprint beside it.
    """
    inner = source.assembly
    runtime = inner.physical
    if runtime is None:
        raise ViewError(
            "this simulation assembly carries no runtime to read; a composition is "
            "identity and ports, so the runtime it was composed from has to travel "
            "with it"
        )
    return _view_from_runtime(runtime, source)


def _view_from_runtime(runtime: Any, source: Any = None) -> ModelView:
    """
    Read one runtime face into the view a compiler consumes.

    ``source`` is the object the runtime came from, when there is one: a
    ``SimulationAssembly`` carries the fingerprint and the provenance, and a view
    built from it must report those.  A caller that hands the runtime over
    directly has neither, and the view reports none rather than inventing one --
    an empty fingerprint already means "this object has no structural identity",
    which is exactly true.
    """
    from ..cases.kc_quasi_static.convert import collapse_spherical_pairs

    mode = runtime.mode
    ideal = tuple(runtime.ideal_constraints)
    compliant = tuple(runtime.constraints)
    active = collapse_spherical_pairs(ideal) if mode == "K" else compliant
    inactive = compliant if mode == "K" else collapse_spherical_pairs(ideal)

    tires: list[Any] = []
    elements: list[Any] = []
    for element in runtime.elements:
        if type(element).__name__ == "VerticalTireElement":
            tires.append(element)
        else:
            elements.append(element)

    inner = getattr(source, "assembly", None)
    return ModelView(
        name=getattr(source, "name", "") or getattr(runtime, "name", "") or "",
        mode=mode,
        bodies=dict(runtime.bodies),
        points=dict(runtime.points),
        joints=tuple(active),
        inactive_joints=tuple(inactive),
        bushings=tuple(runtime.bushings),
        elements=tuple(elements),
        tires=tuple(tires),
        fingerprint=getattr(source, "fingerprint", "") or "",
        provenance=_identity_of(inner) if inner is not None else {},
        physical=runtime,
        source=source if source is not None else runtime,
    )


def _from_runtime(runtime: Any, source: SimulationAssembly) -> ModelView:
    """Read a composition's runtime face, keeping the composition's identity."""
    return _view_from_runtime(runtime, source)
