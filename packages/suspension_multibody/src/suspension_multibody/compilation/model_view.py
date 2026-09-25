"""
One view of a model, whatever shape it arrived in.

A contract compiler needs the same facts every time -- bodies with their mass
properties, body-local points, joints and the built force elements -- and there
are two live spellings of them in the package:

* :class:`~suspension_multibody.preparation.assembly.front_axle.FrontAxleAssembly`
  -- the assembly the historical build produces, millimetres, carrying masses,
  both column sets (K's ``ideal_constraints`` and C's ``constraints``) and the
  built element objects;
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
from dataclasses import dataclass, field, replace
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
    """Return the compile view of one assembly, in whichever shape it arrived."""
    if isinstance(source, SimulationAssembly):
        return _from_simulation_assembly(source)
    return _from_front_axle(source)


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
    Read the composition, delegating the physical facts to the build it kept.

    A composition is *identity and ports*: it is what the global rules were
    checked against and what a fingerprint is taken of.  The physical facts a
    document needs -- masses, the built elements, the column the mode selects --
    belong to the build it was composed from, which is why the composition keeps
    that build rather than re-deriving it.  Asking each level for what it owns is
    what keeps a document and a fingerprint from disagreeing.
    """
    inner = source.assembly
    physical = inner.physical
    if physical is None:
        raise ViewError(
            "this simulation assembly carries no physical build to read; a "
            "composition is identity and ports, so the assembly it was composed "
            "from has to travel with it"
        )
    view = _from_front_axle(physical)
    return replace(
        view,
        fingerprint=source.fingerprint,
        provenance=_identity_of(inner),
        source=source,
    )


def _from_front_axle(source: Any) -> ModelView:
    """
    Read the historical build, selecting the column the mode carries.

    The mode selects the joint set here for the same reason the family emitter
    does it: K drives the wheel centres through the rigid kinematic set, so its
    joints are the collapsed ideal set, while C loads the compliant set.  The
    *other* column is kept on the view rather than dropped, so a caller can ask
    what a C assembly's ideal set would be without rebuilding it.
    """
    from ..preparation.assembly import FrontAxleAssembly

    if not isinstance(source, FrontAxleAssembly):
        raise ViewError(
            "a compile view is built from a FrontAxleAssembly or a "
            f"SimulationAssembly, got {type(source).__name__}"
        )

    from ..cases.kc_quasi_static.convert import collapse_spherical_pairs

    mode = source.mode
    ideal = tuple(source.ideal_constraints)
    compliant = tuple(source.constraints)
    active = collapse_spherical_pairs(ideal) if mode == "K" else compliant
    inactive = compliant if mode == "K" else collapse_spherical_pairs(ideal)

    tires: list[Any] = []
    elements: list[Any] = []
    for element in source.elements:
        if type(element).__name__ == "VerticalTireElement":
            tires.append(element)
        else:
            elements.append(element)

    return ModelView(
        name=getattr(source, "name", "") or "",
        mode=mode,
        bodies=dict(source.bodies),
        points=dict(source.points),
        joints=tuple(active),
        inactive_joints=tuple(inactive),
        bushings=tuple(source.bushings),
        elements=tuple(elements),
        tires=tuple(tires),
        physical=source,
        source=source,
    )
