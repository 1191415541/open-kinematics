"""
The low layer: model declarations, identity, ports and units.

Nothing in this package may import ``templates``, ``subsystems``, ``rigs``,
``connections``, ``preparation``, ``simulation``, ``kernel`` or ``report``.  A
declaration has to be usable without loading the authoring chain that assembles
it, which is the property this layer exists to provide and which
``tests/architecture/test_import_boundaries.py`` enforces by importing these
modules in a fresh process and inspecting what came with them.

The layer is split so that the *shape* of a model can be described before
anything knows how to build it:

``identity``
    stable :class:`EntityId` values, so a reference survives a rebuild.
``ports``
    what an instance offers and what it needs, as semantic ports rather than
    names to be matched by similarity.
``instance``
    :class:`ModelFragment` -- the entities one instance contributes, with the
    provenance needed to trace them back.
``assembly``
    :class:`Assembly` (the device under test) and
    :class:`SimulationAssembly` (that device joined to a rig).
``units``
    the single millimetre-to-metre boundary.
"""

from __future__ import annotations

from .assembly import Assembly, AssemblyError, NestedInstance, SimulationAssembly
from .identity import EntityId, qualified
from .instance import EntityConflictError, FragmentProvenance, ModelFragment
from .ports import (
    ChannelPort,
    GeometryPort,
    PortError,
    PortRequirement,
    PortSpec,
    link_counts_agree,
)
from .units import (
    KG_PER_TONNE,
    METRES_PER_MILLIMETRE,
    N_PER_KN,
    to_kilograms,
    to_metres,
    to_newtons,
)

__all__ = [
    "KG_PER_TONNE",
    "METRES_PER_MILLIMETRE",
    "N_PER_KN",
    "Assembly",
    "AssemblyError",
    "ChannelPort",
    "EntityConflictError",
    "EntityId",
    "FragmentProvenance",
    "GeometryPort",
    "ModelFragment",
    "NestedInstance",
    "PortError",
    "PortRequirement",
    "PortSpec",
    "SimulationAssembly",
    "link_counts_agree",
    "qualified",
    "to_kilograms",
    "to_metres",
    "to_newtons",
]
