"""
Composing an assembly with a rig, and shrinking the rig to fit.

A run is one assembly plus one rig, and the two are registered independently so a
new combination is a registration rather than a code path.  This module resolves
that pair, and the resolution is where the interface question is settled:

* the **assembly reports what it offers** (`AssemblyCapabilities.drive_coordinates`),
  and the only judgement applied is `coordinate in capabilities.drive_coordinates`.
  Probing for a body named `rack` is what turns "this assembly has no steering"
  into a `StopIteration` deep inside the case layer instead of a decision here;
* the **rig's request shrinks** to what the assembly offers.  An assembly without
  steering simply has no rack axis: the drive disappears, the grid loses that
  dimension, and the rig runs.  Filling the axis with zeros would be worse than
  failing, because the run would then *look* steered and not be.

An assembly that cannot offer something the rig needs for its own sake is a
different matter: a rig that has no wheel coordinate at all cannot run, and saying
so is the point of `require`.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..subsystems.capabilities import AssemblyCapabilities
from .rig import DriveSpec, RigError, RigSpec

__all__ = [
    "Composition",
    "CompositionError",
    "compose",
    "resolve_combination",
]


class CompositionError(RigError):
    """An assembly and a rig cannot be combined."""


@dataclass(frozen=True)
class Composition:
    """
    One assembly on one rig, with the rig's interface already shrunk to fit.

    `drives` and `outputs` are what actually happened, not what the rig asked for:
    a caller reads them to learn the shape of the run, so keeping the request here
    would make every downstream consumer re-derive the shrinkage and get it subtly
    different.
    """

    rig: RigSpec
    capabilities: AssemblyCapabilities
    #: The drive coordinates that survived the shrink, in the rig's order.
    drives: tuple[DriveSpec, ...]
    #: The coordinates the rig asked for but the assembly cannot offer.
    dropped: tuple[str, ...] = ()
    #: The rig's own minimum-unit outputs.
    outputs: tuple[str, ...] = ()

    def drives_coordinate(self, coordinate: str) -> bool:
        """Return whether the resolved run drives a coordinate."""
        return coordinate in tuple(drive.coordinate for drive in self.drives)

    @property
    def shrunk(self) -> bool:
        """Return whether the assembly's capabilities removed anything."""
        return bool(self.dropped)


def compose(rig: RigSpec, capabilities: AssemblyCapabilities) -> Composition:
    """
    Shrink `rig`'s interface to what `capabilities` offers.

    Only the coordinates the *assembly* has to provide are subject to the check.
    A road height or a steering-wheel angle is the rig's own input -- it perturbs
    the assembly from outside -- so requiring the assembly to declare it would make
    every dynamics bench unusable on every model, which is the opposite of adapting.

    A drive the assembly cannot offer is dropped, and one dropped coordinate takes
    its couplings with it: a shorthand whose partner is gone would drive half a
    pair.
    """
    offered = capabilities.drive_coordinates
    retained: list[DriveSpec] = []
    dropped: list[str] = []
    for drive in rig.drives:
        if drive.from_assembly and drive.coordinate not in offered:
            dropped.append(drive.coordinate)
            continue
        linked = tuple(
            name
            for name in drive.coupled_with
            if name in offered or not drive.from_assembly
        )
        if drive.coupled_with and not linked:
            # The shorthand drives a set; with the set gone it is no longer the
            # same motion, so it goes too rather than driving one side alone.
            dropped.append(drive.coordinate)
            continue
        retained.append(
            DriveSpec(
                coordinate=drive.coordinate,
                kind=drive.kind,
                coupled_with=linked,
                from_assembly=drive.from_assembly,
            )
        )
    # A rig whose whole point is to drive something now absent cannot run: the
    # quasi-static bench moves wheel travel and nothing else.  Saying so here is
    # what keeps "the rig adapted" from meaning "the rig silently did nothing".
    assembly_drives = [drive for drive in rig.drives if drive.from_assembly]
    if assembly_drives and not retained:
        offered_names = ", ".join(sorted(offered)) or "(none)"
        raise CompositionError(
            f"rig {rig.name!r} drives only {list(rig.coordinate_names())}, and this "
            f"assembly offers none of them; it offers {offered_names}"
        )
    return Composition(
        rig=rig,
        capabilities=capabilities,
        drives=tuple(retained),
        dropped=tuple(dropped),
        outputs=rig.outputs,
    )


#: Every registered assembly name the package can compose, in a stable order.
ASSEMBLIES: tuple[str, ...] = ("axle", "vehicle")


def resolve_combination(
    assembly: str, rig: str, capabilities: AssemblyCapabilities
) -> Composition:
    """
    Resolve one `(assembly, rig)` pair, refusing a pair that cannot exist.

    The check is deliberately about *registration*, not about capability: a rig
    belongs to one kind of assembly (a four-post bench drives a vehicle, not a bare
    axle), while capability is what the shrink handles.  Conflating the two would
    turn "this bench does not fit this kind of thing" into "your axle is missing a
    part", which sends the reader to the wrong place.
    """
    from .rig import get_rig

    name = str(assembly).strip().lower()
    if name not in ASSEMBLIES:
        known = ", ".join(ASSEMBLIES)
        raise CompositionError(
            f"unknown assembly {assembly!r}; the registered assemblies are {known}"
        )
    rig_spec = get_rig(rig)
    _check_registered_pair(name, rig_spec)
    return compose(rig_spec, capabilities)


def check_assembly(
    assembly: str, rig: str, capabilities: AssemblyCapabilities | None
) -> None:
    """
    Raise unless `rig` can run an assembly reporting these capabilities.

    This lives here rather than in each family because the rule is the rig's own:
    a bench that belongs to another kind of assembly is a registration error, and
    a bench the assembly cannot supply a drive for is a capability error.  Both
    arrive as `CompositionError`, which is a `ValueError`, so a family
    preparation can let it through as its own refusal.

    A capability-less assembly is accepted rather than refused: it was assembled
    outside the subsystem path, and every reader of it -- `compose` included --
    falls back to the full coordinate set.  Refusing it here would break that
    documented fallback instead of catching anything.

    Naming one bench at preparation time is the point: the alternative is a run
    that builds a grid against a bench nobody asked about and reports it under
    the wrong name.
    """
    if capabilities is None:
        return
    resolve_combination(assembly, rig, capabilities)


#: Which rigs belong to which assembly.  A rig outside its own kind is a
#: registration error, not a capability shortfall.
_RIG_ASSEMBLIES: dict[str, str] = {
    "kc_quasi_static": "axle",
    "axle_dynamic": "axle",
    "vehicle_kc": "vehicle",
    "vehicle_dynamic": "vehicle",
    "handling": "vehicle",
    "ride_four_post": "vehicle",
    "ride_random_road": "vehicle",
}


def _check_registered_pair(assembly: str, rig: RigSpec) -> None:
    """Refuse a rig that is not registered for this kind of assembly."""
    owner = _RIG_ASSEMBLIES[rig.name]
    if owner != assembly:
        raise CompositionError(
            f"rig {rig.name!r} is registered for the {owner!r} assembly, not "
            f"{assembly!r}; a rig belongs to one kind of assembly"
        )


def combinations() -> tuple[tuple[str, str], ...]:
    """Return every registered `(assembly, rig)` pair, in a stable order."""
    return tuple(
        (owner, name) for name, owner in sorted(_RIG_ASSEMBLIES.items())
    )
