"""
A fragment is immutable data that refuses a collision; an assembly keeps its
nested parts addressable.
"""

from __future__ import annotations

import dataclasses

import pytest

from suspension_multibody.modeling import (
    Assembly,
    AssemblyError,
    EntityConflictError,
    EntityId,
    FragmentProvenance,
    ModelFragment,
    NestedInstance,
    PortRequirement,
    SimulationAssembly,
)


def _body(name: str = "upper_arm") -> dict[str, object]:
    return {name: object()}


def test_fragment_is_frozen() -> None:
    """Mutating a fragment would invalidate every provenance record pointing at it."""
    fragment = ModelFragment(bodies=_body())
    with pytest.raises(dataclasses.FrozenInstanceError):
        fragment.instance = ("axle",)  # type: ignore[misc]


def test_merge_refuses_a_duplicate_body() -> None:
    """Last-write-wins is how a model nobody can explain gets built."""
    left = ModelFragment(bodies=_body("wheel"))
    right = ModelFragment(bodies=_body("wheel"))
    with pytest.raises(EntityConflictError, match="duplicate bodies"):
        left.merged_with(right)


def test_merge_joins_disjoint_fragments() -> None:
    left = ModelFragment(bodies=_body("upper_arm"))
    right = ModelFragment(bodies=_body("lower_arm"), joints={"ball": object()})
    merged = left.merged_with(right)
    assert set(merged.bodies) == {"upper_arm", "lower_arm"}
    assert set(merged.joints) == {"ball"}


def test_merge_refuses_different_mounting_paths() -> None:
    left = ModelFragment(bodies=_body("arm")).mounted(("axle",))
    right = ModelFragment(bodies=_body("other")).mounted(("vehicle",))
    with pytest.raises(EntityConflictError, match="different paths"):
        left.merged_with(right)


def test_a_fragment_reports_references_it_cannot_resolve_yet() -> None:
    """
    A fragment is a *partial* view, so an unresolved reference is a report, not a
    fault.

    Subtask 06 made this concrete: a suspension contributes points on the chassis
    and on the tie rods, which other contributions own.  Treating that as a
    dangling reference would reject every real composition, so the fragment says
    what is still open and the assembly closes it -- see
    ``modeling.assembly.SimulationAssembly``.
    """
    fragment = ModelFragment(bodies=_body("arm"), points={("ghost", "tip"): object()})
    assert fragment.unresolved_point_bodies() == ("ghost",)


def test_a_self_contained_fragment_reports_nothing_unresolved() -> None:
    fragment = ModelFragment(bodies=_body("arm"), points={("arm", "tip"): object()})
    assert fragment.unresolved_point_bodies() == ()


def test_a_fragment_reports_a_port_owner_it_does_not_declare() -> None:
    """Same reasoning as a point: the owner may belong to another contribution."""
    from suspension_multibody.modeling import GeometryPort

    port = GeometryPort(id=EntityId((), "p"), owner=EntityId((), "ghost"), role="r")
    fragment = ModelFragment(bodies=_body("arm"), ports={"p": port})
    assert fragment.unresolved_port_owners() == ("ghost",)


def test_a_port_owned_by_its_own_body_resolves() -> None:
    from suspension_multibody.modeling import GeometryPort

    port = GeometryPort(id=EntityId((), "p"), owner=EntityId((), "arm"), role="r")
    fragment = ModelFragment(bodies=_body("arm"), ports={"p": port})
    assert fragment.unresolved_port_owners() == ()


def test_mounting_qualifies_identity() -> None:
    fragment = ModelFragment(bodies=_body()).mounted(("vehicle", "front"))
    assert fragment.body_id("upper_arm") == EntityId(("vehicle", "front"), "upper_arm")


def test_unknown_body_is_an_error_not_a_none() -> None:
    fragment = ModelFragment(bodies=_body())
    with pytest.raises(KeyError, match="unknown body"):
        fragment.body_id("missing")


def test_empty_fragment_is_recognisable() -> None:
    """A zero-body subsystem is legal, so "empty" has to be a first-class answer."""
    assert ModelFragment().is_empty()
    assert not ModelFragment(requirements=(PortRequirement(role="r"),)).is_empty()


def test_requirements_accumulate_across_a_merge() -> None:
    left = ModelFragment(requirements=(PortRequirement(role="a"),))
    right = ModelFragment(requirements=(PortRequirement(role="b"),))
    assert [r.role for r in left.merged_with(right).requirements] == ["a", "b"]


def test_assembly_refuses_duplicate_child_names() -> None:
    child = Assembly(name="front", fragment=ModelFragment())
    with pytest.raises(AssemblyError, match="duplicate child instance names"):
        Assembly(
            name="vehicle",
            fragment=ModelFragment(),
            children=(NestedInstance("axle", child), NestedInstance("axle", child)),
        )


def test_assembly_refuses_a_port_declared_at_both_levels() -> None:
    from suspension_multibody.modeling import GeometryPort

    port = GeometryPort(id=EntityId((), "p"), owner=EntityId((), "p"), role="r")
    with pytest.raises(AssemblyError, match="collide"):
        Assembly(
            name="axle",
            fragment=ModelFragment(ports={"p": port}),
            ports={"p": port},
        )


def test_nested_children_stay_addressable() -> None:
    """Flattening is what loses the ability to say which template made a body."""
    axle = Assembly(name="axle", fragment=ModelFragment(bodies=_body("upright_L")))
    vehicle = Assembly(
        name="vehicle",
        fragment=ModelFragment(bodies=_body("body")),
        children=(NestedInstance("front", axle),),
    )
    walked = [level.name for level in vehicle.walk()]
    assert walked == ["vehicle", "axle"]
    assert axle.entity("upright_L") == EntityId(("axle",), "upright_L")


def test_simulation_assembly_reports_the_root_kind_for_policy() -> None:
    """Policy follows the root category, so a nested axle is not judged twice."""
    axle = Assembly(name="axle", fragment=ModelFragment(), root_kind="axle")
    rig = Assembly(name="rig", fragment=ModelFragment(), root_kind="axle")
    simulation = SimulationAssembly(name="sim", assembly=axle, rig=rig)
    assert simulation.root_kind == "axle"


def test_simulation_assembly_collects_ids_from_both_sides() -> None:
    axle = Assembly(name="axle", fragment=ModelFragment(bodies=_body("upright_L")))
    rig = Assembly(name="rig", fragment=ModelFragment(bodies=_body("wheel_carrier")))
    simulation = SimulationAssembly(name="sim", assembly=axle, rig=rig)
    ids = simulation.entity_ids()
    assert EntityId(("axle",), "upright_L") in ids
    assert EntityId(("rig",), "wheel_carrier") in ids


def test_provenance_records_the_template_and_its_properties() -> None:
    """A5 traces an entity to its source; a flattened model cannot do that later."""
    provenance = FragmentProvenance(
        template="front_double_wishbone",
        revision="1",
        properties_fingerprint="abc123",
    )
    fragment = ModelFragment(bodies=_body(), provenance=provenance)
    assert fragment.provenance is not None
    assert fragment.provenance.template == "front_double_wishbone"
