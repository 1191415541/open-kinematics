"""
The composed SI assembly matches the one the package already builds.

Subtask 06's contract is that the new path is a *reorganisation* and not a
behaviour change, so the honest test is a comparison against the historical
assembly: same bodies in the same recorded order, the same point set, the same
number of active constraints in each mode.  Anything weaker -- "the new path
produces something plausible" -- would let a silent difference through.

The composition is allowed to be *better* in one respect only: it refuses a
half-built model.  Everything else has to agree.
"""

from __future__ import annotations

import pytest

from suspension_multibody.modeling import SimulationAssembly
from suspension_multibody.schema import FrontAxleModel, MassSpec, Vec3
from suspension_multibody.subsystems.composition import (
    CompositionError,
    SubsystemContribution,
    compose_simulation_assembly,
    fingerprint_assembly,
)
from suspension_multibody.subsystems.entry import compose_axle
from suspension_multibody.subsystems.si_assembly import (
    contributions_for_axle,
    si_assembly_for_axle,
)
from suspension_multibody.subsystems.types import AssemblyRequest, SubsystemOutput


def _model() -> FrontAxleModel:
    return FrontAxleModel(
        hardpoints={
            "uca_front": Vec3(x=-100, y=-500, z=400),
            "uca_rear": Vec3(x=100, y=-500, z=400),
            "uca_outer": Vec3(x=0, y=-700, z=450),
            "lca_front": Vec3(x=-120, y=-500, z=150),
            "lca_rear": Vec3(x=120, y=-500, z=150),
            "lca_outer": Vec3(x=0, y=-700, z=150),
            "tierod_inner": Vec3(x=100, y=-400, z=250),
            "tierod_outer": Vec3(x=50, y=-700, z=250),
            "wheel_center": Vec3(x=0, y=-700, z=300),
            "rack_center": Vec3(x=0, y=0, z=250),
        },
        mass=MassSpec(sprung_mass=1000),
    )


@pytest.mark.parametrize("mode", ["K", "C"])
def test_the_composed_bodies_match_the_historical_order(mode: str) -> None:
    """
    Order included, because the contract document lists bodies in sequence.

    A composed model with the right set of bodies in the wrong order would emit a
    different document, which is exactly the kind of difference that is invisible
    in a summary and visible in every recorded artifact.
    """
    historical = compose_axle(_model(), mode)
    composed = si_assembly_for_axle(_model(), request=AssemblyRequest(mode=mode))
    assert list(composed.assembly.fragment.bodies) == list(historical.bodies)


@pytest.mark.parametrize("mode", ["K", "C"])
def test_the_composed_points_match_the_historical_set(mode: str) -> None:
    historical = compose_axle(_model(), mode)
    composed = si_assembly_for_axle(_model(), request=AssemblyRequest(mode=mode))
    assert set(composed.assembly.fragment.points) == set(historical.points)


@pytest.mark.parametrize("mode,expected", [("K", 13), ("C", 9)])
def test_the_active_constraint_count_is_preserved(mode: str, expected: int) -> None:
    """K has 13 joints and C has 9: C keeps the tie rods, it does not drop them."""
    historical = compose_axle(_model(), mode)
    composed = si_assembly_for_axle(_model(), request=AssemblyRequest(mode=mode))
    assert len(historical.constraints) == expected
    assert len(composed.assembly.fragment.joints) == expected


def test_an_assembly_without_steering_has_no_rack_or_tie_rods() -> None:
    """
    The optional branch disappears wholesale, not as a degenerate body.

    A floating rack would poison the capability check -- the rig would think the
    assembly can be steered -- so absence is the only correct representation.
    """
    request = AssemblyRequest(
        mode="K",
        subsystems=frozenset({"chassis", "suspension", "wheel"}),
    )
    composed = si_assembly_for_axle(_model(), request=request)
    bodies = set(composed.assembly.fragment.bodies)
    assert "rack" not in bodies
    assert not any(name.startswith("tie_rod") for name in bodies)


def test_the_composed_assembly_reports_the_roles_it_carries() -> None:
    composed = si_assembly_for_axle(_model())
    assert composed.assembly.subsystems >= {"chassis", "suspension", "steering"}
    assert "brake" not in composed.assembly.subsystems
    assert "drive" not in composed.assembly.subsystems


def test_the_fingerprint_is_structural_and_stable() -> None:
    """
    Two compositions of the same model agree; a different model does not.

    This is the property ``A6`` leans on: the same assembly read by two studies
    reports one fingerprint, so "same model, two studies" is an identity check
    rather than a hope.
    """
    first = si_assembly_for_axle(_model())
    second = si_assembly_for_axle(_model())
    assert first.fingerprint == second.fingerprint

    other = _model().model_copy(
        update={"hardpoints": {**_model().hardpoints, "wheel_center": Vec3(x=5, y=-700, z=300)}}
    )
    assert si_assembly_for_axle(other).fingerprint != first.fingerprint


def test_the_fingerprint_ignores_the_subsystem_collection_order() -> None:
    """
    Requirements are resolved after all contributions are in hand, so order is
    not an input to the result.
    """
    contributions = contributions_for_axle(_model())
    forward = compose_simulation_assembly(contributions, body_order=tuple(("chassis", "rack")))
    reversed_contributions = tuple(reversed(contributions))
    backward = compose_simulation_assembly(
        reversed_contributions, body_order=tuple(("chassis", "rack"))
    )
    assert forward.fingerprint == backward.fingerprint


def test_a_role_contributed_twice_is_refused() -> None:
    contribution = SubsystemContribution(
        role="chassis", output=SubsystemOutput(), ports={}
    )
    with pytest.raises(CompositionError, match="contributed twice"):
        compose_simulation_assembly([contribution, contribution])


def test_an_unknown_role_is_refused_where_it_is_written() -> None:
    with pytest.raises(CompositionError, match="unknown subsystem role"):
        SubsystemContribution(role="spoiler", output=SubsystemOutput(), ports={})


def test_an_empty_composition_is_refused() -> None:
    with pytest.raises(CompositionError, match="at least one contribution"):
        compose_simulation_assembly([])


def test_a_body_order_naming_a_missing_body_is_refused() -> None:
    """The caller's expectation and the model disagreeing is an error, not a no-op."""
    contributions = contributions_for_axle(_model())
    with pytest.raises(CompositionError, match="no contribution produced"):
        compose_simulation_assembly(contributions, body_order=("chassis", "no_such_body"))


def test_the_result_is_a_simulation_assembly() -> None:
    composed = si_assembly_for_axle(_model())
    assert isinstance(composed, SimulationAssembly)
    assert composed.root_kind == "axle"


def test_composition_does_not_need_the_legacy_path() -> None:
    """
    The composed value is built from the subsystems directly.

    Checked by composing without ever calling the historical builder: if the new
    path secretly depended on the old one having run, the two would be one path
    wearing two names, and 07 could not switch over.
    """
    composed = si_assembly_for_axle(_model())
    assert composed.assembly.fragment.bodies
    assert fingerprint_assembly(composed.assembly) == composed.fingerprint


def test_a_composed_assembly_with_a_dangling_reference_is_refused() -> None:
    """
    The completeness check lives at the assembly level, and it has to bite.

    A composition whose merged model still has a point on a body nobody declared
    is not a model, and the refusal has to happen when the simulation assembly is
    built rather than when someone finally tries to solve it.
    """
    from suspension_multibody.modeling import (
        Assembly,
        AssemblyError,
        EntityId,
        GeometryPort,
        ModelFragment,
        SimulationAssembly,
    )

    fragment = ModelFragment(
        bodies={"arm": object()},
        points={("ghost", "tip"): object()},
    ).mounted(("axle",))
    port = GeometryPort(
        id=EntityId(("axle",), "p"), owner=EntityId(("axle",), "arm"), role="body"
    )
    assembly = Assembly(name="axle", fragment=fragment, ports={"p": port})
    with pytest.raises(AssemblyError, match="resolve every reference"):
        SimulationAssembly(
            name="axle", assembly=assembly, rig=Assembly(name="none", fragment=ModelFragment())
        )


def test_a_composed_assembly_with_every_reference_resolved_is_accepted() -> None:
    """The check must not fire on a model that is complete."""
    composed = si_assembly_for_axle(_model())
    assert composed.assembly.fragment.bodies
