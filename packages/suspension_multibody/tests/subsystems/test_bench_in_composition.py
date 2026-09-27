"""
The bench joins the composition, entities included.

A run is one assembly plus one rig, and the pair is what a *simulation* assembly
is.  Before this, the rig was a declaration the composition discarded: it said
what it drives and offered ports, and its own bodies were never part of the model.
For a single-axle assembly that is the difference between loading a wheel and
loading nothing, because the axle deliberately builds no wheel body (D9) -- the
bench supplies it.

These tests pin three things:

1. the bench has its own level, so a caller can still tell which side an entity
   came from;
2. the bench's entities are reachable from the composition;
3. the switch that turns them off restores the declaration-only bench, so the
   change can be *measured* rather than only asserted.
"""

from __future__ import annotations

import pytest

from suspension_multibody.modeling.assembly import SimulationAssembly
from suspension_multibody.schema import FrontAxleModel, MassSpec, Vec3
from suspension_multibody.subsystems.si_assembly import (
    RIG_ENTITIES_SWITCH,
    si_assembly_for_axle,
)
from suspension_multibody.subsystems.types import AssemblyRequest


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


def _composed(rig: str | None = "kc_quasi_static") -> SimulationAssembly:
    return si_assembly_for_axle(
        _model(), request=AssemblyRequest(mode="K"), rig=rig
    )


def test_a_named_bench_brings_its_own_level() -> None:
    """
    The bench is not merged into the device's fragment.

    Keeping the two levels separate is what lets "which side did this entity come
    from" stay answerable, and it is the same reason a nested axle is not
    flattened into its vehicle.
    """
    composed = _composed()
    assert composed.rig.name != "none"
    assert composed.rig.name == "kc_quasi_static"
    assert composed.assembly.name != composed.rig.name


def test_the_bench_entities_reach_the_composition() -> None:
    """
    A wheel-supplying bench owns the wheel, so the model must contain it.

    The axle builds no wheel body by design; if the bench's body stops reaching
    the model, the axle is loaded through nothing and the run silently answers a
    different question.
    """
    composed = _composed()
    bench_bodies = set(composed.rig.fragment.bodies)
    assert bench_bodies, "the bench contributed no bodies at all"
    assert {"wheel_carrier_L", "wheel_carrier_R"} <= bench_bodies
    # And they are reachable from the whole assembly, which is what a document
    # reader walks.
    reachable = {
        name for level in composed.walk() for name in level.fragment.bodies
    }
    assert bench_bodies <= reachable


def test_the_bench_bodies_reach_the_runtime_face() -> None:
    """
    The bench's wheel is in the solved model, not only in the composition.

    A bench body in the composition but not in the runtime is not loaded by the
    solve, and the console would look right -- which is why this is asserted rather
    than assumed.  The carriers are welded to their uprights, so their presence in
    the runtime is what makes the bench part of the model the kernel solves.
    """
    composed = _composed()
    runtime = composed.assembly.physical
    bench_bodies = set(composed.rig.fragment.bodies)
    assert bench_bodies
    carriers = {name for name in bench_bodies if name.startswith("wheel_carrier_")}
    assert carriers == {"wheel_carrier_L", "wheel_carrier_R"}
    assert carriers <= set(runtime.bodies), (
        "the bench's wheels must be in the solved model, not only in the composition"
    )
    # The loading frame is *not* attached: a single-axle bench's frame has nothing
    # to hang from, so it stays out rather than floating as an unconstrained body.
    assert "bench_frame" not in runtime.bodies
    # And each carrier is attached by a real constraint, not merely present.
    welds = {c.name for c in runtime.constraints if "weld" in c.name}
    assert welds == {"wheel_carrier_L_weld", "wheel_carrier_R_weld"}


def test_the_assembly_declares_it_needs_a_wheel_centre_per_side() -> None:
    """
    The axle knows which sides it has; the bench knows what it supplies.

    The requirement is what makes the two meet without either probing the other's
    body names, and it is optional so that an axle with no upright stays legal.
    """
    composed = _composed()
    roles = {requirement.role for requirement in composed.assembly.requirements}
    assert "wheel_centre" in roles
    offered = {port.role for port in composed.rig.ports.values()}
    assert "wheel_centre" in offered


def test_the_bench_ports_are_bound_by_role_not_by_name() -> None:
    """
    The binding is recorded, so a match can be audited instead of guessed at.
    """
    composed = _composed()
    assert "wheel_centre" in composed.bindings
    assert composed.bindings["wheel_centre"]


def test_a_vehicle_bench_contributes_no_wheels() -> None:
    """
    The branch is a capability, not a bench name.

    A vehicle owns its wheels, so a bench that supplies them would double a mass
    and a force path the global rules already place on the assembly.
    """
    composed = _composed("vehicle_dynamic")
    bench_bodies = set(composed.rig.fragment.bodies)
    assert not {name for name in bench_bodies if name.startswith("wheel_carrier")}
    assert "bench_frame" in bench_bodies


def test_turning_the_switch_off_restores_the_declaration_only_bench(monkeypatch) -> None:
    """
    The change must be measurable against the state it replaced.

    This is the assertion that keeps "the bench contributes entities" from being
    a claim in a document: with the switch off the composition carries no bench
    level at all, which is exactly the pre-change behaviour.
    """
    monkeypatch.setenv(RIG_ENTITIES_SWITCH, "0")
    without = _composed()
    assert without.rig.name == "none"
    assert not without.rig.fragment.bodies

    monkeypatch.delenv(RIG_ENTITIES_SWITCH, raising=False)
    with_entities = _composed()
    assert with_entities.rig.fragment.bodies


def test_no_bench_asked_for_means_no_bench(monkeypatch) -> None:
    """Composing and running are separate questions."""
    monkeypatch.delenv(RIG_ENTITIES_SWITCH, raising=False)
    bare = _composed(None)
    assert bare.rig.name == "none"
    assert not bare.rig.fragment.bodies


@pytest.mark.parametrize("value", ["1", "true", "yes", ""])
def test_the_switch_is_on_for_everything_that_is_not_a_leading_zero(
    monkeypatch, value: str
) -> None:
    """
    Only a leading ``0`` turns the bench off.

    An environment variable that has to spell "on" a particular way is a
    variable nobody can set correctly, so the convention is the one this package
    already uses for the other switches.
    """
    monkeypatch.setenv(RIG_ENTITIES_SWITCH, value)
    assert _composed().rig.fragment.bodies
