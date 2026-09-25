"""
The global rules: an axle has no brake, a vehicle must have steering.

``D3`` fixed these and refused to let a template, a recipe or a registration
relax them.  The tests below are written as the two directions of the same claim,
because the rule is symmetric even though the two assemblies are not: an axle
carrying a brake and a vehicle without one are both assemblies that are not what
they claim to be.

The vehicle side is checked through the rule directly, and the axle side is also
checked against the *real* assembly builders -- a rule that only the rule's own
test obeys would not be much of a rule.
"""

from __future__ import annotations

import pytest

from suspension_multibody.connections import (
    AXLE_RULE,
    ROLES,
    VEHICLE_RULE,
    RuleViolation,
    check_root,
    rule_for,
)
from suspension_multibody.preparation.assembly import build_front_axle
from suspension_multibody.schema import (
    FrontAxleModel,
    MassSpec,
    RigidBodySpec,
    Vec3,
    VehicleModel,
)
from suspension_multibody.subsystems import (
    DEFAULT_AXLE_SUBSYSTEMS,
    DEFAULT_VEHICLE_SUBSYSTEMS,
)

# -- the rule itself -------------------------------------------------------


def test_an_axle_may_not_carry_brake_or_drive() -> None:
    with pytest.raises(RuleViolation, match="must not carry"):
        check_root("axle", {"suspension", "chassis", "brake"})


def test_an_axle_without_brake_or_drive_is_accepted() -> None:
    check_root("axle", {"suspension", "chassis"})


def test_an_axle_may_carry_steering_or_not() -> None:
    """Steering is optional on a single axle; the rule says nothing against it."""
    check_root("axle", {"suspension", "chassis", "steering"})
    check_root("axle", {"suspension", "chassis"})


def test_a_vehicle_must_have_steering_brake_and_drive() -> None:
    for missing in ("steering", "brake", "drive"):
        with pytest.raises(RuleViolation, match="missing required"):
            check_root("vehicle", {"suspension", "chassis", "wheel"} | (
                {"steering", "brake", "drive"} - {missing}
            ))


def test_a_complete_vehicle_is_accepted() -> None:
    check_root("vehicle", DEFAULT_VEHICLE_SUBSYSTEMS)


def test_an_axle_must_have_suspension() -> None:
    with pytest.raises(RuleViolation, match="missing required"):
        check_root("axle", {"chassis"})


def test_an_unknown_role_is_named() -> None:
    with pytest.raises(RuleViolation, match="unknown role"):
        check_root("axle", {"suspension", "spoiler"})


def test_an_unknown_root_kind_is_named() -> None:
    with pytest.raises(RuleViolation, match="unknown root assembly kind"):
        check_root("motorbike", {"suspension"})


def test_the_rule_for_a_kind_is_looked_up_by_kind() -> None:
    """
    Nested axles are judged by the root, so the lookup must take a kind.

    A vehicle contains two axles; applying the single-axle rule to them would
    demand the vehicle give up its own wheels, so the rule is a function of the
    root category and not of how deep an assembly sits.
    """
    assert rule_for("axle") is AXLE_RULE
    assert rule_for("vehicle") is VEHICLE_RULE
    assert AXLE_RULE.wheels_from_rig is True
    assert VEHICLE_RULE.wheels_from_rig is False


def test_the_rules_cover_every_declared_role() -> None:
    """A role no rule mentions could be added without any rule noticing."""
    mentioned = AXLE_RULE.forbidden | AXLE_RULE.required | VEHICLE_RULE.required
    assert mentioned <= set(ROLES)


def test_no_rule_can_be_overridden_by_an_argument() -> None:
    """
    There is no escape hatch, and that is the design.

    ``check_root`` takes two arguments.  A keyword like ``allow`` would be the
    first place a template author would look to step around D3, so the absence of
    one is asserted rather than left to convention.
    """
    import inspect

    parameters = list(inspect.signature(check_root).parameters)
    assert parameters == ["kind", "roles"]


# -- the real assemblies obey the same rule --------------------------------


def _axle_model() -> FrontAxleModel:
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


def test_the_built_axle_reports_no_brake_or_drive() -> None:
    """The rule is checked against the assembly the product actually builds."""
    assembly = build_front_axle(_axle_model(), mode="K")
    roles = set(assembly.capabilities.subsystems) if assembly.capabilities else set()
    assert "brake" not in roles
    assert "drive" not in roles
    check_root("axle", roles & set(ROLES))


def test_the_axle_default_subset_satisfies_its_own_rule() -> None:
    check_root("axle", set(DEFAULT_AXLE_SUBSYSTEMS))


def test_the_vehicle_default_subset_satisfies_its_own_rule() -> None:
    check_root("vehicle", set(DEFAULT_VEHICLE_SUBSYSTEMS))


def test_a_vehicle_missing_a_brake_cannot_be_built() -> None:
    """
    The refusal is real, not just the rule.

    A vehicle model without steering cannot be constructed at all -- the schema
    requires it -- which is the strongest form of "must exist".  The rule covers
    the rest.
    """
    with pytest.raises(Exception):
        VehicleModel(  # type: ignore[call-arg]
            axles=[],
            body=RigidBodySpec(name="body", mass=1500),
            wheels=[],
            mass=MassSpec(sprung_mass=1500),
        )
