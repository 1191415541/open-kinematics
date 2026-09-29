"""
The global rules are decided in one place, and production goes through it.

The rules were applied inline in each builder, which meant "is this assembly
legal?" had as many answers as there were places that built one, and a template or
a recipe could step outside them without any single statement being wrong.  They
now live in ``connections/policy.py`` and are reached through one entry point.

These tests assert the *wiring*, not the rule table -- the rule table has its own
tests.  What matters here is that a production build refuses what the rule
refuses, so the rule is not merely available.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from suspension_multibody.connections.policy import RuleViolation, check_root
from suspension_multibody.schema import FrontAxleModel, MassSpec, Vec3
from suspension_multibody.subsystems.entry import compose_axle
from suspension_multibody.subsystems.types import (
    DEFAULT_AXLE_SUBSYSTEMS,
    AssemblyRequest,
)


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


def _request(roles: frozenset[str]) -> AssemblyRequest:
    return AssemblyRequest(mode="K", subsystems=roles)


def test_an_axle_that_carries_a_brake_is_refused_by_the_rule() -> None:
    """
    The refusal is the rule's, and it names the rule.

    A builder that raised its own message would hide which statement decided the
    assembly was illegal, and a later rule change would leave the message behind.
    """
    with pytest.raises(RuleViolation, match="must not carry"):
        compose_axle(
            _model(), request=_request(DEFAULT_AXLE_SUBSYSTEMS | {"brake"})
        )


def test_an_axle_without_a_chassis_grounds_its_mounts() -> None:
    """
    A single axle carries no chassis (requirement 2), and that is legal.

    The rule used to require one, because the suspension had to hang from a fixed
    support.  The support is the *ground* now: a mount whose far end the assembly
    does not carry is attached there, fixed, so the axle still has something to react
    against and no model has to declare a body it does not have.
    """
    assembly = compose_axle(
        _model(), request=_request(DEFAULT_AXLE_SUBSYSTEMS - {"chassis"})
    )
    assert "chassis" not in assembly.bodies
    assert "ground" in assembly.bodies and assembly.bodies["ground"].fixed


def test_an_axle_without_a_suspension_is_refused() -> None:
    with pytest.raises(RuleViolation, match="missing required"):
        compose_axle(
            _model(), request=_request(DEFAULT_AXLE_SUBSYSTEMS - {"suspension"})
        )


def test_the_default_axle_is_accepted() -> None:
    """The rule must not refuse the assembly every existing caller builds."""
    assembly = compose_axle(_model())
    assert "ground" in assembly.bodies and assembly.bodies["ground"].fixed


def test_an_unknown_role_is_refused() -> None:
    """A typo in a role name is a rule violation, not a silently ignored entry."""
    with pytest.raises(RuleViolation, match="unknown role"):
        compose_axle(
            _model(), request=_request(DEFAULT_AXLE_SUBSYSTEMS | {"spoiler"})
        )


def test_the_rule_has_exactly_one_owner() -> None:
    """
    The rule is stated in `connections/policy.py` and nowhere else.

    Asserted by reading the sources, because the failure this guards against is a
    *second* copy appearing: a copy would agree today and drift later, and no
    behavioural test would notice until the two disagreed.

    The copy this used to look for lived in the axle builder, which has since been
    deleted; the check is now over the whole production tree, which is strictly
    stronger -- a restatement anywhere is caught, not only in one file.
    """
    root = (
        Path(__file__).parents[2] / "src" / "suspension_multibody"
    )
    offenders: list[str] = []
    for path in sorted(root.rglob("*.py")):
        if path.name == "policy.py":
            continue
        text = path.read_text(encoding="utf-8")
        for restated in (
            "must carry the chassis subsystem",
            "must carry the suspension subsystem",
            "belongs to the full-vehicle assembly",
        ):
            if restated in text:
                offenders.append(f"{path.name}: {restated!r}")
    assert not offenders, (
        "these modules restate the global rule instead of going through "
        f"connections.policy: {offenders}"
    )
    # And the rule really is applied by the composition's own door.
    entry = (root / "subsystems" / "si_assembly.py").read_text(encoding="utf-8")
    assert "check_root(" in entry, "the axle composition must apply the rule"


def test_the_rule_itself_still_refuses_what_it_always_did() -> None:
    """
    The rule's own contract, restated here so the wiring test above cannot pass
    against a rule that stopped refusing anything.
    """
    with pytest.raises(RuleViolation):
        check_root("axle", {"suspension", "chassis", "drive"})
    with pytest.raises(RuleViolation):
        check_root("vehicle", {"suspension", "chassis", "wheel"})
    check_root("axle", set(DEFAULT_AXLE_SUBSYSTEMS))
