"""
The anti-roll bar subsystem: its role, its four ports, and its two ends.

The bar used to be two element rows whose ends were the literals ``upright_L``
and ``upright_R`` inside the suspension module.  These tests pin what replaced
that: a subsystem of its own, declaring a torsion bar and a droplink per side,
exposing four named ports, and reading its two ends off a template declaration
instead of a body name.
"""

from __future__ import annotations

import pytest

from suspension_multibody.schema import AntiRollBar, Vec3
from suspension_multibody.subsystems import anti_roll_bar as arb
from suspension_multibody.templates import get, names
from suspension_multibody.templates.builtin import ANTI_ROLL_BAR, ANTI_ROLL_BAR_NAME
from suspension_multibody.templates.model import TemplateError
from suspension_multibody.templates.roles import ROLES, get_role


def test_the_role_and_its_template_are_declared() -> None:
    """
    The role exists, states its four mounts, and has a template that satisfies it.

    A role is only real if a template can be registered under it, so both halves
    are asserted: the declaration and the registration that honours it.
    """
    assert "anti_roll_bar" in ROLES
    spec = get_role("anti_roll_bar")
    assert spec.required_mounts == (
        "chassis_mount_L",
        "chassis_mount_R",
        "droplink_mount_L",
        "droplink_mount_R",
    )
    # The bar feeds no wheel torque: it is a suspension member, not a driveline.
    assert spec.has_torque_channel is False
    assert ANTI_ROLL_BAR_NAME in names()
    assert get(ANTI_ROLL_BAR_NAME) is ANTI_ROLL_BAR
    ANTI_ROLL_BAR.check_role_contract()


def test_the_four_ports_are_declared_verbatim() -> None:
    """
    The four port names are declared, one per side, and each carries its side.

    The labels matter as much as the names: without them a left droplink could
    satisfy a right requirement, which is the failure mode the labels exist to
    stop.
    """
    declared = {port.name: port for port in ANTI_ROLL_BAR.ports}
    assert set(declared) == set(arb.PORTS)
    assert sorted(declared) == [
        "chassis_mount_L",
        "chassis_mount_R",
        "droplink_mount_L",
        "droplink_mount_R",
    ]
    for name, port in declared.items():
        side = name[-1]
        assert port.labels == frozenset({side}), name
        assert port.owner.endswith(f"_{side}"), name
        # Each port hangs on a part the template actually declares.
        assert port.owner in [part.name for part in ANTI_ROLL_BAR.parts], name


def test_the_ports_are_template_declarations_not_runtime_strings() -> None:
    """
    The port set comes off the template, not off a list inside the module.

    `arb.port_owners()` is a *read* of the template; changing the template changes
    the answer.  Asserted against the template's own tuple so the two cannot
    drift.
    """
    assert arb.port_owners() == {port.name: port.owner for port in ANTI_ROLL_BAR.ports}


def test_the_subsystem_declares_a_bar_half_and_a_droplink_per_side() -> None:
    """The bar and the two droplinks are real bodies, one each per side."""
    parts = [part.name for part in ANTI_ROLL_BAR.parts]
    assert parts == ["torsion_bar_L", "torsion_bar_R", "droplink_L", "droplink_R"]
    assert arb.bar_bodies() == ("torsion_bar_L", "torsion_bar_R")
    assert arb.droplink_bodies() == ("droplink_L", "droplink_R")


def test_the_bar_uses_the_elastic_link_law_not_the_driven_actuator() -> None:
    """
    The bar is carried by the elastic family, and the module says so.

    The two families are different physics: `AntiRollBarElement` is elastic
    (`k * rise difference`, stores energy) while `RotationalTorqueElement` is a
    driven actuator (`min(stiffness * demand, max_torque)`, stores nothing).  A
    torsion bar is the former.  Pinned here because swapping them would be a
    silent change of the modelled physics.
    """
    assert arb.BAR_ELEMENT_KIND == "anti_roll_bar"
    spec = AntiRollBar(
        name="bar",
        left_body_mount=Vec3(x=0.0, y=-300.0, z=200.0),
        right_body_mount=Vec3(x=0.0, y=300.0, z=200.0),
        left_arm_end=Vec3(x=0.0, y=-700.0, z=200.0),
        right_arm_end=Vec3(x=0.0, y=700.0, z=200.0),
        left_link_point=Vec3(x=0.0, y=-700.0, z=100.0),
        right_link_point=Vec3(x=0.0, y=700.0, z=100.0),
        torsional_stiffness=120.0,
    )
    stiffness = arb.configure(spec, context=None)  # type: ignore[arg-type]
    assert stiffness == 120.0


def test_a_template_that_declares_no_such_port_cannot_satisfy_the_role() -> None:
    """
    Dropping a mount is refused by name, at registration.

    This is what makes the four ports a contract rather than a convention: a
    template without them cannot be registered under this role at all.
    """
    reduced = ANTI_ROLL_BAR.__class__(
        name="anti_roll_bar_without_droplinks",
        role="anti_roll_bar",
        parts=ANTI_ROLL_BAR.parts,
        connections=tuple(
            connection
            for connection in ANTI_ROLL_BAR.connections
            if not connection.role.startswith("droplink_mount")
        ),
        property_slots=ANTI_ROLL_BAR.property_slots,
    )
    with pytest.raises(TemplateError, match="droplink_mount_L"):
        reduced.check_role_contract()
