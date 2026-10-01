"""
The suspension's declared ports, and the anti-roll bar plugging into them.

The suspension used to offer one port per emitted *body* -- a port that says
"here is a body" and nothing about what a neighbour may attach to.  What an
anti-roll bar's droplink attaches to is a property of the *topology* (the lower
arm on a wishbone, a strut's outer tube on a MacPherson), so it has to be a
declaration on the template.  These tests pin that, and the pairing that follows.
"""

from __future__ import annotations

import pytest

from suspension_multibody.connections.matcher import match_requirements
from suspension_multibody.modeling.identity import EntityId
from suspension_multibody.modeling.ports import GeometryPort, PortRequirement
from suspension_multibody.schema import FrontAxleModel, MassSpec, Vec3
from suspension_multibody.subsystems.si_assembly import contributions_for_axle
from suspension_multibody.subsystems.types import AssemblyRequest
from suspension_multibody.templates import DOUBLE_WISHBONE

_HARDPOINTS = {
    "UPPER_INBOARD_FRONT": Vec3(x=1400.0, y=-500.0, z=500.0),
    "UPPER_INBOARD_REAR": Vec3(x=1550.0, y=-500.0, z=500.0),
    "UPPER_OUTBOARD": Vec3(x=1400.0, y=-750.0, z=350.0),
    "LOWER_INBOARD_FRONT": Vec3(x=1400.0, y=-500.0, z=100.0),
    "LOWER_INBOARD_REAR": Vec3(x=1550.0, y=-500.0, z=100.0),
    "LOWER_OUTBOARD": Vec3(x=1400.0, y=-750.0, z=100.0),
    "TIE_ROD_INBOARD": Vec3(x=1400.0, y=-450.0, z=250.0),
    "TIE_ROD_OUTBOARD": Vec3(x=1400.0, y=-750.0, z=250.0),
    "WHEEL_CENTER": Vec3(x=1400.0, y=-750.0, z=300.0),
    "RACK_CENTER": Vec3(x=1400.0, y=0.0, z=250.0),
}


def _suspension_ports() -> dict[str, GeometryPort]:
    """Return the ports the axle's suspension contribution actually offers."""
    model = FrontAxleModel(
        hardpoints=dict(_HARDPOINTS), mass=MassSpec(sprung_mass=600.0)
    )
    for contribution in contributions_for_axle(
        model, request=AssemblyRequest(mode="K")
    ):
        if contribution.role == "suspension":
            return dict(contribution.ports)
    raise AssertionError("the axle carries no suspension contribution")


def test_the_template_declares_an_arb_mount_per_side() -> None:
    """
    `arb_mount_L`/`arb_mount_R` are template declarations, not body names.

    On the double wishbone the mount is on the **lower arm**, which is what the
    declaration states; nothing derives it from a body-name spelling.
    """
    declared = {port.name: port for port in DOUBLE_WISHBONE.ports}
    assert declared["arb_mount_L"].owner == "lower_arm_L"
    assert declared["arb_mount_R"].owner == "lower_arm_R"
    assert declared["arb_mount_L"].role == "arb_mount"
    assert declared["arb_mount_L"].labels == frozenset({"L"})
    assert declared["arb_mount_R"].labels == frozenset({"R"})


def test_the_declared_ports_reach_the_assembly_product() -> None:
    """
    The offered port set carries the declared names, owned by the declared member.

    Measured on the product rather than on the template, because a declaration
    that never reaches the assembly is not a port a neighbour can use.
    """
    ports = _suspension_ports()
    left = ports["arb_mount_L"]
    right = ports["arb_mount_R"]
    assert left.role == right.role == "arb_mount"
    assert str(left.owner).endswith("lower_arm_L")
    assert str(right.owner).endswith("lower_arm_R")
    assert left.labels == frozenset({"L"})
    assert right.labels == frozenset({"R"})


def test_the_answer_comes_from_the_template_so_another_topology_differs() -> None:
    """
    A MacPherson declares its mount on a different member, and that is the answer.

    This is what separates a declaration from a body-name convention: the
    topology states where the bar attaches, so two topologies that attach it
    differently both describe themselves truthfully.
    """
    from suspension_multibody.templates.model import (
        ConnectionDefinition,
        PartDefinition,
        Template,
    )
    from suspension_multibody.templates.ports import PortDeclaration

    macpherson = Template(
        name="macpherson_probe",
        role="suspension",
        parts=(PartDefinition("strut_L"), PartDefinition("strut_R")),
        connections=(
            ConnectionDefinition("arb_mount_L", "arb_mount_L", owner="strut_L"),
        ),
        property_slots=DOUBLE_WISHBONE.property_slots,
        ports=(
            PortDeclaration(
                name="arb_mount_L",
                role="arb_mount",
                owner="strut_L",
                labels=frozenset({"L"}),
            ),
        ),
    )
    # The strut's outer tube, not the lower arm: the same role, another member.
    assert [port.owner for port in macpherson.ports] == ["strut_L"]
    assert [port.owner for port in DOUBLE_WISHBONE.ports] == [
        "lower_arm_L",
        "lower_arm_R",
    ]


def test_a_mount_naming_an_absent_body_is_not_offered() -> None:
    """
    A declaration whose owner this assembly did not emit is skipped.

    Offering it would be a false claim: a neighbour binding to it would be told
    it may attach to something that is not there.
    """
    from suspension_multibody.subsystems.si_assembly import _declared_ports
    from suspension_multibody.templates.ports import PortDeclaration

    declarations = (
        PortDeclaration(name="ghost", role="arb_mount", owner="not_a_body"),
        PortDeclaration(name="real", role="arb_mount", owner="a_body"),
    )
    offered = _declared_ports(("axle",), declarations, {"a_body": object()})
    assert set(offered) == {"real"}
    assert str(offered["real"].owner).endswith("a_body")


def test_the_bars_mount_requirement_is_met_by_the_declared_port() -> None:
    """
    A requirement for `arb_mount` is satisfied by the suspension's own port.

    The match goes through the existing matcher -- there is no second inference
    path -- and the side label keeps a left requirement off the right port.
    """
    offered = _suspension_ports()
    offered.update(
        {
            "droplink_mount_L": GeometryPort(
                id=EntityId(("axle",), "droplink_mount_L"),
                owner=EntityId(("axle",), "droplink_L"),
                role="droplink_mount_L",
                labels=frozenset({"L"}),
            ),
            "droplink_mount_R": GeometryPort(
                id=EntityId(("axle",), "droplink_mount_R"),
                owner=EntityId(("axle",), "droplink_R"),
                role="droplink_mount_R",
                labels=frozenset({"R"}),
            ),
        }
    )
    report = match_requirements(
        (
            PortRequirement(role="arb_mount", match_labels=frozenset({"L"}), count=1),
            PortRequirement(role="droplink_mount_L", count=1),
        ),
        offered,
    )
    # `binding_for` answers by requirement role: the mount requirement is met by
    # the left mount, and nothing bound the right one on its behalf.
    mount = report.binding_for("arb_mount")
    assert mount is not None
    assert any(str(pid).endswith("arb_mount_L") for pid in mount.port_ids)
    droplink = report.binding_for("droplink_mount_L")
    assert droplink is not None
    assert any(str(pid).endswith("droplink_mount_L") for pid in droplink.port_ids)


def test_a_missing_mount_is_refused_by_name() -> None:
    """
    With no candidate for the mount, the matcher refuses and names the role.

    That is the defined behaviour for a missing pairing: an error naming which
    pairing is missing, rather than a silently unconnected model.
    """
    with pytest.raises(Exception) as caught:
        match_requirements(
            (PortRequirement(role="arb_mount", match_labels=frozenset({"L"}), count=1),),
            {},
        )
    assert "arb_mount" in str(caught.value)
