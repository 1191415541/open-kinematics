"""
The template data model: one structure, six roles.

The point of these tests is that a template's *shape* does not depend on its
role.  A brake template and a suspension template are the same dataclass with
different contents, which is what lets a simplified brake be swapped for a
detailed one without the assembly layer noticing.
"""

from __future__ import annotations

import pytest

from suspension_multibody.templates import (
    DOUBLE_WISHBONE,
    ROLES,
    ConnectionDefinition,
    OutputDeclaration,
    PartDefinition,
    PropertySlot,
    RoleSpecError,
    Template,
    TemplateError,
    clear,
    get,
    get_role,
    names,
    register,
    register_builtins,
    registered,
    role_names,
    template_dumps,
    template_loads,
    template_to_json,
)


def _bare(name: str, role: str) -> Template:
    """Return a template that satisfies no role, for negative cases."""
    return Template(name=name, role=role)


def _satisfying(role: str, name: str = "ok") -> Template:
    """Build a template that satisfies `role` with the least possible content."""
    spec = get_role(role)
    return Template(
        name=name,
        role=role,
        connections=tuple(
            ConnectionDefinition(f"{mount}_point", mount)
            for mount in spec.required_mounts
        ),
        property_slots=tuple(PropertySlot(slot, "-") for slot in spec.required_slots),
    )


def test_six_roles_are_declared() -> None:
    assert role_names() == (
        "brake",
        "chassis",
        "drive",
        "steering",
        "suspension",
        "wheel",
    )


def test_only_brake_and_drive_carry_a_torque_channel() -> None:
    torque = {name for name, spec in ROLES.items() if spec.has_torque_channel}
    assert torque == {"brake", "drive"}


def test_role_interfaces_do_not_leak_implementation_detail() -> None:
    """
    A role says what a subsystem promises, never how it is built.

    `has_bodies` or `body_count` here would make the simplified templates
    un-replaceable, which is the whole thing the architecture is protecting.
    """
    for name, spec in ROLES.items():
        fields = set(vars(spec))
        for leaked in ("body_count", "has_bodies", "is_simple", "bodies"):
            assert leaked not in fields, f"role {name!r} leaks {leaked!r}"


def test_every_role_declares_mounts_slots_and_outputs() -> None:
    for name, spec in ROLES.items():
        assert spec.required_mounts, f"role {name!r} declares no mounts"
        assert spec.required_slots, f"role {name!r} declares no slots"
        assert spec.outputs, f"role {name!r} declares no outputs"


def test_unknown_role_is_named_in_the_error() -> None:
    with pytest.raises(RoleSpecError, match="unknown subsystem role"):
        get_role("transmission")


def test_a_connection_can_declare_neither_or_both_columns() -> None:
    """
    All four cases are legal, and the two interesting ones are both used.

    The inboard rear point carries no constraint of its own in K mode (the
    assembly uses it only to define the arm's rotation axis), and the inboard
    front point is a joint in K and a bushing in C -- the choice the user asked
    for.
    """
    neither = ConnectionDefinition("locator", "upper_rear")
    joint_only = ConnectionDefinition("outer", "upper_outer", "spherical")
    bushing_only = ConnectionDefinition("compliant", "upper_rear", None, "bushing")
    both = ConnectionDefinition("choice", "upper_front", "revolute", "bushing")

    assert neither.columns() == ()
    assert joint_only.columns() == ("joint",)
    assert bushing_only.columns() == ("bushing",)
    assert both.columns() == ("joint", "bushing")


def test_the_builtin_marks_the_arms_inboard_points_as_the_kc_choice() -> None:
    """
    The K/C mapping is transcribed from the assembly, not from the prose.

    In K mode only the inboard *front* point has a joint; the rear point carries
    a bushing column but no joint column, because the assembly puts no constraint
    there.  Both points are bushings in C.
    """
    by_name = {c.name: c for c in DOUBLE_WISHBONE.connections}
    front = by_name["uca_mount_L_inner_front"]
    rear = by_name["uca_mount_L_inner_rear"]
    assert front.joint == "revolute"
    assert front.bushing is not None
    assert rear.joint is None, "the inboard rear point carries no K-mode joint"
    assert rear.bushing is not None


def test_the_builtin_keeps_joint_only_points_joint_in_both_modes() -> None:
    """Arm outer points and tie rod ends have no bushing column."""
    joint_only = [
        c for c in DOUBLE_WISHBONE.connections if c.joint is not None and c.bushing is None
    ]
    roles = {c.role for c in joint_only}
    assert {"upper_outer", "lower_outer", "tie_inner", "tie_outer"} <= roles
    # Nine joint-only points: four per side (two outer, two tie rod ends) plus
    # the rack guide.  Eight of these survive into C mode, which is why C mode
    # has nine constraints rather than none.
    assert len(joint_only) == 9, [c.name for c in joint_only]


def test_template_round_trips_through_json() -> None:
    assert template_loads(template_dumps(DOUBLE_WISHBONE)) == DOUBLE_WISHBONE


def test_serialised_template_is_json_ready() -> None:
    payload = template_to_json(DOUBLE_WISHBONE)
    assert payload["role"] == "suspension"
    assert payload["name"] == DOUBLE_WISHBONE.name
    assert isinstance(payload["connections"], list)
    assert all(isinstance(c["joint"], (str, type(None))) for c in payload["connections"])


def test_structure_does_not_vary_with_role() -> None:
    """
    Two templates of different roles are built from the same constructor.

    This is the assertion behind "all templates are one format": if a role needed
    its own class, constructing both from the same call would be impossible.
    """
    suspension = _satisfying("suspension", "s")
    brake = _satisfying("brake", "b")
    assert type(suspension) is type(brake)
    assert set(vars(suspension)) == set(vars(brake))


def test_missing_mount_is_rejected_and_named() -> None:
    template = Template(
        name="half",
        role="suspension",
        connections=(ConnectionDefinition("uca", "upper_front", "revolute"),),
        property_slots=tuple(
            PropertySlot(slot, "-") for slot in get_role("suspension").required_slots
        ),
    )
    with pytest.raises(TemplateError, match="missing mount"):
        template.check_role_contract()


def test_missing_slot_is_rejected_and_named() -> None:
    spec = get_role("brake")
    template = Template(
        name="partial-brake",
        role="brake",
        connections=tuple(ConnectionDefinition(f"{m}_p", m) for m in spec.required_mounts),
    )
    with pytest.raises(TemplateError, match="missing property slot"):
        template.check_role_contract()


def test_duplicate_property_slots_are_rejected() -> None:
    spec = get_role("brake")
    slots = tuple(PropertySlot(slot, "-") for slot in spec.required_slots) + (
        PropertySlot(spec.required_slots[0], "-"),
    )
    template = Template(
        name="dup",
        role="brake",
        connections=tuple(ConnectionDefinition(f"{m}_p", m) for m in spec.required_mounts),
        property_slots=slots,
    )
    with pytest.raises(TemplateError, match="duplicate property slot"):
        template.check_role_contract()


def test_duplicate_connections_are_rejected() -> None:
    template = Template(
        name="dup-conn",
        role="chassis",
        connections=(
            ConnectionDefinition("ref", "chassis_reference"),
            ConnectionDefinition("ref", "chassis_reference"),
        ),
        property_slots=tuple(
            PropertySlot(slot, "-") for slot in get_role("chassis").required_slots
        ),
    )
    with pytest.raises(TemplateError, match="declares a connection twice"):
        template.check_role_contract()


def test_unfilled_required_slot_is_rejected_at_instantiation() -> None:
    """A registered template may still be incomplete until properties arrive."""
    brake = _satisfying("brake", "needs-properties")
    with pytest.raises(TemplateError, match="unfilled required property slot"):
        brake.check_filled({})


def test_a_default_fills_a_slot_without_a_properties_file() -> None:
    """
    A template may carry its own numbers.

    The existing assembly does exactly that, so a template that gives a default
    must not be reported as unfilled.
    """
    spec = get_role("drive")
    template = Template(
        name="self-contained-drive",
        role="drive",
        connections=tuple(ConnectionDefinition(f"{m}_p", m) for m in spec.required_mounts),
        property_slots=tuple(
            PropertySlot(slot, "-", default=0.0) for slot in spec.required_slots
        ),
    )
    template.check_filled({})


def test_unknown_template_name_is_reported() -> None:
    with pytest.raises(TemplateError, match="is not registered"):
        get("no_such_template")


def test_duplicate_registration_is_rejected_unless_replacing() -> None:
    clear()
    try:
        register(_satisfying("chassis", "first"))
        with pytest.raises(TemplateError, match="already registered"):
            register(_satisfying("chassis", "first"))
        register(_satisfying("chassis", "first"), replace=True)
        assert "first" in names()
    finally:
        clear()
        register_builtins()


def test_registration_checks_the_role_contract() -> None:
    clear()
    try:
        with pytest.raises(TemplateError, match="missing mount"):
            register(_bare("invalid", "suspension"))
    finally:
        clear()
        register_builtins()


def test_the_builtin_is_registered_at_import() -> None:
    assert "double_wishbone" in names()
    assert registered()["double_wishbone"].role == "suspension"


def test_template_without_a_name_is_rejected() -> None:
    with pytest.raises(TemplateError, match="must carry a name"):
        register(Template(name="", role="chassis"))


def test_parts_outputs_and_elastic_slots_are_carried() -> None:
    assert DOUBLE_WISHBONE.parts
    assert DOUBLE_WISHBONE.outputs
    assert DOUBLE_WISHBONE.elastic_slots
    assert all(isinstance(p, PartDefinition) for p in DOUBLE_WISHBONE.parts)
    assert all(isinstance(o, OutputDeclaration) for o in DOUBLE_WISHBONE.outputs)


def test_builtin_declares_the_two_arm_inboard_points_per_side() -> None:
    """Four inboard arm points, each carrying a bushing column."""
    bushings = [c for c in DOUBLE_WISHBONE.connections if c.bushing is not None]
    assert len(bushings) == 8
    assert {c.name for c in bushings} == {
        "uca_mount_L_inner_front",
        "uca_mount_L_inner_rear",
        "lca_mount_L_inner_front",
        "lca_mount_L_inner_rear",
        "uca_mount_R_inner_front",
        "uca_mount_R_inner_rear",
        "lca_mount_R_inner_front",
        "lca_mount_R_inner_rear",
    }
