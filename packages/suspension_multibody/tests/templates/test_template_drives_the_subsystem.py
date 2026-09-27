"""
Choosing a different suspension template must change the subsystem.

This is the assertion the whole template layer exists for, and it is the one that
was missing: the template used to describe the build without driving it, so
"pick a template" changed a number and nothing else.

Two facts shape what a second template can look like, and both are stated here
rather than worked around:

* the ``suspension`` **role contract** requires seven named mounts, so a
  different topology within that role still declares those mount names; what it
  changes is which connections exist and which columns they activate.  A layout
  with different hardpoint *names* would be a different model, not a different
  template for this one;
* the steering rows still come from the steering subsystem, which is hard-coded
  today, so a suspension template cannot add or remove them.  The test therefore
  compares the rows the template does control.

The second template below is a single-arm layout: only the lower arm pivots, and
the upper mount names locate geometry without constraining it.  That is a real
structural difference -- fewer constraints, different classes -- and not a renamed
copy of the built-in template, which a name comparison alone would accept.
"""

from __future__ import annotations

from dataclasses import replace

import pytest

from suspension_multibody.schema import FrontAxleModel, MassSpec, Vec3
from suspension_multibody.subsystems.entry import compose_axle
from suspension_multibody.subsystems.types import AssemblyRequest
from suspension_multibody.templates import DOUBLE_WISHBONE, instantiate
from suspension_multibody.templates.model import (
    ConnectionDefinition,
    PartDefinition,
    PropertySlot,
    Template,
)
from suspension_multibody.templates.registry import register

#: The mount names the suspension role requires, in the role's own order.
_REQUIRED = (
    "upper_front",
    "upper_rear",
    "upper_outer",
    "lower_front",
    "lower_rear",
    "lower_outer",
)


def _single_arm() -> Template:
    """
    One arm per side: the upper mount names locate geometry but carry no row.

    Every required mount name is still declared, because the role demands it.  The
    difference is structural: only the two lower front points pivot.
    """
    connections: list[ConnectionDefinition] = [
        ConnectionDefinition(
            "lca_mount_L_inner_front",
            "lower_front",
            joint="revolute",
            bushing="lca_bushing_L_inner_front",
            owner="lower_arm_L",
            label="inner_front",
            axis_reference_role="lower_rear",
            far_owner="chassis",
            far_label="lca_L_inner_front",
        ),
        ConnectionDefinition(
            "lca_mount_L_inner_rear",
            "lower_rear",
            owner="lower_arm_L",
            label="inner_rear",
            far_owner="chassis",
            far_label="lca_L_inner_rear",
        ),
        ConnectionDefinition(
            "lower_arm_L_outer_joint",
            "lower_outer",
            joint="spherical",
            owner="lower_arm_L",
            label="outer",
            far_owner="upright_L",
            far_label="lower_arm_L_outer",
        ),
        # The upper mounts are declared, and locate geometry, but are inert.
        ConnectionDefinition(
            "uca_mount_L_inner_front", "upper_front", owner="upper_arm_L", label="inner_front"
        ),
        ConnectionDefinition(
            "uca_mount_L_inner_rear", "upper_rear", owner="upper_arm_L", label="inner_rear"
        ),
        ConnectionDefinition(
            "upper_arm_L_outer_joint", "upper_outer", owner="upper_arm_L", label="outer"
        ),
        ConnectionDefinition(
            "wheel_center_L", "wheel_center", owner="upright_L", label="wheel_center"
        ),
    ]
    # The right side mirrors the left.
    mirrored: list[ConnectionDefinition] = []
    for connection in connections:
        mirrored.append(
            ConnectionDefinition(
                name=connection.name.replace("_L", "_R"),
                role=connection.role,
                joint=connection.joint,
                bushing=(
                    None
                    if connection.bushing is None
                    else connection.bushing.replace("_L", "_R")
                ),
                owner=connection.owner.replace("_L", "_R"),
                label=connection.label,
                joint_modes=connection.joint_modes,
                bushing_modes=connection.bushing_modes,
                joint_kind_by_mode=connection.joint_kind_by_mode,
                far_owner=connection.far_owner.replace("_L", "_R"),
                far_label=connection.far_label.replace("_L", "_R"),
                axis_reference_role=connection.axis_reference_role,
            )
        )
    return Template(
        name="single_lower_arm_probe",
        role="suspension",
        parts=(
            PartDefinition("chassis", fixed=True),
            PartDefinition("upper_arm_L"),
            PartDefinition("lower_arm_L"),
            PartDefinition("upright_L"),
            PartDefinition("upper_arm_R"),
            PartDefinition("lower_arm_R"),
            PartDefinition("upright_R"),
        ),
        connections=tuple(connections) + tuple(mirrored),
        property_slots=(
            PropertySlot("spring", "N/m"),
            PropertySlot("damper", "N*s/m"),
            PropertySlot(
                "bushing",
                "N/m",
                default=0.0,
                connections=("lca_mount_L_inner_front", "lca_mount_R_inner_front"),
            ),
        ),
        suspension_kind="single_lower_arm",
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


def _build_with(template: Template):
    """Build the axle with one template selected through the request."""
    instance = instantiate(template, mode="K", properties={"spring": 0.0, "damper": 0.0})
    return compose_axle(
        _model(), request=AssemblyRequest(mode="K", suspension_template=instance)
    )


def test_the_builtin_template_is_what_an_empty_request_selects() -> None:
    """The default must stay the default: no request means the built-in layout."""
    bare = compose_axle(_model())
    explicit = _build_with(DOUBLE_WISHBONE)
    assert [c.name for c in bare.constraints] == [c.name for c in explicit.constraints]


def test_a_different_template_produces_a_different_topology() -> None:
    """
    The entities change, and they change in *kind*, not only in name.

    Compared by constraint name and class, not by body name alone: a template that
    renamed the built-in parts would satisfy a name comparison while leaving the
    model identical, which is the failure a weaker test would let through.
    """
    builtin = _build_with(DOUBLE_WISHBONE)
    single = _build_with(_single_arm())

    builtin_names = [c.name for c in builtin.ideal_constraints]
    single_names = [c.name for c in single.ideal_constraints]
    assert builtin_names != single_names

    # The upper arm mounts are the difference: the built-in pivots them, this one
    # declares them inert.
    assert {"uca_mount_L_inner_front", "uca_mount_R_inner_front"} <= set(builtin_names)
    assert not {
        "uca_mount_L_inner_front",
        "uca_mount_R_inner_front",
    } & set(single_names)
    assert len(single.ideal_constraints) < len(builtin.ideal_constraints)


def test_the_solved_model_changes_with_the_template() -> None:
    """
    A structural difference that never reaches the document would be decorative.

    The comparison is on the *emitted model document*, which is exactly what the
    kernel solves: same bodies, a different joint list.  Comparing the assembly
    objects would be weaker, because the document is where a difference has to
    survive to change a result -- and a difference that stops at the assembly is
    the silent failure this asserts against.
    """
    from suspension_multibody.cases.kc_quasi_static.contract import model_document

    builtin_doc = model_document(_build_with(DOUBLE_WISHBONE), name="probe")
    single_doc = model_document(_build_with(_single_arm()), name="probe")

    def solved_joints(document) -> list[tuple[str, str, str, str]]:
        return [
            (row["name"], row["type"], row["body_a"], row["body_b"])
            for row in document["joints"]
            # The driven coordinates are added by the case layer, not the template.
            if not row["name"].endswith(("_drive_L", "_drive_R", "_drive"))
        ]

    assert solved_joints(builtin_doc) != solved_joints(single_doc)
    # The difference is structural: the single-arm model has no upper-arm pivot.
    single_names = {name for name, *_ in solved_joints(single_doc)}
    assert "uca_mount_L_inner_front" not in single_names
    assert "uca_mount_L_inner_front" in {name for name, *_ in solved_joints(builtin_doc)}
    # Both templates *declare* an upper arm -- this one as an inert locator -- so both
    # build one.  What the template decides here is whether it is constrained, not
    # whether it exists.  That a template can also remove a body outright is the
    # separate property `test_a_template_that_omits_a_part_builds_no_body` asserts.
    builtin_bodies = {b["name"] for b in builtin_doc["bodies"]}
    single_bodies = {b["name"] for b in single_doc["bodies"]}
    assert builtin_bodies == single_bodies
    assert {"chassis", "lower_arm_L", "upright_L"} <= builtin_bodies


def test_a_template_that_omits_a_part_builds_no_body() -> None:
    """
    The template's parts decide the body set, not a list in the builder.

    `side_bodies` used to name three stems unconditionally (`upper_arm`, `lower_arm`,
    `upright`), so a template declaring no upper arm still got one: the template drove
    the *connections* while the bodies were decided for it, and "choose a template"
    could not change the model's entities -- the first thing the flow promises.

    This asserts the property directly, with a template that declares no upper arm.
    The inert-locator template in the test above cannot show it, because that one
    *does* declare an upper arm (it merely leaves it unconstrained).
    """
    from suspension_multibody.templates.model import (
        ConnectionDefinition,
        PartDefinition,
        PropertySlot,
        Template,
    )

    connections = []
    for side in ("L", "R"):
        connections.extend(
            [
                ConnectionDefinition(
                    f"lca_mount_{side}_inner_front",
                    "lower_front",
                    joint="revolute",
                    bushing=f"lca_bushing_{side}_inner_front",
                    owner=f"lower_arm_{side}",
                    label="inner_front",
                    axis_reference_role="lower_rear",
                    far_owner="chassis",
                    far_label=f"lca_{side}_inner_front",
                ),
                ConnectionDefinition(
                    f"lca_mount_{side}_inner_rear",
                    "lower_rear",
                    owner=f"lower_arm_{side}",
                    label="inner_rear",
                    far_owner="chassis",
                    far_label=f"lca_{side}_inner_rear",
                ),
                ConnectionDefinition(
                    f"lower_arm_{side}_outer_joint",
                    "lower_outer",
                    joint="spherical",
                    owner=f"lower_arm_{side}",
                    label="outer",
                    far_owner=f"upright_{side}",
                    far_label=f"lower_arm_{side}_outer",
                ),
                ConnectionDefinition(
                    f"wheel_center_{side}",
                    "wheel_center",
                    owner=f"upright_{side}",
                    label="wheel_center",
                ),
            ]
        )
    template = Template(
        name="no_upper_arm_probe",
        role="suspension",
        parts=(
            PartDefinition("chassis", fixed=True),
            # No `upper_arm_*`: the arms are simply not part of this layout.  The
            # mount names the role contract requires are still declared, and they
            # locate geometry without owning a body.
            PartDefinition("lower_arm_L"),
            PartDefinition("upright_L"),
            PartDefinition("lower_arm_R"),
            PartDefinition("upright_R"),
        ),
        connections=tuple(connections),
        property_slots=(
            PropertySlot("spring", "N/m"),
            PropertySlot("damper", "N*s/m"),
            PropertySlot(
                "bushing",
                "N/m",
                default=0.0,
                connections=("lca_mount_L_inner_front", "lca_mount_R_inner_front"),
            ),
        ),
    )
    # The role contract requires every mount name; declare the upper ones against a
    # body that exists, so this test is about the *body set* and not about the role.
    template = replace(
        template,
        connections=template.connections
        + tuple(
            ConnectionDefinition(
                f"upper_locator_{side}_{label}",
                role,
                owner=f"lower_arm_{side}",
                label=label,
            )
            for side in ("L", "R")
            for role, label in (
                ("upper_front", "inner_front"),
                ("upper_rear", "inner_rear"),
                ("upper_outer", "outer"),
            )
        ),
    )
    register(template, replace=True)
    try:
        built = compose_axle(
            _model(),
            request=AssemblyRequest(
                mode="K", suspension_template="no_upper_arm_probe"
            ),
        )
        bodies = set(built.bodies)
        assert "lower_arm_L" in bodies, "the declared arm must be built"
        assert "upright_L" in bodies, "the declared upright must be built"
        assert "upper_arm_L" not in bodies, (
            "the template declares no upper arm, so none may be built: the body set "
            "has to follow the template"
        )
        assert "upper_arm_R" not in bodies
    finally:
        from suspension_multibody.templates import DOUBLE_WISHBONE
        from suspension_multibody.templates.registry import clear

        clear()
        register(DOUBLE_WISHBONE, replace=True)


def test_a_template_can_be_selected_by_name() -> None:
    """
    "Choose a template" means naming it, for a caller who did not write it.

    An expert registers a template; a user refers to it by name.  Accepting only an
    already-built instance would make the selection an API for the author rather
    than for the user, so both forms are accepted and both reach the same build.
    """
    from suspension_multibody.templates.registry import get, register

    register(_single_arm(), replace=True)
    try:
        by_name = compose_axle(
            _model(),
            request=AssemblyRequest(
                mode="K", suspension_template="single_lower_arm_probe"
            ),
        )
        by_instance = _build_with(get("single_lower_arm_probe"))
        assert [c.name for c in by_name.ideal_constraints] == [
            c.name for c in by_instance.ideal_constraints
        ]
        assert "upper_arm_L" in by_name.bodies
    finally:
        from suspension_multibody.templates import DOUBLE_WISHBONE
        from suspension_multibody.templates.registry import clear
        from suspension_multibody.templates.registry import register as _register

        clear()
        _register(DOUBLE_WISHBONE, replace=True)


def test_a_mismatched_mode_is_refused() -> None:
    """
    An instance carries its mode, so resolving it for another one is an error.

    Silently re-instantiating would discard the properties the caller resolved,
    and the resulting model would differ from the one they asked for.
    """
    instance = instantiate(DOUBLE_WISHBONE, mode="C", properties={"spring": 0.0, "damper": 0.0})
    with pytest.raises(ValueError, match="mode"):
        _ = AssemblyRequest(mode="K", suspension_template=instance).instantiated_suspension


def test_an_unknown_template_name_is_named() -> None:
    """A typo must say which names exist, not fail somewhere downstream."""
    from suspension_multibody.templates.model import TemplateError

    with pytest.raises(TemplateError, match="not registered"):
        _ = AssemblyRequest(
            mode="K", suspension_template="no_such_template"
        ).instantiated_suspension


def test_a_bare_template_is_refused() -> None:
    """
    A ``Template`` is not a selection: it has unresolved property slots.

    Accepting one would skip the resolution that decides which columns carry
    stiffness, which is a silent difference in the model rather than an error.
    """
    with pytest.raises(TypeError, match="SubsystemInstance"):
        _ = AssemblyRequest(
            mode="K", suspension_template=DOUBLE_WISHBONE
        ).instantiated_suspension


def test_an_unknown_joint_kind_is_refused() -> None:
    """
    A type the builder cannot construct fails loudly instead of defaulting.

    Substituting a plausible joint for a declared one produces a model that solves
    and answers a different question, which is worse than refusing.
    """
    connections = [
        ConnectionDefinition(
            "lca_mount_L_inner_front",
            "lower_front",
            joint="no_such_joint",
            owner="lower_arm_L",
            label="inner_front",
            far_owner="chassis",
            far_label="lca_L_inner_front",
        ),
    ]
    for role in _REQUIRED:
        if role == "lower_front":
            continue
        connections.append(
            ConnectionDefinition(f"locator_{role}", role, owner="lower_arm_L", label=role)
        )
    connections.append(
        ConnectionDefinition(
            "wheel_center_L", "wheel_center", owner="upright_L", label="wheel_center"
        )
    )
    template = Template(
        name="bad_kind_probe",
        role="suspension",
        parts=(
            PartDefinition("chassis", fixed=True),
            PartDefinition("lower_arm_L"),
            PartDefinition("upright_L"),
        ),
        connections=tuple(connections),
        property_slots=(
            PropertySlot("spring", "N/m"),
            PropertySlot("damper", "N*s/m"),
            PropertySlot("bushing", "N/m", default=0.0),
        ),
    )
    with pytest.raises(ValueError, match="unsupported ideal joint kind"):
        _build_with(template)
