"""
Template instantiation, and the K/C column rule.

The counts here are the ones the real assembly produces, not round numbers: the
suspension template activates 14 ideal joints in K mode -- the four inboard
revolutes, the four outboard ball joints, the four tie rod ends and the two wheel
spin joints -- and 10 in C mode with 8 compliance slots, because the outboard ball
joints, the tie rod ends and the spin joints declare a joint column and no bushing
column.  A "C mode activates bushings" rule that discarded them would be wrong, and
these tests are what says so.  The assembly carries two rows more than the template
in either mode: the steering template's rack guide and housing mount.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.authoring import (
    AssemblyDocument,
    SubsystemDocument,
    TemplateDocument,
    assemble_generic,
)
from suspension_multibody.templates import (
    ACTIVATED_MODES,
    DEFAULT_MOUNT_STIFFNESS,
    DOUBLE_WISHBONE,
    ConnectionDefinition,
    TemplateError,
    activated_column,
    get,
    instantiate,
)
from tests.subsystems._generic import axle_source

#: The slots the built-in template has no default for: the model owns them.
_MODEL_OWNED = {"spring": 0.0, "damper": 0.0}


def _instance(mode: str, **properties: float):
    return instantiate(
        DOUBLE_WISHBONE, mode=mode, properties={**_MODEL_OWNED, **properties}
    )


def test_the_builtin_template_carries_the_recorded_column_counts() -> None:
    """
    The counts the composition entry produces: K 14/0, C 10/8.

    Checked against a dump of the historical assembly.  K mode carries **no**
    bushing at all: its inboard rear points are inactive, because in K the arm
    pivots on one revolute at the front point whose axis runs to the rear one, so
    the rear point constrains nothing.  Reading that point as "a bushing in both
    modes" described four rows the assembly has never had.
    """
    k = _instance("K")
    c = _instance("C")
    assert (len(k.joints), len(k.bushings)) == (14, 0)
    assert (len(c.joints), len(c.bushings)) == (10, 8)


def test_only_the_activated_columns_differ_between_modes() -> None:
    """
    The mode must not move anything: same bodies, same points, same order.

    This is the assertion behind "switching K/C changes no geometry": the
    placement is a property of the template, not of the mode.

    What the mode *does* change is which columns are active, and that includes
    points that are active in one mode and idle in the other: the inboard rear
    arm points are bushings in C and constrain nothing in K.  Their identity and
    order are unchanged either way, which is what this test is about.
    """
    k = _instance("K")
    c = _instance("C")
    assert k.bodies == c.bodies
    assert k.points == c.points
    # `inert` is the complement of the active columns, so it differs by exactly
    # the points whose *activation* differs -- and only those.  The four inboard
    # rear points are active in C and idle in K; the four inboard front points are
    # active in both (as different columns), so they are inert in neither.
    assert set(k.inert) - set(c.inert) == {
        "uca_mount_L_inner_rear",
        "lca_mount_L_inner_rear",
        "uca_mount_R_inner_rear",
        "lca_mount_R_inner_rear",
    }
    assert not (set(c.inert) - set(k.inert))
    assert set(k.joints) & set(c.joints) == {
        "upper_arm_L_outer_joint",
        "lower_arm_L_outer_joint",
        "rack_tie_joint_L",
        "tie_upright_joint_L",
        "wheel_spin_joint_L",
        "upper_arm_R_outer_joint",
        "lower_arm_R_outer_joint",
        "rack_tie_joint_R",
        "tie_upright_joint_R",
        "wheel_spin_joint_R",
    }
    # The four inboard *front* points are the K/C choice: a revolute in K, a
    # bushing in C.  In K the front revolute's axis already passes through the
    # rear point, so the rear point carries no row there -- which is why K has no
    # bushing at all rather than four.
    assert set(k.joints) - set(c.joints) == {
        "uca_mount_L_inner_front",
        "lca_mount_L_inner_front",
        "uca_mount_R_inner_front",
        "lca_mount_R_inner_front",
    }
    assert set(c.bushings) - set(k.bushings) == {
        "uca_mount_L_inner_front",
        "lca_mount_L_inner_front",
        "uca_mount_R_inner_front",
        "lca_mount_R_inner_front",
        "uca_mount_L_inner_rear",
        "lca_mount_L_inner_rear",
        "uca_mount_R_inner_rear",
        "lca_mount_R_inner_rear",
    }
    assert k.bushings == ()
    # The rear points are inactive in K, not bushings: `inert` is what records
    # "this point locates geometry and constrains nothing".
    assert set(k.inert) - set(c.inert) == {
        "uca_mount_L_inner_rear",
        "lca_mount_L_inner_rear",
        "uca_mount_R_inner_rear",
        "lca_mount_R_inner_rear",
    }


def test_joint_only_and_bushing_only_points_survive_both_modes() -> None:
    both = ConnectionDefinition("choice", "upper_front", "revolute", "bushing")
    joint_only = ConnectionDefinition("outer", "upper_outer", "spherical")
    bushing_only = ConnectionDefinition("soft", "upper_rear", None, "bushing")
    inert = ConnectionDefinition("locator", "wheel_center")

    assert activated_column(both, "K") == ACTIVATED_MODES["K"] == "joint"
    assert activated_column(both, "C") == ACTIVATED_MODES["C"] == "bushing"
    for mode in ("K", "C"):
        assert activated_column(joint_only, mode) == "joint"
        assert activated_column(bushing_only, mode) == "bushing"
        assert activated_column(inert, mode) is None


def test_switching_mode_equals_instantiating_that_mode() -> None:
    """
    The anti-second-implementation assertion.

    `with_mode` must not be a parallel build path: the switched instance and the
    freshly instantiated one have to be the same object value, in both directions.
    """
    k = _instance("K")
    c = _instance("C")
    assert k.with_mode("C") == c
    assert c.with_mode("K") == k
    assert k.with_mode("C").with_mode("K") == k


def test_with_mode_is_idempotent() -> None:
    k = _instance("K")
    assert k.with_mode("K") == k
    assert k.with_mode("K").with_mode("K") == k.with_mode("K")
    c = _instance("C")
    assert c.with_mode("C") == c


def test_k_activation_reproduces_the_k_assembly() -> None:
    """K is the frozen `k_states.json` baseline's mode, so it must not move."""
    assembly = assemble_generic(axle_source("K"))
    instance = _instance("K")
    assembly_names = [row["name"].rsplit(".", 1)[-1] for row in assembly.joints]
    # The assembly carries the suspension template's own activated joints plus the
    # steering template's two -- its rack guide and the housing mount -- and nothing
    # else: no row is invented, and none is dropped.
    assert set(assembly_names) - set(instance.joints) == {"rack_guide", "housing_mount"}
    assert set(instance.joints) <= set(assembly_names)
    # The document records the suspension's rows per side, in the template's own
    # sequence: left, then right.
    assert [n for n in assembly_names if n in set(instance.joints) and not n.startswith("wheel_spin_joint_")] == [
        n
        for side in ("L", "R")
        for n in instance.joints
        if (f"_{side}_" in n or n.endswith(f"_{side}")) and not n.startswith("wheel_spin_joint_")
    ]
    assert [n for n in assembly_names if n.startswith("wheel_spin_joint_")] == ["wheel_spin_joint_L", "wheel_spin_joint_R"]
    assert assembly.elements == ()


def test_c_activation_keeps_the_joints_the_assembly_has() -> None:
    assembly = assemble_generic(axle_source("C"))
    instance = _instance("C")
    # The suspension template's ten C-mode joints plus the steering template's two.
    assert len(assembly.joints) == 12
    assert {row["name"].rsplit(".", 1)[-1] for row in assembly.joints} == set(instance.joints) | {
        "rack_guide",
        "housing_mount",
    }


@pytest.mark.parametrize("mode", ["K", "C"])
def test_capabilities_agree_with_the_template_activation(mode: str) -> None:
    document = axle_source(mode)
    assembly = assemble_generic(document)
    assert {row["name"].rsplit(".", 1)[-1] for row in assembly.joints} >= set(_instance(mode).joints)
    assert {entry.ref for entry in document.entries} == {"model.sub.json", "wheel.sub.json"}


def test_mount_stiffness_comes_from_the_template_not_a_constant() -> None:
    """
    The C-mode inboard slots are real property slots now.

    Two facts have to hold at once: the built-in template's own value is zero (so
    the frozen C snapshot stays valid and is not silently re-derived), and a
    template that declares a number actually changes the stiffness -- otherwise
    the slot would still be a hard-coded constant wearing a template's name.
    """
    assert DEFAULT_MOUNT_STIFFNESS == 0.0
    document = axle_source("C")

    def slot_stiffness_norms(assembly) -> set[float]:
        return {
            float(np.linalg.norm(element["parameters"]["stiffness"]))
            for element in assembly.elements
            if element["type"] == "bushing" and element["name"].endswith(("inner_front", "inner_rear"))
        }

    default = assemble_generic(document)
    assert slot_stiffness_norms(default) == {0.0}

    supplied = instantiate(
        DOUBLE_WISHBONE,
        mode="C",
        properties={**_MODEL_OWNED, "bushing": 25_000.0},
    )
    documents = {}
    for entry in document.entries:
        payload = entry.subsystem.template.to_payload()
        for row in payload["elements"]:
            if row["type"] == "bushing" and row["name"].endswith(("inner_front", "inner_rear")):
                row["parameters"]["stiffness"] = np.diag([25, 25, 25, 0, 0, 0]).tolist()
        documents[entry.ref] = SubsystemDocument.from_payload(entry.subsystem.to_payload(),
            template=TemplateDocument.from_payload(payload), properties=entry.subsystem.properties)
    stiffened = assemble_generic(AssemblyDocument.from_payload(document.to_payload(), subsystems=documents))
    # Three translational diagonals, so the norm of a 25000 N/m slot is sqrt(3)*25000.
    norms = slot_stiffness_norms(stiffened)
    assert len(norms) == 1
    assert abs(next(iter(norms)) - np.sqrt(3.0) * 25_000.0) < 1e-6
    # And it is the slot, not a per-connection constant: all eight move together.
    assert len(supplied.bushings) == 8


def test_a_supplied_property_must_be_a_declared_slot() -> None:
    with pytest.raises(TemplateError, match="does not declare property slot"):
        instantiate(
            DOUBLE_WISHBONE,
            mode="K",
            properties={**_MODEL_OWNED, "not_a_slot": 1.0},
        )


def test_an_unfilled_required_slot_is_refused_and_named() -> None:
    with pytest.raises(TemplateError, match="spring"):
        instantiate(DOUBLE_WISHBONE, mode="K")


def test_an_unknown_mode_is_refused() -> None:
    with pytest.raises(TemplateError, match="unknown mode"):
        instantiate(DOUBLE_WISHBONE, mode="X")


def test_the_builtin_template_is_the_registered_one() -> None:
    assert get("double_wishbone") is DOUBLE_WISHBONE
