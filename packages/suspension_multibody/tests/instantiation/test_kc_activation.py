"""
Template instantiation, and the K/C column rule.

The counts here are the ones the real assembly produces, not round numbers: an
axle in K mode has 13 ideal constraints and no compliance slots, and the same
axle in C mode still has 9 constraints -- the four outboard ball joints, the four
tie rod ends and the rack guide -- because those connections declare a joint
column and no bushing column.  A "C mode activates bushings" rule that discarded
them would be wrong, and these tests are what says so.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.preparation.assembly import build_front_axle
from suspension_multibody.subsystems import DEFAULT_AXLE_SUBSYSTEMS, AssemblyRequest
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
from tests.benchmark_fixture import benchmark_model

#: The slots the built-in template has no default for: the model owns them.
_MODEL_OWNED = {"spring": 0.0, "damper": 0.0}


def _instance(mode: str, **properties: float):
    return instantiate(
        DOUBLE_WISHBONE, mode=mode, properties={**_MODEL_OWNED, **properties}
    )


def test_the_builtin_template_carries_the_recorded_column_counts() -> None:
    k = _instance("K")
    c = _instance("C")
    assert (len(k.joints), len(k.bushings)) == (13, 4)
    assert (len(c.joints), len(c.bushings)) == (9, 8)


def test_only_the_activated_columns_differ_between_modes() -> None:
    """
    The mode must not move anything: same bodies, same points, same order.

    This is the assertion behind "switching K/C changes no geometry": the
    placement is a property of the template, not of the mode.
    """
    k = _instance("K")
    c = _instance("C")
    assert k.bodies == c.bodies
    assert k.points == c.points
    assert k.inert == c.inert
    assert set(k.joints) & set(c.joints) == {
        "upper_arm_L_outer_joint",
        "lower_arm_L_outer_joint",
        "rack_tie_joint_L",
        "tie_upright_joint_L",
        "upper_arm_R_outer_joint",
        "lower_arm_R_outer_joint",
        "rack_tie_joint_R",
        "tie_upright_joint_R",
        "rack_guide",
    }
    # The four inboard *front* points are the K/C choice: a revolute in K, a
    # bushing in C.  The four inboard *rear* points carry no joint column at all
    # (in K the front revolute's axis already passes through them), so they are
    # bushings in both modes -- which is why K has four bushings and not zero.
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
    }
    assert set(k.bushings) == {
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
    assembly = build_front_axle(benchmark_model(), "K")
    instance = _instance("K")
    assembly_names = [c.name for c in assembly.constraints]
    assert len(assembly_names) == len(instance.joints) == 13
    # Every K joint the template activates is a constraint of the assembly, and
    # the two use the same names and the same order.
    assert [n for n in assembly_names if n in set(instance.joints)] == list(
        instance.joints
    )
    assert assembly.element_ids == ()


def test_c_activation_keeps_the_nine_joints_the_assembly_has() -> None:
    assembly = build_front_axle(benchmark_model(), "C")
    instance = _instance("C")
    assert len(assembly.constraints) == 9
    assert sorted(c.name for c in assembly.constraints) == sorted(instance.joints)


@pytest.mark.parametrize("mode", ["K", "C"])
def test_capabilities_agree_with_the_template_activation(mode: str) -> None:
    assembly = build_front_axle(benchmark_model(), mode)
    assert assembly.capabilities is not None
    assert assembly.capabilities.subsystems == DEFAULT_AXLE_SUBSYSTEMS


def test_mount_stiffness_comes_from_the_template_not_a_constant() -> None:
    """
    The C-mode inboard slots are real property slots now.

    Two facts have to hold at once: the built-in template's own value is zero (so
    the frozen C snapshot stays valid and is not silently re-derived), and a
    template that declares a number actually changes the stiffness -- otherwise
    the slot would still be a hard-coded constant wearing a template's name.
    """
    assert DEFAULT_MOUNT_STIFFNESS == 0.0
    model = benchmark_model()

    def slot_stiffness_norms(assembly) -> set[float]:
        return {
            float(np.linalg.norm(element.stiffness))
            for element in assembly.elements
            if element.name.endswith(("inner_front", "inner_rear"))
        }

    default = build_front_axle(model, "C")
    assert slot_stiffness_norms(default) == {0.0}

    supplied = instantiate(
        DOUBLE_WISHBONE,
        mode="C",
        properties={**_MODEL_OWNED, "bushing": 25_000.0},
    )
    stiffened = build_front_axle(
        model,
        "C",
        AssemblyRequest(
            mode="C",
            subsystems=DEFAULT_AXLE_SUBSYSTEMS,
            suspension_template=supplied,
        ),
    )
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
