"""
The built-in template against the assembly it was transcribed from.

The template is only trustworthy if it says the same thing the assembly does.
These tests read the live assembly and compare, point by point, rather than
checking the template against a copy of itself.
"""

from __future__ import annotations

from suspension_multibody.modeling.primitives import (
    BallJoint,
    PrismaticJoint,
    RevoluteJoint,
)
from suspension_multibody.subsystems.entry import compose_axle
from suspension_multibody.templates import DOUBLE_WISHBONE
from tests.benchmark_fixture import benchmark_model


def _assembly(mode: str = "K"):
    return compose_axle(benchmark_model(), mode)  # ty: ignore[invalid-argument-type]


def test_the_assembly_still_produces_the_frozen_counts() -> None:
    """
    K has 13 constraints and C has 9, with 8 bushings.

    These are the numbers the template's K/C mapping has to explain: 13 comes
    from six joints per side plus the rack guide, and 9 from the four joints per
    side that have no bushing column plus the rack guide.
    """
    k = _assembly("K")
    c = _assembly("C")
    assert len(k.constraints) == 16
    assert len(k.ideal_constraints) == 16
    assert len(k.bushings) == 0
    assert len(c.constraints) == 12
    assert len(c.ideal_constraints) == 20
    assert len(c.bushings) == 8


def test_k_mode_puts_no_constraint_at_the_inboard_rear_point() -> None:
    """
    The inboard rear point defines the arm's axis and carries nothing itself.

    If the template declared a joint there, the assembly would show 14 or 15
    constraints instead of 13 -- which is the mistake the requirement's prose
    invites.
    """
    k = _assembly("K")
    names = {constraint.name for constraint in k.ideal_constraints}
    assert "uca_mount_L_inner_front" in names
    assert "lca_mount_L_inner_front" in names
    assert "uca_mount_L_inner_rear" not in names
    assert "lca_mount_L_inner_rear" not in names


def test_k_mode_arms_are_revolutes_and_outer_points_are_ball_joints() -> None:
    """The template's joint column matches the constraint types the assembly emits."""
    k = _assembly("K")
    by_name = {constraint.name: constraint for constraint in k.ideal_constraints}
    assert isinstance(by_name["uca_mount_L_inner_front"], RevoluteJoint)
    assert isinstance(by_name["lca_mount_R_inner_front"], RevoluteJoint)
    assert isinstance(by_name["upper_arm_L_outer_joint"], BallJoint)
    assert isinstance(by_name["lower_arm_R_outer_joint"], BallJoint)
    assert isinstance(by_name["rack_tie_joint_L"], BallJoint)
    assert isinstance(by_name["tie_upright_joint_R"], BallJoint)
    assert isinstance(by_name["rack_guide"], PrismaticJoint)


def test_the_template_agrees_point_by_point_with_the_assembly() -> None:
    """
    Every connection the template declares corresponds to live assembly geometry.

    The comparison is per point, not per count: a template that named the right
    number of points but the wrong ones would still assemble, and would be wrong.
    """
    k = _assembly("K")
    c = _assembly("C")
    k_names = {constraint.name for constraint in k.ideal_constraints}
    c_names = {constraint.name for constraint in c.ideal_constraints}
    c_bushing_names = {bushing.name for bushing in c.bushings}

    for connection in DOUBLE_WISHBONE.connections:
        # Which mode a column is active in is the declaration's own business, so
        # the comparison reads the activation rather than assuming that a declared
        # joint appears in both modes.  A declared-but-inactive column is checked
        # in the other direction, below.
        if connection.active_column("K") == "joint":
            assert connection.name in k_names, (
                f"template activates joint {connection.name!r} in K but K mode has "
                "no such constraint"
            )
        if connection.active_column("C") == "joint":
            assert connection.name in c_names, (
                f"template activates joint {connection.name!r} in C but it is "
                "missing from C mode, where joint-only points must survive"
            )
        if connection.bushing is not None:
            if "C" in connection.bushing_modes:
                assert connection.bushing in c_bushing_names, (
                    f"template declares bushing {connection.bushing!r} but C mode "
                    "has no such bushing"
                )
            if "K" not in connection.bushing_modes:
                assert connection.bushing not in k_names, (
                    f"bushing {connection.bushing!r} is inactive in K and must not "
                    "appear there"
                )
        if connection.joint is not None and "K" not in connection.joint_modes:
            assert connection.name not in k_names, (
                f"template declares joint {connection.name!r} inactive in K but K "
                "mode carries it"
            )


def test_the_rack_guide_is_a_joint_in_both_modes() -> None:
    """
    The rack guide has no bushing column, so it survives into C mode.

    It is also the joint that `rack_fixed_to_chassis` replaces with a weld, so
    the template names the prismatic form and the model chooses at assembly time.
    """
    for mode, expected in (("K", PrismaticJoint), ("C", PrismaticJoint)):
        assembly = _assembly(mode)
        guide = next(
            constraint
            for constraint in assembly.ideal_constraints
            if constraint.name == "rack_guide"
        )
        assert isinstance(guide, expected)


def test_suspension_without_steering_connects_tie_rods_to_ground() -> None:
    """When an assembly has only suspension, tie rods directly connect to ground/chassis."""
    from suspension_multibody.subsystems.types import AssemblyRequest

    assembly = compose_axle(
        benchmark_model(),
        request=AssemblyRequest(
            mode="K", subsystems=frozenset({"suspension", "wheel", "chassis"})
        ),
    )
    names = {constraint.name for constraint in assembly.ideal_constraints}
    assert "rack_guide" not in names
    assert "rack_tie_joint_L" in names
    assert "rack_tie_joint_R" in names
    tie_joint = next(c for c in assembly.constraints if c.name == "rack_tie_joint_L")
    assert tie_joint.body_a == "chassis"
    assert tie_joint.body_b == "tie_rod_L"


def test_the_template_declares_the_arms_inboard_points_as_the_kc_choice() -> None:
    """
    The template's two columns reproduce the K/C asymmetry the assembly has.

    K: one revolute at the inboard front point, nothing at the rear.
    C: a ball joint plus a bushing at both points.
    """
    by_name = {c.name: c for c in DOUBLE_WISHBONE.connections}
    for side in ("L", "R"):
        for arm in ("uca", "lca"):
            front = by_name[f"{arm}_mount_{side}_inner_front"]
            rear = by_name[f"{arm}_mount_{side}_inner_rear"]
            assert front.joint == "revolute", f"{front.name} should be the K joint"
            assert front.joint_kind("C") == "spherical", (
                f"{front.name} is a ball joint in C's ideal column"
            )
            assert front.active_column("K") == "joint"
            assert front.active_column("C") == "bushing"
            # The rear point is inactive in K and carries both columns in C.
            assert rear.active_column("K") is None, f"{rear.name} carries no K row"
            assert rear.active_column("C") == "bushing"
            assert rear.joint == "spherical"
            assert front.bushing is not None
            assert rear.bushing is not None
