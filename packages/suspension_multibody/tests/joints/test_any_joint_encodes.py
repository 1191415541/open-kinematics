"""
Any assembly can encode any joint.

Before the joint table existed, the kc authoring layer mapped three joint types
and rejected the other five, so "all assemblies share one joint foundation" was
false for any model that declared a universal, cylindrical, in-plane,
constant-velocity or fixed joint.  These tests exercise the path that used to
reject them.
"""

from __future__ import annotations

import pytest

from suspension_multibody.cases.kc_quasi_static.contract import model_document
from suspension_multibody.joints import JointAvailabilityError, validate_joint_axes
from suspension_multibody.preparation.assembly.front_axle import build_front_axle
from suspension_multibody.preparation.assembly.types import (
    ConstantVelocityJoint,
    CylindricalJoint,
    InPlaneJoint,
    UniversalJoint,
    WeldJoint,
)
from suspension_multibody.schema import (
    FrontAxleModel,
    IdealJointSpec,
    MassSpec,
    RigidBodySpec,
    Vec3,
)

#: The five joint kinds the kc authoring layer used to reject outright.
PREVIOUSLY_REJECTED = (
    ("universal", UniversalJoint),
    ("cylindrical", CylindricalJoint),
    ("inplane", InPlaneJoint),
    ("fixed", WeldJoint),
    ("constant_velocity", ConstantVelocityJoint),
)

#: Hardpoints every fixture needs: the explicit path looks up `rack_center`
#: whenever a body named "rack" exists, and the kc authoring path derives drive
#: coordinates from both sides' wheel centres.
_HARDPOINTS = {
    "UPPER_OUTBOARD": Vec3(x=0.0, y=-750.0, z=350.0),
    "WHEEL_CENTER": Vec3(x=0.0, y=-750.0, z=300.0),
    "RACK_CENTER": Vec3(x=0.0, y=0.0, z=250.0),
}


def _explicit_model(kind: str) -> FrontAxleModel:
    """
    Build a model whose only joint is of `kind`.

    The topology is `explicit`, so the assembly carries exactly the joint the
    model declares and nothing the symmetric proxy would add.
    """
    return FrontAxleModel(
        name=f"explicit-{kind}",
        topology="explicit",
        hardpoints=dict(_HARDPOINTS),
        mass=MassSpec(sprung_mass=600.0),
        bodies=(
            RigidBodySpec(name="rack", mass=1.0),
            RigidBodySpec(name="upright_L", mass=10.0),
            RigidBodySpec(name="upright_R", mass=10.0),
        ),
        joints=(
            IdealJointSpec(
                name=f"{kind}_joint",
                kind=kind,  # ty: ignore[invalid-argument-type]
                body_a="rack",
                body_b="upright_L",
                point_a=Vec3(x=0.0, y=0.0, z=250.0),
                point_b=Vec3(x=0.0, y=-750.0, z=300.0),
                axis_a=Vec3(x=0.0, y=0.0, z=1.0),
                axis_b=Vec3(x=0.0, y=0.0, z=1.0),
                axis_a_secondary=Vec3(x=1.0, y=0.0, z=0.0),
                axis_b_secondary=Vec3(x=0.0, y=1.0, z=0.0),
            ),
        ),
    )


@pytest.mark.parametrize(("kind", "constraint_type"), PREVIOUSLY_REJECTED)
def test_the_assembly_builds_the_declared_joint(kind: str, constraint_type: type) -> None:
    """The assembly layer already supported all eight; this is the precondition."""
    assembly = build_front_axle(_explicit_model(kind), "K")
    assert any(isinstance(c, constraint_type) for c in assembly.ideal_constraints)


@pytest.mark.parametrize(("kind", "constraint_type"), PREVIOUSLY_REJECTED)
def test_the_document_encodes_the_declared_joint(kind: str, constraint_type: type) -> None:
    """
    The kc authoring layer now encodes the joint instead of raising.

    This is the assertion that failed before: the old `_JOINT_KINDS` held three
    entries, so the encoding loop raised for every kind here except those three.
    """
    del constraint_type
    assembly = build_front_axle(_explicit_model(kind), "K")
    document = model_document(assembly, name=f"doc-{kind}", drive_wheels=True)
    types = {entry["type"] for entry in document["joints"]}
    expected = "convel" if kind == "constant_velocity" else kind
    assert expected in types, f"{kind} did not reach the document; types were {types}"


def test_secondary_axes_and_angle_target_ride_along_for_convel() -> None:
    """
    The constant-velocity joint needs more than a primary axis.

    The kernel treats every axis as optional and substitutes a default, so an
    omission would produce a plausible model that answers a different question.
    """
    assembly = build_front_axle(_explicit_model("constant_velocity"), "K")
    document = model_document(assembly, name="convel-doc", drive_wheels=True)
    entry = next(e for e in document["joints"] if e["type"] == "convel")
    for field in ("axis_a", "axis_b", "axis_a_secondary", "axis_b_secondary"):
        assert field in entry, f"convel entry is missing {field}"
        assert len(entry[field]) == 3
    assert "convel_angle_target" in entry


def test_the_in_plane_joint_carries_only_the_plane_normal() -> None:
    """
    `InPlaneJoint` has one axis, and the document must not invent a second.

    A spurious `axis_b` would be silently accepted by the kernel and would change
    what the row means.
    """
    assembly = build_front_axle(_explicit_model("inplane"), "K")
    document = model_document(assembly, name="inplane-doc", drive_wheels=True)
    entry = next(e for e in document["joints"] if e["type"] == "inplane")
    assert "axis_a" in entry
    assert "axis_b" not in entry


def test_a_degenerate_axis_is_rejected_at_assembly_time() -> None:
    """
    A zero-length axis must fail here, not become a default in the kernel.

    The kernel substitutes a default for a missing axis, so without this check
    the joint would silently point somewhere the author did not intend.
    """
    with pytest.raises(JointAvailabilityError, match="degenerate"):
        validate_joint_axes(
            joint_name="degenerate_revolute",
            kernel_name="revolute",
            axes={"axis_a": [0.0, 0.0, 0.0], "axis_b": [0.0, 0.0, 1.0]},
        )


def test_a_missing_axis_is_rejected_at_assembly_time() -> None:
    """A joint whose required axis was never produced must fail, naming it."""
    with pytest.raises(JointAvailabilityError, match="axis_b"):
        validate_joint_axes(
            joint_name="half_axis",
            kernel_name="revolute",
            axes={"axis_a": [0.0, 0.0, 1.0]},
        )


def test_a_joint_with_no_axis_requirement_accepts_an_empty_mapping() -> None:
    """Spherical and fixed joints carry no axis; an empty mapping is correct."""
    validate_joint_axes(joint_name="ball", kernel_name="spherical", axes={})
    validate_joint_axes(joint_name="weld", kernel_name="fixed", axes={})


def test_an_unknown_joint_kind_is_rejected_with_its_name() -> None:
    with pytest.raises(JointAvailabilityError, match="unknown type"):
        validate_joint_axes(joint_name="mystery", kernel_name="not_a_joint", axes={})
