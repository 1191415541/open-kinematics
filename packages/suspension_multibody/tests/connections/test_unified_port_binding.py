"""Typed port contracts reject incompatible bindings before assembly."""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.authoring import assemble_generic
from suspension_multibody.connections.matcher import BindingError, match_requirements
from suspension_multibody.modeling.identity import EntityId
from suspension_multibody.modeling.ports import (
    ChannelPort,
    GeometryPort,
    PortRequirement,
)
from suspension_multibody.modeling.primitives.spatial import SE3
from suspension_multibody.presets import generic_template
from tests.authoring.test_unified_subsystem_templates import (
    assembly,
    carrier_subsystem,
    subsystem,
)


def test_kind_family_and_units_are_part_of_binding():
    candidates = {
        "geo": GeometryPort(id=EntityId.root("geo"), owner=EntityId.root("body"), role="mount", family="wheel"),
        "channel": ChannelPort(id=EntityId.root("channel"), owner=EntityId.root("body"), role="mount", units="Nm"),
    }
    assert match_requirements([PortRequirement(role="mount", kind="geometry", family="wheel")], candidates, explicit={"mount": "geo"})
    with pytest.raises(BindingError, match="incompatible"):
        match_requirements([PortRequirement(role="mount", kind="geometry", family="wheel")], candidates, explicit={"mount": "channel"})
    with pytest.raises(BindingError, match="incompatible"):
        match_requirements([PortRequirement(role="mount", kind="channel", units="N")], candidates, explicit={"mount": "channel"})


def test_count_and_cardinality_are_checked():
    port = GeometryPort(id=EntityId.root("mount"), owner=EntityId.root("body"), role="mount", cardinality="one")
    with pytest.raises(BindingError, match="candidate set of 2"):
        match_requirements([PortRequirement(role="mount", count=2)], {"mount": port})


def test_port_pose_is_local_and_can_be_installed_without_name_matching():
    port = GeometryPort(id=EntityId(("rig",), "mount"), owner=EntityId(("rig",), "fixture"), role="mount", pose=SE3.identity())
    assert port.owner.local == "fixture"


def test_named_requirements_share_a_role_without_overwriting_bindings():
    candidates = {name: GeometryPort(id=EntityId.root(name), owner=EntityId.root("body"), role="mount") for name in ("left", "right")}
    report = match_requirements([PortRequirement(role="mount", name=name) for name in ("a", "b")], candidates, explicit={"a": "left", "b": "right"})
    assert report.binding_for("a").port_ids == ("left",)
    assert report.binding_for("b").port_ids == ("right",)


def test_installation_transforms_bodies_frames_and_joint_axes_together():
    wheel = subsystem(generic_template("wheel"), {"center": [0, 0, .334]})
    declaration = assembly({"support": carrier_subsystem(), "wheel": wheel})
    payload = declaration.to_payload()
    placement = {"translation": [2, 3, 4], "quaternion": [2**-.5, 0, 0, 2**-.5]}
    for row in payload["subsystems"]:
        row["placement"] = placement
    installed = type(declaration).from_payload(payload, subsystems={"support": carrier_subsystem(), "wheel": wheel})
    original, transformed = [assemble_generic(doc) for doc in (declaration, installed)]
    np.testing.assert_allclose(transformed.bodies["wheel.wheel"].pose.translation, [2, 3, 4.334])
    joint = transformed.joints[0]
    a = transformed.bodies[joint["body_a"]]
    b = transformed.bodies[joint["body_b"]]
    np.testing.assert_allclose(a.pose.transform_point(joint["point_a"]), b.pose.transform_point(joint["point_b"]), atol=1e-12)
    np.testing.assert_allclose(a.pose.rotation @ joint["axis_a"], [-1, 0, 0], atol=1e-12)
    original_delta = original.bodies["wheel.wheel"].pose.translation - original.bodies["support.carrier"].pose.translation
    transformed_delta = transformed.bodies["wheel.wheel"].pose.translation - transformed.bodies["support.carrier"].pose.translation
    np.testing.assert_allclose(transformed_delta, original_delta)
    assert original.resolved_model().subgraph_fingerprint(tuple(original.bodies)) == transformed.resolved_model().subgraph_fingerprint(tuple(transformed.bodies))
