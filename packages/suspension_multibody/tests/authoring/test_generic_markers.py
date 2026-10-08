"""Generic marker frames and typed ports preserve their declared geometry."""
from __future__ import annotations

import copy
from dataclasses import replace

import numpy as np
import pytest

from suspension_multibody.authoring import GenericSubsystemAssembler, TemplateDocument
from suspension_multibody.authoring.errors import AuthoringError
from suspension_multibody.modeling.ports import ChannelPort, GeometryPort, RoadPort

from .test_generic_multibody import _subsystem, _template


def _declared(**changes):
    payload = _template(
        markers=[{"name": "tool", "owner": "carriage", "point": "mount", "quaternion": [np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)]}],
        ports=[
            {"name": "tool_mount", "role": "attachment", "marker": "tool"},
            {"name": "demand", "role": "command", "kind": "channel", "units": "N"},
            {"name": "surface", "role": "road", "kind": "road", "resource": "plane"},
        ],
    )
    payload.update(changes)
    return payload


def test_marker_port_retains_local_rotation_and_owner():
    model = GenericSubsystemAssembler().assemble(_subsystem(_declared()), instance="slider")
    port = model.ports["slider.tool_mount"]
    assert isinstance(port, GeometryPort)
    assert port.owner.local == "slider.carriage"
    np.testing.assert_allclose(port.pose.rotation @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1e-14)
    assert isinstance(model.ports["slider.demand"], ChannelPort)
    assert isinstance(model.ports["slider.surface"], RoadPort)
    assert {row["name"] for row in model.model_document()["markers"]} == {"slider.tool", "slider.tool_mount"}
    frames = {row["name"]: row for row in model.resolved_model().to_document()["frames"]}
    np.testing.assert_allclose(frames["slider.tool"]["quaternion"], model.markers["slider.tool"].pose.quaternion)


def test_mirrored_marker_rotation_is_reflected_with_body_frame():
    model = GenericSubsystemAssembler().assemble(_subsystem(_declared(symmetry="mirrored_xz")))
    left, right = model.markers["tool_L"], model.markers["tool_R"]
    mirror = np.diag([1.0, -1.0, 1.0])
    np.testing.assert_allclose(right.pose.rotation, mirror @ left.pose.rotation @ mirror, atol=1e-14)


def test_subsystem_classification_does_not_choose_a_different_ir_builder():
    first = _declared()
    second = copy.deepcopy(first)
    second["functional_role"] = "suspension"
    assembler = GenericSubsystemAssembler()
    generic = assembler.assemble(_subsystem(first)).resolved_model()
    effective = replace(_subsystem(first).effective(), functional_role="suspension", template=TemplateDocument.from_payload(second))
    automotive = assembler.assemble(effective).resolved_model()
    assert generic.fingerprint == automotive.fingerprint


@pytest.mark.parametrize("change", ["unknown_marker", "owner_conflict", "channel_units", "point_and_marker"])
def test_invalid_marker_or_channel_has_explicit_error(change):
    payload = copy.deepcopy(_declared())
    if change == "unknown_marker":
        payload["ports"][0]["marker"] = "missing"
    elif change == "owner_conflict":
        payload["ports"][0]["owner"] = "support"
    elif change == "channel_units":
        payload["ports"][1].pop("units")
    else:
        payload["ports"][0]["point"] = "mount"
    with pytest.raises(AuthoringError):
        GenericSubsystemAssembler().assemble(_subsystem(payload))
