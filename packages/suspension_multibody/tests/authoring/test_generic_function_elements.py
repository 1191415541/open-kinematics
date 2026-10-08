"""Marker-bound force laws are template data and compile into native programs."""

from __future__ import annotations

import copy

import numpy as np
import pytest

from suspension_multibody.api import validate
from suspension_multibody.authoring import GenericSubsystemAssembler
from suspension_multibody.authoring.errors import AuthoringError
from suspension_multibody.kernel.capabilities import require_function_program_version

try:
    from .test_generic_multibody import _assembly, _case, _subsystem, _template
except ImportError:
    from test_generic_multibody import _assembly, _case, _subsystem, _template


def function_subsystem(kind="torque", expression="demand", bindings=None, **updates):
    element = {"name": "actuation", "type": kind, "action": "action", "reaction": "reaction", "reference": "reaction", "axis": [0, 0, 1],
               "bindings": bindings or {"demand": {"unit": "N" if kind == "force" else "Nm", "value": 2}}}
    if kind == "wrench":
        element["functions"] = ["force", "force", "force", "moment", "moment", "moment"]
        element["bindings"] = {"force": {"unit": "N", "value": 3}, "moment": {"unit": "Nm", "value": 2}}
    else:
        element["function"] = expression
    payload = _template(
        bodies=[{"name": "support", "fixed": True}, {"name": "carriage", "mass": 10, "inertia": np.eye(3).tolist()}],
        markers=[{"name": "action", "owner": "carriage", "point": "mount"}, {"name": "reaction", "owner": "support", "point": "base"}],
        joints=[{"name": "guide", "type": "prismatic" if kind == "force" else "revolute", "body_a": "support", "body_b": "carriage", "point_a": "base", "point_b": "mount", "axis": [0, 0, 1]}],
        elements=[element], property_slots=[], ports=[],
    )
    payload.update(updates)
    return _subsystem(payload, coordinates={"base": [0, 0, 0], "mount": [0, 0, 0]})


@pytest.mark.parametrize("kind", ["force", "torque", "wrench"])
def test_functions_bind_complete_marker_poses(kind):
    built = GenericSubsystemAssembler().assemble(function_subsystem(kind))
    model = built.model_document()
    element = model["elements"][0]
    assert element["type"] == kind
    for key, body in (("action", "carriage"), ("reaction", "support"), ("reference", "support")):
        assert element["parameters"][key] == {"body": body, "point": [0, 0, 0], "quaternion": [1, 0, 0, 0]}
    assert len(model["function_programs"]) == (6 if kind == "wrench" else 1)


def test_program_indices_rebase_across_subsystems():
    assembly = _assembly({"one": function_subsystem(), "two": function_subsystem("wrench")}, gravity=[0, 0, 0])
    compiled = validate(assembly, _case())
    elements = compiled.model_document["elements"]
    assert elements[0]["parameters"]["program_id"] == 0
    assert elements[1]["parameters"]["program_ids"] == list(range(1, 7))


def test_mirrored_force_and_torque_follow_polar_and_axial_axes():
    force = GenericSubsystemAssembler().assemble(function_subsystem("force", symmetry="mirrored_xz"))
    torque = GenericSubsystemAssembler().assemble(function_subsystem("torque", symmetry="mirrored_xz"))
    np.testing.assert_array_equal(force.elements[1]["parameters"]["axis"], [0, 0, 1])
    np.testing.assert_array_equal(torque.elements[1]["parameters"]["axis"], [0, 0, -1])


def test_unknown_marker_and_missing_reaction_are_rejected():
    payload = copy.deepcopy(function_subsystem().template.payload)
    payload["elements"][0]["reaction"] = "missing"
    with pytest.raises(AuthoringError, match="unknown reaction"):
        _subsystem(payload)
    del payload["elements"][0]["reaction"]
    with pytest.raises(AuthoringError, match="reaction"):
        _subsystem(payload)


def test_old_kernel_capability_cannot_silently_accept_program(monkeypatch):
    monkeypatch.setattr("suspension_multibody.kernel.capabilities.kernel_capability_document", lambda: {"contract_version": 1})
    with pytest.raises(ValueError, match="does not support function program"):
        require_function_program_version(1)
