"""Measurements name frames and channels, independently of assembly roles."""

import numpy as np
import pytest
from suspension_contracts import ContractError

from suspension_multibody.authoring import assemble_generic
from suspension_multibody.compilation.resolved import compile_resolved
from suspension_multibody.modeling.resolved import ResolvedModel, ResolvedSolvePlan
from suspension_multibody.results import ResultEnvelope
from suspension_multibody.simulation.runner import run_compiled

from ..physics.test_wheel_spin_boundary import plan, wheel_assembly


def test_alignment_uses_declared_carrier_frame_and_matches_tire_load_channel():
    model = assemble_generic(wheel_assembly()).resolved_model()
    document = model.to_document()
    angle = .12
    document["frames"].append({"name": "alignment", "body": "support.carrier", "point": [0, 0, 0],
        "quaternion": [np.cos(angle/2), 0, 0, np.sin(angle/2)]})
    document["measurements"] = [
        {"name": "toe", "type": "toe", "frame": "alignment", "axis": [0, 1, 0], "reference": "world", "units": "rad"},
        {"name": "camber", "type": "camber", "frame": "alignment", "axis": [0, 1, 0], "reference": "world", "units": "rad"},
        {"name": "wheel_load", "type": "tire_load", "element": "wheel.tire", "units": "N"},
        {"name": "hub_position", "type": "frame_position", "frame": "alignment", "units": "m"},
    ]
    run = plan("prescribed_angle", values=[0, .004, .008], rates=[4, 4, 4]).to_document()
    run["solver"] = {"initialization_mode": "provided_consistent_state", "internal_step_s": .00025}
    run["initial_state"] = {"wheel.wheel": {"omega": [0, 4, 0]}}
    result = run_compiled(compile_resolved(ResolvedModel(document, model.resource_payload), ResolvedSolvePlan(run))).result
    assert isinstance(result, ResultEnvelope)
    np.testing.assert_allclose(result.measure("toe").values, angle, atol=1e-12)
    np.testing.assert_allclose(result.measure("camber").values, 0, atol=1e-12)
    np.testing.assert_array_equal(result.measure("wheel_load").values, result.tire_state("wheel.tire")[:, 4])
    np.testing.assert_array_equal(result.measure("hub_position").values, [[0, 0, .334]]*3)
    with pytest.raises(ValueError):
        result.measure("toe").values[0] = 3


def roll_model(kind):
    bodies = [{"name": "fixture", "mass": 0., "fixed": True,
        "inertia": np.eye(3).tolist(), "position": [0, 0, 0]}]
    joints, frames = [], []
    for side, sign in (("left", -1), ("right", 1)):
        body = f"mechanism.{side}"
        bodies.append({"name": body, "mass": 10., "inertia": np.eye(3).tolist(),
            "position": [0, sign*.7, .334]})
        joints.append({"name": f"mount.{side}", "type": kind, "body_a": body, "body_b": "fixture",
            "point_a": [0, -sign*.2, .266], "point_b": [0, sign*.5, .6],
            "axis_a": [0, 0, 1] if kind == "prismatic" else [1, 0, 0],
            "axis_b": [0, 0, 1] if kind == "prismatic" else [1, 0, 0]})
        for label, point in (("contact", [0, 0, -.334]), ("drive", [0, 0, 0])):
            frames.append({"name": f"{label}.{side}", "body": body, "point": point, "quaternion": [1, 0, 0, 0]})
    return ResolvedModel({"schema_version": 1, "name": "linkage", "units": "SI", "bodies": bodies,
        "joints": joints, "elements": [], "frames": frames, "gravity": [0, 0, 0],
        "measurements": [{"name": "roll", "type": "roll_center", "units": "m", "reference": "world",
            "constraints": ["mount.left", "mount.right"], "contact_frames": ["contact.left", "contact.right"],
            "drive_frames": ["drive.left", "drive.right"]}]})


@pytest.mark.parametrize(("kind", "height"), [("prismatic", 0.), ("revolute", 2.1)])
def test_roll_center_reads_native_constraint_derivative_for_different_linkages(kind, height):
    run = {"schema_version": 1, "name": "linkage", "study": "dynamic", "samples": [0, .001],
        "boundaries": [], "inputs": [], "outputs": [], "solver": {"initialization_mode": "provided_consistent_state"}}
    result = run_compiled(compile_resolved(roll_model(kind), ResolvedSolvePlan(run))).result
    assert isinstance(result, ResultEnvelope)
    assert "constraint_jacobian" in result.raw.blocks
    measured = result.measure("roll")
    assert measured.units == "m"
    np.testing.assert_allclose(measured.values, [[0, height]]*2, atol=1e-10)


def test_invalid_measurement_reference_or_unit_fails_before_submission():
    document = roll_model("prismatic").to_document()
    document["measurements"][0]["units"] = "mm"
    with pytest.raises(ContractError, match="incompatible units"):
        ResolvedModel(document)
    document["measurements"][0]["units"] = "m"
    document["measurements"][0]["contact_frames"][0] = "missing"
    with pytest.raises(ContractError, match="unknown frame"):
        ResolvedModel(document)
