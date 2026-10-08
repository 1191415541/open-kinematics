"""One wheel definition supports free, locked and prescribed relative spin."""

import numpy as np
import pytest

from suspension_multibody.authoring import (
    AssemblyDocument,
    TemplateDocument,
    assemble_generic,
)
from suspension_multibody.compilation.motion import resolve_motion_boundaries
from suspension_multibody.compilation.resolved import compile_resolved
from suspension_multibody.modeling.primitives.spatial import (
    quaternion_multiply,
    quaternion_to_matrix,
)
from suspension_multibody.modeling.resolved import ResolvedModel, ResolvedSolvePlan
from suspension_multibody.presets import generic_template
from suspension_multibody.results.envelope import ResultEnvelope
from suspension_multibody.simulation.runner import run_compiled

from ..authoring.test_unified_subsystem_templates import (
    assembly,
    carrier_subsystem,
    subsystem,
)


def wheel_assembly():
    return assembly({"support": carrier_subsystem(), "wheel": subsystem(generic_template("wheel"), {"center": [0, 0, .334]})})


def plan(mode="locked", *, values=None, rates=None, samples=None):
    boundary = {"name": "lock", "coordinate": "wheel.spin", "mode": mode, "units": "rad"}
    if mode == "locked":
        boundary["value"] = 0
    elif mode == "prescribed_angle":
        boundary["program"] = "angle"
    return ResolvedSolvePlan({"schema_version": 1, "name": "motion", "study": "dynamic",
        "samples": [0, .001, .002] if samples is None else samples, "solver": {}, "boundaries": [boundary],
        "inputs": [] if values is None else [{"name": "angle", "values": values, "rates": rates}], "outputs": []})


def compiled_motion(mode="locked", *, torque=0, omega=0, values=None, rates=None, samples=None):
    source = wheel_assembly()
    built = assemble_generic(source)
    model = built.resolved_model()
    document = model.to_document()
    run = plan(mode, values=values, rates=rates, samples=samples).to_document()
    run["solver"] = {"initialization_mode": "provided_consistent_state", "internal_step_s": .00025}
    run["initial_state"] = {"wheel.wheel": {"omega": [0, omega, 0]}}
    if torque:
        from suspension_multibody.authoring.function_program import compile_function

        document["function_programs"] = [compile_function("demand", output_unit="Nm", bindings={"demand": {"value": torque, "unit": "Nm"}})]
        document["elements"].append({"name": "drive", "type": "torque", "parameters": {
            "action": {"body": "wheel.wheel"}, "reaction": {"body": "support.carrier"},
            "reference": {"body": "support.carrier"}, "axis": [0, 1, 0], "program_id": 0}})
    return compile_resolved(ResolvedModel(document, model.resource_payload), ResolvedSolvePlan(run))


def test_same_wheel_has_explicit_coordinate_and_nonspinning_contact_frame():
    built = assemble_generic(wheel_assembly())
    model = built.resolved_model()
    locked = resolve_motion_boundaries(model, plan()).model.to_document()
    free = resolve_motion_boundaries(model, plan("free")).model.to_document()
    assert locked["bodies"] == free["bodies"]
    assert locked["tires"] == free["tires"]
    assert locked["joints"][:-1] == free["joints"]
    assert locked["joints"][-1]["type"] == "driven_rotation"
    assert locked["tires"][0]["parameters"]["frame_body"] == "support.carrier"
    assert locked["coordinates"][0]["source_joint_id"] == "wheel.bearing"


@pytest.mark.parametrize("proxy", [False, True])
def test_contact_frame_cannot_rotate_with_the_declared_spin(proxy):
    model = assemble_generic(wheel_assembly()).resolved_model()
    document = model.to_document()
    frame_owner = "wheel.wheel"
    if proxy:
        frame_owner = "wheel.proxy"
        document["bodies"].append({**document["bodies"][1], "name": frame_owner})
        document["joints"].append({"name": "wheel.proxy_mount", "type": "fixed",
            "body_a": "wheel.wheel", "body_b": frame_owner, "point_a": [0, 0, 0], "point_b": [0, 0, 0]})
    document["tires"][0]["parameters"]["frame_body"] = frame_owner
    with pytest.raises(ValueError, match="contact frame.*relative spin"):
        ResolvedModel(document, model.resource_payload)


def test_locked_spin_balances_drive_torque_without_removing_bearing():
    result = run_compiled(compiled_motion(torque=7))
    assert result.status == "success"
    index = result.raw.body_names.index("wheel.wheel")
    np.testing.assert_allclose(result.raw.states[:, index, 3:7], [[1, 0, 0, 0]]*3, atol=1e-10)
    wrench = result.raw.blocks["constraint_wrench"]
    assert np.max(np.abs(wrench[..., 4])) == pytest.approx(7, rel=1e-6)


def test_free_spin_rolls_using_identical_wheel_document():
    result = run_compiled(compiled_motion("free", omega=2))
    assert result.status == "success"
    index = result.raw.body_names.index("wheel.wheel")
    assert abs(result.raw.states[-1, index, 5]) > 1e-4


def test_prescribed_motion_crosses_multiple_turns_without_losing_speed():
    samples = np.linspace(0, .06, 121).tolist()
    speed = 300
    angles = [speed*time for time in samples]
    result = run_compiled(compiled_motion("prescribed_angle", omega=speed, values=angles,
        rates=[speed]*len(samples), samples=samples))
    assert result.status == "success"
    index = result.raw.body_names.index("wheel.wheel")
    quaternion = result.raw.states[:, index, 3:7]
    expected = np.column_stack((np.cos(np.asarray(angles)/2), np.zeros(len(samples)),
        np.sin(np.asarray(angles)/2), np.zeros(len(samples))))
    np.testing.assert_allclose(np.abs(np.sum(quaternion*expected, axis=1)), 1, atol=1e-9)
    np.testing.assert_allclose(result.raw.states[:, index, 11], speed, rtol=0, atol=1e-7)


@pytest.mark.parametrize("mirrored", [False, True])
def test_contact_frame_preserves_installed_orientation_across_multiple_turns(mirrored):
    source = wheel_assembly()
    payload = source.to_payload()
    quaternion = np.array([np.cos(.2), np.sin(.2), 0., 0.])
    if mirrored:
        quaternion *= [1, -1, 1, -1]
    for row in payload["subsystems"]:
        row["placement"] = {"translation": [0, 0, 1], "quaternion": quaternion.tolist()}
    source = AssemblyDocument.from_payload(payload, subsystems={row.ref: row.subsystem for row in source.entries})
    model = assemble_generic(source).resolved_model()
    samples = np.linspace(0, .04, 81)
    speed = 300
    run = plan("prescribed_angle", values=(speed*samples).tolist(), rates=[speed]*len(samples), samples=samples.tolist()).to_document()
    run["solver"] = {"initialization_mode": "provided_consistent_state", "internal_step_s": .00025}
    run["initial_state"] = {"wheel.wheel": {"omega": (quaternion_to_matrix(quaternion)@np.array([0, speed, 0])).tolist()}}
    result = run_compiled(compile_resolved(model, ResolvedSolvePlan(run)))
    assert result.status == "success"
    envelope = result.result
    assert isinstance(envelope, ResultEnvelope)
    frame = envelope.frame_pose("support.carrier")
    np.testing.assert_allclose(frame[:, :3, :3], np.broadcast_to(quaternion_to_matrix(quaternion), (len(samples), 3, 3)), atol=1e-10)
    actual = envelope.body_state("wheel.wheel")[:, 3:7]
    expected = np.array([quaternion_multiply(quaternion, np.array([np.cos(speed*time/2), 0, np.sin(speed*time/2), 0])) for time in samples])
    np.testing.assert_allclose(np.abs(np.sum(actual*expected, axis=1)), 1, atol=1e-9)
    tire = model.to_document()["tires"][0]
    assert tire["parameters"]["frame_body"] == "support.carrier"


def test_contact_frame_follows_carrier_steering_without_inheriting_wheel_spin():
    support_data = carrier_subsystem().template.to_payload()
    support_data["bodies"] = [
        {"name": "stand", "fixed": True, "position": [0, 0, 1.334]},
        {"name": "carrier", "position": [0, 0, 1.334], "mass": 20, "inertia": np.eye(3).tolist()},
    ]
    support_data["hardpoints"].append({"name": "stand_center", "owner": "stand"})
    support_data["joints"] = [{"name": "pivot", "type": "revolute", "body_a": "stand", "body_b": "carrier",
                               "point_a": "stand_center", "point_b": "center", "axis": [0, 0, 1]}]
    support_data["coordinates"] = [{"name": "yaw", "joint": "pivot", "kind": "rotation"}]
    support = subsystem(TemplateDocument.from_payload(support_data),
                        {"center": [0, 0, 1.334], "stand_center": [0, 0, 1.334]})
    source = assembly({"support": support, "wheel": subsystem(generic_template("wheel"), {"center": [0, 0, 1.334]})})
    model = assemble_generic(source).resolved_model()
    samples = np.linspace(0, .04, 81)
    wheel_speed, yaw_speed = 300, 2
    run = plan("prescribed_angle", values=(wheel_speed*samples).tolist(), rates=[wheel_speed]*len(samples), samples=samples.tolist()).to_document()
    run["boundaries"].append({"name": "steer", "coordinate": "support.yaw", "mode": "prescribed_angle", "program": "yaw", "units": "rad"})
    run["inputs"].append({"name": "yaw", "values": (yaw_speed*samples).tolist(), "rates": [yaw_speed]*len(samples)})
    run["solver"] = {"initialization_mode": "provided_consistent_state", "internal_step_s": .00025}
    run["initial_state"] = {"support.carrier": {"omega": [0, 0, yaw_speed]},
                            "wheel.wheel": {"omega": [0, wheel_speed, yaw_speed]}}
    submitted = run_compiled(compile_resolved(model, ResolvedSolvePlan(run)))
    assert submitted.status == "success"
    result = submitted.result
    assert isinstance(result, ResultEnvelope)
    contact = result.frame_pose("support.carrier")[:, :3, :3]
    expected_contact = np.array([quaternion_to_matrix(np.array([np.cos(yaw_speed*time/2), 0, 0, np.sin(yaw_speed*time/2)]))
                                 for time in samples])
    np.testing.assert_allclose(contact, expected_contact, atol=1e-9)
    assert np.max(np.abs(result.body_state("wheel.wheel")[:, 3:7]-result.body_state("support.carrier")[:, 3:7])) > .5


def test_suspension_and_wheel_documents_are_identical_with_or_without_rig():
    points = {name: [0, .7, .334] for name in generic_template("suspension").hardpoint_names}
    points.update(upper_rear=[1, .7, .334], lower_rear=[1, .7, .334])
    support = carrier_subsystem()
    support_payload = support.template.payload
    support_payload["ports"][-1]["cardinality"] = "many"
    support = subsystem(TemplateDocument.from_payload(support_payload), {"center": [0, 0, .334]})
    suspension = subsystem(generic_template("suspension"), points)
    left = subsystem(generic_template("wheel"), {"center": [0, .7, .334]})
    right = subsystem(generic_template("wheel"), {"center": [0, -.7, .334]})
    subsystems = {"support": support, "suspension": suspension, "left": left, "right": right}
    pairings = {"left": {"carrier": "suspension.carrier_L"}, "right": {"carrier": "suspension.carrier_R"}}
    vehicle = assemble_generic(assembly(subsystems, pairings=pairings)).resolved_model()
    rig = subsystem(TemplateDocument.from_payload({
        "document": "template", "schema_version": 1, "name": "spin_rig", "functional_role": "generic",
        "allowed_placement_roles": ["any"], "bodies": [{"name": "fixture", "fixed": True}],
        "hardpoints": [], "joints": [], "elements": [], "property_slots": [],
        "needs": [{"name": "spin", "role": "wheel_spin", "kind": "spin", "units": "rad", "count": 1, "required": True}],
        "inputs": [{"name": "hold", "port": "@spin", "kind": "locked", "requires": ["spin"]}],
    }))
    bench = assemble_generic(assembly({**subsystems, "rig": rig}, pairings={**pairings,
        "rig": {"spin": "left.spin"}})).resolved_model()
    selected = tuple(row["name"] for row in vehicle.to_document()["bodies"])
    assert vehicle.subgraph_fingerprint(selected) == bench.subgraph_fingerprint(selected)
    assert bench.to_document()["signals"][0]["owner"] == "left.wheel"


def test_inconsistent_initial_velocity_is_rejected():
    from suspension_multibody.kernel import KernelContractError

    with pytest.raises(KernelContractError, match="initial.*velocity|velocity.*initial"):
        run_compiled(compiled_motion(omega=2))


def test_unknown_coordinate_and_alias_conflict_are_rejected():
    model = assemble_generic(wheel_assembly()).resolved_model()
    data = plan().to_document()
    data["boundaries"][0]["coordinate"] = "missing"
    with pytest.raises(ValueError, match="unknown joint coordinate"):
        resolve_motion_boundaries(model, ResolvedSolvePlan(data))
    data["boundaries"][0]["coordinate"] = "wheel.spin"
    data["boundaries"].append({"name": "other", "coordinate": "wheel.spin", "mode": "free"})
    with pytest.raises(ValueError, match="duplicate"):
        ResolvedSolvePlan(data)
