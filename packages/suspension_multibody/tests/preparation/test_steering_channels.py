"""Each declared steering channel supplies its own element and input table."""

import json

import numpy as np
import pytest

from suspension_multibody.api import simulate, validate
from suspension_multibody.authoring import AssemblyDocument, vehicle_declaration_from
from suspension_multibody.authoring.loader import CaseDocument
from suspension_multibody.authoring.migration import (
    migrate_v1_vehicle_case,
    migrate_v1_vehicle_kc_case,
)
from suspension_multibody.authoring.steering_allocator import (
    AllocationChannel,
    allocate,
)
from suspension_multibody.schema import (
    DynamicSolverSettings,
    RoadSurfaceSpec,
    TimeSignal,
    Vec3,
    VehicleDynamicCase,
)
from tests.authoring.fixtures import write_vehicle_project


def _two_channel_model(tmp_path):
    paths = write_vehicle_project(tmp_path)
    payload = json.loads(paths["vehicle_assembly"].read_text(encoding="utf-8"))
    payload["vehicle"]["steering_channels"] = [{"rack_body": "rack", "ratio": 1.,
        "channel_name": "rear_rack", "placement": "rear"}]
    paths["vehicle_assembly"].write_text(json.dumps(payload), encoding="utf-8")
    return vehicle_declaration_from(AssemblyDocument.load(paths["vehicle_assembly"]))


def _case(model):
    return VehicleDynamicCase(name="two-channel", vehicle=model,
        solver=DynamicSolverSettings(end_time=.004, step_size=.001, internal_step_size=.001,
            min_internal_step_size=.001, adaptive_substepping=False, integrator="generalized_alpha",
            gravity=Vec3(z=-9806.65)), road=RoadSurfaceSpec(kind="plane"),
        steering_input=TimeSignal(times=(0., .004), values=(0., .02)))


def test_the_two_channel_document_states_both_channels(tmp_path):
    model = _two_channel_model(tmp_path)
    assert model.steering.channel_name == "front_rack"
    assert model.steering_channels[0].channel_name == "rear_rack"
    assert model.steering_channels[0].placement == "rear"


def test_the_compiler_builds_one_actuator_per_channel(tmp_path):
    model = _two_channel_model(tmp_path)
    source, case = migrate_v1_vehicle_case(_case(model))
    compiled = validate(source, case)
    steering = [row for row in compiled.model_document["elements"] if row["type"] == "steering_actuator"]
    assert [row["target"] for row in steering] == ["steering_front_rack.front_rack", "steering_rear_rack.rear_rack"]
    assert len({row["parameters"]["body"] for row in steering}) == 2
    for row in steering:
        params = row["parameters"]
        assert len(params["point_local"]) == len(params["reaction_point_local"]) == len(params["axis_local"]) == 3
        assert len(params["reference_quaternion"]) == 4
    targets = [row for row in case.to_payload()["inputs"] if row["role"] == "steering_target"]
    assert len(targets) == 2
    np.testing.assert_array_equal(targets[0]["values"], targets[1]["values"])
    assert targets[0]["values"][-1] == .02/1000


def test_direct_two_channel_run_drives_both_racks(tmp_path):
    run = simulate(*migrate_v1_vehicle_case(_case(_two_channel_model(tmp_path))))
    assert run.status == "success"
    for name in ("front_rack", "rear_rack"):
        assert run.result.element_state("steering_"+name+".actuator")[-1, 2] == pytest.approx(.02/1000, abs=1e-12)


@pytest.mark.parametrize("speed,gain", [(2., -.25), (20., .15)])
def test_four_wheel_steer_reaches_explicit_channel_tables(tmp_path, speed, gain):
    model = _two_channel_model(tmp_path)
    assembly, case = migrate_v1_vehicle_case(_case(model))
    channels = [AllocationChannel(name, placement, 2800., 1400., distance) for name, placement, distance in
        (("front_rack", "front", 0.), ("rear_rack", "rear", 2800.))]
    payload = case.to_payload()
    angles = [allocate("four_wheel_steer", channels, steer_input_rad=.02*t/.004, speed_mps=speed)
        for t in payload["samples"]]
    for row in payload["inputs"]:
        if row["role"] in {"steering_target", "steering_rate"}:
            index = 0 if row["actuator"].endswith("front_rack") else 1
            row["values"] = [item.angles_rad[index]/1000 for item in angles] if row["role"] == "steering_target" else [(.02/.004)*(1 if index == 0 else gain)/1000]*len(angles)
    compiled = validate(assembly, CaseDocument(payload))
    assert compiled.metadata["solve_plan"]["inputs"] == payload["inputs"]
    targets = [row for row in payload["inputs"] if row["role"] == "steering_target"]
    assert targets[0]["values"][-1] == .02/1000
    assert targets[1]["values"][-1] == gain*.02/1000
    assert (targets[1]["values"][-1] > 0) == (gain > 0)


def test_the_direct_law_preserves_a_one_channel_input(full_vehicle_model):
    source = _case(full_vehicle_model)
    _, case = migrate_v1_vehicle_case(source)
    payload = case.to_payload()
    targets = [row for row in payload["inputs"] if row["role"] == "steering_target"]
    assert len(targets) == 1
    np.testing.assert_allclose(targets[0]["values"], [source.steering_input.value_at(t)/1000 for t in payload["samples"]], rtol=1e-15, atol=0)


def test_two_channel_model_keeps_both_elements_and_kc_disables_them(tmp_path):
    model = _two_channel_model(tmp_path)
    original, case = migrate_v1_vehicle_case(_case(model))
    active = validate(original, case)
    assert sum(row["type"] == "steering_actuator" for row in active.model_document["elements"]) == 2
    assembly, grid = migrate_v1_vehicle_kc_case(_case(model), wheel_values_mm=(0.,), rack_values_mm=(0.,), times_s=(0., .001))
    compiled = validate(assembly, grid)
    assert sum(row["type"] == "steering_actuator" for row in compiled.model_document["elements"]) == 0
    assert any(row.get("target", "").endswith(".rack_drive") for row in compiled.model_document["joints"])
    for entry in original.entries:
        assert entry.subsystem.template.payload == next(row for row in assembly.entries if row.ref == entry.ref).subsystem.template.payload
