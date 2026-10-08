"""Offline migration produces ordinary documents and never imports a solver."""

import ast
import json
from pathlib import Path

import numpy as np
import pytest

from suspension_multibody.authoring import AssemblyDocument, assemble_generic, migration
from suspension_multibody.authoring.migration import (
    MigrationError,
    migrate_v1_axle,
    migrate_v1_case,
    migrate_v1_kc_case,
    migrate_v1_vehicle,
    save_migrated_assembly,
)
from suspension_multibody.compilation.resolved import compile_resolved
from suspension_multibody.modeling.resolved import ResolvedSolvePlan
from suspension_multibody.simulation import run_compiled


def explicit_source():
    return {"schema_version": 1, "name": "slider", "topology": "explicit",
        "hardpoints": {"A": [0, 0, 0]}, "mass": {"sprung_mass": 1},
        "bodies": [{"name": "slider", "mass": 2, "inertia": np.eye(3).tolist(),
            "pose": {"translation": [100, 0, 0]}, "center_of_mass": [10, 0, 0]}],
        "joints": [{"name": "guide", "kind": "prismatic", "body_a": "chassis", "body_b": "slider",
            "point_a": [100, 0, 0], "point_b": [100, 0, 0], "axis_a": [1, 0, 0], "axis_b": [1, 0, 0]}],
        "springs": [{"name": "spring", "body_a": "chassis", "body_b": "slider", "point_a": [0, 0, 0],
            "point_b": [100, 0, 0], "stiffness": 20, "free_length": 100}]}


@pytest.mark.parametrize("mode", ["K", "C"])
def test_kc_migration_declares_rig_and_preserves_quasistatic_poses(mode, tmp_path):
    import runpy

    from suspension_multibody.api import validate
    from suspension_multibody.report.metrics.case_specific import wheel_metrics
    from suspension_multibody.schema.model import AxleDeclaration

    root = Path(__file__).resolve().parents[2]
    source = AxleDeclaration.model_validate(json.loads((root/"tests/data/benchmark_axle.json").read_text())["model"])
    if mode == "C":
        source = runpy.run_path(str(root/"tests/cases/kc_quasi_static/kc_fixtures.py"))["_compliant_model"]()
    inputs = {"wheel_values_mm": (-10., 0., 10.), "rack_values_mm": (-5., 0., 5.)} if mode == "K" else {
        "paths": ("fx", "fy", "fz", "mx", "my", "mz"), "levels": 3, "maximum": 1.}
    settings = migration.AxleSolverSettings(position_tolerance_m=1e-12, dynamics_tolerance=1e-12, increment_tolerance=1e-12) if mode == "K" else migration.AxleSolverSettings()
    assembly, case = migrate_v1_kc_case(source, mode=mode, settings=settings, **inputs)
    compiled = validate(assembly, case)
    actual = run_compiled(compiled).result
    frozen = json.loads((root/"tests/data/kc_baseline"/(mode.lower()+"_states.json")).read_text())
    assert len(actual.cases) == (9 if mode == "K" else 18)
    if mode == "K":
        for row, reference in zip(actual.cases, frozen):
            end = int(row["sample_offset"])+int(row["sample_count"])
            class Sample:
                def frame_pose(self, frame_id):
                    return actual.frame_pose(frame_id)[:end]
            for side, stem in (("L", "left"), ("R", "right")):
                metrics = wheel_metrics(Sample(), "wheel.sub.json.wheel_center_"+side, side)
                for key in ("camber_deg", "toe_deg", "wheel_center_x_mm", "wheel_center_y_mm", "wheel_center_z_mm"):
                    assert metrics[stem+"_"+key.removesuffix("_mm")] == pytest.approx(reference[stem+"_"+key], abs=1e-7)
    assert compiled.metadata["compiler"] == "ResolvedModelCompiler"
    assert not assembly.entries[-1].subsystem.template.payload["bodies"]
    assert sum(row["type"] == "revolute" and "wheel_spin_joint" in row["name"]
        for row in compiled.model_document["joints"]) == 2
    assert sum(row["type"] == "driven_rotation" for row in compiled.model_document["joints"]) == 2
    saved = save_migrated_assembly(assembly, tmp_path)
    case.save(tmp_path/"case.json")
    from_file = validate(saved, tmp_path/"case.json")
    assert from_file.model_payload == compiled.model_payload
    assert from_file.case_payload == compiled.case_payload


def test_explicit_model_migrates_to_file_and_memory_with_identical_physics(tmp_path):
    source = explicit_source()
    original = json.dumps(source, sort_keys=True)
    document = migrate_v1_axle(source)
    saved = save_migrated_assembly(document, tmp_path)
    models = [assemble_generic(item).resolved_model() for item in (document, AssemblyDocument.load(saved))]
    assert models[0].fingerprint == models[1].fingerprint
    assert json.dumps(source, sort_keys=True) == original
    graph = models[0].to_document()
    np.testing.assert_allclose(graph["bodies"][1]["position"], [.11, 0, 0])
    np.testing.assert_allclose(graph["joints"][0]["point_b"], [-.01, 0, 0])
    np.testing.assert_allclose(graph["joints"][0]["axis_a"], [1, 0, 0])
    assert graph["elements"][0]["parameters"]["stiffness"] == 20000
    plan = ResolvedSolvePlan({"schema_version": 1, "name": "run", "study": "dynamic",
        "samples": [0, .001], "solver": {"initialization_mode": "provided_consistent_state"},
        "boundaries": [], "inputs": [], "outputs": []})
    run = run_compiled(compile_resolved(models[0], plan))
    assert run.status == "success"


def test_preload_migration_preserves_effective_length():
    source = explicit_source()
    source["springs"][0].pop("free_length")
    source["springs"][0].update(reference_length=100, preload=20)
    graph = assemble_generic(migrate_v1_axle(source)).resolved_model().to_document()
    assert graph["elements"][0]["parameters"]["free_length"] == .099


def test_signed_extension_curve_is_converted_to_native_compression():
    source = explicit_source()
    source["springs"][0]["force_curve"] = [[-20, -100], [0, 0], [10, 80]]
    graph = assemble_generic(migrate_v1_axle(source)).resolved_model().to_document()
    assert graph["elements"][0]["parameters"]["elastic_curve"] == [[-.01, -80], [0, 0], [.02, 100]]


def test_bushing_migration_preserves_legacy_local_orientation():
    source = explicit_source()
    source["bodies"][0]["pose"]["rotation"] = [2**-.5, 0, 0, 2**-.5]
    source["bushings"] = [{"name": "compliance", "body_a": "chassis", "body_b": "slider",
        "pose_a": {"translation": [100, 0, 0]}, "pose_b": {"translation": [100, 0, 0], "rotation": [1, 0, 0, 0]},
        "stiffness": np.eye(6).tolist(), "damping": [0]*6}]
    graph = assemble_generic(migrate_v1_axle(source, mode="C")).resolved_model().to_document()
    parameters = next(row["parameters"] for row in graph["elements"] if row["type"] == "bushing")
    assert parameters["frame_b_quaternion"] == [1, 0, 0, 0]
    np.testing.assert_allclose(parameters["point_b"], [-.01, 0, 0], atol=1e-15)


def test_offline_module_has_no_runtime_builder_or_solver_imports():
    tree = ast.parse(Path(migration.__file__).read_text(encoding="utf-8"))
    imports = [node.module or "" for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)]
    assert not any(word in name for name in imports for word in
        ("subsystems", "preparation", "simulation", "kernel", "compilation", "legacy"))


def test_missing_proxy_geometry_fails_explicitly():
    source = explicit_source()
    source["topology"] = "symmetric_proxy"
    source["joints"] = []
    with pytest.raises(MigrationError, match="missing hardpoint"):
        migrate_v1_axle(source)


def proxy_source():
    return {"schema_version": 1, "name": "proxy", "mass": {"sprung_mass": 1000},
        "hardpoints": {
            "UPPER_FRONT": [-100, -500, 400], "UPPER_REAR": [100, -500, 400], "UPPER_OUTER": [0, -700, 450],
            "LOWER_FRONT": [-120, -500, 150], "LOWER_REAR": [120, -500, 150], "LOWER_OUTER": [0, -700, 150],
            "TIE_INNER": [100, -400, 250], "TIE_OUTER": [50, -700, 250],
            "WHEEL_CENTER": [0, -700, 300], "RACK_CENTER": [0, 0, 250]}}


@pytest.mark.parametrize("mode,arm_joint_count", [("K", 2), ("C", 0)])
def test_proxy_migration_declares_topology_and_wheel_ownership(tmp_path, mode, arm_joint_count):
    source = proxy_source()
    original = json.dumps(source, sort_keys=True)
    document = migrate_v1_axle(source, mode=mode)
    wheel = next(entry for entry in document.entries if entry.functional_role == "wheel")
    assert {row["name"] for row in wheel.subsystem.template.payload["bodies"]} == {"wheel_hub_L", "wheel_hub_R"}
    assert len(wheel.subsystem.template.payload["coordinates"]) == 2
    resolved = assemble_generic(document).resolved_model()
    saved = save_migrated_assembly(document, tmp_path)
    assert resolved.fingerprint == assemble_generic(AssemblyDocument.load(saved)).resolved_model().fingerprint
    graph = resolved.to_document()
    for side in ("L", "R"):
        assert sum(row["body_b"] == f"model.sub.json.upper_arm_{side}" for row in graph["joints"]) == arm_joint_count // 2
    if mode == "C":
        assert len(graph["elements"]) == 8
        assert all(np.count_nonzero(row["parameters"]["stiffness"]) == 0 for row in graph["elements"])
    assert json.dumps(source, sort_keys=True) == original


def test_vertical_tire_and_bar_keep_their_declared_laws(tmp_path):
    source = proxy_source()
    source["tires"] = [{"stiffness": 200, "unloaded_radius": 300, "contact_point": [0, -700, 0]}]
    source["anti_roll_bars"] = [{"name": "bar", "torsional_stiffness": 17,
        "left_body_mount": [0, -500, 150], "right_body_mount": [0, 500, 150],
        "left_arm_end": [0, -700, 150], "right_arm_end": [0, 700, 150],
        "left_link_point": [0, -700, 150], "right_link_point": [0, 700, 150]}]
    document = migrate_v1_axle(source)
    graph = assemble_generic(document).resolved_model().to_document()
    assert len(graph["tires"]) == 2
    for tire in graph["tires"]:
        assert tire["body"].startswith("wheel.sub.json.")
        assert tire["parameters"]["frame_body"].startswith("model.sub.json.")
        assert tire["parameters"]["unloaded_radius"] == .3
        assert tire["parameters"]["vertical_stiffness"] == 200000
        assert tire["parameters"]["longitudinal_brush_stiffness"] == 1
    bar = next(row for row in graph["elements"] if row["name"].endswith(".bar"))
    assert bar["type"] == "bushing"
    assert bar["parameters"]["stiffness"][2][2] == 17000
    saved = save_migrated_assembly(document, tmp_path)
    assert assemble_generic(document).resolved_model().fingerprint == assemble_generic(AssemblyDocument.load(saved)).resolved_model().fingerprint


def test_proxy_modes_use_the_same_subsystem_definitions():
    documents = [migrate_v1_axle(proxy_source(), mode=mode) for mode in ("K", "C")]
    for left, right in zip(documents[0].entries, documents[1].entries):
        assert left.subsystem.template.topology_hash == right.subsystem.template.topology_hash
        assert left.subsystem.payload == right.subsystem.payload
    left = assemble_generic(documents[0]).resolved_model().to_document()
    right = assemble_generic(documents[1]).resolved_model().to_document()
    assert left["bodies"] == right["bodies"]
    assert left["coordinates"] == right["coordinates"]
    for graph in (left, right):
        bearing = next(row for row in graph["joints"] if row["name"].endswith("wheel_spin_joint_L"))
        assert bearing["type"] == "revolute"
        np.testing.assert_allclose(bearing["point_a"], [0, -.7, .3])
        np.testing.assert_allclose(bearing["point_b"], [0, 0, 0])


def test_declared_road_and_static_gauge_are_preserved():
    source = explicit_source()
    source["bodies"][0]["static_rotation_axis_local"] = [1, 0, 0]
    source["road"] = {"kind": "plane", "origin": [0, 0, 25]}
    document = migrate_v1_axle(source)
    assert document.payload["road"]["parameters"]["origin"] == [0, 0, .025]
    assert assemble_generic(document).resolved_model().to_document()["gauges"] == [
        {"body": "model.sub.json.slider", "axis_local": [1, 0, 0]}]


def test_vehicle_migration_uses_ordinary_subsystems_and_preserves_tire_values(full_vehicle_model, tmp_path):
    document = migrate_v1_vehicle(full_vehicle_model)
    model = assemble_generic(document).resolved_model()
    graph = model.to_document()
    saved = save_migrated_assembly(document, tmp_path)
    assert model.fingerprint == assemble_generic(AssemblyDocument.load(saved)).resolved_model().fingerprint
    assert len(graph["tires"]) == 4
    for tire in graph["tires"]:
        assert tire["body"].startswith("wheel_")
        assert tire["parameters"]["frame_body"].endswith(("upright_L", "upright_R"))
        assert tire["parameters"]["vertical_stiffness"] == 20000
    steering = next(row for row in graph["elements"] if row["type"] == "steering_actuator")
    assert steering["parameters"]["type"] == "translation"


def test_vehicle_demand_elements_bind_action_reaction_and_native_channels(full_vehicle_model):
    from suspension_multibody.schema.vehicle import DrivelineSpec

    model = full_vehicle_model.model_copy(update={"driveline": DrivelineSpec(
        torque_demand="both", driven_wheels=("rear_left", "rear_right"), drive_split=(0, 0, .5, .5))})
    graph = assemble_generic(migrate_v1_vehicle(model)).resolved_model().to_document()
    elements = [row for row in graph["elements"] if row["type"] == "rotational_torque"]
    assert len(elements) == 6
    for row in elements:
        assert row["body_a"] != row["body_b"]
        assert row["body_b"].startswith("wheel_")
        assert row["parameters"]["demand_source"] == (2 if row["name"].startswith("brake_") else 1)
        assert 0 <= row["parameters"]["demand_tire"] < 4


def test_vehicle_installed_frames_and_tire_mass_are_explicit(full_vehicle_model, tmp_path):
    from suspension_multibody.schema.common import Pose, Quaternion, Vec3

    wheels = tuple(wheel.model_copy(update={"tire_mass": 3, "pose": Pose(rotation=Quaternion(w=2**-.5, x=2**-.5)),
        "spin_axis": Vec3(y=1), "forward_axis": Vec3(x=1)}) for wheel in full_vehicle_model.wheels)
    model = full_vehicle_model.model_copy(update={"wheels": wheels})
    document = migrate_v1_vehicle(model)
    graph = assemble_generic(document).resolved_model().to_document()
    for tire in graph["tires"]:
        np.testing.assert_allclose(tire["parameters"]["spin_axis_local"], [0, 0, 1], atol=1e-15)
        np.testing.assert_allclose(tire["parameters"]["forward_axis_local"], [1, 0, 0], atol=1e-15)
        assert tire["mass"] == 3
        assert next(body["mass"] for body in graph["bodies"] if body["name"] == tire["body"]) == 20
    saved = save_migrated_assembly(document, tmp_path)
    assert assemble_generic(document).resolved_model().fingerprint == assemble_generic(AssemblyDocument.load(saved)).resolved_model().fingerprint


def test_vehicle_tire_brush_and_fiala_parameters_match_native_units(full_vehicle_model):
    from suspension_multibody.schema.dynamic import TireModelSpec

    spec = TireModelSpec(kind="fiala", unloaded_radius=300, longitudinal_stiffness=120000,
        cornering_stiffness=80000, relaxation_length=250,
        fiala_parameters={"CSLIP": 1500, "RELAX_LENGTH_X": 70, "WIDTH": 210})
    model = full_vehicle_model.model_copy(update={"wheels": tuple(wheel.model_copy(update={"tire": spec}) for wheel in full_vehicle_model.wheels)})
    resolved = assemble_generic(migrate_v1_vehicle(model)).resolved_model()
    graph = resolved.to_document()
    tire = graph["tires"][0]
    assert tire["parameters"]["longitudinal_brush_stiffness"] == 480000
    assert tire["parameters"]["lateral_brush_stiffness"] == 320000
    descriptor = next(row for row in graph["blobs"] if row["name"] == tire["parameters"]["blob"])
    coefficients = np.frombuffer(resolved.resource_payload[descriptor["offset"]:descriptor["offset"]+descriptor["length"]], dtype="<f8")
    assert coefficients[0] == 1500
    assert coefficients[7] == .07
    assert coefficients[8] == .25
    assert coefficients[9] == .21


def _legacy_entity_id(name):
    if name.startswith(("front_model.sub.json.", "front_wheel.sub.json.", "rear_model.sub.json.", "rear_wheel.sub.json.")):
        return name.split("_", 1)[0] + "_" + name.split(".json.", 1)[1]
    if name.startswith("body."):
        return name.removeprefix("body.")
    if name.startswith("wheel_"):
        return name.split(".", 1)[1]
    return name


def test_vehicle_entity_data_matches_existing_producer():
    from suspension_multibody.compilation.resolved import native_model_document
    from tests.vehicle.vehicle_fixtures import _positioned_vehicle, _vehicle

    reference = Path(__file__).parents[1]/"data/vehicle_dynamics_baseline/reference"
    old = json.loads((reference/"0.model.json").read_text())
    old_payload = (reference/"0.model.bin").read_bytes()
    migrated = assemble_generic(migrate_v1_vehicle(_positioned_vehicle(_vehicle()))).resolved_model()
    new = native_model_document(migrated)
    old_bodies = {row["name"]: row for row in old["bodies"]}
    new_bodies = {_legacy_entity_id(row["name"]): row for row in new["bodies"]}
    assert old_bodies.keys() == new_bodies.keys()
    for name, body in old_bodies.items():
        for key in ("mass", "inertia", "position", "quaternion", "fixed", "velocity", "omega"):
            np.testing.assert_array_equal(new_bodies[name].get(key, [0, 0, 0]), body[key])
    old_joints = {(row["body_a"], row["body_b"], row["type"]): row for row in old["joints"]}
    new_joints = {(_legacy_entity_id(row["body_a"]), _legacy_entity_id(row["body_b"]), row["type"]): row for row in new["joints"]}
    assert old_joints.keys() == new_joints.keys()
    assert len(old_joints) == len(old["joints"]) == len(new["joints"])
    for key, joint in old_joints.items():
        fields = ("point_a", "point_b") if joint["type"] in {"fixed", "spherical"} else ("point_a", "point_b", "axis_a", "axis_b")
        for field in fields:
            np.testing.assert_array_equal(new_joints[key][field], joint[field])
    steering_old = next(row["parameters"] for row in old["elements"] if row["type"] == "steering_actuator")
    steering_new = next(row["parameters"] for row in new["elements"] if row["type"] == "steering_actuator")
    for key, value in steering_old.items():
        actual = _legacy_entity_id(steering_new[key]) if key in {"body", "reaction_body"} else steering_new[key]
        if isinstance(value, str):
            assert actual == value
        else:
            np.testing.assert_array_equal(actual, value)
    for tire_old, tire_new in zip(old["tires"], new["tires"]):
        assert _legacy_entity_id(tire_new["body"]) == tire_old["body"]
        assert tire_new["model"] == tire_old["model"]
        for key, value in tire_old["parameters"].items():
            if key not in {"frame_body", "frame_center_local", "blob"}:
                np.testing.assert_array_equal(tire_new["parameters"][key], value)
        # The declared non-spinning carrier replaces the old spinning hub.
        assert tire_old["parameters"]["frame_body"].endswith(("wheel_hub_L", "wheel_hub_R"))
        assert tire_new["parameters"]["frame_body"].endswith(("upright_L", "upright_R"))
        descriptor = next(row for row in old["blobs"] if row["name"] == tire_old["parameters"]["blob"])
        assert len(old_payload[descriptor["offset"]:descriptor["offset"]+descriptor["length"]]) == descriptor["length"]
        assert tire_new["model"] == "native_brush"
        assert "blob" not in tire_new["parameters"]
    assert not new.get("blobs")
    assert migrated.resource_payload == b""


@pytest.mark.parametrize("family,section", [
    ("kc_quasi_static", {"k": {"wheel_values_mm": [0], "rack_values_mm": [], "axis_map": {"wheel": ["drive"]}}}),
    ("vehicle_kc", {"k": {"wheel_values_mm": [0], "rack_values_mm": [0], "axis_map": {"wheel": ["drive"], "rack": "rack"}}}),
    ("vehicle_dynamic", {"inputs": {"static_gauge": {"body": "support", "dof_mask": 3}}}),
    ("axle_dynamic", {}),
    ("handling", {"handling": {"steering": [{"actuator": "steering", "shape": "step", "amplitude": .1}]}}),
    ("ride_four_post", {"four_post": {"corners": [{"tire": "tire", "amplitude_m": .01}]}}),
    ("ride_random_road", {"ride_random_road": {"speed_mps": 10, "wheels": [{"tire": "tire", "components": []}]}}),
])
def test_native_case_migration_preserves_protocol_and_remaps_explicit_ids(family, section, tmp_path):
    from suspension_multibody.authoring.loader import CaseDocument

    source = {"contract": "multibody-case", "contract_version": 1, "kind": "case", "family": family,
        "name": "offline", "time": {"start_s": 0, "end_s": .002, "step_s": .001}, "solver": {}, **section}
    ids = {name: "declared."+name for name in ("drive", "rack", "support", "steering", "tire")}
    document = migrate_v1_case(source, entity_ids=ids)
    payload = document.to_payload()
    assert payload["samples"] == [0, .001, .002]
    assert payload["protocol"] == family
    assert payload["study"] == ("quasi_static" if family in {"kc_quasi_static", "vehicle_kc"} else "dynamic")
    assert "declared." in json.dumps(payload["excitation"]) or not section
    document.save(tmp_path/"case.json")
    assert CaseDocument(json.loads((tmp_path/"case.json").read_text())).to_payload() == payload
    assert source["time"]["step_s"] == .001


def test_sampled_case_migration_pins_irregular_grid_and_converts_dimensional_inputs():
    values = np.array([[2, 3, 4, 1000, 2000, 3000], [4, 5, 6, 2000, 3000, 4000]], dtype="<f8")
    times = np.array([0, .003], dtype="<f8")
    payload = times.tobytes() + values.tobytes()
    case = {"contract": "multibody-case", "contract_version": 1, "kind": "case", "family": "axle_dynamic",
        "name": "tables", "time": {"samples": "times"}, "solver": {}, "blobs": [
            {"name": "times", "role": "sample_times", "offset": 0, "length": times.nbytes, "shape": [2], "dtype": "float64"},
            {"name": "load", "role": "body_wrench", "body": "slider", "offset": times.nbytes, "length": values.nbytes, "shape": [2, 6], "dtype": "float64"}]}
    migrated = migrate_v1_case(case, entity_ids={"slider": "model.slider"}, input_payload=payload, length_scale=.001).to_payload()
    assert migrated["samples"] == [0, .003]
    row = migrated["inputs"][0]
    assert row["body"] == "model.slider"
    np.testing.assert_array_equal(row["values"], [[2, 3, 4, 1, 2, 3], [4, 5, 6, 2, 3, 4]])
    with pytest.raises(MigrationError, match="outside input payload"):
        migrate_v1_case(case, entity_ids={"slider": "model.slider"})


def test_case_migration_refuses_unmapped_target():
    case = {"contract": "multibody-case", "contract_version": 1, "kind": "case", "family": "vehicle_dynamic",
        "name": "unknown", "time": {"start_s": 0, "end_s": .001, "step_s": .001}, "solver": {},
        "inputs": {"static_gauge": {"body": "missing", "dof_mask": 1}}}
    with pytest.raises(MigrationError, match="unmapped entity"):
        migrate_v1_case(case, entity_ids={})


def test_case_migration_preserves_same_named_tables_for_distinct_wheels():
    values = np.array([1000, 2000, 3000, 4000], dtype="<f8")
    case = {"contract": "multibody-case", "contract_version": 1, "kind": "case", "family": "vehicle_dynamic",
        "name": "two_wheels", "time": {"start_s": 0, "end_s": .001, "step_s": .001}, "solver": {}, "blobs": [
            {"name": "torque", "role": "wheel_torque", "tire": target, "offset": index*16, "length": 16,
             "shape": [2], "dtype": "float64"} for index, target in enumerate(("left", "right"))]}
    migrated = migrate_v1_case(case, entity_ids={"left": "wheel.L", "right": "wheel.R"},
        input_payload=values.tobytes(), length_scale=.001).to_payload()
    rows = migrated["inputs"]
    assert len(rows) == len({row["name"] for row in rows}) == 2
    assert {row["tire"]: row["values"] for row in rows} == {"wheel.L": [1, 2], "wheel.R": [3, 4]}


def test_vehicle_case_migration_builds_si_plan_from_object_fixture(tmp_path):
    from suspension_multibody import api
    from suspension_multibody.authoring.migration import migrate_v1_vehicle_case
    from suspension_multibody.schema import TimeSignal
    from tests.vehicle.vehicle_fixtures import _case, _vehicle

    vehicle = _vehicle()
    assembly, case = migrate_v1_vehicle_case(_case(vehicle, steering=TimeSignal(times=(0, .001), values=(0, 1))))
    assert assembly.assembly_kind == "generic_multibody"
    payload = case.to_payload()
    assert payload["protocol"] == "vehicle_dynamic"
    assert len(payload["initial_state"]) == 29
    assert any(row["role"] == "steering_target" for row in payload["inputs"])
    assert any(row["role"] == "steering_rate" for row in payload["inputs"])
    compiled = api.validate(save_migrated_assembly(assembly, tmp_path), case.save(tmp_path/"vehicle.case.json"))
    run = run_compiled(compiled)
    assert run.status == "success"
    assert compiled.metadata["compiler"] == "ResolvedModelCompiler"


def test_vehicle_case_migration_matches_declared_excitation_and_initial_state():
    from suspension_multibody.authoring.migration import migrate_v1_vehicle_case
    from tests.vehicle.vehicle_fixtures import _case, _positioned_vehicle, _vehicle

    vehicle = _positioned_vehicle(_vehicle())
    source = _case(vehicle)
    reference = Path(__file__).parents[1]/"data/vehicle_dynamics_baseline/reference"
    old = json.loads((reference/"0.case.json").read_text())
    blob = (reference/"0.case.bin").read_bytes()
    old_model = json.loads((reference/"0.model.json").read_text())
    assembly, case = migrate_v1_vehicle_case(source)
    graph = assemble_generic(assembly).resolved_model().to_document()
    ids = {"body:"+_legacy_entity_id(row["name"]): row["name"] for row in graph["bodies"]}
    ids.update({row["name"].split(".", 1)[1]: row["name"] for row in graph["tires"]})
    ids.update({row["target"].split(".", 1)[1]: row["target"] for row in graph["elements"] if row["type"] == "steering_actuator"})
    migrated_native = migrate_v1_case(old, entity_ids=ids, input_payload=blob).to_payload()
    actual = case.to_payload()
    assert actual["solver"] == migrated_native["solver"]
    assert actual["samples"] == migrated_native["samples"]
    assert actual["excitation"] == migrated_native["excitation"]
    def inputs(rows):
        return {(row["role"], row.get("tire", row.get("actuator"))): row["values"] for row in rows}
    assert inputs(actual["inputs"]) == inputs(migrated_native["inputs"])
    for body in old_model["bodies"]:
        initial = actual["initial_state"][ids["body:"+body["name"]]]
        for key in ("position", "quaternion", "velocity", "omega"):
            np.testing.assert_array_equal(initial[key], body[key])


def test_vehicle_kc_migration_reuses_wheel_templates_and_binds_ordinary_rig(tmp_path):
    from suspension_multibody.api import validate
    from suspension_multibody.authoring import (
        migrate_v1_vehicle_case,
        migrate_v1_vehicle_kc_case,
    )
    from tests.vehicle.vehicle_fixtures import _case, _vehicle

    source = _case(_vehicle())
    original, _ = migrate_v1_vehicle_case(source)
    assembly, case = migrate_v1_vehicle_kc_case(source, wheel_values_mm=(0, 10), rack_values_mm=(0,),
        times_s=(0, .001, .002))
    base = {entry.ref: entry.subsystem for entry in original.entries}
    for entry in assembly.entries[:-1]:
        assert entry.subsystem.template.to_payload() == base[entry.ref].template.to_payload()
    fixture = assembly.entries[-1]
    assert not fixture.subsystem.template.payload["bodies"]
    assert len(fixture.subsystem.template.payload["joints"]) == 5
    assert len(case.to_payload()["boundaries"]) == 4
    assert all(row["mode"] == "locked" for row in case.to_payload()["boundaries"])
    compiled = validate(assembly, case)
    assert len(compiled.model_document["bodies"]) == 29
    assert sum(row["type"] == "driven_rotation" for row in compiled.model_document["joints"]) == 4
    assert not any(row["type"] == "steering_actuator" for row in compiled.model_document["elements"])
    saved = save_migrated_assembly(assembly, tmp_path)
    from_file = validate(saved, case.save(tmp_path/"case.json"))
    assert from_file.model_payload == compiled.model_payload
    assert from_file.case_payload == compiled.case_payload


def test_dynamic_si_axle_migration_preserves_declared_entities_and_loads():
    import runpy

    from suspension_multibody.authoring.migration import migrate_v1_dynamic_axle
    from suspension_multibody.compilation.resolved import native_model_document

    producer = runpy.run_path(str(Path(__file__).parents[2]/"scripts/run_axle_dynamics_acceptance.py"))
    model = producer["build_axle_model"]()
    case = producer["build_case"]("combined_load")
    assembly, plan = migrate_v1_dynamic_axle(model, case)
    graph = assemble_generic(assembly).resolved_model()
    document = native_model_document(graph)
    def original(name):
        return name.split(".", 1)[-1]
    assert len(document["bodies"]) == len(model.bodies)
    for row in document["bodies"]:
        expected = next(body for body in model.bodies if body.name == original(row["name"]))
        for field, attribute in (("mass", "mass_kg"), ("inertia", "inertia_kg_m2"),
            ("position", "position_m"), ("quaternion", "quaternion_body_to_world"),
            ("fixed", "fixed"), ("velocity", "linear_velocity_m_per_s"), ("omega", "angular_velocity_rad_per_s")):
            np.testing.assert_array_equal(row[field], getattr(expected, attribute))
    assert len(document["joints"]) == len(model.joints)
    for row in document["joints"]:
        expected = next(joint for joint in model.joints if joint.name == original(row["name"]))
        for field in ("body_a", "body_b"):
            assert original(row[field]) == getattr(expected, field)
        for field, attribute in (("point_a", "point_a_m"), ("point_b", "point_b_m"), ("axis_a", "axis_a"), ("axis_b", "axis_b")):
            np.testing.assert_array_equal(row[field], getattr(expected, attribute))
    source_elements = (*model.springs, *model.dampers, *model.bump_stops, *model.bushings, *model.anti_roll_bars)
    parameter_fields = {
        "spring": (("stiffness", "stiffness_n_per_m"), ("free_length", "free_length_m"), ("preload", "preload_n")),
        "damper": (("compression_damping", "compression_damping_n_s_per_m"), ("rebound_damping", "rebound_damping_n_s_per_m"),
            ("gas_stiffness", "gas_stiffness_n_per_m"), ("gas_reference_force", "gas_reference_force_n"),
            ("preload", "preload_n"), ("friction", "friction_n"), ("extension_sign", "extension_sign")),
        "bump_stop": (("clearance", "clearance_m"), ("stiffness", "stiffness_n_per_m"),
            ("direction", "direction"), ("damping", "damping_n_s_per_m")),
        "bushing": (("stiffness", "stiffness"), ("damping", "damping"), ("preload", "preload_in_frame_a_n_n_m"),
            ("frame_a_quaternion", "frame_a_to_body_quaternion"), ("frame_b_quaternion", "frame_b_to_body_quaternion"),
            ("reference_translation", "reference_translation_in_frame_a_m"), ("reference_quaternion", "reference_quaternion_a_to_b")),
        "anti_roll_bar": (("axis_a", "axis_a"), ("reference_quaternion", "reference_quaternion_a_to_b"),
            ("stiffness", "stiffness_n_m_per_rad"), ("damping", "damping_n_m_s_per_rad")),
    }
    assert len(document["elements"]) == len(source_elements)
    for row in document["elements"]:
        expected = next(element for element in source_elements if element.name == original(row["name"]))
        parameters = {key: np.asarray(getattr(expected, attribute)).tolist()
            for key, attribute in parameter_fields[row["type"]]}
        parameters.update({"point_a": list(getattr(expected, "point_a_m", (0, 0, 0))),
            "point_b": list(getattr(expected, "point_b_m", (0, 0, 0)))})
        assert row["parameters"] == parameters
    assert len(document["tires"]) == len(model.tires)
    for row in document["tires"]:
        expected = next(tire for tire in model.tires if tire.name == original(row["name"]))
        assert original(row["body"]) == expected.body
        assert row.get("mass", 0.0) == expected.mass_kg
        np.testing.assert_array_equal(row.get("inertia", np.zeros((3, 3))),
            expected.inertia_kg_m2 if expected.inertia_kg_m2 is not None else np.zeros((3, 3)))
        for field, attribute in (("center_local", "center_local_m"), ("spin_axis_local", "spin_axis_local"),
            ("forward_axis_local", "forward_axis_local"), ("unloaded_radius", "unloaded_radius_m"),
            ("maximum_compression", "maximum_compression_m"), ("vertical_stiffness", "vertical_stiffness_n_per_m"),
            ("vertical_damping", "vertical_damping_n_s_per_m"), ("longitudinal_friction_coefficient", "longitudinal_friction_coefficient"),
            ("lateral_friction_coefficient", "lateral_friction_coefficient"), ("longitudinal_brush_stiffness", "longitudinal_brush_stiffness_n_per_m"),
            ("lateral_brush_stiffness", "lateral_brush_stiffness_n_per_m"), ("longitudinal_relaxation_length", "longitudinal_relaxation_length_m"),
            ("lateral_relaxation_length", "lateral_relaxation_length_m"), ("detached_relaxation_s", "detached_relaxation_s")):
            np.testing.assert_array_equal(row["parameters"][field], getattr(expected, attribute))
    assert plan.to_payload()["samples"] == list(case.times_s)
    assert {row["body"]: row["values"] for row in plan.to_payload()["inputs"] if row["role"] == "body_wrench"} == {
        "sprung.sprung": [list(row) for row in case.body_wrench_n_n_m["sprung"]]}
    from suspension_multibody import api
    from suspension_multibody.simulation.runner import run_compiled
    compiled = api.validate(assembly, plan)
    assert run_compiled(compiled).status == "success"


def test_dynamic_si_axle_document_run_has_bit_identical_states_to_frozen_producer():
    import runpy

    from suspension_multibody import api
    from suspension_multibody.authoring.migration import migrate_v1_dynamic_axle
    from tests.axle_dynamics._unified_entry import solve_axle

    producer = runpy.run_path(str(Path(__file__).parents[2]/"scripts/run_axle_dynamics_acceptance.py"))
    model = producer["build_axle_model"]()
    case = producer["build_case"]("combined_load")
    previous = solve_axle(model, case)
    assembly, plan = migrate_v1_dynamic_axle(model, case)
    result = run_compiled(api.validate(assembly, plan)).result
    for body in model.bodies:
        np.testing.assert_array_equal(result.body_state(body.name+"."+body.name), previous.body_state(body.name))
    for tire in model.tires:
        np.testing.assert_array_equal(result.tire_state(tire.body+"."+tire.name), previous.tire_state(tire.name))
    np.testing.assert_array_equal(result.energy, previous.energy)


def test_pad_migration_drives_ground_without_duplicate_wheels(tmp_path):
    import runpy

    from suspension_multibody.api import validate

    fixture = runpy.run_path(str(Path(__file__).parents[4]/"scripts/acceptance_pad_driven_kc.py"))
    source = fixture["_pad_model"]()
    assembly, case = migrate_v1_kc_case(source, mode="C", pad_height_mm=(0., 10., 20.), drive_mode="pad")
    compiled = validate(assembly, case)
    assert not any("wheel_drive" in row.get("target", "") for row in compiled.model_document["joints"])
    assert len(compiled.model_document["tires"]) == 2
    result = run_compiled(compiled).result
    assert [row["name"] for row in result.cases] == ["pad+0", "pad+10", "pad+20"]
    for index, height in enumerate((0., 10., 20.)):
        last = int(result.cases[index]["sample_offset"])+int(result.cases[index]["sample_count"])-1
        for side in ("L", "R"):
            pose = result.frame_pose("wheel.sub.json.wheel_center_"+side)[last]
            tire = result.tire_state("wheel.sub.json.tire_0_"+side)[last]
            assert pose[2, 3]*1000-source.tires[0].unloaded_radius+tire[2]*1000 == pytest.approx(height, abs=1e-9)
    saved = save_migrated_assembly(assembly, tmp_path)
    from_file = validate(saved, case.save(tmp_path/"case.json"))
    assert from_file.model_payload == compiled.model_payload
    assert from_file.case_payload == compiled.case_payload
    with pytest.raises(MigrationError, match="without wheel/rack"):
        migrate_v1_kc_case(source, mode="C", pad_height_mm=(0.,), wheel_values_mm=(0.,))
