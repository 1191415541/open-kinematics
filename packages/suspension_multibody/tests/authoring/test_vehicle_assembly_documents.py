"""
The vehicle side of the file format: six roles, one rig, and the rules that say
which combinations are a vehicle.

The axle half of the project lives in `test_solver_integration.py`.  What is added
here is the second assembly kind -- two suspensions at two placements plus the four
single-instance roles -- and the two things it carries: the front/rear placement
rule, and a composition whose role set comes from the document rather than from a
constant.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from suspension_multibody.authoring import (
    WHEEL_ENDS,
    AssemblyDocument,
    AuthoringError,
    Project,
    SimulationAssembly,
    vehicle_document_from,
    vehicle_model_from,
)
from suspension_multibody.authoring.solver import assembly_request_for
from suspension_multibody.subsystems.entry import compose_vehicle

from .fixtures import COORDINATES, write_vehicle_project


def test_a_vehicle_project_loads_and_covers_four_wheel_ends(tmp_path: Path) -> None:
    """
    Acceptance 7: front and rear suspensions make a vehicle, and it is complete.

    The four wheel ends are read from the placements rather than from a count, so a
    vehicle whose two suspensions both sat at the front would be refused here even
    though it declares two suspensions.
    """
    paths = write_vehicle_project(tmp_path)
    simulation = SimulationAssembly.load(paths["vehicle_assembly"])

    assert simulation.assembly.assembly_kind == "full_vehicle"
    assert simulation.assembly.wheel_ends() == frozenset(WHEEL_ENDS)
    counts = simulation.assembly.role_counts()
    assert counts[("suspension", "front")] == 1
    assert counts[("suspension", "rear")] == 1
    for role in ("chassis", "steering", "wheel", "brake", "drive"):
        assert counts[(role, "any")] == 1
    # The rig is the file's, bound through the bench it names.
    assert simulation.rig.name == "vehicle_kc"
    assert simulation.channels == ("wheel_load", "rig_frame_pose")


def test_a_vehicle_without_a_brake_subsystem_is_refused(tmp_path: Path) -> None:
    """A car that cannot stop is a rule violation, not an assembly."""
    paths = write_vehicle_project(tmp_path)
    payload = json.loads(paths["vehicle_assembly"].read_text(encoding="utf-8"))
    payload["subsystems"] = [
        entry for entry in payload["subsystems"] if entry["functional_role"] != "brake"
    ]
    paths["vehicle_assembly"].write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(AuthoringError, match="brake"):
        SimulationAssembly.load(paths["vehicle_assembly"])


def test_a_second_front_suspension_is_refused(tmp_path: Path) -> None:
    """
    The rear placement is required, so moving the rear subsystem forward is refused.

    Which of the two checks reports it is not the point -- the assignment no longer
    agrees with the file it names, and saying so is what the check is for.
    """
    paths = write_vehicle_project(tmp_path)
    payload = json.loads(paths["vehicle_assembly"].read_text(encoding="utf-8"))
    for entry in payload["subsystems"]:
        if entry["ref"] == "rear.sub.json":
            entry["placement_role"] = "front"
    paths["vehicle_assembly"].write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(AuthoringError, match="placement_role|rear"):
        SimulationAssembly.load(paths["vehicle_assembly"])


def test_the_vehicle_document_decides_the_composed_vehicle_roles(
    tmp_path: Path, full_vehicle_model
) -> None:
    """
    Phase 5: the document's role set is what the vehicle composition carries, and
    its subsystems are what it builds.

    The role set comes from the file rather than from a constant, which is what
    makes the file route a route into the composition rather than a second
    composition.  The rows are the *document's* as well: a steering subsystem the
    document places supplies the topology of the axle it steers, so the composed
    constraint names are the file template's rather than the built-in's.
    """
    paths = write_vehicle_project(tmp_path)
    request = assembly_request_for(SimulationAssembly.load(paths["vehicle_assembly"]))
    assert request.subsystems == frozenset(
        {"suspension", "chassis", "steering", "wheel", "brake", "drive"}
    )
    assert request.steering_template is not None

    from_file = compose_vehicle(full_vehicle_model, "K", request)
    default = compose_vehicle(full_vehicle_model, "K")
    assert from_file.capabilities.subsystems == default.capabilities.subsystems

    from_file_names = {c.name for c in from_file.constraints}
    default_names = {c.name for c in default.constraints}
    # The document's own subsystems are the ones the composition reads, by name.
    assert request.steering_template.template.name == "file_steering"
    assert request.chassis_template.template.name == "file_chassis"
    # And the steering topology is the one that is built: the file names its
    # tie-rod joints `rack_tie` and `tie_upright`, and the composed vehicle carries
    # those names on the axle it steers.
    assert "front_rack_tie" in from_file_names
    assert "front_tie_upright" in from_file_names
    assert "front_rack_tie_joint_L" not in from_file_names
    assert "front_rack_tie_joint_L" in default_names
    # The file's steering subsystem declares a support body, and it is built.
    assert "front_support" in from_file.bodies
    # Both axles of *this* model have a free rack, so both steer and both read the
    # document's template; an axle the model bolts down keeps the built-in fixed
    # template instead, which `vehicle_model_from` states for a file vehicle.
    assert "rear_rack_tie" in from_file_names


def test_a_vehicle_model_can_take_its_axles_from_files(
    tmp_path: Path, full_vehicle_model
) -> None:
    """
    Phase 8: the compatibility conversion for the existing `VehicleModel`.

    The two axles come from the document's suspension files -- every hardpoint and
    every constitutive law -- while the wheels, the steering ratio and the
    driveline stay the template model's, because those are numbers the file format
    does not describe.  The composed result is a vehicle whose axles were authored
    in files, which is what the conversion exists for.
    """
    from suspension_multibody.authoring.solver import vehicle_model_with_file_axles

    from .fixtures import COORDINATES

    paths = write_vehicle_project(tmp_path)
    simulation = SimulationAssembly.load(paths["vehicle_assembly"])
    model = vehicle_model_with_file_axles(full_vehicle_model, simulation)

    assert model.front_axle.name == "front_full_vehicle"
    assert model.rear_axle.name == "rear_full_vehicle"
    assert model.front_axle.hardpoints["WHEEL_CENTER"].as_tuple() == COORDINATES[
        "wheel_center"
    ]
    # What the format does not describe is the template's, unchanged.
    assert model.wheels == full_vehicle_model.wheels
    assert model.steering == full_vehicle_model.steering
    assert model.driveline == full_vehicle_model.driveline

    runtime = compose_vehicle(model, "K", assembly_request_for(simulation))
    assert runtime.capabilities.subsystems == frozenset(
        {"suspension", "chassis", "steering", "wheel", "brake", "drive"}
    )
    assert any(name.startswith("front_upper_arm") for name in runtime.bodies)


def test_a_vehicle_document_builds_a_vehicle_model_from_its_own_numbers(
    tmp_path: Path,
) -> None:
    """
    Plan step 2: the vehicle-level numbers are the file's, not a template's.

    The chassis mass and inertia, the four wheel ends, the steering ratio and the
    driveline all come from the document's `vehicle` section, while the two axles
    come from its suspension subsystems -- so nothing about this model was handed
    in from Python except the document.
    """
    paths = write_vehicle_project(tmp_path)
    document = AssemblyDocument.load(paths["vehicle_assembly"])
    model = vehicle_model_from(document)

    assert model.name == "full_vehicle"
    assert model.chassis.mass == pytest.approx(1400.0)
    assert model.chassis.center_of_mass.z == pytest.approx(500.0)
    assert model.chassis.inertia[2][2] == pytest.approx(2600.0)
    assert [wheel.name for wheel in model.wheels] == [
        "front_left",
        "front_right",
        "rear_left",
        "rear_right",
    ]
    assert model.wheels[0].body == "wheel_front_left"
    assert model.wheels[0].tire.unloaded_radius == pytest.approx(300.0)
    assert model.steering.rack_body == "rack"
    assert model.steering.ratio == pytest.approx(1.0)
    assert model.driveline.driven_wheels == ("rear_left", "rear_right")
    assert model.driveline.maximum_drive_torque == pytest.approx(2000.0)

    # The axles are the document's subsystems, down to the mirrored side: a file
    # states one side and the composition builds both.
    assert {body.name for body in model.front_axle.bodies} == {
        "rack",
        "upper_arm_L",
        "upper_arm_R",
        "lower_arm_L",
        "lower_arm_R",
        "upright_L",
        "upright_R",
        "tie_rod_L",
        "tie_rod_R",
    }
    assert model.front_axle.hardpoints["WHEEL_CENTER"].as_tuple() == COORDINATES[
        "wheel_center"
    ]
    # One steering system is one steered axle: the rear rack is bolted down.
    assert model.front_axle.rack_fixed_to_chassis is False
    assert model.rear_axle.rack_fixed_to_chassis is True


def test_the_vehicle_numbers_survive_a_trip_through_the_file(tmp_path: Path) -> None:
    """
    The export is the inverse of the reader, and it is a fixed point.

    A round trip that only ran one way would be satisfied by a reader that filled
    the gaps with defaults; comparing the model to itself, and the export to
    itself, is what rules that out.
    """
    paths = write_vehicle_project(tmp_path)
    model = vehicle_model_from(AssemblyDocument.load(paths["vehicle_assembly"]))

    section = vehicle_document_from(model)
    payload = json.loads(paths["vehicle_assembly"].read_text(encoding="utf-8"))
    payload["vehicle"] = section
    paths["vehicle_assembly"].write_text(json.dumps(payload), encoding="utf-8")

    again = vehicle_model_from(AssemblyDocument.load(paths["vehicle_assembly"]))
    assert again == model
    assert vehicle_document_from(again) == section
    # The section is where the *vehicle* numbers live, and the axles are not in it:
    # they are the subsystems the assembly already refers to by file.
    assert set(section) == {"chassis", "wheels", "steering", "driveline"}


def test_a_full_vehicle_without_vehicle_numbers_is_refused(tmp_path: Path) -> None:
    """
    A vehicle model whose mass and wheel geometry were invented here would be a
    model nobody described, so the reader refuses instead of defaulting.
    """
    paths = write_vehicle_project(tmp_path)
    payload = json.loads(paths["vehicle_assembly"].read_text(encoding="utf-8"))
    del payload["vehicle"]
    paths["vehicle_assembly"].write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(AuthoringError, match="'vehicle' section"):
        vehicle_model_from(AssemblyDocument.load(paths["vehicle_assembly"]))

    # And an axle document is not a vehicle, however complete its subsystems are.
    axle = AssemblyDocument.load(paths["assembly"])
    with pytest.raises(AuthoringError, match="full_vehicle"):
        vehicle_model_from(axle)


def test_a_vehicle_whose_numbers_are_invalid_names_the_field(tmp_path: Path) -> None:
    """The bounds are the schema classes', and the failure says which one failed."""
    paths = write_vehicle_project(tmp_path)
    payload = json.loads(paths["vehicle_assembly"].read_text(encoding="utf-8"))
    payload["vehicle"]["steering"]["ratio"] = 0.0
    paths["vehicle_assembly"].write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(AuthoringError, match="steering"):
        vehicle_model_from(AssemblyDocument.load(paths["vehicle_assembly"]))


def test_a_file_vehicle_runs_the_vehicle_kc_reading(tmp_path: Path) -> None:
    """
    Plan step 2, end to end: a vehicle whose every number is a file solves.

    The reading is the package's own vehicle K/C family, unchanged, and the bodies
    it reports are the *file template's* parts -- `front_upper_arm_L`, not the
    built-in double wishbone's names -- so what ran is the document's vehicle
    rather than a template wearing its numbers.
    """
    from suspension_multibody.axle_dynamics.schema import AxleSolverSettings
    from suspension_multibody.cases import (
        vehicle_kc_case_document,
        vehicle_kc_model_document,
    )
    from suspension_multibody.preparation.vehicle_dynamic import prepare_vehicle_run
    from suspension_multibody.schema import (
        DynamicSolverSettings,
        RoadSurfaceSpec,
        TimeSignal,
        Vec3,
        VehicleDynamicCase,
    )
    from suspension_multibody.simulation import SimulationRequest, run_request

    paths = write_vehicle_project(tmp_path)
    model = vehicle_model_from(AssemblyDocument.load(paths["vehicle_assembly"]))
    solver = DynamicSolverSettings(
        end_time=0.001,
        step_size=0.001,
        internal_step_size=0.001,
        min_internal_step_size=0.001,
        adaptive_substepping=False,
        integrator="generalized_alpha",
        gravity=Vec3(x=0.0, y=0.0, z=0.0),
    )
    base = prepare_vehicle_run(
        model,
        VehicleDynamicCase(
            name="file-vehicle",
            vehicle=model,
            solver=solver,
            road=RoadSurfaceSpec(kind="plane"),
            steering_input=TimeSignal(constant=0.0),
            brake_input=TimeSignal(constant=0.0),
        ),
    )
    document = vehicle_kc_case_document(
        name="file-vehicle",
        wheel_values_mm=(0.0,),
        rack_values_mm=(0.0,),
        times_s=(0.0, 1e-3),
        settings=AxleSolverSettings(),
        left_right_mode="single",
    )
    model_document = vehicle_kc_model_document(model, base)
    run = run_request(
        SimulationRequest(
            assembly="vehicle",
            family="vehicle_kc",
            model=model_document,
            case=document,
            context={
                "model_document_pair": model_document,
                "wheels": (),
                "vehicle_assembly": base.assembly,
            },
        )
    ).raw

    assert run.status == "success", dict(run.failure_evidence)
    bodies = run.document["manifest"]["bodies"]
    assert "front_upper_arm_L" in bodies
    assert "wheel_front_left" in bodies


def test_the_project_loads_the_vehicle_and_its_numbers(tmp_path: Path) -> None:
    """The whole directory loads, and the vehicle assembly is one of its documents."""
    paths = write_vehicle_project(tmp_path)
    project = Project.load(tmp_path)
    assert sorted(project.sets["assembly"].documents) == ["front_axle", "full_vehicle"]
    assert project.sets["assembly"].path("full_vehicle") == paths["vehicle_assembly"]
    model = vehicle_model_from(AssemblyDocument.load(paths["vehicle_assembly"]))
    assert model.chassis.mass == pytest.approx(1400.0)


def test_a_bolted_rack_keeps_the_built_in_fixed_template(tmp_path: Path) -> None:
    """
    The model decides *whether* a rack is steered; the file decides *how*.

    `vehicle_model_from` bolts the rear axle's rack down, because the document
    declares one steering system.  That axle therefore keeps the built-in fixed
    template even though the document declares a steering subsystem -- while the
    front axle, which does steer, reads the document's own.
    """
    from suspension_multibody.authoring.solver import assembly_request_for
    from suspension_multibody.subsystems.entry import compose_vehicle

    paths = write_vehicle_project(tmp_path)
    document = AssemblyDocument.load(paths["vehicle_assembly"])
    model = vehicle_model_from(document)
    names = {
        constraint.name
        for constraint in compose_vehicle(
            model, "K", assembly_request_for(document)
        ).constraints
    }
    assert "front_rack_tie" in names
    assert "rear_rack_fixed_to_chassis" in names
    assert "rear_rack_tie" not in names
