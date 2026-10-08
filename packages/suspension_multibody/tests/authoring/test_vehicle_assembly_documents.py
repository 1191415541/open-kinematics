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
    assemble_generic,
    migrate_v1_vehicle,
    vehicle_declaration_from,
    vehicle_document_from,
)
from suspension_multibody.authoring.bridge import axle_declaration_from

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
    document = AssemblyDocument.load(paths["vehicle_assembly"])
    assert frozenset(entry.functional_role for entry in document.entries) == frozenset(
        {"suspension", "chassis", "steering", "wheel", "brake", "drive"}
    )
    from_file = assemble_generic(migrate_v1_vehicle(vehicle_declaration_from(document)))
    default = assemble_generic(migrate_v1_vehicle(full_vehicle_model))
    from_file_names = {row["name"].rsplit(".", 1)[-1] for row in from_file.joints}
    default_names = {row["name"].rsplit(".", 1)[-1] for row in default.joints}
    # The document's own subsystems are the ones the composition reads, by name.
    assert next(entry for entry in document.entries if entry.functional_role == "steering").subsystem.template.name == "file_steering"
    assert next(entry for entry in document.entries if entry.functional_role == "chassis").subsystem.template.name == "file_chassis"
    # And the steering topology is the one that is built: the file declares a
    # `support` body for its rack to slide in, and that body is what the composed
    # vehicle carries.  The tie rod names are the *suspension* template's now
    # (requirement 1 -- the suspension owns the tie rods), so they appear on both
    # axles whichever steering template is read.
    assert "rack_guide" in from_file_names
    assert "rack_tie_joint_L" in from_file_names
    assert "rack_tie_joint_L" in default_names
    # The file's steering subsystem declares a support body, and it is built.
    assert any(name.startswith("front_") and name.endswith(".rack") for name in from_file.bodies)
    # Both axles of *this* model have a free rack, so both steer and both read the
    # document's template; an axle the model bolts down keeps the built-in fixed
    # template instead, which `vehicle_declaration_from` states for a file vehicle.
    assert any(name.startswith("rear_") and name.endswith(".rack") for name in from_file.bodies)


def test_a_vehicle_model_can_take_its_axles_from_files(
    tmp_path: Path, full_vehicle_model
) -> None:
    """
    Phase 8: the compatibility conversion for the existing `VehicleDeclaration`.

    The two axles come from the document's suspension files -- every hardpoint and
    every constitutive law -- while the wheels, the steering ratio and the
    driveline stay the template model's, because those are numbers the file format
    does not describe.  The composed result is a vehicle whose axles were authored
    in files, which is what the conversion exists for.
    """
    from .fixtures import COORDINATES

    paths = write_vehicle_project(tmp_path)
    document = AssemblyDocument.load(paths["vehicle_assembly"])
    axles = {entry.placement_role: axle_declaration_from(entry.effective(), name=entry.placement_role+"_full_vehicle")
        for entry in document.entries if entry.functional_role == "suspension"}
    model = full_vehicle_model.model_copy(update={"front_axle": axles["front"], "rear_axle": axles["rear"]})

    assert model.front_axle.name == "front_full_vehicle"
    assert model.rear_axle.name == "rear_full_vehicle"
    assert model.front_axle.hardpoints["WHEEL_CENTER"].as_tuple() == COORDINATES[
        "wheel_center"
    ]
    # What the format does not describe is the template's, unchanged.
    assert model.wheels == full_vehicle_model.wheels
    assert model.steering == full_vehicle_model.steering
    assert model.driveline == full_vehicle_model.driveline

    runtime = assemble_generic(migrate_v1_vehicle(model))
    assert any(name.startswith("front_") and name.endswith(".upper_arm_L") for name in runtime.bodies)


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
    model = vehicle_declaration_from(document)

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
    # A rack is bolted because its own subsystem file says so, not because a
    # vehicle declares one steering system.  Both of this fixture's suspension
    # files leave it free, and the reader returns what they state.
    assert model.front_axle.rack_fixed_to_chassis is False
    assert model.rear_axle.rack_fixed_to_chassis is (
        axle_declaration_from(next(entry for entry in document.entries
            if entry.functional_role == "suspension" and entry.placement_role == "rear").effective()).rack_fixed_to_chassis
    )


def test_the_vehicle_numbers_survive_a_trip_through_the_file(tmp_path: Path) -> None:
    """
    The export is the inverse of the reader, and it is a fixed point.

    A round trip that only ran one way would be satisfied by a reader that filled
    the gaps with defaults; comparing the model to itself, and the export to
    itself, is what rules that out.
    """
    paths = write_vehicle_project(tmp_path)
    model = vehicle_declaration_from(AssemblyDocument.load(paths["vehicle_assembly"]))

    section = vehicle_document_from(model)
    payload = json.loads(paths["vehicle_assembly"].read_text(encoding="utf-8"))
    payload["vehicle"] = section
    paths["vehicle_assembly"].write_text(json.dumps(payload), encoding="utf-8")

    again = vehicle_declaration_from(AssemblyDocument.load(paths["vehicle_assembly"]))
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
        vehicle_declaration_from(AssemblyDocument.load(paths["vehicle_assembly"]))

    # And an axle document is not a vehicle, however complete its subsystems are.
    axle = AssemblyDocument.load(paths["assembly"])
    with pytest.raises(AuthoringError, match="full_vehicle"):
        vehicle_declaration_from(axle)


def test_a_vehicle_whose_numbers_are_invalid_names_the_field(tmp_path: Path) -> None:
    """The bounds are the schema classes', and the failure says which one failed."""
    paths = write_vehicle_project(tmp_path)
    payload = json.loads(paths["vehicle_assembly"].read_text(encoding="utf-8"))
    payload["vehicle"]["steering"]["ratio"] = 0.0
    paths["vehicle_assembly"].write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(AuthoringError, match="steering"):
        vehicle_declaration_from(AssemblyDocument.load(paths["vehicle_assembly"]))


def test_a_file_vehicle_runs_the_vehicle_kc_reading(tmp_path: Path) -> None:
    """
    Plan step 2, end to end: a vehicle whose every number is a file solves.

    The reading is the package's own vehicle K/C family, unchanged, and the bodies
    it reports are the *file template's* parts -- `front_upper_arm_L`, not the
    built-in double wishbone's names -- so what ran is the document's vehicle
    rather than a template wearing its numbers.
    """
    from suspension_multibody.api import simulate
    from suspension_multibody.authoring.migration import migrate_v1_vehicle_kc_case
    from suspension_multibody.axle_dynamics.schema import AxleSolverSettings
    from suspension_multibody.schema import (
        DynamicSolverSettings,
        RoadSurfaceSpec,
        TimeSignal,
        Vec3,
        VehicleDynamicCase,
    )

    paths = write_vehicle_project(tmp_path)
    model = vehicle_declaration_from(AssemblyDocument.load(paths["vehicle_assembly"]))
    solver = DynamicSolverSettings(
        end_time=0.001,
        step_size=0.001,
        internal_step_size=0.001,
        min_internal_step_size=0.001,
        adaptive_substepping=False,
        integrator="generalized_alpha",
        gravity=Vec3(x=0.0, y=0.0, z=0.0),
    )
    base = VehicleDynamicCase(
            name="file-vehicle",
            vehicle=model,
            solver=solver,
            road=RoadSurfaceSpec(kind="plane"),
            steering_input=TimeSignal(constant=0.0),
            brake_input=TimeSignal(constant=0.0),
    )
    source, document = migrate_v1_vehicle_kc_case(
        base,
        name="file-vehicle",
        wheel_values_mm=(0.0,),
        rack_values_mm=(0.0,),
        times_s=(0.0, 1e-3),
        settings=AxleSolverSettings(),
        left_right_mode="single",
    )
    run = simulate(source, document).raw

    assert run.status == "success", dict(run.failure_evidence)
    bodies = run.document["manifest"]["bodies"]
    assert any(name.startswith("front_") and name.endswith(".upper_arm_L") for name in bodies)
    assert "wheel_front_left.wheel_front_left" in bodies


def test_the_project_loads_the_vehicle_and_its_numbers(tmp_path: Path) -> None:
    """The whole directory loads, and the vehicle assembly is one of its documents."""
    paths = write_vehicle_project(tmp_path)
    project = Project.load(tmp_path)
    assert sorted(project.sets["assembly"].documents) == ["front_axle", "full_vehicle"]
    assert project.sets["assembly"].path("full_vehicle") == paths["vehicle_assembly"]
    model = vehicle_declaration_from(AssemblyDocument.load(paths["vehicle_assembly"]))
    assert model.chassis.mass == pytest.approx(1400.0)


def test_the_rack_branch_follows_the_subsystem_files(tmp_path: Path) -> None:
    """
    The file decides *whether* a rack is bolted; the model only reads it.

    This test used to be `test_a_bolted_rack_keeps_the_built_in_fixed_template`
    and asserted `"rear_rack_fixed_to_chassis" in names`, because
    `vehicle_declaration_from` bolted the rear axle's rack down on the ground that a
    document declaring one steering system declares one *steered* axle.  Subtask
    p2-06 removed that override: a vehicle declares its steering channels
    explicitly, so bolting a rack the file left free would silently refuse the
    rear channel a document asked for.  Measured on this fixture -- whose rear
    suspension file leaves `rack_fixed_to_chassis` free, as `write_vehicle_project`
    writes it -- both axles now build the guide branch, which is what the files
    state.  The other branch is still built, and still covered, by
    `tests/subsystems/test_explicit_in_composition.py::112` and
    `tests/subsystems/test_assembly_matches_snapshot.py`, which drive it from the
    model field directly.

    What the templates decide is unchanged: the *guide* comes from each axle's
    own steering template (the front through the support that template declares,
    the rear through the same one placed at `rear`), the tie rods are the
    *suspension* template's (requirement 1), and neither steering template names
    one.
    """
    paths = write_vehicle_project(tmp_path)
    document = AssemblyDocument.load(paths["vehicle_assembly"])
    model = vehicle_declaration_from(document)
    names = {row["name"] for row in assemble_generic(migrate_v1_vehicle(model)).joints}
    assert any(name.startswith("front_") and name.endswith(".rack_guide") for name in names)
    assert any(name.startswith("rear_") and name.endswith(".rack_guide") for name in names)
    assert sum(name.endswith(".rack_tie_joint_L") for name in names) == 2
    assert not [name for name in names if name.endswith(".rack_tie")]
    # The branch is the *file's*: nothing bolts a rack the file left free.
    assert not [name for name in names if name.startswith("rear_") and name.endswith(".rack_fixed_to_chassis")]
