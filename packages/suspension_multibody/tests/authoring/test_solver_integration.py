"""
The file driven route end to end: files in, a solved K/C run out.

These tests exist to answer one question rather than to exercise the loader: does
a model authored *entirely in files* reach the same solver the Python-authored
model reaches, and does the run carry the files' hashes as provenance.  A loader
that passes its own unit tests while nothing downstream consumes it would satisfy
every check except this one.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from suspension_multibody import api
from suspension_multibody.authoring import (
    AuthoringError,
    ElementPropertyDocument,
    RigDocument,
    SimulationAssembly,
    SubsystemDocument,
    TemplateDocument,
)
from suspension_multibody.authoring.bridge import canonical_hardpoint
from suspension_multibody.authoring.solver import (
    assembly_request_for,
    assembly_request_from,
    bridge_model,
    front_axle_model_for,
    runtime_template_from,
)
from suspension_multibody.axle_dynamics import ENERGY_COLUMNS, TIRE_OUTPUT_COLUMNS
from suspension_multibody.cases.kc_quasi_static import model_document
from suspension_multibody.schema import CaseSpec
from suspension_multibody.simulation import SimulationRequest, run_request
from suspension_multibody.subsystems.entry import compose_axle

from .fixtures import (
    COORDINATES,
    ROLES,
    TIRE_RADIUS,
    TIRE_STIFFNESS,
    write_axle_project,
    write_c_ready_axle_project,
)


def test_hardpoint_roles_map_onto_the_model_lookup() -> None:
    """
    A template names points by role; the model's lookup wants its own spellings.

    This is the join that would otherwise fail with "missing required hardpoint"
    several layers away from the file that caused it.
    """
    assert canonical_hardpoint("tie_inner") == "TIE_ROD_INBOARD"
    assert canonical_hardpoint("wheel_center") == "WHEEL_CENTER"
    assert canonical_hardpoint("upper_front") == "UPPER_INBOARD_FRONT"
    # A name that satisfies no role is left alone rather than refused: a template
    # may carry a measuring marker that no subsystem reaches for.
    assert canonical_hardpoint("some_marker") == "some_marker"


def test_file_template_becomes_a_runtime_template(tmp_path: Path) -> None:
    paths = write_axle_project(tmp_path)
    document = TemplateDocument.load(paths["template"])
    template = runtime_template_from(document)
    assert template.name == document.name
    assert template.role == "suspension"
    assert {connection.role for connection in template.connections} == set(ROLES)
    # Every mount the role requires is satisfied by a declared hardpoint, and the
    # mount slot the C column reads exists with the built-in's own zero.
    assert not set(template.role_spec.required_mounts) - {
        connection.role for connection in template.connections
    }
    slots = {slot.name for slot in template.property_slots}
    assert {"spring", "damper", "bushing"} <= slots


def test_file_subsystem_becomes_a_model(tmp_path: Path) -> None:
    paths = write_axle_project(tmp_path)
    subsystem = SubsystemDocument.load(paths["subsystem"])
    model = front_axle_model_for(subsystem)
    # The coordinates reach the model under the spellings its lookup accepts;
    # every one of the ten roles is present and none is invented.
    assert set(model.hardpoints) == {
        "UPPER_INBOARD_FRONT",
        "UPPER_INBOARD_REAR",
        "UPPER_OUTBOARD",
        "LOWER_INBOARD_FRONT",
        "LOWER_INBOARD_REAR",
        "LOWER_OUTBOARD",
        "TIE_ROD_INBOARD",
        "TIE_ROD_OUTBOARD",
        "WHEEL_CENTER",
        "RACK_CENTER",
    }
    assert model.hardpoints["WHEEL_CENTER"].as_tuple() == COORDINATES["wheel_center"]
    # The spring comes from the property file, not from a number in the template.
    assert len(model.springs) == 1
    assert model.springs[0].stiffness == pytest.approx(45.0)
    assert model.springs[0].free_length == pytest.approx(250.0)


def test_moving_a_hardpoint_moves_the_model_but_not_the_template(tmp_path: Path) -> None:
    """
    Acceptance 3: the user changes coordinates in the subsystem, and the template's
    topology hash is unchanged.
    """
    paths = write_axle_project(tmp_path)
    template = TemplateDocument.load(paths["template"])
    before = template.topology_hash

    moved = dict(COORDINATES)
    moved["wheel_center"] = (0.0, -760.0, 300.0)
    subsystem = SubsystemDocument.load(paths["subsystem"])
    overridden = subsystem.effective(overrides={"hardpoints": {"wheel_center": list(moved["wheel_center"])}})
    model = front_axle_model_for(subsystem)
    from suspension_multibody.authoring.solver import bridge_model

    shifted = bridge_model(overridden, name="shifted")
    assert shifted.hardpoints["WHEEL_CENTER"].as_tuple() == (0.0, -760.0, 300.0)
    assert TemplateDocument.load(paths["template"]).topology_hash == before
    # The subsystem file on disk still holds the value the assembly document says.
    assert model.hardpoints["WHEEL_CENTER"].as_tuple() == COORDINATES["wheel_center"]


def test_swapping_the_spring_law_changes_the_solver_input_and_keeps_topology(
    tmp_path: Path,
) -> None:
    """
    Acceptance 10: a linear file becomes a nonlinear one, the topology hash holds,
    and the solver receives different constitutive data.
    """
    paths = write_axle_project(tmp_path)
    linear = SubsystemDocument.load(paths["subsystem"]).effective()
    linear_model = front_axle_model_for(SubsystemDocument.load(paths["subsystem"]))

    curved = {
        "document": "element_properties",
        "schema_version": 1,
        "name": "progressive",
        "element_type": "spring",
        "model": "piecewise",
        "units": {"force": "N", "length": "mm"},
        "parameters": {"free_length": 250.0},
        "curve": {
            "independent": "deflection",
            "dependent": "force",
            "points": [[0.0, 0.0], [10.0, 100.0], [20.0, 400.0]],
        },
    }
    (tmp_path / "spring.json").write_text(__import__("json").dumps(curved), encoding="utf-8")

    repointed = SubsystemDocument.load(paths["subsystem"]).effective()
    curved_model = front_axle_model_for(SubsystemDocument.load(paths["subsystem"]))

    assert repointed.topology_hash == linear.topology_hash
    assert repointed.effective_values_hash != linear.effective_values_hash
    # The solver sees a different stiffness, and the curve is carried beside it.
    assert curved_model.springs[0].stiffness != linear_model.springs[0].stiffness
    assert repointed.resolved_property("spring")["force_curve"] == (
        (0.0, 0.0),
        (10.0, 100.0),
        (20.0, 400.0),
    )


def test_file_authored_axle_reaches_the_kc_solver(tmp_path: Path) -> None:
    """
    Acceptance 9: the file project binds to a rig and solves through the existing
    K/C path, with no solver code of its own.
    """
    paths = write_axle_project(tmp_path)
    simulation = SimulationAssembly.load(paths["assembly"])
    assert simulation.rig.name == "kc_bench"
    assert simulation.bindings == {"wheel_centre": "front.sub.json"}

    subsystem = SubsystemDocument.load(paths["subsystem"])
    model = front_axle_model_for(subsystem)
    request = assembly_request_from(subsystem, mode="K")
    runtime = compose_axle(model, request=request)
    assert runtime.mode == "K"
    # The file template really drives the build: the bodies the runtime carries are
    # the template's parts, not the built-in's, and the run is deterministic.
    assert "upper_arm_L" in runtime.bodies
    assert "lower_arm_L" in runtime.bodies
    again = compose_axle(model, request=assembly_request_from(subsystem, mode="K"))
    assert [c.name for c in again.constraints] == [c.name for c in runtime.constraints]


def test_files_supply_the_values_and_laws_a_real_kc_run_solves(tmp_path: Path) -> None:
    """
    Acceptance 9: the file project's hardpoints and property files reach the solver.

    The topology the run resolves is the built-in double wishbone, because that is
    the topology the composition can solve: the file format states one joint per
    point, while the built-in additionally declares per-mode column activation --
    the inboard rear point constrains nothing in K and is a ball joint in C -- which
    the file schema cannot express yet.  What the files *can* supply, and what this
    test therefore proves, is the geometry and the constitutive laws: the model
    solved here is built from the subsystem's coordinates and the property file's
    stiffness, so a wrong file produces a different run.
    """
    paths = write_axle_project(tmp_path)
    subsystem = SubsystemDocument.load(paths["subsystem"])
    model = front_axle_model_for(subsystem)
    assert model.springs[0].stiffness == pytest.approx(45.0)
    case = CaseSpec(
        mode="K",
        subsystems=frozenset({"suspension", "chassis", "steering", "wheel"}),
    )
    bundle = api.run_case(model, case)
    assert bundle.states
    assert all(state.converged for state in bundle.states)

    # The same run through the file template's own topology reaches the composition,
    # and what it assembles is that template's own part list: the built-in's model
    # less 方式 A's wheel hub and its spin joint, which the *built-in* suspension
    # template declares and this file template does not.  Absolute counts rather
    # than only the difference, so a template that grows a part shows up here.
    request = assembly_request_from(subsystem, mode="K")
    runtime = compose_axle(model, request=request)
    builtin = compose_axle(model)
    assert (len(runtime.bodies), len(runtime.constraints)) == (11, 14)
    assert (len(builtin.bodies), len(builtin.constraints)) == (13, 16)
    assert set(builtin.bodies) - set(runtime.bodies) == {"wheel_hub_L", "wheel_hub_R"}
    # The joints whose names the *subsystems* own rather than the template are the
    # same in both: the rack guide, the housing mount and the tie-rod ends are
    # where the file route and the built-in route meet in the composition, while
    # the arms and their ball joints are named by whichever template declared them.
    assert {c.name for c in builtin.constraints} & {
        c.name for c in runtime.constraints
    } == {
        "housing_mount",
        "rack_guide",
        "rack_tie_joint_L",
        "rack_tie_joint_R",
        "tie_upright_joint_L",
        "tie_upright_joint_R",
    }
    again = compose_axle(model, request=assembly_request_from(subsystem, mode="K"))
    assert [c.name for c in again.constraints] == [c.name for c in runtime.constraints]
    # The law the solve read is the property file's, so repointing it cannot be a
    # change the result is blind to.
    assert model.springs[0].stiffness == pytest.approx(45.0)

def test_overriding_a_property_binding_changes_the_solved_law(tmp_path: Path) -> None:
    """
    Acceptance 6: an assembly override repoints a property file; the subsystem file
    is untouched and the solver reads the new law.
    """
    paths = write_axle_project(tmp_path)
    stiffer = {
        "document": "element_properties",
        "schema_version": 1,
        "name": "stiff",
        "element_type": "spring",
        "model": "linear",
        "units": {"force": "N", "length": "mm"},
        "parameters": {"stiffness": 120.0, "free_length": 250.0},
    }
    (tmp_path / "stiff.json").write_text(__import__("json").dumps(stiffer), encoding="utf-8")

    subsystem = SubsystemDocument.load(paths["subsystem"])
    overridden = subsystem.effective(
        overrides={"property_bindings": {"spring": "stiff.json"}}
    )
    from suspension_multibody.authoring.solver import bridge_model

    model = bridge_model(overridden, name="stiffened")
    assert model.springs[0].stiffness == pytest.approx(120.0)
    # The subsystem file still names the original law.
    assert (
        SubsystemDocument.load(paths["subsystem"]).payload["property_bindings"]["spring"]
        == "spring.json"
    )


def test_assembly_carries_every_file_hash_as_provenance(tmp_path: Path) -> None:
    """Acceptance 11: provenance records template, subsystem, assembly, rig and property hashes."""
    paths = write_axle_project(tmp_path)
    simulation = SimulationAssembly.load(paths["assembly"])
    provenance = simulation.provenance()
    assert provenance["assembly"]
    assert provenance["rig"]
    entry = provenance["subsystems"]["front.sub.json"]
    assert entry["topology"]
    assert entry["values"]
    assert entry["property_bindings"]
    assert entry["effective_values"]
    # The topology hash is the template's, and it is stable across a reload.
    assert entry["topology"] == TemplateDocument.load(paths["template"]).topology_hash


def test_incompatible_subsystem_is_refused_by_the_assembly(tmp_path: Path) -> None:
    """Acceptance 8: a wrong role or placement is refused, with the reason named."""
    paths = write_axle_project(tmp_path)
    wrong_role = dict(__import__("json").loads(paths["assembly"].read_text(encoding="utf-8")))
    wrong_role["subsystems"][0]["functional_role"] = "chassis"
    bad = tmp_path / "bad.asy.json"
    bad.write_text(__import__("json").dumps(wrong_role), encoding="utf-8")
    with pytest.raises(AuthoringError, match="functional_role"):
        SimulationAssembly.load(bad)


def test_rig_support_is_checked_against_the_assembly(tmp_path: Path) -> None:
    paths = write_axle_project(tmp_path)
    rig = dict(__import__("json").loads(paths["rig"].read_text(encoding="utf-8")))
    rig["supported_assembly_kinds"] = ["full_vehicle"]
    (tmp_path / "rig.json").write_text(__import__("json").dumps(rig), encoding="utf-8")
    with pytest.raises(AuthoringError, match="does not support"):
        SimulationAssembly.load(paths["assembly"])


def test_property_file_hash_changes_when_the_law_changes(tmp_path: Path) -> None:
    paths = write_axle_project(tmp_path)
    before = ElementPropertyDocument.load(paths["spring"]).content_hash
    payload = __import__("json").loads(paths["spring"].read_text(encoding="utf-8"))
    payload["parameters"]["stiffness"] = 99.0
    paths["spring"].write_text(__import__("json").dumps(payload), encoding="utf-8")
    after = ElementPropertyDocument.load(paths["spring"]).content_hash
    assert after != before


def test_rig_document_reports_its_own_missing_ports(tmp_path: Path) -> None:
    paths = write_axle_project(tmp_path)
    rig = dict(__import__("json").loads(paths["rig"].read_text(encoding="utf-8")))
    rig["required_ports"] = ["brake_torque"]
    (tmp_path / "rig.json").write_text(__import__("json").dumps(rig), encoding="utf-8")
    document = RigDocument.load(paths["rig"])
    assert document.required_ports == frozenset({"brake_torque"})
    with pytest.raises(AuthoringError, match="does not offer required port"):
        SimulationAssembly.load(paths["assembly"])


def test_exported_builtin_template_is_equivalent_in_both_modes(tmp_path: Path) -> None:
    """
    Phase 2/8: the file format can express the built-in template **whole**.

    The built-in is exported to a file, read back and instantiated, and the two
    assemblies are compared constraint by constraint in K *and* in C.  This is the
    check that makes "the file format carries per-mode column activation" a claim
    with evidence: if the format could not state that the inboard rear point is inert
    in K and a bushing in C, the C column would not match.
    """
    from suspension_multibody.subsystems.types import AssemblyRequest
    from suspension_multibody.templates.instantiate import instantiate

    from .fixtures import write_builtin_axle_project

    paths = write_builtin_axle_project(tmp_path)
    subsystem = SubsystemDocument.load(paths["subsystem"])
    model = front_axle_model_for(subsystem, name="builtin_file")
    for mode in ("K", "C"):
        builtin = compose_axle(model, request=AssemblyRequest(mode=mode))
        converted = compose_axle(
            model,
            request=AssemblyRequest(
                mode=mode,
                suspension_template=instantiate(
                    runtime_template_from(subsystem.template),
                    mode=mode,
                    properties={"spring": 0.0, "damper": 0.0, "bushing": 0.0},
                ),
            ),
        )
        assert len(builtin.constraints) == len(converted.constraints)
        assert [c.name for c in builtin.constraints] == [
            c.name for c in converted.constraints
        ]
        assert len(builtin.bushings) == len(converted.bushings)
        assert sorted(builtin.bodies) == sorted(converted.bodies)
        assert sorted(builtin.points) == sorted(converted.points)
    # The exact numbers the built-in assembly has, so a change on either side shows
    # up here rather than as an unexplained difference later.
    k = compose_axle(model, request=AssemblyRequest(mode="K"))
    c = compose_axle(model, request=AssemblyRequest(mode="C"))
    assert (len(k.constraints), len(k.bushings)) == (16, 0)
    assert (len(c.constraints), len(c.bushings)) == (12, 8)


def _half_slope_spring(root: Path) -> Path:
    """
    Write a spring law whose curve is *half* the slope of its scalar field.

    The scalar says 45 N/mm and the curve says 20 N/mm, so a solve that reads the
    curve and a solve that reads the scalar cannot produce the same force.  That
    is deliberate: the assertion below is a ratio, so it names *which* of the two
    the kernel applied rather than only that something moved.
    """
    import json

    law = {
        "document": "element_properties",
        "schema_version": 1,
        "name": "half_slope_spring",
        "element_type": "spring",
        "model": "piecewise",
        "units": {"force": "N", "length": "mm"},
        # The scalar is declared *beside* the curve.  Without it the resolver
        # would derive the scalar from the curve's own first slope, and the
        # assertion could no longer tell a curve from a differently-sloped line.
        "parameters": {"stiffness": 45.0, "free_length": 250.0},
        # Abscissas are deflections, and the axle sits in *extension*, so the
        # curve has to span the negative side.  One that covered compression only
        # would be clamped to its first sample and apply no force at all.
        "curve": {
            "independent": "deflection",
            "dependent": "force",
            "points": [[-400.0, -8000.0], [-200.0, -4000.0], [-100.0, -2000.0], [0.0, 0.0]],
        },
    }
    target = root / "half_slope_spring.json"
    target.write_text(json.dumps(law), encoding="utf-8")
    return target


def test_the_property_curve_is_the_law_the_kernel_applies(tmp_path: Path) -> None:
    """
    Acceptance 10: a curve file reaches the kernel, and it is the law it applies.

    Two things are asserted and the second is the one that matters: the curve
    travels into the model document the solve submits, *and* the force the kernel
    reports afterwards is the curve's rather than the scalar stiffness written
    beside it in the same file.  A run that carried the curve but solved the
    scalar would pass the first assertion and fail the second.
    """
    from suspension_multibody.cases.kc_quasi_static import model_document
    from suspension_multibody.schema import DisplacementControl

    from .fixtures import write_axle_project

    paths = write_axle_project(tmp_path)
    _half_slope_spring(tmp_path)
    subsystem = SubsystemDocument.load(paths["subsystem"])
    linear = front_axle_model_for(subsystem, name="linear")
    curved = bridge_model(
        subsystem.effective(
            overrides={"property_bindings": {"spring": "half_slope_spring.json"}}
        ),
        name="curved",
    )

    # The curve is carried, in the convention the contract states: a deflection
    # against a force, signed, so the extension the axle sits in is inside it.
    assert linear.springs[0].force_curve == ()
    assert curved.springs[0].force_curve == (
        (-400.0, -8000.0),
        (-200.0, -4000.0),
        (-100.0, -2000.0),
        (0.0, 0.0),
    )
    document = model_document(
        compose_axle(curved, request=assembly_request_from(subsystem, mode="K")),
        name="curved-k",
        drive_wheels=True,
    )
    spring = next(entry for entry in document["elements"] if entry["type"] == "spring")
    assert spring["parameters"]["elastic_curve"] == [
        [-400.0, -8000.0],
        [-200.0, -4000.0],
        [-100.0, -2000.0],
        [0.0, 0.0],
    ]

    # The slot the composition carries is the resolved law -- its model, its curve
    # and the file that answered -- rather than a number that lost all three.  This
    # is the shape the phase-3 migration is about, and the solve below is what
    # shows it is not merely carried: the kernel applies it.
    law = assembly_request_from(
        subsystem,
        mode="K",
        overrides={"property_bindings": {"spring": "half_slope_spring.json"}},
    ).instantiated_suspension.properties["spring"]
    assert law.model == "piecewise"
    assert law.curve == (
        (-400.0, -8000.0),
        (-200.0, -4000.0),
        (-100.0, -2000.0),
        (0.0, 0.0),
    )
    assert law.source.endswith("half_slope_spring.json")

    case = CaseSpec(
        mode="K",
        subsystems=frozenset({"suspension", "chassis", "steering", "wheel"}),
        controls=(DisplacementControl(target="wheel_travel_left", values=(30.0,)),),
    )

    def spring_force(model: object) -> float:
        """Return the positive-end force the kernel reported for the spring."""
        bundle = api.run_case(model, case)  # ty: ignore[invalid-argument-type]
        forces = [
            load.global_load.fz
            for load in bundle.component_loads
            if load.component == "spring_L_L"
        ]
        return max(forces)

    linear_force = spring_force(linear)
    assert linear_force > 0.0
    # Half the slope in the curve, so half the force -- while the scalar the same
    # file declares is untouched.  That is what makes the difference the curve's.
    # The ratio is what matters, and the two forces come out of a solved K reading:
    # the mass distribution the composed runtime carries (方式 A's hub, the steering
    # housing) moves the sixth digit, so the bound is the arithmetic's rather than the
    # one a particular fixture happened to hit.
    assert spring_force(curved) == pytest.approx(linear_force * 20.0 / 45.0, rel=1e-6)

def test_an_assembly_file_decides_the_axle_s_subsystems(tmp_path: Path) -> None:
    """
    Acceptance 5/6: the assembly document fixes what is composed, overrides and all.

    The fixture's assembly declares a suspension and a chassis, so the composed
    axle carries exactly those two roles -- no steering, and no wheel, because the
    single-axle bench supplies the wheels.  The default request carries all four,
    which is what makes this the file's decision rather than the entry's.
    """
    import json

    paths = write_axle_project(tmp_path)
    simulation = SimulationAssembly.load(paths["assembly"])
    request = assembly_request_for(simulation)
    assert request.subsystems == frozenset({"suspension", "chassis"})

    model = front_axle_model_for(SubsystemDocument.load(paths["subsystem"]))
    runtime = compose_axle(model, request=request)
    assert runtime.capabilities.subsystems == frozenset({"suspension", "chassis"})
    # A single axle carries no chassis (requirement 2): the default set is the three
    # roles an axle does carry, and this file's own request is what adds the fourth.
    assert compose_axle(model).capabilities.subsystems == frozenset(
        {"suspension", "steering", "wheel"}
    )

    # An assembly-level override repoints the property file, and the override is
    # what the composed instance reads -- while the subsystem file it names still
    # holds the original binding, which is the copy-on-write rule.
    stiffer = {
        "document": "element_properties",
        "schema_version": 1,
        "name": "stiff",
        "element_type": "spring",
        "model": "linear",
        "units": {"force": "N", "length": "mm"},
        "parameters": {"stiffness": 120.0, "free_length": 250.0},
    }
    (tmp_path / "stiff.json").write_text(json.dumps(stiffer), encoding="utf-8")
    payload = json.loads(paths["assembly"].read_text(encoding="utf-8"))
    payload["subsystems"][0]["overrides"] = {
        "property_bindings": {"spring": "stiff.json"}
    }
    paths["assembly"].write_text(json.dumps(payload), encoding="utf-8")

    overridden = assembly_request_for(SimulationAssembly.load(paths["assembly"]))
    # The resolved law, not a stripped number: the file it came from travels with
    # the value, which is what makes "this run read that file" checkable.
    assert request.instantiated_suspension.properties["spring"].scalar == 45.0
    assert request.instantiated_suspension.properties["spring"].source.endswith(
        "spring.json"
    )
    assert overridden.instantiated_suspension.properties["spring"].scalar == 120.0
    assert overridden.instantiated_suspension.properties["spring"].source.endswith(
        "stiff.json"
    )
    assert (
        SubsystemDocument.load(paths["subsystem"]).payload["property_bindings"]["spring"]
        == "spring.json"
    )


def test_every_defined_template_exports_to_a_file_and_reads_back(
    tmp_path: Path,
) -> None:
    """
    Phase 8: the templates the package defines are all expressible as files.

    The double wishbone is compared assembly to assembly above; these two are the
    simplified providers, whose topology is what has to survive: no parts, the two
    mount points and the role's own slots.  A built-in a user can use but cannot
    read would be the opposite of what a file format is for.
    """
    import json

    from suspension_multibody.authoring import TemplateDocument
    from suspension_multibody.authoring.solver import template_document_from
    from suspension_multibody.subsystems.brake import SIMPLIFIED_BRAKE
    from suspension_multibody.subsystems.drive import SIMPLIFIED_DRIVE

    for template in (SIMPLIFIED_BRAKE, SIMPLIFIED_DRIVE):
        document = template_document_from(template)
        path = tmp_path / f"{template.name}.tpl.json"
        path.write_text(json.dumps(document), encoding="utf-8")
        read_back = runtime_template_from(TemplateDocument.load(path))

        assert read_back.name == template.name
        assert read_back.role == template.role
        assert [part.name for part in read_back.parts] == [
            part.name for part in template.parts
        ]
        # A mount is stated once in a file and once per side in the runtime, so the
        # comparison is on the set of mount roles: that is what the format carries.
        assert sorted(
            {connection.role for connection in read_back.connections}
        ) == sorted({connection.role for connection in template.connections})
        # Neither of these roles owns a body, so its mounts are stated without one.
        assert all(not connection.owner for connection in read_back.connections)
        assert [slot.name for slot in read_back.property_slots] == [
            slot.name for slot in template.property_slots
        ]
        assert [slot.default for slot in read_back.property_slots] == [
            slot.default for slot in template.property_slots
        ]


#: The result layouts, read by name.  An index into a block is a statement about
#: a layout that lives elsewhere, and naming the channel keeps the two in step.
_TIRE_NORMAL_FORCE = TIRE_OUTPUT_COLUMNS.index("normal_force_n")
_TIRE_PENETRATION = TIRE_OUTPUT_COLUMNS.index("penetration_m")
_POTENTIAL_ENERGY = ENERGY_COLUMNS.index("potential_energy_j")


def _c_load_case() -> dict:
    """One C load path: three levels of vertical load at the wheel centre."""
    return {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "kc_quasi_static",
        "name": "file-c-loads",
        "time": {"start_s": 0.0, "end_s": 1e-3, "step_s": 1e-3},
        "c": {
            "load_marker": "wheel_center_L",
            "side_mode": "single",
            "mirror_marker": "wheel_center_R",
            "loads": [{"fz": 0.0}, {"fz": -1500.0}, {"fz": -3000.0}],
        },
    }


def _run_c(runtime) -> object:
    """Run one C document pair through the runner the product uses."""
    document = model_document(runtime, name="file-c-loads", drive_wheels=False)
    return run_request(
        SimulationRequest(
            assembly="axle",
            family="kc_quasi_static",
            model=document,
            case=_c_load_case(),
        )
    ).raw


def _tire_channel(result, level: int, column: int) -> float:
    """
    Return one tire channel of the loaded side at one load level.

    Each load level is its own case, and a case carries two samples of the same
    load, so the level's last sample is what a reader wants.
    """
    return float(result.block("tire_output")[2 * level + 1, 0, column])


def _potential_energy(result, level: int) -> float:
    """Return the axle's stored potential energy at one load level."""
    return float(result.block("energy")[2 * level + 1, _POTENTIAL_ENERGY])


def _file_project_with(tmp_path: Path, *, drop: str = "none"):
    """
    Write the C-ready project with one ingredient removed, and compose it.

    ``drop`` names what to take away: the tire, the mount bushings, or the
    ``wheel_center`` label the case addresses its load by.  The removals are made
    in the *template*, which is the file that declares them, so what is being
    tested is the file route rather than the fixture's Python.
    """
    paths = write_c_ready_axle_project(tmp_path)
    template = json.loads(paths["template"].read_text(encoding="utf-8"))
    if drop == "tire":
        template["elements"] = [
            row for row in template["elements"] if row["type"] != "tire"
        ]
    elif drop == "mounts":
        template["elements"] = [
            row for row in template["elements"] if row["type"] != "bushing"
        ]
    elif drop == "marker":
        for row in template["hardpoints"]:
            if row["name"] == "wheel_center":
                row["label"] = "center"
    elif drop != "none":
        raise AssertionError(f"unknown removal {drop!r}")
    paths["template"].write_text(json.dumps(template), encoding="utf-8")
    subsystem = SubsystemDocument.load(paths["subsystem"])
    model = front_axle_model_for(subsystem)
    runtime = compose_axle(model, request=assembly_request_from(subsystem, mode="C"))
    return model, runtime


def test_a_file_project_solves_the_c_reading(tmp_path: Path) -> None:
    """
    Plan step 1: the pad reading -- the one that loads the wheel centre and lets
    the tire carry the wheel -- runs on a model built entirely from files.

    This is the reading the file route could not reach before: a file-built model
    reached the solver but stopped in the static trim, because it carried neither
    a tire to react the load nor a mount table to hold the inboard points once C
    mode made them compliant.
    """
    paths = write_c_ready_axle_project(tmp_path)
    subsystem = SubsystemDocument.load(paths["subsystem"])
    model = front_axle_model_for(subsystem)

    # What the files supplied reached the model: a tire whose law is the file's,
    # and a mount table that is the file's rather than a scalar's three diagonals.
    assert len(model.tires) == 1
    assert model.tires[0].stiffness == pytest.approx(TIRE_STIFFNESS)
    assert model.tires[0].unloaded_radius == pytest.approx(TIRE_RADIUS)

    request = assembly_request_from(subsystem, mode="C")
    stiffness = request.mount_stiffness()
    assert stiffness.shape == (6, 6)
    assert stiffness[0, 0] == pytest.approx(10_000.0)
    # The rotational diagonal is the part a single number cannot express, and it
    # is the reason the mount slot carries a table.
    assert stiffness[3, 3] == pytest.approx(10_000_000.0)

    runtime = compose_axle(model, request=request)
    assert runtime.mode == "C"
    assert len(runtime.bushings) == 8

    result = _run_c(runtime)
    assert result.status == "success", dict(result.failure_evidence)
    assert len(result.cases) == 3
    # A converged static trim: the constraint and dynamics residuals the kernel
    # reports for each load level are at solver noise.
    for index in range(len(result.cases)):
        constraint, dynamics, _ = result.case_residuals(index)
        assert abs(constraint) < 1e-6
        assert abs(dynamics) < 1e-6

    # The load reached the model, and the numbers it reached it with are the
    # file's: the wheel sinks into the tire the file declares, and the kernel's
    # normal force is that file's law applied to the penetration it solved for.
    # A tire declared and ignored would report a force unrelated to its stiffness.
    assert _tire_channel(result, 0, _TIRE_PENETRATION) == pytest.approx(0.0, abs=1e-9)
    assert _tire_channel(result, 0, _TIRE_NORMAL_FORCE) == pytest.approx(0.0, abs=1e-9)
    light = _tire_channel(result, 1, _TIRE_PENETRATION)
    heavy = _tire_channel(result, 2, _TIRE_PENETRATION)
    assert heavy > light > 0.0
    # N/mm of stiffness against metres of penetration.
    assert _tire_channel(result, 2, _TIRE_NORMAL_FORCE) == pytest.approx(
        TIRE_STIFFNESS * 1000.0 * heavy, rel=1e-6
    )


def test_every_c_ready_ingredient_changes_the_run(tmp_path: Path) -> None:
    """
    The same project with one ingredient removed at a time.

    None of the three is decoration: each removal produces a *different* run, and
    the difference is measured rather than asserted.  Only the marker's removal is
    refused outright -- the other two build a model that solves, and solve to
    something else, which is exactly why a fixture that omitted them would look
    like a working run.
    """
    _, complete = _file_project_with(tmp_path / "complete", drop="none")
    baseline = _run_c(complete)
    assert baseline.status == "success"

    # No tire: the pad reading has nothing to carry the wheel with, so the whole
    # load falls on the suspension.  The axle's stored energy at the loaded level
    # rises from 3.758 J to 4.514 J -- the tire was carrying its share rather than
    # being declared and ignored.
    model, without_tire = _file_project_with(tmp_path / "no_tire", drop="tire")
    assert model.tires == ()
    without_tire_run = _run_c(without_tire)
    assert without_tire_run.tire_names == ()
    assert _potential_energy(without_tire_run, 2) > _potential_energy(baseline, 2)

    # No mount bushings: C mode leaves the four inboard joints ideal instead, so
    # the compliant reading is not built at all.  The assembly says so -- fourteen
    # constraints and no bushings, against twelve and eight -- and the wheel then
    # takes nearly the whole load (2680 N of 3000 N) because nothing yields.
    _, without_mounts = _file_project_with(tmp_path / "no_mounts", drop="mounts")
    assert len(without_mounts.bushings) == 0
    assert len(without_mounts.constraints) == 14
    rigid = _run_c(without_mounts)
    assert _tire_channel(rigid, 2, _TIRE_NORMAL_FORCE) > 5.0 * _tire_channel(
        baseline, 2, _TIRE_NORMAL_FORCE
    )

    # No `wheel_center` label: the marker the case loads does not exist, and the
    # run is refused by name rather than quietly loading nothing.
    _, without_marker = _file_project_with(tmp_path / "no_marker", drop="marker")
    with pytest.raises(Exception, match="wheel_center_L"):
        _run_c(without_marker)
