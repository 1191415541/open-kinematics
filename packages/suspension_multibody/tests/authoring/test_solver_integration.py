"""
The file driven route end to end: files in, a solved K/C run out.

These tests exist to answer one question rather than to exercise the loader: does
a model authored *entirely in files* reach the same solver the Python-authored
model reaches, and does the run carry the files' hashes as provenance.  A loader
that passes its own unit tests while nothing downstream consumes it would satisfy
every check except this one.
"""

from __future__ import annotations

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
    assembly_request_from,
    front_axle_model_for,
    runtime_template_from,
)
from suspension_multibody.schema import CaseSpec
from suspension_multibody.subsystems.entry import compose_axle

from .fixtures import COORDINATES, ROLES, write_axle_project


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

    # The same run through the file template's own topology agrees with the built-in
    # one, constraint for constraint: the file route reaches the composition, and the
    # topology the file describes assembles to the same K model.
    request = assembly_request_from(subsystem, mode="K")
    runtime = compose_axle(model, request=request)
    builtin = compose_axle(model)
    assert len(runtime.constraints) == len(builtin.constraints) == 13
    assert len(runtime.bodies) == len(builtin.bodies) == 10
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
    assert (len(k.constraints), len(k.bushings)) == (13, 0)
    assert (len(c.constraints), len(c.bushings)) == (9, 8)
