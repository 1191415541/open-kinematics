"""The file driven authoring layer: templates, subsystems, assemblies and rigs."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from suspension_multibody.authoring import (
    AssemblyDocument,
    AuthoringError,
    ElementPropertyDocument,
    RigDocument,
    SimulationAssembly,
    SubsystemDocument,
    TemplateDocument,
)


def _write(path: Path, payload: dict) -> Path:
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def _spring_file(root: Path, name: str = "spring.json", *, model: str = "linear") -> Path:
    """Write a spring property file: linear by default, curve when asked."""
    payload: dict = {
        "document": "element_properties",
        "schema_version": 1,
        "name": name,
        "element_type": "spring",
        "model": model,
        "units": {"force": "N", "length": "mm"},
    }
    if model == "linear":
        payload["parameters"] = {"stiffness": 45.0, "free_length": 250.0}
    else:
        # A curve law still states the spring's length: the kernel needs a
        # reference length to turn a deflection into a force, so the model's own
        # class requires exactly one length definition here too.
        payload["parameters"] = {"free_length": 250.0}
        payload["curve"] = {
            "independent": "deflection",
            "dependent": "force",
            "points": [[0.0, 0.0], [10.0, 100.0], [20.0, 400.0]],
        }
    return _write(root / name, payload)


def _damper_file(root: Path, name: str = "damper.json") -> Path:
    return _write(
        root / name,
        {
            "document": "element_properties",
            "schema_version": 1,
            "name": name,
            "element_type": "damper",
            "model": "linear",
            "units": {"force": "N", "length": "mm", "time": "s"},
            "parameters": {"viscous_damping": 12.0},
        },
    )


def _stop_file(root: Path, name: str = "stop.json") -> Path:
    return _write(
        root / name,
        {
            "document": "element_properties",
            "schema_version": 1,
            "name": name,
            "element_type": "bump_stop",
            "model": "linear",
            "units": {"force": "N", "length": "mm"},
            "parameters": {"stiffness": 500.0, "clearance": 20.0},
        },
    )


def _template(root: Path) -> Path:
    """
    One suspension template with a spring, a damper and a bump stop.

    Each elastic element refers to a property *slot* rather than to numbers, which
    is the point of the format: swapping the slot's file changes the law without
    changing this file, so its topology hash cannot move.
    """
    return _write(
        root / "suspension.tpl.json",
        {
            "document": "template",
            "schema_version": 1,
            "name": "basic",
            "functional_role": "suspension",
            "allowed_placement_roles": ["front", "rear"],
            "bodies": [{"name": "chassis", "fixed": True}, {"name": "arm"}],
            "hardpoints": [
                {"name": "mount", "owner": "chassis"},
                {"name": "wheel", "owner": "arm"},
            ],
            "joints": [
                {
                    "name": "pivot",
                    "type": "revolute",
                    "body_a": "chassis",
                    "body_b": "arm",
                    "point_a": "mount",
                    "point_b": "mount",
                }
            ],
            "elements": [
                {
                    "name": "spring",
                    "type": "spring",
                    "body_a": "chassis",
                    "body_b": "arm",
                    "point_a": "mount",
                    "point_b": "wheel",
                    "property_slot": "spring",
                },
                {
                    "name": "damper",
                    "type": "damper",
                    "body_a": "chassis",
                    "body_b": "arm",
                    "point_a": "mount",
                    "point_b": "wheel",
                    "property_slot": "damper",
                },
                {
                    "name": "stop",
                    "type": "bump_stop",
                    "body_a": "chassis",
                    "body_b": "arm",
                    "point_a": "mount",
                    "point_b": "wheel",
                    "property_slot": "stop",
                },
            ],
            "property_slots": [
                {
                    "name": "spring",
                    "element_type": "spring",
                    "allowed_models": ["linear", "piecewise"],
                    "required": True,
                },
                {
                    "name": "damper",
                    "element_type": "damper",
                    "allowed_models": ["linear"],
                    "required": True,
                },
                {
                    "name": "stop",
                    "element_type": "bump_stop",
                    "allowed_models": ["linear"],
                    "required": True,
                },
            ],
            "ports": [{"name": "wheel_centre", "role": "wheel_centre", "owner": "arm"}],
        },
    )


def _chassis_template(root: Path) -> Path:
    return _write(
        root / "chassis.tpl.json",
        {
            "document": "template",
            "schema_version": 1,
            "name": "chassis",
            "functional_role": "chassis",
            "allowed_placement_roles": ["any"],
            "bodies": [{"name": "chassis", "fixed": True}],
            "hardpoints": [{"name": "mount", "owner": "chassis"}],
            "joints": [],
            "elements": [],
            "property_slots": [],
        },
    )


def _binding(root: Path) -> dict[str, str]:
    """Bind the template's three slots to three files, writing them if needed."""
    _spring_file(root)
    _damper_file(root)
    _stop_file(root)
    return {"spring": "spring.json", "damper": "damper.json", "stop": "stop.json"}


def _suspension_subsystem(root: Path, name: str, placement: str) -> Path:
    return _write(
        root / f"{name}.sub.json",
        {
            "document": "subsystem",
            "schema_version": 1,
            "name": name,
            "template": "suspension.tpl.json",
            "functional_role": "suspension",
            "placement_role": placement,
            "hardpoints": {"mount": [1.0, 2.0, 3.0], "wheel": [4.0, 5.0, 6.0]},
            "property_bindings": _binding(root),
        },
    )


def _chassis_subsystem(root: Path) -> Path:
    return _write(
        root / "chassis.sub.json",
        {
            "document": "subsystem",
            "schema_version": 1,
            "name": "chassis",
            "template": "chassis.tpl.json",
            "functional_role": "chassis",
            "placement_role": "any",
            "hardpoints": {"mount": [0.0, 0.0, 0.0]},
            "property_bindings": {},
        },
    )


def test_template_and_subsystem_preserve_topology_hash(tmp_path: Path) -> None:
    _template(tmp_path)
    subsystem_path = _suspension_subsystem(tmp_path, "front", "front")
    template = TemplateDocument.load(tmp_path / "suspension.tpl.json")
    subsystem = SubsystemDocument.load(subsystem_path)
    assert subsystem.topology_hash == template.topology_hash
    assert subsystem.values_hash != template.topology_hash
    effective = subsystem.effective()
    assert effective.hardpoints["mount"] == (1.0, 2.0, 3.0)
    assert effective.topology_hash == template.topology_hash


def test_subsystem_cannot_change_topology(tmp_path: Path) -> None:
    _template(tmp_path)
    bad = _write(
        tmp_path / "bad.sub.json",
        {
            "document": "subsystem",
            "schema_version": 1,
            "name": "bad",
            "template": "suspension.tpl.json",
            "functional_role": "suspension",
            "placement_role": "front",
            "hardpoints": {},
            "property_bindings": {},
            "bodies": [{"name": "illegal"}],
        },
    )
    with pytest.raises(AuthoringError, match="unexpected fields"):
        SubsystemDocument.load(bad)


def test_subsystem_must_place_every_declared_hardpoint(tmp_path: Path) -> None:
    _template(tmp_path)
    _binding(tmp_path)
    incomplete = _write(
        tmp_path / "incomplete.sub.json",
        {
            "document": "subsystem",
            "schema_version": 1,
            "name": "incomplete",
            "template": "suspension.tpl.json",
            "functional_role": "suspension",
            "placement_role": "front",
            "hardpoints": {"mount": [1.0, 2.0, 3.0]},
            "property_bindings": {"spring": "spring.json", "damper": "damper.json", "stop": "stop.json"},
        },
    )
    with pytest.raises(AuthoringError, match="does not place"):
        SubsystemDocument.load(incomplete)


def test_property_file_checks_type_and_curve_order(tmp_path: Path) -> None:
    good = _spring_file(tmp_path, model="piecewise")
    assert ElementPropertyDocument.load(good).content_hash
    bad = _write(
        tmp_path / "bad.json",
        {
            "document": "element_properties",
            "schema_version": 1,
            "name": "bad",
            "element_type": "spring",
            "model": "piecewise",
            "units": {"force": "N", "length": "mm"},
            "curve": {
                "independent": "x",
                "dependent": "y",
                "points": [[1.0, 1.0], [1.0, 2.0]],
            },
        },
    )
    with pytest.raises(AuthoringError, match="strictly increasing"):
        ElementPropertyDocument.load(bad)


def test_property_file_type_must_match_the_slot(tmp_path: Path) -> None:
    """A spring slot bound to a damper file is a wrong model, not a rounding difference."""
    _template(tmp_path)
    damper = _damper_file(tmp_path)
    mismatched = _write(
        tmp_path / "mismatch.sub.json",
        {
            "document": "subsystem",
            "schema_version": 1,
            "name": "mismatch",
            "template": "suspension.tpl.json",
            "functional_role": "suspension",
            "placement_role": "front",
            "hardpoints": {"mount": [1.0, 2.0, 3.0], "wheel": [4.0, 5.0, 6.0]},
            "property_bindings": {
                "spring": damper.name,
                "damper": damper.name,
                "stop": "stop.json",
            },
        },
    )
    _stop_file(tmp_path)
    with pytest.raises(AuthoringError, match="expected element_type"):
        SubsystemDocument.load(mismatched).effective()


def test_missing_required_property_binding_is_refused(tmp_path: Path) -> None:
    _template(tmp_path)
    _spring_file(tmp_path)
    unbound = _write(
        tmp_path / "unbound.sub.json",
        {
            "document": "subsystem",
            "schema_version": 1,
            "name": "unbound",
            "template": "suspension.tpl.json",
            "functional_role": "suspension",
            "placement_role": "front",
            "hardpoints": {"mount": [1.0, 2.0, 3.0], "wheel": [4.0, 5.0, 6.0]},
            "property_bindings": {"spring": "spring.json"},
        },
    )
    with pytest.raises(AuthoringError, match="required property slot"):
        SubsystemDocument.load(unbound).effective()


def test_swapping_linear_for_curve_keeps_topology_and_changes_values(tmp_path: Path) -> None:
    """
    Acceptance 10: the topology hash stays, the property and value hashes move.

    This is the property that makes a constitutive file a *file* rather than an
    inline number: the law is exchanged while the model's structure is not.
    """
    _template(tmp_path)
    linear = _suspension_subsystem(tmp_path, "front", "front")
    before = SubsystemDocument.load(linear).effective()

    _spring_file(tmp_path, "spring_curve.json", model="piecewise")
    repointed = _write(
        tmp_path / "front.sub.json",
        {
            "document": "subsystem",
            "schema_version": 1,
            "name": "front",
            "template": "suspension.tpl.json",
            "functional_role": "suspension",
            "placement_role": "front",
            "hardpoints": {"mount": [1.0, 2.0, 3.0], "wheel": [4.0, 5.0, 6.0]},
            "property_bindings": {
                "spring": "spring_curve.json",
                "damper": "damper.json",
                "stop": "stop.json",
            },
        },
    )
    after = SubsystemDocument.load(repointed).effective()

    assert after.topology_hash == before.topology_hash
    assert after.property_bindings_hash != before.property_bindings_hash
    assert after.effective_values_hash != before.effective_values_hash
    # The solver receives the curve, and the scalar slope that backs it.
    resolved = after.resolved_property("spring")
    assert resolved["force_curve"] == ((0.0, 0.0), (10.0, 100.0), (20.0, 400.0))
    assert resolved["stiffness"] == pytest.approx(10.0)


def test_assembly_override_does_not_mutate_subsystem(tmp_path: Path) -> None:
    _template(tmp_path)
    _chassis_template(tmp_path)
    _spring_file(tmp_path, "b.json")
    _suspension_subsystem(tmp_path, "front", "front")
    _chassis_subsystem(tmp_path)
    assembly = _write(
        tmp_path / "axle.asy.json",
        {
            "document": "assembly",
            "schema_version": 1,
            "name": "axle",
            "assembly_kind": "suspension_axle",
            "subsystems": [
                {
                    "ref": "front.sub.json",
                    "functional_role": "suspension",
                    "placement_role": "front",
                    "overrides": {
                        "hardpoints": {"mount": [4.0, 5.0, 6.0]},
                        "property_bindings": {"spring": "b.json"},
                    },
                },
                {"ref": "chassis.sub.json", "functional_role": "chassis", "placement_role": "any"},
            ],
        },
    )
    loaded = AssemblyDocument.load(assembly)
    effective = loaded.entries[0].effective()
    assert effective.hardpoints["mount"] == (4.0, 5.0, 6.0)
    assert effective.property_bindings["spring"].path.name == "b.json"
    # The subsystem file on disk is untouched: the override is copy-on-write.
    assert SubsystemDocument.load(tmp_path / "front.sub.json").payload["hardpoints"]["mount"] == [
        1.0,
        2.0,
        3.0,
    ]
    assert loaded.role_counts() == {("suspension", "front"): 1, ("chassis", "any"): 1}


def test_assembly_cannot_override_topology(tmp_path: Path) -> None:
    _template(tmp_path)
    _chassis_template(tmp_path)
    _suspension_subsystem(tmp_path, "front", "front")
    _chassis_subsystem(tmp_path)
    assembly = _write(
        tmp_path / "axle.asy.json",
        {
            "document": "assembly",
            "schema_version": 1,
            "name": "axle",
            "assembly_kind": "suspension_axle",
            "subsystems": [
                {
                    "ref": "front.sub.json",
                    "functional_role": "suspension",
                    "placement_role": "front",
                    "overrides": {"bodies": [{"name": "illegal"}]},
                },
                {"ref": "chassis.sub.json", "functional_role": "chassis", "placement_role": "any"},
            ],
        },
    )
    with pytest.raises(AuthoringError, match="may only restate"):
        AssemblyDocument.load(assembly)


def test_axle_forbids_brake_and_drive(tmp_path: Path) -> None:
    _template(tmp_path)
    _chassis_template(tmp_path)
    _suspension_subsystem(tmp_path, "front", "front")
    _chassis_subsystem(tmp_path)
    assembly = _write(
        tmp_path / "axle.asy.json",
        {
            "document": "assembly",
            "schema_version": 1,
            "name": "axle",
            "assembly_kind": "suspension_axle",
            "subsystems": [
                {"ref": "front.sub.json", "functional_role": "suspension", "placement_role": "front"},
                {"ref": "chassis.sub.json", "functional_role": "chassis", "placement_role": "any"},
                {"ref": "chassis.sub.json", "functional_role": "brake", "placement_role": "any"},
            ],
        },
    )
    with pytest.raises(AuthoringError, match="forbids"):
        AssemblyDocument.load(assembly)


def test_full_vehicle_requires_both_suspensions_and_chassis(tmp_path: Path) -> None:
    _template(tmp_path)
    _suspension_subsystem(tmp_path, "front", "front")
    _suspension_subsystem(tmp_path, "rear", "rear")
    assembly = _write(
        tmp_path / "vehicle.asy.json",
        {
            "document": "assembly",
            "schema_version": 1,
            "name": "vehicle",
            "assembly_kind": "full_vehicle",
            "subsystems": [
                {"ref": "front.sub.json", "functional_role": "suspension", "placement_role": "front"},
                {"ref": "rear.sub.json", "functional_role": "suspension", "placement_role": "rear"},
            ],
        },
    )
    with pytest.raises(AuthoringError, match="exactly one chassis"):
        AssemblyDocument.load(assembly)


def test_full_vehicle_rejects_two_suspensions_at_one_placement(tmp_path: Path) -> None:
    """
    Two suspensions at the front is the shape rule's to refuse.

    What the rule fixes is not "a vehicle has two axles" -- a three-axle truck has
    three -- but that one placement names one subsystem.  A file that names the front
    axle twice does not say which suspension a front wheel came from, and the message
    has to be that rather than a count.
    """
    _template(tmp_path)
    _chassis_template(tmp_path)
    _suspension_subsystem(tmp_path, "front", "front")
    _suspension_subsystem(tmp_path, "front2", "front")
    _chassis_subsystem(tmp_path)
    assembly = _write(
        tmp_path / "vehicle.asy.json",
        {
            "document": "assembly",
            "schema_version": 1,
            "name": "vehicle",
            "assembly_kind": "full_vehicle",
            "subsystems": [
                {"ref": "front.sub.json", "functional_role": "suspension", "placement_role": "front"},
                {"ref": "front2.sub.json", "functional_role": "suspension", "placement_role": "front"},
                {"ref": "chassis.sub.json", "functional_role": "chassis", "placement_role": "any"},
            ],
        },
    )
    with pytest.raises(AuthoringError, match="more than once"):
        AssemblyDocument.load(assembly)


def test_placement_mismatch_is_reported(tmp_path: Path) -> None:
    _template(tmp_path)
    _chassis_template(tmp_path)
    _suspension_subsystem(tmp_path, "front", "front")
    _chassis_subsystem(tmp_path)
    assembly = _write(
        tmp_path / "axle.asy.json",
        {
            "document": "assembly",
            "schema_version": 1,
            "name": "axle",
            "assembly_kind": "suspension_axle",
            "subsystems": [
                {"ref": "front.sub.json", "functional_role": "suspension", "placement_role": "rear"},
                {"ref": "chassis.sub.json", "functional_role": "chassis", "placement_role": "any"},
            ],
        },
    )
    with pytest.raises(AuthoringError, match="placement_role"):
        AssemblyDocument.load(assembly)


def test_simulation_assembly_checks_rig_kind(tmp_path: Path) -> None:
    _template(tmp_path)
    _chassis_template(tmp_path)
    _suspension_subsystem(tmp_path, "front", "front")
    _chassis_subsystem(tmp_path)
    _write(
        tmp_path / "rig.json",
        {
            "document": "rig",
            "schema_version": 1,
            "name": "vehicle-only",
            "supported_assembly_kinds": ["full_vehicle"],
            "required_ports": [],
        },
    )
    assembly = _write(
        tmp_path / "axle.asy.json",
        {
            "document": "assembly",
            "schema_version": 1,
            "name": "axle",
            "assembly_kind": "suspension_axle",
            "subsystems": [
                {"ref": "front.sub.json", "functional_role": "suspension", "placement_role": "front"},
                {"ref": "chassis.sub.json", "functional_role": "chassis", "placement_role": "any"},
            ],
            "rig": "rig.json",
        },
    )
    with pytest.raises(AuthoringError, match="does not support"):
        SimulationAssembly.load(assembly)


def test_simulation_assembly_binds_ports_and_records_provenance(tmp_path: Path) -> None:
    _template(tmp_path)
    _chassis_template(tmp_path)
    _suspension_subsystem(tmp_path, "front", "front")
    _chassis_subsystem(tmp_path)
    rig = _write(
        tmp_path / "rig.json",
        {
            "document": "rig",
            "schema_version": 1,
            "name": "axle-bench",
            "supported_assembly_kinds": ["suspension_axle"],
            "required_ports": ["wheel_centre"],
            "measurements": ["wheel_travel"],
        },
    )
    assembly = _write(
        tmp_path / "axle.asy.json",
        {
            "document": "assembly",
            "schema_version": 1,
            "name": "axle",
            "assembly_kind": "suspension_axle",
            "subsystems": [
                {"ref": "front.sub.json", "functional_role": "suspension", "placement_role": "front"},
                {"ref": "chassis.sub.json", "functional_role": "chassis", "placement_role": "any"},
            ],
            "rig": rig.name,
        },
    )
    simulation = SimulationAssembly.load(assembly)
    assert simulation.bindings == {"wheel_centre": "front.sub.json"}
    provenance = simulation.provenance()
    assert provenance["rig"]
    assert provenance["subsystems"]["front.sub.json"]["topology"]
    assert provenance["subsystems"]["front.sub.json"]["effective_values"]


def test_rig_refuses_a_missing_required_port(tmp_path: Path) -> None:
    _template(tmp_path)
    _chassis_template(tmp_path)
    _suspension_subsystem(tmp_path, "front", "front")
    _chassis_subsystem(tmp_path)
    _write(
        tmp_path / "rig.json",
        {
            "document": "rig",
            "schema_version": 1,
            "name": "axle-bench",
            "supported_assembly_kinds": ["suspension_axle"],
            "required_ports": ["brake_torque"],
        },
    )
    assembly = _write(
        tmp_path / "axle.asy.json",
        {
            "document": "assembly",
            "schema_version": 1,
            "name": "axle",
            "assembly_kind": "suspension_axle",
            "subsystems": [
                {"ref": "front.sub.json", "functional_role": "suspension", "placement_role": "front"},
                {"ref": "chassis.sub.json", "functional_role": "chassis", "placement_role": "any"},
            ],
            "rig": "rig.json",
        },
    )
    with pytest.raises(AuthoringError, match="does not offer required port"):
        SimulationAssembly.load(assembly)


def test_template_rejects_a_hardpoint_owned_by_a_missing_body(tmp_path: Path) -> None:
    _write(
        tmp_path / "bad.tpl.json",
        {
            "document": "template",
            "schema_version": 1,
            "name": "bad",
            "functional_role": "suspension",
            "allowed_placement_roles": ["front"],
            "bodies": [{"name": "arm"}],
            "hardpoints": [{"name": "mount", "owner": "chassis"}],
            "joints": [],
            "elements": [],
            "property_slots": [],
        },
    )
    with pytest.raises(AuthoringError, match="not one of this template's bodies"):
        TemplateDocument.load(tmp_path / "bad.tpl.json")


def test_rig_document_rejects_a_port_declared_twice(tmp_path: Path) -> None:
    path = _write(
        tmp_path / "rig.json",
        {
            "document": "rig",
            "schema_version": 1,
            "name": "bench",
            "supported_assembly_kinds": ["suspension_axle"],
            "required_ports": ["wheel_centre"],
            "optional_ports": ["wheel_centre"],
        },
    )
    with pytest.raises(AuthoringError, match="both required and optional"):
        RigDocument.load(path)
