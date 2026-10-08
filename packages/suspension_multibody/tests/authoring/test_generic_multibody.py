"""Generic data graphs reach native without any suspension role or builder."""

from __future__ import annotations

import copy
import json

import numpy as np
import pytest

from suspension_multibody import simulate, validate
from suspension_multibody.authoring import (
    AssemblyDocument,
    GenericSubsystemAssembler,
    SubsystemDocument,
    TemplateDocument,
    assemble_generic,
)
from suspension_multibody.authoring.errors import AuthoringError
from suspension_multibody.authoring.properties import ElementPropertyDocument
from suspension_multibody.simulation.runner import run_compiled

_EQUILIBRIUM = 0.25 - 10.0 * 9.80665 / 10_000.0


def _template(**updates) -> dict:
    payload = {
        "document": "template",
        "schema_version": 1,
        "name": "slider",
        "functional_role": "generic",
        "allowed_placement_roles": ["any"],
        "symmetry": "asymmetric",
        "units": {"length": "m"},
        "bodies": [
            {"name": "support", "fixed": True},
            {"name": "carriage", "mass": 10.0, "position": [0.0, 0.0, _EQUILIBRIUM]},
        ],
        "hardpoints": [
            {"name": "base", "owner": "support"},
            {"name": "mount", "owner": "carriage"},
        ],
        "joints": [
            {
                "name": "guide",
                "type": "prismatic",
                "body_a": "support",
                "body_b": "carriage",
                "point_a": "base",
                "point_b": "mount",
                "axis": [0.0, 0.0, 1.0],
            }
        ],
        "elements": [
            {
                "name": "spring",
                "type": "spring",
                "body_a": "support",
                "body_b": "carriage",
                "point_a": "base",
                "point_b": "mount",
                "property_slot": "spring",
            },
            {
                "name": "damper",
                "type": "damper",
                "body_a": "support",
                "body_b": "carriage",
                "point_a": "base",
                "point_b": "mount",
                "property_slot": "damper",
            },
        ],
        "property_slots": [
            {"name": "spring", "element_type": "spring", "required": True},
            {"name": "damper", "element_type": "damper", "required": True},
        ],
        "ports": [
            {
                "name": "input",
                "role": "translation_input",
                "owner": "carriage",
                "point": "mount",
            }
        ],
    }
    payload.update(updates)
    return payload


def _subsystem(
    payload: dict | None = None, *, coordinates=None, name="slider"
) -> SubsystemDocument:
    template = TemplateDocument.from_payload(
        _template() if payload is None else payload
    )
    laws = {
        kind: ElementPropertyDocument.from_payload(
            {
                "document": "element_properties",
                "schema_version": 1,
                "name": kind,
                "element_type": kind,
                "model": "linear",
                "units": {
                    "force": "N",
                    "length": "mm",
                    **({"time": "s"} if kind == "damper" else {}),
                },
                "parameters": (
                    {"stiffness": 10.0, "free_length": 250.0}
                    if kind == "spring"
                    else {"viscous_damping": 0.1}
                ),
            }
        )
        for kind in ("spring", "damper")
    }
    slots = template.property_slots
    return SubsystemDocument.from_payload(
        {
            "document": "subsystem",
            "schema_version": 1,
            "name": name,
            "template": template.name,
            "functional_role": "generic",
            "placement_role": "any",
            "hardpoints": coordinates
            or {"base": [0.0, 0.0, 0.0], "mount": [0.0, 0.0, _EQUILIBRIUM]},
            "property_bindings": {slot: slot for slot in slots},
        },
        template=template,
        properties={slot: laws[slot] for slot in slots},
    )


def _assembly(subsystems=None, **updates) -> AssemblyDocument:
    subsystems = {"mechanism": _subsystem()} if subsystems is None else subsystems
    payload = {
        "document": "assembly",
        "schema_version": 1,
        "name": "generic-slider",
        "assembly_kind": "generic_multibody",
        "subsystems": [
            {"ref": name, "functional_role": "generic", "placement_role": "any"}
            for name in subsystems
        ],
    }
    payload.update(updates)
    return AssemblyDocument.from_payload(payload, subsystems=subsystems)


def _case() -> dict:
    return {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "axle_dynamic",
        "name": "equilibrium",
        "time": {"start_s": 0.0, "end_s": 0.002, "step_s": 0.001},
        "solver": {"internal_step_s": 0.00025},
    }


def test_non_suspension_document_reaches_native_and_preserves_equilibrium(
    tmp_path,
) -> None:
    assembly = _assembly()
    run = simulate(assembly, _case())
    assert run.status == "success"
    assert run.compiled.assembly == "generic"
    assert run.compiled.model_document["units"]["length"] == "m"
    index = run.raw.body_names.index("mechanism.carriage")
    np.testing.assert_allclose(
        run.raw.states[:, index, 2], _EQUILIBRIUM, atol=1e-9, rtol=0.0
    )
    np.testing.assert_allclose(run.raw.states[:, index, 7:], 0.0, atol=1e-9, rtol=0.0)
    # Saving and loading the same data must exercise the same public route.
    subsystem = assembly.entries[0].subsystem
    subsystem.template.save(tmp_path / "slider.tpl.json")
    for name, law in subsystem.properties.items():
        (tmp_path / name).write_text(json.dumps(law.payload), encoding="utf-8")
    payload = subsystem.to_payload()
    payload["template"] = "slider.tpl.json"
    (tmp_path / "mechanism").write_text(json.dumps(payload), encoding="utf-8")
    path = assembly.save(tmp_path / "assembly.json")
    from_file = simulate(path, _case())
    np.testing.assert_array_equal(from_file.raw.states, run.raw.states)


def test_rig_is_an_ordinary_template_and_port_connection_drives_it() -> None:
    rig = _subsystem(
        _template(
            name="rig",
            bodies=[{"name": "fixture", "fixed": True}],
            hardpoints=[{"name": "origin", "owner": "fixture"}],
            joints=[],
            elements=[],
            property_slots=[],
            ports=[
                {
                    "name": "actuator",
                    "role": "translation_input",
                    "owner": "fixture",
                    "point": "origin",
                }
            ],
        ),
        coordinates={"origin": [0.0, 0.0, 0.0]},
        name="bench",
    )
    assembly = _assembly(
        {"mechanism": _subsystem(), "bench": rig},
        connections=[
            {
                "name": "actuator",
                "port_a": "mechanism.input",
                "port_b": "bench.actuator",
                "type": "driven_translation",
                "axis": [0.0, 0.0, 1.0],
                "target": "travel",
            }
        ],
    )
    case = _case()
    target = np.asarray([0.0, 0.00005, 0.0002], dtype=np.float64)
    rate = np.asarray([0.0, 0.1, 0.2], dtype=np.float64)
    case["solver"]["initialization_mode"] = "provided_consistent_state"
    case["blobs"] = [
        {
            "role": role,
            "coordinate": "travel",
            "offset": offset,
            "length": 24,
            "dtype": "float64",
            "shape": [3],
        }
        for role, offset in (("driven_offset", 0), ("driven_offset_rate", 24))
    ]
    run = run_compiled(
        validate(assembly, case, case_payload=target.tobytes() + rate.tobytes())
    )
    assert run.status == "success"
    index = run.raw.body_names.index("mechanism.carriage")
    np.testing.assert_allclose(
        run.raw.states[:, index, 2], _EQUILIBRIUM + target, atol=1e-8, rtol=0.0
    )
    assert "bench.fixture" in run.raw.body_names


def test_data_controls_modes_without_changing_geometry() -> None:
    payload = _template()
    payload["joints"][0]["modes"] = ["K"]
    payload["elements"][0]["modes"] = ["C"]
    subsystem = _subsystem(payload)
    assembler = GenericSubsystemAssembler()
    k, c = (
        assembler.assemble(subsystem, mode="K"),
        assembler.assemble(subsystem, mode="C"),
    )
    assert len(k.joints) == 1 and not c.joints
    assert [row["type"] for row in k.elements] == ["damper"]
    assert [row["type"] for row in c.elements] == ["spring", "damper"]
    np.testing.assert_array_equal(
        k.bodies["carriage"].pose.translation, c.bodies["carriage"].pose.translation
    )


def test_mirror_transforms_pose_inertia_and_polar_and_axial_axes() -> None:
    payload = _template(symmetry="mirrored_xz")
    payload["bodies"][0]["symmetric"] = False
    payload["bodies"][1].update(
        position=[0.0, -0.5, 0.2],
        quaternion=[0.5, 0.5, 0.5, 0.5],
        inertia=[[2.0, 0.2, 0.1], [0.2, 3.0, 0.3], [0.1, 0.3, 4.0]],
    )
    payload["joints"][0]["axis"] = [1.0, 2.0, 3.0]
    payload["joints"].append(
        {**payload["joints"][0], "name": "pivot", "type": "revolute"}
    )
    subsystem = _subsystem(
        payload, coordinates={"base": [0.0, 0.0, 0.0], "mount": [0.0, -0.5, 0.2]}
    )
    result = GenericSubsystemAssembler().assemble(subsystem)
    left, right = result.bodies["carriage_L"], result.bodies["carriage_R"]
    mirror = np.diag([1.0, -1.0, 1.0])
    np.testing.assert_allclose(
        right.pose.rotation, mirror @ left.pose.rotation @ mirror
    )
    np.testing.assert_array_equal(
        right.pose.translation, mirror @ left.pose.translation
    )
    np.testing.assert_allclose(right.inertia, mirror @ left.inertia @ mirror)
    rows = {row["name"]: row for row in result.joints}
    np.testing.assert_allclose(
        rows["guide_R"]["axis_a"], mirror @ rows["guide_L"]["axis_a"]
    )
    np.testing.assert_allclose(
        rows["pivot_R"]["axis_a"], -mirror @ rows["pivot_L"]["axis_a"]
    )


def test_generic_graph_allows_multiple_joints_at_one_hardpoint() -> None:
    payload = _template()
    payload["joints"].append(
        {**payload["joints"][0], "name": "second", "type": "spherical"}
    )
    result = GenericSubsystemAssembler().assemble(_subsystem(payload))
    assert [row["name"] for row in result.joints] == ["guide", "second"]


def test_missing_axis_and_unknown_ports_fail_before_submission() -> None:
    payload = _template()
    del payload["joints"][0]["axis"]
    with pytest.raises(AuthoringError, match="needs axis"):
        GenericSubsystemAssembler().assemble(_subsystem(payload))
    assembly = _assembly(
        connections=[
            {
                "name": "bad",
                "port_a": "missing",
                "port_b": "mechanism.input",
                "type": "fixed",
            }
        ]
    )
    with pytest.raises(AuthoringError, match="unknown port"):
        assemble_generic(assembly)


def test_generic_compilation_does_not_mutate_template_or_coordinates() -> None:
    subsystem = _subsystem()
    before = copy.deepcopy(subsystem.to_payload()), subsystem.template.to_payload()
    GenericSubsystemAssembler().assemble(subsystem)
    assert before == (subsystem.to_payload(), subsystem.template.to_payload())


def test_generic_compliance_run_loads_a_declared_port_marker() -> None:
    case = _case()
    case.update(
        family="kc_quasi_static",
        c={
            "load_marker": "mechanism.input",
            "loads": [{"fz": 100.0}],
            "side_mode": "single",
        },
    )
    case["time"] = {"start_s": 0.0, "end_s": 0.02, "step_s": 0.01}
    run = simulate(_assembly(mode="C"), case)
    assert run.status == "success"
    assert run.request.study == "quasi_static"
    assert np.isfinite(run.raw.states).all()
    index = run.raw.body_names.index("mechanism.carriage")
    np.testing.assert_allclose(
        run.raw.case_body_state()[index, 2],
        _EQUILIBRIUM + 100.0 / 10_000.0,
        atol=1e-6,
        rtol=0.0,
    )


def test_axis_can_be_derived_from_declared_hardpoints() -> None:
    payload = _template()
    del payload["joints"][0]["axis"]
    payload["joints"][0]["axis_reference"] = "mount"
    result = GenericSubsystemAssembler().assemble(_subsystem(payload))
    np.testing.assert_array_equal(result.joints[0]["axis_a"], [0.0, 0.0, 1.0])


def test_millimetre_geometry_and_inertia_reach_the_same_native_model() -> None:
    payload = _template(units={"length": "mm"})
    payload["bodies"][1].update(
        position=[0.0, 0.0, 1000.0 * _EQUILIBRIUM],
        inertia=(np.eye(3) * 1_000_000.0).tolist(),
    )
    subsystem = _subsystem(
        payload,
        coordinates={
            "base": [0.0, 0.0, 0.0],
            "mount": [0.0, 0.0, 1000.0 * _EQUILIBRIUM],
        },
    )
    result = assemble_generic(_assembly({"mechanism": subsystem}))
    np.testing.assert_array_equal(
        result.bodies["mechanism.carriage"].inertia, np.eye(3)
    )
    run = simulate(_assembly({"mechanism": subsystem}), _case())
    reference = simulate(_assembly(), _case())
    np.testing.assert_allclose(
        run.raw.states, reference.raw.states, atol=1e-12, rtol=0.0
    )


def test_explicit_asymmetric_sides_preserve_both_authored_graphs() -> None:
    payload = _template(sides=["left", "right"], mirror=False)
    body = payload["bodies"].pop()
    point = payload["hardpoints"].pop()
    joints, elements, ports = payload["joints"], payload["elements"], payload["ports"]
    payload.update(joints=[], elements=[], ports=[])
    coordinates = {"base": [0.0, 0.0, 0.0]}
    for side, y in (("L", -0.5), ("R", 0.6)):
        suffix = "_" + side
        payload["bodies"].append({**body, "name": "carriage" + suffix})
        payload["hardpoints"].append(
            {**point, "name": "mount" + suffix, "owner": "carriage" + suffix}
        )
        coordinates["mount" + suffix] = [0.0, y, _EQUILIBRIUM]
        for key, rows in (("joints", joints), ("elements", elements)):
            payload[key].extend(
                {
                    **row,
                    "name": row["name"] + suffix,
                    "body_b": "carriage" + suffix,
                    "point_b": "mount" + suffix,
                }
                for row in rows
            )
        payload["ports"].extend(
            {
                **row,
                "name": row["name"] + suffix,
                "owner": "carriage" + suffix,
                "point": "mount" + suffix,
            }
            for row in ports
        )
    result = GenericSubsystemAssembler().assemble(
        _subsystem(payload, coordinates=coordinates)
    )
    assert set(result.bodies) == {"support", "carriage_L", "carriage_R"}
    assert {row["name"] for row in result.joints} == {"guide_L", "guide_R"}
    assert set(result.ports) == {"input_L", "input_R"}
    np.testing.assert_allclose(result.ports["input_R"].pose.translation[1], 0.6)


def test_explicit_port_binding_checks_semantics_and_records_optional_outputs() -> None:
    need = {
        "role": "translation_input",
        "count": 1,
        "required": True,
        "requires_capabilities": ["load"],
    }
    subsystem = _subsystem(_template(needs=[need]))
    assembly = _assembly(
        {"mechanism": subsystem, "neighbour": _subsystem(name="other")}
    )
    payload = assembly.to_payload()
    payload["subsystems"][0]["pairings"] = [
        {"requirement_role": "translation_input", "port": "neighbour.input"}
    ]
    assembly = AssemblyDocument.from_payload(
        payload,
        subsystems={"mechanism": subsystem, "neighbour": _subsystem(name="other")},
    )
    with pytest.raises(
        AuthoringError, match="incompatible role, capabilities or labels"
    ):
        assemble_generic(assembly)

    optional = {
        "role": "optional_channel",
        "count": 1,
        "required": False,
        "bound_outputs": ["optional_output"],
    }
    subsystem = _subsystem(
        _template(
            needs=[optional],
            outputs=[
                {"name": "optional_output", "unit": "m"},
                {"name": "position", "unit": "m"},
            ],
        )
    )
    result = assemble_generic(_assembly({"mechanism": subsystem}))
    assert result.bindings["mechanism"].disappeared == ("optional_channel",)
    assert set(result.fragments[0].outputs) == {"position"}


def test_port_bushing_normalises_six_axis_units_and_runs_native() -> None:
    stiffness = np.eye(6) * 10.0
    stiffness[0, 3] = stiffness[3, 0] = 0.25
    connection = {
        "name": "elastic_link",
        "port_a": "mechanism.input",
        "port_b": "neighbour.input",
        "type": "bushing",
        "units": {"length": "mm"},
        "parameters": {"stiffness": stiffness.tolist()},
    }
    subsystems = {"mechanism": _subsystem(), "neighbour": _subsystem(name="other")}
    assembly = _assembly(subsystems, connections=[connection])
    result = assemble_generic(assembly)
    params = result.elements[-1]["parameters"]
    expected = stiffness.copy()
    expected[:3, :3] *= 1000.0
    expected[3:, 3:] *= 0.001
    np.testing.assert_array_equal(params["stiffness"], expected)
    assert simulate(assembly, _case()).status == "success"
    connection["parameters"]["stiffness"] = [[1.0]]
    with pytest.raises(AuthoringError, match="finite 6x6 matrix"):
        assemble_generic(_assembly(subsystems, connections=[connection]))
