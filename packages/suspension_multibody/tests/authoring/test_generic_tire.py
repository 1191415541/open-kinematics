"""A tire unit expands into one inertial body and an explicitly framed native tire."""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.api import validate
from suspension_multibody.authoring import (
    GenericSubsystemAssembler,
    SubsystemDocument,
    TemplateDocument,
)
from suspension_multibody.authoring.errors import AuthoringError
from suspension_multibody.authoring.properties import ElementPropertyDocument
from suspension_multibody.simulation.runner import run_compiled

from .test_generic_multibody import _assembly, _case
from .test_tir_properties import TIR


def _tire_payload() -> dict:
    return {
        "document": "template",
        "schema_version": 1,
        "name": "wheel-unit",
        "functional_role": "generic",
        "allowed_placement_roles": ["any"],
        "symmetry": "asymmetric",
        "units": {"length": "m"},
        "bodies": [{"name": "carrier", "fixed": True, "position": [0, 0, 0.334]}],
        "hardpoints": [
            {"name": "center", "owner": "wheel"},
            {"name": "carrier_center", "owner": "carrier"},
        ],
        "markers": [
            {"name": "center", "owner": "wheel", "point": "center"},
            {"name": "spin", "owner": "wheel", "point": "center"},
            {"name": "contact", "owner": "carrier", "point": "carrier_center"},
        ],
        "joints": [
            {
                "name": "spin",
                "type": "revolute",
                "body_a": "carrier",
                "body_b": "wheel",
                "point_a": "carrier_center",
                "point_b": "center",
                "axis": [0, 1, 0],
            }
        ],
        "elements": [],
        "coordinates": [{"name": "spin", "joint": "spin", "kind": "rotation"}],
        "property_slots": [
            {"name": "mass", "element_type": "mass", "required": True},
            {"name": "inertia", "element_type": "inertia", "required": True},
            {
                "name": "tire",
                "element_type": "tire",
                "required": True,
                "allowed_models": ["pac2002", "fiala", "native_brush"],
            },
        ],
        "ports": [
            {"name": "hub", "role": "hub", "marker": "center"},
            {"name": "contact", "role": "contact", "marker": "contact"},
            {"name": "road", "role": "road", "kind": "road", "resource": "plane"},
        ],
        "tires": [
            {
                "name": "tire",
                "body": {
                    "name": "wheel",
                    "mass_slot": "mass",
                    "inertia_slot": "inertia",
                    "center_marker": "center",
                },
                "model_slot": "tire",
                "mount_port": "hub",
                "spin_marker": "spin",
                "contact_frame": {"port": "contact"},
                "road_port": "road",
            }
        ],
    }


def _tire_subsystem(
    payload: dict | None = None, *, tir: str = TIR, coordinates: dict | None = None
) -> SubsystemDocument:
    template = TemplateDocument.from_payload(payload or _tire_payload())
    common = {
        "document": "element_properties",
        "schema_version": 1,
        "model": "linear",
        "units": {"mass": "kg", "length": "m", "force": "N"},
    }
    properties = {
        "mass": ElementPropertyDocument.from_payload(
            {
                **common,
                "name": "mass",
                "element_type": "mass",
                "parameters": {"value": 20},
            }
        ),
        "inertia": ElementPropertyDocument.from_payload(
            {
                **common,
                "name": "inertia",
                "element_type": "inertia",
                "matrix": {
                    "name": "inertia",
                    "rows": [[2, -0.1, 0], [-0.1, 2, 0], [0, 0, 2]],
                },
            }
        ),
        "tire": ElementPropertyDocument.from_tir_text(tir, name="tire"),
    }
    return SubsystemDocument.from_payload(
        {
            "document": "subsystem",
            "schema_version": 1,
            "name": "wheel-unit",
            "template": template.name,
            "functional_role": "generic",
            "placement_role": "any",
            "hardpoints": coordinates or {"center": [0, 0, 0.334], "carrier_center": [0, 0, 0.334]},
            "property_bindings": {key: key for key in properties},
        },
        template=template,
        properties=properties,
    )


def test_tire_unit_has_one_body_and_no_duplicate_inertia() -> None:
    assembled = GenericSubsystemAssembler().assemble(_tire_subsystem())
    assert len(assembled.bodies) == 2
    assert assembled.bodies["wheel"].mass == 20
    np.testing.assert_array_equal(
        assembled.bodies["wheel"].inertia, [[2, -0.1, 0], [-0.1, 2, 0], [0, 0, 2]]
    )
    tire = assembled.model_document()["tires"][0]
    assert tire["mass"] == 0
    assert np.asarray(tire["inertia"]).sum() == 0
    assert tire["parameters"]["frame_body"] == "carrier"
    assert tire["body"] == "wheel"
    assert assembled.model_document()["blobs"]


@pytest.mark.parametrize("field", ["center_marker", "mass_slot", "inertia_slot"])
def test_tire_bad_reference_rejected_at_template(field: str) -> None:
    payload = _tire_payload()
    payload["tires"][0]["body"][field] = "unknown"
    with pytest.raises(AuthoringError, match="unknown"):
        TemplateDocument.from_payload(payload)


def test_duplicate_tire_body_and_unconnected_frame_are_rejected() -> None:
    payload = _tire_payload()
    payload["bodies"].append({"name": "wheel"})
    with pytest.raises(AuthoringError, match="also declared"):
        TemplateDocument.from_payload(payload)
    payload = _tire_payload()
    payload["joints"] = []
    payload["coordinates"] = []
    with pytest.raises(AuthoringError, match="explicit fixed or revolute mount"):
        GenericSubsystemAssembler().assemble(_tire_subsystem(payload))


def test_tire_mirror_preserves_polar_forward_and_axial_spin() -> None:
    payload = _tire_payload()
    payload["symmetry"] = "mirrored_xz"
    assembled = GenericSubsystemAssembler().assemble(_tire_subsystem(payload))
    left, right = assembled.model_document()["tires"]
    np.testing.assert_array_equal(
        left["parameters"]["forward_axis_local"],
        right["parameters"]["forward_axis_local"],
    )
    np.testing.assert_array_equal(
        left["parameters"]["spin_axis_local"], right["parameters"]["spin_axis_local"]
    )
    np.testing.assert_array_equal(
        assembled.bodies["wheel_R"].inertia, [[2, 0.1, 0], [0.1, 2, 0], [0, 0, 2]]
    )


def _stationary_case():
    return {"schema_version": 1, "name": "stationary", "study": "dynamic", "protocol": "axle_dynamic",
        "samples": [0, .001, .002], "solver": {},
        "boundaries": [{"name": "hold", "coordinate": "unit.spin", "mode": "locked", "value": 0, "units": "rad"}],
        "inputs": [], "outputs": []}


def test_generic_tir_unit_reaches_native_with_parameter_blob() -> None:
    assembly = _assembly({"unit": _tire_subsystem()})
    compiled = validate(assembly, _stationary_case())
    run = run_compiled(compiled)
    assert run.status == "success"
    assert "unit.wheel" in run.raw.body_names
    np.testing.assert_allclose(
        run.raw.states[:, run.raw.body_names.index("unit.wheel"), 2], 0.334, atol=1e-10
    )


def test_rotated_contact_frame_retains_native_axes() -> None:
    payload = _tire_payload()
    quaternion = [2**-0.5, 0, 0, 2**-0.5]
    for marker in payload["markers"]:
        marker["quaternion"] = quaternion
    payload["joints"][0]["axis"] = [-1, 0, 0]
    assembled = GenericSubsystemAssembler().assemble(_tire_subsystem(payload))
    parameters = assembled.model_document()["tires"][0]["parameters"]
    np.testing.assert_allclose(parameters["forward_axis_local"], [0, 1, 0], atol=1e-15)
    np.testing.assert_allclose(parameters["spin_axis_local"], [-1, 0, 0], atol=1e-15)
    assert (
        run_compiled(
            validate(_assembly({"unit": _tire_subsystem(payload)}), _stationary_case())
        ).status
        == "success"
    )


def test_tire_file_and_memory_have_identical_native_payload_and_states(
    tmp_path,
) -> None:
    subsystem = _tire_subsystem()
    assembly = _assembly({"unit": subsystem})
    subsystem.template.save(tmp_path / "wheel.tpl.json")
    for key, law in subsystem.properties.items():
        law.save(tmp_path / key)
    payload = subsystem.to_payload()
    payload["template"] = "wheel.tpl.json"
    import json

    (tmp_path / "unit").write_text(json.dumps(payload), encoding="utf-8")
    loaded = type(assembly).load(assembly.save(tmp_path / "assembly.json"))
    memory = validate(assembly, _stationary_case())
    file = validate(loaded, _stationary_case())
    assert memory.model_payload == file.model_payload
    np.testing.assert_array_equal(
        run_compiled(memory).raw.states, run_compiled(file).raw.states
    )


def test_tire_total_mass_spatial_inertia_matches_explicit_sources() -> None:
    body = GenericSubsystemAssembler().assemble(_tire_subsystem()).bodies["wheel"]
    x, y, z = body.pose.translation
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])

    def spatial(mass, inertia):
        return np.block(
            [
                [mass * np.eye(3), -mass * cross],
                [mass * cross, inertia - mass * cross @ cross],
            ]
        )

    expected = spatial(5, body.inertia * 0.25) + spatial(15, body.inertia * 0.75)
    np.testing.assert_allclose(
        spatial(body.mass, body.inertia), expected, atol=1e-12, rtol=1e-12
    )


def test_fiala_tire_unit_uses_native_fiala_parameter_slots() -> None:
    tir = TIR.replace("PAC2002", "FIALA").replace("USE_MODE = 14", "USE_MODE = 12")
    tir += "\n[FIALA]\nCSLIP = 1000\nCALPHA = 800\nRELAX_LENGTH_X = 0.05\nRELAX_LENGTH_Y = 0.15\n"
    compiled = validate(_assembly({"unit": _tire_subsystem(tir=tir)}), _stationary_case())
    assert compiled.model_document["tires"][0]["model"] == "fiala"
    assert run_compiled(compiled).status == "success"


def test_single_tire_static_normal_load_balances_total_inertia() -> None:
    payload = _tire_payload()
    height = .344 - 21*9.80665/(1000/.01)
    payload["bodies"][0].update(fixed=False, mass=1, position=[0, 0, height])
    payload["bodies"].append({"name": "ground", "fixed": True})
    payload["hardpoints"].append({"name": "origin", "owner": "ground"})
    payload["joints"].append({"name": "slide", "type": "prismatic", "body_a": "ground", "body_b": "carrier", "point_a": "origin", "point_b": "carrier_center", "axis": [0, 0, 1]})
    coordinates = {"center": [0, 0, height], "carrier_center": [0, 0, height], "origin": [0, 0, 0]}
    case = _stationary_case()
    compiled = validate(_assembly({"unit": _tire_subsystem(payload, coordinates=coordinates)}), case)
    result = run_compiled(compiled)
    np.testing.assert_allclose(result.raw.blocks["tire_output"][:, 0, 4], 21*9.80665, atol=1e-8, rtol=1e-12)
    np.testing.assert_allclose(result.raw.states[:, result.raw.body_names.index("unit.wheel"), 2], height, atol=1e-10)


def test_rolling_tire_keeps_contact_frame_on_non_spinning_carrier() -> None:
    from dataclasses import replace

    from suspension_contracts import pack_container, unpack_container

    payload = _tire_payload()
    payload["bodies"][0].update(fixed=False, mass=1)
    payload["bodies"].append({"name": "ground", "fixed": True})
    payload["hardpoints"].append({"name": "origin", "owner": "ground"})
    payload["joints"].append({"name": "slide", "type": "prismatic", "body_a": "ground", "body_b": "carrier", "point_a": "origin", "point_b": "carrier_center", "axis": [1, 0, 0]})
    coordinates = {"center": [0, 0, .334], "carrier_center": [0, 0, .334], "origin": [0, 0, .334]}
    case = _case()
    case["solver"]["initialization_mode"] = "provided_consistent_state"
    compiled = validate(_assembly({"unit": _tire_subsystem(payload, coordinates=coordinates)}), case)
    document, blob = unpack_container(compiled.model_payload)
    for body in document["bodies"]:
        if body["name"] != "unit.ground":
            body["velocity"] = [1, 0, 0]
        if body["name"] == "unit.wheel":
            body["omega"] = [0, 1/.344, 0]
    result = run_compiled(replace(compiled, model_document=document, model_payload=pack_container(document, blob)))
    carrier = result.raw.body_names.index("unit.carrier")
    wheel = result.raw.body_names.index("unit.wheel")
    assert result.raw.states[-1, wheel, 0] > 0
    np.testing.assert_allclose(result.raw.states[:, carrier, 3:7], np.tile([1, 0, 0, 0], (3, 1)), atol=1e-12, rtol=0)
    assert abs(result.raw.states[-1, wheel, 5]) > 1e-4
