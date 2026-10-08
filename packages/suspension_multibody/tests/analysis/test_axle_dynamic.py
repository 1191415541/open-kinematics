"""Axle time-domain result tests."""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.api import simulate
from suspension_multibody.authoring import (
    AssemblyDocument,
    SubsystemDocument,
    TemplateDocument,
)
from suspension_multibody.results import ResultEnvelope
from suspension_multibody.schema import (
    DynamicCaseSpec,
    DynamicSolverSettings,
    MassSpec,
    PrescribedMotion,
    TimeSignal,
    Vec3,
    WrenchInput,
    WrenchSignal,
)
from suspension_multibody.schema.model import AxleDeclaration


def _model() -> AxleDeclaration:
    return AxleDeclaration(
        hardpoints={
            "UPPER_INBOARD_FRONT": Vec3(x=0, y=-300, z=300),
            "UPPER_INBOARD_REAR": Vec3(x=300, y=-300, z=300),
            "UPPER_OUTBOARD": Vec3(x=150, y=-700, z=250),
            "LOWER_INBOARD_FRONT": Vec3(x=0, y=-320, z=0),
            "LOWER_INBOARD_REAR": Vec3(x=320, y=-320, z=0),
            "LOWER_OUTBOARD": Vec3(x=150, y=-720, z=50),
            "TIE_ROD_INBOARD": Vec3(x=100, y=-250, z=100),
            "TIE_ROD_OUTBOARD": Vec3(x=180, y=-700, z=100),
            "WHEEL_CENTER": Vec3(x=160, y=-760, z=150),
            "RACK_CENTER": Vec3(x=100, y=0, z=100),
        },
        mass=MassSpec(sprung_mass=1000.0),
    )


def test_axle_quasi_static_runs_time_series_with_motion_and_loads() -> None:
    case = DynamicCaseSpec(
        mode="axle_dynamic",
        solver=DynamicSolverSettings(
            end_time=0.02,
            step_size=0.01,
            integrator="quasi_static",
        ),
        prescribed_motions=(
            PrescribedMotion(
                target="wheel_travel_left",
                displacement=TimeSignal(times=(0.0, 0.02), values=(0.0, 5.0)),
            ),
        ),
        wrench_inputs=(
            WrenchInput(
                target="right",
                wrench=WrenchSignal(fz=TimeSignal(constant=25.0)),
            ),
        ),
    )

    template = TemplateDocument.from_payload({"document": "template", "schema_version": 1,
        "name": "motion_load_fixture", "functional_role": "generic", "allowed_placement_roles": ["any"],
        "symmetry": "asymmetric", "units": {"length": "m"}, "property_slots": [], "elements": [],
        "bodies": [{"name": "fixture", "fixed": True}, *[{"name": side, "mass": 1, "inertia": np.eye(3).tolist()} for side in ("left", "right")]],
        "hardpoints": [{"name": body, "owner": body, "space": "body"} for body in ("fixture", "left", "right")],
        "joints": [{"name": side+"_slide", "type": "prismatic", "body_a": "fixture", "body_b": side,
            "point_a": "fixture", "point_b": side, "axis": [0, 0, 1]} for side in ("left", "right")],
        "coordinates": [{"name": "travel", "joint": "left_slide", "kind": "translation"}]})
    sub = SubsystemDocument.from_payload({"document": "subsystem", "schema_version": 1, "name": template.name,
        "functional_role": "generic", "placement_role": "any", "template": "fixture.tpl.json",
        "hardpoints": {body: [0, 0, 0] for body in ("fixture", "left", "right")}, "property_bindings": {}}, template=template)
    assembly = AssemblyDocument.from_payload({"document": "assembly", "schema_version": 1,
        "name": "motion-load", "assembly_kind": "generic_multibody", "mode": "K", "gravity": [0, 0, 0],
        "subsystems": [{"ref": "unit", "functional_role": "generic", "placement_role": "any"}]}, subsystems={"unit": sub})
    times = [0, .01, .02]
    signal = case.prescribed_motions[0].displacement
    plan = {"schema_version": 1, "name": case.name, "study": "dynamic", "protocol": "axle_dynamic",
        "samples": times, "solver": {"initialization_mode": "provided_consistent_state"},
        "initial_state": {"unit.left": {"velocity": [0, 0, .25]}},
        "boundaries": [{"name": "wheel_travel_left", "coordinate": "unit.travel", "mode": "prescribed_displacement", "units": "m", "program": "travel"}],
        "inputs": [{"name": "travel", "values": [signal.value_at(time)*.001 for time in times], "rates": [.25]*3},
            {"name": "right_load", "role": "body_wrench", "body": "unit.right", "values": [[0, 0, 25, 0, 0, 0]]*3}], "outputs": []}
    run = simulate(assembly, plan)
    assert isinstance(run.result, ResultEnvelope)
    assert len(run.result.times_s) == 3
    assert run.raw.status == "success"
    assert run.result.model.fingerprint
    assert run.result.body_state("unit.left")[-1, 2] == pytest.approx(.005, abs=1e-9)
    loads = run.raw.block("element_wrench")
    assert np.any(np.isclose(loads[:, :, 2], 25, atol=1e-12))
    assert run.result.body_state("unit.right")[-1, 2] > 0


def test_legacy_axle_integrator_is_rejected() -> None:
    case = DynamicCaseSpec(
        mode="axle_dynamic",
        solver=DynamicSolverSettings(
            end_time=0.01,
            step_size=0.01,
            integrator="generalized_alpha",
        ),
    )

    with pytest.raises(ValueError, match="assembly input"):
        simulate(_model(), case.model_dump(mode="json"))
