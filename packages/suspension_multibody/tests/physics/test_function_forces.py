"""Actual static/dynamic residuals apply generic force programs once per end."""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from suspension_contracts import pack_container

from suspension_multibody.api import validate
from suspension_multibody.simulation.runner import run_compiled

sys.path.insert(0, str(Path(__file__).parents[1] / "authoring"))
from test_generic_function_elements import function_subsystem
from test_generic_multibody import _assembly, _case


def run_function(subsystem=None, *, omega=0, travel=0, time=0.02, gravity=None):
    subsystem = subsystem or function_subsystem()
    assembly = _assembly({"mechanism": subsystem}, gravity=gravity or [0, 0, 0])
    case = _case()
    case["time"] = {"start_s": 0, "end_s": time, "step_s": time / 10}
    case["solver"].update(
        initialization_mode="provided_consistent_state",
        adaptive_step=False,
        rho_inf=1,
        internal_step_s=time / 40,
    )
    compiled = validate(assembly, case)
    model = dict(compiled.model_document)
    bodies = [dict(body) for body in model["bodies"]]
    bodies[1]["omega"] = [0, 0, omega]
    bodies[1]["position"] = [0, 0, travel]
    model["bodies"] = bodies
    compiled = replace(
        compiled, model_document=model, model_payload=pack_container(model)
    )
    return run_compiled(compiled)


@pytest.mark.parametrize("demand", [-3, 3])
def test_signed_drive_torque_acceleration(demand):
    subsystem = function_subsystem(bindings={"demand": {"unit": "Nm", "value": demand}})
    run = run_function(subsystem)
    state = run.raw.blocks["body_state"]
    assert state[-1, 1, 12] == pytest.approx(demand * 0.02, abs=2e-10)
    assert np.sign(state[-1, 1, 6]) == np.sign(demand)


def test_feedback_brake_reads_current_relative_angular_velocity():
    binding = {
        "speed": {
            "source": "measurement",
            "measurement": "relative_angular_velocity",
            "unit": "rad/s",
            "action": "action",
            "reaction": "reaction",
            "reference": "reaction",
            "axis": [0, 0, 1],
        },
        "limit": {"unit": "Nm", "value": 10},
        "reference_speed": {"unit": "rad/s", "value": 1},
    }
    run = run_function(
        function_subsystem(
            expression="-limit*tanh(speed/reference_speed)", bindings=binding
        ),
        omega=2,
    )
    state = run.raw.blocks["body_state"]
    assert 0 < state[-1, 1, 12] < 2
    energy = run.raw.blocks["energy"]
    assert energy[-1, 0] < energy[0, 0]
    assert energy[-1, 4] < 0


def test_force_feedback_static_equilibrium_and_perturbed_decay():
    binding = {
        "position": {
            "source": "measurement",
            "measurement": "relative_position",
            "unit": "m",
            "action": "action",
            "reaction": "reaction",
            "reference": "reaction",
            "axis": [0, 0, 1],
        },
        "stiffness": {"unit": "N/m", "value": 1000},
        "preload": {"unit": "N", "value": 98.0665},
    }
    subsystem = function_subsystem("force", "preload-stiffness*position", binding)
    compiled = validate(_assembly({"mechanism": subsystem}), _case())
    equilibrium = run_compiled(compiled)
    np.testing.assert_allclose(
        equilibrium.raw.blocks["body_state"][:, 1, 2], 0, atol=1e-9
    )
    perturbed = run_function(subsystem, travel=0.01, gravity=[0, 0, -9.80665])
    assert 0 < perturbed.raw.blocks["body_state"][-1, 1, 2] < 0.01


def test_sampling_and_repeat_run_do_not_change_trial_state():
    first = run_function()
    second = run_function()
    for key in first.raw.blocks:
        np.testing.assert_array_equal(
            first.raw.blocks[key], second.raw.blocks[key]
        )


def test_time_law_uses_solver_internal_times():
    subsystem = function_subsystem(expression="gain*time/duration", bindings={
        "gain": {"unit": "Nm", "value": 2}, "duration": {"unit": "s", "value": .02}})
    result = run_function(subsystem)
    assert result.raw.blocks["body_state"][-1, 1, 12] == pytest.approx(.02, abs=1e-8)
