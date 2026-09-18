"""
The vehicle K/C family: the sweep it prescribes is the sweep it was asked for.

This family is a *new capability* -- there was no native vehicle-level K/C -- so
its evidence is physical rather than a comparison against another
implementation:

* a zero sweep returns the assembling pose, because the kernel resolves each
  driven coordinate to the separation it had when the model was assembled;
* a non-zero sweep advances every driven coordinate by exactly the travel it was
  given (that is the *definition* of the K in K/C, so it is asserted to solver
  accuracy rather than loosely);
* the left/right mode redistributes the same travel, so an opposite sweep does
  not move the body the way a symmetric one does.

The model is the **compliant** vehicle.  On the rigid one the driven directions
are only marginally independent -- the analytic Jacobian's pivot sits just below
the rank threshold while central differences put it just above -- and the audit
refuses the set.  That is a conditioning property of the rigid linkage, not a
defect in the sweep, and it is why these tests build the vehicle with bushings.

The sweep is quasi-static, so it is reached by integrating the driven
coordinates from the assembling pose to their target.  The ramp that carries
them there has to be *resolved*: the kernel raises each driven coordinate along
a raised cosine whose rate is zero at both ends, and the adaptive stepper's
local-error controller needs enough samples across that window to follow it.
Too coarse a grid (2 samples over the ramp) is rejected at ``t = 0`` with a
non-convergent Newton step -- a property of the step size, not of the family --
which is why the window and sample count below are part of the acceptance
rather than incidentals.

Consequence worth stating plainly: the body pose this family reports is the
*incremental* response over the declared window, not a per-grid-point static
equilibrium.  A K/C sweep that answers with one converged equilibrium per grid
point is the next capability, and it needs a static solve that steps the driven
target across its load steps; today's ``static_trim`` solves the target it is
handed at ``t = 0`` and does not steer to a far one.  The kinematic assertions
below are independent of that limitation: they test the drive, which is exact.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest

from suspension_multibody.axle_dynamics.schema import AxleSolverSettings
from suspension_multibody.cases import (
    run_vehicle_kc_contract,
    vehicle_kc_case_document,
    vehicle_kc_model_document,
)
from suspension_multibody.core.spatial import quaternion_to_matrix
from suspension_multibody.schema import Bushing6x6, Pose, Vec3
from suspension_multibody.vehicle_dynamics import prepare_vehicle_run

_FIXTURE = Path(__file__).resolve().parents[1] / "vehicle" / "test_native_vehicle.py"

#: The ramp window and its sampling.  20 ms reached at 1 ms steps (21 samples on
#: the contract's uniform grid) is the coarsest grid the error controller
#: accepted: 10 samples over the same window trips the local-error limit and 2
#: samples fail outright at ``t = 0``.  See the module docstring.
_WINDOW_S = 2e-2
_SAMPLES = 21
_TIMES_S = tuple(np.linspace(0.0, _WINDOW_S, _SAMPLES).tolist())

#: The four driven wheel coordinates, in the order the family names them.
_WHEELS = ("front_upright_L", "front_upright_R", "rear_upright_L", "rear_upright_R")

_STIFFNESS = tuple(
    tuple(
        10_000.0 if row == column and row < 3
        else 10_000_000.0 if row == column
        else 0.0
        for column in range(6)
    )
    for row in range(6)
)


def _fixture():
    spec = importlib.util.spec_from_file_location("vehicle_kc_fixture", _FIXTURE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _compliant(axle):
    """Give one axle the four compliant inboard mounts the C mode expects."""

    def mount(name):
        point = axle.hardpoints[name]
        return Pose(translation=Vec3(x=float(point.x), y=float(point.y), z=float(point.z)))

    bushings = tuple(
        Bushing6x6(
            name=f"{body}_{index}",
            body_a="chassis",
            body_b=body,
            pose_a=mount(name),
            pose_b=mount(name),
            stiffness=_STIFFNESS,
        )
        for body, names in (
            ("upper_arm", ("UPPER_INBOARD_FRONT", "UPPER_INBOARD_REAR")),
            ("lower_arm", ("LOWER_INBOARD_FRONT", "LOWER_INBOARD_REAR")),
        )
        for index, name in enumerate(names)
    )
    return axle.model_copy(update={"bushings": bushings})


@pytest.fixture(scope="module")
def prepared():
    fixture = _fixture()
    rigid = fixture._positioned_vehicle(fixture._vehicle())
    model = rigid.model_copy(
        update={
            "front_axle": _compliant(rigid.front_axle),
            "rear_axle": _compliant(rigid.rear_axle),
        }
    )
    case = fixture._case(model)
    return model, case, prepare_vehicle_run(model, case)


def _run(prepared, *, wheels, mode):
    model, case, base = prepared
    sweep = vehicle_kc_case_document(
        name=f"vehicle-kc-{mode}",
        wheel_values_mm=wheels,
        rack_values_mm=(0.0,),
        times_s=_TIMES_S,
        settings=AxleSolverSettings(),
        left_right_mode=mode,
    )
    document = vehicle_kc_model_document(model, base)
    run = run_vehicle_kc_contract(
        document, case=sweep, wheels=(), assembly=base.assembly
    )
    return run, base


def _driven_value(run, base, body: str) -> float:
    """
    Return the wheel-centre separation the kernel drives, measured from the state.

    The kernel's driven-translation row is
    ``dot(point_a - point_b, R_chassis * axis)`` -- the signed separation along
    the reaction body's axis -- so the same quantity can be reconstructed from
    the reported body state and compared with the target.  Model documents are
    written in metres, so the assembly points (millimetres) are scaled.
    """
    names = list(run.document["manifest"]["bodies"])
    row = run.block("body_state")[-1]

    def pose(name: str):
        entry = row[names.index(name)]
        return np.asarray(entry[:3], dtype=float), np.asarray(entry[3:7], dtype=float)

    p_upright, q_upright = pose(body)
    p_chassis, q_chassis = pose("chassis")
    local = np.asarray(base.assembly.points[(body, "wheel_center")], dtype=float) / 1e3
    world = p_upright + quaternion_to_matrix(q_upright) @ local
    axis = quaternion_to_matrix(q_chassis) @ np.array([0.0, 0.0, 1.0])
    return float(np.dot(world - p_chassis, axis))


def test_a_zero_sweep_returns_the_assembling_pose(prepared) -> None:
    run, base = _run(prepared, wheels=(0.0,), mode="symmetric")
    states = run.block("body_state")
    bodies = list(run.document["manifest"]["bodies"])
    for index, body in enumerate(base.native_model.bodies):
        found = states[-1, bodies.index(body.name)]
        assert found[0] == pytest.approx(body.position_m[0], abs=1e-9)
        assert found[1] == pytest.approx(body.position_m[1], abs=1e-9)
        assert found[2] == pytest.approx(body.position_m[2], abs=1e-9)


def test_a_symmetric_bump_advances_every_wheel_drive_by_the_travel(prepared) -> None:
    """
    The K in K/C: every driven wheel coordinate ends at its requested travel.

    Comparing the driven coordinate of a bumped run against a zero run cancels
    the assembled separation, leaving the 10 mm the case asked for.
    """
    zero, base = _run(prepared, wheels=(0.0,), mode="symmetric")
    bump, _ = _run(prepared, wheels=(10.0,), mode="symmetric")
    for body in _WHEELS:
        advance = _driven_value(bump, base, body) - _driven_value(zero, base, body)
        assert advance == pytest.approx(0.010, abs=1e-6), (
            f"{body} advanced {advance * 1e3:.4f} mm, not 10 mm"
        )


def test_the_mode_redistributes_the_same_travel(prepared) -> None:
    """
    The same travel, signed the other way from side to side.

    A zero sweep is mode independent -- every offset is zero -- so one zero run
    is the reference for both modes.
    """
    zero, base = _run(prepared, wheels=(0.0,), mode="symmetric")
    opposite, _ = _run(prepared, wheels=(10.0,), mode="opposite")
    for body in _WHEELS:
        sign = 1.0 if body.endswith("_L") else -1.0
        advance = _driven_value(opposite, base, body) - _driven_value(zero, base, body)
        assert advance == pytest.approx(sign * 0.010, abs=1e-6), (
            f"{body} advanced {advance * 1e3:.4f} mm under an opposite sweep"
        )


def test_an_unknown_mode_is_refused(prepared) -> None:
    model, _, base = prepared
    sweep = vehicle_kc_case_document(
        name="vehicle-kc-bad",
        wheel_values_mm=(0.0,),
        rack_values_mm=(0.0,),
        times_s=_TIMES_S,
        settings=AxleSolverSettings(),
    )
    sweep["k"]["left_right_mode"] = "diagonal"
    document = vehicle_kc_model_document(model, base)
    with pytest.raises(Exception, match="left_right_mode"):
        run_vehicle_kc_contract(
            document, case=sweep, wheels=(), assembly=base.assembly
        )
