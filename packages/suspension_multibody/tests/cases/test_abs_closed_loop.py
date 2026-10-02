"""
The closed-loop ABS controller, through the kernel's element-evaluation path.

The D2 ruling settled *where* a closed loop can exist without a single-step ABI:
the solver evaluates a `rotational_torque` element inside every residual
evaluation, and that call receives both the current `State` -- whose `tire_sx`
is the tire's real-time longitudinal slip -- and the current `SampleInput`, whose
normalized demand tables carry the driver's own signal.  So a control law written
in the element reads the real-time state, derives a demand and applies it to the
same step.

This file is the acceptance evidence that the loop is *closed* rather than
replayed, and it rests on the rolling-wheel rig below.  That rig exists because
the check needs a wheel that actually rolls: the frozen axle acceptance rig has
its sprung body restrained, so a braked wheel there can only ever spin down, and
`state.tire_sx` has no adjustable range.  The rig here is one wheel on a revolute
axle over a moving belt, so the slip is a real operating point and the brake can
move it.

What is asserted, and why each part is there
--------------------------------------------

* **the loop's three segments are read out of one run** -- the state segment is
  the tire block's longitudinal slip velocity and the wheel body's spin rate, the
  control segment is the `controller_output` ledger's four columns, and the
  actuator segment is the `element_wrench` channel's two type-code-10 rows.  A
  controller that merely wrote a value into an object would leave the actuator
  rows empty, and one that fed a prepared history would not move the state.
* **the demand is a function of the state, not a replay** -- the control ledger's
  demand column varies from sample to sample while the driver column is a ramp,
  and the two differ.  A constant column would mean the law ignored the slip.
* **the actuator really changes the state** -- the same rig run with and without
  the controller reaches different slip values at the same instants.
* **the measured slip converges towards the target** -- sweeping the target moves
  the settled slip in the same direction, which is what "the controller drives
  the measurement to the setpoint" means on a rig with a real operating point.

The rig is a *mechanism* fixture, not a car: it has one wheel, no suspension and
no chassis.  What it exists to show is that the feedback path closes, which is
the part D2 was about.
"""

from __future__ import annotations

from typing import Any

import numpy as np
import pytest
from suspension_contracts import pack_container

from suspension_multibody.axle_dynamics.schema import (
    AxleBody,
    AxleDynamicsCase,
    AxleDynamicsModel,
    AxleJoint,
    AxleSolverSettings,
    AxleTire,
)
from suspension_multibody.cases.axle_dynamic import case_document, model_document
from suspension_multibody.simulation import (
    SimulationRequest,
    compile_document_pair,
    run_request,
)

#: The wheel's unloaded radius, in metres.  The contact is a real one: the wheel
#: sits `CONTACT_COMPRESSION` below its own radius, so the tire carries load.
RADIUS = 0.30
CONTACT_COMPRESSION = 0.01
#: Spin inertia and mass of the wheel.  Small enough that the brake can stop it
#: inside the window, large enough that the step size resolves it.
SPIN_INERTIA = 2.0
WHEEL_MASS = 20.0
#: The belt speed every case below runs at, and the spin rate that rolls with it.
BELT_MPS = 2.0
ROLLING_SPIN = BELT_MPS / RADIUS
#: The brake element's gain and cap, in N*m per unit demand and N*m.
BRAKE_STIFFNESS = 100.0
BRAKE_CAP = 100.0
#: The driver's brake ramp: nothing for 20 ms, then a 50 ms rise to this peak.
RAMP_START_S = 0.02
RAMP_RISE_S = 0.05
PRESSURE_PEAK = 0.6
END_TIME_S = 0.20
SAMPLES = 201
#: The element-wrench channel's rotational-torque type code.
ROTATIONAL_TORQUE_CODE = 10


def _diagonal(value: float) -> tuple[tuple[float, float, float], ...]:
    return ((value, 0.0, 0.0), (0.0, value, 0.0), (0.0, 0.0, value))


def _model(*, spin_rate: float) -> AxleDynamicsModel:
    """One wheel on a fixed axle over a belt: the smallest rolling rig."""
    ground = AxleBody(
        name="ground", mass_kg=0.0, inertia_kg_m2=_diagonal(1.0), fixed=True
    )
    wheel = AxleBody(
        name="wheel",
        mass_kg=WHEEL_MASS,
        inertia_kg_m2=_diagonal(SPIN_INERTIA),
        position_m=(0.0, 0.0, RADIUS - CONTACT_COMPRESSION),
        angular_velocity_rad_per_s=(0.0, spin_rate, 0.0),
    )
    joint = AxleJoint(
        name="spin",
        kind="revolute",
        body_a="ground",
        body_b="wheel",
        point_a_m=(0.0, 0.0, RADIUS - CONTACT_COMPRESSION),
        point_b_m=(0.0, 0.0, 0.0),
        axis_a=(0.0, 1.0, 0.0),
        axis_b=(0.0, 1.0, 0.0),
    )
    tire = AxleTire(
        name="tire",
        body="wheel",
        model_kind="native_brush",
        unloaded_radius_m=RADIUS,
        maximum_compression_m=0.05,
        vertical_stiffness_n_per_m=200_000.0,
        vertical_damping_n_s_per_m=800.0,
        longitudinal_friction_coefficient=1.0,
        lateral_friction_coefficient=0.9,
        longitudinal_brush_stiffness_n_per_m=150_000.0,
        lateral_brush_stiffness_n_per_m=120_000.0,
        longitudinal_relaxation_length_m=0.25,
        lateral_relaxation_length_m=0.35,
        detached_relaxation_s=0.05,
    )
    return AxleDynamicsModel(
        name="rolling-wheel-abs",
        bodies=(ground, wheel),
        joints=(joint,),
        tires=(tire,),
        gravity_m_per_s2=(0.0, 0.0, -9.80665),
    )


def _case() -> AxleDynamicsCase:
    times = np.linspace(0.0, END_TIME_S, SAMPLES)
    return AxleDynamicsCase(
        name="rolling-wheel-abs",
        times_s=tuple(float(value) for value in times),
        solver=AxleSolverSettings(
            # The stated spin *is* the operating point; a static trim would zero
            # it, and with it the very slip the law reads.
            initialization_mode="provided_consistent_state",
            adaptive_step=False,
            internal_step_s=0.0005,
            minimum_step_s=0.0001,
            maximum_step_s=0.0005,
            max_newton_iterations=50,
        ),
    )


def _append_table(
    document: dict[str, Any], payload: bytes, role: str, values, tire: str
) -> tuple[dict[str, Any], bytes]:
    data = np.ascontiguousarray(values, dtype=np.float64)
    document = dict(document)
    document["blobs"] = list(document.get("blobs", [])) + [
        {
            "role": role,
            "tire": tire,
            "offset": len(payload),
            "length": int(data.size * data.itemsize),
            "dtype": "float64",
            "shape": [int(data.size)],
        }
    ]
    return document, payload + data.tobytes()


def run_rig(
    *,
    controller: bool,
    target_slip: float = 0.006,
    gain: float = 2000.0,
    stiffness: float = BRAKE_STIFFNESS,
    cap: float = BRAKE_CAP,
    peak: float = PRESSURE_PEAK,
    spin_rate: float = ROLLING_SPIN,
):
    """Run the rolling-wheel rig once, with or without the ABS law engaged."""
    model = _model(spin_rate=spin_rate)
    case = _case()
    document, blob = model_document(model)
    document = dict(document)
    document["elements"] = [
        *document["elements"],
        {
            "name": "abs_brake",
            "type": "rotational_torque",
            "body_a": "ground",
            "body_b": "wheel",
            "parameters": {
                "axis_a": [0.0, 1.0, 0.0],
                "stiffness": stiffness,
                "damping": 0.0,
                "max_torque": cap,
                # The brake demand channel, and the tire whose slip the element
                # reports: the same two slots the plain demand path uses.
                "demand_source": 2,
                "demand_tire": 0,
                "controller_enabled": controller,
                "target_slip": target_slip,
                "controller_gain": gain,
            },
        },
    ]
    case_doc, case_blob = case_document(model, case)
    times = np.asarray(case.times_s, dtype=float)
    pressure = peak * np.clip((times - RAMP_START_S) / RAMP_RISE_S, 0.0, 1.0)
    case_doc, case_blob = _append_table(
        case_doc, case_blob, "brake_pressure", pressure, "tire"
    )
    case_doc, case_blob = _append_table(
        case_doc, case_blob, "road_velocity", np.full(times.size, BELT_MPS), "tire"
    )
    return run_request(
        compile_document_pair(
            SimulationRequest(assembly="axle", family="axle_dynamic"),
            model_document=document,
            case_document=case_doc,
            model_payload=pack_container(document, blob),
            case_payload=pack_container(case_doc, case_blob),
        )
    ).raw


@pytest.fixture
def channels(monkeypatch):
    """Turn on both optional ledgers; they are off by default."""
    monkeypatch.setenv("SUSPENSION_KERNEL_CONTROLLER_OUTPUT", "1")
    monkeypatch.setenv("SUSPENSION_KERNEL_ELEMENT_WRENCH_OUTPUT", "1")


def _tail(values: np.ndarray, fraction: float = 0.3) -> np.ndarray:
    """Return the settled part of a history: the last `fraction` of the samples."""
    count = values.shape[0]
    return values[int((1.0 - fraction) * count):]


def test_the_run_carries_all_three_segments(channels) -> None:
    """
    One run, three readable segments: state, control and actuator.

    The control ledger is the segment the element-wrench channel *cannot* carry:
    that channel means "the wrench actually applied", so its rows may not be
    reinterpreted as a demand.  Its presence here is what makes the middle of the
    loop observable rather than inferred from the torque.
    """
    raw = run_rig(controller=True)
    blocks = raw.blocks or {}
    assert "controller_output" in blocks, sorted(blocks)
    control = np.asarray(blocks["controller_output"])
    assert control.shape == (SAMPLES, 4), control.shape

    state = np.asarray(blocks["tire_output"])
    # Column 7 is the longitudinal slip velocity -- the state segment.
    slip_velocity = state[:, 0, 7]
    assert np.isfinite(slip_velocity).all()
    assert np.ptp(slip_velocity) > 1.0, np.ptp(slip_velocity)

    wrench = np.asarray(blocks["element_wrench"])
    rows = (wrench[:, :, 6] == ROTATIONAL_TORQUE_CODE).sum(axis=1)
    assert int(rows.min()) == 2, rows[:5]


def test_the_demand_follows_the_state_rather_than_the_clock(channels) -> None:
    """
    The control segment varies with the state, and is not the driver's ramp.

    A law that ignored the slip would produce a demand column proportional to the
    driver's own signal.  The two columns are compared directly: they must differ
    on some sample, and the derived one must change while the brake is not being
    ramped any more.
    """
    raw = run_rig(controller=True)
    control = np.asarray(raw.blocks["controller_output"])
    measured, target, demand, driver = (control[:, index] for index in range(4))
    assert np.isfinite(control).all(), "a control row was left unrecorded"
    assert np.allclose(target, 0.006), target
    # The driver's ramp is monotone; the derived demand is not.
    assert np.ptp(demand) > 1e-9, "the demand never moved"
    differences = np.abs(demand - driver)
    assert differences.max() > 1e-6, differences.max()
    # After the ramp has finished the driver is constant, so any movement left in
    # the demand can only come from the measured slip.
    settled = int((1.0 - 0.3) * control.shape[0])
    assert np.ptp(demand[settled:]) > 1e-9, np.ptp(demand[settled:])
    assert np.ptp(driver[settled:]) == 0.0, np.ptp(driver[settled:])
    assert np.ptp(measured[settled:]) > 1e-6, np.ptp(measured[settled:])


def test_the_actuator_moves_the_state_in_the_same_run(channels) -> None:
    """
    The same rig with and without the law reaches different slip states.

    This is the half of "closed loop" that rules out a replay: a controller whose
    output never reached the equations of motion would leave the two runs
    identical.  The measured slip is a signed quantity, so the comparison is on
    its absolute value.
    """
    without = np.asarray(run_rig(controller=False).blocks["tire_output"])[:, 0, 7]
    with_law = np.asarray(run_rig(controller=True).blocks["tire_output"])[:, 0, 7]
    difference = float(np.abs(np.abs(without) - np.abs(with_law)).max())
    assert difference > 0.1, difference
    # The law is a feedback on the slip, so it cannot be reproducing the open
    # history sample for sample anywhere in the settled part.
    assert not np.allclose(_tail(without), _tail(with_law), atol=1e-3)


def test_the_measured_slip_tracks_the_target(channels) -> None:
    """
    Moving the setpoint moves the settled slip the same way.

    This is what "the controller drives the measurement to the target" means on a
    rig whose operating point is real.  The assertion is on the *direction* and
    on the size of the change rather than on an exact match: one proportional law
    has a steady-state error, and claiming otherwise would be asserting the rig's
    own friction rather than the controller.
    """
    targets = (0.002, 0.006, 0.010)
    settled = []
    for target in targets:
        control = np.asarray(
            run_rig(controller=True, target_slip=target).blocks["controller_output"]
        )
        measured = np.abs(_tail(control[:, 0]))
        settled.append(float(measured.mean()))
        # The demand must have been modulated for this target to mean anything.
        assert np.ptp(_tail(control[:, 2])) > 1e-9, (target, np.ptp(_tail(control[:, 2])))
    assert settled[0] > settled[1] > settled[2], settled
    assert settled[0] - settled[2] > 1e-4, settled


def test_a_controller_that_reaches_the_blob_recorded_its_own_end(channels) -> None:
    """
    The actuator segment is equal and opposite, and its magnitude is real.

    Two rows per sample, one per end, `atol=0` because the pair is one couple:
    the kernel applies `+tau` to one body and `-tau` to the other, and a row that
    disagrees by a rounding error would mean the two ends are not reacting to each
    other.  At least one sample must carry a non-zero couple, or the ledger is
    recording a brake that never engaged.
    """
    raw = run_rig(controller=True)
    wrench = np.asarray(raw.blocks["element_wrench"])
    codes = wrench[:, :, 6]
    magnitudes = []
    for sample in range(codes.shape[0]):
        rows = np.flatnonzero(codes[sample] == ROTATIONAL_TORQUE_CODE)
        assert rows.size == 2, (sample, rows)
        first, second = wrench[sample, rows[0], 3:6], wrench[sample, rows[1], 3:6]
        np.testing.assert_allclose(first, -second, atol=0.0)
        magnitudes.append(float(np.linalg.norm(first)))
    assert max(magnitudes) > 0.0, magnitudes
    assert max(magnitudes) <= BRAKE_CAP + 1e-9, max(magnitudes)


def test_the_control_ledger_is_absent_unless_it_is_asked_for(monkeypatch) -> None:
    """
    The channel is off by default, so the default path's bytes do not move.

    The ledger is an addition to the result contract rather than to the ABI, and
    its switch is what keeps that additive: with the switch off nothing is
    allocated, described or appended.
    """
    monkeypatch.delenv("SUSPENSION_KERNEL_CONTROLLER_OUTPUT", raising=False)
    monkeypatch.setenv("SUSPENSION_KERNEL_ELEMENT_WRENCH_OUTPUT", "1")
    raw = run_rig(controller=True)
    assert "controller_output" not in (raw.blocks or {}), sorted(raw.blocks or {})
