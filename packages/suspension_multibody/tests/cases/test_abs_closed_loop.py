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
the wheel's slip has no adjustable range.  The rig here is one wheel on a
revolute axle carried by a body that travels forward, so the slip is a real
operating point the brake can move.

Three properties of the rig were measured before it was written down, and each
one is the reason a simpler rig did not work:

* **the tire's contact frame is the carrier, not the wheel.**  `frame_body`
  defaults to the tire's own body, and the frame carries the `forward` axis the
  slip is measured along -- on a spinning wheel that axis rolls, and the slip
  alternates sign every half turn.
* **the brake torque is the size that settles the slip inside the window.**  The
  wheel's spin-down time constant goes as `I*V/(r_l^2*c)`, so an arbitrarily
  large brake does not settle a working point, it locks the wheel.
* **the open-loop slip has to overshoot the setpoint.**  The control law's
  authority is ``clamp(1 + gain*(target - |slip|), 0, 1)``: it is 1 for every
  slip at or below the target, so a controller can only ever *reduce* the brake.
  A rig whose open-loop slip stops short of the target makes the two runs
  identical, and the assertions below would pass while proving nothing.

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
* **the measured slip converges to the target** -- on the settled tail the
  measured slip sits within the set tolerances of the setpoint, while the same
  rig without the law sits well outside them.
* **moving the setpoint moves the measurement, and moves the demand with it** --
  the settled slip tracks the target across a sweep of targets, and the settled
  demand differs between two targets under an identical driver signal, which is
  what "the control acts on the state" means once the ramp is over.

The rig is a *mechanism* fixture, not a car: it has one wheel, one travelling
carrier and no suspension.  What it exists to show is that the feedback path
closes and that the measurement converges, which is the part D2 was about.
"""

from __future__ import annotations

from typing import Any

import numpy as np
import pytest

from suspension_multibody.api import simulate
from suspension_multibody.authoring import (
    AssemblyDocument,
    SubsystemDocument,
    TemplateDocument,
)
from suspension_multibody.authoring.migration import migrate_v1_dynamic_axle
from suspension_multibody.axle_dynamics.schema import (
    AxleBody,
    AxleDynamicsCase,
    AxleDynamicsModel,
    AxleJoint,
    AxleSolverSettings,
    AxleTire,
)

#: The wheel's unloaded radius, in metres.  The contact is a real one: the wheel
#: sits `CONTACT_COMPRESSION` below its own radius, so the tire carries load.
RADIUS = 0.30
CONTACT_COMPRESSION = 0.01
#: Spin inertia and mass of the wheel.  The inertia is what decides how fast the
#: brake can move the slip: the spin-down time constant goes as
#: ``I*V/(r_l^2*cslip)``, so a large one would spend the whole case transient.
SPIN_INERTIA = 0.02
WHEEL_MASS = 20.0
#: The carrier's mass.  It travels forward under the tire's drag, so it is what
#: sets how much the road speed falls while the brake is applied.
CARRIER_MASS = 1500.0
#: Forward speed both bodies start at, and the spin rate that rolls with it.
#: The wheel spins on the *loaded* radius, which is the radius the tire's own
#: rolling speed uses.
FORWARD_MPS = 10.0
ROLLING_RADIUS = RADIUS - CONTACT_COMPRESSION
ROLLING_SPIN = FORWARD_MPS / ROLLING_RADIUS
#: The brake element: the driver's full command, and the cap it may not pass.
BRAKE_STIFFNESS = 200.0
BRAKE_CAP = 400.0
#: The law's own settings, and the setpoint the assertions are about.
CONTROLLER_GAIN = 30.0
TARGET_SLIP = 0.30
#: The driver's brake ramp: nothing for 8 ms, then a 20 ms rise to full.
RAMP_START_S = 0.008
RAMP_RISE_S = 0.02
PRESSURE_PEAK = 1.0
END_TIME_S = 0.20
SAMPLES = 201
#: The settled part of the history: the last 40% of the samples.  The ramp is
#: over by 28 ms and the wheel's own time constant is about 13 ms, so by the
#: window's start (120 ms) both transients have died.
SETTLED = slice(120, SAMPLES)
#: The tolerances the settled slip is held to.  They are absolute error on a
#: dimensionless slip ratio, and they are met by this rig with room to spare
#: (measured: mean and max both 0.018 at the asserted gain).
CONVERGED_MEAN_ERROR = 0.03
CONVERGED_MAX_ERROR = 0.05
#: Without the law the same rig settles far outside the band; the gap between
#: the two is the evidence that the loop is doing the work.
UNCONTROLLED_MIN_ERROR = 0.10
#: And the controlled error has to be a clear fraction of the uncontrolled one,
#: or the tolerances above would be met by a rig the brake barely moves.
CONTROLLED_FRACTION = 0.8
#: The element-wrench channel's rotational-torque type code.
ROTATIONAL_TORQUE_CODE = 10


def _diagonal(value: float) -> tuple[tuple[float, float, float], ...]:
    return ((value, 0.0, 0.0), (0.0, value, 0.0), (0.0, 0.0, value))


def _model() -> AxleDynamicsModel:
    """One wheel on a travelling carrier: the smallest rig with a real slip."""
    ground = AxleBody(
        name="ground", mass_kg=0.0, inertia_kg_m2=_diagonal(1.0), fixed=True
    )
    carrier = AxleBody(
        name="carrier",
        mass_kg=CARRIER_MASS,
        inertia_kg_m2=_diagonal(50.0),
        position_m=(0.0, 0.0, ROLLING_RADIUS),
        linear_velocity_m_per_s=(FORWARD_MPS, 0.0, 0.0),
    )
    wheel = AxleBody(
        name="wheel",
        mass_kg=WHEEL_MASS,
        inertia_kg_m2=_diagonal(SPIN_INERTIA),
        position_m=(0.0, 0.0, ROLLING_RADIUS),
        linear_velocity_m_per_s=(FORWARD_MPS, 0.0, 0.0),
        angular_velocity_rad_per_s=(0.0, ROLLING_SPIN, 0.0),
    )
    guide = AxleJoint(
        name="guide",
        kind="prismatic",
        body_a="ground",
        body_b="carrier",
        point_a_m=(0.0, 0.0, ROLLING_RADIUS),
        point_b_m=(0.0, 0.0, 0.0),
        axis_a=(1.0, 0.0, 0.0),
        axis_b=(1.0, 0.0, 0.0),
    )
    spin = AxleJoint(
        name="spin",
        kind="revolute",
        body_a="carrier",
        body_b="wheel",
        point_a_m=(0.0, 0.0, 0.0),
        point_b_m=(0.0, 0.0, 0.0),
        axis_a=(0.0, 1.0, 0.0),
        axis_b=(0.0, 1.0, 0.0),
    )
    tire = AxleTire(
        name="tire",
        body="wheel",
        # The frame the contact is evaluated in.  It has to be the *carrier*:
        # `forward_axis_local` is resolved in this body, and on the spinning
        # wheel that axis rolls with the tire, which makes the measured slip
        # alternate sign rather than settle.
        frame_body="carrier",
        frame_center_local_m=(0.0, 0.0, 0.0),
        model_kind="fiala",
        unloaded_radius_m=RADIUS,
        maximum_compression_m=0.05,
        vertical_stiffness_n_per_m=200_000.0,
        vertical_damping_n_s_per_m=800.0,
        longitudinal_friction_coefficient=1.0,
        lateral_friction_coefficient=0.9,
        longitudinal_brush_stiffness_n_per_m=150_000.0,
        lateral_brush_stiffness_n_per_m=120_000.0,
        # The relaxation length is what separates the two time scales of the
        # loop.  At this value the relaxed slip reaches its kinematic value
        # inside the case instead of still rising through it.
        longitudinal_relaxation_length_m=1.0,
        lateral_relaxation_length_m=0.35,
        detached_relaxation_s=0.05,
    )
    return AxleDynamicsModel(
        name="rolling-carrier-abs",
        bodies=(ground, carrier, wheel),
        joints=(guide, spin),
        tires=(tire,),
        gravity_m_per_s2=(0.0, 0.0, -9.80665),
    )


def _case() -> AxleDynamicsCase:
    times = np.linspace(0.0, END_TIME_S, SAMPLES)
    return AxleDynamicsCase(
        name="rolling-carrier-abs",
        times_s=tuple(float(value) for value in times),
        solver=AxleSolverSettings(
            # The stated states *are* the operating point; a static trim would
            # zero the spin, and with it the very slip the law reads.
            initialization_mode="provided_consistent_state",
            adaptive_step=False,
            internal_step_s=END_TIME_S / (SAMPLES - 1) / 4.0,
            minimum_step_s=END_TIME_S / (SAMPLES - 1) / 40.0,
            maximum_step_s=END_TIME_S / (SAMPLES - 1) / 4.0,
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
    target_slip: float = TARGET_SLIP,
    gain: float = CONTROLLER_GAIN,
    stiffness: float = BRAKE_STIFFNESS,
    cap: float = BRAKE_CAP,
    peak: float = PRESSURE_PEAK,
):
    """Run the rig once, with or without the ABS law engaged."""
    model = _model()
    case = _case()
    assembly, declared = migrate_v1_dynamic_axle(model, case)
    template = TemplateDocument.from_payload({"document": "template", "schema_version": 1,
        "name": "abs_brake", "functional_role": "brake", "allowed_placement_roles": ["any"],
        "symmetry": "asymmetric", "units": {"length": "m"}, "bodies": [], "hardpoints": [], "joints": [],
        "needs": [{"name": name, "role": "body:"+name, "count": 1, "required": True} for name in ("carrier", "wheel")],
        "property_slots": [{"name": "law", "element_type": "generic", "required": False, "default": 0}],
        "elements": [{
            "name": "abs_brake",
            "type": "rotational_torque",
            "body_a": "@carrier",
            "body_b": "@wheel",
            "point_a": "@carrier",
            "point_b": "@wheel",
            "property_slot": "law",
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
        }]})
    brake = SubsystemDocument.from_payload({"document": "subsystem", "schema_version": 1,
        "name": "abs_brake", "template": "abs.tpl.json", "functional_role": "brake",
        "placement_role": "any", "hardpoints": {}, "property_bindings": {}}, template=template)
    payload = assembly.to_payload()
    payload["subsystems"].append({"ref": "brake", "functional_role": "brake", "placement_role": "any"})
    assembly = AssemblyDocument.from_payload(payload,
        subsystems={**{entry.ref: entry.subsystem for entry in assembly.entries}, "brake": brake})
    times = np.asarray(case.times_s, dtype=float)
    pressure = peak * np.clip((times - RAMP_START_S) / RAMP_RISE_S, 0.0, 1.0)
    plan = declared.to_payload()
    plan["inputs"].append({"name": "brake_pressure", "role": "brake_pressure", "tire": "wheel.tire", "values": pressure.tolist()})
    # The road's own speed.  The kernel composes it as the *vertical* road
    # velocity, so a carriage that travels forward states its speed on the body
    # rather than in this table -- and the road here is level and still.
    plan["inputs"].append({"name": "road_velocity", "role": "road_velocity", "tire": "wheel.tire", "values": np.zeros(times.size).tolist()})
    return simulate(assembly, plan).raw


def _slip(raw, window: slice = SETTLED) -> np.ndarray:
    """
    Return the settled dimensionless slip ratio from the tire block.

    Column 10 is the relaxed longitudinal slip the controller reads -- not
    column 7, which is the slip *velocity* in metres per second.
    """
    return np.abs(np.asarray(raw.named_blocks["tire_output"])[:, 0, 10])[window]


@pytest.fixture
def channels(monkeypatch):
    """Turn on both optional ledgers; they are off by default."""
    monkeypatch.setenv("SUSPENSION_KERNEL_CONTROLLER_OUTPUT", "1")
    monkeypatch.setenv("SUSPENSION_KERNEL_ELEMENT_WRENCH_OUTPUT", "1")


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
    on some sample, and the derived one must settle somewhere the driver's own
    column is no longer moving.
    """
    raw = run_rig(controller=True)
    control = np.asarray(raw.blocks["controller_output"])
    measured, target, demand, driver = (control[:, index] for index in range(4))
    assert np.isfinite(control).all(), "a control row was left unrecorded"
    assert np.allclose(target, TARGET_SLIP), target
    # The driver's ramp is monotone; the derived demand is not identical to it.
    assert np.ptp(demand) > 1e-9, "the demand never moved"
    differences = np.abs(demand - driver)
    assert differences.max() > 1e-6, differences.max()
    # After the ramp has finished the driver is constant, so the whole settled
    # story is in the measured slip and the demand it produced.
    assert np.ptp(driver[SETTLED]) == 0.0, np.ptp(driver[SETTLED])
    assert 0.0 < demand[SETTLED][-1] < driver[SETTLED][-1], (
        "the law should hold the demand below the driver's command once it has "
        f"something to modulate: {demand[SETTLED][-1]} vs {driver[SETTLED][-1]}"
    )
    # The slip is the settled operating point, so the demand's own movement in
    # the settled window is a small correction rather than a new ramp.
    assert np.ptp(demand[SETTLED]) < 1e-3, np.ptp(demand[SETTLED])


def test_the_actuator_moves_the_state_in_the_same_run(channels) -> None:
    """
    The same rig with and without the law reaches different slip states.

    This is the half of "closed loop" that rules out a replay: a controller whose
    output never reached the equations of motion would leave the two runs
    identical.  The measured slip is a signed quantity, so the comparison is on
    its absolute value.
    """
    without = _slip(run_rig(controller=False), slice(0, SAMPLES))
    with_law = _slip(run_rig(controller=True), slice(0, SAMPLES))
    difference = float(np.abs(without - with_law).max())
    assert difference > 0.05, difference


def test_the_measured_slip_converges_to_the_target(channels) -> None:
    """
    The setpoint is reached: the settled measurement lies in the target band.

    This is the assertion the whole rig exists for, and it is a statement about
    the measurement, not about a comparison with the uncontrolled run: on the
    settled tail the slip is within ``CONVERGED_*_ERROR`` of the target.  The
    uncontrolled run's error is checked too, because a law that could not move
    the operating point would otherwise satisfy the band by luck.
    """
    controlled = _slip(run_rig(controller=True))
    uncontrolled = _slip(run_rig(controller=False))
    controlled_error = np.abs(controlled - TARGET_SLIP)
    uncontrolled_error = np.abs(uncontrolled - TARGET_SLIP)
    assert controlled_error.mean() <= CONVERGED_MEAN_ERROR, controlled_error.mean()
    assert controlled_error.max() <= CONVERGED_MAX_ERROR, controlled_error.max()
    # The brake really has the authority to miss the target when nothing
    # modulates it, and the controlled error is a clear fraction of that miss.
    assert uncontrolled_error.mean() >= UNCONTROLLED_MIN_ERROR, (
        uncontrolled_error.mean()
    )
    assert controlled_error.mean() <= CONTROLLED_FRACTION * uncontrolled_error.mean()
    # The controlled tail is *settled*, not merely inside the band on average:
    # if it were still moving the average could sit inside while the slip
    # crossed the whole band between samples.
    assert np.ptp(controlled) < 1e-3, np.ptp(controlled)


def test_moving_the_setpoint_moves_the_measurement(channels) -> None:
    """
    The settled slip tracks the setpoint across a sweep of targets.

    A rig whose measurement ignored the law's target would settle at the same
    slip for every one of them.  The assertion is on the settled measurement, and
    on the demand that produced it: the driver's signal is identical across the
    sweep, so a demand that changed with the target can only have come from the
    law.
    """
    targets = (0.15, 0.20, 0.25, 0.30, 0.35)
    settled: list[float] = []
    demands: list[float] = []
    for target in targets:
        control = np.asarray(
            run_rig(controller=True, target_slip=target).blocks["controller_output"]
        )
        measured = np.abs(control[SETTLED, 0])
        settled.append(float(measured.mean()))
        demands.append(float(control[SETTLED, 2].mean()))
        # Each target is a real working point: the measured slip is settled.
        assert np.ptp(measured) < 1e-3, (target, np.ptp(measured))
    # Monotone in the target, with a real gap between the ends.
    assert all(
        first < second for first, second in zip(settled, settled[1:])
    ), settled
    assert settled[-1] - settled[0] > 0.1, settled
    # The driver signal does not move with the target; the demand does.
    assert all(
        abs(demand - demands[0]) > 1e-6 for demand in demands[1:]
    ), demands


def test_a_controller_that_reaches_the_blob_recorded_its_own_end(channels) -> None:
    """
    The actuator segment is equal and opposite, and its magnitude is real.

    Two rows per sample, one per end, `atol=0` because the pair is one couple:
    the kernel applies `+tau` to one body and `-tau` to the other, and a row that
    disagrees by a rounding error would mean the two ends are not reacting to
    each other.  At least one sample must carry a non-zero couple, or the ledger
    is recording a brake that never engaged, and none may exceed the cap.
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
