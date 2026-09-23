"""
The simplified brake: no bodies, five parameters, one arithmetic.

Three things are being proven here, and they are the three that make the
simplified brake a *provider* rather than a special case:

1. `build` contributes no rigid body.  That is the whole point of the simplified
   form (the Adams 0-body `_brake_system_4Wdisk.tpl`), and the assertion is on the
   produced mapping, not on the template's declaration -- a template can say
   "no parts" and still have something construct a body for it.
2. the five torque-subset parameters of decision D10 are declared, carry the
   Adams simple-brake values, and survive the JSON round trip unchanged.
3. the per-wheel amplitude is the `SFORCE/31-34` shape.  With the source's own
   constants and demand = 1.0 the front axle must come out at 17400 and the rear
   at 11600, and the amplitude must never be negative -- direction belongs to the
   kernel, which reads the wheel's axial speed sign.
"""

from __future__ import annotations

from suspension_multibody.subsystems import AssemblyRequest, SubsystemContext, brake
from suspension_multibody.templates import (
    ROLES,
    instantiate,
    template_from_json,
    template_to_json,
)
from tests.benchmark_fixture import benchmark_model

#: The frozen `.adm`'s constants, at demand = 1.0 (see
#: `artifacts/adams-fiala-handling/step_steer/adams_raw/handling_step_steer_dynamic.adm`,
#: `SFORCE/31-34`).
ADAMS_PARAMETERS = {
    "piston_area": 2500.0,
    "front_brake_bias": 0.6,
    "brake_mu": 0.4,
    "max_brake_value": 0.1,
    "effective_piston_radius": 145.0,
}

#: 2 * 2500 * 0.6 * 1.0 * 0.1 * 0.4 * 145
FRONT_AMPLITUDE = 17_400.0
#: 2 * 2500 * (1 - 0.6) * 1.0 * 0.1 * 0.4 * 145
REAR_AMPLITUDE = 11_600.0


def _context() -> SubsystemContext:
    return SubsystemContext(
        model=benchmark_model(), request=AssemblyRequest(mode="K")
    )


def _instance():
    return instantiate(brake.SIMPLIFIED_BRAKE, mode="K", properties={})


def test_the_simplified_brake_builds_no_body() -> None:
    output = brake.build(_instance(), _context())
    assert output.bodies == {}
    assert brake.SIMPLIFIED_BRAKE.parts == ()


def test_the_brake_role_contract_is_satisfied_and_complete() -> None:
    spec = ROLES["brake"]
    declared_slots = {slot.name for slot in brake.SIMPLIFIED_BRAKE.property_slots}
    assert declared_slots == set(spec.required_slots)
    declared_mounts = {connection.role for connection in brake.SIMPLIFIED_BRAKE.connections}
    assert set(spec.required_mounts) <= declared_mounts
    assert spec.has_torque_channel
    assert [output.name for output in brake.SIMPLIFIED_BRAKE.outputs] == [
        "brake_torque"
    ]
    # Checking the contract directly is what registration checks; doing it here
    # too means a broken template fails at the assertion, with the missing name.
    brake.SIMPLIFIED_BRAKE.check_role_contract()


def test_the_five_adams_parameters_round_trip_unchanged() -> None:
    template = brake.SIMPLIFIED_BRAKE
    assert template_from_json(template_to_json(template)) == template
    declared = {slot.name: slot.default for slot in template.property_slots}
    for name, expected in ADAMS_PARAMETERS.items():
        assert declared[name] == expected, name


def test_the_amplitude_matches_the_adams_sforce_shape() -> None:
    """
    Demand 1.0 gives the two recorded magnitudes and the recorded 60/40 split.

    The constants are the frozen document's own, so this is a check against
    `SFORCE/31-34` rather than against the module's arithmetic rearranged.
    """
    from suspension_multibody.schema import TimeSignal

    amplitudes = brake.wheel_torque_amplitudes(
        ADAMS_PARAMETERS, times=(0.0,), demand=TimeSignal(constant=1.0)
    )

    assert amplitudes["front_left"] == (FRONT_AMPLITUDE,)
    assert amplitudes["front_right"] == (FRONT_AMPLITUDE,)
    assert amplitudes["rear_left"] == (REAR_AMPLITUDE,)
    assert amplitudes["rear_right"] == (REAR_AMPLITUDE,)
    # 0.6 / 0.4, i.e. the bias shares add to one.
    assert amplitudes["front_left"][0] / amplitudes["rear_left"][0] == 1.5
    assert brake.bias_share(ADAMS_PARAMETERS, "front_left") == 0.6
    assert brake.bias_share(ADAMS_PARAMETERS, "rear_right") == 0.4


def test_the_amplitude_is_a_nonnegative_magnitude_at_every_demand() -> None:
    """
    The signal is a magnitude, not a signed torque.

    Adams decides direction with `STEP(<wheel speed>,-10,1,10,-1)`; the Python
    side must not pre-empt that, so no demand and no pair of parameters can make
    the output negative.  Half the demand is half the amplitude, which also pins
    that the demand enters linearly and unscaled.
    """
    from suspension_multibody.schema import TimeSignal

    full = brake.wheel_torque_amplitudes(
        ADAMS_PARAMETERS, times=(0.0, 0.5, 1.0), demand=TimeSignal(constant=1.0)
    )
    half = brake.wheel_torque_amplitudes(
        ADAMS_PARAMETERS, times=(0.0, 0.5, 1.0), demand=TimeSignal(constant=0.5)
    )
    for wheel, values in full.items():
        assert all(value >= 0.0 for value in values), wheel
        assert len(values) == 3
        assert half[wheel] == tuple(value / 2.0 for value in values), wheel


def test_an_out_of_range_demand_names_the_signal() -> None:
    from suspension_multibody.schema import TimeSignal

    for bad in (-0.1, 1.1):
        try:
            brake.wheel_torque_amplitudes(
                ADAMS_PARAMETERS, times=(0.0,), demand=TimeSignal(constant=bad)
            )
        except ValueError as error:
            assert "brake_input" in str(error)
            assert repr(bad) in str(error)
        else:  # pragma: no cover - the call must raise
            raise AssertionError(f"demand {bad} must be refused")


def test_an_unknown_wheel_is_refused_by_the_bias() -> None:
    try:
        brake.bias_share(ADAMS_PARAMETERS, "spare")
    except ValueError as error:
        assert "spare" in str(error)
    else:  # pragma: no cover - the call must raise
        raise AssertionError("an unknown wheel must not silently get zero bias")
