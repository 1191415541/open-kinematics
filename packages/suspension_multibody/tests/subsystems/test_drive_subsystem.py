"""
The simplified drive subsystem: no bodies, and a torque that matches the build.

"Matches the build" is the load-bearing claim.  The driving branch of
`preparation/vehicle_dynamic.py`'s `_build_wheel_torque_signals` already computes
a per-wheel drive torque, and this subsystem must be a *spelling* of that formula,
not a second model of it.  So the test runs the real function against a real
`VehicleModel`/`VehicleDynamicCase` and compares the two dictionaries entry by
entry, in wheel order.

The comparison is exact `==`, not `pytest.approx`: both sides perform the same
floating-point multiplications in the same association order, so any difference
is a difference in the formula, not a rounding artefact that deserves slack.
`pytest.approx` here would hide exactly the bug the test exists to catch.
"""

from __future__ import annotations

import numpy as np

from suspension_multibody.preparation.vehicle_dynamic import _build_wheel_torque_signals
from suspension_multibody.schema import (
    DrivelineSpec,
    DynamicSolverSettings,
    FrontAxleModel,
    MassSpec,
    RigidBodySpec,
    SteeringSystemSpec,
    TimeSignal,
    TireModelSpec,
    Vec3,
    VehicleDynamicCase,
    VehicleModel,
    WheelSpec,
)
from suspension_multibody.subsystems import AssemblyRequest, SubsystemContext, drive
from suspension_multibody.subsystems.types import WHEELS
from tests.benchmark_fixture import benchmark_model

#: A front-wheel-drive split, and a rear-wheel-drive split, so the comparison
#: covers both an all-driven and a partly-driven corner set.
FRONT_DRIVE = DrivelineSpec(
    driven_wheels=("front_left", "front_right"),
    maximum_drive_torque=2_000.0,
    drive_split=(0.5, 0.5, 0.0, 0.0),
)
REAR_DRIVE = DrivelineSpec(
    driven_wheels=("rear_left",),
    maximum_drive_torque=1_234.5,
    drive_split=(0.0, 0.0, 1.0, 0.0),
)


def _axle(name: str, x: float) -> FrontAxleModel:
    return FrontAxleModel(
        name=name,
        hardpoints={
            "UPPER_INBOARD_FRONT": Vec3(x=x, y=-500.0, z=500.0),
            "UPPER_INBOARD_REAR": Vec3(x=x + 150.0, y=-500.0, z=500.0),
            "UPPER_OUTBOARD": Vec3(x=x, y=-750.0, z=350.0),
            "LOWER_INBOARD_FRONT": Vec3(x=x, y=-500.0, z=100.0),
            "LOWER_INBOARD_REAR": Vec3(x=x + 150.0, y=-500.0, z=100.0),
            "LOWER_OUTBOARD": Vec3(x=x, y=-750.0, z=100.0),
            "TIE_ROD_INBOARD": Vec3(x=x, y=-450.0, z=250.0),
            "TIE_ROD_OUTBOARD": Vec3(x=x, y=-750.0, z=250.0),
            "WHEEL_CENTER": Vec3(x=x, y=-750.0, z=300.0),
            "RACK_CENTER": Vec3(x=x, y=0.0, z=250.0),
        },
        mass=MassSpec(sprung_mass=600.0),
        bodies=tuple(
            RigidBodySpec(name=body, mass=10.0)
            for body in (
                "rack",
                "upper_arm_L",
                "upper_arm_R",
                "lower_arm_L",
                "lower_arm_R",
                "upright_L",
                "upright_R",
                "tie_rod_L",
                "tie_rod_R",
            )
        ),
    )


def _vehicle(driveline: DrivelineSpec) -> VehicleModel:
    return VehicleModel(
        chassis=RigidBodySpec(name="chassis", mass=1200.0),
        front_axle=_axle("front", 1_400.0),
        rear_axle=_axle("rear", -1_400.0),
        wheels=tuple(
            WheelSpec(
                name=name,
                body=f"wheel_{name}",
                center_local=Vec3(),
                mass=20.0,
                axial_inertia=2.0,
                tire=TireModelSpec(kind="fiala", vertical_stiffness=20.0),
            )
            for name in WHEELS
        ),
        steering=SteeringSystemSpec(ratio=16.0),
        driveline=driveline,
    )


def _case(model: VehicleModel, *, drive_input: TimeSignal) -> VehicleDynamicCase:
    return VehicleDynamicCase(
        vehicle=model,
        solver=DynamicSolverSettings(end_time=0.002, step_size=0.001),
        drive_input=drive_input,
    )


def _context() -> SubsystemContext:
    # The drive subsystem owns no geometry, so any axle model serves as the
    # context; the benchmark fixture keeps this test off the vehicle fixtures.
    return SubsystemContext(
        model=benchmark_model(), request=AssemblyRequest(mode="K")
    )


def _instance():
    from suspension_multibody.templates import instantiate

    return instantiate(drive.SIMPLIFIED_DRIVE, mode="K", properties={})


def test_the_simplified_drive_builds_no_body() -> None:
    output = drive.build(_instance(), _context())
    assert output.bodies == {}
    assert drive.SIMPLIFIED_DRIVE.parts == ()


def test_the_drive_role_contract_is_satisfied_and_complete() -> None:
    from suspension_multibody.templates import ROLES

    spec = ROLES["drive"]
    declared_slots = {slot.name for slot in drive.SIMPLIFIED_DRIVE.property_slots}
    assert declared_slots == set(spec.required_slots)
    declared_mounts = {connection.role for connection in drive.SIMPLIFIED_DRIVE.connections}
    assert set(spec.required_mounts) <= declared_mounts
    assert spec.has_torque_channel
    assert [output.name for output in drive.SIMPLIFIED_DRIVE.outputs] == ["drive_torque"]
    drive.SIMPLIFIED_DRIVE.check_role_contract()


def test_the_drive_slots_round_trip_unchanged() -> None:
    from suspension_multibody.templates import template_from_json, template_to_json

    template = drive.SIMPLIFIED_DRIVE
    assert template_from_json(template_to_json(template)) == template
    declared = {slot.name: slot.default for slot in template.property_slots}
    assert declared == {
        "driven_wheels": 0.0,
        "drive_split": 0.0,
        "maximum_drive_torque": 0.0,
    }


def test_the_drive_torque_matches_the_live_build_value_for_value() -> None:
    """
    The subsystem's dictionary is the builder's dictionary, exactly.

    Both an all-driven and a partly-driven split are checked, and the comparison
    includes the wheel order, because the kernel reads the four buffers
    positionally.
    """
    times = np.asarray((0.0, 0.0005, 0.001))
    for driveline in (FRONT_DRIVE, REAR_DRIVE):
        model = _vehicle(driveline)
        case = _case(model, drive_input=TimeSignal(constant=0.75))
        built_drive, _built_brake = _build_wheel_torque_signals(model, case, times, 1.0)
        ours = drive.wheel_torque_amplitudes(
            driveline, times=times, drive=case.drive_input
        )
        assert list(ours) == list(WHEELS)
        assert list(ours) == list(built_drive)
        for name in WHEELS:
            assert ours[name] == built_drive[name], name
        # The value is the product with that wheel's own share, unscaled; the
        # share differs per corner, so read it from the split being tested.
        for share_index, name in enumerate(WHEELS):
            expected = (
                driveline.maximum_drive_torque
                * driveline.drive_split[share_index]
                * 0.75
                if name in driveline.driven_wheels
                else 0.0
            )
            assert ours[name] == (expected, expected, expected), name


def test_the_uncoupled_drive_input_sign_is_preserved() -> None:
    """
    Reverse drive is a negative torque; the subsystem must not abs() it.

    Only the brake is a non-negative magnitude.  A drive torque follows the sign
    of the demand, and the live builder does the same, so both must agree at -1.
    """
    times = np.asarray((0.0,))
    model = _vehicle(FRONT_DRIVE)
    case = _case(model, drive_input=TimeSignal(constant=-1.0))
    built_drive, _ = _build_wheel_torque_signals(model, case, times, 1.0)
    ours = drive.wheel_torque_amplitudes(
        FRONT_DRIVE, times=times, drive=case.drive_input
    )
    assert ours["front_left"] == (-1_000.0,)
    assert ours["front_left"] == built_drive["front_left"]
    assert ours["rear_left"] == (0.0,)


def test_an_out_of_range_drive_input_names_the_signal() -> None:
    for bad in (-1.5, 1.5):
        try:
            drive.wheel_torque_amplitudes(
                FRONT_DRIVE, times=(0.0,), drive=TimeSignal(constant=bad)
            )
        except ValueError as error:
            assert "drive_input" in str(error)
            assert repr(bad) in str(error)
        else:  # pragma: no cover - the call must raise
            raise AssertionError(f"drive_input {bad} must be refused")


def test_drive_torque_without_driven_wheels_is_refused() -> None:
    """
    The same refusal the live builder makes, reached the same way.

    `DrivelineSpec`'s own validator already rejects "torque on, no driven
    wheels", so the state below is reached by `model_copy`, which does not
    revalidate -- exactly the path that leaves `_build_wheel_torque_signals`
    defending itself.  The subsystem has to defend the same way: emitting four
    zeros would look like a working driveline that drives nothing.
    """
    driveline = FRONT_DRIVE.model_copy(
        update={"driven_wheels": (), "drive_split": (0.0, 0.0, 0.0, 0.0)}
    )
    try:
        drive.wheel_torque_amplitudes(
            driveline, times=(0.0,), drive=TimeSignal(constant=1.0)
        )
    except ValueError as error:
        assert "driven_wheels" in str(error)
    else:  # pragma: no cover - the call must raise
        raise AssertionError("a drive torque needs a driven wheel")
