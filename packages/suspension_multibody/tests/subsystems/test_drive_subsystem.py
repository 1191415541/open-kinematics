"""
The simplified drive subsystem: no bodies, and a torque element per driven wheel.

Three claims, in the order a build makes them:

1. the module produces a **torque element**, not a sampled dictionary, and the
   element's wheel torque equals the live builder's value for that wheel -- so
   the one element is a *spelling* of the formula the preparation layer already
   computes, not a second model of it.  The comparison is exact `==`, not
   `pytest.approx`: both sides perform the same floating-point multiplications,
   so any difference is a difference in the formula.  `pytest.approx` here would
   hide exactly the bug the test exists to catch;
2. the transfer slots (`gear_ratio`, `efficiency`, `max_torque`) are what drive
   it, and the identity transfer is what makes claim 1 hold;
3. which body reacts the couple comes from a matched port, never from a name in
   this package.

The live-builder comparison needs a real `VehicleModel`/`VehicleDynamicCase`, so
the fixture below builds one; the element checks use only the subsystem.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.connections.matcher import match_requirements
from suspension_multibody.modeling.identity import EntityId
from suspension_multibody.modeling.ports import GeometryPort, PortRequirement
from suspension_multibody.modeling.primitives import RotationalTorqueElement
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
from suspension_multibody.subsystems.element_build import build_element
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

#: The template's resolved transfer slots.  Identity on purpose: that is the
#: state in which the one element reproduces the live builder's value for value,
#: so `max_torque` reads as the wheel torque the driveline's own field states.
IDENTITY_SLOTS = {"gear_ratio": 1.0, "efficiency": 1.0}

#: The requirement the element fills, and the two ends of the couple.  The
#: reacting body's name is one no table inside the package carries.
SUBFRAME = "subframe"
WHEEL = "wheel_front_left"
INSTANCE = ("axle",)


def _slots(max_torque: float, *, gear_ratio: float | None = None, efficiency: float | None = None):
    """Return the template's resolved slot map for one element."""
    transfer = dict(IDENTITY_SLOTS)
    if gear_ratio is not None:
        transfer["gear_ratio"] = gear_ratio
    if efficiency is not None:
        transfer["efficiency"] = efficiency
    return {**transfer, "max_torque": max_torque}


def _offered(owner: str = SUBFRAME, names: tuple[str, ...] = ("reaction",)):
    """One offered geometric port per name, each owned by `owner`."""
    return {
        str(port.id): port
        for port in (
            GeometryPort(
                id=EntityId(INSTANCE, name),
                owner=EntityId(INSTANCE, owner),
                role=drive.DRIVE_REACTION_ROLE,
            )
            for name in names
        )
    }


def _element(
    driveline: DrivelineSpec,
    *,
    wheel: str,
    drive_input: float,
    owner: str = SUBFRAME,
    gear_ratio: float = 1.0,
    efficiency: float = 1.0,
):
    """Build the drive element the way a composition does: pairing, row."""
    ports = _offered(owner)
    report = match_requirements((PortRequirement(role=drive.DRIVE_REACTION_ROLE),), ports)
    return drive.wheel_torque_element(
        driveline,
        slots=_slots(
            driveline.maximum_drive_torque,
            gear_ratio=gear_ratio,
            efficiency=efficiency,
        ),
        wheel=wheel,
        own_body=WHEEL,
        report=report,
        ports=ports,
        drive=drive_input,
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


def test_the_three_standardized_slots_are_declared() -> None:
    """
    The roadmap's 2.1 slot set, by name, is what the drive role requires.

    `driven_wheels`/`drive_split` are gone and their absence is the change: with
    one element per driven wheel the driven set *is* the set of wheels that have
    an element, and the share is the element's own.  `maximum_drive_torque`
    became `max_torque`, which now reads as the motor's torque because the
    transfer is written out.
    """
    from suspension_multibody.templates import ROLES

    assert set(ROLES["drive"].required_slots) == {
        "gear_ratio",
        "efficiency",
        "max_torque",
    }
    declared = {slot.name for slot in drive.SIMPLIFIED_DRIVE.property_slots}
    assert not declared & {
        "driven_wheels",
        "drive_split",
        "maximum_drive_torque",
    }


def test_the_drive_slots_round_trip_unchanged() -> None:
    from suspension_multibody.templates import template_from_json, template_to_json

    template = drive.SIMPLIFIED_DRIVE
    assert template_from_json(template_to_json(template)) == template
    declared = {slot.name: slot.default for slot in template.property_slots}
    assert declared == {
        "gear_ratio": 1.0,
        "efficiency": 1.0,
        "max_torque": 0.0,
    }


def test_the_drive_torque_matches_the_live_build_value_for_value() -> None:
    """
    The element's wheel torque is the builder's number, exactly.

    Both an all-driven and a partly-driven split are checked, in wheel order,
    because the kernel reads the four buffers positionally.  The element is read
    through its `stiffness`, which is the value the element law uses as the
    amplitude at the demand it was built for.
    """
    times = np.asarray((0.0, 0.0005, 0.001))
    for driveline in (FRONT_DRIVE, REAR_DRIVE):
        model = _vehicle(driveline)
        case = _case(model, drive_input=TimeSignal(constant=0.75))
        built_drive, _built_brake = _build_wheel_torque_signals(model, case, times, 1.0)
        for name in WHEELS:
            element = _element(driveline, wheel=name, drive_input=0.75)
            assert element.spec.stiffness == built_drive[name][0], name
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
            element = _element(driveline, wheel=name, drive_input=0.75)
            assert element.spec.stiffness == expected, name
            assert element.spec.max_torque == pytest.approx(
                driveline.maximum_drive_torque * driveline.drive_split[share_index]
            ), name


def test_the_uncoupled_drive_input_sign_reaches_the_builder_exactly() -> None:
    """
    Reverse drive is a negative torque, and the amplitude keeps the sign.

    The live builder's driving branch does not absolute-value the demand, and
    neither does `drive_amplitude`: the parity with
    `_build_wheel_torque_signals` is exact at -1, which is what makes this a
    spelling of that formula rather than a second model of it.

    The *element*, however, carries the magnitude: the landed kernel family
    (`cpp/src/element/anti_roll.cpp:135-139`) is a resistance law whose demand
    channel is not wired yet, and `RotationalTorqueParameters` refuses a negative
    gain, so a signed couple is not expressible there today.  That limitation is
    asserted rather than hidden, and it is registered for the family's demand
    channel (see `torque_parameters`'s docstring).
    """
    times = np.asarray((0.0,))
    model = _vehicle(FRONT_DRIVE)
    case = _case(model, drive_input=TimeSignal(constant=-1.0))
    built_drive, _ = _build_wheel_torque_signals(model, case, times, 1.0)
    signed = drive.drive_amplitude(
        FRONT_DRIVE,
        slots=_slots(FRONT_DRIVE.maximum_drive_torque),
        wheel="front_left",
        drive=-1.0,
    )
    assert signed == -1_000.0
    assert signed == built_drive["front_left"][0]
    element = _element(FRONT_DRIVE, wheel="front_left", drive_input=-1.0)
    assert element.spec.stiffness == 1_000.0
    # The cap is the forward figure for both signs: a motor's limit is symmetric.
    assert element.spec.max_torque == 1_000.0
    assert _element(FRONT_DRIVE, wheel="rear_left", drive_input=-1.0).spec.stiffness == 0.0


def test_a_reduction_scales_the_wheel_torque() -> None:
    """
    The transfer slots are what make this template-driven, not a second copy.

    A 4:1 reduction at 0.9 efficiency and 500 N*mm of motor torque is 1800 N*mm
    at the wheel for a single driven wheel taking the whole split: the same wheel
    torque the driveline's lumped field states, now reached through the transfer
    the roadmap puts in the template.
    """
    driveline = DrivelineSpec(
        driven_wheels=("front_left",),
        maximum_drive_torque=1_800.0,
        drive_split=(1.0, 0.0, 0.0, 0.0),
    )
    amplifier = drive.drive_amplitude(
        driveline,
        slots=_slots(500.0, gear_ratio=4.0, efficiency=0.9),
        wheel="front_left",
        drive=1.0,
    )
    assert amplifier == 500.0 * 4.0 * 0.9 * 1.0 * 1.0
    assert amplifier == 1_800.0
    # The identity transfer is what makes the same field read as a wheel torque.
    assert drive.drive_amplitude(
        driveline,
        slots=_slots(1_800.0),
        wheel="front_left",
        drive=1.0,
    ) == 1_800.0


def test_an_out_of_range_drive_input_names_the_signal() -> None:
    for bad in (-1.5, 1.5):
        with pytest.raises(ValueError) as caught:
            drive.drive_amplitude(
                FRONT_DRIVE,
                slots=_slots(FRONT_DRIVE.maximum_drive_torque),
                wheel="front_left",
                drive=bad,
            )
        assert "drive_input" in str(caught.value)
        assert repr(bad) in str(caught.value)


def test_drive_torque_without_driven_wheels_is_refused() -> None:
    """
    The same refusal the live builder makes, reached the same way.

    `DrivelineSpec`'s own validator already rejects "torque on, no driven
    wheels", so the state below is reached by `model_copy`, which does not
    revalidate -- exactly the path that leaves `_build_wheel_torque_signals`
    defending itself.  The subsystem has to defend the same way: emitting zeros
    would look like a working driveline that drives nothing.
    """
    driveline = FRONT_DRIVE.model_copy(
        update={"driven_wheels": (), "drive_split": (0.0, 0.0, 0.0, 0.0)}
    )
    with pytest.raises(ValueError, match="driven_wheels"):
        drive.wheel_shares(driveline)


def test_a_driven_wheel_with_no_share_is_refused() -> None:
    """A zero share would be a silent zero for a wheel the model claims to drive."""
    driveline = FRONT_DRIVE.model_copy(update={"drive_split": (0.0, 1.0, 0.0, 0.0)})
    with pytest.raises(ValueError, match="front_left"):
        drive.wheel_shares(driveline)


def test_the_element_is_the_declared_family_and_builds() -> None:
    """What the module produces is a `rotational_torque` row, and it constructs."""
    row = _element(FRONT_DRIVE, wheel="front_left", drive_input=0.75)
    assert row.kind == "rotational_torque"
    assert row.name == "drive_front_left"
    element = build_element(row)
    assert isinstance(element, RotationalTorqueElement)
    assert (element.body_a, element.body_b) == (SUBFRAME, WHEEL)
    assert element.parameters.axis_a == pytest.approx(np.array([0.0, 1.0, 0.0]))


def test_the_reaction_body_comes_from_the_matched_port() -> None:
    """
    The reaction body is whatever the matched port says its owner is.

    Two owners are run through the *same* call -- the roadmap's subframe and a
    vehicle body -- and neither name appears in the production path, so the
    answer cannot be a name rule.
    """
    subframe = _element(FRONT_DRIVE, wheel="front_left", drive_input=1.0)
    body = _element(
        FRONT_DRIVE, wheel="front_left", drive_input=1.0, owner="vehicle_body"
    )
    assert subframe.body_a == SUBFRAME
    assert body.body_a == "vehicle_body"
    assert subframe.body_b == body.body_b == WHEEL


def test_the_drive_path_names_no_body_and_no_reacting_part() -> None:
    """The construction path carries no body-name rule."""
    from pathlib import Path

    source_root = Path(__file__).parents[2] / "src" / "suspension_multibody"
    text = (source_root / "subsystems" / "drive.py").read_text(encoding="utf-8").lower()
    for name in ("upright", "chassis"):
        assert name not in text, name
    # The fixture's own module and the production sources must not share a name
    # that could be mistaken for a rule, so the identity-slot map is asserted
    # here rather than left implicit.
    assert set(IDENTITY_SLOTS) == {"gear_ratio", "efficiency"}
