"""
The simplified brake: no bodies, a standardized slot set, one torque element.

Four things are being proven here, and they are the four that make the
simplified brake a *provider* rather than a special case:

1. `build` contributes no rigid body.  That is the whole point of the simplified
   form (the Adams 0-body `_brake_system_4Wdisk.tpl`), and the assertion is on the
   produced mapping, not on the template's declaration -- a template can say
   "no parts" and still have something construct a body for it.
2. the standardized slot set of the roadmap's 2.1 section is declared and
   survives the JSON round trip unchanged;
3. the magnitude is the `SFORCE/31-34` shape.  With the source's own constants
   and demand = 1.0 at the recorded 60/40 split the front axle must come out at
   17400 and the rear at 11600, and no demand and no share can make it negative;
4. what the module produces is a **torque element**, and which body reacts its
   couple is decided by the matched port -- never by a name in this package.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.connections.matcher import match_requirements
from suspension_multibody.modeling.identity import EntityId
from suspension_multibody.modeling.ports import GeometryPort, PortRequirement
from suspension_multibody.modeling.primitives.elements import (
    RotationalTorqueElement,
)
from suspension_multibody.subsystems import AssemblyRequest, SubsystemContext, brake
from suspension_multibody.subsystems.element_build import build_element
from suspension_multibody.templates import (
    ROLES,
    instantiate,
    template_from_json,
    template_to_json,
)
from tests.benchmark_fixture import benchmark_model

#: The frozen `.adm`'s constants, at demand = 1.0 (see
#: `artifacts/adams-fiala-handling/step_steer/adams_raw/handling_step_steer_dynamic.adm`,
#: `SFORCE/31-34`), under the standardized slot names.
ADAMS_PARAMETERS = {
    "piston_area": 2500.0,
    "effective_radius": 145.0,
    "friction_coeff": 0.4,
    "rotor_inertia": 0.0568,
}

#: 2 * 2500 * 0.6 * 1.0 * 0.1 * 0.4 * 145
FRONT_AMPLITUDE = 17_400.0
#: 2 * 2500 * 0.4 * 1.0 * 0.1 * 0.4 * 145
REAR_AMPLITUDE = 11_600.0

#: The requirement the torque element fills and the two ends of the couple.
#:
#: The requiring side's own body is the wheel end.  The reacting body is
#: deliberately named something no table inside the package carries, so "the
#: port decided it" is checkable rather than assumed.
REACTION_PORT = "reaction"
CALIPER = "caliper_carrier_L"
WHEEL_END = "wheel_carrier"
INSTANCE = ("axle",)


def _context() -> SubsystemContext:
    return SubsystemContext(
        model=benchmark_model(), request=AssemblyRequest(mode="K")
    )


def _instance():
    return instantiate(brake.SIMPLIFIED_BRAKE, mode="K", properties={})


def _offered(owner: str = CALIPER, names: tuple[str, ...] = (REACTION_PORT,)):
    """One offered geometric port per name, each owned by `owner`."""
    return {
        str(port.id): port
        for port in (
            GeometryPort(
                id=EntityId(INSTANCE, name),
                owner=EntityId(INSTANCE, owner),
                role=brake.BRAKE_REACTION_ROLE,
            )
            for name in names
        )
    }


def _element(
    *,
    wheel: str = "front_left",
    share: float = 0.6,
    demand: float = 1.0,
    owner: str = CALIPER,
    parameters: dict[str, float] | None = None,
):
    """Build the brake element the way a composition does: pairing, row."""
    ports = _offered(owner)
    report = match_requirements(
        (PortRequirement(role=brake.BRAKE_REACTION_ROLE),), ports
    )
    return brake.wheel_torque_element(
        ADAMS_PARAMETERS if parameters is None else parameters,
        wheel=wheel,
        own_body=WHEEL_END,
        report=report,
        ports=ports,
        demand=demand,
        share=share,
    )


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


def test_the_four_standardized_slots_are_declared() -> None:
    """
    The roadmap's 2.1 slot set, by name, is what the brake role requires.

    Asserted as an exact set rather than as "these four are present": the point
    of standardizing the names is that a *detailed* template can declare the same
    four and be substituted, and a set assertion is what makes a fifth name (or
    the old `brake_mu`) a failure rather than a detail.
    """
    assert set(ROLES["brake"].required_slots) == {
        "piston_area",
        "effective_radius",
        "friction_coeff",
        "rotor_inertia",
    }
    # The retired names are gone, and their absence is the change: `brake_mu`
    # became `friction_coeff`, `effective_piston_radius` became
    # `effective_radius`, `max_brake_value` became `brake.DEMAND_SCALE`, and
    # `front_brake_bias` became the element's own `share`.
    declared = {slot.name for slot in brake.SIMPLIFIED_BRAKE.property_slots}
    assert not declared & {
        "brake_mu",
        "effective_piston_radius",
        "max_brake_value",
        "front_brake_bias",
    }


def test_the_standardized_slots_round_trip_unchanged() -> None:
    template = brake.SIMPLIFIED_BRAKE
    assert template_from_json(template_to_json(template)) == template
    declared = {slot.name: slot.default for slot in template.property_slots}
    for name, expected in ADAMS_PARAMETERS.items():
        assert declared[name] == expected, name


def test_the_amplitude_matches_the_adams_sforce_shape() -> None:
    """
    Demand 1.0 at the recorded 60/40 split gives the two recorded magnitudes.

    The constants are the frozen document's own, so this is a check against
    `SFORCE/31-34` rather than against the module's arithmetic rearranged.  The
    split is now the element's own `share`, which is where the deleted
    `front_brake_bias` slot's 0.6/0.4 went.
    """
    front = _element(wheel="front_left", share=0.6)
    rear = _element(wheel="rear_left", share=0.4)

    assert front.spec.stiffness == FRONT_AMPLITUDE
    assert rear.spec.stiffness == REAR_AMPLITUDE
    # 0.6 / 0.4, i.e. the shares add to one.
    assert front.spec.stiffness / rear.spec.stiffness == 1.5
    assert brake.brake_amplitude(
        ADAMS_PARAMETERS, demand=1.0, share=0.6
    ) == FRONT_AMPLITUDE
    assert brake.brake_amplitude(
        ADAMS_PARAMETERS, demand=1.0, share=0.4
    ) == REAR_AMPLITUDE


def test_the_amplitude_is_a_nonnegative_magnitude_at_every_demand() -> None:
    """
    The torque is a magnitude, not a signed value.

    Adams decides direction with `STEP(<wheel speed>,-10,1,10,-1)`; the element
    law does the same thing from the real-time relative rate, so no demand and no
    share can make the declared magnitude negative.  Half the demand is half the
    magnitude, which also pins that the demand enters linearly and unscaled.
    """
    full = brake.brake_amplitude(ADAMS_PARAMETERS, demand=1.0, share=0.6)
    half = brake.brake_amplitude(ADAMS_PARAMETERS, demand=0.5, share=0.6)
    assert full == FRONT_AMPLITUDE
    assert half == pytest.approx(full / 2.0, abs=1e-9)
    for share in (0.0, 0.25, 0.6, 1.0):
        assert brake.brake_amplitude(ADAMS_PARAMETERS, demand=0.0, share=share) == 0.0


def test_the_element_caps_at_the_full_demand_magnitude() -> None:
    """
    `max_torque` is the recorded full-demand figure, so the cap never bites.

    The element law is `min(stiffness * demand, max_torque)` and the kernel's own
    demand is a unit demand today (`cpp/src/element/anti_roll.cpp:132`), so the
    cap is what keeps a rising demand from raising the couple past the source
    document's amplitude.  Half the demand against a full-demand cap is the
    assertion that the two halves of the pair are the same number.
    """
    element = _element(demand=0.5, share=0.6)
    assert element.spec.stiffness == FRONT_AMPLITUDE / 2.0
    assert element.spec.max_torque == FRONT_AMPLITUDE
    assert element.spec.magnitude == FRONT_AMPLITUDE / 2.0


def test_an_out_of_range_demand_names_the_signal() -> None:
    for bad in (-0.1, 1.1):
        with pytest.raises(ValueError) as caught:
            brake.brake_amplitude(ADAMS_PARAMETERS, demand=bad, share=0.6)
        assert "brake_input" in str(caught.value)
        assert repr(bad) in str(caught.value)


def test_an_unknown_wheel_is_refused_by_name() -> None:
    """A wheel the model does not have is named, not silently coupled."""
    ports = _offered()
    report = match_requirements(
        (PortRequirement(role=brake.BRAKE_REACTION_ROLE),), ports
    )
    with pytest.raises(ValueError, match="spare"):
        brake.wheel_torque_element(
            ADAMS_PARAMETERS,
            wheel="spare",
            own_body=WHEEL_END,
            report=report,
            ports=ports,
            demand=1.0,
        )


def test_the_element_is_the_declared_family_and_builds() -> None:
    """
    What the module produces is a `rotational_torque` row, and it constructs.

    Asserted through the public construction entry point rather than by reading
    the row's fields alone: a row no constructor accepts would be a declaration
    that never reaches a model.
    """
    row = _element()
    assert row.kind == "rotational_torque"
    assert row.name == "brake_front_left"
    element = build_element(row)
    assert isinstance(element, RotationalTorqueElement)
    assert (element.body_a, element.body_b) == (CALIPER, WHEEL_END)
    assert element.parameters.axis_a == pytest.approx(np.array([0.0, 1.0, 0.0]))


def test_the_reaction_body_comes_from_the_matched_port() -> None:
    """
    The reaction body is whatever the matched port says its owner is.

    Two different owners are run through the *same* call, one of them the name
    the roadmap uses for a brake's reaction (`upright`) and one a caliper
    carrier: nothing here can be reading a name, because both work and neither
    name appears in the production path.
    """
    caliper = _element(owner=CALIPER)
    upright = _element(owner="upright_L")
    assert caliper.body_a == CALIPER
    assert upright.body_a == "upright_L"
    # The driven end is the requiring side's own body both times.
    assert caliper.body_b == upright.body_b == WHEEL_END


def test_a_missing_reaction_port_is_refused_by_role() -> None:
    """
    A wheel the assembly never bound a reaction port for fails by role.

    The report below is what the matcher returns for a model whose only
    requirements are *other* than the one this element fills -- an optional
    branch that disappeared, which is the state a model is in when nothing
    offers a brake reaction.  `pair_torque_bodies` refuses by role name rather
    than falling back to whatever body the caller happened to pass.
    """
    ports = _offered()
    report = match_requirements(
        (PortRequirement(role="some_other_need", required=False),), ports
    )
    assert report.bindings == ()
    assert report.binding_for(brake.BRAKE_REACTION_ROLE) is None
    with pytest.raises(ValueError, match=brake.BRAKE_REACTION_ROLE):
        brake.wheel_torque_element(
            ADAMS_PARAMETERS,
            wheel="front_left",
            own_body=WHEEL_END,
            report=report,
            ports=ports,
            demand=1.0,
        )


def test_the_brake_path_names_no_body_and_no_reacting_part() -> None:
    """
    The construction path carries no body-name rule.

    The check is on the source of this row's own files, because the failure being
    guarded against is a rule that reads a name and infers a role -- and such a
    rule is found by a reader, not by a call.  The two names it looks for are the
    two a suspension model's ends are usually called.
    """
    from pathlib import Path

    source_root = Path(__file__).parents[2] / "src" / "suspension_multibody"
    for relative in ("subsystems/brake.py", "subsystems/drive.py"):
        text = (source_root / relative).read_text(encoding="utf-8").lower()
        for name in ("upright", "chassis"):
            assert name not in text, f"{relative} names {name!r}"
