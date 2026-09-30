"""
The wheel end has one producer, and one *reading* decides whether it stands alone.

Subtask 04's subject, in the form the code can be asked about it:

1. **one producer.**  The wheel body and the tire come from the wheel subsystem
   and from the wheel template it reads -- not from one path per topology.  The
   built-in template declares no wheel body, which is the state every recorded
   baseline was taken in (D9); a template that *does* declare one is honoured.
2. **the reading decides.**  A reading that brings its own wheels -- every bench
   with ``RigSpec.supplies_wheels`` -- gets no second, independent wheel body: the
   one the wheel subsystem declared is condensed into the body carrying the wheel
   centre, through the composition's own ``_merge_fixed_wheel``.
3. **the assembly stage does not decide it by type.**  The
   ``VerticalTireElement`` filter that used to stand in ``assembler`` is gone, and
   what replaced it is a *role*: a vehicle composes its axles without the wheel
   role, because a vehicle's wheel ends are the ones its entries name.

The condensation is measured, not asserted: the entity set, the constraint rows
and the tire rows of a template-declared wheel are compared against the built-in
reading of the same model, which is the equivalence D2 requires.
"""

from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

import pytest

from suspension_multibody.rigs.rig import RIGS
from suspension_multibody.schema import FrontAxleModel
from suspension_multibody.subsystems import wheel as wheel_subsystem
from suspension_multibody.subsystems.assembler import _AXLE_ROLES_IN_A_VEHICLE
from suspension_multibody.subsystems.entry import compose_axle
from suspension_multibody.subsystems.si_assembly import _wheel_end_is_supplied
from suspension_multibody.subsystems.types import (
    DEFAULT_AXLE_SUBSYSTEMS,
    AssemblyRequest,
)
from suspension_multibody.templates.builtin import WHEEL
from suspension_multibody.templates.model import PartDefinition

FIXTURE = Path(__file__).parents[1] / "data" / "benchmark_axle.json"

#: The two bodies a wheel template of our own owns, and what they weigh.
_WHEEL_MASS = 12.0


def _model() -> FrontAxleModel:
    """Return the benchmark axle with one declared vertical tire."""
    raw = dict(json.loads(FIXTURE.read_text(encoding="utf-8"))["model"])
    raw["tires"] = [
        {
            "stiffness": 200.0,
            "unloaded_radius": 320.0,
            "contact_point": {"x": 0.0, "y": 0.0, "z": 0.0},
            "local_axis": {"x": 0.0, "y": 0.0, "z": 1.0},
        }
    ]
    return FrontAxleModel.model_validate(raw)


def _declaring_wheel_template():
    """
    Return a wheel template that owns the wheel bodies it describes.

    This is the "a file declares wheels" case in its smallest form: the template
    names ``wheel_L``/``wheel_R`` as its own parts and hangs its wheel centre on
    them, which is what the built-in cannot do (its wheel centre hangs on the
    suspension's hub).
    """
    return replace(
        WHEEL,
        parts=(
            PartDefinition("wheel_L", mass=_WHEEL_MASS),
            PartDefinition("wheel_R", mass=_WHEEL_MASS),
        ),
        connections=tuple(
            replace(connection, owner=f"wheel_{connection.owner[-1]}")
            if connection.role == "wheel_center"
            else connection
            for connection in WHEEL.connections
        ),
    )


def _tire_owners(runtime) -> dict[str, str]:
    return {
        element.name: element.wheel_body
        for element in runtime.elements
        if getattr(element, "wheel_body", None) is not None
    }


def _body_names(runtime) -> set[str]:
    return set(runtime.bodies)


def test_the_built_in_wheel_subsystem_declares_no_body_and_still_places_the_tire() -> None:
    """
    The recorded state: no wheel body of its own, and the tire on the wheel hub.

    Both halves are the template's declaration, so this is the *built-in*
    template's answer rather than a fact about the axle: nothing here reads a
    name to decide either one.
    """
    model = _model()
    runtime = compose_axle(model, "K")

    assert "wheel_L" not in _body_names(runtime)
    assert _tire_owners(runtime) == {"tire_L": "wheel_hub_L", "tire_R": "wheel_hub_R"}
    # The wheel centre is where the tire acts, and the hub is what carries it.
    assert (runtime.points[("wheel_hub_L", "wheel_center")] == [0.0, -700.0, 300.0]).all()


def test_a_declared_wheel_body_is_condensed_into_the_body_carrying_the_wheel_centre(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    D2: the wheel the wheel subsystem declares does not survive as its own body.

    The measurement is the whole point: the built-in reading and the declaring
    reading of the *same* model have the same entity set, the same constraint
    rows and the same tire rows, and the only difference is where the wheel's own
    mass ended up -- folded into the hub by the composition's composite-mass
    routine.  A condensation that changed the entity set or the row count would
    move the frozen K/C baseline, which is the gate this exists to keep.
    """
    model = _model()
    plain = compose_axle(model, "K")

    monkeypatch.setattr(wheel_subsystem, "WHEEL", _declaring_wheel_template())
    condensed = compose_axle(model, "K")

    assert _body_names(condensed) == _body_names(plain)
    assert "wheel_L" not in _body_names(condensed)
    assert len(condensed.constraints) == len(plain.constraints)
    assert len(condensed.ideal_constraints) == len(plain.ideal_constraints)
    assert _tire_owners(condensed) == _tire_owners(plain)
    assert (
        condensed.points[("wheel_hub_L", "wheel_center")]
        == plain.points[("wheel_hub_L", "wheel_center")]
    ).all()

    # The wheel's mass is inside the hub now: nothing was dropped on the way.
    hub_plain = plain.bodies["wheel_hub_L"]
    hub_condensed = condensed.bodies["wheel_hub_L"]
    assert hub_condensed.mass == pytest.approx(hub_plain.mass + _WHEEL_MASS, rel=1e-12)
    assert hub_condensed.mass > hub_plain.mass
    # Its inertia grew with it -- a mass folded in with no inertia would be a
    # point mass, which is the one shape a wheel does not have.
    assert (condensed.bodies["wheel_hub_L"].inertia >= hub_plain.inertia).all()


def test_the_wheel_subsystem_is_the_producer_of_the_wheel_end(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    The wheel end comes from the wheel role's contribution, for either template.

    Asserting on the *contribution* rather than on the finished runtime is what
    makes "one producer" checkable: the bodies the wheel role emitted are the
    wheel end, whatever they are called, and the composition is what decides
    whether they stand alone.
    """
    from suspension_multibody.subsystems.si_assembly import (
        _wheel_bodies,
        axle_contributions_and_order,
    )

    model = _model()
    request = AssemblyRequest(mode="K")

    contributions, _ = axle_contributions_and_order(model, request=request)
    assert _wheel_bodies(contributions) == ()
    assert wheel_subsystem.build.__module__ == wheel_subsystem.__name__

    monkeypatch.setattr(wheel_subsystem, "WHEEL", _declaring_wheel_template())
    contributions, _ = axle_contributions_and_order(model, request=request)
    assert sorted(_wheel_bodies(contributions)) == ["wheel_L", "wheel_R"]


def test_the_condensation_criterion_is_the_bench_and_not_what_was_emitted() -> None:
    """
    The reading decides, and the reading is `RigSpec.supplies_wheels`.

    The criterion used to be read the other way round -- "the wheel subsystem
    produced no wheel body this time" -- which stopped distinguishing anything
    once both topologies read one wheel declaration.  Every bench that supplies
    wheels is a single-axle reading and condenses; every bench that does not is a
    vehicle reading and does not.
    """
    assert _wheel_end_is_supplied(None) is True  # a bare axle: D9
    for name, spec in RIGS.items():
        assert _wheel_end_is_supplied(name) is spec.supplies_wheels, name
    assert _wheel_end_is_supplied("kc_quasi_static") is True
    assert _wheel_end_is_supplied("axle_dynamic") is True
    assert _wheel_end_is_supplied("vehicle_kc") is False


def test_a_vehicle_composes_its_axles_without_the_wheel_role() -> None:
    """
    The role, not a type test, is what keeps the vehicle's wheels its own.

    An axle inside a vehicle is composed without the wheel role, so the axle
    never describes a wheel end the vehicle would have to un-describe -- which is
    exactly what the removed ``VerticalTireElement`` filter was doing.
    """
    assert _AXLE_ROLES_IN_A_VEHICLE == DEFAULT_AXLE_SUBSYSTEMS - {"wheel"}
    assert "wheel" in DEFAULT_AXLE_SUBSYSTEMS

    # And the filter itself is gone, checked where it lived.  The name may still
    # appear in prose about its removal (it does, in `assembler`'s own module
    # docstring); what must not survive is the *type test* and the import that fed
    # it, because those are what decided the wheel's lifecycle at this stage.
    source = (
        Path(__file__).parents[2]
        / "src"
        / "suspension_multibody"
        / "subsystems"
        / "assembler.py"
    ).read_text(encoding="utf-8")
    assert "isinstance(element, VerticalTireElement)" not in source
    assert "    VerticalTireElement,\n" not in source


def test_tire_rows_exist_only_when_the_assembly_carries_the_wheel_role() -> None:
    """
    The wheel end exists because the assembly carries the role that describes it.

    Two requests for the *same* model, differing only in whether the wheel role
    is carried: one declares the tires the model states, the other declares none,
    and neither reaches for a name or a type to decide it.
    """
    model = _model()
    with_wheel = compose_axle(model, "K")
    without = compose_axle(
        model,
        request=AssemblyRequest(
            mode="K", subsystems=DEFAULT_AXLE_SUBSYSTEMS - {"wheel"}
        ),
    )

    assert len(with_wheel.elements) == 2
    assert without.elements == ()
    assert _body_names(with_wheel) == _body_names(without)


def test_a_condensed_wheel_end_takes_its_places_and_references_with_it(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    Nothing the condensed body carried is lost, and nothing still names it.

    A body is not only its mass: it declares places (`points`) and the assembly's
    accounting refers to it by name (`connections`).  Condensing it away without
    moving those would leave a reading that cannot name a place the model
    described, and an accounting row pointing at a body that is not there -- both
    of which would show up as a missing entry rather than as an error.
    """
    model = _model()
    monkeypatch.setattr(wheel_subsystem, "WHEEL", _declaring_wheel_template())
    condensed = compose_axle(model, "K")

    # Every place the wheel end declared is now a place on the body it was
    # condensed into, under the same label.
    assert (condensed.bodies["wheel_hub_L"].mass > _WHEEL_MASS)
    for label in ("wheel_center",):
        assert ("wheel_hub_L", label) in condensed.points
    # And the accounting refers to bodies this assembly carries.
    for connection in condensed.connections:
        assert connection.body_a in condensed.bodies
        assert connection.body_b in condensed.bodies
    assert not any(name.startswith("wheel_L") or name.startswith("wheel_R") for name in condensed.bodies)
