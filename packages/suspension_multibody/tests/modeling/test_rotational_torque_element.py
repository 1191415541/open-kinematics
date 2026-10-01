"""
The rotational torque declaration: its law, its cap and its two ends.

The declaration is the Python reading of an element the kernel owns, so the tests
here are about the two things that would make the two disagree:

* the **sign law** -- the couple opposes the real-time relative rate, and a rate
  inside the kernel's own epsilon gets no couple at all.  A stationary pair must
  stay stationary; a full-strength couple of arbitrary sign is what the third
  branch exists to prevent.
* the **magnitude** -- ``min(stiffness * demand, max_torque)``, which with the
  unit demand the native law currently applies is ``min(stiffness, max_torque)``.

The element adds no net moment to the model, so the two wrenches are checked to be
equal and opposite; a single-sided couple would silently accelerate the whole
model instead of the pair.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from suspension_multibody.modeling.primitives import (
    SE3,
    ElementError,
    RigidBody,
    RigidBodyState,
    RotationalTorqueElement,
    RotationalTorqueParameters,
)

#: The two ends the fixtures below use.  Neutral on purpose: neither name appears
#: anywhere in the declaration, so a test that passed by recognising one of them
#: would not be a test of this element.
REACTION = "subframe"
DRIVEN = "spinner"


def _parameters(**overrides: object) -> RotationalTorqueParameters:
    fields: dict[str, object] = {"stiffness": 400.0, "max_torque": 250.0}
    fields.update(overrides)
    return RotationalTorqueParameters(**fields)  # type: ignore[arg-type]


def _state(*, turn: float = 0.0, spun: str = REACTION) -> RigidBodyState:
    """Two bodies, the one named ``spun`` rotated about +z by ``turn`` radians."""
    poses = {REACTION: SE3.identity(), DRIVEN: SE3.identity()}
    poses[spun] = SE3(
        np.zeros(3),
        np.array([math.cos(turn / 2.0), 0.0, 0.0, math.sin(turn / 2.0)]),
    )
    return RigidBodyState(
        {name: RigidBody(name, pose) for name, pose in poses.items()}
    )


def _element(**overrides: object) -> RotationalTorqueElement:
    return RotationalTorqueElement(
        name="torque",
        body_a=REACTION,
        body_b=DRIVEN,
        parameters=_parameters(**overrides),
    )


def test_the_couple_opposes_the_relative_rate() -> None:
    """Turning either way is resisted, and the two directions differ in sign."""
    parameters = _parameters()
    assert parameters.couple(2.0) == -250.0
    assert parameters.couple(-2.0) == 250.0
    assert parameters.couple(2.0) == -parameters.couple(-2.0)


def test_a_stationary_pair_gets_no_couple_at_all() -> None:
    """
    The third branch: a rate inside the kernel's epsilon is not a direction.

    Returning the full magnitude here would accelerate a parked pair by the sign
    of a rounding error, so the two ends of the tolerance are checked as well as
    zero itself.
    """
    parameters = _parameters()
    assert parameters.couple(0.0) == 0.0
    assert parameters.couple(1e-13) == 0.0
    assert parameters.couple(-1e-13) == 0.0


def test_the_magnitude_is_capped_by_max_torque() -> None:
    """The cap is a parameter, not an afterthought: it is a ``min``, either way."""
    assert _parameters(stiffness=100.0, max_torque=250.0).magnitude == 100.0
    assert _parameters(stiffness=400.0, max_torque=250.0).magnitude == 250.0
    assert _parameters(stiffness=100.0, max_torque=250.0).couple(1.0) == -100.0


def test_the_axis_is_stored_as_a_unit_vector() -> None:
    """A 2 m axis and a 0.5 m one describe the same rotation and must agree."""
    long_axis = _parameters(axis_a=[0.0, 2.0, 0.0])
    short_axis = _parameters(axis_a=[0.0, 0.5, 0.0])
    assert np.allclose(long_axis.axis_a, [0.0, 1.0, 0.0])
    assert np.allclose(long_axis.axis_a, short_axis.axis_a)
    assert long_axis.couple(1.0) == short_axis.couple(1.0)


def test_parameters_refuse_what_the_native_reader_refuses() -> None:
    """The refusals the reader applies, so neither side accepts alone."""
    with pytest.raises(ElementError, match="stiffness"):
        _parameters(stiffness=-1.0)
    with pytest.raises(ElementError, match="max_torque"):
        _parameters(max_torque=-1.0)
    with pytest.raises(ElementError, match="damping"):
        _parameters(damping=-1.0)
    with pytest.raises(ElementError, match="axis"):
        _parameters(axis_a=[0.0, 0.0, 0.0])
    with pytest.raises(ElementError, match="axis"):
        _parameters(axis_a=[0.0, 1.0])


def test_evaluating_a_pair_gives_equal_and_opposite_moments() -> None:
    """
    A couple, not two forces: the wrenches are pure moments and they cancel.

    The reaction body is ``body_a`` and the driven one ``body_b``, so the moment
    on ``body_b`` is ``+tau`` about the axis and the moment on ``body_a`` is
    ``-tau``.  Either wrench having a force part would mean the element pushes the
    model rather than torquing it, which is what the two zero assertions rule out.
    """
    result = _element().evaluate(_state(), relative_rate=1.0)
    on_reaction = result.body_wrenches_global[REACTION]
    on_driven = result.body_wrenches_global[DRIVEN]
    assert np.allclose(on_reaction[:3], 0.0)
    assert np.allclose(on_driven[:3], 0.0)
    assert np.allclose(on_reaction[3:], -on_driven[3:])
    # The axis is +y here, so the moment about y is the whole of it.
    assert on_driven[4] == pytest.approx(-250.0)
    assert on_driven[3] == pytest.approx(0.0)
    assert on_driven[5] == pytest.approx(0.0)


def test_an_external_couple_carries_no_stored_energy_and_no_tangent() -> None:
    """A demand is not an elastic member: nothing is stored and nothing is stiff."""
    result = _element().evaluate(_state(), relative_rate=1.0)
    assert result.energy == 0.0
    assert result.tangent is None
    assert result.active


def test_the_axis_follows_the_reaction_bodys_pose() -> None:
    """
    A quarter turn about +z takes a body-fixed +y axis to -x.

    The axis is stated in ``body_a``'s frame, so it has to be carried into the
    world by that body's pose and not by the driven one's: reading the wrong pose
    would keep the coupling aligned with ``body_a`` only while the two coincide.
    """
    element = _element()
    turned = _state(turn=math.pi / 2.0, spun=REACTION)
    assert np.allclose(element.axis_world(turned), [-1.0, 0.0, 0.0], atol=1e-12)
    # Turning the *driven* body instead leaves the axis where it was, which is what
    # makes the previous assertion a statement about the pose that was read.
    assert np.allclose(
        element.axis_world(_state(turn=math.pi / 2.0, spun=DRIVEN)),
        [0.0, 1.0, 0.0],
        atol=1e-12,
    )
    moment = element.evaluate(turned, relative_rate=1.0).body_wrenches_global[
        DRIVEN
    ][3:]
    assert np.allclose(moment, [250.0, 0.0, 0.0], atol=1e-12)


def test_a_rate_stated_by_default_is_still() -> None:
    """``relative_rate`` defaults to zero, which is the no-couple branch."""
    wrench = _element().evaluate(_state()).body_wrenches_global[DRIVEN]
    assert np.allclose(wrench, 0.0)


def test_the_element_has_no_rule_that_reads_a_body_name() -> None:
    """
    Which end is driven is the caller's statement, not this class's inference.

    The bodies are named for a rig's own parts, neither of which appears anywhere
    in the declaration: the same element with the two ends stated either way round
    produces the same law and the same pair of wrenches, so nothing here can be
    reading a name to decide a role.
    """
    forward = RotationalTorqueElement("torque", REACTION, DRIVEN, _parameters())
    backward = RotationalTorqueElement("torque", DRIVEN, REACTION, _parameters())
    assert forward.couple(1.0) == backward.couple(1.0)
    state = _state()
    on_forward = forward.evaluate(state, relative_rate=1.0).body_wrenches_global
    on_backward = backward.evaluate(state, relative_rate=1.0).body_wrenches_global
    assert set(on_forward) == set(on_backward) == {REACTION, DRIVEN}
    # Swapping the two arguments swaps which end is driven and nothing else: the
    # magnitudes agree, and neither body is treated as the special one.
    assert np.linalg.norm(on_forward[DRIVEN][3:]) == pytest.approx(
        np.linalg.norm(on_backward[REACTION][3:])
    )
    assert np.allclose(on_forward[DRIVEN][3:], -on_forward[REACTION][3:])
    assert np.allclose(on_backward[DRIVEN][3:], -on_backward[REACTION][3:])
