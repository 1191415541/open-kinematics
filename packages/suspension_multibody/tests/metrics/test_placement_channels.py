"""
The wheel-load channel table follows the placements the run states.

Before p3-05 the table was a constant written twice -- once in
``report/wheel_loads.py``, once in ``report/metrics/vehicle.py`` -- and both copies
spelled ``front`` and ``rear`` out.  A three-axle vehicle has no ``rear`` in that
sense and a bench has no ``front`` at all, so the names are now generated from the
placements the wheel ends' own names state.

Two properties are checked here, and they are different claims:

* **a new placement is served**: three axles produce
  ``normal_load_axle_front``/``_middle``/``_rear``, and the summary answers
  ``middle_axle`` beside the historical ``front_axle``/``rear_axle``;
* **nothing is hardcoded**: a layout whose placements are ``bogie``/``trailer``
  produces channels named after *those*, and no ``front``/``middle``/``rear``
  spelling appears.  Without this second test the first one is satisfied by
  replacing one hardcoded pair with a hardcoded triple.

The three-axle loads come from the assembly ``tests/physics/test_static_loads.py``
builds, so the table is exercised on the frozen static-equilibrium contract rather
than on a hand-written mapping.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.report.metrics import wheel_load_metrics
from suspension_multibody.report.wheel_loads import (
    axle_loads,
    side_loads,
    summarize_wheel_loads,
)
from tests.physics.test_static_loads import G, _solve, _three_axle_assembly

#: The three placements a three-axle vehicle states, in the order its assembly
#: declares them -- not an order this table chose.
_PLACEMENTS = ("front", "middle", "rear")


def _three_axle_loads() -> dict[str, float]:
    """Return the six solved wheel loads of the three-axle assembly."""
    return _solve(_three_axle_assembly()).wheel_loads


def test_a_three_axle_vehicle_gets_one_channel_per_placement() -> None:
    """Six wheel ends, three placements, three axle channels."""
    loads = _three_axle_loads()

    channels = wheel_load_metrics(loads)

    for placement in _PLACEMENTS:
        assert f"normal_load_axle_{placement}" in channels, placement
        # The historical spelling of the same total is still published, so a
        # consumer written against a two-axle vehicle keeps its name.
        assert f"normal_load_{placement}_axle" in channels, placement
        assert (
            channels[f"normal_load_axle_{placement}"]
            == channels[f"normal_load_{placement}_axle"]
            == pytest.approx(
                loads[f"{placement}_left"] + loads[f"{placement}_right"], rel=1e-15
            )
        )
    assert channels["normal_load_total"] == pytest.approx(sum(loads.values()), rel=1e-15)
    # The historical transfer channel names the first and the last placement, so
    # on this vehicle it is still `front_minus_rear`.
    assert "load_transfer_front_minus_rear" in channels
    assert channels["load_transfer_front_minus_rear"] == pytest.approx(
        channels["normal_load_axle_front"] - channels["normal_load_axle_rear"]
    )


def test_a_three_axle_summary_answers_the_middle_axle() -> None:
    """
    The summary generalises with the table: a placement is answered by its name.

    ``middle_axle`` is not a field anyone wrote -- it resolves against the
    placements the loads carry -- and a placement the loads do not carry raises
    ``AttributeError`` rather than inventing a value.
    """
    summary = summarize_wheel_loads(_three_axle_loads())

    assert summary.placements == _PLACEMENTS
    for placement in _PLACEMENTS:
        assert summary.axle_loads[placement] == pytest.approx(summary.total/3, rel=1e-15)
    assert summary.total == pytest.approx(sum(summary.axle_loads.values()), rel=1e-15)
    assert summary.front_rear_delta == summary.front_axle - summary.rear_axle
    assert summary.front_rear_delta == pytest.approx(0, abs=2e-12)
    with pytest.raises(AttributeError, match="no 'fourth' placement"):
        summary.fourth_axle


def test_a_layout_whose_placements_are_not_front_middle_rear_is_named_after_them() -> None:
    """
    The rename experiment: nothing here knows the words front, middle or rear.

    If the table were a hardcoded triple with the names swapped, ``bogie`` and
    ``trailer`` would produce ``front``/``middle``/``rear`` channels or none at
    all.  They produce their own, and the transfer channel is named after the two
    placements that actually exist.
    """
    loads = {
        "bogie_left": 250.0,
        "bogie_right": 150.0,
        "trailer_left": 100.0,
        "trailer_right": 100.0,
    }

    channels = wheel_load_metrics(loads)

    assert set(channels) == {
        "normal_load_bogie_left",
        "normal_load_bogie_right",
        "normal_load_trailer_left",
        "normal_load_trailer_right",
        "normal_load_total",
        "normal_load_axle_bogie",
        "normal_load_bogie_axle",
        "normal_load_axle_trailer",
        "normal_load_trailer_axle",
        "normal_load_left_side",
        "normal_load_right_side",
        "load_transfer_bogie_minus_trailer",
        "load_transfer_right_minus_left",
    }
    assert channels["normal_load_axle_bogie"] == 400.0
    assert channels["normal_load_axle_trailer"] == 200.0
    assert channels["load_transfer_bogie_minus_trailer"] == 200.0
    assert channels["normal_load_total"] == 600.0
    assert not [name for name in channels if "front" in name or "middle" in name]


def test_the_aggregates_follow_the_loads_rather_than_a_corner_count() -> None:
    """
    One wheel end, two, or six: the aggregates are the ones the loads support.

    A single-ended bench has a left side and no right one, and the table says so
    by publishing no right-side channel; the summary refuses that side by name
    instead of returning a zero nobody measured.
    """
    single = {"single_left": 12508758.86425533}

    channels = wheel_load_metrics(single)

    assert channels["normal_load_axle_single"] == 12508758.86425533
    assert channels["normal_load_left_side"] == 12508758.86425533
    assert channels["normal_load_total"] == 12508758.86425533
    assert "normal_load_right_side" not in channels
    assert "load_transfer_right_minus_left" not in channels
    assert axle_loads(single) == {"single": 12508758.86425533}
    assert side_loads(single) == {"left": 12508758.86425533}

    summary = summarize_wheel_loads(single)
    assert summary.single_axle == 12508758.86425533
    assert summary.left_side == 12508758.86425533
    assert summary.front_rear_delta == 0.0
    with pytest.raises(AttributeError, match="no 'right' side"):
        _ = summary.right_side
    with pytest.raises(AttributeError, match="no 'right' side"):
        _ = summary.right_left_delta


def test_a_name_that_states_no_side_is_refused_by_name() -> None:
    """A load no placement or side total can carry is refused where it is read."""
    with pytest.raises(ValueError, match="does not state a side"):
        wheel_load_metrics({"front_left": 100.0, "axle_total": 100.0})
    with pytest.raises(ValueError, match="must be finite"):
        wheel_load_metrics({"front_left": 100.0, "front_right": float("nan")})
    with pytest.raises(ValueError, match="at least one wheel end"):
        wheel_load_metrics({})


def test_the_four_wheel_table_keeps_the_arithmetic_it_had() -> None:
    """
    The four-corner sums are the same sums, not merely equal-looking ones.

    Left-then-right per placement, placement order per side, placement order for
    the total: the accumulation order is what makes the values *bit*-identical to
    the historical ones, and it is easy to lose by reordering a comprehension.
    """
    loads = {
        "front_left": 100.0,
        "front_right": 120.0,
        "rear_left": 80.0,
        "rear_right": 90.0,
    }
    channels = wheel_load_metrics(loads)

    assert channels["normal_load_total"] == (100.0 + 120.0) + (80.0 + 90.0)
    assert channels["normal_load_axle_front"] == 100.0 + 120.0
    assert channels["normal_load_left_side"] == 100.0 + 80.0
    assert channels["load_transfer_right_minus_left"] == (120.0 + 90.0) - (100.0 + 80.0)
    assert list(channels) == [
        "normal_load_front_left",
        "normal_load_front_right",
        "normal_load_rear_left",
        "normal_load_rear_right",
        "normal_load_total",
        "normal_load_axle_front",
        "normal_load_front_axle",
        "normal_load_axle_rear",
        "normal_load_rear_axle",
        "normal_load_left_side",
        "normal_load_right_side",
        "load_transfer_front_minus_rear",
        "load_transfer_right_minus_left",
    ]


def test_the_third_axle_of_a_six_wheel_run_is_a_real_placement_not_a_rounding() -> None:
    """
    The three-axle channels carry measured values, not a repeated constant.

    A longitudinal acceleration moves load from the first axle to the last, which
    is what shows the middle channel is the middle axle's own total rather than a
    copy of a neighbour.
    """
    assembly = _three_axle_assembly()
    static = _solve(assembly)
    accelerated = _solve(
        assembly, acceleration=np.array([1.0, 0.0, 0.0])
    )

    static_channels = wheel_load_metrics(static.wheel_loads)
    accelerated_channels = wheel_load_metrics(accelerated.wheel_loads)

    assert (
        accelerated_channels["normal_load_axle_front"]
        < static_channels["normal_load_axle_front"]
    )
    assert (
        accelerated_channels["normal_load_axle_rear"]
        > static_channels["normal_load_axle_rear"]
    )
    assert accelerated_channels["load_transfer_front_minus_rear"] < 0.0
    assert sum(accelerated.wheel_loads.values()) == pytest.approx(
        static.total_mass * G, rel=0.0, abs=accelerated.residual_tolerance
    )
