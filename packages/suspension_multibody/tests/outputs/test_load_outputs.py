"""
The wheel-load derived outputs are declared from the wheel ends, not written out.

`builtin.py` cannot import `report/wheel_loads.py` -- a report derives *from* a
run's outputs, it is not one of them -- so the channel table exists twice: once as
the report's arithmetic, once as this declaration.  The two are compared value for
value in `tests/metrics/test_outputs_match_legacy.py`; what is checked *here* is
that the declaration half is generated, so another axle's ends add their own
channels instead of needing an entry written out by hand.

The rename experiment at this layer is direct: declare wheel ends under a placement
name no vehicle here uses, and the declared channel names follow.
"""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from suspension_multibody.outputs import BUILTIN, builtin

#: The two placements the declared four corners state, and the names the
#: declaration publishes for them -- the historical eleven plus the two
#: placement-driven axle channels.
_DECLARED_LOAD_OUTPUTS = (
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
)

_LOADS = {
    "front_left": 100.0,
    "front_right": 120.0,
    "rear_left": 80.0,
    "rear_right": 90.0,
}


def _load_outputs() -> list[object]:
    """Return the derived outputs that restate the wheel-load metric."""
    return [
        output
        for output in builtin.DERIVED_OUTPUTS
        if output.legacy == "report.metrics.vehicle.wheel_load_metrics"
    ]


def test_the_declared_load_outputs_are_the_historical_ones_plus_a_placement_channel() -> None:
    """The declared set, in order: the eleven of the legacy table, then the addition."""
    assert [output.name for output in _load_outputs()] == list(_DECLARED_LOAD_OUTPUTS)


def test_every_declared_load_output_reads_every_declared_wheel_end() -> None:
    """
    A load channel is arithmetic over all the declared wheel ends.

    The declaration cannot say "these four" without naming them, and a channel
    that read a subset would be a different metric; the finite check is over the
    whole set, so the declaration reads them all.
    """
    for output in _load_outputs():
        assert output.reads == (
            "wheel_load_front_left",
            "wheel_load_front_right",
            "wheel_load_rear_left",
            "wheel_load_rear_right",
        ), output.name


def test_the_declared_channels_follow_the_declared_wheel_ends(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    The declaration is generated from `_WHEELS`, not written out.

    Renaming the declared ends to a placement no vehicle in this repository states
    must rename the channels with them -- including the transfer channel, which is
    named after the first and last placement rather than after `front`/`rear`.
    """
    monkeypatch.setattr(
        builtin,
        "_WHEELS",
        ("bogie_left", "bogie_right", "trailer_left", "trailer_right"),
    )

    names = builtin._load_channel_names()

    assert names == (
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
    )


def test_the_declared_load_outputs_evaluate_to_the_report_channel_values() -> None:
    """Each declared channel, evaluated from the declared ends, is its own value."""
    result = SimpleNamespace(times_s=np.array([0.0, 1.0]))
    values = builtin.minimum_unit_outputs(result, wheel_loads=_LOADS)

    assert BUILTIN.evaluate("normal_load_total", values) == 390.0
    assert BUILTIN.evaluate("normal_load_axle_front", values) == 220.0
    assert BUILTIN.evaluate("normal_load_front_axle", values) == 220.0
    assert BUILTIN.evaluate("normal_load_axle_rear", values) == 170.0
    assert BUILTIN.evaluate("load_transfer_front_minus_rear", values) == 50.0
    assert BUILTIN.evaluate("load_transfer_right_minus_left", values) == 30.0


def test_a_placement_the_declaration_does_not_declare_is_not_declared() -> None:
    """
    The declaration is bounded by `_WHEELS`, and it says so by not declaring more.

    `RIG_OUTPUTS` -- a different segment of `builtin.py` -- declares the four
    `wheel_load_<corner>` outputs, so a middle axle has no declared input to read
    and no declared channel either.  A three-axle run therefore needs those
    declarations extended (p5-03's territory), and this test is what keeps that
    fact visible instead of silent: the *generation* follows the ends, but the
    declared end set is still the four corners.
    """
    names = {output.name for output in builtin.DERIVED_OUTPUTS}
    assert "normal_load_axle_middle" not in names
    declared = set(builtin.declared_names(builtin.RIG_OUTPUTS))
    assert {f"wheel_load_{wheel}" for wheel in builtin._WHEELS} <= declared
