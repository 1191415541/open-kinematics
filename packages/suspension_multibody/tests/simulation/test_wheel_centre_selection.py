"""
Which body a K/C reading drives the wheel centre on.

The document has to name a body for each `wheel_drive_{side}` coordinate, and the
answer is a *declaration* rather than a spelling.  Subtask 04b closed the last of
it: an assembly that carries its own wheel-end table (a vehicle runtime does)
answers the question itself, which is the only answer for a topology whose
wheel-centre label is not the built-in's -- a file template labels that point
`center`, and a search by label found nothing there.

Two sources, in order, and ambiguity refused rather than guessed at:

1. the assembly's own wheel-end table, when it has one;
2. otherwise the emitted points: the body carrying the `wheel_center` point, with
   the conventional names tried first so the built-in topologies answer exactly as
   they always have.
"""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from suspension_multibody.cases.kc_quasi_static.contract import (
    NativeKcError,
    wheel_centre_body,
)
from suspension_multibody.schema import FrontAxleModel
from suspension_multibody.subsystems.entry import compose_axle

FIXTURE = Path(__file__).parents[1] / "data" / "benchmark_axle.json"


def _benchmark() -> FrontAxleModel:
    return FrontAxleModel.model_validate(
        json.loads(FIXTURE.read_text(encoding="utf-8"))["model"]
    )


def _with_table(entries: dict[str, str]) -> SimpleNamespace:
    """Return a stub assembly whose wheel ends are the entries given."""
    return SimpleNamespace(
        bodies={body: object() for body in entries.values()},
        points={},
        wheel_centers={
            name: (body, np.zeros(3)) for name, body in entries.items()
        },
    )


def test_the_assembly_s_own_table_answers_when_it_has_one() -> None:
    """
    A table decides the side, and it does so without any label to match.

    This is the case a search cannot reach: the bodies are named the way a
    document names them, so a lookup by the built-in's conventional stems finds
    nothing and a lookup by label finds nothing either -- while the assembly is
    saying, in its own table, exactly which body carries the wheel centre.
    """
    assembly = _with_table(
        {
            "front_left": "tracker_left_carrier",
            "front_right": "tracker_right_carrier",
        }
    )
    assert wheel_centre_body(assembly, "L") == "tracker_left_carrier"
    assert wheel_centre_body(assembly, "R") == "tracker_right_carrier"


def test_two_wheel_ends_on_one_side_are_refused() -> None:
    """
    A multi-axle assembly is not a guess: the reading has to name the wheel.

    Both bodies are named in the message, because "there are two" is not enough to
    fix anything: what a caller has to change is which wheel it drives.
    """
    assembly = _with_table(
        {
            "front_left": "front_carrier",
            "rear_left": "rear_carrier",
            "front_right": "front_carrier_r",
        }
    )
    with pytest.raises(NativeKcError) as error:
        wheel_centre_body(assembly, "L")
    assert "front_carrier" in str(error.value)
    assert "rear_carrier" in str(error.value)


def test_an_assembly_without_a_table_still_answers_from_its_own_points() -> None:
    """
    The single-axle case is unchanged, which is what keeps the frozen runs valid.

    A composed axle's wheel ends are *places* on its own bodies rather than entries
    in a table, so the label search is still the one that decides -- and it returns
    the same body it always did.
    """
    runtime = compose_axle(_benchmark(), "K")
    assert not hasattr(runtime, "wheel_centers")
    assert wheel_centre_body(runtime, "L") == "wheel_hub_L"
    assert wheel_centre_body(runtime, "R") == "wheel_hub_R"
