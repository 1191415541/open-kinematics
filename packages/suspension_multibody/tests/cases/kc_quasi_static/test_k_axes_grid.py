"""
The general driven grid: `axes` names any driven coordinates, each with its own
values.

The shorthand the axle families use -- one wheel travel applied to a set of wheel
coordinates with a sign pattern, plus the rack -- cannot say "left +10, right
-20", because it drives the whole wheel set together.  A roll, a single-wheel
bump or a two-axis sweep needs the general form, and the general form has to be
the *same* physics as the shorthand wherever the two describe the same grid.
Both properties are asserted here without a solver reference:

* a one-axis grid over the rack is bit-identical to the shorthand that also
  names the wheels and drives them by zero, because the two documents declare
  exactly the same targets;
* in a two-axis grid over the two wheels, one side's metrics depend only on that
  side's value -- which is what "driven independently" means.
"""

from __future__ import annotations

import numpy as np
from suspension_contracts import validate_case

from suspension_multibody.cases.kc_quasi_static import case_document, model_document
from suspension_multibody.model import build_front_axle
from suspension_multibody.simulation import SimulationRequest, run_request
from tests.benchmark_fixture import benchmark_model

_TIMES_S = (0.0, 1e-3)
_WHEEL_DRIVES = ("wheel_drive_L", "wheel_drive_R")


def _grid_case(axes: dict[str, tuple[float, ...]]) -> dict[str, object]:
    return {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "kc_quasi_static",
        "name": "axes-grid",
        "time": {"start_s": 0.0, "end_s": 1e-3, "step_s": 1e-3},
        "k": {
            "axes": [
                {"coordinate": name, "values_mm": list(values)}
                for name, values in axes.items()
            ],
            "drive": "wheel_center",
        },
    }


def _run(assembly, case: dict[str, object]):
    model = model_document(assembly, name="axes-grid", drive_wheels=True)
    return run_request(
        SimulationRequest(
            assembly="axle",
            family="kc_quasi_static",
            model=model,
            case=case,
        )
    ).raw


def test_a_single_axis_grid_is_the_shorthand_that_drives_the_wheels_by_zero() -> None:
    assembly = build_front_axle(benchmark_model(), "K")
    racks = (-5.0, 0.0, 5.0)
    axes_case = _grid_case({"rack_drive": racks})
    validate_case(axes_case)
    shorthand = case_document(
        assembly,
        family="kc_quasi_static",
        name="axes-grid",
        wheel_values_mm=(0.0,),
        rack_values_mm=racks,
        times_s=_TIMES_S,
        drive_wheels=True,
    )
    produced = _run(assembly, axes_case)
    reference = _run(assembly, shorthand)
    assert len(produced.cases) == len(racks)
    assert np.array_equal(produced.block("body_state"), reference.block("body_state")), (
        "the general grid and the shorthand disagreed on the same targets"
    )


def test_a_two_axis_grid_drives_the_two_sides_independently() -> None:
    assembly = build_front_axle(benchmark_model(), "K")
    left_values = (-10.0, 0.0, 10.0)
    right_values = (-20.0, 5.0)
    run = _run(
        assembly,
        _grid_case(
            {"wheel_drive_L": left_values, "wheel_drive_R": right_values}
        ),
    )
    entries = run.cases
    assert len(entries) == len(left_values) * len(right_values)
    states = run.block("body_state")
    bodies = list(run.document["manifest"]["bodies"])
    left_index = bodies.index("upright_L")
    right_index = bodies.index("upright_R")

    def final(index: int):
        entry = entries[index]
        last = int(entry["sample_offset"]) + int(entry["sample_count"]) - 1
        return states[last, left_index], states[last, right_index]

    # The last axis varies fastest, so the right value cycles inside each left
    # value: one left value must give the same left pose for every right value.
    for row, left in enumerate(left_values):
        baseline_left, _ = final(row * len(right_values))
        for column in range(1, len(right_values)):
            again, _ = final(row * len(right_values) + column)
            assert np.allclose(baseline_left, again, rtol=0.0, atol=1e-9), (
                f"the left pose moved with the right value {right_values[column]!r} "
                f"at left {left!r}"
            )
    # And the two sides really do carry different travel, so the assertion above
    # is not vacuous.
    left_index_zero = left_values.index(0.0)
    _, pulled = final(left_index_zero * len(right_values) + 1)
    _, pushed = final(left_index_zero * len(right_values))
    assert not np.allclose(pulled, pushed, rtol=0.0, atol=1e-6)
