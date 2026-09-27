"""
The pad reading's three outputs: wheel centre, tire load, and the contact point.

The request states these explicitly -- "on convergence, output the current wheel centre
coordinates, the tire load and the real-time contact point, then advance to the next sweep
step" -- so they are a deliverable rather than a convenience.

Two things are asserted, and the second is the one that matters:

1. each is present for every pad height, once per side;
2. the contact point is **consistent with the geometry that produced it**, not merely
   reported.  The pad is what the wheel rests on, so the patch must lie on it:
   ``contact.z == pad height``.  That is an algebraic consequence of
   ``contact = center - [0, 0, radius - delta]``, and it fails if any of the four
   quantities is read from the wrong place -- which is exactly how a plausible-looking
   but wrong decode survives.
"""

from __future__ import annotations

import pytest

from suspension_multibody.cases.kc_quasi_static import model_document
from suspension_multibody.results.kc_state import pad_contact_from_run
from suspension_multibody.simulation import SimulationRequest, run_request
from suspension_multibody.subsystems.entry import compose_axle
from tests.benchmark_fixture import benchmark_model

PAD_HEIGHTS_MM = (0.0, 10.0, 20.0)


def _pad_run(tire_stiffness: float = 200.0, radius: float = 320.0):
    """Solve a pad sweep on an axle whose tires carry the wheel."""
    from suspension_multibody.schema import Vec3, VerticalTire

    model = benchmark_model().model_copy(
        update={
            "tires": (
                VerticalTire(
                    stiffness=tire_stiffness,
                    unloaded_radius=radius,
                    contact_point=Vec3(x=0.0, y=0.0, z=0.0),
                    local_axis=Vec3(x=0.0, y=0.0, z=1.0),
                ),
            )
        }
    )
    assembly = compose_axle(model, "K")
    document = model_document(
        assembly, name="pad-outputs", drive_wheels=True, drive_mode="pad"
    )
    case = {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "kc_quasi_static",
        "name": "pad-outputs",
        "time": {"start_s": 0.0, "end_s": 1e-3, "step_s": 1e-3},
        "k": {"pad_height_mm": list(PAD_HEIGHTS_MM)},
    }
    return run_request(
        SimulationRequest(
            assembly="axle",
            family="kc_quasi_static",
            model=document,
            case=case,
        )
    ).raw


def test_every_pad_height_reports_its_three_outputs() -> None:
    run = _pad_run()
    assert len(run.cases) == len(PAD_HEIGHTS_MM), "one case per pad height"
    for index, pad in enumerate(PAD_HEIGHTS_MM):
        outputs = pad_contact_from_run(run, index)
        assert set(outputs) == {"left", "right"}, f"pad {pad}: both sides"
        for side, values in outputs.items():
            for key in (
                "wheel_center_x_mm",
                "wheel_center_y_mm",
                "wheel_center_z_mm",
                "tire_load_n",
                "contact_x_mm",
                "contact_y_mm",
                "contact_z_mm",
            ):
                assert key in values, f"pad {pad} {side} is missing {key}"


def test_the_contact_point_lies_on_the_pad() -> None:
    """
    The identity that makes the decode checkable: the patch is on the ground.

    ``contact.z == pad height`` follows from ``contact = center - [0, 0, radius - delta]``
    exactly, so any misread of the centre, the penetration or the radius moves it.
    """
    run = _pad_run()
    for index, pad in enumerate(PAD_HEIGHTS_MM):
        for side, values in pad_contact_from_run(run, index).items():
            assert values["contact_z_mm"] == pytest.approx(pad, abs=1e-9), (
                f"pad {pad} {side}: the contact point is not on the pad"
            )


def test_the_tire_load_follows_the_compression() -> None:
    """
    The load is the contact law's own output, so it rises with the pad.

    Compared against the kernel's ``k * delta`` relation rather than a hard number, which
    makes this a statement about the law rather than about one fixture's geometry.
    """
    run = _pad_run()
    loads = [
        pad_contact_from_run(run, index)["left"]["tire_load_n"]
        for index in range(len(PAD_HEIGHTS_MM))
    ]
    assert all(later > earlier for earlier, later in zip(loads, loads[1:])), loads

    block = run.block("tire_output")
    for index in range(len(PAD_HEIGHTS_MM)):
        sample = (
            int(run.cases[index]["sample_offset"])
            + int(run.cases[index]["sample_count"])
            - 1
        )
        penetration = float(block[sample, 0, 2])
        assert loads[index] == pytest.approx(200.0 * 1000.0 * penetration, rel=1e-9)


def test_a_run_without_tires_reports_nothing_rather_than_zero() -> None:
    """
    Absent, not zero -- the distinction the compression channel is already built on.

    A wheel-centre-driven reading declares no tire, so it has no contact point; reporting
    ``0.0`` would claim a measurement that never happened.
    """
    assembly = compose_axle(benchmark_model(), "K")
    document = model_document(
        assembly, name="no-tires", drive_wheels=True, drive_mode="force_balance"
    )
    case = {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "kc_quasi_static",
        "name": "no-tires",
        "time": {"start_s": 0.0, "end_s": 1e-3, "step_s": 1e-3},
        "k": {
            "wheel_values_mm": [0.0],
            "rack_values_mm": [0.0],
            "axis_map": {
                "wheel": ["wheel_drive_L", "wheel_drive_R"],
                "rack": "rack_drive",
            },
        },
    }
    run = run_request(
        SimulationRequest(
            assembly="axle",
            family="kc_quasi_static",
            model=document,
            case=case,
        )
    ).raw
    assert pad_contact_from_run(run, 0) == {}
