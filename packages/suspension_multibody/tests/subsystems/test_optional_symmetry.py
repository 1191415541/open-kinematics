"""
Symmetry is a declaration: the default is the mirror, and one side is a topology.

Subtask 06's subject.  Three things are asserted here, and the third is the one
that keeps the change honest:

1. **one side is a topology.**  An assembly that declares one side carries one
   upright, one arm set, one spring and one tire -- and it composes and solves,
   which is what a single-wheel corner has to do;
2. **mirroring is the default and the explicit right side overrides it.**  A model
   that states `name__R` gets that coordinate on the right; every other role is
   still mirrored, so an asymmetric axle is expressible without giving up the
   shorthand where the parts really are symmetric;
3. **declaring nothing changes nothing.**  The default pair still composes exactly
   what it always did (the frozen snapshot is the byte-level half of that claim).
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from suspension_multibody.cases.kc_quasi_static import model_document
from suspension_multibody.schema import FrontAxleModel
from suspension_multibody.simulation import SimulationRequest, run_request
from suspension_multibody.subsystems.entry import compose_axle
from suspension_multibody.subsystems.geometry import side_hardpoints
from suspension_multibody.subsystems.types import SIDES, AssemblyRequest

FIXTURE = Path(__file__).parents[1] / "data" / "benchmark_axle.json"

#: A one-sided case: the left wheel is driven, and the case says so.
_ONE_SIDED_CASE = {
    "contract": "multibody-case",
    "contract_version": 1,
    "kind": "case",
    "family": "kc_quasi_static",
    "name": "single-corner",
    "time": {"start_s": 0.0, "end_s": 1e-3, "step_s": 1e-3},
    "k": {
        "wheel_values_mm": [-20.0],
        "rack_values_mm": [0.0],
        "axis_map": {"wheel": ["wheel_drive_L"], "rack": "rack_drive"},
        "left_right_mode": "single",
    },
}


#: The same one-sided case with the other corner driven.
_RIGHT_SIDED_CASE = {
    **_ONE_SIDED_CASE,
    "name": "single-corner-right",
    "k": {
        **_ONE_SIDED_CASE["k"],
        "axis_map": {"wheel": ["wheel_drive_R"], "rack": "rack_drive"},
    },
}


def _model() -> FrontAxleModel:
    return FrontAxleModel.model_validate(
        json.loads(FIXTURE.read_text(encoding="utf-8"))["model"]
    )


def test_the_default_still_pairs_both_sides() -> None:
    """
    Declaring nothing is the mirror, and that is D5's rule rather than a fallback.

    A request carries the sides it composes, and an unstated pair means the
    symmetric one -- so every existing caller keeps the assembly it had, which is
    what the frozen snapshot's zero difference measures.
    """
    request = AssemblyRequest(mode="K")
    assert request.sides == SIDES

    runtime = compose_axle(_model(), request=request)
    for stem in ("upper_arm", "lower_arm", "upright", "tie_rod", "wheel_hub"):
        assert f"{stem}_L" in runtime.bodies
        assert f"{stem}_R" in runtime.bodies


def test_one_declared_side_is_a_topology_not_a_hole() -> None:
    """
    A one-sided assembly carries one of everything, and nothing on the other side.

    The single-wheel corner is the smallest case of "the assembly declares its own
    sides": there is no right-hand upright at all, rather than a right-hand
    upright that nothing constrains.
    """
    runtime = compose_axle(_model(), request=AssemblyRequest(mode="K", sides=("L",)))

    assert "upright_L" in runtime.bodies
    assert "upright_R" not in runtime.bodies
    assert not any(name.endswith("_R") for name in runtime.bodies)
    assert not any(name.endswith("_R") for _body, name in runtime.points)
    # And the recorded document order follows the same declaration.
    assert all(not name.endswith("_R") for name in runtime.bodies)
    assert [
        getattr(row, "name", "") for row in runtime.constraints if "R" in str(getattr(row, "name", ""))[-1:]
    ] == []


def test_a_one_sided_assembly_runs_a_quasi_static_study() -> None:
    """
    G1 (b), the single-wheel half: it composes *and* solves.

    The document is authored from the runtime, so it drives exactly the sides the
    assembly has -- one wheel coordinate, no right-hand one -- and the solve
    returns a wheel load for it.  A one-sided assembly that only composed would be
    a body set rather than a corner.
    """
    assembly = compose_axle(_model(), request=AssemblyRequest(mode="K", sides=("L",)))
    document = model_document(assembly, name="single-corner", drive_wheels=True)

    drives = [joint["name"] for joint in document["joints"]]
    assert "wheel_drive_L" in drives
    assert "wheel_drive_R" not in drives

    run = run_request(
        SimulationRequest(
            assembly="axle",
            rig="kc_quasi_static",
            family="kc_quasi_static",
            model=document,
            case=_ONE_SIDED_CASE,
        )
    ).raw
    # It solved, and the solved state is the one-sided model: finite numbers, and
    # no right-hand body anywhere in the document it solved.
    assert np.isfinite(run.block("body_state")).all()
    reported = {body["name"] for body in document["bodies"]}
    assert "upright_L" in reported
    assert not any(name.endswith("_R") for name in reported)


def test_a_declared_right_side_overrides_the_mirror() -> None:
    """
    The mirror is the default; an explicit `name__R` states the right side's own
    coordinates and wins.

    Both halves matter.  A declared role that did *not* win would make asymmetry
    inexpressible, and a role the model says nothing about that stopped being
    mirrored would silently move the symmetric case.
    """
    raw = json.loads(FIXTURE.read_text(encoding="utf-8"))["model"]
    # The right outboard point of the lower arm, moved out by 40 mm and up by 5.
    raw["hardpoints"]["lca_outer__R"] = [-40.0, 740.0, 155.0]
    model = FrontAxleModel.model_validate(raw)

    sides = side_hardpoints(model.hardpoints, "R")
    assert sides["lca_outer"].as_tuple() == (-40.0, 740.0, 155.0)
    # An undeclared role is still the mirror of the left one.
    assert sides["uca_front"].as_tuple() == (-100.0, 500.0, 400.0)
    # And the left side is untouched by the right-hand declaration.
    assert side_hardpoints(model.hardpoints, "L")["lca_outer"].as_tuple() == (
        0.0,
        -700.0,
        150.0,
    )

    runtime = compose_axle(model, request=AssemblyRequest(mode="K"))
    assert runtime.points[("lower_arm_R", "outer")] == pytest.approx(
        np.array([-40.0, 740.0, 155.0])
    )
    assert runtime.points[("lower_arm_L", "outer")] == pytest.approx(
        np.array([0.0, -700.0, 150.0])
    )
    # The assembly's own per-side table carries the coordinate the model declared,
    # not the mirror of the left one -- which is what "the declaration wins" means
    # for the layer that actually composes.
    assert runtime.hardpoints["lca_outer__R"].as_tuple() == (-40.0, 740.0, 155.0)
    assert runtime.hardpoints["lca_outer__L"].as_tuple() == (0.0, -700.0, 150.0)


def test_a_three_wheeled_vehicle_is_two_suspensions_and_one_corner() -> None:
    """
    G1 (b), the three-wheel half: two axles, and one of them is a single corner.

    The entry that owns the corner says so (`AxleEntry.sides`), so the vehicle
    carries three wheel ends rather than four with one inert: the rear axle's
    left-hand bodies do not exist at all.  That is what makes a three-wheeled
    vehicle a topology instead of a four-wheeled one with a missing wheel.
    """
    from suspension_multibody.schema import Vec3, WheelSpec
    from suspension_multibody.subsystems.assembler import (
        AxleEntry,
        compose_entries_runtime,
    )
    from suspension_multibody.subsystems.types import DEFAULT_VEHICLE_SUBSYSTEMS

    def wheel(placement: str, side: str) -> WheelSpec:
        return WheelSpec(
            name=f"{placement}_{side}",
            body=f"{placement}_{side}_wheel",
            center_local=Vec3(x=0.0, y=760.0 if side == "left" else -760.0, z=320.0),
            mass=20.0,
            inertia=((2.0, 0.0, 0.0), (0.0, 2.0, 0.0), (0.0, 0.0, 2.0)),
        )

    entries = (
        AxleEntry(
            placement="front",
            prefix="front_",
            axle=_model(),
            replace_bodies={},
            wheels=(wheel("front", "left"), wheel("front", "right")),
        ),
        AxleEntry(
            placement="rear",
            prefix="rear_",
            axle=_model(),
            replace_bodies={},
            wheels=(wheel("rear", "right"),),
            # The corner: this axle carries one side, and the vehicle is a
            # three-wheeled one because of it.
            sides=("R",),
        ),
    )
    vehicle = compose_entries_runtime(
        entries,
        chassis_name="chassis",
        mode="K",
        request=AssemblyRequest(mode="K", subsystems=DEFAULT_VEHICLE_SUBSYSTEMS),
    )

    assert "front_upright_L" in vehicle.bodies and "front_upright_R" in vehicle.bodies
    assert "rear_upright_R" in vehicle.bodies
    assert "rear_upright_L" not in vehicle.bodies
    assert not any(name.startswith("rear_") and name.endswith("_L") for name in vehicle.bodies)
    assert sorted(vehicle.wheel_centers) == ["front_left", "front_right", "rear_right"]

    # The corner itself is a reading, not only a merge: a *right*-handed single
    # corner composes and solves, which is what says the side is a declaration
    # rather than an assumption that a one-sided assembly is the left one.
    corner = compose_axle(_model(), request=AssemblyRequest(mode="K", sides=("R",)))
    document = model_document(corner, name="right-corner", drive_wheels=True)
    drives = [joint["name"] for joint in document["joints"]]
    assert "wheel_drive_R" in drives
    assert "wheel_drive_L" not in drives
    run = run_request(
        SimulationRequest(
            assembly="axle",
            rig="kc_quasi_static",
            family="kc_quasi_static",
            model=document,
            case=_RIGHT_SIDED_CASE,
        )
    ).raw
    assert np.isfinite(run.block("body_state")).all()
