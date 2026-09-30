"""
A three-axle vehicle assembles, driven by its own entries, and its axles stay readable.

The point of this module is the *count*: an assembly whose driver is a list of
entries has no reason to stop at two axles, and the shape rule that used to fix
"exactly two suspensions, at front and rear" is the thing that made three
impossible.  What is asserted here is the chain that exists today -- three entries
assemble, each axle's bodies carry its own entry's prefix, every declared wheel end
is accounted for, and a repeated placement is refused.

One boundary is stated rather than crossed: the K/C *contract* reader still selects
a wheel centre by side alone, so a three-axle assembly presents it with three
candidates per side and it refuses instead of guessing.  That reader is 03c's part
of the job; the last test says so out loud so nobody mistakes it for coverage.
"""

from __future__ import annotations

import pytest

from suspension_multibody.connections.policy import RuleViolation, check_assembly_shape
from suspension_multibody.schema import FrontAxleModel, Vec3, WheelSpec
from suspension_multibody.studies import build_study_assembly, study_model_document
from suspension_multibody.subsystems.assembler import AxleEntry, compose_entries_runtime
from suspension_multibody.subsystems.types import (
    DEFAULT_VEHICLE_SUBSYSTEMS,
    AssemblyRequest,
)
from tests.benchmark_fixture import benchmark_model

#: A vehicle-shaped role set: the global 'vehicle' rule refuses one without brake or
#: drive, which is a statement about vehicles rather than about axle count.
VEHICLE_REQUEST = AssemblyRequest(mode="K", subsystems=DEFAULT_VEHICLE_SUBSYSTEMS)

#: The roles and placements every vehicle in these tests declares.
COMMON = [
    ("chassis", "any"),
    ("steering", "front"),
    ("brake", "any"),
    ("drive", "any"),
]


def _axle() -> FrontAxleModel:
    """Return the shared benchmark axle, which every entry reuses."""
    return benchmark_model()


def _wheel(placement: str, side: str) -> WheelSpec:
    """Return one wheel end of ``placement``, named after the placement itself."""
    return WheelSpec(
        name=f"{placement}_{side}",
        body=f"{placement}_{side}_wheel",
        center_local=Vec3(x=0.0, y=760.0 if side == "left" else -760.0, z=320.0),
        # A wheel end is a body of the dynamics like any other: a reading that
        # advances the vehicle needs a mass for it, and zero stops the reading.
        mass=20.0,
        inertia=((2.0, 0.0, 0.0), (0.0, 2.0, 0.0), (0.0, 0.0, 2.0)),
    )


def _entries(placements: tuple[str, ...]) -> tuple[AxleEntry, ...]:
    """Return one entry per placement, each with its own prefix and its own wheels."""
    return tuple(
        AxleEntry(
            placement=placement,
            prefix=f"{placement}_",
            axle=_axle(),
            replace_bodies={},
            wheels=(_wheel(placement, "left"), _wheel(placement, "right")),
        )
        for placement in placements
    )


def _wheels(*placements: str) -> list[tuple[str, str]]:
    """Return the wheel assignments of the named placements."""
    return [
        ("wheel", f"{placement}_{side}") for placement in placements for side in ("left", "right")
    ]


def _assemble(placements: tuple[str, ...]):
    return compose_entries_runtime(
        _entries(placements),
        chassis_name="chassis",
        mode="K",
        request=VEHICLE_REQUEST,
    )


def test_three_axles_assemble_and_each_keeps_its_own_prefix() -> None:
    runtime = _assemble(("front", "middle", "rear"))
    names = set(runtime.bodies)
    for prefix in ("front_", "middle_", "rear_"):
        assert any(name.startswith(prefix) for name in names), prefix
    assert runtime.wheel_body_names == {
        f"{placement}_{side}": f"{placement}_{side}_wheel"
        for placement in ("front", "middle", "rear")
        for side in ("left", "right")
    }


def test_the_three_axle_shape_is_accepted_and_the_two_axle_one_still_is() -> None:
    check_assembly_shape(
        "full_vehicle",
        [
            ("suspension", "front"),
            ("suspension", "middle"),
            ("suspension", "rear"),
            *COMMON,
            *_wheels("front", "middle", "rear"),
        ],
    )
    check_assembly_shape(
        "full_vehicle",
        [("suspension", "front"), ("suspension", "rear"), *COMMON, *_wheels("front", "rear")],
    )


def test_a_middle_axle_whose_wheels_nobody_claims_is_refused() -> None:
    with pytest.raises(RuleViolation, match="middle_left"):
        check_assembly_shape(
            "full_vehicle",
            [("suspension", "front"), ("suspension", "rear"), *COMMON, *_wheels("front", "middle", "rear")],
        )


def test_two_suspensions_at_one_placement_are_refused() -> None:
    with pytest.raises(RuleViolation, match="more than once"):
        check_assembly_shape(
            "full_vehicle",
            [("suspension", "front"), ("suspension", "front"), *COMMON, *_wheels("front")],
        )


def test_a_vehicle_with_no_suspension_at_all_is_refused() -> None:
    with pytest.raises(RuleViolation, match="at least one suspension"):
        check_assembly_shape("full_vehicle", [*COMMON, *_wheels("front")])


def test_an_axle_an_entry_carries_still_runs_a_quasi_static_study() -> None:
    """An entry's model is a usable axle, not something only the engine can read."""
    entries = _entries(("front", "middle", "rear"))
    study = build_study_assembly(entries[1].axle, study="quasi_static", mode="K")
    document = study_model_document(study, name="middle_axle")
    assert document["name"] == "middle_axle"
    assert document["bodies"]


def test_the_three_axle_assembly_itself_is_not_yet_readable_by_the_kc_contract() -> None:
    """
    The boundary, stated: a K/C document drives one wheel centre *per side*.

    ``cases/kc_quasi_static/contract.py`` now asks the assembly's own wheel table
    first (subtask 04b), so the answer no longer depends on how a label or a body
    is spelled -- but a three-axle assembly has three wheel ends per side, and a
    per-side coordinate cannot say which one it drives.  The reader therefore
    still refuses, and it now names the three bodies it had to choose between
    rather than reporting a name collision.  That is the correct behaviour for an
    ambiguous reading and the wrong *state* for a three-axle one: closing it needs
    the reading to name its wheel, which is a document-level statement rather than
    a better search.  Asserting the gap here keeps it visible instead of letting
    it read as coverage.

    The K/C contract is reached through ``cases.kc_quasi_static.model_document``,
    which is what a K/C reading of an assembly calls.
    """
    from suspension_multibody.cases.kc_quasi_static import model_document

    runtime = _assemble(("front", "middle", "rear"))
    with pytest.raises(Exception, match="more than one wheel end"):
        model_document(runtime, name="three_axle", drive_wheels=True)


def test_the_three_axle_vehicle_assembles_and_runs_one_study() -> None:
    """
    Three axles under one body are a vehicle that solves, not just a body set.

    The fixture axles declare no body masses of their own, so the assembled
    vehicle has nothing the dynamics can advance and the reading refuses it --
    that is a property of the fixture rather than of the assembly, and it is why
    this test states the axles' bodies the way a dynamic reading needs them.
    What it asserts is the chain 03 owns: the entry list assembles, the result is
    readable as a study, and the solve completes with all three axles' own bodies
    in it, each under its entry's prefix.
    """
    import numpy as np

    from suspension_multibody.axle_dynamics import (
        AxleDynamicsCase,
        AxleSolverSettings,
        run_axle_dynamics,
    )
    from suspension_multibody.schema import RigidBodySpec
    from suspension_multibody.studies import axle_dynamics_model, build_study_assembly
    from suspension_multibody.subsystems.assembler import runtime_for_study

    # The bodies the *template* supplies carry no mass of their own -- the axis
    # template invents the arms and the hub, so a caller that wants the dynamics
    # has to state their mass.  The names are read from the assembled axle rather
    # than written here, so a template that renames a part does not need this test
    # edited; the rebuild is what gives each one the same mass.
    probe = compose_entries_runtime(
        (
            AxleEntry(
                placement="front",
                prefix="front_",
                axle=_axle(),
                replace_bodies={},
                wheels=(),
            ),
        ),
        chassis_name="chassis",
        mode="K",
        request=VEHICLE_REQUEST,
    )
    described = _axle().model_copy(
        update={
            "bodies": tuple(
                RigidBodySpec(
                    name=name.removeprefix("front_"),
                    mass=10.0,
                    inertia=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                )
                for name in probe.bodies
                if name.startswith("front_")
            )
        }
    )
    entries = tuple(
        AxleEntry(
            placement=placement,
            prefix=f"{placement}_",
            axle=described,
            replace_bodies={},
            wheels=(_wheel(placement, "left"), _wheel(placement, "right")),
        )
        for placement in ("front", "middle", "rear")
    )
    runtime = compose_entries_runtime(
        entries,
        chassis_name="chassis",
        mode="K",
        request=VEHICLE_REQUEST,
    )
    model = axle_dynamics_model(
        build_study_assembly(runtime_for_study(runtime), study="dynamic", mode="K"),
        name="three_axle",
    )
    case = AxleDynamicsCase(
        name="three_axle",
        times_s=(0.0, 0.0005, 0.001),
        solver=AxleSolverSettings(
            internal_step_s=0.00025,
            # The fixture declares no tires, so nothing carries the vehicle and a
            # static equilibrium has no solution to find; the case supplies the
            # consistent state instead of asking the solver to invent one.
            initialization_mode="provided_consistent_state",
        ),
    )
    result = run_axle_dynamics(model, case)
    for prefix in ("front_", "middle_", "rear_"):
        assert any(body.name.startswith(prefix) for body in model.bodies), prefix
        assert result.body_state(f"{prefix}upright_L").shape[0] == 3
    # Every axle advanced by the same solve, and none of them moved away: the
    # sample count is the requested grid's.
    assert np.all(np.isfinite(result.body_state("middle_upright_L")))
