"""
A towed vehicle assembles: two body-level units joined by a joint of the pair.

The point of this module is a *second body side*.  Everything an axle entry
describes hangs from one body, and a tractor with a trailer is not more axles --
it is two units whose only relation is the joint between them.  That joint is
neither an axle's nor the vehicle's, so it is stated explicitly, by the names the
finished assembly uses, and built once every entry has contributed its bodies.

What is asserted is the chain that exists: the entry list carries both bodies',
the articulation becomes a real joint row, an articulation naming a body nobody
produced is refused, and the assembly is readable as a study and *runs* -- a
joint someone merely constructed would prove nothing about the solver.

The joint kinds are the ones the assembly already builds (``weld`` and
``revolute``).  A new kind of joint would be a kernel question, and this module
says so rather than inventing one.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.axle_dynamics import (
    AxleDynamicsCase,
    AxleSolverSettings,
    run_axle_dynamics,
)
from suspension_multibody.modeling.primitives import RigidBody
from suspension_multibody.studies import axle_dynamics_model, build_study_assembly
from suspension_multibody.subsystems.assembler import (
    ArticulationSpec,
    compose_entries_runtime,
    runtime_for_study,
)
from suspension_multibody.subsystems.types import (
    DEFAULT_VEHICLE_SUBSYSTEMS,
    AssemblyRequest,
)

REQUEST = AssemblyRequest(mode="K", subsystems=DEFAULT_VEHICLE_SUBSYSTEMS)


def _ground() -> RigidBody:
    return RigidBody("ground", mass=0.0, inertia=np.eye(3), fixed=True)


def _tractor() -> RigidBody:
    return RigidBody("tractor", mass=1200.0, inertia=np.eye(3) * 1000.0)


def _trailer() -> RigidBody:
    return RigidBody("trailer", mass=800.0, inertia=np.eye(3) * 800.0)


def _tow(
    *articulations: ArticulationSpec,
    units: dict[str, RigidBody] | None = None,
):
    """Assemble the towed unit: one vehicle body, one trailer body, and joints."""
    return compose_entries_runtime(
        (),
        chassis_name="tractor",
        chassis_body=_tractor(),
        mode="K",
        request=REQUEST,
        unit_bodies={"ground": _ground(), "trailer": _trailer(), **(units or {})},
        articulations=articulations,
    )


def _run(assembly, *, name: str = "tow"):
    """Read the assembly as a dynamic study and run it once."""
    model = axle_dynamics_model(
        build_study_assembly(runtime_for_study(assembly), study="dynamic", mode="K"),
        name=name,
    )
    case = AxleDynamicsCase(
        name=name,
        times_s=(0.0, 0.0005, 0.001),
        solver=AxleSolverSettings(internal_step_s=0.00025),
    )
    return run_axle_dynamics(model, case)


def test_two_body_sides_assemble_under_one_articulation() -> None:
    """A tractor and a trailer are two bodies of one assembly, joined once."""
    assembly = _tow(
        ArticulationSpec(kind="weld", body_a="ground", body_b="tractor", name="anchor"),
        ArticulationSpec(
            kind="revolute",
            body_a="tractor",
            body_b="trailer",
            point_a_local=(3.0, 0.0, 0.5),
            point_b_local=(0.0, 0.0, 0.5),
            name="hitch",
        ),
    )
    assert {"ground", "tractor", "trailer"} <= set(assembly.bodies)
    # The articulation is a real row in both columns, not a record of an intent:
    # the K reading drives the ideal-joint column and the C reading keeps it.
    assert [row.name for row in assembly.constraints] == ["anchor", "hitch"]
    assert [row.name for row in assembly.ideal_constraints] == ["anchor", "hitch"]
    hitch = next(row for row in assembly.constraints if row.name == "hitch")
    assert {hitch.body_a, hitch.body_b} == {"tractor", "trailer"}


def test_an_articulation_naming_a_body_nobody_produced_is_refused() -> None:
    """The names are the finished assembly's, and an unknown one is named."""
    with pytest.raises(ValueError, match="does not carry"):
        _tow(
            ArticulationSpec(
                kind="weld", body_a="tractor", body_b="semitrailer", name="hitch"
            )
        )


def test_an_unknown_joint_kind_is_refused_rather_than_invented() -> None:
    """A kind the assembly cannot build is refused where it is stated."""
    with pytest.raises(ValueError, match="unknown kind"):
        ArticulationSpec(kind="ball", body_a="tractor", body_b="trailer")  # type: ignore[arg-type]


def test_a_joint_between_a_body_and_itself_is_refused() -> None:
    """A joint from a body to itself states nothing and is refused."""
    with pytest.raises(ValueError, match="to itself"):
        ArticulationSpec(kind="weld", body_a="tractor", body_b="tractor")


def test_an_articulation_is_the_only_thing_that_changes_the_bodies() -> None:
    """No articulation means no extra row: the assembly is what it was."""
    assert [row.name for row in _tow().constraints] == []


def test_the_towed_unit_carries_two_body_sides_and_runs() -> None:
    """
    The towed unit is readable as a study, and the solver accepts it.

    A joint that was only constructed would say nothing about whether the
    topology can be advanced, so this reads the assembly as a dynamic study --
    where the articulation has to convert into a joint row the kernel knows --
    and runs one case to completion.  Both body sides come back advanced by the
    same solve, which is the part constructing a joint alone cannot show.
    """
    assembly = _tow(
        ArticulationSpec(kind="weld", body_a="ground", body_b="tractor", name="anchor"),
        ArticulationSpec(
            kind="revolute",
            body_a="tractor",
            body_b="trailer",
            point_a_local=(3.0, 0.0, 0.5),
            point_b_local=(0.0, 0.0, 0.5),
            name="hitch",
        ),
    )
    model = axle_dynamics_model(
        build_study_assembly(runtime_for_study(assembly), study="dynamic", mode="K"),
        name="tow",
    )
    # The articulation reaches the solver as the joint kind it declared, so the
    # reading has the row rather than silently dropping it.
    hitch = next(joint for joint in model.joints if joint.name == "hitch")
    assert hitch.kind == "revolute"

    result = _run(assembly)
    tractor_pose = result.body_state("tractor")
    trailer_pose = result.body_state("trailer")
    # Both units are advanced by the same solve, one sample per requested time.
    assert tractor_pose.shape == trailer_pose.shape
    assert tractor_pose.shape[0] == 3
    assert np.all(np.isfinite(trailer_pose))
    # The chassis is welded to the ground, so it stays where it was assembled:
    # the state rows carry position and orientation, so only the position columns
    # are compared against the assembling pose.
    np.testing.assert_allclose(tractor_pose[:, :3], 0.0, atol=1e-9)
