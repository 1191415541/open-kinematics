"""
Generic kinematic driven coordinates (Adams joint MOTION) at the axle ABI.

A driven coordinate appends one constraint row that holds a single relative degree
of freedom between two bodies equal to a prescribed function of time.  These tests
pin the four things that make it usable rather than merely present:

* the coordinate is held **exactly** (a hard constraint, not a stiff spring),
* the drive reaction is available and equals the force the coordinate has to
  supply -- which is what a bench comparison wants to look at,
* targets that cannot be represented (outside the principal rotation branch) and
  redundant drivers (a degree of freedom a joint already locks) are **rejected**,
  not silently wrapped or silently singular,
* the explicit target rate reaches the velocity-level rows: a ramped target moves
  the body at the prescribed speed instead of being treated as stationary.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from suspension_multibody.axle_dynamics import (
    AxleBody,
    AxleDrivenCoordinate,
    AxleDynamicsCase,
    AxleDynamicsModel,
    AxleJoint,
    AxleSolverSettings,
    NativeAxleError,
    run_axle_dynamics,
)

_UNIT_INERTIA = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _translation_model(
    *,
    driven: tuple[AxleDrivenCoordinate, ...],
    joints: tuple[AxleJoint, ...] | None = None,
    initial_velocity_m_per_s: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> AxleDynamicsModel:
    """Ground plus one 10 kg body on a prismatic x guide."""
    return AxleDynamicsModel(
        name="driven-translation",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            AxleBody(
                name="ground",
                mass_kg=0.0,
                inertia_kg_m2=_UNIT_INERTIA,
                fixed=True,
            ),
            AxleBody(
                name="slider",
                mass_kg=10.0,
                inertia_kg_m2=_UNIT_INERTIA,
                linear_velocity_m_per_s=initial_velocity_m_per_s,
            ),
        ),
        joints=joints
        if joints is not None
        else (
            AxleJoint(
                name="guide",
                kind="prismatic",
                body_a="ground",
                body_b="slider",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                axis_a=(1.0, 0.0, 0.0),
                axis_b=(1.0, 0.0, 0.0),
            ),
        ),
        driven_coordinates=driven,
    )


def _case(
    *,
    samples: int = 11,
    step_s: float = 0.01,
    wrench: tuple[float, float, float, float, float, float] = (0.0,) * 6,
    target: tuple[float, ...] | None = None,
    rate: tuple[float, ...] | None = None,
    name: str = "slide",
) -> AxleDynamicsCase:
    times = tuple(index * step_s for index in range(samples))
    values = target if target is not None else tuple(0.0 for _ in times)
    return AxleDynamicsCase(
        name="driven-case",
        times_s=times,
        body_wrench_n_n_m={"slider": tuple(wrench for _ in times)},
        driven_target_m={name: values},
        driven_target_rate={} if rate is None else {name: rate},
        solver=AxleSolverSettings(
            initialization_mode="provided_consistent_state",
            adaptive_step=False,
            internal_step_s=step_s,
            minimum_step_s=1e-9,
            maximum_step_s=step_s,
        ),
    )


def _slide(column: str) -> AxleDrivenCoordinate:
    return AxleDrivenCoordinate(
        name=column,
        kind="translation",
        body="slider",
        reaction_body="ground",
        axis_local=(1.0, 0.0, 0.0),
    )


def _result(model: AxleDynamicsModel, case: AxleDynamicsCase):
    return run_axle_dynamics(model, case)


def test_a_constant_target_holds_the_coordinate_exactly() -> None:
    """A 100 N pull along the driven axis cannot move the body at all."""
    model = _translation_model(driven=(_slide("slide"),))
    case = _case(wrench=(100.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    result = _result(model, case)

    slider = result.body_state("slider")
    # The x column stays at its assembled value for the whole run: the row is a
    # hard constraint, so the residual is at solver tolerance rather than a
    # stiffness-dependent sag.
    np.testing.assert_allclose(slider[:, 0], 0.0, atol=1e-9)
    np.testing.assert_allclose(slider[:, 7], 0.0, atol=1e-9)


def test_the_drive_reaction_carries_the_applied_load() -> None:
    """
    The reported reaction is the force the coordinate has to supply.

    The reported wrench is the one the constraint applies to ``body_b``, the
    reaction body: pulling the free body +x by 100 N means the free body pushes the
    fixed reference +x by 100 N, so the reaction reads +100 N.  The drive itself
    supplies the opposite to the driven body.  With no external load there is
    nothing to hold and the reaction is zero.
    """
    model = _translation_model(driven=(_slide("slide"),))
    loaded = _result(model, _case(wrench=(100.0, 0.0, 0.0, 0.0, 0.0, 0.0)))
    free = _result(model, _case())

    wrench = loaded.joint_wrench_on_body_b("slide")
    assert wrench.shape == (11, 6)
    np.testing.assert_allclose(wrench[:, 0], 100.0, rtol=1e-6, atol=1e-6)
    np.testing.assert_allclose(wrench[:, 1:], 0.0, atol=1e-9)
    # Nothing to hold against: the driven coordinate is idle.
    np.testing.assert_allclose(
        free.joint_wrench_on_body_b("slide")[:, 0], 0.0, atol=1e-9
    )


def test_a_ramped_target_moves_the_body_at_the_prescribed_rate() -> None:
    """
    ``target = 2 t`` must translate the body at 2 m/s, not pin it.

    This is the case that fails if the explicit target rate never reaches the
    velocity-level rows: the position row alone cannot tell a moving target from
    a stationary one between samples.
    """
    # The velocity-level row is ``J q_dot = target_rate``, so a ramped target
    # requires a matching initial velocity; the initial-state gate rejects the
    # inconsistent pair instead of quietly starting with a velocity step.
    model = _translation_model(
        driven=(_slide("slide"),), initial_velocity_m_per_s=(2.0, 0.0, 0.0)
    )
    case = _case(
        samples=101,
        target=tuple(2.0 * 0.01 * index for index in range(101)),
    )
    result = _result(model, case)

    slider = result.body_state("slider")
    times = np.asarray(result.times_s)
    np.testing.assert_allclose(slider[:, 0], 2.0 * times, rtol=1e-9, atol=1e-9)
    np.testing.assert_allclose(slider[:, 7], 2.0, rtol=1e-6, atol=1e-6)
    # A linear ramp at constant speed needs no force beyond the initial step.
    np.testing.assert_allclose(
        result.joint_wrench_on_body_b("slide")[1:, 0], 0.0, atol=1e-6
    )


def test_an_explicit_rate_reproduces_the_derived_one() -> None:
    """Stating the rate must reach the kernel, not be ignored in favour of slope."""
    target = tuple(2.0 * 0.01 * index for index in range(101))
    model = _translation_model(
        driven=(_slide("slide"),), initial_velocity_m_per_s=(2.0, 0.0, 0.0)
    )
    derived = _result(model, _case(samples=101, target=target))
    stated = _result(
        model,
        _case(samples=101, target=target, rate=tuple(2.0 for _ in range(101))),
    )

    np.testing.assert_allclose(
        stated.body_state("slider"), derived.body_state("slider"), atol=1e-9
    )


def test_the_drive_work_enters_the_energy_ledger() -> None:
    """
    A driver that moves the body against a load must be booked as doing work.

    The body runs at a constant 2 m/s under a -100 N load, so the drive supplies
    +200 W and the load absorbs -200 W.  Kinetic energy never changes, so the
    ledger only closes if the constraint's own work ``-lambda * target_rate`` is
    attributed to the drive: without that term the residual would grow at 200 W
    and this model would look like it creates energy.
    """
    model = _translation_model(
        driven=(_slide("slide"),), initial_velocity_m_per_s=(2.0, 0.0, 0.0)
    )
    samples = 101
    load = (-100.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    case = _case(
        samples=samples,
        wrench=load,
        target=tuple(2.0 * 0.01 * index for index in range(samples)),
    )
    result = _result(model, case)

    duration = 0.01 * (samples - 1)
    energy = result.energy
    # 4 is external_work, 6 is drive_work, 3 is the residual; each column is the
    # work done during one output interval, so the totals are the column sums.
    np.testing.assert_allclose(
        energy[:, 4].sum(), -200.0 * duration, rtol=1e-9, atol=1e-9
    )
    np.testing.assert_allclose(
        energy[:, 6].sum(), 200.0 * duration, rtol=1e-9, atol=1e-9
    )
    assert abs(energy[:, 3].sum()) < 1e-9, (
        f"energy residual {energy[:, 3].sum()!r} over {duration} s"
    )

    # A held coordinate does no work: the same load with a constant target.  The
    # body must start at rest here, because a constant target also prescribes a
    # zero rate and the initial-state gate rejects the inconsistent pair.
    held_model = _translation_model(driven=(_slide("slide"),))
    held = _result(held_model, _case(samples=samples, wrench=load))
    np.testing.assert_allclose(held.energy[:, 6].sum(), 0.0, atol=1e-9)
    assert abs(held.energy[:, 3].sum()) < 1e-9, (
        f"held energy residual {held.energy[:, 3].sum()!r}"
    )


def test_a_rate_only_request_moves_the_coordinate_at_that_rate() -> None:
    """
    Prescribing only a rate integrates it into the displacement the row tracks.

    The coordinate still follows the integral exactly (the row is position-level),
    which is what a caller means by "drive this degree of freedom at this speed":
    the body moves at 2 m/s from the assembling pose.  A true velocity-level DAE
    row -- where the position is left free and only its rate is constrained -- is a
    separate change and is not claimed here.
    """
    model = _translation_model(
        driven=(_slide("slide"),), initial_velocity_m_per_s=(2.0, 0.0, 0.0)
    )
    samples = 11
    case = _case(samples=samples, rate=tuple(2.0 for _ in range(samples)))
    del case.driven_target_m["slide"]
    result = _result(model, case)

    times = np.asarray(result.times_s)
    slider = result.body_state("slider")
    np.testing.assert_allclose(slider[:, 0], 2.0 * times, rtol=1e-12, atol=1e-12)
    np.testing.assert_allclose(slider[:, 7], 2.0, rtol=1e-9, atol=1e-9)


def test_a_rotation_target_is_held_in_the_principal_branch() -> None:
    """The rotational variant holds ``(angle about the axis) - target`` at zero."""
    model = AxleDynamicsModel(
        name="driven-rotation",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            AxleBody(
                name="ground",
                mass_kg=0.0,
                inertia_kg_m2=_UNIT_INERTIA,
                fixed=True,
            ),
            AxleBody(
                name="arm",
                mass_kg=1.0,
                inertia_kg_m2=_UNIT_INERTIA,
                # The driven angle is measured from the *assembling* pose, so the
                # body starts already rotated by the target: otherwise the initial
                # state violates the row it is supposed to hold.
                quaternion_body_to_world=(
                    math.cos(0.15), 0.0, 0.0, math.sin(0.15)
                ),
            ),
        ),
        joints=(
            AxleJoint(
                name="hinge",
                kind="revolute",
                body_a="ground",
                body_b="arm",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                axis_a=(0.0, 0.0, 1.0),
                axis_b=(0.0, 0.0, 1.0),
            ),
        ),
        driven_coordinates=(
            AxleDrivenCoordinate(
                name="toe",
                kind="rotation",
                body="arm",
                reaction_body="ground",
                axis_local=(0.0, 0.0, 1.0),
            ),
        ),
    )
    case = AxleDynamicsCase(
        name="driven-rotation-case",
        times_s=tuple(index * 0.01 for index in range(11)),
        # +5 N*m about z tries to open the hinge; the coordinate resists it and
        # the reaction on the reference body reads +5 N*m.
        body_wrench_n_n_m={
            "arm": tuple((0.0, 0.0, 0.0, 0.0, 0.0, 5.0) for _ in range(11))
        },
        driven_target_m={"toe": tuple(0.3 for _ in range(11))},
        solver=AxleSolverSettings(
            initialization_mode="provided_consistent_state",
            adaptive_step=False,
            internal_step_s=0.01,
            minimum_step_s=1e-9,
            maximum_step_s=0.01,
        ),
    )
    result = run_axle_dynamics(model, case)

    arm = result.body_state("arm")
    # Quaternion z component is sin(theta/2) for a rotation about z.
    np.testing.assert_allclose(
        arm[:, 6], math.sin(0.15), rtol=1e-6, atol=1e-9
    )
    wrench = result.joint_wrench_on_body_b("toe")
    np.testing.assert_allclose(wrench[:, 5], 5.0, rtol=1e-6, atol=1e-6)


def test_a_multi_turn_rotation_target_follows_the_unwrapped_rate() -> None:
    """
    A rotational target beyond one turn is folded into the principal branch.

    The position row compares a *principal* rotation vector against the target, and
    an orientation is periodic, so folding both by the same amount keeps their
    difference continuous across the branch cut.  The rate row keeps the unwrapped
    derivative, so the body still turns at the prescribed speed instead of a
    branch-shifted alias -- this is what makes a driven wheel (many revolutions)
    representable at all.
    """
    model = AxleDynamicsModel(
        name="driven-multiturn",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            AxleBody(
                name="ground",
                mass_kg=0.0,
                inertia_kg_m2=_UNIT_INERTIA,
                fixed=True,
            ),
            AxleBody(
                name="arm",
                mass_kg=1.0,
                inertia_kg_m2=_UNIT_INERTIA,
                angular_velocity_rad_per_s=(0.0, 0.0, 50.0),
            ),
        ),
        joints=(
            AxleJoint(
                name="hinge",
                kind="revolute",
                body_a="ground",
                body_b="arm",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                axis_a=(0.0, 0.0, 1.0),
                axis_b=(0.0, 0.0, 1.0),
            ),
        ),
        driven_coordinates=(
            AxleDrivenCoordinate(
                name="spin",
                kind="rotation",
                body="arm",
                reaction_body="ground",
                axis_local=(0.0, 0.0, 1.0),
            ),
        ),
    )
    samples = 101
    times = tuple(index * 0.01 for index in range(samples))
    case = AxleDynamicsCase(
        name="multiturn-case",
        times_s=times,
        # 50 rad/s for 1 s is ~8 revolutions: far outside the principal branch.
        driven_target_m={"spin": tuple(50.0 * time for time in times)},
        driven_target_rate={"spin": tuple(50.0 for _ in times)},
        solver=AxleSolverSettings(
            initialization_mode="provided_consistent_state",
            adaptive_step=False,
            internal_step_s=0.01,
            minimum_step_s=1e-9,
            maximum_step_s=0.01,
        ),
    )
    result = run_axle_dynamics(model, case)
    arm = result.body_state("arm")

    np.testing.assert_allclose(arm[:, 12], 50.0, rtol=1e-9, atol=1e-9)
    # Orientation about z, reconstructed from the quaternion (double cover: the
    # angle is only defined modulo 2*pi, which is exactly the point of the fold).
    angle = 2.0 * np.arctan2(arm[:, 6], arm[:, 3])
    expected = np.mod(50.0 * np.asarray(times) + np.pi, 2.0 * np.pi) - np.pi
    error = np.angle(np.exp(1j * (angle - expected)))
    assert np.max(np.abs(error)) < 1e-6, np.max(np.abs(error))

def test_a_redundant_driver_is_rejected_as_rank_deficient() -> None:
    """
    Driving a degree of freedom a joint already locks must fail loudly.

    Every driven coordinate appends a row after ``build_model`` has audited the
    system, so this only works because the audit runs again after registration: a
    missing second audit would turn this into a singular KKT matrix at the first
    step instead of a build error.
    """
    model = _translation_model(
        driven=(_slide("slide"),),
        joints=(
            AxleJoint(
                name="welded",
                kind="fixed",
                body_a="ground",
                body_b="slider",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
            ),
        ),
    )
    with pytest.raises(NativeAxleError, match="rank deficient"):
        run_axle_dynamics(model, _case())


def test_a_missing_target_is_an_error_not_a_silent_zero() -> None:
    """No target and no rate must fail: a silently pinned DOF looks plausible."""
    model = _translation_model(driven=(_slide("slide"),))
    case = _case()
    case.driven_target_m.clear()
    with pytest.raises(ValueError, match="neither a target nor a rate"):
        run_axle_dynamics(model, case)


def test_the_schema_rejects_a_driver_without_a_distinct_reaction_body() -> None:
    with pytest.raises(ValueError, match="two distinct bodies"):
        AxleDrivenCoordinate(
            name="slide",
            kind="translation",
            body="slider",
            reaction_body="slider",
            axis_local=(1.0, 0.0, 0.0),
        )
    with pytest.raises(ValueError, match="axis_local must be nonzero"):
        AxleDrivenCoordinate(
            name="slide",
            kind="translation",
            body="slider",
            reaction_body="ground",
            axis_local=(0.0, 0.0, 0.0),
        )
    with pytest.raises(ValueError, match="reference_quaternion"):
        AxleDrivenCoordinate(
            name="slide",
            kind="rotation",
            body="slider",
            reaction_body="ground",
            axis_local=(0.0, 0.0, 1.0),
            reference_quaternion=(1.0, 0.0, 0.0, 0.5),
        )


def test_the_schema_rejects_driving_a_fixed_body() -> None:
    with pytest.raises(ValueError, match="unknown body|fixed body"):
        AxleDynamicsModel(
            name="driven-fixed",
            bodies=(
                AxleBody(
                    name="ground",
                    mass_kg=0.0,
                    inertia_kg_m2=_UNIT_INERTIA,
                    fixed=True,
                ),
                AxleBody(
                    name="slider",
                    mass_kg=10.0,
                    inertia_kg_m2=_UNIT_INERTIA,
                ),
            ),
            joints=(
                AxleJoint(
                    name="guide",
                    kind="prismatic",
                    body_a="ground",
                    body_b="slider",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                    axis_a=(1.0, 0.0, 0.0),
                    axis_b=(1.0, 0.0, 0.0),
                ),
            ),
            driven_coordinates=(
                AxleDrivenCoordinate(
                    name="slide",
                    kind="translation",
                    body="ground",
                    reaction_body="slider",
                    axis_local=(1.0, 0.0, 0.0),
                ),
            ),
        )
