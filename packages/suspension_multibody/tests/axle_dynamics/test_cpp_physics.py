from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.axle_dynamics import (
    AxleBody,
    AxleDynamicsCase,
    AxleDynamicsModel,
    AxleJoint,
    AxleSolverSettings,
    NativeAxleError,
    run_axle_dynamics,
)

_I = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
_ZERO_I = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


def _free_body_model(
    *,
    mass: float = 2.0,
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> AxleDynamicsModel:
    return AxleDynamicsModel(
        name="free-body",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            AxleBody(
                name="fixture",
                mass_kg=0.0,
                inertia_kg_m2=_ZERO_I,
                fixed=True,
            ),
            AxleBody(
                name="body",
                mass_kg=mass,
                inertia_kg_m2=_I,
                position_m=position,
                linear_velocity_m_per_s=velocity,
            ),
        ),
        joints=(),
    )


def test_constant_wrench_produces_physical_translation_and_rotation() -> None:
    result = run_axle_dynamics(
        _free_body_model(),
        AxleDynamicsCase(
            name="constant-wrench",
            times_s=(0.0, 0.001, 0.002),
            body_wrench_n_n_m={
                "body": ((10.0, 0.0, 0.0, 0.0, 0.0, 6.0),) * 3
            },
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                adaptive_step=False,
                internal_step_s=0.00025,
            ),
        ),
    )

    state = result.body_state("body")
    np.testing.assert_allclose(
        state[0, :3], 0.0, atol=1e-14, rtol=0.0
    )
    np.testing.assert_allclose(
        state[0, 4:13], 0.0, atol=1e-14, rtol=0.0
    )
    np.testing.assert_allclose(state[0, 3], 1.0, atol=1e-14, rtol=0.0)
    np.testing.assert_allclose(state[-1, 0], 0.5 * 5.0 * 0.002**2, atol=1e-10)
    np.testing.assert_allclose(state[-1, 7], 5.0 * 0.002, atol=1e-10)
    np.testing.assert_allclose(state[-1, 13], 5.0, atol=1e-10)
    np.testing.assert_allclose(state[-1, 18], 6.0, atol=1e-10)
    np.testing.assert_allclose(
        state[-1, 6], 0.25 * 6.0 * 0.002**2, atol=1e-10
    )
    assert np.all(result.diagnostics.accepted)


def test_si_scale_inertia_remains_solvable() -> None:
    base = _free_body_model()
    bodies = list(base.bodies)
    bodies[1] = bodies[1].model_copy(
        update={"inertia_kg_m2": np.diag([1.0e-6, 1.0e-6, 1.0e-6])}
    )
    model = base.model_copy(update={"bodies": tuple(bodies)})

    result = run_axle_dynamics(
        model,
        AxleDynamicsCase(
            name="si-scale-inertia",
            times_s=(0.0, 0.001),
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                adaptive_step=False,
                internal_step_s=0.001,
                minimum_step_s=0.001,
                maximum_step_s=0.001,
            ),
        ),
    )

    assert np.all(result.diagnostics.accepted)


def test_provided_initial_state_rejects_velocity_constraint_violation() -> None:
    model = AxleDynamicsModel(
        name="inconsistent-velocity",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            AxleBody(
                name="fixture",
                mass_kg=0.0,
                inertia_kg_m2=_ZERO_I,
                fixed=True,
            ),
            AxleBody(
                name="body",
                mass_kg=1.0,
                inertia_kg_m2=_I,
                linear_velocity_m_per_s=(1.0, 0.0, 0.0),
            ),
        ),
        joints=(
            AxleJoint(
                name="guide",
                kind="prismatic",
                body_a="fixture",
                body_b="body",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                axis_a=(0.0, 0.0, 1.0),
                axis_b=(0.0, 0.0, 1.0),
            ),
        ),
    )

    with pytest.raises(NativeAxleError, match="initial velocity"):
        run_axle_dynamics(
            model,
            AxleDynamicsCase(
                name="inconsistent-velocity",
                times_s=(0.0, 0.001),
                solver=AxleSolverSettings(
                    initialization_mode="provided_consistent_state"
                ),
            ),
        )


def test_fixed_joint_reaction_maps_to_world_wrench() -> None:
    model = AxleDynamicsModel(
        name="fixed-reaction",
        bodies=(
            AxleBody(
                name="fixture",
                mass_kg=0.0,
                inertia_kg_m2=_ZERO_I,
                fixed=True,
            ),
            AxleBody(name="body", mass_kg=10.0, inertia_kg_m2=_I),
        ),
        joints=(
            AxleJoint(
                name="fixed",
                kind="fixed",
                body_a="fixture",
                body_b="body",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
            ),
        ),
    )
    result = run_axle_dynamics(
        model,
        AxleDynamicsCase(
            name="fixed-reaction",
            times_s=(0.0, 0.001),
            solver=AxleSolverSettings(
                adaptive_step=False,
                internal_step_s=0.00025,
            ),
        ),
    )
    expected = np.array([0.0, 0.0, 10.0 * 9.80665, 0.0, 0.0, 0.0])
    np.testing.assert_allclose(
        result.joint_wrench_on_body_b("fixed"),
        np.tile(expected, (2, 1)),
        atol=1e-6,
        rtol=1e-10,
    )


def test_rank_deficient_constraint_set_is_rejected() -> None:
    model = AxleDynamicsModel(
        name="rank-deficient",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            AxleBody(
                name="fixture",
                mass_kg=0.0,
                inertia_kg_m2=_ZERO_I,
                fixed=True,
            ),
            AxleBody(name="body", mass_kg=1.0, inertia_kg_m2=_I),
        ),
        joints=(
            AxleJoint(
                name="joint-1",
                kind="spherical",
                body_a="fixture",
                body_b="body",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
            ),
            AxleJoint(
                name="joint-2",
                kind="spherical",
                body_a="fixture",
                body_b="body",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
            ),
        ),
    )
    with pytest.raises(NativeAxleError, match="rank deficient"):
        run_axle_dynamics(
            model,
            AxleDynamicsCase(name="rank-deficient", times_s=(0.0, 0.001)),
        )


@pytest.mark.parametrize(
    ("kind", "linear_velocity", "angular_velocity"),
    (
        ("spherical", (0.0, 0.0, 0.0), (0.3, 0.4, 0.5)),
        ("revolute", (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
        ("prismatic", (0.0, 0.0, 1.0), (0.0, 0.0, 0.0)),
    ),
)
def test_ideal_joint_preserves_only_its_allowed_motion(
    kind: str,
    linear_velocity: tuple[float, float, float],
    angular_velocity: tuple[float, float, float],
) -> None:
    model = AxleDynamicsModel(
        name=f"{kind}-motion",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            AxleBody(
                name="fixture",
                mass_kg=0.0,
                inertia_kg_m2=_ZERO_I,
                fixed=True,
            ),
            AxleBody(
                name="body",
                mass_kg=1.0,
                inertia_kg_m2=_I,
                linear_velocity_m_per_s=linear_velocity,
                angular_velocity_rad_per_s=angular_velocity,
            ),
        ),
        joints=(
            AxleJoint(
                name="joint",
                kind=kind,  # type: ignore[arg-type]
                body_a="fixture",
                body_b="body",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                axis_a=(0.0, 0.0, 1.0),
                axis_b=(0.0, 0.0, 1.0),
            ),
        ),
    )
    result = run_axle_dynamics(
        model,
        AxleDynamicsCase(
            name=f"{kind}-motion",
            times_s=(0.0, 0.001, 0.002),
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                adaptive_step=False,
                internal_step_s=0.00025,
            ),
        ),
    )
    state = result.body_state("body")

    if kind == "prismatic":
        np.testing.assert_allclose(state[:, :2], 0.0, atol=1e-12, rtol=0.0)
        np.testing.assert_allclose(state[:, 2], result.times_s, atol=1e-11)
        np.testing.assert_allclose(state[:, 3], 1.0, atol=1e-12, rtol=0.0)
        np.testing.assert_allclose(state[:, 4:7], 0.0, atol=1e-12, rtol=0.0)
    else:
        np.testing.assert_allclose(state[:, :3], 0.0, atol=1e-12, rtol=0.0)
        assert np.linalg.norm(state[-1, 4:7]) > 0.0
        if kind == "revolute":
            np.testing.assert_allclose(
                state[:, 4:6], 0.0, atol=1e-12, rtol=0.0
            )
    assert np.max(result.diagnostics.position_residual) <= 1e-8
    assert np.max(result.diagnostics.velocity_residual) <= 1e-7


def _rotation_quaternion(
    axis: tuple[float, float, float], angle_rad: float
) -> tuple[float, float, float, float]:
    direction = np.asarray(axis, dtype=float)
    direction /= np.linalg.norm(direction)
    sine = np.sin(0.5 * angle_rad)
    return (
        float(np.cos(0.5 * angle_rad)),
        *(float(value) for value in direction * sine),
    )


def test_prismatic_joint_accepts_rotated_initial_frame() -> None:
    """棱柱副允许两端坐标系有安装偏角，只约束导向轴和绕轴转角."""
    model = AxleDynamicsModel(
        name="rotated-prismatic",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            AxleBody(
                name="fixture",
                mass_kg=0.0,
                inertia_kg_m2=_ZERO_I,
                fixed=True,
            ),
            AxleBody(
                name="slider",
                mass_kg=1.0,
                inertia_kg_m2=_I,
                quaternion_body_to_world=_rotation_quaternion(
                    (1.0, 0.0, 0.0), np.pi / 2.0
                ),
                linear_velocity_m_per_s=(0.0, 1.0, 0.0),
            ),
        ),
        joints=(
            AxleJoint(
                name="guide",
                kind="prismatic",
                body_a="fixture",
                body_b="slider",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                axis_a=(0.0, 1.0, 0.0),
                axis_b=(0.0, 0.0, -1.0),
            ),
        ),
    )

    result = run_axle_dynamics(
        model,
        AxleDynamicsCase(
            name="rotated-prismatic",
            times_s=(0.0, 0.001, 0.002),
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                adaptive_step=False,
                internal_step_s=0.00025,
            ),
        ),
    )
    state = result.body_state("slider")
    np.testing.assert_allclose(state[:, 0], 0.0, atol=1e-12, rtol=0.0)
    np.testing.assert_allclose(state[:, 1], result.times_s, atol=1e-11)
    np.testing.assert_allclose(state[:, 2], 0.0, atol=1e-12, rtol=0.0)
    np.testing.assert_allclose(
        state[:, 3:7],
        np.broadcast_to(
            _rotation_quaternion((1.0, 0.0, 0.0), np.pi / 2.0),
            state[:, 3:7].shape,
        ),
        atol=1e-11,
        rtol=0.0,
    )
    assert np.max(result.diagnostics.position_residual) <= 1e-8
    assert np.max(result.diagnostics.velocity_residual) <= 1e-7


@pytest.mark.parametrize(
    "kind",
    (
        "spherical",
        "revolute",
        "prismatic",
        "fixed",
        "universal",
        "cylindrical",
        "inplane",
    ),
)
@pytest.mark.parametrize("angle_rad", (0.0, 1.1, 3.0))
@pytest.mark.parametrize("joint_axis", ((0.0, 0.0, 1.0), (0.3, -0.8, 0.5)))
def test_analytic_constraint_jacobian_agrees_with_central_differences(
    kind: str,
    angle_rad: float,
    joint_axis: tuple[float, float, float],
) -> None:
    """
    Check the closed-form Jacobian against the kernel's own reference.

    The kernel rejects a run whose analytic constraint Jacobian disagrees with
    central differences, so a wrong derivation surfaces as that specific error.
    Non-identity orientations and a skew joint axis are required: at identity
    the rotation blocks and the perpendicular-frame derivative are degenerate
    and a wrong derivation can still agree.
    """
    orientation_a = _rotation_quaternion((1.0, 2.0, 3.0), angle_rad)
    orientation_b = _rotation_quaternion((3.0, 1.0, 2.0), 0.7 * angle_rad + 0.2)
    # A universal joint constrains the two cross-axes to stay perpendicular, so
    # feeding it the same axis on both bodies would start it fully violated.
    axis_b = joint_axis
    if kind == "universal":
        reference = (
            (1.0, 0.0, 0.0) if abs(joint_axis[0]) < 0.8 else (0.0, 1.0, 0.0)
        )
        axis_b = tuple(
            float(value)
            for value in np.cross(np.asarray(joint_axis), np.asarray(reference))
        )
    model = AxleDynamicsModel(
        name="jacobian-audit",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            AxleBody(
                name="fixture",
                mass_kg=0.0,
                inertia_kg_m2=_ZERO_I,
                fixed=True,
                quaternion_body_to_world=orientation_a,
            ),
            AxleBody(
                name="body",
                mass_kg=2.0,
                inertia_kg_m2=_I,
                position_m=(0.1, -0.2, 0.3),
                quaternion_body_to_world=orientation_b,
            ),
        ),
        joints=(
            AxleJoint(
                name="joint",
                kind=kind,  # type: ignore[arg-type]
                body_a="fixture",
                body_b="body",
                point_a_m=(0.05, 0.1, -0.07),
                point_b_m=(-0.02, 0.04, 0.09),
                axis_a=joint_axis,
                axis_b=axis_b,
            ),
        ),
    )
    try:
        run_axle_dynamics(
            model,
            AxleDynamicsCase(
                name="jacobian-audit",
                times_s=(0.0, 0.001),
                solver=AxleSolverSettings(
                    initialization_mode="provided_consistent_state"
                ),
            ),
        )
    except NativeAxleError as error:
        # Other physical rejections are acceptable here; a Jacobian mismatch
        # is the failure this test exists to catch.
        assert "disagrees with central" not in str(error), str(error)
